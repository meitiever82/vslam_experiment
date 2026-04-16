# 深度学习视觉里程计

本文档分析 `src/` 下 **3 个** DL-VO 仓库：

| 仓库 | 算法库 | 类型 |
|------|--------|------|
| `droid_w_ros2` | **DROID-W**（上游：`/home/steve/Documents/GitHub/slam/DROID-W/`） | ROS2 wrapper |
| `cuvslam_ros` | **cuVSLAM**（上游：`/home/steve/Documents/GitHub/slam/cuVSLAM/`） | ROS2 wrapper |
| `DPVO` | **DPVO / Deep Patch VSLAM**（Teed/Lipson/Deng，2023/2024） | 独立 Python（带 `COLCON_IGNORE`） |

前两个是 wrapper，算法分析压掉 ROS 壳；DPVO 是上游算法本身。

---

## 1. droid_w_ros2 / DROID-W（深度分析）

> 2026-04-15 重写：聚焦上游 **DROID-W**（`/home/steve/Documents/GitHub/slam/DROID-W/`），同时保留原版 **DROID-SLAM**（`/home/steve/Documents/GitHub/slam/DROID-SLAM/`）对照。ROS2 壳压到 §1.12。

### 1.1 上游与定位
- **DROID-SLAM**（ECCV 2021，Teed & Deng）——RAFT-style 光流 + 稠密 BA 的端到端深度 SLAM。
- **DROID-W**（GlORIE-SLAM 实验室）——把 DROID 改造成"野外"版本，加入 **单目深度先验（Metric3D）**、**DINO v2 特征**、**动态不确定性**、**Gaussian Splatting mapper**。相对原版 src/ 目录 Python 文件从 ~15 涨到 ~50+。

相对原版的三类主要增量：
1. 架构：多了 `Mapper` 线程（Gaussian Splatting）+ `Tracker` 线程分离
2. 先验：`utils/mono_priors/{metric_depth_estimators,img_feature_extractors}.py` 接入 Metric3D V2 + DINO v2
3. 动态处理：`utils/dyn_uncertainty/` 学习仿射变换把 DINO 特征映到像素不确定性

### 1.2 整体流程（DROID-W）
```
视频帧
  ↓
MotionFilter (motion_filter.py:52)  ─ 光流阈值 >2.5 px 的帧进关键帧
  │    └─ 首帧额外：Metric3D 预测 mono_depth、DINO 提特征
  ↓
Tracker (tracker.py:15-145)  ─ 新线程
  ├─ Frontend (frontend.py:6-149)       局部因子图、3+2 轮 LocalBA
  │   └─ 可选 loop_closing（Backend 实例复用）
  └─ Backend (backend.py:19-121)        异步全局 BA，含循环闭合 BA
  ↓
DepthVideo (depth_video.py:24)  ─ 共享内存张量，跨线程同步
  │    └─ 新增 mono_disps / dino_feats / uncertainties / affine_weights
  ↓
Mapper (mapper.py:52)  ─ 可选线程
  └─ Gaussian Splatting（gaussian_splatting + diff-gaussian-rasterization-w-pose）
  ↓
输出：轨迹（keyframe + 非 KF 插值）+ 深度 + 3DGS 场景
```

### 1.3 神经网络（`modules/droid_net/`）
和原版几乎一致，只有参数名 `output_dim → out_dim` 等微调（`droid_net.py:159`）。主干：

| 子网 | 输出 | 作用 |
|------|------|------|
| `fnet`（BasicEncoder 128 维） | `(B, N, 128, H/8, W/8)` | 相关性体计算 |
| `cnet`（BasicEncoder 256 维，split 128+128） | `tanh(net) / relu(inp)` | GRU 隐状态 & 上下文输入 |

```python
# droid_net.py:154-169
fmaps = self.fnet(images)
net, inp = self.cnet(images).split([128, 128], dim=2)
net = torch.tanh(net); inp = torch.relu(inp)
```

`CorrBlock`（`modules/corr.py:23-72`）：all-pairs 4D 相关性 + 4 层金字塔；`corr_index_forward/backward` 是自定义 CUDA（`droid_backends/`）。

`ConvGRU`（`modules/gru.py:5-32`）带全局调制：
```python
glo = (torch.sigmoid(self.w(net)) * net).view(b,c,h*w).mean(-1)
z = sigmoid(convz(cat[net,inp]) + convz_glo(glo))
net = (1-z)*net + z*q
```

`UpdateModule` 每次输出 `(net, delta, weight, eta, upmask)`。DROID-W 在 `GraphAgg` 里返回 `0.01 * eta` 作为不确定性标量（`droid_net.py:48-80`）。

### 1.4 Dense Bundle Adjustment（核心创新）
```python
# geom/ba.py:48   DROID-W 签名（新增 sensor_disps, alpha）
def BA(target, weight, eta, poses, disps, intrinsics, ii, jj,
       sensor_disps=None, lm=1e-4, ep=0.1, alpha=0.05, fixedp=1, rig=1):
    coords, valid, (Ji, Jj, Jz) = pops.projective_transform(
        poses, disps, intrinsics, ii, jj, jacobian=True)
    r = (target - coords).view(B, N_edges, -1, 1)
    w = 1e-3 * (valid * weight).view(B, N_edges, -1, 1)
    # Hessian 分块
    Hii = wJiT @ Ji;  Hij = wJiT @ Jj;  Hjj = wJjT @ Jj
    Ei  = (wJiT · Jz).sum(-1);  Ej = (wJjT · Jz).sum(-1)
    Ck  = (w * Jz * Jz).sum(-1);  wk = (w * r * Jz).sum(-1)
```

**Schur 补求解**（`geom/chol.py:46-73`）—— 把 H×W 个独立深度消去：
```python
H += (ep + lm*H) * I
S  = H - E @ (Q*Et);    v -= E @ (Q*w)
dx = CholeskySolver.apply(S, v)
dz = Q * (w - Et @ dx)
```
数值失败时返回 0（训练中断保护）。梯度走 `torch.cholesky_solve` 自动求导。

**Retraction**（SE(3) 上指数更新）：
```python
# geom/ba.py:21-28
pose_retr(poses, dx, ii) = poses.retr(scatter_sum(dx, ii, dim=1))
disp_retr(disps, dz, ii) = disps + scatter_sum(dz, ii, dim=1)
```

CUDA 后端 `droid_backends.ba(...)`（`modules/droid_backend/`）负责块 Hessian 组装 + Schur + Cholesky，支持 `motion_only`。

### 1.5 共享张量 `DepthVideo`（DROID-W 扩展）
```python
class DepthVideo:   # depth_video.py:24
    counter, ready         = Value('i',0), Value('i',0)
    poses   = (B,7) fp32   # [tx,ty,tz,qx,qy,qz,qw]，lietorch
    disps   = (B, H/8, W/8) fp32
    fmaps   = (B, c, 128, H/8, W/8) fp16
    nets, inps = (B, 128, H/8, W/8) fp16
    images  = (B, 3, H, W) uint8            # DROID-W 放 CPU 省显存
    tstamp, dirty, red = ...
    disps_sens, disps_up, intrinsics = ...

    # === DROID-W 新增 ===
    mono_disps, mono_disps_up, depth_scale, depth_shift
    valid_depth_mask, valid_depth_mask_small
    dino_feats           # (B, H/14 or /16, W/14 or /16, 384) CPU
    dino_feats_resize    # (B, 384, H/8, W/8) GPU
    uncertainties        # (B, H/8, W/8) 像素不确定性
    affine_weights       # (384+1,) 学到的特征→不确定性仿射
    temp_y_cdot
    npc_dirty            # 点云脏标记
```

核心方法：`append()` / `get_lock()` / `update_valid_depth_mask()` / `set_dirty()` / `reproject()` / `ba(...)` / `distance()`（调 `droid_backends.frame_distance`，用于回环近邻度量）。

### 1.6 关键帧选择（`motion_filter.py:10-145`）
```python
def track(self, tstamp, image, intrinsics=None):
    # 首帧：一次性拿 Metric3D 单目深度 + DINO 特征
    if self.video.counter.value == 0:
        mono = predict_metric_depth(self.metric_depth_estimator, tstamp, image, ...)
        if self.uncertainty_aware:
            feat = predict_img_features(self.feat_extractor, tstamp, image, ...)
        self.video.append(tstamp, image, Id, 1.0, mono, K, gmap, net, inp, feat)
        return

    # 新帧：corr + 一次 GRU → delta.norm() 衡量光流
    corr = CorrBlock(self.fmap[None,[0]], gmap[None,[0]])(coords0)
    _, delta, weight = self.update(net, inp, corr)
    if delta.norm().mean() > 2.5:
        self.video.append(...)
    # 可选：cfg.force_keyframe_every_n_frames 强插
```

### 1.7 前端（`frontend.py:6-149`）
```python
# 初始化（warmup 帧到齐后）：8 轮邻域 BA + 2 近邻因子 + 8 轮 BA
# 常态每帧：
self.graph.rm_factors(self.graph.age > 20, store=True)    # 旧边搬 inactive
self.graph.add_proximity_factors(
    t0=max(t1-window, 0), t1=t1-5,
    rad=radius, nms=nms, thresh=16.0, beta=0.25, remove=True)
for itr in range(3):
    self.graph.update(None, None, use_inactive=True,
        enable_update_uncer=self.enable_opt_dyn_mask,     # ← DROID-W
        enable_udba=self.enable_opt_dyn_mask)
# 关键帧冗余检查
if self.enable_loop and cur_t > window:                   # ← DROID-W 新增
    self.loop_closing.loop_ba(0, cur_t, steps=2, ...)

if self.video.enable_affine_transform:                    # ← 不确定性
    y_cdot = dino_feats_resize[t1].permute(1,2,0) @ aff[:-1] + aff[-1]
    self.video.uncertainties[t1] = log(1.1 + exp(y_cdot))
```

### 1.8 后端（`backend.py:19-121`）
```python
# 参数化 BA（原版只有固定 12 轮全局）
def backend_ba(t_start, t_end, steps, graph, nms, radius, thresh,
               max_factors, t_start_loop=None, loop=False,
               enable_update_uncer=False, enable_udba=False, ...)
def dense_ba(steps=6, enable_wq=True, ...)
def loop_ba (t_start, t_end, steps=6, motion_only=False, local_graph=None, ...)
```
配置里 `backend_thresh / radius / nms / normalize / loop_window / loop_thresh / metric_depth_reg` 都能调，原版这块写死。

### 1.9 因子图（`factor_graph.py:9-413`）
新增字段：`damping = 1e-6 * ones_like(disps)`；`ii_inac/jj_inac/target_inac/weight_inac`（非活跃边，用于回环 + long-range 约束）；`corr_impl ∈ {"volume", "alt"}` 支持低显存替代相关体。

关键方法：`add_neighborhood_factors` / `add_proximity_factors` / **`add_backend_proximity_factors`**（DROID-W 新，支持 loop）/ `filter_edges` / `clear_edges` / `update_lowmem(enable_wq, enable_update_uncer, enable_udba, ...)`。

近邻度量：`droid_backends.frame_distance(poses, disps, intrinsics, ii, jj, beta)` 返回 bi-directional 光流幅度，用来做 NMS + 阈值化采样。

### 1.10 Lietorch SE(3) 约定
```python
SE3(data)                 # (...,7) [tx,ty,tz,qx,qy,qz,qw]
T_ij = T_j @ T_i.inv()
v    = dT.log()           # se(3), (6,) [ω, v]
T    = lietorch.SE3.exp(v)
Jac_ = T.adjT(J)
X_   = T[:,None,None] * X # 作用于齐次 (X,Y,Z,D)
```
可选 Sim(3)（带尺度），立体模式时用。

### 1.11 Mapper（DROID-W 独有，`mapper.py:52-100+`）
初始化 `GaussianModel`、在线优化、最终 20000 轮精化。Loss 项含 `alpha * RGB + (1-alpha) * SSIM`（默认 `alpha=0.5`）+ 深度 loss +（可选）不确定性加权。外部依赖：`thirdparty/{gaussian_splatting, diff-gaussian-rasterization-w-pose, simple-knn, depth_anything_v2}`。

### 1.12 ROS2 壳（`src/droid_w_ros2/`）
`droid_w_ros2/droid_w_node.py` 的核心逻辑：
```
_on_image (rclpy subscription)
  → buffer.push(image)                 # frame_buffer.py 线程安全 FIFO
  → 服务 ~/start 触发 _launch_slam：
      - BufferedRGB(cfg, frames)       # FIFO 冻结为数据集迭代器
      - SLAM(cfg, stream)              # 启动 DROID-W
      - threading.Thread(daemon=True) 跑 SLAM
_on_publish_tick (10 Hz wall timer)
  → video.poses[:counter.value] 快照 → 发 ~/odom + ~/kf_pose + ~/kf_path + TF
```
特点：**批处理**（frozen 缓冲后不能加新帧）、相机内参**不看 `CameraInfo`**、TUM 格式轨迹离线写盘、`max_frames=1500`。

### 1.13 关键文件速查
| 模块 | DROID-W 路径 | DROID-SLAM 对照 |
|------|--------------|------------------|
| 主协调 | `src/slam.py:27-100` | `droid_slam/droid.py:16-86` |
| Tracker 线程 | `src/tracker.py:15-145` | （无，合并于 droid.py） |
| Mapper 线程 | `src/mapper.py:52-100+` | **无** |
| Frontend | `src/frontend.py:6-149` | `droid_slam/droid_frontend.py:13-165` |
| Backend | `src/backend.py:19-121` | `droid_slam/droid_backend.py:9-89` |
| Factor graph | `src/factor_graph.py:9-413` | `droid_slam/factor_graph.py:17-157` |
| DepthVideo | `src/depth_video.py:24-100+` | `droid_slam/depth_video.py:12-74` |
| MotionFilter | `src/motion_filter.py:10-145` | `droid_slam/motion_filter.py:1-100` |
| BA | `src/geom/ba.py:48` | `droid_slam/geom/ba.py:31` |
| DroidNet | `src/modules/droid_net/droid_net.py` | `droid_slam/droid_net.py:146` |
| Mono 先验 | `src/utils/mono_priors/` | **无** |
| 动态不确定性 | `src/utils/dyn_uncertainty/` | **无** |
| YAML 配置 | `configs/{droid_w,Dynamic/*}.yaml` | **无**（命令行参数） |
| ROS2 节点 | `src/droid_w_ros2/droid_w_node.py` | — |

### 1.14 依赖与硬件
- Python 3.11+ / PyTorch 2.1+ / CUDA 12.1+（DROID-W 比原版新）
- `lietorch`、`diff-gaussian-rasterization-w-pose`、`simple-knn`、`mmcv`（Metric3D 需要）、`xformers`、`torch_scatter`
- GPU 显存：2–4 GB（仅 SLAM，900 帧 VGA）；开 Mapper 后 8+ GB
- 非实时；批处理模式

### 1.15 优缺点
| ✅ | ❌ |
|----|----|
| 端到端，动态 / 低纹理 / 反光场景 SOTA | 批处理，不实时 |
| 完整 YAML 配置（~100+ 参数），工程化强 | 硬件门槛高（PyTorch 2.1 + CUDA 12.1） |
| 支持回环、在线 BA、动态掩码、GS 建图 | 整序列驻留内存，`max_frames=1500` |
| 单目深度先验 + DINO 特征让野外场景可用 | 无 `CameraInfo` 自动读取，必须手配 YAML |

### 1.16 实战记录（TBD）
> 留给 benchmark 阶段填入（Bonn Dynamic / TUM RGB-D Dynamic 跑通情况、Mapper 开/关显存曲线、与 cuvslam 在相同数据集的 ATE 对比等）。

---

## 2. cuvslam_ros / cuVSLAM（深度分析）

> 2026-04-15 重写：聚焦上游 **cuVSLAM**（`/home/steve/Documents/GitHub/slam/cuVSLAM/`）。**注意**：之前文档把 cuVSLAM 当作闭源 SDK 写，**错了**——cuVSLAM 是 NVIDIA 完整开源的 VO/VI-SLAM 库，有 `libs/odometry`、`libs/sba`、`libs/map`、`libs/imu`、`libs/epipolar`、`libs/cuda_modules` 等完整实现。ROS 壳压到 §2.11。

### 2.1 算法流水线
```
Image/IMU
  ↓
SOF 特征检测 + 追踪            libs/sof/, cuda_kernels/{gftt,lk_tracker,st_tracker}.cu
  ├─ GFTT 角点（Harris）                cuda_kernels/gftt.cu
  ├─ LK Tracker（Lucas-Kanade）        cuda_kernels/lk_tracker.cu
  └─ ST Tracker（Sampling & Differentiability）  sof/st_tracker.h + cuda_kernels/st_tracker.cu
  ↓
MultiVisualOdometry / StereoInertialOdometry   libs/odometry/
  ├─ 运动预测（motion model / IMU）
  ├─ 初始化：Homography / Fundamental RANSAC   libs/epipolar/
  └─ PnP + Triangulation                        libs/pnp/multicam_pnp.h
  ↓
Pose 优化（Levenberg-Marquardt）
  ├─ 单帧 PnP                                   libs/pnp/multicam_pnp.h
  └─ 多帧 SBA（GPU Schur 补）                   libs/sba/, cuda_kernels/sba_v1.cu
  ↓
Map Management                                  libs/map/
  ├─ Keyframe Selection                         libs/sof/kf_selector.h
  ├─ Landmark Triangulation
  └─ Pose Graph
  ↓
SLAM / Loop Closure                             libs/slam/
  ├─ 位置识别 + PnP 定位
  └─ Pose Graph Optimization (PGO)
  ↓
输出：Pose / Map / Landmarks
```

对外入口头文件：`libs/cuvslam/cuvslam2.h`（`class Odometry` L336-479；`class Slam` L638-900）。

### 2.2 前端：GFTT 角点
```cuda
// libs/cuda_kernels/gftt.cu:61-66   Harris 最小特征值（对数形式）
__device__ __forceinline__ float GFTTMeasure(const float gxx, const float gxy,
                                             const float gyy) noexcept {
  const float D = __fsqrt_rn((gxx - gyy)*(gxx - gyy) + 4.f * gxy * gxy);
  const float T = gxx + gyy;
  const float eMin = (T - D) * 0.5f;
  return __logf(eMin + 1.f);
}

cudaError_t gftt_values(cudaTextureObject_t gradX, cudaTextureObject_t gradY,
                        float* values, uint2 image_size, cudaStream_t stream);
```
后处理：NMS + 8 倍下采样网格采样（`cuda_kernels/cuda_kernels.h:108-130`）。**不用 ORB/BRIEF 描述子**，仅靠帧间 KLT/ST 追踪维持对应关系——因此没有内建 BoW 式回环。

10 级高斯金字塔（`cuda_kernels.h:29-32`，`PATCH_DIM=9, PATCH_HALF=4.5, PYRAMID_LEVELS=10`），支持大位移追踪（搜索半径达 2048 px）。

### 2.3 前端：两种追踪器
**LK Tracker**（`cuda_kernels/lk_tracker.cu:22-25`）：
```cpp
#define MAX_POINT_TRACK_ITERATIONS 11
#define LK_CONVERGENCE_THRESHOLD   0.12f

cudaError_t lk_track(Pyramid prev_gradX, Pyramid prev_gradY,
                     Pyramid prev_img,   Pyramid cur_img,
                     TrackData* track_data, int num_tracks, cudaStream_t stream);
```
迭代最小二乘，9×9 patch，亚像素精度。

**ST Tracker**（`libs/sof/st_tracker.h:30-53`）：
```cpp
class STTracker : public IFeatureTracker {
  bool trackPoint(const GradientPyramidT& prev_grad, const GradientPyramidT& cur_grad,
                  const ImagePyramidT& prev_img,   const ImagePyramidT& cur_img,
                  const Vector2T& prev_uv, Vector2T& cur_uv,
                  Matrix2T& info,             // ← 直接输出 2×2 Fisher
                  float search_radius_px, float ncc_threshold) const override;
};
```
2 阶段（shift-only + full mapping），预缓存 patch 减少纹理读取。

### 2.4 初始化：对极几何
**Fundamental** RANSAC（`libs/epipolar/fundamental_ransac.h:30-48`）：
```cpp
class Fundamental : public math::HypothesisBase<float, Vector2TPair, Matrix3T, 8> {
    enum Criteria { Epipolar, Cheirality };
    void setOptions(const Criteria c, const float t = 0, const float a = 0, const bool e = true);
};
```
- 最小 8 点；`Epipolar` 判据 `x'Fx < t`，`Cheirality` 要求三角化点在两相机前；`e=true` 强制 `det(F)=0`。

**Homography** RANSAC（`libs/epipolar/homography_ransac.h:50`）—— 4 点最小集，单应分解成 4 种 (R,t) 组合选最优解。适合平面或低纹理场景。

通用框架：`libs/math/ransac.h:29-100`。

### 2.5 PnP / 单帧定位
```cpp
// libs/pnp/multicam_pnp.h:55-78
struct PNPSettings {
    bool  recalc_cov  = true;
    float lambda      = 1e-3;    // LM 阻尼
    float huber       = 2e-2;    // Huber robust
    int   max_iteration = 13;
    float cost_thresh  = 0.6;
};
class PNPSolver {
    bool solve(Isometry3T& rig_from_world, Matrix6T& static_info_exp,
               const std::vector<camera::Observation>& observations,
               const std::unordered_map<TrackId, Vector3T>& landmarks) const;
    void  build_hessian(const Isometry3T& rig_from_world, Matrix6T& H, Vector6T& rhs) const;
    float evaluate_cost (const Isometry3T& rig_from_world) const;
};
```
直接在 SE(3) 上 LM 优化，Huber 鲁棒核，输出信息矩阵（Hessian 的逆）。

### 2.6 多帧 SBA（核心算法）
```cpp
// libs/sba/bundle_adjustment_problem.h:34-68
struct BundleAdjustmentProblem {
    std::vector<Vector3T>   points;                       // 3D 地标
    int32_t                 num_fixed_points = 0;
    std::vector<Vector2T>   observation_xys;              // 2D 投影
    std::vector<Matrix2T>   observation_infos;
    std::vector<Isometry3T> rig_from_world;               // KF 位姿
    int32_t                 num_fixed_key_frames = 0;
    camera::Rig             rig;
    int32_t                 max_iterations   = 20;
    float                   robustifier_scale = 1.f;
    // 输出
    int32_t                 iterations    = 0;
    float                   initial_cost = 0, last_cost = 0;
};
```

**GPU Schur 补**（`libs/sba/schur_complement_bundler_gpu.h`）：
```cpp
class SchurComplementBundlerGpu {
public:
    bool solve(BundleAdjustmentProblem& problem);
private:
    class Impl;
    std::unique_ptr<Impl> impl;
};
```

CUDA kernel 流程（`cuda_kernels/sba_v1.cu`）：
1. **残差 + Jacobian**：每个 block 处理观测，算 `r_ij = π(R_i p_j + t_i) - u_ij`、`J = ∂r/∂ξ_i, ∂r/∂p_j`
2. **Hessian 块累加**（L~200-400）：稀疏块 `H_ii, H_ij`
3. **Schur 补**：消去点参数 `S = B^T A⁻¹ B`，解位姿增量 `Δξ`，回代求 `Δp = -A⁻¹(g_p + B Δξ)`
4. **点秩缺陷 SVD**（L52-71）：`class JacobiSVD` 用 Jacobi 旋转算 3×3 对称 SVD

**异步 SBA**：`Odometry::Config::async_sba = true`（`cuvslam2.h:406`）—— 后台线程跑 SBA，追踪线程不阻塞。

### 2.7 地图管理与回环
```cpp
// libs/map/map.h:64-117
class UnifiedMap {
public:
    void add_keyframe(int64_t time_ns, const State& state,
                      const IMUPreintegration& preint,
                      const std::vector<camera::Observation>& observations,
                      const std::vector<pipelines::Landmark>& triangulated_tracks);
    SubMap get_recent_submap(size_t max_last_keyframes, bool filter_landmarks = false) const;
    std::optional<Vector3T> get_gravity() const;
private:
    std::deque<KeyframeWithPreint> consecutive_keyframes;
    std::vector<LandmarkAndObserv> landmark_and_obs;   // 最多 600 个/KF
};
```

关键帧判据（`libs/sof/kf_selector.h`）：位移 > ~0.1 m、旋转 > ~5°、追踪点数显著下降。三角化：≥2 KF 观测 + DLT + cheirality 检查。

**回环 / PGO**（`libs/slam/`）：`Slam::Track(state)` 驱动位置识别 + PnP 验证新 KF 与旧地图 → 位姿图优化：
```cpp
// cuvslam2.h:720-745
struct PoseGraph {
    std::vector<PoseGraphNode> nodes;
    std::vector<PoseGraphEdge> edges;   // 相对姿态约束
};
```
具体 BoW 实现未在 `libs/slam/async_slam/async_slam.h` 和 `libs/slam/map/pose_graph/pose_graph.h` 里直接找到——可能是空间网格索引 + PnP 验证的组合，没有公开 DBoW 式词袋。

### 2.8 IMU 预积分（`libs/imu/imu_preintegration.h:34-106`）
```cpp
class IMUPreintegration {
public:
    Matrix9T    cov_matrix_ = Matrix9T::Zero();   // (dR, dV, dP) 9×9 协方差
    Matrix3T    dR = Matrix3T::Identity();
    Vector3T    dV = Vector3T::Zero();
    Vector3T    dP = Vector3T::Zero();
    // 对 bias 的 Jacobian
    Matrix3T    JRg, JVg, JVa, JPg, JPa;

    void IntegrateNewMeasurement(const imu::ImuCalibration& calib,
                                 const imu::ImuMeasurement& m);
    void SetNewBias(const Vector3T& new_gyro_bias, const Vector3T& new_acc_bias);
};
```
中点法或 RK4 积分；带 bias 修正 `ω_corr = ω_raw - b_g`，`a_corr = a_raw - b_a`；协方差 EKF 传播 `C = F·C·Fᵀ + G·Σ·Gᵀ`。

**紧耦合 VIO**：`libs/odometry/stereo_inertial_odometry.h` 把 IMU 约束作为 BA 额外项（`libs/imu/imu_sba.h` 的 `class IMUSBAMonoGPU`）。模式 `OdometryMode::Inertial` 启用。

IMU 噪声（`cuvslam2.h:199-214`）：`gyroscope_noise_density`、`gyroscope_random_walk`、`accelerometer_noise_density`、`accelerometer_random_walk`（Kalibr 约定）。

### 2.9 CUDA kernels 速查
| Kernel | 文件 | 用途 |
|--------|------|------|
| `gftt_values` | `gftt.cu` | Harris 角点强度 |
| `lk_track` | `lk_tracker.cu` | LK 光流追踪 |
| `st_track` | `st_tracker.cu` | ST 追踪 |
| `select_features` | `selection_v2.cu` | 网格采样 |
| `calc_residuals_kernel` | `sba_v1.cu` | 残差 + Jacobian |
| `calc_hessian_kernel` | `sba_v1.cu` | Hessian 块累加 |
| `schur_complement_kernel` | `sba_v1.cu` | Schur 补 |
| `conv_grad_x/y` | `convolutions.cu` | Sobel 梯度 |
| `gaussian_scaling` | `image_pyramid.cu` | 高斯下采样 |
| `sba_imu_*_kernel` | `sba_imu_v1.cu` | IMU 约束 BA |

### 2.10 与 ORB-SLAM3 / open_vins 的差异
| 维度 | cuVSLAM | ORB-SLAM3 | open_vins |
|------|---------|-----------|-----------|
| 特征 | **GFTT（Harris）无描述子** | ORB（FAST+BRIEF） | 可选 KLT / ORB |
| 追踪 | LK/ST GPU | KLT + 描述子匹配 | KLT 光流 |
| 初始化 | H/F RANSAC | H/F 分解 | MSCKF 静/动态 |
| VO | PnP LM + SBA（GPU） | 滑窗 BA（g2o） | MSCKF EKF |
| 后端 | GPU Schur 补 SBA | g2o / Ceres | error-state EKF |
| 回环 | PGO（BoW 具体实现不明） | DBoW2 完整 | 无 |
| GPU | **全 GPU** | 部分 GPU | 无 |
| 模式 | mono / stereo / RGBD / Inertial 多相机 | 单 / 双 / RGB-D ± IMU | 单 / 双 / 多 ± IMU |

**关键差异**：
1. **没有描述子** —— 仅依赖相邻帧 KLT/ST 追踪维持对应；没有 ORB-SLAM3 那种强鲁棒的重定位
2. **全 GPU 流水线** —— 从特征检测到 BA 都在 GPU，避免 CPU-GPU 同步开销
3. **原生 IMU 紧耦合** —— OpenVINS 风格，但用 SBA 而非 EKF；ORB-SLAM3 的 IMU 是后加的

### 2.11 ROS2 壳（`src/cuvslam_ros/`）
`CuvslamROS` 继承 `rclcpp::Node`，构造流程：`declare_parameters → WarmUpGPU → setup_publishers → setup_camera_rig → setup_tracker → setup_subscribers`。

关键 SDK 调用（壳里）：
```cpp
// src/cuvslam_ros/cuvslam_ros.cpp:225-244
config.use_gpu             = true;
config.async_sba           = true;        // 硬编码启用
config.use_motion_model    = true;
config.odometry_mode       = Inertial | RGBD | Mono | Multicamera;
config.multicam_mode       = Precision;

// L416 Stereo / L492 Mono / L601 RGBD
estimate = odometry_->Track(images[, {}, depths]);
// L435 可选 SLAM 层
slam_pose = slam_->Track(odometry_->GetState());
// L662 IMU
odometry_->RegisterImuMeasurement(0, imu);
```

订阅（`ApproximateTime` q=10）：stereo / mono / rgbd 三种配置 + 可选 IMU buffer（线程安全 `std::deque` + `std::mutex`，`drain_imu_until` 在 Track 前排空到当前图像时间戳）。

发布：`~/odom`、`~/pose`、`~/path`、`~/landmarks`、TF `odom_frame_id → base_frame_id`。执行器 `MultiThreadedExecutor`，IMU 与图像两个 `MutuallyExclusive` 回调组并发。

关键配置（50+ 参数）在 `config/params.yaml` / `config/geoscan_*.yaml`；所有相机内 / 外参、IMU 噪声、模式开关均参数化。

运行：
```bash
ros2 launch cuvslam_ros cuvslam.launch.py \
     config:=$(ros2 pkg prefix cuvslam_ros)/share/cuvslam_ros/config/params.yaml
# Composable
ros2 component load <container> cuvslam_ros cuvslam_ros::CuvslamROS \
     --ros-args -p odometry_mode:=inertial -p enable_slam:=true
```

> **注**：wrapper CMake 里 `CUVSLAM_SRC_DIR` 硬编码到 `$HOME/Documents/GitHub/slam/cuVSLAM`，迁移时需改。

### 2.12 代码入口速查
| 模块 | 文件 | 关键行 |
|------|------|--------|
| 对外 API | `libs/cuvslam/cuvslam2.h` | 40-900（`Odometry` / `Slam`） |
| VO 基类 | `libs/odometry/multi_visual_odometry_base.h` | 36-71 |
| 立体 VO | `libs/odometry/multi_visual_odometry.h` | 31-42 |
| 立体惯性 VO | `libs/odometry/stereo_inertial_odometry.h` | — |
| GFTT | `libs/cuda_modules/cuda_kernels/gftt.cu` | 61-66 |
| LK 追踪 | `libs/cuda_modules/cuda_kernels/lk_tracker.cu` | 22-100 |
| ST 追踪 | `libs/sof/st_tracker.h` | 30-53 |
| PnP | `libs/pnp/multicam_pnp.h` | 55-78 |
| BA 问题 | `libs/sba/bundle_adjustment_problem.h` | 34-68 |
| GPU BA | `libs/cuda_modules/cuda_kernels/sba_v1.cu` | 1-400+ |
| Fundamental | `libs/epipolar/fundamental_ransac.h` | 30-100 |
| Homography | `libs/epipolar/homography_ransac.h` | 50-80 |
| 地图 | `libs/map/map.h` | 64-120 |
| IMU 预积分 | `libs/imu/imu_preintegration.h` | 34-106 |
| RANSAC 框架 | `libs/math/ransac.h` | 29-100 |
| ROS2 节点 | `src/cuvslam_ros/cuvslam_ros.cpp` | 构造 11-47 / Track 416/492/601 |

### 2.13 优缺点
| ✅ | ❌ |
|----|----|
| **完整开源**（不是闭源 SDK），算法可审阅 / 修改 | 没有描述子 → 重定位弱 |
| 全 GPU 流水线，Jetson 生态友好 | Jetson 之外机器要自己编译 CUDA 内核 |
| 原生多相机 + IMU，`Odometry::Config` 一个参数切模式 | 回环 BoW 细节不公开（至少 libs 里没看到 DBoW） |
| 异步 SBA，追踪线程不阻塞 | wrapper CMake 路径硬编码 |

### 2.14 实战记录（TBD）
> 留给 benchmark 阶段填入（EuRoC ATE、Orbbec Gemini 336 实机 CPU / 延迟、GeoScan stereo-inertial 数据集等）。

---

## 3. DPVO / Deep Patch Visual SLAM（深度分析）

> 2026-04-15 新增。源码 `/home/steve/vslam_ws/src/DPVO/`（带 `COLCON_IGNORE`，纯 Python 研究代码，不进 colcon 构建）。

### 3.1 定位
Princeton VL（Teed / Lipson / Deng，也就是 DROID 作者组）：
- **v1** DPVO — *Deep Patch Visual Odometry*，NeurIPS 2023
- **v2** DPV-SLAM — *Deep Patch Visual SLAM*，ECCV 2024（加 DBoW2 + pypose PGO 回环）

**核心创新**：用**稀疏 Patch**（每帧 80–96 个 3×3）替代 DROID 的**稠密光流**，速度 5× 提升、显存降 ~40%，精度可比。

### 3.2 整体架构
- 输入：单目图像流 + 相机内参
- 输出：关键帧轨迹 SE(3) + Patch 深度（稀疏）+（v2）回环修正后的全局轨迹
- 单线程前端（`dpvo.py` L20-474），v2 可选独立线程做长期 LC（`loop_closure/long_term.py`）

与 DROID 的结构对应：
| DROID | DPVO |
|-------|------|
| `DepthVideo` 共享张量 | `PatchGraph`（`patchgraph.py` L11-55） |
| `FactorGraph` | Patch 间稀疏图（`ii / jj / kk` 索引帧 i / 帧 j / Patch k） |
| 稠密光流 | **稀疏 Patch 追踪** |
| `CorrBlock` 4D all-pairs | `CorrBlock` 两级金字塔（`levels=[1,4]`, `radius=3`） |
| 稠密 BA（每像素深度） | **稀疏 BA**（每 Patch 一个逆深度） |

### 3.3 Patch 选择（`net.py:95-157` `Patchifier`）
```python
# 每帧随机采样 M=80~96 个 Patch（可选梯度偏置）
x = torch.randint(1, w-1, (n, patches_per_image), device="cuda")
y = torch.randint(1, h-1, (n, patches_per_image), device="cuda")

# 可选 GRADIENT_BIAS：取梯度幅值最大的位置
if strat == "GRADIENT_BIAS":
    g = self.__image_gradient(images)         # 4×4 池化梯度
    ix = torch.argsort(g, dim=1)              # 降序
    x = torch.gather(x, 1, ix[:, -patches_per_image:])
```
每个 Patch 编码 `(x, y, inv_depth)`，Tensor 形状 `[B, N·M, 3, 3, 3]`（B=batch，N=帧，M=Patch/帧）。

两级特征提取：
| 分支 | 用途 | 维度 |
|------|------|------|
| `fnet`（`BasicEncoder4`, InstanceNorm） | 相关性 | `[B, pmem·M, 128, 3, 3]` |
| `inet`（`BasicEncoder4`） | 上下文 | `[B, pmem·M, 384]` |

### 3.4 Update 算子（`net.py:27-92`）
每轮迭代：
```python
net = net + inp + self.corr(corr)                  # 相关性注入
net = net + self.c1(mask_ix * net[:, ix])          # 空间邻域聚合
net = net + self.c2(mask_jx * net[:, jx])          # 时序邻域聚合
net = net + self.agg_kk(net, kk)                   # Patch 软聚合 (scatter_softmax)
net = net + self.agg_ij(net, ii*12345 + jj)        # 边聚合
net = self.gru(net)                                # 2 层 GatedResidual GRU
```
输出：`delta`（Patch 中心 Δ(x,y)）+ `weight`（2-dim 鲁棒权重）。深度更新由 BA 层统一处理。

### 3.5 Patch Graph（`patchgraph.py`）
```python
self.patches_     = torch.zeros(N, M, 3, 3, 3)   # Patch 坐标 + 逆深度
self.poses_       = torch.zeros(N, 7)             # [tx,ty,tz,qx,qy,qz,qw]
self.intrinsics_  = torch.zeros(N, 4)
self.net          = torch.zeros(1, E, DIM)        # 边特征
# 稀疏边（索引）
self.ii / self.jj / self.kk                       # 帧 i → 帧 j，Patch k
self.target / self.weight                         # 重投影目标 + 权重
```

边添加规则（`dpvo.py:362-375`）：
- `__edges_forw()` 前向：当前帧 Patch 投到之前 `PATCH_LIFETIME=12~13` 帧
- `__edges_back()` 后向：历史 Patch 投到新帧
- **v2 回环边** `edges_loop()`（`patchgraph.py:56-82`）：跨远距离的旧 Patch 通过流量筛选加入

关键帧管理（`dpvo.py:266-310`）：光流 `< KEYFRAME_THRESH=12.5~15` 时边缘化较早关键帧；滑窗 `REMOVAL_WINDOW=20~22`（保留活跃）、`OPTIMIZATION_WINDOW=10~12`（BA 范围）。

### 3.6 Sparse BA（核心创新）
两层实现：
- **纯 PyTorch**（`ba.py:12-77`）—— Cholesky + Schur 补消深度，保留 6×6 位姿 Hessian
- **CUDA 加速**（`fastba/ba.py:7-8`）—— `cuda_ba.forward(poses, patches, intrinsics, target, weight, lmbda, ii, jj, kk, t0, t1, M, iterations)`

关键区别于 DROID：
| | DROID 稠密 BA | DPVO 稀疏 BA |
|---|---|---|
| 深度维度 | H·W 每像素一个 | M 每 Patch 一个（M=80~96） |
| Hessian 密度 | Schur 后仍有 H·W 块 | 远小，快得多 |
| λ damping | 1e-4 + 自适应 ep | `lmbda=1e-4`（固定 damping） |

### 3.7 主处理循环（`dpvo.py:377-474` `__call__`）
```python
# 1. 提 Patch
fmap, gmap, imap, patches = self.network.patchify(
    image, patches_per_image=80, centroid_sel_strat='RANDOM')

# 2. 边增添
self.append_factors(*self.__edges_forw())
self.append_factors(*self.__edges_back())

# 3. 初始化判据（n=8 帧时）
if self.motion_probe() < 2.0:       # 运动不足 → 跳帧
    self.pg.delta[t] = (t-1, Id)

# 4. 迭代更新（首次 12 轮 + 每帧 1 轮）
self.update()      # → reproject + corr + network.update + fastba.BA
```

初始位姿：**DAMPED_LINEAR 运动模型**（L411-424），前两帧外推 + `MOTION_DAMPING=0.5` 缓冲快速加速。

### 3.8 回环（v2 扩展，`loop_closure/long_term.py`）
- **检索**：DBoW2 地点识别（图像哈希索引）
- **匹配**：**DISK + LightGlue** 关键点（每图 2048 个特征）
- **优化**：**pypose Sim(3) 对齐 + 位姿图优化（PGO）**
- 触发：`GLOBAL_OPT_FREQ=15` 帧一次

### 3.9 关键配置
| 参数 | 默认 | 作用 |
|------|------|------|
| `PATCHES_PER_FRAME` | 80–96 | 每帧 Patch 数 |
| `PATCH_LIFETIME` | 12–13 | Patch 最大跨帧数 |
| `REMOVAL_WINDOW` | 20–22 | 活跃 Patch 窗口 |
| `OPTIMIZATION_WINDOW` | 10–12 | BA 优化范围 |
| `KEYFRAME_THRESH` | 12.5–15 | 关键帧剔除光流阈值 |
| `MOTION_DAMPING` | 0.5 | 初值外推衰减 |
| `MIXED_PRECISION` | True | FP16 推理 |

### 3.10 硬件门槛 & 依赖
- GPU：RTX 3090 级可跑 120 Hz @ 320×240；更小分辨率在 Jetson 也能实时
- 显存：1.2–1.8 GB（DROID 同输入 3–4 GB）
- Python + PyTorch + CUDA；自带 `DBoW2/`、`DPRetrieval/`、`DPViewer/` 第三方
- 权重 / 数据由 `download_models_and_data.sh` 拉

### 3.11 代码入口速查
| 文件:行 | 内容 |
|---------|------|
| `dpvo/dpvo.py:20-66` | `DPVO` 类初始化、缓冲管理 |
| `dpvo/dpvo.py:328-356` | `update()` —— 核心优化循环 |
| `dpvo/dpvo.py:377-474` | `__call__()` —— 帧入口 |
| `dpvo/patchgraph.py:11-55` | `PatchGraph` 数据结构 |
| `dpvo/patchgraph.py:56-82` | `edges_loop()` 回环边 |
| `dpvo/net.py:95-157` | `Patchifier` 特征 + Patch 采样 |
| `dpvo/net.py:27-92` | `Update` 递归算子 |
| `dpvo/net.py:176-184` | `VONet` 网络包装 |
| `dpvo/ba.py:12-77` | 纯 PyTorch 稀疏 BA |
| `dpvo/fastba/ba.py:7-8` | CUDA BA 入口 |
| `dpvo/blocks.py:31-47` | `SoftAgg` 图聚合 |
| `dpvo/loop_closure/long_term.py:20-80` | 回环 + PGO |
| `dpvo/config.py:1-38` | 默认配置 |

### 3.12 与本 workspace 既有 DROID-W 的关键差异
| 维度 | DROID-W（`droid_w_ros2`） | DPVO v2 |
|------|----------------------------|---------|
| 光流 | 稠密每像素 | **稀疏 Patch**（80/帧） |
| BA | 稠密（H·W 深度） | **稀疏**（M 深度） |
| 速度 | 25–40 Hz | **120+ Hz**（3090） |
| 显存 | 2–4 GB（含 Mapper 8 GB+） | **1.2–1.8 GB** |
| 回环 | `loop_closing` BA | DBoW2 + DISK/LightGlue + pypose PGO |
| 先验 | Metric3D 单目深度 + DINO 特征 | 无外部先验 |
| ROS | 有 `droid_w_ros2` | 无（需自行封装） |
| 适用 | 离线精度 + 动态场景 | 实时 / 嵌入式 |

### 3.13 优缺点
| ✅ | ❌ |
|----|----|
| 5× 加速、显存减 40%，精度仍接近 DROID | 无 ROS 集成；纯 Python 研究代码 |
| Patch 稀疏图便于理解和改 | 本 workspace 有 `COLCON_IGNORE`，暂未构建 |
| v2 自带 DBoW2 + pypose PGO 回环 | 单目 only，不含 IMU |

### 3.14 实战记录（TBD）
> 留给 benchmark 阶段填入（EuRoC / TUM-Mono / Orbbec 数据集 ATE、3090 vs Jetson FPS）。

---

## 横向对比

| 维度 | droid_w_ros2 / DROID-W | cuvslam_ros / cuVSLAM | DPVO / v2 |
|------|------------------------|-------------------------|------------|
| 类型 | 端到端 DL + **稠密** BA | 传统几何 + GPU 加速 | 端到端 DL + **稀疏 Patch** BA |
| 语言 | Python + CUDA backend | C++ + CUDA libs | Python + CUDA |
| 上游开源 | ✅ | ✅（NVIDIA） | ✅（Princeton VL） |
| 处理模式 | 离线批处理 | 流式实时 | 实时（可）+ v2 回环 |
| 相机支持 | 单目 RGB | mono / stereo / RGBD / 多 | 单目 |
| IMU | ❌ | ✅（紧耦合） | ❌ |
| 速度 | 25–40 Hz | 30–60 Hz | **120+ Hz**（3090） |
| 显存 | 2–4 GB（+Mapper 8+） | 0.5–1.5 GB | **1.2–1.8 GB** |
| 回环 / 全局 BA | loop_closing | `Slam::Track` + PGO | DBoW2 + pypose PGO |
| 先验 | Metric3D + DINO | — | — |
| 动态场景 | **SOTA** | 一般 | 中等 |
| ROS 生态 | ROS2 Jazzy | 标准 ROS2 + Composable | **无原生 ROS**（COLCON_IGNORE） |
| 典型用途 | 离线精修 + 动态 | 机器人 / 无人机 | 学术 benchmark / 嵌入式实时 |

### 选型建议
- **精度优先 + 动态场景**：`droid_w_ros2`（DROID-W + Metric3D + DINO）
- **实时 ROS 机器人 / Jetson**：`cuvslam_ros`（多相机 + IMU）
- **极致速度 / 小显存 / 离线**：`DPVO`（v2 带回环，单目 only）

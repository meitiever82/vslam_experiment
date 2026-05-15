# DPVO 系统总览(Phase 0+1)

> **范围**:`src/DPVO/`(princeton-vl/DPVO,v2 with DBoW2 PGO 长期回环)
> **生成方法**:codebase-analysis-skill (强制 [VERIFY:] 标签,所有断言可定位到文件:行)
> **生成时间**:2026-05-15
> **配套文档**:`01-data-structures.md`,`02-data-flow.md`,`03-algorithms.md`,`04-key-functions.md`

---

## 1. 项目身份(What is DPVO)

DPVO = **Deep Patch Visual Odometry**,Princeton-VL 出品,**单目深度 VO** —— 不是完整 SLAM,但 v2 加了 DBoW2 长期回环 + PGO 后向 PoseGraphOptim,功能上接近 SLAM。

**核心思想**(对比传统 ORB-SLAM):
- 传统:稀疏 ORB 特征点 + 描述子匹配 + g2o BA
- DPVO:**稀疏 patch**(3×3 像素小块)+ CNN feature map 4D correlation + **可微 Schur BA**
- Patch 数固定(每帧 96 个,RANDOM 或 GRADIENT_BIAS 选 centroid),不像 ORB 取决于场景纹理
- Update 模块用 GRU + SoftAgg 在 patch 图上迭代,每步输出 (Δtarget, weight)
- BA 同时优化 pose(6 DoF) + 逆深度(1 DoF / patch),不优化 patch 描述子

**性能数字**(来自 README,未在本机验证):
- ~120 FPS on RTX 3090 with default config
- EuRoC MH avg APE 0.13m,TartanAir avg 0.65m
- 4060 实测在 GeoScan B1 上约 ~12 FPS(见 `06-benchmarks.md` DPVO 行)

[VERIFY: src/DPVO/README.md] 项目自述

---

## 2. 仓库结构

```
src/DPVO/
├── dpvo/                       核心 Python 包(1888 LOC)
│   ├── dpvo.py            473  DPVO 主类,track 入口
│   ├── net.py             273  VONet,Patchifier + Update GRU
│   ├── extractor.py       264  BasicEncoder/BasicEncoder4 (ResNet-like)
│   ├── ba.py              182  Python BA (Schur complement, autograd)
│   ├── projective_ops.py  130  iproj/proj/transform + jacobians
│   ├── blocks.py          118  GatedResidual / SoftAgg / GradClip
│   ├── patchgraph.py      111  PatchGraph(所有状态张量)
│   ├── stream.py           89  image_stream/video_stream(离线)
│   ├── utils.py            88  Timer / coords_grid / flatmeshgrid
│   ├── plot_utils.py       64  轨迹绘制
│   ├── logger.py           58  TensorBoard wrapper(训练用)
│   ├── config.py           38  yacs CfgNode 默认值
│   ├── altcorr/                CUDA correlation lookup
│   │   ├── correlation.cpp     pybind11 入口
│   │   ├── correlation_kernel.cu  332 行,corr + patchify forward/backward
│   │   └── correlation.py      Python autograd wrapper
│   ├── fastba/                 CUDA BA (主路径,不接 IMU)
│   │   ├── ba.cpp              pybind11 入口 + Eigen sparse PGO solver
│   │   ├── ba_cuda.cu          616 行,reprojection_residuals_and_hessian
│   │   ├── block_e.cu          Schur block helpers
│   │   └── ba.py               Python thin wrapper
│   ├── lietorch/               SE3/Sim3 Lie 群(princeton-vl 自家库)
│   └── loop_closure/           v2 长期回环
│       ├── long_term.py    266  LongTermLoopClosure(DISK + LightGlue + RANSAC + PGO)
│       ├── optim_utils.py  243  Sim3 PGO LM solver
│       └── retrieval/          DBoW2 检索 + ImageCache
├── dpvo_ros_node.py        250  ROS2 节点(本仓库下游加的,见 §6)
├── demo.py                 104  离线 image-folder/video 入口
├── train.py                     训练入口
├── evaluate_{euroc,tum,kitti,tartan,icl_nuim,geoscan}.py  各数据集评测
├── config/
│   ├── default.yaml         20  默认 96 patch / 22 removal window
│   └── fast.yaml                fast 模式(更少 patch)
├── DBoW2/                       submodule:DBoW2 词袋检索
├── DPRetrieval/                 submodule:DPVO 自己的 retrieval wrap
├── DPViewer/                    submodule:Pangolin 可视化
├── Pangolin/                    submodule:viewer 依赖
├── thirdparty/eigen-3.4.0/      Eigen 头(BA cpp 用)
├── COLCON_IGNORE                空文件 → 不让 colcon 编(用自家 .venv)
├── setup.py                     pip install -e . 装 CUDA 扩展
├── dpvo.pth                14M  预训练权重(EuRoC + TartanAir mix)
└── .venv/                       Python 3.10 隔离环境
```

[VERIFY: src/DPVO/ ls 输出] 目录树
[VERIFY: src/DPVO/COLCON_IGNORE] 0 字节标记文件 → workspace 约定:colcon 跳过 DPVO,因为它有自己的 .venv 和 CUDA 扩展(见 CLAUDE.md `Packages marked COLCON_IGNORE` 段)

---

## 3. 数据流粗略示意(Phase 3 详细版见 `02-data-flow.md`)

```
ROS2 /left_camera/image      (10 Hz, 1280×1024 fisheye on GeoScan B1)
        │ cv2.remap (fisheye → pinhole, dpvo_ros_node.py:119)
        ▼
RGB tensor (3, H, W)         (H,W 已裁到 16 倍数)
        │ DPVO.__call__(tstamp, image, intrinsics)
        ▼
Patchifier (net.py:95)       fnet + inet 双流 ResNet → fmap (128 ch) + imap (DIM=384 ch)
        │ patchify (altcorr CUDA): 每帧采 96 个 patch (3×3)
        ▼
PatchGraph (patchgraph.py:11)
  ├── poses_   [4096, 7]     SE3,世界→相机,xyz+quat (w 末位)
  ├── patches_ [4096, 96, 3, 3, 3]  每 patch:(u,v,d) × 3×3 grid
  ├── intrinsics_ [4096, 4]  (fx,fy,cx,cy) / RES=4
  ├── ii,jj,kk  edge lists   ii=patch 源帧, jj=目标帧, kk=patch index
  ├── target [B, |edges|, 2] BA 目标像素坐标(Update GRU 输出)
  ├── weight [B, |edges|, 2] BA 权重(σ⁻¹,Update GRU 输出)
  └── net    [B, |edges|, 384]  GRU 隐藏态
        │ Update GRU 迭代 (net.py:74)
        ▼
fastba.BA (CUDA) — 残差 = target - reproject(poses, patches), 2D × 2 行 × 6 DoF pose
        │ in-place 改写 poses_ 和 patches_[..., 2] (深度通道)
        ▼
keyframe (dpvo.py:266) — 用 flow_mag 判断是否丢中间帧
        ▼
最终 self.terminate() → (poses[N,7], tstamps[N]) TUM 格式
```

详见 `02-data-flow.md`,本图只是骨架。

---

## 4. 关键算法点位(Phase 4 详见 `03-algorithms.md`)

| 算法 | 文件:行 | 备注 |
|------|---------|------|
| Patch BA Schur complement(Python,可微) | `dpvo/ba.py:86-182` | 训练用,推理 `__run_global_BA` 也走这条 |
| CUDA BA(主推理路径) | `dpvo/fastba/ba_cuda.cu:232-376` | reprojection_residuals_and_hessian,**2D 残差 + 6 DoF pose Jacobian 硬编码** |
| 投影 + 解析 Jacobian | `dpvo/projective_ops.py:53-108` | transform(...) 返回 (x1, valid, (Ji, Jj, Jz)) |
| Update GRU + SoftAgg | `dpvo/net.py:27-92` | 输入 (net, ctx, corr) → (Δtarget, weight) |
| 4D correlation lookup | `dpvo/altcorr/correlation_kernel.cu:332 行` | CUDA,corr + patchify |
| 关键帧判定 + 帧删除 | `dpvo/dpvo.py:266-310` | flow_mag < KEYFRAME_THRESH 就丢中间帧 |
| 长期回环(v2) | `dpvo/loop_closure/long_term.py:140-266` | DBoW2 → DISK+LightGlue → RANSAC Umeyama Sim3 → PGO LM |
| Sim3 PGO LM 求解 | `dpvo/loop_closure/optim_utils.py:202-243` | 用 `cuda_ba.solve_system`(Eigen sparse Cholesky) |
| 运动模型(damped linear) | `dpvo/dpvo.py:410-424` | (P1·P2⁻¹)^(MOTION_DAMPING·Δt) 外推下一帧 pose |

[VERIFY: src/DPVO/dpvo/fastba/ba_cuda.cu:313-374] CUDA kernel 显式两行 `for (int row=0; row<2; row++)`,row=0 处理 x 残差,row=1 处理 y 残差;Jacobian `Jj` 维度固定 6;原子加到 `B[6*ix+i][6*ix+j]`。**这就是为什么 IMU 残差进不了 CUDA 路径**(`08-dpvio-design.md` 决策 D2 的代码证据)。

---

## 5. 模块依赖图(import 关系,grep `^from` / `^import`)

```
                        ┌─────────────┐
                        │   demo.py   │  入口 A:离线
                        └──┬──────────┘
                           │
       ┌───────────────────┼─────────────────────┐
       │                   │                     │
       ▼                   ▼                     ▼
  ┌────────┐          ┌────────┐           ┌────────┐
  │ stream │          │ config │           │  dpvo  │  主类
  │ .py    │          │ .py    │           │ .py    │
  └────────┘          └────────┘           └──┬─────┘
                                              │
   ┌──────────┬──────────┬──────────┬─────────┼──────────┬──────────┐
   ▼          ▼          ▼          ▼         ▼          ▼          ▼
┌──────┐ ┌─────────┐ ┌────────┐ ┌────────┐ ┌─────┐ ┌──────────┐ ┌────────────┐
│ net  │ │ pat-    │ │ ba.py  │ │ projec │ │ fast│ │ altcorr  │ │ loop_      │
│ .py  │ │ chgraph │ │(python)│ │ tive_  │ │ ba  │ │  CUDA    │ │ closure/   │
│ +    │ │ .py     │ │        │ │ ops    │ │ CUDA│ │          │ │ long_term  │
│extract│ │         │ │        │ │ .py    │ │     │ │          │ │            │
└──────┘ └─────────┘ └────────┘ └────────┘ └─────┘ └──────────┘ └────────────┘
                                                                       │
                                                                       ▼
                                                          ┌────────────────────────┐
                                                          │ DBoW2 + DISK + LightGlue│
                                                          │ retrieval/             │
                                                          └────────────────────────┘

  ┌────────────────────┐
  │ dpvo_ros_node.py   │  入口 B:ROS2(本仓库下游加)
  └────────┬───────────┘
           │
           ▼ 同样进 DPVO.__call__
```

[VERIFY: src/DPVO/demo.py:1-15] demo 导入
[VERIFY: src/DPVO/dpvo_ros_node.py:1-23] ROS 节点导入
[VERIFY: src/DPVO/dpvo/dpvo.py:1-11] DPVO 类导入

---

## 6. 入口对比

### 入口 A:离线 `demo.py`

```python
# [VERIFY: src/DPVO/demo.py:24-56]
@torch.no_grad()
def run(cfg, network, imagedir, calib, stride=1, skip=0, viz=False, timeit=False):
    slam = None
    queue = Queue(maxsize=8)
    reader = Process(target=image_stream, args=(queue, imagedir, calib, stride, skip))
    reader.start()
    while 1:
        (t, image, intrinsics) = queue.get()
        if t < 0: break
        image = torch.from_numpy(image).permute(2,0,1).cuda()
        intrinsics = torch.from_numpy(intrinsics).cuda()
        if slam is None:
            slam = DPVO(cfg, network, ht=H, wd=W, viz=viz)
        slam(t, image, intrinsics)
    ...
    return slam.terminate(), (points, colors, ...)
```

- 多进程:reader 在子进程读图,主进程跑 SLAM。Queue 大小 8。
- `t` 是 **整数帧序号**(`enumerate(image_list)`),不是 wall-clock 时间戳。这是 DPVO 没有时间维度的根本原因 —— **`08-dpvio-design.md` S1 要解决的就是这件事**。
- 输出:`slam.terminate()` 返回 `(poses[N,7], tstamps[N])`,poses 是 SE3 c2w(因为 `terminate` 里做 `.inv()`),tstamps 是整数帧 id。

### 入口 B:在线 `dpvo_ros_node.py`(本仓库添加,非 upstream)

```python
# [VERIFY: src/DPVO/dpvo_ros_node.py:94-144]
@torch.no_grad()
def image_cb(self, msg):
    ...
    img_undist = cv2.remap(img, MAP1, MAP2, cv2.INTER_LINEAR)   # fisheye → pinhole
    image_t = torch.from_numpy(img_undist).permute(2, 0, 1).cuda()
    intrinsics_t = torch.from_numpy(INTRINSICS).cuda()
    ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9  # wall-clock
    self.timestamps.append(ts)
    t = len(self.timestamps) - 1                                 # 帧序号
    if self.slam is None:
        self.slam = DPVO(cfg, self.network_path, ht=image_t.shape[1], wd=image_t.shape[2], viz=False)
    self.slam(t, image_t, intrinsics_t)                          # 注意:传的还是 t (序号)
    if self.publish_pose and self.slam.n > 0:
        self._publish_latest_pose(msg.header.stamp)              # 发布时再补 wall-clock
```

**关键点**:
- ROS 节点在外面记 `self.timestamps[]` 把帧序号 ↔ wall-clock 映射存下来。
- 传进 `DPVO.__call__` 的 `tstamp` 仍是序号 t,DPVO 内部并不知道 wall-clock。
- 发布 `/dpvo/pose` 时再从 `msg.header.stamp` 取 wall-clock。这就是 `07-dpvo-glim-bridge.md` Scheme C bridge 能拿到 DPVO 姿态时间戳的原因。
- 标定硬编码在文件顶 `FX=465.30..., FY=464.44...`,fisheye 畸变系数 4 个:Kalibr `equidistant` 模型(`KannalaBrandt`),来自 GeoScan B1 左相机(参考 `memory/project_geoscan_benchmark.md`)。
- QoS:`BEST_EFFORT`(GeoScan bag replay 用 BEST_EFFORT 推图,见 `dpvo_ros_node.py:69-73`)。
- 输出 `/dpvo/pose` 是 **c2w**(`SE3(pose_w2c).inv()`),与 TUM 习惯一致。
- 内部 `pg.poses_` 是 **w2c**(`08-dpvio-design.md` §3 已记)。这是 DPVO 多个文件读 pose 时常被踩的坑。

[VERIFY: src/DPVO/dpvo_ros_node.py:149-173] `_publish_latest_pose` 的 inv() 转换

---

## 7. 配置参数(`config/default.yaml` vs `config.py` 默认)

`config.py` 是基线默认,`default.yaml` 是发布的"推荐"覆盖,两者关键差异:

| 参数 | config.py 默认 | default.yaml 覆盖 | 含义 |
|------|----------------|-------------------|------|
| `PATCHES_PER_FRAME` | 80 | **96** | 每帧 patch 数 |
| `REMOVAL_WINDOW` | 20 | **22** | 老 edge 移出窗口的帧数 |
| `OPTIMIZATION_WINDOW` | 12 | **10** | 局部 BA 窗口大小 |
| `PATCH_LIFETIME` | 12 | **13** | patch 最多被多少帧看到(forward edge 范围)|
| `KEYFRAME_THRESH` | 12.5 | **15.0** | flow_mag 低于此值丢中间帧 |
| `BUFFER_SIZE` | 4096 | 4096 | poses_/patches_/intrinsics_ 容量上限 |
| `KEYFRAME_INDEX` | 4 | 4 | 看 `n-KEYFRAME_INDEX` 是否可丢 |
| `MOTION_MODEL` | 'DAMPED_LINEAR' | 'DAMPED_LINEAR' | 运动外推 |
| `MOTION_DAMPING` | 0.5 | 0.5 | (P1·P2⁻¹)^0.5 等效"半线性外推" |
| `MIXED_PRECISION` | True | True | fp16 推理 |
| `CENTROID_SEL_STRAT` | 'RANDOM' | 'RANDOM' | patch 中心选取(GRADIENT_BIAS 是另一选项)|
| `LOOP_CLOSURE` | False | (未覆盖,= False) | v2 简单回环(append loop edges 到 graph)|
| `CLASSIC_LOOP_CLOSURE` | False | (未覆盖,= False) | v2 全程 PGO 长期回环 |
| `BACKEND_THRESH` | 64.0 | (未覆盖) | loop edge flow_mag 阈值 |
| `MAX_EDGE_AGE` | 1000 | (未覆盖) | LC 候选 patch 最大年龄 |
| `GLOBAL_OPT_FREQ` | 15 | (未覆盖) | 每 15 帧考虑加 loop edges |

[VERIFY: src/DPVO/dpvo/config.py:1-38] 全部默认值
[VERIFY: src/DPVO/config/default.yaml:1-20] yaml 覆盖项

> **本仓库 dpvo_ros_node.py 启动时用 `--backend_thresh 64.0` 覆盖,其他默认**。

---

## 8. 关键架构约束(为后续修改必读)

### 8.1 Patch graph 是 **滑动窗口**,不是全局图

- `pg.poses_` 容量是 `BUFFER_SIZE=4096` 帧,但实际"活跃"区间是 `[max(0, n-OPTIMIZATION_WINDOW), n)`。
- 早于 `n - REMOVAL_WINDOW` 的 edge 被移除(`dpvo.py:305-310`),pose 仍留在 `poses_` 但不再被 BA 优化。
- **丢帧**(`keyframe`)会真删:`poses_[k:n-1]` 整体左移一格,`pg.delta[t1] = (t0, dP)` 存被删帧到前一帧的相对位姿,terminate 时插值还原。

[VERIFY: src/DPVO/dpvo/dpvo.py:266-310] keyframe 实现
[VERIFY: src/DPVO/dpvo/dpvo.py:166-171] `get_pose(t)` 通过 `delta` 字典递归回溯被删帧

### 8.2 时间维度只有"帧序号",**没有 wall-clock**

- `pg.tstamps_[i]` 存 `self.counter`(`dpvo.py:400`),`counter` 在 `__call__` 末尾 `+= 1`(`dpvo.py:440`)。
- 运动模型 `DAMPED_LINEAR` 处理变 fps 用 `(c-b)/(b-a)`(`dpvo.py:416`),其中 `a,b,c` 是最近三个 `self.tlist` 元素 —— `tlist` 存的是 `__call__` 传进来的 `tstamp` 参数,**可以是帧序号也可以是 wall-clock**,DPVO 不假设。
- **ROS 节点和离线 demo 都传序号,导致 `(c-b)/(b-a)` 在等帧率下恒等于 1**,运动模型实质退化为常 step。

[VERIFY: src/DPVO/dpvo/dpvo.py:399-419] tstamps 和运动模型
[VERIFY: src/DPVO/dpvo_ros_node.py:130-140] 节点传 `t = len(self.timestamps) - 1`(序号)进 `self.slam(t, ...)`

### 8.3 Pose 表示:7D `[tx,ty,tz,qx,qy,qz,qw]`,**w 在末位**

- `pg.poses_` 形状 `(BUFFER_SIZE, 7)`,初始化 `poses_[:,6] = 1.0`(`patchgraph.py:38`,quaternion identity)。
- 与 GTSAM `Pose3` 内部用 `(qw, qx, qy, qz)` 顺序不同 —— 跨库时必须重排。
- ROS `geometry_msgs/PoseStamped` 用 `(x, y, z, w)`,与 DPVO 一致。

[VERIFY: src/DPVO/dpvo/patchgraph.py:27,38] poses_ 张量形状和 identity 初始化
[VERIFY: src/DPVO/dpvo_ros_node.py:161-172] ROS 节点 unpack `tx,ty,tz,qx,qy,qz,qw`

### 8.4 内部 pose 语义:**world → camera**(`pg.poses_` 是 w2c)

- 推断证据:`dpvo.py:194-198` terminate 末尾 `poses.inv().data.cpu().numpy()` 返回前才取逆 → 输出是 c2w → 内部 `poses_` 是 w2c。
- `dpvo_ros_node.py:156-157` 也佐证:`pose_w2c = self.slam.pg.poses_[...]`(显式命名 w2c),再 `SE3(pose_w2c).inv()` 发布 c2w。
- `projective_ops.py:60` `Gij = poses[:, jj] * poses[:, ii].inv()` —— 把第 i 帧的世界点(经 ii 的相机 → 世界)推到第 j 帧相机 → 验证了 poses_ 是 w2c。

[VERIFY: src/DPVO/dpvo/dpvo.py:192] `poses.inv().data.cpu().numpy()`
[VERIFY: src/DPVO/dpvo/projective_ops.py:60] `Gij = poses[:, jj] * poses[:, ii].inv()`

### 8.5 第一帧 pose 固定为 identity,**不可改**(`fixedp=1`)

- CUDA BA 和 Python BA 都默认 `fixedp=1`:`ba.py:86`,`fastba/ba.py:7-8`(转发 t0/t1)。
- 这是 monocular VO 的标准做法:轨迹只有相对意义,固定第一帧消除 6 DoF gauge freedom。
- 后果:DPVO 输出 trajectory 总是从 origin (0,0,0) + identity orientation 出发。

[VERIFY: src/DPVO/dpvo/ba.py:86,133] `BA(..., fixedp=1, ...)` 和 `n = n - fixedp`

### 8.6 第一帧 patch 深度 **随机初始化**,init 期间靠 motion_probe 等运动够大

- `dpvo.py:427` `patches[:,:,2] = torch.rand_like(...)` 每帧 patch 的深度通道(第三个 channel)都先随机。
- `__call__` 第 442 行 `if self.motion_probe() < 2.0: ... return` —— 前两帧若运动 flow 中位数 < 2 px 直接退出(不入图),记到 `pg.delta` 等下次。
- 直到 `self.n == 8` 才宣告 `is_initialized = True`,并连跑 12 次 `update()` 做 cold-start BA(`dpvo.py:461-465`)。
- **这是为什么 GeoScan bag 必须 `--start-offset 10`**:静止启动期 motion_probe 永远 < 2,DPVO 卡在 init 阶段;skip 10s 给 IMU 和 motion 都热起来。

[VERIFY: src/DPVO/dpvo/dpvo.py:427-430] 深度随机
[VERIFY: src/DPVO/dpvo/dpvo.py:441-444] motion_probe 阈值 2.0
[VERIFY: src/DPVO/dpvo/dpvo.py:461-465] init 触发条件 n==8

---

## 9. Phase 4 计划:本次先深挖的算法

| 算法 | 输出位置 | 优先级 | 长度估计 |
|------|----------|--------|----------|
| Patch BA(Schur complement,Python 路径) | `03-algorithms.md` §1 | ⭐⭐⭐ | ~400 行 |
| 投影 Jacobian(SE3 解析推导)  | `03-algorithms.md` §2 | ⭐⭐⭐ | ~300 行 |
| Update GRU + SoftAgg(为什么用 patch 图 NN)| `03-algorithms.md` §3 | ⭐⭐ | ~200 行 |
| CUDA fastba 残差/Hessian 内核 | `03-algorithms.md` §4 | ⭐⭐ | ~200 行 |
| keyframe 删除策略 | `03-algorithms.md` §5 | ⭐ | ~100 行 |
| Sim3 PGO LM 闭环 | `03-algorithms.md` §6 | ⭐ | ~150 行 |

不深挖:retrieval/DBoW2 wrap(主要是 C++ 库 binding,价值低)、训练 loss(目前是推理场景)、`lietorch` 数学库(已成熟,跳过)。

---

## 10. 已发现的非平凡事实(供下游决策)

1. **CUDA BA 残差结构 2D 硬编码**:`ba_cuda.cu:313-374` 显式 `for (int row=0; row<2; row++)` + `Jj` 维度固定 6 + `B[6*ix+i][6*ix+j]` 原子加。任何想加 IMU 残差的项目(`08-dpvio-design.md` 是个例子)**必须走 Python BA 路径**或重写 CUDA kernel。
2. **`__run_global_BA` 走 Python BA**:`dpvo.py:312-326` 中 `fastba.BA(...)` 调的还是 CUDA fastba —— 即使是 global BA 也是 CUDA。Python `ba.py` 的 `BA` 函数主要用在训练(`net.py:260-261`)。
3. **patch graph 边的 (ii,jj,kk) 语义**:`ii` = patch 来源帧,`jj` = patch 被重投到的帧,`kk` = patch 在 `pg.patches_` 中的扁平 index。`kk` 对应 `ii` 满足 `ii == pg.ix[kk]`(`dpvo.py:218`)。
4. **Patch 5D 张量 (3, 3, 3)**:`pg.patches_[i, m]` 形状 `(3, 3, 3)`,前 channel = (x, y, d) 三层各 3×3:`patches[..., 0, :, :]` 是 x grid,`[1,:,:]` 是 y grid,`[2,:,:]` 是 disparity grid。BA 时只用中心点 `[1,1]`(`ba.py:96` `r = targets - coords[...,p//2,p//2,:]`)。
5. **Update GRU 不优化 patch 描述子**:权重和 Δtarget 是 update 输出,但 patch 的 9 个深度 grid `[2,:,:]` 都被 BA 用同一个 `dZ` retr(`ba.py:175-177`,只用 `disps` 一维 scalar 更新整个 grid)。意味着 patch 内部 9 个像素深度 **绑定为同一 disparity**,只是 BA 收到一个标量 dZ。
6. **`v2` 长期 LC 走 Sim3 而非 SE3**:`optim_utils.py:211-243` LM solver 优化 7D Sim3(SE3 + scale)。Visual mono → scale 不可观,LC 在 Sim3 自然。

---

## 11. 验证摘要

| 检查项 | 状态 |
|--------|------|
| 仓库结构 ls 输出对照 | ✓ |
| 主要文件行数 wc -l | ✓ |
| `dpvo/dpvo.py` 整文件读完 | ✓ |
| `dpvo/patchgraph.py` 整文件读完 | ✓ |
| `dpvo/ba.py` 整文件读完 | ✓ |
| `dpvo/net.py` 整文件读完 | ✓ |
| `dpvo/projective_ops.py` 整文件读完 | ✓ |
| `dpvo/fastba/ba.cpp` + `ba_cuda.cu` 关键 kernel 读到 | ✓ |
| `dpvo_ros_node.py` 整文件读完 | ✓ |
| `config/default.yaml` 读完 | ✓ |
| 长期回环 `loop_closure/long_term.py` + `optim_utils.py` 读完 | ✓ |
| `demo.py` 入口读完 | ✓ |
| 仍需读:`blocks.py`,`extractor.py`,`utils.py`,`stream.py` | ✓(已读完) |
| 未读:`train.py`,`evaluate_*.py`,`logger.py`,`plot_utils.py` | 跳过(训练/评测脚本,与算法主线无关) |
| 未读:DBoW2/DPRetrieval/DPViewer submodules | 跳过(C++ 第三方库 binding) |
| 未读:`altcorr/correlation_kernel.cu` 332 行 CUDA | 跳过(本文档只需要它"做相关查表"的抽象语义,内部 thread block 优化不上下文需要) |
| 未读:`lietorch_backends.cpython-310-x86_64-linux-gnu.so` | 不可读(binary)— 用 `lietorch` 接口语义即可 |

下一步:`01-data-structures.md`(Phase 2,详细枚举所有张量字段)。

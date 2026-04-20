# Benchmark 记录

本文件记录各 SLAM 系统在实际数据集上的运行情况、结果、踩坑和对比。
**首要数据集**：GeoScan B1（见 `memory/project_geoscan_benchmark.md`）。

**填表约定**：
- 状态：⬜ 未开始 / 🟡 调试中 / 🟢 跑通 / 🔴 失败（暂时放弃）
- ATE / RPE 用 `evo_ape` / `evo_rpe` 算；轨迹参考文件写清楚
- CPU / 延迟用 `top` / `htop` 或节点自带 stats

---

## 矩阵总览

**机器标注**(2026-04-20 加):
- 🟦 = DGX Spark pending（128GB 统一显存机器,待迁移后跑）
- 其他无标记条目 = 本机 4060 Laptop 8GB 能做,计划在本机完成

| 系统 \ 数据集 | GeoScan B1 mono-inertial | GeoScan B1 stereo-inertial | EuRoC V101 | TUM RGB-D |
|---------------|--------------------------|-----------------------------|------------|-----------|
| open_vins | ⬜ | ⬜ | ⬜ | — |
| sqrtVINS | ⬜ | ⬜ | ⬜ | — |
| mins | ⬜ (ROS1) | ⬜ (ROS1) | ⬜ | — |
| EPLF-VINS | ⬜ (ROS1) | — | ⬜ | — |
| VINGS-Mono 🟦 | — | — | ⬜ 🟦 | — |
| ORB_SLAM3_ROS2 | 🟢（参考基线） | 🟢 | ⬜ | ⬜ |
| VINS-Fusion-ROS2 | ⬜ | ⬜ | ⬜ | — |
| AirSLAM | ⬜ | 🟢 2026-04-16（Z 漂修复后 Z∈[0,1.1]m, 535 KF）| ⬜ | — |
| droid_w_ros2 / DROID-W 🟦 | ⬜ 🟦 (批处理) | — | ⬜ 🟦 | ⬜ 🟦 |
| cuvslam_ros | ⬜ | ⬜ | ⬜ | ⬜ |
| DPVO | 🟢 2026-04-17 mono-only, APE 0.60m (1000f) | — | 🟢 2026-04-20 V1_01 APE 0.046m (1435 pairs, stride=2) | — |
| MapAnything（前馈 multi-view）🟦 | 🔴 2026-04-20 s130 33 views: pose-only APE 24.6m; MVS 模式(喂 finder)mesh 散乱。结论：4060 稀疏采样不适用,**Spark 上用 stride=5/2 重跑** 🟦 | — | ⬜ 🟦 | — |
| VGGT-SLAM 🟦 | ⬜ 🟦 | — | ⬜ 🟦 | — |
| Gaussian-LIC 🟦 | — | ⬜ 🟦 即使 Spark 也要先 fork 适配 CUDA 13 / TensorRT 10 / ROS1→2 port,见 `memory/project_gaussian_lic_blockers.md` | — | — |
| GS_ICP_SLAM 🟦 | — | — | — | ⬜ 🟦 |
| SGS-SLAM 🟦 | — | — | — | ⬜ 🟦 |
| Mobile-GS 🟦 | — | ⬜ 🟦 离线 3DGS,走 ORB_SLAM3/AirSLAM KFs 导 COLMAP | — | ⬜ 🟦 |
| rgbdslam_v2 | — | — | — | ⬜ |
| rtabmap | 在 `~/rtabmap_ws/`，独立记录 | — | — | — |
| limap（3D 线后处理，非 SLAM） | — | 🟢 2026-04-16 finder VIO + BA hybrid 出 280 条 nv≥4（14× 纯 VIO，2.4× 纯 SfM） | ⬜ | — |
| slim-vdb（LiDAR+cam+VIO 语义体素） | — | 🟢 2026-04-18 1381 帧 stride3, 296万点云, 82×75×16m, Cityscapes-19 语义 | ⬜ | — |

> 单元格内容示例：`🟢 ATE 0.28m / CPU 42%` 或 `🔴 IMU init 失败，见 §2.3`

**🟦 = 等 DGX Spark**:详见 `memory/project_big_vram_blocked.md`,这些仓库在 4060 8GB 跑不动或跑出来是退化结果,实际 benchmark 数据到 Spark 上补。Spark 适配检查项在 `memory/project_spark_adaptation.md`(待用户首次部署时回填)。

---

## 数据集

### GeoScan B1（主）
- **bag**：`~/Documents/Datasets/geoscan/B1/2026-02-12-16-47-48/2026-02-12-16-47-48.db3`
- **标定**：`~/Documents/Datasets/geoscan/geoscan_camchain-{all,imucam}.yaml`
- **相机**：鱼眼，`equidistant` / `KannalaBrandt8` 模型
- **帧率**：10 fps（已知局限，见 docs/01 §6.14）
- **ground truth**：`KeyFrameTrajectory.txt`（ORB_SLAM3 mono-inertial 产物，**不是**严格 GT，只能做相对参考）
- **bag 播放约定**：`ros2 bag play <path> --clock --rate 0.5 --start-offset 10`（跳开头静止段）

### EuRoC MAV（备用）
- **bag**：`~/Documents/Datasets/vl/v101` 等
- **标定**：上游系统自带 YAML
- **有真实 GT**：Vicon 轨迹

### TUM RGB-D（备用）
- **bag**：`~/Documents/Datasets/tum/rgbd_dataset_freiburg1_xyz` 等
- **有真实 GT**

---

## 参考基线：ORB_SLAM3 mono-inertial on GeoScan B1

> 这是**目前唯一跑通**的系统，其余系统的 ATE / 轨迹都要和它对比。

### 启动命令
```bash
ros2 launch orbslam3_ros2 mono-inertial.launch.py \
    # 详见 ~/vslam_ws/src/ORB_SLAM3_ROS2/launch/mono-inertial.launch.py
```
配置：`~/vslam_ws/src/ORB_SLAM3_ROS2/config/monocular-inertial/GeoScan_*.yaml`（TBD：把实际文件名填进来）。

### 输出
`KeyFrameTrajectory.txt` 写入 bag 目录或 ORB_SLAM3 工作目录。TUM 格式：`timestamp tx ty tz qx qy qz qw`。

### 关键踩坑（已在 docs/01 §6.14 记录）
- 必须 `--start-offset 10`，否则 IMU 初始化 "Not enough motion"
- 10 fps + 鱼眼 + 室内转弯导致 **mono-inertial 频繁跟丢**；生产上建议 stereo-inertial
- VIBA 完成后基本稳定，但初始地图点 30–50（健康值 200+）

---

## 运行记录（按日期倒序）

### 2026-04-20 — MapAnything feedforward mono on GeoScan B1 cam0 🔴

**目标**：把 MapAnything(Meta, facebook/map-anything, ViT-g14 1B)当作"前馈 SfM"放到 GeoScan cam0 上,一次性回归稠密 pts3d + 相机位姿 + 度量尺度,和 finder / open-vins / ORB_SLAM3 轨迹做 ATE 对比。

**Pipeline**(脚本在 `src/vslam_experiment/scripts/mapanything/`):
1. `extract_cam0_undist.py`:bag(`/left_camera/image` 4183 帧, equidistant fisheye)→ `cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(balance=0)` + `remap` → 针孔 JPEG + ns 时间戳。
2. `run_mapanything_geoscan.py`:`mapanything.utils.image.load_images(resolution_set=518)` → 模型 bf16 → `model.infer(memory_efficient_inference=True, minibatch_size=1, use_amp=False)` → TUM 轨迹 + `run_stats.json`。
3. `evo_ape tum ref est -vas --t_max_diff 0.5`(Sim3 对齐,时间戳松 0.5s 因为我们抽帧的 10fps → 0.077fps / 0.3fps 跟 baseline 100Hz 不对齐)。

**结果**(stride=130,33 views,覆盖 416s):
| 参考 | ATE RMSE(m) | mean | median | max | Sim3 scale | 说明 |
|---|---|---|---|---|---|---|
| finder_localization | **24.61** | 22.61 | 22.37 | 41.74 | 0.964 | 32 pose pairs |
| open-vins | **24.22** | 22.28 | 22.24 | 40.25 | 0.960 | 32 pose pairs |

推理 13.6s / 33 views @ peak VRAM 4.27 GB,`metric_scaling_factor=9.94`(单场景常数)。

**核心教训**(写进 memory): **ViT-g14 encoder 不分片,跟 view 数线性耗显存**。RTX 4060 8GB 实测上限:
- 21 views → 3.55 GB OK
- 53 views → 8 GB+ OOM
- 105 views → OOM
所以 4060 最多跑 ~30 views,stride=10(目标 418 views)完全不可行。`memory_efficient_inference=True` 只影响稠密 prediction head,不影响 encoder;`minibatch_size=1` 同理无效。

**踩的坑**(顺时间):
1. **HF 下载 GFW 卡住**:`huggingface.co` 在国内会停在某个 blob 上 20+ 分钟不动,用 `HF_ENDPOINT=https://hf-mirror.com` 一发入魂,2GB/5 分钟下完。权重 ~4.6GB(ViT-g14)。
2. **ViT-g14 fp32 OOM**:`model = MapAnything.from_pretrained().to(cuda)` 直接吃 4.4 GB,加上 xorg 3 GB 在 8GB 上爆 。必须 `model.to(torch.bfloat16).to(cuda)`(在 CPU 上先 cast,再搬)。
3. **bf16 dtype 瘟疫**(这是主要复杂性,连环补丁):
   - 直接 bf16 模型 + fp32 输入:`patch_embed conv2d` 炸 `Input(float) vs bias(bfloat16)` → 手动 cast `view["img"].to(bf16)`。
   - cam_trans_encoder 用内部 dummy fp32 pose tensor:Linear 炸 `mat1 Float mat2 BFloat16` → `torch.set_default_dtype(torch.bfloat16)` 让内部 `torch.zeros()` 默认 bf16。
   - `recover_pinhole_intrinsics_from_ray_directions` 里 `torch.linalg.solve` 在 bf16 autocast 下 cusolver 没 bf16 LU → monkey-patch + `with autocast(enabled=False)` 强制 fp32 解 LU。
   - postprocess `.cpu().numpy()` 在 bf16 上 `TypeError: Got unsupported ScalarType BFloat16` → 全局 monkey-patch `torch.Tensor.numpy` 先 `.float()`。
4. **GLB / evo 的 scipy-numpy 2.x ABI 不兼容**:系统 scipy 1.8(apt)编译时绑的是 numpy 1.x,和 pip 装 mapanything 拉进来的 numpy 2.2.6 不合,`from scipy.spatial._ckdtree`崩 `dtype size changed`。`pip install --user 'scipy>=1.13'` 装 scipy 1.15.3 user-space 覆盖掉系统的即可。GLB 本次跳过(`--no_glb`)。

**24m ATE 为什么这么烂**:MapAnything 训练任务是密集重叠的多视图 SfM/MVS,期望 view 之间有强共视。我们 stride=130 两帧间隔 13 秒 + 走廊大视差,超出它有效工作假设。文档 `docs/05-feedforward-recon.md §2` 本来就写了"适合离散多视图重建,不适合长轨迹 SLAM"——这次是 empirical 验证。轨迹 xyz 分布 `x∈[-1,11] y∈[-5,0] z∈[-1,40]` 几乎 98% 集中在 Z 轴(相机光轴),完全没展开 finder 里看到的 62×64 m 平面移动,印证前馈模型在稀疏长轨迹上会退化为"相机沿光轴前进"的 prior。

**进一步测试:MVS 模式(喂外部位姿)**:怀疑轨迹错导致建图散,把 `finder_localization.txt` 当 camera_poses 输入(脚本加 `--pose_tum`),MapAnything 只负责稠密重建,不估 pose。得到 `full_s130_mvs_finder/scene.glb` 150 MB mesh(13.7s 推理, peak VRAM 4.30 GB, metric_scale=15.625)。**建图依然散乱** — 33 帧 @ stride=130 之间共视太少,MVS 也救不回来。**结论:feedforward 模型在这个稀疏采样下,pose 或 mesh 都不能用**。proper use case 要 stride=2~5(800-2000 views)+ A100 级显卡。

**MVS 模式的额外 bf16 坑**(在 pose-only 模式之外新发现):
- `camera_poses` 作为 (4,4) 矩阵传 → 模型内部 decomp 到 quat+trans 时 `einsum(R1_inv, trans1)` 混了 bf16/fp32 crash。改用 `(quat, trans)` tuple(bf16)形式直接传,绕开 decomp。
- `intrinsics` (bf16) → 内部 `get_rays_in_camera_frame` 算出 `ray_directions` 是 fp32,喂到 `_encode_and_fuse_ray_dirs` 的 bf16 buffer 里 `Index put` crash。Monkey-patch `preprocess_input_views_for_inference`,在它返回后把 `ray_directions` / `ray_directions_cam` / `depth_along_ray` / `cam_quats` / `cam_trans` 统一 cast 到 `img.dtype`。
- 还要注意 monkey-patch 必须打在 `mapanything.models.mapanything.model` 的模块命名空间里,因为它是 `from X import Y` 进来的 — 只 patch `mapanything.utils.inference.Y` 不生效。

**结论和清退方向**:feedforward multi-view 在这个 bag 上既不能做 pose 估计也不能做稠密重建,primary use case(稀疏长走廊)不适配。**4060 8GB 上前馈路线整个清退**,全部转大显存机器继续(A100 40/80GB 或 H100,候选 views 数:A100-40GB ~500 / 80GB ~1000 / H100-80GB ~1200)。详见 `memory/project_feedforward_big_gpu_todo.md`。

脚本已经备好(`src/vslam_experiment/scripts/mapanything/{extract_cam0_undist.py, run_mapanything_geoscan.py}`),到大显存机上要做的改造:
- 去掉本机的 bf16 monkey-patch 链(fp32 直接跑就行,不缺显存)
- 环境换 uv venv(快 + 不污染 pyenv-shim)
- stride=2~5 出 800-2000 views 的 MVS
- 走 `demo_colmap.py` 风格导出 COLMAP → gsplat 训 3DGS

**短期(继续在 4060)**:走线/点 SLAM 路线(AirSLAM stereo-inertial / ORB_SLAM3 / DPVO)+ slim-vdb 体素化,已跑通,不再试前馈。

**后续可尝试**:
- 切滑窗(每 30 views 一段,段间做 ICP/Sim3 对齐),会降低每段 ATE 但得拼接。
- 换 Apache 小模型`facebook/map-anything-apache`(同尺寸,同训练算力)不会省显存。
- 用低分辨率(缩小 `resolution_set` → 384 或 256)或更小的 base-model(如 Pi3-X 用 ViT-L 代替 ViT-g),能容纳更多 views。
- 真正想看"前馈 SfM"给 gsplat 的完整 pipeline,换到 A100 40GB 跑 full stride=10(419 views)再比。

**产物**:`~/vslam_ws/runs/mapanything_geoscan/`:
- `smoke_s200/` — 21 views 通路验证(pose-only)
- `full_s130/` — 33 views pose-only:`trajectory_mapanything.tum`, `eval_vs_{finder,openvins}.zip`, `traj_plot.png`
- `full_s130_mvs_finder/` — 33 views MVS:`scene.glb` 150 MB, `trajectory_mapanything.tum`(喂回的,不是估计的), `run_stats.json`

### 2026-04-15 — 文档初始化
- 建 `docs/06-benchmarks.md`
- 矩阵所有格子暂为 ⬜
- 待办：先配 open_vins + sqrtVINS 的 GeoScan mono-inertial

### 2026-04-15 — limap 3D 线三角化 on GeoScan B1 stereo-inertial 🔴
**目标**：把 ORB_SLAM3 stereo-inertial 的位姿喂给 limap，出 3D 线地图（车库 3000㎡ 期望几千条线）。

**镜像**：`limap:latest`（限定容器跑，见 `memory/project_limap_docker.md`）。

**pipeline**：ORB-SLAM3 KF/Frame 位姿 → `scripts/geoscan_prepare.py`（鱼眼→pinhole undistort + 按 TUM 轨迹时间戳匹配图像 + 写空 points3D 的 COLMAP sparse）→ `scripts/geoscan_colmap_tri.py`（pycolmap 顺序匹配 + point triangulator 填 sparse_tri）→ `runners/colmap_triangulation.py`（limap 正式入口）。

**实验矩阵**（同一 bag，输入都是 677 KF 除非另标）：
| 配置 | N2 | N4 | N6 | N8 |
|---|---|---|---|---|
| DeepLSD + wireframe + gluestick（默认）nv=4 | — | 73 | 22 | 6 |
| 同上，nv=2 | 128 | 73 | 22 | 6 |
| angle_threshold 1.0 → 0.5 | 135 | 76 | 23 | 6 |
| n_neighbors 20 → 30 | 128 | 73 | 22 | 6 |
| n_neighbors 20 → 50 | 128 | 73 | 22 | 6 |
| LSD 检测器 | 137 | 61 | 16 | 5 |
| **1735 帧（全帧，CameraTrajectory 抽出来）** | **110** | **55** | **3** | **1** |

**核心结论**：
- N2 硬上限 \~150，不管怎么调；track 构建阶段原始 candidate 就只有 \~310 条（日志 `tracks after iterative remerging: 293 / 295`），filter 之后剩 128。
- 输入足量：sparse_tri 重投影 0.8 px / 23936 点（KF 版）或 1.1 px / 62955 点（全帧版），每对平均 1350 个线匹配。
- **高帧密度反而降**：1735 帧出 110 条（相邻基线太短被 `line_tri_angle_threshold: 1.0` 毙掉 + 长 track 被切成碎片）。
- 车库低纹理 + ORB-SLAM KF 稀疏抽样 → 每个 feature 只能跨 \~5 帧（`点 track 中位 = 5`），线段跨视图一致性差，跟 limap 在稠密 SfM 场景（Tanks&Temples/ETH3D）设计假设不合。

**踩过的坑**（已并入 `memory/project_limap_docker.md`）：
- DeepLSD 权重在 cache 里 0 字节空壳（ETH 源被 GFW 打断），从 `src/limap/models/deeplsd_md.tar` 覆盖；验权重用 `stat -c %s` 看字节数，别只 `ls`。
- ORB_SLAM3 `SaveTrajectoryTUM` 在 IMU-init 反复 reset 后析构时 SIGSEGV（悬垂 KF 指针），已给 orbslam3_ros2 五个 node 加 try-catch 保护。
- ORB_SLAM3 `LocalMapping::CreateNewMapPoints` / `KeyFrameCulling` 运行时访问悬垂/已 cull KF → SIGSEGV。改了 `~/Documents/GitHub/slam/ORB_SLAM3/src/LocalMapping.cc`：`pKF2` 前加 null+isBad 守护，`KeyFrameCulling` 主循环先 isBad 再 mnId/GetMap，`aux_KF` prevKF 链路 isBad 守护。重编 `.so` 即可，wrapper 不动。
- ORB_SLAM3 shutdown 有时死锁（LocalMapping/LoopClosing 线程 futex_wait），`SaveTrajectoryTUM` 跑不到 → 兜底录 `/orb_slam3_stereo_inertial/path` topic，最后一条 Path 消息累积了全部帧位姿，写脚本抽出来转 TUM（参见 `vslam_ws/extract_traj.py`）。

**决定**：limap 这条路不适合当前 bag，不继续调参。转向 AirSLAM（线原生一等公民，在线建图，不靠离线 SfM 后处理）。**2026-04-16 结论更新**：发现问题不在 limap，在输入 pose 的精度——见下面 2026-04-16 条目，换 finder VIO + BA hybrid 后线数翻了一个数量级。

**产物**：
- 最佳 obj：`~/vslam_ws/src/limap/outputs/geoscan_B1/triangulated_lines_nv2.obj`（128 条）
- 全帧实验 obj：`outputs/geoscan_B1_allframes/`（110 条，留着对照）
- ORB-SLAM 全帧轨迹：`~/vslam_ws/FrameTrajectory.txt`（1735 帧，从 bag record 抽出）
- ORB-SLAM3 LocalMapping 补丁：`~/Documents/GitHub/slam/ORB_SLAM3/src/LocalMapping.cc`（本地改动，未上游）

### 2026-04-16 — AirSLAM stereo-inertial on GeoScan B1 🟢

**目标**：跑通 AirSLAM（线原生 VIO），作为 limap 失败后的替代线地图方案。

**数据**：同一个 bag（`2026-02-12-16-47-48`）。容器：`airslam:latest`（`src/AirSLAM/docker/run.sh`）。启动：
```bash
ros2 launch air_slam vo_ros.launch.py visualization:=false saving_dir:=...
# host:
ros2 bag play ~/Documents/Datasets/geoscan/B1/2026-02-12-16-47-48 --clock
```

**调试两个根因**（细节见 `memory/project_airslam_geoscan_rectify_patch.md`）：
1. **fisheye stereoRectify 退化**（2026-04-15 已修）：GeoScan equi 标定几乎是针孔，`cv::fisheye::stereoRectify` 返回 focal ≈ 1.28e-3 的 P 矩阵，stereo 匹配全被 `max_x_diff = bf/0.2 ≈ 6e-4 px` 过滤。补丁位于 `src/camera.cc:68-75`，检测 `P0[0,0] < 1.0` 退化后用原始 K0 + baseline 重建 P。
2. **IMU 加速度单位 g vs m/s²**（2026-04-16 新修）：`/handsfree/imu` 静止 `|acc|≈1.01`（g 单位，非 ROS 标准 m/s²），preintegration 错误地把 1g 当 1 m/s²，`ComputeVelocity` 解出 `|gw|≈1.01` 而不是 9.8，init 后 backend 用 `IMU_G_VALUE=9.80665` 传播，和 preintegration 实际量级不符 → 系统性 ~8.8 m/s² 竖直误差。修复：`demo/vo_ros_node.cpp::OnImu` 里 `msg->linear_acceleration.{x,y,z} * 9.80665`。

**结果**（两 bug 都修完后）：
- 535 keyframes，388s（bag 418s）
- XY 范围 58×60m（停车场约半圈）
- **Z 范围 [0, 1.1] m**（修前 492m）
- path length 336m
- `[IMU_INIT] |gw|=9.81`, `|g_world|=9.80665`, `acc_bias≈4e-3`, `gyr_bias≈7e-4`, er/ev/ep < 0.04
- trajectory + mapv0.bin 存 `src/AirSLAM/debug/zdrift_run3/`

**踩坑**：
- `saving_dir` 默认指 `install/.../debug` 不存在，`ofstream` 静默失败要传绝对路径
- `rviz/vo.rviz` ROS1→ROS2 迁移：class 名 `rviz/X` → `rviz_default_plugins/X`/`rviz_common/X`，所有 Topic 字段要嵌套 struct（备份 `vo.rviz.bak`）
- 诊断打印（保留）：`src/map.cc` 的 `[IMU_INIT] gw_raw/gw_dir/Rwg_before/Rwg_after/g_world`

**产物**：
- trajectory：`src/AirSLAM/debug/zdrift_run3/trajectory_v0.txt`
- map bin：`src/AirSLAM/debug/zdrift_run3/AirSLAM_mapv0.bin`（238 MB）
- 轨迹图：`src/AirSLAM/debug/zdrift_run3/traj_xy.png`
- map 回放器：`demo/map_viewer.cpp`（新增，1Hz 发 `/AirSLAM/{map,mapline,keyframe}`）

### 2026-04-16 — limap 3D 线 on GeoScan B1（finder VIO + BA hybrid）🟢

**目标**：重新跑 limap，用自研 VIO（finder，OpenVINS fork）轨迹替代 ORB-SLAM3，看线数能否跳出 2026-04-15 的 ~150 上限。

**流程**：
1. **bag → 523 KF**：`scripts/geoscan_prepare.py` 新加 `--stride 5 --pose_frame body`，从 `finder_localization.txt`（TUM 格式 IMU 体坐标系 2615 pose）每 5 个取 1 个，配 Kalibr `T_cam_imu` 乘出 `T_wc`。fisheye→pinhole undistort。
2. **COLMAP triangulation + BA**：`scripts/geoscan_colmap_tri.py --ba`。用 finder pose 作已知 pose 跑 `pycolmap.triangulate_points` 得 14503 点（reproj 1.54 px），再 `pycolmap.bundle_adjustment(options=CAUCHY(scale=2.0), refine_focal_length=False, refine_sensor_from_rig=False)` 精化，后置 `ObservationManager.filter_all_points3D(max_reproj_error=4, min_tri_angle=0.3)`。**不能 BA 前 filter，鲁棒损失自己处理 outlier**。
3. **limap**：`runners/colmap_triangulation.py -a ... -m sparse_tri -i images`。

**三路对比**（523 KF，同一组图像）：

| | finder 纯 VIO | COLMAP 纯 SfM | **finder + BA 精化** |
|---|---|---|---|
| 可用帧数 | 523 / 523 | 73 / 523（forward motion 视差不足）| **523 / 523** |
| COLMAP reproj | 1.55 px | 0.56 px | **0.37 px** |
| LIMAP nv≥4 | 20 | 117 | **280** |
| nv≥6 | 2 | 72 | 111 |
| nv≥8 | 0 | 46 | 42 |
| obj 大小 | 879 B | 15 KB | **36 KB** |

**核心结论**：
- 2026-04-15 上限 ~150 的根因不是 limap 本身，是 **pose 精度**——ORB-SLAM3 KF pose 的 reprojection 残差让线匹配/三角化大量失败。
- finder VIO 解决了「覆盖」（全 523 帧 vs COLMAP 73 帧），COLMAP BA 解决了「精度」（0.37px vs 1.55px），两者组合线数 14×。
- driving scene 小视差需要把 `min_tri_angle` 从默认 1° 放宽到 0.3°（连续帧 0.5m baseline 看 20m 处只有 ~1.4°）。

**踩坑**：
- BA 默认 `TRIVIAL` 损失会发散（reproj 跑到 1e+153），要换 `CAUCHY(scale=2.0)`
- 默认 `refine_focal_length=True` 对 PINHOLE 小视差 scene 也会发散，固定所有 intrinsic
- `Reconstruction` 没有 filter 方法，要 `ObservationManager(rec, db_cache.correspondence_graph).filter_all_points3D(...)`
- `geoscan_prepare.py` 写 `sparse/`，`geoscan_colmap_tri.py` 读 `sparse_poses/`，要 `ln -sfn sparse sparse_poses`

**产物**：
- 最佳 obj：`~/vslam_ws/outputs/limap_geoscan_finder_ba/limap_out/triangulated_lines_nv4.obj`（36 KB，280 条 nv≥4）
- 对比 obj：`outputs/limap_geoscan_finder/`（纯 VIO，20 条）、`outputs/limap_geoscan_colmap_only/`（纯 SfM，117 条）
- 新脚本：`scripts/geoscan_colmap_full_sfm.py`（纯 COLMAP 对照组）

### 2026-04-20 — DPVO mono on EuRoC V1_01_easy 🟢

**目标**:DPVO 在硬核 benchmark(EuRoC Vicon GT)上拿数,跟 GeoScan 的 0.60m 做对照。

**环境**:DPVO venv 补装 `plyfile`(1 行),其他不动。

**数据**:`GlowBond/EuRoC_MAV_Dataset` (HuggingFace, 6GB 一个 zip 含 V1 三段)通过 `HF_ENDPOINT=hf-mirror.com hf download` 抓下来(ETH 原站国内基本不通,卡 20 分钟都不下)。`vicon_room1.zip` 解出 `V1_01_easy.zip` 再解出 `mav0/cam0/data/*.png`(2912 帧)。GT 已随 DPVO 仓库提供(`datasets/euroc_groundtruth/V1_01_easy.txt`),calib 也现成(`calib/euroc.txt`)。

**跑法**:一次性脚本 `src/DPVO/run_v1_01.py`(改编自 `evaluate_euroc.py`,跳过 11 个场景循环,只跑 V1_01)。

```bash
cd ~/vslam_ws/src/DPVO
.venv/bin/python run_v1_01.py    # stride=2, default config
```

**结果**:
| 指标 | 值 |
|---|---|
| APE RMSE | **0.046 m** |
| APE mean / median / max | 0.043 / 0.041 / 0.087 m |
| 匹配 pose pairs | 1435 / 1456 est / 2871 ref |
| Sim3 scale 对齐 | 是 |
| 显存峰值 | ~6.6 GB |
| 推理耗时 | ~3 min(stride=2, 1456 frames) |

跟 DPVO paper 的 V1_01 ~0.04m 对得上,**这是目前工作区 ATE 最好的数字**。和 GeoScan 的 0.60m 一对比,就看出:**数据集质量压倒一切** — GeoScan 10fps + 鱼眼 + 室内大视差的场景,学习型纯单目里程计受限,EuRoC 20Hz + 小畸变 + 室内有纹理的场景就能干到 paper 水平。

**踩坑**(给将来 DPVO 跨数据集用):
1. `evaluate_euroc.py::run` 开子进程,外层 caller **必须**放 `if __name__ == "__main__"`,否则 multiprocessing spawn 炸。
2. DPVO 约定 EuRoC 时间戳是 **ns-as-float**(直接拿文件名当 float 用,不除 1e9),跟 finder/TUM 秒制不同。GT 文件 `datasets/euroc_groundtruth/V1_01_easy.txt` 也是 ns 制。写新 eval 脚本时别下意识除 1e9。
3. ETH 官方下载站 `robotics.ethz.ch/~asl-datasets/` 国内基本卡死,**HF mirror `hf-mirror.com` + GlowBond 的镜像** 是目前最靠谱路径。

**产物**:
- 轨迹:`src/DPVO/results/dpvo_euroc_v1_01.tum`(1456 poses,ns-as-float 时间戳)
- eval 脚本:`src/DPVO/run_v1_01.py`
- 原始数据:`src/DPVO/datasets/EUROC/V1_01_easy/`(~800MB);`vicon_room1.zip`(5.6GB,保留方便以后跑 V1_02/V1_03)

### 2026-04-17 — DPVO mono on GeoScan B1 🟢

**目标**：跑通 DPVO（学习型纯视觉里程计），与 finder VIO 轨迹对比。

**系统**：DPVO（princeton-vl/DPVO, commit 89f8b27 DBoW2）。纯单目，无 IMU，无回环。

**环境**：独立 venv `src/DPVO/.venv`（uv, Python 3.10, torch 2.3.1+cu121），不走 colcon。CUDA extensions（cuda_corr, cuda_ba, lietorch_backends）需 `pip install --no-build-isolation -e .` 编译，依赖 `thirdparty/eigen-3.4.0`（手动下载，非 submodule）。

**数据**：bag `/left_camera/image`（10 fps, 4183 帧）。ROS2 wrapper `dpvo_ros_node.py` 订阅图像，内部 `cv2.fisheye.initUndistortRectifyMap` 去畸变（Kalibr equidistant → pinhole），直接喂 DPVO。

**启动命令**：
```bash
# term A: DPVO node
cd ~/vslam_ws/src/DPVO
source /opt/ros/humble/setup.bash
.venv/bin/python dpvo_ros_node.py \
  --network dpvo.pth --config config/default.yaml \
  --save_trajectory results/dpvo_ros_geoscan_traj.txt

# term B: bag play
ros2 bag play ~/Documents/Datasets/geoscan/B1/2026-02-12-16-47-48 --topics /left_camera/image
```

**结果**（前 1000 帧，~101 秒）：
- 匹配 971 poses（vs finder_localization.txt，`evo_ape --t_max_diff 0.1`）
- **APE RMSE = 0.60m**，median = 0.24m，max = 3.60m（起始段初始化不稳）
- Scale correction: 18.5×（纯单目无绝对尺度，正常）
- 轨迹形状与 finder 高度一致（见 `results/dpvo_vs_finder_1000f_map.png`）
- 中后段（初始化稳定后）误差很小，起始段红区 max 3.6m

**对比参考**：
| 参考轨迹 | 匹配 poses | APE RMSE | APE median | scale |
|---|---|---|---|---|
| finder_localization.txt | 971 | 0.60m | 0.24m | 18.5× |

**踩坑**：
- limap 预处理的 523 帧（~1.3fps）跑出退化轨迹——帧率太低光流失效。必须用 bag 原始 10fps
- 全量 4183 帧超过 `BUFFER_SIZE=4096`，8GB 显存 OOM（在 ~4100 帧崩）。解决方案：用前 1000 帧或 stride 2（~2090 帧）
- ROS2 订阅队列要开大（5000），否则 bag 全速播 DPVO 处理不过来丢帧
- `pip install -e .` 需 `--no-build-isolation`（隔离环境找不到 torch）
- 依赖零散没有 requirements.txt：需 torch, torchvision, torch-scatter, einops, numba, pypose, opencv-python, matplotlib, scipy, evo, yacs, tqdm, gdown

**产物**：
- ROS2 wrapper：`src/DPVO/dpvo_ros_node.py`
- 轨迹：`src/DPVO/results/dpvo_ros_geoscan_traj.txt`（TUM 格式，1000 poses）
- 对比图：`src/DPVO/results/dpvo_vs_finder_1000f_map.png`
- calib：`src/DPVO/calib/geoscan.txt`（pinhole intrinsics）

### 2026-04-18 — slim-vdb LiDAR+Camera 语义体素地图 on GeoScan B1 🟢

**目标**：跑 slim-vdb（OpenVDB 概率语义体素融合框架），用 livox LiDAR 出几何、右目鱼眼 + SegFormer 出语义、finder VIO 出位姿，生成停车场的稠密语义 3D 地图。

**系统**：slim-vdb（umfieldrobotics/slim-vdb）+ openvdb slim-vdb 分支。全部在 `slimvdb_dev` docker 容器里编译运行（CUDA 12.1 + OpenCV 4.6 + Open3D 0.18，宿主 uid 1000 直通）。

**数据流**：
| 源 | 产物 |
|---|---|
| `/livox/lidar`（4183 帧，20064 点/帧）| `sequences/00/velodyne/*.bin` (float32 xyz+intensity, 1.3GB) |
| `/right_camera/image` + SegFormer-B0 Cityscapes | `sequences/00/velodyne_predictions_txt/*.txt` (每点 "inst sem") |
| `finder_localization.txt`（2615 TUM 位姿）| `poses.txt` (`T_world_velo = T_world_imu * T_imu_velo`) |

**关键外参**（GeoScan 标定文件）：
- Livox → 右目 (cam1)：`Rcl`/`Pcl`，~26° pitch（非重复扫描 Mid-360 的物理安装倾角）
- cam1 ↔ IMU：Kalibr `T_cam1_imu`（equi, 0.13px）

**启动命令**：
```bash
# 1. 抽取 LiDAR + poses
python3 src/slim-vdb/scripts_geoscan/extract_geoscan.py \
    --bag ~/Documents/Datasets/geoscan/B1/2026-02-12-16-47-48 \
    --poses ~/Documents/Datasets/geoscan/B1/2026-02-12-16-47-48/finder_localization.txt \
    --out_dir outputs/slimvdb_geoscan --sequence 00 --max_frames -1

# 2. 生成语义 labels（流式读 bag，避免 OOM）
src/DPVO/.venv/bin/python src/slim-vdb/scripts_geoscan/generate_labels.py \
    --seq_dir outputs/slimvdb_geoscan/sequences/00 \
    --bag ~/Documents/Datasets/geoscan/B1/2026-02-12-16-47-48

# 3. 跑 pipeline（容器内）
docker exec slimvdb_dev ./kitti_pipeline \
    --config config/kitti_geoscan.yaml --sequence 01 \
    outputs/slimvdb_geoscan outputs/slimvdb_mesh
```

**结果**（sequence 01 = stride-3，1381/4143 帧；voxel_size=0.15m，prune_interval=10）：
- TSDF 几何范围：**82×75×16m**（停车场一层走廊 + 上下层坡道）
- 2.96M 活跃体素，OpenVDB 稀疏存储只 **9MB in-core**
- **296 万**彩色点云（`_pointcloud.ply`，86MB）
- 19 个 per-class triangle mesh（class 0=road, 2=building, 等），共 ~720MB
- 集成吞吐：起手 17 FPS → 地图长大后 0.86 FPS，Render time 是瓶颈（随体素数线性增长，0.05s → 1.15s/帧）

**踩坑 / 修复**：
1. **openvdb slim-vdb fork 缺 `VecXFGrid` / `VecXf`**：只有 `VecXIGrid` / `VecXi`（int 版），slim-vdb 的 `std::conditional_t<CLOSED, VecXIGrid, VecXFGrid>` 类型查找失败。手动在 4 个 header 补 float 变体定义，覆盖装回 `/usr/local/include/`（避免 touch 源头触发全量重编）。
2. **Docker build Open3D 0.18 依赖 gitlab.com eigen archive 403 GFW 打断**：从 GitHub eigen-mirror 下对应 commit tarball（2.7MB），Dockerfile 里 `COPY` 进 `/opt/deps/`，`sed` 改 `3rdparty/eigen/eigen.cmake` 的 URL 到 `file://`、删 `URL_HASH`（我们的 tarball SHA 跟 gitlab 版对不上）。
3. **容器里 `/home/steve` 是 root:root 755**（只 bind-mount 了 workspace，不是整个 home），ccache 写 `~/.ccache` 权限被拒。`export CCACHE_DIR=${WS}/.ccache` 搞定。
4. **libtorch 硬编码 hint 路径**（`/home/anjashep-frog-lab/libtorch`）；KITTI pipeline 其实不用 torch（只 scenenet/realworld 的实时分割用）。注释掉 `find_dependencies.cmake` 里的 libtorch include + 让 `datasets` CMake 只编 `KITTIOdometry.cpp`。
5. **OpenCV `cv::imshow` 在 `nanovdb.cuh:244,273,312`** 导致 headless 运行 GTK init 失败。sed 全注释，重编 slim-vdb core。
6. **KITTI pipeline 期望目录名 `velodyne_predictions_txt/`**（不是 `labels/`），还要 `intrinsics.txt`（`fx: / fy: / cx: / cy:` 格式）。
7. **`poses.txt` 的坐标系**：pipeline 应用 `final = T_velo_cam * poses[i] * T_cam_velo` 共轭变换，对非恒等 `Tr`（~26° 旋转）会把 pose 扭曲 —— 最早 Z 方向漂 17m。修：**`calib.txt` 的 `Tr` 写成 identity**，`poses.txt` 直接写 `T_world_velo = T_world_imu * T_imu_velo`（预先把 body→velo 的静态变换吃掉）。修后 Z 跨度降到 5m（水平地面 slab 清晰）。
8. **generate_labels.py 把 4183 帧图像全缓存内存**（16GB）OOM。改成流式：按时间戳顺序推进 LiDAR 和 camera 迭代器，同步配对。内存降到 <1GB，吞吐 22 FPS。
9. **`prune_interval: -1`（默认）+ voxel_size 0.1m 全量跑 4143 帧卡 1.5h 无产出**：Render per-frame 随 map 体积线性增长，无修剪就失控。改 `voxel_size 0.15` + `prune_interval 10` + stride 3 后 25 min 跑完 1381 帧。
10. **Cityscapes 19 类对地下停车场不对口**：道路/建筑/交通牌类主导（class 0/2/7），没有"柱子/停车线/天花板"语义。几何正确，语义"噪声性"正确。要解决需换 ADE20K 150 类室内模型或上 open-set CLIP。

**产物**：
- `outputs/slimvdb_mesh/kitti_01_-1_scans.vdb`（17MB, TSDF）
- `outputs/slimvdb_mesh/kitti_01_-1_semantics.vdb`（723MB, 逐体素 19 类概率）
- `outputs/slimvdb_mesh/kitti_01_-1_scans_pointcloud.ply`（86MB, 296 万带色点）
- `outputs/slimvdb_mesh/kitti_01_-1_scans_{0..18}.ply`（per-class mesh，720MB 合计）
- `outputs/slimvdb_mesh/preview_full.png`（三视图预览）
- 数据 pipeline：`src/slim-vdb/scripts_geoscan/{extract_geoscan.py,generate_labels.py,build_in_container.sh}`
- 调参 config：`src/slim-vdb/examples/cpp/config/kitti_geoscan.yaml`
- Dockerfile patch：`src/slim-vdb/docker/builder/{Dockerfile,deps/eigen-*.tar.gz}`

**下一步**：
- ~~换 ADE20K / open-set CLIP 跑室内合适的语义~~（2026-04-20 完成，见下）
- ~~把 Render step 从主循环解耦（只在 terminate 调一次），恢复 13+ FPS~~（2026-04-20 完成，见下）
- ~~跟 AirSLAM stereo mesh / limap 线地图横向比~~（2026-04-20 完成，见下）

### 2026-04-20 — slim-vdb 收尾三件套 🟢

#### (1) Render decouple（恢复 13+ FPS）

**改动**：`examples/cpp/utils/Config.h` 加 `SLIMVDBConfig::render_every_frame_`（default **false**）；
`examples/cpp/kitti_pipeline.cpp` 主循环里 gate 住 `tsdf_volume.Render(...)`，把 "最终一帧预览 render" 移到所有 VDB/mesh/pointcloud 磁盘写入**之后**，这样即使 render 崩溃也不会丢输出。`config/kitti_geoscan.yaml` 加 `render_every_frame: False`。

**效果**：200 frames、NCLASSES=19 smoke test（`n_scans=200`）：
- 之前（render 每帧）：17 FPS → 0.86 FPS（map 涨大后）
- 现在（render 关掉）：**~150 FPS 稳定**（基本就是 Integrate 本身的时间），200 帧 6.3s 跑完
- Render 列时间：4e-7 ~ 1e-6 s（分支被跳过的零开销）

#### (2) ADE20K 150-class 语义

**Python 侧（`scripts_geoscan/generate_labels.py`）**：加 `--label-space auto|cityscapes|ade20k` 和 `--max_frames N`。auto 按 `model` 名字嗅（含 "ade" 即 ade20k）。`ade20k_palette()` 里 seed=42 随机色板 + 几个 anchor（wall=0 floor=3 ceiling=5 pillar=42 light=83 pole=94）盯住可读性。

**C++ 侧**：`SLIMVDB_NCLASSES` 从 19 → **150**，整库（cuda_lib + slimvdb + pybind）重编 + `cmake --install` 覆盖 `/usr/local/{include,lib}/`，再重编 `examples/cpp/kitti_pipeline`。`ccache` 缓存放 `${WS}/.ccache`，先 `chown -R 1000:1000` 避免 root-owned 旧 cache 阻塞。

**CUDA 701 修复**：NCLASSES=150 + render block size 512 → kernel register budget 爆，launch 时 `cudaErrorLaunchOutOfResources`。`src/slimvdb/slimvdb/nanovdb_utils/common.h:79` 把 `computeForEach` block size 改成 **128**，rebuild cuda_lib + reinstall 即可。

**ADE20K 200 帧 smoke test**：
- 集成速率：**50 – 100 FPS**（NCLASSES 从 19 → 150 带来 3–5× 语义向量 IO，但仍远超原 0.86 FPS）
- 类别分布符合预期：wall（class 0）4.2MB ply 最大，floor（3）2.7MB，ceiling（5）2.5MB，railing（38）也有出货
- 总耗时 9.7s 含最终 render + 磁盘写

**配置**：`examples/cpp/config/kitti_geoscan_ade20k.yaml`。跑法：
```bash
# 1. ADE20K labels（任意子集，--max_frames N）
src/DPVO/.venv/bin/python src/slim-vdb/scripts_geoscan/generate_labels.py \
    --seq_dir outputs/slimvdb_geoscan/sequences/00 \
    --bag ~/Documents/Datasets/geoscan/B1/2026-02-12-16-47-48 \
    --model nvidia/segformer-b0-finetuned-ade-512-512 \
    --max_frames 200 --save_viz

# 2. kitti_pipeline（NCLASSES=150 构建）
docker exec slimvdb_dev bash -lc '
  cd /home/steve/vslam_ws/src/slim-vdb/examples/cpp &&
  ./kitti_pipeline --config config/kitti_geoscan_ade20k.yaml --sequence 01 \
    --n_scans 200 /home/steve/vslam_ws/outputs/slimvdb_geoscan \
    /home/steve/vslam_ws/outputs/slimvdb_mesh_ade20k'
```

#### (3) 横向比 slim-vdb / limap / AirSLAM

**脚本**：`scripts_geoscan/compare_maps.py`。读 `.ply`（stream 子采 150k 点）+ `.obj`（limap 线段）+ 两条 TUM trajectory，对 AirSLAM 轨迹对 finder 做 Umeyama 配准后三联图输出到 `outputs/map_comparison.png`。

**坐标系**：slim-vdb 和 limap 都用 finder_localization.txt 的 body-frame 姿态做三角化/体素融合，原生在 finder world frame。AirSLAM 自己初始化，需要 Umeyama 对齐。

**注意**：AirSLAM trajectory 用 `src/AirSLAM/debug/zdrift_run3/trajectory_v0.txt`，top-level `debug/trajectory_v0.txt` 是 IMU g-unit 修复前的数据（Z 漂到 −492m，无法对齐）。

**指标**（post-fix AirSLAM vs finder，342 对共同时间戳）：
- Umeyama scale s=1.10（AirSLAM 轨迹长度相对 finder +10%）
- APE: **rmse 2.12 m, max 4.36 m, mean 1.84 m**
- bbox: slim-vdb 80×73×16m, limap 70×64×12m（limap 只覆盖 finder 走过的核心区）

**跑法**：
```bash
src/DPVO/.venv/bin/python src/slim-vdb/scripts_geoscan/compare_maps.py
# → outputs/map_comparison.png
```

#### (4) 延伸：全长 ADE20K + AirSLAM 稠密地图导出

**全长 ADE20K rerun**：相同 stride=3 config，n_scans=-1，1381 frames。
- 集成速率 **66 – 98 FPS**（render decouple 之后）
- Integrate time 稳在 10–19 ms（prune_interval=10 控 map 体积）
- 写磁盘：VDB 60ms、semantic VDB 3.4s（NCLASSES=150 下逐体素语义向量更厚）、mesh 38.5s、pointcloud 2.7s
- 产物：`outputs/slimvdb_mesh_ade20k_full/kitti_01_-1_scans_pointcloud.ply`（ADE20K 语义着色稠密点云）
- 跑中有一次 `CUDA error 2: out of memory` 但非致命，集成继续；最终 Render 先做磁盘写再调用，OOM 也不吃输出

**AirSLAM 稀疏图导出**：新增 `src/AirSLAM/demo/map_to_ply.cpp`（link `${PROJECT_NAME}_lib` + rclcpp 但不 init），用 boost `binary_iarchive` 反序列化 `AirSLAM_mapv0.bin`，dump 出：
- `<prefix>.ply`       — mappoints（valid，world-frame，白色）
- `<prefix>_lines.obj` — maplines（EndpointsValid 的）
- `<prefix>_kftraj.txt` — keyframe 轨迹 TUM 格式

zdrift_run3 导出后：47581 valid points、610 valid lines、535 keyframes。文件大小 1.8MB。

**跑法**（容器内，因为依赖 TensorRT）：
```bash
docker run --rm --gpus all --user "$(id -u):$(id -g)" -e USER=steve -e HOME=/tmp \
  -v /home/steve/vslam_ws:/home/steve/vslam_ws -w /home/steve/vslam_ws \
  airslam:latest bash -lc '
    source /opt/ros/humble/setup.bash && source install/setup.bash
    install/air_slam/lib/air_slam/map_to_ply \
      src/AirSLAM/debug/zdrift_run3/AirSLAM_mapv0.bin \
      outputs/airslam_exports/zdrift_run3'
```

**3-way 稠密比**（`compare_maps.py` 扩到 2×2 panel）：
- slim-vdb 稠密点云（ADE20K 语义色）
- limap 结构线
- AirSLAM 稀疏三角化点（蓝）+ map lines（绿），经 Umeyama 对齐到 finder 后 **47279/47581 pts (99.4%) 落在 slim-vdb bbox 内**；610/610 segments 全落在 bbox 内
- 三轨迹 overlay

AirSLAM 有少量外点落在 slim-vdb bbox 外（远距离错误三角化）——画图时 clip 到 slim-vdb bbox ±5m。最终产物：`outputs/map_comparison_ade20k.png`。

**跑法**：
```bash
src/DPVO/.venv/bin/python src/slim-vdb/scripts_geoscan/compare_maps.py \
    --slimvdb outputs/slimvdb_mesh_ade20k_full/kitti_01_-1_scans_pointcloud.ply \
    --out outputs/map_comparison_ade20k.png
```

---

### （模板：每次运行填一条）
**日期**：YYYY-MM-DD
**系统**：`<repo_name>`（版本 / commit）
**数据集**：GeoScan B1 mono-inertial
**配置文件**：路径
**启动命令**：
```bash
...
```
**运行情况**：🟢 / 🟡 / 🔴
**指标**：
- ATE（vs KeyFrameTrajectory.txt）：X.XX m（用 `evo_ape tum ref.txt est.txt -va --plot`）
- 平均 CPU：XX%（4 核）
- 单帧延迟：XX ms
- 显存（若 GPU）：XX GB
**踩坑**：
- ...
**下一步**：
- ...

---

## 通用工具链

### evo 装 & 常用命令
```bash
pip install evo --upgrade --no-binary evo

# ATE（绝对轨迹误差）
evo_ape tum ref_traj.txt est_traj.txt -va --plot --plot_mode xy --save_plot ape.pdf

# RPE（相对位姿误差）
evo_rpe tum ref_traj.txt est_traj.txt -va --delta 1 --delta_unit f --plot

# 批量
evo_traj tum est_*.txt --ref ref_traj.txt -p --plot_mode xy
```

### 录 rosbag（若需重跑）
```bash
ros2 bag record /cam0/image_raw /imu0 -o my_run
```

### 时间戳对齐
若轨迹时间戳单位不一致（纳秒 vs 秒）——用 `evo_traj tum --t_offset` 或 Python 脚本先归一化。

### CPU / 延迟采样
```bash
# 单节点 CPU
ps -p $(pgrep -f vins_node) -o %cpu,%mem --no-headers
# 或跑 htop 手动截图
```

---

## 横向对比结论（跑完后填）

（先空着，跑到 3+ 系统后再补此节）

---

## 待办清单

- [ ] 跑 `open_vins` + `sqrtVINS` 的 GeoScan mono-inertial（共用 `ov_core`，一份 config）
- [ ] 跑 `VINS-Fusion-ROS2` 的 GeoScan mono（单目，光流前端）
- [ ] 跑 `cuvslam_ros` 的 GeoScan stereo-inertial
- [x] 跑 `AirSLAM` 的 GeoScan stereo-inertial（2026-04-16 完成，见上）
- [x] 跑 `DPVO` 的 GeoScan B1 mono（2026-04-17 完成，APE 0.60m/1000f，纯单目无尺度）
- [x] 跑 `slim-vdb` 的 GeoScan B1（2026-04-18 完成，LiDAR+cam+VIO 融合，82×75×16m 稠密语义体素地图）
- [ ] 跑 `ORB_SLAM3` 的 EuRoC V101 做横向参照（有真实 GT）
- [ ] 写一个 `scripts/` 下的 `run_benchmark.sh` 批量脚本（待几个系统都跑通后再做）

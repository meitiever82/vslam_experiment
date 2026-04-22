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
| open_vins | ⬜ | 🟢 2026-04-16 stereo-IMU APE 0.76m vs finder (SE3),轨迹 `~/Documents/Datasets/geoscan/B1/2026-02-12-16-47-48/open-vins.txt` | ⬜ | — |
| sqrtVINS | ⬜ | 🔴 2026-04-22 stereo+IMU 发散:3 套 config(tuned ZUPT / euroc defaults / calib-refine off)都线性飞到 km 级。18633 次 `Negative depth detected` — 立体三角化全被拒,filter 退化成纯 IMU 积分。open_vins 同标定 0.76m ok,是 SR-VINS 后端的某个约束在这个 10fps 鱼眼 + FB100 高噪声 IMU 组合下收敛不住,非 config 问题。构件齐了待续调 | ⬜ | — |
| mins | ⬜ (ROS1) | ⬜ (ROS1) | ⬜ | — |
| EPLF-VINS | ⬜ (ROS1) | — | ⬜ | — |
| VINGS-Mono 🟦 | 🔴 2026-04-22 Spark, Docker + submodules 完整编(NGC 25.09 base, pip torch cu128, GTSAM vio-branch + -fpermissive, torch-scatter + CUDA 12.8 vs 13.0 mismatch patch, open3d stub). 837 帧 stride=5 pinhole, 跑完 16min 无崩, 但 DROID frontend.new_frame_added 从未触发 → mapper 0 次运行 → 无 ply/traj/c2w 输出. 和 DROID-W 一样撞 mono + wide FOV fail mode (fx≈169). image: `vings-mono-spark:with-submodules` 25.9GB | — | ⬜ 🟦 | — |
| ORB_SLAM3_ROS2 | 🟢（参考基线） | 🟢 | ⬜ | ⬜ |
| VINS-Fusion-ROS2 | — | 🔴 2026-04-21 stereo+IMU(两次调参都死): baseline APE 17.5m/31s; 调过 min_dist 8/max_cnt 500/F_thresh 3/flow_back 0/rate 0.5/estimate_extrinsic 0 后反而 31.5m/20s。LK 光流前端对 10fps 鱼眼+低纹理车库根本不够用,常态 1-10 特征,不是参数能救的 | ⬜ | — |
| AirSLAM | ⬜ | 🟢 2026-04-16（Z 漂修复后 Z∈[0,1.1]m, 535 KF）| 🟢 2026-04-22 V101 stereo-inertial **APE SE3 0.087m / Sim3 0.085m** (274 KF / 2912 frames @ 41.9 FPS; `use_superpoint: 0` 绕开 TRT 10 SuperPoint ONNX int64/int32 bug, 用 PLNet 原生点+线检测) | — |
| droid_w_ros2 / DROID-W(VO only, mapper off) | 🔴 2026-04-21 Spark, native uncertainty on + FiT3D + dpt2_vitl_hypersim_20, undistort balance=0.0(fx≈240). **z 漂 160m**(GT z≈0 平面), Umeyama 秩退化. APE Sim3 6.25m / SE3 40.58m 只是数字, 实际轨迹形态完全错(dy=11.7 vs GT 64, z 单调飞). mono + 学习深度在走廊/车库 fail mode. 试 vkitti depth 更差(Sim3 9.06m) | — | ⬜ | ⬜ |
| droid_w_ros2 / DROID-W(+ 3DGS mapper) 🟦 | 🔴 2026-04-21 VO 轨迹已飞, mapper 无意义. 另外 aarch64 open3d 无 wheel 要源码编, 本次不 pursue | — | ⬜ 🟦 | ⬜ 🟦 |
| cuvslam_ros | ⬜ | 🟡 stereo-only 0.56m (122s 段, 2026-04-16 首选). stereo-inertial 2026-04-22 试 2 次: generic noise APE SE3 9.7m (z 飘 18m), Kalibr FB100 noise APE SE3 3.36m / Sim3 3.26m (z 飘 5m, 2994 poses) — IMU 都没打过 stereo-only, 锁 stereo-only | 🟢 2026-04-22 V101 **stereo-inertial** (odometry_mode=inertial, T_C0_IMU 修正): **APE SE3 0.237m / Sim3 0.228m** (vs stereo-only 0.467m, IMU 带 2×). 2876 poses, 144s, plumb_bob | ⬜ |
| DPVO | 🟢 2026-04-17 mono-only, APE 0.60m (1000f) | — | 🟢 2026-04-20 V1_01 APE 0.046m (1435 pairs, stride=2) | — |
| MapAnything（前馈 multi-view）🟦 | 🟢 2026-04-21 Spark, 100 dense views (stride=5 前 50s): **APE Sim3 0.106m / max 0.303m**(shape 极好, 比 DPVO 60× 好), SE3 3.44m(metric_scale 4.97× 偏大, mono 固有). GLB 700MB. mb=4 memeff=True 165s 16GB VRAM. 验证 feed-forward 需要密集近邻视图(s130 稀疏失败的反面) | — | 🟢 2026-04-22 V1_01 100 dense frames cam0: **APE Sim3 0.84mm / SE3 6.6mm** (窄 FOV 室内 + Vicon GT, 前馈 SOTA, 比 DPVO 0.046m 好 7×). 128.8s infer 15GB VRAM. scale 3.35× 仍偏大但极小绝对误差 | — |
| VGGT-SLAM 🟦 | 🔴 2026-04-21 mono right-cam 392px submap=4: SL(4) 后端在 submap 36 奇异崩掉,前 35 子图 175 poses APE 16.0m / max 30.4m (Sim3). 再加 SL(4) 退化和 8GB VRAM 双重上限,**Spark 上重跑 518px + submap=16** 🟦 | — | ⬜ 🟦 | — |
| Gaussian-LIC 🟦 | — | ⬜ 🟦 即使 Spark 也要先 fork 适配 CUDA 13 / TensorRT 10 / ROS1→2 port,见 `memory/project_gaussian_lic_blockers.md` | — | — |
| GS_ICP_SLAM 🟦 | — | — | — | ⬜ 🟦 |
| SGS-SLAM 🟦 | — | — | — | ⬜ 🟦 |
| Mobile-GS 🟦 | — | ⬜ 🟦 离线 3DGS,走 ORB_SLAM3/AirSLAM KFs 导 COLMAP | — | ⬜ 🟦 |
| rgbdslam_v2 | — | — | — | ⬜ |
| rtabmap | 在 `~/rtabmap_ws/`，独立记录 | — | — | — |
| limap（3D 线后处理，非 SLAM） | — | 🟢 2026-04-16 finder VIO + BA hybrid 出 280 条 nv≥4（14× 纯 VIO，2.4× 纯 SfM） | ⬜ | — |
| slim-vdb（LiDAR+cam+VIO 语义体素） | — | 🟢 2026-04-18 1381 帧 stride3, 296万点云, 82×75×16m, Cityscapes-19 语义 | ⬜ | — |
| gsplat offline 3DGS（cam0 + finder pose） | 🟢 2026-04-22 300 帧 / L1+0.2·SSIM / 30k iter / 165k 高斯 / 348s / VRAM 1.25GB / **PSNR 25.43dB, SSIM 0.85**(真实 MSE,v4 的 L1-近似 PSNR~33 不可比,同 PLY 重算真实 24.72/0.84)。SfM-init 留到下次 | — | — | — |
| GLIM LIO + dpvo_frontend(Scheme C 紧耦合) | 🟢 2026-04-21 早:用 Python bag_replay_monotone.py 绕开 ros2 bag play 的时序 bug,**baseline 1.88m vs +DPVO 0.73m(SE3),RMSE 提升 2.6×**(不同段对照,但趋势明确) | — | — | — |
| GLIM LIO + cam0 RGB 彩色点云(在线 Option A) | 🟢 2026-04-21 `colored_cloud_node.py` 实时投影 LiDAR→fisheye cam0 发 `/colored_cloud`,`colored_cloud_to_ply.py` 累积落 PLY。113s bag 段 1.36M 点 / 20MB,`runs/geoscan_colored/geoscan_b1_glim_colored.ply` | — | — | — |

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

### 2026-04-22 下午 — sqrtVINS (ov_srvins) stereo+IMU on GeoScan B1 🔴

**目标**:拿 sqrtVINS 的平方根 KF 后端和 open_vins 的 MSCKF 对照(同一家 Delaware RPNG,共用 `ov_core`),看 SR-VINS 数值稳定性是否在这个 10fps 鱼眼 + FB100 高噪声 IMU 场景下给出优于 open_vins 0.76m 的结果。

**构件**(都已落地保留):
- `src/sqrtVINS/config/geoscan/{estimator_config.yaml, kalibr_imucam_chain.yaml, kalibr_imu_chain.yaml}` — 标定用 `T_imu_cam = inv(Kalibr T_cam_imu)` + FB100 噪声 ×3 + 鱼眼 `equidistant` 模型
- `logs/sqrtvins/scale_imu.py` — `/handsfree/imu`(g-unit)→ `/imu_scaled`(m/s²) ×9.81 republish(openvins 系无 `imu_acc_scale` 开关,只能 ROS 层转)
- `logs/sqrtvins/dump_pose.py` — 订 `/poseimu` 落 TUM,timestamp 用 int-sec + 9 位 ns 避免 float64 精度损
- `logs/sqrtvins/run_geoscan.sh` — 驱动:scaler + dumper + `ros2 run ov_srvins run_subscribe_msckf` + `ros2 bag play`

**构建踩坑**:
1. **`ov_core` 重名** — sqrtVINS 和 open_vins 各自带一份 `ov_core`,colcon 不允许重名包。`src/open_vins/COLCON_IGNORE` + `src/map-anything.git/COLCON_IGNORE`(后者 `setup.py` 让 colcon identification 炸 `SyntaxError`)才能正常 build
2. **Humble 下 `image_transport` segfault** — `ROS2Visualizer` 里 `ImageTransport` 是 ctor 栈局部,ctor 退出后被销毁,但后台 `std::thread` 在每 50ms 跑 `publish_images()` 访问 `it_pub_tracks`,这时已是悬垂指针。**关 `multi_threading_pubs: false`** 绕开(我们不需要 track 可视化图片)
3. **必填参数 `record_timing_filepath`** — 配缺就 "unable to parse all parameters" 但不提示哪个,得翻 log 定位
4. **topic 命名空间** — `ros2 run ov_srvins run_subscribe_msckf` 绕过 `subscribe.launch.py`,发布 topic 没 `ov_srvins/` 前缀,订 `/poseimu` 不是 `/ov_srvins/poseimu`
5. **stdout 行缓冲** — C++ 进程重定向到文件时是块缓冲,`kill` 时丢 buffer。用 `stdbuf -oL -eL` 强制行缓冲才能看到日志

**失败模式**:所有 3 套配置全都线性发散到 km 级,**不是 OOM / init failed / segfault,是 filter 数值爆掉**:

| 配置 | 关键参数 | 最终 dist (m) | pose 数 |
|---|---|---|---|
| v1 ZUPT + 低阈值 | `try_zupt: true, init_imu_thresh: 0.5, zupt_only_at_beginning: true` | 300,000+ | 134 |
| v2 euroc defaults | `try_zupt: false, init_imu_thresh: 2.0, calib_cam_extrinsics: true` | 14,000+ | 200+ |
| v3 defaults + calib fixed | v2 基础上 `calib_cam_extrinsics: false, calib_cam_intrinsics: false` | 77,000+ | 1366 |

**根因诊断**(srvins.log 里 18633 次 `Negative depth detected`):立体三角化在初始化 1-2 秒后就全军覆没,filter 丢失视觉约束,**退化成纯 IMU dead-reckoning**,FB100 bias 一步步累积 → Z 方向以 10-20m/step 匀速漂,q 也跟着乱转。原因猜测:
1. SR-VINS 后端对初始化几何精度更敏感,在 10fps + 近针孔鱼眼 + 低基线 (9.7cm) 的组合下容易进入奇异 Jacobian 状态
2. FB100 在静止段 `|acc|≈1.07g`(理想 1.00g)→ 7% 偏差,最终 `ba_z=0.84 m/s²`,bg 也飘到 2.3°/s/轴 — filter 估得到但补偿不及时就炸了
3. stereo 时间戳同步可能比 VINS-Fusion 更严格,10fps 下左右目 header 差值常超过 sqrtVINS 内部 sync tolerance,导致图像被丢

**open_vins(MSCKF,同 bag + 同标定) 0.76m 收敛**,说明calib 和 IMU 都没问题,**是 SR-VINS 这个后端的某个收敛约束在当前传感器组合下不合**。

**结论**:**不花更多本机时间调这个**。open_vins 已经作为 stereo-IMU 里 "ov_core 系" 的代表给了 0.76m,sqrtVINS 在本 bag 目前状态 🔴,构件(build + config + script)齐全落地留待:
- 换数据(EuRoC V1 / Mars rover 这类"干净"场景)看是否一切如常 — 会区分"sqrtVINS 本身实现问题"vs"GeoScan 特别不合"
- 或在 Spark 上启 `calib_imu_intrinsics: true` + `init_dyn_use: true` 深调(更多 CPU 空间)

### 2026-04-22 — gsplat offline 3DGS 收尾(L1+SSIM + 30k iter)🟢

**目标**:把 2026-04-20 🟡 基线(L1-only, 15k iter, 报"PSNR~33")推到 paper 默认配置(L1+SSIM, 30k iter),出最终 benchmark 数字并收尾。

**改动**(`src/vslam_experiment/scripts/gsplat/train_3dgs_densify.py`):
1. 原 `ssim_loss()` 是 L1 占位(注释里明写"避免 kernel rewrite,用 L1 顶着")。换成真 11×11 Gaussian 窗口 SSIM(纯 PyTorch `conv2d`,~20 LOC,核缓存)。`fused-ssim` 没 PyPI wheel,git 源码编译太慢,**纯 PyTorch 在 30k iter / 350s 总耗时里完全够用**,不值得折腾 CUDA 扩展。
2. Loss 改为 `(1-λ)·L1 + λ·(1-SSIM)`,`λ=0.2`(paper 默认),加 `--ssim_lambda` 参数。
3. `--iters 30000`。
4. **关键修复**:原 per-iter 日志的 PSNR 是 `-20*log10(ema_L1)` — 这不是真 PSNR(PSNR 要 MSE 不是 L1)。约高估 ~1.9 dB,再加上是单随机 view 的 ema,之前矩阵里"PSNR~33"是错的。改成真 MSE-based PSNR。
5. 训练结束后加一遍全 300 views 的 eval pass,输出 mean PSNR + mean SSIM 作为 benchmark 数字。
6. 新增 `eval_3dgs_ply.py` 独立 eval 脚本,可以加载 `.ply` 重算真 PSNR/SSIM(用来回测 v4 PLY 的真实表现)。

**命令**:
```bash
python3 src/vslam_experiment/scripts/gsplat/train_3dgs_densify.py \
  --data runs/geoscan_3dgs \
  --out runs/geoscan_3dgs/densify_ssim_30k \
  --iters 30000 --ssim_lambda 0.2
```

**结果(真实 MSE-PSNR + 窗口 SSIM,300 views 平均)**:

| 指标 | v4(L1 only, 15k iter)| **v5(L1+0.2·SSIM, 30k iter)** | Δ |
|---|---|---|---|
| PSNR | 24.72 dB | **25.43 dB** | +0.71 |
| SSIM | 0.8400 | **0.8499** | +0.0099 |
| 高斯数 | 115,974 | **165,539** | +49,565 |
| 训练时间 | 73 s | 348 s | +275s |
| 峰值 VRAM | 1.18 GB | 1.25 GB | +0.07 |
| PLY 大小 | 19 MB | 27 MB | +8 |

**老数字怎么错的**:v4 训练日志里的 `psnr~33` 来自 `-20*log10(L1_ema)`,这是 L1 近似,不是真 PSNR。用 v4 的 `point_cloud.ply` 重跑真 eval pass:**真实 PSNR 只有 24.72**,SSIM 0.84。所以 v5 提升不是"33→25",而是"**24.72→25.43**",小但真实。

**为什么提升有限(~0.7 dB)**:两个 lever 在这次都没动:
- **随机 bbox init**:165k 高斯还是从随机点(+mean_color)开始。Paper 初始化用 COLMAP / SfM 稀疏点云,起点就贴着几何。想继续涨 PSNR,必须把 AirSLAM/ORB_SLAM3 的 map-points 导出作初始 gaussians — 这是下次主要工作。
- **图像本身**:GeoScan cam0 是鱼眼 → pinhole undistort 之后边缘有扭曲残差,加上车库暗 + 低纹理,PSNR 天花板本身就不高。

**产物**:
- `~/vslam_ws/runs/geoscan_3dgs/densify_ssim_30k/point_cloud.ply`(27 MB)
- `train_log.txt` / `console.log` / `render_val_*.jpg`(0→30k 每 2k 一张)
- v4 的 `densify_output/point_cloud.ply` 留着作对照(19 MB)

**SfM-init 路线(下次做)**:
1. AirSLAM stereo-inertial 产物 `AirSLAM_mapv0.bin` 已有 `map_to_ply.cpp` 可导出 → 3D point + RGB(从 KF 反投影)。
2. 位姿系对齐:AirSLAM 轨迹 vs finder 轨迹,用前 10s 静止段对齐到同一 world。
3. 把导出的 ~10k-50k map points 当 gaussians 初始位置,color = 反投影 RGB,scale 基于 KNN 距离(paper 做法)。
4. 重训 30k iter L1+SSIM,目标 PSNR ≥ 27 dB。

### 2026-04-21 下午 — VINS-Fusion-ROS2 stereo+IMU on GeoScan B1 🔴

**目标**:把现成的 VINS-Fusion-ROS2(`install/vins`,`install/loop_fusion`,`install/camera_models` 早就 build 过,源码树现在有 COLCON_IGNORE)跑一把 GeoScan B1 stereo-inertial,看 VINS 的 Ceres 滑窗能不能在 10fps 鱼眼上撑住。

**配置**:`config/geoscan/geoscan_stereo_imu_config.yaml`(user 已经调过 `max_cnt 300 / min_dist 15 / equalize 1` 给低纹理场景)。**关键**:config 里有 `imu_acc_scale: 9.81`,VINS 源码`vins/src/rosNodeTest.cpp:154-156` 和 `parameters.cpp:117` 会读并乘进 `imu_msg->linear_acceleration`——所以 GeoScan 的 IMU g-unit 已经被 handle 了,**不是 IMU 量级问题**。

**Launch**:没走 `geoscan_stereo_imu.launch.py`(其默认开 rviz 而 `vins_rviz_config.rviz` 当前有 git 合并冲突未解),直接 `ros2 run vins vins_node <cfg>` + `ros2 run loop_fusion loop_fusion_node <cfg>` 并行。驱动脚本 `logs/vins_fusion/run_geoscan.sh`。

**bag 播放**:`ros2 bag play --clock --start-offset 10`(跳开头静止段)。

**结果**:
| 指标 | 值 |
|---|---|
| 运行时长 | bag 全长 418s 播完 |
| VINS 输出 poses | **145**(vio.csv) |
| 有效覆盖 | ts 1770885781 → 812,**只有 31s**,之后 **tracking lost** |
| 最终 n_pts | 2(健康值 > 100) |
| APE vs finder(SE3 aligned) | **rmse 17.53m / mean 15.56m / max 38.19m** |
| loop_fusion 输出 | 空(vio_loop.csv 0 行) |

**跟丢轨迹**:init 后前 30s 缓慢爬升到 (-57, 20, 20) — x 合理但 **Z 飘到 20m**(车库是平的,应 0-3m)。随后 n_pts 崩到 2,无法 recover,bag 后面 ~370s 全没输出。

**根因诊断**:
1. **10fps + 鱼眼 + 暗车库**对 VINS 自带的 LK 光流前端太辛苦。日志里 `n_pts size: 5/4/8/11` 是常态,不够滑窗 BA 约束。对比同场景下 open_vins 0.76m / ORB_SLAM3 稳定:后者前端是 ORB 特征 + 描述子匹配,对帧间大 baseline 更鲁棒;VINS 的光流假设小位移,10fps 下直接断。
2. **stereo 时序**:日志 `throw img0/throw img1` 反复出现——左右目 header 时间差超过 VINS 内部 sync tolerance,扔图拖累 BA。
3. **`show_track: 1`**:启用了 OpenCV imshow 窗口,headless 运行时无害但略拖。

**未做的调参**(之后回头搞或 Spark 上):
- `freq: 10` 改 `freq: 20` 让 subpixel 平滑不受 10fps 限制
- `F_threshold` 从 1.5 调到 3.0 放松 RANSAC
- 换 `flow_back: 0` 关反向光流(走廊均质时反而贡献异常匹配)
- 换 mono 模式(`num_of_cam: 1`)+ 只用 right_camera(跟 AirSLAM 已经验证能跑的一致)

**结论**:**这个 bag 对 VINS-Fusion 不友好**,不是 bug 是前端假设不合。benchmark 结果 🔴 记录以示该系统在 GeoScan 类数据上不推荐,优先跑 open_vins / AirSLAM / ORB_SLAM3。

**调参再尝试**(2026-04-21 下午稍晚):按照"试图让它跑完整 bag"的目标,又做了 3 轮:

1. **mono + flow_back=0 + F_thresh 3 + min_dist 8 + max_cnt 500 + estimate_extrinsic 0 + bag rate 0.5 + 跳 10s 静止**:**卡在 IMU excitation not enough / Not enough parallax 死循环**,init 从不完成。mono-inertial init 在 10fps 静止+慢速车场景下不够 observability。
2. **stereo + 同上调参 + estimate_extrinsic 0 + rate 0.5 + 跳 10s**:init 过了,但 **1.7s 后 Z 速度 -0.49m/s 下凿,到 ts 871 pose 爆到 1e176**。关掉 `estimate_extrinsic` 让 IMU bias 错误无法 in-run 补偿。
3. **stereo + 调参 + estimate_extrinsic 0 + rate 0.5 + 保留 10s 静止**(希望静止段给 VINS 做 gyro bias 校准):init 过,**137 poses / 20s** 跟丢,位置 (-92, 61, 8)m。APE **31.5m**,比 baseline 17.5m **更差**。

**用户反馈**(记 memory):"不要估计外参,这个功能不可控"——`estimate_extrinsic: 1` 在其他 VINS-Fusion 数据上工作但 GeoScan 场景下行为不稳。但关掉后需要初始化完全准确,GeoScan 慢速车无法给够 observability → 死循环。**鸡生蛋**。

**最终定论**:**VINS-Fusion-ROS2 在 GeoScan B1 bag 上不可恢复**,前端和这个数据不兼容,优化方向超出参数调整范围。

**产物**:`logs/vins_fusion/geoscan_B1/` 下 `vio.csv`(最后一次 137 poses)、`vio_tum.txt`、`ape.zip`(31.5m)、`vins_node.log`、`loop_fusion.log`、`bag_play.log`。另有 `config/vins_rviz_config.rviz` 有 **6 处 git 合并冲突标记未解**——和 benchmark 无关但待清理。

---

### 2026-04-21 — VGGT-SLAM feedforward mono on GeoScan B1 🔴

**目标**:把 MIT-SPARK 的 **VGGT-SLAM v2**(`src/VGGT-SLAM/`, MapAnything 前馈 + SL(4) 因子图后端)放到 GeoScan B1 右目鱼眼上,看它能不能在 feedforward 失败之后把长轨迹 SLAM 救回来(跟 MapAnything 24m 那次的对照)。

**Pipeline**:
1. 抽图:`scripts_geoscan/extract_geoscan_images.py` 从 bag `/right_camera/image` 读 equidistant fisheye 4082 帧 → `cv2.fisheye.initUndistortRectifyMap` 手动构新 pinhole K(scale CAM1_K 而不是 `estimateNewCameraMatrixForUndistortRectify`——后者对近针孔 equi 返回 focal≈0 的退化 K,跟 AirSLAM `camera.cc` 修过的是同一个坑)→ 写 pinhole PNG + `timestamps.txt` + `pinhole_K.txt`。`--start_offset_s 10` 跳开头静止段。
2. 跑 SLAM:docker `vggt-slam:latest`(Ubuntu 22.04 + CUDA 12.1 cudnn-devel + Py 3.11 venv + torch 2.3.1 + gtsam-develop + VGGT_SPARK)。
3. 权重:先一股脑下齐,`torch.hub.load_state_dict_from_url` 不听 `HF_ENDPOINT`,用 `huggingface_hub.hf_hub_download()`(HF_ENDPOINT=hf-mirror)把 VGGT-1B `model.pt` 5.0GB 一次拉到 `~/.cache/torch/hub/checkpoints/model.pt`(HF Xet 分块 curl 跟不全,必须走 huggingface_hub)。`dinov2_vitb14_pretrain.pth` 和 `dino_salad.ckpt` 从 fbaipublicfiles / GitHub releases 另外 curl 到同一目录。
4. Post-process:`postprocess_poses.py` 把 VGGT 的 `frame_id tx ty tz qx qy qz qw`(col 0 是图序号不是时间戳)替换成 timestamps.txt 里的真时间戳 → `poses_tum.txt`。
5. Eval:`evo_ape tum -s -va --t_max_diff 0.5`(Sim3,mono 尺度不可观)。

**VRAM 预算斗争**(4060 Laptop 8GB,xorg+gnome-shell 先占 2-3GB):
| 配置 | 撑到 |
|---|---|
| 518px + submap=8 | OOM at submap 0(第一个 submap 9 帧 fit 不下) |
| 518px + submap=4 | 34 submaps 前稳,到 35 OOM |
| 392px + submap=4 | 31 submaps 前稳,到 32 OOM |
| 392px + submap=4 + `PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True` + 每 submap `empty_cache` + `--skip_dense_log` | 35 submaps,**在 submap 36 SL(4) constructor 奇异崩**(`Initial total error` 从 62 → **2447934** → `RuntimeError: SVD singular values product = 0`) |
| 336px + submap=4 | **submap 0 OOM**(图像太小,ViT patch drops 导致某些 conv 的 padding 加分配) |

最终取 392px + submap=4 + 周期 dump poses(main.py 改了,每 submap 完直接 `solver.map.write_poses_to_file` 存档)。

**结果**(175 poses = 35 个 submap 的 keyframes,覆盖 bag 前 ~30%):
| 参考 | APE rmse(m) | mean | median | max | Sim3 scale |
|---|---|---|---|---|---|
| finder_localization(175 paired) | **16.03** | 15.06 | 16.14 | 30.40 | 0.1932 |

scale = 0.19 说明 VGGT 估的单目尺度跟真实米制差 5×(跟 MapAnything 0.96 的 stride=130 比:VGGT 因为分 submap + 内部分别估 K,每 submap 独立假设,尺度漂移更大)。

**对比 MapAnything 那次**:
- MapAnything:APE **24.6m**,33 views 一次前馈过完整 416s,尺度常数但前馈模型对 13s 大间隔假设失败
- VGGT-SLAM:APE **16.0m** 在跑完 30% 就崩,SL(4) 后端数值爆了。**后端没救回来前馈的定位,反而先于前馈自己死掉**

**核心教训**(写进 memory):
1. **GeoScan B1 这种"驾驶走廊+均质墙面+10fps+鱼眼"的场景对 feedforward + SL(4) 双暴击**:VGGT 前馈估 K 本来就漂,submap 间 SL(4) 拟合没有公共点约束(`Not enough overlapping points to estimate scale factor, using a less restrictive mask` 刷屏),15-DoF 单应误差积累,到某个 submap 后因子图 SVD 彻底奇异。
2. **8GB 不够**:即使 392px + submap=4 + 所有节流,也只能到 submap 36。这个系统要在 Spark 24GB+ 上跑 518px + submap=16(默认配置)才能验证其"v2 真实的 SL(4) 优化能力"——这就是 matrix 里为什么打 🟦。
3. **HF Xet 分块下载**:`torch.hub.load_state_dict_from_url` 吃不了 Xet,curl -L 只跟到 redirect 就缺数据,必须用 `huggingface_hub.hf_hub_download()` +  `HF_ENDPOINT=hf-mirror`。这是所有后续 HF 大模型(VGGT-1B / ViT-g14 / 任何 >1GB 的)的通用做法。

**产物**:`logs/vggt_slam/geoscan_B1/full/` 下 `poses.txt`(VGGT 原格式,frame_id col)、`poses_tum.txt`(真时间戳)、`console.log`、`ape.zip`、`timestamps.txt`、`pinhole_K.txt`、`images/*.png`(4082 帧 392×518 pinhole,后续可复用)。对比图 `outputs/map_comparison_vggt.png`(6 panel:slim-vdb ADE20K / limap lines / VGGT dense(skip_dense_log 所以空) / AirSLAM sparse / 三轨迹 overlay / VGGT vs finder)。

**跑法**(Spark 或大 VRAM 机复用):
```bash
# 权重一次性就位
bash src/VGGT-SLAM/scripts_geoscan/setup_benchmark.sh  # 在 vggt-slam:deps 容器里
docker commit vggt_sl_setup2 vggt-slam:latest

# 抽图
src/DPVO/.venv/bin/python src/VGGT-SLAM/scripts_geoscan/extract_geoscan_images.py \
    --bag ~/Documents/Datasets/geoscan/B1/2026-02-12-16-47-48 \
    --out_dir logs/vggt_slam/geoscan_B1/full \
    --start_offset_s 10.0 --short_side 518

# 跑 SLAM(在 Spark 上恢复默认 submap=16,--skip_dense_log 可去掉)
bash src/VGGT-SLAM/scripts_geoscan/run_geoscan.sh all full

# 评估
evo_ape tum ~/Documents/Datasets/geoscan/B1/2026-02-12-16-47-48/finder_localization.txt \
    logs/vggt_slam/geoscan_B1/full/poses_tum.txt -s -va --t_max_diff 0.5
```

---

### 2026-04-16 — cuvslam_ros stereo VO on GeoScan B1 🟢

**目标**：调通 NVIDIA cuVSLAM 15.0 双目模式在 GeoScan B1 鱼眼数据集上的轨迹输出。

**系统**：cuvslam_ros（自研 ROS2 wrapper for cuVSLAM 15.0.0），stereo VO 模式（无 IMU）。

**数据**：`2026-02-12-16-47-48`（418s，10fps 双目鱼眼 1280×1024）。

**发现的两个 bug + 修复**：

1. **stereo extrinsic quaternion 错误**：手写 `rig_from_cam1_qx/qy/qz/qw` 漏掉 cam0→cam1 的 ~0.73° X 轴旋转（qx 差 18×，符号也反了）。ORB_SLAM3 直接吃 rotation matrix 不受影响。**修**：从 ORB_SLAM3 的 `Stereo.T_c1_c2` 矩阵用 Shepperd 公式算出 `(-0.006382, 0.000441, -0.000351, 0.999979)`。
2. **cuVSLAM 内部 fisheye 处理在 ~108° HFOV 退化**：修完四元数后 tracking 不丢了，但轨迹完全跑飞（RMSE 21.7m，zigzag 集中在小区域）。cuVSLAM 文档说 fisheye 模式是 "pinhole + undistort"，对宽视角鱼眼 stereo matching 在边缘误差大。**修**：在 `stereo_callback` 里用 `cv::fisheye::initUndistortRectifyMap(K, D, R=I, P=K)` 预先把左右鱼眼去畸变为 pinhole，cuvslam Rig 改成 `Pinhole` 无畸变参数、保留原始 stereo extrinsics。

**注**：`cv::fisheye::stereoRectify` 在 OpenCV 4.6 上返回退化 P 矩阵（fx≈0.00128），无法使用。改用 per-camera undistort 不做 stereo rectification，cuvslam 自己处理小的 inter-camera 旋转。

**结果**（前 122s 段，1214 poses，Sim3 对齐 vs `finder_localization.txt`）：

| 配置 | APE RMSE | APE max | path length |
|---|---|---|---|
| 原始 fisheye 直喂（quaternion 修完）| 21.7 m | 36.5 m | 839.9 m（GT 386.6m）|
| **+ rectify_fisheye 预 undistort** | **0.56 m** | **1.19 m** | 90.3 m（122s 段）|

**横向对比（同一 bag，全段 finder GT）**：
| 系统 | APE RMSE vs finder | 备注 |
|---|---|---|
| open_vins mono-inertial | 0.76 m | 全段 414s |
| **cuvslam stereo VO (rectify)** | **0.56 m** | 前 122s 段 |
| ORB_SLAM3 stereo-inertial | （参考基线）| — |

**启动命令**：
```bash
ros2 launch cuvslam_ros geoscan_stereo.launch.py \
    rviz:=false trajectory_output_path:=results/cuvslam_rect_geoscan.tum
# 另一终端:
ros2 bag play ~/Documents/Datasets/GeoScan/B1/2026-02-12-16-47-48 --rate 1.0
```

**配置**：`src/cuvslam_ros/config/geoscan_stereo.yaml`（`rectify_fisheye: true`）。

**产物**：
- 轨迹：`results/cuvslam_geoscan.tum`（无 rectify，4130 poses，完整 418s）
- 轨迹：`results/cuvslam_rect_geoscan.tum`（rectify，1214 poses，前 122s）
- 对比图：`results/cuvslam_vs_openvins_xy_trajectories.png`（无 rectify vs GT vs openvins）
- 对比图：`results/cuvslam_rect_xy_trajectories.png`（rectify vs GT）

**下一步**：
- [ ] 跑完整 418s rectify 拿全段 ATE
- [ ] 试 inertial 模式（IMU 之前 "暂时不可用"，现在 stereo 稳了可以重新尝试）
- [ ] 调 SLAM loop closure（全程 `lc_events=0`，没触发过回环）

---

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

### 2026-04-20 — gsplat offline 3DGS on GeoScan B1 cam0(finder pose)🟡

**目标**:用已知 pose(finder_localization.txt)+ cam0 图像做离线 3DGS,验证"传统 SLAM pose + 现代稠密重建"路线 — 替代 MapAnything 前馈 SfM 失败的那条线。

**Pipeline**(脚本在 `src/vslam_experiment/scripts/gsplat/`):
1. `prepare_3dgs_data.py`:从 finder(2615 poses)线性抽 300 帧 → 对每个 pose 时间戳找 bag 里最近的 cam0 图像 → fisheye(Kalibr equidistant)→ pinhole 去畸变 → 写 `images/NNNNNN.jpg` + `poses.json` + `intrinsics.json` + `scene_stats.json`。**关键点**:`finder_localization.txt` 是 **IMU(body)系位姿**,必须右乘 `T_imu_cam0 = inverse(T_cam_imu)`(从 Kalibr yaml 读)才是 cam0 的 cam2world。
2. `train_3dgs_densify.py`:gsplat 1.5.3 `DefaultStrategy` 带动态 split/clone/prune,100k 初始高斯,15000 iter,L1 损失,rand 初始化(bbox = traj_bbox + pad_xy=8m + pad_z=6m,因为 finder 轨迹 Z 只有 0.4m 范围,必须给 ceiling/floor 大 padding)。
3. 输出 `.ply`(3DGS 官方格式,SuperSplat / antimatter15 viewer 打得开)+ val 渲染。

**结果**:
| 指标 | 值 |
|---|---|
| 高斯数(init → final) | 100k → **115,974** |
| 训练时间 | **73 s** |
| 峰值显存 | **1.18 GB** |
| PSNR(L1 近似) | ~33 dB |
| 点云文件 | 19 MB `densify_output/point_cloud.ply` |

定性:车库走廊结构可辨认(顶灯条 / 地面透视 / 两侧柱子 / 车辆轮廓),但细节糊 — 缺 SSIM 损失 + 迭代数偏少(paper 30k)+ 随机初始化(paper 从 SfM 点云)。

**迭代过程**(三次,印证"小 bug 几个叠加搞得看起来模型不工作"):
1. **v1** 50k 固定高斯 + 随机 bbox init(未做 IMU→cam 变换)+ 位姿系不对 → 渲染全黑,PSNR 20,用户"完全不对"。
2. **v2** 加 `T_imu_cam0` 右乘 → 仍全黑,PSNR 20 — pose 虽然对了,但 `traj_bbox Z 范围只 0.4m`,50k 高斯全铺在地面薄层,天花板和墙没覆盖。
3. **v3** pad_xy=8 / pad_z=6 + 200k 高斯 + 平均场景色初始化 → PSNR 26,渲染开始出形状。
4. **v4(当前)**`DefaultStrategy` densify/prune + 15000 iter → PSNR ~33,车库可辨认。

**gsplat 安装**:`pip install --user "gsplat>=1.3.0"`(实际装到 1.5.3),**只装库够用**,examples/requirements.txt 那一堆(pycolmap / viser / fused-ssim / tyro / numpy<2 / nerfview)不装也能训 — 自己写 trainer 用 `gsplat.rasterization` 和 `gsplat.strategy.DefaultStrategy` 即可。注意:
- **从 `src/gsplat/` 下跑会 shadow import 崩**(仓库的 `gsplat/` 目录被当成包,找不到已安装的 CUDA 扩展)。在 `~/vslam_ws/` 或 `/tmp` 根下跑可以。
- Blackwell/Ada 首次运行会 JIT 编译 CUDA kernel(~2 分钟),之后缓存。

**产物**:`~/vslam_ws/runs/geoscan_3dgs/`:
- `images/000000.jpg ~ 000299.jpg`(针孔去畸变 cam0 帧)
- `poses.json` / `intrinsics.json` / `scene_stats.json`
- `output/` — v3 基线训练(50k, 5000 iter, no densify)
- `densify_output/point_cloud.ply`(v4,当前最好结果,19MB)

**Viewer**:`.ply` 本地拖进 https://antimatter15.com/splat/ 或 SuperSplat / Spectacular.app 渲染。rerun 也支持但要 `rerun-sdk >= 0.24` 的 Gaussian viewer 插件。

**待办**(下次继续):
- 装 `fused-ssim`(有 arm/x86 wheel)加真 SSIM 损失
- iters 拉到 30000(paper 默认)
- 用 ORB_SLAM3 / AirSLAM 的 map points 做 SfM init 替换随机 bbox init
- 换 AirSLAM stereo-inertial 的 535 KF pose(是不是比 finder 更利于收敛?)
- 尝试加入 cam1 数据做双目 3DGS(gsplat 能吃任意数量 view)

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

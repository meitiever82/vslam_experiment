# Benchmark 记录

本文件记录各 SLAM 系统在实际数据集上的运行情况、结果、踩坑和对比。
**首要数据集**：GeoScan B1（见 `memory/project_geoscan_benchmark.md`）。

**填表约定**：
- 状态：⬜ 未开始 / 🟡 调试中 / 🟢 跑通 / 🔴 失败（暂时放弃）
- ATE / RPE 用 `evo_ape` / `evo_rpe` 算；轨迹参考文件写清楚
- CPU / 延迟用 `top` / `htop` 或节点自带 stats

---

## 矩阵总览

| 系统 \ 数据集 | GeoScan B1 mono-inertial | GeoScan B1 stereo-inertial | EuRoC V101 | TUM RGB-D |
|---------------|--------------------------|-----------------------------|------------|-----------|
| open_vins | ⬜ | ⬜ | ⬜ | — |
| sqrtVINS | ⬜ | ⬜ | ⬜ | — |
| mins | ⬜ (ROS1) | ⬜ (ROS1) | ⬜ | — |
| EPLF-VINS | ⬜ (ROS1) | — | ⬜ | — |
| VINGS-Mono | — | — | ⬜ | — |
| ORB_SLAM3_ROS2 | 🟢（参考基线） | 🟢 | ⬜ | ⬜ |
| VINS-Fusion-ROS2 | ⬜ | ⬜ | ⬜ | — |
| AirSLAM | ⬜ | 🟢 2026-04-16（Z 漂修复后 Z∈[0,1.1]m, 535 KF）| ⬜ | — |
| droid_w_ros2 / DROID-W | ⬜ (批处理) | — | ⬜ | ⬜ |
| cuvslam_ros | ⬜ | ⬜ | ⬜ | ⬜ |
| DPVO | ⬜ (COLCON_IGNORE) | — | ⬜ | — |
| rgbdslam_v2 | — | — | — | ⬜ |
| rtabmap | 在 `~/rtabmap_ws/`，独立记录 | — | — | — |
| limap（3D 线后处理，非 SLAM） | — | 🟢 2026-04-16 finder VIO + BA hybrid 出 280 条 nv≥4（14× 纯 VIO，2.4× 纯 SfM） | ⬜ | — |

> 单元格内容示例：`🟢 ATE 0.28m / CPU 42%` 或 `🔴 IMU init 失败，见 §2.3`

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
- [ ] 跑 `ORB_SLAM3` 的 EuRoC V101 做横向参照（有真实 GT）
- [ ] 写一个 `scripts/` 下的 `run_benchmark.sh` 批量脚本（待几个系统都跑通后再做）

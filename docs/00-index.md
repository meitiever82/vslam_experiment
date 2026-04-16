<!--
 * @Author: meitiever
 * @Date: 2026-04-15 14:22:24
 * @LastEditors: meitiever
 * @LastEditTime: 2026-04-15 14:27:20
 * @Description: content
-->
# vslam_ws 仓库总览

本目录对 `~/vslam_ws/src/` 下 **19 个 SLAM 仓库** 做代码级 / 摘要分析，按类别拆分到 5 个子文档。其中：

- 2 个系统（`ORB_SLAM3_ROS2` / `VINS-Fusion-ROS2`）迁自 `src/vslam-summary.md`；`rgbdslam_v2` 同批次迁入
- 3 个系统（`limap` / `MapAnything` / `VGGT-SLAM`）迁自 `src/vslam_summary_v2.md`
- `MapAnything` 本工作区无源码（只通过 pip / HF 用），其余均在本工作区

分析基于真实代码（`src/` / `include/` / `launch/` / `CMakeLists.txt` / `package.xml` 等），不只是 README。

## 文档导航

| 类别 | 文档 | 包含仓库 |
|------|------|----------|
| VIO / Visual-Inertial SLAM | [01-vio-vi-slam.md](01-vio-vi-slam.md) | `open_vins`, `mins`, `sqrtVINS`, `EPLF-VINS`, `VINGS-Mono`, `ORB_SLAM3_ROS2`, `VINS-Fusion-ROS2`, `AirSLAM` |
| 深度学习 VO | [02-deep-vo.md](02-deep-vo.md) | `droid_w_ros2`, `cuvslam_ros`, `DPVO` |
| 建图 / 稠密重建 | [03-mapping-dense.md](03-mapping-dense.md) | `slim-vdb`, `rgbdslam_v2` |
| 3D Gaussian Splatting SLAM | [04-gaussian-slam.md](04-gaussian-slam.md) | `Gaussian-LIC`, `GS_ICP_SLAM`, `Mobile-GS`, `SGS-SLAM` |
| 前馈式 3D 重建 / SLAM | [05-feedforward-recon.md](05-feedforward-recon.md) | `limap`, `MapAnything`（pip）, `VGGT-SLAM` |
| Benchmark 记录 | [06-benchmarks.md](06-benchmarks.md) | 各系统实际跑 GeoScan / EuRoC / TUM 的结果矩阵、踩坑、运行记录 |
| Benchmark 迁移 SOP | [PLAN-benchmark-machine-setup.md](PLAN-benchmark-machine-setup.md) | 把 benchmark 环境搬到另一台机器的标准流程 |

> **建图主力 `rtabmap`** 见 `~/rtabmap_ws/docs/overview.md`。

## 19 仓库速查表

**深度** = 代码级分析（文件:行号、类名、参数默认值）；**摘要** = 概述级（核心算法 + 输入输出 + 代码入口）。

| # | 仓库 | 类别 | 核心算法 | 主要输入 | ROS2 原生 | 实时 | 语言 | 分析深度 |
|---|------|------|----------|----------|-----------|------|------|----------|
| 1 | open_vins | VIO | MSCKF (error-state EKF) | cam + IMU | ✅ | ✅ | C++ | **深度** |
| 2 | mins | VIO (multi-sensor) | MSCKF + GPS/LiDAR/wheel/Vicon updaters | cam + IMU + 多传感器 | ❌ (ROS1) | ✅ | C++ | 摘要 |
| 3 | sqrtVINS | VIO | Square-root KF (Cholesky) | cam + IMU | ✅ | ✅ | C++ | **深度** |
| 4 | EPLF-VINS | VIO (点+线) | Ceres 滑窗 + EDLines | cam + IMU | ❌ (ROS1) | ✅ | C++ | 摘要 |
| 5 | VINGS-Mono | VIO + NeRF-SLAM | DL 前端 + GTSAM + 3DGS | cam + IMU（离线） | ❌ | ❌ | Python | 摘要 |
| 6 | ORB_SLAM3_ROS2 | 特征点 SLAM | ORB + Atlas + g2o BA | 单/双/RGB-D ± IMU | ✅ | ✅ | C++ | **深度** |
| 7 | VINS-Fusion-ROS2 | VIO | Ceres 滑窗 + 光流 | 单/双 + IMU ± GPS | ✅ | ✅ | C++ | **深度** |
| 8 | droid_w_ros2 | 深度 VO | DROID-SLAM (RAFT + DBA) | 单目 RGB | ✅ Jazzy | ❌（批） | Python | 摘要 |
| 9 | cuvslam_ros | 深度 VO | NVIDIA cuVSLAM SDK wrap | mono/stereo/RGBD + IMU | ✅ | ✅ | C++ | **深度** |
| 10 | slim-vdb | 稠密重建 | OpenVDB TSDF + 语义 | RGB-D / LiDAR + 外部位姿 | ❌ | ✅ | C++/Py | 摘要 |
| 11 | rgbdslam_v2 | RGB-D SLAM | 3D-3D RANSAC + g2o 位姿图 | RGB-D | ✅（移植版） | 中等 | C++ | **深度** |
| 12 | Gaussian-LIC | 3DGS SLAM | Coco-LIC VIO + 3DGS | LiDAR + cam + IMU | ❌ (ROS1) | ✅ | C++ | 摘要 |
| 13 | GS_ICP_SLAM | 3DGS SLAM | Fast-GICP + 3DGS 双线程 | RGB-D | ❌ | ✅ | Python | 摘要 |
| 14 | Mobile-GS | 3DGS 离线重建 | Mini-Splatting + GPCC 压缩 | COLMAP 图像集 | ❌ | ❌ | Python | 摘要 |
| 15 | SGS-SLAM | 3DGS + 语义 SLAM | SplaTAM 扩展 + 语义 Loss | RGB-D + 语义 | ❌ | ❌ | Python | 摘要 |
| 16 | limap | 前馈 3D 重建 | 线特征 SfM（C++ 优化） | 多视 RGB + COLMAP 位姿 | ❌ | ❌（离线） | C++/Py | 摘要 |
| 17 | VGGT-SLAM | 前馈 SLAM | VGGT 前馈 + SL(4) 因子图 | 有序 RGB 视频（未标定） | ❌ | ✅ v2 | Python | 摘要 |
| 18 | AirSLAM | 特征点 SLAM | 点+线 g2o BA + SuperPoint/PLNet/LightGlue | 单/双 ± IMU | ❌（独立 C++） | ✅ | C++ | **深度** |
| 19 | DPVO | 深度 VO | 稀疏 Patch + Sparse BA + DBoW2 PGO(v2) | 单目 | ❌（COLCON_IGNORE） | ✅ 120+Hz | Python | **深度** |

> `MapAnything` 无本地仓库（pip / HF 调用），未列入表。见 [05-feedforward-recon.md](05-feedforward-recon.md) §2。

## 关键观察

1. **ROS2 原生**：`open_vins`、`sqrtVINS`、`cuvslam_ros`（vslam_ws 内）；`rtabmap_ros`（独立 workspace）。其余若要接入 ROS2 主干，需要自行写 bridge 或解决 ROS1→2 迁移。
2. **Delaware RPNG 系**：`open_vins` / `mins` / `sqrtVINS` 共用 `ov_core`，建议只 clone 一次核心库后切换上层应用。
3. **Gaussian Splatting 4 个均非 ROS2 原生**：`Gaussian-LIC` 只有 ROS1，其他三个是离线 Python 脚本；若要实时在线运行需要额外工程。
4. **v0.2 目标架构（多模态 SLAM）**：
   - 里程计：GLIM pure LIO + 视觉扩展（他人在做，仿 ORB 思路）—— **不在本 workspace**
   - 建图 + 全局 SLAM + 视觉回环：**rtabmap** with superpoint+LightGlue —— **独立 workspace `~/rtabmap_ws/`**
   - 设计文档：`~/finder_localization/doc/multimodal-slam/`

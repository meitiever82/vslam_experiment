# 3D Gaussian Splatting SLAM

本文档分析 `src/` 下 4 个基于 3D Gaussian Splatting（3DGS）的 SLAM / 重建仓库：
**Gaussian-LIC**、**GS_ICP_SLAM**、**Mobile-GS**、**SGS-SLAM**。

---

## 1. Gaussian-LIC

### 简介
Xiaolei Lang / Xingxing Zuo / Lina Liu 等（浙大 APRIL Lab），ICRA 2025（arxiv 2404.06926）。首个紧耦合 **LiDAR-Inertial-Camera** 三传感器 3DGS SLAM。位姿估计由 Coco-LIC（紧耦合 LIO/VIO）提供，高斯基元由 LiDAR 点云直接初始化，解决了尺度问题并显著提升长序列稳定性。

### 核心算法
- **追踪**：不自己做追踪，位姿来自 **Coco-LIC**（LiDAR-Inertial VIO）。
- **高斯表示**：
  - 初始化：LiDAR 点云 + RGB-D 深度（通过 **SPNet** 深度补全）
  - 协方差：完整 3×3；透明度 sigmoid；SH ≤ 3 阶
- **建图**：
  - 梯度驱动 densification（split + clone），低透明度 / 屏幕过大剪枝
  - Loss：`0.5 · L1_RGB + 0.5 · SSIM` + 深度约束 + 曝光补偿项
- **加速**：SPNet 用 TensorRT 推理；自定义 CUDA 光栅化器 + `fused-ssim`。
- **全局一致性**：无独立回环，依赖 VIO 位姿图。

### 输入 / 输出
- **输入**：RGB + LiDAR + IMU（rosbag，Coco-LIC 消费）。
- **输出**：渲染 RGB / 深度、`.ply` 高斯文件（`result/` 目录）。

### 依赖与平台
- CUDA 11.7、LibTorch、TensorRT 8.6.1、OpenCV 4.7（CUDA + cuDNN 编译）。
- **ROS1**（Coco-LIC 基于 ROS1），**不是原生 ROS2**。
- GPU：RTX 3090 / 4090，显存 ≥ 24 GB 推荐。

### 优缺点
| ✅ | ❌ |
|----|----|
| 实时 LIC 紧耦合，长序列稳定 | 依赖 Coco-LIC 部署复杂 |
| LiDAR 初始化消除尺度不确定性 | 无视觉回环 |
| SPNet 补全深度提升几何精度 | 仍只支持 ROS1 |

### 代码入口
- `src/mapping.cpp` — ROS 节点主循环
- `src/gaussian.cpp` — 高斯管理
- `src/rasterizer/renderer.cpp` — 渲染管线
- 配置例：`config/ct_odometry_fastlivo2.yaml`

---

## 2. GS_ICP_SLAM

### 简介
Seongbo Ha / Jiung Yeon 等，ECCV 2024（arxiv 2403.12550）。以 **Fast-GICP** 做追踪、3DGS 做建图的**双线程** RGB-D SLAM，宣称 100 FPS 级追踪。以"快"著称。

### 核心算法
- **追踪**：Fast-GICP（广义 ICP，法线/特征加权 + PCL k-d tree），上一关键帧 → 当前帧点云配准。
- **高斯表示**：
  - 初始化：RGB-D 深度→点云，尺度 `depth / ((fx+fy)/2)`
  - 完整 3×3 协方差；SH ≤ 3 阶；透明度初值 0.5
- **建图**：
  - 梯度驱动 densify，低 α（<0.05）+ 屏幕>50 px 剪枝
  - Loss：`L1_RGB + 0.2 · SSIM`，无深度约束
- **关键设计**：
  - `Trackable` 掩码 —— 分离"可追踪高斯"（用于 ICP）与"仅渲染高斯"
  - 多进程：`Tracker` / `Mapper` 共享内存并行
- 无回环 / 全局 BA。

### 输入 / 输出
- **输入**：RGB + depth（Replica / TUM-RGBD 格式）
- **输出**：渲染 RGB、Rerun 实时可视化、SIBR viewer、`.ply`

### 依赖与平台
- PyTorch 2.0 + CUDA 11.8；PCL（fast-GICP）；`diff-gaussian-rasterization` 修改版；`simple-knn`。
- **无 ROS / ROS2**，离线 Python 脚本。
- GPU：显存 ≥ 8 GB。

### 优缺点
| ✅ | ❌ |
|----|----|
| 极高追踪 FPS（100+） | 纯 RGB-D，无单目 |
| ICP 对光照鲁棒 | 长序列 ICP 漂移累积，无回环 |
| 代码清晰 | 无语义 |

### 代码入口
- `gs_icp_slam.py` — 主入口
- `mp_Tracker.py` — Fast-GICP 线程
- `mp_Mapper.py` — 高斯优化线程
- `scene/gaussian_model.py` — 高斯参数

---

## 3. Mobile-GS

### 简介
Xiaobiao Du / Yida Wang（浙大），预印本（arxiv 2603.11531，2026 年 3 月）。面向**移动端 / 边缘设备**的高斯重建，强调压缩 + 多视图微调。当前仓库只有 CUDA 训练代码，Vulkan 移动端推理闭源。

### 核心算法
- **追踪**：**无**，用 COLMAP 预计算位姿。属离线重建，不是 SLAM。
- **高斯表示**：
  - 初始化：COLMAP 稀疏点 → 稠密 SfM
  - 完整协方差 + sigmoid 透明度 + 可配 SH 阶
  - **创新**：`OpaictyPhiNN` 学习型 MLP 预测逐高斯透明度 / 重要性
- **建图两阶段**：
  1. Mini-Splatting 预训练（30K 迭代，压缩初始集）
  2. Fine-tune：多视图蒸馏 + 渐进 SH 升级
- **Loss**：`L1 + LPIPS + SSIM`；重要性驱动 densify 与动态剪枝。
- **压缩**：GPCC（MPEG 点云编码）+ Morton 排序；`.xz` 压缩文件，支持增量加载。

### 输入 / 输出
- **输入**：COLMAP 结构化图像目录
- **输出**：`.ply`（未压缩）或 `.xz`（压缩），兼容任意 3DGS viewer

### 依赖与平台
- PyTorch 2.5.1 + CUDA 11.8；`tiny-cuda-nn`；`cuml`（Rapids 聚类）；GPCC `tmc3`；修改版 `diff-gaussian-rasterization`。
- **无 ROS / ROS2**。
- GPU：8–24 GB（视场景大小）。

### 优缺点
| ✅ | ❌ |
|----|----|
| 优秀压缩率，移动端传输友好 | 纯离线，不是 SLAM |
| 分阶段训练灵活 | Vulkan 移动推理部分闭源 |
| 动态重要性 MLP 减少冗余高斯 | 无时序位姿估计能力 |

### 代码入口
- `pretrain.py` — Mini-Splatting 压缩阶段
- `train.py` — 主训练（支持 `--start_checkpoint`, `--mv`）
- `render.py` — 渲染
- `scene/gaussian_model.py` — 含 `OpaictyPhiNN`
- `utils/gpcc_utils.py` — 压缩工具链

---

## 4. SGS-SLAM

### 简介
Mingrui Li / Shuhong Liu / Heng Zhou 等（云天励飞），ECCV 2024（arxiv 2402.03246）。首个把**语义**融入 3DGS 的 SLAM 系统，基于 SplaTAM 架构扩展，同时优化几何 + 外观 + 语义三通道。

### 核心算法
- **追踪**：类 SplaTAM，**优化式追踪**——40 次迭代优化相机位姿（R+t），高斯参数固定；位姿初值来自前向传播或 GT。
- **高斯表示**：
  - 初始化：RGB-D → 点云，尺度由相邻点均方距离推导
  - 完整协方差；logit_opacity；SH 3 阶
  - **扩展**：每个高斯额外带语义特征 embedding 通道
- **建图多通道 Loss**：
  - RGB L1（权 0.5）
  - Depth L2（权 1.0）
  - **Semantic Feature Loss**（权 0.05）—— 缓解 oversmoothing
- **densify**：梯度 + Silhouette 引导添加；低 α / 屏幕过大剪枝。
- **关键帧**：**Semantic-guided keyframe selection**，按语义一致性选帧。
- **后优化**（`post_slam_opt.py`）：全局 BA，所有高斯 + 位姿联合优化 → 回环一致性。

### 输入 / 输出
- **输入**：RGB + depth + 语义标签图（Replica / ScanNet / ScanNet++ 格式）
- **输出**：
  - `.npz`（外观 + 语义）
  - `.ply`（RGB / 语义渲染）
  - 可用 SuperSplat Web / Open3D 查看

### 依赖与平台
- PyTorch 2.0.1 + CUDA 11.8；`diff-gaussian-rasterization`；`simple-knn`；GradSLAM 数据加载；wandb；Open3D。
- **无 ROS / ROS2**，离线 Python。
- GPU：Replica ≥ 12 GB，ScanNet > 24 GB。

### 优缺点
| ✅ | ❌ |
|----|----|
| 首个端到端语义 3DGS SLAM | 离线，不支持实时追踪 |
| 语义约束提升物体级几何 | 依赖 GT 或前向传播位姿 |
| 后全局 BA 改进闭环一致性 | 多通道优化计算成本高 |

### 代码入口
- `scripts/slam.py` — SLAM 主循环
- `scripts/gaussian_splatting.py` — 高斯管理
- `scripts/post_slam_opt.py` — 全局优化
- `utils/slam_helpers.py` — 语义 & Loss 计算
- 配置例：`configs/replica/slam.py`

---

## 横向对比

| 维度 | Gaussian-LIC | GS_ICP_SLAM | Mobile-GS | SGS-SLAM |
|------|--------------|-------------|-----------|----------|
| 发表 | ICRA 2025 | ECCV 2024 | Preprint 2026 | ECCV 2024 |
| 传感器 | LiDAR + RGB + IMU | RGB-D | 单目 + COLMAP（离线） | RGB-D + 语义 |
| 追踪方法 | Coco-LIC VIO | Fast-GICP | 无（COLMAP 预计算） | 优化式（40 iter） |
| 实时性 | ✅ 实时 | ✅ 100 FPS 追踪 | ❌ 离线 | ❌ 离线 |
| 语义 | ❌ | ❌ | ❌ | ✅ 原生 |
| 回环 / 全局 BA | 依赖 VIO | ❌ | ❌ | ✅ 后优化阶段 |
| Loss | RGB + SSIM + 曝光 + 深度 | RGB + 0.2 SSIM | RGB + LPIPS + SSIM | RGB + Depth + 语义 |
| densify 策略 | 梯度 + SPNet 补全 | 梯度 + Trackable 掩码 | 重要性 MLP + 动态阈值 | 梯度 + Silhouette |
| GPU 显存 | ≥ 24 GB | ≥ 8 GB | 8–24 GB | 12–24 GB |
| ROS | **ROS1** | 无 | 无 | 无 |
| 典型定位 | 户外 LIC 重建 | 快速 RGB-D SLAM | 移动端离线重建 | 室内语义建图 |

### 选型建议
- **带 LiDAR 的户外实时场景** → Gaussian-LIC（但要接受 ROS1）
- **RGB-D 室内，追求速度** → GS_ICP_SLAM
- **离线重建 + 移动端部署** → Mobile-GS
- **需要语义地图** → SGS-SLAM
- 目前这 4 个都**没有原生 ROS2 节点**，若要接入本 workspace 的 ROS2 管线，需要自行写 bridge（订阅图像 / LiDAR / TF，按其期望格式喂入）。

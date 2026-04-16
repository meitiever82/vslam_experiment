# 建图 / 稠密重建 模块

本文档对 `src/` 下 **2 个**建图 / 稠密重建仓库进行代码级分析：**slim-vdb**、**rgbdslam_v2**。

> §2 `rgbdslam_v2` 从 `src/vslam-summary.md` 迁移合成（2026-04-15）。
> **`rtabmap` 已迁到独立 workspace `~/rtabmap_ws/`**（因为 v0.2 要在 core 层改 PyMatcher 接 XFeat+LightGlue，不能用系统 apt 也不和 vslam_ws 混编）。文档见 `~/rtabmap_ws/docs/overview.md`。

---

## 1. slim-vdb

### 简介
Anja Sheppard 等（University of Michigan），2025 年 IEEE RA-L 发表。基于 VDBFusion / OpenVDB 的**轻量级体素化概率语义地图**框架，强调实时性与编译期特化。支持开集（open-set，连续嵌入）与闭集（closed-set，类别索引）两种语义。**注意它不是完整 SLAM**——只做体素融合 + 语义概率更新，假定外部已提供位姿。

### 核心算法
- **回环检测**：**无**，位姿由外部输入。
- **图优化**：**无**，逐帧 TSDF 融合。
- **地图结构**：三层 OpenVDB 稀疏网格
  - `tsdf_`：`FloatGrid`（有符号距离场）
  - `weights_`：`FloatGrid`（融合权重）
  - `semantics_`：`VecXGrid<L, NCLASSES>`（类别概率向量或连续特征向量）
- **关键创新**：通过模板参数 `Language L` 与 `NCLASSES` 把语义空间在编译期常数化，消除运行期条件分支；NanoVDB 子树支持 GPU 渲染（CUDA kernels）。
- **显式地图**：需要时通过 marching cubes 从 TSDF 提取 triangle mesh。

### 输入 / 输出
- **输入**：RGB-D 图像对、3D 点云、4×4 相机 / 传感器位姿，可选语义标签（`uint32` 闭集或 `float[]` 开集嵌入）。
- **输出**：`.vdb` 二进制网格文件、OBJ/PLY triangle mesh、任意视角渲染图、Python 端返回 `openvdb::FloatGrid*`。

### 依赖
- OpenVDB、Eigen3、pybind11（Python 绑定）。
- 可选：CUDA / NanoVDB（实时 GPU 渲染）、libtorch（实时语义推理）。
- **无 ROS / ROS2 依赖**。
- 编译期参数：`SLIMVDB_LANGUAGE`（OPEN/CLOSED）、`SLIMVDB_NCLASSES`。

### ROS2 集成
**无原生节点**。使用时需要自行写 Python 或 C++ 适配层，把 `sensor_msgs/PointCloud2`、`sensor_msgs/Image`、`nav_msgs/Odometry` 转成 Eigen 类型后调用：
```python
from slimvdb.pybind import VDBVolumeWrapper
volume = VDBVolumeWrapper(voxel_size=0.05, ...)
volume.Integrate(points, labels, origin)
```

### 用法示例
```bash
./scenenet_pipeline --config config/scenenet.yaml \
                    --sequence 2 ../../data/ mesh_output
```
关键配置项：`voxel_size`、`sdf_trunc`（截断距离）、`space_carving`、`rgbd`、`realtime_segmentation`、`color_map`。

### 优缺点
| 优势 | 劣势 |
|------|------|
| 体素稀疏存储，内存高效 | 无回环 / 全局优化，完全吃外部里程计漂移 |
| GPU 渲染（NanoVDB） | 无 ROS2 原生节点，集成工作量大 |
| 编译期特化，运行期分支极少 | 研究级代码，文档与样例有限 |
| 支持开 / 闭集语义融合 | 长序列累积误差无法自愈 |

### 代码位置与入口
- `src/slimvdb/slimvdb/VDBVolume.h`（模板类 `VDBVolume<Language L>`，约 180 行声明）
- `src/slimvdb/slimvdb/VDBVolume.cpp`（TSDF 融合实现，500+ 行）
- `src/slimvdb/slimvdb/VDBVolumeWrapper.{h,cpp}`（Python 绑定辅助）
- `src/slimvdb/pybind/slimvdb_pybind.cpp`（pybind11 入口）
- `examples/cpp/scenenet_pipeline.cpp`（完整 C++ 样例）
- `examples/cpp/config/scenenet.yaml`（配置样例）

---

## 2. rgbdslam_v2（深度分析）

> 迁自 `src/vslam-summary.md`（2026-04-15）。从 ROS 1 Kinetic **完整移植到 ROS 2 Humble**（43 个文件，1855 行 + / 2109 行 −）。

### 3.1 架构总览
Uni Freiburg 开发的 **RGB-D 图优化 SLAM**。与 `rtabmap_ros` 的共同点：g2o 位姿图；区别：无 BoW 词袋，用 **3D-3D RANSAC** 直接从深度图求相对变换；前端特征可配（ORB / SIFT / SURF）；自带 Qt5 + OpenGL GUI（不是依赖 RViz）；原生输出 XYZRGB 稠密点云 + OctoMap。

**特点**：
- 特征匹配（ORB / SIFT / SURF / SIFTGPU）
- **3D-3D RANSAC** 刚体变换估计（直接用深度）
- g2o 稀疏位姿图（非 BA，无地图点联合优化）
- OctoMap 三维占据网格
- Qt5 OpenGL 可视化（独立 GUI 主循环 + ROS 子线程）
- 稠密 XYZRGB 点云重建（导出 PLY / PCD）

**ROS2 架构（Qt ↔ ROS 桥接）**：
```
Qt 主线程              ↔ QtROS 适配层 (qt_ros.cpp)
  ├─ QApplication (GUI)
  ├─ QGLViewer (OpenGL)
  └─ Qt signals/slots

ROS 2 子线程
  ├─ openni_listener.cpp     订阅 + ApproximateTime 同步
  ├─ graph_manager.cpp       g2o 位姿图
  └─ parameter_server.cpp    参数管理（单例）
```

**主要源码**：
```
src/
├── main.cpp                    # 入口：QApplication + rclcpp
├── qt_gui.{cpp,h}              # 主窗口
├── openni_listener.cpp         # ROS 2 订阅 + Node 构建
├── graph_manager.cpp           # g2o 位姿图管理
├── node.{cpp,h}                # 关键帧：特征 / 匹配 / RANSAC
├── features.{cpp,h}            # 检测器 / 描述子工厂
├── glviewer.{cpp,h}            # OpenGL 可视化
├── parameter_server.cpp        # 参数（单例）
├── ColorOctomapServer.cpp      # OctoMap 服务
└── transformation_utility.cpp
```

**数据流**：
```
RGB-D topic（ApproximateTime）
  → openni_listener.cpp:100 ImageCallback
    → graph_manager.cpp:200 addNode()
      → node.cpp:50  extractFeatures()
      → node.cpp:150 featureMatching()    # FLANN + Lowe ratio
      → node.cpp:250 getRelativeTransformationTo()  # 3D-3D RANSAC
      → graph_manager.cpp:300 addEdge()    # g2o SparseOptimizer
      → graph_manager.cpp:400 optimizeGraph()
  → glviewer.cpp:500 draw()（OpenGL 点云 + KF 位姿 + OctoMap）
  → ColorOctomapServer.cpp 更新
```

### 3.2 特征前端
工厂（`features.cpp`）支持 FAST / SURF / SIFT / ORB 检测，ORB / SURF / SIFT / BRIEF 描述子，可选 `DynamicAdaptedFeatureDetector` 自适应调节特征数。

```cpp
// node.cpp — extractFeatures()
void Node::extractFeatures(const cv::Mat &image) {
    cv::Mat masked_image = image.clone();
    masked_image.setTo(0, depth_mask);   // 深度无效区域 = 黑

    detector->detect(masked_image, feature_locations_2d_);          // 可 3×3 网格
    extractor->compute(masked_image, feature_locations_2d_, feature_descriptors_);
    projectTo3D(feature_locations_2d_, feature_locations_3d_, depth, cam_info);
}
```

关键参数（launch 默认）：
```yaml
feature_detector_type:       ORB
feature_extractor_type:      ORB
max_keypoints:               600
max_matches:                 300
detector_grid_resolution:    3        # 3×3 网格
ransac_iterations:           100
```

### 3.3 特征匹配 + 3D-3D RANSAC
```cpp
// node.cpp — featureMatching()
cv::FlannBasedMatcher matcher(
    new cv::flann::KDTreeIndexParams(),
    new cv::flann::SearchParams());
vector<vector<cv::DMatch>> knn_matches;
matcher.knnMatch(descs_a, descs_b, knn_matches, 2);   // KNN=2

// Lowe ratio test
for (const auto &m : knn_matches)
    if (m.size() == 2 && m[0].distance / m[1].distance < 0.7)
        good.push_back(m[0]);
```

```cpp
// node.cpp — getRelativeTransformationTo()
// 1. 把 2D 匹配点按深度反投影到 3D
// 2. RANSAC 在 3D-3D 对齐：随机 3 点对 → SVD 得 SE(3) → 内点统计
// 3. 输出 transformation + rmse
// 4. 若内点数 > MIN_INLIERS，成功
```

### 3.4 IMU 融合
**不支持**。`rgbdslam_v2` 纯视觉，深度图本身提供尺度。

### 3.5 后端（g2o 位姿图）
```cpp
// graph_manager.cpp
class GraphManager : public QObject {
    g2o::SparseOptimizer* optimizer_;
    std::map<int, Node*>  graph_;
    // 顶点：SE(3) KF 位姿
    // 边：MatchingResult 提供的相对变换
};
```

求解器选择：
```yaml
backend_solver:           pcg        # 或 Cholmod / CSparse
optimization_iterations:  5
```

典型耗时 ~10 ms（取决于图规模）。**纯位姿图，不做 BA**。

### 3.6 回环检测（隐式）
**无专门词袋**，靠随机采样历史帧做全图匹配实现隐式回环：
```yaml
predecessor_candidates:    4   # 与前 4 帧匹配
neighbor_candidates:       4   # 与图邻居匹配
min_sampled_candidates:    4   # 随机采样历史帧
nn_distance_ratio:         0.6
ransac_iterations:         100
```
低复杂度、无词袋数据库，但检测效率低（暴力匹配），长序列闭合能力偏弱。

### 3.7 初始化
**极简**：接第一帧 RGB-D → 提特征 → 反投影到 3D → 设为 identity → 之后每帧 RANSAC 对齐到第一帧。深度直接给尺度，无单目歧义。

### 3.8 地图管理
- 特征 3D 点由深度图反投影
- 稠密 XYZRGB 点云（VoxelGrid 降采样，`cloud_creation_skip_step`）
- `ColorOctomapServer` 维护全局 OctoMap（默认 `octomap_resolution: 0.05`）
- 持久化：TUM 轨迹 / OctoMap 二进制 / PLY / PCD

### 3.9 话题 & launch
| 订阅 | 类型 | 同步 |
|------|------|------|
| RGB 图像 | `sensor_msgs/Image` | NoCloudSyncPolicy |
| 深度图 | `sensor_msgs/Image` | ApproximateTime |
| CameraInfo | `sensor_msgs/CameraInfo` | ApproximateTime |

发布：`~/point_cloud`、`~/octomap_binary`、`~/keyframe_poses`（`MarkerArray`）。

运行：
```bash
ros2 launch rgbdslam rgbdslam_tum_demo.launch.py \
     feature_detector_type:=ORB feature_extractor_type:=ORB \
     max_keypoints:=600 max_matches:=300
ros2 bag play /home/steve/Documents/Datasets/tum/rgbd_dataset_freiburg1_xyz \
     --clock --rate 0.5
```

### 3.10 代码入口速查
| 文件:行 | 内容 |
|---------|------|
| `src/main.cpp:1` | 入口 |
| `src/qt_gui.cpp:100` | Qt 主窗口 |
| `src/openni_listener.cpp:100` | ROS2 订阅 + Node 构建 |
| `src/graph_manager.cpp:1` | 位姿图类 |
| `src/graph_manager.cpp:200` | `addNode()` |
| `src/graph_manager.cpp:400` | `optimizeGraph()` |
| `src/node.cpp:50` | `extractFeatures()` |
| `src/node.cpp:150` | `featureMatching()` |
| `src/node.cpp:250` | `getRelativeTransformationTo()` |
| `src/features.cpp:50` | `createDetector()` |
| `src/glviewer.cpp:500` | OpenGL 可视化 |

### 3.11 优缺点
| ✅ | ❌ |
|----|----|
| 3D-3D 匹配直接用深度，几何约束强 | 无 IMU，快速运动不行 |
| 稠密点云 + OctoMap 导航友好 | SIFT/SURF 慢（~50 ms/帧） |
| RGB-D 直接给尺度 | 无重定位 / 词袋，长期闭合弱 |
| 初始化简单，快速启动 | 对深度图质量敏感 |
| Qt GUI + OpenGL 自带可视化 | ROS2 迁移后编译依赖复杂 |

### 3.12 ROS 2 移植要点
从 Kinetic → Humble 改动：
1. **QtROS 桥接**：Qt signals/slots 与 `rclcpp` 回调整合（线程安全）
2. **cv_bridge 路径**：Humble 里用 `cv_bridge/cv_bridge.h`（不是 `.hpp`）
3. **参数 API**：`roscpp::param` → `rclcpp::declare_parameter / get_parameter`
4. **消息命名空间**：`sensor_msgs/msg/Image` 等
5. **时间戳**：`ros::Time` → `rclcpp::Clock().now()`
6. **pub/sub**：`ros::subscribe()` → `node->create_subscription<>()`
7. ROS2 节点在 `openni_listener.cpp` 中用 `rclcpp::Node::SharedPtr` 创建，`message_filters::ApproximateTimeSynchronizer` 同步 RGB + Depth + CameraInfo。

### 3.13 实战记录（TBD）
> 留给后续用 Orbbec Gemini 336 在 B1 跑 `rgbdslam_v2` 的实机记录。

---

## 横向对比

| 维度 | slim-vdb | rgbdslam_v2 |
|------|----------|-------------|
| 功能定位 | 稠密体素重建 + 语义融合 | RGB-D 图优化 SLAM |
| 输入传感器 | RGB-D / 3D LiDAR + 外部位姿 | 仅 RGB-D |
| 地图表示 | OpenVDB TSDF + 语义网格 | 稠密 XYZRGB 点云 + OctoMap |
| 回环检测 | 无 | 隐式（随机历史帧匹配） |
| 后端 | 无（逐帧 TSDF） | g2o 位姿图（非 BA） |
| 语义 | 原生（开集 / 闭集） | 无 |
| 前端 | 外部位姿 | ORB / SIFT / SURF（可选） + 3D-3D RANSAC |
| ROS2 支持 | 无原生节点 | ROS1 完整移植到 ROS2 |
| 成熟度 | 研究级，2025 | 成熟（Freiburg）+ 迁移版 |
| 实时性 | 100–200 ms/帧 | SIFT ~50ms / ORB 快 |
| GPU 加速 | 原生（NanoVDB CUDA） | 可选 SIFTGPU |
| 长时间运行 | 受限（无回环） | 中等（隐式回环弱） |
| 典型搭配 | 接 LIO / VIO 后做重建 | 独立 RGB-D SLAM + 稠密云 |

> **选型**：已有外部位姿只做稠密重建走 `slim-vdb`；历史遗留 RGB-D 方案或需要 Qt GUI + 稠密点云原生输出走 `rgbdslam_v2`。
> v0.2 主力建图走 **rtabmap**（独立 workspace `~/rtabmap_ws/`，详见那里的 `docs/overview.md`）。

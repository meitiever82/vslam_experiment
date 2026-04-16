# VIO / Visual-Inertial SLAM

本文档分析 `src/` 下 **8 个** VIO / VI-SLAM / 特征点 SLAM 仓库：

- **VIO / VI-SLAM**：`open_vins`、`mins`、`sqrtVINS`、`EPLF-VINS`、`VINS-Fusion-ROS2`、`VINGS-Mono`
- **特征点 SLAM**：`ORB_SLAM3_ROS2`、`AirSLAM`

> §6 / §7 迁自 `src/vslam-summary.md`（2026-04-15）；§8 `AirSLAM` 为新增（TRO 2025，独立 C++）。

---

## 1. open_vins（深度分析）

> 以下章节以代码级颗粒度重写（2026-04-15）。

### 1.1 架构总览
University of Delaware RPNG Lab（Patrick Geneva、Guoquan Huang 等）出品。error-state MSCKF（Multi-State Constraint Kalman Filter）+ CPI（Continuous Preintegration），**原生同时支持 ROS1 与 ROS2**（CMake 自动检测）。

主数据流（文件:行号）：

```
IMU 回调   → VioManager::feed_measurement_imu()             VioManager.cpp:166
                ├─ Propagator::feed_imu()
                └─ UpdaterZeroVelocity::feed_imu()

图像回调 → VioManager::track_image_and_update()            VioManager.cpp:256
            → trackFEATS->feed_new_camera()                   (ov_core)
            → VioManager::try_to_initialize() (若未初始化)    :311
            → VioManager::do_feature_propagate_update()       :323
                ├─ Propagator::propagate_and_clone()          :341
                ├─ feats_lost / feats_marg / feats_slam 分类  :369-376
                ├─ UpdaterMSCKF::update(feats_msckf)          :525
                ├─ UpdaterSLAM::update() + delayed_init()     :541
                └─ StateHelper::marginalize_clone()           :553-600
```

### 1.2 特征前端（`ov_core::track/`）
继承体系（基类 `ov_core/src/track/TrackBase.h:72`）：
- `TrackKLT`（`TrackKLT.h:39`）—— FAST + 金字塔 Lucas-Kanade，单 / 立体。
- `TrackDescriptor`（`TrackDescriptor.h:37`）—— ORB + KNN + 比例测试（`knn_ratio=0.70`）+ RANSAC。
- `TrackAruco`（`TrackAruco.h:43`）—— OpenCV `DICT_6X6_1000`，四角点进入状态。

网格化提取：`ov_core/src/track/Grider_FAST.h:46`（把图像切成 `grid_x × grid_y` 区域，每格均匀抽 FAST 角点）。

默认参数（`ov_msckf/config/euroc_mav/estimator_config.yaml:80-90`）：
```yaml
num_pts:          200     # 每相机特征数
fast_threshold:   20
grid_x:            5
grid_y:            5
min_px_dist:      10      # 特征间最小像素距离
knn_ratio:        0.70    # 描述子模式下
track_frequency:  21.0    # Hz
histogram_method: HISTOGRAM
```
初始化阶段特征数会减到 `init_max_features / num_cameras`，完成后升到 `num_pts`（`VioManager.cpp:131-140`）。

### 1.3 IMU 预积分（CPI）
两个版本（`ov_core/src/cpi/`）：
- **CpiV1**（`CpiV1.h:52`）—— 分段常数测量假设（Eckenhoff WAFR 2016）。
- **CpiV2**（`CpiV2.h`）—— 连续时间模型，利用 bias / orientation 线性化点（Eckenhoff IJRR 2019），更精确。

连续时间噪声矩阵在 `CpiBase` 构造中初始化（`ov_core/src/cpi/CpiBase.h:60-74`）：
```cpp
Q_c.block(0,0,3,3) = sigma_w  * sigma_w  * I;   // gyro white noise
Q_c.block(3,3,3,3) = sigma_wb * sigma_wb * I;   // gyro bias random walk
Q_c.block(6,6,3,3) = sigma_a  * sigma_a  * I;   // accel white noise
Q_c.block(9,9,3,3) = sigma_ab * sigma_ab * I;   // accel bias random walk
```
`feed_IMU(t0, t1, w, a, ...)` 做区间积分；积分方法由 YAML 的 `integration` 选择：`discrete` / `rk4` / `analytical`。

### 1.4 后端更新器
| Updater | 文件 | 作用 |
|---------|------|------|
| `UpdaterMSCKF` | `ov_msckf/src/update/UpdaterMSCKF.h:48` | 丢失 / 边缘化特征的 null-space 投影 + 卡方测试 + EKF 更新 |
| `UpdaterSLAM` | `UpdaterSLAM.h:50` | 长寿命特征转 SLAM 点（`delayed_init` + `change_anchors`） |
| `UpdaterZeroVelocity` | `UpdaterZeroVelocity.h:54` | 零速 / 低视差检测，静止时以 ZUPT 替代特征更新 |
| `UpdaterHelper` | `UpdaterHelper.h:49` | Jacobian 计算、null-space 投影、测量压缩公用函数 |

`UpdaterOptions`（`UpdaterOptions.h:32`）默认：
```cpp
double chi2_multipler = 5;   // 卡方阈值放大
double sigma_pix      = 1;   // 像素标准差
double sigma_pix_sq   = 1;
```

**null-space 投影**（`UpdaterHelper.cpp:426-454`，Givens 旋转逐列消除 `H_f`）：
```cpp
for (int n = 0; n < H_f.cols(); ++n)
  for (int m = H_f.rows()-1; m > n; --m) {
    tempHo_GR.makeGivens(H_f(m-1,n), H_f(m,n));
    H_f.block(m-1, n, 2, H_f.cols()-n).applyOnTheLeft(0,1, tempHo_GR.adjoint());
    H_x.block(m-1, 0, 2, H_x.cols()    ).applyOnTheLeft(0,1, tempHo_GR.adjoint());
    res.block(m-1, 0, 2, 1            ).applyOnTheLeft(0,1, tempHo_GR.adjoint());
  }
```

**卡方测试**（`UpdaterMSCKF.cpp:208-234`）：
```cpp
Eigen::MatrixXd S = H_x * P_marg * H_x.transpose();
S.diagonal() += sigma_pix_sq * Eigen::VectorXd::Ones(S.rows());
double chi2 = res.dot(S.llt().solve(res));
if (chi2 > chi2_multipler * chi_squared_table[res.rows()])
    (*it)->to_delete = true;
```

### 1.5 初始化（`ov_init/`）
- `InertialInitializer`（`init/InertialInitializer.h:60`）总调度，入口 `initialize(timestamp, cov, order, imu, wait_for_jerk)`（:79-98）。
- `StaticInitializer`（`static/StaticInitializer.h`）—— 设备静止时，基于重力方向 + 零速假设求解姿态。
- `DynamicInitializer`（`dynamic/DynamicInitializer.h`）—— 运动中初始化，Ceres 求解 cam-IMU 外参、初始速度、重力、特征位置。

触发条件（`ov_init/src/init/InertialInitializer.cpp:79-95`）：
```yaml
init_window_time:    2.0     # s
init_imu_thresh:     1.5     # 加速度方差阈值
init_max_disparity: 10.0     # 像素
init_dyn_use:       false
init_dyn_mle_max_iter: 50
init_dyn_num_pose:    6
init_dyn_min_deg:    10.0
init_dyn_inflation_*: 10-100
```

### 1.6 状态管理
**`State`**（`ov_msckf/src/state/State.h:49-180`）持有：
```cpp
std::shared_ptr<ov_type::IMU>                                      _imu;            // q,p,v,bg,ba
std::map<double, std::shared_ptr<ov_type::PoseJPL>>                _clones_IMU;     // 滑窗
std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>>     _features_SLAM;
std::unordered_map<size_t, std::shared_ptr<ov_type::PoseJPL>>      _calib_IMUtoCAM; // 外参
std::unordered_map<size_t, std::shared_ptr<ov_type::Vec>>          _cam_intrinsics; // 内参
std::shared_ptr<ov_type::Vec>                                      _calib_dt_CAMtoIMU;
std::shared_ptr<ov_type::Vec> _calib_imu_dw, _calib_imu_da, _calib_imu_tg;          // 可选 IMU 内参
Eigen::MatrixXd                                                    _Cov;
std::vector<std::shared_ptr<ov_type::Type>>                        _variables;
```

**`StateHelper`**（`StateHelper.h:45`）静态工具：`EKFPropagation`、`EKFUpdate`、`marginalize_clone`、`marginalize_slam`、`get_marginal_covariance` 等。

**`Propagator`**（`Propagator.h:44`）持有 `NoiseManager`（sigma_w / sigma_wb / sigma_a / sigma_ab）与重力矢量；`propagate_and_clone(state, t)` 是主入口。

### 1.7 ROS2 节点 & 话题
可视化：`ov_msckf/src/ros/ROS2Visualizer.{h,cpp}:78`。发布话题（全部 QoS depth=2）：

| topic | 消息 |
|-------|------|
| `poseimu` | `geometry_msgs/PoseWithCovarianceStamped` |
| `odomimu` | `nav_msgs/Odometry` |
| `pathimu` | `nav_msgs/Path`（累积） |
| `points_msckf` / `points_slam` / `points_aruco` / `points_sim` | `sensor_msgs/PointCloud2` |
| `trackhist` | `sensor_msgs/Image` |
| `posegt` / `pathgt` | ground truth（仿真） |
| `/tf` | `world→imu`、`imu→cam_i` |

Launch（`ov_msckf/launch/subscribe.launch.py:10-47`）：
```python
namespace      = "ov_msckf"
config         = "euroc_mav"        # 或 tum_vi / rpng_aruco
config_path    = ""                 # 显式 YAML 路径（优先级更高）
use_stereo     = True
max_cameras    = 2
rviz_enable    = False
save_total_state = False
verbosity      = "INFO"
```

### 1.8 关键 YAML 配置（EuRoC 示例）
`ov_msckf/config/euroc_mav/estimator_config.yaml`：
```yaml
# EKF
use_fej:       true
integration:   rk4        # discrete | rk4 | analytical
# 标定开关
calib_cam_extrinsics: true
calib_cam_intrinsics: true
calib_cam_timeoffset: true
# 状态规模
max_clones:             11
max_slam:               50
max_slam_in_update:     25
max_msckf_in_update:    40
dt_slam_delay:           1   # s
# 特征表示
feat_rep_msckf: GLOBAL_3D
feat_rep_slam:  ANCHORED_MSCKF_INVERSE_DEPTH
feat_rep_aruco: ANCHORED_MSCKF_INVERSE_DEPTH
```

IMU 噪声（`kalibr_imu_chain.yaml`，EuRoC 值）：
```yaml
accelerometer_noise_density: 0.002
accelerometer_random_walk:   0.0003
gyroscope_noise_density:     0.0001
gyroscope_random_walk:       0.000004
update_rate: 200
```

### 1.9 代码入口速查
| 文件:行 | 内容 |
|---------|------|
| `ov_msckf/src/run_subscribe_msckf.cpp:46` | 节点 `main` |
| `ov_msckf/src/core/VioManager.h:62` | 核心类 |
| `ov_msckf/src/core/VioManager.cpp:50` | 构造（装配 tracker / propagator / init） |
| `ov_msckf/src/core/VioManager.cpp:166` | `feed_measurement_imu` |
| `ov_msckf/src/core/VioManager.cpp:256` | `track_image_and_update` |
| `ov_msckf/src/core/VioManager.cpp:323` | `do_feature_propagate_update` |
| `ov_msckf/src/state/{State,StateHelper,Propagator}.*` | 状态 / 协方差 / 预积分 |
| `ov_msckf/src/update/Updater*.*` | 4 个 updater |
| `ov_msckf/src/ros/ROS2Visualizer.*` | ROS2 发布 / 订阅 |
| `ov_init/src/init/InertialInitializer.h:60` | 初始化调度 |
| `ov_core/src/{track,cpi,feat}/*` | 前端 / 预积分 / 特征库 |

### 1.10 条件编译（ROS1 / ROS2）
`ov_msckf/CMakeLists.txt:39-53`：
```cmake
find_package(catkin QUIET COMPONENTS roscpp)
find_package(ament_cmake QUIET)
if (catkin_FOUND AND ENABLE_ROS)
    include(cmake/ROS1.cmake)       # -DROS_AVAILABLE=1
elseif (ament_cmake_FOUND AND ENABLE_ROS)
    include(cmake/ROS2.cmake)       # -DROS_AVAILABLE=2
else ()
    include(cmake/ROS1.cmake)       # -DROS_AVAILABLE=0
endif ()
```
源码通过 `#if ROS_AVAILABLE == 2` 切换到 `ROS2Visualizer.h` + `rclcpp`（`run_subscribe_msckf.cpp:28-34`）。

### 1.11 运行示例
```bash
# EuRoC（双目 + IMU），用预置 config
ros2 launch ov_msckf subscribe.launch.py \
     config:=euroc_mav use_stereo:=true verbosity:=INFO

# 自定义 YAML
ros2 launch ov_msckf subscribe.launch.py \
     config_path:=/path/to/my_estimator_config.yaml
```

### 1.12 优缺点
| ✅ | ❌ |
|----|----|
| 学术影响力大、文档完整 | 每帧 EKF 更新 CPU 偏高 |
| 原生 ROS2、多相机、初始化鲁棒 | 不支持 LiDAR / GPS / 轮速（那是 mins） |
| `ov_core` 公共库可跨项目复用 | 参数多，调优门槛 |

### 1.13 实战记录（TBD）
> 留待 benchmark 阶段填入 EuRoC ATE、Orbbec 实机 CPU / 延迟等数据。

---

## 2. mins

### 简介
同 Delaware RPNG（Woosik Lee, Guoquan Huang）。**Open-VINS 的多传感器扩展**，把 GPS / LiDAR / 轮速 / Vicon 作为独立 updater 集成进同一个 error-state MSCKF。

### 核心算法
- **框架**：MSCKF error-state EKF + 模块化 updater。
- **前端**：`ov_core` KLT。
- **Updater 列表**：
  - `UpdaterCamera` —— 同 open_vins
  - `UpdaterGPS` —— 伪距 / 载波约束或绝对位置
  - `UpdaterLidar` —— 点云与上一帧 ICP，使用 `ikd-tree` 加速
  - `UpdaterWheel` —— 里程计约束
  - `UpdaterVicon` —— 绝对位姿
- **典型组合**：cam+IMU、cam+IMU+GPS、cam+IMU+LiDAR、cam+IMU+wheel。

### 输入 / 输出
相机 + IMU + 任意一个或多个 {GPS, LiDAR, wheel, Vicon}；输出 `nav_msgs/Odometry` 及各 bias。

### 依赖
- Eigen3 / OpenCV / Boost / `ov_core`。
- `libpointmatcher`（ICP）、`libnabo`（近邻）、PCL。
- **catkin（ROS1 only）**。

### ROS2 集成
**不支持**，仅 ROS1（`package.xml` 是 catkin）。

### 优缺点
| ✅ | ❌ |
|----|----|
| 多传感器融合，工业导向 | 仅 ROS1 |
| LiDAR + 轮速可显著抑制漂移 | 依赖复杂，多 updater 并发 CPU 占用高 |
| | 文档相对少 |

### 代码入口
- `SystemManager`（`src/core/SystemManager.*`）
- `src/update/cam/UpdaterCamera.*`、`src/update/gps/UpdaterGPS.*`、`src/update/lidar/UpdaterLidar.*`、`src/update/wheel/UpdaterWheel.*`
- `src/state/State.*`、`src/state/Propagator.*`

---

## 3. sqrtVINS（深度分析）

> 以下章节以代码级颗粒度重写（2026-04-15）。

### 3.1 架构总览
Delaware RPNG 2025 新作（Yuxiang Peng, Patrick Geneva 等），**TRO 2025**。把 open_vins 从标准 EKF 改成 **Square-Root Filter (SRF)**：维护协方差 Cholesky 因子 **U**（上三角），`P = U·Uᵀ`，所有量测更新用 LLT / QR 代替稠密矩阵运算，保证数值稳定性与正定性。宣称可在 32 位嵌入式平台（Jetson Nano 5 W）<1 GHz CPU 上运行。

调用链（与 open_vins 相似，但关键步骤替换为 sqrt 版本）：

```
IMU callback  → VioManager::feed_measurement_imu → Propagator::feed_imu
image callback→ VioManager::track_image_and_update       VioManager.cpp:256
                ├─ 初始化：InertialInitializer::initialize
                └─ do_feature_propagate_update
                    ├─ Propagator::propagate_and_clone
                    │   └─ StateHelper::propagate(state, Φ, Q_sqrt)   StateHelper.cpp:202-234
                    ├─ UpdaterMSCKF::update
                    │   └─ state->store_update_factor(R_sqrt_inv_H_UT, R_inv_res)  State.h:150
                    ├─ StateHelper::update_llt(state)                 StateHelper.cpp:275-355  ← SRF 核心
                    └─ StateHelper::marginalize_*
```

### 3.2 与 open_vins 的分工
**共享**（`ov_core` 直接链入，`cmake/ROS1.cmake:4-21` / `ROS2.cmake:4-14`）：
- 特征前端：`ov_core::TrackKLT` / `TrackDescriptor`
- 特征初始化与数据库：`FeatureDatabase` / `FeatureInitializer`
- IMU 预积分：CPI（`CpiV1` / `CpiV2`）

**sqrtVINS 重写**：
- `State`（持 `U_` 矩阵与一批 `EigenMatrixBuffer` 工厂）
- `StateHelper`（全部改为 sqrt 传播、sqrt 更新、sqrt 边缘化）
- `Propagator`（给 CPI 输出增加 `Q_sqrt` 构造）
- `UpdaterMSCKF` / `UpdaterSLAM` / `UpdaterZeroVelocity`（改成往 `State` 推入预计算因子而非直接算 Kalman gain）

### 3.3 IMU 预积分（sqrt 版本）
沿用 `CpiV1`/`CpiV2`，但协方差传播切换到平方根形式：

```cpp
// StateHelper::propagate(state, Phi, Q_sqrt)   StateHelper.h:132-134
//   输入：状态转移 Φ（n×n），离散噪声 Q_sqrt = chol(Qd)
// 实现片段 StateHelper.cpp:202-234
U_new.block<15,15>(0, imu->id()) = Q_sqrt;        // 把过程噪声块填入新 U
// 再通过 (Phi · U) 组合出新因子，整体维持 U 的上三角结构
```
积分方式仍由 YAML `integration: rk4 | discrete | analytical` 选择；`Propagator::predict_mean_{discrete,rk4}()` 在 `Propagator.h:174/208`。

### 3.4 后端：SRF 核心
状态成员（`ov_srvins/src/state/State.h:259-268`）：
```cpp
MatX U_;                          // 上三角 sqrt 协方差
EigenMatrixBuffer R_sqrt_inv_H_UT_;   // 预计算 R⁻¹ᐟ² · H · Uᵀ
EigenMatrixBuffer HT_R_inv_res_;      // 预计算 Hᵀ · R⁻¹ · r
EigenMatrixBuffer factor_init_tri_, factor_init_dense_;  // 延迟 SLAM 初始化因子
VecX xk_minus_xk1_, xk_minus_x0_;     // 迭代更新增量
```

**LLT 更新算法**（`StateHelper.cpp:275-355`）步骤：
1. 收集所有 updater push 进来的 `R_sqrt_inv_H_UT`、`HT_R_inv_res`；`reverse_mat()` 做列翻转以利用块结构（L293）
2. 批量 rank 更新：`matrix_multiplier_ATA(R_sqrt_inv_H_UT, FT_F)`（L297）
3. 加单位矩阵：`FT_F.diagonal() += 1`
4. Cholesky：`Eigen::LLT<MatX> llt(FT_F.selfadjointView<Upper>());  MatX F = llt.matrixU();`（L299）
5. 通过两次 `reverse_mat` 把 F 变回上三角（L303-306）
6. 三角求解：`F.transpose().triangularView<Upper>().solveInPlace(...)`（L318-327）
7. 对所有 `variables_` 调 `variable->update(dx)`（L339-342）

**常用矩阵工具**（`ov_srvins/src/utils/Helper.h`）：
| 函数 | 行 | 用途 |
|------|----|------|
| `reverse_mat` | 117-123 | 行 / 列翻转，暴露块结构 |
| `triangular_matrix_inverse_solver` | 148-149 | 块化解 `U⁻ᵀ · X = B` |
| `triangular_matrix_multiplier_UU` | 158-160 | `U1 · U2` 上三角乘 |
| `triangular_matrix_multiplier_LLT` | 168-169 | `L · Lᵀ` |
| `matrix_multiplier_ATA` | 175 | 高效 `Aᵀ · A` |
| `efficient_QR` | 91,98,104 | Givens / Householder，利用稀疏 |

与 EKF 相比：
| | open_vins | sqrtVINS |
|---|-----------|----------|
| 协方差存储 | 稠密 **P** | 上三角 **U**（`P=U·Uᵀ`） |
| 更新 | 稠密 Kalman gain | 预计算因子 + LLT |
| 数值稳定 | 长时可能丢正定 | 严格对称正定 |
| 32-bit float | 不稳 | 稳 |
| 边缘化 | QR 块消元 | sqrt-preserving QR / Cholesky |

### 3.5 初始化（sqrt 感知）
入口：`ov_srvins/src/initializer/InertialInitializer.cpp:77`。

静态初始化假设零速 + 已知重力；动态初始化 100 ms 窗口内 SfM，使用 LM + `iterative_update_llt()`（`StateHelper.h:157`）。

**`StateHelper::initialize()`**（`StateHelper.cpp:417-470`）把新 SLAM 特征 3D 分量的 Jacobian `H = [H_f | H_x]` QR 分解，取前 3 行约束新特征，剩余行做 null-space 量测：
```cpp
reverse_mat(H_L);
reverse_mat(H_R);
efficient_QR(H_R, res, H_L);        // Givens QR
reverse_mat(H_R);
// 切块：Hx_init (3×n)、Hf_init (3×3 下三角)、H_update (m-3×n)
state->store_init_factor(new_var, tri_factor, dense_factor);   // State.h:145
// 之后在 update_llt 统一 integrate into U
```

**初始 sqrt 协方差**（`StateHelper.cpp:53-55`）—— 从 YAML 对角线直接赋给 `U_.diagonal()`：
```yaml
# estimator_config_srvins.yaml:57-69
init_prior_q:  0.1
init_prior_p:  0.0
init_prior_v:  1.0
init_prior_bg: 0.01
init_prior_ba: 0.05
```

### 3.6 状态管理（sqrt 边缘化）
`marginalize_old_clone()` / `marginalize_slam()` / `marginalize()`（`StateHelper.cpp:95-270`）：提取 U 的相关子块 → 块级 QR / Cholesky 消元 → 保持 U 仍是上三角并缩小维度。还有 `propagate_slam_anchor_feature()`（`StateHelper.h:243-246`）处理锚点克隆被移除时把 SLAM 特征重新锚到活着的克隆。

### 3.7 ROS2 节点 & 话题
`ov_srvins/src/ros/ROS2Visualizer.h:30-227`。发布：

| topic | 类型 |
|-------|------|
| `/ov_srvins/poseimu` | `PoseWithCovarianceStamped`（`P = U·Uᵀ`） |
| `/ov_srvins/pathimu` | `nav_msgs/Path` |
| `/ov_srvins/feats_msckf` | `PointCloud2`（MSCKF 特征 3D） |
| `/ov_srvins/feats_slam` | `PointCloud2`（SLAM landmark） |
| TF | `/odom → /imu` |

订阅回调：
```cpp
callback_inertial(Imu::SharedPtr);                         // /imu0
callback_monocular(Image::SharedPtr, size_t cam_id);       // /camera0/image_raw
callback_stereo(Image x2, size_t cam_id0, size_t cam_id1); // 立体
```
QoS 默认 best-effort、depth=1（面向嵌入式）。

### 3.8 关键 YAML 配置
`config/euroc_mav/estimator_config_srvins.yaml`：
```yaml
integration:           rk4
use_fej:               true
max_clones:            11
max_slam:              50
max_msckf_in_update:   40
feat_rep_msckf:        GLOBAL_3D
feat_rep_slam:         ANCHORED_MSCKF_INVERSE_DEPTH
init_window_time:      0.15
init_dyn_use:          false
init_dyn_num_pose:     5
init_max_disparity:    5.0

# 初始 sqrt 对角
init_prior_q:  0.1
init_prior_p:  0.0
init_prior_v:  1.0
init_prior_bg: 0.01
init_prior_ba: 0.05
```

### 3.9 代码入口速查
| 文件:行 | 内容 |
|---------|------|
| `ov_srvins/src/ros1_serial_msckf.cpp:49` | `main`（serial 模式） |
| `ov_srvins/src/core/VioManager.cpp:59` | 构造，装配 tracker / propagator / updater |
| `ov_srvins/src/state/State.h:259-268` | `U_` / 预计算因子缓冲 |
| `ov_srvins/src/state/StateHelper.cpp:275-355` | **`update_llt()` SRF 核心** |
| `ov_srvins/src/state/StateHelper.cpp:53-55` | `set_initial_imu_square_root_covariance` |
| `ov_srvins/src/state/StateHelper.cpp:417-470` | `initialize()` 新 SLAM 特征整合 |
| `ov_srvins/src/state/Propagator.cpp:91` | `propagate_and_clone` |
| `ov_srvins/src/initializer/InertialInitializer.cpp:77` | `initialize()` 调度 |
| `ov_srvins/src/ros/ROS2Visualizer.h:30` | ROS2 可视化 |
| `ov_srvins/src/utils/Helper.h` | sqrt 专用矩阵工具 |

### 3.10 构建 & 条件编译
`ov_srvins/CMakeLists.txt` 类似 open_vins 的自动检测：
```cmake
find_package(catkin QUIET COMPONENTS roscpp)
find_package(ament_cmake QUIET)
if (catkin_FOUND) include(cmake/ROS1.cmake)
elseif (ament_cmake_FOUND) include(cmake/ROS2.cmake)
else () include(cmake/ROS1.cmake) endif ()

option(USE_FLOAT "Use float version when built" ON)
if (USE_FLOAT)
    add_definitions(-DUSE_FLOAT=1)   # 启用 float32（嵌入式关键开关）
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer \
     -fno-signed-zeros -fno-math-errno -funroll-loops")
```
`USE_FLOAT=ON` 是嵌入式默认——配合 sqrt-form 才能稳定跑 float32。

### 3.11 运行示例
```bash
# EuRoC，ROS2，double
ros2 launch ov_srvins serial.launch.py config:=euroc_mav

# Jetson Nano / 32-bit 平台
colcon build --packages-select ov_srvins \
    --cmake-args -DUSE_FLOAT=ON -DCMAKE_BUILD_TYPE=Release
```

### 3.12 优缺点
| ✅ | ❌ |
|----|----|
| 严格对称正定，长时不退化 | 实现复杂度 / 调试门槛高 |
| float32 稳定 → 嵌入式可跑 | 文献较新（TRO 2025），生态小 |
| 前端沿用 `ov_core`，迁移成本低 | 内存占用与 EKF 类似 |
| ROS1/2 双端，`serial.launch` 兼容 open_vins | |

### 3.13 实战记录（TBD）
> 留待 benchmark 阶段填入（EuRoC ATE 与 open_vins diff、Jetson 上 float32 稳定性）。

---

## 4. EPLF-VINS

### 简介
哈工大工业机器人研究院（Lei Xu 等）。基于 VINS-Mono 与 PL-VINS 的改进，核心创新是**高效点线混合特征**——EDLines 提线 + 光流增量跟踪。

### 核心算法
- **框架**：Ceres 紧耦合滑窗优化 + 边缘化（与 VINS-Mono 同构）；`WINDOW_SIZE=10`；状态 = 位置 / 四元数 / 速度 / 加速度 bias / 陀螺 bias。
- **前端**：
  - 点：`feature_tracker`（光流）
  - 线：`linefeature_tracker` 使用 **EDLines**（需 OpenCV contrib）；关键是 incremental flow，避免每帧重提线段
- **后端因子**：
  - `ReprojectionFactor`（Huber 点重投影）
  - `LineProjectionFactor`（4D 线参数化）
  - `IMUFactor`（预积分） + 边缘化因子
- **回环**：DBoW2 + BRIEF。

### 输入 / 输出
**单目 + IMU**（论文标题 "Real-Time Monocular VI-SLAM"；`config/euroc/euroc_config.yaml` 仅一个 `image_topic`、一组内参，无 stereo 字段）。点、线、点+线三种模式可选；输出 TF / 位姿 / 特征点云可视化。与 VINS-Mono → PL-VINS → EPLF-VINS 同一脉络，都是纯单目。

### 依赖
- Ceres、Eigen3、**OpenCV 3.4+（必须带 contrib 以用 EDLines）**。
- `camera_model`、DBoW2。
- **catkin（ROS1）**。

### ROS2 集成
**无**，只有 `.launch` XML。

### 优缺点
| ✅ | ❌ |
|----|----|
| 点线融合，低纹理 / 结构化场景鲁棒 | 仅 ROS1 |
| EDLines incremental flow 性能好 | 依赖 OpenCV contrib |
| 自带 DBoW2 回环 | 两路前端 CPU 占用高 |

### 代码入口
- `Estimator`（`src/estimator.*`）
- `FeatureManager`（`src/feature_manager.*`）
- `linefeature_tracker/src/linefeature_tracker.cpp` 的 `LinefeatureTracker`
- `src/factor/line_parameterization.cpp`、`line_projection_factor.cpp`
- `lf_feature_tracker/src/feature_tracker.cpp`

---

## 5. VINGS-Mono

### 简介
复旦 MAGIC Lab，2025。**深度学习前端 + 因子图 VIO + Gaussian Splatting 建图**的混合 SLAM 系统。三种模式：纯 VO、VIO、NeRF-SLAM（DROID 前端 + GS 建图 + VIO）。

### 核心算法
- **前端（DL）**：DroidNet（光流 + 深度）、SuperPoint + LightGlue 做匹配、Lietorch 做局部 BA。
- **VIO（`frontend_vo`）**：GTSAM 因子图；IMU 预积分因子 + 视觉重投影。
- **建图**：Gaussian Splatting + 单目深度（Metric3D / ZoeDepth）。
- **回环**：LightGlue 特征匹配。
- **总体**：神经网络前端 → 因子图 → 3DGS 重建的三层架构。

### 输入 / 输出
单/立体相机 + IMU（VIO 模式必须），可选 GNSS / 轮速；支持 KITTI / KITTI-360 / Waymo / EuRoC / TUM / RGBD。输出轨迹、3DGS 参数（均值 / 协方差 / SH）、BEV 地图、深度图、点云。

### 依赖
- Python 3.9+ / PyTorch / CUDA 11.8+
- `lietorch`、`gtsam`（Python 绑定）、`droid_backends`、`gaussian-splatting`、OpenCV、Scipy
- 可选 COLMAP、FastSAM

### ROS2 集成
**无**。纯 Python 脚本 + YAML 配置，离线批处理，不订阅 ROS topic。

### 优缺点
| ✅ | ❌ |
|----|----|
| 最新 3D 表示（GS） | 只能离线 |
| DL 前端鲁棒 | Python 延迟高，GPU / 显存需求大 |
| VIO + 视觉双融合 | 初始化复杂（需足量运动） |

### 代码入口
- `scripts/run.py`
- `scripts/frontend/dbaf.py`（`DBAFusion`）、`dbaf_frontend.py`
- `scripts/frontend_vo/vio_slam.py`、`vo_factor_graph/factor_graph.py`
- `scripts/gaussian/gaussian_model.py`
- `scripts/datasets/*`

---

## 6. ORB_SLAM3_ROS2（深度分析）

> 2026-04-15 重写：聚焦 **上游 `/opt/fusion/v2.0/orb-slam3/`（亦可参考 `/home/steve/Documents/GitHub/slam/ORB_SLAM3/`）** 核心算法；ROS2 壳压缩到一小节。

### 6.1 三线程架构
萨拉戈萨大学，支持单/双/RGB-D × 纯视觉/惯性 共 **6 种模态**，`System` 构造时启动 3 个线程，共享 `Atlas` 做数据交换：

```
┌──────────────────────────────────────────────────────┐
│ System / Atlas / ORBVocabulary + KeyFrameDatabase    │
│   └─ Map (mspKeyFrames, mspMapPoints)                │
└──────────────────────────────────────────────────────┘
       ▲              ▲              ▲
       │              │              │
  Tracking      LocalMapping     LoopClosing
  (主线程)       (后台)            (后台)
  Track()        Run()             Run()
```
状态机：`NOT_INITIALIZED → OK → RECENTLY_LOST/LOST → Relocalization`。互斥由 `Map::mMutexMapUpdate` 保护（Tracking 持有它防止建图覆盖）。

### 6.2 ORB 特征（`ORBextractor.h/cc`）
主入口 `operator()`（L1086-1168）：
```cpp
// 1. 图像金字塔（8 层，scaleFactor=1.2，EDGE_THRESHOLD=31）
ComputePyramid(image);
// 2. 八叉树分裂 FAST（iniThFAST=20，minThFAST=7 为降级阈值）
ComputeKeyPointsOctTree(allKeypoints);
// 3. 高斯模糊 + BRIEF 256-bit 描述子，预定义 pattern[]
computeDescriptors(workingMat, keypoints, desc, pattern);
// 4. 立体/单目分流（vLappingArea 定义重叠区）
if (kp.pt.x >= vLappingArea[0])
    _keypoints.at(stereoIndex--) = kp;
else
    _keypoints.at(monoIndex++) = kp;
```

匹配器 `ORBmatcher`（`ORBmatcher.h:45-88`）提供：
- `SearchByProjection(F, MPs)` —— 局部地图追踪
- `SearchByProjection(F, lastF, th, mono)` —— 运动模型匹配
- `SearchByProjection(F, KF, ...)` —— 重定位
- `SearchByProjection(KF, Sim3, ...)` —— 回环 / 融合投影
- `SearchByBoW(...)` —— 词袋节点内加速匹配（重定位、回环初筛）
- `SearchForTriangulation(...)` —— 新 MapPoint 生成（极线约束）
- `Fuse(...)` —— 地图点融合

距离：`DescriptorDistance` 二进制 Hamming。

### 6.3 前端跟踪 `Tracking::Track()`（`Tracking.cc:1794`）
```
OK 分支（按优先级）:
  ├─ PredictStateIMU()          # 惯性系统且已初始化
  ├─ TrackWithMotionModel()      # mVelocity 有效
  │    ORBmatcher(0.9).SearchByProjection(curF, lastF, 15.0, bMono)
  ├─ TrackReferenceKeyFrame()    # 退化到参考帧
  │    ORBmatcher(0.7).SearchByBoW(refKF, curF, matches)
  └─ Relocalization()            # LOST 时
       BoW 检索 → PnP RANSAC → PoseOptimization()

接着：
  PreintegrateIMU()              # IMU 约束累积
  TrackLocalMap()                # 共视图扩展、投影匹配
  Optimizer::PoseOptimization()  # 单帧 SE3 精化
  NeedNewKeyFrame() → CreateNewKeyFrame()
```

`PoseOptimization`（`Optimizer.cc:814`）：g2o 图 = 1 个 `VertexSE3Expmap` + 若干固定 MapPoint 顶点，边有
`EdgeSE3ProjectXYZOnlyPose`（单目）/ `EdgeStereoSE3ProjectXYZOnlyPose`（立体）/ `EdgeSE3ProjectXYZOnlyPoseToBody`（鱼眼）。Huber 核 δ=√5.991（单）或 √7.815（立）。

`NeedNewKeyFrame()`（`Tracking.cc:3064`）判据：IMU 未初始化时 ≥ 0.25s 强插；否则看 `nRefMatches < nKFs*thRefRatio`（单目 0.9，其他 0.75）或 `近点追丢 > 70`。

### 6.4 局部建图 `LocalMapping::Run()`（`LocalMapping.cc:64`）
```cpp
while (1) {
    SetAcceptKeyFrames(false);
    if (CheckNewKeyFrames() && !mbBadImu) {
        ProcessNewKeyFrame();      // BoW 向量化 + 加入 KeyFrameDatabase + 更新共视图
        MapPointCulling();         // < 25% visibility 剔除
        CreateNewMapPoints();      // 邻近 KF 未配特征三角化（极线约束、前向检查）
        if (!CheckNewKeyFrames()) {
            SearchInNeighbors();   // 邻域 MP 融合
            if (mbInertial && isImuInitialized())
                Optimizer::LocalInertialBA(mpCurrentKF, ...);
            else
                Optimizer::LocalBundleAdjustment(mpCurrentKF, ...);
            InitializeIMU(...);    // VIBA 三阶段首次
            KeyFrameCulling();     // 冗余 KF（>90% MP 被他 KF 观测）剔除
        }
    }
    SetAcceptKeyFrames(true);
}
```

### 6.5 g2o 优化器（`Optimizer.h:46-102`）
| 函数 | 优化对象 | 调用处 |
|------|----------|--------|
| `PoseOptimization` | Frame 位姿 | Tracking |
| `LocalBundleAdjustment`（L1116） | 局部 KF + MP | LocalMapping |
| `LocalInertialBA` | 局部 + IMU | LocalMapping（惯性） |
| `GlobalBundleAdjustemnt` | 全局 KF + MP | LoopClosing |
| `FullInertialBA` | 全局 + IMU | 首次 IMU init / 回环 |
| `OptimizeSim3`（L2115） | Sim3 7-DoF | LoopClosing::DetectAndReffineSim3 |
| `OptimizeEssentialGraph` | 位姿图 | LoopClosing::CorrectLoop |

`LocalBundleAdjustment` 骨架：
```cpp
list<KeyFrame*>  lLocalKFs, lFixedKFs;   // 局部 KF BFS 共视图；fixed = 观测到局部 MP 但自身不在局部集的 KF
list<MapPoint*>  lLocalMPs;

g2o::BlockSolver_6_3 solver;              // 6: SE3, 3: 3D 点
g2o::OptimizationAlgorithmLevenberg algo;

for (auto kf : lLocalKFs) addVertex(new VertexSE3Expmap(kf->GetPose()));
for (auto kf : lFixedKFs) vVertex->setFixed(true);
for (auto mp : lLocalMPs) addVertex(new VertexSBAPointXYZ(mp->GetWorldPos()));
for (auto obs : observations) addEdge(new EdgeSE3ProjectXYZ(...));
optimizer.optimize(5);   // 5 次迭代
```

### 6.6 IMU 预积分（`ImuTypes.h:143-251`）
`Preintegrated` 持 9×9 协方差、相对 `dR / dV / dP`、对 gyro/accel bias 的 5 个 Jacobian（`JRg, JVg, JVa, JPg, JPa`）、`Info = C⁻¹`。

`IntegrateNewMeasurement(a, w, dt)`：
```cpp
dR *= Sophus::SO3f::exp(w * dt);
dV += dR * a * dt;
dP += dV * dt + 0.5 * dR * a * dt * dt;
// Jacobian 右乘链式更新；协方差 C = F·C·Fᵀ + G·Σ·Gᵀ
```

**VIBA 三阶段**（`LocalMapping.cc:130-230`）：
1. **BA1 (0–5 s)**：估计 Rwg / 尺度 s / accel bias ba
2. **BA2 (5–50 s)**：Rwg 锁定，优化 (T, V, bg, ba)
3. **BA3**：`FullInertialBA` 地图融合后全局

g2o 中 IMU 约束 = `EdgeInertial`（顶点 KFᵢ / KFᵢ₊₁ 各自的 `(T,V,Bias)`，测量是预积分的 dR/dV/dP）。

### 6.7 回环 & Atlas 融合（`LoopClosing::Run()`，L90）
```
NewDetectCommonRegions()
  ├─ BoW 检索：mpKeyFrameDB->DetectLoopCandidates(curKF, minScore=0.015)
  └─ Sim(3) 验证（RANSAC）
       Sim3Solver.find()：每轮随机 3 对 3D 点 → Kabsch SVD 求 R + 缩放 → CheckInliers
       若 nInliers > 阈值：mg2oLoopSlw = 估计出的 Sim3

若 loop: CorrectLoop()
  ├─ 用 Sim3 修正当前 KF 邻域的 Tcw
  ├─ OptimizeEssentialGraph(map, loopKF, curKF, ...)
  └─ 异步 RunGlobalBundleAdjustment()

若 merge: MergeLocal() / MergeLocal2()
  ├─ 跨地图 MP 投影 + 融合
  ├─ 共视关系连接
  └─ 非惯性 → LocalBA；惯性 → MergeInertialBA（保重力方向）
```

`Sim3Solver` 的 Kabsch（`Sim3Solver.cc`）：
```cpp
H = P1 * P2.transpose();
SVD(H) → R = U * Vᵀ;
s = 两点集方差比（单目才需要，fixScale=false）;
```

### 6.8 相机模型（抽象 + 2 实现）
```cpp
class GeometricCamera {                     // CameraModels/GeometricCamera.h
    virtual cv::Point2f project(cv::Point3f) = 0;
    virtual cv::Point3f unproject(cv::Point2f) = 0;
    virtual Eigen::Matrix<double,2,3> projectJac(Eigen::Vector3d) = 0;
    virtual bool ReconstructWithTwoViews(...) = 0;
    virtual bool epipolarConstrain(GeometricCamera*, ...) = 0;
    // CAM_PINHOLE = 0; CAM_FISHEYE = 1
};
```
- `Pinhole` —— 4 参 `{fx,fy,cx,cy}`，标准针孔 + radtan 畸变
- `KannalaBrandt8` —— 8 参 `{fx,fy,cx,cy,k1..k4}`，鱼眼 4 阶径向多项式

### 6.9 词袋（DBoW2）
两个全局容器：
- `ORBVocabulary` —— 从 `Vocabulary/ORBvoc.txt`（~139 MB，100 万词）加载
- `KeyFrameDatabase` —— 倒排索引，`DetectLoopCandidates(pKF, minScore)` O(1) 均摊检索

`ProcessNewKeyFrame` 中调：
```cpp
mpORBVocabulary->transform(pKF->mDescriptors, mBowVec, mFeatVec);
pKF->SetBowVector(mBowVec); pKF->SetFeatureVector(mFeatVec);
mpKeyFrameDB->add(pKF);
```

### 6.10 代码入口速查
| 模块 | 文件:行 |
|------|---------|
| 系统 | `System.h:83-265` / `System.cc` |
| 追踪 | `Tracking.cc:1794`（Track）/ `3064`（NeedNewKeyFrame） |
| 特征 | `ORBextractor.cc:1086`（operator()）/ `1170`（ComputePyramid） |
| 匹配 | `ORBmatcher.cc:43, 427`（SearchByProjection） |
| 建图 | `LocalMapping.cc:64`（Run）/ `1173`（InitializeIMU） |
| 优化 | `Optimizer.cc:814`（PoseOpt）/ `1116`（LocalBA）/ `2115`（OptimizeSim3） |
| 回环 | `LoopClosing.cc:90`（Run） |
| IMU | `ImuTypes.h:143-251`（Preintegrated） |
| Sim3 | `Sim3Solver.h:33` / `Sim3Solver.cc` |
| 地图 | `Map.h:41` / `KeyFrame.h:52` / `MapPoint.h:44` / `Atlas.h:49` |
| 相机 | `GeometricCamera.h:43` / `Pinhole.h:29` / `KannalaBrandt8.h` |

### 6.11 ROS2 壳（`src/ORB_SLAM3_ROS2/`）
5 个可执行节点（`monocular / monocular-inertial / rgbd / stereo / stereo-inertial`）各一个 cpp。RGB-D 回调长这样：
```cpp
void RgbdSlamNode::GrabRGBD(ImageMsg::SharedPtr msgRGB, ImageMsg::SharedPtr msgD) {
    m_SLAM->TrackRGBD(cv_bridge::toCvShare(msgRGB)->image,
                      cv_bridge::toCvShare(msgD)->image,
                      Utility::StampToSec(msgRGB->header.stamp));
}
```
Stereo-inertial 用 `SyncWithImu()` 在独立线程汇聚 IMU 缓冲到当前帧。运行：
```bash
ros2 run orbslam3_ros2 rgbd $ORB_VOC ~/vslam_ws/src/ORB_SLAM3_ROS2/config/rgb-d/TUM1.yaml \
    --ros-args -r camera/rgb:=camera/rgb/image_color -r camera/depth:=camera/depth/image

ros2 launch orbslam3_ros2 stereo-inertial.launch.py \
    vocabulary:=$ORB_VOC \
    config:=~/vslam_ws/src/ORB_SLAM3_ROS2/config/stereo-inertial/GeoScan_fisheye.yaml
```

关键 YAML 字段（`Camera0.fx/fy/cx/cy`、`Camera.bf`、`Camera.fps`、`Camera.RGB`、`Stereo.ThDepth`、`IMU.T_b_c`、`System.LoadAtlasFromFile/SaveAtlasToFile`）与 IMU 噪声（`IMU.NoiseGyro=1.7e-4`、`IMU.NoiseAcc=2.0e-3`、`IMU.GyroWalk=1.9393e-5`、`IMU.AccWalk=3.0e-3`、`IMU.Frequency=200`）都走标准 EuRoC 格式。

### 6.12 优缺点
| ✅ | ❌ |
|----|----|
| 6 模态最完整；Atlas 多地图可跨会话 | Pangolin 在 headless 环境会崩 |
| 3 线程 + 3 阶段 VIBA，鲁棒性好 | ORB 对光照变化敏感 |
| 回环 + 重定位 + 全局 BA 完整 | ROS2 壳用的 vocab 加载慢（~2 s） |
| DBoW2 O(1) 检索 | 参数多，鱼眼场景调参门槛高 |

### 6.13 实战记录（GeoScan B1 双目鱼眼）
**数据集**：B1 楼内手持双目鱼眼 + IMU（Kalibr 标定），绕一圈 7 分钟，10 fps。

**踩坑与修复**：

| 问题 | 现象 | 根因 | 修复 |
|------|------|------|------|
| `libpangolin.so` 缺 | mono-inertial exit 127 | rpath 指向 pyslam 的 Pangolin | Pangolin 源码 `make install` + `ldconfig` |
| `std::bad_array_new_length` SIGABRT | 启动立刻崩 | FastDDS 与 librealsense 符号污染 | CMakeLists 删 librealsense2 链接（L133-141） |
| YAML 老 parser 读不全 | 内参 = 0 → 崩 | 缺 `File.version` 字段 | 升级：加 `File.version: "1.0"`，字段改 `Camera1.*` |
| IMU accel 单位错 | mono-inertial 反复 reset | bag 里 accel ≈ 1.0（g 单位）被当成 m/s² | GrabImu 加 40 样本自动检测：mean ‖a‖ 7–12 为 m/s²，0.7–1.2 为 g |
| 节点无 topic 发布 | rviz 一片空 | MonocularInertialNode 只 sub 不 pub | 加 `~/odom` / `~/path` / `~/map_points` + rviz 配置 |
| mono-inertial 反复 LOST | "Less than 15 matches" 循环 | 10 fps 鱼眼 + 室内转弯，constant-velocity 预测器跟不上 | 结论：10 fps 不够；生产上换 stereo-inertial |

**stereo-inertial 跑通要点**：
- 必须跳开头静止段（`--start-offset 10`），否则 IMU init "Not enough motion"
- `Stereo.ThDepth: 100.0`（默认 40 太严：97 mm 基线 × 40 = 3.88 m，室内多数特征被丢）
- `nFeatures: 2500`、`iniThFAST: 10 / minThFAST: 3`
- 双目 init 仅 30–50 点（健康值 200+），疑时间同步或标定精度问题
- 运行中**无法 save Atlas**（`System::SaveAtlas` 是 private），需暴露成 ROS service

---

## 7. VINS-Fusion-ROS2（深度分析）

> 迁自 `src/vslam-summary.md`（2026-04-15）。

### 7.1 架构总览
HKUST 沈劭劼组（Tong Qin、Shaozu Cao 等）。**光流前端 + 紧耦合 IMU 滑窗优化**，支持单 / 双目 + IMU，可选 GPS 全局融合。与 ORB_SLAM3 的区别：不用描述子、不用 KF / 地图点联合 BA，而是 Ceres 滑动窗口。

**特点**：
- 金字塔 Lucas-Kanade 光流（无描述子）
- 紧耦合 IMU 预积分（中值积分）
- 滑动窗口 Ceres NLS（同时优化 11 帧）
- DBoW2 + BRIEF 回环（需 `loop_fusion` 模块）
- 4-DOF / 6-DOF 位姿图优化
- GPS 全局融合（`global_fusion` 模块）

**模块**：
```
vins/                 # VIO 估计器
├── src/estimator/
├── src/featureTracker/
├── src/factor/
├── src/initial/
├── src/rosNodeTest.cpp
└── src/utility/
loop_fusion/          # 回环 + 位姿图
global_fusion/        # GPS 全局融合
camera_models/        # 相机模型（针孔 / 鱼眼 / …）
```

### 7.2 状态向量与数据流
```cpp
// parameters.h
enum StateOrder {
    O_P = 0, O_R = 3, O_V = 6, O_BA = 9, O_BG = 12
};
// SIZE_POSE=7（位置 3 + 四元数 4），SIZE_SPEEDBIAS=9
// 特征：SIZE_FEATURE=1（逆深度）
// 滑窗：WINDOW_SIZE=10 → 同时优化 11 帧（0..10）
```

主干：
```
IMU callback  → subscribe /imu0
image callback→ subscribe /cam0/image_raw (+ /cam1)
     ↓
featureTracker::trackImage()                   feature_tracker.cpp
     ├─ LK 光流 + 反向验证 + F 矩阵 RANSAC
     ├─ goodFeaturesToTrack 补特征
     └─ 去畸变 + 像素速度
     ↓
estimator::inputImage()                        estimator.cpp:48
     ↓
estimator::processImage()                      :200
     ├─ 初始化检查 → initialAlign()
     ├─ slideWindow()
     ├─ optimization()  (Ceres)                :300
     ├─ 边缘化（MARGIN_OLD / MARGIN_SECOND_NEW）
     └─ 发布 odom + tf
     ↓
loop_fusion::addKeyFrame()                     pose_graph.cpp:150
     ├─ 提取 BRIEF
     ├─ DBoW2 detectLoop()
     ├─ findConnection() (F + PnP RANSAC)
     └─ 4/6-DOF 位姿图优化
```

### 7.3 光流前端
```cpp
// feature_tracker.cpp :80
// 1. LK 光流 (21x21, 3 层金字塔)
cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, ...);

// 2. 反向验证
if (FLOW_BACK) {
    cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, ...);
    // 双向距离 > 0.5 px 则丢弃
}

// 3. 基本矩阵 RANSAC 外点剔除
rejectWithF();

// 4. 空间分布掩码 + 补充新特征
setMask();
cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(),
                        0.01, MIN_DIST, mask);

// 5. camodocal 反投影到归一化平面
undistortedPoints();

// 6. 像素速度（用于在线 td 标定）
ptsVelocity();
```

**参数（`euroc_stereo_imu_config.yaml`）**：
```yaml
max_cnt: 150          # 最大特征数
min_dist: 30          # 特征最小像素间距
freq:    10           # 发布频率
F_threshold: 1.0      # F RANSAC 阈值
flow_back:   1        # 反向光流开
camera_model: "pinhole"   # 或 "equidistant"
```

### 7.4 IMU 预积分
```cpp
// estimator.cpp
void Estimator::processIMU(double t, double dt,
                           const Vector3d &accel, const Vector3d &gyro) {
    // 中值积分
    Vector3d un_gyr = 0.5*(gyr_0 + gyro) - Bgs[frame_count];
    Rs[frame_count] *= Utility::deltaQ(un_gyr*dt).toRotationMatrix();
    Vector3d un_acc_0 = Rs[frame_count]*(acc_0 - Bas[frame_count]) - g;
    Vector3d un_acc_1 = Rs[frame_count]*(accel - Bas[frame_count]) - g;
    Vector3d un_acc = 0.5*(un_acc_0 + un_acc_1);
    Ps[frame_count] += dt*Vs[frame_count] + 0.5*dt*dt*un_acc;
    Vs[frame_count] += dt*un_acc;

    pre_integrations[frame_count]->push_back(dt, accel, gyro);
}

// fastPredictIMU：在 10 Hz 优化帧之间用高频 IMU 传播，输出 200 Hz odom
```

### 7.5 重投影因子（单目两帧）
```cpp
// projectionTwoFrameOneCamFactor.cpp
// 参数：
//   [0] pose_i(7)  [1] pose_j(7)  [2] ex_pose(7)
//   [3] inv_depth(1)  [4] td(1)
// 步骤：
//   pts_i_td = pts_i - (td - td_i) * velocity_i
//   pts_camera_i = pts_i_td / inv_dep_i
//   camera_i → IMU_i → world → IMU_j → camera_j
//   residual = K * pts_camera_j / z_j - pts_j
//   sqrt_info = FOCAL_LENGTH / 1.5 * I₂
```

### 7.6 滑动窗口优化
```
优化变量 = [位姿(7)×11] + [速度偏置(9)×11] + [相机外参(7)×2]
         + [逆深度(1)×N] + [时间偏移(1)]
维度 ≈ 191 + N

因子：
  1. 边缘化先验
  2. IMU 预积分（相邻 KF）
  3. 重投影：
     ProjectionTwoFrameOneCamFactor
     ProjectionTwoFrameTwoCamFactor
     ProjectionOneFrameTwoCamFactor
```

**求解器配置**：
```yaml
max_solver_time:    0.04   # 40 ms 预算
max_num_iterations: 8
keyframe_parallax:  10.0   # KF 插入视差（像素）
```
实测单帧 5–15 ms。

**边缘化**（`slideWindow`）：`MARGIN_OLD`（正常，丢最老帧）/ `MARGIN_SECOND_NEW`（视差不足时丢次新）。

### 7.7 回环检测（`loop_fusion`）
```cpp
// pose_graph.cpp :150
void PoseGraph::addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop) {
    // 1. 提 BRIEF（独立于前端光流）
    // 2. DBoW2 查候选
    int loop_index = detectLoop(cur_kf, cur_kf->index);
    // 3. 几何验证
    if (loop_index != -1) {
        KeyFrame* old_kf = getKeyFrame(loop_index);
        if (cur_kf->findConnection(old_kf)) {
            if (use_imu_) optimize4DoF();   // yaw + 平移
            else          optimize6DoF();   // 全自由度
        }
    }
    db.add(cur_kf->brief_descriptors);
}

// keyframe.cpp — findConnection()
// 1. BRIEF KNN Hamming 匹配
// 2. F 矩阵 RANSAC
// 3. PnP RANSAC（3D-2D）
// MIN_LOOP_NUM = 25
```

词典：`support_files/brief_k10L6.bin`（~10 MB）。

### 7.8 初始化
```cpp
// initial_sfm.cpp
// 1. 在滑窗选视差最大帧对（帧 0 与帧 k）
// 2. 五点法 E 矩阵恢复相对位姿
// 3. 三角化共视特征
// 4. PnP + 三角化扩到所有窗口帧
// 5. 全局 BA 精化（相机位姿 + 特征）

// initial_alignment.cpp — visualInitialAlign()
// 1. SFM ↔ IMU 预积分对齐
// 2. 陀螺偏置（SFM 旋转 vs IMU 旋转）
// 3. 尺度 + 重力方向
// 4. 切线空间流形优化精化重力
// 5. 由重力导出全局速度
```

### 7.9 地图策略
**无全局地图**。滑动窗口维持 10 帧，边缘化时 Schur 补保留信息。特征逆深度随帧消亡。全局轨迹由 `loop_fusion` 位姿图维护，回环后修正。

### 7.10 话题
| 参数 | 类型 | YAML 示例 |
|------|------|-----------|
| `IMAGE0_TOPIC` | `sensor_msgs/Image` | `/cam0/image_raw` |
| `IMAGE1_TOPIC` | `sensor_msgs/Image` | `/cam1/image_raw` |
| `IMU_TOPIC` | `sensor_msgs/Imu` | `/imu0` |

发布：
- `/vins_estimator/odometry`（`nav_msgs/Odometry`）
- `/vins_estimator/path`（`nav_msgs/Path`）
- `/vins_estimator/camera_pose`（`PoseStamped`）
- `/vins_estimator/point_cloud`（`PointCloud2`）
- `/pose_graph_visualization/marker_array`（`loop_fusion` 节点）

### 7.11 关键 YAML 参数
```yaml
%YAML:1.0
IMAGE0_TOPIC: "/cam0/image_raw"
IMAGE1_TOPIC: "/cam1/image_raw"
IMU_TOPIC:    "/imu0"
OUTPUT_PATH:  "/home/steve/output"

Camera0.fx/fy/cx/cy: 458.654 / 457.296 / 367.215 / 248.375
Camera0.k1/k2/p1/p2: -0.2834 / 0.0740 / 0.00200 / 0.00110

imu_acc_scale:   9.81          # 单位转换（VINS 这里 yaml 写死）
estimate_td:     1             # 在线 td 优化
td:              -0.0146       # 初值（Kalibr）
estimate_extrinsic: 1          # 在线微调外参

max_cnt:   150
min_dist:  30
freq:      10
flow_back: 1

max_solver_time:    0.04
max_num_iterations: 8
keyframe_parallax:  10.0

init_window_time:    2.0
init_imu_thresh:     1.5
init_max_disparity: 10.0
```

### 7.12 代码入口速查
| 文件:行 | 内容 |
|---------|------|
| `vins/src/rosNodeTest.cpp:50` | 节点 main |
| `vins/src/estimator/estimator.cpp:1` | 估计器类 |
| `vins/src/estimator/estimator.cpp:48` | `inputImage()` |
| `vins/src/estimator/estimator.cpp:200` | `processImage()` |
| `vins/src/estimator/estimator.cpp:300` | `optimization()` |
| `vins/src/featureTracker/feature_tracker.cpp:80` | `trackImage()` |
| `vins/src/initial/initial_sfm.h` | SFM 初始化 |
| `vins/src/initial/initial_alignment.cpp` | VI 对齐 |
| `loop_fusion/src/pose_graph.cpp:150` | 回环 |
| `camera_models/` | 相机模型库 |

### 7.13 运行示例
```bash
# 终端 1：VIO
ros2 run vins vins_node \
  ~/vslam_ws/src/VINS-Fusion-ROS2/config/euroc/euroc_stereo_imu_config.yaml

# 终端 2：回环
ros2 run loop_fusion loop_fusion_node \
  ~/vslam_ws/src/VINS-Fusion-ROS2/config/euroc/euroc_stereo_imu_config.yaml

# 终端 3：放 bag（跳静止段）
ros2 bag play /home/steve/Documents/Datasets/vl/v101 \
  --clock --rate 0.5 --topics /imu0 /cam0/image_raw /cam1/image_raw

# 终端 4：RViz
ros2 launch vins vins_rviz.launch.xml
```

### 7.14 优缺点
| ✅ | ❌ |
|----|----|
| 光流快（~5 ms/帧），无需描述子 | 仅纯视觉 SFM 初始化，复杂 |
| 紧耦合 VI，输出 200 Hz | `vins_node` 本身无回环，需 `loop_fusion` |
| 在线 td / 外参标定 | 无重定位（光流前端没有全局描述子） |
| 内存占用小（滑窗） | 不支持 RGB-D |
| 支持 GPS 全局融合 | Ceres / cv_bridge 编译冲突多 |

### 7.15 实战记录（GeoScan B1 双目鱼眼）
**数据集**：GeoScan B1 双目鱼眼 + FB100 高频 IMU（Kalibr）。

**踩坑与修复**：

| 问题 | 现象 | 根因 | 修复 |
|------|------|------|------|
| `Ceres not found` | `camera_models` colcon build 失败 | cmake 找不到 Ceres | `sudo apt install libceres-dev` |
| Ceres 版本冲突 | Eigen 版本不匹配 | `/home/steve/lib` 的 ceres 用 Eigen 3.3.7 | `--cmake-args -DCeres_DIR=/usr/lib/cmake/Ceres` |
| `'CUDA' is not member of 'ceres'` | `estimator.cpp:1171` 编译错 | Ceres 2.1+ 才有 CUDA 后端，系统是 2.0 | 加 `#if defined(CERES_VERSION_MAJOR) && (... >= 2.1)` 守卫 |
| `glog SIGSEGV in dl_init` | `vins_node` 启动即段错误 | libglog.so 同时链 v0.4 和 v0.6 | 卸载 `/usr/local` 源码 glog，只留系统 apt 版 |
| `argc != 2` 直接退出 | 启动 1 秒就死 | 严格判等，但 ROS2 会注入 `--ros-args` | 改成 `argc < 2` |
| `loop_fusion` 找不到 vocab | `support_files/brief_k10L6.bin` 缺 | CMakeLists 没 install | `ln -sf src/.../support_files install/loop_fusion/share/` |

**GeoScan 实测配置**：
```yaml
max_cnt:   300        # 鱼眼边缘损失多，200→300
min_dist:  15         # 密度调高，25→15
equalize:  1          # CLAHE，对弱纹理友好
keyframe_parallax: 8  # 10 fps 放宽

imu_acc_scale:      9.81
estimate_td:        1
td:                 -0.0146
estimate_extrinsic: 1
```

**与 ORB_SLAM3 在此数据集的对比**：
| 维度 | ORB-SLAM3 stereo-inertial | VINS-Fusion stereo |
|------|----------------------------|---------------------|
| 初始化时间 | ~5 s（跳静止段） | ~14 s |
| 初始地图点数 | 30–50（偏低） | `n_pts` 5–25（经常丢失） |
| 鲁棒性 | VIBA 后 OK，偶跟丢 | 频繁 throw img，边缘易丢 |
| 漂移控制 | 内置回环 + 全局 BA | `vins_node` 单独无回环，需 `loop_fusion` |
| 一圈闭合 | 基本闭合 | 加 `loop_fusion` 后接近 ORB |

> 单独比 `vins_node` 与 ORB_SLAM3 并不公平——前者是 VIO 滑窗，不含回环。

---

## 8. AirSLAM（深度分析）

> 2026-04-15 新增。源码 `/home/steve/vslam_ws/src/AirSLAM/`，独立 C++ 项目（不是 ROS wrapper）。

### 8.1 定位
Kuan Xu（NTU CARTIN）+ Chen Wang（Buffalo SAIR Lab），**IEEE TRO 2025**（前身 AirVO，IROS 2023）。**双模**（V-SLAM / VI-SLAM）的**点线紧耦合**视觉 SLAM，核心卖点是**光照鲁棒**——用**学习特征**（SuperPoint + PLNet）替代 ORB，配合 **LightGlue** 做匹配，暗光 / 变照明场景稳定。

### 8.2 架构
三线程 + g2o 后端（`include/map_builder.h:64` 为入口类）：

```
ExtractFeatureThread (src/map_builder.cc)     ← 图像队列
  └─ SuperPoint(点) + PLNet(线+点) + 立体匹配 → 特征队列
TrackingThread (src/map_builder.cc:157)        ← 特征队列
  ├─ 可选 IMU 预测初值
  ├─ PnP / g2o FramePoseOptimization (L285)
  └─ 关键帧判决 + 局部 BA                    → 位姿 / 地图
后端 (g2o_optimization/g2o_optimization.cc)
  ├─ LocalmapOptimization (L79)   3–5 KF 滑窗
  ├─ IMUInitialization (L30)      VI 模式三阶段
  └─ GlobalBA (L50)                回环后全局
MapRefiner (include/map_refiner.h:52)
  └─ LoopDetection (DBoW2)
MapUser
  └─ 重定位：加载离线地图 + 查询帧 PnP
```

可执行程序 3 个：`visual_odometry` / `map_refinement` / `relocalization`。

### 8.3 点特征：SuperPoint + TensorRT
```cpp
// include/super_point.h:26
bool infer(const cv::Mat &image,
           Eigen::Matrix<float, 259, Eigen::Dynamic> &features);
// 259 维 = 256 描述子(L2 归一化) + (y, x, confidence)
```
- 最多 400 个关键点；边界 4 px 剔除
- TensorRT INT8 量化 → 0.8 ms/帧 @ 512×512
- **光照鲁棒**的来源：CNN 在多光照数据上训练 + L2 归一化消光度尺度

### 8.4 线特征：PLNet（统一网络）
```cpp
// include/plnet.h:23-24
bool infer(const cv::Mat &image,
           Eigen::Matrix<float,259,Dynamic> &features,
           std::vector<Eigen::Vector4d> &lines,
           Eigen::Matrix<float,259,Dynamic> &junctions,
           bool junction_detection = false);
```
两阶段 ONNX（Stage0 特征 + Stage1 线段），共 ~4 ms。相比 EPLF-VINS 的 EDLines + 光流，AirSLAM 是**纯学习型**，且点线可共用同一 CNN 主干。

点线关系（`include/line_processor.h:24-28`）：
```cpp
void AssignPointsToLines(lines, points, relation);   // 点到最近线的距离映射
// 在 BA 中把 "点落在线上" 作为额外约束
```

### 8.5 匹配：LightGlue / SuperGlue
`src/point_matcher.cc` + `NormalizeKeypoints`（L39-48）。默认 **LightGlue**（O(N log N) 注意力）：
- 关键帧处理 73 Hz（PC）/ 40 Hz（Jetson）
- 立体对应 <1 ms
- 配合 F-矩阵 RANSAC 剔除外点

切换参数：`configs/.../vo_euroc.yaml` → `point_matcher.matcher: 0`（LightGlue）/ `1`（SuperGlue）。

### 8.6 后端 BA（g2o 多约束联合）
`src/g2o_optimization/g2o_optimization.cc:79` `LocalmapOptimization()`：
```cpp
void LocalmapOptimization(
    MapOfPoses& poses, MapOfPoints3d& points, MapOfLine3d& lines,
    MapOfVelocity& velocities, MapOfBias& biases,
    std::vector<CameraPtr>& camera_list,
    VectorOfMonoPointConstraints& mono_point_constraints,
    VectorOfStereoPointConstraints& stereo_point_constraints,
    VectorOfMonoLineConstraints& mono_line_constraints,
    VectorOfStereoLineConstraints& stereo_line_constraints,
    VectorOfPointLineConstraints& point_line_constraints,
    VectorOfIMUConstraints& imu_constraints, Eigen::Matrix3d& Rwg);
```

四类投影边 + IMU 边：
| 约束 | 维度 | 源 |
|------|------|------|
| 单目点投影 | 2 | `edge_project_point.h` |
| 立体点投影 | 3 | `StereoPointConstraint` |
| 单目线投影 | 4 | `edge_project_line.h` |
| 立体线投影 | 8 | `StereoLineConstraint` |
| 点-线约束 | 1 | 点在线上 |
| IMU 预积分 | 9/15 | `imu.h` |

Huber 鲁棒核；滑窗 3–5 KF；典型耗时 <0.1 s。

### 8.7 IMU 融合（VINS-Mono 风格预积分）
```cpp
// include/imu.h:47-70
class Preinteration {
    Eigen::Vector3d dR, dP, dV;                     // 增量
    Eigen::Matrix3d JRg, JVg, JVa, JPg, JPa;        // 对 bias Jacobian
    Matrix15d Cov;                                  // 15×15 协方差
    void Propagate(double dt, const Vector3d& a, const Vector3d& w);
};
```

**VI 三阶段初始化**（`g2o_optimization.cc:30` `IMUInitialization`）：
1. 陀螺 bias 估计 — 最小二乘拟合前 N 帧旋转
2. 速度 / 重力 — 已知旋转回归
3. 锁定 Rwg + 冻结位姿，进入紧耦合模式

### 8.8 回环 + 全局 BA
DBoW2（`include/bow/database.h:74-78`）—— SuperPoint 259 维转 256-bit 二进制入词袋；倒排索引检索候选 KF → SuperGlue 几何验证 → 添加 `RelativePoseConstraint` → **GlobalBA**（`g2o_optimization.cc:50`，LM 两轮，先宽松后严格剔外点）。

### 8.9 地图数据结构
```cpp
// include/mappoint.h:18-43
class Mappoint {
    int _id;  Eigen::Vector3d _position;
    Eigen::Matrix<float,256,1> _descriptor;        // SuperPoint
    std::map<int,int> _obversers;                   // frame_id → kpt_idx
};

// include/mapline.h:24-62
class Mapline {
    Vector6d _endpoints;                            // 两端点
    Line3DPtr _line_3d;                             // Plücker 参数
    std::map<int,int> _obversers;
};
```
三角化：点用 DLT / 双视图；线用立体配对或双帧椎面交点。

持久化：`Map::SaveMap(map_root)`（Boost 序列化），可离线复用做重定位（`MapUser`）。

### 8.10 配置与数据集
`configs/` 支持 EuRoC、TartanAir、OIVIO、UMA-Bumblebee、**暗光 EuRoC**。典型：
```yaml
# configs/visual_odometry/vo_euroc.yaml
plnet:
  use_superpoint: 1         # 1=SuperPoint+PLNet 双分支，0=纯 PLNet
  max_keypoints: 400
  keypoint_threshold: 0.004
  line_threshold: 0.75
point_matcher:
  matcher: 0                # 0=LightGlue, 1=SuperGlue
```

### 8.11 代码入口速查
| 文件:行 | 内容 |
|---------|------|
| `include/map_builder.h:64` | 入口类 |
| `src/map_builder.cc:157` | `TrackingThread()` |
| `src/map_builder.cc:285` | `FramePoseOptimization` |
| `include/super_point.h:26` / `src/super_point.cpp` | SuperPoint TensorRT |
| `include/plnet.h:23-24` / `src/plnet.cpp` | PLNet 两阶段 |
| `include/feature_detector.h:8` | `FeatureDetector` |
| `src/point_matcher.cc:39-48` | 归一化 + 匹配 |
| `include/line_processor.h:24` | 点线关联 |
| `include/imu.h:47-117` | `Preinteration` |
| `g2o_optimization/g2o_optimization.cc:79` | LocalmapOptimization |
| `g2o_optimization/g2o_optimization.cc:30` | IMU 初始化 |
| `g2o_optimization/g2o_optimization.cc:50` | GlobalBA |
| `include/map_refiner.h:52` | 回环检测 |
| `include/mappoint.h:18` / `include/mapline.h:24` | 地图元素 |

### 8.12 依赖 & 性能
- g2o、DBoW2、TensorRT、OpenCV、Eigen、Ceres（无强制 ROS）
- CPU 占用 ~30–40%（PC）/ ~50–60%（Jetson）
- 特征队列 3、跟踪队列 5 → 前后端解耦
- 实现代码约 8 800 行

### 8.13 与既有系统的算法层差异
| vs | 差异 |
|----|------|
| **ORB-SLAM3** | 学习特征（SuperPoint）替代 ORB → 光照不变性；PLNet 原生支持点线联合 BA（ORB-SLAM3 线特征为社区分支） |
| **VINS-Mono / VINS-Fusion** | 点线紧耦合 BA；后端用 g2o 而非 Ceres；IMU 预积分一致 |
| **EPLF-VINS** | 单次 CNN 统一点线（EPLF-VINS 是 EDLines + 光流两路前端）；目标是**光照鲁棒**而非极线约束优化 |

### 8.14 优缺点
| ✅ | ❌ |
|----|----|
| 学习特征 + LightGlue，暗光 / 光照变化鲁棒 | 需要 TensorRT 部署，模型权重自带但移植门槛高 |
| 原生点线紧耦合 + VI-SLAM + 回环 + 地图持久化 | 无 ROS 原生节点，要接 ROS2 得自己写 wrapper |
| MapUser 支持离线地图重定位 | 代码量大（~8.8k 行），学习曲线陡 |

### 8.15 实战记录（TBD）
> 留给 benchmark 阶段（EuRoC / 暗光 EuRoC、Orbbec Gemini 336 实机、TensorRT 部署踩坑等）。

---

## 横向对比

| 系统 | 状态估计类型 | 前端 | 相机 | 其他传感器 | IMU 耦合 | 回环 | ROS | 语言 | 亮点 |
|------|--------------|------|------|------------|----------|------|-----|------|------|
| open_vins | error-state MSCKF EKF | KLT / ORB | 单/双/多 | — | 紧耦合 CPI | 无（可扩） | ROS1/2 原生 | C++ | 基础平台，精度高 |
| mins | error-state MSCKF EKF | KLT | 单/双/多 | GPS / LiDAR / 轮速 / Vicon | 紧耦合 CPI | 无 | ROS1 | C++ | 多传感器模块化 |
| sqrtVINS | sqrt-form KF | KLT | 单/双/多 | — | 紧耦合 sqrt-CPI | 无 | ROS1/2 原生 | C++ | 数值稳定、长时运行 |
| EPLF-VINS | 滑窗因子图（Ceres） | 点+线（EDLines） | **单目** | — | 紧耦合预积分因子 | DBoW2 | ROS1 | C++ | 点线融合、低纹理鲁棒 |
| VINGS-Mono | GTSAM 因子图 + DL | SuperPoint + LightGlue + DroidNet | 单/立体 | 可选 GNSS / 轮速 | 紧耦合 GTSAM IMU 因子 | LightGlue | 无（离线 Python） | Python | 神经前端 + 3DGS 建图 |
| ORB_SLAM3_ROS2 | 特征点 + Atlas + g2o BA | ORB + DBoW2 | 单/双/RGB-D | — | 紧耦合 MAP | DBoW2 | ROS2（4 节点） | C++ | 6 模态最完整，Atlas 多地图 |
| VINS-Fusion-ROS2 | 滑窗 Ceres NLS | 金字塔 LK 光流（无描述子） | 单/双 | 可选 GPS | 紧耦合预积分 | DBoW2 + BRIEF（`loop_fusion`） | ROS2 | C++ | 200 Hz 高频位姿，在线 td 标定 |
| AirSLAM | 点+线紧耦合 BA（g2o） | SuperPoint + PLNet + LightGlue（学习型） | 单/双 | — | 紧耦合 VINS-Mono 风格预积分 | DBoW2（SuperPoint 量化） | **无 ROS**（独立 C++ + TensorRT） | C++ | 光照鲁棒、双模 V/VI、原生点线 |

### 选型建议
- **基线 / benchmark / ROS2 实机**：`open_vins`
- **长时间 / 鲁棒性要求极高**：`sqrtVINS`
- **多传感器（GPS/LiDAR/wheel）**：`mins`（但需 ROS1）
- **低纹理 / 室内结构化**：`EPLF-VINS`
- **暗光 / 光照剧烈变化**：`AirSLAM`（学习特征 + LightGlue）
- **离线精修 + 重建**：`VINGS-Mono`

> Delaware RPNG 系（open_vins / mins / sqrtVINS）共用 `ov_core`，可以只 clone 一次核心库切换上层应用。

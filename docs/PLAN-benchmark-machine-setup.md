# Benchmark 机器迁移 SOP

把 benchmark 环境迁到另一台机器（下称 **target**）的标准流程。可复用，不绑定日期。

## 目标批次
- **P0**：`open_vins` + `sqrtVINS` 的 GeoScan B1 mono-inertial
- **P1**：`VINS-Fusion-ROS2` mono 对照
- **P2**：`cuvslam_ros` stereo-inertial（若 target 是 NVIDIA）

## 迁移前（source 机）的已知状态
- ORB_SLAM3 mono-inertial 已跑通 → `KeyFrameTrajectory.txt` 可作参考轨迹
- 20 仓库全量文档 `docs/`
- GeoScan B1 rosbag 33 G，标定 yaml 两份

---

## 第 1 步：迁移清单（估算 45 G）

| 项目 | 路径 | 大小 | 迁移方式 |
|------|------|------|----------|
| GeoScan B1 rosbag | `~/Documents/Datasets/geoscan/B1/2026-02-12-16-47-48/` | 33 G | **外接 SSD 或内网 rsync**（USB 3 约 15 min，千兆网 5–6 min） |
| 标定文件 | `~/Documents/Datasets/geoscan/*.yaml` + `.md` | <1 M | 随便什么方式 |
| `vslam_ws/src/` | `~/vslam_ws/src/` | 9 G | 能否只拷要用的 3 个仓库？见下表 |
| `vslam_ws/docs/` | `~/vslam_ws/docs/` | <1 M | 直接拷 |

**src/ 子集优化**（P0+P1 只需要）：
- `open_vins/` + `ov_core/`（若独立仓）
- `sqrtVINS/`
- `VINS-Fusion-ROS2/`
- `ORB_SLAM3_ROS2/`（作基线参考，不一定需要再跑）

实际执行：
```bash
# source 机上
cd ~
tar -cf /tmp/vslam-minimal.tar \
    vslam_ws/docs \
    vslam_ws/src/{open_vins,sqrtVINS,VINS-Fusion-ROS2,ORB_SLAM3_ROS2} \
    Documents/Datasets/geoscan/*.yaml \
    Documents/Datasets/geoscan/*.md \
    Documents/Datasets/geoscan/B1/2026-02-12-16-47-48
# 约 42 G，加压缩意义不大（bag 已经是二进制）
ls -lh /tmp/vslam-minimal.tar
rsync -avP /tmp/vslam-minimal.tar target:/tmp/
# 或走 USB SSD
```

---

## 第 2 步：target 机器环境确认

**必须**：
- Ubuntu 22.04（ROS2 Humble）或 24.04（Jazzy）—— 和 source 对齐
- ROS2 + `rosdep`
- `colcon`、`evo`（`pip install evo`）

**编译需要**：
- Eigen3、OpenCV 4、Boost、Ceres（`libceres-dev`）
- `ros-<distro>-cv-bridge`、`ros-<distro>-message-filters`、`ros-<distro>-tf2-ros`

**若 target 是 NVIDIA**（用于 P2 `cuvslam_ros`）：
- CUDA 11.8+、NVIDIA Container Toolkit（如果用 docker）
- cuVSLAM SDK 本体（`~/Documents/GitHub/slam/cuVSLAM/`）也要拷

先跑 preflight check：
```bash
lsb_release -a                          # 系统版本
source /opt/ros/$ROS_DISTRO/setup.bash  # ROS 能 source
which colcon evo_ape
dpkg -l | grep -E 'libceres-dev|libeigen3-dev|libopencv-dev'
```

---

## 第 3 步：编译（target）

```bash
cd ~/vslam_ws
rosdep install --from-paths src -y --ignore-src   # 自动补依赖

# 先编 ov_core（open_vins/sqrtVINS 共享）
colcon build --packages-select ov_core ov_init ov_msckf
# sqrtVINS 子包名可能是 ov_srvins，以实际为准
colcon build --packages-select ov_srvins
# VINS-Fusion
colcon build --packages-select camera_models vins loop_fusion
```
**已知坑**（迁自 `docs/01 §7.13`）：
- VINS 报 `CUDA not member of ceres` → 加 `#if CERES_VERSION_MAJOR >= 2.1` 守卫
- VINS 报 `argc != 2` → 改成 `argc < 2`
- VINS `loop_fusion` 找不到 vocab → `ln -sf src/.../support_files install/loop_fusion/share/`

---

## 第 4 步：跑 open_vins mono-inertial（P0）

### 4a. 配 YAML（关键）
参考 `~/vslam_ws/src/ORB_SLAM3_ROS2/launch/mono-inertial.launch.py` 和 `config/monocular-inertial/GeoScan_*.yaml`，从中抽：
- 相机内参 `fx, fy, cx, cy`
- 鱼眼畸变 `k1..k4`（**相机模型 = equidistant / KannalaBrandt8**，不是 pinhole）
- IMU-cam 外参 `T_imu_cam`
- IMU 噪声（Kalibr）

对应到 open_vins 的 `ov_msckf/config/` 模板（建议复制 `rpng_aruco` 或 `euroc_mav` 作起点），主要字段：
```yaml
model:                 kannala_brandt    # 鱼眼！
intrinsics:            [fx, fy, cx, cy]
distortion_coeffs:     [k1, k2, k3, k4]
T_imu_cam0:            4x4 矩阵（从 geoscan_camchain-imucam.yaml 抄）
accelerometer_noise_density:   ...       # 从 geoscan_camchain-all.yaml 抄
accelerometer_random_walk:     ...
gyroscope_noise_density:       ...
gyroscope_random_walk:         ...
```

### 4b. 启动命令
```bash
# 终端 1：open_vins
source ~/vslam_ws/install/setup.bash
ros2 launch ov_msckf subscribe.launch.py \
    config_path:=/path/to/geoscan_mono_inertial.yaml \
    use_stereo:=false \
    max_cameras:=1 \
    verbosity:=INFO

# 终端 2：放 bag（跳静止段）
ros2 bag play ~/Documents/Datasets/geoscan/B1/2026-02-12-16-47-48/ \
    --clock --rate 0.5 --start-offset 10
```

### 4c. 录轨迹
open_vins 会发 `/ov_msckf/poseimu`，用 `evo_traj` 或手动 subscribe 存 TUM 格式：
```bash
ros2 topic echo /ov_msckf/poseimu > /tmp/openvins.txt   # 需转 TUM 格式
# 或用 ov_msckf 自带的 output save 参数
```

### 4d. 算 ATE
```bash
evo_ape tum \
    ~/Documents/Datasets/geoscan/B1/2026-02-12-16-47-48/KeyFrameTrajectory.txt \
    /tmp/openvins.txt \
    -va --plot --plot_mode xy --save_plot /tmp/openvins_ape.pdf
```
> 注意 `KeyFrameTrajectory.txt` **不是真实 GT**，是 ORB 自己的估计，只作粗对齐。想要严肃 ATE 需要 EuRoC V101（Vicon GT）做横向参照。

---

## 第 5 步：跑 sqrtVINS（P0 续）

与 open_vins 同一套 YAML（只需字段名略调），启动命令几乎一样：
```bash
ros2 launch ov_srvins serial.launch.py \
    config_path:=/path/to/geoscan_mono_inertial.yaml
```
重点验证 sqrtVINS 的**长时数值稳定性**——跑完整 7 分钟 bag，看 ATE 是否比 open_vins 小或持平（论文宣称持平但 float32 更稳）。

---

## 第 6 步：记录结果

填回 `docs/06-benchmarks.md`：
1. 更新总览矩阵（🟢 / 🟡 / 🔴）
2. `运行记录` 加一条当天日期，按模板填 ATE / CPU / 踩坑
3. `docs/01 §1.13`（open_vins）和 `§3.13`（sqrtVINS）的 **TBD 实战记录** 填实际数据

---

## 第 7 步：返回 source 机时

如果改了配置 / 修了 bug：
```bash
# target 机
rsync -avP ~/vslam_ws/src/<changed_repos>/ source:~/vslam_ws/src/<changed_repos>/
rsync -avP ~/vslam_ws/docs/06-benchmarks.md source:~/vslam_ws/docs/
```
或者在 target 直接 git push（如果各仓库已经是 git repo），source 上 pull。

---

## 风险与 B 计划

| 风险 | 迹象 | B 计划 |
|------|------|--------|
| target 机 ROS 版本不一致 | `colcon build` 一堆 ABI 报错 | 上 Docker；用 `osrf/ros:humble-desktop` 基底 |
| 鱼眼模型配错 | open_vins 起来就飘 / 特征全飞 | 先跑 `euroc_mav` 预置 config 验证编译，再改 GeoScan |
| IMU 初始化一直失败 | "Not enough motion" 循环 | 加大 `--start-offset` 到 15 或 20；或 `init_imu_thresh` 调低 |
| KeyFrameTrajectory 时间戳和 bag 不对齐 | evo 报错 or 轨迹错位 | `evo_traj --t_offset` 手动偏移；或直接用 EuRoC V101 换 GT |
| 33 G bag 拷不完 | 网不好 | 先在 source 切一小段：`ros2 bag convert -i orig -o small --start-offset 10 --duration 120` → 只拿 2 分钟 ~1 G 先验证通路 |

**强烈推荐先做 B 计划最后一条**（切 2 分钟子集），通路跑通再 pull 完整 bag。

---

## 时间预算参考（顺利时 ~4 小时；遇到风险项可能翻倍）

| 阶段 | 时间 |
|------|------|
| 打包 + 传输（含切短 bag） | 45 min |
| target 环境确认 + 装缺失依赖 | 30 min |
| 编译 `ov_core` + `ov_msckf` + `ov_srvins` + VINS | 45 min |
| 配 GeoScan YAML（鱼眼内参 + IMU 噪声） | 30 min |
| open_vins 跑通（含调试） | 60 min |
| sqrtVINS 跑通 | 15 min（共用 config） |
| 记录结果填文档 | 15 min |
| 缓冲 | 20 min |

---

## 带走清单（必带）

- [ ] GeoScan B1 rosbag（完整 33 G 或 2 分钟子集）
- [ ] `geoscan_camchain-{all,imucam}.yaml` + `GeoScan 多传感器标定总结.md`
- [ ] `vslam_ws/src/{ov_core(若独立),open_vins,sqrtVINS,VINS-Fusion-ROS2,ORB_SLAM3_ROS2}`
- [ ] `vslam_ws/docs/`（全量，尤其 `06-benchmarks.md`）
- [ ] ORB 产出的 `KeyFrameTrajectory.txt`
- [ ] 本文件 `PLAN-benchmark-machine-setup.md`

可选：
- [ ] `cuvslam_ros` + `~/Documents/GitHub/slam/cuVSLAM/`（若 target 是 NVIDIA）
- [ ] `AirSLAM`（需要 TensorRT，门槛较高，下次再说）

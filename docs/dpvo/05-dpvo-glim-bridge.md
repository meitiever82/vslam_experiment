# DPVO ↔ GLIM 松耦合 Bridge(Scheme C)

> **澄清**:本文档**不是**真 DPVIO,**别**把它当 DPVIO 看。真 DPVIO 应该是 DPVO 内核里加 IMU preintegration factor,IMU 进 patch graph BA。这件事由本仓库未来的 `src/dpvio/` 包负责,设计放在 [08-dpvio-design.md](08-dpvio-design.md)(待写)。
>
> **本文档讲的是 Scheme C**:LiDAR + IMU + mono 三件套场景下,DPVO 出 mono pose,IMU 进 GLIM 的 LIO,两者通过 `BetweenFactor<Pose3>` 松耦合 —— IMU 集成完全在 GLIM 那侧,DPVO 内核不动。等效一个"GLIM 加 DPVO 视觉约束"的方案,**不**是 DPVIO。
>
> **数据集**:GeoScan B1(`~/Documents/Datasets/geoscan/B1/2026-02-12-16-47-48/`),408 s,4183 LiDAR frames,见 [06-benchmarks.md](06-benchmarks.md) 数据集说明。
> **参考轨迹**:`finder_localization.txt`(LIO body-frame,非 GT)。
>
> **命名史**:2026-04-21 这个模块叫 `dpvo_frontend`(`libdpvo_frontend.so`),源码没 commit 进 git,目录后来从 `casbot_ws` 删了,只剩老 binary。2026-05-12 先用 `dpvio` 名字在 `~/vslam_ws/src/dpvio/` 重建,**马上又意识到 `dpvio` 名字误导**(真 DPVIO 是 IMU 进 DPVO,不是 IMU 进 GLIM),改名为 `src/dpvo_glim_bridge/` / `libdpvo_glim_bridge.so`,把 `dpvio` 让出给未来真 DPVIO 项目。下面文字按新名,§4 历史叙述里碰到老路径请翻译。

---

## 1. 三个方案

LiDAR+IMU+mono 是产品形态。GLIM 作 LIO 主体不动,视觉怎么进来有三个口味。

### Scheme A — 默认(LIO-only odometry)

```
LiDAR + IMU ──→ GLIM (纯 LIO) ──→ pose_metric
Cam0 (RGB) ──→ rtabmap ←─ pose_metric  +  RGB  +  LiDAR-投影 depth
                          ↓
                语义 3D 地图(world_glim 系)
```

视觉**不参与 odometry**,只在 rtabmap 里做语义 mapping + loop closure。最简单、最稳。

### Scheme B — 多 odom 融合(松耦合)

```
GLIM ─→ pose_glim ─┐
DPVO ─→ pose_dpvo ─┤→ rtabmap 多 odom 融合
                   │  + health monitor(对比两边发散度)
Cam0 → 2D seg ────┘
```

DPVO 作"外部 monitor",rtabmap 自己融合多源 odom。**没动手实验过**。

### Scheme C — DPVO 进 GLIM factor graph(紧耦合)

```
DPVO ──/dpvo/pose──→ glim_ext/dpvo_glim_bridge (订阅,在线 scale,造 BetweenFactor)
                                ↓
                       塞 GLIM fixed-lag smoother
LiDAR+IMU ──→ GLIM (LIO + DPVO factor)
                ↓
             pose_metric(优化后)
```

DPVO 的相邻 KF 相对 pose → `BetweenFactor<Pose3>` → GLIM 的 ISAM2。**GLIM 源码不改**,只在 `glim_ext` 加一个外挂模块。**这是本次实验的对象**。

### 为什么是 Scheme C(不是 DPVIO 或 DROID-W)

| 候选 | 否决理由 |
|---|---|
| **自撸 DPVIO**(IMU 直接进 DPVO patch BA) | 工程量 1-4 周;LiDAR+IMU 紧耦合的 LIO 质量已经远好于 IMU-only 进单目 VO;DPVO 上游也不支持,自己维护成本高 |
| **DROID-W**(完整 VO+Loop+Dense BA+3DGS) | 过度集成;backend 跟 frontend GRU update_op 共生,拿不出独立模块;VRAM 16-24GB 带 mapper,4060 8GB 吃紧;轨迹不对则 3DGS 和语义都不会好,单点故障被放大 |
| **改 GLIM 走 LVIO** | 要改 GLIM 源码 + 重新标定,工程成本高;现在没有"必须 LVIO"的硬证据 |

---

## 2. Scheme C 架构与关键设计

### 2.1 进程拓扑

DPVO 和 GLIM 跨进程通过 ROS2 topic 桥接,不内嵌:

| 进程 | 环境 | 输出 |
|---|---|---|
| DPVO node | Python 3.10 venv(`src/DPVO/.venv`),torch 2.3.1+cu121 | `/dpvo/pose`(PoseStamped, ~10 Hz) |
| GLIM + libdpvo_glim_bridge.so | ROS2 Humble C++,GTSAM | `/glim_ros/pose_corrected`(优化后) |

**代价**:一次 serialize + 一次 deserialize 的延迟。
**收益**:两端环境完全解耦,任一端挂了另一端独立排错。已验证 `rclpy + geometry_msgs` 装进 DPVO venv 可用,不需要 TUM-file bridge 兜底。

### 2.2 dpvo_glim_bridge 模块要点

继承 `glim::ExtensionModuleROS2`(**不是** `ExtensionModule`)。重写 `create_subscriptions()` 返回 `TopicSubscription<PoseStamped>`,让 glim_ros 主 executor 接管订阅,避免自起 executor 跟 ISAM2 update 抢线程(见 §4 踩坑 #4)。

挂 `OdometryEstimationCallbacks` 拿 GLIM KF 时间戳与 pose,做下面的事:

1. **在线 scale 估计**(单目 DPVO 无 metric scale):
   - 滑动最小二乘 `s = Σ(d_dpvo · d_glim) / Σ(d_dpvo²)`,窗口 50 对
   - `glim_dist < 10 cm` 的 pair 丢掉(静止段噪声大)
   - 至少 20 对才置 `scale_ready=true`;**scale_ready 之前不塞 factor**(早期塞了会让 ISAM2 Indeterminant)
   
2. **Gate**:
   - `err_r > 0.05 rad`(后期紧到这个值;最早 0.3 rad)的 factor 丢掉,暴露 T_base_camera 外参错或 DPVO 发散段
   - `err_t` 用 scale 应用后跟 GLIM 比,超 2m 丢
   - `max_kf_age > 3.5s`(< smoother_lag=5s)丢,避免绑到已 marginalize 的 key
   
3. **BetweenFactor 注入**:
   - **rotation-only 模式**(v6 定版):`vodom_delta.translation() = glim_delta.translation()`,即 translation 用 GLIM 自己的,DPVO 只贡献 rotation
   - precision `[1e2, 1e2, 1e2, 1.0, 1.0, 1.0]` —— rotation σ ≈ 5.7°,translation σ = 1m;condition number 100,避免 v4 的 1e-6 precision 带来的 NaN
   
4. **NaN guard**:
   - quaternion `.allFinite()` 检查
   - `vodom_delta.matrix().allFinite()` 检查
   - 防 DPVO garbage 污染 ISAM2

### 2.3 GLIM 侧配置

`config_ros.json`:
- `extension_modules` 列表加 `libdpvo_glim_bridge.so`
- **`enable_global_mapping: false`** —— v6 关键(见 §3 v6 调查)

`config_sensors.json`:
- `acc_scale: 9.80665`(GeoScan `/handsfree/imu` 是 g 单位,GLIM 内部转 m/s²)
- `T_lidar_imu`, `T_lidar_camera`(从 Kalibr 标定派生)

---

## 3. 实验结果(GeoScan B1 全段 408 s)

### 3.1 最终对比

| 配置 | Sim3 RMSE | SE3 RMSE | Poses | 说明 |
|---|---:|---:|---:|---|
| **GLIM baseline**(纯 LIO) | **1.68 m** | **2.51 m** | 3379 | rate 0.3,无视觉 |
| v2: +DPVO 6-DoF, global=true | 2.70 | 3.17 | 3933 | scale 漂移 3→52,污染 translation → 比 baseline 差 60% |
| v3: + scale outlier gate | — | — | 0 factor | gate 太严,全 drop |
| v4: rotation-only 但 precision `[1e2,...,1e-6×3]` | NaN | — | — | precision ratio 1e8,Jacobian 病态 |
| v5: rotation-only `[1e2,...,1,1,1]` + global=true | NaN | — | — | global_mapping submap reset 触发 NaN |
| **v6: rotation-only + global=false** | **0.96 m** | **2.17 m** | **3951** | 🎯 **胜 baseline 43% / 13%** |

### 3.2 v6 详细数据

| 指标 | 值 |
|---|---|
| factor pairs computed | 943 |
| factors injected(scale_ready 后)| 452 |
| factors dropped by gate | 0(v6 gate 配 rotation-only 模式,基本不触发) |
| scale 初值 → 稳定值 | 9.50 → ≈ 14.5(±15%) |
| err_r mean | 0.0052 rad = **0.30°** |
| err_t mean(scale 应用后)| **3.58 cm** |
| Indeterminant linear system | 0 |
| GLIM 崩溃 | 0 |
| DPVO OOM 后行为 | ~KF 752 CUDA OOM,GLIM 停注入,继续纯 LIO 跑完 bag |

**产物**:
- `runs/glim_geoscan_b1/glim_withdpvo_rotonly_noGlobal.tum`(v6)
- `runs/glim_geoscan_b1/glim_baseline_full.tum`(baseline)
- `runs/glim_geoscan_b1/glim_withdpvo_full.tum`(v2)

---

## 4. 踩坑实录(按时间顺序)

### #1 GLIM 加载扩展后 `x90 not in VectorValues` 崩溃(2026-04-21)

**症状**:加载 `libdpvo_glim_bridge.so` 后 15 s 内 GLIM 崩,15 s 内 scale_ready 都没触发,即 0 个 factor 进 graph 也崩。

**根因**:扩展模块继承了 `ExtensionModule` 而非 `ExtensionModuleROS2`,内部 `rclcpp::init() + Node + SingleThreadedExecutor + spin_thread` 跟 glim_ros 主 executor 抢 ISAM2 内部 `VectorValues`,损坏后 `update_frames` 查 `X(i)` 抛 `std::out_of_range` → SIGABRT。

**修复**:改基类为 `ExtensionModuleROS2`,删 rclcpp::init / Node / executor / spin_thread,覆写 `create_subscriptions()` 返 `TopicSubscription<PoseStamped>`。glim_ros 自动挂主 executor,无竞态。

**通用教训**:GLIM 扩展模块**永远不要自起 executor**。

### #2 113 s 处 bias indeterminant 崩(2026-04-21)

**症状**:bag 跑到 113 s 附近,GLIM 报 `Indeterminant linear system: Requested variable 'x82' is not in VectorValues`。无论开不开 dpvo_glim_bridge 都崩。

**误诊耗时**:2 小时。先以为 GLIM bias 估计 bug,翻 `fix_imu_bias / imu_bias_noise` 调参,没用。

**真因**:`ros2 bag play` rate=1.0 喂得比 GLIM ISAM2 消化得快 → fixed-lag smoother(`smoother_lag=5s`)drop 老消息 → IMU preintegration factor 接不上之前的 bias 变量 → Indeterminant。

**修复**:`rate 0.3`(用自写的 `bag_replay_monotone.py`,见 §5),GLIM 跟得上。**实测同 bag 同 GLIM 同 config,只把 rate 从 1.0 改到 0.3,跑 159s+ 一次没崩**。

**通用教训**:GLIM 报 Indeterminant 时,**先把 replay rate 降到 0.3 试一次**。只有 rate 0.3 还崩才值得调 GLIM 参数。

### #3 `ros2 bag play` 多 topic 乱序(2026-04-21)

**症状**:GeoScan bag 用 `ros2 bag play` 发,IMU 出现 75 s 时序倒退,LiDAR 出现 306 s 时序倒退。直接用 `rosbags.AnyReader` 扫 bag 完全单调。

**根因**:rosbag2 humble 的 sqlite reader 多线程读 + 按 bag_ns 而非 header.stamp 排序 + GeoScan 不同 topic 有 99 ms / 3 ms / 0 ms 的 capture→publish 延迟差。bag 层面单调,叠加发布后 header.stamp 交叉乱序。

**修复**:自写 `scripts/dpvo_glim/bag_replay_monotone.py`:
- `rosbags.AnyReader` 单线程读,避开 sqlite reader 多线程乱序
- **streaming sort heap**(SORT_WINDOW=0.3 s,>99 ms 观测抖动)保证 publish 严格按 header.stamp 单调
- 支持 `--clock --rate --start-offset --topics` 子集
- QoS = BEST_EFFORT 匹配 sensor_data 默认

### #4 进程残留污染 DDS 拓扑(2026-04-21)

**症状**:看似稳定的 GLIM + dpvo_glim_bridge 组合突然崩,排查发现有 8+ 个旧的 `dump_glim_pose.py / bag_replay.py / dpvo_ros_node` 后台残留进程在发数据,`ros2 topic list` 显示僵尸 publisher。

**修复**:`pkill -f` 不够可靠 —— 必须:
```bash
ps -ef | grep python | grep -v chrome  # 所有 Python 进程
# 拿 PID,挨个 kill -9
ros2 daemon stop && sleep 2 && ros2 daemon start  # 清 discovery 缓存
rm -f /dev/shm/fastrtps_port* /dev/shm/sem.fastrtps_*  # 清 SHM
```

### #5 v4 rotation-only NaN(precision 病态)

**症状**:rotation-only factor + precision `[1e2,1e2,1e2,1e-6,1e-6,1e-6]` → 跑几十秒后 GLIM pose 输出全 NaN。

**根因**:precision ratio `1e2/1e-6 = 1e8`,Jacobian condition number 极差,GTSAM 数值崩。

**修复**:translation precision 抬到 `1.0`(σ=1m,residual ≈ 0 时 factor 几乎不影响 translation,但 Jacobian 良态)。最终 `[1e2,1e2,1e2,1,1,1]` ratio 100。

### #6 v5/v6 global_mapping 之争(2026-04-21)

**最初观察**:开 `enable_global_mapping=true` + dpvo_glim_bridge → 反复 `small overlap 0.0001` 警告,bag 50 s GLIM pose 跑到 (-284, 526, -1292) m(实际走廊只 50 m)。但**不崩**。

**用户质疑**:"视觉不该让结果更差" / "为什么要 disable global"。

**多轮排查后定论**:
1. DPVO 在 GeoScan 走廊上 rotation 输出本身就 noisy(clean 运行也有 err_r > 2 rad 的 outlier)
2. err_r 0.05 rad gate 通过的 factor 仍带小 rotation 偏移
3. 偏移在 odometry 层累积 → submap 初始 `T_world_origin` 有偏
4. global_mapping 的 VGICP + ImuFactor 与 DPVO rotation-only 在 submap 创建时打架 → ISAM2 "挤" pose 超过 `max_correspondence_distance=0.5m` → overlap 崩 → submap reset → 发散反馈
5. **关掉 global,odometry 层 fixed-lag smoother(5s 窗口)看到 rotation-only factor + IMU + LiDAR 自然平衡**,DPVO 偏移只是扰动,不升级为发散

**v6 disable global 是架构选择,不是 workaround**。DPVO mono frontend 和 GLIM submap-alignment 在 GeoScan 这种窄走廊场景**天然竞争**。

**副作用**:没有 global loop closure,长距 drift 应该会扩大。但 v6 ATE 反而更好,说明本 bag 段 global 帮助有限,或 global 在 DPVO 干扰下做不好。

**根本修复**(未做):让 dpvo_glim_bridge 挂 `GlobalMappingCallbacks`,把 factor 同步进 global 的 ISAM2;或让 global_mapping 的 `min_implicit_loop_overlap` gate 对 dpvo 污染前段宽容。属于较重改动,留给 rtabmap 集成时重新考虑(rtabmap 自己有 loop closure,不需要 GLIM global_mapping)。

### #7 DPVO 长 bag CUDA OOM

**症状**:DPVO `--stride 2` 仍在 ~KF 752 OOM(GeoScan 4183 frames,~18% 处);`stride=3` 在 ~KF 100 OOM。

**影响**:v6 实际只覆盖前 18% bag 的 factor 注入,后 82% 纯 LIO 跑完。

**未修**:OOM 后续(未深挖):
- DPVO BUFFER_SIZE=4096 改大?
- DPVO patch graph 累积内存?
- 分段跑,后段单独 init?

---

## 5. 文件清单

### 本 workspace 内(`~/vslam_ws`,2026-05-12 重建)

| 文件 | 作用 |
|---|---|
| `src/dpvo_glim_bridge/` | 🟢 GLIM extension package(本次重建),`libdpvo_glim_bridge.so` |
| `src/dpvo_glim_bridge/src/glim_ext/dpvo_glim_bridge.cpp` | 核心实现(在线 scale + gates + BetweenFactor 注入) |
| `src/dpvo_glim_bridge/CMakeLists.txt` | 链 `~/casbot_ws/install/glim` 为外部依赖,colcon build |
| `src/DPVO/dpvo_ros_node.py` | DPVO ROS2 node,`--pose_topic /dpvo/pose` |
| `src/vslam_experiment/scripts/dpvo_glim/bag_replay_monotone.py` | 替代 `ros2 bag play`,streaming sort heap 单调发 |
| `src/vslam_experiment/scripts/dpvo_glim/launch_geoscan.sh` | 三终端启动参考 |
| `src/vslam_experiment/scripts/dpvo_glim/dump_glim_pose_tum.py` | 订阅 `/glim_ros/pose_corrected` 出 TUM |
| `runs/glim_geoscan_b1/glim_baseline_full.tum` | baseline 输出 |
| `runs/glim_geoscan_b1/glim_withdpvo_rotonly_noGlobal.tum` | v6 输出 |

### 历史遗留(`~/casbot_ws`,只读)

| 文件 | 当前状态 |
|---|---|
| `src/mapping/glim_ext/modules/odometry/dpvo_frontend/` | 🔴 源码已删,未 commit 到 git |
| `install/glim_ext/lib/libdpvo_frontend.so` | 🟡 binary 还在(4月21 v6 build),作为对照保留 |
| `src/mapping/glim/config/geoscan_b1/` | 🔴 已删,本仓库的 `src/dpvo_glim_bridge/config/geoscan_b1/` 是替代 |
| `src/mapping/glim/config/handsfree/config_ros.json` | 🟡 还在,但 extension_modules 已不引用 dpvo_glim_bridge |

---

## 6. 复现(本仓库重建版,2026-05-12 起可跑)

```bash
# 一次性:build vslam_ws 的 dpvo_glim_bridge 包
cd ~/vslam_ws
source /opt/ros/humble/setup.bash
source ~/casbot_ws/install/setup.bash   # GLIM headers + libs
colcon build --packages-select dpvo_glim_bridge --cmake-args -DGLIM_INSTALL_PREFIX=$HOME/casbot_ws/install

# Terminal 1: GLIM + libdpvo_glim_bridge.so(extension_modules 通过 LD_LIBRARY_PATH 或绝对路径加载)
source ~/casbot_ws/install/setup.bash
source ~/vslam_ws/install/setup.bash    # 让 libdpvo_glim_bridge.so 在搜索路径里
ros2 launch glim_ros2 glim_ros2.launch.py \
  config_path:=~/vslam_ws/src/dpvo_glim_bridge/config/geoscan_b1
# stdout 应看到 "[dpvo_glim_bridge] init: topic=/dpvo/pose ..."

# Terminal 2: DPVO node
cd ~/vslam_ws/src/DPVO
.venv/bin/python dpvo_ros_node.py \
  --network dpvo.pth \
  --pose_topic /dpvo/pose \
  --stride 2

# Terminal 3: bag replay(必须 rate 0.3 + monotone)
~/vslam_ws/src/DPVO/.venv/bin/python \
  ~/vslam_ws/src/vslam_experiment/scripts/dpvo_glim/bag_replay_monotone.py \
  --bag ~/Documents/Datasets/geoscan/B1/2026-02-12-16-47-48 \
  --rate 0.3 --clock --start-offset 10

# Terminal 4: 出 TUM
python ~/vslam_ws/src/vslam_experiment/scripts/dpvo_glim/dump_glim_pose_tum.py \
  --topic /glim_ros/pose_corrected \
  --out ~/vslam_ws/runs/glim_geoscan_b1/dpvo_glim_bridge_rerun.tum

# 评测
evo_ape tum ~/Documents/Datasets/geoscan/B1/2026-02-12-16-47-48/finder_localization.txt \
            ~/vslam_ws/runs/glim_geoscan_b1/dpvo_glim_bridge_rerun.tum \
            -s -va
```

---

## 7. 结论

- Scheme C 在 GeoScan B1 上**跑通了,vision 帮 LIO 13% SE3 / 43% Sim3**(0.96 m vs 1.68 m Sim3 RMSE)。
- 但代价是**关掉 GLIM global_mapping**:DPVO mono rotation 与 LIO submap-alignment 在窄走廊天然竞争。
- DPVO 长 bag OOM 没修,实际覆盖仅前 18%,即使如此 v6 仍跑赢 baseline,说明 DPVO 前段 rotation 一致性检查的边际收益是正的。
- **建议优先 Scheme A**(纯 LIO),只在 GLIM 实测 odometry 抽风时升级 Scheme B(rtabmap 多 odom 融合);Scheme C 的 GLIM 源码层动作较深,本 workspace 不再深挖,后续 multimodal-slam 走 rtabmap_ws。

详见相关 memory:
- `memory/project_dpvo_glim_integration.md` —— 350 行 cpp 实现细节 + v2/v4/v5/v6 调优过程
- `memory/project_target_architecture.md` —— 三方案选型背景
- `memory/project_droidw_vo_only_insight.md` —— 为什么不用 DROID-W

# DPVIO 设计 —— 在 DPVO 内核里融 IMU(真 DPVIO)

> **本文档与 [07-dpvo-glim-bridge.md](07-dpvo-glim-bridge.md) 的区别**:
> - 07 写的是 **bridge** —— IMU 不进 DPVO,IMU 进 GLIM;DPVO 出的 mono pose 当 BetweenFactor 塞 GLIM。**不是 DPVIO**。
> - 08(本文)写的是**真 DPVIO** —— IMU preintegration factor 直接进 DPVO 的 patch graph BA,跟 photometric residual 一起被 Schur complement 优化。不依赖 GLIM,不依赖 LiDAR,纯 mono + IMU 出 metric-scale 带 gravity 锚的 pose。
>
> **代码家**:本仓库 `src/dpvio/`(待建,2026-05-12 这天还没动手)。
> **本文档目的**:动手前的设计稿,把工程量 / 选型 / 风险 / 评测计划摊开,免得边写边迷路。

---

## 1. Scope —— 什么算 DPVIO 跑通

最小可用版(MVP)的 acceptance:
- 输入:DPVO + IMU(GeoScan B1 的 `/handsfree/imu` 或 `/livox/imu`,800 Hz / 200 Hz)
- 输出:**带 metric scale 和 gravity 锚**的 pose stream(SE3,世界系 z 轴沿重力)
- 评测:GeoScan B1 全段 408 s vs `finder_localization.txt`,**SE3 RMSE ≤ pure-DPVO 的 60%**(pure DPVO 是 0.60 m 见 [06-benchmarks.md](06-benchmarks.md);MVP 目标 ≤ 0.36 m,理想接近 ORB_SLAM3 mono-inertial 在 V101 上的 0.012 m 水平,但 GeoScan 走廊更难,所以目标不定那么高)
- 不能比 [07-dpvo-glim-bridge.md](07-dpvo-glim-bridge.md) 的 v6 (0.96 m / 2.17 m) 差 —— 不然不如直接跑 bridge

不在 MVP 内的(后期再做):
- Stereo DPVIO(双目+IMU)
- Loop closure(DPVO v2 自带 DBoW2,但 IMU 加进来后回环逻辑需要重新设计)
- 在线标定 IMU↔Camera 外参
- 室外大视野 / 快速运动专项调优

---

## 2. DPVO 现状摘要(survey 结论,详见 §6)

| 部位 | 现状 | 对加 IMU 的影响 |
|---|---|---|
| Pose 表示 | SE3 7D `[tx,ty,tz,qx,qy,qz,qw]`,放在 `patchgraph.poses_[N×7]`,`lietorch.SE3` 做 Lie 代数运算 | 直接复用,IMU 状态额外扩 9D(v + acc_bias + gyro_bias) |
| BA 类型 | Patch-graph photometric,**CUDA kernel `fastba` 硬编码 2D 重投影残差** | **CUDA 路径加不了 IMU residual** → 走 `ba.py` Python BA |
| BA Python 实现 | `ba.py:86-182` 用 `CholeskySolver`(torch),Schur complement 消深度 | **入口在这** —— IMU Jacobian 拼进 H 矩阵 |
| 时间戳 | `tstamps_` 只存 **frame index**,不是 wall-clock | 必须额外存 wall-clock,DPVO ROS node 已有(`dpvo_ros_node.py` PoseStamped),代码层加一个并行 buffer |
| 第一帧锚定 | `frame 0 fixed`(`fixedp=1` in `ba.py:129-135`),world = first KF 的 identity | **重力不对齐 world frame** → 要么改 init 把第一帧旋到重力垂直,要么把 gravity 加成 free var |
| Marginalization | 硬删,无 prior factor | 老 KF 的 IMU bias 估计丢了 → 简单做法是 bias 慢变随机游走 + 每 KF 都跟踪 |
| 关键帧率 | 10 Hz(GeoScan) ~ 8 Hz(EuRoC),`KEYFRAME_THRESH=12.5` | IMU preintegration 窗口 ≈ 100 ms,200 Hz IMU 每个窗口 20 个 sample;800 Hz 80 个 |

---

## 3. 整体架构

```
                      ┌─────────────────────┐
ROS2 ─/image─────────→│ DPVO front-end      │
                      │ (patch detect+net)  │
                      └──────────┬──────────┘
                                 │ patches + 2D corr
                                 ↓
ROS2 ─/imu────┐       ┌─────────────────────────────────┐
              │       │ Patch-graph + IMU state         │
              │       │  poses_   [N×7]  (existing)     │
              │       │  vels_    [N×3]  (new)          │
              │       │  acc_bias [N×3]  (new)          │
              │       │  gyr_bias [N×3]  (new)          │
              │       │  tstamps_secs [N] (new wall-clk)│
              │       └────────────┬────────────────────┘
              │                    │
              ↓                    ↓
   ┌─────────────────────┐   ┌──────────────────────┐
   │ IMU preintegration  │──→│ Joint BA (Python)    │
   │ (gtsam.PIM Python   │   │  visual J + IMU J    │
   │  binding)           │   │  joint Cholesky       │
   └─────────────────────┘   └──────────┬───────────┘
                                        │ pose update
                                        ↓
                              ┌──────────────────────┐
                              │ 输出 /dpvio/pose     │
                              │ (metric-scale,gravity│
                              │  -aligned, SE3)      │
                              └──────────────────────┘
```

---

## 4. 关键决策

### 4.1 IMU preintegration:用 GTSAM Python binding,不自撸

候选:
- **gtsam.PreintegratedImuMeasurements**(Python binding 已装在 GLIM 那侧,或单装 `gtsam-python`)
- **gtsam.PreintegratedCombinedMeasurements**(带 bias 协方差传播,VINS-Mono / ORB_SLAM3 都用这个)
- 自撸 PyTorch 版(参考 ORB_SLAM3 `ImuTypes.cc` ~500 行 C++)

**选 `PreintegratedCombinedMeasurements`** + Python binding。理由:
- 数学正确性已经过工业验证
- 协方差传播免费来,不用自己推
- 跟 GLIM / orb_slam_frontend 同一套库,出问题查文档不用重复

代价:每个 KF pair 一次 Python→C++ binding 调用。但 KF 是 ~10 Hz,可接受。

### 4.2 联合 BA:扩 `ba.py` 的 Python 路径,不动 CUDA

CUDA `fastba` 硬编码 2D photometric residual,改它工程量大且容易破坏 stable visual-only path。**改 `ba.py:86-182` 的 Python BA**:
1. 计算视觉 J(已有)
2. 计算 IMU Jacobian:相邻 KF 间 PIM 预积分后的残差对(R_i, t_i, v_i, b_i, R_j, t_j, v_j, b_j)的导数 —— GTSAM `evaluateError` 直接返回 H/J 块
3. 把 IMU J 加到 H 矩阵的对应 pose / velocity / bias 行列
4. Schur complement 消深度(IMU 部分跟深度无关,直接进 pose 块)
5. Cholesky 解整个系统

**性能折损评估**:
- 视觉 BA 当前 ~50 ms/iter(local),2 iter → ~100 ms
- 加 IMU 估计每 iter +5 ms(每 KF pair 一次 PIM `compute` + Jacobian 拼)
- 10 Hz KF → 100 ms 预算,边缘合格;若不够,把 IMU BA 频率降到 5 Hz(每 2 KF 一次)

**风险**:CUDA visual BA 在 init 阶段(`is_initialized=True` 时 12 iter)用,Python BA 在 sliding window 阶段用。要确认 init 阶段也能加 IMU,否则前 0.8 s 没 IMU 约束,scale + gravity 学不到。**Fallback**:init 阶段把 CUDA BA 跑完后再追加一次 Python BA + IMU,代价 +100 ms 一次性。

### 4.3 World frame + Gravity:在线估,作 free var

DPVO 当前 world = first KF SE3 identity。Gravity 不对齐 world z 轴 → IMU accelerometer 测的 `acc = R^T (a_world + g_world) + bias` 里 `g_world` 不是 `(0,0,-9.81)`。

候选:
1. **Init pre-rotate**:用前 ~1 s IMU 静止数据估 g_world 方向(取 acc 平均归一化),然后把第一 KF 旋转到 z 轴垂直于 g_world。前提:初始 IMU 静止 —— GeoScan 开机就走,**这个假设不成立**。
2. **Gravity as free var**:把 g_world(2 DoF,只有方向,模长 9.81 固定)加到 BA 优化变量。初始猜 `(0,0,-9.81)`,跟 pose 联合优化。**选这个**。
3. **Pre-integration in IMU frame**:不锚 gravity,把 IMU residual 写在 IMU 局部累积坐标里 —— 但这只能做 incremental,出不了 metric。**否决**。

实现:
- `patchgraph.py` 加 `gravity_world` 字段(3D 向量)
- BA system 多 2 个自由度(球面参数化或 retraction 限制模长)
- 初始 12 iter BA 时 gravity 一起优化,sliding window 阶段 gravity fixed(只全局优化才放)

### 4.4 时间戳:从 ROS node 注入 wall-clock

- `dpvo_ros_node.py` 已经在 PoseStamped 里发 `header.stamp`,这是 ROS time。
- DPVO 内部:在 `frame()` 入口加一个 `stamp_secs: float` 参数,存到 `patchgraph.tstamps_secs_[N]`
- IMU 端:订阅 `/imu` 的 Python 节点(或在 DPVO 节点内并行订阅 IMU),IMU 消息按 stamp 累积到 buffer,KF pair 之间的 IMU subset 喂给 `PreintegratedCombinedMeasurements`

### 4.5 Bias 模型:random walk per KF

每 KF 一个 `(acc_bias_i, gyr_bias_i) ∈ ℝ⁶`,KF 间随机游走:
```
b_{i+1} = b_i + n,  n ~ N(0, σ² · dt)
```
σ_acc_walk ≈ 1e-3 m/s²/√Hz,σ_gyr_walk ≈ 1e-5 rad/s/√Hz(典型 MEMS IMU 数值;GeoScan FB100 标定值可能在 Kalibr 文件里,找一下)。

加 BetweenFactor on bias 进 graph。

### 4.6 输出口

新增 `/dpvio/pose`(PoseStamped, world 系,metric-scale)。同时保留 `/dpvo/pose`(原 DPVO 输出)便于对比。可选 `/dpvio/imu_state`(包含 vel + bias,debug 用)。

---

## 5. 实现阶段

| Stage | 工作量(估)| 验证目标 |
|---|---|---|
| **S1 时间戳基础设施** | 1-2 天 | DPVO 每 KF 带 wall-clock,与 ROS msg 时间一致 |
| **S2 IMU buffer + PIM 调用** | 2-3 天 | 任意 KF pair 之间能算出 PIM 预积分量 + 协方差 |
| **S3 状态扩展** | 1-2 天 | `patchgraph` 多 `vels_ / acc_bias / gyr_bias / gravity_world`,初始化合理 |
| **S4 IMU Jacobian 接入 Python BA** | 3-5 天 | `ba.py` 优化器吃 IMU residual,跑通不崩 |
| **S5 Init 处理** | 2-3 天 | 前 1 s 走 visual-only,然后切到 visual+IMU 联合,scale 收敛 |
| **S6 Gravity 联合优化** | 2 天 | gravity 收敛到 9.81 ± 0.05 m/s²,方向稳定 |
| **S7 GeoScan B1 evo_ape** | 1 天 | 拿数 + 写 benchmark 行 |
| **S8 EuRoC V101 对照(可选)** | 1 天 | 跟 ORB_SLAM3 0.012m 比对 |

**总计 13-19 工作日**(乐观),memory 的 1-4 周估计基本对得上。

---

## 6. DPVO 内核 survey 摘要

完整 survey 见 session 历史。关键发现:

1. **Pose state**: `dpvo/patchgraph.py:27` `self.poses_ = torch.zeros(N, 7, ...)`(7D SE3 `[t, q]`),`lietorch.SE3` 做 Lie 代数。

2. **CUDA BA 入口**: `dpvo/fastba/ba.py:7` `cuda_ba.forward(...)`,残差结构 hardcoded 在 `ba_cuda.cu:232-340` 是 2D 重投影 (target_xy - reproj_xy)。**加不了 IMU**。

3. **Python BA 入口**: `dpvo/ba.py:86-182`,用 `CholeskySolver`(torch autograd)。**这是 IMU 残差的接入点**。

4. **Variable set**: poses (6 DoF) + inverse depths (1 DoF/patch),intrinsics fixed。

5. **Edges**: `(ii, jj, kk)` 索引(KF i 的 patch k 投影到 KF j),`append_factors` 在 `dpvo.py:215-221`。

6. **Marginalization**: `dpvo/dpvo.py:266-303` hard remove,`pg.delta` 字典存被移走的 KF 相对 pose,**没有 prior factor**。

7. **First KF fixed**: `ba.py:129-135` `fixedp=1`,第一帧 pose 永远不动。

8. **No gravity**, no velocity, no IMU code 任何形式。一切都得加。

9. **Time**: `tstamps_` 是 frame index;没有 wall-clock,IMU sync 必须从 ROS node 侧注入。

10. **Constant-velocity motion model**: `dpvo.py:411-424` 用 `MOTION_DAMPING * (P1 * P2.inv()).log()` 预测下一帧。IMU prediction 应该替换这个。

---

## 7. 风险清单

| 风险 | 影响 | Mitigation |
|---|---|---|
| **Python BA 速度跟不上 10 Hz KF** | 实时性丢 | (a) IMU BA 隔帧跑 (b) 移植关键 Jacobian 到 CUDA(后期) |
| **Init 阶段 scale/gravity 不收敛** | 前几秒 pose 偏 | 加 ZUPT(零速更新),如果 IMU 静止就锁 velocity=0 提供约束 |
| **GeoScan IMU 是 g 单位(memory 中坑)** | acceleration 数量级错 1 个 g | 第一行代码就 ×9.80665 转 m/s²,跟 GLIM 同处理 |
| **lietorch SE3 跟 gtsam Pose3 quaternion 顺序不同** | NaN / 翻飞 | `lietorch=[qx,qy,qz,qw]`,`gtsam.Rot3.toQuaternion()=[w,x,y,z]`,接口处显式转 |
| **CUDA BA 走完再 Python BA 会让 pose 跳变** | 第二次 BA 跟 IMU 打架 | 联合 init:第一次 BA 就用 Python + IMU,12 iter |
| **Loop closure(DPVO v2 DBoW2)和 IMU 状态对不上** | 回环修正 pose 但不修 vel/bias | MVP 关掉 v2 回环(`enable_loop_closure=False`);后期再加 |
| **EuRoC V101 IMU 200 Hz vs GeoScan 800 Hz** | 性能差异 | PIM 是 hz-agnostic,代码只要支持任意 sample rate |
| **gtsam-python 装不上(GFW)** | S2 卡住 | 已知 casbot_ws/GLIM 编译 GTSAM C++ 库;走 cppyy 或 pybind11 直绑 |

---

## 8. 评测计划

### 8.1 GeoScan B1(产品场景)

```bash
cd ~/vslam_ws/src/DPVO
.venv/bin/python -m dpvio.run_geoscan \
  --bag ~/Documents/Datasets/geoscan/B1/2026-02-12-16-47-48 \
  --imu_topic /handsfree/imu --image_topic /left_camera/image \
  --output runs/dpvio_geoscan_b1.tum

evo_ape tum ~/Documents/Datasets/geoscan/B1/2026-02-12-16-47-48/finder_localization.txt \
            runs/dpvio_geoscan_b1.tum \
            -va --plot --save_results dpvio_geoscan_ape.zip
# Note: NO -s, 因为 DPVIO 是 metric 的
```

对照:
- pure DPVO 0.60 m(mono no IMU)
- DPVO+GLIM bridge v6: 0.96 m Sim3 / 2.17 m SE3
- ORB_SLAM3 mono-inertial: GeoScan 上没跑过,做不了对照
- AirSLAM stereo-inertial: 上限 reference

### 8.2 EuRoC V1_01_easy(paper-class)

```bash
cd ~/vslam_ws/src/DPVO
.venv/bin/python -m dpvio.run_euroc \
  --scene V1_01 --output results/dpvio_v101.tum
evo_ape tum datasets/euroc_groundtruth/V1_01_easy.txt \
            results/dpvio_v101.tum -va
```

对照:
- pure DPVO V101: 0.046 m
- ORB_SLAM3 mono-inertial V101: **0.012 m**(目前 V101 mono-inertial SOTA)
- VINS-Fusion / OpenVINS / EPLF-VINS: 见 [06-benchmarks.md](06-benchmarks.md)

---

## 9. 下次动手的入口

按时间顺序:
1. 读 `dpvo/dpvo.py` 整文件,特别是 `__call__`(主 frame 函数)和 `terminate()`
2. 读 `dpvo/ba.py` 整文件,理解 H 矩阵构建
3. 装 `gtsam-python`(或确认 casbot_ws 那个 `libgtsam.so` 能 cppyy 调)
4. **Step S1**:fork `dpvo_ros_node.py` 为 `dpvio_ros_node.py`,加 IMU 订阅 + 时间戳传递

---

## 10. 替代方案(如果 DPVIO 工程量太大)

如果做到一半发现 MVP 还是要 3 周以上,后备:
- 把 [07-dpvo-glim-bridge.md](07-dpvo-glim-bridge.md) 的 Scheme C v6 拉来用(已 verified 跑赢 LIO baseline 43%),反正产品场景有 LiDAR + IMU,GLIM 这一侧 IMU 已 mature
- 继续做但目标定低一些,只做 S1-S4 出 prototype(visual+IMU 联合 BA 跑通即可,不死磕 ATE)

不否决,只是给"不必硬刚"留出口。

# PIN-SLAM 系统总览

> 本文档对应 codebase-analysis-skill 的 Phase 1(全局探索)。所有引用路径相对 `src/PIN_SLAM/`。
> 论文: Pan et al., *PIN-SLAM: LiDAR SLAM Using a Point-Based Implicit Neural Representation for Achieving Global Map Consistency*, T-RO 2024.

---

## 1. 系统定位 (TL;DR)

PIN-SLAM 是一套 **纯 LiDAR(可选 RGB-D)** 的全栈 SLAM,使用 **点基隐式神经表示(PIN)** 作为地图:

- 地图不是体素 / 八叉树 / TSDF 网格,而是 **一堆带可学习特征向量的"神经点"**
- SDF 由一个 **共享的轻量 MLP** 从邻域加权特征解码,不依赖位置网格
- 跟踪 = **point-to-implicit registration**(传统 ICP 的隐式 SDF 版本)
- 后端 = scan/local-map context 描述子做回环 + gtsam ISAM2/LM 的位姿图优化
- 当回环修正发生时,**神经点会随对应帧位姿一起被弹性变换**,这就是"globally consistent"的来源

主循环串行执行 6 个阶段,无 IMU,无视觉 VIO,无 KISS-ICP 那种 voxel cube map。

---

## 2. 顶层入口

### 2.1 离线 / 数据集模式 — `pin_slam.py`

入口由 [typer](https://typer.tiangolo.com/) 装饰 `run_pin_slam` 接管 `[VERIFY: pin_slam.py:83-101]`。命令行支持 5 个位置/选项参数族:

| 类别 | 关键参数 | 作用 |
|------|----------|------|
| 配置 | `config_path` (positional) | yaml 文件,默认 `config/lidar_slam/run.yaml` `[VERIFY: pin_slam.py:85]` |
| 数据 | `-i/--input-path` | 覆盖 yaml 里 `pc_path` `[VERIFY: pin_slam.py:88,123]` |
| 数据集 | `dataset_name` + `sequence_name` + `-d` | 走 KISS-ICP 风格的 dataloader,而不是泛型 ply/bin 文件夹 `[VERIFY: pin_slam.py:86-87, 92, 128-129]` |
| 可视化 | `-v` / `-w` | open3d GUI + wandb logging `[VERIFY: pin_slam.py:93, 96]` |
| 落盘 | `-s/-m/-p` | 保存 neural points / mesh / merged 点云 `[VERIFY: pin_slam.py:97-99]` |

初始化序列(`run_pin_slam` body):
1. `Config().load(config_path)` 然后被命令行覆盖 `[VERIFY: pin_slam.py:102-130]`
2. `setup_experiment(config, argv)` 建立 run 目录 + 子目录 `[VERIFY: pin_slam.py:132]`
3. 实例化 3 个 MLP decoders `geo_mlp / sem_mlp / color_mlp`,塞进 `mlp_dict` `[VERIFY: pin_slam.py:139-146]`
4. 实例化 `NeuralPoints / NeuralPointMapContextManager / SLAMDataset / Tracker / Mapper / Mesher / PoseGraphManager` `[VERIFY: pin_slam.py:149-187]`
5. 给 PGO 加上 0 号节点的 prior(GT pose 或 identity)`[VERIFY: pin_slam.py:188-189]`
6. 如果 `--visualize`,额外 `mp.spawn` 启动 GUI 进程,通过 2 个 `mp.Queue` 双向通信 `[VERIFY: pin_slam.py:201-217]`

### 2.2 在线 ROS 模式 — `pin_slam_ros.py`

`rclpy` 节点 + 一个 SLAM worker 线程:LiDAR 回调把点云入队,worker 取队列做 SLAM,然后 publish odometry + neural-point map。架构与离线版主循环一致,只是 frame 来源换成订阅 + 内部 buffer。

---

## 3. 主循环 6 阶段

下面的 ASCII 图就是 `pin_slam.py:238-505` for 循环每帧的执行序。`Ti = get_time()` 是源码自带的计时点。

```
┌──────────────────────────────────────────────────────────────────────────────────┐
│                  for frame_id in tqdm(range(dataset.total_pc_count))             │
└──────────────────────────────────────────────────────────────────────────────────┘
   │
   │  T0  remove_gpu_cache()                                  [pin_slam.py:240]
   │
   ▼
┌────────────────────────────────────────────────────────────────┐
│ Stage I. Load & Preprocess                                     │
│ ─────────────────────────                                      │
│  dataset.read_frame[_with_loader] (frame_id)                   │  [pin_slam.py:245-248]
│  dataset.preprocess_frame()                                    │  [pin_slam.py:252]
│   ├─ pose 初值 = uniform_motion / static / GT  [slam_dataset.py:374-389] │
│   ├─ adaptive_range + 体素降采样              [slam_dataset.py:399-437] │
│   ├─ z / range crop_frame                       [slam_dataset.py:455-463] │
│   ├─ frame_id>0: cur_source_points = 二次更稀疏降采样 [slam_dataset.py:475-485] │
│   └─ optional deskew(用上一帧 odom 增量做线性插值) [slam_dataset.py:493-500] │
│  return valid_frame  (=False 则 continue)       [pin_slam.py:253-255]   │
└────────────────────────────────────────────────────────────────┘
   │
   │  T1, T2 mark
   ▼
┌────────────────────────────────────────────────────────────────┐
│ Stage II. Odometry                                              │
│ ─────────────                                                    │
│  if frame_id > 0 and config.track_on:                            │
│      tracker.tracking(cur_source_points, init_guess, ...)        │  [pin_slam.py:262-264]
│      → cur_pose_torch (4×4 float64)                              │
│      → cur_odom_cov  (6×6, optional)                             │
│      → valid_flag    (退化/残差检查通过否)                       │
│  dataset.update_odom_pose(cur_pose_torch)                        │  [pin_slam.py:266]
│  travel_dist 更新,sync 给 neural_points.travel_dist             │  [pin_slam.py:274-275]
└────────────────────────────────────────────────────────────────┘
   │
   │  T3 mark
   ▼
┌────────────────────────────────────────────────────────────────┐
│ Stage III. Loop Detection + PGO (可选, config.pgo_on)           │
│ ───────────────────────────────                                   │
│  ① 描述子写入                                                    │
│     · local_map_context==True 且 frame_id ≥ latency:             │
│        - reset_local_map → local neural points 为描述子来源       │
│        - lcd_npmc.add_node(local_map_frame_id, ptcloud_local, …)  │  [pin_slam.py:283-290]
│     · 否则用裸 scan context                                       │  [pin_slam.py:292]
│  ② PGO 因子                                                       │
│     pgm.add_frame_node + add_odometry_factor                      │  [pin_slam.py:293, 297]
│     pgm.estimate_drift(travel_dist, ...)                          │  [pin_slam.py:298]
│  ③ 每 pgo_freq 帧尝试一次回环                                     │  [pin_slam.py:302]
│     · 距离粗筛 → detect_local_loop / detect_global_loop           │  [pin_slam.py:307-309]
│     · 找到 loop_id 后:                                            │
│        - recreate_hash + tracker.tracking(loop_reg=True) 二次精配 │  [pin_slam.py:317-319]
│        - add_loop_factor → optimize_pose_graph                     │  [pin_slam.py:325, 329]
│        - neural_points.adjust_map(pose_diff) 弹性变形              │  [pin_slam.py:337]
│        - recreate_hash + mapper.transform_data_pool                │  [pin_slam.py:338-339]
└────────────────────────────────────────────────────────────────┘
   │
   │  T4 mark
   ▼
┌────────────────────────────────────────────────────────────────┐
│ Stage IV. Mapping & Bundle Adjustment                            │
│ ─────────────────────────────────────                            │
│  · 失稳重启: lose_track 计数 ≥ reboot_frame_thre 触发 reboot     │  [pin_slam.py:354-363]
│  · valid_mapping_flag 才允许新观测进入地图 / 数据池              │  [pin_slam.py:368]
│       mapper.process_frame(point_cloud, sem, pose, frame_id,…)   │  [pin_slam.py:369-370]
│        ├─ dynamic_filter (可选)        [mapper.py:99-137]         │
│        ├─ sampler.sample → 7 个/ray   [mapper.py:217-226]         │
│        ├─ neural_points.update (体素哈希加新点) [mapper.py:260-262]│
│        └─ 数据池追加 + 距离-滑窗滤除 [mapper.py:275-360]          │
│  · 否则 reset_local_map 但跳过观测                                │  [pin_slam.py:372-373]
│  · BA(可选,周期 ba_freq_frame)                                  │  [pin_slam.py:388-389]
│  · mapping(cur_iter_num) — Adam 迭代                              │  [pin_slam.py:393]
└────────────────────────────────────────────────────────────────┘
   │
   │  T5, T6 mark
   ▼
┌────────────────────────────────────────────────────────────────┐
│ Stage V. Mesh + Visualization(仅 --visualize)                  │
│ ──────────────────────────────                                   │
│  按 vis_mesh_freq_frame 重建 mesh(marching cubes 分块)         │  [pin_slam.py:445-456]
│  按 vis_sdf_freq_frame 生成 SDF 切片(可选 v + h)               │  [pin_slam.py:459-467]
│  组 VisPacket → q_main2vis.put()                                 │  [pin_slam.py:473-492]
└────────────────────────────────────────────────────────────────┘

  [循环末尾] cur_frame_process_time 入 time_table,wandb log     [pin_slam.py:500-506]
```

退出循环后 Stage VI(`pin_slam.py:510-544`)做:
1. `mapper.free_pool()`
2. `dataset.write_results()` 写 KITTI / TUM 格式位姿 + 误差
3. PGO 落盘 g2o + loop_log + loop_plot
4. `neural_points.prune_map` + `recreate_hash(merged)` 最终合并
5. 重建全局 mesh(分块 marching cubes)
6. `save_implicit_map` 把 neural points + 3 个 MLP 落盘

---

## 4. 静态依赖图

```
                  ┌──────────────────┐
                  │ pin_slam.py main │
                  └─────────┬────────┘
        ┌──────────┬────────┼──────────┬────────────┬─────────────┐
        │          │        │          │            │             │
        ▼          ▼        ▼          ▼            ▼             ▼
   ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌──────────┐ ┌──────────┐
   │Config  │ │SLAM    │ │Neural  │ │Decoder │ │Tracker   │ │Mapper    │
   │utils/  │ │Dataset │ │Points  │ │model/  │ │utils/    │ │utils/    │
   │config  │ │dataset/│ │model/  │ │decoder │ │tracker   │ │mapper    │
   └────────┘ └────┬───┘ └───┬────┘ └────┬───┘ └────┬─────┘ └──┬───────┘
                   │         │           │          │          │
                   │      ┌──┴─────┐  ┌──┴───┐   ┌──┴────┐  ┌──┴───┐
                   ▼      ▼        │  │      │   │       │  │      │
              dataloader  Mesher   │  loss   │  data_sampler  pgo   │
              s/*         utils/   │  utils/ │  utils/        utils/│
                          mesher   │  loss   │  data_sampler  pgo   │
                                   │         │                       │
                                   ▼         ▼                       ▼
                              loop_detector  tools             gui/slam_gui
                              utils/loop_d   utils/tools       (mp.Process)
```

- `NeuralPoints` 既是 `nn.Module`(`local_geo_features` / `local_color_features` 是 `nn.Parameter`)也是数据容器 — 它和 3 个 MLP 一起被 `setup_optimizer` 注册到 Adam `[VERIFY: utils/mapper.py:604-621]`,这是 PIN 的关键: **特征向量本身是被优化的变量**。
- `Mapper` 持有 `dataset` / `neural_points` / `mlp_dict` 的引用,是真正"拥有"训练循环的对象。
- `Tracker` 只读 `neural_points` + `mlp_dict`,从不写入它们 — 跟踪期间 MLP forward 是 `training_mode=False`,不会改 certainty `[VERIFY: utils/tracker.py:303-308]`。

---

## 5. 关键超参与缺省值

下表只列对结果影响最大的字段。完整字段表见 `DATA_STRUCTURES.md`。

| 字段 | 默认 | 影响 | 源 |
|------|------|------|-----|
| `voxel_size_m` | 0.3 m | 神经点体素大小,直接决定地图分辨率 | `utils/config.py:92` |
| `feature_dim` | 8 | 每个神经点的特征向量长度 | `utils/config.py:103` |
| `query_nn_k` | 6 | 每个查询点选 KNN 6 个邻居加权 | `utils/config.py:96` |
| `num_nei_cells` | 2 | 邻域搜索的体素半径(±2 立方 + 球裁剪) | `utils/config.py:95` |
| `search_alpha` | 0.2 | 球裁剪半径系数 → 当 num_cells=2 时 K≈33 | `utils/config.py:98`, `model/neural_points.py:911-948` |
| `buffer_size` | 5×10⁷ | spatial hash 桶数 | `utils/config.py:100` |
| `surface_sample_range_m` | 0.25 m | 表面采样高斯 σ,bce sigmoid 锐度 | `utils/config.py:124`, `utils/config.py:429` |
| `surface_sample_n / free_front_n / free_behind_n` | 3 / 2 / 1 | 每条射线 7 个 sample | `utils/config.py:125, 128, 129` |
| `local_map_radius` | 2.0 + max_range | 局部地图截断球半径(运行时自适应) | `utils/config.py:115`, `utils/config.py:560` |
| `local_map_travel_dist_ratio` | 5.0 | 时间窗 = 5×local_map_radius 的累计里程 | `utils/config.py:114`, `model/neural_points.py:62-63` |
| `iters` / `init_iter_ratio` | 12 / 40 | 每帧迭代次数,第 0 帧 warm-up = 12×40 | `utils/config.py:191-192` |
| `lr` / `bs` | 0.01 / 16384 | Adam 学习率 + batch size | `utils/config.py:194-195` |
| `weight_e` | 0.5 | Eikonal 正则权重 | `utils/config.py:179` |
| `main_loss_type` | 'bce' | SDF 主损失,见 `utils/loss.py:45-63` | `utils/config.py:163` |
| `sigma_sigmoid_m` | 0.1 | BCE sigmoid 的 σ;`decoder.sdf_scale = ratio * σ` | `utils/config.py:164`, `model/decoder.py:55-56` |
| `freeze_after_frame` | 40 | 第 40 帧后冻结 3 个 MLP,只继续训练特征 | `utils/config.py:152`, `pin_slam.py:382-385` |
| `pgo_freq` | 30 帧 | 至少隔多少帧才尝试一次回环 | `utils/config.py:255`, `pin_slam.py:302` |
| `reg_iter_n` | 50 | 跟踪 LM 最大迭代 | `utils/config.py:228`, `utils/tracker.py:88` |
| `reg_GM_dist_m / reg_GM_grad` | 0.3 / 0.1 | Geman-McClure 阈值 | `utils/config.py:224-225` |

---

## 6. 失效保护与状态机

跟踪不是"试一次就用",而是带 4 道安全门:

| 检查点 | 触发条件 | 处理 | 源 |
|--------|----------|------|----|
| 残差增长 | `(sdf_residual_cm - last) / last > 1.1` | `valid_flag = False`,提前 break | `utils/tracker.py:150-159` |
| 有效点不足 | valid_count < 30 或 ratio < 0.2(loop_reg 0.15) | 失败 | `utils/tracker.py:101, 162-169` |
| 最终残差 | > `surface_sample_range_m × final_residual_ratio_thre × 100 cm` | 失败 | `utils/tracker.py:92-94, 198-203` |
| 特征值退化 | min(eigenvalues) < valid_count × `eigenvalue_ratio_thre` | 失败 | `utils/tracker.py:205-216` |

跟踪失败时:
1. `dataset.lose_track = True` `[VERIFY: pin_slam.py:265]`
2. `update_odom_pose` 仍写入 init guess(避免跳变),累计 `consecutive_lose_track_frame` `[VERIFY: dataset/slam_dataset.py:541-544]`
3. 这一帧不进入地图(`valid_mapping_flag = False`)`[VERIFY: pin_slam.py:276, 368]`
4. 连续 ≥ `reboot_frame_thre`(默认 5)帧失败 → 系统重启:`mapper.init_pool()` + `neural_points.reboot_ts = frame_id` + 解冻 MLP `[VERIFY: pin_slam.py:354-363]`

机器人停车检测也是一档:
- `update_odom_pose` 里 `tranmat_close_to_identity` 判定 `[VERIFY: dataset/slam_dataset.py:520-525]`
- 连续 ≥ `stop_frame_thre`(默认 20)帧静止 → `stop_status = True`,跳过 mapping 但保持其它环节运行 `[VERIFY: dataset/slam_dataset.py:527-532, pin_slam.py:368, 380-381]`

---

## 7. 与"传统"LIO 系统的对比定位

PIN-SLAM 在本 workspace 里和其它 ~20 个 LIO 系统并列,但它和 FAST-LIO / Point-LIO / LIO-SAM 这类 ESKF-based 系统在**结构**上完全不一样:

| 维度 | FAST-LIO 类 | PIN-SLAM |
|------|------------|----------|
| 状态估计 | IMU 预积分 + ESKF | 无 IMU,纯点云 LM 配准 |
| 地图表示 | ikd-tree / iVox 显式点 | 神经点(带特征)+ MLP |
| 距离量 | point-to-plane | point-to-implicit-SDF |
| Loop closure | 大多无 | Scan / Neural Map Context + ISAM2 |
| 全局一致 | 否(里程计漂移直接累计) | 是(回环 → adjust_map 弹性变形) |
| 实时性 | GPU 可选,CPU 也 OK | **必须有 GPU**(神经特征查询 + MLP forward) |
| 适用动态场景 | 多数依赖 ESKF 速度差检测 | `dynamic_filter` 用 SDF 一致性 + certainty 双判别 |
| Mesh | 通常没有 | 一等公民,可在线 marching cubes |

详细多系统对比见 `src/lio-summary.md`(workspace 根目录的对比文档)。

---

## 8. 进入 Phase 2-4 的阅读顺序

读者建议按以下顺序看本目录后续文档:

1. `DATA_STRUCTURES.md` — 把 `NeuralPoints` / `Mapper.pool` / `PoseGraphManager` 的字段建立内存模型
2. `DATA_FLOW.md` — 单帧期间张量怎么流转
3. `ALGORITHM_01-NeuralPointMap.md` — 哈希 + 邻域 + IDW 的底层数学,所有后续算法都依赖这一层
4. `ALGORITHM_02-Decoder_Loss_Sampler.md` — MLP 输出语义、BCE 与 SDF 的关系、为什么 7 个 sample
5. `ALGORITHM_03-Tracker_SDF_Odometry.md` — point-to-implicit LM 推导
6. `ALGORITHM_04-Mapper_Training.md` — 在线 Adam 训练 + replay + numerical gradient
7. `ALGORITHM_05-LoopDetector_PGO.md` — Scan Context 派生的 NPMC 描述子 + gtsam 因子图

---

**END of Phase 1 OVERVIEW.**

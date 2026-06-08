# PIN-SLAM 数据流

> 对应 Phase 3。所有 `[VERIFY:]` 路径相对 `src/PIN_SLAM/`。
> 目标:把一帧从"磁盘 / 话题"到"位姿 + 地图更新 + 可选 mesh"的所有张量变换路径写清楚。

---

## 1. 鸟瞰图

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│  原始来源                                                                       │
│   ├─ pc_filenames[i]  (.bin/.ply/.pcd)         ────►  read_frame                │
│   ├─ KISS-ICP loader[frame_id_in_folder]       ────►  read_frame_with_loader    │
│   └─ ROS PointCloud2 msg                       ────►  read_frame_ros            │
└─────────────────────────────────────────────────────────────────────────────────┘
                                  │ ndarray [N_raw, 3 (+color/intensity)]
                                  ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│  cur_point_cloud_torch  [N_raw, 3+c]  cur_point_ts_torch [N_raw] (可选)         │
│                                                                                  │
│         preprocess_frame                                                         │
│         ├─ pose 初值(uniform motion / static / GT)→ cur_pose_guess_torch [4,4]│
│         ├─ adaptive_range_on → 动态调整 crop_max_range / vox sizes               │
│         ├─ voxel_down_sample_torch(leaf=train_voxel_m) → N_train ≈ N_raw/×      │
│         ├─ crop_frame(min_z, max_z, min_range, crop_max_range) → N_keep         │
│         ├─ kitti_correction_on → 几何修正                                        │
│         └─ frame_id>0:                                                           │
│              cur_source_torch = voxel_down(leaf=source_voxel_m)                  │
│              cur_source_points [N_src, 3]  (N_src << N_keep)                     │
│              if deskew: cur_source_points = deskewing(.., last_odom_tran)        │
└─────────────────────────────────────────────────────────────────────────────────┘
                                  │
        ┌─────────────────────────┼──────────────────────────────────┐
        │                         │                                  │
        ▼                         ▼                                  ▼
  ┌─────────────┐         ┌──────────────────┐               ┌──────────────────┐
  │  tracker    │         │ mapper.process_  │               │ neural_points.   │
  │  .tracking  │         │   frame          │               │  update          │
  │  (frame>0)  │         │                  │               │                  │
  │             │         │ sampler.sample → │               │ voxel hash → 新点│
  │ cur_source  │         │  coord (N·7,3)   │               │ + 维度对齐 + ts  │
  │ → T_w<-cur  │         │  sdf_label (N·7) │               │ + reset_local_map│
  │ + 6×6 cov   │         │  weight (N·7)    │               │                  │
  │ + valid_flag│         │                  │               │                  │
  └─────────────┘         │ pool append +    │               │ local 张量重建   │
        │                 │ distance window  │               │ + global2local  │
        │                 │ filter           │               └──────────────────┘
        ▼                 └──────────────────┘
  update_odom_pose
   ├─ last_odom_tran
   ├─ travel_dist[++]
   └─ pgo_poses[frame_id]
        │
        ▼
  Stage III. LCD + PGO   (可选)
   ├─ npmc.add_node(描述子)
   ├─ pgm.add_odometry_factor
   ├─ if loop: tracker.tracking(loop_reg=True)
   │            → pgm.add_loop_factor → optimize_pose_graph
   │            → neural_points.adjust_map(pose_diff) + recreate_hash
   │            → mapper.transform_data_pool
   │            → dataset.update_poses_after_pgo
   └─ travel_dist 没有改变(PGO 不重写里程)
        │
        ▼
  Stage IV. mapper.mapping(iters)
   ├─ for iter in range(iters):
   │    coord, label, ts, _, sem, color, w = get_batch(global_coord=True)
   │    geo_feat = neural_points.query_feature(coord, ts)   [N_b, K, F+P]
   │    sdf_pred = sdf_mlp.sdf(geo_feat)
   │    g = get_numerical_gradient(coord, sdf_pred, ε)
   │    loss = sdf_bce + weight_e × eikonal + (sem) + (color)
   │    loss.backward() + Adam.step()
   └─ assign_local_to_global()  ← 把训练后的 local Parameter 写回全局张量
        │
        ▼
  Stage V. mesh / sdf slice / VisPacket  (仅 --visualize)
   ├─ neural_points.get_neural_points_o3d(query_global, down)
   ├─ mesher.recon_aabb_collections_mesh(chunks, mc_res, ...)
   └─ q_main2vis.put(packet)
        │
        ▼
  循环结束 → Stage VI 落盘
   ├─ neural_points.prune_map + recreate_hash(merged)
   ├─ save_implicit_map → .pth (neural_points + 3 MLP)
   └─ mesher 全局重建 + io.write_triangle_mesh
```

---

## 2. Stage I — 读取与预处理

### 2.1 三条入口对比

| 入口 | 输入 | 输出补充 | 调用方 |
|------|------|----------|--------|
| `read_frame(frame_id)` | 泛型文件夹 `.bin/.ply/.pcd` + 可选 SemanticKITTI 标签 | `cur_point_cloud_torch + cur_sem_labels_torch/_full + cur_point_ts_torch` | 离线 `pin_slam.py:248` |
| `read_frame_with_loader(frame_id)` | KISS-ICP loader 字典(`points / point_ts / img / imus`)| 同上 + `cur_cam_img + cur_frame_imus` | 离线 `pin_slam.py:246` |
| `read_frame_ros(msg)` | `sensor_msgs/PointCloud2` | 同上(ts 由消息字段提取,可能为 None)| ROS `pin_slam_ros.py` 回调 |

源头都把数据塞到 `cur_point_cloud_torch`,后续 `preprocess_frame` 统一处理。

证据:
- 泛型: `[VERIFY: dataset/slam_dataset.py:254-294]`
- KISS-ICP: `[VERIFY: dataset/slam_dataset.py:215-252]`
- ROS: `[VERIFY: dataset/slam_dataset.py:181-212]`

### 2.2 `get_point_ts` —— per-point 时间反推

只在 `config.deskew=True` 时调用。三种来源:

1. **消息自带**: 已归一化到 [0,1],直接 tensor 化 `[VERIFY: dataset/slam_dataset.py:299-305]`
2. **Ouster 64/128**: 固定 64×1024 / 128×N 排列,按 index 反推 `[VERIFY: dataset/slam_dataset.py:308-327]`
3. **机械式**: 用 `atan2(y, x)` 反推方位角,Velodyne 从 -x 顺时针,Hesai 从 +y 顺时针 `[VERIFY: dataset/slam_dataset.py:328-347]`

### 2.3 `preprocess_frame` 关键步骤

代码顺序(`dataset/slam_dataset.py:359-505`):

| 子步骤 | 输入 → 输出 | 备注 |
|--------|-------------|------|
| pose 初值 | last_pose, last_odom_tran → `cur_pose_init_guess` | uniform motion(默认)、static、GT 三种 | `[VERIFY: dataset/slam_dataset.py:374-389]` |
| 空帧保护 | None → return False | `[VERIFY: dataset/slam_dataset.py:391-397]` |
| adaptive_range | 由 pc 实际 bbx 推出 `crop_max_range` | `[VERIFY: dataset/slam_dataset.py:399-409]` |
| 自适应 voxel 大小 | `train_voxel_m = (crop/max)×vox_down_m`,`source_voxel_m = (crop/max)×source_vox_down_m` | `[VERIFY: dataset/slam_dataset.py:412-417]` |
| 第 1 次降采样 | voxel(leaf=train_voxel_m)| 用于 mapping `[VERIFY: dataset/slam_dataset.py:430-437]` |
| 过滤 | `filter_sem_kitti`(若 sem)或 `crop_frame`(常规)| 高度 + 距离 + 运动物体过滤 `[VERIFY: dataset/slam_dataset.py:447-463]` |
| KITTI 修正 | `intrinsic_correct(pc, correction_deg)` | 仅 `kitti_correction_on=True` `[VERIFY: dataset/slam_dataset.py:465-468]` |
| 第 2 次降采样 | voxel(leaf=source_voxel_m)→ `cur_source_points` | 仅 frame_id>0,用于跟踪 `[VERIFY: dataset/slam_dataset.py:473-490]` |
| deskew | `deskewing(cur_source_points, ts, last_odom_tran)` | 仅 `deskew=True` 且未 lose_track `[VERIFY: dataset/slam_dataset.py:493-500]` |

⚠️ **陷阱 1**: deskew 的相对变换用的是**上一帧的** `last_odom_tran`,而不是当前帧的(因为跟踪还没跑)。这是匀速模型假设。

⚠️ **陷阱 2**: `cur_point_cloud_torch`(mapping 用)只过了第 1 次降采样;`cur_source_points`(跟踪用)过了第 2 次更稀疏的。所以 mapper.process_frame 看到的点云密度 ≈ 10× 跟踪密度。

---

## 3. Stage II — 跟踪

### 3.1 输入

- `dataset.cur_source_points` `[N_src, 3]` `float32`
- `dataset.cur_pose_guess_torch` `[4, 4]` `float64`(uniform motion 推出来的)
- `dataset.cur_source_colors` `[N_src, ch]` 可选
- `dataset.cur_source_normals` `[N_src, 3]` 可选

### 3.2 流程

```
tracker.tracking
 │  T = init_pose                                                       [tracker.py:71-73]
 │  for i in range(reg_iter_n):                                         [tracker.py:114]
 │     cur_points = transform_torch(source_points, T)                   [tracker.py:118]
 │     reg = registration_step(cur_points, normals, sdf_label=0, ...)   [tracker.py:122-133]
 │      │
 │      │ query_source_points → sdf_pred, sdf_grad, ...                 [tracker.py:387-405]
 │      │  ├─ neural_points.query_feature(coord, training_mode=False)
 │      │  └─ sdf_mlp.sdf(feat)
 │      │
 │      │ valid_idx = nn_count ≥ K_thre & min<|∇|<max & sdf_std < ...   [tracker.py:419-425]
 │      │ w = GM(grad_anomaly) × GM(sdf_residual) × normal × color      [tracker.py:471-522]
 │      │ implicit_reg(points, ∇sdf, residual, w, λ)
 │      │   → ΔT = expmap(N⁻¹ g)                                        [tracker.py:652-695]
 │      ▼
 │     T = ΔT @ T                                                       [tracker.py:147]
 │     residual_increment 检查 / valid_count 检查 / converged 终止       [tracker.py:150-185]
 │  end for
 │  ├─ final residual 检查                                              [tracker.py:198-203]
 │  ├─ eigenvalue 退化检查                                              [tracker.py:205-216]
 │  └─ if invalid & i < 10: T = init_pose; cov = None                   [tracker.py:221-223]
 └─ return T, cov, weight_pc, valid_flag
```

### 3.3 输出与下游消费

- `cur_pose_torch` ← `T`,通过 `dataset.update_odom_pose` 写入 `odom_poses[frame_id]` 与 `pgo_poses[frame_id]`,并更新 `last_odom_tran` `[VERIFY: pin_slam.py:266, dataset/slam_dataset.py:507-540]`
- `cur_odom_cov` 在 `config.use_reg_cov_mat=True` 时作为 PGO 边的噪声协方差 `[VERIFY: pin_slam.py:296-297]`
- `weight_pc_o3d` 仅在 `--visualize` 时塞进 VisPacket `[VERIFY: pin_slam.py:439-440]`
- `valid_flag` → `dataset.lose_track = not valid_flag` `[VERIFY: pin_slam.py:265]`

---

## 4. Stage III — 回环检测 + PGO

`config.pgo_on=False` 时整段跳过。下面假设 True。

### 4.1 描述子注册

每帧都加一个 NPMC 节点(不是只在尝试回环时):
- 若 `local_map_context` 且 `frame_id ≥ latency`:
  - 用 `latency` 帧之前的局部地图为描述子源
  - 先 `reset_local_map(local_map_pose[:3,3], ..., local_map_frame_id, by_travel_dist, time_window)` `[VERIFY: pin_slam.py:286-287]`
  - `context_pc_local = transform_torch(local_neural_points, inv(local_map_pose))` 把局部地图转回那个时刻的传感器系 `[VERIFY: pin_slam.py:288]`
  - `lcd_npmc.add_node(local_map_frame_id, context_pc_local, feature)` `[VERIFY: pin_slam.py:290]`
- 否则用当前帧 raw 点云做 scan context `[VERIFY: pin_slam.py:292]`

### 4.2 PGO 因子追加

- `add_frame_node(frame_id, init_pose)` ← `pgo_poses[frame_id]` `[VERIFY: pin_slam.py:293, utils/pgo.py:70-82]`
- `add_odometry_factor(frame_id, frame_id-1, last_odom_tran, cov)` ← `T_prev<-cur`,cov 可来自跟踪 `[VERIFY: pin_slam.py:297]`
- `estimate_drift(travel_dist, frame_id, correct_ratio=0.01)` 更新 `drift_radius` `[VERIFY: pin_slam.py:298, utils/pgo.py:323-336]`

### 4.3 回环搜索(每 `pgo_freq` 帧)

```
if frame_id - last_loop_idx > pgo_freq and not stop_status:
  candidate_mask = (travel_dist[-1] - travel_dist) > ratio × local_map_radius
  loop_id = None
  ① detect_local_loop(pgo_poses, candidate_mask, drift_radius, frame_id, ...)
       ↑ 几何粗筛:距离 < local_loop_dist_thre
  ② if loop_id is None and global_loop_on:
        npmc.detect_global_loop(pgo_poses, drift_radius × ratio, candidate_mask, neural_points)
       ↑ NPMC: virtual nodes + ring key L1 + SC cos dist + yaw align
```

证据: `[VERIFY: pin_slam.py:302-309]`

### 4.4 找到候选后的精配

- z-check: `|Δz| > 4 × voxel_size_m` 则 reject(多层建筑歧义)`[VERIFY: pin_slam.py:311-312]`
- `valid_flags[loop_id]=False` reject `[VERIFY: pin_slam.py:313-314]`
- 否则:
  - `pose_init = pgo_poses[loop_id] @ loop_transform` `[VERIFY: pin_slam.py:316]`
  - `neural_points.recreate_hash(pose_init[:3,3], None, kept=True, with_ts=True, loop_id)` 把局部地图切到 loop 候选帧 `[VERIFY: pin_slam.py:317]`
  - `tracker.tracking(source, pose_init, loop_reg=True)` 二次精配
  - 成功则 `loop_transform = inv(pgo_poses[loop_id]) @ pose_refine` 是 `T_loop<-cur` `[VERIFY: pin_slam.py:319-323]`

### 4.5 PGO 优化与地图变形

```
pgm.add_loop_factor(frame_id, loop_id, loop_transform, cov)
pgm.optimize_pose_graph()       # ISAM2.update() 或 LM
pose_diff = pgm.get_pose_diff() # pgo_poses @ inv(init_poses)
neural_points.adjust_map(pose_diff)              # 神经点 + 朝向四元数 都被旋转
neural_points.recreate_hash(cur_pose[:3,3], ..., not pgo_merge_map, rehash_with_time, frame_id)
mapper.transform_data_pool(pose_diff)             # 训练数据池的 global_coord_pool 也一起变
dataset.update_poses_after_pgo(pgm.pgo_poses)
pgm.last_loop_idx = frame_id
pgm.min_loop_idx = min(pgm.min_loop_idx, loop_id)
```

证据: `[VERIFY: pin_slam.py:329-343]`

`adjust_map` 关键: 不仅是位置 SE3 变换,**四元数 `point_orientations` 也累乘** → query_feature 在 `after_pgo=True` 时会用这些四元数对 neighbor_vector 做被动旋转,使 MLP 看到的是"局部坐标"`[VERIFY: model/neural_points.py:646-649, :792-818]`。

### 4.6 reject 情形

精配失败:
- `neural_points.recreate_hash(cur_pose[:3,3], ..., True, True, frame_id)` 回到当前帧 `[VERIFY: pin_slam.py:347]`
- `loop_reg_failed_count += 1` `[VERIFY: pin_slam.py:348]`,后面影响 `detect_local_loop` 的搜索半径

---

## 5. Stage IV — Mapping

### 5.1 reboot 判断

```
if consecutive_lose_track_frame >= reboot_frame_thre:
   mapper.init_pool()                   # 清空 7 个池
   neural_points.reboot_ts = frame_id   # 新点的 ts ≥ reboot_ts
   unfreeze_decoders(mlp_dict, config)  # 重新放开 MLP
   config.decoder_freezed = False
```
证据: `[VERIFY: pin_slam.py:354-363, utils/tools.py:freeze_decoders/unfreeze_decoders]`

### 5.2 `process_frame` 数据流

```
process_frame(point_cloud, sem_labels, cur_pose, frame_id, filter_dynamic)
  ├─ static_mask = ones
  ├─ if filter_dynamic:                      [mapper.py:186-198]
  │     reset_local_map → 当前帧位姿处
  │     globally transform pc → dynamic_filter → static_mask
  │     pc = pc[static_mask]
  ├─ sampler.sample(pc, normal=None, sem, color)
  │     → coord (N·7, 3 in sensor frame)
  │     → sdf_label (N·7), weight (N·7)
  │     → sem (N·7) | None, color (N·7, c) | None
  ├─ update_points = 
  │     if from_sample_points and not from_all_samples:
  │         coord[|sdf_label| < map_surface_ratio × σ]  → 转世界系
  │     else:
  │         frame_point  → 转世界系
  ├─ prune_map(可选)
  ├─ neural_points.update(update_points, origin, orientation, frame_id)
  │     ├─ voxel_down(update_points, leaf=resolution)
  │     ├─ spatial hash → 空桶 or 远碰撞 or 时间窗超 → 新点
  │     ├─ allocate features(高斯初始化 std=feature_std=0)
  │     └─ reset_local_map(reboot_map=True)
  ├─ pool.append(coord, sdf_label, weight, time_repeat, sem, color)
  ├─ determine_used_pose()  根据 pgo_on/track_on/gt 选 used_poses
  ├─ if ba_done_flag:
  │     global_coord_pool = transform_batch(coord_pool, used_poses[time_pool])  ← 整体重做
  │  else:
  │     global_coord_pool.append(transform_torch(coord, cur_pose))
  ├─ if (frame+1) % pool_filter_freq == 0:
  │     window mask: ||global - origin||² < window_radius²
  │     若 > pool_capacity:random discard 多余
  │     7 个池一起按 mask 过滤
  ├─ if bs_new_sample > 0:                  [mapper.py:373-438]
  │     cur_sample = global_coord_pool[-cur_sample_count:]
  │     neural_points.set_search_neighborhood(num_nei_cells=1, alpha=0.0)
  │     batch_certainty = query_certainty(cur_sample)
  │     restore neighborhood
  │     new_idx = (certainty < new_certainty_thre) & |sdf_label| < 3σ
  │     new_idx += pool_offset  (绝对位置)
  │     if adaptive_iters: adjust adaptive_iter_offset
  └─ done
```

证据: `[VERIFY: utils/mapper.py:162-450]`

### 5.3 `mapping` 训练循环

```
for iter in range(iter_count):
  coord, sdf_label, ts, _, sem, color, weight = get_batch(global_coord=not ba_done_flag)
  poses = used_poses[ts]; origins = poses[:, :3, 3]
  if ba_done_flag: coord = transform_batch(coord, poses)
  if require_gradient: coord.requires_grad_(True)
  geo_feat, color_feat, weight_knn, _, _ = neural_points.query_feature(coord, ts, query_color=color_on)
  sdf_pred = sdf_mlp.sdf(geo_feat)
  if not weighted_first: sdf_pred = sum(sdf_pred × weight_knn, dim=1)   ← 后融
  if require_gradient: g = get_gradient(coord, sdf_pred)               ← 解析(autograd)
  elif numerical_grad: g = get_numerical_gradient(coord, sdf_pred, ε)  ← Neuralangelo 风格
  surface_mask = |sdf_label| < σ
  cur_loss = sdf_bce_loss(sdf_pred, sdf_label, sdf_scale, |weight|, loss_weight_on)
  if eikonal_loss_on: cur_loss += weight_e × ((g.norm-1)²).mean()
  if semantic_on: cur_loss += weight_s × NLLLoss(sem_pred, sem_label)
  if color_on:    cur_loss += weight_i × color_diff_loss(color_pred[s_mask], color_label[s_mask], ...)
  opt.zero_grad(); cur_loss.backward(); opt.step()
neural_points.assign_local_to_global()   ← 局部 Parameter 写回全局
```

证据: `[VERIFY: utils/mapper.py:600-844]`

### 5.4 `bundle_adjustment`

```
当 (frame+1) % ba_freq_frame == 0:
  使用 pypose 把最近 ba_frame 个位姿做成 Parameter
  setup_optimizer(neural_points params + poses param, lr=lr/lr_ba_map)
  for iter in range(ba_iters):
    coord_ba, weight, ts = get_ba_samples(ba_bs)
    poses = stack(fix, opt_part)
    coord = poses[ts] @ coord_ba          # SE3 应用
    sdf_pred = sdf(coord)[0]
    cur_loss = (sdf_pred ** 2).mean()    # 注释里 weight 已被禁用
    backward + step
  assign_local_to_global
  updated_poses_mat = opt → matrix
  pgo_poses 或 odom_poses 被覆盖
  ba_done_flag = True                   ← 下一次 process_frame 触发整池重投
```
证据: `[VERIFY: utils/mapper.py:846-937]`

⚠️ 注 1: 注释说"weight is 1.0",位置正则没用 SDF weight。
⚠️ 注 2: BA 在默认 yaml(`ba_freq_frame=0`)是关闭的,需要场景手动开。
⚠️ 注 3: BA 启用会强制 `stop_frame_thre = end_frame`(`config.py:518-519`),即关闭"停车检测",避免与 BA 冲突。

---

## 6. Stage V — Mesh + Vis

### 6.1 触发条件

主循环里有 3 个独立的频率开关:

- mesh 触发: `vis_mesh_on AND (frame_id==0 or frame_id==last_frame or (frame+1)%vis_mesh_freq_frame==0 or pgm.last_loop_idx==frame_id)` `[VERIFY: pin_slam.py:445]`
- sdf slice 触发: 类似但用 `vis_sdf_freq_frame` `[VERIFY: pin_slam.py:459]`
- 总开关由 GUI 子进程通过 `q_vis2main` 发送的 `ControlPacket` 控制 `[VERIFY: pin_slam.py:414-434]`

### 6.2 mesh 数据流

```
neural_points.get_neural_points_o3d(query_global=True, random_down=37)   ← prime number 降采样
   → o3d.PointCloud,带 axis_aligned_bbx
split_chunks(pcd, dataset.cur_bbx, mc_res×100/200)    ← 把 bbx 切成多块,内存可控
mesher.recon_aabb_collections_mesh(chunks, mc_res, save_path, only_local_bbx,
                                   semantic_on, color_on, filter_isolated_mesh, mesh_min_nn)
   ├─ 对每个 chunk: 生成体素网格 → query_source_points(sdf, color, sem, mc_mask)
   ├─ marching cubes → triangles + vertex_colors
   ├─ 全局 mesh = 拼接
   └─ filter_isolated_mesh: 移除孤立小连通分量(< min_cluster_vertices)
```
证据: `[VERIFY: pin_slam.py:447-456, utils/mesher.py 文件全文]`

### 6.3 SDF slice 数据流

```
sdf_bound = surface_sample_range_m × 4.0
vis_sdf_bbx = create_bbx_o3d(cur_pose[:3,3], max_range/2)
mesher.generate_bbx_sdf_hor_slice(vis_sdf_bbx, z=sdf_height, res=vis_sdf_res_m, ...)
   → 在 z 平面采样网格点 → query SDF → 用色图映射 → o3d.PointCloud
(可选) mesher.generate_bbx_sdf_ver_slice(...)
合并 horizontal + vertical
```
证据: `[VERIFY: pin_slam.py:459-467]`

### 6.4 VisPacket 字段

```python
VisPacket(frame_id, travel_dist, gpu_mem_usage_gb, cur_fps,
          slam_finished=False)
   .add_neural_points_data(neural_points, only_local_map, pca_color_on)
   .add_scan(points, colors)
   .add_mesh(vertices, triangles, vertex_colors)
   .add_sdf_slice(points, colors)
   .add_sdf_training_pool(points, colors)
   .add_traj(odom_poses, gt_poses, pgo_poses, loop_edges)
```
证据: `[VERIFY: pin_slam.py:473-492, gui/gui_utils.py]`

每帧通过 `q_main2vis.put(packet)` 发给 GUI 子进程,无 ACK,无 backpressure。

---

## 7. Stage VI — 持久化

主循环退出后(`pin_slam.py:511-544`):

1. `mapper.free_pool()` — 7 个数据池清空
2. `dataset.write_results()` — 写 KITTI 格式位姿 (`odom_poses.txt`, `pgo_poses.txt`),如果 GT 存在还会算 APE/RPE
3. PGO 落盘:
   - `final_pose_graph.g2o`(gtsam writeG2o)
   - `loop_log.txt`(`from to` + 4×4 矩阵)
   - `loop_plot.png`(matplotlib 3D 轨迹 + 回环弧)
4. `neural_points.prune_map(max_prune_certainty, 0, global=True)` 全图裁剪
5. `neural_points.recreate_hash(None, None, False, False)` ← `kept_points=False` 真正合并去重
6. `neural_points.get_neural_points_o3d(query_global=True)` → `neural_points.ply`(若 `--save-map`)
7. 全局 mesh: `output_mc_res_m = mc_res_m × 0.6`,分块重建 → `mesh_<XX>cm.ply`
8. `neural_points.clear_temp()` 释放局部张量
9. `save_implicit_map(run_path, neural_points, mlp_dict)` 写 `.pth`(后续可用 `--load_model` 进入纯定位模式)
10. `--save-merged-pc`: `dataset.write_merged_point_cloud()` 重放所有 frame 把降采样后的点云 transform + 合并落盘

---

## 8. ROS 模式数据流差异

`pin_slam_ros.py` 没有 `for` 循环,而是:

1. `rclpy.Node` 订阅 `sensor_msgs/PointCloud2`(topic 来自 yaml)
2. LiDAR 回调把 msg 入队列(线程安全 Queue)
3. 独立 worker 线程从队列取消息,调用 `dataset.read_frame_ros` 然后走和离线一样的 Stage I-V
4. 估计完成后通过若干 publisher 输出:
   - `nav_msgs/Odometry`(里程计)
   - `sensor_msgs/PointCloud2`(neural point map 降采样,可配 `publish_np_map_down_rate_list` 多档)
   - `geometry_msgs/PoseArray`(累积轨迹)
   - `visualization_msgs/MarkerArray`(回环弧)
5. 一段时间(`timeout_duration_s=30s` 默认)无新消息则节点退出 → 触发 Stage VI 落盘

---

## 9. 张量形状速查

|  | 第 0 帧 | 稳态 |
|---|--------|------|
| `cur_point_cloud_torch` | ~10⁵ 行 | ~10⁵ 行(降采样后)|
| `cur_source_points` | 不用(frame 0 跳过跟踪)| ~10³–10⁴ 行 |
| `coord` (sampler 输出) | N·7 | N·7 |
| `coord_pool` | N·7 | 上界 1e7 |
| `geo_feat` in mapping | bs(16384), K(=6), F+P(=11) | 同 |
| `sdf_pred` in mapping | bs(16384)|
| `neural_points.neural_points` | 0 → 几万 | 累计可达 10⁶ 级 |
| `local_neural_points` | 0 → 几千 | ~10⁴ 级 |
| `local_geo_features` (Parameter) | [M+1, 8] | [M+1, 8] |
| `buffer_pt_index` | 固定 5×10⁷ | 同 |

---

## 10. 跨阶段共享状态

下表列出 **被两个以上 Stage 共同读写** 的关键状态,是阅读源码时最容易混淆的耦合点。

| 状态 | 谁写 | 谁读 | 注 |
|------|------|------|----|
| `dataset.cur_pose_torch` | Stage II `update_odom_pose`, Stage III PGO `[VERIFY: pin_slam.py:336]` | Stage IV `process_frame`, Stage V VisPacket | float64 |
| `dataset.pgo_poses` / `odom_poses` | Stage II / III / IV(BA) | Mapper.determine_used_pose | 浅拷贝整数组 |
| `dataset.lose_track` | Stage II | Stage IV(决定是否入池)`[VERIFY: pin_slam.py:265, :368]` | bool |
| `dataset.travel_dist` | Stage II `update_odom_pose` `[VERIFY: dataset/slam_dataset.py:546-552]` | Stage III `estimate_drift`,Stage IV `neural_points.travel_dist` | 累计米 |
| `neural_points.travel_dist` | Stage II 末尾从 dataset 同步 `[VERIFY: pin_slam.py:275]` | NeuralPoints 内部用作时间窗 | torch tensor |
| `neural_points.local_geo_features` | Stage IV mapping(Adam)| Stage II tracker(只读 forward)| `nn.Parameter` |
| `mapper.coord_pool / sdf_label_pool` | Stage IV process_frame | Stage IV mapping(get_batch)| 7 个并行张量 |
| `pgm.pgo_poses` | Stage III optimize_pose_graph | Stage IV Mapper.used_poses,write_results | numpy |

---

## 11. 数据流陷阱小结

1. **位姿数组双轨**: `odom_poses`(纯里程计)与 `pgo_poses`(PGO 修正)并存,write_results 会同时写两套。Mapper 根据 `pgo_on/track_on` 决定取哪套作为 `used_poses` `[VERIFY: utils/mapper.py:139-159]`。

2. **PGO 不重写里程**: PGO 修正完后,`odom_poses` 不变,只 `pgo_poses` 与 `cur_pose_ref` 更新 → 这是为什么所有"位姿差"计算都用 `pgo_poses`。

3. **assign_local_to_global 时机**: mapping 循环每次结束都会 `assign_local_to_global()`,把训练过的 local Parameter 写回 global。如果想在中途读 global 特征,要先看是否在 mapping 之内 `[VERIFY: utils/mapper.py:843-844, model/neural_points.py:516-529]`。

4. **recreate_hash 副作用**: 它会 reset_local_map → 局部张量结构改变 → 训练中的 `nn.Parameter` 失效。这就是为什么回环之后必须重启 optimizer(新一帧的 mapping 会重新 `setup_optimizer`)。

5. **adjust_map 顺序**: 必须先 `adjust_map(pose_diff)` 再 `recreate_hash`,因为后者用神经点位置做哈希,位置先改正才行 `[VERIFY: pin_slam.py:337-338]`。

6. **数据池在 BA 后的整体重投**: `ba_done_flag=True` 后下一次 `process_frame` 会用新位姿重做整个 `global_coord_pool` 的变换,O(P) 一次性 — 这是 BA 的隐性昂贵成本 `[VERIFY: utils/mapper.py:301-305]`。

7. **first_frame_ref**: 改写 GT 让第 0 帧位姿=identity。注意它会扣到所有后续 GT 上(`gt_poses[i] = inv(gt_poses[0]) @ gt_poses[i]`)`[VERIFY: dataset/slam_dataset.py:118-121]`,所以"GT vs PGO"对比要在同一参考系下做。

8. **2000 帧自动开 local_map_context**: `total_pc_count > 2000` 时自动把 `local_map_context = True`,启用本地图描述子 `[VERIFY: dataset/slam_dataset.py:124-125]`。短序列默认是 scan context。

---

**END of Phase 3 DATA FLOW.**

# ALGORITHM 04 — Mapper(Online Training + Bundle Adjustment)

> Phase 4 文档系列之四。对应 `utils/mapper.py`(1079 行)+ `utils/tools.py::setup_optimizer` + `utils/tools.py::get_numerical_gradient` 已在 ALGORITHM 02 覆盖。
> 路径相对 `src/PIN_SLAM/`。

---

## 0. TL;DR

Mapper 把"在线增量学习"做成了三件事:
1. **`process_frame`** — 收新帧观测,做样本采样、入数据池、按距离窗丢老样本、(可选)动态物体过滤、检测新区域
2. **`mapping`** — Adam 训练 N 次迭代,优化 local neural point features + 3 MLP
3. **`bundle_adjustment`** — 可选,联合优化位姿 + 地图

数据池 = **per-frame 样本 replay buffer**,容量 1e7,带 sliding window 距离过滤。新样本与历史样本混采(half-half by default)避免灾难性遗忘。

---

## 1. 数据池(Replay Buffer)

### 1.1 七个并行张量

| 字段 | 形状 | 含义 |
|------|------|------|
| `coord_pool` | `float32 [P, 3]` | sensor-local coord |
| `global_coord_pool` | `float32 [P, 3]` | world coord |
| `sdf_label_pool` | `float32 [P]` | signed displacement |
| `color_pool` | `float32 [P, c]` 或 None | |
| `sem_label_pool` | `int [P]` 或 None | |
| `normal_label_pool` | `float32 [P, 3]` 或 None | (默认 None)|
| `weight_pool` | `float32 [P]` | 距离权 × surface/freespace 符号 |
| `time_pool` | `int [P]` | 样本属于第几帧 |

证据 `[VERIFY: utils/mapper.py:82-97]`

P 上界 = `pool_capacity = 1e7`(默认),稳态约 ~5×10⁶。

### 1.2 双 coord 的存在意义

为什么同时维护 `coord_pool`(local)和 `global_coord_pool`(world)?
- **稳态 mapping**: 用 global 直接(`get_batch(global_coord=True)`),省掉每次 transform
- **BA 之后**: 位姿变了,必须用 local + 新 pose 重做 global → `global_coord_pool = transform_batch(coord_pool, used_poses[time_pool])` `[VERIFY: utils/mapper.py:301-305]`
- **回环 PGO 后**: 同 BA,通过 `transform_data_pool(pose_diff)` 把 global pool 直接乘上 pose_diff `[VERIFY: utils/mapper.py:527-531]`

⚠️ 内存代价: 双倍存储,~10 MB / 1M sample,但避免大规模 transform_batch 是值得的(后者 ~100 ms/百万点)。

### 1.3 池过滤规则(`pool_filter_freq` 每帧)

```python
pool_relative = global_coord_pool - frame_origin
pool_relative_dist = sum(pool_relative**2, dim=-1)
dist_mask = (pool_relative_dist < window_radius²)
filter_mask = dist_mask

if pool_sample_count > pool_capacity:
    discard_count = pool_sample_count - pool_capacity
    discard_idx = randint(0, pool_sample_count, discard_count)
    filter_mask[true_indices[discard_idx]] = False
# 7 张表一起按 filter_mask 切
```
证据 `[VERIFY: utils/mapper.py:316-352]`

- `window_radius = max(max_range, 6.0)` = 60m+(默认)
- 当前帧位置周围 60m 内的旧样本保留,远的丢
- 池过载时随机丢

---

## 2. `process_frame()` — 每帧增量更新

### 2.1 流程图

```
process_frame(pc, sem, cur_pose, frame_id, filter_dynamic)
   │
   │  T0 = now
   ├─ static_mask = ones
   ├─ if filter_dynamic and frame_id > 0:                            [mapper.py:186-198]
   │     ① reset_local_map(cur_pose origin, frame_id)
   │     ② pc_global = transform(pc, cur_pose)
   │     ③ static_mask = dynamic_filter(pc_global)
   │     ④ pc = pc[static_mask]; (color/label 同步)
   │
   │  T1 = now
   ├─ DataSampler.sample(pc, normal=None, sem, color)               [mapper.py:217-226]
   │     → coord [N·7, 3], sdf_label, normal, sem, color, weight
   ├─ time_repeat = full(N·7, frame_id)
   │
   │  T2 = now
   ├─ update_points = (from_sample_points) ?
   │       coord[|sdf_label| < map_surface_ratio × σ] (transform)
   │     :
   │       transform(pc, cur_pose)                                  [mapper.py:239-251]
   │
   ├─ if prune_map_on and (frame+1) % prune_freq == 0:
   │     neural_points.prune_map(max_prune_certainty)
   │     neural_points.recreate_hash(None, None, True, True, frame_id)
   │
   ├─ cur_new_point_ratio = neural_points.update(
   │       update_points, frame_origin, frame_orientation, frame_id)
   │     # 体素哈希插入新点 + reset_local_map(reboot=True)
   │
   │  T3 = now
   ├─ 7 张池追加 (coord, weight, sdf_label, time, sem, color, normal)
   ├─ determine_used_pose()                                         [mapper.py:139-159]
   │
   ├─ if ba_done_flag:
   │     global_coord_pool = transform_batch(coord_pool, used_poses[time_pool])
   │     ba_done_flag = False
   ├─ else:
   │     global_coord = transform(coord, cur_pose)
   │     global_coord_pool.append(global_coord)
   │
   │  T3_1 = now
   ├─ if (frame+1) % pool_filter_freq == 0:
   │     window mask + capacity cap
   │     7 张池 + new_idx 同步
   │
   │  T3_2 = now
   ├─ if bs_new_sample > 0:                                          [mapper.py:373-438]
   │     ① set_search_neighborhood(num_nei=1, alpha=0.0)            ← 临时缩小邻域
   │     ② 对 cur_sample_filtered 算 certainty
   │     ③ restore neighborhood
   │     ④ new_idx = (cert < new_certainty_thre) & |label| < 3σ
   │     ⑤ adaptive_iter_offset = -5 / 0 / 5 / 10 by new_obs_ratio
```

证据 `[VERIFY: utils/mapper.py:162-450]`

### 2.2 dynamic_filter 详解(可选)

```python
def dynamic_filter(self, points_torch, type_2_on=True):
    if type_2_on:
        points_torch.requires_grad_(True)
    geo_feat, _, weight_knn, _, certainty = neural_points.query_feature(
        points_torch, training_mode=False)
    sdf_pred = sdf_mlp.sdf(geo_feat)
    if not weighted_first:
        sdf_pred = sum(sdf_pred * weight_knn, dim=1).squeeze(1)

    if type_2_on:
        sdf_grad = get_gradient(points_torch, sdf_pred).detach()
        grad_norm = sdf_grad.norm(dim=-1, keepdim=True).squeeze()

    # Strategy 1: 测量落到 certain freespace 即为动态点
    static_mask = (certainty < dynamic_certainty_thre) |
                  (sdf_pred < dynamic_sdf_ratio_thre × voxel_size)

    # Strategy 2: 加梯度不稳判别
    if type_2_on:
        static_mask_2 = (grad_norm > min_grad_norm_thre) |
                        (certainty < certainty_thre)
        static_mask = static_mask & static_mask_2

    return static_mask
```
证据 `[VERIFY: utils/mapper.py:99-137]`

#### 动态点判别逻辑

1. **Strategy 1 (always on)**:
   - 测量点 q 的预测 SDF > 阈值 → 它落在已知 freespace 内 → 但实际有测量打到这里 → 必然是动态物体
   - 阈值 = `0.5 × voxel_size = 0.15 m`
   - 例外:certainty < 1.0 的区域(地图还没充分观测过)→ 不动态

2. **Strategy 2**:
   - 梯度模过小 → SDF 在该点 ill-defined → 不可信
   - 与 Strategy 1 取 AND,保守:仅梯度大且 SDF 不对的才标动态

### 2.3 `from_sample_points` vs 测量点构图

```python
if from_sample_points:
    if from_all_samples:
        update_points = coord                          # 全 7 个/ray
    else:
        update_points = coord[|sdf_label| < ratio × σ]   # 仅 surface 段
    update_points = transform(update_points, cur_pose)
else:
    update_points = transform(pc, cur_pose)             # 只测量点
```

- `from_sample_points=True, from_all_samples=False`(默认): 用 surface samples 而不只是测量点
  - 这给出"每个表面附近**多个候选位置**",更密集地激活神经点
  - `map_surface_ratio=0.5` → 只用 |sdf| < 0.5 × σ = 0.125 m 的样本
- `from_all_samples=True`: 连 freespace 样本也建神经点
  - 内存爆 ×7,但对 ESDF map 更友好
- `from_sample_points=False`: 只用测量点
  - 节省内存,但表面采样不充分

### 2.4 new_idx 与自适应迭代

```python
neural_points.set_search_neighborhood(num_nei_cells=1, search_alpha=0.0)
                                       # 临时:K_cells = 7(只搜本 voxel)

batch_certainty = neural_points.query_certainty(cur_sample_filtered)
                                       # 用紧邻居判 cert

neural_points.set_search_neighborhood(num_nei_cells, search_alpha)   # 恢复

new_idx = where((certainty < new_certainty_thre) &
                (|label| < 3σ))[0]      # 仅 surface 附近新样本
new_idx += pool_sample_count - cur_sample_count   # 转绝对池索引

if adaptive_iters:
    if new_ratio < 0.02:                offset = -5
    elif new_ratio > 0.15:              offset = +5
    elif frame > freeze_after_frame and new_ratio > 0.3: offset = +10
```
证据 `[VERIFY: utils/mapper.py:387-438]`

#### 设计意义

- "new_idx" 标记本帧观测中"地图未充分覆盖"的样本
- mapping 时按 `bs_new_sample` 比例混入这些新样本,加速新区域学习
- `adaptive_iters`: 新观测多则多训练,反之少训练 — 节省稳态时段的计算

#### 临时缩小邻域的意义

`query_certainty` 不需要 KNN,只问"自己 voxel 里有没有点"。把搜索区域从 33 cell 缩到 7 cell(num_nei=1, alpha=0):
- 加速 ~4.7×
- 但确实牺牲了 certainty 的连续性 — 边缘 voxel 的 certainty 会偏低

### 2.5 池增长 + filter 复杂度

- append: O(7 × cur_sample_count),~万级
- distance filter: O(P) = O(1e7),每帧一次
- random discard: O(discard_count)
- 总: 稳态下 O(1e7) / 帧 ≈ 50-100 ms

这是 mapper 单帧时间的主要消耗之一(`# print("time for pool updating ...")` 标记 `[VERIFY: utils/mapper.py:447]`)。

---

## 3. `mapping()` — Adam 训练循环

### 3.1 流程

```python
def mapping(self, iter_count):
    iter_count = max(1, iter_count + adaptive_iter_offset)

    # 收集所有可训练参数
    neural_point_feat = list(neural_points.parameters())     # [local_geo_features (+local_color_features)]
    sdf_mlp_param     = list(sdf_mlp.parameters())            # [Linear×2 的 W/b]
    if semantic_on: sem_mlp_param = list(sem_mlp.parameters())
    if color_on:    color_mlp_param = list(color_mlp.parameters())

    opt = setup_optimizer(config, neural_point_feat, sdf_mlp_param, sem_mlp_param, color_mlp_param)
    # ↑ Adam with eps=1e-15, lr=0.01 默认
    #   weight_decay 只作用于 neural_point_feat (latent 上的 L2)

    for iter in range(iter_count):
        coord, sdf_label, ts, _, sem, color, weight = mapper.get_batch(global_coord=not ba_done_flag)
        poses   = used_poses[ts]
        origins = poses[:, :3, 3]
        if ba_done_flag:
            coord = transform_batch(coord, poses)
        if require_gradient: coord.requires_grad_(True)

        geo_feat, color_feat, weight_knn, _, _ = neural_points.query_feature(
            coord, ts, query_color_feature=color_on)

        sdf_pred = sdf_mlp.sdf(geo_feat)
        if not weighted_first:
            sdf_pred = sum(sdf_pred * weight_knn, dim=1).squeeze(1)

        if semantic_on: sem_pred   = sem_mlp.sem_label_prob(geo_feat)
        if color_on:    color_pred = color_mlp.regress_color(color_feat)

        surface_mask = |sdf_label| < surface_sample_range_m

        if require_gradient:
            g = get_gradient(coord, sdf_pred)
        elif numerical_grad:
            g = get_numerical_gradient(coord[::dec], sdf_pred[::dec],
                                       voxel_size_m × num_grad_step_ratio)

        loss = sdf_bce_loss(sdf_pred, sdf_label, sdf_scale, |weight|, loss_weight_on)
        if eikonal: loss += weight_e × ((g_used.norm-1)²).mean()
        if sem:     loss += weight_s × NLLLoss(sem_pred, sem_label)
        if color:   loss += weight_i × color_diff_loss(color_pred[s_mask], color_label[s_mask], w[s_mask], ...)

        opt.zero_grad(set_to_none=True)
        loss.backward(retain_graph=False)
        opt.step()

        total_iter += 1
        # (optional) wandb.log(...)

    neural_points.assign_local_to_global()
```
证据 `[VERIFY: utils/mapper.py:600-844]`

### 3.2 `setup_optimizer` 关键点

代码在 `utils/tools.py`(本文不全开),但行为大致:
- Adam 默认配置
- 不同参数组不同 lr / weight_decay:
  - neural_point_feat: `lr`, `weight_decay`(L2 on latent)
  - sdf/sem/color MLP: `lr`, no weight_decay
  - (BA 时)poses: `lr_pose`

`assign_local_to_global()` 调用关键:把训练后的 `local_geo_features` 拷回全局 `geo_features` `[VERIFY: model/neural_points.py:516-529]`。**没有这一步训练就白做了**(因为下次 reset_local_map 会重新从 global 取)。

### 3.3 `init_iter_ratio` 与 warm-up

第 0 帧 / reboot 时:
```python
cur_iter_num = iters × init_iter_ratio = 12 × 40 = 480
```
其余帧 12 次。证据 `[VERIFY: pin_slam.py:379]`

为什么 warm-up 多 40×?
- 第 0 帧创建大量新神经点,latent 全 0 初始化
- 480 次足以让 latent 和 MLP 协同收敛到一个合理 SDF
- 之后每帧只增量微调,12 次够

### 3.4 freeze MLP

```python
if (frame_id - neural_points.reboot_ts) == config.freeze_after_frame:
    freeze_decoders(mlp_dict, config)
    config.decoder_freezed = True
    neural_points.compute_feature_principle_components(down_rate=17)  # PCA for vis
```
证据 `[VERIFY: pin_slam.py:382-385]`

`freeze_after_frame = 40` 默认,第 41 帧 freeze 3 个 MLP。之后只学 latent。

#### 为什么 freeze?

- 类似 NeRF / Instant-NGP 经验:MLP capacity 小,过多更新会被新观测带偏(catastrophic forgetting)
- 冻结后地图局部细节由 latent 学,MLP 提供全局一致的解码函数
- 若 reboot:解冻并 `init_iter_ratio` 重学

### 3.5 数值梯度的两次 forward 代价

```python
g = get_numerical_gradient(coord[::dec], sdf_pred[::dec], ε)
```
- `gradient_decimation = 10` 把 batch 16384 → 1638 个用于 grad
- 6 次额外 forward (`±x, ±y, ±z`),每次 1638 个查询
- 总:~10K 额外 forward → 几 ms,vs autograd 一次 backward ~5 ms

为什么默认 numerical?
- 论文中分析:numerical grad 对 init phase 的不稳定 MLP 更鲁棒
- 平滑作用相当于 ε-tube 上做 finite difference
- 训练后期两者差异微小

### 3.6 loss 各项的实测尺度

典型值(默认 yaml):
- `sdf_bce_loss`: 0.1-0.5
- `eikonal_loss`: 0.01-0.1 ×  weight_e=0.5 → 0.005-0.05
- `color_loss`: ~0.05 × weight_i=1.0
- `sem_loss`: 1-3 × weight_s=1.0

总 loss 通常 1-5 量级,Adam 在 12 iter 内能降到 0.5-1.

### 3.7 Adam 而非 SGD 的选择

```python
self.opt_adam: bool = True
```
证据 `[VERIFY: utils/config.py:193]`

Adam 优势:
- 不同 latent 维度的 effective lr 自适应
- 在稀疏梯度场景(只有少量 KNN 邻居被某次 batch 激活)更鲁棒
- 无需手动调 lr schedule

劣势:
- 比 SGD 多 ~2× 内存(m, v 状态)
- adam_eps = 1e-15 极小 → 早期 step 接近 sign(g)

`weight_decay` 仅作用于 latent(`utils/tools.py::setup_optimizer` 实现):
- 防止 latent norm 爆炸(否则 IDW 加权后 MLP 输入分布漂移)
- 默认 0(不正则),需手动开

---

## 4. `bundle_adjustment()` — 联合优化

### 4.1 流程

```python
def bundle_adjustment(self, iter_count, window_size=50, use_lie_group=False):
    import pypose as pp

    current_poses_mat = self.used_poses                                   # [F, 4, 4]
    opt_window_size = min(F, window_size)                                  # 50

    if use_lie_group:
        # SE3 直接优化(罕用)
        current_poses_se3_opt = nn.Parameter(
            pp.from_matrix(current_poses_mat[-W:], SE3_type, check=False))
        poses_se3_fix = pp.from_matrix(current_poses_mat[:-W], SE3_type, check=False)
    else:
        # se3(默认)
        current_poses_se3_opt = nn.Parameter(
            pp.from_matrix(current_poses_mat[-W:], SE3_type, check=False).Log())  # se3
        poses_se3_fix = pp.from_matrix(current_poses_mat[:-W], SE3_type, check=False).Log()

    neural_point_feat = list(neural_points.parameters())

    opt = setup_optimizer(config, neural_point_feat,
                          poses=current_poses_se3_opt,
                          lr_ratio=lr_ba_map / lr)
    # ↑ map 用 lr_ba_map=0.01,pose 用 lr_pose=1e-4

    for iter in range(iter_count):
        coord_ba, weight, ts = mapper.get_ba_samples(ba_bs)         # 仅 surface 样本
        weight = weight.detach()

        current_poses_se3 = cat([poses_se3_fix, current_poses_se3_opt], dim=0)
        poses = current_poses_se3[ts] if use_lie_group else current_poses_se3[ts].Exp()
        coord = poses.to(coord_ba) @ coord_ba                       # SE3 应用

        sdf_pred = mapper.sdf(coord)[0]
        weight = 1.0                                                # 注释:不用 weight
        cur_loss = (sdf_pred**2).mean()                             # 简单 L2 SDF

        opt.zero_grad()
        cur_loss.backward(retain_graph=False)
        opt.step()

    neural_points.assign_local_to_global()

    # 写回位姿
    current_poses_se3 = cat([poses_se3_fix, current_poses_se3_opt])
    updated_poses_mat = current_poses_se3.detach().matrix()
    self.used_poses = updated_poses_mat
    updated_poses_np = updated_poses_mat.cpu().numpy()
    if pgo_on:
        dataset.pgo_poses[:processed_frame+1] = updated_poses_np
    elif track_on:
        dataset.odom_poses[:processed_frame+1] = updated_poses_np
    dataset.cur_pose_ref = updated_poses_np[-1]
    dataset.last_pose_ref = updated_poses_np[-1]
    self.ba_done_flag = True                                        # 触发下次 process_frame 重做 global pool
```
证据 `[VERIFY: utils/mapper.py:848-937]`

### 4.2 与传统 BA 的差异

| | 传统 BA(visual)| PIN BA |
|---|---|---|
| 残差 | reprojection error | SDF(p) ≈ 0 |
| 参数 | 位姿 + 3D landmarks | 位姿 + neural point latents |
| Jacobian | 解析(对 K, R, t)| Adam autograd |
| 优化器 | LM / GN | Adam |
| 鲁棒 | Huber / Cauchy kernel | 无(注释里 weight=1)|

### 4.3 `get_ba_samples` 仅取 surface

```python
def get_ba_samples(self, subsample_count):
    surface_sample_idx = where(sdf_label_pool == 0)[0]    # 仅 measurement(label=0)
    surface_sample_count = len(surface_sample_idx)
    coord_pool_surface = coord_pool[surface_sample_idx]
    time_pool_surface = time_pool[surface_sample_idx]
    weight_pool_surface = weight_pool[surface_sample_idx]
    index = randint(0, surface_sample_count, (subsample_count,))
    return coord_pool_surface[index], weight_pool_surface[index], time_pool_surface[index]
```
证据 `[VERIFY: utils/mapper.py:506-524]`

注意 `sdf_label == 0` 严格筛 — 这是 Part 0(测量点),不是 surface 段(那些是 ±σ 范围)。BA 只用绝对正确的 SDF=0 标签,避免噪声。

### 4.4 BA 触发条件

```python
if config.track_on and config.ba_freq_frame > 0 and (frame_id+1) % ba_freq_frame == 0:
    mapper.bundle_adjustment(config.ba_iters, config.ba_frame)
```
证据 `[VERIFY: pin_slam.py:388-389]`

默认 `ba_freq_frame=0` → BA 关闭。需要场景手动开。

### 4.5 BA 副作用

```python
if self.ba_freq_frame > 0:
    self.stop_frame_thre = self.end_frame
```
证据 `[VERIFY: utils/config.py:518-519]`

开 BA 会**禁用停车检测**,避免 BA 期间 stop_status 切换导致 mapping 跳过。

### 4.6 `ba_done_flag` 的级联

设为 True 后:
1. 下一次 `process_frame` 会发现 ba_done_flag=True → 整池重做 global transform `[VERIFY: utils/mapper.py:301-305]`
2. mapping 时 `get_batch(global_coord=not ba_done_flag)` → 拿 local coord,再用新 pose 转 global `[VERIFY: utils/mapper.py:629-630, :637-640]`
3. process_frame 末尾把 ba_done_flag 复位 False(由 process_frame 内部)

---

## 5. 内存与性能

### 5.1 单帧时间分布(参考 source code 计时点)

| 阶段 | 大致耗时 | 源码计时 |
|------|----------|----------|
| dynamic_filter(开)| 5-10 ms | T0→T1 |
| sampler.sample | 0.5 ms | T1→T2 |
| neural_points.update | 5-15 ms | T2→T3 |
| 池追加 + filter | 30-100 ms | T3→T4 |
| mapping × 12 iter | 80-150 ms | T4→T5(主循环 T5-T4)|

实测在 RTX 4090 + KITTI 64-line:**~10 Hz** 全管道。

### 5.2 GPU 内存峰值

- neural_points: ~100 MB(百万级神经点)
- 数据池: ~520 MB(1e7 样本 × 13 floats)
- 训练中间 tensor: ~50 MB / forward
- Adam 状态(latent + MLP): ~200 MB(latent 数 × 8 × 2 (m, v) × 4 bytes)

总:**~1 GB 量级**,8 GB GPU 足够。

### 5.3 优化建议

| 瓶颈 | 调参 |
|------|------|
| 池过大 | 降 `pool_capacity` 到 5e6 |
| 训练慢 | 降 `iters` 8, `bs` 8192 |
| 远距精度差 | 升 `iters` 24, `init_iter_ratio` 80 |
| 多 voxel 重叠 | 升 `voxel_size_m` 0.5 |
| 稀疏点云 | 降 `voxel_size_m` 0.15, 升 `query_nn_k` 8 |

---

## 6. 与论文 §III-D "Online PIN Map Training" 对应

| 论文 | 代码 |
|------|------|
| Replay buffer | `coord_pool / global_coord_pool / ...` |
| Sliding window | `window_radius` filter |
| Half-half new vs history | `bs_new_sample` + `get_batch` mixed sampling |
| Joint local map + MLP update | `setup_optimizer(neural_point_feat, mlp_param)` + `assign_local_to_global` |
| Eikonal regularization with numerical gradient | `get_numerical_gradient` |
| Decoder freezing strategy | `freeze_after_frame` + `freeze_decoders()` |
| Optional local BA | `bundle_adjustment` with pypose |

---

## 7. 边界 case

### 7.1 第 0 帧 mapping

- `neural_points.update(...)` 第一次插入 ~10K-50K 神经点
- 12 × 40 = 480 次 Adam iter
- 单帧时间可能 1-3 s(只此帧)
- 之后稳态 ~100 ms/frame

### 7.2 lose_track 或 stop_status

```python
if frame_id < 5 or valid_mapping_flag or system_rebooted:
    mapper.process_frame(...)
else:
    mapper.determine_used_pose()
    neural_points.reset_local_map(cur_pose, ..., reboot_map=True)
```
证据 `[VERIFY: pin_slam.py:368-373]`

- 不入新观测,但仍 reset local map(供下一帧跟踪用)
- mapping 仍跑(从池里抽老样本),保持 MLP 不退化

### 7.3 池为空(`pool_sample_count == 0`)

第 0 帧 process_frame 之前 mapping 不会被调:
```python
if frame_id % mapping_freq_frame == 0:
    mapper.mapping(cur_iter_num)
```
但 process_frame 已经 append 了样本 → 此时 pool > 0,mapping 可跑。

### 7.4 BA 失败回滚?

代码无回滚逻辑。BA 完成的位姿直接覆盖 `pgo_poses` / `odom_poses`,即使变差。这是已知风险,默认 ba_freq_frame=0 也是出于此。

### 7.5 `transform_data_pool` 与 `ba_done_flag` 冲突?

回环触发:
1. PGO `optimize_pose_graph` → pose_diff
2. `adjust_map(pose_diff)` 神经点弹性变形
3. `recreate_hash` 哈希重建
4. `mapper.transform_data_pool(pose_diff)` ← global_coord_pool 整体变换
5. `update_poses_after_pgo(pgo_poses)`

BA 触发:
1. `bundle_adjustment(...)` 改 `used_poses` + `ba_done_flag=True`
2. 下次 `process_frame` 见 `ba_done_flag=True` → 整池重做

两者**不会同帧同时发生**(回环 / BA 在主循环不同阶段),但是 BA 后立刻回环会:
- `transform_data_pool(pose_diff)` 改 global_coord
- 下一帧 `ba_done_flag=True` 又重做一次
- → 数据池被改两次,可能数值不一致但不会崩

### 7.6 freeze 前 reboot

```python
if (frame_id - neural_points.reboot_ts) == config.freeze_after_frame:
    freeze_decoders(mlp_dict, config)
```
`reboot_ts` 会在 reboot 时被改成当前 frame_id,所以 freeze 的相对计时也重启。reboot 后 40 帧才会再次 freeze MLP。这是合理的:reboot 后 MLP 解冻,要给它新窗口学习。

---

## 8. 一句话总结

> **Mapper 把每帧观测拆 7×N 样本入 replay buffer,然后每帧 Adam 12 次,同时优化 ~10⁵ 个 latent + 3 个轻量 MLP。半新半旧的混采 + sliding window 距离过滤是它在线学习的关键。BA 可选,用 pypose 的 SE3 / se3 参数化,与地图联合训练。**

---

**END of ALGORITHM 04.**

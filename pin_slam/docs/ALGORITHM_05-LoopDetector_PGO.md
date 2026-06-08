# ALGORITHM 05 — Loop Detector + PGO

> Phase 4 文档系列之五。对应 `utils/loop_detector.py`(606 行)+ `utils/pgo.py`(401 行)。
> 路径相对 `src/PIN_SLAM/`。

---

## 0. TL;DR

PIN-SLAM 后端由两部分组成:

1. **Loop Detector**(`NeuralPointMapContextManager`,简称 NPMC)— 把每帧(或滞后几帧的局部地图)编码成 [20×60] 极坐标格 + ring-key 向量,新帧通过 ring-key L1 粗筛 + 完整 SC cos 距离精筛找候选,再用 yaw alignment 给出 SE(3) 初始变换
2. **Pose Graph Optimizer**(`PoseGraphManager`)— 基于 gtsam,ISAM2 增量或 LM 批量优化 6-DoF 位姿图,节点 = 每帧位姿,边 = odom(里程计相对变换) + loop(回环相对变换)

回环验证通过后立即:
- 对神经点做 `adjust_map(pose_diff)` 弹性变形(ALGORITHM 01 §9)
- 对数据池做 `transform_data_pool(pose_diff)`
- 写回所有历史位姿到 `dataset.pgo_poses`

---

## 1. Scan Context 数学

### 1.1 极坐标格化

输入: 点云 `P` ∈ R^{N×3}(传感器系)
输出: SC 矩阵 `M` ∈ R^{R×S},R=20, S=60(默认)

```
foreach point (x, y, z):
    r = √(x² + y²)
    if r >= max_length: skip
    θ = atan2(y, x) ∈ [-π, π]
    θ_deg = θ × 180/π + 180 ∈ [0, 360]
    i_ring = clamp(r // (max_length / R), 0, R-1)
    j_sec  = clamp(θ_deg // (360 / S), 0, S-1)
    M[i_ring, j_sec] = max(M[i_ring, j_sec], z)
```

代码:
```python
def ptcloud2sc_torch(ptcloud, pt_feature, sc_shape, max_length):
    r = torch.norm(ptcloud, dim=1)
    kept_mask = r < max_length
    points = ptcloud[kept_mask]
    r = r[kept_mask]
    num_ring, num_sector = sc_shape    # 20, 60
    gap_ring = max_length / num_ring
    gap_sector = 360.0 / num_sector
    sc = torch.zeros(num_ring * num_sector, ..., device=...)
    theta = torch.atan2(points[:,1], points[:,0])
    theta_degrees = theta * 180/π + 180
    idx_ring   = torch.clamp((r // gap_ring).long(), 0, num_ring-1)
    idx_sector = torch.clamp((theta_degrees // gap_sector).long(), 0, num_sector-1)
    grid_indices = idx_ring * num_sector + idx_sector
    sc = sc.scatter_reduce_(dim=0, index=grid_indices, src=points[:,2],
                            reduce='amax', include_self=False)
    sc = sc.view(num_ring, num_sector)
    # 若带 feature:类似 scatter_reduce_(reduce='mean')
    return sc, sc_feature
```
证据 `[VERIFY: utils/loop_detector.py:482-545]`

#### 关键点

- **存最大 z**: 每格只记录最高点的高度,捕获建筑 / 树等垂直结构
- **不带 feature 的 SC**: shape `[R, S]`,纯几何
- **带 feature 的 SC_feature**: shape `[R, S, F]`,每格用 `mean` 聚合 latent
- `scatter_reduce_` GPU 非确定性,但实测影响小

### 1.2 Ring Key

```python
def sc2rk(sc):
    return torch.mean(sc, dim=1)        # [R]
```
证据 `[VERIFY: utils/loop_detector.py:548-549]`

每个 ring 跨所有 sector 取均值 → `[R]` 维向量。**对 sector 旋转不变**(yaw 不影响 ring key,因为 mean 与 shift 无关)。

### 1.3 SC 旋转对齐距离

```python
def distance_sc_torch(sc1, sc2):
    num_sectors = sc1.shape[1]
    sim_for_each_cols = torch.zeros(num_sectors)
    for i in range(num_sectors):
        sc1 = torch.roll(sc1, 1, 1)      # shift 1 sector
        cossim = F.cosine_similarity(sc1, sc2, dim=0)    # [S]
        sim_for_each_cols[i] = mean(cossim)
    yaw_diff = argmax(sim_for_each_cols) + 1
    sim = max(sim_for_each_cols)
    return (1 - sim).item(), yaw_diff.item()
```
证据 `[VERIFY: utils/loop_detector.py:552-576]`

- 对 sc1 做 0..S-1 次循环移位,每次算 sc1 vs sc2 的 ring-wise cos 相似度,取均值
- 最大相似度对应的 shift = yaw 差(以 sector 为单位)
- 返回 cos 距离(1 - sim)

复杂度: `O(S × R)` per shift × `S` shifts = `O(S² × R) = 60 × 60 × 20 = 72000`。GPU 上 < 1 ms。

### 1.4 SC + feature 距离

```python
def distance_sc_feature_torch(sc1, sc2):    # R×S×D
    ...
    for i in range(num_sectors):
        sc1 = torch.roll(sc1, 1, 1)
        cossim = F.cosine_similarity(sc1.view(num_rings, -1),
                                     sc2.view(num_rings, -1), dim=0)
        sim_for_each_cols[i] = mean(cossim)
    yaw_diff = argmax + 1
    sim = max
    return 1 - sim, yaw_diff
```
证据 `[VERIFY: utils/loop_detector.py:579-606]`

把 `[R, S, D]` reshape 成 `[R, S×D]`,然后和 `distance_sc_torch` 一样的 cos 距离逻辑。这样每个 sector 内 D 维 feature 被当成"一个长向量"参与匹配。

---

## 2. NPMC 主流程

### 2.1 `add_node`

```python
def add_node(self, frame_id, ptcloud, ptfeatures=None, valid_flag=True):
    sc, sc_feature = ptcloud2sc_torch(ptcloud, ptfeatures, des_shape, max_length)
    rk = sc2rk(sc)
    self.curr_node_idx = frame_id
    self.contexts[frame_id] = sc
    self.ringkeys[frame_id] = rk
    self.valid_flags[frame_id] = valid_flag

    if sc_feature is not None:
        rk_feature = sc2rk(sc_feature)
        self.contexts_feature[frame_id] = sc_feature
        self.ringkeys_feature[frame_id] = rk_feature

    self.query_contexts = []     # 重置虚拟节点队列
    self.tran_from_frame = []
```
证据 `[VERIFY: utils/loop_detector.py:59-80]`

每帧都加(无论是否当前要查回环),后续可被任何更晚的帧搜索。

### 2.2 `set_virtual_node` — 虚拟位置增广

为了对**横向小偏移**鲁棒,生成当前帧周围 11 个(默认)虚拟位置的描述子。

```python
def set_virtual_node(self, ptcloud_global, frame_pose, last_frame_pose, ptfeatures=None):
    # 计算移动方向 + 侧向方向
    if last_frame_pose is not None:
        tran_dir = frame_pose[:3,3] - last_frame_pose[:3,3]
        tran_dir_unit = tran_dir / tran_dir.norm()
        lat_rot = [[0,-1,0],[1,0,0],[0,0,1]]      # 绕 +z 旋 90°
        lat_dir_unit = lat_rot @ tran_dir_unit    # 侧向单位向量
    else:
        lat_dir_unit = [0, 1, 0]

    dx = arange(-virtual_side_count, virtual_side_count+1) × virtual_step_m
                                                  # default [-5,-4,...,5] × 2m = -10..10 m
    lat_tran = dx.view(-1,1) @ lat_dir_unit.view(1,3)    # [11, 3]

    virtual_positions = frame_pose[:3,3] + lat_tran      # [11, 3]

    for idx in range(11):
        cur_lat_tran = lat_tran[idx]
        cur_tran_from_frame = eye(4)
        cur_tran_from_frame[:3, 3] = cur_lat_tran
        cur_virtual_pose = frame_pose @ inv(cur_tran_from_frame)
                        # = T_w<-c'
        if |cur_lat_tran| == 0:                  # 中心位置,直接复用原 SC
            cur_sc = contexts[curr_node_idx]
        else:                                    # 把全局点云投到虚拟位姿系下重算 SC
            ptcloud = transform(ptcloud_global, inv(cur_virtual_pose))
            cur_sc, cur_sc_feature = ptcloud2sc_torch(...)

        self.query_contexts.append(cur_sc)
        self.tran_from_frame.append(cur_tran_from_frame)
```
证据 `[VERIFY: utils/loop_detector.py:83-155]`

#### 几何意义

回环时车辆可能与上次经过的位置有几米横向偏差(车道差异)。虚拟节点在 `[-10, +10]` m 横向区间生成 11 个候选 SC,每个 SC 都参与匹配 — 自然吸收横向 offset。

代价:11× SC 计算 + 11× cos dist 匹配,约 10 ms 额外。

### 2.3 `detect_global_loop` 主入口

```python
def detect_global_loop(self, cur_pgo_poses, dist_thre, loop_candidate_mask,
                       neural_points, dist_filter=True):
    # ① 距离粗筛
    if dist_filter:
        dist_to_past = norm(cur_pgo_poses[:,:3,3] - cur_pgo_poses[curr_node_idx,:3,3], axis=1)
        dist_search_mask = (dist_to_past < dist_thre)
        global_loop_candidate_idx = where(loop_candidate_mask & dist_search_mask)[0]
    else:
        global_loop_candidate_idx = where(loop_candidate_mask)[0]

    # ② 检验候选 frame 是否 valid(跟踪未失败 / 未停车)
    if len(global_loop_candidate_idx) > 0:
        valid_mask = [valid_flags[idx] for idx in global_loop_candidate_idx]
        global_loop_candidate_idx = global_loop_candidate_idx[valid_mask]

    # ③ 生成虚拟节点
    if len(global_loop_candidate_idx) > 0:
        context_pc = neural_points.local_neural_points.detach()
        cur_pose = cur_pgo_poses[curr_node_idx]
        last_pose = cur_pgo_poses[curr_node_idx - 1] if curr_node_idx > 0 else None
        neural_points_feature = neural_points.local_geo_features[:-1].detach() if loop_with_feature else None
        self.set_virtual_node(context_pc, cur_pose, last_pose, neural_points_feature)

    # ④ 调用 detect_loop 做粗筛 + 精筛
    loop_id, loop_cos_dist, loop_transform = self.detect_loop(
        global_loop_candidate_idx, use_feature=loop_with_feature)

    local_map_context_loop = False
    if loop_id is not None:
        if local_map_context:
            # local-map 描述子有 latency,需把 loop_transform 从"latency 帧"换算到"当前帧"
            loop_transform = loop_transform @ inv(cur_pgo_poses[curr_node_idx]) @ cur_pgo_poses[-1]
            local_map_context_loop = True

    return loop_id, loop_cos_dist, loop_transform, local_map_context_loop
```
证据 `[VERIFY: utils/loop_detector.py:158-229]`

#### dist_thre 的来源

主循环传的是 `pgm.drift_radius × loop_dist_drift_ratio_thre` `[VERIFY: pin_slam.py:309]`。drift 越大,搜索半径越大 — 因为我们对当前位姿的不确定性越大,候选可能离当前估计位置更远。

### 2.4 `detect_loop` — 粗筛 + 精筛

```python
def detect_loop(self, candidate_idx, use_feature=False):
    if len(candidate_idx) == 0:
        return None, None, None

    # ─── 粗筛:ring key L1/cos 距离 ───
    if use_feature:
        ringkey_feature_history = stack([ringkeys_feature[i] for i in candidate_idx])
    else:
        ringkey_history = stack([ringkeys[i] for i in candidate_idx])

    min_dist_ringkey = 1e5
    min_loop_idx = None
    min_query_idx = 0

    if len(query_contexts) == 0:        # 无虚拟节点 → 用 self.contexts[curr_node_idx]
        tran_from_frame.append(eye(4))
        query_contexts.append(contexts[curr_node_idx])

    for query_idx in range(len(query_contexts)):
        if use_feature:
            query_rk_feature = sc2rk(query_contexts[query_idx])
            dist_to_history = 1 - F.cosine_similarity(query_rk_feature.view(1,-1),
                                                       ringkey_feature_history.view(history_count,-1),
                                                       dim=1)
        else:
            query_rk = sc2rk(query_contexts[query_idx])
            diff_to_history = query_rk - ringkey_history
            dist_to_history = norm(diff_to_history, p=1, dim=1)   # L1 norm

        min_idx_in_cand = argmin(dist_to_history)
        cur_min_idx_in_candidates = candidate_idx[min_idx_in_cand]
        cur_dist_ringkey = dist_to_history[min_idx_in_cand]

        if cur_dist_ringkey < min_dist_ringkey:
            min_dist_ringkey = cur_dist_ringkey
            min_loop_idx = cur_min_idx_in_candidates
            min_query_idx = query_idx

    if min_dist_ringkey > ringkey_dist_thre:
        return None, None, None       # 粗筛失败

    # ─── 精筛:完整 SC 距离 ───
    if use_feature:
        cosdist, yaw_diff = distance_sc_feature_torch(
            contexts_feature[min_loop_idx], query_contexts[min_query_idx])
    else:
        cosdist, yaw_diff = distance_sc_torch(
            contexts[min_loop_idx], query_contexts[min_query_idx])

    # ─── 阈值判定 + 输出 T_l<-c' ───
    if cosdist < sc_cosdist_threshold:
        yawdiff_deg = yaw_diff × (360 / S)
        yawdiff_rad = radians(yawdiff_deg)
        # 构造 T_l<-c' (绕 z 旋 yaw,无平移)
        transformation = eye(4)
        transformation[0,0] = cos(yaw_rad); transformation[0,1] = sin(yaw_rad)
        transformation[1,0] = -sin(yaw_rad); transformation[1,1] = cos(yaw_rad)
        # 拼上 query 的虚拟变换:T_l<-c = T_l<-c' @ T_c'<-c
        transformation = transformation @ tran_from_frame[min_query_idx].cpu().numpy()
        return min_loop_idx, cosdist, transformation
    else:
        return None, None, None
```
证据 `[VERIFY: utils/loop_detector.py:231-348]`

### 2.5 距离 / cos / yaw 三个阈值

| 阈值 | 默认 | 作用 |
|------|------|------|
| `ringkey_dist_thre` | `(max_z - min_z) × 0.25` ≈ 21 m;启用 feature 时 0.25 cos | 粗筛 L1 距离上限 |
| `sc_cosdist_threshold` | 0.2;启用 local_map_context +0.08;启用 feature 再 +0.08 | 精筛 cos 距离上限 |
| 距离粗筛 `dist_thre` | `drift_radius × loop_dist_drift_ratio_thre` 动态 | 几何邻域 |

三层级筛流水:
1. 几何邻域(drift_radius × ratio)→ N₁ 候选
2. valid_flags 过滤 → N₂
3. ring key L1/cos 距离 < thre → N₃ ≤ 1(取最小)
4. 完整 SC cos 距离 < thre → 通过 / 失败

### 2.6 local_map_context vs scan context

```python
if local_map_context and frame_id >= local_map_context_latency:
    # local map 描述子
    local_map_frame_id = frame_id - local_map_context_latency       # 5 帧之前
    local_map_pose = pgo_poses[local_map_frame_id]
    if local_map_context_latency > 0:
        neural_points.reset_local_map(local_map_pose[:3,3], None, local_map_frame_id,
                                      loop_local_map_by_travel_dist, loop_local_map_time_window)
    context_pc_local = transform_torch(neural_points.local_neural_points, inv(local_map_pose))
    lcd_npmc.add_node(local_map_frame_id, context_pc_local, feature)
else:
    lcd_npmc.add_node(frame_id, dataset.cur_point_cloud_torch, valid_flag=valid_mapping_flag)
```
证据 `[VERIFY: pin_slam.py:283-292]`

#### 为什么 latency?

LCD 描述子的质量与"该帧的局部地图是否已充分训练"相关。刚处理完的当前帧,神经点 latent 几乎全 0,SC_feature 没有信息。等 5 帧之后再回头编码,latent 经过 5 × 12 = 60 次 Adam 优化,质量好得多。

代价:
- 当前帧不能立刻被搜索(早 5 帧)
- 但回环本来就是大尺度对应,5 帧滞后不影响最终路径

#### 当前帧的位姿是 `local_map_pose`

回环匹配出来的 yaw 是基于"latency 那一帧的传感器系" → 需要变换到当前帧:
```python
loop_transform = loop_transform @ inv(cur_pgo_poses[curr_node_idx]) @ cur_pgo_poses[-1]
                                # T_l<-c = T_l<-c' @ T_c'<-c
```
其中 c' 是 latency 帧,c 是当前帧。

### 2.7 `detect_local_loop`(几何回环,不用 SC)

```python
def detect_local_loop(pgo_poses, loop_candidate_mask, cur_drift, cur_frame_id,
                      loop_reg_failed_count=0, dist_thre=1.0, drift_thre=3.0, silence=False):
    dist_to_past = norm(pgo_poses[:,:3,3] - pgo_poses[-1,:3,3], axis=1)
    min_dist = min(dist_to_past[loop_candidate_mask])
    min_index = where(dist_to_past == min_dist)[0]
    if (min_dist < dist_thre and cur_drift < drift_thre and loop_reg_failed_count < 3):
        loop_id, loop_dist = min_index[0], min_dist
        loop_transform = inv(pgo_poses[loop_id]) @ pgo_poses[-1]
        return loop_id, loop_dist, loop_transform
    return None, None, None
```
证据 `[VERIFY: utils/loop_detector.py:443-479]`

#### 触发条件

1. 当前位置离某历史位置 < `dist_thre`(默认 1m,或 yaml `local_loop_dist_thre`)
2. 漂移估计 < `drift_thre`(默认 3m,即 `dist_thre × 3`)
3. 连续 reject 次数 < 3(避免反复尝试同一坏候选)

#### 与 global loop 互补

- local loop:小漂移场景,纯几何
- global loop:大漂移场景,需 SC 描述子

主循环先试 local,失败再试 global `[VERIFY: pin_slam.py:307-309]`。

---

## 3. PoseGraphManager 详解

### 3.1 因子图模型

节点: 每帧位姿 `T_world<-frame_id` ∈ SE(3),用 `gtsam.symbol("x", frame_id)` 索引
边:
- **Prior**: 第 0 帧 fixed,固定为 GT pose(若有)或 identity
- **Odometry**: 相邻帧间 `T_{frame_id-1} <- frame_id`,从跟踪得到
- **Loop**: 回环帧对 `T_{loop_id} <- frame_id`,从 SC + tracker(loop_reg)得到

### 3.2 噪声模型

```python
self.fixed_cov = gtsam.noiseModel.Diagonal.Sigmas(
    [1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9])     # 几乎为 0,锚定第 0 帧

self.const_cov = np.array([
    radians(rot_std), radians(rot_std), radians(rot_std),    # 3 旋转 σ
    tran_std, tran_std, tran_std                              # 3 平移 σ
])   # gtsam 约定:前 3 rot,后 3 tran
self.odom_cov = gtsam.noiseModel.Diagonal.Sigmas(self.const_cov)
self.loop_cov = gtsam.noiseModel.Diagonal.Sigmas(self.const_cov)
```
证据 `[VERIFY: utils/pgo.py:24-42]`

默认 `pgo_tran_std=0.04m`, `pgo_rot_std=0.01°`。

`use_reg_cov_mat=True` 时切换:
```python
cur_edge_cov = cur_odom_cov if config.use_reg_cov_mat else None
pgm.add_odometry_factor(frame_id, frame_id-1, last_odom_tran, cov=cur_edge_cov)
# add_odometry_factor 里:
cov_model = gtsam.noiseModel.Gaussian.Covariance(cov) if cov is not None else self.odom_cov
```
证据 `[VERIFY: pin_slam.py:296-297, utils/pgo.py:130-133]`

### 3.3 鲁棒核(未使用)

```python
mEst = gtsam.noiseModel.mEstimator.GemanMcClure(1.0)
self.robust_loop_cov = gtsam.noiseModel.Robust(mEst, self.loop_cov)
self.robust_odom_cov = gtsam.noiseModel.Robust(mEst, self.odom_cov)
```
证据 `[VERIFY: utils/pgo.py:44-47]`

定义了但**没用**(代码里 `add_*_factor` 还是用普通 cov)。注释说 "figure it out"。所以错误回环依赖**前置 reject 启发式**(SC 阈值 + tracker valid_flag + z-check),而不是 PGO 内部的 M-estimator。

### 3.4 优化器:ISAM2 vs LM

```python
def optimize_pose_graph(self):
    if config.pgo_with_isam:                   # 默认 True
        self.isam.update(graph_factors, graph_initials)
        graph_optimized = self.isam.calculateEstimate()
    else:
        opt_param = gtsam.LevenbergMarquardtParams()
        opt_param.setMaxIterations(pgo_max_iter)             # 50
        opt = gtsam.LevenbergMarquardtOptimizer(
            graph_factors, graph_initials, opt_param)
        graph_optimized = opt.optimizeSafely()
        error_before = graph_factors.error(graph_initials)
        error_after = graph_factors.error(graph_optimized)
        self.last_error = error_after

    graph_initials = graph_optimized

    # 把 gtsam Values 拷回 numpy
    pgo_poses = init_poses.copy()
    for idx in range(curr_node_idx+1):
        pgo_poses[idx] = get_node_pose(graph_optimized, idx)
    cur_pose = pgo_poses[curr_node_idx]
    pgo_count += 1

    if config.pgo_with_isam:
        # 增量优化:已合并的 factors / values 不再保留在临时图里
        self.graph_factors = gtsam.NonlinearFactorGraph()
        self.graph_initials.clear()
```
证据 `[VERIFY: utils/pgo.py:190-234]`

#### ISAM2 优势

- 增量:只对受影响的子图重新线性化
- 内部维护 Bayes Tree,平均 O(已变节点数) per update
- 对大图(数千帧)远快于批量 LM

#### LM 优势

- 全图收敛,数值更准
- 可拿 `error_before / error_after` 监控质量
- 失败 outlier 可被 reject(`reject_outlier=True`)

⚠️ `reject_outlier` 启发式只在 LM 模式生效:
```python
if reject_outlier and not config.pgo_with_isam:
    cur_error = graph_factors.error(graph_initials)
    valid_thre = last_error + (cur_id - last_loop_idx) × pgo_error_thre_frame
    if reject_outlier and cur_error > valid_thre:
        graph_factors.remove(graph_factors.size() - 1)
        return False
```
证据 `[VERIFY: utils/pgo.py:174-187]`

逻辑:刚加的 loop edge 让 graph error 大幅增加 → 视为坏边,移除。`pgo_error_thre_frame=500` 是 per-frame 允许的增量(基于距上次回环的帧数)。

### 3.5 漂移估计

```python
def estimate_drift(self, travel_dist, used_frame_id, drift_ratio=0.01, correct_ratio=0.005):
    self.drift_radius = (
        travel_dist[used_frame_id] - travel_dist[last_loop_idx]
    ) * drift_ratio
    if min_loop_idx < last_loop_idx:                         # 之前已修正过
        self.drift_radius += (
            travel_dist[min_loop_idx] + travel_dist[used_frame_id] * correct_ratio
        ) * drift_ratio
```
证据 `[VERIFY: utils/pgo.py:323-336]`

#### 物理意义

LiDAR 里程计典型漂移率 ~1% travel_dist(经验值,实际 0.5%-2%)。
- 距上次成功回环已走 100m → drift ~1m
- 这个值用作 `global_loop` 搜索半径 `[VERIFY: pin_slam.py:309]`

主循环里 `correct_ratio=0.01` 而 PGO 里 `correct_ratio=0.005`,两处略不同但都在合理范围。

### 3.6 落盘函数

| 函数 | 输出 |
|------|------|
| `write_g2o(out_file)` | gtsam.writeG2o → `final_pose_graph.g2o`,可在 g2o-viewer 打开 |
| `write_loops(out_file)` | 文本: `loop_id frame_id\n 4×4 矩阵`,每条边 5 行 |
| `read_loops(in_file, subsample)` | 反向解析,用于离线复现 |
| `offline_pgo(odom_poses)` | 给定 odom poses + 已有 loop edges,跑一次 PGO(debug)|
| `plot_loops(path, vis)` | matplotlib 3D 绘制 trajectory + 绿色 loop 弧 |

证据 `[VERIFY: utils/pgo.py:237-380]`

---

## 4. 主循环里的回环-PGO 完整流程

### 4.1 每帧固定步骤

```
for frame_id in range(...):
    # ...(跟踪)
    if config.pgo_on:
        ① add_node(描述子)
        ② pgm.add_frame_node(frame_id, init_pose=pgo_poses[frame_id])
        ③ pgm.init_poses = pgo_poses[:frame_id+1]
        ④ pgm.add_odometry_factor(frame_id, frame_id-1, last_odom_tran, cov)
        ⑤ pgm.estimate_drift(travel_dist, frame_id)
        ⑥ if pgo_with_pose_prior: pgm.add_pose_prior(frame_id, pgo_poses[frame_id])
```

### 4.2 周期回环搜索

```
if frame_id - last_loop_idx > pgo_freq and not stop_status:
    loop_candidate_mask = (travel_dist[-1] - travel_dist) > min_loop_travel_dist_ratio × local_map_radius
    loop_id = None
    if any(loop_candidate_mask):
        # ① 几何回环(快路径)
        loop_id, loop_dist, loop_transform = detect_local_loop(
            pgo_poses, loop_candidate_mask, drift_radius, frame_id,
            loop_reg_failed_count, local_loop_dist_thre, local_loop_dist_thre×3)

        # ② NPMC 全局回环(慢路径)
        if loop_id is None and global_loop_on:
            loop_id, loop_cos_dist, loop_transform, local_map_context_loop = \
                lcd_npmc.detect_global_loop(pgo_poses, drift_radius × loop_dist_drift_ratio_thre,
                                            loop_candidate_mask, neural_points)
```

### 4.3 z 检查 + valid 检查

```
if loop_id is not None:
    if loop_z_check_on and |loop_transform[2,3]| > voxel_size × 4:
        loop_id = None                # 多层建筑歧义
    if not lcd_npmc.valid_flags[loop_id]:
        loop_id = None                # 候选 frame 跟踪曾失败
```
证据 `[VERIFY: pin_slam.py:311-314]`

### 4.4 精配 + 验证

```
if loop_id is not None:
    pose_init = pgo_poses[loop_id] @ loop_transform
    neural_points.recreate_hash(pose_init[:3,3], None, True, True, loop_id)
    pose_refine, loop_cov, weight_pc, reg_valid = tracker.tracking(
        source_points.clone(), pose_init, loop_reg=True)

    if reg_valid:
        loop_transform = inv(pgo_poses[loop_id]) @ pose_refine
        cur_edge_cov = loop_cov if use_reg_cov_mat else None
        reg_valid = pgm.add_loop_factor(frame_id, loop_id, loop_transform, cov=cur_edge_cov)
```

### 4.5 优化 + 地图变形

```
if reg_valid:
    pgm.optimize_pose_graph()
    cur_loop_vis_id = frame_id - local_map_context_latency if local_map_context_loop else frame_id
    pgm.loop_edges_vis.append([loop_id, cur_loop_vis_id])
    pgm.loop_edges.append([loop_id, frame_id])
    pgm.loop_trans.append(loop_transform)

    # 弹性地图修正
    pose_diff = pgm.get_pose_diff()                       # [F+1, 4, 4]
    dataset.cur_pose_torch = pgm.cur_pose
    neural_points.adjust_map(pose_diff_torch)             # ALGORITHM 01 §9
    neural_points.recreate_hash(cur_pose[:3,3], None, not pgo_merge_map, rehash_with_time, frame_id)
    mapper.transform_data_pool(pose_diff_torch)           # 全局 pool 一起变
    dataset.update_poses_after_pgo(pgm.pgo_poses)
    pgm.last_loop_idx = frame_id
    pgm.min_loop_idx = min(pgm.min_loop_idx, loop_id)
    loop_reg_failed_count = 0
else:
    neural_points.recreate_hash(cur_pose[:3,3], None, True, True, frame_id)   # 回滚
    loop_reg_failed_count += 1
```
证据 `[VERIFY: pin_slam.py:325-348]`

---

## 5. 性能 / 内存

### 5.1 描述子内存

- 每帧 SC: `20 × 60 × 4B = 4.8 KB`
- 每帧 ring key: `20 × 4B = 80 B`
- 1 万帧:50 MB(可忽略)
- 启用 feature:`20 × 60 × 8 × 4B = 38.4 KB`,1 万帧 = 380 MB(可观)

### 5.2 单次回环耗时

| 步骤 | 时间 | 备注 |
|------|------|------|
| 11 个 virtual 节点 SC 生成 | ~10 ms | 仅当大尺度回环 |
| 粗筛(ring key L1)| < 1 ms | 候选 < 100 个 |
| 精筛(60 sector shift × cos)| < 1 ms | per candidate |
| recreate_hash | 10-30 ms | 大地图 |
| tracker.tracking(loop_reg) | ~100 ms | 50 iter ×  2ms/iter |
| adjust_map | 10-50 ms | 神经点全量旋转 |
| ISAM2 / LM 优化 | < 10 ms | 增量 / 5 千节点以下 |
| transform_data_pool | 20-100 ms | 池大小 1e7 |
| 总 | ~300 ms | 回环触发帧的额外延迟 |

### 5.3 优化建议

| 场景 | 调参 |
|------|------|
| 室内小循环 | `pgo_freq=10`(更频繁尝试),`min_loop_travel_dist_ratio=2`(短距即可)|
| 大尺度城市 | `pgo_freq=60`,`local_map_context=True` |
| 多层建筑 | `loop_z_check_on=True`,`voxel_size_m × 4` 阈值约 1.2m |
| 高速场景 | `context_virtual_step_m=4`,`virtual_side_count=3`(扩大搜索)|
| 稠密回环 | `pgo_with_isam=True` 必须 |

---

## 6. 与论文 §III-E "Loop Closure & PGO" 对应

| 论文 | 代码 |
|------|------|
| Neural Point Map Context | `ptcloud2sc_torch` + `local_map_context` |
| Ring-key 粗筛 | `sc2rk` + `argmin(diff)` |
| Sector-shifted cosine 精筛 | `distance_sc_torch` |
| Yaw alignment | `yaw_diff × 360/S` |
| Virtual node augmentation | `set_virtual_node` |
| Geometric verification | `tracker.tracking(loop_reg=True)` |
| ISAM2 vs LM | `pgo_with_isam` |
| Pose-graph elastic deformation | `adjust_map(pose_diff)` |

---

## 7. 边界 case

### 7.1 第 0 帧

- `add_pose_prior(0, init_pose, fixed=True)` `[VERIFY: pin_slam.py:189]`
- 第 0 帧无 odom edge,无 loop search

### 7.2 极短序列(< 30 帧)

- `pgo_freq=30` 默认 → 永远不触发回环
- 描述子仍每帧入库
- 可改 `pgo_freq=10` 测试小回环

### 7.3 停车期间(`stop_status=True`)

- `if ... and not dataset.stop_status` 跳过回环搜索 `[VERIFY: pin_slam.py:302]`
- 描述子仍 add(可被未来搜索)
- 防止"无新观测时反复匹配同一帧"

### 7.4 跟踪失败但描述子要不要加?

```python
lcd_npmc.add_node(local_map_frame_id, context_pc_local, ..., valid_flag=valid_mapping_flag)
```
`valid_mapping_flag = not lose_track and not stop_status`,失败帧的描述子带 `valid=False`。后续 `detect_global_loop` 会过滤掉 `not valid_flags[idx]` 的候选 `[VERIFY: utils/loop_detector.py:172-174]`。

### 7.5 加边后 PGO 失败

LM 模式有 `reject_outlier` 启发式,会移除最新一条边返回 False。ISAM2 模式无 reject,坏边永久污染图。这是 ISAM2 的已知风险。

### 7.6 `pgo_with_pose_prior=True` 的副作用

`add_pose_prior(frame_id, pgo_poses[frame_id])` 每帧加 prior:
- 协方差 σ ∝ `drift_radius`,初期小后期大
- 把"里程计估计"作为软约束加入图,防止 PGO 过度依赖少数回环边
- 默认 False,通常需要 GT-aware 场景才开

### 7.7 `pgo_merge_map=True` 的连锁

回环后:
```python
neural_points.recreate_hash(cur_pose, None,
                            not pgo_merge_map,    # kept_points
                            rehash_with_time, frame_id)
```
- `pgo_merge_map=False`(默认):`kept_points=True` → 不删旧点,只重建哈希
- `pgo_merge_map=True`:`kept_points=False` → **真合并去重**,神经点数量大幅减少

副作用:`adjust_map` 后旧点已经被旋转到新位姿,合并时会"折叠"到附近的 voxel — 可能丢失分辨率。所以默认不开。

### 7.8 `rehash_with_time` 的意义

```python
if with_ts:
    ts_diff = |ts_used - cur_ts|
    sample_idx = voxel_down_sample_min_value_torch(neural_points, r, ts_diff)
    # 每 voxel 保留 ts_diff 最小(最近时间)的代表点
else:
    sample_idx = voxel_down_sample_min_value_torch(neural_points, r,
                                                   point_certainties.max() - certainty)
    # 保留 certainty 最大的
```
证据 `[VERIFY: model/neural_points.py:843-862]`

- `True` 默认:回环后用"距当前时间最近"作 voxel 代表 → 优先保留新观测
- `False`:用 certainty 排序 → 优先保留稳定点

---

## 8. 一句话总结

> **Loop Detector = Scan Context (20×60 极坐标格,存最大 z) + Ring Key (沿 sector 取均值,旋转不变) + 虚拟节点横向增广 + 旋转对齐 cos 距离匹配。Yaw 差作为初始变换给 tracker(loop_reg=True)做精配。** PGO 用 gtsam ISAM2 / LM 增量优化 6-DoF 位姿图,**回环后 `adjust_map` 让神经点跟着对应帧位姿弹性变形** — 这是 PIN-SLAM "globally consistent" 的根本机制。

---

**END of ALGORITHM 05.**

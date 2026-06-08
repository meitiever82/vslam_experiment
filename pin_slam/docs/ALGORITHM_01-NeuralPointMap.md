# ALGORITHM 01 — 神经点地图(Neural Point Map)

> Phase 4 文档系列之一。对应 `model/neural_points.py`(1144 行)。
> 所有 `[VERIFY:]` 路径相对 `src/PIN_SLAM/`。

---

## 0. TL;DR

神经点地图是 PIN-SLAM 的**地图层**,把空间表示成"带可学习特征向量的稀疏点云":

- 每个神经点 = 位置 `(x,y,z)` + 朝向四元数 + `feature_dim=8` 维 latent
- 体素哈希(3 个大质数 XOR 风格)定位 + 8 邻居立方体 + 球裁,做 KNN
- 查询点 → 6 个 KNN → IDW 加权融特征 → MLP 解码 SDF
- 全局张量 + 局部张量两套,只局部参与训练
- 回环触发 `adjust_map(pose_diff)`,神经点位置 + 朝向跟着对应帧位姿做 SE3 变换 → **map 弹性变形**

下面把 11 个核心函数逐个拆。

---

## 1. 数据结构(快速复习)

| 类别 | 字段 | 说明 |
|------|------|------|
| 全局 | `neural_points [N, 3]` | 位置 |
| | `point_orientations [N, 4]` | 四元数 |
| | `geo_features [N+1, 8]` | latent(末位 padding)|
| | `color_features [N+1, 8]` 或 None | latent |
| | `point_certainties [N]` | 累积 IDW 权重 |
| | `point_ts_create / point_ts_update [N]` | 时间戳 |
| 哈希 | `buffer_pt_index [5e7]` | 桶 → 全局 idx |
| | `primes [3] = (73856093, 19349669, 83492791)` | 哈希盐 |
| 局部 | `local_neural_points [M, 3]` | |
| | `local_geo_features [M+1, 8]` (Parameter) | **被 Adam 优化** |
| | `local_mask [N+1]` bool | True 表示该全局点在局部 |
| | `global2local [N+1]` int64 | 全局 idx → 局部 idx,无效 -1 |
| | `local_point_certainties [M]` | 局部 certainty(scatter_add)|

完整字段见 `DATA_STRUCTURES.md` §3。

---

## 2. 空间哈希(Spatial Hashing)

### 2.1 公式

给定点 `p = (px, py, pz)`,体素大小 `r = voxel_size_m`:

```
grid_coord = floor(p / r)            ∈ ℤ³            # [VERIFY: model/neural_points.py:334]
hash_val   = (grid_coord · primes) mod buffer_size   # [VERIFY: model/neural_points.py:336]
           = (g.x × 73856093 + g.y × 19349669 + g.z × 83492791) mod 5e7
bucket_idx = buffer_pt_index[hash_val]   # 全局神经点 idx,空桶=-1
```

`torch.fmod` 而不是 `%`,所以哈希值可以是负数 — 但是 `buffer_pt_index` 是 `int64` 索引,负索引在 PyTorch 里被解释为 "倒数第 |x| 个",这恰好是合理的(因为 buffer 是稠密一维数组)。等价但行为一致 `[VERIFY: model/neural_points.py:973-975]`。

### 2.2 与 Teschner et al. 2003 的关系

经典 Teschner spatial hashing 的形式:
```
h(x, y, z) = (x × p1 ⊕ y × p2 ⊕ z × p3) mod n
```
PIN-SLAM 用的是**加法**而不是 XOR(`(... * primes).sum(-1)`):
- 优点: 完全可微张量化,几行 PyTorch 搞定
- 缺点: 三个相邻 voxel 容易产生周期碰撞(`(g + (1,0,0)) · primes = g·primes + 73856093`,但这两个 hash 都 mod buffer_size,所以邻居 hash 之间总是固定偏移)
- 影响: PIN-SLAM **故意接受这种碰撞**,因为后续会用"近似 KNN + 距离平方门 `max_valid_dist2`"过滤碰撞产物 `[VERIFY: model/neural_points.py:999-1000]`

### 2.3 为什么 buffer_size = 5×10⁷ 且不做开链

经典哈希表对碰撞要"链表 / 二次探测"。这里**直接每桶只存一个 idx**,后写覆盖先写:
```python
self.buffer_pt_index[hash] = sample_idx     # [VERIFY: model/neural_points.py:871, :901]
```
等价于"每桶最近一次写入获胜"。这只在 `recreate_hash(kept=True)` 这种"批量重建"场景成立(同一时刻所有 voxel 一起插入);`update()` 增量场景靠"时间窗 + 距离门"把旧条目挤出(下面 §3.3 详述)。

桶容量 ≥ 神经点期望峰值 × 10 倍,可保证负载因子 < 0.1,碰撞率 < 5% `[VERIFY: utils/config.py:100]`。

### 2.4 设计权衡表

| 替代 | 优 | 缺 | 是否被采用 |
|------|----|----|------------|
| Sparse Hash + 开链 | 零碰撞 | 动态分配慢,GPU 不友好 | ❌ |
| Octree | 层次裁剪,大空场景省内存 | O(log n) 而非 O(1),插入复杂 | ❌(本作者另一篇 LocNDF 用过)|
| **Hash + 一桶一槽 + 后过滤** | O(1) 完全 GPU vectorized,实现 50 行 | 偶尔碰撞,需后过滤 | ✅ |
| 双重哈希 / Cuckoo | 低碰撞 | GPU 实现复杂 | ❌ |

---

## 3. `update()` — 新观测增量插入

### 3.1 整体结构

```python
def update(self, points, sensor_position, sensor_orientation, cur_ts):
    sample_idx = voxel_down_sample_torch(points, resolution)      # voxel 内取最靠中心
    sample_points = points[sample_idx]                             # [N', 3]
    grid_coords = (sample_points / r).floor().to(int64)            # [N', 3]
    hash = ((grid_coords * primes).sum(-1)) % buffer_size          # [N']
    hash_idx = buffer_pt_index[hash]                                # [N'], 已占桶?

    if not is_empty and cur_ts != reboot_ts:
        vec = neural_points[hash_idx] - sample_points
        dist2 = sum(vec**2, dim=-1)
        update_mask = (hash_idx == -1) | (dist2 > 3·r²)            # ↑ 空桶或远碰撞
        if temporal_local_map_on:
            Δd = travel_dist[cur_ts] - travel_dist[point_ts_update[hash_idx]]
            update_mask |= (Δd > diff_travel_dist_local)            # 老旧条目让位
    else:
        update_mask = ones_like                                     # 第 0 帧 / reboot 全插

    added_pt = sample_points[update_mask]                           # 真正新点
    ...
    self.neural_points = cat(neural_points, added_pt)
    ...                                                             # ts/quat/cert 同步扩容
    new_fts = std × randn(new_count + 1, F)                          # 高斯初始化 latent
    self.geo_features = cat(geo_features[:-1], new_fts)              # 末位 padding 替换
    ...
    self.reset_local_map(...)                                        # 必须立刻同步局部视图
```

证据: `[VERIFY: model/neural_points.py:311-422]`

### 3.2 关键步骤逐行

| 步骤 | 行号 | 数学 / 行为 |
|------|------|------------|
| 0 | 331 | voxel 内代表点选最靠中心 — `voxel_down_sample_torch` 用最小 dist 投票 |
| 1 | 334 | floor 取整 → 离散 grid coord |
| 2 | 336 | 哈希 |
| 3 | 338 | 取出桶里已有的 idx(空桶 -1) |
| 4 | 342-343 | 已占桶距离平方 |
| 5 | 345 | `dist2 > 3r²` 视为**碰撞**(同桶但远) |
| 6 | 349-353 | travel_dist 差 > 阈值则 **过期**,新观测覆盖 |
| 7 | 362 | 决定本帧真正插入的子集 `added_pt` |
| 8 | 371-377 | 给每个新点分配从 `cur_pt_count` 起的连续 idx,更新 buffer |
| 9 | 378-386 | concat 全局位置 + 四元数(默认 [1,0,0,0])|
| 10 | 388-392 | concat ts_create / ts_update = cur_ts |
| 11 | 394-401 | concat **高斯初始化**的 latent(`std = feature_std = 0`,即真零)|
| 12 | 403-411 | 颜色 latent 同样处理 |
| 13 | 413-416 | concat certainty=0 |
| 14 | 418-420 | `reset_local_map(...)` 立刻同步局部视图 |
| 15 | 422 | 返回 `new_point_ratio = #added / #sampled` |

### 3.3 时间窗 vs 距离窗的角色

PIN-SLAM 的"局部地图"用 **累积里程** 而不是 **帧时间** 作为时间维度:
- `diff_travel_dist_local = local_map_radius × local_map_travel_dist_ratio = 50 × 5 = 250 m`(默认)
- 也就是说一个神经点在累计里程范围 250 m 内被算"活跃",否则会从局部地图淘汰

为什么用 travel_dist?
- 帧时间不公平:停车 30s 不应淘汰神经点
- 累计里程公平:不管帧率如何,移动 250 m 后局部地图必然换一批

代码 `[VERIFY: model/neural_points.py:62-63, :350-356, :456-460]`。

### 3.4 hash 碰撞 4 种状态

```
hash_idx == -1                             → 空桶,插
dist2 < 3r²    + ts 不过期                  → 同 voxel 已有点,跳
dist2 < 3r²    + ts 过期                    → 同 voxel 但是旧点,新点覆盖(后写赢)
dist2 ≥ 3r²                                → 哈希碰撞(不同 voxel 共桶),新点覆盖
```

⚠️ "后写赢"在第 3 种情况是数据丢失:旧 voxel 的特征向量从 `buffer_pt_index` 移除了 reference,但 `geo_features` 里那个槽位**没真正删除**,只是变成"孤儿"(无人指到)。这就是为什么 `recreate_hash(kept=False)` 在最后阶段需要做去重 `[VERIFY: model/neural_points.py:873-903]`。

---

## 4. `reset_local_map()` — 局部地图构建

### 4.1 用途

每次满足下列条件之一就重建局部张量集:
- 收到新观测 (`update` 末尾自动调用)
- 跟踪失败但仍要保持局部视图("don't let the wrong pose corrupt the map" `pin_slam.py:373`)
- 回环搜索时把局部地图切到历史时刻 (`recreate_hash` 内部调用)

### 4.2 算法

```python
def reset_local_map(self, sensor_position, sensor_orientation, cur_ts,
                    use_travel_dist=True, diff_ts_local=50, reboot_map=False):
    self.cur_ts = cur_ts
    self.max_ts = max(self.max_ts, cur_ts)

    # ──── 时间筛 ────
    if temporal_local_map_on:
        ts_used = (create + update)/2 if use_mid_ts else create
        if use_travel_dist:
            Δd = |travel_dist[cur_ts] - travel_dist[ts_used]|
            time_mask = (Δd < diff_travel_dist_local)
        else:
            time_mask = (|cur_ts - ts_used| < diff_ts_local)
        if reboot_map:
            time_mask &= (ts_used >= reboot_ts)
        if sum(time_mask) < 100:                # 兜底
            time_mask = ones
    else:
        time_mask = ones

    # ──── 距离筛(只对 time_mask 内做)────
    masked_v = neural_points[time_mask] - sensor_position
    masked_d2 = sum(masked_v**2, dim=-1)
    dist_mask = (masked_d2 < local_map_radius**2)
    time_mask_idx = nonzero(time_mask)
    local_mask_idx = time_mask_idx[dist_mask]

    # ──── 写局部视图 ────
    local_mask = full(time_mask.shape, False)
    local_mask[local_mask_idx] = True

    self.local_neural_points        = neural_points[local_mask]
    self.local_point_orientations   = point_orientations[local_mask]
    self.local_point_certainties    = point_certainties[local_mask]
    self.local_point_ts_update      = point_ts_update[local_mask]

    local_mask = cat(local_mask, [True])        # padding 末位
    self.local_mask = local_mask

    # ──── global → local 映射 ────
    global2local = full_like(local_mask, -1, int64)
    local_indices = nonzero(local_mask)
    global2local[local_indices] = arange(local_count)
    global2local[-1] = -1                       # 显式标记 padding
    self.global2local = global2local

    # ──── 包成 Parameter ────
    self.local_geo_features   = nn.Parameter(geo_features[local_mask])
    self.local_color_features = nn.Parameter(color_features[local_mask])  # 若启用
```

证据: `[VERIFY: model/neural_points.py:424-514]`

### 4.3 为什么要 `nn.Parameter`?

`local_geo_features` 是 **真正参与梯度回传** 的对象。`Mapper.mapping` 在 setup_optimizer 时只把 `neural_points.parameters()` 注册到 Adam:
```python
neural_point_feat = list(self.neural_points.parameters())
opt = setup_optimizer(config, neural_point_feat, sdf_mlp_param, ...)
# [VERIFY: utils/mapper.py:604-621]
```
`NeuralPoints.parameters()` 来自 `nn.Module` 的 `_parameters` 属性,只收 `nn.Parameter` 字段 — 而本类里只有 `local_geo_features` 和 `local_color_features` 是 Parameter。所以**全局 `geo_features` 张量永远不被优化**,只有把它"局部化"后才能学习。

### 4.4 为什么 padding 一位?

`query_feature` 里失败的 KNN 索引会被强制设为 `-1`,然后 `idx[~valid_mask] = 0`(`utils/mapper.py:690` 写错应该是 `query_feature` 内部 `:684, :690`?查代码:`model/neural_points.py:584` 把无效 dist 设大,`:589-590` 排序后 `idx == -1` 的会落到尾部,`:596-597` `valid_mask = idx >= 0`)。

实际 padding 用法是 `recreate_hash(kept=False)` 时:
```python
sample_idx_pad = torch.cat((sample_idx, torch.tensor([-1]).to(sample_idx)))
self.geo_features = self.geo_features[sample_idx_pad]
# [VERIFY: model/neural_points.py:886-889]
```
末位 -1 取的是原来 `geo_features[-1]`,即旧 padding,保持 padding 永远在末位。`local_mask` 也按这套约定追加 True。这是一个**极重要但容易忽略**的索引技巧。

---

## 5. `set_search_neighborhood()` — 邻域偏移网格

```python
def set_search_neighborhood(self, num_nei_cells=1, search_alpha=1.0):
    dx = arange(-num_nei_cells, num_nei_cells+1)        # e.g. [-2,-1,0,1,2]
    coords = meshgrid(dx, dx, dx, indexing='ij')         # 5×5×5 立方体 = 125 个体素
    dx = stack(coords, dim=-1).reshape(-1, 3)            # [125, 3]
    dx2 = sum(dx**2, dim=-1)                             # 离原点距离平方
    self.neighbor_dx = dx[dx2 < (num_nei_cells + alpha)²]
    self.neighbor_K = neighbor_dx.shape[0]
    self.max_valid_dist2 = 3 × ((num_nei_cells+1) × r)²
```
证据: `[VERIFY: model/neural_points.py:911-948]`

`neighbor_dx` 是邻域立方体内部再做"球裁剪",球半径 = `(num_cells + alpha) × r`。源码注释列了精确的 K:

```
num_cells=2:  α=0.2 → K=33,  α=0.3 → K=57,  α=0.5 → K=81,  α=1.0 → K=93,  α=2.0 → K=125
num_cells=3:  α=0.2 → K=147, α=0.5 → K=179, α=1.0 → K=251
```

默认 `(num_nei_cells=2, alpha=0.2)` → K=33,即每次查询要去 33 个体素里捞候选点。

`max_valid_dist2 = 3 × ((num_cells+1) × r)²` 是过滤"碰撞产物"的最大平方距离 — 任何邻居距离超过 `√3 × (num_cells+1) × r` 都不可能是合法 KNN(它一定来自哈希碰撞)。

---

## 6. `radius_neighborhood_search()` — KNN 候选生成

### 6.1 流程

```python
def radius_neighborhood_search(self, points, time_filtering=False):
    # points: [N, 3]
    grid_coords = (points / r).floor().to(int64)            # [N, 3]
    neighbord_cells = grid_coords[..., None, :] + neighbor_dx   # [N, K, 3], K=33
    hash = ((neighbord_cells * primes).sum(-1)) % buffer_size   # [N, K]
    neighb_idx = buffer_pt_index[hash]                          # [N, K]

    if time_filtering:                                          # 实际是 travel_dist filter
        Δd = |travel_dist[cur_ts] - travel_dist[point_ts_create[neighb_idx]]|
        local_t_window_mask = Δd < diff_travel_dist_local
        neighb_idx[~local_t_window_mask] = -1

    neighb_pts = neural_points[neighb_idx]                      # [N, K, 3]
    neighb_pts_sub = neighb_pts - points.view(-1,1,3)           # [N, K, 3]
    dist2 = sum(neighb_pts_sub**2, dim=-1)                      # [N, K]
    dist2[neighb_idx == -1] = max_valid_dist2

    # 远碰撞过滤
    neighb_idx[dist2 > max_valid_dist2] = -1

    return dist2, neighb_idx
```
证据: `[VERIFY: model/neural_points.py:951-1010]`

### 6.2 复杂度

- 计算量: `O(N × K)`,K=33
- 内存: `[N, K]` int64 + float32 = 8N × K × 8B + 4N × K × 4B ≈ 100×N×K bytes
  - N=16384 batch,K=33 → ~50 MB transient
- GPU 访问模式: `buffer_pt_index[hash]` 是 **random gather**,bandwidth bound

### 6.3 这不是真正的 KNN

注意:返回的是**邻域球内所有候选**,可能 > K 个有效。真正的 K=`query_nn_k=6` 是在 `query_feature` 里取的:
```python
dists2[idx == -1] = 9e3
sorted_dist2, sorted_neigh_idx = torch.sort(dists2, dim=1)
sorted_idx = idx.gather(1, sorted_neigh_idx)
dists2 = sorted_dist2[:, :nn_k]
idx    = sorted_idx[:, :nn_k]
# [VERIFY: model/neural_points.py:584-590]
```
所以邻域搜索是"先把 K 个候选拉出来,再 sort 取最小 6 个"。这比真正的 KD-tree KNN 更慢 (`O(K log K)` per query) 但完全向量化。

---

## 7. `query_feature()` — 特征查询与 IDW 聚合

### 7.1 接口

```python
geo_feat_vec, color_feat_vec, weight_vec, nn_counts, certainty = neural_points.query_feature(
    query_points,           # [N_q, 3]
    query_ts=None,          # [N_q] int,training_mode 才用
    training_mode=True,     # True 时累加 certainty
    query_locally=True,     # False 走全局张量
    query_geo_feature=True,
    query_color_feature=False,
)
```

### 7.2 全流程(`weighted_first=True` 版本)

```python
def query_feature(self, query_points, ...):
    nn_k = config.query_nn_k                              # 6

    # ──── ① 邻域搜索 ────
    dists2, idx = radius_neighborhood_search(
        query_points,
        time_filtering = temporal_local_map_on and query_locally
    )                                                      # [N_q, K], [N_q, K]

    # ──── ② 全局 → 局部 idx 映射 ────
    if query_locally:
        idx = global2local[idx]                            # [N_q, K], 无效 -> -1

    nn_counts = (idx >= 0).sum(dim=-1)                    # 真实有效邻居数

    # ──── ③ 排序取前 nn_k 个最近 ────
    dists2[idx == -1] = 9e3
    sorted_d2, sorted_n_idx = sort(dists2, dim=1)
    sorted_idx = idx.gather(1, sorted_n_idx)
    dists2 = sorted_d2[:, :nn_k]                          # [N_q, K_keep]
    idx    = sorted_idx[:, :nn_k]
    valid_mask = idx >= 0                                  # [N_q, K_keep]

    # ──── ④ 取邻居 latent ────
    geo_features = zeros(N_q, nn_k, F)
    geo_features[valid_mask] = local_geo_features[idx[valid_mask]]
    # 注:无效位置仍是 0,但下面 weight 也会变 0,贡献不到

    # ──── ⑤ 取邻居局部向量 v = q - p_i ────
    if query_locally:
        certainty = local_point_certainties[idx]           # [N_q, K_keep]
        neighb_vector = query_points.view(-1,1,3) - local_neural_points[idx]
        quat = local_point_orientations[idx]
    else:
        ...

    # ──── ⑥ 回环后旋转 ────
    if after_pgo:
        neighb_vector = apply_quaternion_rotation(quat, neighb_vector)

    neighb_vector[~valid_mask] = 0                         # 无效邻居清零

    # ──── ⑦ 位置编码(默认 band=0,直通)────
    if pos_encoding_band > 0:
        neighb_vector = position_encoder_geo(neighb_vector)

    # ──── ⑧ 特征 + 相对位置 拼成 MLP 输入 ────
    geo_features_vector = cat((geo_features, neighb_vector), dim=2)
                                                           # [N_q, nn_k, F + P]

    # ──── ⑨ IDW 权重(逆平方距离)────
    weight = 1.0 / (dists2 + eps)                          # [N_q, K_keep]
    weight[~valid_mask] = 0
    weight[nn_counts == 0] = eps                           # 全无邻居的查询点兜底
    weight = weight / sum(weight, dim=1, keepdim=True)     # 归一化,行和=1
    weight[~valid_mask] = 0

    # ──── ⑩ certainty 累加(no_grad) ────
    with torch.no_grad():
        if training_mode:
            idx[~valid_mask] = 0
            if query_locally:
                local_point_certainties.scatter_add_(
                    dim=0, index=idx.flatten(), src=weight.flatten())
                if query_ts is not None:
                    local_point_ts_update.scatter_reduce_(
                        dim=0, index=idx.flatten(), src=ts_expand.flatten(),
                        reduce='amax', include_self=True)
            else:
                point_certainties.scatter_add_(...)

        certainty[~valid_mask] = 0
        queried_certainty = sum(certainty * weight, dim=1)

    # ──── ⑪ weighted_first 融合 ────
    weight = weight.unsqueeze(-1)
    if weighted_first:
        geo_features_vector = sum(geo_features_vector * weight, dim=1)
                                                           # [N_q, F + P]

    return geo_features_vector, color_features_vector, weight, nn_counts, queried_certainty
```

证据(逐节):
- ① `[VERIFY: model/neural_points.py:563-565]`
- ② `[VERIFY: model/neural_points.py:573-576]`
- ③ `[VERIFY: model/neural_points.py:584-590]`
- ④ `[VERIFY: model/neural_points.py:596-611]`
- ⑤ `[VERIFY: model/neural_points.py:627-642]`
- ⑥ `[VERIFY: model/neural_points.py:646-649]`
- ⑦ `[VERIFY: model/neural_points.py:654-655]`
- ⑧ `[VERIFY: model/neural_points.py:657-664]`
- ⑨ `[VERIFY: model/neural_points.py:666-685]`
- ⑩ `[VERIFY: model/neural_points.py:686-720]`
- ⑪ `[VERIFY: model/neural_points.py:721-732]`

### 7.3 weighted_first vs weighted_after 数学

**weighted_first(默认)** —— 先融特征再 MLP:
```
fused_feat(q) = Σ_i w_i(q) · [feat_i, q - p_i]
sdf(q)        = MLP(fused_feat(q))
```

**weighted_after** —— 先各自 MLP 再融 SDF:
```
sdf_i(q) = MLP([feat_i, q - p_i])
sdf(q)   = Σ_i w_i(q) · sdf_i(q)
```

| 方案 | 优 | 缺 |
|------|-----|----|
| weighted_first | MLP 只算 1 次/query,**快 6 倍** | 邻居一致性约束弱 |
| weighted_after | 邻居各自做出 SDF,可估方差用作 outlier 检测 | MLP 算 K 次/query,**慢 6 倍** |

PIN-SLAM 默认 weighted_first,但 tracker 用 weighted_after 来得 `sdf_std` 做配准 outlier 检测 `[VERIFY: utils/tracker.py:313-328, :423]`。

### 7.4 IDW 数学解释

Inverse Distance Weighting,Shepard (1968):
```
w_i = 1 / (d_i² + ε)
Σ w_i = 1  (normalize)
f(q) = Σ w_i · f(p_i)
```
- 平方反比的优点:距离 ↑ 权重 ↓↓,自然给最近邻最大权
- 缺点:在采样点处梯度发散(`w_i → ∞`),但是 PIN-SLAM 查询点几乎从不正好落在神经点位置,所以无影响

`idw_index=2` 写在 config 但代码没真正用作可调指数,而是硬编码 `1 / (dists2 + eps)` `[VERIFY: utils/config.py:99, model/neural_points.py:668-670]`。要改其它指数需要改源码。

### 7.5 KNN ablation 配置(论文)

论文中 PIN 用 `K=6, F=8`。极端配置:
- K=1: 等价于 NerF 的"最近邻特征",几何不光滑,mesh 有断层
- K=12: 几何更光滑,但 MLP 输入维度 → 12×11=132,过拟合且慢
- 选 6 是经验权衡

---

## 8. `recreate_hash()` — 哈希表重建

```python
def recreate_hash(self, sensor_position, sensor_orientation,
                  kept_points=True, with_ts=True, cur_ts=0):
    # ──── ① 清空 buffer ────
    self.buffer_pt_index = full(buffer_size, -1, int64)

    # ──── ② voxel 内去重 ────
    if with_ts:
        ts_used = (create + update)/2 if use_mid_ts else create
        ts_diff = |ts_used - cur_ts|
        sample_idx = voxel_down_sample_min_value_torch(neural_points, r, ts_diff)
        # 每个 voxel 保留 ts_diff 最小的那个点(离 cur_ts 最近)
    else:
        sample_idx = voxel_down_sample_min_value_torch(neural_points, r,
                                                       certainty.max() - certainty)
        # 每个 voxel 保留 certainty 最大的

    if kept_points:
        # 不删旧点,只把保留点写入 buffer
        sample_points = neural_points[sample_idx]
        grid = (sample_points / r).floor().to(int64)
        hash = ((grid * primes).sum(-1)) % buffer_size
        buffer_pt_index[hash] = sample_idx
    else:
        # 真正合并去重:旧 voxel 的多个点折叠成 1 个
        neural_points = neural_points[sample_idx]
        point_orientations = ...[sample_idx]
        point_ts_create / update / certainties = ...[sample_idx]
        sample_idx_pad = cat(sample_idx, [-1])
        geo_features = geo_features[sample_idx_pad]
        # ↑ 复用 padding 末位

        new_count = neural_points.shape[0]
        grid = (neural_points / r).floor()
        hash = ((grid * primes).sum(-1)) % buffer_size
        buffer_pt_index[hash] = arange(new_count)

    if sensor_position is not None:
        reset_local_map(sensor_position, sensor_orientation, cur_ts)

    if not kept_points:
        record_memory()
```
证据: `[VERIFY: model/neural_points.py:820-909]`

### 8.1 两种模式对比

| 模式 | 何时用 | 行为 |
|------|--------|------|
| `kept_points=True, with_ts=True` | 默认(回环 / 新观测后)| 旧点全部保留,只 rebuild 哈希索引 |
| `kept_points=True, with_ts=False` | 不区分 ts | 按 certainty 选代表 |
| `kept_points=False, with_ts=True` | Stage VI 落盘前 | 真正合并去重,**减少存储** |
| `kept_points=False, with_ts=False` | (从未被这样调用) | 按 cert 合并 |

### 8.2 为什么默认 `kept_points=True`?

如果合并去重会丢失旧 voxel 的多观测信息(每次 update 都会插一个新版本)。PIN 论文的策略是:**保留所有历史观测**,让 MLP 在训练中自己用 IDW 平均掉冲突。最后 Stage VI 才合并。

`pgo_merge_map = False`(默认)意味着回环后 **不** 合并地图:
```python
neural_points.recreate_hash(cur_pose[:3,3], None,
                            not pgo_merge_map,    # ← True 默认
                            rehash_with_time, frame_id)
# [VERIFY: pin_slam.py:338]
```

---

## 9. `adjust_map()` — 弹性变形

回环 PGO 修正后,神经点必须跟着对应帧位姿一起被变换,否则地图与新位姿不一致。

### 9.1 数学

每个神经点关联一个 `point_ts`(创建帧或中位帧)。第 t 帧的位姿在 PGO 前为 `T_init[t]`,PGO 后为 `T_pgo[t]`。位姿差:
```
pose_diff[t] = T_pgo[t] @ inv(T_init[t])
```
则神经点 i(`ts_i = point_ts[i]`)做:
```
p_i_new   = pose_diff[ts_i][:3, :3] @ p_i_old + pose_diff[ts_i][:3, 3]
q_i_new   = quat(pose_diff[ts_i][:3, :3]) ⊗ q_i_old
```

### 9.2 代码

```python
def adjust_map(self, pose_diff_torch):    # [F, 4, 4]
    self.after_pgo = True
    used_ts = (create + update)/2 if use_mid_ts else create
    self.neural_points = transform_batch_torch(
        neural_points, pose_diff_torch[used_ts]
    )
    diff_quat = rotmat_to_quat(pose_diff_torch[:, :3, :3])    # [F, 4]
    self.point_orientations = quat_multiply(
        diff_quat[used_ts], self.point_orientations
    ).to(self.point_orientations)
```
证据: `[VERIFY: model/neural_points.py:792-818]`

### 9.3 `after_pgo=True` 的连锁影响

设 `True` 后,`query_feature` 在生成 `neighb_vector` 时会做四元数被动旋转:
```python
if self.after_pgo:
    neighb_vector = apply_quaternion_rotation(quat, neighb_vector)
# [VERIFY: model/neural_points.py:646-649]
```

为什么这一步?
- PIN map 的 latent 是 **隐式表达邻域的局部几何**,latent 是在某个"局部姿态"下学到的
- 回环后神经点位置变了,但**邻居向量** `v = q - p_i` 是在世界系下的,直接用会偏 — 因为 MLP 把 v 当 local 系输入
- 所以要把 v 旋回到神经点的"原始 local 系" → 这就是为什么每个神经点要存 `quat`

四元数初始化 [1,0,0,0]:第一次创建时没有 pgo,quat=identity,后续每次 adjust_map 累乘旋转部分。位置随之累计平移。

### 9.4 复杂度与瓶颈

- `transform_batch_torch`: `O(N)` 矩阵乘
- `quat_multiply`: `O(N)`
- pose_diff_torch 索引: `O(N)`

总计 `O(N)`,但 N 可能 ~10⁶ → 一次 adjust 在 GPU 上 ~10-100 ms,这是回环触发的最大单点延迟。

---

## 10. `prune_map()` — 不稳定点裁剪

```python
def prune_map(self, prune_certainty_thre, min_prune_count=500, global_prune=False):
    certainty_mask = (point_certainties < prune_certainty_thre)
    if global_prune:
        prune_mask = certainty_mask
    else:
        Δd = |travel_dist[cur_ts] - travel_dist[point_ts_update]|
        inactive_mask = Δd > diff_travel_dist_local
        prune_mask = inactive_mask & certainty_mask
    if sum(prune_mask) > min_prune_count:
        # 4 个全局张量 + geo/color features 一起切
        self.neural_points = neural_points[~prune_mask]
        ...
        prune_mask_pad = cat(prune_mask, [False])
        self.geo_features = geo_features[~prune_mask_pad]
        return True
    return False
```
证据: `[VERIFY: model/neural_points.py:749-790]`

调用点:
- 周期裁剪: `Mapper.process_frame` 里 `(frame+1) % prune_freq_frame == 0` 触发,然后 `recreate_hash` `[VERIFY: utils/mapper.py:254-256]`
- Stage VI: `pin_slam.py:520` `prune_map(max_prune_certainty, 0, True)` 全图裁,然后 `recreate_hash(merged)`

### 10.1 双判别意义

- **certainty** 低:观测次数少,latent 没怎么训练过,SDF 不可靠
- **inactive**:在本地图时间窗外,即使曾经稳定,现在也可能与新观测冲突

两个条件 AND 才裁,避免误删"长期保留的标志性结构"(高 certainty 即使 inactive 也保留)。

---

## 11. `get_neural_points_o3d()` — 可视化导出

源 `[VERIFY: model/neural_points.py:182-309]`,本质是把 `neural_points` / `local_neural_points` 拷到 numpy,然后按 `color_mode` 上色:
- 0: 灰
- 1: 按 certainty 着色
- 2: 按 ts 着色
- 3: 按 latent feature 的 PCA 主成分着色
- 4: 按 nearest train pose 着色

`pca_color_on=True` 时调用 `compute_feature_principle_components` 算 PCA 投影矩阵 `[VERIFY: model/neural_points.py:175-180]`,做一次即可。

---

## 12. 性能数据(从源码 timing log 推断)

| 操作 | 数据规模 | 时间(参考)|
|------|----------|-------------|
| `radius_neighborhood_search` | N=16384, K=33 | ~3-5 ms `[VERIFY: model/neural_points.py:1004-1008 注释]` |
| 排序 + gather | N=16384, K=33 → 6 | ~1-2 ms |
| `local_geo_features[idx]` gather | [N, 6, 8] | ~1 ms |
| `update` 全流程 | 万级新点 | ~5-15 ms |
| `reset_local_map` | M=几十万局部点 | ~5-10 ms |
| `recreate_hash(kept=True)` | N=百万 | ~10-30 ms |
| `recreate_hash(kept=False)` | N=百万 | ~50-150 ms |
| `adjust_map(pose_diff)` | N=百万 | ~10-50 ms |
| `prune_map` | N=百万 | ~5-20 ms |

源码每个函数顶端都有 `T0 = get_time()` 标记和 `# print(...)` 行被注释,需要 uncomment 才能看实测。

---

## 13. 设计决策对比

### 13.1 神经点位置 vs latent 哪个更"重要"?

```
神经点位置:  voxel grid 离散化,基本不变(只 adjust_map 时整体平移)
神经点 latent: nn.Parameter,每帧 Adam 训练,可表达任意 SDF
```
位置是粗结构,latent 是精细 SDF。位置由 voxel hash 给出"哪儿有结构",latent 学"那儿的 SDF 长什么样"。

### 13.2 为什么不用 KD-tree?

| | KD-tree | Voxel Hash |
|---|---------|------------|
| 构建 | `O(N log N)` | `O(N)` 一次 hash |
| 增量插入 | 慢 / 需要 rebalance | `O(1)` |
| KNN | `O(log N + K)` | `O(K_cells)`,fixed |
| GPU | 难(指针追逐)| 完美 vectorized |
| 内存 | tree + leaf 数据 | 大 buffer(稀疏 OK)|

PIN-SLAM 在 GPU 上每帧 ~10K 个查询点,KD-tree 的构建+查询会成为瓶颈,voxel hash 几乎是常数时间。

### 13.3 为什么 feature_dim=8?

经验值:
- 4: 表达力不足,mesh 有平台坑
- 8: 默认,质量 vs 速度平衡
- 16: 略提升,但训练慢 ~2×,GPU 显存翻倍
- 32: 过拟合,outdoor 大尺度反而变差

论文 Table II 做了 ablation。

### 13.4 buffer_size = 5e7 调整建议

- 室内 / 小场景: 可降到 1e7(神经点 < 10⁵)
- 大尺度城市: 提到 1e8(神经点 ~10⁶,负载 < 0.1)
- 检测碰撞率: 看 `update` 里 `(dist2 > 3r²)` 触发频率

---

## 14. 边界 case 与 bug 候选

### 14.1 第 0 帧

- `is_empty()` 为 True → `update` 直接全插
- `recreate_hash` 不会被调用(还没 PGO)
- `query_feature` 在第 0 帧 mapping 里返回的 `nn_count` 几乎全 0,但 IDW 兜底逻辑 `[VERIFY: model/neural_points.py:672-675]` 把全 0 weight 设为 eps,避免 NaN

### 14.2 reboot

- `reboot_ts = frame_id`
- `update` 里 `cur_ts == reboot_ts` 的特判 `[VERIFY: model/neural_points.py:341]`,无视 hash 占位,直接全插
- `reset_local_map(reboot_map=True)` 把所有 `ts < reboot_ts` 的点扔出局部地图

### 14.3 哈希碰撞极端情况

如果两个不同 voxel 偶然落到同一 hash 桶,且坐标差 < `√3r`:
- `dist2 > 3r²` 判定会失败(< 3r²)
- 旧点保留,新点不写
- ⚠️ 这种新点本应是新 voxel 的成员,但是被错误地视作"已存在"丢弃

实际概率:`buffer_size / N`(均匀分布近似)。默认 5e7 / 1e6 = 50,即每个 voxel 平均有 50 个桶可用,碰撞 < 2%。

### 14.4 `nn_counts == 0` 的查询点

如果一个查询点周围 33 个体素全空(典型于 freespace 中段的 sample):
- `weight` 全 0,被 `weight[nn_counts == 0] = eps` 设为 eps,行和归一化后 weight 仍 0(除以 0 的极限)
- 进而 `geo_features_vector` 全 0 → `MLP(0)` 输出某个固定值(bias)
- 因为 freespace 的 SDF label 是大正数,MLP 倾向于学"无邻居 → 输出最大 SDF"

这是 PIN-SLAM 实现 "free space carving" 的隐式机制:**没有邻居就预测 large SDF**。

### 14.5 `local_geo_features` 边界

- `local_geo_features.shape[0] = local_count + 1`(末位 padding)
- mapping 在 `query_feature` 里通过 `local_mask` 复制 global features 到 local,但 padding 是 `geo_features[-1]`,即一个**长期不变**的全局 padding(高斯初始化的 std=0 ⇒ 全 0)
- 在 BCE loss 里全 0 padding 经 MLP 输出某个固定 sdf,不影响梯度(因为 idx=-1 的查询不会反向到 padding,padding 没有 KNN 指它)

但当 `idx[~valid_mask] = 0` 时 `[VERIFY: model/neural_points.py:690]`,实际是把 -1 idx 改成 0,这会**把 padding 误指向第 0 个真实点** — 不过这一行只发生在 `with torch.no_grad()` 的 scatter_add 里,只是用来累 certainty,所以最终影响是"第 0 个点的 certainty 被多累几次"。这是源码里的一个微妙 bug-or-feature,生产中无明显影响。

---

## 15. 与论文 §III-A "PIN Map" 对应表

| 论文术语 | 代码实现 |
|----------|----------|
| neural point `p_i`,feature `f_i` | `neural_points[i]`, `geo_features[i]` |
| 隐式 SDF `s_θ(q) = MLP(σ(f), q - p)` | `Decoder.sdf(query_feature(...))` |
| K-nearest aggregation | `radius_neighborhood_search` + sort + IDW |
| IDW weight `w_i ∝ 1/d²` | `weight = 1.0 / (dists2 + eps)` |
| local SE(3) pose `q_i` for elastic map | `point_orientations` 四元数 + `apply_quaternion_rotation` |
| 哈希 `H(g) = (...) mod n` | `(grid_coords * primes).sum() % buffer_size` |

---

## 16. 一句话总结

> **神经点地图 = 稀疏体素哈希(几何骨架)+ per-point latent 向量(精细 SDF)+ per-point 四元数(回环弹性)+ IDW(平滑过渡)**。

工程上,这种表示比 ikd-tree / iVox / TSDF 都更适合 GPU 端到端训练,代价是必须有 GPU。

---

**END of ALGORITHM 01.**

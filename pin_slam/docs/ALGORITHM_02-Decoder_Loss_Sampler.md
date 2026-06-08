# ALGORITHM 02 — Decoder + Loss + Sampler

> Phase 4 文档系列之二。对应 `model/decoder.py`(113 行)+ `utils/loss.py`(176 行)+ `utils/data_sampler.py`(260 行)。
> 路径相对 `src/PIN_SLAM/`。

---

## 0. TL;DR

这三个模块联合定义了"PIN 系统怎么从点云监督 SDF 网络":

1. **Sampler** — 把每个 LiDAR 测量沿射线扩成 7 个 sample(1 测量 + 3 surface + 2 free_front + 1 free_behind),并给每个 sample 一个带符号 SDF label
2. **Decoder** — 共享 MLP,从 `[neural_point_feat, q - p]` 输入预测 SDF / 颜色 / 语义
3. **Loss** — 默认 BCE(把 SDF 通过 sigmoid 转 occupancy prob,与 label 做 cross entropy);备选 L1/L2/Zhong

它们和 `NeuralPoints.query_feature`(ALGORITHM 01)拼起来就是一次 forward。

---

## 1. Sampler 详解

### 1.1 几何动机

LiDAR 一束射线打到表面,提供的是"那个点的 SDF=0"信息。但只用零样本训练 MLP 远远不够:
- 表面前方的空间应该是正 SDF(自由空间)
- 表面后方很短一段应该是负 SDF(物体内部)
- 表面附近应该有 SDF 平滑过渡

所以每个测量被扩成 7 个 sample,覆盖一段沿射线的 SDF 监督。

### 1.2 7 个 sample 的位置

设射线方向单位向量 `r̂`,深度 `d`,真实击中点 `q* = d · r̂`。

| Part | n | dist_ratio | displacement | SDF label |
|------|---|------------|--------------|-----------|
| 0 (measurement) | 1 | 1.0 | 0 | 0 |
| 1 (surface) | 3 | `1 + ε / d`,`ε ∼ 𝒩(0, σ)` | `ε` | `-ε`(behind 为正 → 取反)|
| 2 (free_front) | 2 | `U[ratio_min, 1 - 2σ/d]` | `(ratio - 1) · d`(负值)| `-(ratio-1)·d` 即正 |
| 3 (free_behind) | 1 | `U[1 + 2σ/d, 1 + L/d]` | `(ratio - 1) · d`(正值)| `-(ratio-1)·d` 即负 |

其中:
- σ = `surface_sample_range_m`(默认 0.25 m)
- ratio_min = `free_sample_begin_ratio`(默认 0.3)
- L = `free_sample_end_dist_m`(默认 1.0 m,可被 yaml 改为 `4σ`)
- `sigma_ratio = 2.0` 写死,保证 freespace 区段距离表面至少 2σ,避免和 surface 段重叠

证据(逐 part):
- Part 0: `[VERIFY: utils/data_sampler.py:44-46]`
- Part 1: `[VERIFY: utils/data_sampler.py:48-66]`(`torch.randn × σ`)
- Part 2: `[VERIFY: utils/data_sampler.py:67-89]`(`torch.rand × (max - min) + min`)
- Part 3: `[VERIFY: utils/data_sampler.py:90-110]`

### 1.3 SDF label 的符号约定

代码里有两次符号操作,容易看错:

1. `sdf_label = displacement` 直接赋值
   - displacement 是"距离表面的 signed offset",surface 段是 `ε`(正负都有),free_front 段是负(在表面前方),free_behind 段是正(在表面后方)
   - 来源 `[VERIFY: utils/data_sampler.py:175-177]`

2. `sdf_label_tensor *= -1`
   - 来源 `[VERIFY: utils/data_sampler.py:220]`,注释 `# convert to the same sign as`(不完整,但意思是把符号转成 "正 = freespace,负 = 物体内部")
   - 之后下游 Decoder.sdf 用的也是这个约定,所以一致

### 1.4 weight 的双重含义

`weight_tensor` 既是"距离衰减权重"也是"sample 类型 flag":

```python
weight_tensor = ones_like(depths)
if dist_weight_on:
    weight_tensor[:surface_count] = 1 + 0.5·s - (d/max_range)·s   # [0.6, 1.4]
weight_tensor[surface_count:] *= -1.0    # ← 关键
```
证据 `[VERIFY: utils/data_sampler.py:142-168]`

- **绝对值**: 距离权重(远点权小,近点权大)
- **符号**: 正 = surface,负 = freespace

下游使用方:
- mapper.mapping 里 `weight = abs(weight).detach()` 把绝对值取出来给 BCE `[VERIFY: utils/mapper.py:729-731]`
- `surface_mask = |sdf_label| < σ` 单独判 surface 与否 `[VERIFY: utils/mapper.py:673-675]`
- 颜色 loss 只取 `weight[surface_mask]`,即只在表面附近监督颜色

### 1.5 ray-major → sample-major 重排

源码末尾的 reshape 是为了把张量从 "所有 ray 的 part0 / 所有 ray 的 part1 / ..." 的顺序,转成 "ray0 的 7 个 sample / ray1 的 7 个 sample / ..." 的顺序:

```python
all_sample_points = (all_sample_points
    .reshape(all_sample_n, -1, 3)     # [7, N, 3]
    .transpose(0, 1)                   # [N, 7, 3]
    .reshape(-1, 3))                   # [7N, 3] 但是 ray-major
```
证据 `[VERIFY: utils/data_sampler.py:211-242]`

⚠️ 这个重排很容易看错。重排后的索引含义:
- `all_sample_points[0..6]` 是 ray 0 的 7 个 sample
- `all_sample_points[7..13]` 是 ray 1 的 7 个 sample
- ...

### 1.6 设计权衡

| 替代方案 | 优 | 缺 | 是否采用 |
|----------|----|----|----------|
| 只用 measurement 点(N 个) | 简单 | 监督信息少,需更多帧才能收敛 | ❌ |
| 全均匀采样整条射线(N×M 个) | 监督密集 | 训练样本 ×M,GPU 内存爆 | ❌ |
| **测量 + surface 高斯 + freespace 均匀** | 表面密集 + freespace 稀疏 | 需要超参 σ / ratio | ✅ |
| 真实 ray-marching(类似 NeRF) | 物理正确 | 仅 N × 7 也够,而且 NeRF 式 marching 慢 100× | ❌ |

PIN-SLAM 的 7-per-ray 是 KISS-ICP 风格(只取测量)与 NeRF 风格(密集 marching)的折衷。

### 1.7 性能数据

源码注释里:
- "all super fast, all together in 0.5 ms" `[VERIFY: utils/data_sampler.py:251]`

实测 100K rays × 7 samples = 700K samples 仅需 0.5 ms(纯 GPU 张量运算)。

---

## 2. Decoder 详解

### 2.1 网络结构

```python
class Decoder(nn.Module):
    def __init__(self, config, hidden_dim, hidden_level, out_dim,
                 is_time_conditioned=False):
        # position dim (默认 0 → 直通 3 维 [q - p])
        if use_gaussian_pe:
            position_dim = pos_input_dim + 2 * pos_encoding_band     # 3 + 0 = 3
        else:
            position_dim = pos_input_dim * (2 * pos_encoding_band + 1)  # 3 × 1 = 3

        feature_dim = config.feature_dim     # 8
        input_dim = feature_dim + position_dim  # 11

        layers = []
        for i in range(hidden_level):
            if i == 0:
                layers.append(nn.Linear(input_dim, hidden_dim, bias_on))
            else:
                layers.append(nn.Linear(hidden_dim, hidden_dim, bias_on))
        self.layers = nn.ModuleList(layers)
        self.lout = nn.Linear(hidden_dim, out_dim, bias_on)

        self.sdf_scale = 1.0
        if main_loss_type == "bce":
            self.sdf_scale = logistic_gaussian_ratio * sigma_sigmoid_m
            # = 0.55 × 0.1 = 0.055
```

证据 `[VERIFY: model/decoder.py:14-58]`

### 2.2 默认配置实例化

| Decoder | hidden_dim | hidden_level | out_dim | 实例化点 |
|---------|------------|--------------|---------|----------|
| `geo_mlp` | 64 | 1 | 1 | `pin_slam.py:139` |
| `sem_mlp` | 64 | 1 | `sem_class_count + 1` = 21 | `pin_slam.py:140`(`semantic_on` 时)|
| `color_mlp` | 64 | 1 | `color_channel`(1 或 3)| `pin_slam.py:141`(`color_on` 时)|

最小网络:
```
Linear(11, 64) → ReLU → Linear(64, 1)
                                     ↓ × sdf_scale
                                    SDF (m)
```

共 11×64 + 64 + 64×1 + 1 = 833 参数。极轻量。

### 2.3 forward 路径

```python
def mlp(self, features):
    for k, l in enumerate(self.layers):
        if k == 0:
            h = activation(l(features))
        else:
            h = activation(l(h))
    out = self.lout(h)
    return out
```
- `features.shape` 灵活:`[N, F+P]`(weighted_first)或 `[N, K, F+P]`(weighted_after)
- `squeeze(1)` 在 `sdf()` 里收掉 out_dim=1 维度

证据 `[VERIFY: model/decoder.py:61-79]`

### 2.4 sdf_scale 的关键作用

```python
def sdf(self, features):
    out = self.mlp(features).squeeze(1) * self.sdf_scale     # [N]
    return out
```
证据 `[VERIFY: model/decoder.py:83-85]`

`sdf_scale = ratio × σ`,作用是**把 MLP 的"logit"范围映射到米尺度的 SDF**。

为什么需要?
- BCE 损失内部:`label_op = sigmoid(label / σ)`,即把 SDF label 通过 sigmoid 转成 occupancy probability,σ 是 sigmoid 锐度
- MLP 输出的"原始 SDF" 经 `× scale` 后,在 BCE 里又被 `/ σ`,等价于 MLP 的输出尺度被自动缩到 `ratio × 1` 的量级,**而不是直接预测米尺度**
- 这样数值稳定性更好(MLP 输出不会跑到 ±10 这种极端值)

`ratio = 0.55` 是用 logistic 拟合高斯的最佳系数 — 见论文 §III-C eq.5:
```
sigmoid(s/σ) ≈ Φ(s / (ratio·σ))    (ratio = 0.5503...)
```

### 2.5 三种输出方法

| 方法 | 数学 | 用途 |
|------|------|------|
| `sdf(f)` | `mlp(f) × sdf_scale` | 主输出,残差 |
| `occupancy(f)` | `sigmoid(-sdf / sdf_scale)` ∈ [0,1] | dynamic_filter / mesh 提取 |
| `sem_label_prob(f)` | `log_softmax(mlp(f), dim=-1)` | 语义分类 |
| `sem_label(f)` | `argmax(sem_label_prob)` | 离散语义 |
| `regress_color(f)` | `sigmoid(mlp(f))` ∈ [0,1] | 颜色 / 强度 |

证据 `[VERIFY: model/decoder.py:81-113]`

### 2.6 `sdf()` 返回值符号

源码注释:`predict the sdf (opposite sign to the actual sdf)`(`model/decoder.py:82`)。

含义:
- 实际 SDF 约定: 物体外正,物体内负
- MLP 输出: 物体外负,物体内正

这是因为 BCE 标签里 `sigmoid(label / σ)` 把负标签映射到 [0, 0.5),正标签映射到 (0.5, 1] —— 而 occupancy 期望正标签是 "occupied"。所以 MLP 学到的"原始输出"恰好与传统 SDF 相反。`occupancy()` 里 `sigmoid(-sdf / sdf_scale)` 的负号正好抵消。

### 2.7 PositionalEncoder 与 GaussianFourierFeatures

默认 `pos_encoding_band = 0` 时:
- `PositionalEncoder.forward(x) = x`(直通)
- `featureSize = 3`

如果开启 `pos_encoding_band > 0`:
```python
# PositionalEncoder (NeRF 风格)
sin_terms = [sin(2^k · π · x) for k in 0..band-1]
cos_terms = [cos(2^k · π · x) for k in 0..band-1]
out = cat(x, sin_terms, cos_terms)        # 3 × (2·band + 1) 维
```
证据 `[VERIFY: model/neural_points.py:1077-1124]`

- `use_gaussian_pe=True`: GaussianFourierFeatures 用随机高斯矩阵投影 `[VERIFY: model/neural_points.py:1125-1143]`

实际中 PIN-SLAM 默认 band=0,因为 IDW 已经隐式编码了"q 离哪个邻居近"的信息,显式位置编码反而过拟合。

### 2.8 设计权衡:Decoder 容量

| hidden_dim × hidden_level | 参数数 | 表达力 | 推理速度 |
|---------------------------|--------|--------|----------|
| 32 × 1 | ~430 | 不够 | 最快 |
| **64 × 1** | ~830 | 默认 | 快 |
| 128 × 2 | ~33K | 好 | 中 |
| 256 × 4 | ~200K | 高 | 慢 |

PIN-SLAM 默认 64×1 因为它和大量 latent 配合(每帧 ~10⁵ 个 nn.Parameter 受训),latent 才是主要"信息存储",MLP 只是一个轻量的解码器。

---

## 3. Loss 详解

### 3.1 BCE Loss(默认主损失)

```python
def sdf_bce_loss(pred, label, sigma, weight, weighted=False, bce_reduction="mean"):
    if weighted:
        loss_bce = nn.BCEWithLogitsLoss(reduction=bce_reduction, weight=weight)
    else:
        loss_bce = nn.BCEWithLogitsLoss(reduction=bce_reduction)
    label_op = torch.sigmoid(label / sigma)         # SDF → occupancy prob ∈ [0,1]
    loss = loss_bce(pred / sigma, label_op)         # 把 pred 也按 σ 缩
    return loss
```
证据 `[VERIFY: utils/loss.py:45-63]`

#### 数学推导

设
- `s` = 真实 SDF(米),`ŝ` = 预测 SDF
- `p_obs = σ(s / σ)`,`p_pred = σ(ŝ / σ)`,σ 是 sigmoid 锐度参数

`BCEWithLogitsLoss(pred/σ, σ(s/σ))` 数学上等价于:
```
loss = -[p_obs · log(σ(ŝ/σ)) + (1 - p_obs) · log(1 - σ(ŝ/σ))]
     = -[σ(s/σ) · log(σ(ŝ/σ)) + σ(-s/σ) · log(σ(-ŝ/σ))]
```

为什么这是 SDF 监督的合理 loss?
1. 当 `s = 0`(表面)→ `p_obs = 0.5`,要求 `p_pred = 0.5` ⇔ `ŝ = 0` ✅
2. 当 `s >> 0`(远离表面外)→ `p_obs ≈ 0`,要求 `p_pred = 0` ⇔ `ŝ → -∞`(注意符号反转)
3. 当 `s << 0`(物体内部)→ `p_obs ≈ 1`,要求 `ŝ → +∞`

⚠️ 这里 `p_obs = 0` 对应"在物体外/自由空间","物体内"对应 `p_obs = 1`,所以 occupancy 的语义是 occupancy(被占据)。

#### σ 的角色

σ 决定 sigmoid 的"软度":
- σ 大:转换平缓,远距 SDF 区分度低,SDF 输出范围大
- σ 小:转换锐利,接近 surface 才有信号,远距退化为 0/1 二值
- 默认 σ = `sigma_sigmoid_m = 0.1 m`(可被 yaml 改成 `vox_down_m`)

σ 应该与 LiDAR 测量噪声同量级 — 太小则把噪声当结构,太大则边缘模糊。

#### sdf_scale 与 σ 的精妙耦合

注意 `Decoder.sdf` 的 `× sdf_scale = × (ratio × σ)`,然后 BCE 里又 `/ σ`,所以 MLP 输出最终被 `× ratio` —— **σ 被消掉了**。这意味着:
- σ 只通过 `label / σ` 影响"label 的尖锐度"
- MLP 输出尺度不受 σ 影响

这就是为什么调 σ 主要是改变 loss 对远距样本的敏感度。

### 3.2 Zhong Loss(备选)

```python
def sdf_zhong_loss(pred, label, trunc_dist=None, weight=None, weighted=False):
    loss = zeros_like(label)
    middle_point = label / 2.0
    middle_point_abs = |middle_point|
    shift_difference_abs = |pred - middle_point|
    mask = shift_difference_abs > middle_point_abs
    loss[mask] = (shift_difference_abs - middle_point_abs)[mask]
    if trunc_dist is not None:
        surface_mask = |label| < trunc_dist
        loss[surface_mask] = |pred - label|[surface_mask]
    loss *= weight
    return loss.mean()
```
证据 `[VERIFY: utils/loss.py:67-84]`

#### 几何含义

这是 SHINE-Mapping (Zhong et al. ICRA 2023) 的损失。把 SDF label 解释为"占用概率的距离":
- 让 `pred` 落在 `[0, label]` 之间总是无损失(允许 pred 提前回到 0,但不许超出 label)
- 落在区间外按 L1 惩罚
- 表面附近(trunc_dist 内)切换到普通 L1

优点:对 freespace 的容忍度大,避免 MLP 在远距过度拟合细节。

PIN-SLAM 默认不用,但代码保留。

### 3.3 L1 / L2 Loss

```python
def sdf_diff_loss(pred, label, weight, scale=1.0, l2_loss=True):
    diff = pred - label
    diff_m = diff / scale
    if l2_loss:
        loss = (weight * diff_m**2).sum() / count
    else:
        loss = (weight * |diff_m|).sum() / count
    return loss
```
证据 `[VERIFY: utils/loss.py:10-18]`

直接的 SDF 距离损失。简单但远距样本太刚性,不适合大尺度场景(论文实验中 BCE 优于 L1)。

### 3.4 Color Loss

```python
def color_diff_loss(pred, label, weight, weighted=False, l2_loss=False):
    diff = pred - label
    if not weighted:
        weight = 1.0
    else:
        weight = weight.unsqueeze(1)
    if l2_loss:
        loss = (weight * diff**2).mean()
    else:
        loss = (weight * |diff|).mean()
    return loss
```
证据 `[VERIFY: utils/loss.py:31-41]`

简单 L1,只对 `surface_mask` 内的样本算 `[VERIFY: utils/mapper.py:802-812]`。

### 3.5 Eikonal Loss(隐式约束)

在 `Mapper.mapping` 里:
```python
if ekional_loss_on and weight_e > 0:
    g_used = g                                       # 默认 ekional_add_to='all'
    eikonal_loss = ((g_used.norm(2, dim=-1) - 1.0)**2).mean()
    cur_loss += weight_e * eikonal_loss
```
证据 `[VERIFY: utils/mapper.py:760-780]`

#### 数学

Eikonal equation: `‖∇sdf‖ = 1`

这是 SDF 的**定义性约束** — 严格 SDF 的梯度模到处为 1。MLP 学出的 sdf 不一定满足此约束,加 `(‖∇‖ - 1)²` 把它推向 SDF 流形。

#### 梯度计算两种方式

PIN-SLAM 默认用 **numerical gradient**(Neuralangelo 风格):
```python
def get_numerical_gradient(self, x, sdf_x=None, eps=0.02, two_side=True):
    eps_x = [eps, 0, 0]; eps_y = [0, eps, 0]; eps_z = [0, 0, eps]
    if two_side:
        x_pos = cat(x±eps_x, x±eps_y, x±eps_z)    # 6N×3
        sdf = self.sdf(x_pos)
        ∂sdf/∂x = (sdf[:N] - sdf[N:2N]) / (2·eps)
        ...
    gradient = cat([gx, gy, gz], dim=1)            # [N, 3]
    return gradient
```
证据 `[VERIFY: utils/mapper.py:986-1036]`

vs analytical(`utils/tools.py:get_gradient`):
- `torch.autograd.grad(sdf_pred, coord, ones_like(sdf_pred))[0]` 一次 backward
- 解析梯度精确,但训练初期(MLP 噪声大)梯度也噪声大,导致 eikonal 不稳定
- numerical 用 6 次 forward,梯度被 eps 平滑,更稳

默认 `eps = voxel_size × num_grad_step_ratio = 0.3 × 0.2 = 0.06 m`,只对 `gradient_decimation=10` 的 1/10 样本算(减计算量)。

#### `ekional_add_to` 三种模式

| 选项 | 应用范围 | 几何意义 |
|------|----------|----------|
| `'all'`(默认)| 所有 sample 的梯度 | 全空间 SDF |
| `'surface'` | 仅表面样本 | 严格满足 SDF 仅在表面附近 |
| `'freespace'` | 仅 freespace 样本 | 优先 freespace 平整 |

### 3.6 Consistency Loss(梯度平滑)

`consistency_loss_on=True` 时(默认 False):
```python
near_index = randint(...)
random_shift = uniform(-r, +r, size=(N, 3))      # r = 5 cm
coord_near = coord + random_shift
g_near = get_gradient(coord_near, sdf(coord_near))
consistency_loss = (1 - cos_similarity(g[near_index], g_near)).mean()
cur_loss += weight_c × consistency_loss
```
证据 `[VERIFY: utils/mapper.py:699-725, :753-758]`

强制相邻点的梯度方向一致 → SDF 平滑。但默认关闭,因为 eikonal 已经隐含平滑。

### 3.7 Semantic Loss

NLLLoss(`weight_s = 1.0` 默认):
```python
loss_nll = nn.NLLLoss(reduction='mean')
if freespace_label_on:
    label_mask = sem_label >= 0          # -1 不参与
else:
    label_mask = sem_label > 0           # 0(freespace)也不参与
sem_loss = loss_nll(sem_pred[label_mask][::sem_label_decimation],
                    sem_label[label_mask][::sem_label_decimation])
cur_loss += weight_s × sem_loss
```
证据 `[VERIFY: utils/mapper.py:783-800]`

`sem_label_decimation` 把语义样本进一步降采样(默认 1,不降),用于平衡类别 / 加速。

### 3.8 总损失组成

```
L = sdf_bce_loss                                    # 主
  + weight_c × consistency_loss   (可选 default 0)
  + weight_e × eikonal_loss       (默认 0.5)
  + weight_s × sem_loss           (semantic_on 时)
  + weight_i × color_loss         (color_on 时)
```

证据 `[VERIFY: utils/mapper.py:728-812]`

---

## 4. 三者联动:一次完整 forward 拆解

以 `Mapper.mapping` 单次迭代为例 `[VERIFY: utils/mapper.py:600-844]`:

```python
# Step 0: 从池里抓 batch
coord, sdf_label, ts, _, sem, color, w = mapper.get_batch(global_coord=True)
# coord [B, 3] 世界系
# sdf_label [B]
# w [B] 带符号
# ts [B] int

# Step 1: 取对应 pose,如果 BA 改过则重转
poses = used_poses[ts]              # [B, 4, 4]
origins = poses[:, :3, 3]
if ba_done_flag:
    coord = transform_batch_torch(coord, poses)

# Step 2: 准备梯度
if require_gradient:
    coord.requires_grad_(True)

# Step 3: 查询神经点特征(ALGORITHM 01)
geo_feat, color_feat, weight_knn, _, _ = neural_points.query_feature(
    coord, ts, query_color_feature=color_on)
# geo_feat [B, F+P] (weighted_first) 或 [B, K, F+P]

# Step 4: SDF 解码
sdf_pred = sdf_mlp.sdf(geo_feat)     # [B]
if not weighted_first:
    sdf_pred = sum(sdf_pred * weight_knn, dim=1).squeeze(1)

# Step 5: 类别 / 颜色头(可选)
if semantic_on:
    sem_pred = sem_mlp.sem_label_prob(geo_feat)
if color_on:
    color_pred = color_mlp.regress_color(color_feat)

# Step 6: surface_mask
surface_mask = |sdf_label| < surface_sample_range_m

# Step 7: SDF 梯度(autograd or numerical)
if require_gradient:
    g = get_gradient(coord, sdf_pred)
elif numerical_grad:
    g = mapper.get_numerical_gradient(coord[::dec], sdf_pred[::dec], ε)

# Step 8: 损失叠加
loss = sdf_bce_loss(sdf_pred, sdf_label, sdf_scale, |w|, loss_weight_on)
if eikonal_loss_on:
    loss += weight_e × ((g.norm-1)²).mean()
if semantic_on:
    loss += weight_s × NLLLoss(sem_pred, sem_label)
if color_on:
    loss += weight_i × color_diff_loss(color_pred[surface_mask],
                                       color_label[surface_mask],
                                       w[surface_mask], ...)

# Step 9: Adam
opt.zero_grad()
loss.backward()
opt.step()
```

---

## 5. 实数案例

设
- 一束射线击中 d=10m 的墙
- σ = 0.25m, ratio_min=0.3, L=1m, sigma_ratio=2

7 个 sample 位置(沿 ray 距离):
| Part | i | dist_ratio | dist (m) | displacement | sdf_label |
|------|---|-----------|----------|--------------|-----------|
| 0 | 0 | 1.000 | 10.00 | 0 | 0 (× -1 → 0) |
| 1 | 1 | 1.025 | 10.25 | +0.25 | -0.25 |
| 1 | 2 | 0.985 | 9.85 | -0.15 | +0.15 |
| 1 | 3 | 1.010 | 10.10 | +0.10 | -0.10 |
| 2 | 4 | 0.450 | 4.50 | -5.50 | +5.50 |
| 2 | 5 | 0.900 | 9.00 | -1.00 | +1.00 |
| 3 | 6 | 1.075 | 10.75 | +0.75 | -0.75 |

注意:
- 表面 (i=0) 的 label = 0,通过 sigmoid(0/σ) = 0.5 → BCE 推 pred → 0
- freespace_front (i=4) label = +5.5,sigmoid(5.5/0.1) ≈ 1 → BCE 推 pred → +∞(可学到大正值)
- freespace_behind (i=6) label = -0.75,sigmoid(-7.5) ≈ 0 → BCE 推 pred → -∞(代表物体内)

---

## 6. 边界 case

### 6.1 远距 sample 数值

`free_sample_end_dist_m = 1m` 但 `free_sample_begin_ratio = 0.3` —— 对 d=100m 的远距点:
- free_front 起点 = 30m,终点 = 99.5m
- free_behind 起点 = 100.5m,终点 = 101m

这种远样本的 label 很大(几十米),sigmoid(label/σ) ≈ 1,几乎是常数,梯度信号微弱。这是为什么:
- `dist_weight_on = True` 默认开,远距样本的 weight 降到 0.6
- 训练后期(MLP 冻结后)主要靠 surface 样本细化 latent

### 6.2 σ 太小

`sigma_sigmoid_m = 0.01`(过激进):
- sigmoid 几乎二值化,SDF label 区分度差
- gradient explosion 风险
- BCE 损失对小偏差极敏感,数值不稳

### 6.3 σ 太大

`sigma_sigmoid_m = 1.0`:
- sigmoid 平滑,远距样本与近距样本无差异
- 学到的 SDF 过 smooth,mesh 边缘模糊

### 6.4 全 freespace 帧

如果点云全打到天空 / 缺失数据:
- N=0 sample,sampler 返回空张量
- Mapper.process_frame 仍 append 空,池容量不变
- mapping 仍可跑(从池里抽其它帧)

### 6.5 颜色样本与 surface mask 不对齐

`color_pool` 长度 = `coord_pool` 长度,但 color_loss 只在 surface_mask 内算。如果 `surface_mask` 在 batch 内全 False,`color_loss = nan`(空 tensor mean)。代码无显式守护,但实际几乎不发生(σ 默认 0.25,每个 ray 7 个里至少 4 个 surface)。

---

## 7. 与 PIN 论文 §III-C 对应

| 论文符号 | 代码实现 |
|----------|----------|
| `L_sdf = BCE(σ(ŝ/σ), σ(s/σ))` | `sdf_bce_loss` |
| `L_eik = (‖∇ŝ‖ - 1)²` | `((g.norm-1)²).mean()` |
| `L_color = ‖c_pred - c‖_1` | `color_diff_loss(l2_loss=False)` |
| `L_total = L_sdf + λ_e·L_eik + λ_c·L_color` | mapping loss 叠加 |
| logistic-Gaussian 近似 `σ_log = ratio · σ` | `sdf_scale = 0.55 × σ` |
| sample 7-per-ray | `1 + 3 + 2 + 1` |

---

## 8. 一句话总结

> **Sampler 把每束射线变 7 个 sample;Decoder 是个 11→64→1 的两层 MLP;Loss 是把 SDF 通过 sigmoid 转成 occupancy 后做 BCE,辅以 Eikonal 推 ‖∇‖→1。** 三者构成 PIN 的"监督信号生成 + 解码 + 优化目标"三件套。

---

**END of ALGORITHM 02.**

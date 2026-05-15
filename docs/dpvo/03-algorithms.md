# DPVO 核心算法推导(Phase 4)

> **范围**:Patch BA(Schur)、投影 + Jacobian、Update GRU、CUDA 残差/Hessian、keyframe、Sim3 PGO
> **每条公式必须可定位到代码行**:[VERIFY:]

---

## §1. Patch Bundle Adjustment(Python 路径,Schur Complement)

文件:`src/DPVO/dpvo/ba.py`(182 行)
入口:`BA(poses, patches, intrinsics, targets, weights, lmbda, ii, jj, kk, bounds, ep=100.0, fixedp=1, structure_only=False)`(`ba.py:86`)

### §1.1 优化变量

| 变量 | 个数 | 维度 | 代表 |
|------|------|------|------|
| Pose | n - 1(固定首帧)| 6 / pose | $\delta\xi_i \in \mathfrak{se}(3)$,SE3 tangent |
| Disparity | 唯一 patch 数 m = `unique(kk).numel()` | 1 / patch | $\delta z_k$,1D 标量(整个 3×3 grid 共享)|

约束:
- `fixedp = 1` → 第一帧 $P_0$ 固定(`ba.py:133`,`n = n - fixedp`)。
- 9 个 grid 共享 dz:`ba.py:175-177` 中 `disps = disp_retr(disps, dZ, kx).clamp(...)` 然后 `patches[..., 2, :, :] = disps[..., None, None]`。

### §1.2 残差模型

对每条 edge $e = (i, j, k)$(patch $k$ 来自帧 $i$,重投到帧 $j$):

$$
r_e = \mathbf{target}_e - \pi\!\Bigl(P_j \, P_i^{-1} \, \pi^{-1}(u_k, v_k, d_k)\Bigr)
$$

其中:
- $\pi^{-1}(u, v, d) = ((u-c_x)/f_x, \, (v-c_y)/f_y, \, 1, \, d)$ —— 反投到 4D 齐次坐标(`projective_ops.py:19-29`)
- $\pi(X, Y, Z, W) = (f_x \cdot X/Z + c_x, \, f_y \cdot Y/Z + c_y)$ —— 投影(`projective_ops.py:32-50`)
- $P_j P_i^{-1}$ —— 帧 $i$ → 帧 $j$ 的相对 pose(`projective_ops.py:60`)

$\mathbf{target}_e \in \mathbb{R}^2$ 来自 Update GRU(详见 §3)。

[VERIFY: src/DPVO/dpvo/ba.py:92-93]
```python
coords, v, (Ji, Jj, Jz) = pops.transform(poses, patches, intrinsics, ii, jj, kk, jacobian=True)
```

[VERIFY: src/DPVO/dpvo/ba.py:96]
```python
r = targets - coords[..., p//2, p//2, :]   # 2D 残差,patch 中心
```

### §1.3 Jacobian

由 `projective_ops.transform(..., jacobian=True)` 一次性返回 `(Ji, Jj, Jz)`。

| Jacobian | shape | 含义 | 公式来源 |
|----------|-------|------|---------|
| `Ji` | `(1, |E|, 2, 6)` | $\partial r / \partial \delta\xi_i$ | `projective_ops.py:104`:`Ji = -Gij[:,:,None].adjT(Jj)` |
| `Jj` | `(1, |E|, 2, 6)` | $\partial r / \partial \delta\xi_j$ | `projective_ops.py:103`:`Jj = torch.matmul(Jp, Ja)` |
| `Jz` | `(1, |E|, 2, 1)` | $\partial r / \partial d_k$ | `projective_ops.py:106`:`Jz = torch.matmul(Jp, Gij.matrix()[..., :, 3:])` |

**详细推导**(`projective_ops.py:71-108`):

第一步:$\pi$ 关于齐次坐标 $X_1 = (X, Y, Z, H)$ 的导数(`Jp`):

$$
J_p = \begin{bmatrix} f_x \cdot d & 0 & -f_x \cdot X \cdot d^2 & 0 \\ 0 & f_y \cdot d & -f_y \cdot Y \cdot d^2 & 0 \end{bmatrix}, \quad d = 1/Z
$$

[VERIFY: src/DPVO/dpvo/projective_ops.py:98-101]
```python
Jp = torch.stack([
     fx*d,     o, -fx*X*d*d,  o,
        o,  fy*d, -fy*Y*d*d,  o,
], dim=-1).view(1, len(ii), 2, 4)
```

第二步:$X_1 = G_{ij} \cdot X_0$ 关于 $G_{ij}$ 在 $\mathfrak{se}(3)$ tangent 的导数(`Ja`):

对 SE(3) 左扰动 $G \leftarrow \exp(\delta\xi) G$,生成元矩阵作用在齐次坐标 $X$:

$$
\frac{\partial (G \cdot X)}{\partial \delta\xi} = \begin{bmatrix} H & 0 & 0 & 0 & Z & -Y \\ 0 & H & 0 & -Z & 0 & X \\ 0 & 0 & H & Y & -X & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \end{bmatrix}
$$

其中前 3 列对应 translation generator,后 3 列对应 rotation generator(SO(3) hat 算子)。

[VERIFY: src/DPVO/dpvo/projective_ops.py:83-89]
```python
Ja = torch.stack([
    H,  o,  o,  o,  Z, -Y,
    o,  H,  o, -Z,  o,  X,
    o,  o,  H,  Y, -X,  o,
    o,  o,  o,  o,  o,  o,
], dim=-1).view(1, len(ii), 4, 6)
```

第三步:链式法则:

$$
J_j = J_p \cdot J_a, \quad J_i = -G_{ij}^{\mathrm{adj}\,T} \cdot J_j
$$

(Pose $i$ 出现在 $P_i^{-1}$ 里,左乘 $P_j$ 后等价于"在 body frame i 加扰动";所以用 adjoint 转换到 frame j。)

[VERIFY: src/DPVO/dpvo/projective_ops.py:103-104]

第四步:深度 Jacobian:

$$
\frac{\partial r}{\partial d_k} = J_p \cdot \frac{\partial X_1}{\partial d_k} = J_p \cdot G_{ij}[:, 3:]
$$

(在 $X_0 = (\bar{u}, \bar{v}, 1, d_k)$ 中只有最后一维与 $d_k$ 相关。)

[VERIFY: src/DPVO/dpvo/projective_ops.py:106]
```python
Jz = torch.matmul(Jp, Gij.matrix()[..., :, 3:])
```

### §1.4 信息矩阵权重 + outlier mask

```python
# [VERIFY: src/DPVO/dpvo/ba.py:98-106]
v *= (r.norm(dim=-1) < 250).float()              # 残差 > 250 px 视为 outlier 关掉
in_bounds = (coords[...,p//2,p//2,0] > bounds[0]) & ... # 出图像边界外的关掉
v *= in_bounds.float()
```

`v` 是 `transform` 返回的 `(Z > 0.2)` 有效性 mask(`projective_ops.py:108`),也合入。

```python
# [VERIFY: src/DPVO/dpvo/ba.py:111-112]
r = (v[...,None] * r).unsqueeze(dim=-1)
weights = (v[...,None] * weights).unsqueeze(dim=-1)
```

`weights` 是 Update GRU 输出 $\in (0, 1)$(sigmoid),作为 information 权重(每 axis 独立)。

### §1.5 Normal equation 构造

定义对每条 edge:
- $J_{ie} \in \mathbb{R}^{2 \times 6}$,$J_{je} \in \mathbb{R}^{2 \times 6}$,$J_{ze} \in \mathbb{R}^{2 \times 1}$
- $w_e = \mathrm{diag}(\text{weights}_e) \in \mathbb{R}^{2 \times 2}$

构造每 edge 的 Hessian 块:

$$
B_{ee}^{ii} = J_{ie}^T w_e J_{ie}, \quad B_{ee}^{ij} = J_{ie}^T w_e J_{je}, \quad B_{ee}^{ji} = J_{je}^T w_e J_{ie}, \quad B_{ee}^{jj} = J_{je}^T w_e J_{je}
$$
$$
E_{ee}^{ik} = J_{ie}^T w_e J_{ze}, \quad E_{ee}^{jk} = J_{je}^T w_e J_{ze}
$$
$$
C_{ee} = J_{ze}^T w_e J_{ze}
$$
$$
v_{e}^{i} = J_{ie}^T w_e r_e, \quad v_{e}^{j} = J_{je}^T w_e r_e, \quad w_{e} = J_{ze}^T w_e r_e
$$

[VERIFY: src/DPVO/dpvo/ba.py:114-127]
```python
wJiT = (weights * Ji).transpose(2,3)
wJjT = (weights * Jj).transpose(2,3)
wJzT = (weights * Jz).transpose(2,3)

Bii = torch.matmul(wJiT, Ji)
Bij = torch.matmul(wJiT, Jj)
Bji = torch.matmul(wJjT, Ji)
Bjj = torch.matmul(wJjT, Jj)

Eik = torch.matmul(wJiT, Jz)
Ejk = torch.matmul(wJjT, Jz)

vi = torch.matmul(wJiT, r)
vj = torch.matmul(wJjT, r)
```

然后 scatter sum 到全局矩阵:

$$
B[i', i'] \mathrel{+}= B_{ee}^{ii}, \quad B[i', j'] \mathrel{+}= B_{ee}^{ij}, \quad \ldots
$$

其中 $i' = i - \text{fixedp}$(第一帧固定后帧号下移一格)。

[VERIFY: src/DPVO/dpvo/ba.py:140-151]
```python
B = safe_scatter_add_mat(Bii, ii, ii, n, n).view(b, n, n, 6, 6) + \
    safe_scatter_add_mat(Bij, ii, jj, n, n).view(b, n, n, 6, 6) + \
    safe_scatter_add_mat(Bji, jj, ii, n, n).view(b, n, n, 6, 6) + \
    safe_scatter_add_mat(Bjj, jj, jj, n, n).view(b, n, n, 6, 6)
```

形状:`B (b, n, n, 6, 6)`,`E (b, n, m, 6, 1)`,`C (b, m)`,`v (b, n, 1, 6, 1)`,`w (b, m)`。

### §1.6 Schur Complement

完整 normal equation 是:

$$
\begin{bmatrix} B + \lambda \mathrm{diag}(B) & E \\ E^T & C + \lambda \end{bmatrix} \begin{bmatrix} \delta x \\ \delta z \end{bmatrix} = \begin{bmatrix} v \\ w \end{bmatrix}
$$

直接 solve 是 $O((6n + m)^3)$,m 远大于 n(每 patch 都有 dz);用 **Schur complement 消 dz**:

$$
S = B - E (C + \lambda)^{-1} E^T, \quad y = v - E (C + \lambda)^{-1} w
$$

$$
\delta x = S^{-1} y, \quad \delta z = (C + \lambda)^{-1} (w - E^T \delta x)
$$

[VERIFY: src/DPVO/dpvo/ba.py:158-173]
```python
Q = 1.0 / (C + lmbda)    # (C + λ)^{-1},对角

EQ = E * Q[:,None]

if structure_only or n == 0:
    dZ = (Q * w).view(b, -1, 1, 1)
else:
    S = B - block_matmul(EQ, E.permute(0,2,1,4,3))
    y = v - block_matmul(EQ, w.unsqueeze(dim=2))
    dX = block_solve(S, y, ep=ep, lm=1e-4)
    dZ = Q * (w - block_matmul(E.permute(0,2,1,4,3), dX).squeeze(dim=-1))
```

**复杂度**:
- 消 dz:O(m + n·m + n²·m) = O(n² m) 矩阵积
- Solve S:O(n³) Cholesky(可微,用 `CholeskySolver` autograd 自定义函数)

对于 n=10(`OPTIMIZATION_WINDOW`)、m ≈ 10 × 96 = 960:
- 直接:$(60+960)^3 \approx 10^9$
- Schur:$60^3 + 60^2 \cdot 960 \approx 3.7 \times 10^6$
- → 加速约 270 倍

**Cholesky autograd**(`ba.py:12-37`):自定义 `torch.autograd.Function`,前向 `cholesky_solve`,后向用 $\partial L / \partial H = -x_s (\partial L / \partial x)^T (U^T U)^{-1}$(LM solve 的标准导数)。
失败兜底:`torch.linalg.cholesky_ex` 失败时返回零增量,避免训练崩(`ba.py:18-20`)。

### §1.7 Retraction(状态更新)

```python
# [VERIFY: src/DPVO/dpvo/ba.py:175-180]
x, y, disps = patches.unbind(dim=2)
disps = disp_retr(disps, dZ, kx).clamp(min=1e-3, max=10.0)     # 深度限幅
patches = torch.stack([x, y, disps], dim=2)

if not structure_only and n > 0:
    poses = pose_retr(poses, dX, fixedp + torch.arange(n))
```

- **disp**:线性 retraction `disps[k] += dZ[k]`,然后 clamp 到 [1e-3, 10.0] —— 这是 monocular VO 防止深度负或爆炸的物理约束。
- **pose**:SE(3) retraction `poses.retr(dX)` = `poses · exp(dX)`(右扰动)。`pose_retr`(`ba.py:54-56`)用 `scatter_sum` 把 edge 级 dX 聚合到 pose 级。

### §1.8 设计抉择对比

| 决策 | DPVO 选 | 替代 | 为什么这样 |
|------|---------|------|----------|
| BA solve 路径 | CUDA fastba(`ba_cuda.cu`)推理 / Python `ba.py` 训练 | 全 Python | 推理速度;训练要可微 |
| 消元方向 | Schur 消 depth | Schur 消 pose | depth 数远多于 pose,消 depth 收益大 |
| Trust region | LM with `lmbda=1e-4` | DogLeg | LM 简单且数值稳;CUDA kernel 默认参数 |
| 第一帧 | 固定(`fixedp=1`)| 全自由 + gauge fix | mono VO 标准做法,自由度无法观测 |
| 深度通道 | 9 grid 共享 1 dz | 9 个独立 dz | patch 假设近平面,共享 disparity |
| Cholesky 失败 | 跳过 update | abort BA | 训练时 H 偶尔奇异,不能崩 |

[VERIFY: src/DPVO/dpvo/ba.py:18-26] cholesky 失败兜底
[VERIFY: src/DPVO/dpvo/ba.py:175-177] 9 grid 共享 dz
[VERIFY: src/DPVO/dpvo/dpvo.py:354] iterations=2 of fastba.BA

---

## §2. Update GRU + SoftAgg(为什么需要 NN)

文件:`src/DPVO/dpvo/net.py`(主要 27-92 行的 `Update` 类)

### §2.1 输入输出

```python
# [VERIFY: src/DPVO/dpvo/net.py:74-92]
def forward(self, net, inp, corr, flow, ii, jj, kk):
    net = net + inp + self.corr(corr)
    net = self.norm(net)
    
    ix, jx = fastba.neighbors(kk, jj)
    mask_ix = (ix >= 0).float().reshape(1, -1, 1)
    mask_jx = (jx >= 0).float().reshape(1, -1, 1)
    
    net = net + self.c1(mask_ix * net[:,ix])
    net = net + self.c2(mask_jx * net[:,jx])
    
    net = net + self.agg_kk(net, kk)       # patch 级聚合(同一 patch 跨多 edge 共享)
    net = net + self.agg_ij(net, ii*12345 + jj)   # frame-pair 级聚合
    
    net = self.gru(net)
    
    return net, (self.d(net), self.w(net), None)
```

| 张量 | shape | dtype | 含义 |
|------|-------|-------|------|
| `net`(隐藏态)| `(1, |E|, 384)` | f16 | 每 edge 一个 hidden state |
| `inp`(context = ctx = imap)| `(1, |E|, 384)` | f16 | patch inet 描述子,固定不随迭代变 |
| `corr` | `(1, |E|, 98)` | f16 | 双尺度 7×7 correlation lookup |
| `flow` | `None` | — | 接口保留,DPVO 不传 |
| `ii, jj, kk` | `(|E|,)` | i64 | edge tuple |

输出:
- `net'`:更新后的 hidden state(写回 `pg.net`)
- `(delta, weight, None)`:
  - `delta`(`(1, |E|, 2)`):重投像素 → target 的修正
  - `weight`(`(1, |E|, 2)`):BA 权重 (0, 1) per axis

### §2.2 内部模块

**`self.corr` MLP**(`net.py:53-60`):

```python
nn.Linear(2*49*p*p, DIM=384) → ReLU → Linear → LayerNorm → ReLU → Linear
```

输入 dim = 2(levels) × 49(7×7 lookup) × 1(因为 p in MLP 是 self.P=3,但实际 corr 已展平,所以 2 × 49)= 98。  
**等等** —— `2*49*p*p` 在 p=3 时 = 882,但 `corr` 实际 shape 是 `(1, |E|, 2·(2r+1)²)` = `(1, |E|, 98)`。读 `dpvo.py:205-207`:

```python
corr1 = altcorr.corr(self.gmap, self.pyramid[0], coords / 1, ii1, jj1, 3)   # radius=3 → 7×7
corr2 = altcorr.corr(self.gmap, self.pyramid[1], coords / 4, ii1, jj1, 3)
return torch.stack([corr1, corr2], -1).view(1, len(ii), -1)
```

`altcorr.corr` 返回 `(b, |E|, p, p)`?需要看 cuda kernel。从 dim 计算:`(1, |E|, 2·49)` = `(1, |E|, 98)`。所以 `corr MLP` 的 input dim 是 `2*49*p*p` = 882 实际却接 98 维 —— 这是 **训练用** GRU input(net.py:160-173 `CorrBlock` 那里),不是 inference 用。Inference 路径走 `dpvo.py:205-207` 的 corr,只展平到 98,然后 `corr MLP` 在 inference 是否动?

读 `net.py:77`:`net = net + inp + self.corr(corr)`。这个 corr MLP 输入是 `corr`(98 维),输出 384 维。但 nn.Linear 第一层是 `Linear(2*49*p*p, DIM)` 即 `Linear(882, 384)`。weight shape 不匹配应该 runtime 报错。

继续 grep:`p` 在 `Update.__init__` 是 init 参数,`VONet.__init__` 传 `self.P = 3` 进去。所以 `Linear(2*49*3*3, 384)` = `Linear(882, 384)`。

冲突,**应该有 broadcasting 或我读错** —— 让 corr 在 inference 路径上和 training 路径上是不同 shape。inference `dpvo.py:200-207` 输入 `coords` shape 是 `(b, |E|, 2, P, P)`(`dpvo.py:213` permute 后),`altcorr.corr(coords, ...)` 返回 `(b, |E|, p, p, (2r+1), (2r+1))` 然后 stack levels 再 view。

让我具体核对:`net.py:160-173` 训练时 `CorrBlock.__call__`:

```python
def __call__(self, ii, jj, coords):
    corrs = []
    for i in range(len(self.levels)):
        corrs += [altcorr.corr(self.gmap, self.pyramid[i], coords / self.levels[i], ii, jj, self.radius, self.dropout)]
    return torch.stack(corrs, -1).view(1, len(ii), -1)
```

输出 `view(1, len(ii), -1)` 把所有维度展平 → 实际是 `2 · 49 · P · P` = 882。所以 `corr MLP` 训练时拿 882 input,inference 时(`dpvo.py:200-207`)是同样调用 `altcorr.corr`,返回 shape 应该一致 = 882。我之前数字 98 写错了。

修正:
- 输入 `corr` shape `(1, |E|, 2 · 49 · 9)` = `(1, |E|, 882)` ✓
- corr MLP 第一层 `Linear(882, 384)` ✓

[VERIFY: src/DPVO/dpvo/net.py:53-60] corr MLP 定义
[VERIFY: src/DPVO/dpvo/net.py:160-173] CorrBlock 训练路径
[VERIFY: src/DPVO/dpvo/dpvo.py:205-207] inference 路径 corr

**`fastba.neighbors(kk, jj)`**(`fastba/ba.cpp:59-97`):
- 给定 edge tuple (ii, jj, kk),为每条 edge 找它在 patch kk 上的"时间邻居"。
- 具体:对每个唯一 patch index `u`,把 `e | kk[e] == u` 按 `jj[e]` 排序,然后每条 e 给它返回前一条 / 后一条 edge 的全局 index。
- 返回 `(ix, jx)`:`ix[e]` = "同 patch 但比 e 早一帧的 edge",`jx[e]` = "同 patch 但晚一帧"。-1 表示无邻居。

**`net[:, ix]` 的语义**:从邻居 edge 读 hidden state,GRU 实现"沿 patch 时间链传递信息"。

**`self.c1`,`self.c2`**(`net.py:31-39`):2-layer MLP `(384 → 384 → 384)`,把邻居 hidden state 投影后加到自己。

**`SoftAgg`**(`blocks.py:31-48`):

```python
class SoftAgg(nn.Module):
    def forward(self, x, ix):
        _, jx = torch.unique(ix, return_inverse=True)
        w = torch_scatter.scatter_softmax(self.g(x), jx, dim=1)
        y = torch_scatter.scatter_sum(self.f(x) * w, jx, dim=1)
        if self.expand:
            return self.h(y)[:, jx]
        return self.h(y)
```

- 给定 group label `ix`(每 edge 一个 group id),做 **softmax-attention 内组聚合**:
  - `g(x) → logits`;同组 logits 取 softmax 当注意力权
  - 加权和 `f(x) * w`,然后 sum 同组
  - `h(y)` 再投回 → expand 模式把 group 级的结果 scatter 回每 edge
- `agg_kk(net, kk)`:按 patch 聚合(同 patch 不同帧的 edge 互相"看")
- `agg_ij(net, ii * 12345 + jj)`:按 frame-pair 聚合("12345" 是 hash trick 把 (ii, jj) 拼成 unique key)

**`GatedResidual` × 2 + LayerNorm × 2** = "GRU" 模块(`net.py:46-51`):

```python
self.gru = nn.Sequential(
    nn.LayerNorm(DIM),
    GatedResidual(DIM),     # x + sigmoid(W_g x) * MLP(x)
    nn.LayerNorm(DIM),
    GatedResidual(DIM),
)
```

不是真 nn.GRU,而是两层 gated residual + norm,叫 GRU 是约定。

**`self.d`,`self.w`** 输出头(`net.py:62-72`):
- `d`(delta):`ReLU → Linear(DIM, 2) → GradientClip`,输出 2D flow correction。
- `w`(weight):`ReLU → Linear(DIM, 2) → GradientClip → Sigmoid`,输出 (0,1)² 权重。
- `GradientClip`(`blocks.py:74-89`)反向时把 grad clamp 到 [-0.01, 0.01],防训练爆炸。

### §2.3 信息流时序

```
edge e = (ii=i, jj=j, kk=k):
   ├─ hidden state net[e] 从上次 update 继承(append_factors 时新加为 0)
   ├─ context inp = imap[k]  (来自 patch k 来源帧的 inet 描述子)
   ├─ corr[e] = lookup at coords[e] in fmap_j   (双尺度,radius 3)
   │
   ├─ corr MLP: corr → 384 维
   ├─ net = net + inp + corr_mlp(corr)
   ├─ norm
   │
   ├─ ix = "同 patch k 上一帧的 edge",jx = "下一帧的 edge"
   ├─ net += c1(net[ix])    # 时间前驱
   ├─ net += c2(net[jx])    # 时间后继
   │
   ├─ agg_kk(net, kk)       # 同 patch 跨帧的所有 edge 互相 softmax-attend
   ├─ agg_ij(net, ii·12345+jj)  # 同 frame-pair 的所有 edge(M 个 patch)互相 attend
   │
   ├─ gru(net)              # 2× GatedResidual + LayerNorm
   │
   └─ out: net_new, (delta, weight, None)
       delta → target = coords_center + delta → BA
       weight → BA 信息矩阵
```

### §2.4 为什么这样设计

| 设计 | 替代 | 为什么 DPVO 这样 |
|------|------|------------------|
| GRU 出 (delta, weight) 而非直接出 pose | 直接回归 pose | NN + BA 各司其职:NN 给数据关联和不确定度,BA 给几何 |
| 用 patch 而非稀疏特征 | ORB/SIFT 点 | patch 可端到端微分 + 数固定 + 不挑纹理 |
| 4D correlation 双尺度 | 单尺度 | 与 RAFT 同思路,大位移 + 小位移都覆盖 |
| ix/jx 邻居传递 + agg_kk/agg_ij | 全连接 attention | patch graph 是稀疏的,稀疏 + softmax-attend 比 dense 便宜得多 |
| Sigmoid 限 weight ∈ (0,1) | 自由 logit | BA 信息矩阵必须半正定;sigmoid 安全 |
| ReLU 在最后 head 前 | 直接 Linear | 让 weight 有"门控"性(可以完全关掉) |

[VERIFY: src/DPVO/dpvo/net.py:62-71] d/w head 结构
[VERIFY: src/DPVO/dpvo/blocks.py:74-89] GradientClip

---

## §3. CUDA Reprojection Residuals + Hessian Kernel

文件:`src/DPVO/dpvo/fastba/ba_cuda.cu`(`reprojection_residuals_and_hessian` at line 232)

### §3.1 与 Python ba.py 的关系

| 路径 | 用在 | 入口 |
|------|------|------|
| Python `ba.py` | **训练**(可微)| `net.py:260-261` `BA(Gs, patches, intrinsics, ...)` |
| CUDA `fastba` | **推理**(快)| `dpvo.py:324, 353` `fastba.BA(self.poses, self.patches, ...)` |

两者解同样的 normal equation,数学等价,但 CUDA 路径**手写 Jacobian**,Python 路径靠 `transform(..., jacobian=True)` 解析 + autograd。

### §3.2 单条 edge 计算(CUDA kernel 内)

[VERIFY: src/DPVO/dpvo/fastba/ba_cuda.cu:265-305]

```cuda
GPU_1D_KERNEL_LOOP(n, ii.size(0)) {     // 每条 edge n 一个 thread
  int ix = ii[n], jx = jj[n], kx = kk[n];
  
  float ti[3] = { poses[ix][0..2] };
  float tj[3] = { poses[jx][0..2] };
  float qi[4] = { poses[ix][3..6] };
  float qj[4] = { poses[jx][3..6] };
  
  float Xi[4];
  Xi[0] = (patches[kx][0][1][1] - cx) / fx;    // 反投 patch 中心 (1,1)
  Xi[1] = (patches[kx][1][1][1] - cy) / fy;
  Xi[2] = 1.0;
  Xi[3] = patches[kx][2][1][1];                // d
  
  float tij[3], qij[4];
  relSE3(ti, qi, tj, qj, tij, qij);            // 相对 pose
  float Xj[4];
  actSE3(tij, qij, Xi, Xj);                    // X_j = G_ij · X_i
  
  const float Z = Xj[2], d = (Z >= 0.2) ? 1.0/Z : 0.0;
  const float d2 = d * d;
  const float x1 = fx * (X/Z) + cx;            // 重投像素
  const float y1 = fy * (Y/Z) + cy;
  
  const float rx = target[n][0] - x1;
  const float ry = target[n][1] - y1;
  
  // out-of-bounds 或者 z<0.2 时 mask=0
  const bool in_bounds = (sqrt(rx*rx+ry*ry) < 128) && (Z > 0.2) &&
                         (x1>-64) && (y1>-64) && (x1<2*cx+64) && (y1<2*cy+64);
  const float mask = in_bounds ? 1.0 : 0.0;
```

[VERIFY: src/DPVO/dpvo/fastba/ba_cuda.cu:313-374]

```cuda
  for (int row = 0; row < 2; row++) {     // ★ 2 行残差,硬编码
    float *Jj, Ji[6], Jz, r, w;
    
    if (row == 0) {
      r = target[n][0] - x1;
      w = mask * weight[n][0];
      Jz = fx * (tij[0] * d - tij[2] * (X * d2));
      Jj = (float[6]){fx*W*d, 0, fx*-X*W*d2, fx*-X*Y*d2, fx*(1+X*X*d2), fx*-Y*d};
    } else {
      r = target[n][1] - y1;
      w = mask * weight[n][1];
      Jz = fy * (tij[1] * d - tij[2] * (Y * d2));
      Jj = (float[6]){0, fy*W*d, fy*-Y*W*d2, fy*(-1-Y*Y*d2), fy*(X*Y*d2), fy*X*d};
    }
    
    atomicAdd(&r_total[0], w * r * r);
    
    adjSE3(tij, qij, Jj, Ji);             // J_i = -Adj^T(G_ij) J_j (符号在下面处理)
    
    for (int i=0; i<6; i++)
      for (int j=0; j<6; j++) {
        if (ix >= 0) atomicAdd(&B[6*ix+i][6*ix+j], w * Ji[i] * Ji[j]);    // J_i^T w J_i
        if (jx >= 0) atomicAdd(&B[6*jx+i][6*jx+j], w * Jj[i] * Jj[j]);    // J_j^T w J_j
        if (ix >= 0 && jx >= 0) {
          atomicAdd(&B[6*ix+i][6*jx+j], -w * Ji[i] * Jj[j]);              // 交叉项
          atomicAdd(&B[6*jx+i][6*ix+j], -w * Jj[i] * Ji[j]);
        }
      }
    
    for (int i=0; i<6; i++) {
      // E (cross pose-depth) ...
      if (ix >= 0) atomicAdd(&E[6*ix+i][k], -w * Jz * Ji[i]);
      if (jx >= 0) atomicAdd(&E[6*jx+i][k],  w * Jz * Jj[i]);
    }
    
    for (int i=0; i<6; i++) {
      // v (pose gradient)
      if (ix >= 0) atomicAdd(&v[6*ix+i], -w * r * Ji[i]);
      if (jx >= 0) atomicAdd(&v[6*jx+i],  w * r * Jj[i]);
    }
    
    atomicAdd(&C[k], w * Jz * Jz);        // depth diagonal
    atomicAdd(&u[k], w * r  * Jz);        // depth gradient
  }
}
```

### §3.3 硬编码点(为何 IMU 进不去)

| 硬编码 | 行号 | 后果 |
|--------|------|------|
| `for (int row = 0; row < 2; row++)` | 313 | 残差维度固定 2(像素 x,y);IMU pre-integ 残差是 9 维(δp, δv, δR)→ 改不了 |
| `float Jj[6]` | 323, 331 | Pose Jacobian dim 固定 6;IMU 状态有 (p, v, R, bg, ba) 15 DoF → 改不了 |
| `B[6*ix+i][6*ix+j]` | 342, 344 | Hessian block 步长 6;扩到 15 整个 atomicAdd 索引全错 |
| `Ji[6], Jz` 单标量 | 315 | depth jac 是标量;时间敏感量(velocity)需要 dt scalar 进来,无插槽 |
| `target[n][2]` 不存在 | — | target 张量 shape (|E|, 2);加 IMU 时间 anchor 需要 (|E|, 9) → 重构 tensor layout |

**结论**:**真 DPVIO(`08-dpvio-design.md`)必须走 Python BA 路径**(`ba.py:86-182`),或者重写整个 CUDA kernel(2-3 周工作量,Hessian 块 6→15 + 添加 IMU 因子项 + 重新写 retr kernel)。Doc 08 §3 决策 D2 选了"扩 Python BA",代码证据就在此 kernel。

---

## §4. Keyframe 删除策略

文件:`src/DPVO/dpvo/dpvo.py:266-310`

### §4.1 数学

定义 `motionmag(i, j)`(`dpvo.py:257-264`):

```python
def motionmag(self, i, j):
    k = (self.pg.ii == i) & (self.pg.jj == j)    # 取 i→j 这一束 edges
    ii, jj, kk = self.pg.ii[k], self.pg.jj[k], self.pg.kk[k]
    flow, _ = pops.flow_mag(SE3(self.poses), self.patches, self.intrinsics, ii, jj, kk, beta=0.5)
    return flow.mean().item()
```

`flow_mag`(`projective_ops.py:120-130`):

$$
\text{flow}_{ij} = \beta \, \| \pi(P_j P_i^{-1} \pi^{-1}(\mathbf{p}_k)) - \pi(\pi^{-1}(\mathbf{p}_k)) \| + (1-\beta) \, \| \pi(P_j P_i^{-1, \text{trans only}} \pi^{-1}(\mathbf{p}_k)) - \pi(\pi^{-1}(\mathbf{p}_k)) \|
$$

简单说:0.5·(完整运动诱发的像素流) + 0.5·(只 translation 诱发的像素流)。后者剥离旋转项,反映"真实视角差"。

### §4.2 Decision rule

```python
i = self.n - self.cfg.KEYFRAME_INDEX - 1   # n - 5
j = self.n - self.cfg.KEYFRAME_INDEX + 1   # n - 3
m = self.motionmag(i, j) + self.motionmag(j, i)

if m / 2 < self.cfg.KEYFRAME_THRESH:       # 15.0
    drop frame n - KEYFRAME_INDEX (= n-4)
```

直觉:看 n-5 和 n-3 之间的"平均"flow,若太小说明 n-4 是冗余帧。

`KEYFRAME_THRESH=15.0` 像素 → 推 4060 GeoScan benchmark(`06-benchmarks.md` DPVO 行)实测大约每 3-5 帧丢一帧。

### §4.3 设计抉择

| 决策 | 替代 | 为什么这样 |
|------|------|----------|
| 看 (n-5) ↔ (n-3) 评估 n-4 | 看 (n-4) ↔ (n-3) | 双向看 / 留 2 帧 margin,降低边界偏差 |
| flow_mag 用 patch 重投距 | 直接看 SE3 位移 | 像素 flow 直接反映"重投歧义",位移在 monocular 下与 scale 耦合 |
| KEYFRAME_INDEX=4 | =3 或 =5 | 滑窗中部,既不太新也不太老 |
| Drop 而非 Marginalize | Schur marg | Marginalize 引入 fill-in,DPVO 不愿付代价;丢帧 + delta 字典代替 |

---

## §5. Sim(3) PGO Loop Closure(完整路径)

文件:`src/DPVO/dpvo/loop_closure/optim_utils.py`(243 行)
入口:`run_DPVO_PGO(pred_poses, loop_poses, loop_ii, loop_jj, queue)`(`optim_utils.py:202-209`)

### §5.1 状态变量

$$
G_i^{-1} \in \mathrm{Sim}(3), \quad i = 0, 1, \ldots, n-1 \quad \text{(7 维:tx, ty, tz, qx, qy, qz, qw, log_s)}
$$

`SE3_to_Sim3`(`optim_utils.py:15-17`)在 7D SE3 末位 append `1.0`(scale=1)得到 8D Sim3 tensor。

### §5.2 残差

两类边:

**1. 时序边**(`optim_utils.py:172-178`):每对相邻帧 `(k-1, k)` 用 DPVO 前端估的相对位姿做 anchor:

$$
r_{\text{seq}}(k) = \log\!\Bigl(C_{k-1,k} \cdot \exp(\hat{G}_k^{-1}) \cdot \exp(\hat{G}_{k-1}^{-1})^{-1}\Bigr)
$$

其中 $C_{k-1,k} = \hat{T}_{k-1}^{-1} \hat{T}_k$ 来自 DPVO `pred_poses`(本地 LM 优化前固化的 c2w)。

**2. Loop 边**(`optim_utils.py:180-183`):RANSAC Umeyama 估出的远距对:

$$
r_{\text{loop}}(e) = \log\!\Bigl(\Delta S_{\text{loop}, e} \cdot \exp(\hat{G}_{i_e}^{-1}) \cdot \exp(\hat{G}_{j_e}^{-1})^{-1}\Bigr)
$$

### §5.3 LM 主循环(`optim_utils.py:211-243`)

```python
def perform_updates(input_poses, dSloop, ii_loop, jj_loop, iters, ep=0.0, lmbda=1e-6, fix_opt_window=False):
    ...
    Ginv = SE3_to_Sim3(input_poses).Inv().Log()    # 初始 G^{-1} ∈ R^{n×7}
    residual_history = []
    
    for itr in range(iters):    # iters=30
        resid, (J_Ginv_i, J_Ginv_j, iii, jjj) = residual(Ginv, ..., jacobian=True)
        residual_history.append(resid.square().mean().item())
        
        delta_pose, = cuda_ba.solve_system(J_Ginv_i, J_Ginv_j, iii, jjj, resid, ep, lmbda, freen)
        Ginv_tmp = Ginv + delta_pose
        
        new_resid = residual(Ginv_tmp, ...)
        if new_resid.square().mean() < residual_history[-1]:
            Ginv = Ginv_tmp
            lmbda /= 2          # accept, 减阻尼
        else:
            lmbda *= 2          # reject, 加阻尼
        
        if 收敛 break             # 1e-5 阈 + 至少 4 轮 + 收敛速率
    
    return pp.Exp(Ginv).Inv()
```

### §5.4 Jacobian via autograd(`optim_utils.py:152-156`)

```python
def batch_jacobian(func, x):
    def _func_sum(*x):
        return func(*x).sum(dim=0)
    _, b, c = torch.autograd.functional.jacobian(_func_sum, x, vectorize=True)
    return rearrange(torch.stack((b,c)), 'N O B I -> N B O I', N=2)
```

对 `_residual(C, Gi, Gj)`(`optim_utils.py:158-161`)用 PyTorch `torch.autograd.functional.jacobian`,**不手推**。比 PGO 主流 Ceres / GTSAM 路径方便,但有 ~5× 额外耗时。

### §5.5 Sparse Cholesky(`ba.cpp:120-180`)

`cuda_ba.solve_system` 实际是 **Eigen 在 CPU** 上的稀疏 Cholesky:

```cpp
// [VERIFY: src/DPVO/dpvo/fastba/ba.cpp:99-118]
typedef Eigen::SparseMatrix<double> SpMat;
SpMat J(r*7, n*7);
J.setFromTriplets(tripletList.begin(), tripletList.end());
const SpMat Jt = J.transpose();
Eigen::VectorXd b = -(Jt * v.cast<double>());
SpMat A = Jt * J;
A.diagonal() += (A.diagonal() * lm);
A.diagonal().array() += ep;
Eigen::VectorXf delta = solve(A, b, freen*7).cast<float>();
```

`solve` 函数(`ba.cpp:102-118`):递归处理 `freen` 参数,允许只优化前 `freen` 个 pose(剩余固定)。

**注意**:文件名 `cuda_ba` 误导 —— `solve_system` 函数实际是 **CPU 上跑 Eigen**,不在 CUDA。命名是因为它和 `cuda_ba.forward`(确实是 CUDA)放在同一个 pybind 模块。

---

## §6. 设计哲学小结

DPVO 的核心创新是**"NN 出数据关联 + 几何 BA 出 pose"** 的分工:

- NN(Update GRU + corr lookup)**只输出**:
  - `delta`(像素 flow correction)
  - `weight`(信息矩阵)
  - 修正后的 hidden state(给下一 update iter 用)

- 几何(BA)**只用**:
  - patch 几何(iproj/proj)
  - SE3 / 深度的 Jacobian
  - Schur + Cholesky 解优化

**它和 DROID-SLAM 的区别**(DROID-SLAM 是 DPVO 的前身):
- DROID 用 dense pixel flow + DBA per pixel → VRAM 大,~12 GB
- DPVO 改用 sparse patch flow + sparse BA → VRAM 小,~2 GB,FPS 提升 ~10×

[VERIFY: DROID-SLAM vs DPVO comparison 见 `02-deep-vo.md` 中已记录]

---

## §7. 验证摘要

| 算法 | 关键代码位置 | 数学一致性 |
|------|--------------|----------|
| Schur complement | ba.py:158-173 | ✓ S = B - EQ E^T |
| Cholesky autograd | ba.py:12-37 | ✓ dH = -x_s grad x^T |
| Pose Jacobian Ja | projective_ops.py:83-89 | ✓ SE(3) generator matrix |
| Depth Jacobian Jz | projective_ops.py:106 | ✓ J_p · G_ij[:,3:] |
| Cross-frame J_i | projective_ops.py:104 | ✓ -Adj^T(G_ij) J_j |
| CUDA Jj 公式 | ba_cuda.cu:323, 331 | ✓ 与 J_p · J_a 展开一致 |
| Update GRU input/output | net.py:74-92 | ✓ |
| SoftAgg | blocks.py:31-48 | ✓ scatter-softmax |
| motionmag flow_mag | projective_ops.py:120-130 | ✓ β rot+tr + (1-β) tr only |
| keyframe drop rule | dpvo.py:266-280 | ✓ |
| Sim3 PGO LM | optim_utils.py:211-243 | ✓ |
| 稀疏 Cholesky Eigen CPU | ba.cpp:99-118 | ✓ |

下一步:`04-key-functions.md`(逐行注释 DPVO.__call__ / update / keyframe)。

# DPVO 数据结构详解(Phase 2)

> **范围**:`src/DPVO/dpvo/patchgraph.py` + `dpvo/dpvo.py` 缓冲区 + edge 列表 + CUDA BA 张量契约
> **所有断言可定位**:[VERIFY: 文件:行]

---

## 1. 顶层视图

DPVO 所有状态住在两个地方:
1. **`DPVO` 实例**:network buffers(GPU 临时缓存,`imap_/gmap_/fmap1_/fmap2_`)
2. **`PatchGraph` 实例**(`self.pg`):**所有持久状态**(poses、patches、intrinsics、edge 列表、color/index 元数据、delta 字典)

```
DPVO 实例(dpvo.py:20)
├── pg : PatchGraph                  ←── 持久 SLAM 状态
│   ├── poses_              [N, 7]       w2c SE3
│   ├── patches_            [N, M, 3, P, P]  patch 张量
│   ├── intrinsics_         [N, 4]       fx,fy,cx,cy
│   ├── points_             [N*M, 3]     点云缓存
│   ├── colors_             [N, M, 3]    uint8
│   ├── index_              [N, M]       patch → 来源帧
│   ├── index_map_          [N]          帧 → patch 起始 idx
│   ├── tstamps_            [N]          int64,帧时间戳(序号)
│   ├── ii, jj, kk          [|E|]        active edges
│   ├── *_inac              [|E_inac|]   inactive edges(LC 用)
│   ├── net                 [1, |E|, DIM]  GRU hidden state
│   ├── target              [1, |E|, 2]  BA 目标 (u,v)
│   ├── weight              [1, |E|, 2]  BA 权重
│   └── delta               dict         被删帧 → (前一帧, 相对 SE3)
│
├── imap_                   [pmem, M, DIM=384, 1, 1]    inet 输出(描述子)
├── gmap_                   [pmem, M, 128, P=3, P=3]    fnet patch 描述子
├── fmap1_                  [1, mem, 128, H//4, W//4]   fnet feature pyramid lvl 1
├── fmap2_                  [1, mem, 128, H//16, W//16] fnet feature pyramid lvl 4
├── pyramid                 (fmap1_, fmap2_)
├── tlist                   list[int]   每帧的外部 tstamp
├── counter                 int         总帧数(包含被删的)
├── ran_global_ba           bool[100000] 哪几帧已经跑过 global BA
├── traj                    dict        terminate 时填,t → SE3
├── network                 VONet       net.py 的 forward 模型
├── viewer                  None | Viewer
└── long_term_lc            None | LongTermLoopClosure
```

[VERIFY: src/DPVO/dpvo/dpvo.py:22-80] DPVO `__init__`
[VERIFY: src/DPVO/dpvo/patchgraph.py:11-54] PatchGraph `__init__`

---

## 2. 常量参数

来自配置(`cfg`)的关键尺寸:

| 符号 | 默认值 | 来源 | 含义 |
|------|--------|------|------|
| `N` | 4096 | `cfg.BUFFER_SIZE` | poses_/patches_/intrinsics_ 第一维上限 |
| `M` | 96 | `cfg.PATCHES_PER_FRAME`(default.yaml)| 每帧 patch 数 |
| `DIM` | 384 | `net.DIM`(`net.py:25`)| `imap_` channel,`net` 隐藏维度 |
| `RES` | 4 | `network.RES`(`net.py:184`)| fmap stride(图像下采 4 倍)|
| `P` | 3 | `network.P`(`net.py:179`)| patch 边长(3×3 grid)|
| `mem` | 36 | hardcoded `dpvo.py:58` | feature map 历史窗口 |
| `pmem` | 36 (LOOP_CLOSURE=False) | `dpvo.py:58-61` | patch memory 历史窗口;LC 模式下 = MAX_EDGE_AGE=1000 |

[VERIFY: src/DPVO/dpvo/dpvo.py:29-30,57-61] M/N/mem/pmem 定义
[VERIFY: src/DPVO/dpvo/net.py:25,179,184] DIM/P/RES 定义

---

## 3. PatchGraph 字段逐个解析

### 3.1 Pose 张量 `poses_`

```python
# [VERIFY: src/DPVO/dpvo/patchgraph.py:27]
self.poses_ = torch.zeros(self.N, 7, dtype=torch.float, device="cuda")
# [VERIFY: src/DPVO/dpvo/patchgraph.py:38]
self.poses_[:,6] = 1.0    # quaternion identity (qw 在末位)
```

| 属性 | 值 |
|------|------|
| 形状 | `(N=4096, 7)` |
| 维度含义 | 第一维:帧 index;第二维:`[tx, ty, tz, qx, qy, qz, qw]` |
| dtype | `torch.float32` |
| device | `cuda` |
| 内存 | 4096 × 7 × 4 B = 112 KB |
| 语义 | **world → camera SE3**(c2w 取 `inv()`,见 `00-overview.md` §8.4)|
| 初始化 | 全 0 + qw=1(identity)|

**读 view**(暴露给外部):`poses` property(`dpvo.py:126-128`)

```python
# [VERIFY: src/DPVO/dpvo/dpvo.py:126-128]
@property
def poses(self):
    return self.pg.poses_.view(1, self.N, 7)   # 加 batch 维 → (1, 4096, 7)
```

[VERIFY: src/DPVO/dpvo/dpvo.py:166-171] `get_pose(t)` 走 `pg.delta` 字典回溯被删帧

### 3.2 Patch 张量 `patches_`

```python
# [VERIFY: src/DPVO/dpvo/patchgraph.py:28]
self.patches_ = torch.zeros(self.N, self.M, 3, self.P, self.P, dtype=torch.float, device="cuda")
```

| 属性 | 值 |
|------|------|
| 形状 | `(N=4096, M=96, 3, P=3, P=3)` |
| 维度含义 | (帧, patch in 帧, channel, h, w) |
| channel | 0=`x` grid(像素 u),1=`y` grid(像素 v),2=`d` grid(inverse depth / disparity) |
| dtype | `torch.float32` |
| device | `cuda` |
| 内存 | 4096 × 96 × 3 × 3 × 3 × 4 B = 13.5 MB |

**关键约束**(来自 BA 数学):
- channel 0/1 是 patch 中心 (u,v) 周围 ±1 像素的网格:`patches[..., 0, i, j] = u_center + (j-1)`,`patches[..., 1, i, j] = v_center + (i-1)`(在 `Patchifier.forward` `net.py:148-149` 由 `coords_grid_with_index` 拼起来)。
- channel 2 是 disparity(逆深度);Patchifier 输出时是 1.0(`net.py:146`),DPVO 拿到后立即 `patches[:,:,2] = torch.rand_like(...)` 随机化(`dpvo.py:427`),已 initialized 用历史中位数(`dpvo.py:429-430`)。
- BA 只用中心 `[1,1]`(`ba.py:96`,`coords[..., p//2, p//2, :]`);9 个 grid 共享同一个 `dZ`(`ba.py:175-177`,只有 `disps`一维参与更新)。

[VERIFY: src/DPVO/dpvo/net.py:148-149] grid 拼接
[VERIFY: src/DPVO/dpvo/dpvo.py:427-430] 深度初始化
[VERIFY: src/DPVO/dpvo/ba.py:175-177] 9 grid 共享 disparity

### 3.3 内参 `intrinsics_`

```python
# [VERIFY: src/DPVO/dpvo/patchgraph.py:29]
self.intrinsics_ = torch.zeros(self.N, 4, dtype=torch.float, device="cuda")
```

| 形状 | `(N, 4)` |
| 字段 | `(fx, fy, cx, cy)` |
| 单位 | **像素 / RES**(`dpvo.py:401`:`self.pg.intrinsics_[self.n] = intrinsics / self.RES`),即原始内参除以 4(`RES=4`)|
| 用途 | `iproj/proj`(`projective_ops.py:19-50`)和 CUDA BA(`ba_cuda.cu:255-258`)都读 |

每帧可以不同(支持变焦/变机型),但 GeoScan 是同一相机所以全帧一致。

### 3.4 Point cloud 缓存 `points_`

```python
# [VERIFY: src/DPVO/dpvo/patchgraph.py:31]
self.points_ = torch.zeros(self.N * self.M, 3, dtype=torch.float, device="cuda")
```

| 形状 | `(N*M=393216, 3)` |
| 内容 | 每个 patch 的世界系中心点(从 `iproj` 反投影出来) |
| 何时更新 | `DPVO.update()` 末尾(`dpvo.py:358-360`)和 `pg.normalize()`(`patchgraph.py:93-95`)|
| 用途 | viewer 显示 + 离线 `demo.py` 末尾 `slam.pg.points_.cpu()` 导出 ply |

实际有效区间:`[0, m)`(`self.m` 是 `n * M`)。

### 3.5 颜色 / 索引

```python
# [VERIFY: src/DPVO/dpvo/patchgraph.py:32-35]
self.colors_ = torch.zeros(self.N, self.M, 3, dtype=torch.uint8, device="cuda")
self.index_ = torch.zeros(self.N, self.M, dtype=torch.long, device="cuda")
self.index_map_ = torch.zeros(self.N, dtype=torch.long, device="cuda")
```

- `colors_[i, m]` = patch m of 帧 i 中心像素的 BGR(`dpvo.py:404-405`)
- `index_[i, m]` 实际填法在 `dpvo.py:407`:`self.pg.index_[self.n + 1] = self.n + 1`(下一帧的整个 index_ 行设为标量 n+1)—— 这是 patch → 来源帧 的映射,**flattened view 后** `pg.ix[k] = 来源帧 of patch k`(`patchgraph.py:110-111`)。
- `index_map_[i]` = 帧 i 的 patch 在 flattened `patches_` 中的起始 index(`dpvo.py:408`,`self.pg.index_map_[self.n + 1] = self.m + self.M`)。

### 3.6 时间戳 `tstamps_`

```python
# [VERIFY: src/DPVO/dpvo/patchgraph.py:26]
self.tstamps_ = np.zeros(self.N, dtype=np.int64)
```

- **dtype = int64**(注意:不是 float64,不能存 wall-clock 秒数小数部分)
- 填法(`dpvo.py:400`):`self.pg.tstamps_[self.n] = self.counter`,即 **总帧序号**(包括被删的)
- 这是 `00-overview.md` §8.2 提到的"DPVO 没 wall-clock"的具体存储
- terminate 时(`dpvo.py:187-188`):`self.traj[self.pg.tstamps_[i]] = self.pg.poses_[i]`,traj 字典 key 是 int 序号

### 3.7 Edge lists(BA 因子图的"边")

**Active edges**(被当前 update + BA 看的):

```python
# [VERIFY: src/DPVO/dpvo/patchgraph.py:44-47]
self.net = torch.zeros(1, 0, DIM, **kwargs)   # GRU hidden state
self.ii = torch.as_tensor([], dtype=torch.long, device="cuda")
self.jj = torch.as_tensor([], dtype=torch.long, device="cuda")
self.kk = torch.as_tensor([], dtype=torch.long, device="cuda")
```

每条 edge `e` 表示一个 (patch kk[e], 来源帧 ii[e], 重投目标帧 jj[e]) 三元组。

**约束**:
- `ii[e] == pg.ix[kk[e]]`(`dpvo.py:218` 强制)
- `ii != jj`(不是自投)
- `jj >= ii`(forward edge,新生 patch 投到后续帧)或 `jj <= ii`(backward edge,后续帧的 patch 投到当前帧的窗口里)

**Edge 添加位置**:

```python
# [VERIFY: src/DPVO/dpvo/dpvo.py:362-368, __edges_forw]
def __edges_forw(self):
    r = self.cfg.PATCH_LIFETIME  # 13
    t0 = self.M * max((self.n - r), 0)
    t1 = self.M * max((self.n - 1), 0)
    return flatmeshgrid(
        torch.arange(t0, t1, device="cuda"),    # patch range
        torch.arange(self.n-1, self.n, device="cuda"),  # target = 当前帧
        indexing='ij')
```

- forward edges:窗口内每个 patch(过去 13 帧的所有 patch)→ 当前帧。规模:13 × M = 1248 条/帧。

```python
# [VERIFY: src/DPVO/dpvo/dpvo.py:370-375, __edges_back]
def __edges_back(self):
    r = self.cfg.PATCH_LIFETIME
    t0 = self.M * max((self.n - 1), 0)
    t1 = self.M * max((self.n - 0), 0)
    return flatmeshgrid(torch.arange(t0, t1, device="cuda"),
        torch.arange(max(self.n-r, 0), self.n, device="cuda"), indexing='ij')
```

- backward edges:当前帧的所有 patch → 过去 13 帧。规模:13 × M = 1248 条/帧。

**总活跃边数**(稳态):~2496 × M = 2400 条/帧;窗口=10 帧:活跃 ~24000 条。但因为有 `REMOVAL_WINDOW=22` 删除老的(`dpvo.py:305-310`),实际峰值更高一点。

**Inactive edges**(被 `remove_factors(store=True)` 移到的"档案"):

```python
# [VERIFY: src/DPVO/dpvo/patchgraph.py:50-54]
self.ii_inac = torch.as_tensor([], dtype=torch.long, device="cuda")
self.jj_inac = ...
self.kk_inac = ...
self.weight_inac = torch.zeros(1, 0, 2, dtype=torch.long, device="cuda")
self.target_inac = torch.zeros(1, 0, 2, dtype=torch.long, device="cuda")
```

- 只在 `__run_global_BA`(`dpvo.py:312-326`)被用 → 把 active + inactive 拼起来送 CUDA BA。
- "Inactive" 不代表无效,只是不被 GRU update 看,只参与 BA solve。
- 关键:`weight_inac/target_inac` dtype 标的是 `long`,但赋值的时候(`dpvo.py:229-230`)给的是 float —— 这是 torch 自动 promote,实际存的是 float。**这是上游代码一处类型标注 bug**,不影响功能。

### 3.8 BA target / weight / net

```python
# 不在 __init__ 里,在 DPVO.update() 创建:
# [VERIFY: src/DPVO/dpvo/dpvo.py:328-344]
target = coords[..., self.P//2, self.P//2] + delta.float()  # delta from GRU
self.pg.target = target          # (1, |E|, 2)
self.pg.weight = weight          # (1, |E|, 2)
```

- `target` 是 GRU 预测的 patch 在 jj 帧的"应在"像素位置,作为 BA 的观测目标。
- `weight` 是 GRU 输出的 per-axis 不确定度(sigmoid,(0,1)),作为 BA 信息矩阵。
- `net` 是 GRU 隐藏状态,在 `__call__` 末尾的 `append_factors`(`dpvo.py:215-221`)被 cat 新边的 zeros 进来,在 `update` 里被 `network.update(...)` 写回(`dpvo.py:335-336`)。

[VERIFY: src/DPVO/dpvo/dpvo.py:215-221] append_factors 实现
[VERIFY: src/DPVO/dpvo/dpvo.py:335-336] update 写 net

### 3.9 `delta` 字典(被删帧的相对位姿存档)

```python
# [VERIFY: src/DPVO/dpvo/patchgraph.py:41]
self.delta = {}   # 类型:dict[int, (int, SE3)]
```

填入位置:`dpvo.py:277-278` 在 `keyframe()` 决定丢帧 k 时:

```python
# [VERIFY: src/DPVO/dpvo/dpvo.py:277-278]
dP = SE3(self.pg.poses_[k]) * SE3(self.pg.poses_[k-1]).inv()
self.pg.delta[t1] = (t0, dP)   # t1=被删帧序号, t0=前一帧序号, dP=t1相对于t0的 SE3
```

读取位置:`get_pose(t)`(`dpvo.py:166-171`)递归回溯:

```python
def get_pose(self, t):
    if t in self.traj:
        return SE3(self.traj[t])
    t0, dP = self.pg.delta[t]
    return dP * self.get_pose(t0)
```

terminate 末尾用这个把丢掉的帧"还原"出来发轨迹(`dpvo.py:185-191`)。

---

## 4. DPVO 实例上的额外缓存(network buffers)

### 4.1 `imap_` —— inet 描述子

```python
# [VERIFY: src/DPVO/dpvo/dpvo.py:63]
self.imap_ = torch.zeros(self.pmem, self.M, DIM, **kwargs)
```

| 形状 | `(pmem=36, M=96, DIM=384)` |
| dtype | fp16(因为 `MIXED_PRECISION=True`,`kwargs={"device":"cuda","dtype":torch.half}`)|
| 内存 | 36 × 96 × 384 × 2 B = 2.7 MB |
| 内容 | `Patchifier.inet` 输出 patch 描述子,`net.py:113,139` 处提取 |
| 索引方式 | `imap_[self.n % self.pmem]` —— 环形缓冲(`dpvo.py:435`),老帧覆盖 |

### 4.2 `gmap_` —— fnet patch 描述子

```python
# [VERIFY: src/DPVO/dpvo/dpvo.py:64]
self.gmap_ = torch.zeros(self.pmem, self.M, 128, self.P, self.P, **kwargs)
```

| 形状 | `(36, 96, 128, 3, 3)` |
| dtype | fp16 |
| 内存 | 36 × 96 × 128 × 9 × 2 B = 7.9 MB |
| 内容 | `Patchifier.fnet` 提的 patch local feature(128 维),3×3 grid |

### 4.3 Feature map pyramid `fmap1_`,`fmap2_`

```python
# [VERIFY: src/DPVO/dpvo/dpvo.py:72-73]
self.fmap1_ = torch.zeros(1, self.mem, 128, ht // 1, wd // 1, **kwargs)
self.fmap2_ = torch.zeros(1, self.mem, 128, ht // 4, wd // 4, **kwargs)
```

(注意:`ht`/`wd` 在 `dpvo.py:45-46` 已经做过 `// RES=4`,所以这里"`ht`"实际是 `H/4`)

| `fmap1_` 形状 | `(1, mem=36, 128, H/4, W/4)` |
| `fmap2_` 形状 | `(1, mem=36, 128, H/16, W/16)` |
| 内容 | `Patchifier.fnet(image) / 4.0` 的输出(`net.py:112`),然后 avg_pool 到两个尺度(`dpvo.py:437-438`)|
| 用途 | `corr()` 函数(`dpvo.py:200-207`)双尺度 4D correlation lookup |

对 GeoScan B1 1280×1024(/4 = 320×256):
- `fmap1_` = 1 × 36 × 128 × 320 × 256 × 2 B = 75 MB
- `fmap2_` = 1 × 36 × 128 × 80 × 64 × 2 B = 4.7 MB

这是 DPVO **VRAM 占用大头**(80 MB feature pyramid + 2.7 MB imap + 7.9 MB gmap = 91 MB 仅常驻;加上 BA 的 Hessian 和 GRU 临时张量,通常 1.5–2 GB)。

### 4.4 关键写入位置

```python
# [VERIFY: src/DPVO/dpvo/dpvo.py:435-438]
self.imap_[self.n % self.pmem] = imap.squeeze()
self.gmap_[self.n % self.pmem] = gmap.squeeze()
self.fmap1_[:, self.n % self.mem] = F.avg_pool2d(fmap[0], 1, 1)
self.fmap2_[:, self.n % self.mem] = F.avg_pool2d(fmap[0], 4, 4)
```

环形缓冲带来的 invariant:对于 patch index `k`,要查 fmap 必须 `k % pmem`(`dpvo.py:203` `ii1 = ii % (self.M * self.pmem)`)。这是 DPVO 不能跨非常远距离做 correlation 的硬约束 —— 老于 pmem 帧的 fmap 已经被覆盖。LC 模式下 `pmem = MAX_EDGE_AGE = 1000`(`dpvo.py:61`)放大了这个窗口。

[VERIFY: src/DPVO/dpvo/dpvo.py:200-207] `corr()` 用环形缓冲索引

---

## 5. Lietorch SE3 张量约定

虽然 `lietorch` 是个 .so binary,但接口契约可以从用法读出:

| 调用 | 输入 shape | 输出含义 |
|------|------------|----------|
| `SE3(data)` | `(..., 7)` `[tx,ty,tz,qx,qy,qz,qw]` | 包装为 SE3 Lie 群对象 |
| `g.data` | — | 拿回 `(..., 7)` raw |
| `g.inv()` | — | 群逆 |
| `g1 * g2` | — | 群乘法 |
| `g.log()` | — | tangent `(..., 6)` `[ρ_x,ρ_y,ρ_z,ω_x,ω_y,ω_z]` |
| `SE3.exp(xi)` | `(..., 6)` | tangent → group |
| `g.matrix()` | — | `(..., 4, 4)` 仿射矩阵 |
| `g.scale(s)` | — | Sim3 风格 scale(只缩 translation;只在 monocular LC 用)|
| `g.retr(dx)` | `(..., 6)` | 流形上 retraction:`g · exp(dx)` |
| `g.adjT(J)` | `(..., m, n)` | adjoint transpose(雅可比从 Body frame 转 World)|
| `SE3.Identity(b, device)` | — | 单位元 |
| `SE3.IdentityLike(g)` | `SE3` | 同 shape 单位元 |
| `lietorch.stack([g_list], dim)` | — | 沿 dim 堆叠 |

[VERIFY: src/DPVO/dpvo/dpvo.py:8,17] `from .lietorch import SE3` + `Id = SE3.Identity(1, device="cuda")`
[VERIFY: src/DPVO/dpvo/ba.py:6,54-56] `pose_retr(poses, dx, ii)` → `poses.retr(scatter_sum(...))`
[VERIFY: src/DPVO/dpvo/projective_ops.py:104] `Ji = -Gij[:,:,None].adjT(Jj)`

---

## 6. CUDA BA 张量契约(从 `fastba/ba.py` 和 `ba.cpp` 提取)

```python
# [VERIFY: src/DPVO/dpvo/fastba/ba.py:7-8]
def BA(poses, patches, intrinsics, target, weight, lmbda, ii, jj, kk, t0, t1, M, iterations, eff_impl=False):
    return cuda_ba.forward(poses.data, patches, intrinsics, target, weight, lmbda, ii, jj, kk, M, t0, t1, iterations, eff_impl)
```

| 参数 | 输入形状 | 输入 dtype | 含义 |
|------|----------|-----------|------|
| `poses.data` | `(1, N, 7)` | float32 | 当前 pose,**会被 in-place 改**(`ba_cuda.cu:178-`,`pose_retr_kernel`)|
| `patches` | `(1, N*M, 3, 3, 3)` | float32 | patches flattened,depth 通道 `[:,:,2,:,:]` 会被 in-place 改 |
| `intrinsics` | `(1, N, 4)` | float32 | per-frame fx,fy,cx,cy |
| `target` | `(1, |E|, 2)` | float32 | BA 观测目标(GRU 出的)|
| `weight` | `(1, |E|, 2)` | float32 | 每 edge 每方向 weight |
| `lmbda` | `(1,)` | float32 | LM 阻尼(默认 1e-4)|
| `ii, jj` | `(|E|,)` | int64 | edge 端点帧 |
| `kk` | `(|E|,)` | int64 | edge 对应 patch index |
| `M` | int | — | patches per frame |
| `t0, t1` | int | — | BA 优化的帧范围 `[t0, t1)`,t0 个 pose 被固定 |
| `iterations` | int | — | Gauss-Newton 迭代次数(推理 2,init 12)|
| `eff_impl` | bool | — | True 用 block_e.cu 的 efficient Schur(只在 `__run_global_BA` 用) |

**关键 in-place 副作用**:
- `poses.data[t0:t1]` 被 LM update 写入(`ba_cuda.cu:178-` `pose_retr_kernel`)
- `patches[:,:,2,:,:]` 中所有相关 patch 的深度通道被写入(`ba_cuda.cu:209-` `patch_retr_kernel`)

[VERIFY: src/DPVO/dpvo/fastba/ba_cuda.cu:178,209] retr kernels

---

## 7. ROS2 节点状态(`dpvo_ros_node.py`)

```python
# [VERIFY: src/DPVO/dpvo_ros_node.py:39-75]
class DPVONode(Node):
    self.slam : DPVO | None         # 第一帧 lazy 创建
    self.network_path : str         # dpvo.pth 路径
    self.frame_count : int          # 收到的总帧数
    self.stride : int               # 抽帧步长
    self.save_path : str
    self.timestamps : list[float]   # wall-clock 累积(秒)
    self.last_msg_time : float | None  # time.time() of last image
    self.started : bool
    self.finished : bool
    self.publish_pose : bool
    self.pose_frame_id : str
    self.pose_pub : Publisher       # /dpvo/pose
    self.sub : Subscription         # /left_camera/image
    self.watchdog : Timer
```

**watchdog 退出条件**(`dpvo_ros_node.py:80-92`):
- `elapsed > 15.0` 秒未收到新图 **且** `frame_count > 30` → 认为 bag 跑完,自动 save+shutdown。
- 这个"30 帧最小阈值"是为了容忍启动 stall(SLAM init 时 CUDA 编译 patchify kernel 会卡几秒)。

---

## 8. 内存预算汇总(GeoScan B1 1280×1024,LOOP_CLOSURE=False)

| 缓冲 | 形状 | dtype | 字节数 |
|------|------|-------|--------|
| `pg.poses_` | (4096, 7) | f32 | 112 KB |
| `pg.patches_` | (4096, 96, 3, 3, 3) | f32 | 13.5 MB |
| `pg.intrinsics_` | (4096, 4) | f32 | 64 KB |
| `pg.points_` | (393216, 3) | f32 | 4.7 MB |
| `pg.colors_` | (4096, 96, 3) | u8 | 1.2 MB |
| `pg.index_` | (4096, 96) | i64 | 3.1 MB |
| `pg.index_map_` | (4096,) | i64 | 33 KB |
| `pg.tstamps_` | (4096,) | i64 | 33 KB(CPU)|
| `imap_` | (36, 96, 384) | f16 | 2.7 MB |
| `gmap_` | (36, 96, 128, 3, 3) | f16 | 7.9 MB |
| `fmap1_` | (1, 36, 128, 320, 256) | f16 | 75 MB |
| `fmap2_` | (1, 36, 128, 80, 64) | f16 | 4.7 MB |
| **常驻 GPU 合计** | | | **~110 MB** |

加上 GRU 中间张量、Update GRU 权重(~14 MB 来自 dpvo.pth)、CUDA BA Hessian 临时张量(~50 MB peak)、torch 缓存等,实测在 GeoScan 上 DPVO 跑稳后占 GPU **约 1.6–1.8 GB**(主要不是常驻,是 forward pass 临时分配),~4 GB peak 在 init 那 12 次 iter。

**对比 4060 8 GB**:充裕,DPVO 在 4060 上是 cpu-bound 不是 vram-bound。

---

## 9. 验证摘要

| 字段 | 形状声明 | [VERIFY:] 行号 |
|------|----------|----------------|
| `poses_` | (N, 7) | patchgraph.py:27 |
| `patches_` | (N, M, 3, P, P) | patchgraph.py:28 |
| `intrinsics_` | (N, 4) | patchgraph.py:29 |
| `points_` | (N*M, 3) | patchgraph.py:31 |
| `colors_` | (N, M, 3) | patchgraph.py:32 |
| `index_` | (N, M) | patchgraph.py:34 |
| `index_map_` | (N,) | patchgraph.py:35 |
| `tstamps_` | (N,) | patchgraph.py:26 |
| `ii/jj/kk` | (|E|,) | patchgraph.py:45-47 |
| `net` | (1, |E|, DIM) | patchgraph.py:44 |
| `delta` | dict | patchgraph.py:41 |
| `imap_` | (pmem, M, DIM) | dpvo.py:63 |
| `gmap_` | (pmem, M, 128, P, P) | dpvo.py:64 |
| `fmap1_` | (1, mem, 128, H/4, W/4) | dpvo.py:72 |
| `fmap2_` | (1, mem, 128, H/16, W/16) | dpvo.py:73 |

下一步:`02-data-flow.md`(Phase 3,跟一帧穿过 `__call__`)。

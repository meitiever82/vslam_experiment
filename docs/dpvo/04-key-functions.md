# DPVO 关键函数逐行解析(Phase 5)

> **范围**:`DPVO.__call__`、`DPVO.update`、`DPVO.keyframe`、`DPVO.terminate`、`ba.BA`、`Patchifier.forward`
> **每一段都引用代码 file:行号**

---

## §1. `DPVO.__call__`(dpvo.py:377-473)— 单帧入口

```python
def __call__(self, tstamp, image, intrinsics):
    """ track new frame """
```

### §1.1 行 380-381:Classic LC 钩子

```python
if self.cfg.CLASSIC_LOOP_CLOSURE:
    self.long_term_lc(image, self.n)
```

- 仅 `CLASSIC_LOOP_CLOSURE=True` 时执行。
- 调 `LongTermLoopClosure.__call__`(`long_term.py:61-64`):把图加进 DBoW2 检索库和 imcache。
- **不阻塞**,但每帧都跑 DBoW2 add image 是 ~5ms 开销。

### §1.2 行 383-384:Buffer 边界检查

```python
if (self.n+1) >= self.N:
    raise Exception(f'The buffer size is too small. You can increase it using "--opts BUFFER_SIZE={self.N*2}"')
```

- `self.N = cfg.BUFFER_SIZE = 4096`(默认)。
- GeoScan B1 ~ 418 s × 10 fps = 4180 帧,**这已经压到 4096 边界**。但因为 `keyframe()` 会丢中间帧(`dpvo.py:266`),实际 `self.n` 通常 << 4180。
- 若真撞到上限会直接抛异常 → ROS 节点的 try/except 不接,SLAM 整个挂。**风险点**:超长 bag 必须手动调 BUFFER_SIZE。

### §1.3 行 386-387:可视化更新

```python
if self.viewer is not None:
    self.viewer.update_image(image.contiguous())
```

- viewer 默认 None(ROS 节点 viz=False);DPViewer Pangolin 实例化时才有。

### §1.4 行 389:归一化

```python
image = 2 * (image[None,None] / 255.0) - 0.5
```

- 入参 `image`:`(3, H, W)` uint8,但函数体里立即 `image[None,None]` 加两维 → `(1, 1, 3, H, W)`,匹配 batch+seq 维度。
- 范围:`[0, 255] / 255 → [0, 1]`,× 2 → `[0, 2]`,−0.5 → **`[-0.5, 1.5]`**(不是常见的 `[-1, 1]`!)
- 这个奇特的归一化是上游训练时定的:训练时 image **uint8 输入** → `(image / 255.0 - 0.5) * 2 = [-1, 1]`?让我再读一遍 —— 行 389 `2 * (image[None,None] / 255.0) - 0.5`,运算顺序:`(image/255) → 2*(image/255) → 2*(image/255) - 0.5`,所以 (0, 2) - 0.5 = **(-0.5, 1.5)**。
- 对比 `net.py:191`(训练 forward):`images = 2 * (images / 255.0) - 0.5` —— 完全一致。这确实是 DPVO 的内部归一化约定。

### §1.5 行 391-396:Patchifier(主 CNN forward)

```python
with autocast(enabled=self.cfg.MIXED_PRECISION):
    fmap, gmap, imap, patches, _, clr = \
        self.network.patchify(image,
            patches_per_image=self.cfg.PATCHES_PER_FRAME,    # 96
            centroid_sel_strat=self.cfg.CENTROID_SEL_STRAT,  # 'RANDOM'
            return_color=True)
```

- `MIXED_PRECISION=True` → `autocast` 半精度上下文,fnet/inet 在 fp16 跑。
- 返回 5+1 元组:`fmap, gmap, imap, patches, index_dummy, clr`(详见 `02-data-flow.md` §2.1)。
- 第五项 `_` 是 patch-source-index,但这里只有 1 帧,内容是全 0(`net.py:151-152`)。

[VERIFY: src/DPVO/dpvo/net.py:110-157] Patchifier.forward 完整实现

### §1.6 行 398-405:状态记录

```python
self.tlist.append(tstamp)
self.pg.tstamps_[self.n] = self.counter      # 注意:存 counter 不是 tstamp!
self.pg.intrinsics_[self.n] = intrinsics / self.RES

# color info for visualization
clr = (clr[0,:,[2,1,0]] + 0.5) * (255.0 / 2)   # 反归一化 + BGR→RGB
self.pg.colors_[self.n] = clr.to(torch.uint8)
```

- `self.tlist` 保存外部传入的 tstamp(给运动模型用,`dpvo.py:416`)。
- `self.pg.tstamps_[i]` 存的是 `self.counter`(总帧数),不是 tstamp。**这两个时序变量名都叫 "tstamps" 但完全不同**。
- intrinsics 除以 `RES=4` —— 因为后续所有几何都在 H/4 × W/4 feature 尺度。
- color 反归一化:`(clr + 0.5) / 2 * 255` = `(clr/2 + 0.25) * 255`;但代码是 `(clr + 0.5) * (255.0 / 2)` = `(clr + 0.5) * 127.5`。看 §1.4 归一化:`clr` 来自 `Patchifier` 的 `altcorr.patchify(images[0], 4*(coords+0.5), 0)`(`net.py:143`),`images` 已是 `[-0.5, 1.5]` 范围 → `clr * 127.5 + 63.75`,**但实际代码** `(clr + 0.5) * 127.5` = `clr * 127.5 + 63.75` ✓,等价。
- `[2,1,0]` 索引把 RGB(Patchifier 是按图像 BGR 输入存的)反成 BGR→RGB(供 viewer)。

### §1.7 行 407-408:Index 写入

```python
self.pg.index_[self.n + 1] = self.n + 1       # 患者帧 id
self.pg.index_map_[self.n + 1] = self.m + self.M    # patch 起始 idx
```

**注意 `+1`**:写入的是下一行(`self.n + 1`),不是当前行。`pg.index_[i, m]` 设计上等于 "patch m of 帧 i 的来源帧 = i"(`patchgraph.py:34` 初始化是 0,但实际填 i,这里填 i+1 是 bug 还是 feature?)。

进一步分析:`pg.ix` 是 `index_.view(-1)`(`patchgraph.py:110-111`),即 flatten 一维 `(N*M,)`。`pg.ix[k]` 应当返回 "patch index k 的来源帧"。

当 `self.n = 5`(刚处理完 0,1,2,3,4 帧,正在处理 5):
- 这一行:`pg.index_[6] = 6`(整行 96 个 patch 的来源帧 id 都被设为 6)
- 但 patch 5 的 96 个 patch 在哪写?**在前一次 `__call__` 时**(那时 `self.n=4` → 写 `pg.index_[5] = 5`)。

OK,理解了:`pg.index_[i]` 在 **`__call__` 处理 i-1 帧时**被写入(为下一帧 i 预设),所以这里看 `self.n + 1`。这导致 `pg.index_[0]` 永远是 0(初始化值),逻辑上"帧 0 的 patch 来自帧 0",正确。

`self.m` 是当前总 patch 数(`= self.n * M`),所以 `self.m + self.M` = 帧 (n+1) 的 patch 起始 index。

### §1.8 行 410-424:运动模型(DAMPED_LINEAR)

```python
if self.n > 1:
    if self.cfg.MOTION_MODEL == 'DAMPED_LINEAR':
        P1 = SE3(self.pg.poses_[self.n-1])
        P2 = SE3(self.pg.poses_[self.n-2])
        
        *_, a, b, c = [1]*3 + self.tlist   # 取末三项,前 3 个 1 填充
        fac = (c-b) / (b-a)
        
        xi = self.cfg.MOTION_DAMPING * fac * (P1 * P2.inv()).log()
        tvec_qvec = (SE3.exp(xi) * P1).data
        self.pg.poses_[self.n] = tvec_qvec
    else:
        tvec_qvec = self.poses[self.n-1]      # 沿用上帧 pose(等于静止)
        self.pg.poses_[self.n] = tvec_qvec
```

- 数学详见 `03-algorithms.md` §1.8(其实在 §1 末尾,运动模型这部分本质是 `02-data-flow.md` §2.2)。
- **`*_, a, b, c = [1]*3 + self.tlist`** 这个 Python 解包技巧:前面填 3 个 1,然后接 `self.tlist`,`*_` 接收前面所有元素,`a,b,c` 接末三个。
- `n == 2` 时 `tlist = [t0, t1]`,前 3 个 1 + tlist = `[1, 1, 1, t0, t1]`,末三个 = `(1, t0, t1)`;`fac = (t1-t0) / (t0-1)`,**这个 fac 在 init 阶段几乎没意义**(分母是 t0-1,t0 大时 fac 趋近 0,运动模型几乎不外推)。
- 稳态(`n > 4`)`tlist 末三项 = (t_{n-3}, t_{n-2}, t_{n-1})`,`fac = (t_{n-1}-t_{n-2}) / (t_{n-2}-t_{n-3})` —— 等帧率下恒等于 1。

### §1.9 行 426-432:Patch 深度初始化

```python
# TODO better depth initialization
patches[:,:,2] = torch.rand_like(patches[:,:,2,0,0,None,None])
if self.is_initialized:
    s = torch.median(self.pg.patches_[self.n-3:self.n,:,2])
    patches[:,:,2] = s

self.pg.patches_[self.n] = patches
```

- 第一步:整个 (96, 3, 3) 深度通道用 `rand_like` 替换 → uniform(0, 1)。注释 `# TODO` 表明作者也觉得这是 hack。
- 第二步:若已 initialized(`n >= 8`),用最近 3 帧 disparity 的全局中位数覆盖。这就是 monocular VO 的"scale 一致性"维持机制。
- patch shape:`(1, 96, 3, 3, 3)`,`patches[:,:,2]` 是 `(1, 96, 3, 3)`,赋值 `rand_like(patches[:,:,2,0,0,None,None])` 是 `(1, 96, 1, 1)` broadcast 到 `(1, 96, 3, 3)`,即每个 patch 一个 random 标量复制 9 份。

### §1.10 行 435-438:Buffer 写入(环形)

```python
self.imap_[self.n % self.pmem] = imap.squeeze()
self.gmap_[self.n % self.pmem] = gmap.squeeze()
self.fmap1_[:, self.n % self.mem] = F.avg_pool2d(fmap[0], 1, 1)
self.fmap2_[:, self.n % self.mem] = F.avg_pool2d(fmap[0], 4, 4)
```

- 注意 `pmem` 用于 imap/gmap(patch memory),`mem` 用于 fmap1/fmap2(feature memory)。
- LOOP_CLOSURE=False 时两者相同(36),LOOP_CLOSURE=True 时 pmem=1000,mem=36。
- `F.avg_pool2d(fmap[0], 1, 1)` —— kernel=1, stride=1,**等价于直接拷贝**(no-op 但保留尺寸)。这里是为了让两个 fmap 路径形状对称。
- `F.avg_pool2d(fmap[0], 4, 4)` —— kernel=4, stride=4,下采 4 倍。

### §1.11 行 440-447:Counter / Init gate / Index 更新

```python
self.counter += 1

if self.n > 0 and not self.is_initialized:
    if self.motion_probe() < 2.0:
        self.pg.delta[self.counter - 1] = (self.counter - 2, Id[0])
        return    # 整个 __call__ 直接结束

self.n += 1
self.m += self.M
```

- `counter` 每次 `__call__` 都 +1(即使被 motion_probe gate 拒绝)。
- Init gate:`motion_probe()` 返回当前帧 patch 重投到自己的 flow 中位数。
- 若 < 2 px:把这帧记入 `pg.delta`(标记为"与上帧相同位姿,delta = Identity"),不让 `self.n` 递增 → 不入 patch graph。
- `Id = SE3.Identity(1, device="cuda")`(`dpvo.py:17`),`Id[0]` 是 7D 张量。
- 这就是"静止启动 DPVO 卡 init"的根源 —— `counter` 涨但 `n` 不涨,LiDAR 已经走 5 秒,DPVO 第 0 帧还没生成。

### §1.12 行 449-455:Loop closure edges

```python
if self.cfg.LOOP_CLOSURE:
    if self.n - self.last_global_ba >= self.cfg.GLOBAL_OPT_FREQ:
        lii, ljj = self.pg.edges_loop()
        if lii.numel() > 0:
            self.last_global_ba = self.n
            self.append_factors(lii, ljj)
```

- 每 `GLOBAL_OPT_FREQ=15` 帧考虑加 loop edges。
- `pg.edges_loop()`(`patchgraph.py:56-82`)用 `flow_mag < BACKEND_THRESH=64` 筛选远距帧对,nms=1 + max 1000 edges。
- 触发 `last_global_ba = n` 标记,下一次 `__run_global_BA` 会被允许跑。

### §1.13 行 458-459:Append edges

```python
self.append_factors(*self.__edges_forw())   # 13×96=1248 条 forward
self.append_factors(*self.__edges_back())   # 13×96=1248 条 backward
```

详见 `02-data-flow.md` §4。

### §1.14 行 461-469:Init / Update 分支

```python
if self.n == 8 and not self.is_initialized:
    self.is_initialized = True
    
    for itr in range(12):
        self.update()

elif self.is_initialized:
    self.update()
    self.keyframe()
```

- Init 触发条件:`self.n == 8` —— 即第 8 个**有效**帧到达(motion_probe gate 失败的不算)。
- Init 时连跑 12 轮 `update()`,这是"cold-start BA"让前 8 帧的 pose 和 patch 深度收敛。
- 已 init 状态:每帧 1 轮 update + 1 次 keyframe 判断。

### §1.15 行 471-473:Classic LC 末尾

```python
if self.cfg.CLASSIC_LOOP_CLOSURE:
    self.long_term_lc.attempt_loop_closure(self.n)
    self.long_term_lc.lc_callback()
```

详见 `02-data-flow.md` §6 关于异步 PGO。

---

## §2. `DPVO.update`(dpvo.py:328-360)

```python
def update(self):
    with Timer("other", enabled=self.enable_timing):
        coords = self.reproject()                                          # (1) 重投

        with autocast(enabled=True):
            corr = self.corr(coords)                                       # (2) correlation lookup
            ctx = self.imap[:, self.pg.kk % (self.M * self.pmem)]          # (3) context
            self.pg.net, (delta, weight, _) = \
                self.network.update(self.pg.net, ctx, corr, None,
                                    self.pg.ii, self.pg.jj, self.pg.kk)    # (4) GRU forward

        lmbda = torch.as_tensor([1e-4], device="cuda")
        weight = weight.float()
        target = coords[..., self.P//2, self.P//2] + delta.float()         # (5) target

    self.pg.target = target
    self.pg.weight = weight

    with Timer("BA", enabled=self.enable_timing):
        try:
            # (6) BA 路由
            if (self.pg.ii < self.n - self.cfg.REMOVAL_WINDOW - 1).any() \
               and not self.ran_global_ba[self.n]:
                self.__run_global_BA()
            else:
                t0 = self.n - self.cfg.OPTIMIZATION_WINDOW if self.is_initialized else 1
                t0 = max(t0, 1)
                fastba.BA(self.poses, self.patches, self.intrinsics,
                    target, weight, lmbda, self.pg.ii, self.pg.jj, self.pg.kk,
                    t0, self.n, M=self.M, iterations=2, eff_impl=False)
        except:
            print("Warning BA failed...")

        # (7) point cloud 缓存更新
        points = pops.point_cloud(SE3(self.poses), self.patches[:, :self.m], self.intrinsics, self.ix[:self.m])
        points = (points[..., 1, 1, :3] / points[..., 1, 1, 3:]).reshape(-1, 3)
        self.pg.points_[:len(points)] = points[:]
```

### §2.1 步骤逐段

**(1) reproject**(`dpvo.py:209-213`):每条 edge 算 patch 重投到 jj 帧的像素坐标 `(b, |E|, 2, P, P)`。Wraps `projective_ops.transform(jacobian=False)`。

**(2) corr** lookup:`coords` 是 `(b, |E|, 2, P, P)`,在两尺度 fmap1_/fmap2_ 上各做 7×7 lookup → 拼接得 `(b, |E|, 2·49·9)` = `(b, |E|, 882)`。

**(3) ctx**:从 `imap` 取 patch 描述子。`self.pg.kk % (M * pmem)` 是因为 `kk` 用扁平 patch index `[0, n*M)`,但 `imap_` 只有 `pmem` 个槽(环形)。

**(4) network.update**:GRU 单步前向(详见 `03-algorithms.md` §2)。输入(net, ctx, corr, None, ii, jj, kk),输出 (new_net, (delta, weight, None))。`None` 那个位置是接口保留(原 paper 有第三个输出 occlusion mask,实际未用)。

**(5) target**:`coords[..., P//2, P//2]` 是 patch 中心 `(b, |E|, 2)`,加上 GRU 的 delta(2D 像素 flow),得 BA 目标。

**(6) BA 路由**:
- 条件:`pg.ii` 中有任何 edge 起点早于 `n - REMOVAL_WINDOW - 1 = n - 23`,**且** `ran_global_ba[n]` 未跑过 → 走 `__run_global_BA()`(`dpvo.py:312-326`)。
- 否则 local BA:`t0 = n - 10` 起,iterations=2 次 GN。
- `eff_impl=False` —— 标准 Schur kernel(`eff_impl=True` 只在 global BA)。

**(7) Point cloud**:重投 `iproj` 出 patch 中心齐次坐标 `(b, m, 3, 3, 4)`,取 `[1, 1]`(中心)的 (x, y, z, w) → 齐次除 w 得 3D 点。

### §2.2 异常处理

```python
try:
    ...BA...
except:
    print("Warning BA failed...")
```

- **空 `except`**(捕获所有异常,包括 KeyboardInterrupt)—— 这是 DPVO 上游一直存在的过宽 except,会吞掉 Ctrl-C。
- BA 失败的真实情况:Cholesky 解失败(patches/poses 数值发散),CUDA error。
- 失败后 `target/weight` 已经写入 pg,但 poses_/patches_ 没改;下一帧继续。

---

## §3. `DPVO.keyframe`(dpvo.py:266-310)

详细数学见 `03-algorithms.md` §4。这里关注代码结构:

```python
def keyframe(self):
    i = self.n - self.cfg.KEYFRAME_INDEX - 1     # n-5
    j = self.n - self.cfg.KEYFRAME_INDEX + 1     # n-3
    m = self.motionmag(i, j) + self.motionmag(j, i)

    if m / 2 < self.cfg.KEYFRAME_THRESH:         # 15.0
        k = self.n - self.cfg.KEYFRAME_INDEX     # n-4 要被丢
        t0 = self.pg.tstamps_[k-1]               # 前一帧序号
        t1 = self.pg.tstamps_[k]                 # 被丢帧序号

        dP = SE3(self.pg.poses_[k]) * SE3(self.pg.poses_[k-1]).inv()
        self.pg.delta[t1] = (t0, dP)             # 存档

        # 删该帧相关 edges,不存档
        to_remove = (self.pg.ii == k) | (self.pg.jj == k)
        self.remove_factors(to_remove, store=False)

        # 整体左移
        self.pg.kk[self.pg.ii > k] -= self.M    # 高于 k 的 patch index 减 M(每帧 M 个 patch)
        self.pg.ii[self.pg.ii > k] -= 1          # 高于 k 的帧 id 减 1
        self.pg.jj[self.pg.jj > k] -= 1

        for i in range(k, self.n-1):
            self.pg.tstamps_[i] = self.pg.tstamps_[i+1]
            self.pg.colors_[i] = self.pg.colors_[i+1]
            self.pg.poses_[i] = self.pg.poses_[i+1]
            self.pg.patches_[i] = self.pg.patches_[i+1]
            self.pg.intrinsics_[i] = self.pg.intrinsics_[i+1]

            self.imap_[i % self.pmem] = self.imap_[(i+1) % self.pmem]
            self.gmap_[i % self.pmem] = self.gmap_[(i+1) % self.pmem]
            self.fmap1_[0, i%self.mem] = self.fmap1_[0, (i+1)%self.mem]
            self.fmap2_[0, i%self.mem] = self.fmap2_[0, (i+1)%self.mem]

        self.n -= 1
        self.m -= self.M

        if self.cfg.CLASSIC_LOOP_CLOSURE:
            self.long_term_lc.keyframe(k)

    # 阶段 2:移老 edge
    to_remove = self.ix[self.pg.kk] < self.n - self.cfg.REMOVAL_WINDOW
    if self.cfg.LOOP_CLOSURE:
        lc_edges = ((self.pg.jj - self.pg.ii) > 30) & (self.pg.jj > (self.n - self.cfg.OPTIMIZATION_WINDOW))
        to_remove = to_remove & ~lc_edges
    self.remove_factors(to_remove, store=True)
```

### §3.1 注意点

- **整体左移 for 循环不矢量化**:`pg.tstamps_/colors_/poses_/patches_/intrinsics_` 顺序左移 1 格 + buffer 同步。**Python 循环耗时** —— 但因为这分支不每帧触发(只有 flow_mag 小时),实测占比小。
- **环形缓冲的左移**(`imap_[i % pmem] = imap_[(i+1) % pmem]`):因为 `pmem` 是环形,左移时要在环里做。若 `i+1` 跨界(`i = pmem-1`),`(i+1) % pmem = 0`,正确包裹。
- **`store=False` vs `store=True`**:
  - 丢帧时:`store=False` → 该帧 edges **彻底丢**(不进 inactive)。理由:这帧已经被相对位姿 delta 记下,edges 没用。
  - 移老 edge:`store=True` → 进 inactive,留给 `__run_global_BA` 用。

---

## §4. `DPVO.terminate`(dpvo.py:173-198)

```python
def terminate(self):
    if self.cfg.CLASSIC_LOOP_CLOSURE:
        self.long_term_lc.terminate(self.n)

    if self.cfg.LOOP_CLOSURE:
        self.append_factors(*self.pg.edges_loop())

    for _ in range(12):
        self.ran_global_ba[self.n] = False    # 重置 flag,让 __run_global_BA 可触发
        self.update()

    """ interpolate missing poses """
    self.traj = {}
    for i in range(self.n):
        self.traj[self.pg.tstamps_[i]] = self.pg.poses_[i]

    poses = [self.get_pose(t) for t in range(self.counter)]
    poses = lietorch.stack(poses, dim=0)
    poses = poses.inv().data.cpu().numpy()        # w2c → c2w
    tstamps = np.array(self.tlist, dtype=np.float64)
    if self.viewer is not None:
        self.viewer.join()

    # Poses: x y z qx qy qz qw
    return poses, tstamps
```

### §4.1 详解

**循环 12 次 update**(`dpvo.py:181-183`):每次清掉 `ran_global_ba[n]` 让 BA 再跑。这是"清扫"阶段:确保所有 edges 都被解优化收敛。

**轨迹还原**(`dpvo.py:185-191`):
- `self.traj[t] = pose` 字典:把当前活着的帧编入。
- `[self.get_pose(t) for t in range(self.counter)]`:对 0..counter-1 所有"曾经存在过"的帧序号:
  - 在 traj 字典里 → 直接拿 SE3。
  - 不在(被 keyframe 丢了)→ 走 `pg.delta` 字典递归:`dP * get_pose(t0)`。
- 这就完成了"丢帧补 + 活帧拷贝"的合并。

**w2c → c2w**(`dpvo.py:192`):`poses.inv().data.cpu().numpy()`。这是 DPVO 的输出契约:**外部看到的轨迹是 c2w**(camera in world),内部 `pg.poses_` 是 w2c。

**tstamps 来源**(`dpvo.py:193`):`self.tlist` 是 `__call__` 接收的 tstamp 参数列表(从 `dpvo_ros_node.py` 进来的是 `len(self.timestamps) - 1` 即整数帧 id)。`np.float64` cast 不改值。

---

## §5. `ba.BA`(ba.py:86-182)— 训练用 Python BA

完整逐行已在 `03-algorithms.md` §1 涵盖。这里补充一些上下文:

### §5.1 调用方

```python
# [VERIFY: src/DPVO/dpvo/net.py:260-261]
for itr in range(2):
    Gs, patches = BA(Gs, patches, intrinsics, target, weight, lmbda, ii, jj, kk, 
        bounds, ep=ep, fixedp=1, structure_only=structure_only)
```

- 训练 forward 里每个 GRU iteration 后跑 2 次 BA。
- 与推理 `fastba.BA(iterations=2)` 一致 —— 推理用 CUDA 更快,训练用 Python 因为要可微。

### §5.2 与 fastba.BA 的对比

| 项目 | Python ba.BA | CUDA fastba.BA |
|------|--------------|----------------|
| 接口 | `BA(poses, patches, ..., bounds, ep, PRINT, fixedp, structure_only)` | `BA(poses, patches, ..., t0, t1, M, iterations, eff_impl)` |
| 返回值 | `(poses, patches)`(新对象) | None,**in-place** 改写 `poses.data` 和 `patches[:,:,2]` |
| Jacobian | `projective_ops.transform(jacobian=True)`,torch autograd 兼容 | 手写在 CUDA kernel(`ba_cuda.cu:313-374`)|
| Cholesky | `CholeskySolver`(`ba.py:12-37`),autograd 兼容 | Eigen sparse Cholesky(`ba.cpp:99-118`)|
| Iter 控制 | 调用者控制(外层循环)| 内部循环 iterations=2 次 |
| `eff_impl` | 不支持 | 支持(global BA 用) |
| `bounds` | 显式(out-of-image clip) | 硬编码(`ba_cuda.cu:305-306`)|
| `structure_only` | 支持(只优化 depth) | 不直接支持(改 fixedp=n 等价) |

### §5.3 关键差异:`r.norm < 250` 阈值

Python:
```python
# [VERIFY: src/DPVO/dpvo/ba.py:98]
v *= (r.norm(dim=-1) < 250).float()
```

CUDA:
```cuda
// [VERIFY: src/DPVO/dpvo/fastba/ba_cuda.cu:305]
const bool in_bounds = (sqrt(rx*rx+ry*ry) < 128) && ...
```

**数字不同**(250 vs 128)!可能是上游优化时改的,或者训练/推理需求不同。这是个细微 inconsistency,**没记在 README 或 changelog**。

---

## §6. `Patchifier.forward`(net.py:110-157)— CNN 主入口

```python
def forward(self, images, patches_per_image=80, disps=None,
            centroid_sel_strat='RANDOM', return_color=False):
    fmap = self.fnet(images) / 4.0
    imap = self.inet(images) / 4.0

    b, n, c, h, w = fmap.shape
    P = self.patch_size      # 3

    # patch 中心采样
    if centroid_sel_strat == 'GRADIENT_BIAS':
        g = self.__image_gradient(images)
        x = torch.randint(1, w-1, size=[n, 3*patches_per_image], device="cuda")
        y = torch.randint(1, h-1, size=[n, 3*patches_per_image], device="cuda")

        coords = torch.stack([x, y], dim=-1).float()
        g = altcorr.patchify(g[0,:,None], coords, 0).view(n, 3 * patches_per_image)

        ix = torch.argsort(g, dim=1)
        x = torch.gather(x, 1, ix[:, -patches_per_image:])
        y = torch.gather(y, 1, ix[:, -patches_per_image:])

    elif centroid_sel_strat == 'RANDOM':
        x = torch.randint(1, w-1, size=[n, patches_per_image], device="cuda")
        y = torch.randint(1, h-1, size=[n, patches_per_image], device="cuda")

    else:
        raise NotImplementedError(...)

    coords = torch.stack([x, y], dim=-1).float()
    imap = altcorr.patchify(imap[0], coords, 0).view(b, -1, DIM, 1, 1)
    gmap = altcorr.patchify(fmap[0], coords, P//2).view(b, -1, 128, P, P)

    if return_color:
        clr = altcorr.patchify(images[0], 4*(coords + 0.5), 0).view(b, -1, 3)

    if disps is None:
        disps = torch.ones(b, n, h, w, device="cuda")

    grid, _ = coords_grid_with_index(disps, device=fmap.device)
    patches = altcorr.patchify(grid[0], coords, P//2).view(b, -1, 3, P, P)

    index = torch.arange(n, device="cuda").view(n, 1)
    index = index.repeat(1, patches_per_image).reshape(-1)

    if return_color:
        return fmap, gmap, imap, patches, index, clr

    return fmap, gmap, imap, patches, index
```

### §6.1 GRADIENT_BIAS 模式

**目的**:patch 不要全在天空/纯色区域。
- 先采 3× 候选(3·96=288 个),计算每点周围 gradient(`__image_gradient`,`net.py:102-108`)。
- 按 gradient 排序,取 top 96。
- 注意 grad 是在 `images` 上算然后 avg_pool 4× → 在 fmap 尺度。

GeoScan benchmark 用 'RANDOM'(`config/default.yaml:19`)—— 是默认 simple 模式。

### §6.2 `patchify` 调用细节

- `imap`: radius=0,只扣 1×1 中心 → 描述子(DIM=384)。
- `gmap`: radius=P//2=1,扣 3×3 patch → fnet 描述子(128 ch)。
- `clr`: 在原图(`images[0]`)上扣;coords 乘 4 是从 fmap 尺度还原到原图 4× 尺度,加 0.5 是 sub-pixel offset(双线性插值中心)。
- `patches`: `grid` 是 `coords_grid_with_index` 出的 `(b, n, 3, h, w)` 张量(`utils.py:39-54`),前两通道是像素坐标 (x, y),第三通道是 disparity。`disps=None` 时初始化为 1.0(全 1)。扣 3×3 后形状 `(b, |patch|, 3, P, P)`。

### §6.3 设计奇怪点

- `disps=None` 时初始化为全 1。但 `DPVO.__call__` 立刻又把第三通道随机化(`dpvo.py:427`)→ Patchifier 出的 disp=1 完全没用,这是上游冗余 code。
- `index` 在 n=1 时全是 0(只有一帧)→ `DPVO.__call__` 也没用这个返回值(`dpvo.py:392` 解包是 `fmap, gmap, imap, patches, _, clr`,`_` 接收 index)→ DPVO 用自己的 `pg.index_` 维护映射。

---

## §7. 验证摘要

| 函数 | 行数范围 | 关键点 | 是否完整解析 |
|------|----------|--------|--------------|
| `DPVO.__call__` | 377-473 | 14 个分段 | ✓ |
| `DPVO.update` | 328-360 | 7 个步骤 + 异常 | ✓ |
| `DPVO.keyframe` | 266-310 | 2 阶段 + 整体左移 | ✓ |
| `DPVO.terminate` | 173-198 | 清扫 BA + 轨迹还原 | ✓ |
| `ba.BA` | 86-182 | 详见 03 文档 §1,差异点列表已补 | ✓ |
| `Patchifier.forward` | 110-157 | 5+1 元组语义 + 设计奇怪点 | ✓ |
| `__run_global_BA` | 312-326 | 详见 02 文档 §2.6 | ✓ |
| `motion_probe` | 240-255 | 详见 02 文档 §2.3 | ✓ |
| `motionmag` | 257-264 | 详见 03 文档 §4 | ✓ |
| `append_factors` | 215-221 | 详见 02 文档 §4 | ✓ |
| `remove_factors` | 223-238 | 关键:`store=True/False` 二态 | ✓(行内) |

---

## §8. 全局重要发现汇总

1. **`__call__` 行 384**:Buffer 满直接抛异常,**ROS 节点没接** → 长 bag 必须手动调 `BUFFER_SIZE`。
2. **`__call__` 行 389**:归一化是 `[-0.5, 1.5]` 而非 `[-1, 1]`,与上游训练一致。
3. **`__call__` 行 400**:`tstamps_[n] = counter` 不是 `tstamp`,**两个时序变量同名不同义**。
4. **`__call__` 行 407**:`pg.index_[n+1]` 写下一行(为下一帧预设),边界条件 frame 0 永远是 0(初始化值)。
5. **`__call__` 行 419**:`MOTION_DAMPING=0.5` × `fac` × `log` —— **fac 在等帧率时恒为 1**,DAMPING 是主导。
6. **`__call__` 行 442**:`motion_probe() < 2.0` 是**静止启动失败**的根因。
7. **`update` 行 355**:`except:` 过宽,吞 Ctrl-C。
8. **`keyframe` 行 287-297**:Python for 循环左移 buffer,不矢量化,但触发频率低不影响 throughput。
9. **`terminate` 行 192**:`.inv()` 把内部 w2c 转 c2w —— **外部 trajectory 总是 c2w**。
10. **`ba.BA` vs `fastba.BA` 残差 outlier 阈值**:Python 250 vs CUDA 128,有微妙不一致。

---

## §9. 阅读完整图(供下次接手)

```
读 DPVO 的推荐顺序:
   1. config/default.yaml             [起点:看默认 hyperparams]
   2. dpvo_ros_node.py                [入口:看怎么进 DPVO.__call__]
   3. dpvo/dpvo.py                    [主类:13 个分段,逐段读]
      ├─ __init__                     (内存布局)
      ├─ __call__                     (单帧入口)
      ├─ update                       (核心 BA + GRU)
      ├─ keyframe                     (帧删除)
      └─ terminate                    (轨迹还原)
   4. dpvo/patchgraph.py              [PatchGraph 字段]
   5. dpvo/projective_ops.py          [iproj/proj/transform + Jacobian]
   6. dpvo/ba.py                      [Python BA,可微]
   7. dpvo/net.py                     [VONet + Update GRU + Patchifier + CorrBlock]
   8. dpvo/fastba/ba_cuda.cu          [CUDA BA,推理 fast 路径]
   9. dpvo/altcorr/correlation.py     [4D corr lookup,只接口]
   10. dpvo/loop_closure/long_term.py + optim_utils.py  [v2 LC,可选]

时间预算:
   - 一次性懂大局(到 4):~半天
   - 懂 BA 数学(到 7):~1 天
   - 懂训练/LC(到 10):~半天再
```

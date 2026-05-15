# DPVO 数据流(Phase 3)

> **范围**:跟一帧图像从 ROS Image 到 `/dpvo/pose` 发布的完整路径
> **重点**:每步张量 shape、改写副作用、分支条件

---

## 1. 顶层时序图(单帧)

```
ROS msg /left_camera/image                              t=10.250 s, header.stamp
        │
        ▼
DPVONode.image_cb(msg)                                  [dpvo_ros_node.py:94]
   │ ┌─ stride 抽帧                                     [行 100-101]
   │ ├─ encoding 解析(mono8/bgr8/rgb8 → BGR)         [行 105-116]
   │ ├─ cv2.remap(MAP1, MAP2)                           [行 119]   fisheye → pinhole
   │ ├─ 裁到 16 倍数                                    [行 122-123]
   │ ├─ to CUDA tensor (3, H, W)                        [行 126]
   │ ├─ self.timestamps.append(ts)                      [行 130]   累积 wall-clock
   │ ├─ t = len(self.timestamps) - 1                    [行 132]   帧序号
   │ ├─ 第一次:DPVO(cfg, network, ht, wd, viz=False)   [行 135-137]
   │ └─ self.slam(t, image_t, intrinsics_t)             [行 140]   ──┐
   │                                                                 │
   ├──────────────────────────────────────────────────────────────────┘
   │                                                                 │
   ▼                                                                 ▼
DPVO.__call__(tstamp=t, image, intrinsics)              [dpvo.py:377]
   │
   ├─ classic LC?                                       [行 380-381]   on=记入 retrieval/imcache
   │
   ├─ buffer 检查 self.n+1 >= self.N → 抛                [行 383-384]
   │
   ├─ image = 2 * (image[None,None] / 255.0) - 0.5      [行 389]      归一化 [-0.5, 1.5]
   │                                                                  shape: (1, 1, 3, H, W)
   ├─ Patchifier(image, M=96, strat='RANDOM')           [行 392-396]  ←── 关键 CNN forward
   │   ↓ 出
   │   fmap   (1, 1, 128, H/4, W/4)
   │   gmap   (1, 96, 128, 3, 3)         patch fnet 描述子
   │   imap   (1, 96, 384, 1, 1)         patch inet 描述子
   │   patches (1, 96, 3, 3, 3)          (x,y,d) × 3×3
   │   _      (index, 来源帧 id,全 0 因为单帧)
   │   clr    (1, 96, 3)                 patch 中心 BGR(uint8 0-255 范围)
   │
   ├─ 状态记录:
   │   tlist.append(tstamp)                              [行 399]
   │   pg.tstamps_[self.n] = self.counter                [行 400]  注意:存 counter 不是 tstamp
   │   pg.intrinsics_[self.n] = intrinsics / RES         [行 401]  ÷4
   │   pg.colors_[self.n] = ((clr+0.5)*127.5)[..,[2,1,0]] [行 404-405]  BGR → RGB
   │   pg.index_[self.n + 1] = self.n + 1                [行 407]  患者帧 id 标到下一行
   │   pg.index_map_[self.n + 1] = self.m + self.M       [行 408]
   │
   ├─ 运动模型外推(n > 1 才走这条)                     [行 410-424]
   │   MOTION_DAMPING * (c-b)/(b-a) 的 SE3.log
   │   exp(xi) * P1 → pg.poses_[self.n]                  [行 419-421]
   │
   ├─ patch 深度初始化:
   │   patches[:,:,2] = rand_like(...)                  [行 427]   始终随机
   │   if is_initialized: 改用最近 3 帧 disp 中位数       [行 429-430]
   │
   ├─ pg.patches_[self.n] = patches                      [行 432]
   │
   ├─ network buffers 写入(环形):
   │   imap_[n%pmem] / gmap_[n%pmem]                    [行 435-436]
   │   fmap1_[:,n%mem] = F.avg_pool2d(fmap[0], 1, 1)    [行 437]
   │   fmap2_[:,n%mem] = F.avg_pool2d(fmap[0], 4, 4)    [行 438]
   │
   ├─ self.counter += 1                                  [行 440]   总帧计数
   │
   ├─ ★ Init 阶段 motion_probe gate                      [行 441-444]
   │   if n>0 and not is_initialized:
   │       if motion_probe() < 2.0:    ←── 中位 flow_mag < 2 px
   │           pg.delta[counter-1] = (counter-2, Id)
   │           return    ←── 整个 __call__ 直接 return,n 不递增,跳过 update/keyframe
   │
   ├─ self.n += 1;  self.m += M                          [行 446-447]
   │
   ├─ Loop closure 加 LC edges                           [行 449-455]   LOOP_CLOSURE=True 才走
   │
   ├─ 加 forward + backward edges                        [行 458-459]
   │   forw: 前 13 帧 × M patch → 当前帧                  ~1248 条
   │   back: 当前帧 M patch → 前 13 帧                    ~1248 条
   │
   ├─ ★ Init / Update 分支                              [行 461-469]
   │   if n == 8 and not is_initialized:
   │       is_initialized = True
   │       for itr in range(12): self.update()    ←── cold-start BA
   │   elif is_initialized:
   │       self.update()                          ←── 1 次 GRU + BA
   │       self.keyframe()                        ←── 决定是否丢中间帧
   │
   └─ Classic LC 末尾两步                                [行 471-473]
        long_term_lc.attempt_loop_closure(self.n)
        long_term_lc.lc_callback()
        ←── PGO 完成后把 poses_ 整体 retr,详见 §6


主 __call__ 返回后:
        │
        ▼
DPVONode 继续 image_cb:
   ├─ if self.slam.n > 0:
   │     self._publish_latest_pose(msg.header.stamp)     [dpvo_ros_node.py:142-144]
   │         读 pg.poses_[n-1], SE3.inv() → c2w
   │         发 PoseStamped 到 /dpvo/pose
   │         frame_id = 'dpvo_world'
   │         时间戳 = 原图 header stamp(wall-clock)
   │
   └─ if t % 100 == 0: log "Processed frame {t}"
```

---

## 2. 关键子流程展开

### 2.1 `Patchifier.forward`(`net.py:110-157`)

**输入**:image `(1, 1, 3, H, W)`,M=96(patches_per_image)

```python
# [VERIFY: src/DPVO/dpvo/net.py:112-113]
fmap = self.fnet(images) / 4.0     # (1, 1, 128, H/4, W/4)
imap = self.inet(images) / 4.0     # (1, 1, 384, H/4, W/4)
```

两个并行 CNN encoder(`BasicEncoder4`,`extractor.py:200-264`):
- fnet `norm_fn='instance'`,output_dim=128 → 给 BA 的 patch fnet 描述子用
- inet `norm_fn='none'`(no normalization),output_dim=384 → 给 GRU 的 context 输入用

**Patch 中心选取**(`net.py:119-137`):

```python
if centroid_sel_strat == 'RANDOM':
    x = torch.randint(1, w-1, size=[n, patches_per_image], device="cuda")
    y = torch.randint(1, h-1, size=[n, patches_per_image], device="cuda")
```

(还有 `GRADIENT_BIAS` 模式:多采 3M 个候选,按 image gradient 排序取 top M)

**Patch 抠图**:

```python
# [VERIFY: src/DPVO/dpvo/net.py:138-149]
coords = torch.stack([x, y], dim=-1).float()
imap = altcorr.patchify(imap[0], coords, 0).view(b, -1, DIM, 1, 1)     # patch 中心 inet vec
gmap = altcorr.patchify(fmap[0], coords, P//2).view(b, -1, 128, P, P)  # 3×3 patch fnet
clr  = altcorr.patchify(images[0], 4*(coords+0.5), 0).view(b, -1, 3)   # 中心像素 BGR

grid, _ = coords_grid_with_index(disps, device=fmap.device)
patches = altcorr.patchify(grid[0], coords, P//2).view(b, -1, 3, P, P) # 3×3 (x,y,d) grid
```

`altcorr.patchify(net, coords, radius)`(`altcorr/correlation.py:51-68`):用 bilinear interpolation 在 `net` 张量上以 coords 为中心扣 `(2*radius+1)^2` 大小的 patch。

注意:`imap` 用 radius=0(只扣 1×1 中心),`gmap` 用 radius=P//2=1(扣 3×3),`patches` 也 radius=1(扣 3×3 网格)。

**输出 5-tuple**:`fmap, gmap, imap, patches, index, clr`(`net.py:155`,return_color=True)

### 2.2 运动模型(`dpvo.py:410-424`)

**条件**:`self.n > 1`(至少有两帧才能算速度)且 `MOTION_MODEL == 'DAMPED_LINEAR'`

```python
# [VERIFY: src/DPVO/dpvo/dpvo.py:411-421]
P1 = SE3(self.pg.poses_[self.n-1])    # 最新已固化的 pose
P2 = SE3(self.pg.poses_[self.n-2])    # 上上帧

*_, a, b, c = [1]*3 + self.tlist      # 取 self.tlist 末三项(用 1 填短)
fac = (c-b) / (b-a)                   # 时间间隔比

xi = self.cfg.MOTION_DAMPING * fac * (P1 * P2.inv()).log()   # SE3 tangent
tvec_qvec = (SE3.exp(xi) * P1).data
self.pg.poses_[self.n] = tvec_qvec
```

**等价数学**:
$$
P_{\text{pred}} = \exp\!\Bigl(\alpha \cdot \frac{\Delta t_{n,n-1}}{\Delta t_{n-1,n-2}} \cdot \log(P_{n-1} P_{n-2}^{-1})\Bigr) \cdot P_{n-1}
$$

- $\alpha$ = `MOTION_DAMPING` = 0.5 → 取实际相对位姿的"一半"作为下一步增量
- $\frac{\Delta t}{\Delta t}$ 为变 fps 做线性补偿,但 DPVO 传的 tstamp 是帧序号 → 等帧率下恒等于 1
- **不调用 IMU**,这是 pure VO 的运动模型

**`(P1 * P2.inv()).log()` 的几何**:
- 假设 poses_ 是 w2c:$P_{n-1} = T_{n-1 \leftarrow w}$,$P_{n-2}^{-1} = T_{w \leftarrow n-2}$
- 乘积 $P_{n-1} P_{n-2}^{-1} = T_{n-1 \leftarrow n-2}$ = 相邻两帧相机系相对位姿
- log → SE3 tangent(6 维 twist),然后乘 alpha,exp 回 group。

### 2.3 `motion_probe`(`dpvo.py:240-255`)

Init 阶段(前 8 帧)用来判断"运动够不够大"。逻辑:

```python
def motion_probe(self):
    kk = torch.arange(self.m - self.M, self.m, device="cuda")  # 当前帧的 M 个 patch
    jj = self.n * torch.ones_like(kk)                          # 重投到当前帧本身(n)
    ii = self.ix[kk]                                           # 来源帧(在 patch 在 m-M..m 之间时,ix 是 n-1)
    
    # 跑一次 GRU(不更新 net)
    net = torch.zeros(1, len(ii), self.DIM, **self.kwargs)
    coords = self.reproject(indicies=(ii, jj, kk))
    corr = self.corr(coords, indicies=(kk, jj))
    ctx = self.imap[:, kk % (self.M * self.pmem)]
    net, (delta, weight, _) = self.network.update(net, ctx, corr, None, ii, jj, kk)
    
    return torch.quantile(delta.norm(dim=-1).float(), 0.5)  # 中位数 flow
```

- 重投 patch (来源帧 n-1) → 目标帧 (n);算 GRU 的 delta(flow 估计);取中位数。
- 若中位数 < 2.0 px → 运动太小,不入图(`dpvo.py:441-444`)。
- 这是为什么 **静止启动期不要把 DPVO 启动** —— 静止时 motion_probe 永远 < 2,DPVO 卡在 init,LiDAR 已经走完前 5 秒它还在等。

[VERIFY: src/DPVO/dpvo/dpvo.py:240-255] motion_probe 实现
[VERIFY: src/DPVO/dpvo/dpvo.py:441-444] init gate

### 2.4 `DPVO.update`(`dpvo.py:328-360`)—— 核心 BA + GRU 步

```python
def update(self):
    coords = self.reproject()                # (1, |E|, 2, P, P)  重投每条 edge 的 patch
    corr = self.corr(coords)                 # (1, |E|, 2·(2r+1)² ) 4D correlation lookup
    ctx = self.imap[:, self.pg.kk % (self.M * self.pmem)]   # (1, |E|, DIM=384)
    self.pg.net, (delta, weight, _) = \
        self.network.update(self.pg.net, ctx, corr, None, self.pg.ii, self.pg.jj, self.pg.kk)
    
    lmbda = torch.as_tensor([1e-4], device="cuda")
    weight = weight.float()
    target = coords[..., self.P//2, self.P//2] + delta.float()
    
    self.pg.target = target
    self.pg.weight = weight
    
    # BA 路由
    if 长程 edge 存在 and 没跑过 global BA:
        self.__run_global_BA()           # CUDA BA + 拼接 active+inactive
    else:
        t0 = self.n - self.cfg.OPTIMIZATION_WINDOW if is_initialized else 1
        t0 = max(t0, 1)
        fastba.BA(...)                    # local BA, fixedp = t0
    
    # 更新 point cloud
    points = pops.point_cloud(SE3(self.poses), self.patches[:, :self.m], self.intrinsics, self.ix[:self.m])
    points = (points[...,1,1,:3] / points[...,1,1,3:]).reshape(-1, 3)
    self.pg.points_[:len(points)] = points[:]
```

**关键张量 shape 演化**(`|E|` = 当前活跃 edge 数,推理稳态 ~24000):

| 量 | shape | dtype |
|----|-------|-------|
| `coords` | `(1, |E|, P, P, 2)` | f32 |
| `corr` | `(1, |E|, 2·(2·3+1)²)` = `(1, |E|, 98)` | f16 |
| `ctx` | `(1, |E|, 384)` | f16 |
| `delta` | `(1, |E|, 2)` | f16 |
| `weight` | `(1, |E|, 2)` | f16 → cast f32 |
| `target` | `(1, |E|, 2)` | f32 |

[VERIFY: src/DPVO/dpvo/dpvo.py:328-360] update 函数
[VERIFY: src/DPVO/dpvo/dpvo.py:200-207] corr() impl

### 2.5 `reproject`(`dpvo.py:209-213`)

```python
def reproject(self, indicies=None):
    (ii, jj, kk) = indicies if indicies is not None else (self.pg.ii, self.pg.jj, self.pg.kk)
    coords = pops.transform(SE3(self.poses), self.patches, self.intrinsics, ii, jj, kk)
    return coords.permute(0, 1, 4, 2, 3).contiguous()
```

- 调用 `projective_ops.transform`(`projective_ops.py:53`),传 jacobian=False 路径。
- 出 `(b, |E|, P, P, 2)`,permute 到 `(b, |E|, 2, P, P)` 给 correlation 用(channel-major,适合 fmap 索引)。

### 2.6 `__run_global_BA`(`dpvo.py:312-326`)

只在 `LOOP_CLOSURE=True` 且 `OPTIMIZATION_WINDOW` 外有 edges 时触发(`dpvo.py:348`)。

```python
def __run_global_BA(self):
    # 拼 active + inactive edges
    full_target = torch.cat((self.pg.target_inac, self.pg.target), dim=1)
    full_weight = torch.cat((self.pg.weight_inac, self.pg.weight), dim=1)
    full_ii = torch.cat((self.pg.ii_inac, self.pg.ii))
    full_jj = torch.cat((self.pg.jj_inac, self.pg.jj))
    full_kk = torch.cat((self.pg.kk_inac, self.pg.kk))
    
    self.pg.normalize()                  # 归一化深度+pose 防止 monocular scale 漂
    lmbda = torch.as_tensor([1e-4], device="cuda")
    t0 = self.pg.ii.min().item()
    fastba.BA(self.poses, self.patches, self.intrinsics,
        full_target, full_weight, lmbda, full_ii, full_jj, full_kk,
        t0, self.n, M=self.M, iterations=2, eff_impl=True)   # ★ eff_impl=True 走 block_e.cu
    self.ran_global_ba[self.n] = True
```

- `eff_impl=True` 切换 CUDA kernel 用更紧凑的 patch-major Schur block 存储(`block_e.cu`)。
- 一次 global BA 后 `ran_global_ba[self.n] = True` 防止下一帧重跑。

### 2.7 `keyframe`(`dpvo.py:266-310`)

**目标**:窗口里中间某帧若运动太小(redundant)就丢掉。

```python
def keyframe(self):
    i = self.n - self.cfg.KEYFRAME_INDEX - 1    # KEYFRAME_INDEX=4
    j = self.n - self.cfg.KEYFRAME_INDEX + 1
    m = self.motionmag(i, j) + self.motionmag(j, i)
    
    if m / 2 < self.cfg.KEYFRAME_THRESH:        # KEYFRAME_THRESH=15.0
        k = self.n - self.cfg.KEYFRAME_INDEX    # 要丢的帧
        t0 = self.pg.tstamps_[k-1]
        t1 = self.pg.tstamps_[k]
        
        dP = SE3(self.pg.poses_[k]) * SE3(self.pg.poses_[k-1]).inv()
        self.pg.delta[t1] = (t0, dP)            # 存档以便 terminate 还原
        
        to_remove = (self.pg.ii == k) | (self.pg.jj == k)
        self.remove_factors(to_remove, store=False)
        
        # 整体左移
        self.pg.kk[self.pg.ii > k] -= self.M
        self.pg.ii[self.pg.ii > k] -= 1
        self.pg.jj[self.pg.jj > k] -= 1
        for i in range(k, self.n-1):
            ...
        self.n -= 1
        self.m -= self.M
    
    # 移老 edge
    to_remove = self.ix[self.pg.kk] < self.n - self.cfg.REMOVAL_WINDOW
    if self.cfg.LOOP_CLOSURE:
        lc_edges = ((self.pg.jj - self.pg.ii) > 30) & (self.pg.jj > (self.n - self.cfg.OPTIMIZATION_WINDOW))
        to_remove = to_remove & ~lc_edges       # 保留 LC edges
    self.remove_factors(to_remove, store=True)
```

**两阶段**:
1. **可能丢中间帧**:看 `n-5` 和 `n-3` 之间的 flow magnitude;若太小,丢 `n-4` 帧(`KEYFRAME_INDEX=4`)。
2. **移老 edge**:`ix[kk] < n - 22` 的 edge 全部 store=True 移到 inactive(为 global BA 保留)。

[VERIFY: src/DPVO/dpvo/dpvo.py:257-264] motionmag (flow_mag 累加)

### 2.8 `terminate`(`dpvo.py:173-198`)

```python
def terminate(self):
    if CLASSIC_LOOP_CLOSURE: long_term_lc.terminate(self.n)
    if LOOP_CLOSURE: append_factors(*pg.edges_loop())
    
    for _ in range(12):                          # final cleanup BA × 12
        self.ran_global_ba[self.n] = False
        self.update()
    
    # 插值还原丢帧
    self.traj = {pg.tstamps_[i]: pg.poses_[i] for i in range(self.n)}
    poses = [self.get_pose(t) for t in range(self.counter)]   # 0..counter-1 都填回去
    poses = lietorch.stack(poses, dim=0)
    poses = poses.inv().data.cpu().numpy()       # ★ w2c → c2w
    tstamps = np.array(self.tlist, dtype=np.float64)
    return poses, tstamps
```

- 最后 12 次 update 让 BA 收敛到结尾。
- `get_pose(t)` 递归走 `pg.delta` 把丢掉的帧 SE3 复原(`dpvo.py:166-171`)。
- 返回的 `poses` 是 **c2w**,`tstamps` 是 `self.tlist`(就是当初传给 `__call__` 的 tstamp 参数)。
- ROS 节点会把这个 tstamps(序号)再 map 回 wall-clock(`dpvo_ros_node.py:185-193`)。

---

## 3. 一帧的"分支决策树"

```
                  __call__(tstamp, image, intrinsics)
                          │
                          ▼
              n+1 < BUFFER_SIZE ?  ── No → raise Exception
                          │ Yes
                          ▼
                   Patchify + 写状态
                          │
                          ▼
              n > 1 and DAMPED_LINEAR? ── No (前两帧) → 沿用上帧 pose
                          │ Yes
                          ▼
                  运动模型外推 P_pred
                          │
                          ▼
                深度初始化 random / 历史中位
                          │
                          ▼
              ┌─── n > 0 and not is_initialized? ───┐
              │                                     │
              Yes                                  No
              │                                     │
              ▼                                     │
        motion_probe < 2.0?                         │
              │                                     │
        ┌─────┴─────┐                               │
        │           │                               │
       Yes          No                              │
        │           │                               │
        ▼           ▼                               │
   记 delta,      n+=1,                            │
   return         m+=M                              │
                   │                                │
                   ▼                                │
              加 LC edges                           │
              加 forw + back edges                  │
                   │                                │
                   ▼                                │
              ┌─── n == 8 ? ───┐                   │
              │                │                   │
             Yes               No                  │
              │                │                   │
              ▼                ▼                   │
       is_initialized = True   self.update()       │
       12 × self.update()      self.keyframe()     │
                                                   │
                          ─────────────────────────┘
                          │
                          ▼
                  ROS 节点继续:_publish_latest_pose
```

[VERIFY: src/DPVO/dpvo/dpvo.py:377-473] `__call__` 完整逻辑

---

## 4. Edge 生命周期(单条 edge 的视角)

某 edge `e = (ii=10, jj=15, kk=10·M+5)` 表示"第 10 帧第 5 号 patch 重投到第 15 帧"。

```
帧 11 入图(this 帧)
   ├─ append_factors(forw + back)
   ├─ forw: (kk=10·M+5 .. , jj=11) [作为 forward edge 引入]
   │   此时 e 的 ii=10, jj=11(不是 15;15 还没到)
   │
帧 12..15 入图
   ├─ 每次 update() 会重新 GRU + BA,e 不变
   ├─ e 的 (target, weight) 每次被 GRU 覆盖
   │
帧 15 入图,加入 forw edge (kk=10·M+5, jj=15)
   ├─ 这是另一条 edge,不是同一个 e
   ├─ DPVO 不"延长" edge,每对 (ii, jj, kk) 都是独立 edge
   │
帧 N 时:if ix[kk] < N - 22 → e 进入 inactive
   ├─ remove_factors(store=True) 把 e 的 (ii, jj, kk, weight, target) 拷到 *_inac
   ├─ active edges 删除 e
   │
若 LOOP_CLOSURE=True 且 jj-ii > 30:跳过删除,保留为 LC edge
   │
若 keyframe 决定丢中间帧 k=ii(10):
   ├─ remove_factors((ii==10)|(jj==10), store=False)  # 不存档,直接丢
   ├─ 所有 ix > 10 的 ii/jj/kk 减 1(整体左移)
```

[VERIFY: src/DPVO/dpvo/dpvo.py:215-238] append_factors / remove_factors

---

## 5. ROS 节点 → DPVO 的张量契约

**`DPVONode.image_cb` → `DPVO.__call__` 入参**:

| 参数 | shape | dtype | device | 来源 |
|------|-------|-------|--------|------|
| `tstamp` | scalar int | int | cpu | `len(self.timestamps) - 1`(`dpvo_ros_node.py:132`)|
| `image` | `(3, H, W)` | uint8 | cuda | `cv2.remap` → torch tensor(`dpvo_ros_node.py:119-126`)|
| `intrinsics` | `(4,)` | float64 → cast f32 | cuda | `np.array([FX,FY,CX,CY])` 硬编码(`dpvo_ros_node.py:36`)|

**H, W 约束**:必须是 16 的倍数(`dpvo_ros_node.py:122-123` 自动裁,但 fmap pyramid 需要)。GeoScan 是 1280×1024 → 已经合规,不需要裁。

**Intrinsics 单位**:像素(全分辨率,DPVO 内部 `__call__` 里 / RES=4)。

**publish 路径** `_publish_latest_pose`(`dpvo_ros_node.py:149-173`):

```
pg.poses_[n-1:n]  (1, 7) w2c
   │ SE3(...).inv()
   ▼
(1, 7) c2w
   │ .data.cpu().numpy()[0]
   ▼
(tx, ty, tz, qx, qy, qz, qw)
   │
   ▼
PoseStamped:
  header.stamp = msg.header.stamp  (原图 wall-clock)
  header.frame_id = 'dpvo_world'
  pose.position = (tx, ty, tz)
  pose.orientation = (qx, qy, qz, qw)
   │ publish to /dpvo/pose (queue 50)
   ▼
GLIM ext / casbot bridge 订阅
```

[VERIFY: src/DPVO/dpvo_ros_node.py:149-173] `_publish_latest_pose`

---

## 6. v2 长期 LC 的并行流(`loop_closure/long_term.py`)

DPVO 有两套 LC:
- **`LOOP_CLOSURE`**(简单):`edges_loop()`(`patchgraph.py:56-82`)在窗口里加远距 edge,顺手让 BA 多优化几下。
- **`CLASSIC_LOOP_CLOSURE`**(完整 PGO):多进程并行 PGO + DBoW2 检索 + DISK+LightGlue feature match。

**完整 PGO 流(LOOP_CLOSURE_CLASSIC 模式)**:

```
__call__(tstamp, image, intrinsics):
   ├─ long_term_lc(image, n)                        # imcache + retrieval(每帧)
   │   ├─ retrieval(img_np, n)                       # DBoW2 add image
   │   └─ imcache(img_np, n)                         # 缓存原图(用于后续 LC triangulation)
   │
   ├─ keyframe → long_term_lc.keyframe(k)            # imcache 也同步丢帧
   │
   └─ 末尾:
       ├─ long_term_lc.attempt_loop_closure(n)
       │   ├─ retrieval.detect_loop(thresh=LOOP_RETR_THRESH=0.04, num_repeat=LOOP_CLOSE_WINDOW_SIZE=3)
       │   │       返回候选对 (i, j) 或 None
       │   ├─ close_loop(i, j, n):
       │   │   ├─ estimate_3d_keypoints(i) 和 (j)
       │   │   │       用 DISK 检测 2048 点 → LightGlue 匹配 i-1↔i↔i+1 三视图
       │   │   │       triangulate → 3D points + features
       │   │   ├─ matcher({i_feat, j_feat}) 跨 loop pair 匹配
       │   │   ├─ ransac_umeyama(i_pts, j_pts) → Sim3 rt s (numba JIT, 400 iter)
       │   │   └─ lc_pool.apply_async(run_DPVO_PGO, ...) ── 异步进程
       │   │           run_DPVO_PGO 在子进程跑 LM × 30 iter,完了 queue.put
       │   │           [VERIFY: optim_utils.py:202-243]
       │   └─ retrieval.confirm_loop(i, j) 防止重复触发
       │
       └─ long_term_lc.lc_callback()
           ├─ if not queue.empty():
           │     final_est = queue.get()
           │     pg.poses_[:safe_i] = SE3(res).inv().data
           │     pg.patches_[:safe_i,:,2] /= s.view(...)
           │     _rescale_deltas(s1)
           │     pg.normalize()
           └─ 主进程的 pg 状态被异步进程的 PGO 结果覆盖
```

**关键设计**:
- LC 跑在 `mp.Pool(processes=1)` 子进程,主进程不阻塞(`long_term.py:30-31`)。
- 子进程 PGO 完后通过 `manager.Queue` 把 PGO 后的 `pp.Sim3` 数据传回主进程。
- 主进程 `lc_callback` 调一次:检 queue 非空 → 写回 `pg.poses_` 和 `pg.patches_[:,:,2]`(深度按 s rescale)。
- 子进程的 PGO 是 7-DoF Sim3 优化(`optim_utils.py:163-200` 残差,`solve_system` Cholesky 稀疏)。

**未在本机验证**:CLASSIC_LOOP_CLOSURE 在 GeoScan 上需要 `kornia.feature.DISK` 和 `LightGlue` 的预训练权重(`long_term.py:46-47`)。GeoScan benchmark 跑的是 `LOOP_CLOSURE=False`(`config/default.yaml` 默认),即纯前端 VO。

---

## 7. 一帧的时间预算(经验值,RTX 4060 1280×1024 GeoScan)

从主仓库 benchmark 记录(`06-benchmarks.md` DPVO 行)和 timer 抽样估算:

| 步骤 | 耗时 | 备注 |
|------|------|------|
| cv2.remap fisheye → pinhole | ~3 ms | CPU |
| torch tensor + .cuda() | ~1 ms | |
| Patchifier forward(fnet + inet)| ~15 ms | fp16 |
| Update GRU(× 1 iter)| ~12 ms | fp16 |
| corr(双尺度 lookup)| ~3 ms | CUDA |
| BA(CUDA, 2 iter local)| ~8 ms | |
| keyframe(motionmag 等)| ~2 ms | |
| **单帧合计**(非 init)| **~45 ms** | ≈ 22 FPS 理论 |
| 实测 throughput | **~12 FPS** | Python overhead + GIL |

Init 阶段(n==8 时):12 × update ≈ 250 ms,这是为什么 ROS watchdog 用 15s 才超时(`dpvo_ros_node.py:88`)。

---

## 8. 验证摘要

| 数据流路径 | 证据 |
|-----------|------|
| ROS Image → cv2.remap → tensor | dpvo_ros_node.py:104-126 |
| `__call__` 入口流 | dpvo.py:377-473 整段 |
| Patchifier 出 5-tuple | net.py:110-157 |
| 运动模型 SE3 外推 | dpvo.py:410-424 |
| motion_probe gate | dpvo.py:240-255, 441-444 |
| append_factors forw/back | dpvo.py:362-375, 458-459 |
| update GRU + BA | dpvo.py:328-360 |
| __run_global_BA | dpvo.py:312-326 |
| keyframe 丢帧逻辑 | dpvo.py:266-310 |
| terminate 还原丢帧 | dpvo.py:173-198 |
| _publish_latest_pose w2c→c2w | dpvo_ros_node.py:149-173 |
| LC 异步进程流 | long_term.py:140-203 |

下一步:`03-algorithms.md`(BA Schur 推导 + Jacobian 推导 + Update GRU 设计)。

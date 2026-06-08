# ALGORITHM 03 — Tracker(SDF Odometry)

> Phase 4 文档系列之三。对应 `utils/tracker.py`(810 行)。
> 路径相对 `src/PIN_SLAM/`。

---

## 0. TL;DR

PIN-SLAM 的里程计 = **点到隐式 SDF 的 LM 配准**。

- 输入: 源点云 `cur_source_points`(传感器系,N_src ~ 千级)+ 上一帧位姿外推的初值
- 每次迭代: 把源点云用当前 T 变到世界系 → query SDF & ∇SDF → 计算 6-DoF Jacobian → 解 LM 步进 → 更新 T
- 最多 50 次迭代,带 4 道安全门
- 输出: T(4×4 float64)、6×6 协方差、退化方向特征值、`valid_flag`

数学上类似 KISS-ICP 的 point-to-plane,但**平面被换成 SDF 的零等值面**,法线 = ∇SDF 单位向量。

---

## 1. 数学模型

### 1.1 优化目标

给定:
- 源点云 `P = {p_i}_{i=1}^N`(传感器系)
- 隐式 SDF `s(q)`(由 NeuralPoints + MLP 定义,世界系)

求 `T ∈ SE(3)` 使得变换后所有源点都在零等值面上:
```
min_T  Σ_i  w_i · ‖ s(T · p_i) ‖²
```
其中 `w_i` 是鲁棒权重(Geman-McClure + 距离 + 法线一致性 + 颜色一致性)。

### 1.2 一阶展开

设当前估计 T_k,扰动 ξ ∈ se(3) = (φ, t) ∈ R⁶:
```
T_{k+1} = exp(ξ) · T_k    (左乘扰动)
```
对 `s(q)` 沿 q 一阶展开:
```
s(exp(ξ) · T_k · p_i) ≈ s(q_i) + ∇s(q_i)^T · (q_i × φ + t)
                                 ^^^^^^^^^^^^^^^^^^^^^^^^
                                 = (q_i ∧ ∇s_i,  ∇s_i) · ξ
                                 = J_i · ξ
```
其中
- `q_i = T_k · p_i`(世界系当前位置)
- `J_i ∈ R^{1×6}` = `[cross(q_i, ∇s_i), ∇s_i]` ← **先 rot,再 trans**

证据 `[VERIFY: utils/tracker.py:652-655]`:
```python
cross = torch.linalg.cross(points, sdf_grad, dim=-1)         # [N, 3]
J_mat = torch.cat([cross, sdf_grad], -1)                      # [N, 6]
```

### 1.3 GN / LM 正规方程

带权 Gauss-Newton:
```
N = J^T · W · J    ∈ R^{6×6}
g = -J^T · W · r   ∈ R^{6}
   N · ξ = g
   ξ = N⁻¹ · g
```
其中 `r_i = s_pred_i - s_label_i`(label 通常 0)。LM 加阻尼:
```
N ← N + λ · diag(N)
```
证据 `[VERIFY: utils/tracker.py:656-673]`:
```python
N_mat = J_mat.T @ (weight * J_mat)
N_mat += lm_lambda * torch.diag(torch.diag(N_mat))
g_vec = -(J_mat * weight).T @ sdf_residual
t_vec = torch.linalg.inv(N_mat) @ g_vec
```

### 1.4 ξ → SE(3)

```python
T_mat = eye(4)
T_mat[:3, :3] = expmap(ξ[:3])      # 旋转 axis-angle → R
T_mat[:3, 3]  = ξ[3:]              # 平移直接赋
```
证据 `[VERIFY: utils/tracker.py:677-679]`,`expmap` 是经典 Rodrigues `[VERIFY: utils/tracker.py:784-795]`。

⚠️ 注意:这是**增量**T_mat,主循环里 `T = ΔT @ T` `[VERIFY: utils/tracker.py:147]`。

### 1.5 协方差(可选)

`require_cov=True` 时:
```
mse = mean(w · r²)
Σ = N_raw⁻¹ · mse      (6×6)
```
其中 `N_raw` 是未加 λ damping 的版本。证据 `[VERIFY: utils/tracker.py:689-693]`。

### 1.6 退化检测

`require_eigen=True` 时:
```
Σ_trans = N_raw[3:, 3:]            # 平移部分 3×3
eigvals = eigvals(Σ_trans)
```
最小特征值用作"退化方向"指标:`min(eigvals) < n_valid × eigenvalue_ratio_thre` 判退化 `[VERIFY: utils/tracker.py:684-686, :205-216]`。

---

## 2. `tracking()` 主循环

### 2.1 接口

```python
T, cov_mat, weight_pc, valid_flag = tracker.tracking(
    source_points,           # [N, 3] sensor frame
    init_pose=None,          # [4, 4] float64 (uniform motion / loop init)
    source_colors=None,      # [N, c]
    source_normals=None,     # [N, 3]  (一般无)
    source_semantics=None,   # 未用
    source_sdf=None,         # 默认 None → 等价于 [0,…,0] 全零(zero level set)
    cur_ts=None,             # 未用
    loop_reg=False,          # True 走回环精配,放宽阈值
    vis_result=False,        # True 时生成 weight_point_cloud
)
```

### 2.2 主循环骨架

```python
def tracking(self, source_points, init_pose, ...):
    T = init_pose if init_pose is not None else eye(4, float64)
    cov_mat = None

    # 超参从 config 读
    min_grad_norm   = config.reg_min_grad_norm    # 0.5
    max_grad_norm   = config.reg_max_grad_norm    # 2.0
    GM_dist         = config.reg_GM_dist_m        # 0.3
    GM_grad         = config.reg_GM_grad           # 0.1
    lm_lambda       = config.reg_lm_lambda         # 1e-4
    iter_n          = config.reg_iter_n            # 50
    term_thre_deg   = 0.01
    term_thre_m     = 0.001

    max_final_sdf_cm    = σ × final_residual_ratio_thre × 100   # σ × 0.6 × 100 = 15 cm
    min_valid_ratio     = 0.2 (loop_reg 0.15)
    max_inc_ratio       = 1.1                # 残差增长比例上限
    eigval_ratio_thre   = 0.005
    min_valid_pts       = 30
    converged = False
    valid_flag = True
    last_sdf_residual_cm = 1e5

    if source_sdf is None:
        source_sdf = zeros(N_src)            # zero level set 默认

    for i in range(iter_n):
        cur_points = transform_torch(source_points, T)        # apply current T
        (ΔT, cov, eigenvalues, weight_pc, valid_pts,
         sdf_residual_cm, photo_residual) = self.registration_step(
            cur_points, normals, source_sdf, colors,
            min_grad_norm, max_grad_norm, GM_dist, GM_grad,
            lm_lambda, (vis_result and converged))

        T = ΔT @ T

        # ─── 安全门 1: 残差爆炸 ────
        if (sdf_residual_cm - last_sdf_residual_cm) / last_sdf_residual_cm > max_inc_ratio:
            valid_flag = False
        else:
            last_sdf_residual_cm = sdf_residual_cm

        # ─── 安全门 2: 有效点不足 ────
        if (valid_pts < min_valid_pts) or
           (valid_pts / N_src < min_valid_ratio):
            valid_flag = False

        if not valid_flag or converged:
            break

        # ─── 收敛检查 ────
        rot_angle_deg = rotation_matrix_to_axis_angle(ΔT[:3, :3]) × 180 / π
        tran_m = ΔT[:3, 3].norm()
        if |rot_angle_deg| < term_thre_deg and tran_m < term_thre_m or i == iter_n - 2:
            converged = True                  # 再跑 1 次拿可视化数据

    # ─── 安全门 3: 最终残差 ────
    if sdf_residual_cm > max_final_sdf_cm:
        valid_flag = False

    # ─── 安全门 4: 退化 ────
    if eigenvalues is not None:
        min_eigv = min(eigenvalues)
        if config.eigenvalue_check and min_eigv < valid_pts × eigval_ratio_thre:
            valid_flag = False

    if not valid_flag and i < 10:
        T = init_pose                # 没跑够 iter 就废,回退初值
        cov_mat = None

    return T, cov_mat, weight_pc, valid_flag
```

证据(逐块):
- 初始化 `[VERIFY: utils/tracker.py:70-104]`
- 主循环 `[VERIFY: utils/tracker.py:114-186]`
- 安全门 3+4 `[VERIFY: utils/tracker.py:198-216]`
- fallback `[VERIFY: utils/tracker.py:221-223]`

### 2.3 安全门组合的语义

| 失败模式 | 触发门 | 含义 |
|----------|--------|------|
| 残差跳变(初值差太多)| 1 | LM 走错方向 |
| 大部分点无邻居 | 2 | 地图覆盖不足或位姿初值远 |
| 表面 ill-fit | 3 | 几何不匹配,可能动态场景 |
| 几何退化(隧道 / 长走廊)| 4 | Hessian 病态,平移不可观 |

任一触发 → `valid_flag = False` → 上层逻辑:
- `dataset.lose_track = True` `[VERIFY: pin_slam.py:265]`
- 这一帧不入数据池 `[VERIFY: pin_slam.py:368]`
- 连续 ≥5 帧 → reboot

---

## 3. `registration_step()` 单步

### 3.1 流程图

```
┌───────────────────────────────────────────────────┐
│ registration_step(points, normals, sdf_labels,    │
│                   colors, ..., lm_lambda, vis)    │
└───────────────────────────────────────────────────┘
       │
       │  ① query_source_points → sdf_pred, sdf_grad,
       │     color_pred, color_grad, mask, certainty, sdf_std
       ▼
   ┌───────────────────────────────────────┐
   │ valid_idx 合成 mask                   │
   │  · nn_count ≥ track_mask_query_nn_k   │
   │  · min < ‖∇s‖ < max                   │
   │  · sdf_std < σ × max_sdf_std_ratio    │
   └───────────────────────────────────────┘
       │
       │  ② 鲁棒权重 w = w_grad × w_res × w_normal × w_color × w_std
       ▼
   ┌───────────────────────────────────────┐
   │ implicit_reg (或 implicit_color_reg)  │
   │  · J = [cross(q, ∇s), ∇s]             │
   │  · N = J^T W J + λ diag(N)            │
   │  · g = -J^T W r                       │
   │  · ξ = N⁻¹ g                          │
   │  · ΔT = [expmap(ξ_rot), ξ_trans]      │
   └───────────────────────────────────────┘
       │
       ▼
   return ΔT, cov_mat, eigenvalues, weight_pc, valid_points,
          sdf_residual_cm, photo_residual
```

### 3.2 `query_source_points`(关键子调用)

```python
def query_source_points(self, coord, bs, query_sdf=True, query_sdf_grad=True,
                        query_color=False, query_color_grad=False, query_sem=False,
                        query_mask=True, query_certainty=True, query_locally=True,
                        mask_min_nn_count=4):
    # 输出: 每个点的 sdf / sdf_grad / color / color_grad / sem / mc_mask / certainty / sdf_std
    iter_n = ceil(N / bs)
    for n in range(iter_n):
        batch_coord = coord[n*bs : (n+1)*bs]
        if query_sdf_grad or query_color_grad:
            batch_coord.requires_grad_(True)

        # ALGORITHM 01 的 query_feature
        geo_feat, color_feat, weight_knn, nn_count, certainty = neural_points.query_feature(
            batch_coord, training_mode=False,        # 注意!跟踪不写 certainty
            query_locally=query_locally,
            query_color_feature=query_color)

        if query_sdf:
            batch_sdf = sdf_mlp.sdf(geo_feat)
            if not weighted_first:
                # weighted_after 模式:每个邻居先 sdf,再加权聚合 + 算 std
                batch_sdf_mean = sum(batch_sdf * weight_knn, dim=1)
                batch_sdf_var  = sum(weight_knn * (batch_sdf - mean)**2, dim=1)
                batch_sdf_std  = sqrt(var)
                batch_sdf = batch_sdf_mean
                sdf_std[...] = batch_sdf_std.detach()

            if query_sdf_grad:
                batch_sdf_grad = get_gradient(batch_coord, batch_sdf)
                                            # ↑ autograd,跟踪用解析梯度而非数值
                sdf_grad[...] = batch_sdf_grad.detach()
            sdf_pred[...] = batch_sdf.detach()

        # ... color / sem 类似
        if query_mask:
            mc_mask[...] = nn_count >= mask_min_nn_count    # default 4
        if query_certainty:
            certainty[...] = batch_certainty.detach()

    return sdf_pred, sdf_grad, color_pred, color_grad, sem_pred, mc_mask, certainty, sdf_std
```

证据 `[VERIFY: utils/tracker.py:227-365]`

#### 关键点

1. **`training_mode=False`** — 跟踪不允许写 certainty,因为 certainty 是"地图被观测多少次"的指标,跟踪是只读
2. **解析梯度**(`get_gradient`) — 与 mapping 用 numerical_grad 不同,因为跟踪只需要一次性梯度,可以接受 backward 的开销
3. **`bs = config.infer_bs = bs × 32 = 16384 × 32 = 524288`** — 推理 batch 远大于训练
4. **`weighted_first=False` 才算 std** — 因为只有"每邻居独立预测"才有"邻居方差"的意义

#### `mc_mask` 命名混淆

`mc_mask` 字面上是 marching cubes mask,但跟踪复用它表示 "nn_count ≥ K_thre 的有效点"。tracker 里 `mask_min_nn_count = config.track_mask_query_nn_k`(默认 = `query_nn_k` = 6)`[VERIFY: utils/tracker.py:404]`。

### 3.3 valid mask 组合

```python
valid_idx = mask
          & (grad_norm < max_grad_norm)     # ‖∇s‖ < 2.0  → 滤掉病态梯度
          & (grad_norm > min_grad_norm)     # ‖∇s‖ > 0.5  → 必须有足够梯度信息
          & (sdf_std < max_sdf_std_ratio × σ)   # σ × 1.0 = 0.25,std 不能太大
# 注释掉的 & (sdf_pred_abs < max_sdf)   ← 不用绝对 SDF 范围
```
证据 `[VERIFY: utils/tracker.py:419-425]`

`max_grad_norm = 2.0` 而不是 1.0(理论值):因为数值梯度 + 噪声 latent 总会偏出 1。`min_grad_norm = 0.5` 滤掉"远离表面区域 ∇s ≈ 0"的点。

### 3.4 鲁棒权重(关键)

```python
grad_anomaly = grad_norm - 1.0           # 偏离理论 ‖∇s‖=1
sdf_residual = sdf_pred - sdf_labels      # 残差(label 多为 0)

# Geman-McClure
w_grad = (GM_grad / (GM_grad + grad_anomaly²))²   ∈ [0, 1]
w_res  = (GM_dist / (GM_dist + sdf_residual²))²   ∈ [0, 1]

# 法线一致性(若有法线)
w_normal = 0.5 + |valid_normals · valid_grad_unit|    ∈ [0.5, 1.5]

w_color = 1.0
if colors_on and not photo_loss_on and consist_wieght_on:
    w_color = exp(-mean(|colors - color_pred|, dim=-1))     ∈ [0, 1]

w = w_res × w_grad × w_normal × w_color × w_certainty × w_std   # w_cert/w_std = 1
w = w / (2 × mean(w))                     # 归一化(为了可视化稳定)
```
证据 `[VERIFY: utils/tracker.py:471-524]`

#### Geman-McClure 数学

`GM(r) = (k / (k + r²))²`,k 是尺度参数。

- r → 0:GM → 1(内点权 1)
- r → ∞:GM → (k/r²)² → 0(外点权快速降到 0)
- r = k:GM = 0.25(过渡点)

对比 Huber:Huber 在 r > δ 时 weight 线性衰减,GM **二次方衰减**,对大残差惩罚更弱 → 更多内外点容忍。

#### 论文 ref

> "the Geman-McClure robust weight here (https://arxiv.org/pdf/1810.01474.pdf)"  
> 源码注释指向 BARF 论文,但实际 GM 起源更早(Geman & McClure 1987)。Wiesmann LocNDF 2023 用了类似形式。

### 3.5 photometric branch(可选)

`photometric_loss_on=True` 时(color_on 必须先开)`utils/tracker.py:529-543`:

```python
T = implicit_color_reg(valid_points, sdf_grad, sdf_residual,
                       colors, color_grad, color_residual,
                       w, w_photo_loss=photometric_loss_weight,
                       lm_lambda=lm_lambda)
```

`implicit_color_reg`(`utils/tracker.py:699-744`)把 geo + color 两路 Jacobian 拼起来:
```
N = J_geo^T W J_geo + w_photo × Σ_c J_color_c^T W J_color_c
g = -J_geo^T W r_geo - w_photo × Σ_c J_color_c^T W r_color_c
```
其中 `J_color_c = [cross(q, ∇c_c), ∇c_c]` 与 SDF 形式一致,只是把 SDF 梯度换成颜色梯度。

代价:每个颜色通道多 6×6 矩阵装配 → 慢 ~30%。

---

## 4. 数值与超参对位姿质量的影响

### 4.1 `reg_iter_n`(默认 50)

- 5 次: 残差未收敛,误差 ~10cm 级
- 20 次: 实际多数场景已收敛
- 50 次(默认): 安全冗余
- 100 次: 性能 / 速度损失,无明显增益

可视化时设 `converged=True` 后再跑 1 次以拿到稳定的 weight_pc。

### 4.2 `reg_GM_dist_m`(默认 0.3)

- 太小(0.1): 表面附近样本权重 ↓,远点不参与 → 表面平滑度高但收敛慢
- 默认 0.3 ≈ surface_sample_range × 1.2,适配 σ
- 太大(1.0): 等价 GM → L2,失去鲁棒性

### 4.3 `reg_GM_grad`(默认 0.1)

- 控制对"梯度异常"的容忍度
- 太小:仅 ‖∇s‖ ≈ 1 的点参与,边缘 / 转角点被剔除 → 退化敏感
- 太大:接受 ‖∇s‖ ∈ [0, 2] 的点,但这些点本身是 SDF 学得不好的区域

### 4.4 `reg_lm_lambda`(默认 1e-4)

- 0(纯 GN): 在 ill-conditioned 时易跳变
- 默认 1e-4: 轻微正则,鼓励 GN 行为
- 1e-2(强 LM): 偏 gradient descent,慢但稳

### 4.5 `eigenvalue_ratio_thre`(默认 0.005)

- 退化判别: `min(eigvals) < n_valid × 0.005`
- 长走廊场景(纵向不可观)经常触发
- 关掉(`eigenvalue_check = False`)可以容忍退化但风险大

---

## 5. 协方差矩阵与 PGO 集成

### 5.1 协方差含义

```python
mse = mean(w · r²)
cov_mat = inv(N_raw) × mse   # 6×6
```

理论上这是高斯近似下的 ξ 后验协方差(`ξ | data`)。但 ⚠️ 注意:
- 用的是 `N_raw`(不加 λ),所以是 GN 协方差,不是 LM
- `weight` 在归一化后已经平均化,所以 `mse` 不是真正残差方差
- 顺序: **rot, then trans**(与 J 的列顺序一致)

### 5.2 与 gtsam 集成

`config.use_reg_cov_mat=True` 时:
```python
cur_edge_cov = cur_odom_cov if config.use_reg_cov_mat else None
pgm.add_odometry_factor(frame_id, frame_id-1, last_odom_tran, cov=cur_edge_cov)
```
证据 `[VERIFY: pin_slam.py:296-297, utils/pgo.py:130-133]`

注:`gtsam.noiseModel.Gaussian.Covariance(cov)` 期望 σ 矩阵的格式(协方差),PIN 给的 6×6 矩阵 ord 是 rot+trans。gtsam 内部用同样约定,所以直接传入即可。

### 5.3 默认 False

`use_reg_cov_mat = False` 默认,因为:
- 协方差估计严重依赖 weight 选择,经验上不稳
- 默认用 `pgo_tran_std=0.04`, `pgo_rot_std=0.01°` 的常数对角矩阵更鲁棒
- 想用 reg_cov 需要先证明跟踪的协方差是 well-calibrated

---

## 6. 回环精配(`loop_reg=True`)

### 6.1 流程

主循环里 (`pin_slam.py:317-321`):
```python
# 找到 loop_id,初始估计 pose_init = T_w<-loop @ loop_transform
neural_points.recreate_hash(pose_init[:3,3], None, True, True, loop_id)
# ↑ 把局部地图切到 loop 候选帧处
pose_refine, loop_cov, weight_pc, reg_valid = tracker.tracking(
    cur_source_points.clone(),
    pose_init,
    loop_reg=True)
```

### 6.2 与正常跟踪的差异

只有一个: `min_valid_ratio = 0.15`(正常 0.2)`[VERIFY: utils/tracker.py:96-97]`。

为什么放宽?
- 回环时位姿初值偏差大,部分源点本来就 fall outside 局部地图
- 即使 15% 有效也算成功(配准会自动收敛到稳定位姿)

### 6.3 验证

`reg_valid_flag = pgm.add_loop_factor(...)` 决定是否真正加入 PGO 边 `[VERIFY: pin_slam.py:325]`,不仅看 tracker 的 valid_flag,还看 PGO 的 error threshold。

---

## 7. 实数案例

设
- LiDAR ~ 60° 视场,每帧 1024 × 64 = 65K 点
- preprocess 降采样到 ~30K
- source 第二次降采样到 ~3K(`source_vox_down_m = 0.8`)
- query_nn_k = 6,track_mask_query_nn_k = 6

每次 tracking 迭代:
- `query_source_points`: 3K × (1 sdf forward + 1 grad backward) ≈ 5-10 ms
- `valid_idx` 过滤 → 通常 60-80% 有效 → ~2K 有效点
- `implicit_reg`: 2K × 6 jacobian = 12K matmul + 6×6 inv,~1 ms
- 总: ~10-15 ms/iter × 平均 8 iter = ~100 ms/frame

实际可在 ~10 Hz 处理 64 线激光。

---

## 8. 边界 case 与陷阱

### 8.1 第 0 帧:无跟踪

```python
if frame_id > 0:
    if config.track_on:
        tracking_result = tracker.tracking(...)
    ...
```
证据 `[VERIFY: pin_slam.py:260-266]`

第 0 帧跳过跟踪,直接用 cur_pose_ref(GT 或 identity)初始化。

### 8.2 lose_track 后的连续帧

跟踪失败的下一帧:
- `cur_pose_init_guess = self.last_pose_ref`(static,因为 `not self.lose_track` 为 False)`[VERIFY: dataset/slam_dataset.py:374-381]`
- 即把 uniform motion 关掉,假设车没动
- 这是因为 lose_track 后 last_odom_tran 不可靠

### 8.3 退化场景

长直走廊:
- 横向位移无 SDF 信号(∇s 都垂直走廊轴)
- N_mat 的某个特征值 → 0
- `eigenvalue_check` 判定 → valid=False
- 但实际上可能算出一个完全错的位姿,直接 reject 反而比"用错位姿"安全

### 8.4 photometric 模式陷阱

`photometric_loss_on=True` 时:
- `w_color = 1.0`(因为 color 已在 loss 项里)
- `w_normal` 仍生效
- 颜色 Jacobian 与 SDF Jacobian 的尺度可能不匹配 → `photometric_loss_weight=0.01` 默认很小

### 8.5 第 ≤ 10 iter 失败的特殊回退

```python
if not valid_flag and i < 10:
    T = init_pose
    cov_mat = None
```
证据 `[VERIFY: utils/tracker.py:221-223]`

如果优化在前 10 次就失败,不用最后那个 T(可能已经走偏),回退到 init_pose。i ≥ 10 才用当前 T 即使 invalid,因为说明它至少收敛了一段。

### 8.6 残差爆炸 vs 真正发散

判定 `(cur - last) / last > 1.1` 即 110% 增长 → 退出。但这个阈值经验地选,有时 LM 会先弹一下再收敛,过早退出。

可调: 把 `max_increment_sdf_residual_ratio = 1.1` 改大(如 1.5),容忍更大波动。

### 8.7 valid_pts == 0

- 第 0 行 `if valid_point_count < 10: return T_eye, None, None, None, valid_points, 0.0, 0.0` `[VERIFY: utils/tracker.py:430-432]`
- 返回 identity ΔT,主循环 `T = ΔT @ T = T`,然后立刻被安全门 2(valid < 30)判 invalid

### 8.8 cov_mat = None 后的 PGO

`cur_odom_cov = None` 传入 `add_odometry_factor` → `cov_model = self.odom_cov`(默认常数)。`use_reg_cov_mat=True` 但 cov=None 不会崩,只是降级到默认 σ。

---

## 9. 与 KISS-ICP / 传统 ICP 对比

| | KISS-ICP | PIN tracker |
|---|----------|-------------|
| 数据 | source pts + voxel cube map | source pts + neural points |
| 距离 | point-to-plane | point-to-implicit-SDF |
| 邻居 | KD-tree KNN(or fast voxel)| voxel hash + IDW |
| 解算 | small se(3) GN | small se(3) LM |
| 法线 | 现场 PCA | ∇SDF(MLP 微分)|
| 鲁棒 | thresholded TLS / Cauchy | Geman-McClure |
| 退化 | 不显式处理 | 显式特征值检查 |
| 速度 | CPU 实时(~50-100 Hz)| GPU 实时(~10-30 Hz)|

主要优势(对 PIN):**没有显式法线**,SDF 梯度作为隐式法线,在曲面 / 边缘也工作。劣势:必须有 GPU,延迟更高。

---

## 10. 论文对应

| 论文符号 | 代码 |
|----------|------|
| `T_k+1 = exp(ξ) ⊞ T_k` | `T = ΔT @ T` |
| `ξ ∈ se(3) = [ω, v]` | `t_vec[:3]` 是旋转,`t_vec[3:]` 是平移 |
| `J_i = [(T·p_i)^∧ · ∇s, ∇s]` | `J = [cross(q, ∇s), ∇s]` |
| GM kernel `ψ(r) = k²/(k² + r²)` | `((k / (k + r²))²)` |
| Hessian `H = J^T W J` | `N_mat = J.T @ (W * J)` |
| LM step `(H + λ·diag(H))Δξ = -J^T W r` | 同 |
| degeneracy `min(λ_i) < threshold` | `eigvals = eigvals(N_raw_trans); min < n × thre` |

---

## 11. 一句话总结

> **Tracker = 50 次 GN/LM 迭代,每次:点云变换 → query SDF & ∇SDF → 6-DoF Jacobian → 解正规方程 → 增量更新 T。鲁棒性靠 Geman-McClure 权重 + 4 道安全门 + 特征值退化检测。** 数学上是 point-to-plane ICP 的隐式版本,法线由 MLP 反向传播得到。

---

**END of ALGORITHM 03.**

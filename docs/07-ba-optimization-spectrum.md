# SLAM 后端优化谱系：因子图 vs 可微 BA

> 本文是一份**跨库技术脉络**梳理，不针对单个仓库，而是把 SLAM/3D 重建的「后端优化」放在一条谱系上对比：
> **纯优化（因子图）→ 可微但真解 BA → 学习模仿 BA（不解）**。
> 整理日期：2026-06-10。源于对 GTSAM / gtsam_points / PyPose / bae / DPVO / BA-T 的代码与论文阅读。
>
> 涉及的代码/论文位置：
> - GTSAM 4.3 / gtsam_points：`~/Documents/GitHub/ztpilot/finder_deps/gtsam/`（更详尽的逐行版见该目录 `GTSAM_设计说明.md`）
> - PyPose / bae：`~/Documents/GitHub/tools/pypose/`、`~/Documents/GitHub/tools/bae/`
> - DPVO：`~/vslam_ws/src/DPVO/`（本体系 [02-deep-vo.md](02-deep-vo.md) §DPVO、[dpvo/](dpvo/) 有逐行分析）
> - BA-T：论文 arXiv 2606.03287v1（2026-06-02，TU Munich/ETH，Cremers 组），**代码承诺开源于 github.com/zhangganlin/BA-T，截至本文尚未释出**

---

## 0. 一句话地图

所有这些方法都在做同一件事：**反复「算残差/对应 → 更新相机位姿 + 几何」**。区别只在「这一步更新怎么算出来」：

```
真解优化·不可微          真解优化·可微（learned前端 + 真BA solve）          不解优化·学习模仿BA结构
GTSAM / gtsam_points  →  DROID-SLAM / DPVO / bae / PyPose / Theseus    →   BA-T
（C++ 因子图，Schur补    （网络预测对应+权重，                              （Transformer 用 attention
 +Cholesky，CPU为主）     仍真解 2阶系统，梯度穿过求解器）                    模仿 Schur 补的信息流，不解系统）
```

共同祖先是 **BA-Net（2019）**——最早把可微 BA 层塞进端到端网络。DPVO 与 BA-T 都自这一脉派生。

---

## 1. 统一视角：Newton 类迭代

不管哪一派，优化内核都是：

```
非线性问题
  │ 线性化 / 算残差        ← 在当前估计点展开
  ▼
线性系统 (J/H, b)
  │ 求解（或学习预测）增量
  ▼
delta
  │ retract（流形回缩，SE3/SO3 在切空间更新）
  ▼
new estimate → 重新线性化 ...
```

- **因子图阵营**：每步是真的线性化 + 消元/Cholesky 解线性系统。
- **可微真解阵营（DPVO/bae）**：网络预测残差与权重，但仍真解 2 阶系统（Schur 补 + Cholesky），梯度反传穿过求解器。
- **学习模仿阵营（BA-T）**：连线性系统都不构造，用 attention 直接学出 delta。

---

## 2. 因子图阵营：GTSAM / gtsam_points

### 2.1 GTSAM 优化器设计（精简）

GTSAM 把问题建模成**因子图上的 MAP = 非线性最小二乘**。设计三要点：

1. **模板方法**：抽象基类 `NonlinearOptimizer`，`optimize()` 是固定外层循环，`iterate()` 纯虚交给子类。批量优化器有 4 个：`GaussNewton` / `LevenbergMarquardt` / `Dogleg` / `NonlinearConjugateGradient`，区别只在 `iterate()` 怎么算增量。
   - 代码：`nonlinear/NonlinearOptimizer.{h,cpp}`、`nonlinear/GaussNewtonOptimizer.cpp`
2. **LM 信赖域**：外层线性化一次，内层不断试探阻尼系数 `lambda`；用 modelFidelity（实际下降/预测下降）决定接受并降 lambda、或拒绝并升 lambda。
   - 代码：`nonlinear/LevenbergMarquardtOptimizer.cpp` `tryLambda()`
3. **后端可插拔**：`solve()` 按参数派发到 多波前消元(QR/Cholesky) / 顺序消元 / 迭代法(PCG/Subgraph)。直接法通过变量消元构造**贝叶斯树**求解，ordering（COLAMD）决定稀疏性。
4. **流形原生**：`retract` 让 SE(3)/SO(3) 等李群变量在切空间求增量、回缩更新。
5. **增量优化 ISAM2**：维护贝叶斯树，新因子来时只**局部重新线性化+消元**（fluid relinearization）——在线实时 SLAM 的关键，可微阵营目前没有对位物。

### 2.2 鲁棒核（M-Estimator）：两层解耦 + IRLS

- **解耦**：`mEstimator::Base`（损失 ρ / 权重 w，`linear/LossFunctions.h`）× `noiseModel::Base`（协方差），由 `noiseModel::Robust` 组合 → 任意核 × 任意噪声自由配。
- **落地**：`Robust::WhitenSystem` 先白化再 `reweight`，对 Jacobian A 和残差 b 同乘 `sqrt(weight)`（`linear/LossFunctions.cpp` `reweight`）。残差越大权重越小 → **外点自动降权**，即 IRLS。
- 内置核：Huber / Cauchy / Tukey / Welsch / GemanMcClure / DCS / Fair / L2WithDeadZone / Custom。

### 2.3 gtsam_points：GTSAM 的激光扩展

- **定位**：koide3（GLIM 作者）出品，*"GTSAM factors and optimizers for range-based SLAM"*。**不改 GTSAM 源码**，继承其 `NonlinearFactor` / `NonlinearOptimizer` 扩展。
- **因子**：点云配准类（ICP/GICP/VGICP/LOAM/连续时间 ICP/彩色 ICP）。基类 `IntegratedMatchingCostFactor`，`linearize()` **自己解析算 6×6 Hessian H 和 6×1 梯度 b，直接塞进 `gtsam::HessianFactor`**（不走自动求导）。
- **额外**：KdTree/iVox 近邻、RANSAC/GNC 全局配准、CUDA、点云分割。

### 2.4 GPU 批量线性化怎么和 LM 对接（gtsam_points 的精髓）

矛盾：GTSAM 逐因子串行 `linearize()`，单个 GPU 因子工作量小、kernel launch + 拷贝开销大 → 逐因子上 GPU 反而慢。解法「先批量预算，后逐因子取缓存」：

```
LM iterate() ─┬─ ① linearization_hook_->linearize(values)   ← 批量：所有 GPU 因子一次往返
              │      打包输入→cudaMemcpy H2D→issue 全部kernel→sync→D2H→分发缓存
              └─ ② graph_.linearize(values)                  ← GTSAM 原生串行
                     GPU 因子 linearize() 直接返回①缓存的 HessianFactor（不碰 GPU）
                     CPU 因子正常 CPU 线性化
```

- 入口：`LevenbergMarquardtOptimizerExt::iterate()` 比原版只多 `linearization_hook_->linearize()` 一行
- 批量五段式：预算缓冲 → set_linearization_point（只传 `Isometry3f` 位姿，点云/体素常驻 GPU）→ 单次 H2D → 批量 issue kernel + 统一 sync → 单次 D2H 分发
- 因子 `linearize()` 退化为读 `linearization_result` 缓存 → 包成 `HessianFactor`
- 需配 `*Ext` 优化器（`LevenbergMarquardtOptimizerExt`/`ISAM2Ext`/…），且需 `-DBUILD_WITH_CUDA=ON`

### 2.5 关键边界：适用范围 & GPU

**适用范围（视觉 vs 激光）——常见误区**：

| | 视觉 SLAM / VIO | 激光 / 点云 SLAM |
|---|---|---|
| **GTSAM** | ✅ 一流（ProjectionFactor / SmartFactor / IMU 预积分 / 多相机模型 Cal3* / StereoFactor） | ✅ 也能用 |
| **gtsam_points** | ❌ 不针对，因子全是点云配准 | ✅ 专为此设计，带 GPU |

> 「不适合视觉」的是 **gtsam_points 扩展包**，不是 **GTSAM 框架**。GTSAM 是 Kimera-VIO 等视觉 VIO 的主流后端。

**GPU 加速边界**：

| | GPU | 说明 |
|---|---|---|
| GTSAM 本身 | ❌ | 纯 CPU（Eigen/TBB/MKL），源码 0 个 `.cu` |
| gtsam_points（激光） | ✅ 有条件 | 需 `BUILD_WITH_CUDA`；仅 VGICP GPU 因子；需 `*Ext` 优化器；**只加速因子线性化，不加速线性求解（消元/Cholesky 仍 CPU）** |
| 视觉用 GTSAM | ❌ | 无现成 GPU 路径 |

GPU 只算「成千上万点的配准代价 H/b」（高度并行），位姿图本身的线性求解留在 CPU（变量少，上 GPU 收益小）。

---

## 3. 可微阵营：PyPose / bae

### 3.1 PyPose

- **定位**：*"A Library for Robot Learning with Physics-based Optimization"*，PyTorch-based、可微、把深度学习 + 物理优化结合。
- **招牌**：`LieTensor`——把李群/李代数（SO3/SE3/Sim3 + 对应代数）做成**原生可微、带 batch 维**的 tensor，并行 Jacobian。
- **优化器**：`GaussNewton` / `LevenbergMarquardt`（可微、可嵌入网络训练）。
- **机器人模块**：EKF/UKF/PF、IMU 预积分、EPnP、LQR、ICP、点云配准（PCR）等。
- **激光 SLAM 现状**：**有积木（ICP 模块 + PGO 例子），无整机**。`pypose/module/icp.py` 是基础 point-to-point ICP（KNN+SVD），`examples/module/pgo/` 用 g2o 文件演示位姿图优化（非激光端到端）。issue #151 请求过 ICP-SLAM 例子，但只落地到配准 API。要做激光 SLAM 需自己串「ICP 前端 + PGO 后端」，参考 PyICP-SLAM 或 PIN-SLAM。

### 3.2 bae（Bundle Adjustment in the Eager-mode）

- **定位**：sair-lab（Chen Wang 组），IEEE T-RO 2026，arXiv 2409.12190。PyTorch 稀疏二阶优化库，专做 **BA / PGO**，GPU 原生（cusparse/cusolver/warp、Schur 补 `bae/utils/schur.py`）。
- **与 PyPose 关系**：**bae 是 PyPose 的稀疏后端**。代码铁证（`bae/optim/optimizer.py`）：
  ```python
  from pypose.optim import LevenbergMarquardt as ppLM
  class LM(ppLM):           # 直接继承 pypose 的 LM
      def step(self, ...):   # 重写 step，用稀疏 CSR Jacobian + 自己的稀疏 solver
          J = ....to_sparse_csr()
          D = self.solver(A, -J_T @ R)   # bae 的 CUDA 稀疏 solver
  ```
  复用 pypose 的 LieTensor / LM 策略 / psjac，只替换稀疏线性代数后端，补 pypose 原生 LM 在大规模稀疏 BA 上的效率洞。
- **与 GTSAM 关系**：**无依赖**。仅 README 致谢「借鉴 GTSAM 的重投影 Jacobian 概念」+ 一个坐标系转换函数 `openGL2gtsam`（`bae/utils/ba.py`）。属同问题域不同实现的竞品。

---

## 4. 可微 vs 因子图：优劣势

| 维度 | 可微阵营（PyPose/bae、Theseus） | 因子图阵营（GTSAM/Ceres） |
|---|---|---|
| 端到端可微 / 融合深度学习 | ✅ 梯度穿过优化器，学噪声/特征/初值 | ❌ 不原生（DeepFactors 等需外挂） |
| GPU + batch 并行多问题 | ✅ LieTensor batch 原生 | ❌ 为单大问题设计 |
| 开发效率 / 自动求导 | ✅ autograd，无需手推雅可比 | ❌ 需手写解析 J/H + C++ |
| 大规模稀疏效率 | ⚠️ PyTorch 稀疏弱（bae 专补此洞） | ✅ 消元/贝叶斯树天然吃稀疏 |
| 增量优化（在线） | ❌ 基本 batch 全量重优化 | ✅ ISAM2 fluid relinearization |
| 工程成熟 / 鲁棒性 | ⚠️ 较新 | ✅ 十几年沉淀，鲁棒核/退化/约束齐全 |
| 内存 | ⚠️ autograd 计算图占用大 | ✅ 不保留计算图 |

**选型**：在线实时大规模 → 因子图；把优化嵌进网络训练 / batch / 快速原型 → 可微；大规模可微 BA（既要可微又要快）→ bae。趋势是混合：**因子图做高效后端 + 可微层做可学习前端**。

---

## 5. 可微 BA 谱系：BA-Net → DROID/DPVO → BA-T

### 5.1 DPVO（真解 BA 的代表）

- **身份**：Deep Patch Visual Odometry（Teed/Lipson/Deng, Princeton-VL, NeurIPS 2023）+ Deep Patch Visual SLAM（ECCV 2024）。纯**单目** VO（不用 IMU），是 **DROID-SLAM 的稀疏化变体**（dense flow → sparse patch，省 1/3 内存、快 3×）。
- **机制**（`~/vslam_ws/src/DPVO/dpvo/`）：
  ```
  提 patch（高梯度采样）→ patch graph
    循环:
      ① CorrBlock 算 correlation（patch 外观相关性，altcorr CUDA）
      ② Update operator（recurrent，门控残差 + 1D 时序卷积 + softmax 图聚合 + transition）
         → Factor Head 输出 Δ(2D 光流修正) + Σ(置信度)
      ③ 可微 BA（ba.py）：构 Jacobian + Schur 补 + Cholesky，跑 2 次 Gauss-Newton，
         retract 更新 pose + inverse depth，梯度反传穿过求解器
  ```
  - 关键：**真解 2 阶系统**。`ba.py:86` `BA()` 真有 Jacobian / Schur 补 / `CholeskySolver`（自定义 backward），对应代码 `for itr in range(2)`。
  - 完整在线系统：时序、patch graph、滑窗、回环（`loop_closure/`、v2 加 DBoW2 PGO）。
- 本体系细节见 [02-deep-vo.md](02-deep-vo.md)、[dpvo/](dpvo/)（逐行 + BA Schur 推导）、[08-dpvio-design.md](08-dpvio-design.md)（往 ba.py 加 IMU 因子的 DPVIO 设计）。

### 5.2 BA-T（不解 BA，Transformer 模仿）

- **身份**：BA-T: An Iterative Transformer for Two-View Bundle Adjustment（arXiv 2606.03287v1，2026-06-02，TU Munich/ETH Zurich，Cremers 组）。**代码尚未释出**。
- **动机**：前馈大模型（DUSt3R/VGGT）靠深堆叠 cross-view attention，缺迭代自纠错；经典/可微 BA（DROID/DPVO）虽迭代但「推理时要构造求逆大 Hessian」开销大、难融进前馈架构。BA-T 想**要 BA 的迭代自纠错，但不付出解线性系统的代价**。
- **方法**：把 BA 看成「camera token ↔ geometry token 反复传播信息」。一个**共享权重的轻量 Transformer 层**迭代 K 次（`ΔT=F_θ(T)`, `T←T+ΔT`），每次三步对应经典 BA：
  1. Camera-Conditioned Geometry Transform（用 camera token 经 adaLN 把 a 的几何投到 b 空间）
  2. Token-level Correspondence Matching（cross-attention 软匹配）+ Latent Residual（MLP）
  3. Camera Update（`Δc=CrossAttn(c, r)`，模仿 Schur 补里的相机更新）+ 对称 Geometry Update
  - **没有 solver**：残差、相机/几何更新全是 learned attention。
- **结果**：解码器仅 38M（DUSt3R 227M / VGGT 605M 的 5–16%），two-view 位姿最优、几何有竞争力；3–4 次迭代收敛；3D 对应误差降 44%（同尺寸 attention 迭代基线只降 27%）；低共视仍鲁棒；可扩展多视图。

### 5.3 DPVO 与 BA-T 的关系：同门血缘，路线反叛

| | DPVO | BA-T |
|---|---|---|
| 优化 | ✅ **真解 BA**（2×GN + Schur 补 + Cholesky） | ❌ **不解**，attention 学出更新 |
| 更新算子 | recurrent 门控算子 + correlation volume | Transformer（adaLN + cross-attention） |
| 几何表示 | patch + inverse depth（显式，光流空间） | latent geometry token（隐式 token 空间） |
| 残差 | 显式像素/光流空间 | latent 空间（MLP 学出） |
| 任务 | 完整在线单目 VO/SLAM | two-view 重建 refinement 层（可扩展多视图） |
| 推理代价 | 每步构造+求逆 2 阶 Hessian | 仅前向 attention，无矩阵求逆 |

- **共同祖先**：DPVO 论文 Related Work 自己点名 **BA-Net** 为「把可微 BA 层嵌进网络」的先驱；BA-T 同出此脉。
- **对立点**：BA-T 的 motivation（避开「推理时构造求逆 Hessian」）**正是冲着 DROID/DPVO 这种真解 BA 的开销去的**。

```
                    BA-Net (2019, 可微BA层入网络)  ← 共同祖先
                   ╱                            ╲
  DROID-SLAM → DPVO                          前馈大模型 (DUSt3R/VGGT)
  「真解BA：门控前端 + 2×GN + Schur补」           ╲              ╱
                                                  BA-T (2026)
                                       「不解BA：Transformer 模仿 Schur 补信息流」
```

---

## 6. 收尾索引

| 库/方法 | 阵营 | 解不解优化 | 可微 | GPU | 角色 |
|---|---|---|---|---|---|
| GTSAM | 因子图 | 真解（消元/Cholesky） | ❌ | ❌ | 通用后端（视觉/激光皆可） |
| gtsam_points | 因子图 | 真解 | ❌ | ✅ 仅因子线性化 | 激光配准扩展 |
| PyPose | 可微 | 真解（GN/LM） | ✅ | ✅ | 可微机器人优化框架 |
| bae | 可微 | 真解（稀疏 LM/Schur） | ✅ | ✅ 原生 | PyPose 的稀疏 BA/PGO 后端 |
| DROID-SLAM / DPVO | 可微 | 真解（2×GN+Schur） | ✅ | ✅ | learned 前端 + 可微 BA 的在线 VO/SLAM |
| BA-T | 学习模仿 | **不解** | ✅ | ✅ | BA 启发的 two-view refinement Transformer |

> **总纲**：从「纯优化」到「学习模仿」是一条连续谱。GTSAM/gtsam_points 在最优化端（稳、快、增量、但不可微）；BA-T 在最学习端（轻、可微、前馈友好、但不真解）；DPVO/bae/PyPose 在中间（可微又真解）。选型本质是在「工程成熟度/实时性」与「可学习性/与网络融合」之间取舍。

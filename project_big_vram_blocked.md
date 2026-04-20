---
name: 本机(RTX 4060 8GB)VRAM 阻塞仓库清单
description: vslam_ws 19 个仓库里哪些在 4060 Laptop 8GB 上跑不动 — 约 40% 必须转大显存机器才能继续
type: project
originSessionId: a0afd5e9-217a-4b94-886c-30ecb9a94586
---
**背景**:steve-Zenbook RTX 4060 Laptop **8GB** 显存 + Xorg 占 ~3GB = 可用 ~5GB。19 个仓库里差不多一半的 GPU-bound 路线在本机上没法做到 paper-level 质量,必须转大显存机器(**24GB(4090/3090/A5000)起步;A100 40/80GB 或 H100 80GB 最稳**)。

## 分类

### 🔴 明确需要大显存(empirical 验证 或 upstream 明确要求 24GB+)
- **MapAnything**(ViT-g14,1B 参数):4060 实测上限 ~30 views;long trajectory pose 24m ATE,MVS 模式喂 pose 仍散乱。详见 `project_mapanything_geoscan.md`。
- **Gaussian-LIC**(3DGS LIC SLAM, ZJU):upstream 3090/4090 测过,还有 6 个 serial blocker(ROS1 + CUDA 11.7 硬编码 + SPNet 权重 GFW + Coco-LIC 依赖 + 无 fisheye)。详见 `project_gaussian_lic_blockers.md`。
- **GS_ICP_SLAM**(3DGS + Fast-GICP 双线程):**已经 `COLCON_IGNORE`**;3DGS 在线建图都是 16-24GB 量级。
- **SGS-SLAM**(SplaTAM 衍生 + 语义):3DGS 家族,同量级 VRAM。
- **Mobile-GS**(Mini-Splatting + GPCC 离线压缩):3DGS 训练 24GB 起,4060 只能玩小规模。
- **VINGS-Mono**(GTSAM + 3DGS 离线):3DGS 家族 + 大参数 PGO。
- **VGGT-SLAM**(VGGT 1B 前馈 + SL(4) 因子图):ViT-Large 骨干比 MapAnything 小 ~2.7×,但多视 attention 仍贵,估计 16GB 线。
- **DROID-W / droid_w_ros2**(DROID-SLAM 批处理):recurrent correlation volume 记忆重,全序列跑 16-24GB。

### 🟡 可能能跑但显存紧(未 empirical 验证)
- **limap**(线特征 SfM,C++ 优化):主要 CPU bound,但 DeepLSD 提线段 + GlueStick 匹配要 GPU。本机已跑通(`06-benchmarks.md` 2026-04-15/16 条目),算不在阻塞名单。
- **cuvslam_ros**(NVIDIA SDK):显存需求低(几百 MB),但未在 4060 测过,算灰色。

### 🟢 本机完全 OK(已 empirical 跑通)
- **ORB_SLAM3_ROS2** / **AirSLAM** / **DPVO**(DPVO 实测 0.60m ATE,120+Hz):传统/轻量 DL SLAM,几 GB 够。
- **open_vins / sqrtVINS / mins / EPLF-VINS / VINS-Fusion**:纯 VIO,基本不用 GPU。
- **slim-vdb**:OpenVDB CUDA 体素化,本机跑通(296 万点云,见 `project_slimvdb_tuning.md`)。
- **rgbdslam_v2**:classic RGB-D PGO,无大 GPU 需求。

## 行动准则(How to apply)

1. **用户要"在 GeoScan / EuRoC 跑 [仓库 X]"** → 先查仓库在上面哪一类:
   - 🔴 名单里 → **直接说本机跑不了,建议转大显存机器**,不要盲目开 build。只在必要时上 memory 找该仓库具体 blocker 文件。
   - 🟡 未验证 → 可以小试,但设好 OOM 兜底(`torch.cuda.max_memory_allocated` 监控,前 50 帧就能判。
   - 🟢 已验证 → 直接继续。

2. **目标机器候选**(够做 MapAnything full stride=10 / Gaussian-LIC / 3DGS 家族的):
   - RTX 4090 / 3090 / A5000 24GB — 能做 Gaussian-LIC / GS_ICP_SLAM / MapAnything ViT-L 配置
   - A100 40GB — 能做 MapAnything stride=5 (~800 views) / 完整 3DGS 训练
   - A100 80GB / H100 80GB — 能做 MapAnything stride=2 (~2000 views) 极限配置

3. **短期(继续 4060)** 明确聚焦:
   - 传统 SLAM 比较(open_vins / VINS-Fusion / cuvslam,填 benchmark 矩阵)
   - AirSLAM / ORB_SLAM3 / DPVO 互相对齐 APE 做严格横比
   - slim-vdb 稠密重建收尾
   - **gsplat offline**:`pip install gsplat==1.3.0` + ORB_SLAM3 KF 轨迹 → COLMAP sparse 转换 → simple_trainer.py 训练。gsplat 小场景 4GB 能跑,不在阻塞名单,是当前最值得推的"看起来像 3DGS"路线。
   - 完全不要碰 🔴 名单里的仓库,直到有大显存机器。

4. **脚本复用**:前馈家族(MapAnything / VGGT / Pi3-X)的 extract/infer 脚本已写好(`src/vslam_experiment/scripts/mapanything/`),上大显存机器后可直接改骨干复用 — 要去掉 bf16 monkey-patch 链(大显存用 fp32 直接跑更稳),换 uv venv。

## 为什么这么多仓库都卡显存

**20-23 年这波 3DGS + feedforward 3D(DUSt3R/MASt3R/VGGT/MapAnything/Pi3)** 核心路线是"堆参数 + 堆 views",显存爆炸是设计取舍不是 bug。上游作者实验室标配 A100/H100,paper 报的 "20 GB VRAM" 是给后人看的下限。走这条技术路线就绕不开大显存。**传统几何 SLAM 家族(ORB/VINS/OpenVINS)和 OpenVDB 路线反而才是本机友好的**。

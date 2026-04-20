---
name: DGX Spark 迁移与适配清单
description: 把 vslam_ws 里 🟦 标记的仓库搬到 DGX Spark(GB10, 128GB unified mem, ARM64, Blackwell)时的预判坑和步骤 — 用户会在首次部署时回填实测
type: project
originSessionId: a0afd5e9-217a-4b94-886c-30ecb9a94586
---
**硬件**:NVIDIA DGX Spark(GB10 Grace-Blackwell,20C ARM64 + Blackwell GPU,**128GB LPDDR5x unified memory**,带宽 ~273 GB/s,FP16 ~500 TFLOPS,FP4 1 PFLOPS)。出厂 Ubuntu 24.04 + ROS Jazzy friendly。

## 目标仓库(benchmark 矩阵 🟦 标注的)
- **完全解决**:MapAnything, VGGT-SLAM, GS_ICP_SLAM, SGS-SLAM, Mobile-GS, VINGS-Mono, DROID-W (DROID-SLAM batch)
- **部分解决**:Gaussian-LIC(硬件够,但 ROS1+CUDA 11.7 硬编码+Coco-LIC+SPNet GFW 仍在,要 fork 上游适配,预计数周)

## 迁移原则(别把 4060 的 workspace 直接镜像搬过去)

1. **新开 `~/vslam_ws_spark/`** 独立 workspace,不共用现有 `~/vslam_ws/`。系统 ROS2 版本、Python/PyTorch 栈都不一样,混在一起会互相污染。
2. **数据集用 sshfs 或 rsync 挂过去**:`~/Documents/Datasets/geoscan/` 照搬,标定 yaml / bag 不动。
3. **Git clone 仓库重做 `rosdep install --from-paths src -y --ignore-src`**,拿到 ARM64 / Jazzy 的依赖图。

## 预判坑(按优先级)

### A. Blackwell 的 PyTorch/CUDA 版本硬下限
- Blackwell(sm_100)最低 **CUDA 12.8 + PyTorch 2.6(nightly)** 才认架构。上游仓库 pip 直接装的 torch 2.6.0+cu124 会 `no kernel image available for sm_100` 报错。
- 固定做法:`pip install --index-url https://download.pytorch.org/whl/cu128 torch torchvision torchaudio`(用 cu128 或更新的)。
- uv 用 `UV_TORCH_BACKEND=cu128 uv pip install torch`(如果 uv 版本支持的话)。

### B. ARM64 wheel 缺失
- 大包(torch / opencv / numpy / scipy / pytorch3d)都有 manylinux_aarch64 wheel。
- 易坑的:`uniception==0.1.7`(MapAnything 依赖)、`bitsandbytes`、`flash-attn`、`pycolmap`、`tensorrt`(TRT 10+ for Blackwell,老版不行)。
- 遇到就 `pip install --no-binary <pkg>` 从源码编,或找 nvidia-pyindex 的 arm64 预编译。

### C. 硬编码 x86 路径
- `Gaussian-LIC/CMakeLists.txt:36` 写死 `/usr/local/cuda-11.7`,改成 `$ENV{CUDA_HOME}` 或系统默认。
- LibTorch 路径同理:`~/Software/libtorch/share/cmake/Torch` 改成 `$(python -c 'import torch; print(torch.utils.cmake_prefix_path)')`。

### D. HF / Google Drive 下载(GFW 适用性)
- Spark 通常在国内用也需要 `HF_ENDPOINT=https://hf-mirror.com`(和本机一样)。
- Google Drive 权重(Gaussian-LIC SPNet `Large_300.pth`, 4.6GB)在国内很难直连,考虑本机下完用 rsync 或 U 盘转过去。

### E. ROS 版本
- Spark 默认 Ubuntu 24.04 → ROS Jazzy。本 workspace 是 Humble。`ORB_SLAM3_ROS2` / `AirSLAM` / `VINS-Fusion-ROS2` 这些 Humble 原生的**可能要改 package.xml depend 版本**。幸好 Jazzy API 基本兼容 Humble。
- **ROS1 仓库(mins / EPLF-VINS / Gaussian-LIC / Coco-LIC)** 在 Jazzy/Noble 上没有官方 apt,基本等于放弃;或者 docker ROS Noetic on ARM64(也勉强,没官方 arm64 镜像)。

### F. Benchmark 命令不动
- 现有脚本(`src/vslam_experiment/scripts/mapanything/{extract_cam0_undist.py, run_mapanything_geoscan.py}`)逻辑不变,但要:
  - **删 bf16 monkey-patch 链**(128GB 随便用 fp32,bf16 补丁反而会把 ray_directions / Linear / rgb / numpy 转 fp32 的那堆破坏掉)。保留一份"4060 版本"(gitignore 一个 tag 或者 `branch 4060-bf16`)。
  - stride=10 可行甚至 stride=5(~850 views)/ stride=2(~2000 views)都塞得下。
  - `memory_efficient_inference=False` 跑最快。

### G. 带宽是主要瓶颈,不是显存
- LPDDR5x 273 GB/s 比 A100 HBM2e(2 TB/s)慢 ~7×。**3DGS 训练 rasterization / DROID-SLAM DBA 这种 bandwidth-bound 的 kernel 会慢 3-5×**,但"能跑 vs 不能跑"才是第一顺位。
- 有时间就上 rerun / tensorboard profiler,量一下 kernel time,别盲目归因 GPU 慢。

## 首次部署回填模板(跑完后来更新本文件)

待回填:
- [ ] 装了哪个 Ubuntu 版本 + CUDA 版本 + PyTorch 版本
- [ ] uv / conda 的实际环境路径
- [ ] 最先测的一个 🟦 仓库是啥,实际显存峰值、速度
- [ ] Jazzy vs Humble 迁移上哪些 package.xml 要改
- [ ] ARM64 缺失 wheel 清单(装不上的包 → 解决方案)
- [ ] ROS1 仓库(mins / EPLF-VINS / Gaussian-LIC)最终怎么处理(docker / skip / 其他)
- [ ] MapAnything full stride=10 跑完的 ATE、推理时间、GLB 质量

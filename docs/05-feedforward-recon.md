# 前馈式 3D 重建 / SLAM

> 迁自 `src/vslam_summary_v2.md`（2026-04-15）。本文档覆盖 **无 ROS、GPU 驱动、离线或半在线**的新一代重建范式——和 `01` / `02` / `03` / `04` 里的算法**不在同一层对比**。

本目录包含 3 个库：

| 库 | 仓库位置 | 是否在本工作区 | 类型 |
|----|----------|----------------|------|
| **LIMAP** | `src/limap/` | ✅ | 线特征 SfM 模块（C++ + Python） |
| **MapAnything** | 通过 `pip install` + HuggingFace | ❌（纯 pip） | 一次性多任务前馈求解器 |
| **VGGT-SLAM** | `src/VGGT-SLAM/` | ✅ | 前馈 VGGT + 因子图后端 SLAM |

**一句话差异**：
- **LIMAP** = 传统几何路线（C++ 优化 + 线特征），**模块**。
- **MapAnything** = 单一大 Transformer，**模型**。
- **VGGT-SLAM** = 把 VGGT 这个"模型"包进**系统**（frontend + backend + loop closure）。

---

## 1. LIMAP

### 1.1 简介
ETH CVG，《3D Line Mapping Revisited》，**CVPR 2023 Highlight**（`cvg/limap`）。把线特征提升为 first-class citizen 的 SfM 模块。传统 SfM 几乎只用点特征，在弱纹理 / 重复结构（建筑立面、室内）上失败；LIMAP 在已有点 SfM（COLMAP / Bundler / VSfM）基础上融合 2D 线检测、匹配、三角化、验证与融合，构建 **3D 线地图**。

### 1.2 核心架构
- 2D 线检测：**LSD** / **DeepLSD**（深度学习）/ LineTR
- 2D 线匹配：**SOLD2** / **GlueStick** / Endpoints NN
- VP（vanishing point）估计：JLinkage / Progressive-X / DeepLSD
- 三角化 + 多视验证 + 融合为 3D track
- C++ 后端（libigl、ceres、colmap、RansacLib、pybind11）
- Python 前端（hloc、pycolmap、deeplsd、gluestick、pytlsd）
- 提供点-线混合定位（hloc 接口）

### 1.3 仓库结构
```
limap/
├── src/        # C++ 后端（triangulation / fitting / optimization）
├── limap/      # Python 包（cfgs loader, runners 接口）
├── runners/    # 每个数据集 / pipeline 的入口脚本
├── cfgs/       # YAML 配置（detector / matcher / optimizer 组合）
├── cmake/      # FindCeres / FindCOLMAP
└── docker/     # Dockerfile.deps + Dockerfile
```

### 1.4 工厂模式：detector / extractor / matcher
```python
# limap/line2d/__init__.py 风格
def get_detector(cfg):
    # 支持: lsd / deeplsd / linetr
def get_extractor(cfg):
    # 支持: sold2 / lbd / wireframe
def get_matcher(cfg):
    # 支持: gluestick / sold2 / endpoints_nn
```

### 1.5 典型配置（`cfgs/triangulation/default.yaml` 风格）
```yaml
line2d:
  detector: {method: "deeplsd", skip_exists: False}
  extractor: {method: "wireframe", skip_exists: False}
  matcher:  {method: "gluestick", topk: 10}
triangulation:
  use_vp: True                         # 用 vanishing point 先验
  use_endpoints_triangulation: False
  min_length_2d: 20.0
```

### 1.6 典型 pipeline
```python
# runners/colmap_triangulation.py 风格
colmap_model = pycolmap.Reconstruction(colmap_dir)       # 先跑点 SfM
detector = create_line_detector(cfg['line_detector'])     # DeepLSD / LSD / LineTR
matcher  = create_line_matcher(cfg['line_matcher'])       # GlueStick / SOLD2
lines2d  = detector.detect_all(images)
matches  = matcher.match_all(lines2d, neighbors)
tracks   = triangulate_and_fuse(lines2d, matches, poses)  # C++ 后端
optimize_tracks(tracks, colmap_model)                     # ceres 精化
```

### 1.7 输入输出
| 维度 | 值 |
|------|------|
| 图像 | 多视 RGB |
| 时序 | 无要求（但要先有 SfM） |
| 位姿 | **必须**（从 COLMAP 读） |
| 内参 K | **必须** |
| IMU | — |
| 输出 | 3D 线段地图（track） |
| 尺度 | SfM 相对尺度 |

### 1.8 后端 & 回环
Ceres BA（**点-线联合**），SE(3) 位姿 + 3D 线参数化。没有回环——"回环"实际在上游 COLMAP 阶段就做掉了，LIMAP 只管线层融合。

### 1.9 运行入口
```bash
python runners/colmap_triangulation.py -c cfgs/triangulation/default.yaml
# CI 测试（必须限定 tests/，避免扫到 src/third-party/*/tests/）
pytest tests/ -m ci_workflow -v
```

### 1.10 Docker 工作流（国内推荐）
本工作区已记录过的做法（见 auto-memory `project_limap_docker.md`）：

**两份 Dockerfile**（`docker/`）：
- `Dockerfile.deps` —— **推荐**，只装依赖，源码 `-v` 挂载到容器现场编译，镜像可跨机器复用。
- `Dockerfile` —— 上游原版，一次成型。

**依赖镜像构建**（国内用 tuna 源）：
```bash
cd src/limap
docker build -f docker/Dockerfile.deps -t limap-deps:latest .
# Ubuntu 22.04 + CUDA 11.8 + Python 3.10 venv + torch + COLMAP 全依赖，约 16.5 GB
```

**源码挂载编译**（必须透传代理拉 pytlsd / DeepLSD / GlueStick / hloc）：
```bash
PROXY=http://127.0.0.1:7897
docker run -d --name limap-build --gpus all --network host --shm-size 8G \
    -e http_proxy=$PROXY -e https_proxy=$PROXY \
    -e no_proxy=localhost,127.0.0.1,pypi.tuna.tsinghua.edu.cn \
    -v $(pwd):/limap -w /limap \
    limap-deps:latest bash -lc '
      pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple &&
      pip install -r requirements.txt &&
      pip install -Ive .
    '
docker logs -f limap-build        # 首次 15–30 min
docker commit limap-build limap:latest   # 必须固化（否则 editable 元数据丢失）
docker rm limap-build
```

### 1.11 踩坑记录
| 问题 | 现象 | 根因 / 解决 |
|------|------|-------------|
| `docker --gpus all` 报 `failed to discover GPU vendor from CDI` | 容器无 GPU | 没装 NVIDIA Container Toolkit；用 `mirrors.ustc.edu.cn/libnvidia-container` 装 |
| 容器内 `git clone github.com ...` 超时 | pip install 中断 | 忘加 `--network host` + `http_proxy/https_proxy` |
| `pytest` 报 `No module named romatch` / `pybind11_tests` | 扫到非 CI 项 | 必须 `pytest tests/ -m ci_workflow`，避开 `scripts/` 和 `src/third-party/*/tests/` |
| apt 装 `nvidia-container-toolkit` tuna 404 | — | tuna 不一定有此仓库，改用 `mirrors.ustc.edu.cn` |
| torch 从 `download.pytorch.org` 下载龟速 | — | 改走 pypi 清华源，不要 pin `--index-url https://download.pytorch.org/...` |
| `pip install -Ive .` 卡住 | 编译 C++ 后端 | 正常，15–30 min；`docker logs -f limap-build` 跟进度 |

### 1.12 优缺点
| ✅ | ❌ |
|----|----|
| 几何解释强，线贴合建筑结构 | 必须依赖点 SfM 先验 |
| C++ 后端速度快 | 编译依赖复杂 |
| 点-线混合定位（hloc 接口） | 离线为主 |

### 1.13 代码入口
| 路径 | 内容 |
|------|------|
| `src/limap/runners/<dataset>/<task>.py` | 入口脚本 |
| `src/limap/cfgs/triangulation/*.yaml` | triangulation 配置 |
| `src/limap/cfgs/localization/*.yaml` | 定位配置 |
| `src/limap/limap/line2d/__init__.py` | 检测器 / 匹配器工厂 |
| `src/limap/src/` | C++ 后端 |
| `src/limap/docker/` | Dockerfile 与 README |

---

## 2. MapAnything

### 2.1 简介
《MapAnything: Universal Feed-Forward Metric 3D Reconstruction》（arXiv 2509.13414，2026，Meta + CMU）。**一个 Transformer 模型 → 12+ 种 3D 重建任务**（multi-view SfM / MVS、monocular metric depth、depth completion、registration 等）。直接回归"分解的度量 3D 几何"（depth along ray + ray direction + pose + metric scale），一次前馈输出所有几何量 + 置信度。

仓库：`facebookresearch/map-anything`；主页：map-anything.github.io。

> **本工作区不存源码**，通过 `pip install -e .` + HuggingFace 按需拉权重使用。

### 2.2 核心架构
```
Input: [N 张图, optional(K_i, pose_i, depth_i)]
        │
        ├── Image tokenizer（逐视图）
        ├── Cross-view attention blocks  ← 处理任意视图数
        └── Heads:
              ├── ray_direction       # 单位射线方向
              ├── depth_along_ray     # 沿射线深度
              ├── metric_scale        # 度量尺度因子
              ├── pose (cam2world)    # 每视图外参
              ├── K (pinhole)         # 每视图内参
              └── confidence          # per-pixel 置信度
```

**分解的度量 3D 几何** —— 不是直接回归 xyz，而是分别回归方向 + 沿射线深度，这样可共享 pipeline 解决 depth completion / pose estimation 等子任务。

### 2.3 特点
- 统一 Transformer（modular UniCeption 组件）
- WAI 统一数据格式（训练用）
- HuggingFace 分发：`facebook/map-anything`（主）/ `facebook/map-anything-apache`
- 2000 张视图可在 140 GB GPU 上 memory-efficient 推理
- 权重**双 license**：主模型 CC-BY-NC，`-apache` 变体 Apache 2.0
- 提供 Gradio / Rerun 交互 demo，支持导出 COLMAP 格式与 3DGS 集成

### 2.4 输入输出
| 维度 | 值 |
|------|------|
| 图像 | 任意 RGB（1 ~ 2000 张） |
| 时序 | 无要求（任意顺序） |
| 位姿 | 可选 |
| 内参 K | 可选 |
| 深度 | 可选（部分视图） |
| 输出 | 世界 / 相机系 3D 点云 + depth + K + pose + confidence |
| 尺度 | **度量尺度**（metric scaling factor 自动估计） |

### 2.5 典型 pipeline
```python
from map_anything import MapAnythingModel
model = MapAnythingModel.from_pretrained("facebook/map-anything-apache")
out = model(images, intrinsics=None, poses=None)           # 一次前馈
pts3d, depth, K, pose, conf = out['pts3d'], out['depth'], \
                              out['K'], out['pose'], out['conf']
```

### 2.6 后端 & 回环
**无**。一次性求解器，无时序 / 无回环 / 无增量。

### 2.7 部署
纯 Python 包：
```bash
pip install -e .                  # 数分钟
pip install -e .[all]             # 额外拉 8 个模型（VGGT/DUSt3R/MASt3R/MUSt3R/Pi3/Pow3r/DA3/rmvd）做 benchmark
```
Docker 策略：可直接复用 `limap-deps:latest` 底座（Python 3.10 + CUDA 11.8）；挂 `~/.cache/huggingface`，设 `HF_ENDPOINT=https://hf-mirror.com`。

### 2.8 优缺点
| ✅ | ❌ |
|----|----|
| 零配置：丢图即出结果 | 一次求解，无时序 / 无回环 / 无增量 |
| 度量尺度自动估计 | 大批量图推理显存需求高 |
| 双 license 兼顾学界 / 业界 | 主模型 CC-BY-NC，商用要用 `-apache` 变体 |

### 2.9 踩坑记录（TBD）
> 本工作区尚未实际集成 MapAnything，留给后续跑 benchmark 时回填。

---

## 3. VGGT-SLAM

### 3.1 简介
MIT-SPARK，《VGGT-SLAM: Dense RGB SLAM Optimized on the SL(4) Manifold》：
- **v1**：arXiv 2505.12549，NeurIPS 2025
- **v2**：arXiv 2601.19887，2026.01，**实时**（Jetson Thor SM_121）

仓库：`MIT-SPARK/VGGT-SLAM`。把 **VGGT**（前馈稠密 3D 重建 Transformer）套进经典 SLAM 框架。

### 3.2 核心架构
- **Frontend**：每 N 帧一个 submap，用 VGGT 前馈推断 submap 内所有几何（depth / K / pose / pointmap）
- **Backend**：submap 间在 **SL(4) 流形**上做 **15-DoF 单应**对齐——解决未标定单目场景下无法用简单相似变换配准的歧义问题——通过 **GTSAM** 做因子图 + 回环约束
- **Loop closure**：v2 复用 VGGT 自身某层 attention 做**免训练**的图像检索与回环验证

### 3.3 VGGT 基础模型
> **Visual Geometry Grounded Transformer** · Oxford VGG + Meta · CVPR 2025 Best Paper
> 仓库：`facebookresearch/vggt`

- 输入：1 张 ~ 数百张图像（无需顺序 / 位姿 / 内参）
- 输出（多头）：相机 K + 外参、深度图、per-pixel point map、3D point tracks
- 规模：1B 参数；`VGGT-1B`（非商用）/ `VGGT-1B-Commercial`（需申请）
- 范式：**DUSt3R / MASt3R 的下一代** —— 不再成对配对再融合，而是任意多图一次处理
- 深度 unproject 通常比 point map 头更准
- Ampere（SM 8.0+）用 bfloat16，更低用 fp16

### 3.4 仓库结构
```
VGGT-SLAM/
├── main.py             # 入口（--image_folder / --max_loops / --vis_map）
├── vggt_slam/          # SLAM 核心（frontend / backend / loop）
├── third_party/        # setup.sh 会 clone 到这里
│   ├── vggt/           # MIT-SPARK/VGGT_SPARK fork
│   ├── salad/          # 回环检索（Dominic101/salad）
│   ├── perception_models/  # 可选 3D 开放集检测
│   └── sam3/           # 可选 SAM3
├── docker/             # Dockerfile.deps（Python 3.11 + CUDA 12.1）
├── setup.sh            # 自动 clone + pip install -e
└── setup_docker.sh     # Docker 专用：跳过已存在 third_party
```

### 3.5 前端（submap builder）
```python
# vggt_slam/frontend.py 风格
class SubmapBuilder:
    def __init__(self, vggt_model, submap_size=N):
        self.vggt = vggt_model
    def build(self, frames):
        # 每 N 帧丢给 VGGT 出一个子地图
        return self.vggt(frames)   # depth / K / pose / pointmap
```

### 3.6 后端（SL(4) 因子图）
两个**未标定** submap 之间的对齐 **不能**用 SE(3) 或 Sim(3)：缺少内参 → 尺度 + 透视歧义。VGGT-SLAM 用 **15-DoF 单应 H ∈ SL(4)** 表达 submap 变换，在流形上优化（GTSAM 2025.08 合入 SL(4) 类型）。因子图 = submap 节点 + 相邻帧 H 约束 + 回环 H 约束。

### 3.7 v1 vs v2 对比
| | v1（NeurIPS 2025） | v2（2026） |
|---|---|---|
| 运行方式 | 离线 / 近实时 | **实时**（Jetson Thor） |
| 因子图 | SL(4) 15-DoF 单应 | 重设计：消除 15-DoF 高维漂移 & 平面退化 |
| 回环 | 标准检索 | **注意力层免训练检索 + 验证** |
| TUM 位姿误差 | baseline | 比 v1 ↓ ~23% |
| 额外能力 | — | 开放集 3D 物体检测（SAM 3 + Perception Encoder） |

### 3.8 输入输出
| 维度 | 值 |
|------|------|
| 图像 | **有序** RGB 视频流 |
| 时序 | **必须**（顺序帧成 submap） |
| 位姿 | 不需要 |
| 内参 K | **不需要**（未标定） |
| IMU | — |
| 输出 | 稠密点云地图 + 相机轨迹 + submap 位姿 |
| 尺度 | 相对（未标定） |

### 3.9 Docker 工作流（国内推荐）
**`docker/Dockerfile.deps` 关键差异**：CUDA 12.1 + Python 3.11（因为依赖 `gtsam-develop` + PyTorch 2.3.1），**不能复用 `limap-deps:latest` 底座**。

```bash
cd src/VGGT-SLAM
docker build -f docker/Dockerfile.deps -t vggt-slam-deps:latest .
```

**`setup_docker.sh` 替代原生 `setup.sh`**（避免 Docker 场景下重下 third_party）：
```bash
clone_if_missing https://github.com/Dominic101/salad.git                    third_party/salad
clone_if_missing https://github.com/MIT-SPARK/VGGT_SPARK.git                third_party/vggt
clone_if_missing https://github.com/facebookresearch/perception_models.git  third_party/perception_models
clone_if_missing https://github.com/facebookresearch/sam3.git               third_party/sam3
pip install -e ./salad ./vggt ./perception_models ./sam3
pip install -e .
```

**在容器内跑**：
```bash
docker run -d --name vggt-slam-build --gpus all --network host --shm-size 16G \
    -e http_proxy=$PROXY -e https_proxy=$PROXY \
    -e HF_ENDPOINT=https://hf-mirror.com \
    -v $(pwd):/ws -v ~/.cache/huggingface:/hf-cache -e HF_HOME=/hf-cache \
    -w /ws vggt-slam-deps:latest bash setup_docker.sh
docker commit vggt-slam-build vggt-slam:latest
```

### 3.10 运行入口
```bash
python main.py --image_folder /data/office_loop --max_loops 1 --vis_map
```

### 3.11 踩坑记录
| 问题 | 现象 | 根因 / 解决 |
|------|------|-------------|
| `setup.sh` 每次重下 third_party | 浪费带宽 | 改用 `setup_docker.sh`（`clone_if_missing` 检查目录非空跳过） |
| VGGT 权重下载慢 | `from_pretrained("facebook/VGGT-1B")` 挂起 | 设 `HF_ENDPOINT=https://hf-mirror.com`，挂 `~/.cache/huggingface` |
| GTSAM import 失败 | `ImportError: gtsam ...` | 需要 `gtsam-develop`（2025.08+ 含 SL(4)），不是 apt 版 |
| Python 3.11 不兼容 3.10 docker | — | 必须独立镜像 `vggt-slam-deps:latest`，不能共用 `limap-deps` |

### 3.12 优缺点
| ✅ | ❌ |
|----|----|
| 未标定单目 + 实时（v2） | 必须 GPU，Jetson Thor 级才能实时 |
| 免训练回环（利用 VGGT 自身 attention） | 依赖 `gtsam-develop`，首次构建 20+ min |
| 稠密地图（前馈直接出 point map） | SL(4) 流形优化门槛高 |

### 3.13 代码入口
| 路径 | 内容 |
|------|------|
| `src/VGGT-SLAM/main.py` | 入口 |
| `src/VGGT-SLAM/vggt_slam/frontend.py` | submap builder |
| `src/VGGT-SLAM/vggt_slam/backend.py` | SL(4) 因子图 |
| `src/VGGT-SLAM/vggt_slam/loop.py` | 回环检测 |
| `src/VGGT-SLAM/third_party/{vggt,salad,perception_models,sam3}` | 依赖 clone |
| `src/VGGT-SLAM/docker/` | Dockerfile.deps |
| `src/VGGT-SLAM/setup_docker.sh` | Docker 友好 setup |

---

## 横向对比

| 维度 | LIMAP | MapAnything | VGGT-SLAM v2 |
|------|-------|-------------|--------------|
| 定位 | 线特征 SfM / 建图 | 统一前馈多任务 3D 重建 | 前馈稠密单目 SLAM |
| 发表 | ETH CVG, CVPR 2023 | Meta + CMU, 2026 | MIT-SPARK, NeurIPS 2025 / 2026 |
| 增量 / 实时 | 否（离线） | 否（批处理） | **是**（Jetson Thor 实时） |
| 回环 | — | 无 | **有**（SL(4) PGO + attention 回环验证） |
| 范式 | 传统几何 + C++ 优化 | 单一 Transformer | VGGT 前馈 + 因子图后端 |
| GPU | 推荐（深度 detector/matcher） | 必须 | 必须 |
| 构建 | C++ (pybind) + Python，编译麻烦 | 纯 Python `pip install -e .` | Python + GTSAM (C++)，`setup.sh` |
| ROS 依赖 | 否 | 否 | 否 |
| 本工作区 | `src/limap/` | 无（pip） | `src/VGGT-SLAM/` |
| 典型场景 | 建筑 / 室内建模、线约束 SfM | 离线多任务重建 benchmark | 机器人 / AR 在线建图 |

### 国内环境通用踩坑（三者共用）
1. **都要翻墙**——至少 HuggingFace、GitHub、torch 的部分 wheel。
2. **推荐 Docker 方案**：
   - `limap-deps:latest` 可同时作为 MapAnything 底座（Python 3.10 + torch + CUDA 11.8）
   - VGGT-SLAM 用 Python 3.11 + CUDA 12.1，**独立镜像** `vggt-slam-deps:latest`
3. **HF 缓存挂载**：`-v ~/.cache/huggingface:/hf-cache -e HF_HOME=/hf-cache`
4. **代理透传**：`--network host -e http_proxy=... -e https_proxy=...`
5. **HF 国内镜像**：`-e HF_ENDPOINT=https://hf-mirror.com`
6. **LIMAP 必须 `docker commit`**：容器退出后要固化，否则 editable 安装的 `.egg-info` 元数据丢失
7. **VGGT-SLAM 的 `setup.sh → setup_docker.sh`**：原脚本每次都 `git clone`，Docker 场景必须跳过已存在目录
8. **VGGT-SLAM 的 `gtsam-develop`**：SL(4) 支持 2025.08 合入官方仓库，首次构建预留 20+ min

### 选型决策
| 需求 | 首选 |
|------|------|
| 建筑 / 室内，希望地图里有"结构线" | **LIMAP** |
| 离线多视图，想一次拿全度量几何（depth / K / pose） | **MapAnything** |
| 机器人 / 手持视频在线建图 + 闭环 | **VGGT-SLAM v2** |
| 只想试试前馈稠密重建啥感觉 | **VGGT**（不用 SLAM 包装） |

### 镜像 / 缓存路径速查
```
Docker 镜像:
  limap-deps:latest        —— LIMAP / MapAnything 共用底座（~16.5 GB）
  limap:latest             —— limap-deps + 编译产物（docker commit 固化）
  vggt-slam-deps:latest    —— VGGT-SLAM 底座（Py3.11 + CUDA 12.1）
  vggt-slam:latest         —— + setup_docker.sh 产物

HF / 模型缓存:
  ~/.cache/huggingface/    —— 挂载: -v ~/.cache/huggingface:/hf-cache -e HF_HOME=/hf-cache
  ~/.cache/torch/          —— hloc / torch hub 权重

环境变量:
  HTTP(S)_PROXY=http://127.0.0.1:7897
  HF_ENDPOINT=https://hf-mirror.com
  PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple
```

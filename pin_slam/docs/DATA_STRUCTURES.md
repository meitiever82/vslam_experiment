# PIN-SLAM 数据结构详解

> 对应 Phase 2。所有路径相对 `src/PIN_SLAM/`。
> 命名规则: 字段 ▸ 类型/形状 ▸ 含义 ▸ `[VERIFY:]`。

---

## 1. 顶层关系图

```
┌──────────────────────────────────────────────────────────────────┐
│ Config (utils/config.py)                                         │
│   - 200+ 个超参                                                   │
│   - YAML loader,所有模块通过它共享配置                               │
└──────────────────────────────────────────────────────────────────┘
        │ (引用)
        ▼
┌────────────────┐   ┌──────────────────┐   ┌────────────────────┐
│ SLAMDataset    │   │ NeuralPoints     │   │ Decoder × 3        │
│ (1314 lines)   │   │ (1144 lines)     │   │ (geo / sem / color)│
│  ┌────────────┐│   │  全局张量集 +      │   │  nn.Module         │
│  │ 当前帧缓存  ││    │  局部张量集 +     │   │  layers + lout     │
│  │ 位姿数组×2  ││   │  哈希表 +         │   │  sdf_scale         │
│  │ 时间表      ││   │  邻域偏移         │   │                    │
│  └────────────┘│   │  certainty       │   │                    │
└────────────────┘   └──────────────────┘   └────────────────────┘
        │                    │                       │
        └───────┬────────────┴──────────┬────────────┘
                ▼                       ▼
        ┌────────────┐        ┌──────────────────────┐
        │ Tracker    │        │ Mapper               │
        │ (810 ln)   │        │ (1079 ln)            │
        │  read-only │        │  ┌────────────────┐  │
        │  3 MLP +   │        │  │ Replay buffer  │  │
        │  neural pts│        │  │ 7 个并行张量   │  │
        └────────────┘        │  └────────────────┘  │
                              │  DataSampler (260 ln)│
                              └──────────────────────┘
                                        │
                       ┌────────────────┼────────────────────┐
                       ▼                ▼                    ▼
                ┌──────────────┐ ┌────────────────┐ ┌──────────────┐
                │ PoseGraphMgr │ │ NeuralPointMap │ │ Mesher       │
                │ gtsam ISAM2  │ │ ContextManager │ │ marching cube│
                │ Pose3 / G2O  │ │ Scan Context   │ │              │
                └──────────────┘ │ + virtual nodes│ └──────────────┘
                                 └────────────────┘
```

---

## 2. `Config`(`utils/config.py:12-562`)

200+ 字段,分 10 大组。下表只列**显式 yaml 可设**或**对算法行为有判别作用**的字段。所有默认值都来自 `__init__` `[VERIFY: utils/config.py:13-313]`,YAML 覆盖在 `load()` `[VERIFY: utils/config.py:318-561]`。

### 2.1 setting 组(yaml `setting:` 节)

| 字段 | 默认 | 含义 | 源 |
|------|------|------|----|
| `name` | `"dummy"` | 实验名,会拼到 run_name | `utils/config.py:18` |
| `output_root` | `"experiments"` | 输出根 | `utils/config.py:22` |
| `pc_path` | `""` | 点云目录(支持 `.ply/.pcd/.bin`)| `utils/config.py:23` |
| `pose_path` | `""` | GT 位姿 txt(kitti / tum)| `utils/config.py:24` |
| `calib_path` | `""` | KITTI calib.txt(对 GT 做坐标系变换)| `utils/config.py:25` |
| `label_path` | `""` | SemanticKITTI .label 目录 | `utils/config.py:26` |
| `use_dataloader` | `False` | True 走 KISS-ICP 风格 `dataloaders/*.py` | `utils/config.py:29` |
| `load_model` | `False` | 用预训练 PIN map 进入"纯定位"模式 | `utils/config.py:33` |
| `model_path` | `"/"` | 上面那个 `.pth` 路径 | `utils/config.py:34` |
| `first_frame_ref` | `False` | True 把第 0 帧位姿设成 identity | `utils/config.py:36-37` |
| `begin/end/step_frame` | 0 / 100000 / 1 | 帧 slicing | `utils/config.py:38-40` |
| `seed` | 42 | 随机种子 | `utils/config.py:42` |
| `device` | `"cuda"` | 也支持 `"cpu"`(~5× 慢)| `utils/config.py:44`, `:354` |
| `kitti_correction_on` | `False` | KITTI 内参垂直角修正 | `utils/config.py:48-49` |
| `stop_frame_thre` | 20 | 连续 N 帧近静止 → stop_status | `utils/config.py:50` |
| `deskew` | `False` | 运动畸变补偿 | `utils/config.py:53` |
| `lidar_type_guess` | `"velodyne"` | 反推 per-point ts 时用 | `utils/config.py:54` |

### 2.2 process 组

| 字段 | 默认 | 用途 |
|------|------|------|
| `min_range` / `max_range` | 2.5 / 60 m | 距离裁剪 `[VERIFY: utils/config.py:58-59]` |
| `min_z` / `max_z` | -5 / 80 m | 高度裁剪 `[VERIFY: utils/config.py:63-64]` |
| `rand_downsample` + `rand_down_r` | False / 1.0 | 随机降采样比 | `[VERIFY: utils/config.py:66-68]` |
| `vox_down_m` | 0.05 m | 体素降采样 leaf | `[VERIFY: utils/config.py:67]` |
| `adaptive_range_on` | False | 按当前点云范围动态调整 max_range / voxel | `[VERIFY: utils/config.py:60]` |
| `dynamic_filter_on` | False | 用 SDF 一致性过滤动态点 | `[VERIFY: utils/config.py:86]` |
| `dynamic_certainty_thre` | 1.0 | 神经点 certainty 阈值 | `[VERIFY: utils/config.py:87]` |
| `dynamic_sdf_ratio_thre` | 0.5 | SDF < 0.5 × voxel → 视为表面附近 | `[VERIFY: utils/config.py:88]` |
| `dynamic_min_grad_norm_thre` | 0.25 | 梯度 < 0.25 → 不稳 | `[VERIFY: utils/config.py:89]` |

### 2.3 neuralpoints 组(决定地图表示)

| 字段 | 默认 | 算法意义 |
|------|------|---------|
| `voxel_size_m` | 0.3 | 神经点的体素 leaf,决定地图分辨率 `[VERIFY: utils/config.py:92, :393]` |
| `num_nei_cells` | 2 | KNN 搜索的 ± 体素半径(决定立方体网格大小)`[VERIFY: utils/config.py:95]` |
| `search_alpha` | 0.2 | 立方体内再球裁,见 `[VERIFY: model/neural_points.py:911-948]` |
| `query_nn_k` | 6 | KNN 排序后只取前 6 `[VERIFY: utils/config.py:96]` |
| `weighted_first` | True | True: 先 IDW 融特征再 MLP;False: 先 MLP 再融 SDF `[VERIFY: utils/config.py:93]` |
| `feature_dim` | 8 | 每个神经点 latent 长度 `[VERIFY: utils/config.py:103]` |
| `buffer_size` | 5e7 | 哈希桶数 `[VERIFY: utils/config.py:100]` |
| `idw_index` | 2 | IDW 指数(平方反比距离权)`[VERIFY: utils/config.py:99]` |
| `from_sample_points` | True | 用 surface samples 而不只是测量点构建神经点 `[VERIFY: utils/config.py:108]` |
| `map_surface_ratio` | 0.5 | `|sdf_label| < ratio × σ` 的样本才会触发新点 `[VERIFY: utils/config.py:110, utils/mapper.py:243-249]` |
| `local_map_radius` | 50 → runtime 自适应为 `max_range + 2` | 局部地图截断球半径 `[VERIFY: utils/config.py:115, :560]` |
| `local_map_travel_dist_ratio` | 5.0 | 时间窗 = 5 × radius 的累积里程 `[VERIFY: utils/config.py:114]` |
| `prune_map_on` | False | 是否周期裁剪不稳神经点 | `[VERIFY: utils/config.py:118]` |
| `max_prune_certainty` | 3.0 | < 该值的神经点被裁 | `[VERIFY: utils/config.py:119]` |
| `prune_freq_frame` | 100 | 裁剪周期 | `[VERIFY: utils/config.py:120]` |
| `use_mid_ts` | False | True 时用 `(create_ts + update_ts)/2`,否则只用 create_ts | `[VERIFY: utils/config.py:97, model/neural_points.py:449-454]` |

### 2.4 sampler 组(决定 7 个/ray 的 SDF 监督样本)

| 字段 | 默认 | 含义 |
|------|------|------|
| `surface_sample_range_m` | 0.25 | surface 采样高斯 σ(可被 yaml 覆盖为 `3×vox_down_m`)`[VERIFY: utils/config.py:124, :384]` |
| `surface_sample_n` | 3 | surface 采样个数 `[VERIFY: utils/config.py:125]` |
| `free_sample_begin_ratio` | 0.3 | front freespace 最近比例 `[VERIFY: utils/config.py:126]` |
| `free_sample_end_dist_m` | 1.0 | behind freespace 最大长度(可改为 `4σ`)`[VERIFY: utils/config.py:127, :386]` |
| `free_front_n` | 2 | front freespace 采样个数 | `[VERIFY: utils/config.py:128]` |
| `free_behind_n` | 1 | behind freespace 采样个数 | `[VERIFY: utils/config.py:129]` |

### 2.5 loss 组

| 字段 | 默认 | 含义 |
|------|------|------|
| `main_loss_type` | `'bce'` | `'bce' / 'zhong' / 'sdf_l1' / 'sdf_l2'` | `[VERIFY: utils/config.py:163]` |
| `sigma_sigmoid_m` | 0.1 | BCE sigmoid 的 σ(yaml 默认 = `vox_down_m`)| `[VERIFY: utils/config.py:164, :429]` |
| `logistic_gaussian_ratio` | 0.55 | 用 logistic 拟合 gaussian 的系数 | `[VERIFY: utils/config.py:165]` |
| `loss_weight_on` | False | 加权 vs 改 sigmoid 形状 | `[VERIFY: utils/config.py:168]` |
| `dist_weight_on` | True | 远距离样本权重降低 | `[VERIFY: utils/config.py:170]` |
| `dist_weight_scale` | 0.8 | 权重范围 [0.6, 1.4] | `[VERIFY: utils/config.py:171]` |
| `numerical_grad` | True | True 用 Neuralangelo 风格数值梯度做 eikonal | `[VERIFY: utils/config.py:173]` |
| `gradient_decimation` | 10 | numerical grad 时点降采样比 | `[VERIFY: utils/config.py:174]` |
| `num_grad_step_ratio` | 0.2 | grad 步长 = ratio × voxel_size_m | `[VERIFY: utils/config.py:175]` |
| `ekional_loss_on` | True | eikonal `(\|∇\|−1)²` 正则 | `[VERIFY: utils/config.py:177]` |
| `ekional_add_to` | `'all'` | `'all' / 'surface' / 'freespace'` | `[VERIFY: utils/config.py:178]` |
| `weight_e` | 0.5 | eikonal 权重 | `[VERIFY: utils/config.py:179]` |
| `weight_s` | 1.0 | 语义 loss 权重 | `[VERIFY: utils/config.py:186]` |
| `weight_i` | 1.0 | 颜色/强度 loss 权重 | `[VERIFY: utils/config.py:187]` |

### 2.6 optimizer 组

| 字段 | 默认 | 含义 |
|------|------|------|
| `mapping_freq_frame` | 1 | 每帧都做一次 mapping | `[VERIFY: utils/config.py:190]` |
| `iters` | 12 | 每次 mapping 迭代次数 | `[VERIFY: utils/config.py:191]` |
| `init_iter_ratio` | 40 | 第 0 帧 warm-up iters = 40 × iters | `[VERIFY: utils/config.py:192]` |
| `bs` | 16384 | batch size | `[VERIFY: utils/config.py:194]` |
| `lr` | 0.01 | Adam 学习率 | `[VERIFY: utils/config.py:195]` |
| `adam_eps` | 1e-15 | Adam ε | `[VERIFY: utils/config.py:199]` |
| `adaptive_iters` | False | 新样本少时减 iters | `[VERIFY: utils/config.py:200-203]` |
| `ba_freq_frame` | 0 | 0 = 关 BA | `[VERIFY: utils/config.py:206]` |
| `ba_frame` | 50 | BA 滑窗位姿数 | `[VERIFY: utils/config.py:207]` |
| `ba_iters` | 80 | BA 迭代数 | `[VERIFY: utils/config.py:208]` |
| `lr_pose` | 1e-4 | BA 位姿学习率 | `[VERIFY: utils/config.py:196]` |
| `lr_ba_map` | 0.01 | BA 地图学习率 | `[VERIFY: utils/config.py:197]` |

### 2.7 tracker 组

| 字段 | 默认 | 含义 |
|------|------|------|
| `track_on` | False(yaml 出现 `tracker:` 节自动开)| 关掉就是"用 GT 做增量建图" | `[VERIFY: utils/config.py:212, :453-454]` |
| `source_vox_down_m` | 0.8 | 源点云第 2 次降采样(更稀疏)| `[VERIFY: utils/config.py:216, :461]` |
| `uniform_motion_on` | True | True 用匀速模型外推初值 | `[VERIFY: utils/config.py:217]` |
| `reg_min_grad_norm` | 0.5 | SDF 梯度模太小则视为无效点 | `[VERIFY: utils/config.py:218]` |
| `reg_max_grad_norm` | 2.0 | SDF 梯度模太大则视为外点 | `[VERIFY: utils/config.py:219]` |
| `track_mask_query_nn_k` | = query_nn_k | 跟踪期间 valid 点最少邻居数 | `[VERIFY: utils/config.py:220]` |
| `max_sdf_ratio` | 5.0 | 残差上限 = ratio × σ | `[VERIFY: utils/config.py:221]` |
| `reg_GM_dist_m` | 0.3 | Geman-McClure 残差尺度 | `[VERIFY: utils/config.py:224]` |
| `reg_GM_grad` | 0.1 | Geman-McClure 梯度异常尺度 | `[VERIFY: utils/config.py:225]` |
| `reg_lm_lambda` | 1e-4 | LM damping λ | `[VERIFY: utils/config.py:227]` |
| `reg_iter_n` | 50 | LM 最大迭代数 | `[VERIFY: utils/config.py:228]` |
| `reg_term_thre_deg` | 0.01° | 旋转终止阈值 | `[VERIFY: utils/config.py:229]` |
| `reg_term_thre_m` | 0.001 m | 平移终止阈值 | `[VERIFY: utils/config.py:230]` |
| `eigenvalue_check` | True | 退化方向检测开关 | `[VERIFY: utils/config.py:231]` |
| `eigenvalue_ratio_thre` | 0.005 | 退化阈值,< n_pts × ratio 判退化 | `[VERIFY: utils/config.py:232]` |
| `final_residual_ratio_thre` | 0.6 | 最终残差上限 = ratio × σ | `[VERIFY: utils/config.py:233]` |

### 2.8 pgo 组

| 字段 | 默认 | 含义 |
|------|------|------|
| `pgo_on` | False(yaml 出现 `pgo:` 节自动开,需先 track_on)| 后端开关 | `[VERIFY: utils/config.py:254, :476-478]` |
| `pgo_freq` | 30 帧 | 最少多少帧才尝试一次回环 | `[VERIFY: utils/config.py:255]` |
| `pgo_with_isam` | True | True 用 ISAM2,False 用 LM batch | `[VERIFY: utils/config.py:256]` |
| `pgo_max_iter` | 50 | LM 最大迭代 | `[VERIFY: utils/config.py:257]` |
| `pgo_tran_std` / `pgo_rot_std` | 0.04 m / 0.01° | 默认协方差 σ | `[VERIFY: utils/config.py:259-260]` |
| `use_reg_cov_mat` | False | True 用跟踪输出的 6×6 协方差 | `[VERIFY: utils/config.py:261]` |
| `pgo_error_thre_frame` | 500.0 | per-frame 错误阈值,用于 reject loop | `[VERIFY: utils/config.py:262]` |
| `pgo_merge_map` | False | 回环后是否合并神经点 | `[VERIFY: utils/config.py:263]` |
| `rehash_with_time` | True | True 用时间差 rehash,False 用 certainty | `[VERIFY: utils/config.py:264]` |
| `global_loop_on` | True | 用 NPMC 做全局回环 | `[VERIFY: utils/config.py:236]` |
| `local_map_context` | False | True 用本地图,False 用 scan context | `[VERIFY: utils/config.py:237]` |
| `loop_with_feature` | False | 把神经点特征编进描述子 | `[VERIFY: utils/config.py:238]` |
| `local_map_context_latency` | 5 帧 | local map 描述子的滞后帧数 | `[VERIFY: utils/config.py:240]` |
| `context_shape` | [20, 60] | [ring, sector] | `[VERIFY: utils/config.py:244]` |
| `npmc_max_dist` | 60 m | NPMC 极坐标最大半径 | `[VERIFY: utils/config.py:245]` |
| `context_cosdist_threshold` | 0.2 | 候选 cos 距离阈值 | `[VERIFY: utils/config.py:247]` |
| `context_virtual_side_count` | 5 | 单侧虚拟节点数(共 11 个虚拟位置)| `[VERIFY: utils/config.py:248]` |
| `context_virtual_step_m` | 2.0 m | 虚拟节点横向步长 | `[VERIFY: utils/config.py:249]` |
| `min_loop_travel_dist_ratio` | 4.0 | 候选必须里程差 > ratio × local_map_radius | `[VERIFY: utils/config.py:239]` |
| `local_loop_dist_thre` | 2.0 m | 当前位姿距历史位姿在此值内 → 当作 local loop | `[VERIFY: utils/config.py:243]` |
| `loop_z_check_on` | False | True 时 reject Δz > 4×voxel 的回环(多层建筑歧义)| `[VERIFY: utils/config.py:250]` |
| `loop_dist_drift_ratio_thre` | 2.0 | 全局回环搜索半径 = ratio × drift_radius | `[VERIFY: utils/config.py:251]` |

### 2.9 关联派生(load() 末尾自动计算,yaml 不可直设)

```python
self.infer_bs = self.bs * 32                    # config.py:557
self.consistency_count = int(self.bs / 4)        # config.py:558
self.window_radius = max(self.max_range, 6.0)    # config.py:559
self.local_map_radius = self.max_range + 2.0     # config.py:560 (覆盖 yaml)
self.vis_frame_axis_len = self.max_range / 40.0  # config.py:561
self.vis_sdf_res_m = self.voxel_size_m * 0.3     # config.py:562
```

⚠️ **陷阱**: `local_map_radius` 字段在 yaml `neuralpoints:` 节即使写了也会被这一行覆盖。

---

## 3. `NeuralPoints`(`model/neural_points.py:29`)

继承 `nn.Module`,既是数据容器又是模型,可放入 optimizer。

### 3.1 全局张量集(整个地图的真值)

| 字段 | dtype / shape | 含义 | 源 |
|------|---------------|------|----|
| `buffer_pt_index` | `int64 [buffer_size]` | 哈希桶 → 全局神经点 idx;空桶 = -1 | `model/neural_points.py:88-90` |
| `neural_points` | `float32 [N, 3]` | 全局神经点位置 | `model/neural_points.py:92` |
| `point_orientations` | `float32 [N, 4]` | 四元数,默认 [1,0,0,0](回环后可能非单位)| `model/neural_points.py:93-95` |
| `geo_features` | `float32 [N+1, F]` | 几何特征,末位为 padding(-1 idx 用)| `model/neural_points.py:96-98` |
| `color_features` | `float32 [N+1, F]` 或 None | 颜色特征 | `model/neural_points.py:101-106` |
| `point_ts_create` | `int [N]` | 创建该点时的 frame_id | `model/neural_points.py:112-114` |
| `point_ts_update` | `int [N]` | 最后一次被查询时的 frame_id(scatter_reduce amax)| `model/neural_points.py:115-117` |
| `point_certainties` | `float32 [N]` | 累积 IDW 权重和 — 用作 stability 指标 | `model/neural_points.py:118` |

### 3.2 局部张量集(当前局部地图,真正参与优化)

| 字段 | dtype / shape | 含义 | 源 |
|------|---------------|------|----|
| `local_neural_points` | `float32 [M, 3]` | 局部位置 | `model/neural_points.py:121-123` |
| `local_point_orientations` | `float32 [M, 4]` | 局部姿态四元数 | `model/neural_points.py:124-126` |
| `local_geo_features` | `nn.Parameter [M+1, F]` | **关键**: 被 Adam 优化的几何特征 | `model/neural_points.py:127` |
| `local_color_features` | `nn.Parameter [M+1, F]` | 同上颜色版本 | `model/neural_points.py:128` |
| `local_point_certainties` | `float32 [M]` | 局部 certainty | `model/neural_points.py:129-131` |
| `local_point_ts_update` | `int [M]` | 局部更新时间 | `model/neural_points.py:132-134` |
| `local_mask` | `bool [N+1]` | True 表示该全局点在局部地图,末位 True 作 padding | `model/neural_points.py:483-495` |
| `global2local` | `int64 [N+1]` | 全局 idx → 局部 idx,非局部 = -1 | `model/neural_points.py:499-508` |

### 3.3 元数据 / 控制字段

| 字段 | 含义 | 源 |
|------|------|----|
| `resolution` | = `config.voxel_size_m` | `model/neural_points.py:56` |
| `buffer_size` | 哈希桶数 | `model/neural_points.py:58` |
| `primes` | `int64 [3]` = (73856093, 19349669, 83492791) | `model/neural_points.py:82-84` |
| `temporal_local_map_on` | True(纯定位关掉)| `model/neural_points.py:60` |
| `local_map_radius` | 截断球半径 | `model/neural_points.py:61` |
| `diff_travel_dist_local` | 时间窗 = `radius × ratio` | `model/neural_points.py:62-64` |
| `reboot_ts` | 系统重启时间戳,reset_local_map 用 | `model/neural_points.py:70` |
| `local_orientation` | 当前传感器姿态(目前未用)| `model/neural_points.py:72` |
| `cur_ts`, `max_ts` | 当前 / 见过的最大 frame_id | `model/neural_points.py:74-75` |
| `travel_dist` | 由 dataset 同步过来的累计里程数组 | `model/neural_points.py:77` |
| `after_pgo` | True 时 query_feature 会对邻域向量做四元数旋转 | `model/neural_points.py:79, :646-649` |
| `position_encoder_geo/color` | `PositionalEncoder` 或 `GaussianFourierFeatures`(默认前者,band=0 即不编码)| `model/neural_points.py:43-48, :1077-1144` |
| `neighbor_dx` | `int64 [K, 3]` 邻域偏移网格 | `model/neural_points.py:920-933` |
| `neighbor_K` | K(num_cells=2 时 K=33,num_cells=3 时 K=147)| `model/neural_points.py:935-947` |
| `max_valid_dist2` | 哈希碰撞过滤阈值 = `3 × ((num_cells+1) × resolution)²` | `model/neural_points.py:948` |
| `cur_memory_mb`, `memory_footprint` | 内存追踪 | `model/neural_points.py:143-144, :160-173` |

### 3.4 `PositionalEncoder`(同文件,默认未启用)

- band=0 时 `forward(x) = x`,featureSize = 3(`config.pos_encoding_band = 0`)`[VERIFY: utils/config.py:157, model/neural_points.py:1077-1124]`
- band>0 时拼接 sin/cos 高频项;`use_gaussian_pe=True` 改用高斯傅里叶投影 `[VERIFY: model/neural_points.py:1125-1143]`

---

## 4. `Decoder`(`model/decoder.py:14`)

只有 113 行,本质是 `Linear → ReLU → … → Linear` 栈。

### 4.1 字段

| 字段 | 类型 / 形状 | 含义 |
|------|-------------|------|
| `layers` | `nn.ModuleList` of `nn.Linear` | hidden 层栈,长度 = `hidden_level`(默认 1)`[VERIFY: model/decoder.py:45-51]` |
| `lout` | `nn.Linear(hidden_dim, out_dim)` | 输出投影 `[VERIFY: model/decoder.py:52]` |
| `out_dim` | 1(SDF)/ `sem_class_count+1`(语义)/ `color_channel`(颜色)| `[VERIFY: pin_slam.py:139-141]` |
| `use_leaky_relu` | `config.mlp_leaky_relu`(默认 False) | `[VERIFY: model/decoder.py:27]` |
| `sdf_scale` | `logistic_gaussian_ratio × sigma_sigmoid_m` = 0.55 × 0.1 = 0.055(默认) | `[VERIFY: model/decoder.py:54-56]` |

### 4.2 输入维度推导

```
input_dim = feature_dim + position_dim
```
其中 `position_dim`:
- `use_gaussian_pe`: `pos_input_dim + 2 × pos_encoding_band = 3 + 0 = 3`(默认)
- 否则: `pos_input_dim × (2 × pos_encoding_band + 1) = 3 × 1 = 3`
- 来源 `[VERIFY: model/decoder.py:31-37]`

默认: `input_dim = 8 + 3 = 11`

### 4.3 forward 接口

| 方法 | 输出 | 说明 |
|------|------|------|
| `mlp(features)` | `[N, hidden_dim]` → `lout` → `[N, out_dim]` | 通用前向 | `[VERIFY: model/decoder.py:61-79]` |
| `sdf(features)` | `[N]`,标量 SDF(单位 m,经 sdf_scale 缩放) | 实际是 `−SDF`,见 §2 注释 | `[VERIFY: model/decoder.py:81-85]` |
| `time_conditionded_sdf(features, ts)` | 同上 | feature 末位拼 ts(目前未启用)| `[VERIFY: model/decoder.py:87-92]` |
| `occupancy(features)` | `[N]` ∈ [0,1] | `sigmoid(-sdf / sdf_scale)` | `[VERIFY: model/decoder.py:95-97]` |
| `sem_label_prob(features)` | `[N, n_class]` | `log_softmax` | `[VERIFY: model/decoder.py:100-102]` |
| `sem_label(features)` | `[N]` int | argmax | `[VERIFY: model/decoder.py:104-106]` |
| `regress_color(features)` | `[N, color_channel]` ∈ [0,1] | `sigmoid` 输出 | `[VERIFY: model/decoder.py:112-113]` |

⚠️ **陷阱**: `Decoder.sdf` 返回的 SDF 在源码里是"opposite sign to the actual sdf"`[VERIFY: model/decoder.py:82]`。occupancy/loss 调用时都做了对应反号,使用方不需要再手动反号。

---

## 5. `SLAMDataset`(`dataset/slam_dataset.py:37`)

### 5.1 当前帧缓存

| 字段 | 类型 | 含义 |
|------|------|------|
| `cur_point_cloud_torch` | `float32 [N, 3 或 3+color_ch]` | 当前帧降采样 + crop 后的点(传感器系)| `[VERIFY: dataset/slam_dataset.py:171, :287-289]` |
| `cur_point_ts_torch` | `float32 [N]` ∈ [0,1] 或 None | 每点归一化时间戳(deskew 用)| `[VERIFY: dataset/slam_dataset.py:172, :297-347]` |
| `cur_sem_labels_torch` | `int [N]` | reduced 语义标签(20 类)| `[VERIFY: dataset/slam_dataset.py:173, :280-282]` |
| `cur_sem_labels_full` | `int [N]` | full 语义标签 | `[VERIFY: dataset/slam_dataset.py:174, :283-285]` |
| `cur_source_points` | `float32 [Ns, 3]`(Ns << N)| 二次更稀疏降采样,用于跟踪 | `[VERIFY: dataset/slam_dataset.py:177, :475-485]` |
| `cur_source_normals` | `float32 [Ns, 3]` 或 None | 法线(未默认计算)| `[VERIFY: dataset/slam_dataset.py:178]` |
| `cur_source_colors` | `float32 [Ns, color_ch]` 或 None | 跟踪用颜色 | `[VERIFY: dataset/slam_dataset.py:179, :483-484]` |
| `cur_pose_ref` | `np.float64 [4,4]` | 当前帧参考位姿(GT 或 identity)| `[VERIFY: dataset/slam_dataset.py:148, :349-357]` |
| `cur_pose_guess_torch` | `float64 [4,4]` | 跟踪初值(uniform motion 或 static)| `[VERIFY: dataset/slam_dataset.py:387-389]` |
| `cur_pose_torch` | `float64 [4,4]` | 当前帧已估计位姿(detach,用于 mapping)| `[VERIFY: dataset/slam_dataset.py:514-516]` |
| `last_pose_ref` | `np.float64 [4,4]` | 上一帧位姿(用于推算 last_odom_tran)| `[VERIFY: dataset/slam_dataset.py:146]` |
| `last_odom_tran` | `np.float64 [4,4]` = `inv(last) @ cur` | 上一帧到当前帧的相对变换 | `[VERIFY: dataset/slam_dataset.py:147, :518]` |
| `static_mask` | `bool [N]` | 动态过滤结果(被 mapper 写回)| `[VERIFY: dataset/slam_dataset.py:168, utils/mapper.py:212]` |
| `cur_frame_o3d` | `o3d.PointCloud` | 可视化点云缓存 | `[VERIFY: dataset/slam_dataset.py:160]` |
| `cur_bbx` / `map_bbx` | `o3d.AABB` | 当前帧 / 全图包围盒 | `[VERIFY: dataset/slam_dataset.py:162, :166]` |

### 5.2 位姿数组与里程

| 字段 | shape | 含义 |
|------|-------|------|
| `odom_poses` | `np.float64 [max_frame_number, 4, 4]` | 纯里程计输出 | `[VERIFY: dataset/slam_dataset.py:128-130]` |
| `pgo_poses` | `np.float64 [max_frame_number, 4, 4]` | 经过 PGO 修正的位姿(默认 = odom)| `[VERIFY: dataset/slam_dataset.py:132-134]` |
| `gt_poses` | `np.float64 [total_pc_count, 4, 4]` 或 None | KITTI/TUM 解析的 GT | `[VERIFY: dataset/slam_dataset.py:51, :115-121]` |
| `gt_pose_provided` | bool | GT 是否存在 | `[VERIFY: dataset/slam_dataset.py:69, :97-99]` |
| `travel_dist` | `np.float32 [max_frame_number]` | 累计里程 | `[VERIFY: dataset/slam_dataset.py:136, :546]` |
| `time_table` | `list of np.array(5)` | 每帧的 5 段耗时 | `[VERIFY: dataset/slam_dataset.py:137, pin_slam.py:500-501]` |
| `processed_frame` | int | 已处理帧数 | `[VERIFY: dataset/slam_dataset.py:139]` |
| `lose_track` / `consecutive_lose_track_frame` | bool / int | 跟踪失败状态 | `[VERIFY: dataset/slam_dataset.py:141-142]` |
| `stop_count` / `stop_status` | int / bool | 停车状态 | `[VERIFY: dataset/slam_dataset.py:150-151]` |
| `shift_ts` | float | rosbag 模式的时间偏移 | `[VERIFY: dataset/slam_dataset.py:140]` |
| `color_scale` | float = 255.0(KITTI raw 改 1.0)| 颜色除以该值 → [0,1] | `[VERIFY: dataset/slam_dataset.py:145, :154-157]` |

### 5.3 calib / 加载器

| 字段 | 含义 |
|------|------|
| `calib["Tr"]` | `np.float64 [4,4]`,从相机系到激光系 | `[VERIFY: dataset/slam_dataset.py:52, :73, :102]` |
| `loader` | KISS-ICP `dataset_factory` 返回的 dataloader 实例(或 None)| `[VERIFY: dataset/slam_dataset.py:54-80]` |
| `K_mats` / `T_c_l_mats` | 相机内参 / 外参字典(由 loader 给)| `[VERIFY: dataset/slam_dataset.py:74-78]` |
| `cur_cam_img` | dict[cam_name → ndarray] 当前帧多相机图像 | `[VERIFY: dataset/slam_dataset.py:238]` |
| `cur_frame_imus` | IMU 队列(目前未消费)| `[VERIFY: dataset/slam_dataset.py:240-241]` |
| `pc_filenames` | natsort 后的文件名列表(泛型 loader)| `[VERIFY: dataset/slam_dataset.py:87-91]` |

---

## 6. `Tracker`(`utils/tracker.py:20`)

### 6.1 字段

| 字段 | 含义 |
|------|------|
| `config` | `Config` 引用 `[VERIFY: utils/tracker.py:28]` |
| `neural_points` | `NeuralPoints` 引用(只读)`[VERIFY: utils/tracker.py:30]` |
| `sdf_mlp / sem_mlp / color_mlp` | 来自 `decoders` dict `[VERIFY: utils/tracker.py:31-33]` |
| `reg_local_map` | True(纯定位模式被外部置 False,改查全局)`[VERIFY: utils/tracker.py:38, pin_slam.py:177]` |
| `sdf_scale` | = `logistic_gaussian_ratio × sigma_sigmoid_m` = 0.055(默认)`[VERIFY: utils/tracker.py:40]` |

### 6.2 `tracking()` 返回元组

```python
T, cov_mat, weight_point_cloud, valid_flag = tracker.tracking(...)
```
- `T` ▸ `float64 [4,4]`,世界系 → 当前帧位姿 `[VERIFY: utils/tracker.py:71-73, :225]`
- `cov_mat` ▸ `numpy [6,6]`(可能为 None)`[VERIFY: utils/tracker.py:218-219]`
- `weight_point_cloud` ▸ `o3d.geometry.PointCloud` 用于可视化每点权重(`vis_result=True` 才填)`[VERIFY: utils/tracker.py:466, :558-589]`
- `valid_flag` ▸ bool,通过 4 道安全门则为 True `[VERIFY: utils/tracker.py:103, :150-216]`

### 6.3 单步 `registration_step()` 返回

```python
T, cov_mat, eigenvalues, weight_pc, valid_points, sdf_residual_cm, photo_residual = ...
```
关键中间张量(`utils/tracker.py:367-611`):
- `sdf_pred / sdf_grad` ▸ 来自 `query_source_points`,梯度由 `get_gradient` 解析求 `[VERIFY: utils/tracker.py:330-334]`
- `grad_norm` ▸ ∇sdf 模长,理论上 ≈ 1 `[VERIFY: utils/tracker.py:409]`
- `valid_idx` ▸ 复合 mask:nn_count + grad_norm 范围 + sdf_std + (可选 sdf 大小)`[VERIFY: utils/tracker.py:419-425]`
- 鲁棒权重 `w_grad / w_res / w_normal / w_color / w_std` ▸ Geman-McClure + 法线一致性 + 颜色一致性 `[VERIFY: utils/tracker.py:471-522]`

---

## 7. `Mapper`(`utils/mapper.py:33`)

### 7.1 持有引用

| 字段 | 含义 |
|------|------|
| `config / dataset / neural_points` | 三大上下文 `[VERIFY: utils/mapper.py:42-45]` |
| `sdf_mlp / sem_mlp / color_mlp` | 同 Tracker 但**可写**(Adam 训练目标)`[VERIFY: utils/mapper.py:46-48]` |
| `sampler` | `DataSampler` 实例 `[VERIFY: utils/mapper.py:69]` |
| `ray_sample_count` | = 1 + 3 + 1 + 2 = 7(默认)`[VERIFY: utils/mapper.py:70-72]` |
| `sdf_scale` | = 0.55 × σ `[VERIFY: utils/mapper.py:66]` |
| `require_gradient` | 根据 loss/regularization 配置自动决定是否对 coord requires_grad `[VERIFY: utils/mapper.py:52-64]` |
| `total_iter` | 累计 Adam 步数 `[VERIFY: utils/mapper.py:65]` |
| `used_poses` | `float64 [F+1, 4, 4]`,根据 pgo/track/gt 选 `[VERIFY: utils/mapper.py:139-159]` |
| `ba_done_flag` | BA 后需要重做 `global_coord_pool` 变换 `[VERIFY: utils/mapper.py:75, :301-305]` |
| `adaptive_iter_offset` | 根据 new sample ratio 加减 iters `[VERIFY: utils/mapper.py:76, :424-438]` |
| `static_mask` | dynamic_filter 结果 cache `[VERIFY: utils/mapper.py:183, utils/mapper.py:212]` |
| `cur_sample_count / pool_sample_count` | 这一帧 / 总池容量 `[VERIFY: utils/mapper.py:233-234]` |
| `new_idx` | 长 ≤ cur_sample_count 的索引,指向"新观测"样本 `[VERIFY: utils/mapper.py:408-414]` |

### 7.2 训练数据池(7 个并行张量,逻辑上 N×… 同长)

| 字段 | dtype / shape | 含义 |
|------|---------------|------|
| `coord_pool` | `float32 [P, 3]` | 各帧本地坐标系下的采样点 | `[VERIFY: utils/mapper.py:82-84]` |
| `global_coord_pool` | `float32 [P, 3]` | 转到世界系后的同样点(BA / 回环后会被重做)| `[VERIFY: utils/mapper.py:85-87]` |
| `sdf_label_pool` | `float32 [P]` | SDF 监督标签(带符号 displacement)| `[VERIFY: utils/mapper.py:88]` |
| `color_pool` | `float32 [P, color_ch]` | 颜色 / 强度 | `[VERIFY: utils/mapper.py:89-91]` |
| `sem_label_pool` | `int [P]` | 语义标签 | `[VERIFY: utils/mapper.py:92]` |
| `normal_label_pool` | `float32 [P, 3]` | 法线(未默认计算)| `[VERIFY: utils/mapper.py:93-95]` |
| `weight_pool` | `float32 [P]` | 距离权重,符号位标记 surface(+)/freespace(-)| `[VERIFY: utils/mapper.py:96, utils/data_sampler.py:168]` |
| `time_pool` | `int [P]` | 这些样本属于的 frame_id | `[VERIFY: utils/mapper.py:97]` |

容量管控:
- `pool_capacity` 默认 1e7,超过随机丢 `[VERIFY: utils/mapper.py:328-336]`
- `window_radius` = `max(max_range, 6.0)`,池外丢 `[VERIFY: utils/mapper.py:317-322]`
- `pool_filter_freq` = 1 → 每帧都 filter `[VERIFY: utils/config.py:136, utils/mapper.py:316]`

### 7.3 `get_batch()` 返回

```python
coord, sdf_label, ts, normal, sem, color, weight = mapper.get_batch(global_coord=True/False)
```
按 `bs_new_sample` 拆"新/旧"两半:`bs_new_sample` 个新 + `bs - bs_new_sample` 个池随机 `[VERIFY: utils/mapper.py:452-503]`。

---

## 8. `DataSampler`(`utils/data_sampler.py:12`)

无状态(只 `config / dev`),`sample(points, normal, sem, color)` 返回 6 元组:

| 输出 | shape | 含义 |
|------|-------|------|
| `all_sample_points` | `float32 [N × 7, 3]` | 每束射线 7 个 sample(顺序: ray0-surface0..6, ray1-surface0..6...) `[VERIFY: utils/data_sampler.py:211-216]` |
| `sdf_label_tensor` | `float32 [N × 7]` | 带符号 displacement(behind +, in-front -),最后乘 -1 `[VERIFY: utils/data_sampler.py:217-220]` |
| `normal_label_tensor` | `float32 [N × 7, 3]` 或 None | `[VERIFY: utils/data_sampler.py:228-232]` |
| `sem_label_tensor` | `int [N × 7]` 或 None,freespace 类为 0 | `[VERIFY: utils/data_sampler.py:186-195, :234-236]` |
| `color_tensor` | `float32 [N × 7, color_ch]` 或 None,freespace 全 0 | `[VERIFY: utils/data_sampler.py:199-208, :237-242]` |
| `weight_tensor` | `float32 [N × 7]` | surface 段 [0.6, 1.4] 距离权重,freespace 段乘 -1 作为"类型标志"(后端按 abs 用)`[VERIFY: utils/data_sampler.py:148-168, :222-224]` |

7 个 sample 详细组成:
- 1 个 measurement (Part 0,dist_ratio=1.0,displacement=0)
- 3 个 surface (Part 1,高斯 σ = `surface_sample_range_m`)
- 2 个 free_front (Part 2,uniform [`free_sample_begin_ratio`, `1 - 2σ/d`])
- 1 个 free_behind (Part 3,uniform [`1 + 2σ/d`, `1 + free_sample_end_dist_m/d`])

> `sigma_ratio = 2.0` 写死在 `utils/data_sampler.py:71`,意思是 freespace 始终距离表面 ≥ 2σ。

---

## 9. `PoseGraphManager`(`utils/pgo.py:18`)

### 9.1 字段

| 字段 | 类型 | 含义 |
|------|------|------|
| `fixed_cov` | `gtsam.noiseModel.Diagonal.Sigmas` σ=1e-9 | 固定初始节点的协方差 | `[VERIFY: utils/pgo.py:24-26]` |
| `const_cov` | `np.array(6,)` | 默认 [rot×3, tran×3] σ | `[VERIFY: utils/pgo.py:31-40]` |
| `odom_cov` / `loop_cov` | 同上 wrap 后的 gtsam 噪声模型 | `[VERIFY: utils/pgo.py:41-42]` |
| `robust_odom_cov` / `robust_loop_cov` | Geman-McClure 鲁棒 wrapper(目前未使用)| `[VERIFY: utils/pgo.py:45-47]` |
| `isam` | `gtsam.ISAM2()` 增量优化器 | `[VERIFY: utils/pgo.py:49]` |
| `graph_factors` | `gtsam.NonlinearFactorGraph()` 当前批次因子 | `[VERIFY: utils/pgo.py:51]` |
| `graph_initials` | `gtsam.Values()` 当前批次初值 | `[VERIFY: utils/pgo.py:52]` |
| `cur_pose` | `np.float64 [4,4]` 当前帧优化后的位姿 | `[VERIFY: utils/pgo.py:54, :227]` |
| `curr_node_idx` | int 当前帧 id | `[VERIFY: utils/pgo.py:55, :76]` |
| `graph_optimized` | `gtsam.Values` 上一次优化结果 | `[VERIFY: utils/pgo.py:56, :196, :207]` |
| `init_poses` / `pgo_poses` | `np.float64 [F+1, 4, 4]` 优化前/后位姿矩阵 | `[VERIFY: utils/pgo.py:57-58, :223-225]` |
| `loop_edges_vis` | `list of [from, to] np.uint32` 仅供可视化(本地图带 latency 时为 frame−latency) | `[VERIFY: utils/pgo.py:60, pin_slam.py:330-331]` |
| `loop_edges` | `list of [loop_id, frame_id]` 真实回环边 | `[VERIFY: utils/pgo.py:61, pin_slam.py:332]` |
| `loop_trans` | `list of np.float64 [4,4]` 每条回环边的相对变换 | `[VERIFY: utils/pgo.py:62, pin_slam.py:333]` |
| `min_loop_idx` / `last_loop_idx` | int 历史回环的最小 / 最近 | `[VERIFY: utils/pgo.py:64-65]` |
| `drift_radius` | float (m),估计的累计漂移 | `[VERIFY: utils/pgo.py:66, :323-336]` |
| `pgo_count` | int 已成功 PGO 次数 | `[VERIFY: utils/pgo.py:67]` |
| `last_error` | float LM 上次残差 | `[VERIFY: utils/pgo.py:68, :211-212]` |

### 9.2 因子类型

- 节点: `gtsam.symbol("x", frame_id)` `[VERIFY: utils/pgo.py:78-82, :114-117]`
- Prior: `gtsam.PriorFactorPose3` `[VERIFY: utils/pgo.py:113-117]`
- Odom edge: `gtsam.BetweenFactorPose3(x_prev, x_cur, T_prev<-cur, odom_cov)` `[VERIFY: utils/pgo.py:135-142]`
- Loop edge: `gtsam.BetweenFactorPose3(x_loop, x_cur, T_loop<-cur, loop_cov)` `[VERIFY: utils/pgo.py:165-172]`

⚠️ 注: 代码中 `add_loop_factor` 里有 `reject_outlier` 启发式,**仅当 `pgo_with_isam=False`** 才会触发(`utils/pgo.py:174`)。ISAM2 路径下回环边一旦加入就不会再被回滚。

---

## 10. `NeuralPointMapContextManager`(`utils/loop_detector.py:18`)

### 10.1 字段

| 字段 | shape / 类型 | 含义 |
|------|--------------|------|
| `des_shape` | tuple = (20, 60) | (ring, sector) `[VERIFY: utils/loop_detector.py:27]` |
| `num_candidates` | 1 | 排名前 K 候选 `[VERIFY: utils/loop_detector.py:28]` |
| `ringkey_dist_thre` | `(max_z - min_z) × 0.25` m(或 0.25 cos)| 一次粗筛阈值 `[VERIFY: utils/loop_detector.py:29, :36]` |
| `sc_cosdist_threshold` | 默认 0.2,启用 local map 自动 +0.08,带 feature 再 +0.08 | 二次精筛阈值 `[VERIFY: utils/loop_detector.py:31-35]` |
| `max_length` | `npmc_max_dist` = 60 m | 极坐标最大半径 `[VERIFY: utils/loop_detector.py:38]` |
| `ENOUGH_LARGE` | `config.end_frame` | 描述子数组容量 `[VERIFY: utils/loop_detector.py:41]` |
| `contexts` | `list[None] × ENOUGH_LARGE` | 每帧 SC,shape `[R, S]` `[VERIFY: utils/loop_detector.py:43]` |
| `ringkeys` | `list[None] × ENOUGH_LARGE` | 每帧 ring key,shape `[R]` `[VERIFY: utils/loop_detector.py:44]` |
| `contexts_feature` | `list[None] × …` | 带特征版 SC,shape `[R, S, F]` `[VERIFY: utils/loop_detector.py:45]` |
| `ringkeys_feature` | `list[None] × …` | shape `[R, F]` `[VERIFY: utils/loop_detector.py:46]` |
| `valid_flags` | `list[False] × …` | 标记某帧描述子是否可信(基于 lose_track / stop)`[VERIFY: utils/loop_detector.py:47]` |
| `query_contexts` / `tran_from_frame` | list 缓存当前帧的虚拟节点描述子与相对变换 | `[VERIFY: utils/loop_detector.py:49-50, :79-80]` |
| `curr_node_idx` | int 当前最新帧 id | `[VERIFY: utils/loop_detector.py:51, :69]` |
| `virtual_step_m` | `context_virtual_step_m` = 2.0 m | 虚拟节点横向步长 `[VERIFY: utils/loop_detector.py:54]` |
| `virtual_side_count` | 5 → 共 11 个虚拟位置 | `[VERIFY: utils/loop_detector.py:55]` |
| `virtual_sdf_thre` | 0.0(目前注释掉,未启用)| `[VERIFY: utils/loop_detector.py:56, :113-118]` |

### 10.2 `GTLoopManager`(`utils/loop_detector.py:376`,仅 debug)

直接用 GT 位姿距离判回环,实验对照用,不进入主流程。

---

## 11. `Mesher`(`utils/mesher.py`,本文档不全开)

只列对外接口:
- `recon_aabb_collections_mesh(chunks_aabb, mc_res_m, save_path, only_local_bbx, semantic_on, color_on, filter_isolated_mesh, mesh_min_nn)` ▸ `o3d.TriangleMesh`
- `generate_bbx_sdf_hor_slice(bbx, height, res, return_pcd, sdf_min, sdf_max)` ▸ `o3d.PointCloud`(用颜色编码 SDF)
- `generate_bbx_sdf_ver_slice(bbx, x, res, return_pcd, sdf_min, sdf_max)` ▸ 同上

Mesher 内部反复调用 `Mapper.sdf_batch` / `query_source_points`,所以它的"内存模型"与 mapper 共享。

---

## 12. 内存量级速算

默认设置下:
- 每个神经点:`3 + 4 + 8 (geo) + 8 (color) = 23 个 float32 = 92 B`
- 100 万神经点 ≈ 92 MB(对应 `~10⁴ m² × (0.3m grid) ≈ 1e5` 量级)
- 数据池容量 `pool_capacity = 1e7`,每条目 `coord 3 + global 3 + sdf 1 + weight 1 + ts 1 + 可选 color 3 + sem 1` ≈ 13 floats → **~520 MB**
- batch_size 16384 × `7 × 11(input_dim) × 4 B` ≈ 5 MB 中间张量 / forward
- LiDAR 点云 ~12 万点降采样到 vox 0.05 后 ~5 万,跟踪源 ~5 千点

实测见 `NeuralPoints.record_memory()` `[VERIFY: model/neural_points.py:160-173]`,会把当前 mb 写进 `memory_footprint` list,可在 PGO 时打印。

---

**END of Phase 2 DATA STRUCTURES.**

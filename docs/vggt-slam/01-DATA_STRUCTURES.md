# VGGT-SLAM 2.0 — Data Structures (数据结构详解)

> Every persistent piece of state in VGGT-SLAM lives in one of seven
> classes. This document enumerates each field, the type/shape it carries,
> the line that creates it and the line(s) that consume it. Field
> descriptions never go beyond what the code does.
>
> Repo root: `/home/steve/vslam_ws/src/VGGT-SLAM` (all `[VERIFY:]` paths
> are relative to it).

---

## Table of Contents

1. [`Submap`](#1-submap-vggt_slamsubmappy) — per-submap state container
2. [`GraphMap`](#2-graphmap-vggt_slammappy) — dict registry of submaps
3. [`PoseGraph`](#3-posegraph-vggt_slamgraphpy) — GTSAM SL(4) factor graph
4. [`Solver`](#4-solver-vggt_slamsolverpy) — top-level orchestrator
5. [`ImageRetrieval`](#5-imageretrieval-vggt_slamloop_closurepy) — SALAD retriever
6. [`LoopMatch` & `LoopMatchQueue`](#6-loopmatch--loopmatchqueue) — bounded heap of loop candidates
7. [`FrameTracker`](#7-frametracker-vggt_slamframe_overlappy) — keyframe selector
8. [`Viewer`](#8-viewer-vggt_slamviewerpy) — viser front-end state
9. [`predictions` dict](#9-predictions-dictionary) — VGGT output passed Solver→add_points
10. [`Accumulator`](#10-accumulator-vggt_slamslam_utilspy) — timing helper

---

## 1. `Submap` (`vggt_slam/submap.py`)

`Submap` is the central per-batch container. A submap holds **everything
the network predicted plus everything we re-derived from those
predictions for one batch of keyframes**.

### 1.1 Constructor (initial values)

[VERIFY: vggt_slam/submap.py:11-28]
```python
class Submap:
    def __init__(self, submap_id):
        self.submap_id = submap_id
        self.R_world_map = None
        self.poses = None
        self.frames = None
        self.proj_mats = None
        self.retrieval_vectors = None
        self.colors = None              # (S, H, W, 3)
        self.conf = None                # (S, H, W)
        self.conf_masks = None          # (S, H, W)
        self.conf_threshold = None
        self.pointclouds = None         # (S, H, W, 3)
        self.voxelized_points = None
        self.last_non_loop_frame_index = None
        self.frame_ids = None
        self.is_lc_submap = False
        self.img_names = []
        self.semantic_vectors = []
```

### 1.2 Field reference

| Field                       | Type / shape                                       | Created at                                                | Consumed at                                                                                          | Meaning                                                                                                                                            |
| --------------------------- | -------------------------------------------------- | --------------------------------------------------------- | --------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| `submap_id`                 | `int`                                              | `__init__` [VERIFY: vggt_slam/submap.py:12]              | `get_id()` everywhere; node-key arithmetic in `Solver.add_edge`                                     | Integer ID, equal to the GTSAM key of the submap's **first** pose.                                                                                |
| `R_world_map`               | unused (`None` everywhere)                          | [VERIFY: vggt_slam/submap.py:13]                          | —                                                                                                    | Declared in `__init__` but never assigned/read anywhere in `vggt_slam/*.py`. Dead field.                                                            |
| `poses`                     | `np.ndarray (S, 4, 4)` — world-to-cam SE(3)         | `add_all_poses` [VERIFY: vggt_slam/submap.py:33-34]      | `get_all_poses` [VERIFY: vggt_slam/submap.py:111], inside `Solver.add_edge` loop                    | The raw VGGT poses (no global alignment yet). Used to derive the inner-submap relative homographies.                                              |
| `frames`                    | `torch.Tensor (S, 3, H, W)` (kept on GPU)           | `add_all_frames` [VERIFY: vggt_slam/submap.py:46-47]      | `get_frame_at_index` [VERIFY: vggt_slam/submap.py:64], retrieval, semantic, viewer                  | The actual preprocessed image tensor returned by `load_and_preprocess_images`. Used to compute SALAD/CLIP embeddings and for re-running VGGT on lc. |
| `proj_mats`                 | `np.ndarray (S, 4, 4)` — `K_4x4`                    | `add_all_points(..., intrinsics_inv)` [VERIFY: vggt_slam/submap.py:36-41] | `get_all_poses_world` [VERIFY: vggt_slam/submap.py:118], `Solver.add_edge` [VERIFY: vggt_slam/solver.py:140] | Per-frame 4×4 intrinsic, embedded as `[[K, 0],[0, 1]]`. Used to convert SL(4) world poses back into 3×4 projection matrices for decomposition.    |
| `retrieval_vectors`         | `torch.Tensor (S, D)` (D from SALAD)                | `set_all_retrieval_vectors` [VERIFY: vggt_slam/submap.py:154-155] | `find_loop_closures` [VERIFY: vggt_slam/loop_closure.py:80]                                          | SALAD/DINO descriptors used for place recognition.                                                                                                |
| `colors`                    | `np.ndarray (S, H, W, 3) uint8`                     | `add_all_points` [VERIFY: vggt_slam/submap.py:36-38]      | `get_points_in_world_frame` (color filtering) [VERIFY: vggt_slam/submap.py:241]                     | Per-pixel RGB, derived from `images.transpose(0,2,3,1)*255` in Solver. Used for the visualised point cloud.                                       |
| `conf`                      | `np.ndarray (S, H, W)`                              | `add_all_points` [VERIFY: vggt_slam/submap.py:39]         | `filter_data_by_confidence` [VERIFY: vggt_slam/submap.py:170-172], `conf_threshold` calc           | Raw per-pixel confidence from VGGT's depth head.                                                                                                  |
| `conf_threshold`            | `float`                                             | `add_all_points` [VERIFY: vggt_slam/submap.py:40]         | `filter_data_by_confidence`, `get_conf_threshold`                                                   | `np.percentile(conf, init_conf_threshold) + 1e-6`. Lower-N percent of points get rejected.                                                        |
| `conf_masks`                | `np.ndarray (S, H, W)`                              | `set_conf_masks(conf)` [VERIFY: vggt_slam/submap.py:157-158] | `get_conf_masks_frame` [VERIFY: vggt_slam/submap.py:61-62], `Solver.add_edge` scale step           | **Identical contents to `conf`** at the time of `add_points` (see [§1.4](#14-conf-vs-conf_masks-the-redundancy)), but compared against `conf_threshold` *later* (after rectification). |
| `pointclouds`               | `np.ndarray (S, H, W, 3) float32`                   | `add_all_points` [VERIFY: vggt_slam/submap.py:37]         | `get_frame_pointcloud` [VERIFY: vggt_slam/submap.py:131-132], `get_points_in_world_frame`            | The unprojected 3-D point map in **the submap's local frame** (the frame of camera 0 after `tranform_submap_to_canonical`, although that helper is not actually called in `add_points` — see §1.5). |
| `voxelized_points`          | `o3d.geometry.PointCloud` or `None`                 | Lazy in `get_voxel_points_in_world_frame` [VERIFY: vggt_slam/submap.py:212-228] | Same method                                                                                          | Cached voxel-downsampled cloud after `voxel_down_sample` + `remove_radius_outlier`.                                                                |
| `last_non_loop_frame_index` | `int`                                               | `set_last_non_loop_frame_index` [VERIFY: vggt_slam/submap.py:151-152] | Node-key arithmetic in `Solver`                                                                      | Index of the last *non-overlap* frame inside the submap; equals `images.shape[0] - 1` for a build submap [VERIFY: vggt_slam/solver.py:320] and `1` for a loop submap [VERIFY: vggt_slam/solver.py:276]. |
| `frame_ids`                 | `list[float]`                                       | `set_frame_ids` [VERIFY: vggt_slam/submap.py:134-149]     | `GraphMap.write_poses_to_file` [VERIFY: vggt_slam/map.py:145]                                       | Numerical timestamp extracted from each image filename by regex `\d+(?:\.\d+)?`. Crucial for `evo` evaluation.                                    |
| `is_lc_submap`              | `bool`                                              | `set_lc_status` [VERIFY: vggt_slam/submap.py:30-31]       | `GraphMap.add_submap` [VERIFY: vggt_slam/map.py:21-22], filtering everywhere                       | `True` when this submap was created **only** to hold a loop-closure pair (`[query_kf, retrieved_kf]`).                                              |
| `img_names`                 | `list[str]`                                         | `set_img_names` [VERIFY: vggt_slam/submap.py:43-44]       | `get_img_names_at_index` [VERIFY: vggt_slam/submap.py:70-71]                                       | Original image paths, used for diagnostic prints and the loop-closure name pair.                                                                  |
| `semantic_vectors`          | `np.ndarray (S, D_clip) float`                      | `set_all_semantic_vectors` [VERIFY: vggt_slam/submap.py:160-161] | `retrieve_best_semantic_frame` [VERIFY: vggt_slam/map.py:52-53]                                     | Optional CLIP embeddings (Perception-Encoder L14-336), only populated with `--run_os`.                                                            |

### 1.3 Invariants

* `len(poses) == len(frames) == len(pointclouds) == len(conf) == len(proj_mats)`.
  All five are populated in lockstep by `add_all_poses`/`add_all_points`
  ([VERIFY: vggt_slam/submap.py:33-41]) and consumed by index in
  `get_all_poses_world` ([VERIFY: vggt_slam/submap.py:114-129]).
* For *build* submaps, `frames.shape[0] == submap_size + overlapping_window_size`
  ([VERIFY: main.py:118]).
* For *loop-closure* submaps, `frames.shape[0] == 2`
  ([VERIFY: vggt_slam/solver.py:272-280]).
* `is_lc_submap == False` ⇒ this submap participates in
  `non_lc_submap_ids` and the trajectory output
  ([VERIFY: vggt_slam/map.py:21-22]).

### 1.4 `conf` vs. `conf_masks` — the redundancy

Both fields are populated in `Solver.add_points`:

```python
# [VERIFY: vggt_slam/solver.py:247-248]
self.current_working_submap.add_all_points(world_points, colors, conf, self.init_conf_threshold, K_4x4)
self.current_working_submap.set_conf_masks(conf)
```

At creation time **they are identical**, but they are used differently:

* `conf` is used to compute `conf_threshold` once, at the start
  ([VERIFY: vggt_slam/submap.py:40]).
* `conf_masks` is what every downstream function *thresholds against*
  (`get_points_in_world_frame` [VERIFY: vggt_slam/submap.py:202],
  `Solver.add_edge` overlap-scale logic
  [VERIFY: vggt_slam/solver.py:129-132]).

The dual storage exists so that another pass could in principle overwrite
`conf_masks` with rectified/auto-cal confidences while leaving `conf`
intact for percentile recomputation. In the released code no such
overwrite happens; the two arrays remain identical for a build submap's
lifetime.

### 1.5 The "first-camera-is-identity" canonicalisation

There is a helper that *would* transform a submap into a canonical
frame where camera-0 is `[I | 0]`:

[VERIFY: vggt_slam/solver.py:101-116]
```python
def tranform_submap_to_canonical(self, proj_mat_world_to_cam, world_points):
    P_first_cam = proj_mat_world_to_cam[0].copy()
    proj_mat_world_to_cam = proj_mat_world_to_cam @ np.linalg.inv(P_first_cam)
    ...
```

**This helper is defined but never called** anywhere in
`vggt_slam/*.py` (verified by `grep -rn tranform_submap_to_canonical`).
The pointclouds stored in `Submap.pointclouds` are therefore in
VGGT's raw world frame for the submap. The canonicalisation that
matters is in `add_edge`, where the *between* factor uses
`np.linalg.inv(prev_submap.proj_mats[-1]) @ curr_submap.proj_mats[0]`
([VERIFY: vggt_slam/solver.py:140,153]) — i.e. canonicalisation is
done *implicitly* via the overlap pair, not by mutating the stored
cloud.

---

## 2. `GraphMap` (`vggt_slam/map.py`)

A thin, ordered registry around `submaps : dict[int, Submap]`.

### 2.1 Fields

[VERIFY: vggt_slam/map.py:9-13]
```python
class GraphMap:
    def __init__(self):
        self.submaps = dict()
        self.rectifying_H_mats = []
        self.non_lc_submap_ids = []
```

| Field                | Type                           | Created                                                                 | Consumed                                                                                                          | Meaning                                                                                                                            |
| -------------------- | ------------------------------ | ----------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| `submaps`            | `dict[int, Submap]`            | `add_submap` [VERIFY: vggt_slam/map.py:18-20]                          | `get_submap`, `get_largest_key`, `ordered_submaps_by_key`, etc.                                                  | All submaps in the run, keyed by `submap.get_id()`.                                                                                |
| `rectifying_H_mats`  | `list`                         | Declared empty; never appended in the released code (`grep` confirms).  | `write_poses_to_file` length check [VERIFY: vggt_slam/map.py:138-139], `save_framewise_pointclouds` length assertion [VERIFY: vggt_slam/map.py:175] | Reserved for an "auto-calibration" / metric-rectification pass that is not exercised by `main.py`. Always empty in the released pipeline. |
| `non_lc_submap_ids`  | `list[int]`                    | `add_submap` (only if `not is_lc_submap`) [VERIFY: vggt_slam/map.py:21-22] | `retrieve_best_score_frame` [VERIFY: vggt_slam/map.py:78]                                                       | Cached "exclude the current submap when looking for loops" list.                                                                  |

### 2.2 Two retrieval helpers

* `retrieve_best_score_frame(query, current_id, ignore_last_submap=True)`
  ([VERIFY: vggt_slam/map.py:68-102]):
  iterates over every non-LC submap whose key is not the current one
  and (optionally) not the previous one, computes
  `||embedding - query||` for every stored frame, returns the
  argmin and its score. Used inside `find_loop_closures`.
* `retrieve_best_semantic_frame(query_text_vector)`
  ([VERIFY: vggt_slam/map.py:42-66]): the open-set search variant.
  Uses cosine similarity over CLIP embeddings, argmax instead of
  argmin.

### 2.3 IO helpers

Three IO functions live on `GraphMap`:

* `write_poses_to_file(file_name, graph, kitti_format=False)`
  ([VERIFY: vggt_slam/map.py:134-162]): writes one line per non-LC
  frame: either KITTI-style flattened 3×4 or TUM-style
  `tstamp x y z qx qy qz qw`. Calls `decompose_camera` to recover
  R,t from the projective camera matrix.
* `save_framewise_pointclouds(graph, file_name)`
  ([VERIFY: vggt_slam/map.py:164-175]): saves per-frame
  `.npz` files containing `pointcloud` and confidence `mask`.
  **Bug warning**: there is a dead `count` increment under a
  conditional `continue`
  ([VERIFY: vggt_slam/map.py:168-170]) so the trailing assertion
  `count == len(self.rectifying_H_mats)` only succeeds when both
  are 0 (i.e. no LC submaps exist and `rectifying_H_mats` was never
  populated). This is consistent with the "rectifying mats are
  unused" comment in §2.1.
* `write_points_to_file(graph, file_name)`
  ([VERIFY: vggt_slam/map.py:178-192]): merged PCD across all
  submaps. Called from a commented-out line in `main.py`
  ([VERIFY: main.py:221]).

---

## 3. `PoseGraph` (`vggt_slam/graph.py`)

A wrapper around two GTSAM objects plus a couple of `noiseModel`s.

### 3.1 Fields

[VERIFY: vggt_slam/graph.py:14-27]
```python
class PoseGraph:
    def __init__(self):
        self.graph = NonlinearFactorGraph()
        self.values = Values()
        inner_noise = 0.05*np.ones(15, dtype=float)
        intra_noise = 0.05*np.ones(15, dtype=float)
        self.inner_submap_noise = noiseModel.Diagonal.Sigmas(inner_noise)
        self.intra_submap_noise = noiseModel.Diagonal.Sigmas(intra_noise)
        self.anchor_noise = noiseModel.Diagonal.Sigmas([1e-6] * 15)
        self.initialized_nodes = set()
        self.num_loop_closures = 0
        self.auto_cal_H_mats = dict()
```

| Field                  | Type                              | Notes                                                                                                                                                                                                                                                                                                            |
| ---------------------- | --------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `graph`                | `gtsam.NonlinearFactorGraph`      | Holds the `PriorFactorSL4` and `BetweenFactorSL4` factors. [VERIFY: vggt_slam/graph.py:17]                                                                                                                                                                                                                       |
| `values`               | `gtsam.Values`                    | Holds the current best estimate of every `SL4` node. Replaced wholesale after `optimize()` [VERIFY: vggt_slam/graph.py:127].                                                                                                                                                                                     |
| `inner_submap_noise`   | `noiseModel.Diagonal (15)`        | σ=0.05 isotropic. Applied to between factors that connect successive frames *inside* a build submap [VERIFY: vggt_slam/solver.py:191].                                                                                                                                                                          |
| `intra_submap_noise`   | `noiseModel.Diagonal (15)`        | σ=0.05 isotropic. Applied to between factors that connect the *overlap* frame of two adjacent submaps and the between factor of a loop closure [VERIFY: vggt_slam/solver.py:161].                                                                                                                                |
| `anchor_noise`         | `noiseModel.Diagonal (15)`        | σ=1e-6. Used by the single `add_prior_factor` call that pins the very first node [VERIFY: vggt_slam/solver.py:170].                                                                                                                                                                                              |
| `initialized_nodes`    | `set[gtsam.Key]`                  | Mirrors `values.keys()` but is used as a fast membership test inside `add_homography` and `add_between_factor` [VERIFY: vggt_slam/graph.py:34-46].                                                                                                                                                               |
| `num_loop_closures`    | `int`                             | Incremented from `Solver.run_predictions` when a loop survives the `image_match_ratio` check [VERIFY: vggt_slam/solver.py:377]. Exposed via `get_num_loops()` for the final report [VERIFY: main.py:164].                                                                                                       |
| `auto_cal_H_mats`      | `dict[int, np.ndarray]`           | Optional rectification homographies applied at `get_homography` time. Filled by `update_all_homographies` [VERIFY: vggt_slam/graph.py:143-151]. **Never called from `main.py`/`solver.py`**; placeholder for an off-line rectification step. `get_homography` falls back to identity when the key is absent. |

### 3.2 The three factor types

```python
# [VERIFY: vggt_slam/graph.py:9]
from gtsam import SL4, PriorFactorSL4, BetweenFactorSL4
```

* **`SL4`** — Lie-group representation of a 4×4 matrix with `det = 1`.
  Tangent dimension 15. All "Sigmas of length 15" in the constructor
  encode the dim of `sl(4)` (i.e. trace-zero 4×4 matrices).
* **`PriorFactorSL4(X(k), SL4(H), anchor_noise)`** — adds a prior on
  a single key. Used exactly once to anchor key 0 to identity
  ([VERIFY: vggt_slam/solver.py:169-170]).
* **`BetweenFactorSL4(X(k1), X(k2), SL4(H), noise)`** — adds a relative
  homography constraint between two keys; the residual is the SL(4)
  logarithm of `H_meas · (H_2 · H_1^{-1})^{-1}` (handled internally
  by GTSAM).

### 3.3 `get_homography` and the optional rectification

[VERIFY: vggt_slam/graph.py:56-67]
```python
def get_homography(self, node_id):
    auto_cal_H = np.eye(4)
    if node_id in self.auto_cal_H_mats:
        auto_cal_H = self.auto_cal_H_mats[node_id]
    node_id = X(node_id)
    return auto_cal_H @ self.values.atSL4(node_id).matrix()
```

This is the **only** path that ever reads a node back out of the graph.
Every downstream call (`Submap.get_all_poses_world`,
`GraphMap.write_poses_to_file`, the viewer) reads through this method,
so any future "rectification" step only has to populate
`auto_cal_H_mats` to take effect.

### 3.4 Known bug

[VERIFY: vggt_slam/graph.py:69-77]
```python
def get_projection_matrix(self, node_id):
    homography = self.get_homography(node_id)
    projection_matrix = np.linalg.inv(homography)
    return projection_matri        # ← TYPO: missing 'x'
```

`projection_matri` is undefined; this function will raise `NameError`
the first time it is called. It is consumed by
`Submap.get_last_pose_world` ([VERIFY: vggt_slam/submap.py:107]), which
in turn is **not called anywhere in the released main loop** (verified
by `grep get_last_pose_world`). So the typo is latent — it bites only
if downstream code starts using that method.

---

## 4. `Solver` (`vggt_slam/solver.py`)

The top-level state container. Every other module is reachable from it.

### 4.1 Fields

[VERIFY: vggt_slam/solver.py:37-60]
```python
class Solver:
    def __init__(self, init_conf_threshold, lc_thres=0.80, vis_voxel_size=None):
        self.init_conf_threshold = init_conf_threshold
        self.vis_voxel_size = vis_voxel_size

        self.viewer = Viewer()
        self.flow_tracker = FrameTracker()
        self.map = GraphMap()
        self.graph = PoseGraph()
        self.image_retrieval = ImageRetrieval()
        self.current_working_submap = None

        self.lc_thres = lc_thres
        self.temp_count = 0
        self.vggt_timer = Accumulator()
        self.loop_closure_timer = Accumulator()
        self.clip_timer = Accumulator()
```

| Field                     | Type                | Notes                                                                                                                                                  |
| ------------------------- | ------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `init_conf_threshold`     | `float`             | A percentile (e.g. `25` = drop the bottom 25 % of pixels by confidence). Passed straight to `Submap.add_all_points` [VERIFY: vggt_slam/solver.py:247]. |
| `vis_voxel_size`          | `float \| None`     | If set, downsample the cloud sent to viser via `o3d.PointCloud.voxel_down_sample` [VERIFY: vggt_slam/solver.py:62-69].                                 |
| `viewer`                  | `Viewer`            | Owns the running viser server (port 8080) and the per-submap handles.                                                                                  |
| `flow_tracker`            | `FrameTracker`      | The optical-flow keyframer; used only inside `main.py`'s per-image loop [VERIFY: main.py:110].                                                         |
| `map`                     | `GraphMap`          | The dict of submaps.                                                                                                                                   |
| `graph`                   | `PoseGraph`         | The GTSAM SL(4) factor graph.                                                                                                                          |
| `image_retrieval`         | `ImageRetrieval`    | SALAD-based loop detector.                                                                                                                              |
| `current_working_submap`  | `Submap \| None`    | The submap currently being filled inside `run_predictions`/`add_points`. Set in [VERIFY: vggt_slam/solver.py:329]; consumed in [VERIFY: vggt_slam/solver.py:236]. |
| `lc_thres`                | `float`             | Threshold on **distance** (i.e. `||a - b||`) for retrieval acceptance. Despite the name, *lower* is more similar; the helper compares with `<` [VERIFY: vggt_slam/loop_closure.py:82]. The default of 0.95 is unusually permissive — see `09-KEY_QUESTIONS.md`. |
| `temp_count`              | `int`               | Declared but never read or incremented anywhere (`grep temp_count`). Dead.                                                                              |
| `vggt_timer`              | `Accumulator`       | Times `load_and_preprocess_images` + every `model(images)` call.                                                                                       |
| `loop_closure_timer`      | `Accumulator`       | Times the `find_loop_closures` call (excludes the second VGGT call).                                                                                   |
| `clip_timer`              | `Accumulator`       | Times CLIP embedding pass (only nonzero with `--run_os`).                                                                                              |

### 4.2 What lives in `current_working_submap` between `run_predictions` and `add_points`

`run_predictions` fills five fields on the working submap before returning:

```python
# [VERIFY: vggt_slam/solver.py:317-322]
new_submap = Submap(new_pcd_num)
new_submap.add_all_frames(images)
new_submap.set_frame_ids(image_names)
new_submap.set_last_non_loop_frame_index(images.shape[0] - 1)
new_submap.set_all_retrieval_vectors(self.image_retrieval.get_all_submap_embeddings(new_submap))
new_submap.set_img_names(image_names)
```

`add_points` then fills the remaining four (`poses`, `pointclouds`,
`colors`, `conf`, `conf_masks`, `proj_mats`):

```python
# [VERIFY: vggt_slam/solver.py:246-248]
self.current_working_submap.add_all_poses(world_to_cam)
self.current_working_submap.add_all_points(world_points, colors, conf, self.init_conf_threshold, K_4x4)
self.current_working_submap.set_conf_masks(conf)
```

Only **after** these two stages is the submap eligible to be added to
the map ([VERIFY: vggt_slam/solver.py:249]).

---

## 5. `ImageRetrieval` (`vggt_slam/loop_closure.py`)

### 5.1 Fields

[VERIFY: vggt_slam/loop_closure.py:52-58]
```python
class ImageRetrieval:
    def __init__(self, input_size=224):
        ckpt_pth = os.path.join(torch.hub.get_dir(), "checkpoints/dino_salad.ckpt")
        self.model = load_model(ckpt_pth)
        self.model.eval()
        self.transform = input_transform((input_size, input_size))
```

| Field        | Type            | Meaning                                                                                                                              |
| ------------ | --------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| `model`      | `torch.nn.Module` | The DINO+SALAD pretrained model loaded by `salad.eval.load_model`. Inference-only (`eval()`).                                       |
| `transform`  | `torchvision.transforms.Compose` | `Resize(input_size) → ToTensor → Normalize` with ImageNet statistics. Applied per-frame before being stacked and fed to the model. |

### 5.2 Three callable entry points

[VERIFY: vggt_slam/loop_closure.py:60-86]
* `get_single_embeding(cv_img)` — one image at a time.
* `get_batch_descriptors(imgs)` — `(B, C, H, W)` tensor, returns `(B, D)`.
* `get_all_submap_embeddings(submap)` — calls `get_batch_descriptors`
  on `submap.get_all_frames()`.
* `find_loop_closures(map, submap, max_similarity_thres, max_loop_closures)`
  — calls `map.retrieve_best_score_frame` for every embedding in the
  new submap, builds a bounded `LoopMatchQueue`, returns the matches.

---

## 6. `LoopMatch` & `LoopMatchQueue`

### 6.1 `LoopMatch`

[VERIFY: vggt_slam/loop_closure.py:25-30]
```python
class LoopMatch(NamedTuple):
    similarity_score: float
    query_submap_id: int
    query_submap_frame: int
    detected_submap_id: int
    detected_submap_frame: int
```

All five fields are read in `Solver.add_points` when handling
detected loops ([VERIFY: vggt_slam/solver.py:255-287]).

### 6.2 `LoopMatchQueue` — capacity-bounded heap

[VERIFY: vggt_slam/loop_closure.py:32-49]

`max_size` is set from `Solver.run_predictions(... max_loops=args.max_loops)`
([VERIFY: vggt_slam/solver.py:341]). The queue is implemented as a
**min-heap of negated similarity scores**, so the heap's root is the
*least* similar surviving match; `heappushpop` evicts it when a more
similar one arrives:

```python
item = (-match.similarity_score, match)
if len(self.heap) < self.max_size:
    heapq.heappush(self.heap, item)
else:
    heapq.heappushpop(self.heap, item)
```

The README states: *"Only default of 1 supported right now or 0 to
disable loop closures"* ([VERIFY: main.py:30]). With `max_loops=1`
the heap has at most one element, so `LoopMatchQueue` degenerates into
"keep the single best loop".

There is a subtle naming inversion: SALAD `||a - b||` is a *distance*
(lower = better), but the heap is labelled `similarity_score`. Inside
`find_loop_closures` the value stored is the raw distance
([VERIFY: vggt_slam/loop_closure.py:81-83]):

```python
best_score, best_submap_id, best_frame_id = map.retrieve_best_score_frame(query_vector, ...)
if best_score < max_similarity_thres:
    new_match_data = LoopMatch(best_score, ...)
```

Because the heap is `(-similarity_score, match)`, "best" loop = lowest
distance = most negative `-score` = root of the min-heap. So with
`max_size=1` the heap correctly keeps the *closest* (best) match.
With `max_size > 1` the eviction rule prefers small *distances* (which
are also small `-score`s with that variable used as `similarity_score`),
so the heap retains the loops with the smallest distance — i.e. the
ones most likely to be true positives. The variable name is misleading
but the behaviour is what you want.

---

## 7. `FrameTracker` (`vggt_slam/frame_overlap.py`)

[VERIFY: vggt_slam/frame_overlap.py:6-22]
```python
class FrameTracker:
    def __init__(self):
        self.last_kf = None
        self.kf_pts = None
        self.kf_gray = None

    def initialize_keyframe(self, image):
        self.last_kf  = image
        self.kf_gray  = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.kf_pts   = cv2.goodFeaturesToTrack(
            self.kf_gray, maxCorners=1000, qualityLevel=0.01,
            minDistance=8, blockSize=7
        )
```

| Field      | Type                              | Meaning                                                                                |
| ---------- | --------------------------------- | -------------------------------------------------------------------------------------- |
| `last_kf`  | `np.ndarray (H, W, 3) uint8`      | The full BGR image of the previous keyframe (kept for visualisation).                  |
| `kf_gray`  | `np.ndarray (H, W) uint8`         | Greyscale version, fed to `calcOpticalFlowPyrLK`.                                      |
| `kf_pts`   | `np.ndarray (N, 1, 2) float32`    | Shi-Tomasi corners detected once per keyframe. `N ≤ maxCorners=1000`.                  |

There is one decision rule:

> Re-keyframe iff `mean ||next_pts - kf_pts|| > min_disparity` over the
> successfully tracked corners.

When the criterion fires, the tracker overwrites itself with the new
image (`initialize_keyframe`), and `compute_disparity` returns `True`
([VERIFY: vggt_slam/frame_overlap.py:58-60]). When fewer than 10
corners survive the LK pass, the function does the same (re-init and
return `True`) — corner exhaustion is treated as a forced re-keyframe.

---

## 8. `Viewer` (`vggt_slam/viewer.py`)

[VERIFY: vggt_slam/viewer.py:9-31]

| Field                   | Type                                                    | Notes                                                                                                                                                |
| ----------------------- | ------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| `server`                | `viser.ViserServer`                                     | Started in the constructor on port 8080; lives for the entire process lifetime.                                                                       |
| `gui_show_frames`       | `viser.GuiHandle (checkbox)`                            | "Show Cameras" toggle. Triggers `_on_update_show_frames` to hide/show all frustums and frame triads in one go [VERIFY: vggt_slam/viewer.py:88-96].   |
| `btn_walkthrough`       | `viser.GuiHandle (button)`                              | "Play Walkthrough"; iterates over `submap_frames` in sorted ID order and animates the client camera ([VERIFY: vggt_slam/viewer.py:169-202]).        |
| `submap_frames`         | `Dict[int, List[viser.FrameHandle]]`                    | One list per submap, in the order they were added. **Overwritten** by `visualize_frames` if called twice for the same submap (loop-closure refresh). |
| `submap_frustums`       | `Dict[int, List[viser.CameraFrustumHandle]]`            | Parallel structure for the camera frustums (the small "pyramids" with the image overlaid).                                                            |
| `random_colors`         | `np.ndarray (250, 3) uint8`                             | Pre-sampled random colours; each submap is assigned the next available colour index in `submap_id_to_color` ([VERIFY: vggt_slam/viewer.py:43-46]).   |
| `submap_id_to_color`    | `dict[int, int]`                                        | Index into `random_colors` for stable per-submap colouring across redraws.                                                                            |
| `obj_id`                | `int`                                                   | Monotonically incremented OBB counter so multiple bounding boxes don't collide on the same scene name [VERIFY: vggt_slam/viewer.py:159-160].         |

---

## 9. `predictions` dictionary

The contract between `run_predictions` and `add_points` is an
explicitly documented dict. The docstring on `Solver.add_points`
states it:

[VERIFY: vggt_slam/solver.py:197-209]
```python
def add_points(self, pred_dict):
    """
    Args:
        pred_dict (dict):
        {
            "images": (S, 3, H, W)   - Input images,
            "world_points": (S, H, W, 3),
            "world_points_conf": (S, H, W),
            "depth": (S, H, W, 1),
            "depth_conf": (S, H, W),
            "extrinsic": (S, 3, 4),
            "intrinsic": (S, 3, 3),
        }
    """
```

Reality is richer than the docstring. After `run_predictions` returns,
the dict actually contains:

| Key                  | Shape / type                              | Producer                                                                  |
| -------------------- | ----------------------------------------- | ------------------------------------------------------------------------- |
| `pose_enc`           | `(S, 9)`                                  | VGGT camera head [VERIFY: third_party/vggt/vggt/models/vggt.py:68]        |
| `pose_enc_list`      | per-iteration list                        | VGGT camera head [VERIFY: third_party/vggt/vggt/models/vggt.py:69]        |
| `depth`              | `(S, H, W, 1)`                            | VGGT depth head                                                            |
| `depth_conf`         | `(S, H, W)`                               | VGGT depth head                                                            |
| `images`             | `(S, 3, H, W)` in `[0,1]`                 | Returned by VGGT during inference [VERIFY: third_party/vggt/vggt/models/vggt.py:94] |
| `target_tokens`      | tensor (left as `torch.Tensor`)           | VGGT aggregator [VERIFY: third_party/vggt/vggt/models/vggt.py:97]         |
| `image_match_ratio`  | `float` (per call)                        | VGGT aggregator                                                            |
| `extrinsic`          | `(S, 3, 4)` numpy                         | `pose_encoding_to_extri_intri` [VERIFY: vggt_slam/solver.py:364-366]      |
| `intrinsic`          | `(S, 3, 3)` numpy                         | Same                                                                       |
| `detected_loops`     | `list[LoopMatch]`                         | `find_loop_closures` [VERIFY: vggt_slam/solver.py:368]                     |
| `extrinsic_lc`       | `(2, 3, 4)` numpy (only if loop survives) | Second VGGT call on `[query, retrieved]` pair [VERIFY: vggt_slam/solver.py:378-379] |
| `intrinsic_lc`       | `(2, 3, 3)` numpy                          | Same                                                                       |
| `depth_lc`           | `(2, H, W, 1)` numpy                       | Same                                                                       |
| `depth_conf_lc`      | `(2, H, W)` numpy                          | Same                                                                       |
| `frames_lc`          | `(2, 3, H, W)` torch                       | Slice of `lc_frames` [VERIFY: vggt_slam/solver.py:390]                    |
| `frames_lc_names`    | `[str, str]`                               | Pair of original image paths [VERIFY: vggt_slam/solver.py:391]            |
| `world_points`       | **not produced** by VGGT in this code path  | The point-head branch is commented out in the upstream VGGT [VERIFY: third_party/vggt/vggt/models/vggt.py:78-83]. Solver reconstructs world points from `depth + extrinsic + intrinsic` instead [VERIFY: vggt_slam/solver.py:222]. |

The post-VGGT loop at the bottom of `run_predictions`
([VERIFY: vggt_slam/solver.py:385-387]) does:

```python
for key in predictions.keys():
    if isinstance(predictions[key], torch.Tensor) and key != "target_tokens":
        predictions[key] = predictions[key].float().cpu().numpy().squeeze(0)
```

So everything in the dict that was a tensor — *except* `target_tokens`
— is converted to a CPU numpy array with the leading batch dim
squeezed out. `target_tokens` is left on the GPU because it is only
consumed at the model level on subsequent calls; in this build of the
pipeline it is not actually read again.

---

## 10. `Accumulator` (`vggt_slam/slam_utils.py`)

[VERIFY: vggt_slam/slam_utils.py:204-213]
```python
class Accumulator:
    def __init__(self):
        self.total_time = 0

    def __enter__(self):
        self.start = time.perf_counter()
        return self

    def __exit__(self, *args):
        self.total_time += (time.perf_counter() - self.start)
```

A minimal context manager. Every `with self.vggt_timer:` block in
`Solver` and the `keyframe_time`/`backend_time` blocks in `main.py` go
through this class. Total time is printed at exit
([VERIFY: main.py:155-161]).

---

## 11. Dead / orphaned fields

For completeness, fields that are declared but neither written nor read
outside their declaration:

| Field                                    | Status                                                                                                                                 |
| ---------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| `Submap.R_world_map`                     | Never assigned, never read [VERIFY: vggt_slam/submap.py:13].                                                                          |
| `Solver.temp_count`                      | Never incremented or read [VERIFY: vggt_slam/solver.py:57].                                                                            |
| `GraphMap.rectifying_H_mats`             | Only read by an assertion that needs it to stay empty; never appended [VERIFY: vggt_slam/map.py:11].                                  |
| `PoseGraph.auto_cal_H_mats`              | Populated only by `update_all_homographies`, which is **not called** from `main.py`/`solver.py` (verified via `grep`). Inert. |
| `Submap.get_last_pose_world`            | Uses `graph.get_projection_matrix` which contains the `projection_matri` typo — bug is latent because the method is unused. |

These are mostly hooks for the (not-yet-merged) "auto-calibration"
post-process: in the released code path they're zeroed out and have
no effect on correctness.

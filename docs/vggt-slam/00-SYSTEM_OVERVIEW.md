# VGGT-SLAM 2.0 — System Overview (系统概述)

> **Scope**: This document is the entry point for the technical analysis of
> VGGT-SLAM 2.0. All claims are tagged `[VERIFY: file:line]` so the reader
> can jump from any statement back to the exact line of code that justifies
> it. No claim in this analysis exists without a corresponding code reference.
>
> **Repo root for all paths below**: `/home/steve/vslam_ws/src/VGGT-SLAM`.
> When a tag reads `[VERIFY: vggt_slam/solver.py:222]` the file is
> `<repo>/vggt_slam/solver.py` and the relevant code is on or near line 222.

---

## 1. What VGGT-SLAM is

VGGT-SLAM 2.0 is a **dense, monocular, RGB-only SLAM** system built on top
of the **VGGT** feed-forward 3D reconstruction transformer
[VERIFY: third_party/vggt/vggt/models/vggt.py:17]. Instead of running a
classical front-end (feature extraction + matching + triangulation), it
hands a small batch of images to a neural network that simultaneously
predicts:

* per-pixel depth maps and per-pixel confidence,
* a 9-vector camera-pose encoding per image
  ([VERIFY: third_party/vggt/vggt/models/vggt.py:68]),
* an "image match ratio" used to gate weak loop-closure pairs
  ([VERIFY: third_party/vggt/vggt/models/vggt.py:98],
   [VERIFY: vggt_slam/solver.py:371-374]),
* image retrieval tokens for the place-recognition front end
  ([VERIFY: third_party/vggt/vggt/models/vggt.py:97]).

Those raw predictions are turned into a globally consistent map by an
**SL(4) homography pose graph**: each keyframe is a node carrying a 4×4
homography with `det = 1` (a 15-DOF element of `SL(4)`), and edges are
relative homographies whose product is enforced to be the identity in the
nonlinear least-squares sense
([VERIFY: vggt_slam/graph.py:9],
 [VERIFY: vggt_slam/graph.py:47]).

This is the central design choice of the paper: because the monocular
feed-forward predictions are only correct **up to an unknown projective
ambiguity**, aligning them with SE(3) or Sim(3) is the wrong group. The
correct group is `SL(4)` — the 15-DOF Lie group of 4×4 matrices with
unit determinant. GTSAM's `SL4` / `PriorFactorSL4` / `BetweenFactorSL4`
factors are used as the optimisation back-end
([VERIFY: vggt_slam/graph.py:9-10]).

---

## 2. High-level architecture

```
                    ┌──────────────────────────────────────────────┐
                    │                     main.py                   │
                    │  argparse → device → load VGGT → image loop  │
                    └────────────────────────┬─────────────────────┘
                                             │
                  ┌──────────────────────────┼──────────────────────────┐
                  ▼                          ▼                          ▼
        ┌──────────────────┐       ┌──────────────────┐       ┌────────────────────┐
        │ FrameTracker     │       │ Solver           │       │ Viewer (viser)     │
        │ (frame_overlap)  │──img──│ - VGGT model     │──pcd──│ scene + frustums   │
        │ optical-flow kf  │       │ - SALAD retriever│       │ + walkthrough      │
        │ selection        │       │ - GraphMap       │       └────────────────────┘
        └──────────────────┘       │ - PoseGraph      │
                                    │ - Submap (curr.) │
                                    │ - ScaleSolver    │
                                    └────────┬─────────┘
                                             │
                       ┌─────────────────────┼─────────────────────┐
                       ▼                     ▼                     ▼
              ┌────────────────┐    ┌────────────────┐    ┌────────────────┐
              │ Submap         │    │ GraphMap       │    │ PoseGraph      │
              │ (submap.py)    │    │ (map.py)       │    │ (graph.py)     │
              │ frames, poses, │    │ dict[id]→Submap│    │ GTSAM NLFG     │
              │ pointcloud,    │    │ retrieval/IO   │    │ SL4 factors    │
              │ conf masks,    │    │ pose dump      │    │ LM optimizer   │
              │ retrieval emb  │    └────────────────┘    └────────────────┘
              └────────────────┘
```

* **`main.py`** is a thin orchestrator: argument parsing, loading the
  VGGT checkpoint from HuggingFace, looping over images, and asking the
  `Solver` to do everything else
  ([VERIFY: main.py:36-229]).

* **`Solver`** is the single object that owns every stateful piece of the
  pipeline: the visualiser, the keyframe selector, the map, the pose
  graph, the image-retrieval front end and the "currently being built"
  submap ([VERIFY: vggt_slam/solver.py:37-60]).

* **`FrameTracker`** decides which raw frames become keyframes by
  measuring Lucas–Kanade optical flow against the last keyframe — only
  frames with enough mean disparity make it into a submap
  ([VERIFY: vggt_slam/frame_overlap.py:23-60]).

* **`Submap`** is the unit of granularity: a fixed-size batch of
  keyframes plus all of VGGT's predictions for them
  ([VERIFY: vggt_slam/submap.py:10-28]).

* **`GraphMap`** is the registry of submaps — a dict keyed by integer
  ID, with helpers for retrieval, ordered iteration, and writing
  trajectories/point clouds to disk
  ([VERIFY: vggt_slam/map.py:9-23]).

* **`PoseGraph`** wraps GTSAM's `NonlinearFactorGraph` plus a `Values`
  container of `SL4` elements, exposing four operations: `add_homography`,
  `add_between_factor`, `add_prior_factor`, `optimize`
  ([VERIFY: vggt_slam/graph.py:14-128]).

* **`ImageRetrieval`** loads a Salad/DINO checkpoint and produces a
  per-frame retrieval descriptor; it also keeps the top-K loop matches
  in a bounded heap
  ([VERIFY: vggt_slam/loop_closure.py:32-49],
   [VERIFY: vggt_slam/loop_closure.py:52-86]).

* **`Viewer`** is a [viser](https://viser.studio) front-end with two
  GUI controls (a "Show Cameras" checkbox and a "Play Walkthrough"
  button) plus per-submap frustum/point-cloud handles
  ([VERIFY: vggt_slam/viewer.py:9-31]).

---

## 3. Module inventory

| Module                                     | LOC | Role                                                                                                | Verify                                                |
| ------------------------------------------ | --- | --------------------------------------------------------------------------------------------------- | ----------------------------------------------------- |
| `main.py`                                   | 229 | CLI entry point; image loop; periodic pose dump for crash-safety                                    | [VERIFY: main.py:1-229]                              |
| `vggt_slam/solver.py`                      | 393 | Orchestrates submap creation, VGGT inference, edge/loop addition                                    | [VERIFY: vggt_slam/solver.py:1-393]                  |
| `vggt_slam/submap.py`                      | 253 | Per-submap state: frames, poses, conf, point cloud, retrieval vectors                                | [VERIFY: vggt_slam/submap.py:1-253]                  |
| `vggt_slam/map.py`                         | 192 | Dict-of-submaps + retrieval helpers + pose/point-cloud IO                                            | [VERIFY: vggt_slam/map.py:1-192]                     |
| `vggt_slam/graph.py`                       | 151 | GTSAM SL(4) factor graph wrapper                                                                     | [VERIFY: vggt_slam/graph.py:1-151]                   |
| `vggt_slam/loop_closure.py`                |  86 | SALAD-based image retrieval + bounded loop-match heap                                                | [VERIFY: vggt_slam/loop_closure.py:1-86]             |
| `vggt_slam/scale_solver.py`                |  24 | Median-of-norm-ratios scale estimation between overlapping submaps                                   | [VERIFY: vggt_slam/scale_solver.py:15-24]            |
| `vggt_slam/slam_utils.py`                  | 213 | RQ camera decomposition, SL(4) normalisation, CLIP embeddings, OBB, time accumulator                | [VERIFY: vggt_slam/slam_utils.py:1-213]              |
| `vggt_slam/frame_overlap.py`               |  61 | Lucas-Kanade keyframe selector                                                                       | [VERIFY: vggt_slam/frame_overlap.py:1-61]            |
| `vggt_slam/viewer.py`                      | 202 | Viser server, per-submap frustums, walkthrough animation                                             | [VERIFY: vggt_slam/viewer.py:1-202]                  |
| `third_party/vggt/vggt/models/vggt.py`     | ~100| Feed-forward VGGT model (camera head + depth head + aggregator)                                      | [VERIFY: third_party/vggt/vggt/models/vggt.py:17-100]|
| `third_party/salad`                         |  —  | Place-recognition descriptor (DINO+SALAD), loaded by `ImageRetrieval`                                | [VERIFY: vggt_slam/loop_closure.py:10]               |
| `third_party/perception_models`             |  —  | Optional Perception-Encoder CLIP (only with `--run_os`)                                              | [VERIFY: main.py:55-67]                              |
| `third_party/sam3`                          |  —  | Optional SAM3 (only with `--run_os`, for open-set masks)                                             | [VERIFY: main.py:56-62]                              |

Total VGGT-SLAM (Python, excluding third-party): **1,804 lines** across 10
files (see `wc -l` over `main.py` and `vggt_slam/*.py`). The bulk of the
intellectual content is split between `solver.py` (orchestration + SL(4)
construction) and `graph.py` (GTSAM glue); the rest is short.

---

## 4. End-to-end frame loop

The complete sequence of operations for one raw image, as implemented in
`main.py`, is:

```
   raw image (cv2.imread)
        │
        ▼ FrameTracker.compute_disparity
        │     [VERIFY: main.py:108-113][VERIFY: vggt_slam/frame_overlap.py:23-60]
        │     │       LK optical flow vs. last kf;
        │     │       mean displacement > min_disparity → keyframe
        │     ▼
        ▼ buffer keyframe paths into image_names_subset
        │     [VERIFY: main.py:112]
        │
        ▼ when len(buffer) == submap_size + overlapping_window_size:
        │     [VERIFY: main.py:118]
        │
        ▼ Solver.run_predictions(image_names_subset, model, max_loops, …)
        │     [VERIFY: main.py:122][VERIFY: vggt_slam/solver.py:298-393]
        │     │     load_and_preprocess_images       ─▶ tensor (S,3,H,W)
        │     │     new Submap; embed retrieval vectors (SALAD)
        │     │     (optional) CLIP embeddings per frame
        │     │     VGGT(images)                     ─▶ pose_enc, depth, depth_conf,
        │     │                                          target_tokens, image_match_ratio
        │     │     find_loop_closures               ─▶ list[LoopMatch]
        │     │     if a loop is detected:
        │     │         build 2-frame [query, retrieved] tensor
        │     │         VGGT(lc_frames, compute_similarity=True)
        │     │     pose_encoding_to_extri_intri      ─▶ extrinsic/intrinsic
        │     │     drop loop closure if image_match_ratio < 0.85
        │     ▼     return predictions (numpy, on CPU)
        ▼
        ▼ Solver.add_points(predictions)
        │     [VERIFY: main.py:126][VERIFY: vggt_slam/solver.py:197-287]
        │     │     unproject_depth_map_to_point_map  ─▶ world_points (S,H,W,3)
        │     │     fill in current_working_submap (poses, points, conf)
        │     │     map.add_submap(current)
        │     │     add_edge(submap_id_curr, 0, submap_id_prev, frame_id_prev)
        │     │         · scale = median(|p2|/|p1|) over overlap frame
        │     │         · H_overlap_prior_overlap_curr = inv(prev_proj) · curr_proj · diag(s,s,s,1)
        │     │         · add SL4 between factor with `intra_submap_noise`
        │     │     for each inner pose i ≥ 1:
        │     │         H_inner = prev_pose · inv(curr_pose); add SL4 between factor
        │     │     if loop closures:
        │     │         create lc_submap (2 frames)
        │     │         add_edge(lc_submap, 0, query_submap, query_frame)         ← normal edge
        │     │         add_edge(loop.detected_submap, loop.detected_frame, lc, 1, is_loop_closure=True)
        ▼
        ▼ PoseGraph.optimize()
        │     [VERIFY: main.py:129][VERIFY: vggt_slam/graph.py:80-127]
        │     │     LevenbergMarquardtOptimizer over the SL(4) NLFG
        │     │     self.values = result
        ▼
        ▼ (optional) visualiser update
        │     [VERIFY: main.py:132-136]
        │
        ▼ torch.cuda.empty_cache(), periodic pose dump
              [VERIFY: main.py:141-148][VERIFY: main.py:100-105]
```

The loop has been deliberately instrumented for the GeoScan benchmark
machine (RTX 4060 Laptop, 8 GB): a per-submap `torch.cuda.empty_cache()`
plus a `_safe_dump_poses()` after every backend pass so a mid-run OOM still
leaves a partial trajectory on disk
([VERIFY: main.py:97-105], [VERIFY: main.py:141-148]). These two
additions are local modifications and do not appear in stock VGGT-SLAM
upstream.

---

## 5. Dependencies — what comes from where

The interesting boundaries are visible in `solver.py`:

```python
# [VERIFY: vggt_slam/solver.py:11-13]
from vggt.utils.geometry  import closed_form_inverse_se3, unproject_depth_map_to_point_map
from vggt.utils.pose_enc  import pose_encoding_to_extri_intri
from vggt.utils.load_fn   import load_and_preprocess_images
```

| Boundary       | Pulled in from                                  | Why                                                                                    |
| -------------- | ----------------------------------------------- | -------------------------------------------------------------------------------------- |
| Backbone net   | `vggt.models.vggt.VGGT`                         | Predicts `pose_enc`, `depth`, `depth_conf`, retrieval tokens, image-match ratio        |
| Geometry utils | `vggt.utils.geometry.*`                         | Closed-form SE(3) inverse; depth→world point unprojection                              |
| Pose decode    | `vggt.utils.pose_enc.pose_encoding_to_extri_intri` | Converts the 9-vec pose encoding into (extrinsic 3×4, intrinsic 3×3)                |
| Image loading  | `vggt.utils.load_fn.load_and_preprocess_images`  | Resize/letterbox to network input size                                                 |
| Place rec      | `salad.eval.load_model`                          | Loads `dino_salad.ckpt` from `torch.hub.get_dir()/checkpoints/`                        |
| Open-set (opt) | `core.vision_encoder.pe` + `sam3.*`              | Only loaded when `--run_os` is passed                                                  |
| Back-end       | `gtsam` (`SL4`, `PriorFactorSL4`, `BetweenFactorSL4`, `LevenbergMarquardtOptimizer`) | All optimisation. SL(4) integration landed in upstream GTSAM in Aug 2025 (see README News). |
| Viewer         | `viser==0.2.23`                                  | Browser-based 3D scene with frustums and walkthrough                                   |
| 3D ops         | `open3d`                                         | Voxel downsampling + radius outlier removal for visualisation                          |

Verified by `requirements.txt`
([VERIFY: requirements.txt:1-24]) and `setup.sh`
([VERIFY: setup.sh:10-36]).

---

## 6. Two execution paths: keyframe build vs. loop closure

There are two distinct VGGT invocations per submap:

* **Build path** — the network is called once on a batch of
  `submap_size + 1` images (the +1 is the *overlap* with the previous
  submap). The result populates the current submap's poses, depths and
  confidences ([VERIFY: vggt_slam/solver.py:335]).

* **Loop-closure path** — if SALAD reports a candidate match, the
  network is called *a second time* with `compute_similarity=True`,
  on a 2-image tensor `[query_keyframe, retrieved_keyframe]`. The
  network returns its own predictions plus an `image_match_ratio`
  used as a sanity check ([VERIFY: vggt_slam/solver.py:340-348],
  [VERIFY: vggt_slam/solver.py:371-374]). If the ratio is below
  **0.85**, the loop is silently dropped — the prediction is not
  trusted.

The second call is what makes the system *robust*: SALAD/DINO retrieval
can over-fire, so the system uses VGGT itself as the geometric verifier.

---

## 7. ID and node-key arithmetic (a subtle correctness invariant)

Every camera pose in the GTSAM graph is keyed by a single integer:

```python
# [VERIFY: vggt_slam/graph.py:33]
self.values.insert(X(key), SL4(global_h))
```

The integer is built by a *flat* numbering scheme where each submap
"owns" a contiguous range of integer keys:

* The first pose of submap `s` has key `s`.
* The i-th pose of submap `s` has key `s + i`.
* The next submap's id is `prev_largest + prev_last_non_loop_frame_idx + 1`
  so the ranges never overlap.

The relevant arithmetic appears in four places:

```python
# [VERIFY: vggt_slam/solver.py:158]
self.graph.add_homography(submap_id_curr + frame_id_curr, H_w_submap)

# [VERIFY: vggt_slam/solver.py:188]
self.graph.add_homography(submap_id_curr + index, current_node)

# [VERIFY: vggt_slam/solver.py:270]
lc_submap_num = self.map.get_largest_key() + self.map.get_latest_submap().get_last_non_loop_frame_index() + 1

# [VERIFY: vggt_slam/solver.py:313]
new_pcd_num = self.map.get_largest_key() + self.map.get_latest_submap().get_last_non_loop_frame_index() + 1
```

If `submap_size = 16` and `overlapping_window_size = 1`, each submap
contributes 17 keys; the first submap occupies `0..16`, the next
starts at `17`, and so on. Loop-closure submaps are interleaved on the
fly and are *not* counted when computing the next non-LC submap id
([VERIFY: vggt_slam/map.py:24-34]: `ignore_loop_closure_submaps`).

This is one of the easiest places to introduce a graph-construction
bug, so the analysis devotes a dedicated section (see
`04-ALGORITHM-pose_graph_construction.md`) to it.

---

## 8. Real-time properties (what the timing tells us)

`main.py` collects six timing accumulators and prints them at exit:

```python
# [VERIFY: main.py:154-161]
print("Total time for VGGT calls:", solver.vggt_timer.total_time)
print("Average VGGT time per frame:", solver.vggt_timer.total_time / image_count)
print("Average loop closure time per frame:", solver.loop_closure_timer.total_time / image_count)
print("Average keyframe selection time per frame:", keyframe_time.total_time / image_count)
print("Average backend time per frame:", backend_time.total_time / image_count)
print("Average semantic time per frame:", solver.clip_timer.total_time / image_count)
print("Average total time per frame:", total_time / image_count)
print("Average FPS:", 1 / average_fps)
```

These are populated by `Accumulator` context managers
([VERIFY: vggt_slam/slam_utils.py:204-213]). Locations where the
accumulators are used:

* `vggt_timer`  — both `load_and_preprocess_images` and `model(images)`
  ([VERIFY: vggt_slam/solver.py:301], [VERIFY: vggt_slam/solver.py:334]).
* `loop_closure_timer` — wraps `find_loop_closures`
  ([VERIFY: vggt_slam/solver.py:340]).
* `clip_timer` — wraps optional CLIP embedding pass
  ([VERIFY: vggt_slam/solver.py:324]).
* `keyframe_time` — wraps the LK keyframe selector
  ([VERIFY: main.py:108]).
* `backend_time` — wraps `solver.graph.optimize()`
  ([VERIFY: main.py:128-129]).

Two stages dominate runtime in practice: VGGT inference (proportional
to submap size and resolution) and the LM optimisation as the graph
grows. The optical-flow keyframer is intentionally trivial (CPU-only,
LK over ~1000 corners) so it is negligible.

---

## 9. Reading order for the rest of this analysis

1. `01-DATA_STRUCTURES.md` — every persistent field that survives a
   frame, with the file:line that creates and consumes it.
2. `02-DATA_FLOW.md` — the path of pixels, depth maps and homographies
   through the system.
3. `03-ALGORITHM-sl4_homography_alignment.md` — the actual SL(4) /
   scale-bridging maths (the heart of the paper).
4. `04-ALGORITHM-pose_graph_construction.md` — how nodes and between
   factors are wired up, including the loop-closure auxiliary submap.
5. `05-ALGORITHM-loop_closure.md` — SALAD retrieval, the bounded
   max-heap, and the VGGT-based geometric verifier.
6. `06-ALGORITHM-keyframe_selection.md` — Lucas-Kanade keyframer in
   `frame_overlap.py`.
7. `07-ALGORITHM-scale_solver.md` — the (deliberately simple) median
   ratio scale solver.
8. `08-KEY_FUNCTIONS.md` — line-by-line dissection of the five
   functions where the design lives or dies.
9. `09-KEY_QUESTIONS.md` — design rationale Q&A.

---

## 10. What this analysis is **not** about

* **The internals of the VGGT transformer.** We treat `VGGT(images)` as a
  black box whose contract is the dict it returns
  ([VERIFY: third_party/vggt/vggt/models/vggt.py:40-52]). The aggregator
  and the per-head DPT decoders are upstream work.
* **The internals of SALAD.** Place recognition is a single
  `self.model(imgs)` call ([VERIFY: vggt_slam/loop_closure.py:70]); the
  descriptor architecture is upstream.
* **GTSAM's SL(4) Lie-group integration.** We rely on it as a black
  box and only describe how factors are constructed and what the
  noise/anchor models look like.

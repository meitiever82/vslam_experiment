# Key Functions — Line-by-Line Analysis

> The five functions in which VGGT-SLAM's design lives or dies. For
> each one we walk through every meaningful line, paired with the
> exact code location.

---

## Table of Contents

1. [`Solver.run_predictions`](#1-solverrun_predictions) — VGGT orchestration
2. [`Solver.add_points`](#2-solveradd_points) — submap → graph
3. [`Solver.add_edge`](#3-solveradd_edge) — SL(4) factor wiring
4. [`PoseGraph.optimize`](#4-posegraphoptimize) — LM back-end
5. [`ImageRetrieval.find_loop_closures`](#5-imageretrievalfind_loop_closures) — top-K loop search
6. [`Submap.get_all_poses_world`](#6-submapget_all_poses_world) — read-time global pose recovery
7. [`FrameTracker.compute_disparity`](#7-frametrackercompute_disparity) — keyframer decision
8. [`GraphMap.write_poses_to_file`](#8-graphmapwrite_poses_to_file) — TUM/KITTI dump

---

## 1. `Solver.run_predictions`

[VERIFY: vggt_slam/solver.py:298-393]

### 1.1 Signature

```python
def run_predictions(self, image_names, model, max_loops, clip_model, clip_preprocess):
```

| Arg               | Type             | Source                                                                 |
| ----------------- | ---------------- | ---------------------------------------------------------------------- |
| `image_names`     | `list[str]`      | `main.py:122` — the current submap buffer plus 1 overlap frame         |
| `model`           | `VGGT`           | `main.py:72-78` — loaded from HuggingFace, on device, in bf16          |
| `max_loops`       | `int`            | `main.py:30` — CLI flag, default 1                                     |
| `clip_model`      | `pe.CLIP \| None`| `main.py:64-65, 68-70` — None unless `--run_os`                        |
| `clip_preprocess` | `T.Compose \| None`| same as above                                                        |

Returns: the `predictions` dict (see `01-DATA_STRUCTURES.md §9`).

### 1.2 Image loading

[VERIFY: vggt_slam/solver.py:299-304]
```python
device = "cuda" if torch.cuda.is_available() else "cpu"
t1 = time.time()
with self.vggt_timer:
    images = load_and_preprocess_images(image_names).to(device)
print(f"Loaded and preprocessed {len(image_names)} images in {time.time() - t1:.2f} seconds")
print(f"Preprocessed images shape: {images.shape}")
```

* `device` recomputed every call — cheap, no harm.
* `vggt_timer` covers I/O as well as model inference. That's a design
  choice: the I/O is structurally tied to the network step and only
  exists to feed it.
* `load_and_preprocess_images` is `mode="crop"` by default
  ([VERIFY: third_party/vggt/vggt/utils/load_fn.py:97]) — letterboxed
  to 518 longest side.

### 1.3 Dtype handling (dead path)

[VERIFY: vggt_slam/solver.py:307]
```python
dtype = torch.bfloat16 if torch.cuda.get_device_capability()[0] >= 8 else torch.float16
```

`dtype` is computed but **never used** — `grep dtype` confirms it's
local and discarded. The model itself was already cast to
`bf16` in `main.py:77`, so this is dead code. Note: this line would
crash on CPU because `torch.cuda.get_device_capability()` requires a
CUDA device — but the line is hit unconditionally, before any CPU
guard. On a CPU-only box this function will raise here.

### 1.4 New-submap id computation

[VERIFY: vggt_slam/solver.py:310-315]
```python
if self.map.get_largest_key() is None:
    new_pcd_num = 0
else:
    new_pcd_num = self.map.get_largest_key() + self.map.get_latest_submap().get_last_non_loop_frame_index() + 1

print(f"Creating new submap with id {new_pcd_num}")
```

The flat-numbering arithmetic discussed in
`04-ALGORITHM-pose_graph_construction.md §2`. Key invariant: this id
equals the GTSAM key of the submap's first frame.

### 1.5 Submap pre-population

[VERIFY: vggt_slam/solver.py:317-322]
```python
new_submap = Submap(new_pcd_num)
new_submap.add_all_frames(images)
new_submap.set_frame_ids(image_names)
new_submap.set_last_non_loop_frame_index(images.shape[0] - 1)
new_submap.set_all_retrieval_vectors(self.image_retrieval.get_all_submap_embeddings(new_submap))
new_submap.set_img_names(image_names)
```

* Frames live on GPU (the network input). `set_frame_ids` extracts a
  float timestamp via regex from each filename
  ([VERIFY: vggt_slam/submap.py:134-149]).
* `last_non_loop_frame_index = S - 1` because every frame is a "build"
  frame at this point — the overlap frame becomes the *previous*
  submap's last non-loop frame, not this one's. Wait — that's
  reversed. Let me re-check. For submap `s`:
  - Buffer reset on the previous iteration kept the previous submap's
    last keyframe at position 0 (the overlap frame).
  - New keyframes are appended at positions 1..S-1.
  - So position 0 *is* the overlap with the previous submap; position
    S-1 is the new "last" keyframe.
  - `last_non_loop_frame_index = S - 1` is correct.
* `get_all_submap_embeddings` runs the SALAD model on `submap.frames`,
  immediately ([VERIFY: vggt_slam/loop_closure.py:72-75]).

### 1.6 Optional CLIP embedding pass

[VERIFY: vggt_slam/solver.py:324-327]
```python
with self.clip_timer:
    if clip_model is not None and clip_preprocess is not None:
        image_embs = compute_image_embeddings(clip_model, clip_preprocess, image_names)
        new_submap.set_all_semantic_vectors(image_embs)
```

* Only fires when `--run_os` was passed.
* `compute_image_embeddings` ([VERIFY: vggt_slam/slam_utils.py:85-106])
  re-loads each image from disk with PIL (i.e. *not* the already-loaded
  GPU tensor), runs them through Perception-Encoder's CLIP-style
  `encode_image`, and L2-normalises.
* Resulting embeddings are stored on the submap for later text-query
  retrieval.

### 1.7 The build inference

[VERIFY: vggt_slam/solver.py:329-336]
```python
self.current_working_submap = new_submap
print(f"Created new submap in {time.time() - t1:.2f} seconds")

with torch.no_grad():
    t1 = time.time()
    with self.vggt_timer:
        predictions = model(images)
    print(f"VGGT model inference took {time.time() - t1:.2f} seconds")
```

* `self.current_working_submap` is the only piece of state that bridges
  `run_predictions` and `add_points`. It must be set before any code
  path that calls `add_points` is taken.
* `torch.no_grad()` matters: VGGT's encoder is large; gradient
  bookkeeping during inference would double the memory footprint.
* No batch dim is added — VGGT auto-unsqueezes
  ([VERIFY: third_party/vggt/vggt/models/vggt.py:55-56]).

### 1.8 Loop-closure detection & verifier

[VERIFY: vggt_slam/solver.py:339-361]
```python
predictions_lc = None
with self.loop_closure_timer:
    detected_loops = self.image_retrieval.find_loop_closures(
        self.map, new_submap, max_loop_closures=max_loops, max_similarity_thres=self.lc_thres)
loop_closure_frame_names = []
if len(detected_loops) > 0:
    print(colored("detected_loops", "yellow"), detected_loops)
    retrieved_frames = self.map.get_frames_from_loops(detected_loops)
    with torch.no_grad():
        lc_frames = torch.stack((new_submap.get_frame_at_index(detected_loops[0].query_submap_frame),
                                 retrieved_frames[0]), axis=0)
        predictions_lc = model(lc_frames, compute_similarity=True)
        loop_closure_frame_names = [new_submap.get_img_names_at_index(detected_loops[0].query_submap_frame),
        self.map.get_submap(detected_loops[0].detected_submap_id).get_img_names_at_index(detected_loops[0].detected_submap_frame)]

    # Visualize loop closure frames
    if DEBUG:
        ...
```

* `find_loop_closures` is called *after* the build inference so SALAD
  has up-to-date retrieval vectors. The `loop_closure_timer` measures
  only the descriptor search, not the verifier VGGT call (which lands
  in `vggt_timer`).
* Only `detected_loops[0]` is verified — the rest are dropped silently
  even if SALAD returned them. With `max_loops=1` (the only supported
  value, [VERIFY: main.py:30]) this is correct.
* `loop_closure_frame_names` is a 2-list of original image paths,
  attached to the LC submap for diagnostic purposes.

### 1.9 Extrinsic/intrinsic decode

[VERIFY: vggt_slam/solver.py:363-366]
```python
print("Converting pose encoding to extrinsic and intrinsic matrices...")
extrinsic, intrinsic = pose_encoding_to_extri_intri(predictions["pose_enc"], images.shape[-2:])
predictions["extrinsic"] = extrinsic
predictions["intrinsic"] = intrinsic
```

The pose head emits a compact 9-vector per frame; this expands it to
the canonical (extrinsic 3×4, intrinsic 3×3) form used by Geometry.

### 1.10 LC verifier acceptance gate

[VERIFY: vggt_slam/solver.py:368-383]
```python
predictions["detected_loops"] = detected_loops

if predictions_lc is not None:
    image_match_ratio = predictions_lc["image_match_ratio"]
    if image_match_ratio < 0.85:
        print(colored("Loop closure image match ratio too low, skipping loop closure", "red"))
        predictions_lc = None
        predictions["detected_loops"] = []
    else:
        self.graph.increment_loop_closure()
        extrinsic_lc, intrinsic_lc = pose_encoding_to_extri_intri(predictions_lc["pose_enc"], retrieved_frames[0].shape[-2:])
        predictions["extrinsic_lc"] = extrinsic_lc
        predictions["intrinsic_lc"] = intrinsic_lc
        predictions["depth_lc"] = predictions_lc["depth"]
        predictions["depth_conf_lc"] = predictions_lc["depth_conf"]
```

Two-tiered gating:

* `len(detected_loops) > 0` — SALAD passed.
* `image_match_ratio ≥ 0.85` — VGGT verifier passed.

Only when both pass does `predictions` carry the `_lc` keys that
`add_points` will use to build the LC submap.

### 1.11 Tensor → numpy conversion

[VERIFY: vggt_slam/solver.py:385-387]
```python
for key in predictions.keys():
    if isinstance(predictions[key], torch.Tensor) and key != "target_tokens":
        predictions[key] = predictions[key].float().cpu().numpy().squeeze(0)
```

The whole dict is downgraded to CPU numpy except `target_tokens`
(left on GPU because it's a model-internal tensor that nothing reads
later). `squeeze(0)` drops the leading batch dim (`B=1` always at this
point).

### 1.12 LC frame attachment

[VERIFY: vggt_slam/solver.py:389-392]
```python
if predictions_lc is not None:
    predictions["frames_lc"] = lc_frames[0:2,...]
    print(loop_closure_frame_names)
    predictions["frames_lc_names"] = loop_closure_frame_names
```

Notice `lc_frames` is the GPU tensor — not the numpy form. `add_points`
later converts it on the fly when computing colours
([VERIFY: vggt_slam/solver.py:279]).

### 1.13 What can go wrong

* `images.shape[0] != args.submap_size + args.overlapping_window_size`
  for the final submap (short buffer). The function handles this
  transparently — `set_last_non_loop_frame_index(images.shape[0] - 1)`
  uses the actual size. The fixed-arity loops downstream
  (`add_edge` inner chain) iterate over `range(len(self.poses))` so
  they adapt.
* `find_loop_closures` returning > 1 match (if `max_loops > 1`):
  silently dropped. The function never verifies any but the first.

---

## 2. `Solver.add_points`

[VERIFY: vggt_slam/solver.py:197-287]

The "give the predictions to the system" entry point.

### 2.1 Signature & dict unpacking

[VERIFY: vggt_slam/solver.py:197-220]
```python
def add_points(self, pred_dict):
    ...
    images = pred_dict["images"]
    extrinsics_cam = pred_dict["extrinsic"]
    intrinsics_cam = pred_dict["intrinsic"]

    detected_loops = pred_dict["detected_loops"]

    depth_map = pred_dict["depth"]
    conf = pred_dict["depth_conf"]
```

Six keys read from `pred_dict`; one more (`detected_loops`) is read for
control flow. The function operates purely on the dict, not on
`self.current_working_submap`'s frames/embeddings (those were set in
`run_predictions`).

### 2.2 Geometry reconstruction

[VERIFY: vggt_slam/solver.py:222-232]
```python
world_points = unproject_depth_map_to_point_map(depth_map, extrinsics_cam, intrinsics_cam)

colors = (images.transpose(0, 2, 3, 1) * 255).astype(np.uint8)
cam_to_world = closed_form_inverse_se3(extrinsics_cam)
h, w = world_points.shape[1:3]

# Create projection matrices
N = cam_to_world.shape[0]
K_4x4 = np.tile(np.eye(4), (N, 1, 1))
K_4x4[:, :3, :3] = intrinsics_cam
world_to_cam = np.linalg.inv(cam_to_world)
```

Three computations:

1. **3-D points** in the submap's local frame via depth-unprojection
   (see `02-DATA_FLOW.md §6.3`).
2. **Per-pixel RGB** in `(S, H, W, 3) uint8`.
3. **Pose matrices** — `cam_to_world` (closed form) and `world_to_cam`
   (numerical inverse).
4. **K_4x4** — intrinsic embedded into 4×4 (see
   `03-ALGORITHM-sl4_homography_alignment.md §4`).

`h, w` are extracted but unused later in this function — vestigial.

### 2.3 Edge bookkeeping

[VERIFY: vggt_slam/solver.py:235-243]
```python
submap_id_prev = self.map.get_largest_key(ignore_loop_closure_submaps=True)
submap_id_curr = self.current_working_submap.get_id()
frame_id_curr = 0
frame_id_prev = None

first_edge = submap_id_prev is None

if not first_edge:
    frame_id_prev = self.map.get_latest_submap(ignore_loop_closure_submaps=True).get_last_non_loop_frame_index()
```

Three things to notice:

* `ignore_loop_closure_submaps=True` is used here, **unlike** in
  `run_predictions` where the unfiltered call is used. The filtering
  is essential here because we want to connect the current submap to
  the previous *build* submap, not a possibly-newer LC submap.
* `frame_id_curr = 0` — the new submap's overlap frame is always
  position 0.
* `frame_id_prev` is the previous build submap's last non-loop frame,
  i.e. its overlap frame on the other side. `last_non_loop_frame_index`
  was set in `run_predictions` ([VERIFY: vggt_slam/solver.py:320]).

### 2.4 Submap finalisation

[VERIFY: vggt_slam/solver.py:246-249]
```python
self.current_working_submap.add_all_poses(world_to_cam)
self.current_working_submap.add_all_points(world_points, colors, conf, self.init_conf_threshold, K_4x4)
self.current_working_submap.set_conf_masks(conf)
self.map.add_submap(self.current_working_submap)
```

Fills the four remaining fields and inserts the submap into the dict.
At this point the submap is "complete" — every subsequent function
can index into it correctly.

### 2.5 Edge construction (build submap)

[VERIFY: vggt_slam/solver.py:252]
```python
self.add_edge(submap_id_curr, frame_id_curr, submap_id_prev, frame_id_prev, is_loop_closure=False)
```

Single call. The branches inside `add_edge` handle "first submap vs.
not first submap" via `submap_id_prev is None`.

### 2.6 Loop-closure handling

[VERIFY: vggt_slam/solver.py:254-287] — the LC branch is exactly what
`04-ALGORITHM-pose_graph_construction.md §5` describes. The five
ingredients are:

1. `closed_form_inverse_se3(pred_dict["extrinsic_lc"])` to get
   `cam_to_world_lc`.
2. `K_4x4_lc` for the 2-frame batch.
3. `unproject_depth_map_to_point_map(depth_map_lc, ...)` for the
   `world_points_lc`.
4. A new `Submap(lc_submap_num)` flagged `is_lc_submap=True` and
   populated with all the `_lc` fields.
5. Two `add_edge` calls — see [§3](#3-solveradd_edge).

### 2.7 Critical interplay with `run_predictions`

If `run_predictions` returns *without* `predictions["detected_loops"]`
in the dict, `add_points` will raise `KeyError` on
[VERIFY: vggt_slam/solver.py:217]. So `run_predictions` *must* set the
key — and it always does, at [VERIFY: vggt_slam/solver.py:368], before
the LC gate. The empty list case is safe because
`for index, loop in enumerate(detected_loops)` iterates over nothing.

---

## 3. `Solver.add_edge`

[VERIFY: vggt_slam/solver.py:118-195]

Detailed branch-by-branch walk-through is in
`04-ALGORITHM-pose_graph_construction.md`. Here I'll focus on the
**six places** where this function makes a decision or transformation
that could be a source of bugs.

### 3.1 Assertion at entry

[VERIFY: vggt_slam/solver.py:119]
```python
assert not (is_loop_closure and submap_id_prev is None), "Loop closure must have a previous submap"
```

Catches the case where a caller flags an edge as a loop closure but
forgets the `submap_id_prev`. Such a call would otherwise try to
build a between factor against `None`.

### 3.2 Identity initialisation of `H_w_submap`

[VERIFY: vggt_slam/solver.py:122]
```python
H_w_submap = np.eye(4)
```

Initialised *outside* the branches so the first-edge case can fall
through and use it.

### 3.3 The three confidence-mask fallbacks

[VERIFY: vggt_slam/solver.py:131-138]

Already covered in `03-ALGORITHM-sl4_homography_alignment.md §6.1`. Worth
re-emphasising: the warning is printed in red via `termcolor`
([VERIFY: vggt_slam/solver.py:135]) which is the audible signal that
the fallback fired.

### 3.4 The composed measurement

[VERIFY: vggt_slam/solver.py:153]
```python
H_overlap_prior_overlap_current = np.linalg.inv(prior_submap.proj_mats[-1]) @ current_submap.proj_mats[0] @ H_scale
```

The order matters — `H_scale` is right-multiplied, which means it acts
in the *current* submap's frame (the rightmost transformation in a
right-acting convention). If it were left-multiplied, the scale would
be applied in the previous submap's frame, which would mean *the
previous submap's depths* get rescaled. The chosen order is correct:
we are correcting the *new* submap to match the existing graph.

### 3.5 Node insertion guard for LC

[VERIFY: vggt_slam/solver.py:157-158]
```python
if not is_loop_closure:
    self.graph.add_homography(submap_id_curr + frame_id_curr, H_w_submap)
```

When `is_loop_closure=True`, the node `X(submap_id_curr +
frame_id_curr)` already exists (it was inserted earlier as a build
node), so `add_homography` would be an idempotent no-op — but the
guard makes the intent explicit and saves a function call.

### 3.6 Early return for LC

[VERIFY: vggt_slam/solver.py:174-176]
```python
# Loop closure only gets intra submap constraints.
if is_loop_closure:
    return
```

Without this, the LC call would also wire up an inner-submap chain
for the *target* submap — which is wrong (that chain already exists).

---

## 4. `PoseGraph.optimize`

[VERIFY: vggt_slam/graph.py:80-127]

### 4.1 Parameter setup

[VERIFY: vggt_slam/graph.py:82-86]
```python
params = gtsam.LevenbergMarquardtParams()
if verbose:
    params.setVerbosityLM("SUMMARY")
    params.setVerbosity("ERROR")
```

Default LM parameters from GTSAM. The verbose path is for debugging
only; in `main.py` it's never enabled.

### 4.2 Optimiser construction

[VERIFY: vggt_slam/graph.py:88]
```python
optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.values, params)
```

Importantly the optimizer takes `self.values` (the *current* estimate)
as input. This is the warm-start mechanism — the previous optimum
seeds the new optimisation.

### 4.3 Initial error

[VERIFY: vggt_slam/graph.py:91-92]
```python
initial_error = self.graph.error(self.values)
print(f"Initial total error: {initial_error:.6f}")
```

Printed on every call so the user can see "did the new submap cause a
visible error spike?" In practice the new factor's residual is small
because the between factor's initial value was set to satisfy it
exactly ([VERIFY: vggt_slam/solver.py:154]); the only error contribution
comes from how the new node connects to the existing graph (e.g. a
loop closure perturbs old nodes).

### 4.4 Optimisation

[VERIFY: vggt_slam/graph.py:109]
```python
result = optimizer.optimize()
```

Internally this runs LM until convergence (per GTSAM's default
tolerances). No `MaxIter` override.

### 4.5 Result assignment

[VERIFY: vggt_slam/graph.py:127]
```python
self.values = result
```

Wholesale replacement. The previous `Values` object is discarded — any
external code that cached an SL(4) value will now read stale data.
There is no such caching in the released codebase, so this is safe.

### 4.6 What this function does NOT do

* It does *not* clear or modify `self.graph` (the factor container) —
  factors live forever once added.
* It does *not* clear `self.initialized_nodes` — that set is
  authoritative for "what keys exist".
* It does *not* publish events; downstream code re-reads node values
  via `get_homography` on the next call.

So calling `optimize()` is idempotent in the sense that calling it
twice in a row yields the same result (LM converged in the first call,
the second call starts from the converged values and immediately
re-converges to them).

---

## 5. `ImageRetrieval.find_loop_closures`

[VERIFY: vggt_slam/loop_closure.py:77-86]

### 5.1 Per-query loop

[VERIFY: vggt_slam/loop_closure.py:79-85]
```python
matches_queue = LoopMatchQueue(max_size=max_loop_closures)
query_id = 0
for query_vector in submap.get_all_retrieval_vectors():
    best_score, best_submap_id, best_frame_id = map.retrieve_best_score_frame(query_vector, submap.get_id(), ignore_last_submap=True)
    if best_score < max_similarity_thres:
        new_match_data = LoopMatch(best_score, submap.get_id(), query_id, best_submap_id, best_frame_id)
        matches_queue.add(new_match_data)
    query_id += 1
```

* The query vectors are iterated in order, so `query_id` (the index of
  the query frame within the current submap) increases monotonically.
* `retrieve_best_score_frame` is called once per query — N queries,
  each scanning the full corpus.
* Only matches *below the threshold* are added to the queue. The queue
  caps the total returned matches at `max_size`.

### 5.2 Bounded-heap selection

The queue is the textbook "top-K with smallest distance" data
structure:

* If fewer than `max_size` matches have been seen, push.
* Otherwise, `heappushpop` — push and pop in one shot, evicting the
  worst current match.

This ensures memory complexity is `O(max_loops)` regardless of how
many candidates the threshold lets through.

### 5.3 Return order

[VERIFY: vggt_slam/loop_closure.py:47-49]
```python
def get_matches(self):
    """Return sorted list of matches (lowest value first)"""
    return [match for _, match in sorted(self.heap, reverse=True)]
```

Sort by `(-score, match)` descending — equivalent to sorting by score
ascending. So the returned list runs from **smallest score = best
match** to **largest score = worst surviving match**. The docstring
says "lowest value first" — correct for the score, but the consumer
(`add_points`) only reads `[0]` so the rest are unused.

---

## 6. `Submap.get_all_poses_world`

[VERIFY: vggt_slam/submap.py:114-129]

The read-side dual of `add_edge`. Returns one optimised pose per
frame in the submap.

### 6.1 Core loop

```python
def get_all_poses_world(self, graph, give_camera_mat=False):
    homography_list = [graph.get_homography(i + self.get_id()) for i in range(len(self.poses))]
    poses = []
    for index, homography_world in enumerate(homography_list):
        projection_mat = self.proj_mats[index] @ np.linalg.inv(homography_world)
        projection_mat = projection_mat / projection_mat[-1,-1]
        if give_camera_mat:
            poses.append(projection_mat)
        else:
            cal, rot, trans, scale = decompose_camera(projection_mat[0:3,:])

            pose = np.eye(4)
            pose[0:3, 0:3] = rot
            pose[0:3,3] = trans
            poses.append(pose)
    return np.stack(poses, axis=0)
```

### 6.2 What's happening

* `homography_list[i]` is the optimised `SL(4)` value of node `i +
  submap_id` — i.e. the node belonging to frame `i` of this submap.
* `self.proj_mats[index]` is `K_4x4` (the 4×4 intrinsic embedding).
* `K_4x4 @ inv(H)` produces a **4×4 projection matrix** that maps
  world points to image coordinates (the standard pinhole projection
  composed with the SL(4) world-to-cam transform). Dividing by
  `[-1,-1]` normalises to standard homography form.
* If `give_camera_mat=True`, this matrix is returned directly. Used
  by `GraphMap.write_poses_to_file` ([VERIFY: vggt_slam/map.py:135]).
* If False, the top 3×4 is RQ-decomposed via
  `decompose_camera` ([VERIFY: vggt_slam/slam_utils.py:45-83]) into
  `(K, R, t, scale)`. A 4×4 SE(3) pose is reconstructed from `(R, t)`.

This is the function that turns the SL(4) graph values into
human-interpretable SE(3) camera poses for the viewer
([VERIFY: vggt_slam/solver.py:85-89]).

### 6.3 The hidden RQ decomposition

`decompose_camera` ([VERIFY: vggt_slam/slam_utils.py:45-83]) uses
`scipy.linalg.rq` to factor the left 3×3 block of the projection
matrix into upper-triangular `K` and orthogonal `R`. It then enforces
positive `K` diagonals (sign-correcting the corresponding rows of
`R`).

For the viewer this gives a properly-oriented camera triad. For the
output trajectory (TUM/KITTI), it provides the rotation matrix that
gets converted to a quaternion
([VERIFY: vggt_slam/map.py:160]).

### 6.4 Why not store the SE(3) directly?

Because SL(4) optimisation can *deform* the projective transformation
in ways that an SE(3) approximation would lose. After optimisation the
graph might have moved cameras in ways that don't correspond to a
pure rigid transformation; RQ-decomposing the resulting 4×4 returns
the *best-fitting* SE(3), with the residual absorbed into the
recovered K. The user gets a sensible camera pose plus an updated K
(visible if they ask for `give_camera_mat=True`).

---

## 7. `FrameTracker.compute_disparity`

[VERIFY: vggt_slam/frame_overlap.py:23-60]

Already covered in detail in `06-ALGORITHM-keyframe_selection.md §2.2`.
The five branches in order of execution:

1. **First-call branch** (`last_kf is None` or `kf_pts < 10`).
   Re-init and return True.
2. **LK call** — track corners.
3. **Lost-corners branch** (`good_kf < 10` after LK).
   Re-init and return True.
4. **Optional visualisation**.
5. **Disparity threshold branch** — re-init and return True if
   `mean_disparity > min_disparity`, else return False.

The function maintains exactly three pieces of state (`last_kf`,
`kf_gray`, `kf_pts`) and they are written only inside
`initialize_keyframe`, never elsewhere.

---

## 8. `GraphMap.write_poses_to_file`

[VERIFY: vggt_slam/map.py:134-162]

### 8.1 Pose collection

[VERIFY: vggt_slam/map.py:135]
```python
all_poses = self.get_all_cam_matricies(give_camera_mat=True, graph=graph)
```

`get_all_cam_matricies` ([VERIFY: vggt_slam/map.py:125-132])
concatenates `submap.get_all_poses_world(graph, give_camera_mat=True)`
across every non-LC submap. This is the global camera-matrix list for
all build frames.

### 8.2 Filtering and indexing

[VERIFY: vggt_slam/map.py:141-150]
```python
count = 0
for submap_index, submap in enumerate(self.ordered_submaps_by_key()):
    if submap.get_lc_status():
        continue
    frame_ids = submap.get_frame_ids()
    print(frame_ids)
    for frame_index, frame_id in enumerate(frame_ids):
        pose = all_poses[count]
        K, rotation_matrix, t, scale = decompose_camera(pose)
        count += 1
```

* Iterates submaps in sorted-id order.
* Skips LC submaps explicitly — they never appear in the output file.
* Uses `frame_ids` (the numeric tokens extracted from filenames) as
  the timestamp column.
* `count` advances through `all_poses` in lockstep — this only works
  because `get_all_cam_matricies` skipped LC submaps too. If those
  two functions ever fall out of sync about which submaps to skip,
  the output trajectory would mis-align.

### 8.3 KITTI vs. TUM branch

[VERIFY: vggt_slam/map.py:152-162]
```python
x, y, z = t
if kitti_format:
    pose_matrix = np.eye(4)
    pose_matrix[:3, :3] = rotation_matrix
    pose_matrix[:3, 3] = t
    output = pose_matrix.flatten()[:-4]
    output = np.array([float(frame_id), *output])
else:
    quaternion = R.from_matrix(rotation_matrix).as_quat() # x, y, z, w
    output = np.array([float(frame_id), x, y, z, *quaternion])
f.write(" ".join(f"{v:.8f}" for v in output) + "\n")
```

* **KITTI**: row-major flattened 3×4 matrix → 12 numbers, prefixed by
  the frame timestamp. The `[:-4]` strips off the last row of the 4×4
  (which is `[0, 0, 0, 1]`).
* **TUM**: `tstamp x y z qx qy qz qw`. The quaternion is `(x, y, z,
  w)` per `scipy.spatial.transform.Rotation.as_quat()` convention.
  The comment confirms ([VERIFY: vggt_slam/map.py:160]).

### 8.4 Quirky behaviour with `rectifying_H_mats`

[VERIFY: vggt_slam/map.py:138-140]
```python
if self.rectifying_H_mats:
    assert len(self.rectifying_H_mats) == len(all_poses), "Number of rectifying mats and number of poses do not match"
    print("Using rectifying homographies when writing poses to file.")
```

* The branch is dead in the released pipeline (`rectifying_H_mats`
  is never appended to).
* The print message says "Using rectifying homographies" but **nothing
  in the file-writing loop actually applies them**. So even if
  `rectifying_H_mats` were populated, the output would not use them.
  The hook is incomplete.

This is consistent with the general pattern in this codebase: the
auto-calibration / rectification code path is a placeholder for a
future feature, not active in the released pipeline.

---

## 9. Function call graph (the load-bearing edges)

```
main.py main():
    │
    ├─▶ FrameTracker.compute_disparity (per raw frame)
    │
    ├─▶ Solver.run_predictions (per submap)
    │         │
    │         ├─▶ load_and_preprocess_images
    │         ├─▶ ImageRetrieval.get_all_submap_embeddings
    │         │       └─▶ ImageRetrieval.get_batch_descriptors
    │         ├─▶ (optional) compute_image_embeddings (CLIP)
    │         ├─▶ VGGT(images)                                   # build inference
    │         ├─▶ ImageRetrieval.find_loop_closures
    │         │       └─▶ GraphMap.retrieve_best_score_frame
    │         │       └─▶ LoopMatchQueue.add
    │         ├─▶ (optional) VGGT(lc_frames, compute_similarity=True)  # verifier
    │         └─▶ pose_encoding_to_extri_intri
    │
    ├─▶ Solver.add_points (per submap)
    │         │
    │         ├─▶ unproject_depth_map_to_point_map
    │         ├─▶ closed_form_inverse_se3
    │         ├─▶ Submap.{add_all_poses, add_all_points, set_conf_masks}
    │         ├─▶ GraphMap.add_submap
    │         └─▶ Solver.add_edge   (1× normal + 2× per LC)
    │                   │
    │                   ├─▶ estimate_scale_pairwise
    │                   ├─▶ PoseGraph.get_homography
    │                   ├─▶ PoseGraph.add_homography
    │                   ├─▶ PoseGraph.add_between_factor
    │                   ├─▶ (first edge only) PoseGraph.add_prior_factor
    │                   └─▶ Submap.get_all_poses
    │
    ├─▶ PoseGraph.optimize (per submap)
    │         └─▶ gtsam.LevenbergMarquardtOptimizer.optimize
    │
    ├─▶ (optional) Solver.update_{all,latest}_submap_vis
    │         └─▶ Submap.get_points_in_world_frame, get_all_poses_world
    │                   └─▶ Submap.proj_mats[i] @ inv(graph.get_homography(...))
    │                       └─▶ slam_utils.decompose_camera (RQ)
    │
    ├─▶ torch.cuda.empty_cache (per submap)
    │
    └─▶ _safe_dump_poses (per submap) → GraphMap.write_poses_to_file
```

Every function in this list has a corresponding section above.

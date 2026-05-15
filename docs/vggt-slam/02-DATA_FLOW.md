# VGGT-SLAM 2.0 — Data Flow (数据流分析)

> Reference for how a pixel becomes a globally-consistent map vertex.
> Every arrow in every diagram is anchored to a `[VERIFY:]` tag.
>
> Repo root: `/home/steve/vslam_ws/src/VGGT-SLAM`.

---

## 1. Eight stages from disk to map

```
   (1) disk          (2) tracker       (3) preprocess   (4) VGGT inference
  ┌────────┐       ┌────────────┐    ┌─────────────┐   ┌───────────────────────┐
  │ *.jpg, │──cv2─▶│ FrameTracker│───▶│ load_and_   │──▶│ VGGT.forward(images)  │
  │ *.png  │       │  LK flow   │    │ preprocess_ │   │ pose_enc, depth,      │
  │ folder │       │  vs last kf│    │ images()    │   │ depth_conf,           │
  └────────┘       └────────────┘    └─────────────┘   │ target_tokens,        │
                                                       │ image_match_ratio     │
                                                       └────────┬──────────────┘
                                                                ▼
        (5) decode pose enc + unproject depth                  (6) loop closure
        ┌──────────────────────────────────────┐               ┌──────────────────┐
        │ pose_encoding_to_extri_intri →       │               │ SALAD descriptors│
        │   extrinsic (S,3,4), intrinsic (S,3,3)│              │ + ||q - db||     │
        │ closed_form_inverse_se3 → cam_to_world│             │ + 2-frame VGGT   │
        │ unproject_depth_map_to_point_map →    │              │   re-run         │
        │   world_points (S,H,W,3)              │              └──────────────────┘
        └──────────────────────────────────────┘                        │
                            │                                            │
                            ▼                                            ▼
                  (7) SL(4) edge construction (Solver.add_edge)
        ┌──────────────────────────────────────────────────────────────┐
        │  scale = median(|p_curr|/|p_prev|)  (overlap frame)            │
        │  H_olap = inv(prev_proj_last) · curr_proj_0 · diag(s,s,s,1)    │
        │  H_w_submap = graph[prev_overlap] · H_olap                     │
        │  graph.add_homography(curr,0, H_w_submap)                      │
        │  graph.add_between_factor(prev_overlap, curr,0, H_olap, σ_intra)│
        │  for i ≥ 1:                                                    │
        │    H_inner = pose[i-1] · inv(pose[i])                          │
        │    graph.add_homography(curr,i, graph[curr,i-1] · H_inner)     │
        │    graph.add_between_factor(curr,i-1, curr,i, H_inner, σ_inner) │
        └──────────────────────────────────────────────────────────────┘
                            │
                            ▼
                  (8) LM optimisation
        ┌────────────────────────────────────────────────────────┐
        │  LevenbergMarquardtOptimizer(NLFG, Values).optimize()  │
        │  PoseGraph.values ← result                              │
        └────────────────────────────────────────────────────────┘
                            │
                            ▼
                  globally consistent SL(4) graph
```

Each stage is detailed below with the exact code that runs it.

---

## 2. Stage 1 — disk to memory

[VERIFY: main.py:82-89]
```python
image_names = [f for f in glob.glob(os.path.join(args.image_folder, "*"))
            if "depth" not in os.path.basename(f).lower()
            and "txt" not in os.path.basename(f).lower()
            and "db" not in os.path.basename(f).lower()]

image_names = utils.sort_images_by_number(image_names)
downsample_factor = 1
image_names = utils.downsample_images(image_names, downsample_factor)
```

Three filtering rules:

1. Anything whose basename contains `depth`, `txt` or `db` is dropped.
2. `sort_images_by_number` ([VERIFY: vggt_slam/slam_utils.py:23-30])
   sorts by the numeric portion of the basename **immediately
   before** the extension — `re.search(r'\d+(?:\.\d+)?(?=\.[^.]+$)')`.
   Decimal timestamps (e.g. `1305031453.359684.png` from TUM) sort
   correctly because `\d+(?:\.\d+)?` matches the whole number.
3. `downsample_factor` is hard-coded to `1` (no skipping). The function
   exists ([VERIFY: vggt_slam/slam_utils.py:32-43]) for future use.

The list is then iterated; each path is **only** read via `cv2.imread`
([VERIFY: main.py:109]) — at this stage we are still on CPU.

---

## 3. Stage 2 — keyframe selection

For every raw image, `FrameTracker.compute_disparity` is called:

[VERIFY: main.py:107-113]
```python
if use_optical_flow_downsample:
    with keyframe_time:
        img = cv2.imread(image_name)
        enough_disparity = solver.flow_tracker.compute_disparity(img, args.min_disparity, args.vis_flow)
        if enough_disparity:
            image_names_subset.append(image_name)
            image_count += 1
```

`compute_disparity` ([VERIFY: vggt_slam/frame_overlap.py:23-60]):

* On the very first call (`last_kf is None`) it always returns `True`
  after calling `initialize_keyframe`.
* On subsequent calls it runs Lucas–Kanade pyramidal flow against the
  previous keyframe:

```python
# [VERIFY: vggt_slam/frame_overlap.py:31-35]
next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
    self.kf_gray, curr_gray, self.kf_pts, None,
    winSize=(21, 21), maxLevel=3,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
```

* It then computes the mean Euclidean displacement of the surviving
  tracked corners
  ([VERIFY: vggt_slam/frame_overlap.py:46-47]):

```python
displacement   = np.linalg.norm(good_next - good_kf, axis=1)
mean_disparity = np.mean(displacement)
```

* When `mean_disparity > min_disparity` (CLI default: 50 px,
  [VERIFY: main.py:31]) — or fewer than 10 tracked corners remain
  ([VERIFY: vggt_slam/frame_overlap.py:24,41-43]) — the image is
  promoted to a new keyframe and the function returns `True`.

The buffering loop ([VERIFY: main.py:107-118]) lets keyframes
accumulate until the count reaches `submap_size + overlapping_window_size`
or the input is exhausted.

**Edge case** — when the buffer reaches the threshold mid-stream, the
loop calls `Solver.run_predictions(...)` *immediately*, even if many
non-keyframes still follow. The "image is the last one" branch is the
only way a shorter trailing buffer ever triggers a submap:

```python
# [VERIFY: main.py:118]
if len(image_names_subset) == args.submap_size + args.overlapping_window_size or image_name == image_names[-1]:
```

So a video whose last batch contains, say, 7 keyframes still produces
a trailing submap of size 7 (no padding).

---

## 4. Stage 3 — preprocess on GPU

[VERIFY: vggt_slam/solver.py:300-304]
```python
with self.vggt_timer:
    images = load_and_preprocess_images(image_names).to(device)
print(f"Loaded and preprocessed {len(image_names)} images in {time.time() - t1:.2f} seconds")
print(f"Preprocessed images shape: {images.shape}")
```

`load_and_preprocess_images` lives in the VGGT package
([VERIFY: third_party/vggt/vggt/utils/load_fn.py:97]). It loads each
image with PIL, normalises to `[0,1]`, and resizes/letterboxes to the
network's expected `518` longest side (mode `"crop"`). The result is a
single `torch.Tensor` of shape `(S, 3, H, W)` already on the device.

The tensor is then made half-precision because the upstream VGGT model
itself casts everything to `bfloat16` at the entry of `forward`
([VERIFY: third_party/vggt/vggt/models/vggt.py:60]):

```python
images = images.to(torch.bfloat16)
```

(Note that `main.py` also calls `.to(torch.bfloat16)` on the *model*
itself [VERIFY: main.py:77]; the redundant cast on the tensor inside
`forward` is a no-op when the tensor is already bf16.)

---

## 5. Stage 4 — VGGT inference

Two calls per submap:

[VERIFY: vggt_slam/solver.py:332-336]
```python
with torch.no_grad():
    t1 = time.time()
    with self.vggt_timer:
        predictions = model(images)
    print(f"VGGT model inference took {time.time() - t1:.2f} seconds")
```

This produces the build-submap `predictions` dict (see
`01-DATA_STRUCTURES.md §9`).

If SALAD reports a candidate loop, a second VGGT call follows:

[VERIFY: vggt_slam/solver.py:346-351]
```python
with torch.no_grad():
    lc_frames = torch.stack(
        (new_submap.get_frame_at_index(detected_loops[0].query_submap_frame),
         retrieved_frames[0]), axis=0)
    predictions_lc = model(lc_frames, compute_similarity=True)
    loop_closure_frame_names = [...]
```

* The model is run on a 2-frame stack `[query_kf, retrieved_kf]`.
* `compute_similarity=True` makes the aggregator return a non-trivial
  `image_match_ratio` ([VERIFY: third_party/vggt/vggt/models/vggt.py:61]),
  which is the gate used at
  [VERIFY: vggt_slam/solver.py:371-374] to drop poor matches:

```python
if image_match_ratio < 0.85:
    print("Loop closure image match ratio too low, skipping loop closure")
    predictions_lc = None
    predictions["detected_loops"] = []
```

---

## 6. Stage 5 — decoding the predictions

Three transformations turn the network output into something Geometry
can consume.

### 6.1 Pose encoding → extrinsic/intrinsic

[VERIFY: vggt_slam/solver.py:363-366]
```python
extrinsic, intrinsic = pose_encoding_to_extri_intri(predictions["pose_enc"], images.shape[-2:])
predictions["extrinsic"] = extrinsic
predictions["intrinsic"] = intrinsic
```

`pose_encoding_to_extri_intri`
([VERIFY: third_party/vggt/vggt/utils/pose_enc.py:62]) decodes the
9-vector pose encoding produced by VGGT's camera head into a
`(S, 3, 4)` extrinsic and a `(S, 3, 3)` intrinsic.

### 6.2 Extrinsic → camera-to-world

`add_points` does the inversion:

[VERIFY: vggt_slam/solver.py:225]
```python
cam_to_world = closed_form_inverse_se3(extrinsics_cam)  # (S, 4, 4)
```

`closed_form_inverse_se3` ([VERIFY: third_party/vggt/vggt/utils/geometry.py:120])
does the classic `[R^T | -R^T t]` inversion without an LU solve.

The world-to-camera form is then recovered with a plain matrix
inverse:

[VERIFY: vggt_slam/solver.py:232]
```python
world_to_cam = np.linalg.inv(cam_to_world)
```

These `world_to_cam` matrices are stored on `Submap.poses`
([VERIFY: vggt_slam/solver.py:246]).

### 6.3 Depth → world points

[VERIFY: vggt_slam/solver.py:222]
```python
world_points = unproject_depth_map_to_point_map(depth_map, extrinsics_cam, intrinsics_cam)
```

`unproject_depth_map_to_point_map`
([VERIFY: third_party/vggt/vggt/utils/geometry.py:15-49]) iterates per
frame, calls `depth_to_world_coords_points`, and returns a
`(S, H, W, 3)` array of points in the VGGT-predicted world frame.

`Submap.add_all_points` stores these directly without further
transformation ([VERIFY: vggt_slam/submap.py:37]). They are later
mapped into the **global SL(4) frame** at read time, not write time:

[VERIFY: vggt_slam/submap.py:174-190]
```python
def get_points_list_in_world_frame(self, graph, ...):
    homography_list = [graph.get_homography(i + self.get_id()) for i in range(len(self.poses))]
    ...
    points_transformed = (homography_list[index] @ points_homogeneous.T).T
```

i.e. the optimised SL(4) node value left-multiplies the local point
cloud (in homogeneous coordinates, then dehomogenised). The optimiser
can therefore freely move the global poses around without touching the
raw point data.

### 6.4 K → K_4x4

[VERIFY: vggt_slam/solver.py:230-231]
```python
K_4x4 = np.tile(np.eye(4), (N, 1, 1))
K_4x4[:, :3, :3] = intrinsics_cam
```

This 4×4 form is stored as `Submap.proj_mats`
([VERIFY: vggt_slam/submap.py:41]) and is the embedding of the 3×3
intrinsic into SL(4)'s native size. It re-enters the maths in
`add_edge` (computing the relative homography) and in
`get_all_poses_world` (decomposition back into R, t, K).

### 6.5 Colours

[VERIFY: vggt_slam/solver.py:224]
```python
colors = (images.transpose(0, 2, 3, 1) * 255).astype(np.uint8)  # (S, H, W, 3)
```

Plain per-pixel RGB; carried alongside the cloud for visualisation.

---

## 7. Stage 6 — loop closure (data-flow detail)

The full loop-closure data flow is:

```
                        new_submap.frames  (S, 3, H, W)
                                │
                                ▼
                        ImageRetrieval.get_all_submap_embeddings
                                │      [VERIFY: vggt_slam/solver.py:321]
                                ▼
                 SALAD descriptors stored on submap.retrieval_vectors
                                │
                                ▼
         find_loop_closures(map, submap, max_loops, lc_thres)
                                │      [VERIFY: vggt_slam/loop_closure.py:77]
                                ▼
            for each query_vector:
                map.retrieve_best_score_frame(query_vector, …)
                  └▶ argmin over all non-LC, non-current submaps
            keep the best ≤ max_loops in a bounded heap
                                │
                                ▼
                  detected_loops : list[LoopMatch]
                                │
                  ┌─────────────┴─────────────┐
                  │                            │
       no loop survives                two-frame VGGT pass
       cutoff (lc_thres)                  on [query, retrieved]
                  │                            │
                  ▼                            ▼
       skip lc branch         image_match_ratio < 0.85 ?
                                ┌─yes─▶ drop
                                │
                                no
                                ▼
                       store extrinsic_lc, intrinsic_lc,
                       depth_lc, depth_conf_lc,
                       frames_lc, frames_lc_names
                                │
                                ▼
                    Solver.add_points handles them in its
                    "for index, loop in enumerate(detected_loops):" loop
                    [VERIFY: vggt_slam/solver.py:254-287]
```

Inside `add_points`, the loop branch creates a *new* `Submap` flagged
as `is_lc_submap=True` ([VERIFY: vggt_slam/solver.py:272-273]), fills
it with the 2-frame VGGT predictions, and connects it to the rest of
the graph via two `add_edge` calls
([VERIFY: vggt_slam/solver.py:286-287]):

```python
self.add_edge(lc_submap_num, 0, loop.query_submap_id, loop.query_submap_frame, is_loop_closure=False)
self.add_edge(loop.detected_submap_id, loop.detected_submap_frame, lc_submap_num, 1, is_loop_closure=True)
```

The first creates a regular intra-submap factor `query ↔ lc_submap[0]`.
The second creates the actual loop factor `lc_submap[1] ↔ detected`
flagged with `is_loop_closure=True` to suppress the inner-submap loop
in `add_edge` ([VERIFY: vggt_slam/solver.py:174-176]):

```python
# Loop closure only gets intra submap constraints.
if is_loop_closure:
    return
```

So the LC submap acts as a small two-node "bridge" that splices the
loop into the SL(4) graph without re-injecting the inner-submap chain.

---

## 8. Stage 7 — SL(4) edge construction (`Solver.add_edge`)

This is where the most subtle data transformations live. The full body
spans [VERIFY: vggt_slam/solver.py:118-195].

### 8.1 First submap path (`first_edge == True`)

[VERIFY: vggt_slam/solver.py:167-172]
```python
else:
    assert (submap_id_curr == 0 and frame_id_curr == 0)
    self.graph.add_homography(submap_id_curr + frame_id_curr, H_w_submap)  # H_w_submap == I
    self.graph.add_prior_factor(submap_id_curr + frame_id_curr, H_w_submap)
```

The very first submap pins its first pose at identity with a tight
`anchor_noise` prior ([VERIFY: vggt_slam/graph.py:23,49-54]). No
between factor is added at this point — only the inner-submap chain
that follows.

### 8.2 Subsequent submaps — overlap → scale → between factor

For every submap after the first, the overlap frame is used to bridge
scales:

[VERIFY: vggt_slam/solver.py:127-146]
```python
prior_submap = self.map.get_submap(submap_id_prev)
current_conf = current_submap.get_conf_masks_frame(frame_id_curr)
prior_conf   = prior_submap.get_conf_masks_frame(frame_id_prev)
good_mask = (prior_conf > prior_submap.get_conf_threshold()) * \
            (current_conf > prior_submap.get_conf_threshold())
good_mask = good_mask.reshape(-1)

if np.sum(good_mask) < 100:
    print("Not enough overlapping points to estimate scale factor, using a less restrictive mask")
    good_mask = (prior_conf > prior_submap.get_conf_threshold()).reshape(-1)
    if np.sum(good_mask) < 100:
        good_mask = (prior_conf > 0).reshape(-1)

P_temp = np.linalg.inv(prior_submap.proj_mats[-1]) @ current_submap.proj_mats[0]
t1 = (P_temp[0:3,0:3] @ current_submap.get_frame_pointcloud(frame_id_curr).reshape(-1, 3)[good_mask].T).T
t2 = prior_submap.get_frame_pointcloud(frame_id_prev).reshape(-1, 3)[good_mask]
scale_factor_est_output = estimate_scale_pairwise(t1, t2)
scale_factor = scale_factor_est_output[0]
H_scale = np.diag((scale_factor, scale_factor, scale_factor, 1.0))
```

* The mask is the AND of "prior frame confident" and "current frame
  confident" — both thresholded by the *prior* submap's threshold.
* If fewer than 100 pixels survive, the mask is loosened twice (drop
  the current-frame requirement; finally drop the threshold entirely
  while keeping `prior_conf > 0`).
* `P_temp` is the relative pose **between two submap-local frames**:
  it maps points in the current submap's first-frame coordinates into
  the previous submap's last-frame coordinates.
  Its rotation part is applied to the current cloud so the two clouds
  are co-rotated before scale estimation.
* `estimate_scale_pairwise(t1, t2)` ([VERIFY: vggt_slam/scale_solver.py:15-24])
  returns `(median(||y||/||x||), None)` — see
  `07-ALGORITHM-scale_solver.md`.
* The scalar is embedded into a 4×4 homography
  `diag(s, s, s, 1)` which, by SL(4) convention, scales the 3-D
  translation between the two submaps while leaving the projective
  scale alone.

The composed transformation is then:

[VERIFY: vggt_slam/solver.py:153-158]
```python
H_overlap_prior_overlap_current = np.linalg.inv(prior_submap.proj_mats[-1]) @ current_submap.proj_mats[0] @ H_scale
H_w_submap = self.graph.get_homography(overlapping_node_id_prev) @ H_overlap_prior_overlap_current

if not is_loop_closure:
    self.graph.add_homography(submap_id_curr + frame_id_curr, H_w_submap)

self.graph.add_between_factor(
    overlapping_node_id_prev, submap_id_curr + frame_id_curr,
    H_overlap_prior_overlap_current, self.graph.intra_submap_noise)
```

i.e.:

* `H_overlap_prior_overlap_current` is built **without** consulting
  the global graph; it is purely a between-submap measurement.
* `H_w_submap` is the *initial value* of the new node, propagated from
  the previous submap's optimised SL(4) homography. This gives LM a
  warm start.
* The between factor uses `intra_submap_noise` (σ=0.05).

### 8.3 Inner submap chain

For every non-overlap frame in the current submap:

[VERIFY: vggt_slam/solver.py:179-191]
```python
world_to_cam = current_submap.get_all_poses()
for index, pose in enumerate(world_to_cam):
    if index == 0:
        continue
    H_inner = world_to_cam[index-1] @ np.linalg.inv(pose)
    current_node = self.graph.get_homography(submap_id_curr + index - 1) @ H_inner
    self.graph.add_homography(submap_id_curr + index, current_node)
    self.graph.add_between_factor(
        submap_id_curr + index - 1, submap_id_curr + index, H_inner,
        self.graph.inner_submap_noise)
```

* `H_inner` is the relative world-to-cam between frames `i-1` and `i`
  *inside* the submap — taken straight from VGGT's predicted poses, no
  scale fix-up.
* `current_node` is the initial value: previous node's optimised SL(4)
  homography times the local relative.
* The between factor uses `inner_submap_noise` (σ=0.05, same value but
  conceptually distinct from `intra_submap_noise`).

### 8.4 Visual summary

```
prior submap                        current submap
[0] [1] [2] ... [last]              [0] [1] [2] ... [last]
              │                      │
              │      H_olap          │
              └──────────────────────┘   (intra_submap_noise)
              │                      │
                                     │ H_inner   inner_submap_noise
                                     └───────────┐
                                                  ▼
                                               [1] etc.
```

---

## 9. Stage 8 — LM optimisation

[VERIFY: main.py:128-129]
```python
with backend_time:
    solver.graph.optimize()
```

`PoseGraph.optimize` ([VERIFY: vggt_slam/graph.py:80-127]) is a thin
wrapper:

```python
params = gtsam.LevenbergMarquardtParams()
optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.values, params)
initial_error = self.graph.error(self.values)
result = optimizer.optimize()
final_error = self.graph.error(result)
self.values = result
```

There is no IS-AM, no fixed-lag smoothing, no marginalisation: the
**entire** factor graph is reoptimised from scratch after every new
submap. This works because:

* `Values` is reused, so the previous optimum is the warm start.
* SL(4) BetweenFactors are well-conditioned with the σ=0.05 noise
  model and converge in a handful of LM iterations.
* Submap count grows linearly; with stock TUM/7-Scenes the graph never
  exceeds a few thousand nodes.

Costs per call are reported as `backend_time` in `main.py`'s exit
summary ([VERIFY: main.py:158]).

---

## 10. Stage 9 — point cloud → viewer

After the optimisation, `main.py` decides whether to push points to the
viewer:

[VERIFY: main.py:132-136]
```python
if args.vis_map:
    if loop_closure_detected:
        solver.update_all_submap_vis()
    else:
        solver.update_latest_submap_vis()
```

`update_latest_submap_vis` only pushes the newest submap's cloud and
frustums; `update_all_submap_vis` re-pushes everything (necessary
after a loop closure because the SL(4) optimisation just moved the
older submaps).

Both go through `Solver.set_submap_point_cloud`
([VERIFY: vggt_slam/solver.py:78-83]) and
`Solver.set_submap_poses`
([VERIFY: vggt_slam/solver.py:85-89]). The cloud goes through optional
Open3D voxel downsampling
([VERIFY: vggt_slam/solver.py:62-69]) before being added with
`viewer.server.scene.add_point_cloud` and the frustums with
`add_camera_frustum`
([VERIFY: vggt_slam/viewer.py:76-86]).

---

## 11. Stage 10 — crash-safe pose dump

A modification specific to the GeoScan benchmark setup
([VERIFY: main.py:97-105]):

```python
# GeoScan/8GB: on low-VRAM boxes the run can OOM mid-way. Dump poses
# after every submap so a crash still leaves us a partial trajectory to
# evo_ape against finder. Write is <10ms per call.
def _safe_dump_poses():
    try:
        if args.log_results:
            solver.map.write_poses_to_file(args.log_path, solver.graph, kitti_format=False)
    except Exception as _e:
        print(f"[warn] partial pose dump failed: {_e}")
```

After every backend pass + visualiser update:

[VERIFY: main.py:144-148]
```python
torch.cuda.empty_cache()
# Periodic pose dump so a mid-run OOM still produces data.
_safe_dump_poses()
```

This is **not present in stock upstream VGGT-SLAM** — it was added to
allow `evo_ape` on partial trajectories when the 8 GB VRAM box OOMs
around submap ~34 with the GeoScan B1 bag.

---

## 12. Loop-iteration timing (the critical path)

For one frame that *is* selected as a keyframe and triggers a submap:

| Stage                                                          | Dominant op                                                | Lives in                                                              |
| -------------------------------------------------------------- | ---------------------------------------------------------- | --------------------------------------------------------------------- |
| Keyframer                                                      | `calcOpticalFlowPyrLK` on 1000 corners                     | `keyframe_time`                                                       |
| `load_and_preprocess_images`                                   | PIL load + resize + ToTensor                               | `vggt_timer`                                                          |
| `model(images)`                                                | VGGT forward                                               | `vggt_timer`                                                          |
| `find_loop_closures`                                           | SALAD descriptors + L2 search across all prior submaps     | `loop_closure_timer`                                                  |
| Loop verifier (second VGGT call on 2 frames)                   | VGGT forward (smaller batch, but adds time)                | `vggt_timer`                                                          |
| `unproject_depth_map_to_point_map`                             | per-frame depth → world via Eigen-style loops (Python)     | (untimed; small)                                                      |
| `closed_form_inverse_se3` + `np.linalg.inv`                    | trivial                                                    | (untimed)                                                             |
| `Solver.add_edge` scale/edge wiring                            | `estimate_scale_pairwise` (median of norms) + GTSAM glue   | (untimed)                                                             |
| `PoseGraph.optimize`                                           | LM over full SL(4) NLFG                                    | `backend_time`                                                        |
| Viewer update (optional)                                       | Open3D voxel downsample + viser scene ops                  | (untimed; only with `--vis_map`)                                      |
| `torch.cuda.empty_cache` + safe pose dump                      | dump = a few hundred lines of TUM text                     | (untimed; ~<10 ms per call by comment)                                |

Per-frame averages are printed at the end:

[VERIFY: main.py:155-161]
```python
print("Average VGGT time per frame:", solver.vggt_timer.total_time / image_count)
print("Average loop closure time per frame:", solver.loop_closure_timer.total_time / image_count)
print("Average keyframe selection time per frame:", keyframe_time.total_time / image_count)
print("Average backend time per frame:", backend_time.total_time / image_count)
print("Average semantic time per frame:", solver.clip_timer.total_time / image_count)
```

Note that `image_count` is the number of *raw* frames the keyframer saw
(incremented at [VERIFY: main.py:113]), not the number of submaps, so
"average per frame" mixes the cost of a submap across all of its
member frames.

---

## 13. End-of-run flush

When the image loop exits:

[VERIFY: main.py:213-225]
```python
if not args.vis_map:
    # just show the map after all submaps have been processed
    solver.update_all_submap_vis()

if args.log_results:
    solver.map.write_poses_to_file(args.log_path, solver.graph, kitti_format=False)
    if not args.skip_dense_log:
        # Log the dense point cloud for each submap.
        solver.map.save_framewise_pointclouds(solver.graph, args.log_path.replace(".txt", "_logs"))
```

* `update_all_submap_vis` is called *only if `--vis_map` was off* — it
  is the "show me the map at the end" branch.
* Final pose dump overwrites the periodic one, producing the
  authoritative trajectory text file.
* If `--skip_dense_log` is *not* set, per-frame `.npz` files are also
  written (`save_framewise_pointclouds`,
  [VERIFY: vggt_slam/map.py:164-175]).

---

## 14. Summary — what is **immutable** vs. **mutated**

| Lifecycle                                                   | Mutated?                                                                                                                       |
| ----------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| `Submap.pointclouds` (local-frame)                          | **No** — written once, read everywhere via `graph.get_homography(...) @ p` on demand.                                          |
| `Submap.poses` (world-to-cam, local)                        | **No** — written once. The SL(4) updates live in `PoseGraph.values`, not on the submap.                                       |
| `Submap.conf` / `conf_masks`                                | **No** after `add_points` — but Apricot reserved for an auto-cal pass.                                                         |
| `PoseGraph.values`                                          | **Yes** — replaced wholesale by `optimize()` ([VERIFY: vggt_slam/graph.py:127]).                                                |
| `PoseGraph.graph` (factors)                                 | **Yes (append-only)** — every `add_homography`/`add_between_factor` adds to it; nothing removes factors.                       |
| `GraphMap.submaps`                                          | **Yes (insert-only)** — `add_submap` only grows the dict.                                                                     |
| `Viewer.submap_frames` / `submap_frustums`                  | **Overwritten** — `visualize_frames` replaces each list when called twice for the same submap (e.g. after a loop closure).    |
| `FrameTracker.{kf_pts, kf_gray, last_kf}`                   | **Yes** — `initialize_keyframe` replaces all three when a new keyframe is selected.                                            |

This split is the reason a loop closure does not have to re-run VGGT
on the *old* submaps: their raw outputs are immutable, only the SL(4)
nodes that transform them into the global frame change.

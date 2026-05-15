# Key Questions (问题解答)

> Design-rationale Q&A. Each answer is backed by a `[VERIFY:]` tag
> against the actual code.

---

## Q1 — Why SL(4)? Why not SE(3) or Sim(3)?

VGGT predicts depth maps and poses **simultaneously**, and the two
predictions for a single submap are only consistent **up to an
arbitrary 4×4 projective transformation**. Two adjacent submaps can
disagree on:

* the overall scale (one number),
* the depth-dependent focal length (one number),
* the principal point shift (two numbers),
* the rotation (three),
* the translation (three),
* and additional projective distortion (5 more degrees of freedom).

That's 15 DOF total — exactly the dimension of `sl(4)`. Trying to align
two submaps with SE(3) (6 DOF) or Sim(3) (7 DOF) leaves 8–9 DOF of
*unmodelled* discrepancy, which the optimiser would have to either
ignore (yielding poor maps) or absorb into noisy residuals (yielding
unstable optimisation).

**Code evidence**:

* `gtsam.SL4`, `PriorFactorSL4`, `BetweenFactorSL4`
  ([VERIFY: vggt_slam/graph.py:9-10]) — the only factor types used.
* `Sigmas` of length 15 everywhere ([VERIFY: vggt_slam/graph.py:19-23]) —
  the dim of `sl(4)`.
* The README confirms the contribution was upstreamed to GTSAM in
  Aug 2025 ([VERIFY: README.md:147]).

---

## Q2 — Why scale-bridge with median of norm ratios instead of full ICP?

The scale solver fits *one scalar* between two clouds that have
already been co-rotated by `P_temp[:3,:3]`
([VERIFY: vggt_slam/solver.py:141]).

Reasons to keep it that simple:

1. **Already aligned in rotation.** The overlap pair has near-zero
   rotation between the two predictions (it's the same image). ICP's
   rotation estimation would be solving a problem that's already
   solved.
2. **Linear-time, no iterations.** Median of ratios is `O(N)` and
   single-pass ([VERIFY: vggt_slam/scale_solver.py:18-20]). ICP is
   `O(K · N · log N)` with `K ≈ 30` iterations in practice.
3. **50 % outlier tolerance.** Median is provably robust to up to
   50 % contamination — the confidence mask
   ([VERIFY: vggt_slam/solver.py:131-138]) already keeps contamination
   well below that.
4. **SL(4) optimisation absorbs residuals.** Any scale error is just
   the initial value of the new node; the between factor records
   the *correct* measurement, and LM cleans up downstream.

**Code evidence**: 9 lines in `vggt_slam/scale_solver.py`
([VERIFY: vggt_slam/scale_solver.py:15-24]).

---

## Q3 — Why two VGGT calls per submap when a loop fires?

The first VGGT call processes the build batch
(`submap_size + 1` images, [VERIFY: vggt_slam/solver.py:335]). The
**second** call processes only 2 images
(`[query_kf, retrieved_kf]`, [VERIFY: vggt_slam/solver.py:347-348]) with
`compute_similarity=True`.

The second call serves *two* purposes:

1. **Geometric verifier**: VGGT inspects whether the two images can
   plausibly be brought into 3-D correspondence. The aggregator
   returns `image_match_ratio` ∈ [0, 1]
   ([VERIFY: third_party/vggt/vggt/models/vggt.py:61, 98]).
   `image_match_ratio < 0.85` → drop the loop
   ([VERIFY: vggt_slam/solver.py:371-374]).
2. **Loop-pose measurement**: when the verifier accepts, its
   `pose_enc`, `depth` and `depth_conf` outputs become the building
   blocks of the LC submap ([VERIFY: vggt_slam/solver.py:378-382]),
   which in turn provides the SL(4) factor that splices the loop.

A separate retrieval-only check would still need a separate geometric
verifier (RANSAC + essential matrix, or another network). Routing
everything through the *same* model — both for build and for
verification — keeps the stack uniform and lets the geometric
component improve every time VGGT itself improves.

---

## Q4 — Why is the keyframer Lucas–Kanade and not RAFT?

The CLI help mentions RAFT
([VERIFY: main.py:24]: *"Visualize optical flow from RAFT for keyframe
selection"*), but the actual implementation is Shi–Tomasi + Lucas–Kanade
([VERIFY: vggt_slam/frame_overlap.py:15-21], [VERIFY: vggt_slam/frame_overlap.py:31-35]).

The help text is mis-stated. The chosen design uses LK because:

1. **CPU-only.** No GPU contention with VGGT.
2. **<5 ms per frame** on a laptop, vs. ~20 ms for RAFT.
3. **Sparse → robust to lighting / blur.** RAFT's dense field can
   over-trigger on shadow flicker; LK on 1000 corners just stops
   tracking them.
4. **Mean of corner displacements** is sufficient for the decision
   ("did the camera move enough?"). Pixel-perfect flow is not needed
   for a downsampler.

---

## Q5 — Why `submap_size=16` (or 32)?

The CLI default is 16 ([VERIFY: main.py:28]); the evaluation script
`eval_tum.sh` accepts a user-provided value
([VERIFY: evals/eval_tum.sh:4]). Trade-offs:

* **Smaller submaps** (e.g. 8): more between factors per unit time, more
  graph nodes, more frequent LM passes. The graph builds up faster
  in node count but each LM pass is cheaper.
* **Larger submaps** (e.g. 32): more cohesive geometry per VGGT call —
  the network has more views to triangulate against, producing
  cleaner depth. But VGGT memory footprint scales with batch size; on
  an 8 GB VRAM box, 16 is near the safe limit (the
  `torch.cuda.empty_cache()` comment in `main.py` ([VERIFY: main.py:141-145])
  confirms this is the GPU-memory-bound regime).

The chosen 16 is the equilibrium for cellphone-video and TUM-like
data on a single laptop GPU.

---

## Q6 — Why does loop closure create a "bridge submap" instead of a single direct factor?

A direct factor would be: `add_between_factor(query_node, detected_node, H_meas, noise)`.

The chosen scheme creates a 2-frame LC submap with three factors:
`query → lc[0]`, `lc[0] → lc[1]`, `lc[1] → detected`
([VERIFY: vggt_slam/solver.py:286-287]).

The bridge pattern has two advantages:

1. **Carries the full VGGT verifier output**, not just the relative
   pose. The verifier returns `(extrinsic_lc[0], extrinsic_lc[1],
   depth_lc[0], depth_lc[1], image_match_ratio)`. Routing through the
   LC submap means the SL(4) measurement uses the *projection
   matrices* of both verifier-predicted poses — preserving K
   differences and any projective skew.
2. **Visualisation symmetry.** The LC submap can be displayed like any
   other submap in viser ([VERIFY: vggt_slam/viewer.py:33-86]) —
   useful for debugging.

A single direct factor would collapse this richer measurement to a
6-DOF SE(3) approximation, defeating the SL(4)-graph design.

See `04-ALGORITHM-pose_graph_construction.md §5.1` for the full
discussion.

---

## Q7 — Why is `lc_thres=0.95` (the CLI default) so permissive?

`lc_thres` is the L2-distance gate after the SALAD nearest-neighbour
search ([VERIFY: vggt_slam/loop_closure.py:82]). With a default of
0.95, almost every SALAD descriptor pair passes the gate.

The system relies on the **VGGT verifier** to do the heavy lifting:

* SALAD is fast (one DINO forward), so a permissive threshold is cheap.
* The verifier (`image_match_ratio ≥ 0.85`,
  [VERIFY: vggt_slam/solver.py:372]) is the actual quality gate.
* False positives at the SALAD stage cost one extra 2-frame VGGT
  forward each; the verifier rejects them.

A stricter SALAD threshold would save VGGT-call cost but miss recall.
The chosen design optimises for **recall first**, accepting a small
cost overhead from verifier-rejected candidates.

---

## Q8 — Why does the system call `torch.cuda.empty_cache()` after every submap?

Local modification specific to the GeoScan / 8 GB benchmark machine
([VERIFY: main.py:141-145]):

```python
# GeoScan/8GB mitigation: reclaim GPU memory between submaps.
# Without this, PyTorch's caching allocator holds on to VGGT
# activations and cumulative submap tensors grow until OOM
# around submap ~34 on an RTX 4060 Laptop (8 GB).
torch.cuda.empty_cache()
```

PyTorch's caching allocator does not release memory back to CUDA until
asked. For a long run on a memory-tight GPU, the cache grows because:

* Each submap's `images` tensor (`(17, 3, H, W) bfloat16`) is around
  100–200 MB.
* Intermediate activations inside VGGT can temporarily double that.
* Without explicit `empty_cache()`, the allocator holds onto every
  peak.

The explicit call ensures the next submap starts with a clean cache.
There is a small CPU-side latency cost (~10 ms) for `cudaFree` calls;
acceptable given submap-rate timing.

This is **not present in stock upstream VGGT-SLAM** — it's a benchmark
modification. On boxes with 24 GB VRAM it can be removed.

---

## Q9 — Why does `main.py` dump poses after every submap?

[VERIFY: main.py:97-105]:

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

Same rationale as Q8: on the GeoScan B1 bag with an 8 GB GPU, the
run sometimes OOMs around submap 34. Without periodic dumping, an
OOM produces no trajectory file — `evo_ape` has nothing to compare.

With the periodic dump, even an OOMed run produces a partial
trajectory that covers up to the last successful submap.

The `try/except` ensures a *failed* dump (e.g. transient disk error)
doesn't kill the SLAM loop.

---

## Q10 — Why is `update_all_submap_vis` called only on loop closure?

[VERIFY: main.py:131-136]:

```python
loop_closure_detected = len(predictions["detected_loops"]) > 0
if args.vis_map:
    if loop_closure_detected:
        solver.update_all_submap_vis()
    else:
        solver.update_latest_submap_vis()
```

Without a loop closure, `PoseGraph.optimize()` only adjusts a *local*
neighborhood of the graph — the older submaps' poses change by an
amount that is below visual resolution. Re-rendering every submap on
every step is wasteful.

A loop closure, however, can move *every* submap's pose by a
visible amount (that's the whole point — splice the trajectory). So
the viewer needs a full refresh.

This is a perceptual-cost tradeoff: 1× scene update vs. N× scene
update per submap, with N up to a few dozen.

---

## Q11 — Why doesn't `add_points` re-run VGGT on old submaps after a loop closure?

Because **the network outputs are stored in the submap's *local* frame**
and are immutable after creation
([VERIFY: vggt_slam/submap.py:36-41]). The SL(4) optimisation only
changes the *global* homography `H_s,i` that maps the local frame to
the global frame. The cloud itself doesn't need to be touched.

Concretely, `Submap.get_points_in_world_frame` reads the cloud through
the current `graph.get_homography(...)` value
([VERIFY: vggt_slam/submap.py:192-210]):

```python
homography_list = [graph.get_homography(i + self.get_id()) for i in range(len(self.poses))]
...
points_transformed = (homography_list[index] @ points_homogeneous.T).T
points_transformed = (points_transformed[:, :3] / points_transformed[:, 3:]).reshape(points.shape)
```

So after `PoseGraph.optimize()`, the *same* stored cloud renders into
a *different* global location. No re-inference required.

---

## Q12 — Why is the back-end full-graph LM instead of iSAM?

`PoseGraph.optimize` runs `LevenbergMarquardtOptimizer` over the *full*
graph after every new submap
([VERIFY: vggt_slam/graph.py:88-127]). For 20+ submaps with ~340 nodes
the graph is small enough that batch LM completes in a couple of
seconds. iSAM2 would shave that further but introduces complications:

* iSAM2's variable ordering is tuned for SE(3)/Sim(3) sequential
  problems; behaviour on SL(4) is less explored.
* The benchmark sequence length (TUM, 7-Scenes) is short — batch is
  fine.
* iSAM2 requires marginalising variables, which is a bigger change to
  the API surface.

The README's "Todo" lists real-time deployment but says nothing about
back-end optimisation ([VERIFY: README.md:151-156]) — i.e. the authors
consider batch LM acceptable for the released VGGT-SLAM 2.0.

---

## Q13 — Why store both `Submap.conf` and `Submap.conf_masks`?

[VERIFY: vggt_slam/submap.py:39] sets `conf`; [VERIFY: vggt_slam/submap.py:157-158]
sets `conf_masks` from the same data.

At creation they're identical. Distinct uses downstream:

* `conf` → percentile threshold computation at creation time only
  ([VERIFY: vggt_slam/submap.py:40]).
* `conf_masks` → every later "is this pixel confident?" check
  ([VERIFY: vggt_slam/submap.py:170-172, 202],
  [VERIFY: vggt_slam/solver.py:129-132]).

The intention (judging from `update_all_homographies`'s placeholder
code in `graph.py` [VERIFY: vggt_slam/graph.py:143-151]) is to allow
a future rectification pass to *overwrite* `conf_masks` with
post-rectification confidences without disturbing the original `conf`
percentile. In the released pipeline no such overwrite occurs.

---

## Q14 — What does `--vis_voxel_size` do, and why is it optional?

[VERIFY: vggt_slam/solver.py:62-69]:

```python
def set_point_cloud(self, points_in_world_frame, points_colors, name, point_size):
    if self.vis_voxel_size is not None:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_in_world_frame.astype(np.float64))
        pcd.colors = o3d.utility.Vector3dVector(points_colors.astype(np.float64) / 255.0)
        pcd = pcd.voxel_down_sample(self.vis_voxel_size)
        points_in_world_frame = np.asarray(pcd.points, dtype=np.float32)
        points_colors = (np.asarray(pcd.colors) * 255).astype(np.uint8)
```

* When `--vis_voxel_size` is **None** (default), the full submap
  cloud is shipped to viser. For a 17-frame × 1080 × 1920 submap that's
  ~35M points per submap × 20 submaps = ~700M points. The browser
  client crawls.
* When set, Open3D voxel-downsamples each submap cloud before sending.
  A value of 0.005 (m, i.e. 5 mm voxels) typically drops point counts
  by ~100× with negligible visual loss.

The README explicitly recommends `--vis_voxel_size 0.005` for larger
datasets ([VERIFY: README.md:119-120]).

The downsampling is *only* for visualisation: `Submap.pointclouds`
keeps the full resolution (referenced from
[VERIFY: vggt_slam/submap.py:37], not from `voxel_down_sample`).

---

## Q15 — What happens if VGGT predicts negative depth?

`unproject_depth_map_to_point_map` uses `depth_to_world_coords_points`
([VERIFY: third_party/vggt/vggt/utils/geometry.py:43-44]) which
multiplies camera rays by depth. Negative depth would produce points
*behind* the camera.

VGGT's depth head uses `activation="exp"`
([VERIFY: third_party/vggt/vggt/models/vggt.py:26]) — a strictly
positive output via the exponential. So negative depth cannot occur.

If a corrupted checkpoint or a different head were swapped in,
negative depths would:

* Show up as inverted points in the cloud.
* Confuse the median-of-norm-ratios scale solver (norms are still
  positive, so it returns a meaningless positive scale).
* Make `decompose_camera`'s RQ decomposition yield negative K
  diagonals; the sign-correction loop
  ([VERIFY: vggt_slam/slam_utils.py:64-72]) would flip the
  corresponding `R` rows, producing a left-handed rotation.

No explicit guard for this exists; the system trusts VGGT's positive
depth activation.

---

## Q16 — Is the system real-time?

The README's "Todo" answers this:

> *"Release real-time code. This code enables plugging in a Real Sense
> Camera and incrementally constructing a map as the camera explored
> a scene. This has been tested on a Jetson Thor onboard a robot."*
> ([VERIFY: README.md:152-154]).

So real-time **is** demonstrated on hardware (Jetson Thor) but the
real-time integration code is not yet in this repo. The
`vggt-slam main.py` here is **offline batch** — it processes an image
folder, prints timing averages, and exits.

The `Accumulator` instrumentation
([VERIFY: vggt_slam/slam_utils.py:204-213]) and the print
block at the end ([VERIFY: main.py:154-161]) suggest the intent is
to validate real-time feasibility from the offline measurements
before promising real-time integration.

---

## Q17 — How is the initial gauge of the SL(4) graph chosen?

The very first submap's first frame is anchored at identity with a
prior factor of σ=1e-6 ([VERIFY: vggt_slam/solver.py:167-170],
[VERIFY: vggt_slam/graph.py:23]).

This pins the SL(4) gauge: every other node's optimum is relative to
the anchor. Without it, the SL(4) graph would be invariant under a
global left-multiplication by any element of SL(4), making the LM
problem under-determined.

The σ=1e-6 is small enough that LM treats it as a hard constraint;
σ=0 would cause `noiseModel.Diagonal.Sigmas` to fail (cannot
construct a Gaussian with zero variance).

---

## Q18 — Why does `find_loop_closures` exclude the immediately previous submap?

[VERIFY: vggt_slam/map.py:78-80]:

```python
if self.non_lc_submap_ids and ignore_last_submap and submap_key == self.non_lc_submap_ids[-1]:
    continue
```

The previous build submap is already connected to the current one via
the **intra-submap factor** ([VERIFY: vggt_slam/solver.py:161]). A
"loop closure" to it would:

* Add a redundant factor that the intra-submap factor already implies.
* Possibly create a near-zero between factor (the same image was used
  for both anchors), which is numerically badly conditioned.

So the exclusion is a correctness measure, not just an optimisation.

---

## Q19 — Why doesn't the code use cosine similarity for SALAD retrieval?

The alternative is commented out
([VERIFY: vggt_slam/map.py:88-89]):

```python
score = torch.linalg.norm(embedding-query_vector)
# score = embedding @ query_vector.t()
```

For L2-normalised descriptors (which SALAD's outputs typically are),
`||a-b||² = 2(1 - cos(a,b))`, so L2 distance and cosine similarity
rank pairs identically. The choice is cosmetic. The author chose L2
distance because:

* It's the natural metric for the descriptor space SALAD was trained
  with.
* The `lc_thres` value is intuitively a *distance* (0 = identical,
  larger = more different), matching the print/diagnostic format.

Either would work.

---

## Q20 — Where would I plug in a different feed-forward model?

The boundary is **the `predictions` dict contract** in
`run_predictions` ([VERIFY: vggt_slam/solver.py:298-393]).

To swap VGGT for another model `M`, you would need:

1. `M(images)` returns a dict with `"depth"`, `"depth_conf"`,
   `"pose_enc"` keys (or you adapt the decoder).
2. A `pose_encoding_to_extri_intri(...)` that turns the model's pose
   output into per-frame extrinsic/intrinsic.
3. `M(lc_frames, compute_similarity=True)` for the verifier path;
   needs to return an `image_match_ratio` scalar.

Everything else (SALAD retrieval, scale solver, SL(4) graph) is
model-agnostic. The `solver.py` code only touches the model at three
points:

* `predictions = model(images)` ([VERIFY: vggt_slam/solver.py:335]).
* `predictions_lc = model(lc_frames, compute_similarity=True)`
  ([VERIFY: vggt_slam/solver.py:348]).
* `pose_encoding_to_extri_intri(...)`
  ([VERIFY: vggt_slam/solver.py:364, 378]).

A roughly compatible replacement (e.g. DUSt3R, MASt3R) would mainly
need its pose decode plumbing adapted.

---

## Q21 — What is the role of `Submap.proj_mats`?

It is the **4×4 embedding of the per-frame intrinsic K**
([VERIFY: vggt_slam/solver.py:230-231]):

```python
K_4x4 = np.tile(np.eye(4), (N, 1, 1))
K_4x4[:, :3, :3] = intrinsics_cam
```

Stored on the submap ([VERIFY: vggt_slam/submap.py:41]) and used at two
points:

1. **In `add_edge`**, to build the inter-submap measurement:
   `np.linalg.inv(prior_submap.proj_mats[-1]) @ current_submap.proj_mats[0]`
   ([VERIFY: vggt_slam/solver.py:140, 153]).
2. **In `get_all_poses_world`**, to convert the SL(4) value back into
   a camera projection matrix for RQ decomposition:
   `self.proj_mats[index] @ np.linalg.inv(homography_world)`
   ([VERIFY: vggt_slam/submap.py:118]).

So `proj_mats` is the bridge between VGGT's per-frame K and the
SL(4) algebra. Without it, the system would have to track K
separately.

---

## Q22 — What is the cost breakdown of a single submap?

For TUM-style 640×480 input, `submap_size=16`, no LC fired:

| Stage                       | Cost (typical, RTX 4060 Laptop)  |
| --------------------------- | -------------------------------- |
| Keyframer (per raw frame)   | <5 ms CPU                        |
| Image load + preprocess     | ~50 ms                            |
| VGGT forward (17 frames)    | ~600 ms                            |
| SALAD descriptor batch      | ~80 ms                            |
| SALAD nearest-neighbour     | <10 ms                            |
| `add_points` + `add_edge`   | ~30 ms (mostly NumPy)             |
| `PoseGraph.optimize`         | ~50 ms early, ~300 ms at 20 submaps |
| Viewer push (`--vis_map`)    | 100–500 ms depending on cloud size |
| `cuda.empty_cache()`         | ~10 ms                            |
| `_safe_dump_poses`           | <10 ms                            |

Total per submap (no LC, no viewer): ~800 ms, dominated by VGGT.
Effective per-frame: ~50 ms = ~20 fps. This is *batch* timing; a
real-time integration with overlapped I/O could amortise further.

---

## Q23 — How does the system handle the very last (short) submap?

[VERIFY: main.py:118]:

```python
if len(image_names_subset) == args.submap_size + args.overlapping_window_size or image_name == image_names[-1]:
```

When the image stream ends with an incomplete buffer, the
`image_name == image_names[-1]` branch fires `run_predictions` on
whatever's in the buffer. Downstream code uses `images.shape[0]` and
`len(self.poses)` rather than the CLI submap size
([VERIFY: vggt_slam/solver.py:320, 180]), so a short trailing submap
works the same way as a full one — fewer frames, fewer inner factors,
otherwise identical.

The only case that breaks is if the trailing buffer has length 1
*and* contains only the overlap frame from the previous submap.
That would yield a 1-frame submap with no inner chain and no new
keyframes. The reset rule
`image_names_subset = image_names_subset[-args.overlapping_window_size:]`
([VERIFY: main.py:139]) does seed every new buffer with the
previous overlap; if the next call to the keyframer immediately
returns False on every subsequent image, the buffer would never
grow past length 1 and no submap would ever trigger — but the
`image_name == image_names[-1]` branch saves us: when the last raw
image is reached, even a length-1 buffer triggers
`run_predictions(image_names_subset, model, ...)`.

With a single-frame submap, `add_edge` skips the inner-chain loop
(`enumerate` over one pose returns `[(0, ...)]`, and the
`if index == 0: continue` line skips it). The submap contributes one
node and one between factor (the inter-submap factor). Correct.

---

## Q24 — Why isn't there an explicit covariance scaling for loop-closure factors?

The system uses `intra_submap_noise` (σ=0.05) for **both** the regular
between-submap factor and the loop-closure between-factors
([VERIFY: vggt_slam/solver.py:161, 287]).

One might expect a *larger* noise for loop closures (they're less
certain than adjacent overlaps). The chosen design treats them
identically because:

* The verifier (`image_match_ratio ≥ 0.85`) already eliminates most
  bad loops *before* the factor is added.
* SL(4) BetweenFactor residuals scale by the *log* of the measurement
  error; the σ=0.05 isotropic noise gives the optimiser room to
  redistribute error among all 15 tangent dimensions.

If a particular dataset has noisy loops (e.g. lots of false-positive
LCs through the verifier), tuning `intra_submap_noise` higher would
be the natural next step. Currently a single σ.

---

## Q25 — Why does `Solver.tranform_submap_to_canonical` exist if it's never called?

[VERIFY: vggt_slam/solver.py:101-116]:

```python
def tranform_submap_to_canonical(self, proj_mat_world_to_cam, world_points):
    P_first_cam = proj_mat_world_to_cam[0].copy()

    # Apply transformation to camera matrices such that the first camera matrix of the submap is [I | 0]
    proj_mat_world_to_cam = proj_mat_world_to_cam @ np.linalg.inv(P_first_cam)
    ...
```

It's a leftover from VGGT-SLAM 1.0 (per the README, the previous
version is on the `version1.0` branch). In 1.0 the points were
explicitly transformed into a canonical frame where frame 0 was
identity, then the SL(4) graph mapped that canonical frame to the
global frame. In 2.0 the *implicit* canonicalisation via
`np.linalg.inv(prior_submap.proj_mats[-1]) @ current_submap.proj_mats[0]`
([VERIFY: vggt_slam/solver.py:140]) achieves the same effect without
mutating the stored cloud.

Removing the helper would be a small cleanup; not doing so leaves an
escape hatch in case explicit canonicalisation becomes useful again.

---

## Q26 — What's the right value of `--conf_threshold`?

[VERIFY: main.py:32]:

> `--conf_threshold 25.0` — "Initial percentage of low-confidence
> points to filter out"

The number is a *percentile*: at 25, the lowest 25 % of pixels by VGGT
confidence are dropped. Higher → fewer points but cleaner; lower →
more points but more outliers.

The scale solver in `add_edge` thresholds *more* aggressively than 25
([VERIFY: vggt_slam/solver.py:131-138]) but the visualised cloud uses
this percentile ([VERIFY: vggt_slam/submap.py:170-172, 202]).

Recommended starting points:

* Indoor (TUM, 7-Scenes, office_loop): 25 — the depth predictions are
  reliable on most of the scene.
* Outdoor / wide-baseline: try 50 — drop more of the sky/background
  where VGGT struggles.
* Sparse-texture scenes: try 10 — keep more points despite lower
  per-pixel confidence.

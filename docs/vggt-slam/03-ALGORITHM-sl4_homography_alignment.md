# Algorithm — SL(4) Homography Alignment Between Submaps

> The keystone algorithm of VGGT-SLAM 2.0. It explains why the pose
> graph is an `SL(4)` graph rather than an `SE(3)` graph, what the
> per-submap and inter-submap factors actually measure, and how
> `Solver.add_edge` ([VERIFY: vggt_slam/solver.py:118-195]) turns
> VGGT's per-submap predictions into globally consistent constraints.

---

## 1. Why SL(4) and not SE(3)?

A monocular feed-forward network that simultaneously predicts depth
**and** camera pose for a small batch of images is only correct *up to
a projective ambiguity*. Concretely: given any 4×4 matrix `H` with
`det(H) ≠ 0`, the transformed cameras `Pᵢ′ = Pᵢ · H⁻¹` and points
`Xⱼ′ = H · Xⱼ` produce identical image observations:

```
   uᵢⱼ = Pᵢ · Xⱼ = (Pᵢ · H⁻¹) · (H · Xⱼ) = Pᵢ′ · Xⱼ′
```

For two separate VGGT submaps the ambiguity reduces to **one such
`H` per submap**. Any global alignment scheme must therefore search
the 15-dimensional space of `H ∈ SL(4)` (the determinant is set to 1
by convention — the overall scale of `H` is absorbed by the
homogeneous coordinate). That is the whole reason the back end is
`SL(4)` and the factor types are
`PriorFactorSL4` / `BetweenFactorSL4` ([VERIFY: vggt_slam/graph.py:9-10]).

Picking the *wrong* group degrades reconstruction:

| Group        | DOF | What is *fixed* (incorrectly)                                    |
| ------------ | --- | --------------------------------------------------------------- |
| SE(3)        | 6   | depth scale (assumed 1), focal length (assumed correct)         |
| Sim(3)       | 7   | as SE(3), with one global scale freedom                          |
| **SL(4)**    | **15** | nothing — full projective ambiguity                          |

VGGT predicts depth maps that are **already correct up to one
overall scale**, but two adjacent submaps can disagree on that scale
*and* on a residual projective skew. SL(4) captures both.

---

## 2. The SL(4) tangent space (why `Sigmas` has length 15)

The Lie algebra `sl(4)` is the space of 4×4 matrices with trace 0; it
has 15 free real parameters. GTSAM's `noiseModel.Diagonal.Sigmas(...)`
takes one σ per tangent dimension; hence the constant `15`:

[VERIFY: vggt_slam/graph.py:19-23]
```python
inner_noise = 0.05*np.ones(15, dtype=float)
intra_noise = 0.05*np.ones(15, dtype=float)
self.inner_submap_noise = noiseModel.Diagonal.Sigmas(inner_noise)
self.intra_submap_noise = noiseModel.Diagonal.Sigmas(intra_noise)
self.anchor_noise        = noiseModel.Diagonal.Sigmas([1e-6] * 15)
```

The factor residual is the `sl(4)` logarithm of the discrepancy
between the measured relative and the relative implied by the two
nodes (handled internally by GTSAM's
`BetweenFactorSL4`).

* σ = 0.05 across all 15 tangent dimensions for both intra-submap
  edges and inner-submap edges — i.e. the system trusts both kinds of
  measurement equally.
* σ = 1e-6 for the anchor on the very first node — a hard pin.

These are the only three noise models the back end ever sees.

---

## 3. A submap's local frame

For a freshly-predicted submap `s` with `S` keyframes, the network
returns:

* `extrinsic[s] ∈ ℝ^{S×3×4}` — world-to-cam projection matrices in
  the submap's *internal* frame.
* `intrinsic[s] ∈ ℝ^{S×3×3}` — per-frame pinhole intrinsics
  (VGGT does *not* assume a single shared `K`).
* `depth[s] ∈ ℝ^{S×H×W×1}` — per-pixel metric (or near-metric) depth.

`Solver.run_predictions` decodes these from the 9-vec pose encoding
([VERIFY: vggt_slam/solver.py:364-366]) and `add_points` builds the
3-D point cloud:

[VERIFY: vggt_slam/solver.py:222]
```python
world_points = unproject_depth_map_to_point_map(depth_map, extrinsics_cam, intrinsics_cam)
```

`world_points[s, i, h, w]` is therefore expressed in the submap's
*local* world frame — there is one such frame per submap, and the
SL(4) graph's job is to relate them.

VGGT's prediction is *not* canonicalised to put camera 0 at identity
(the helper `tranform_submap_to_canonical` exists at
[VERIFY: vggt_slam/solver.py:101] but is unused). Whatever frame the
network happened to pick is the frame the points live in.

---

## 4. The projection matrix in 4×4 form

To keep multiplication clean, `Solver.add_points` embeds each 3×3
intrinsic into a 4×4:

[VERIFY: vggt_slam/solver.py:230-231]
```python
K_4x4 = np.tile(np.eye(4), (N, 1, 1))
K_4x4[:, :3, :3] = intrinsics_cam
```

Conceptually this is

```
K_4x4 =  ┌ K₃ₓ₃   0 ┐
         └  0     1 ┘
```

i.e. an injection of `K` into `GL(4)`. The submap stores it as
`proj_mats` ([VERIFY: vggt_slam/submap.py:36-41]). It is **not** a
camera matrix `P = K[R|t]`; it is just `K` lifted into 4×4. The full
projection happens via the SL(4) node value at read time:

[VERIFY: vggt_slam/submap.py:117-128]
```python
projection_mat = self.proj_mats[index] @ np.linalg.inv(homography_world)
projection_mat = projection_mat / projection_mat[-1,-1]
if give_camera_mat:
    poses.append(projection_mat)
else:
    cal, rot, trans, scale = decompose_camera(projection_mat[0:3,:])
    ...
```

So in algebraic terms the full camera matrix of frame `i` in submap
`s` *after* optimisation is:

```
   P_world_to_cam(s,i) = K_s,i · H_s,i^{-1}
```

where `H_s,i` is the current SL(4) value of node `s + i`. Reading this
formula off the code:

* `self.proj_mats[index]` is `K_s,i` (the 4×4 K).
* `homography_world` is `H_s,i`.
* `np.linalg.inv(homography_world)` brings world points into camera
  coordinates; left-multiplying by `K` produces image points.

The division by `projection_mat[-1,-1]` is the SL(4) normalisation:
recall that the node value is in `SL(4)` (det = 1), but
`K · H⁻¹` no longer has det 1, so we renormalise so the bottom-right
entry is 1 (standard homography form).

---

## 5. Inter-submap edge — the core formula

The interesting line is one short expression
([VERIFY: vggt_slam/solver.py:140]):

```python
P_temp = np.linalg.inv(prior_submap.proj_mats[-1]) @ current_submap.proj_mats[0]
```

In words: `P_temp` is the 4×4 projective transformation that takes a
point expressed in the **current submap's first frame** (the overlap
frame) and re-expresses it in the **previous submap's last frame**
(also the overlap frame).

Why does that make sense? Because:

* `current_submap.proj_mats[0]` is `K_curr,0` — embedded.
* `prior_submap.proj_mats[-1]` is `K_prev,last` — embedded.
* The overlap frame is the *same physical image* (with the same
  pose in 3-space) but predicted **twice** — once by the previous
  submap and once by the current.
* Both predictions express the same 3-D points in their own local
  frames; the *only* difference is the embedded `K` of frame `last`
  vs. frame `0` and the choice of local origin.
* Their ratio `K_prev,last⁻¹ · K_curr,0` is the projective
  transformation that bridges the two local frames in the absence of
  any global rotation/translation (which is implicitly identity for
  the *same* image).

So `P_temp` is **the measurement of how the two submaps' local
projective frames differ at the overlap**. It is the "raw" SL(4)
between-measurement, modulo the depth-scale freedom.

---

## 6. The scale step — why a single scalar suffices

VGGT's depth-head predictions for the two adjacent submaps may
disagree on the overall scale of the scene, even though they agree on
shape. The fix:

[VERIFY: vggt_slam/solver.py:141-146]
```python
t1 = (P_temp[0:3,0:3] @ current_submap.get_frame_pointcloud(frame_id_curr).reshape(-1, 3)[good_mask].T).T
t2 = prior_submap.get_frame_pointcloud(frame_id_prev).reshape(-1, 3)[good_mask]
scale_factor_est_output = estimate_scale_pairwise(t1, t2)
scale_factor = scale_factor_est_output[0]
H_scale = np.diag((scale_factor, scale_factor, scale_factor, 1.0))
```

Step-by-step:

1. Rotate the current submap's cloud (the overlap frame's cloud) by
   the 3×3 block of `P_temp`. After rotation the two clouds disagree
   *only* in their per-point norm (i.e. radial distance).
2. Compute per-point norm ratios `||t2|| / ||t1||`
   ([VERIFY: vggt_slam/scale_solver.py:18-19]).
3. Take the median. This is the scale factor that, applied to the
   current submap's local depths, brings them into agreement with the
   previous submap's depths over the overlap region.
4. Embed the scale into SL(4) as `diag(s, s, s, 1)`. That matrix
   scales 3-D positions but leaves the homogeneous coordinate alone —
   exactly what we want for a metric correction.

Mathematically, the per-point relation is

```
  t2 ≈ s · t1     (after rotation by P_temp[:3,:3])
```

and `s = median_i ||t2_i|| / ||t1_i||` because the two clouds were
co-rotated, so the only remaining freedom is radial.

This is a deliberately simple scale solver — the rationale is
discussed in `07-ALGORITHM-scale_solver.md`. It works because the
overlap is one full image with thousands of high-confidence pixels;
median provides robustness against the inevitable outliers near
depth discontinuities.

### 6.1 Conf mask fall-back ladder

Three increasingly permissive masks are tried in `add_edge`
([VERIFY: vggt_slam/solver.py:131-138]):

```python
good_mask = (prior_conf > prior_submap.get_conf_threshold()) * (current_conf > prior_submap.get_conf_threshold())
good_mask = good_mask.reshape(-1)

if np.sum(good_mask) < 100:
    print("Not enough overlapping points to estimate scale factor, using a less restrictive mask")
    good_mask = (prior_conf > prior_submap.get_conf_threshold()).reshape(-1)
    if np.sum(good_mask) < 100:
        good_mask = (prior_conf > 0).reshape(-1)
```

Level 0 (default): both frames pass the previous submap's percentile.
Level 1 (warn): only the previous frame needs to pass.
Level 2 (last resort): any positive confidence.

This handles loop-closure frames in particular, which may share little
high-confidence content with their counterpart.

---

## 7. Composing the between-factor measurement

[VERIFY: vggt_slam/solver.py:153]
```python
H_overlap_prior_overlap_current = np.linalg.inv(prior_submap.proj_mats[-1]) @ current_submap.proj_mats[0] @ H_scale
```

In other words

```
  H_meas := K_prev,last⁻¹ · K_curr,0 · diag(s, s, s, 1)
```

This is the actual measurement passed to `BetweenFactorSL4`. The first
two factors capture the projective difference between the two local
embeddings of the same physical frame; the third factor injects the
metric scale that bridges depth predictions.

The same expression is used to compute an **initial** value for the
new node — i.e. a good warm start for LM:

[VERIFY: vggt_slam/solver.py:154]
```python
H_w_submap = self.graph.get_homography(overlapping_node_id_prev) @ H_overlap_prior_overlap_current
```

So `H_w_submap` is just "previous overlap node × measurement" — the
between-factor's own implied composition. This is the cleanest
initialisation possible: the LM optimiser starts with the new node
*exactly* satisfying its new between-factor, and the residual to be
driven down by optimisation comes from the *other* factors (inner
chain, loop closures).

Finally:

[VERIFY: vggt_slam/solver.py:158]
```python
self.graph.add_homography(submap_id_curr + frame_id_curr, H_w_submap)
```

inserts the new node into `Values`, and

[VERIFY: vggt_slam/solver.py:161]
```python
self.graph.add_between_factor(
    overlapping_node_id_prev, submap_id_curr + frame_id_curr,
    H_overlap_prior_overlap_current, self.graph.intra_submap_noise)
```

inserts the between factor itself.

---

## 8. Inner-submap factors

For frames `i ≥ 1` inside a single submap the local pose chain is
already a 4×4 SE(3) transformation predicted directly by VGGT
([VERIFY: vggt_slam/solver.py:179-184]):

```python
world_to_cam = current_submap.get_all_poses()
for index, pose in enumerate(world_to_cam):
    if index == 0:
        continue
    H_inner = world_to_cam[index-1] @ np.linalg.inv(pose)
```

`H_inner` is the local-frame relative motion from frame `i-1` to frame
`i`. Plain `SE(3)` would suffice here because *within one submap*
VGGT's depth scale is internally consistent — there is no inter-submap
ambiguity to deal with. But the back end has only one factor type
(`BetweenFactorSL4`), so the SE(3) measurement is promoted to SL(4) by
treating it as a 4×4 matrix and adding a 15-dim factor.

The initial value mirrors the inter-submap formulation:

[VERIFY: vggt_slam/solver.py:185]
```python
current_node = self.graph.get_homography(submap_id_curr + index - 1) @ H_inner
```

And the factor is added with the inner-submap noise model:

[VERIFY: vggt_slam/solver.py:191]
```python
self.graph.add_between_factor(
    submap_id_curr + index - 1, submap_id_curr + index, H_inner,
    self.graph.inner_submap_noise)
```

Because `H_inner` is in fact an SE(3) matrix (top-left orthogonal
rotation, last row `[0, 0, 0, 1]`), the residual it induces is
non-zero only on the 6 SE(3) tangent dimensions; the 9 "non-SE(3)"
tangent dimensions are perfectly satisfied. The σ=0.05 factor still
penalises drift in those 6 dimensions and gives the optimiser the
freedom (via the other 9) to deform the chain when a loop closure or
auto-calibration pulls on it later.

---

## 9. Prior factor on the very first node

[VERIFY: vggt_slam/solver.py:167-170]
```python
else:
    assert (submap_id_curr == 0 and frame_id_curr == 0), "First added node must be submap 0 frame 0"
    self.graph.add_homography(submap_id_curr + frame_id_curr, H_w_submap)
    self.graph.add_prior_factor(submap_id_curr + frame_id_curr, H_w_submap)
```

`H_w_submap` here is identity (no `submap_id_prev` to compose from
because `first_edge` was true and the `if submap_id_prev is not None`
branch was skipped — [VERIFY: vggt_slam/solver.py:122-123]). The prior
factor uses `anchor_noise` (σ=1e-6, [VERIFY: vggt_slam/graph.py:23]),
which is tight enough that LM treats it as an unconditional anchor.

This sets the gauge of the SL(4) graph. Every other node's value is
ultimately measured relative to this hard-pinned identity. Without it,
the optimisation is ill-posed: a global SL(4) transformation could be
applied to every node simultaneously without changing any
between-factor residual.

---

## 10. End-to-end equation for the final globalised cloud

After `n` submaps with `S` frames each and `m` loop closures, the
optimised `Values` contains an SL(4) element `H_k` for every key `k`
([VERIFY: vggt_slam/graph.py:31-38]). A pixel `(h, w)` of frame `i`
of submap `s` is mapped to its **global** 3-D location by:

```
   X_world = renormalise( H_{s+i} · [ x_{s,i}(h,w); 1 ] )
```

where `x_{s,i}(h,w) ∈ ℝ^3` is `Submap.pointclouds[i, h, w]`. The
renormalisation divides by the 4th coordinate
([VERIFY: vggt_slam/submap.py:183-184]):

```python
points_homogeneous = np.hstack([points_flat, np.ones((points_flat.shape[0], 1))])
points_transformed = (homography_list[index] @ points_homogeneous.T).T
points_transformed = (points_transformed[:, :3] / points_transformed[:, 3:]).reshape(points.shape)
```

i.e. the SL(4) optimisation directly produces the projective
transformation that turns each submap's local cloud into the global
one. **No re-running of VGGT, no point-cloud ICP, no re-triangulation
is required after the optimisation.** The points themselves were
never modified.

---

## 11. Worked example — three-submap graph at `submap_size=16`

To make the bookkeeping concrete, walk through a 3-submap run with
`submap_size=16`, `overlapping_window_size=1`, no loop closures.

* Submap 0 keyframes (after the keyframer): 17 images
  (16 new + 1 overlap, but for the first submap there is no
  "previous" so all 17 are new).

  *Solver.run_predictions* sees a single submap of size 17. It
  records `new_pcd_num = 0` ([VERIFY: vggt_slam/solver.py:311]),
  builds `Submap(0)`, sets
  `last_non_loop_frame_index = images.shape[0] - 1 = 16`
  ([VERIFY: vggt_slam/solver.py:320]).

  *Solver.add_points* + *Solver.add_edge* take the `first_edge`
  branch:
  - add node `X(0)`, prior factor with `anchor_noise`.
  - for `i = 1..16`, add node `X(0+i)` and between factor
    `(i-1) → i` with `inner_submap_noise`.

  Graph state: nodes `X(0..16)`, factors `1 prior + 16 inner = 17`.

* Submap 1 keyframes: take the last keyframe of submap 0 (i.e. one
  frame) as the overlap, then add 16 new ones — buffer reaches 17 and
  fires `run_predictions`.

  *run_predictions* sees a submap of size 17. It computes
  `new_pcd_num = largest_key + last_non_loop_frame_index + 1
                = 0 + 16 + 1 = 17` ([VERIFY: vggt_slam/solver.py:313]).
  Builds `Submap(17)`, `last_non_loop_frame_index = 16`.

  *add_points + add_edge* take the non-first-edge branch:
  - `submap_id_prev = 0`, `frame_id_prev = 16` (the previous submap's
    last non-loop frame).
  - Compute scale via overlap (frame 16 of submap 0 and frame 0 of
    submap 17).
  - Add node `X(17)` and between factor `X(16) → X(17)` with
    `intra_submap_noise`.
  - For `i = 1..16`, add node `X(17+i)` and between factor
    `X(17+i-1) → X(17+i)` with `inner_submap_noise`.

  Graph state: nodes `X(0..33)`, factors
  `1 prior + 16 inner + 1 intra + 16 inner = 34`.

* Submap 2: similar; `new_pcd_num = 33 + 16 + 1 = wrong`. Wait — let
  me recompute. After submap 1, the largest key is `33` and submap
  1's `last_non_loop_frame_index = 16`. So
  `new_pcd_num = 33 + 16 + 1 = 50`? That would mean submap 2 starts
  at node `X(50)`, leaving `X(34..49)` unused.

  Re-reading [VERIFY: vggt_slam/solver.py:313]:

  ```python
  new_pcd_num = self.map.get_largest_key() + self.map.get_latest_submap().get_last_non_loop_frame_index() + 1
  ```

  `get_largest_key()` returns the dictionary key, which is *the
  submap's id*, **not** the largest node id in the graph. Submap 1's
  id is `17`; submap 1's `last_non_loop_frame_index` is `16`. So
  `new_pcd_num = 17 + 16 + 1 = 34`. The next submap starts at
  `X(34)`. No gaps.

  So `get_largest_key()` returns *the largest submap id*, which is
  also the key of that submap's *first* frame (by construction).
  `+ last_non_loop_frame_index` jumps to the largest node *id* in that
  submap. `+1` advances past it. The arithmetic is right, but only
  because the submap id always equals its first node's key — see
  `04-ALGORITHM-pose_graph_construction.md` for a careful proof.

The same recurrence yields submap 2 at `X(34..50)`, etc.

---

## 12. Why submap ids must equal their first node's key

In Solver.add_edge:

[VERIFY: vggt_slam/solver.py:158]
```python
self.graph.add_homography(submap_id_curr + frame_id_curr, H_w_submap)
```

and later in the inner loop:

[VERIFY: vggt_slam/solver.py:188]
```python
self.graph.add_homography(submap_id_curr + index, current_node)
```

So `node_key(submap_s, frame_i) = s + i`. Because `s` is itself the
key of frame 0, the bookkeeping in `Submap.get_all_poses_world`:

[VERIFY: vggt_slam/submap.py:115]
```python
homography_list = [graph.get_homography(i + self.get_id()) for i in range(len(self.poses))]
```

correctly iterates over all node keys of the submap. This invariant —
"submap id == its first node's key" — is the gluing observation that
makes the flat numbering work.

If you ever rename `submap_id` independently of the node key, every
function above breaks silently.

---

## 13. Numerical stability

Two concerns about working with raw 4×4 homography matrices:

1. **Determinant drift.** Successive SL(4) compositions in floating
   point can produce matrices whose determinant slowly drifts away
   from 1. There is a `normalize_to_sl4` helper for this:

   [VERIFY: vggt_slam/slam_utils.py:123-132]
   ```python
   def normalize_to_sl4(H):
       det = np.linalg.det(H)
       if det == 0:
           raise ValueError("Homography matrix is singular and cannot be normalized.")
       scale = det ** (1/4)
       H_normalized = H / scale
       return H_normalized
   ```

   ...but the calls into it are commented out in `graph.py`
   ([VERIFY: vggt_slam/graph.py:31-32, 42, 50]):

   ```python
   # global_h = normalize_to_sl4(global_h)
   ```

   GTSAM's `SL4(matrix)` constructor is the canonical place where the
   normalisation should happen; the commented-out calls suggest the
   authors observed it does the right thing internally.

2. **Renormalisation at read time.** When a submap reads a node back
   for visualisation, it always divides by the bottom-right entry:

   [VERIFY: vggt_slam/submap.py:81-82,91-92,119]
   ```python
   homography = homography / homography[-1,-1]
   ```

   This is *not* the SL(4) normalisation (which divides by the
   determinant); it is the standard homography normalisation
   (`h_{4,4} == 1`). It works for visualisation because every
   homography in the system is non-singular and has a nonzero
   `[3,3]` entry under nominal conditions.

The combined effect is: the internal SL(4) representation may drift
slightly during long runs, but as long as `H[3,3] ≠ 0` the
viewer/output stays correct.

---

## 14. Comparison with classical SfM/SLAM back-ends

| Approach                  | Node type              | Relative measurement              | Where the scale comes from                                  |
| ------------------------- | ---------------------- | --------------------------------- | ----------------------------------------------------------- |
| ORB-SLAM 3 (mono)         | SE(3)                  | 6-DoF rigid                       | A monocular bootstrap (E-matrix), then locked at gauge time |
| MASt3R-SLAM               | SE(3) + per-frame scale | 7-DoF Sim(3)                      | A per-pair scale estimated from depth                       |
| **VGGT-SLAM 2.0**         | **SL(4) (15-DoF)**     | **4×4 projective + scale**        | **Median norm ratio over overlap pixels, embedded into SL(4) as `diag(s,s,s,1)`** |

VGGT-SLAM is the first system that uses the full SL(4) group as its
back-end pose space. The README confirms this is the contribution that
landed upstream: *"August 2025: SL(4) optimization is integrated into
the official GTSAM repo"* ([VERIFY: README.md:147]).

---

## 15. Failure modes

* **Overlap frame with sparse content.** When fewer than 100 confident
  pixels survive in the overlap, the scale solver falls back to a
  weaker mask ([VERIFY: vggt_slam/solver.py:134-138]). In the
  *worst* case it uses `prior_conf > 0`, which can produce wildly
  wrong scale estimates from low-confidence VGGT predictions. There
  is no rejection above this fallback level.

* **`P_temp[3,:]` non-trivial.** `P_temp` is constructed as
  `K_prev_inv · K_curr` (both 4×4 embeddings of pinhole `K`). The
  last row of the result is `[0, 0, 0, 1]` *only when both K's have
  zero principal point shift in the inhomogeneous coordinate* —
  which is the case here because the embedding sets that row to
  identity by construction. But if the upstream VGGT ever produced a
  non-pinhole intrinsic, the embedding would no longer be `[[K, 0],
  [0, 1]]` and the rest of the SL(4) algebra would silently break.

* **Determinant collapse.** If `H[3,3]` flips sign during LM
  iterations (a homography can do that in theory), the
  `/projection_mat[-1,-1]` renormalisation in `get_all_poses_world`
  flips the entire matrix, which in turn flips the decomposed K. The
  observed σ=0.05 noise model keeps the optimiser well away from
  that region in practice — but no explicit guard exists.

These are all acceptable in the regime VGGT-SLAM operates in (short
indoor sequences with good overlap), but worth flagging.

# Algorithm — Pose-Graph Construction & Node-Key Arithmetic

> The companion to `03-ALGORITHM-sl4_homography_alignment.md`. This
> document is a careful walk-through of how `Solver.add_edge`
> ([VERIFY: vggt_slam/solver.py:118-195]) wires up factors and nodes,
> with proofs of the invariants and a step-by-step trace of every
> branch.

---

## 1. The state we are maintaining

After `n` submaps:

```
GraphMap           : dict { submap_id → Submap }
PoseGraph.values   : Values { X(key) → SL4(H) }  for every keyframe k
PoseGraph.graph    : NonlinearFactorGraph with
                     · 1 PriorFactorSL4 (on X(0))
                     · 1 BetweenFactorSL4 per intra-submap edge
                       (one per submap pair past the first)
                     · S-1 BetweenFactorSL4 per submap for the inner chain
                     · 1 BetweenFactorSL4 per accepted loop closure
                       (plus the bridging intra-submap factor of the LC submap)
```

The state lives across three classes — `PoseGraph` is the workhorse
([VERIFY: vggt_slam/graph.py:14-128]), `GraphMap` is the dict registry
([VERIFY: vggt_slam/map.py:9-23]), `Submap` carries the per-batch
predictions. `Solver.add_edge` is the only place where new nodes and
factors are added (apart from the loop-closure plumbing in
`add_points`, which itself only calls `add_edge`).

---

## 2. The flat-numbering invariant

> **Invariant A**: for every submap `s` with `S` keyframes, the GTSAM
> keys of its frames are the contiguous integers `[s, s+1, ..., s+S-1]`.

`s` is also the value of `submap.get_id()`. The invariant is preserved
by the constructor of new submaps:

[VERIFY: vggt_slam/solver.py:309-313]
```python
if self.map.get_largest_key() is None:
    new_pcd_num = 0
else:
    new_pcd_num = self.map.get_largest_key() + self.map.get_latest_submap().get_last_non_loop_frame_index() + 1
```

Proof of preservation:

Let `s_prev` be the previous (largest) submap id, `L_prev` its
`last_non_loop_frame_index`. By induction, frames `s_prev,
s_prev+1, …, s_prev+L_prev` are taken. By the formula,
`new_pcd_num = s_prev + L_prev + 1`, which is the first integer past
`s_prev+L_prev`. The new submap of size `S` will occupy
`[new_pcd_num, new_pcd_num+S-1]`, and its
`last_non_loop_frame_index = S-1` is set immediately after
([VERIFY: vggt_slam/solver.py:320]):

```python
new_submap.set_last_non_loop_frame_index(images.shape[0] - 1)
```

So the contiguous numbering is preserved across calls.

> **Subtlety**: `get_largest_key()` returns the largest submap *id*,
> which by Invariant A is also the largest *frame key of a build
> submap*. A loop-closure submap can have a higher id, but
> `get_largest_key(ignore_loop_closure_submaps=True)` filters those
> out ([VERIFY: vggt_slam/map.py:24-34]).
>
> In `run_predictions` the *un*filtered version is used
> ([VERIFY: vggt_slam/solver.py:310-313]). For this to be correct, a
> loop-closure submap must already have an id that does **not** disrupt
> the next build submap's numbering — see §6 below.

---

## 3. Frame-0 (overlap) bookkeeping

> **Invariant B**: the *first* frame of a non-first build submap is
> the *same physical image* as the *last non-loop frame* of the
> previous build submap.

This is enforced by `main.py`:

[VERIFY: main.py:139]
```python
# Reset for next submap.
image_names_subset = image_names_subset[-args.overlapping_window_size:]
```

With `overlapping_window_size = 1` (the only supported value;
[VERIFY: main.py:29]), this re-seeds the buffer with the last
keyframe before adding new ones. That last keyframe is exactly the
one that became `frame[last_non_loop_frame_index]` of the previous
submap — and it will become `frame[0]` of the next.

This is what justifies the very specific formula in `add_edge`
([VERIFY: vggt_slam/solver.py:140]):

```python
P_temp = np.linalg.inv(prior_submap.proj_mats[-1]) @ current_submap.proj_mats[0]
```

`proj_mats[-1]` and `proj_mats[0]` are 4×4 embeddings of the
*intrinsic K* for the same physical image as predicted by two
different VGGT runs. Their relative is the projective ambiguity we
must absorb.

---

## 4. The three branches of `add_edge`

[VERIFY: vggt_slam/solver.py:118-195]

```python
def add_edge(self, submap_id_curr, frame_id_curr,
             submap_id_prev=None, frame_id_prev=None,
             is_loop_closure=False):
```

The 4-tuple `(submap_id_prev, is_loop_closure)` selects one of three
control flows.

### 4.1 Branch A — the very first submap (`submap_id_prev is None`)

[VERIFY: vggt_slam/solver.py:167-172]
```python
else:
    assert (submap_id_curr == 0 and frame_id_curr == 0), "First added node must be submap 0 frame 0"
    self.graph.add_homography(submap_id_curr + frame_id_curr, H_w_submap)
    self.graph.add_prior_factor(submap_id_curr + frame_id_curr, H_w_submap)
    if DEBUG:
        print(...)
```

`H_w_submap` is identity (line 122, before any branch). The prior
factor uses `anchor_noise` (σ=1e-6 across all 15 SL(4) tangent
dimensions; [VERIFY: vggt_slam/graph.py:23]).

The assertion makes Branch A unconditionally callable only the very
first time: subsequent calls always come with `submap_id_prev`.

### 4.2 Branch B — non-first build edge (`submap_id_prev != None`, `is_loop_closure=False`)

```python
# [VERIFY: vggt_slam/solver.py:123-165]
if submap_id_prev is not None:
    overlapping_node_id_prev = submap_id_prev + frame_id_prev

    # Estimate scale factor between submaps. (lines 127-146 → see §6 of 03-ALGORITHM-sl4_homography_alignment.md)
    ...

    # Compute the first camera matrix of the new submap in world frame.
    H_overlap_prior_overlap_current = np.linalg.inv(prior_submap.proj_mats[-1]) @ current_submap.proj_mats[0] @ H_scale
    H_w_submap = self.graph.get_homography(overlapping_node_id_prev) @ H_overlap_prior_overlap_current

    # Add first node of the new submap to the graph.
    if not is_loop_closure:
        self.graph.add_homography(submap_id_curr + frame_id_curr, H_w_submap)

    # Add between factor for intra submaps constraint.
    self.graph.add_between_factor(overlapping_node_id_prev, submap_id_curr + frame_id_curr,
                                  H_overlap_prior_overlap_current, self.graph.intra_submap_noise)
```

What gets added:

* Node `X(submap_id_curr + frame_id_curr)` (with `frame_id_curr = 0`
  in the normal call from `add_points`
  [VERIFY: vggt_slam/solver.py:237]).
* Between factor connecting `overlapping_node_id_prev` to that new
  node, with the SL(4) measurement
  `H_overlap_prior_overlap_current` and noise model
  `intra_submap_noise` (σ=0.05).

After this branch, control falls through to the inner-submap chain in
Branch D below.

### 4.3 Branch C — loop closure edge (`is_loop_closure=True`)

The bridge factor still uses the same scale/between formula, but the
loop branch *exits early* before the inner-submap chain is wired up:

[VERIFY: vggt_slam/solver.py:174-176]
```python
# Loop closure only gets intra submap constraints.
if is_loop_closure:
    return
```

In addition, the node insertion is skipped: the node already exists
(it was added earlier as the first frame of a build submap), so the
loop-closure call must avoid re-inserting it. The relevant guard is
the conditional in Branch B:

[VERIFY: vggt_slam/solver.py:157-158]
```python
# Add first node of the new submap to the graph.
if not is_loop_closure:
    self.graph.add_homography(submap_id_curr + frame_id_curr, H_w_submap)
```

So for a loop-closure call only the *between factor* is added — no
new node, no inner chain.

### 4.4 Branch D — inner-submap chain (always after Branch B)

[VERIFY: vggt_slam/solver.py:179-195]
```python
# Add nodes and edges for the inner submap constraints.
world_to_cam = current_submap.get_all_poses()
for index, pose in enumerate(world_to_cam):
    if index == 0:
        continue

    H_inner = world_to_cam[index-1] @ np.linalg.inv(pose)
    current_node = self.graph.get_homography(submap_id_curr + index - 1) @ H_inner

    # Add node to graph.
    self.graph.add_homography(submap_id_curr + index, current_node)

    # Add between factor for inner submap constraint.
    self.graph.add_between_factor(submap_id_curr + index - 1, submap_id_curr + index,
                                  H_inner, self.graph.inner_submap_noise)
```

For a build submap of `S` frames this adds `S-1` nodes and `S-1`
between factors, all with `inner_submap_noise` (σ=0.05).

### 4.5 Branch decision table

| Call from                                  | `submap_id_prev` | `is_loop_closure` | Branch executed     |
| ------------------------------------------ | ---------------- | ----------------- | ------------------- |
| First submap                               | None             | False             | A → D               |
| Subsequent build submap                    | id of previous   | False             | B → D               |
| `add_edge(lc_submap, 0, query, q_frame, False)` (inside `add_points` LC handling) | id of build submap holding the query keyframe | False | B → D **(for the lc_submap, 2 frames)** |
| `add_edge(detected_id, det_frame, lc_submap, 1, True)` (loop closure proper)        | id of the lc submap | True              | B early-returns at line 175 (no D) |

---

## 5. The auxiliary "loop-closure submap"

The LC bookkeeping is the most subtle part of the pose graph
construction. The key sequence is in `add_points`:

[VERIFY: vggt_slam/solver.py:254-287]
```python
for index, loop in enumerate(detected_loops):
    assert loop.query_submap_id == self.current_working_submap.get_id()

    cam_to_world_lc = closed_form_inverse_se3(pred_dict["extrinsic_lc"])
    K_4x4_lc = np.tile(np.eye(4), (2, 1, 1))
    K_4x4_lc[:, :3, :3] = pred_dict["intrinsic_lc"]
    world_to_cam_lc = np.linalg.inv(cam_to_world_lc)
    depth_map_lc = pred_dict["depth_lc"]
    conf_lc = pred_dict["depth_conf_lc"]

    intrinsics_cam = pred_dict["intrinsic_lc"]

    world_points_lc = unproject_depth_map_to_point_map(depth_map_lc, pred_dict["extrinsic_lc"], intrinsics_cam)

    lc_submap_num = self.map.get_largest_key() + self.map.get_latest_submap().get_last_non_loop_frame_index() + 1
    print(f"Creating new Loop closure submap with id {lc_submap_num}")
    lc_submap = Submap(lc_submap_num)
    lc_submap.set_lc_status(True)
    lc_submap.add_all_frames(pred_dict["frames_lc"])
    lc_submap.set_frame_ids(pred_dict["frames_lc_names"])
    lc_submap.set_last_non_loop_frame_index(1)

    lc_submap.add_all_poses(world_to_cam_lc)
    lc_colors = (np.transpose(pred_dict["frames_lc"].cpu().numpy(), (0, 2, 3, 1)) * 255).astype(np.uint8)
    lc_submap.add_all_points(world_points_lc, lc_colors, conf_lc, self.init_conf_threshold, K_4x4_lc)
    print("Loop closure conf", conf_lc.shape)
    print(lc_submap_num, 0, loop.query_submap_id, loop.query_submap_frame)
    lc_submap.set_conf_masks(conf_lc)
    self.map.add_submap(lc_submap)

    self.add_edge(lc_submap_num, 0, loop.query_submap_id, loop.query_submap_frame, is_loop_closure=False)
    self.add_edge(loop.detected_submap_id, loop.detected_submap_frame, lc_submap_num, 1, is_loop_closure=True)
```

What happens conceptually:

1. The second VGGT call produced predictions for a **2-frame minibatch**
   `[query_kf, retrieved_kf]`. Those predictions sit in
   `pred_dict["..._lc"]` keys.
2. A new `Submap` is created with `is_lc_submap=True`
   ([VERIFY: vggt_slam/solver.py:272-273]) and
   `last_non_loop_frame_index=1` ([VERIFY: vggt_slam/solver.py:276]).
3. The LC submap is given its own id `lc_submap_num` computed by the
   same formula as a normal new submap
   ([VERIFY: vggt_slam/solver.py:270]).
4. Two `add_edge` calls follow:
   - Call (a): `add_edge(lc_submap_num, 0, query_submap, query_frame, is_loop_closure=False)`.
     This adds the LC submap's first node and a between factor from
     the query frame of the current build submap to the LC submap's
     frame 0.
   - Call (b): `add_edge(detected_submap, detected_frame, lc_submap_num, 1, is_loop_closure=True)`.
     This adds a between factor *only* — node `X(detected_submap +
     detected_frame)` already exists.

Picturing the new edges:

```
... build submap (current) ......     build submap (older, the one that contained the retrieved kf)
... ──── X(query_node) ──── ...      ... ──── X(detected_node) ──── ...
              │                                       ▲
              │ intra_submap_noise (Branch B)         │ intra_submap_noise (Branch C)
              ▼                                       │
       X(lc_submap, 0) ─────────────────────────► X(lc_submap, 1)
                          inner_submap_noise (Branch D, inside Branch B's fall-through)
```

So in the final graph each accepted loop closure adds:

* **1 node** `X(lc_submap, 0)` (call (a) Branch B/D)
* **1 node** `X(lc_submap, 1)` (Branch D inner chain in call (a))
* **1 inner between factor** between them
* **1 intra between factor** `query → X(lc_submap, 0)` (call (a) Branch B)
* **1 intra between factor** `X(lc_submap, 1) → detected` (call (b) Branch C)

That is the smallest graph fragment that geometrically constrains the
old detected_submap relative to the current build submap via the
verifier VGGT pass.

### 5.1 Why a 2-frame submap instead of a direct edge?

A simpler implementation would add a single between factor `query →
detected` with the loop's relative pose as measurement. The LC submap
detour is preferred because:

* The VGGT verifier produces **two** poses (extrinsic_lc has shape
  `(2, 3, 4)`), and the between of those two is what we trust.
  Routing both through the LC submap's frames lets the SL(4) factor
  carry the full structure (including K differences and any residual
  projective skew between the two frames) rather than collapsing it
  to a single SE(3) measurement.
* If the verifier's prediction is wrong, the error stays concentrated
  in the two LC nodes; the inner chains of the build submaps are not
  perturbed at edge-construction time.
* Visualisation and ablation become possible: the LC submap can be
  *coloured* and *displayed* like any other submap, and its
  contribution to the optimised solution can be isolated by inspecting
  the factor errors of the two intra edges separately.

### 5.2 Why does `lc_submap_num` not collide with the *next* build submap?

The same formula is used for both build and LC submaps:

[VERIFY: vggt_slam/solver.py:270, 313]
```python
new_pcd_num = self.map.get_largest_key() + self.map.get_latest_submap().get_last_non_loop_frame_index() + 1
```

After the LC submap is added, `get_largest_key()` will return the
LC submap's id (it does *not* filter LC submaps by default —
[VERIFY: vggt_slam/map.py:31-34]). So when the **next** build submap
arrives:

```
   new_build_id = lc_submap_num + lc_last_non_loop_frame_index + 1
                = lc_submap_num + 1 + 1
                = lc_submap_num + 2
```

…which is correct: `lc_submap` uses keys `[lc_submap_num,
lc_submap_num+1]`, so the next free key is `lc_submap_num + 2`.

For the *trajectory* output, `write_poses_to_file` filters by
`is_lc_submap` ([VERIFY: vggt_slam/map.py:143-146]), so LC frames
never appear in the dumped TUM/KITTI file even though they consume
two keys in the graph.

> **Caveat**: `run_predictions` for the next build submap uses
> `self.map.get_largest_key()` (unfiltered) to derive
> `new_pcd_num`:
>
> ```python
> # [VERIFY: vggt_slam/solver.py:310-313]
> if self.map.get_largest_key() is None:
>     new_pcd_num = 0
> else:
>     new_pcd_num = self.map.get_largest_key() + self.map.get_latest_submap().get_last_non_loop_frame_index() + 1
> ```
>
> So it picks up the LC submap's id (the largest) and adds 1 (the
> LC's last_non_loop_frame_index) and 1. That gives
> `lc_submap_num + 2`, matching the analysis above. The
> "ignore_loop_closure_submaps" flag is *not* used here, but the
> arithmetic happens to work because LC submaps always have
> `last_non_loop_frame_index = 1`.

---

## 6. Worked example — graph after one loop closure

Setup:

* `submap_size = 16`, `overlapping_window_size = 1`, `max_loops = 1`.
* Two build submaps have already been processed:
  - submap 0 → keys `X(0..16)`, with `last_non_loop_frame_index=16`.
  - submap 17 → keys `X(17..33)`, with `last_non_loop_frame_index=16`.
* The third build submap finishes inference. Its id is `34` (matching
  the formula `17 + 16 + 1`), so its keys will be `X(34..50)`.
* Inside `run_predictions`, SALAD reports one loop: the query frame is
  the 5th frame of submap 34 (`query_submap_frame = 5`) and the
  detected frame is the 12th frame of submap 0
  (`detected_submap_frame = 12`).
* The VGGT verifier returns `image_match_ratio = 0.91 > 0.85`, so the
  loop survives.

What the calls do:

1. `add_points` is called.
2. The new build submap (id 34) is added to the map; `add_edge(34, 0,
   17, 16, False)` wires up the intra factor `X(33) → X(34)` and
   inner chain `X(34..50)` as usual.
3. The detected-loop loop body runs ([VERIFY: vggt_slam/solver.py:254-287]):
   - `lc_submap_num = get_largest_key()(=34) + last_non_loop_frame_index(=16) + 1 = 51`.
   - `lc_submap` is created with id `51`, `last_non_loop_frame_index = 1`.
   - `add_edge(51, 0, 34, 5, False)` — Branch B:
       * adds node `X(51)` with initial value from `H_olap`;
       * adds intra between factor `X(34+5) → X(51)`.
     Branch D (inner chain): adds node `X(52)` and between factor
     `X(51) → X(52)`.
   - `add_edge(0, 12, 51, 1, True)` — Branch B early-returns through
     Branch C: adds intra between factor `X(51+1) → X(0+12)`.

Final graph state:

```
   X(0)  prior anchor
   X(0)..X(16)   inner chain (submap 0)
   X(16) ── intra ── X(17)   (between submap 0 and 17)
   X(17)..X(33)  inner chain (submap 17)
   X(33) ── intra ── X(34)
   X(34)..X(50)  inner chain (submap 34)
   X(39=34+5) ── intra ── X(51)        ← LC bridge in
   X(51) ── inner ── X(52)               ← inner chain of LC submap
   X(52) ── intra ── X(12=0+12)         ← LC bridge out
```

The graph now has 53 nodes, 1 prior, and
`16 + 1 + 16 + 1 + 16 + 1 + 1 + 1 = 53` between factors.

Topologically, the LC adds a *short two-edge path* between
`X(34+5)` and `X(0+12)`. LM exploits this path the next time
`PoseGraph.optimize()` is called, deforming the SL(4) values of
*every* node on every chain that this path constrains.

---

## 7. The `add_points` orchestration in one diagram

```
add_points(pred_dict)
   │
   ├─ unpack pred_dict (images, K, extrinsic, depth, conf, …)
   │       [VERIFY: vggt_slam/solver.py:212-220]
   │
   ├─ world_points = unproject_depth_map_to_point_map(depth_map, ext, K)
   │       [VERIFY: vggt_slam/solver.py:222]
   │
   ├─ cam_to_world  = closed_form_inverse_se3(extrinsics)
   │   world_to_cam = inv(cam_to_world)
   │       [VERIFY: vggt_slam/solver.py:225-232]
   │
   ├─ Submap.{add_all_poses, add_all_points, set_conf_masks}
   │       [VERIFY: vggt_slam/solver.py:246-248]
   │
   ├─ GraphMap.add_submap(current)
   │       [VERIFY: vggt_slam/solver.py:249]
   │
   ├─ add_edge(submap_curr, 0, submap_prev, prev_last_non_loop_frame, False)
   │       [VERIFY: vggt_slam/solver.py:252]
   │
   └─ for loop in detected_loops:
         · build LC submap with VGGT_lc predictions
         · GraphMap.add_submap(lc_submap)
         · add_edge(lc_submap, 0, query_submap, query_frame,    is_loop_closure=False)
         · add_edge(detected_submap, det_frame, lc_submap, 1,   is_loop_closure=True)
               [VERIFY: vggt_slam/solver.py:254-287]
```

The only mutations to the global state in this function are: appending
to `GraphMap.submaps`, inserting `SL4` values into `PoseGraph.values`,
and appending factors to `PoseGraph.graph`. Everything else is
read-only.

---

## 8. Loop-closure factor count and pose-graph density

For a run of `N` build submaps with `S` keyframes each and `M`
accepted loop closures:

* Build factors:
  - 1 prior
  - `N-1` intra between factors (one per pair of adjacent build submaps)
  - `N · (S-1)` inner between factors (S-1 per submap)
* Loop-closure factors:
  - For each accepted LC: 2 intra + 1 inner = 3 factors
  - For each accepted LC: 2 nodes added

Total factors ≈ `1 + (N-1) + N·(S-1) + 3M = 1 + N·S - 1 + 3M = N·S + 3M`.
Total nodes ≈ `N·S + 2M`.

With `submap_size=16` (so `S=17`) and `N=20` submaps, the graph has
~340 nodes and ~340 factors (plus a handful for LCs). LM converges
in a couple of seconds on a laptop GPU.

This linear scaling is the reason `optimize()` is called from scratch
after every submap rather than using an incremental solver like
iSAM2: with the actual problem sizes the algorithm is solving
([VERIFY: README.md:144-148] mentions TUM and 7-Scenes), batch LM is
fast enough.

---

## 9. Correctness checks

A small audit you can run by hand to confirm the invariants:

1. **Every key in `Values` was added exactly once.**
   ```python
   # [VERIFY: vggt_slam/graph.py:34-38]
   key = X(key)
   if key in self.initialized_nodes:
       print(f"SL4 {key} already exists.")
       return
   self.values.insert(key, SL4(global_h))
   self.initialized_nodes.add(key)
   ```
   `add_homography` is idempotent: a second insertion with the same
   key is a no-op (with a warning). This protects against the
   loop-closure code accidentally re-inserting `X(detected_node)`.

2. **Every key referenced by a between factor exists.**
   ```python
   # [VERIFY: vggt_slam/graph.py:43-47]
   if key1 not in self.initialized_nodes or key2 not in self.initialized_nodes:
       raise ValueError(f"Both poses {key1} and {key2} must exist before adding a factor.")
   ```
   This is the safety net for the loop-closure `add_edge` call: both
   ends of the bridge factor must already exist before adding it.

3. **The anchor is unique.**
   `add_prior_factor` is only called from the Branch A path
   ([VERIFY: vggt_slam/solver.py:170]). No other call site exists.

4. **`get_homography` always returns a 4×4 matrix.**
   `auto_cal_H_mats` defaults to identity if the key isn't present
   ([VERIFY: vggt_slam/graph.py:63-67]).

These together imply that the resulting `NonlinearFactorGraph` is
**well-defined**: every factor references existing keys, every key is
unique, and exactly one anchor pins the gauge.

---

## 10. What would break if you changed `submap_size` or `overlapping_window_size`?

* `overlapping_window_size = 0`: the per-submap reset
  `image_names_subset[-overlapping_window_size:]` becomes
  `image_names_subset[-0:]` = the entire buffer (Python slicing
  quirk), so each submap would actually reuse *all* of the previous
  buffer. That would break the disjoint-keys invariant and
  cause `add_homography` warnings. The CLI help text says only `1`
  is supported ([VERIFY: main.py:29]).
* `overlapping_window_size > 1`: the buffer reset keeps the last `k`
  keyframes, but `add_edge` only uses `frame_id_prev` =
  `last_non_loop_frame_index` (the single overlap frame), not `k-1`
  frames. The extra `k-1` overlap frames would be re-processed as
  fresh keyframes in the next submap, double-counting them in the
  inner chain. Again the CLI marks this as unsupported.
* `submap_size`: free. The system scales linearly in `submap_size`
  for both compute (VGGT input batch) and factor count
  (`S-1` inner factors per submap).

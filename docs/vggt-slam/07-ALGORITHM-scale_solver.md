# Algorithm — Scale Solver (Median of Norm Ratios)

> The scale solver is intentionally tiny: 9 effective lines of NumPy
> ([VERIFY: vggt_slam/scale_solver.py:15-24]). This document explains
> why a 9-line solver is the right answer, what it actually computes,
> and the assumptions baked into it.

---

## 1. The full source

[VERIFY: vggt_slam/scale_solver.py:1-24]
```python
import numpy as np
import open3d as o3d

def debug_visualize(pcd1_points, pcd2_points):
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(pcd1_points)
    pcd1.paint_uniform_color([1, 0, 0])  # red

    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(pcd2_points)
    pcd2.paint_uniform_color([0, 0, 1])  # blue

    o3d.visualization.draw_geometries([pcd1, pcd2], window_name="Pairwise Point Clouds")

def estimate_scale_pairwise(X, Y, DEBUG=False):
    assert X.shape == Y.shape
    x_dists = np.linalg.norm(X, axis=1)
    y_dists = np.linalg.norm(Y, axis=1)
    scales = y_dists / x_dists
    scale = np.median(scales)

    if DEBUG:
        debug_visualize(X*scale, Y)

    return scale, None
```

Two callable entry points: one to draw two clouds in Open3D
(unconditional `o3d.visualization` requires a display), and the actual
solver `estimate_scale_pairwise`.

---

## 2. The mathematical model

Two clouds `X = {xᵢ}` and `Y = {yᵢ}` are passed in. They are the
3-D points produced by VGGT for the *same physical pixels* but
predicted by **two different submap inferences** — one in the
previous submap's frame, one in the current submap's frame
(post-rotation by the inter-submap projective transform; see
`03-ALGORITHM-sl4_homography_alignment.md §6`).

Under the assumption that the only freedom is a global scale `s` (the
rotation has already been removed by `P_temp[:3,:3]` in
`Solver.add_edge`, [VERIFY: vggt_slam/solver.py:141]):

```
   yᵢ ≈ s · xᵢ                                    (1)
```

Taking norms:

```
   ‖yᵢ‖ ≈ s · ‖xᵢ‖    (because s > 0)
   ‖yᵢ‖ / ‖xᵢ‖ ≈ s                                (2)
```

So *every* point pair yields an independent estimate of `s`. A robust
aggregator (the median) collapses them to one number:

```
   ŝ = median_i ( ‖yᵢ‖ / ‖xᵢ‖ )                    (3)
```

That is exactly what `estimate_scale_pairwise` computes:

```python
x_dists = np.linalg.norm(X, axis=1)
y_dists = np.linalg.norm(Y, axis=1)
scales  = y_dists / x_dists
scale   = np.median(scales)
```

---

## 3. Why median and not least-squares?

**Sum of squared residuals** in linear form:
```
   ŝ_LS = argmin_s Σᵢ ‖yᵢ - s·xᵢ‖²
        = (Σᵢ xᵢ·yᵢ) / (Σᵢ xᵢ·xᵢ)                  (4)
```

This is the closed-form least-squares scale solution. It minimises
squared residual under the assumption that errors `yᵢ - s·xᵢ` are
i.i.d. Gaussian. **But** VGGT's depth-confidence mask already removed
"obvious" outliers; what survives in the overlap region still contains
non-Gaussian outliers:

* Pixels near depth discontinuities (object edges, depth boundaries).
* Pixels in poorly-textured regions where the two inferences disagree
  on depth.
* Pixels that fall in different "scales" of the scene (foreground vs.
  background).

A least-squares fit is heavily distorted by these outliers — a single
point with a wrong factor-of-10 depth move the LS estimate by 30 % or
more. The median is the canonical robust estimator: it tolerates up
to 50 % of contamination before breaking down.

Concretely: with ~thousand-pixel overlap masks (post-conf-filtering)
and typically a few percent of outliers, the median is unaffected
while LS would be biased.

The median doesn't need an outlier rejection round (RANSAC, MAD
thresholding, etc.) because it is *inherently* robust to up to 50 %
contamination — and `Solver.add_edge` already provides a confidence
mask to keep the contamination rate well below that.

---

## 4. Assumption A — rotation is zero

The relation `yᵢ ≈ s·xᵢ` only holds if there is no rotation between
the two clouds. That is **only true after** the caller pre-rotates
one cloud:

[VERIFY: vggt_slam/solver.py:140-142]
```python
P_temp = np.linalg.inv(prior_submap.proj_mats[-1]) @ current_submap.proj_mats[0]
t1 = (P_temp[0:3,0:3] @ current_submap.get_frame_pointcloud(frame_id_curr).reshape(-1, 3)[good_mask].T).T
t2 = prior_submap.get_frame_pointcloud(frame_id_prev).reshape(-1, 3)[good_mask]
```

`P_temp[:3, :3]` is the 3×3 block of the projective transform that
maps the current submap's first-frame coordinates to the previous
submap's last-frame coordinates. The 3×3 block of `K_prev⁻¹ · K_curr`
is in general **not orthogonal** — it is the product of two
upper-triangular intrinsic matrices, one inverted. So strictly
speaking the "rotation" here is a non-orthogonal linear map.

Why does the median-of-norms scheme still work? Because:

* `K_prev` and `K_curr` are both close to identity in this setting:
  VGGT predicts intrinsics-per-frame, but adjacent frames have very
  similar `K`. Their product `K_prev⁻¹ · K_curr` is close to identity.
* The norms `‖t1ᵢ‖` and `‖t2ᵢ‖` are *relative* magnitudes; small
  perturbations from the off-identity `K_prev⁻¹ · K_curr` perturb
  every `xᵢ` equally and cancel out under division.

In other words, the calibration mismatch behaves like a near-identity
linear deformation that scales every distance similarly, so the
median-of-ratios is largely insensitive to it.

---

## 5. Assumption B — same physical pixel correspondences

The two clouds `X` and `Y` must correspond pixel-wise. The caller
guarantees this by indexing both via the same `good_mask`
([VERIFY: vggt_slam/solver.py:131-138]):

```python
good_mask = (prior_conf > prior_submap.get_conf_threshold()) * (current_conf > prior_submap.get_conf_threshold())
good_mask = good_mask.reshape(-1)
```

And then both clouds are indexed by the same mask
([VERIFY: vggt_slam/solver.py:141-142]):

```python
t1 = (P_temp[0:3,0:3] @ current_submap.get_frame_pointcloud(frame_id_curr).reshape(-1, 3)[good_mask].T).T
t2 = prior_submap.get_frame_pointcloud(frame_id_prev).reshape(-1, 3)[good_mask]
```

So `X[i]` and `Y[i]` are the predictions of *the same image pixel* in
the overlap frame, just made by two different inferences. The
assumption holds by construction.

---

## 6. Edge cases

### 6.1 Zero-norm points (`x = 0`)

`x_dists[i] = 0` would produce `y_dists[i] / x_dists[i] = inf`. The
median is robust to such infinities (one inf doesn't move the median
of ~thousand points), but technically `np.median` propagates `inf`
without complaint.

In practice this never fires: VGGT's depth maps are bounded below by
some small positive value, and any pixel with depth 0 would be filtered
by the confidence mask.

### 6.2 Negative scale

`np.median` of strictly positive numbers is strictly positive (norms
are non-negative; the only way to get zero is the zero-norm case
above). So `scale` is always non-negative; in practice always strictly
positive.

A *flipped* depth prediction by VGGT would land both clouds in opposite
half-spaces and produce a positive scale anyway (norms are sign-agnostic),
so this solver cannot detect a sign flip. That responsibility is
delegated to VGGT's downstream `image_match_ratio` check in
`run_predictions` ([VERIFY: vggt_slam/solver.py:371-374]).

### 6.3 The (`scale`, `None`) return tuple

[VERIFY: vggt_slam/scale_solver.py:24]
```python
return scale, None
```

The second element is reserved for future use (probably a confidence
score or inlier mask). The caller unpacks it as
`scale_factor_est_output = estimate_scale_pairwise(t1, t2)` then takes
`scale_factor = scale_factor_est_output[0]`
([VERIFY: vggt_slam/solver.py:143-145]).

So the `None` is currently inert; safely changeable to a real
confidence value without breaking the caller.

---

## 7. Numerical complexity

For an overlap frame with `N` confidence-passed pixels:

| Op                                  | Cost                            |
| ----------------------------------- | ------------------------------- |
| `np.linalg.norm(X, axis=1)`          | O(N)                            |
| `np.linalg.norm(Y, axis=1)`          | O(N)                            |
| Element-wise `/`                     | O(N)                            |
| `np.median`                          | O(N) (introselect)              |

So linear in the number of confident overlap pixels. Negligible
compared to anything else in the pipeline.

---

## 8. Comparison with other scale estimators

| Method                            | Robustness          | Complexity      | When to prefer                                              |
| --------------------------------- | ------------------- | --------------- | ----------------------------------------------------------- |
| Least-squares (eq. 4)             | None                | O(N)            | When point clouds are clean (perfect synthetic data)        |
| Mean of ratios                    | None (worse than LS) | O(N)            | Rarely useful                                               |
| **Median of ratios (this one)**   | **50 %**            | **O(N)**        | **Default for noisy real-world cloud pairs (our case)**     |
| RANSAC over LS                    | 50–90 %             | O(K·N)          | When the inlier ratio is unknown / low                      |
| Trimmed LS (10 % trim)            | ~10 %               | O(N log N)      | When you know exactly what fraction of outliers to expect   |
| Geman–McClure / Tukey M-estimator | smooth              | O(K·N)          | When you can afford iterative reweighted least squares       |

The implementation chose median because:

1. The confidence mask already rejects the worst outliers, so the
   surviving contamination is small (well below 50 %).
2. The overlap pair is *one* frame, so there is no temporal redundancy
   to exploit with a fancier estimator.
3. The cost difference between median and RANSAC is meaningful at
   ~thousand-pixel scale; median is one-shot and constant in inlier
   count.
4. The system can tolerate small bias in the scale (the SL(4)
   optimisation cleans up the residual). The bigger risk is *gross*
   bias from outliers, which the median prevents.

---

## 9. Why one scalar `s` is enough (and not `(s_x, s_y, s_z)`)

The model `Y ≈ s · X` uses an *isotropic* scale. Could VGGT produce
predictions that disagree on axis-wise scale? In principle yes —
depth resolution along the optical axis can differ from tangential
resolution. In practice no — VGGT's depth-head produces metric
depths, and the 3-D points are computed by `unproject_depth_map_to_point_map`
([VERIFY: third_party/vggt/vggt/utils/geometry.py:15]) which uses the
*same* depth per pixel along all three axes (via the camera ray).
Any cloud-wide scale disagreement is therefore inherently isotropic.

Anisotropic scale could only emerge from `K_prev⁻¹ · K_curr` being
non-trivial, which is the very thing the caller-side rotation pre-pass
absorbs.

So one scalar suffices, and it is embedded into SL(4) as
`diag(s, s, s, 1)` ([VERIFY: vggt_slam/solver.py:146]):

```
H_scale = ┌ s 0 0 0 ┐
          │ 0 s 0 0 │
          │ 0 0 s 0 │
          └ 0 0 0 1 ┘
```

which scales 3-D positions by `s` while leaving the homogeneous
coordinate (and therefore the projective scale) unchanged. This is
the *correct* embedding for a metric scale correction in SL(4).

> Note: `det(H_scale) = s³`. Strictly speaking this matrix is not in
> SL(4) (det ≠ 1) unless `s = 1`. The downstream factor still uses it
> as the measurement; GTSAM's `SL4(matrix)` constructor normalises by
> dividing by `det^(1/4)` internally, so the system effectively works
> with `H_scale / s^(3/4)`. The resulting SL(4) element has det 1 and
> represents the same projective transformation up to overall scaling
> of the homogeneous coordinate — which is the equivalence class
> SL(4) actually parametrises.

---

## 10. Failure modes

The solver itself essentially can't fail; the surrounding
pre-conditions can:

| Pre-condition failure                                       | Symptom                                                                                          | Existing guard                                                                 |
| ----------------------------------------------------------- | ----------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------ |
| Fewer than 100 confident pixels in overlap                  | Few-sample median is high-variance                                                              | Mask loosened twice in `add_edge` [VERIFY: vggt_slam/solver.py:133-138]        |
| Two clouds have systematic depth bias (factor-of-2)         | scale ≈ 2; SL(4) optimisation absorbs it                                                       | None needed; this is what `H_scale` is for                                     |
| One cloud has a flipped sign of depth                       | median norm ratio is positive (norms ignore sign), but the directions disagree                  | VGGT verifier `image_match_ratio` ≥ 0.85 guards against this in LC context     |
| Both clouds are noisy in the same direction                 | Scale estimate is biased toward the noisy direction                                              | Median; further smoothed by SL(4) optimisation                                 |
| Calibration `K_prev` vs `K_curr` very different             | `P_temp[:3,:3]` is far from orthogonal                                                          | Not handled; assumes adjacent submaps share calibration                        |

The fall-through behaviour is: a wildly wrong scale produces a wildly
wrong initial value `H_w_submap`. The between factor still records
the correct measurement; LM corrects the node value during the next
`optimize()`. So a single bad scale estimate does **not** wreck the
trajectory irreversibly — the global optimisation has a chance to
recover.

---

## 11. Why this matters for the whole system

Without the scale step, every submap's depth scale would drift
independently. VGGT's depth scale is metric to within a few percent for
typical indoor scenes, but those few percent compound over 20+ submaps
into trajectories that are visibly wrong. The median-of-norm-ratios
estimator brings them into agreement *one pair at a time*, and the
SL(4) optimisation then enforces global consistency.

The whole VGGT-SLAM design philosophy is: *delegate to the network,
keep the geometric code minimal*. The scale solver is the cleanest
expression of that philosophy in the repo.

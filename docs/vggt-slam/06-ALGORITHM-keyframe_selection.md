# Algorithm — Keyframe Selection (Lucas-Kanade Disparity Gate)

> The cheapest stage of the pipeline: a CPU-only Lucas–Kanade flow
> tracker that decides which raw frames are worth handing to VGGT.
> Implemented entirely in `vggt_slam/frame_overlap.py` ([VERIFY:
> vggt_slam/frame_overlap.py:1-61]).

---

## 1. Why a keyframe selector at all?

VGGT inference dominates per-frame cost ([VERIFY: vggt_slam/solver.py:335],
[VERIFY: main.py:154]). For video at 30 fps, processing every raw frame
is wasteful — consecutive frames share most of their pixels and add no
new geometric information. A lightweight pre-filter that drops "no
motion" frames keeps GPU time productive.

The chosen filter is **mean optical-flow displacement** of a sparse
corner set, measured against the *last accepted keyframe* (not the
previous frame). The threshold is the CLI argument `--min_disparity`
(default 50 px, [VERIFY: main.py:31]).

---

## 2. The two-phase state machine

`FrameTracker` has two methods that together implement a textbook
"track until decorrelated, then re-seed" state machine.

### 2.1 Initialisation — `initialize_keyframe`

[VERIFY: vggt_slam/frame_overlap.py:12-21]
```python
def initialize_keyframe(self, image):
    self.last_kf = image
    self.kf_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    self.kf_pts = cv2.goodFeaturesToTrack(
        self.kf_gray,
        maxCorners=1000,
        qualityLevel=0.01,
        minDistance=8,
        blockSize=7
    )
```

* `cv2.goodFeaturesToTrack` is the Shi–Tomasi corner detector.
  Returns up to **1000** corners — the most "trackable" points by
  eigenvalue ratio.
* `qualityLevel=0.01` keeps corners whose response is at least 1 % of
  the maximum response in the image. With 1000-corner ceiling this is
  permissive; in textured indoor scenes the limit is hit, in
  textureless scenes far fewer corners are returned.
* `minDistance=8` enforces a minimum pixel separation between corners
  — avoids piling many corners on the same texture patch.
* `blockSize=7` is the autocorrelation window used by the Shi–Tomasi
  scoring.

The chosen image is stashed (`last_kf`, `kf_gray`, `kf_pts`) and the
function returns nothing. It is called both:

* On the very first frame ([VERIFY: vggt_slam/frame_overlap.py:25]).
* Whenever the tracker decides the current frame should become the
  new keyframe ([VERIFY: vggt_slam/frame_overlap.py:42, 59]).

### 2.2 Decision step — `compute_disparity`

[VERIFY: vggt_slam/frame_overlap.py:23-60]
```python
def compute_disparity(self, image, min_disparity, visualize=False):
    if self.last_kf is None or self.kf_pts is None or len(self.kf_pts) < 10:
        self.initialize_keyframe(image)
        return True

    curr_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Track keyframe points into current frame
    next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
        self.kf_gray, curr_gray, self.kf_pts, None,
        winSize=(21, 21), maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

    status = status.flatten()
    good_kf   = self.kf_pts[status == 1]
    good_next = next_pts[status == 1]

    if len(good_kf) < 10:
        self.initialize_keyframe(image)
        return True

    # Measure displacement from keyframe to current frame
    displacement   = np.linalg.norm(good_next - good_kf, axis=1)
    mean_disparity = np.mean(displacement)

    if visualize:
        ...

    if mean_disparity > min_disparity:
        self.initialize_keyframe(image)
        return True
    else:
        return False
```

Three early-return paths and one threshold:

1. **First-ever call** (`last_kf is None`) → re-init, return `True`.
2. **Corner pool too thin** (`len(kf_pts) < 10`) → re-init, return
   `True`. This protects subsequent calls from a degenerate
   `calcOpticalFlowPyrLK` invocation.
3. **Tracker lost too many corners** (`len(good_kf) < 10`) → re-init,
   return `True`. A heavy loss of tracked corners is itself evidence
   of large camera motion or occlusion.
4. **Mean displacement > threshold** → re-init, return `True`.
5. **Otherwise** → return `False` (frame is *not* a keyframe; current
   `kf_*` state is left untouched).

---

## 3. The Lucas–Kanade call

[VERIFY: vggt_slam/frame_overlap.py:31-35]
```python
next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
    self.kf_gray, curr_gray, self.kf_pts, None,
    winSize=(21, 21), maxLevel=3,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
```

Parameters:

| Parameter             | Value                        | Why                                                                                                              |
| --------------------- | ---------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| Previous image        | `self.kf_gray`               | the *keyframe*, **not** the previous raw frame — so the tracker measures motion since the last keyframe          |
| Current image         | `curr_gray`                  | the current raw frame                                                                                            |
| Previous points       | `self.kf_pts` (≤1000)        | the Shi–Tomasi corners detected at keyframe time                                                                 |
| Next points           | `None`                       | OpenCV fills this in                                                                                              |
| Window size           | `(21, 21)`                   | LK integration window per corner                                                                                  |
| Pyramid levels        | `3`                          | 4 levels total (level 0 + 3 downsamples) — enough to track 100-ish pixel motion                                  |
| Termination criteria  | `EPS \| COUNT, 30, 0.01`     | 30 inner iterations or 0.01 displacement change, whichever first                                                 |

OpenCV returns:

* `next_pts` — the predicted corner positions in the current frame.
* `status` — per-corner success flag (1 = converged, 0 = lost).
* `err` — per-corner tracking error (unused).

The displacement is computed only over `status == 1` corners (lines
36-39).

---

## 4. The decision metric

> **mean Euclidean displacement of the surviving tracked corners**

[VERIFY: vggt_slam/frame_overlap.py:46-47]
```python
displacement = np.linalg.norm(good_next - good_kf, axis=1)
mean_disparity = np.mean(displacement)
```

Note the choice of `np.mean`, **not** `np.median`. Implications:

* A few large outliers (e.g. corners that landed on a moving person)
  inflate the mean and bias toward "select keyframe earlier". Given
  that "earlier" is the conservative direction (we'd rather have a
  keyframe with slightly less motion than miss one entirely),
  this is acceptable.
* In nearly-static scenes the mean is low and the tracker waits.
* In high-disparity scenes the mean is comfortably above the
  threshold; outliers don't matter.

The threshold is the CLI `--min_disparity` argument
([VERIFY: main.py:31]):

> `parser.add_argument("--min_disparity", type=float, default=50, help="Minimum disparity to generate a new keyframe")`

50 pixels is a moderate value — at 1920×1080 it's ~2.5 % of image
width. Sensible for cellphone-video baselines (~10 fps after
downsampling, modest camera motion). The README's instructions for
recording custom data use `fps=10` for the ffmpeg extraction
([VERIFY: README.md:111]).

---

## 5. Re-keyframe trigger semantics

The crucial subtlety: **the keyframe is *always* re-initialised to
the current frame whenever `compute_disparity` returns `True`**, *not*
to the next frame. This means once the disparity threshold is
crossed, the next call measures motion from *this* frame onward, not
from the previous keyframe.

Visually:

```
   keyframe ──────────────────────────────────→
        │       displacement grows
        │    ───────────────────────►
        │                              ▲ threshold crossed
        │                              │
        │                              return True
        │                              re-seed corners on this frame
        │                              ▼
        new keyframe ─────────────────→...
```

The `image_names_subset.append(image_name)` line in `main.py`
([VERIFY: main.py:112]) is conditional on this `True` return, so the
keyframe list grows by one whenever the tracker re-seeds.

---

## 6. Visualisation hook

[VERIFY: vggt_slam/frame_overlap.py:49-56]
```python
if visualize:
    vis = image.copy()
    for p1, p2 in zip(good_kf, good_next):
        p1 = tuple(p1.ravel().astype(int))
        p2 = tuple(p2.ravel().astype(int))
        cv2.arrowedLine(vis, p1, p2, color=(0, 255, 0), thickness=1, tipLength=0.3)
    cv2.imshow("Optical Flow", vis)
    cv2.waitKey(1)
```

A debug overlay that draws green arrows from each keyframe corner to
its tracked position in the current frame. Triggered by
`--vis_flow` ([VERIFY: main.py:24]) — disabled by default.

`cv2.imshow + cv2.waitKey(1)` requires a graphical display and a
display-capable build of OpenCV; on headless boxes this branch will
raise. The CLI flag is therefore opt-in.

---

## 7. Complexity

| Op                          | Cost                                                                              |
| --------------------------- | --------------------------------------------------------------------------------- |
| `cv2.cvtColor`              | O(H·W) — single pass                                                              |
| `cv2.goodFeaturesToTrack`   | O(H·W · log H·W) on cold call                                                     |
| `cv2.calcOpticalFlowPyrLK`  | O(N_corners · window² · pyramid_levels · iters)                                    |
| Displacement                | O(N_corners)                                                                      |

For 1000 corners on a 1280×720 frame the LK call is well under 5 ms on
a recent laptop CPU. Negligible compared to a VGGT call (~200 ms or
more per frame for 17-frame submaps).

Notice the asymmetry: corner *detection* runs only on keyframes
(rarely), corner *tracking* runs on every raw frame. The cost imbalance
matters because the system processes every frame even when many of
them get dropped.

---

## 8. Why mean LK disparity, and not something fancier?

The simplest possible criterion is "skip every Kth frame". That loses
adaptivity: when the camera is static, no skipping is needed; when it
moves quickly, every frame might matter. The next-simplest is "skip
frames whose absolute time-stamp delta is < Δt". That ignores
*motion*.

Mean LK disparity captures the right intuition: skip frames where
**the camera hasn't moved enough relative to the last keyframe to add
new information**. It does so without:

* Needing camera intrinsics.
* Needing depth.
* Needing the network.

That's why it works well as a preprocessing filter even without any
metric calibration.

The trade-off is **rotation insensitivity**: an in-place rotation
moves *all* corners by approximately the same amount, so the mean
disparity is high (e.g. a 5° in-plane rotation moves corners by 50+
pixels depending on radius). That is what we want — purely rotational
motion still benefits the system because it gives VGGT new views of
the scene.

For pure translational dolly-zoom on a flat texture, mean disparity
still scales linearly with translation × inverse depth, which is the
correct dependency: motion through a near-field scene yields more
disparity than the same motion through a far-field scene, and we
keyframe more often in the near field.

---

## 9. Edge cases & known issues

* **Black frames or low-texture areas** can drop `kf_pts` count below
  the 10-corner re-init trigger early. The keyframer then keeps
  re-initialising on every frame, effectively forcing every frame to
  become a keyframe. This is the right behaviour — texture-poor
  scenes don't have reliable optical-flow features anyway, so VGGT
  has to do the work.

* **Drifty corners on dynamic objects** (cars, people) — LK still
  tracks them, so they contribute to mean disparity even when the
  camera is static. With sufficient static background corners the
  bias is small; with many moving objects the system can over-keyframe.

* **No use of `cv2.calcOpticalFlowPyrLK`'s error output** — the
  `_` capture on [VERIFY: vggt_slam/frame_overlap.py:31] discards the
  per-corner error. A robust median-of-error filter could trim
  outliers further, but isn't implemented.

* **No reverse check** — the system only computes forward flow.
  RAFT-style "consistency" or LK back-and-forth check could detect
  scale lock-up where the tracker happily reports zero motion despite
  large optical motion. Not implemented.

* The README mentions RAFT in the CLI help: `--vis_flow` is described
  as "Visualize optical flow from RAFT for keyframe selection"
  ([VERIFY: main.py:24]) — but the keyframer **never uses RAFT**;
  it uses Lucas–Kanade. The help text is mis-stated. The actual
  visualisation is the LK arrows shown above.

---

## 10. Comparison with alternatives

| Method                                | Pros                                          | Cons                                                   | Where used                                                  |
| ------------------------------------- | --------------------------------------------- | ------------------------------------------------------ | ------------------------------------------------------------ |
| Fixed-rate (e.g. every 5th frame)     | Trivial                                       | No adaptivity                                          | Many academic baselines                                      |
| Time-since-last-keyframe              | Trivial                                       | Doesn't capture motion                                 | Some VIO front-ends                                          |
| Pose-change threshold (Sim(3))        | Most accurate                                 | Needs a working tracker — chicken/egg in mono-init    | ORB-SLAM keyframe spawning                                  |
| Bag-of-words / NetVLAD similarity     | Captures view variation, not just motion      | Heavy                                                  | DBoW-based systems                                           |
| **Mean LK disparity (this system)**   | **Cheap, adaptive, intuition-aligned**        | **Fooled by dynamic objects; no rotation/translation split** | **VGGT-SLAM 2.0**                                |
| RAFT or other dense flow              | Robust to texture failures                    | GPU-expensive, contradicts the "cheap pre-filter" idea | Some learning-based SLAM front-ends (mentioned in CLI help) |

The chosen design is "good enough" for the operating range (cellphone
video / RGB indoor datasets like TUM and 7-Scenes).

---

## 11. Effect on the rest of the pipeline

Every time `compute_disparity` returns `True`, three things happen in
`main.py`:

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

1. The image path is appended to the current submap buffer.
2. `image_count` is incremented (this is the denominator for all
   per-frame timing averages).
3. Once `len(image_names_subset) == submap_size + overlapping_window_size`
   ([VERIFY: main.py:118]), VGGT inference is triggered.

So tuning `--min_disparity` controls **how much wall-clock motion**
each submap covers. Lower threshold → smaller motion per submap →
denser map but more submaps. Higher threshold → larger motion per
submap → sparser map, faster runs, but VGGT may struggle with frames
that look too different from each other within a single batch.

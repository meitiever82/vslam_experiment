# Algorithm — Loop Closure (SALAD retrieval + VGGT geometric verifier)

> Loop closure in VGGT-SLAM is **two-stage**: SALAD descriptors propose
> matches, and a second VGGT call on the candidate pair geometrically
> verifies them. This document walks through both stages and the
> bounded-heap data structure that selects the top-K candidates.

---

## 1. The contract

[VERIFY: vggt_slam/loop_closure.py:77-86]
```python
def find_loop_closures(self, map, submap, max_similarity_thres=0.80, max_loop_closures=0):
    matches_queue = LoopMatchQueue(max_size=max_loop_closures)
    query_id = 0
    for query_vector in submap.get_all_retrieval_vectors():
        best_score, best_submap_id, best_frame_id = map.retrieve_best_score_frame(
            query_vector, submap.get_id(), ignore_last_submap=True)
        if best_score < max_similarity_thres:
            new_match_data = LoopMatch(best_score, submap.get_id(), query_id, best_submap_id, best_frame_id)
            matches_queue.add(new_match_data)
        query_id += 1

    return matches_queue.get_matches()
```

Inputs:

* `map` — the global `GraphMap` (search corpus).
* `submap` — the just-built submap (query).
* `max_similarity_thres` — the *distance* below which a match is
  considered a candidate (lower = more confident match; semantics
  inverted from the variable name — see §5.3).
* `max_loop_closures` — capacity of the bounded heap.

Output: a list of at most `max_loop_closures` `LoopMatch` named tuples,
sorted by similarity from best to worst.

---

## 2. Stage 1 — SALAD descriptor extraction

[VERIFY: vggt_slam/loop_closure.py:52-58]
```python
class ImageRetrieval:
    def __init__(self, input_size=224):
        ckpt_pth = os.path.join(torch.hub.get_dir(), "checkpoints/dino_salad.ckpt")
        self.model = load_model(ckpt_pth)
        self.model.eval()
        self.transform = input_transform((input_size, input_size))
```

A single DINO-SALAD checkpoint is loaded from
`~/.cache/torch/hub/checkpoints/dino_salad.ckpt`. Descriptors are
produced via the standard ImageNet preprocessing pipeline:

[VERIFY: vggt_slam/loop_closure.py:17-23]
```python
def input_transform(image_size=None):
    MEAN = [0.485, 0.456, 0.406]
    STD = [0.229, 0.224, 0.225]
    transform_list = [T.ToTensor(), T.Normalize(mean=MEAN, std=STD)]
    if image_size:
        transform_list.insert(0, T.Resize(image_size, interpolation=T.InterpolationMode.BILINEAR))
    return T.Compose(transform_list)
```

Inputs are squashed to `224×224`, then converted to a tensor and
normalised with ImageNet statistics. The result is a per-frame
descriptor `(D,)` of whatever dimensionality SALAD outputs.

Batched extraction for a whole submap:

[VERIFY: vggt_slam/loop_closure.py:65-75]
```python
def get_batch_descriptors(self, imgs):
    with torch.no_grad():
        pil_imgs = [tensor_transform(img) for img in imgs]
        imgs = torch.stack([self.transform(img) for img in pil_imgs])
        return self.model(imgs.to(device))

def get_all_submap_embeddings(self, submap):
    frames = submap.get_all_frames()
    return self.get_batch_descriptors(frames)
```

`Solver.run_predictions` stashes these on the submap right after
inference is started but before it ends:

[VERIFY: vggt_slam/solver.py:321]
```python
new_submap.set_all_retrieval_vectors(self.image_retrieval.get_all_submap_embeddings(new_submap))
```

So each submap carries its own `retrieval_vectors`, ready to be matched
against all *other* (non-LC) submaps in the corpus.

---

## 3. Stage 1 — nearest-neighbour search via L2 distance

`find_loop_closures` calls `GraphMap.retrieve_best_score_frame` once
per query embedding:

[VERIFY: vggt_slam/map.py:68-102]
```python
def retrieve_best_score_frame(self, query_vector, current_submap_id, ignore_last_submap=True):
    overall_best_score = 1000
    overall_best_submap_id = 0
    overall_best_frame_index = 0
    sorted_keys = sorted(self.submaps.keys())
    for index, submap_key in enumerate(sorted_keys):
        if submap_key == current_submap_id:
            continue

        if self.non_lc_submap_ids and ignore_last_submap and submap_key == self.non_lc_submap_ids[-1]:
            continue

        else:
            submap = self.submaps[submap_key]
            if submap.get_lc_status():
                continue
            submap_embeddings = submap.get_all_retrieval_vectors()
            scores = []
            for index, embedding in enumerate(submap_embeddings):
                score = torch.linalg.norm(embedding-query_vector)
                # score = embedding @ query_vector.t()
                scores.append(score.item())

            # for now assume we can only have at most one loop closure per submap

            best_score_id = np.argmin(scores)
            best_score = scores[best_score_id]

            if best_score < overall_best_score:
                overall_best_score = best_score
                overall_best_submap_id = submap_key
                overall_best_frame_index = best_score_id

    return overall_best_score, overall_best_submap_id, overall_best_frame_index
```

Four exclusions apply during the linear scan:

1. **Current submap** (`submap_key == current_submap_id`) — never
   match against yourself.
2. **Immediately previous submap** (only if `ignore_last_submap=True`
   and the last id in `non_lc_submap_ids`) — the previous build submap
   is already connected via the intra-submap factor, so a "loop" to
   it would be redundant and destabilising.
3. **Loop-closure submaps** (`submap.get_lc_status()`) — only build
   submaps live in the search corpus.
4. **(commented-out alternative)** A cosine-similarity score
   `embedding @ query_vector.t()` is shown as an alternative on
   [VERIFY: vggt_slam/map.py:89]. The released code uses **L2
   distance**, so smaller is better.

Within each candidate submap, every frame's descriptor is compared
against the query and the argmin is recorded. The outer loop tracks
the *overall* best across all submaps.

> **Important**: the function returns the best across *all submaps* —
> there is only one returned 3-tuple. The comment on
> [VERIFY: vggt_slam/map.py:92] confirms: "for now assume we can only
> have at most one loop closure per submap." For
> `max_loop_closures > 1`, multiple *query frames* in the same submap
> can still each return a different best detected submap; those
> populate the heap. But a single query frame produces at most one
> candidate match.

Cost: `O(N_corpus_frames · D)` per query embedding, all done in
PyTorch. For a 20-submap run with 17 keyframes each that is
~340 descriptor norms per query embedding, multiplied by ~17 query
embeddings (one per frame in the current submap), so ~5700 norm
computations per submap. Negligible compared to the VGGT call.

---

## 4. Stage 1 — `LoopMatchQueue`

[VERIFY: vggt_slam/loop_closure.py:32-49]
```python
class LoopMatchQueue:
    def __init__(self, max_size: int):
        self.max_size = max_size
        self.heap = []  # Simulated max-heap by negating scores

    def add(self, match: LoopMatch):
        # Negate similarity_score to turn min-heap into max-heap
        item = (-match.similarity_score, match)
        if len(self.heap) < self.max_size:
            heapq.heappush(self.heap, item)
        else:
            # Push new element and remove the largest (i.e., smallest negated)
            heapq.heappushpop(self.heap, item)

    def get_matches(self):
        """Return sorted list of matches (lowest value first)"""
        return [match for _, match in sorted(self.heap, reverse=True)]
```

What this is doing, given that `similarity_score` is actually an L2
*distance* (lower = better):

* The heap stores `(-distance, match)` pairs.
* `heapq` is a *min-heap* — so the **root has the smallest** `-dist`,
  i.e. the **largest distance** (worst match) at the root.
* `heappushpop` adds a new pair and pops the root; this discards the
  current *worst* surviving match.

Net effect: the heap retains the `max_size` matches with **smallest
distance** (best L2 closeness) — the most likely true positives.

`get_matches` returns the heap sorted by `-dist` *descending* (reverse
of natural order), which means the returned list runs from worst to
best. Note the comment claims "lowest value first" — it is technically
wrong, but doesn't matter for the consumer: `add_points` indexes
`detected_loops[0]` (the first element) and the heap was only used
because `max_loops=1` for stock runs, so the list has one element.

If `max_loops > 1` is ever supported (the README says it isn't yet,
[VERIFY: main.py:30]), the consumer would have to handle each loop's
relative order explicitly.

---

## 5. Stage 2 — VGGT geometric verifier

Back in `Solver.run_predictions`:

[VERIFY: vggt_slam/solver.py:340-351]
```python
predictions_lc = None
with self.loop_closure_timer:
    detected_loops = self.image_retrieval.find_loop_closures(
        self.map, new_submap, max_loop_closures=max_loops, max_similarity_thres=self.lc_thres)
loop_closure_frame_names = []
if len(detected_loops) > 0:
    print("detected_loops", detected_loops)
    retrieved_frames = self.map.get_frames_from_loops(detected_loops)
    with torch.no_grad():
        lc_frames = torch.stack((new_submap.get_frame_at_index(detected_loops[0].query_submap_frame),
                                 retrieved_frames[0]), axis=0)
        predictions_lc = model(lc_frames, compute_similarity=True)
        loop_closure_frame_names = [...]
```

This packs the first loop candidate into a 2-frame batch
`[query_kf, retrieved_kf]` and runs VGGT a second time with
`compute_similarity=True`. The model's aggregator computes an
`image_match_ratio` ([VERIFY: third_party/vggt/vggt/models/vggt.py:61, 98])
which is the fraction of internal cross-attention scores above some
threshold — see §5.1 below.

Then:

[VERIFY: vggt_slam/solver.py:370-383]
```python
if predictions_lc is not None:
    image_match_ratio = predictions_lc["image_match_ratio"]
    if image_match_ratio < 0.85:
        print("Loop closure image match ratio too low, skipping loop closure")
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

* Hard threshold `image_match_ratio < 0.85 → drop` ([VERIFY: vggt_slam/solver.py:372]).
* On survival, increment the loop counter ([VERIFY: vggt_slam/solver.py:377])
  and store the per-pair predictions on the dict for `add_points`.
* `pose_encoding_to_extri_intri` re-uses the network's pose head to
  produce `(2, 3, 4)` extrinsics for the pair.

### 5.1 What is `image_match_ratio`?

From the upstream VGGT code:

[VERIFY: third_party/vggt/vggt/models/vggt.py:61]
```python
aggregated_tokens_list, patch_start_idx, target_tokens, image_match_ratio = self.aggregator(images, compute_similarity)
```

The aggregator's `image_match_ratio` is set only when
`compute_similarity=True`; it is a scalar in `[0, 1]` reporting how
much of the two images the network believes is in geometric
correspondence after looking at the cross-attention pattern between
the two image tokens. A value of 0.85 is the empirical threshold
chosen by the VGGT-SLAM authors as "sane minimum for a real overlap";
below it, the geometry produced by the second VGGT call is judged
unreliable.

In other words: SALAD is the proposer, VGGT is the verifier, and the
verifier rejects anything below `image_match_ratio = 0.85`.

### 5.2 Failure-mode summary

| Failure                                                 | Stage                       | Handler                                                                                              |
| ------------------------------------------------------- | --------------------------- | ---------------------------------------------------------------------------------------------------- |
| SALAD distance ≥ `lc_thres`                             | SALAD distance gate         | `find_loop_closures` simply does not add the match to the heap [VERIFY: vggt_slam/loop_closure.py:82] |
| VGGT verifier `image_match_ratio < 0.85`                | VGGT geometric verifier     | Reset `predictions_lc = None`, set `detected_loops = []` [VERIFY: vggt_slam/solver.py:373-375]      |
| Verifier OK, but scale-overlap-mask <100 pixels         | `Solver.add_edge` scale ladder | Loosen mask twice, ultimately use `prior_conf > 0` [VERIFY: vggt_slam/solver.py:133-138]            |

There is no explicit RANSAC step or outlier rejection beyond these
three gates.

### 5.3 The `lc_thres` parameter and its CLI semantics

The CLI help reads:

> `--lc_thres 0.95` — *Threshold for image retrieval. Range: [0, 1.0].
> Higher = more loop closures*
> [VERIFY: main.py:33]

But internally the check is

[VERIFY: vggt_slam/loop_closure.py:82]
```python
if best_score < max_similarity_thres:
    ...
```

where `best_score` is the L2 distance of normalised SALAD descriptors.
Higher `max_similarity_thres` therefore admits *larger* distances —
which is exactly what "Higher = more loop closures" means in
the help text. So the help is correct in spirit but counter-intuitive
mathematically: the underlying number is a distance, not a similarity.

The default of `0.95` is unusually permissive — almost every SALAD
descriptor pair is admitted, and the VGGT verifier does the heavy
lifting. The system trades a higher Stage-1 false-positive rate for
the cost of a single extra VGGT call (the verifier).

---

## 6. End-to-end "did a loop close?" decision tree

```
                  new submap S_cur, query frame index q
                                │
                                ▼
                  q_vec = submap_embeddings[q]
                                │
              ┌─────────────────┴─────────────────┐
              │ for each non-LC submap s ≠ S_cur, │
              │ ≠ last-non-LC submap:             │
              │   d_min = min ||emb - q_vec||     │
              │   keep best (d_min, s, frame)     │
              └─────────────────┬─────────────────┘
                                ▼
                  d_min < lc_thres?
                  ┌────yes────┐    ┌─no→ no loop for this query
                  ▼            ▼
        push (d_min, match) into LoopMatchQueue
                                │
              after iterating over every query:
                                │
              return top-`max_loop_closures` by smallest d
                                │
                                ▼
              detected_loops = [LoopMatch(...)]
                                │
                                ▼
      VGGT(lc_frames, compute_similarity=True)
                                │
                                ▼
        image_match_ratio ≥ 0.85?
        ┌────yes────┐    ┌─no→ drop loop, predictions_lc=None
        ▼            ▼
   add_points proceeds to build LC submap + add_edge pair
                                │
                                ▼
          PoseGraph.optimize() restores global consistency
```

A loop closure that survives both gates contributes:

* 2 new SL(4) nodes (the LC submap's two frames),
* 2 intra between factors (`query → lc[0]` and `lc[1] → detected`),
* 1 inner between factor (`lc[0] → lc[1]`),
* `num_loop_closures` is incremented by 1 ([VERIFY: vggt_slam/solver.py:377]).

---

## 7. Performance characteristics

| Sub-step                                                         | Cost                                                              | Lives in                                       |
| ---------------------------------------------------------------- | ----------------------------------------------------------------- | ---------------------------------------------- |
| SALAD descriptors for a 17-frame submap                          | one batched forward at `(17, 3, 224, 224)`                        | `vggt_timer` (mis-attributed) and CLIP timer for `--run_os` only |
| `find_loop_closures` linear scan over corpus                     | `O(N_corpus_frames · D)` torch norms                              | `loop_closure_timer` [VERIFY: vggt_slam/solver.py:340-341]      |
| `LoopMatchQueue.add`                                             | `O(log max_loops)` per call (heap push/popush)                    | trivial                                       |
| `model(lc_frames, compute_similarity=True)`                      | second VGGT forward on 2 frames                                   | `vggt_timer` [VERIFY: vggt_slam/solver.py:348] |
| `add_edge` × 2 with scale solving                                | one median + a couple of `np.linalg.inv`                          | untimed                                       |
| `PoseGraph.optimize` after LC                                    | LM on the full graph                                              | `backend_time`                                |

Bottlenecks in practice: the 2-frame VGGT call (Stage 2) — for an
N-submap run with one loop per submap, the system pays roughly an
*extra* VGGT inference per submap on top of the build inference.
SALAD itself is a single fast DINO pass; the linear search is fast in
PyTorch.

For *not* paying the verifier cost when the search finds nothing,
note that the `model(lc_frames, ...)` call is only made when
`len(detected_loops) > 0` ([VERIFY: vggt_slam/solver.py:343]). With a
sufficiently strict `lc_thres`, most submaps incur zero loop-closure
cost beyond SALAD.

---

## 8. Edge cases

### 8.1 First submap

`find_loop_closures` runs on the first submap, but
`retrieve_best_score_frame` iterates over an empty `submaps` dict and
returns the initial sentinel `(1000, 0, 0)` for every query, which
fails the `best_score < lc_thres` gate (1000 ≥ 0.95). No loops fire
([VERIFY: vggt_slam/map.py:68-71], [VERIFY: vggt_slam/loop_closure.py:82]).

### 8.2 Second submap

The corpus has exactly one prior submap. `ignore_last_submap=True`
filters that out, so the corpus is empty and again no loops fire.

### 8.3 LC submap excluded from corpus

LC submaps are added to `GraphMap.submaps` ([VERIFY: vggt_slam/map.py:18-22])
but excluded from the loop search by the `get_lc_status()` check
([VERIFY: vggt_slam/map.py:83-84]). They are also excluded from
`non_lc_submap_ids` and therefore from the `ignore_last_submap` check.

### 8.4 What if SALAD's checkpoint is missing?

```python
# [VERIFY: vggt_slam/loop_closure.py:55-56]
ckpt_pth = os.path.join(torch.hub.get_dir(), "checkpoints/dino_salad.ckpt")
self.model = load_model(ckpt_pth)
```

`load_model` (from `salad.eval`) presumably raises if the path
doesn't exist. There is no auto-download; the setup script
([VERIFY: setup.sh:11-15]) installs SALAD but does not seed the
checkpoint. Users must either run SALAD's own download script or
ship the file manually.

### 8.5 `max_loops = 0`

The CLI defaults to `1` ([VERIFY: main.py:30]); a value of `0` causes
`LoopMatchQueue(max_size=0)` ([VERIFY: vggt_slam/loop_closure.py:32]).
The first `heappush` then bypasses the size check (`len(self.heap)
< 0` is false) and instead `heappushpop`s into an empty heap, which
`heapq` does support but returns immediately the just-pushed element
(net effect: heap stays empty). So `max_loops = 0` correctly disables
loop closure.

The CLI confirms: *"`--max_loops 1` or 0 to disable loop closures"*
([VERIFY: main.py:30]).

---

## 9. Comparison with classical loop closure pipelines

| Stage                    | ORB-SLAM 3                                       | MASt3R-SLAM                                  | **VGGT-SLAM 2.0**                                                 |
| ------------------------ | ------------------------------------------------ | -------------------------------------------- | ----------------------------------------------------------------- |
| Proposer (place rec.)    | DBoW3 bag-of-words                               | DINO-SALAD                                   | **DINO-SALAD**                                                    |
| Verifier (geometric)     | Sim(3) consistency / RANSAC                      | Iterative MASt3R re-prediction               | **Single VGGT pass on 2-frame batch**                              |
| Gate                     | Inlier count                                     | Cross-corr / RANSAC inliers                  | **`image_match_ratio ≥ 0.85`**                                    |
| Map update               | Sim(3) BA + global BA                            | Sim(3) BA                                    | **SL(4) BetweenFactor + LM**                                       |
| Cost per LC              | Variable; can be expensive                       | One MASt3R call                              | One VGGT call (already paid for build, doubled for LC)             |

The big differentiator is that VGGT-SLAM uses the *same network* as
the verifier — eliminating the need for separately-trained geometric
matching heads.

---

## 10. Known issues and TODOs (from the code)

* `max_loop_closures > 1` is not supported in the released pipeline
  ([VERIFY: main.py:30]). `add_points` always indexes
  `detected_loops[0]` ([VERIFY: vggt_slam/solver.py:347]), even
  though the loop body iterates over all loops
  ([VERIFY: vggt_slam/solver.py:255]). The second and subsequent
  loops in `detected_loops` would re-use the same verifier
  predictions, which is incorrect.
* The cosine-similarity alternative is commented out
  ([VERIFY: vggt_slam/map.py:89]); switching to it would require
  reversing the sign of the gate (and the heap).
* The heap is a 3-line wrapper around `heapq`, but the documentation
  string in `get_matches` is wrong about ordering
  ([VERIFY: vggt_slam/loop_closure.py:47-48]).
* SALAD checkpoint is not auto-downloaded by `setup.sh`.

None of these break the current single-loop-per-submap pipeline.

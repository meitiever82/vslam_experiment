# VGGT-SLAM 2.0 — Technical Analysis

> Comprehensive code-level analysis of VGGT-SLAM 2.0
> (`/home/steve/vslam_ws/src/VGGT-SLAM`). Every claim in every document
> below is anchored to the exact `file:line` of the code it describes
> via `[VERIFY:]` tags.

## Reading order

| #   | Document                                                                      | What it covers                                                                 |
| --- | ----------------------------------------------------------------------------- | ------------------------------------------------------------------------------ |
| 00  | [SYSTEM_OVERVIEW.md](00-SYSTEM_OVERVIEW.md)                                   | High-level architecture, module inventory, end-to-end frame loop               |
| 01  | [DATA_STRUCTURES.md](01-DATA_STRUCTURES.md)                                    | Every persistent field in every class, with create/consume sites               |
| 02  | [DATA_FLOW.md](02-DATA_FLOW.md)                                                | The 10-stage path from a JPEG on disk to a globally-consistent map vertex      |
| 03  | [ALGORITHM-sl4_homography_alignment.md](03-ALGORITHM-sl4_homography_alignment.md) | The keystone: why SL(4), the between-factor algebra, scale embedding         |
| 04  | [ALGORITHM-pose_graph_construction.md](04-ALGORITHM-pose_graph_construction.md) | `Solver.add_edge` walk-through; node-key arithmetic; loop-closure bridge submap |
| 05  | [ALGORITHM-loop_closure.md](05-ALGORITHM-loop_closure.md)                       | SALAD retrieval + VGGT geometric verifier + bounded heap                       |
| 06  | [ALGORITHM-keyframe_selection.md](06-ALGORITHM-keyframe_selection.md)           | Lucas-Kanade keyframer in `frame_overlap.py`                                   |
| 07  | [ALGORITHM-scale_solver.md](07-ALGORITHM-scale_solver.md)                       | Median-of-norm-ratios estimator (the 9-line solver)                            |
| 08  | [KEY_FUNCTIONS.md](08-KEY_FUNCTIONS.md)                                          | Line-by-line dissection of the 8 most load-bearing functions                   |
| 09  | [KEY_QUESTIONS.md](09-KEY_QUESTIONS.md)                                          | Design-rationale Q&A (26 questions)                                            |

## Methodology

These documents follow the
[`codebase-analysis-skill`](../../.claude/skills/codebase-analysis-skill/)
workflow:

1. **Code reading first** — every file in `vggt_slam/`, `main.py`, and
   the relevant upstream VGGT entry points was read in full before
   any claim was made.
2. **Mandatory verification tags** — every factual claim carries a
   `[VERIFY: path/to/file:line]` tag that points to the supporting
   code.
3. **Automated checks** — a script (`/tmp/verify_check.sh`) extracts
   every `[VERIFY:]` tag, parses the file:line pair, and verifies
   that the file exists and the line number is within range. All
   354 unique tags pass.
4. **Manual content audit** — a sample of high-stakes references
   (the SL(4) measurement equation, the scale solver, the loop-closure
   gate, the node-key arithmetic, the documented typo at
   `graph.py:77`) was re-read against the docs for content
   consistency.

## What the analysis does *not* cover

* **Internals of the VGGT transformer.** Treated as a black box whose
  contract is the dict it returns
  ([VERIFY: third_party/vggt/vggt/models/vggt.py:40-52]).
* **Internals of SALAD or Perception-Encoder.** Single calls into
  pre-trained checkpoints.
* **GTSAM's SL(4) Lie-group integration.** Used as a black-box back end.

## Known issues identified during analysis

The walkthrough surfaced a handful of dormant issues — none of them
break the released pipeline, but they are documented for completeness:

| Where                                                  | Issue                                                                                                              | Doc                                                  |
| ------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------ | ---------------------------------------------------- |
| `vggt_slam/graph.py:77`                                 | `return projection_matri` — missing `x`, latent NameError; method never called from main loop                       | 01-DATA_STRUCTURES.md §3.4                            |
| `vggt_slam/map.py:168-170`                              | Dead `count` increment under a `continue`-protected branch in `save_framewise_pointclouds`                           | 01-DATA_STRUCTURES.md §2.3                            |
| `vggt_slam/solver.py:101-116`                           | `tranform_submap_to_canonical` defined but never called                                                              | 01-DATA_STRUCTURES.md §1.5                            |
| `vggt_slam/solver.py:57`                                | `Solver.temp_count` declared but unused                                                                             | 01-DATA_STRUCTURES.md §11                              |
| `vggt_slam/submap.py:13`                                | `Submap.R_world_map` declared but unused                                                                            | 01-DATA_STRUCTURES.md §11                              |
| `vggt_slam/solver.py:307`                               | `dtype` computed but unused; calls `torch.cuda.get_device_capability()` unconditionally — crashes on CPU-only boxes | 08-KEY_FUNCTIONS.md §1.3                              |
| `main.py:24` CLI help                                   | "RAFT" mentioned but the keyframer uses Lucas–Kanade                                                                | 06-ALGORITHM-keyframe_selection.md §6                 |
| `vggt_slam/loop_closure.py:47-48`                       | `get_matches` docstring inverted relative to actual order                                                            | 05-ALGORITHM-loop_closure.md §4                       |
| `main.py:30` and `vggt_slam/solver.py:347`              | `max_loops > 1` indexes only `detected_loops[0]` — silently broken                                                  | 05-ALGORITHM-loop_closure.md §10                      |
| `vggt_slam/graph.py:143-151`                            | `update_all_homographies` defined but never called                                                                  | 01-DATA_STRUCTURES.md §3.1                            |
| `vggt_slam/map.py:138-140`                              | "Using rectifying homographies" message prints but the code path doesn't apply them                                  | 08-KEY_FUNCTIONS.md §8.4                              |

## Provenance

This analysis was produced via the `/codebase-analysis-skill` Claude
Code skill on 2026-05-15. Source tree state:

* Branch: `main`
* Working-tree mods: `main.py` (the GeoScan/8GB modifications), plus
  the workspace-local `docker/`, `docs/`, `scripts_geoscan/`,
  `setup_docker.sh` additions.
* HEAD commit: `66630f7` (Merge pull request #37 from Fnhid/main).

All `[VERIFY:]` line numbers were valid against that state at the time
of writing. Subsequent commits may shift line numbers — re-run the
verification script (`/tmp/verify_check.sh`, included in the
methodology section above) after pulling upstream changes.

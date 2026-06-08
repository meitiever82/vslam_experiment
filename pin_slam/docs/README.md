# PIN-SLAM 技术文档

本目录由 `codebase-analysis-skill` 在 2026-06-08 生成,严格基于 `src/PIN_SLAM/` 实际源码,每个断言带 `[VERIFY: <path>:<line>]` 锚定。

## 阅读顺序

1. **[SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md)** — 主循环 6 阶段 + 模块清单 + 关键超参表
2. **[DATA_STRUCTURES.md](DATA_STRUCTURES.md)** — 8 个核心类的字段逐项
3. **[DATA_FLOW.md](DATA_FLOW.md)** — 单帧期间张量怎么流转
4. **[ALGORITHM_01-NeuralPointMap.md](ALGORITHM_01-NeuralPointMap.md)** — 体素哈希 + IDW + 弹性变形(`model/neural_points.py`)
5. **[ALGORITHM_02-Decoder_Loss_Sampler.md](ALGORITHM_02-Decoder_Loss_Sampler.md)** — MLP + BCE + 7/ray 采样(`model/decoder.py` + `utils/loss.py` + `utils/data_sampler.py`)
6. **[ALGORITHM_03-Tracker_SDF_Odometry.md](ALGORITHM_03-Tracker_SDF_Odometry.md)** — 点到 SDF LM 配准(`utils/tracker.py`)
7. **[ALGORITHM_04-Mapper_Training.md](ALGORITHM_04-Mapper_Training.md)** — 在线 Adam + Replay + BA(`utils/mapper.py`)
8. **[ALGORITHM_05-LoopDetector_PGO.md](ALGORITHM_05-LoopDetector_PGO.md)** — Neural Point Map Context + gtsam(`utils/loop_detector.py` + `utils/pgo.py`)
9. **[PHASE4_PLAN.md](PHASE4_PLAN.md)** — 规划文件,记录文档边界与覆盖度

## 总体指标

- 总长 ~4900 行 markdown,跨 9 个文档
- 502 个 `[VERIFY:]` 引用段全部通过 `verify_refs.sh` 自动校验(2026-06-08)
- 覆盖源码 ~10K 行(`pin_slam.py` + `model/` + `utils/`)

## 校验

```bash
bash docs/verify_refs.sh
```

输出格式:
```
Validated segments: 502
OK:                 502
MISSING_FILE:       0
OUT_OF_RANGE:       0
```

修改源码后(尤其是行号偏移)请重跑此脚本,根据 `OUT_OF_RANGE` 输出对应修正文档。

## 注意

- 所有路径相对 `src/PIN_SLAM/`,从 `/home/steve/lio_ws/src/PIN_SLAM/` 目录运行 `verify_refs.sh`
- 文档中的 ASCII 图、数学公式与 `[VERIFY:]` 锚点是 ground truth;若发现冲突以源码为准并提 issue 修正文档
- 当代码做 refactor(函数移行)时,只需要更新 `[VERIFY:]` 的行号,文字断言通常不变 — 这正是这套引用方案的目的

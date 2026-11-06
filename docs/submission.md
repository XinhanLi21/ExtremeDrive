# Submissions and evaluation

## Submission format

JSONL with one record per token: `token`, `times_s=[0.5,...,4.0]`, `poses` of shape 8×3. See [schema](DATA_FORMAT.md). Duplicate tokens, wrong horizon, invalid shapes and nonfinite coordinates are rejected.

```bash
extremedrive --assets assets validate --submission predictions.jsonl
extremedrive --assets assets evaluate --submission predictions.jsonl --workers 4 --out results/model
```

The default requires every token in the 131-window cohort. `--subset` permits explicitly labeled diagnostics only. Full test displacement evaluation requires all 341 predictions:

```bash
extremedrive --assets assets displacement --submission full_test.jsonl --out results/test
```

## Outputs

`summary.json` records score, scope and unknowns; `scores/` contains per-window metrics/events and identity hashes; `rollouts/` contains states, control requests and actuator response. `metrics.csv` and `index.html` are offline results. `visualize` exports an executed-trajectory BEV.

Unknown evidence stays in the denominator and produces bounds; it is not silently removed or assigned a pass. Compare runs with the same toolkit, asset and protocol versions, cohort, and review policy.

## Archived reference

```bash
extremedrive --assets assets reference --model pluto --out results/reference
```

This verifies prediction/result hashes and aggregates the stored reviewed results. It does not run inference or simulation. Historical human decisions apply only to their original trajectories. Fresh evaluation does not inherit those decisions. Thus automated and reviewed results must retain their respective labels.

This repository provides a local evaluation library; it does not claim to host a public submission server or an official NAVSIM leaderboard.

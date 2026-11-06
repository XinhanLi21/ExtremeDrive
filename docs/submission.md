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

Windows with missing required evidence remain in the evaluation set and contribute to score bounds. Compare runs with the same toolkit, asset and protocol versions, cohort, and review policy.

## Archived reference

```bash
extremedrive --assets assets reference --model pluto --out results/reference
```

The `reference` command verifies prediction/result hashes and aggregates the supplied reviewed results. Trajectory-specific review records apply to their associated predictions. The `evaluate` command runs automatic execution and scoring for new submissions. Reports distinguish these evaluation modes.

Evaluation runs locally through the toolkit CLI.

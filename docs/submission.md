# Submissions and evaluation

## Submission format

JSONL with one record per token: `token`, `times_s=[0.5,...,4.0]` and 8×3 poses at 2 Hz, or `times_s=[0.25,...,4.0]` and 16×3 poses at 4 Hz. See [schema](DATA_FORMAT.md). Duplicate tokens, wrong horizon, invalid shapes and nonfinite coordinates are rejected.

```bash
extremedrive --assets assets validate --submission predictions.jsonl
extremedrive --assets assets evaluate --submission predictions.jsonl --workers 4 --out results/model
```

The default requires every token in the 2,574-clip cohort. `--subset` permits explicitly labeled diagnostics only. Full test displacement evaluation requires all 2,574 predictions:

```bash
extremedrive --assets assets displacement --submission full_test.jsonl --out results/test
```

## Outputs

`summary.json` records score, scope and unknowns; `scores/` contains per-window metrics/events and identity hashes; `rollouts/` contains states, control requests and actuator response. `metrics.csv` and `index.html` are offline results. `visualize` exports an executed-trajectory BEV.

All configured clips remain in the evaluation denominator. Indeterminate metric outcomes contribute to the reported score bounds. Compare runs with the same toolkit, asset and protocol versions, cohort, and review policy.

## Formal comparison

```bash
extremedrive --assets assets compare --summary results/model_a/summary.json \
  --summary results/model_b/summary.json --out results/comparison.json
```

Only complete `sensor_only` / `automatic_only_v1` runs with identical evaluation identities are accepted. Predictions produced by `predict` carry `input_policy`. External submissions must declare it accurately; missing declarations are accepted for diagnostic scoring but cannot enter formal comparisons. Mixed policies in one submission are rejected. This metadata is a disclosure requirement, not a security guarantee against an adapter reading labels.

Evaluation runs locally through the toolkit CLI.

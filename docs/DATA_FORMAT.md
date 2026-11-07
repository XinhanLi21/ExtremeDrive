# Dataset, assets and submissions

## Directory layout

```text
assets/
  splits/{train,val,test}.json   # original fixed split files, byte-preserved
  cohort.json                   # ordered execution token definitions
  contexts/*.json.xz            # per-window data and shared-geometry references
  context_blobs/*.json.xz       # content-addressed static maps and road geometry
  sampling/                    # 4 Hz sensor-derived labels and real history-frame selections
  protocol.json                 # geometry and timing rules
  mohe_profile.json             # fixed vehicle parameter candidate
  huggingface.json              # source repository, revision and file index
  reference/<model>/            # predictions and reviewed per-window results
  reference_bindings.json       # exact prediction/result identities
  manual_review.json            # scoped review provenance
  manifest.json                 # asset SHA-256 checksums
```

## Python API

```python
from extremebench.toolkit.data import Dataset

ds = Dataset('assets', dataroot='/path/to/data', split='test')
inputs = ds.inputs(ds.tokens()[0])
execution_tokens = ds.tokens(execution=True)
```

`inputs` exposes token, image paths, policy-controlled text and measured speed, sampled history images, history and validity flags, and the requested prediction time grid. `target(token)` is a separate supervised-label accessor. `context(token)` is reference annotations for evaluation and must never be supplied to the planning agent. The toolkit does not use CoC future descriptions as planner inputs.

`context` includes scene geometry in metres, anchor pose, observed actors in the same map frame, corrected relative observation times, reviewed drivable geometry, and measured initial state. Its original source-path fields are provenance; runtime data loading uses the supplied data root. Coordinates are not inferred from source directory names.

## JSONL submission

Each line contains exactly one window prediction. Required keys:

- `token`: a fixed split/cohort token.
- `times_s`: 0.5-second steps through 4.0 s (2 Hz) or 0.25-second steps through 4.0 s (4 Hz).
- `poses`: eight or sixteen `[x_m,y_m,yaw_rad]` rows in the capture-anchor ego frame, x forward/y left, yaw counterclockwise.

Duplicates, missing/extra cohort tokens, wrong shape, invalid timing and nonfinite numbers are rejected. The `input_policy` field records the information contract. Missing policies are diagnostic only; formally comparable submissions declare `sensor_only`. Additional model metadata cannot change evaluation scope.

Use `examples/smoke_predictions.jsonl` as a valid example. All 131-window reference prediction files use the same format. Current planning inputs contain three selected camera streams; raw upstream episodes contain additional sensors available to custom adapters. Adding sensor inputs does not change the frozen split or scoring protocol.

## Reproducibility

Original split files include paths from the source machine. `Dataset.inputs` remaps them to `dataroot` at access time, so split bytes and their original hashes remain unchanged. The corrected labels in these files supersede nominal filename-timing reconstructions. Source dataset directory split labels are metadata, not benchmark split assignments.

LQR dependency modules are included under a private namespace, with original and relocated file hashes. Only import namespaces were changed. No external NAVSIM checkout is needed by the released primary execution path.

## Lossless storage (0.4.1)

Static scene metadata/features and road objects are stored by SHA-256 content identity. Exactly equal objects share one file; distinct revisions or geometries are never merged based on scene name. Actor frames and their relative timestamps remain window-specific. JSON preserves all coordinates, numbers, fields and array order; LZMA/XZ compresses bytes without quantization or geometry simplification.

At load time, the data API verifies shared-blob hashes, reconstructs the original context, and verifies its canonical JSON SHA-256. No shared mutable object cache is used: modifying one returned context cannot modify another window. The old `.json.gz` context layout is still readable. Only storage changes; evaluators receive the same Python structure.

NAVSIM also uses LZMA for metric caches ([upstream implementation](https://github.com/autonomousvision/navsim/blob/main/navsim/planning/metric_caching/metric_cache.py)). This release uses JSON rather than pickle and additionally deduplicates shared map objects. No OpenScene-style temporal subsampling is applied.

Sampling-rate selection and real capture timestamps are documented in [sampling.md](sampling.md).

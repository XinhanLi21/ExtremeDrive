# Validation and reproducibility

The repository contains one current dataset manifest and one benchmark implementation.

## Current release

- The planning manifests contain 11,541 training, 1,285 validation and 2,574 test clips: 15,400 clips across the original 336 source episodes. Episode and scene membership is fixed across splits.
- No source episode, map scene, anchor image or token is shared across splits. Clips overlap within their own episode.
- Both 2 Hz and 4 Hz inputs/targets were checked on every clip. Shared target timestamps agree within 1e-10; causal image slots carry explicit validity masks.
- The execution cohort contains 2,574 contexts. Their reconstruction was checked; eight representative recorded-trajectory checks cover six categories and actor-evidence handling. Another 152 recorded-trajectory executions validate observed overlap-frame alignment, with no execution failures. These are integration checks, not model-performance results. See the [pose-alignment check](pose-alignment-check.json).
- The 3D-only policy has explicit tests for unmatched 2D detections, observed 3D collisions, observation validity and temporal continuity.
- The asset manifest verifies current data files by SHA-256. The downloader resolves 56 execution-source episodes and 336 planning-source episodes.
- Conda manages the runtime environment. CI checks Python 3.10 and 3.12, unit tests, asset integrity, execution regression cases and packaging.

## Execution checks

The smoke example checks fresh execution outputs, token coverage, finite trajectories and report generation. Asset integrity and unit tests validate the current distribution.

The default comfort configuration reproduces the earlier effective computation within floating-point tolerance. Threshold-change tests verify that configuration edits actually affect scoring. Formal comparisons require the same protocol, assets, cohort, trajectory frequency and input policy, with automatic scoring for every model.

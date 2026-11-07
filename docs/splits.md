# Data splits

| Split | Clips | Source episodes | Interface |
|---|---:|---:|---|
| Training (`train`) | 11,541 | 248 | Agent inputs and supervised trajectory targets |
| Validation (`val`) | 1,285 | 32 | Checkpoint selection and trajectory evaluation |
| Test (`test`) | 2,574 | 56 | Full-test ADE/FDE |
| Execution evaluation | 2,422 | 56 | Trajectory tracking, vehicle execution and safety metrics |

A **clip** is a four-second planning sample. Anchors follow each available 0.25-second filename slot within an episode. Labels retain the corrected real capture and sensor clocks; changing trajectory sampling between 2 Hz and 4 Hz does not change the clip count. Clips overlap and are not independent driving episodes.

The original episode and scene assignments are preserved across train, validation and test. Clip manifests have been regenerated with dense anchors; they are not byte-identical to the former sparse manifests. Episodes, map scenes, anchor images and tokens are disjoint across splits. Rebuild feature caches and predictions for the current manifests rather than relabeling earlier results.

`assets/cohort.json` specifies the execution tokens. The remaining 152 test clips lack the required map-pose/time coverage in the reviewed snapshot and remain available for displacement evaluation. See [annotation coverage](coverage.md) for the current evidence scope.

The execution cohort covers all six categories: normal driving, complex traffic, critical driving, rain, low light and snow. Comfort applies to noncritical clips; emergency-safety success applies to critical-driving clips. Each metric reports its applicability denominator. Aggregate safety metrics only over the declared execution cohort, not over all test clips.

Use test labels only for evaluation. Any model's prior exposure to source episodes must be disclosed independently: benchmark split assignments do not erase pretraining overlap.

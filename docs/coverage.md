# Execution annotation coverage

The current release provides 2,422 executable contexts from 2,574 test clips, using 37 accepted maps. Existing reviewed road surfaces, actor boxes and sensor-clock alignment are reused. The remaining 152 clips lack map-pose/time coverage in the current snapshot; no map pose is extrapolated to fill that gap.

| Evidence | Clips |
|---|---:|
| Connected execution contexts | 2,422 |
| Annotated-3D collision observations available | 2,422 |
| No missing 3D actor velocity evidence in the coverage check | 1,065 |
| At least one 3D actor with incomplete velocity evidence | 1,357 |
| Missing map-pose/time coverage; not connected | 152 |

Collision and TTC use annotated 3D actors only. Unmatched 2D detections do not invalidate a readable observation. An empty 3D actor set is evaluated as empty; it does not certify the absence of unannotated real-world actors. Missing annotation files or temporal gaps remain unknown.

A velocity-evidence gap is not automatically an unknown TTC result. The scorer tests whether the target can conflict with the submitted executed trajectory; only relevant targets require sufficient velocity evidence. Coverage counts are annotation diagnostics, not model scores.

[The coverage manifest](../assets/execution_coverage.json) lists every test token, source episode, map revision, connection status and remaining evidence gaps. The protocol identity records the 3D-only policy. Historical scores must not be reused as results on this expanded cohort.

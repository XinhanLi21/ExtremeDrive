# Execution evaluation coverage

All 2,574 test clips support execution-based evaluation using 37 accepted maps. Each clip is associated with a scene coordinate frame, observed ego poses, synchronized timestamps, reviewed road surfaces and recorded 3D actors.

| Resource | Count |
|---|---:|
| Test clips | 2,574 |
| Execution contexts | 2,574 |
| Source episodes | 56 |
| Accepted maps | 37 |

Road geometry follows the reviewed scene polygons. Ego poses and timestamps are aligned with the original sensor records through the established scene transforms. Collision and TTC evaluate the annotated 3D actor set; 2D detections are retained as annotation metadata and do not participate in geometric scoring.

TTC combines adjacent-frame motion estimates with trajectory-dependent conflict analysis. The [evaluation protocol](PROTOCOL.md) defines metric applicability, scoring states and aggregation.

The [execution manifest](../assets/cohort.json) identifies the test clips used for execution evaluation. The [annotation metadata](../assets/execution_coverage.json) records scene associations, map revisions and evidence provenance. All methods must use the same released inputs and evaluation protocol.

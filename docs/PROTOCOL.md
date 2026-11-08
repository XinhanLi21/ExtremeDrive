# Evaluation protocol

ExtremeDrive evaluates planned trajectories under a common execution protocol. This document defines the trajectory-execution process, metrics and aggregation rules. Geometry and temporal thresholds are specified in `assets/protocol.json`.

## Input contract

The default input policy removes scenario-derived action hints. The input contract uses sensor observations and measured vehicle state; it does not infer route commands from scenario annotations. See [input provenance and ablations](input_policy.md).

## Execution

1. Eight (2 Hz) or sixteen (4 Hz) ego-relative poses over four seconds are transformed into the aligned scene frame.
2. The reference is sampled for LQR at 10 Hz. Measured initial longitudinal/lateral velocity and yaw rate initialize the vehicle.
3. Acceleration and steering-rate requests are translated into wheel torque and steering targets, with actuator delay, lag, limits and rate constraints.
4. A seven-degree-of-freedom model integrates planar motion and four wheel rotations at 1 kHz. Effective lateral parameters are fitted from vehicle recordings. Parameter provenance and validation status are included in the execution output.
5. Geometry metrics sample executed motion at 20 Hz. Other actors follow recorded observations; they do not react to the simulated ego vehicle. The planner is not queried again during this four-second execution.

Vehicle initialization uses measured motion states together with fixed actuator and wheel-state assumptions. Parameter values and initialization details are retained in the configuration and execution logs.

## Metrics

| Metric | Definition |
|---|---|
| Collision avoidance N | No overlap above the specified area threshold with observed traffic participants |
| Road compliance R | Joint compliance with white/yellow markings, the reviewed drivable surface, and annotated physical exclusion regions. All applicable conditions must hold. Marking tolerance is 0 cm; the road-surface geometry allowance is 0.15 m. |
| TTC T | Constant-velocity footprint conflict test over one second, at 0.1 s increments |
| Progress P | Route-relative progress within the specified route-distance threshold |
| Comfort C | Duration-weighted severity of executed acceleration, jerk and yaw motion; only noncritical test windows; [active limits and filtering](comfort.md) |
| Emergency safety | Critical-driving windows satisfying collision, road and TTC safety conditions over the four-second horizon |

The 0.15 m road-surface allowance is distinct from the **0 cm marking rule**; it does not permit crossing a marked boundary. Emergency safety measures compliance with the specified safety criteria over the evaluation horizon.

For applicable comfort flag a ∈ {0,1}:

`score = 100 × N × R × (5P + 5T + 2aC) / (10 + 2a)`

Physical exclusion constraints apply where annotated. A known failed hard gate gives score zero. Otherwise, unavailable required evidence produces an unknown score, not an automatic pass. EC is absent. Traffic lights and counterfactual follow-on scenes are outside this protocol.

## TTC evidence and review

TTC evaluates predicted footprint conflicts within the configured horizon using the released actor scope and adjacent-frame motion estimates. Reviewed actor relevance annotations are shared across models; observed overlaps remain violations.

Formal comparisons use `automatic_only_v1` for every model: collision and TTC are computed by the same code on the same annotation snapshot, with no prediction-specific human overrides. Unknown evidence remains unknown. Archived reviewed results retain their prediction bindings but are marked `benchmark_eligible=false`; they cannot enter the `compare` command. Re-scoring an archived prediction recomputes the metrics and does not import review outcomes.

## Aggregation

The default execution run requires all 2,574 tokens. Unknowns remain in the denominator; the output reports lower/upper bounds and separately labeled known-only means. Comfort and emergency safety include their own applicability counts. A partial diagnostic run is explicitly marked as incomplete.

The `compare` command requires complete sensor-only automatic runs with equal protocol, asset, cohort and output-rate identities. Hint-conditioned runs and unspecified input declarations remain diagnostic results. Comparisons use the same split, execution configuration, metric definitions and review policy. These definitions constitute the ExtremeDrive evaluation protocol.

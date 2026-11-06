# Evaluation protocol

ExtremeDrive evaluates planned trajectories under a common execution protocol. This document defines the trajectory-execution process, metrics and aggregation rules. Geometry and temporal thresholds are specified in `assets/protocol.json`.

## Execution

1. Eight ego-relative poses at 0.5–4.0 seconds are transformed into the aligned scene frame.
2. The reference is sampled for LQR at 10 Hz. Measured initial longitudinal/lateral velocity and yaw rate initialize the vehicle.
3. Acceleration and steering-rate requests are translated into wheel torque and steering targets, with actuator delay, lag, limits and rate constraints.
4. A seven-degree-of-freedom model integrates planar motion and four wheel rotations at 1 kHz. Effective lateral parameters are fitted from vehicle recordings. Parameter provenance and validation status are included in the execution output.
5. Geometry metrics sample executed motion at 20 Hz. Other actors follow recorded observations; they do not react to the simulated ego vehicle. The planner is not queried again during this four-second execution.

Vehicle initialization uses measured motion states together with fixed actuator and wheel-state assumptions. Parameter values and initialization details are retained in the configuration and execution logs.

## Metrics

| Metric | Definition |
|---|---|
| Collision avoidance N | No overlap above the specified area threshold with observed traffic participants |
| Marking compliance M | Joint compliance with white and yellow road markings, evaluated using the vehicle footprint; zero crossing tolerance and zero persistence |
| Drivable-area compliance D | Executed body lies within the reviewed road surface and a 0.15 m outward geometry allowance |
| Direction Q | Same/opposite direction from road topology; wrong-way distance within one second produces factors 1, 0.5, 0 at 2 m and 6 m thresholds |
| Exclusion X | Compliance with annotated physical exclusion geometry, when available |
| TTC T | Constant-velocity footprint conflict test over one second, at 0.1 s increments |
| Progress P | Route-relative progress within the specified route-distance threshold |
| Comfort C | Duration-weighted severity of simulated acceleration, jerk and yaw motion; only noncritical test windows |
| Emergency safety | Critical-driving windows satisfying collision, marking, road/direction and TTC safety conditions over the four-second horizon |

The 0.15 m road-surface allowance is distinct from the **0 cm marking rule**; it does not permit crossing a marked boundary. Emergency safety measures compliance with the specified safety criteria over the evaluation horizon.

For applicable comfort flag a ∈ {0,1}:

`score = 100 × N × M × D × Q × X × (5P + 5T + 2aC) / (10 + 2a)`

Unavailable exclusion geometry contributes no additional gate. A known failed hard gate gives score zero. Otherwise, unavailable required evidence produces an unknown score, not an automatic pass. EC is absent. Traffic lights and counterfactual follow-on scenes are outside this protocol.

## TTC evidence and review

Actor association and local adjacent-frame fits are supplied with the annotation snapshot. Unknown-velocity actors are excluded from TTC only when a conservative reachable set cannot intersect the ego sweep. Risk-relevant actors without velocity evidence remain unknown. Source annotations and review decisions are kept distinct.

Reference results include trajectory-specific collision and TTC review records. Each result is bound to its corresponding prediction by a content hash. New submissions use automatic evaluation; reference review decisions apply exclusively to the associated predictions. Reports identify the evaluation and review policy used.

## Aggregation

The default execution run requires all 131 tokens. Unknowns remain in the denominator; the output reports lower/upper bounds and separately labeled known-only means. Comfort and emergency safety include their own applicability counts. A partial diagnostic run is explicitly marked as incomplete.

Comparisons use the same split, execution configuration, metric definitions and review policy. These definitions constitute the ExtremeDrive evaluation protocol.

# Evaluation protocol

The release freezes the existing ExtremeDrive execution benchmark. `assets/protocol.json` stores geometry and temporal thresholds; source and asset SHA-256 manifests accompany the release.

## Execution

1. Eight ego-relative poses at 0.5–4.0 seconds are transformed into the aligned scene frame.
2. The reference is sampled for LQR at 10 Hz. Measured initial longitudinal/lateral velocity and yaw rate initialize the vehicle.
3. Acceleration and steering-rate requests are translated into wheel torque and steering targets, with actuator delay, lag, limits and rate constraints.
4. A seven-degree-of-freedom model integrates planar motion and four wheel rotations at 1 kHz. The effective lateral parameter candidate is fitted from the MoHe records; its experimental validation status remains in the output.
5. Geometry metrics sample executed motion at 20 Hz. Other actors follow recorded observations; they do not react to the simulated ego vehicle. The planner is not queried again during this four-second execution.

Initial steering and torque are zero; initial wheel speed is longitudinal speed divided by wheel radius. These assumptions and the SL03 reference-point offset are not newly measured by this release. The fitted parameter snapshot is unchanged.

## Metrics

| Metric | Definition |
|---|---|
| Collision avoidance N | No overlap above the frozen area threshold with observed actor polygons |
| Marking compliance M | Combined white/yellow violation rule, 0 cm tolerance and zero persistence, using the tapered SL03 approximate outline (4.82 × 1.89 m) |
| Drivable-area compliance D | Executed body lies within the reviewed road surface and the existing 0.15 m outward geometry allowance |
| Direction Q | Same/opposite direction from road topology; wrong-way distance within one second produces factors 1, 0.5, 0 at 2 m and 6 m thresholds |
| Exclusion X | Compliance with annotated physical exclusion geometry, when available |
| TTC T | Constant-velocity footprint conflict test over one second, at 0.1 s increments |
| Progress P | Route-relative progress under the frozen route-distance gate |
| Comfort C | Duration-weighted severity of simulated acceleration, jerk and yaw motion; only noncritical test windows |
| Emergency safety | Critical-driving windows satisfying collision, marking, road/direction and TTC safety conditions over the four-second horizon |

The 0.15 m road-surface allowance is distinct from the **0 cm marking rule**; it does not permit crossing a marked boundary. Emergency safety is a short-horizon safety metric, not proof of completing an entire evasive maneuver.

For applicable comfort flag a ∈ {0,1}:

`score = 100 × N × M × D × Q × X × (5P + 5T + 2aC) / (10 + 2a)`

Unavailable exclusion geometry contributes no additional gate. A known failed hard gate gives score zero. Otherwise, unavailable required evidence produces an unknown score, not an automatic pass. EC is absent. Traffic lights and counterfactual follow-on scenes are outside this protocol.

## TTC evidence and review

Actor association and local adjacent-frame fits are supplied with the annotation snapshot. Unknown-velocity actors are excluded from TTC only when a conservative reachable set cannot intersect the ego sweep. Risk-relevant actors without velocity evidence remain unknown. Source annotations and review decisions are kept distinct.

The historical reference includes the user's scoped collision/TTC review decisions. Their exported result and prediction hashes are bound together; reference mode only loads those exact records. Fresh predictions never inherit historical review verdicts. Fresh automatic scores and historical reviewed results must be identified explicitly when compared.

## Aggregation

The default execution run requires all 131 tokens. Unknowns remain in the denominator; the output reports lower/upper bounds and separately labeled known-only means. Comfort and emergency safety include their own applicability counts. A partial diagnostic run is explicitly marked as incomplete.

The seven reference models and the old split stay fixed. This toolkit does not claim numerically interchangeable NAVSIM/nuScenes leaderboard scores.

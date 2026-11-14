# Scene friction

The execution model uses empirical road-condition coefficients:

| Surface | Friction coefficient |
|---|---:|
| Snow | 0.30 |
| Wet | 0.60 |
| Dry | 0.85 |

`assets/scene_friction.json` contains source-episode assignments, source annotation hashes and bindings for every execution clip. Source road-surface annotations determine the surface class; snow-scene labels identify snow-covered/slushy driving. Weather and lighting alone do not determine friction. The assignments are empirical estimates, not direct friction measurements.

All planners use the same assignment for a given source scene. Where a map spans episodes with different road-surface annotations, each clip retains its source-episode assignment. No coefficient is inferred from the planner's prediction or its score. These evaluation parameters are not added to default planner inputs.

The assigned coefficient is applied to the vehicle configuration before trajectory execution. Execution outputs record the surface, coefficient, source episode and assignment-table hash. Formal comparisons require identical assignment-table hashes. Missing assignments or inconsistent episode bindings stop execution instead of silently falling back to a global coefficient.

The standalone vehicle class retains its dry-surface default. Benchmark execution overrides it with the released scene assignment. Changes to friction require new execution runs; existing scores are not reinterpreted as results under the changed coefficients.

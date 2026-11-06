# Contributing

Use a separate branch. Install with `pip install -e '.[dev]'`, run unit tests, verify assets, and execute the smoke evaluation before proposing changes.

Benchmark definitions are versioned: changes to splits, coordinates, clocks, geometry, controller/vehicle parameters, metric thresholds or review scope require an explicit new protocol/asset version and before/after comparisons. Do not silently regenerate accepted annotations.

Agent inputs must remain causal. Unknown evidence must not be replaced with an automatic pass. Keep source attribution and third-party notices. Do not commit raw sensor dumps, credentials, checkpoints or result caches.

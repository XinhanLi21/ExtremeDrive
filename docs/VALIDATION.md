# Release validation — 2026-11-06

- Seven unit tests passed: fixed splits and input/label separation; upstream episode coverage; unsafe archive traversal and link rejection; archive layout; duplicate/nonfinite submission rejection; unknown denominator handling; prediction identity changes.
- Asset manifest verifies 155 files, including all 131 execution contexts.
- All 917 prediction/result bindings verified. Aggregates for seven reference models match the existing reviewed result tables within 1e-10.
- Three existing PLUTO predictions (critical driving, low light, snow) were rerun through LQR, actuators, seven-DOF dynamics and scoring. In the original runtime, all three execution trajectories and composite scores exactly matched the corresponding existing results.
- A fresh Python 3.12 environment installed the toolkit and its declared dependencies. The same three cases were executed from `/tmp`, outside the source tree. Maximum trajectory element difference was below 8e-14; composite-score difference was below 9e-14.
- The pinned Hugging Face repository file index was retrieved; all 13 execution-source episodes and all 336 episodes required by the fixed splits were resolved. A README and a real episode metadata.jsonl were downloaded successfully through huggingface_hub. Large sensor archives were not downloaded again; extraction layout and unsafe-member rejection were tested with fixture archives.
- Offline HTML/CSV export and BEV rendering were exercised.

Validation does not constitute a new evaluation of all seven models: their 917-result table is a verified historical reference. New execution testing covered three representative windows. Use the full cohort command for a new model evaluation.

## 0.4.1 storage-only revision

- 131 contexts reconstructed from shared geometry and LZMA/XZ storage; all canonical JSON hashes match 0.4.0. See compression-validation.json.
- No scorer, split, vehicle model, observation, geometry or numeric precision change.
- Three cross-condition fresh executions reproduce exactly the original trajectory, score, component metrics and comfort values. See compression-score-check.json.

## 0.5.0 Git repository packaging

- All 296 asset files (including the manifest) remain byte-identical to 0.4.1.
- Nine local unit tests passed, including the public agent interface.
- Three fresh smoke executions match frozen trajectories, scores and safety gates within 1e-8.
- Wheel and source distributions built successfully; the wheel includes the agent API and vendored licenses, and the source distribution includes benchmark assets and the notebook.
- Main documentation links were checked locally. GitHub Actions is configured but has not run on GitHub because this repository has not been pushed.

# Validation and reproducibility

The current toolkit uses one benchmark implementation and one release archive.

## Runtime and evaluation

- The Conda environment was created from `environment.yml` with Python 3.12.
- Twelve unit tests cover data splits, input isolation, agent outputs, submission validation, archive extraction, protocol defaults and score applicability.
- Three representative trajectories are executed through the controller and vehicle model. Executed trajectories, composite scores and safety gates match the reference records within 1e-8.
- The 294 files listed in the asset manifest are verified by SHA-256. All 917 reference prediction/result bindings were verified, and seven-model aggregate results matched within 1e-10.

## Data and storage

- All 131 reconstructed execution contexts were checked for lossless equivalence. The 140 shared geometry blobs are referenced, with no missing or unused blobs.
- The pinned Hugging Face index resolves 13 execution-source episodes and 336 episodes used by the planning splits. Dataset documentation and an episode metadata file were downloaded successfully. Large sensor archives were not downloaded again during toolkit validation.
- Package contents exclude environments, Git history, bytecode, build caches and generated evaluation outputs.

## Reproduction records

- [Lossless context validation](compression-validation.json)
- [Storage-change execution checks](compression-score-check.json)
- [Environment portability checks](portability-check.json)
- [Reference aggregate checks](reference-check.json)

These records distinguish the verified historical reference table from fresh execution tests. Full new-model evaluation uses the complete execution cohort. GitHub workflows are configured; local validation has been run independently of GitHub.

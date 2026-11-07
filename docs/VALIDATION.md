# Validation and reproducibility

The current toolkit uses one benchmark implementation and one release archive.

## Runtime and evaluation

- The Conda environment was created from `environment.yml` with Python 3.12.
- Twenty-eight unit tests cover data splits, input isolation, agent outputs, submission validation, archive extraction, protocol defaults and score applicability.
- Three representative trajectories are executed through the controller and vehicle model. Executed trajectories, composite scores and safety gates match the reference records within 1e-8.
- The 298 files listed in the asset manifest are verified by SHA-256. All 917 reference prediction/result bindings were verified, and seven-model aggregate results matched within 1e-10.

- All 1,188 windows retain identical labels at shared 2 Hz / 4 Hz timestamps (absolute tolerance 1e-10). Causal image history is checked for missing slots and reused observations.
- Three constant-velocity diagnostic trajectories produce matching execution scores at 2 Hz and 4 Hz.

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

## Input and scoring contract

The current protocol removes unverified scenario hints from default inputs and requires automatic-only formal comparisons. Comfort limits and filter settings have one active configuration. Tests verify that editing a threshold changes comfort, default effective values reproduce the previous calculation, emergency hints cannot become online navigation, and archived/mixed-policy results cannot enter formal comparisons. Original split files and archived result bindings remain unchanged.

The input-provenance audit matched all 1,188 split prompts to their source episode metadata. Paired inference was completed on all 341 test windows for the two command-conditioned sensor adapters. The 293 windows without emergency hints remained numerically identical in the emergency-hint-only ablation. This is an inference-time test, not no-hint retraining.

Automatic-only scoring was recomputed for seven models on 131 windows (917 records), without historical review overrides. The two command-conditioned models additionally completed 262 fresh executions using neutral inputs. The resulting seven summaries passed the shared protocol, assets, cohort, sampling and input-policy comparison checks. Automatic unknowns remain unknown.

Across 623 applicable historical trajectories, the centralized default comfort calculation differed from the previous effective implementation by at most 3.33e-16. This configuration repair does not relax comfort limits.

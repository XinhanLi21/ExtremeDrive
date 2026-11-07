# Input provenance and controlled hint removal

## Source and timing

The original split `language` was constructed by `prepare_hdp_time_aligned.py` from the aligned episode's `metadata.jsonl` `prompt` plus measured speed. The prompt is a source scenario annotation. Its creation timestamp and its availability to the driver/planner at the capture time are **not recorded**. A capture timestamp identifies the observation; it does not establish when the prompt was written. The toolkit therefore does not certify any source scenario prompt as an online navigation command.

`Dataset.instruction_annotation(token)` exposes source episode, capture timestamp, original annotation, a normalized navigation annotation (when recognized), and a separate emergency-maneuver hint. These are audit/training annotations, not default planner inputs. Unknown annotation creation and online availability are represented by null, not fabricated dates. Original split files and labels are unchanged.

## Input policies

| Policy | Text supplied to the planner | Use |
|---|---|---|
| `sensor_only` (default) | Current speed and generic four-second prediction request | Formal benchmark; no scenario/navigation/action annotation |
| `navigation_only` | Same neutral text in this release; `navigation_valid=false` | Reserved for future verified online route commands; never infer a command from scenario labels |
| `no_maneuver_hint` | Remove emergency/avoidance/collision hints; retain other original scenario text | Controlled ablation only; retained navigation-like annotations are still not proven online |
| `scenario_hint` | Original annotated instruction | Historical reproduction and hint-conditioned ablation only |

Here `sensor_only` defines the **text-information policy**, not a claim that every external model uses the same sensor modality. Camera, camera/LiDAR and annotated-vector models must still disclose their different input tracks. Inputs include `navigation_command=null`, `navigation_valid=false`, and `navigation_source=unavailable`; archived maneuver hints are never included as hidden default metadata.

```bash
extremedrive --assets assets predict --agent my_agent:Agent --dataroot data \
  --input-policy sensor_only --execution-only --out no_hint.jsonl
extremedrive --assets assets predict --agent my_agent:Agent --dataroot data \
  --input-policy no_maneuver_hint --execution-only --out no_emergency_hint.jsonl
extremedrive --assets assets predict --agent my_agent:Agent --dataroot data \
  --input-policy scenario_hint --execution-only --out with_hint.jsonl
```

The generic CLI passes policy-specific text. Native adapters that encode navigation as a numeric command must also remove the corresponding command features, rather than merely removing a string that the model never reads. Do not derive commands from episode paths, scenario labels, future trajectories or audit annotations. Stable tokens and source paths are identifiers, not semantic model inputs.

## Paired experiment

`scripts/experiments/ablate_cached_sensor_commands.py` evaluates the original TransFuser and DiffusionDrive checkpoints on all 341 unchanged test windows. It reuses their exact frozen perception embeddings and changes only the four-element scenario-command input. No-command is an all-zero vector, not the historical `other` category. Checkpoint hashes, feature-cache identity, sample order, eight-sample batches and per-batch random seeds are shared across conditions. Both the full text-removal and emergency-hint-only conditions are run.

This is an **inference-time counterfactual on checkpoints trained with scenario commands**. It measures dependence on the supplied hint; it does not establish the performance of a model trained without hints. A no-hint training experiment must use the neutral input policy in train, validation and test, and select checkpoints using validation only. Existing adapters that never consume scenario language are negative controls, not evidence that hints are harmless for language-conditioned methods.

The script requires the original experiment environment and cached trained checkpoints; it is not a dependency of the standalone toolkit. For new models, use the policy flags above with the same checkpoint and paired random seeds. Report the two experimental conditions separately; only the declared sensor-only automatic run enters the formal comparison command.

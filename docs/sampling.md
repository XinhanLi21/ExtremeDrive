# Sampling and timestamps

## Two sampling modes

| Setting | 2 Hz | 4 Hz |
|---|---|---|
| Trajectory step | 0.5 s | 0.25 s |
| Four-second prediction | 8 poses | 16 poses |
| 1.5-second history, including anchor | 4 slots | 7 slots |

Both rates are supported by the same toolkit. `--input-hz` selects history sampling; `--trajectory-hz` selects the prediction grid. They can be set independently and default to 2 Hz. Clips are anchored at each available 0.25-second filename slot. Selecting an input or trajectory frequency does not add, drop or reassign clips.

```bash
extremedrive --assets assets predict --agent extremebench.agents.constant_velocity:ConstantVelocityAgent \
  --dataroot data --input-hz 4 --trajectory-hz 4 --execution-only --out predictions_4hz.jsonl
extremedrive --assets assets validate --submission predictions_4hz.jsonl --trajectory-hz 4
extremedrive --assets assets evaluate --submission predictions_4hz.jsonl --trajectory-hz 4 --out results/4hz
```

Evaluation can infer the trajectory rate from `times_s`; specifying `--trajectory-hz` additionally checks that the submission matches that rate. All records in one submission must use the same trajectory rate. No prediction is silently downsampled or upsampled. Agents return poses at the `target_times_s` provided in their inputs.

## Timestamp convention

Release filenames use a nominal 0.25-second grid. Recovered capture timestamps in the 336 source episodes used by the planning split are nonuniform: median inter-frame interval 0.2069 s, with 10th/90th percentiles 0.1842/0.2773 s. The nominal release grid therefore does not define the vehicle-motion clock.

History images are selected from actual observations at or before each requested timestamp. Each observation is selected at most once per history, with a maximum age of one requested sampling interval. Unsupported slots contain `images: null` and `valid: false`. Actual capture timestamps and offsets accompany the selected images. No synthetic or repeated image is inserted into an unsupported slot.

`images` remains the current camera input. `image_history` contains the sampled real image sequence. `history`, `history_times_s` and `history_valid` describe ego-motion history; image validity and motion-history validity are separate. The agent must respect the relevant validity masks.

## Labels and evaluation

The 4 Hz trajectory labels and causal motion history were reconstructed from the same raw velocity and heading records used by the existing corrected labels. Future labels use sensor-time integration; history uses measurements at or before the capture anchor. The 4 Hz values match the 2 Hz values at every shared time for all 15,400 clips (absolute tolerance 1e-10).

`Dataset(..., input_hz=4, trajectory_hz=4)` exposes the 4 Hz configuration. `Dataset.target(token, hz=4)` accesses the sixteen-pose reference. Frequency-specific records are stored losslessly in `assets/sampling/`; `audit.json` records source fingerprints and split identities. Rebuilding these records requires the original aligned episodes and sensor logs; normal toolkit use needs only the bundled assets.

LQR, vehicle integration and metric sampling retain their existing clocks. ADE/FDE use references at the submission's sampling grid. A denser trajectory can change interpolation and tracking behavior, so scores at different output rates should be labeled with their rate.

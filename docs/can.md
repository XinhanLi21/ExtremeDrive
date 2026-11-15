# ExtremeDrive CAN signals

Aligned CAN measurements for 589 source episodes and 30,186 keyframes.
The export retains the matched source values without rounding or interpolation.
It is a keyframe-aligned export, not the complete high-rate CAN bus log.

## Download

[Download CAN signals](../assets/can/ExtremeDrive_CAN.zip?raw=true) · [SHA-256](../assets/can/ExtremeDrive_CAN.zip.sha256)

The 1.85 MB archive is included in Git clones and source ZIP downloads under
`assets/can/`. Download it independently using the link above, or read it directly
from the repository:

```bash
python examples/read_can.py --summary
```

No Hugging Face authentication is required for this bundled CAN export. RGB images,
LiDAR, original metadata and CoC annotations remain available through the
[source dataset download](dataset.md).

## Contents

- `frames.jsonl`: one record per source episode and zero-based metadata frame index.
- `episode_index.json`: episode identifiers and frame counts.
- `alignment_report.json`: source-lineage and timing audit.
- `manifest.json`: SHA-256 checksums and schema identifier.
- `read_can.py`: standard-library reader, usable without installing the toolkit.

## Signal fields

| Field | Meaning | Unit / interpretation |
| --- | --- | --- |
| `canfd.steer` / `steer` | Steering-wheel angle | Degrees; not front road-wheel angle |
| `canfd.whl_fl`, `whl_fr`, `whl_rl`, `whl_rr` | Front-left, front-right, rear-left, rear-right wheel channels | Original logger values; physical scaling and sign require the source signal specification |
| `canfd.spd` | Vehicle-speed channel | Original logger value; confirm scaling before physical calculations |
| `canfd.tq` / `tq` | Source torque-related channel | Raw value; do not assume calibrated Nm or propulsion torque |
| `canfd.stamp` | Matched CAN timestamp | Unix seconds |

`steer` and `tq` are convenience aliases of the corresponding `canfd` fields.
`canfd.brake_pressure` is present in 302 records only, in original logger units;
its absence is not zero pressure. No actuator command targets are supplied.

## Timing and joins

Join using `(episode, frame_index)` against the corresponding Hugging Face episode's
`metadata.jsonl`. `frame_index` is zero-based. `time_str` identifies the released
image filename timestamp; `source_time_str` preserves the original capture time
(Asia/Shanghai). The release filename clock was normalized; it is not a substitute
for the original acquisition clock when analyzing physical dynamics.

`release_rename_minus_source_ms` and `canfd_stamp_minus_source_frame_ms` record
the two timing differences. The latter has an absolute median of 5.80 ms and
95th percentile of 19.51 ms in the alignment report. `matched` means a source
record was associated; it does not establish sensor calibration accuracy.
`source_id` is an opaque recording identifier and `source_line` preserves its
logger-line reference. Local absolute paths are excluded.

The episode's `train/` or `val/` prefix belongs to the source dataset. For benchmark
training/validation/testing, use `assets/splits/*.json` and each clip's
`context.source_episode`; do not redefine the benchmark partition from these prefixes.
The original dataset `action` contains trajectory deltas, not CAN control commands.

## Read

After extracting this archive:

```bash
python read_can.py --archive . --summary
python read_can.py --archive . --episode "train/Critical_Driving/Emergency_Left_Turn_To_Avoid_Obstacle/episode_5_part2" --limit 2
```

The reader also accepts the ZIP file directly.

```python
import json
from read_can import iter_can

episode = "train/Critical_Driving/Emergency_Left_Turn_To_Avoid_Obstacle/episode_5_part2"
by_frame = {r["frame_index"]: r for r in iter_can(".", episode=episode)}
with open("/path/to/dataset/" + episode + "/metadata.jsonl", encoding="utf-8") as f:
    metadata = [json.loads(line) for line in f if line.strip()]
assert set(by_frame) == set(range(len(metadata)))
for i, frame in enumerate(metadata):
    frame["canfd"] = by_frame[i]["canfd"]
    frame["can_source_time_str"] = by_frame[i]["source_time_str"]
```

## License

Source-derived data are distributed under CC BY-NC 4.0, following the source
dataset license and attribution: https://huggingface.co/datasets/Stary108/Extreme_Driving_Conditions_Dataset
License terms: https://creativecommons.org/licenses/by-nc/4.0/

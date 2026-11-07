# Executed-motion comfort

The sole active comfort configuration is `assets/protocol.json` → `comfort`, validated by `ComfortConfig`. `dynamic_comfort(logs, protocol.comfort)` reads this object. The former unused 4/4/8 threshold fields are removed; supplying them to `Protocol` is rejected rather than silently accepted. The default values below preserve the previously executed comfort calculation.

| Signal | Limit |
|---|---:|
| Positive longitudinal acceleration | 2.4 m/s² |
| Deceleration magnitude | 4.05 m/s² |
| Lateral acceleration magnitude | 4.89 m/s² |
| Inertial jerk norm | 8.37 m/s³ |
| Yaw-rate magnitude | 0.95 rad/s |
| Yaw-acceleration magnitude | 1.93 rad/s² |
| Longitudinal jerk magnitude | 4.13 m/s³ |

Signals come from the executed vehicle model, not derivatives of sparse submitted trajectory points. Longitudinal/lateral acceleration and yaw rate are filtered with a Savitzky–Golay filter: nominal 0.3 s window, polynomial order 3, SciPy `interp` boundary handling. The sample count is `max(5, int(round(window_s / dt)) // 2 * 2 + 1)`, capped at the largest odd count within the log. At least seven uniformly spaced samples are required. Logs report the effective sample count, time span and full configuration.

For body-frame filtered acceleration `(ax, ay)` and yaw rate `r`, the inertial jerk expressed in the body frame is `(dax/dt − r ay, day/dt + r ax)`. The norm of this vector is assessed separately from longitudinal jerk `dax/dt`. No warm-up portion is discarded: the complete executed time interval is evaluated.

Each channel is divided by its limit. At each instant, let `R(t)` be the maximum channel ratio. The instantaneous quality is `max(1, R(t)) ** (−severity_power)`, with `severity_power=2`. Comfort is its trapezoidal time integral divided by evaluated duration. This avoids averaging away a severe channel or counting correlated channels multiple times.

- `score`: continuous value in [0,1], used by the composite score.
- `strict_pass`: all ratios ≤ 1 + `strict_ratio_tolerance` (default 1e−9).
- `strict_time_fraction`: time fraction within the same limits.
- `severe_time_fraction`: time fraction with a ratio ≥ `severe_ratio` (default 2).

There is no additional persistence/grace threshold. Duration enters through the integral, so brief exceedances reduce the score in proportion to their duration and severity. These are benchmark comfort limits, **not measured physical vehicle limits**, and this continuous-duration score is not a claim of exact NAVSIM comfort compatibility.

Only noncritical windows contribute comfort to aggregate and composite scores; `Critical_Driving` windows are excluded. Diagnostic comfort signals may still be recorded for those windows. Low-light and snow windows remain applicable. `metrics.comfort` is the same active dynamic score when applicable, and null otherwise; there is no parallel geometry-derived comfort score.

Changing a comfort setting changes the protocol hash recorded in evaluation output. Compare only runs with identical protocol identities. A changed configuration is a new experiment, not the same frozen benchmark. The asset integrity manifest must be regenerated deliberately when distributing a revised protocol; never relabel historical scores as having used a new configuration.

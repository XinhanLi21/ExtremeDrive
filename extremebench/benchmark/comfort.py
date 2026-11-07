"""Duration-weighted comfort severity for executed vehicle motion."""
from dataclasses import dataclass
import numpy as np

def fraction(mask,t):
 return float(np.trapezoid(np.asarray(mask,float),t)/(t[-1]-t[0]))


@dataclass(frozen=True)
class ComfortConfig:
    version: str = 'executed-comfort-v1'
    acceleration_limit_mps2: float = 2.4
    deceleration_limit_mps2: float = 4.05
    lateral_acceleration_limit_mps2: float = 4.89
    inertial_jerk_limit_mps3: float = 8.37
    yaw_rate_limit_radps: float = .95
    yaw_acceleration_limit_radps2: float = 1.93
    longitudinal_jerk_limit_mps3: float = 4.13
    smoothing_window_s: float = .3
    smoothing_polynomial_order: int = 3
    severity_power: float = 2.
    severe_ratio: float = 2.
    strict_ratio_tolerance: float = 1e-9

    def validate(self):
        from dataclasses import asdict
        import math
        for key, value in asdict(self).items():
            if isinstance(value, (int, float)) and (not math.isfinite(value) or value < 0):
                raise ValueError('invalid comfort setting: '+key)
            if (key.endswith(('mps2','mps3','radps','radps2')) or key in ('smoothing_window_s','severity_power')) and value <= 0:
                raise ValueError('comfort setting must be positive: '+key)
        if self.severe_ratio <= 1 or self.smoothing_polynomial_order != 3:
            raise ValueError('severe_ratio must exceed 1; supported smoothing polynomial order is 3')
        return self


def aggregate_ratios(time,ratios,config=None):
    c=(config or ComfortConfig()).validate();t=np.asarray(time,float);r=np.asarray(ratios,float)
    if t.ndim!=1 or len(t)<2 or r.ndim!=2 or r.shape[1]!=len(t) or not np.isfinite(t).all() or not np.isfinite(r).all() or np.any(np.diff(t)<=0) or np.any(r<0):raise ValueError('finite monotonic time and nonnegative channel ratios required')
    if c.severity_power<=0 or c.severe_ratio<=1:raise ValueError('invalid severity settings')
    # Worst channel at each instant: avoid diluting a bad longitudinal signal
    # with five easy channels, and avoid double-counting correlated signals.
    worst=np.max(r,axis=0);quality=np.maximum(1.,worst)**(-c.severity_power)
    return dict(score=fraction(quality,t),strict_pass=int(np.all(worst<=1.+c.strict_ratio_tolerance)),strict_time_fraction=fraction(worst<=1.+c.strict_ratio_tolerance,t),severe_time_fraction=fraction(worst>=c.severe_ratio,t),maximum_limit_ratio=float(worst.max()),quality=quality.tolist(),worst_ratio=worst.tolist())


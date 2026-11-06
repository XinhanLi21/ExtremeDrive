"""Duration-weighted comfort severity for executed vehicle motion."""
from dataclasses import dataclass
import numpy as np

def fraction(mask,t):
 return float(np.trapezoid(np.asarray(mask,float),t)/(t[-1]-t[0]))


@dataclass(frozen=True)
class ComfortConfig:
    version:str='comfort-severity-duration-v4-proposal-1'
    longitudinal_jerk_limit_mps3:float=4.13
    severity_power:float=2.
    severe_ratio:float=2.


def aggregate_ratios(time,ratios,config=None):
    c=config or ComfortConfig();t=np.asarray(time,float);r=np.asarray(ratios,float)
    if t.ndim!=1 or len(t)<2 or r.ndim!=2 or r.shape[1]!=len(t) or not np.isfinite(t).all() or not np.isfinite(r).all() or np.any(np.diff(t)<=0) or np.any(r<0):raise ValueError('finite monotonic time and nonnegative channel ratios required')
    if c.severity_power<=0 or c.severe_ratio<=1:raise ValueError('invalid severity settings')
    # Worst channel at each instant: avoid diluting a bad longitudinal signal
    # with five easy channels, and avoid double-counting correlated signals.
    worst=np.max(r,axis=0);quality=np.maximum(1.,worst)**(-c.severity_power)
    return dict(score=fraction(quality,t),strict_pass=int(np.all(worst<=1.+1e-9)),strict_time_fraction=fraction(worst<=1.+1e-9,t),severe_time_fraction=fraction(worst>=c.severe_ratio,t),maximum_limit_ratio=float(worst.max()),quality=quality.tolist(),worst_ratio=worst.tolist())


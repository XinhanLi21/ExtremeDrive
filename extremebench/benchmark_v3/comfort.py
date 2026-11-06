"""Versioned ExtremeDrive severity-and-duration comfort proposal.
Same simulator, state filter and five legacy physical limits; adds the missing
NAVSIM longitudinal jerk check. Continuous score is not NAVSIM binary HC.
No human-trajectory waiver, initial-time exclusion or model-specific calibration.
"""
from dataclasses import dataclass,asdict
import numpy as np
from scipy.signal import savgol_filter
from .motion_metrics import future_comfort,MetricConfig,fraction
from ..benchmark_v2.metrics import Settings

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

def comfort_score(execution,config=None):
    c=config or ComfortConfig();lim=Settings();legacy=future_comfort(execution);t=np.array(legacy['time_s']);dt=t[1]-t[0];v=np.array([u['speed_mps'] for u in execution['controls']]);n=max(5,int(round(MetricConfig().state_filter_s/dt))//2*2+1);n=min(n,len(t) if len(t)%2 else len(t)-1)
    lonjerk=savgol_filter(v,n,3,deriv=2,delta=dt)
    s={k:np.asarray(x) for k,x in legacy['signals'].items()};s['longitudinal_jerk']=lonjerk
    ratios=dict(longitudinal=np.where(s['longitudinal']>=0,s['longitudinal']/lim.long_accel_max,s['longitudinal']/lim.long_accel_min),lateral=np.abs(s['lateral'])/lim.lateral_accel_max,jerk=np.abs(s['jerk'])/lim.jerk_max,yaw_rate=np.abs(s['yaw_rate'])/lim.yaw_rate_max,yaw_acceleration=np.abs(s['yaw_acceleration'])/lim.yaw_accel_max,longitudinal_jerk=np.abs(lonjerk)/c.longitudinal_jerk_limit_mps3)
    result=aggregate_ratios(t,np.array(list(ratios.values())),c)
    result.update(config=asdict(c),legacy_score=legacy['score'],legacy_strict_pass=legacy['strict_pass'],component_time_fraction={k:fraction(v<=1.+1e-9,t) for k,v in ratios.items()},component_continuous_score={k:fraction(np.maximum(1.,v)**(-c.severity_power),t) for k,v in ratios.items()},peak_limit_ratio={k:float(v.max()) for k,v in ratios.items()},time_s=t.tolist(),signals={k:v.tolist() for k,v in s.items()},thresholds=dict(longitudinal_min=lim.long_accel_min,longitudinal_max=lim.long_accel_max,lateral=lim.lateral_accel_max,jerk=lim.jerk_max,longitudinal_jerk=c.longitudinal_jerk_limit_mps3,yaw_rate=lim.yaw_rate_max,yaw_acceleration=lim.yaw_accel_max),signal_source=legacy['signal_source'],history_join_status=legacy['history_join_status'],status='proposed_engineering_comfort_proxy_not_subjective_comfort_or_official_NAVSIM',initial_transient_policy='included; no hidden grace period',threshold_source='https://github.com/autonomousvision/navsim/blob/main/navsim/planning/simulation/planner/pdm_planner/scoring/pdm_comfort_metrics.py')
    return result

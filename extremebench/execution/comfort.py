"""Same severity limits, actual dynamic acceleration/rate (no bicycle inference)."""
import numpy as np
from scipy.signal import savgol_filter
from ..benchmark.comfort import aggregate_ratios, ComfortConfig
from dataclasses import asdict

def dynamic_comfort(logs, config=None):
 c=(ComfortConfig(**config) if isinstance(config,dict) else config or ComfortConfig()).validate()
 t=np.array([r['t'] for r in logs]);dt=np.diff(t)
 if len(t)<7 or not np.allclose(dt,dt[0]):raise ValueError('at least 7 uniform samples required')
 n=max(5,int(round(c.smoothing_window_s/dt[0]))//2*2+1);n=min(n,len(t) if len(t)%2 else len(t)-1)
 def filtered(key,deriv=0):return savgol_filter([r[key] for r in logs],n,c.smoothing_polynomial_order,deriv=deriv,delta=dt[0])
 ax=filtered('ax');ay=filtered('ay');r=filtered('yaw_rate');lonjerk=filtered('ax',1)
 # Inertial jerk expressed in rotating body coordinates.
 jerk=np.hypot(lonjerk-r*ay,filtered('ay',1)+r*ax)
 ratios=np.array([np.where(ax>=0,ax/c.acceleration_limit_mps2,ax/-c.deceleration_limit_mps2),abs(ay)/c.lateral_acceleration_limit_mps2,jerk/c.inertial_jerk_limit_mps3,abs(r)/c.yaw_rate_limit_radps,abs(filtered('yaw_rate',1))/c.yaw_acceleration_limit_radps2,abs(lonjerk)/c.longitudinal_jerk_limit_mps3])
 out=aggregate_ratios(t,ratios,c);out.update(signal_source='executed dynamic ax, ay and yaw rate',config=asdict(c),filter_window_samples=n,filter_window_effective_s=(n-1)*float(dt[0]),evaluated_duration_s=float(t[-1]-t[0]));return out

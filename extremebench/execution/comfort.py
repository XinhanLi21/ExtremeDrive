"""Same severity limits, actual dynamic acceleration/rate (no bicycle inference)."""
import numpy as np
from scipy.signal import savgol_filter
from ..benchmark.comfort import aggregate_ratios

def dynamic_comfort(logs):
 t=np.array([r['t'] for r in logs]);dt=np.diff(t)
 if len(t)<7 or not np.allclose(dt,dt[0]):raise ValueError('at least 7 uniform samples required')
 n=max(5,int(round(.3/dt[0]))//2*2+1);n=min(n,len(t) if len(t)%2 else len(t)-1)
 def filtered(key,deriv=0):return savgol_filter([r[key] for r in logs],n,3,deriv=deriv,delta=dt[0])
 ax=filtered('ax');ay=filtered('ay');r=filtered('yaw_rate');lonjerk=filtered('ax',1)
 # Inertial jerk expressed in rotating body coordinates.
 jerk=np.hypot(lonjerk-r*ay,filtered('ay',1)+r*ax)
 ratios=np.array([np.where(ax>=0,ax/2.4,ax/-4.05),abs(ay)/4.89,jerk/8.37,abs(r)/.95,abs(filtered('yaw_rate',1))/1.93,abs(lonjerk)/4.13])
 out=aggregate_ratios(t,ratios);out['signal_source']='dynamic model ax ay yaw_rate; same limits as comfort v4; not official NAVSIM HC';return out

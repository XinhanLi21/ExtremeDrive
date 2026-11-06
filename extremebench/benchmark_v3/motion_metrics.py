"""State-derived future comfort and time-resolved adjacent-plan agreement.
Not NAVSIM HC/EC: no fabricated history dynamics, reference waiver, or tuning of
physical thresholds. Historical join comfort is unavailable without a verified
sensor-clock/acceleration history. Original binary diagnostics remain separate.
"""
from dataclasses import dataclass,asdict
import numpy as np
from scipy.signal import savgol_filter
from scipy.interpolate import CubicSpline
from ..benchmark.scoring import resample
from ..benchmark_v2.metrics import Settings

@dataclass(frozen=True)
class MetricConfig:
 version:str='motion-metrics-v3-proposal-1'
 state_filter_s:float=.30
 sampling_s:float=.05
 minimum_overlap_s:float=1.
 position_tolerance_m:float=.5
 yaw_rate_tolerance_radps:float=.2

def fraction(mask,t):
 return float(np.trapezoid(np.asarray(mask,float),t)/(t[-1]-t[0]))

def future_comfort(execution,config=None):
 c=config or MetricConfig();lim=Settings();u=execution['controls'];a=np.asarray(execution['trajectory'],float);t=np.array([x['t'] for x in u]);v=np.array([x['speed_mps'] for x in u]);steer=np.array([x['steering_rad'] for x in u]);dt=np.diff(t)
 if len(t)<7 or not np.isfinite(a).all() or not np.isfinite(v).all() or not np.isfinite(steer).all() or np.any(dt<=0) or not np.allclose(dt,dt[0]):raise ValueError('uniform finite native simulator states required')
 if len(a)!=len(t) or not np.allclose(a[:,0],t):raise ValueError('control/state clock mismatch')
 n=max(5,int(round(c.state_filter_s/dt[0]))//2*2+1);n=min(n,len(t) if len(t)%2 else len(t)-1)
 ax=savgol_filter(v,n,3,deriv=1,delta=dt[0]);dax=savgol_filter(v,n,3,deriv=2,delta=dt[0])
 # Actual kinematic yaw rate, from unchanged simulator state and steering.
 yr=v*np.tan(steer)/execution['config']['wheelbase_m'];yr=savgol_filter(yr,n,3);ya=savgol_filter(yr,n,3,deriv=1,delta=dt[0]);ay=v*yr;day=savgol_filter(ay,n,3,deriv=1,delta=dt[0]);jerk=np.hypot(dax-ay*yr,day+ax*yr)
 values=dict(longitudinal=ax,lateral=ay,jerk=jerk,yaw_rate=yr,yaw_acceleration=ya)
 masks=dict(longitudinal=(ax>=lim.long_accel_min)&(ax<=lim.long_accel_max),lateral=np.abs(ay)<=lim.lateral_accel_max,jerk=jerk<=lim.jerk_max,yaw_rate=np.abs(yr)<=lim.yaw_rate_max,yaw_acceleration=np.abs(ya)<=lim.yaw_accel_max)
 ok=np.logical_and.reduce(list(masks.values()));score=fraction(ok,t)
 return dict(score=score,strict_pass=int(ok.all()),duration_s=float(t[-1]-t[0]),component_time_fraction={k:fraction(x,t) for k,x in masks.items()},peak_values={k:float(np.max(np.abs(v))) for k,v in values.items()},longitudinal_min=float(ax.min()),longitudinal_max=float(ax.max()),signal_source='unchanged native simulator speed and steering, not position third derivatives',history_join_status='unavailable_unverified_sensor_clock',time_s=t.tolist(),compliant=ok.tolist(),signals={k:v.tolist() for k,v in values.items()},config=asdict(c))

def adjacent_agreement(first,second,offset=.5,config=None):
 c=config or MetricConfig()
 if second is None:return dict(score=None,status='unavailable',reason='adjacent_plan_missing')
 if offset<=0:raise ValueError('positive adjacent offset required')
 x=np.asarray(first,float);y=np.asarray(second,float).copy()
 for z in (x,y):
  if z.ndim!=2 or z.shape[1]!=4 or len(z)<4 or not np.isfinite(z).all() or np.any(np.diff(z[:,0])<=0):raise ValueError('invalid plan')
 y[:,0]+=offset;lo=max(x[0,0],y[0,0]);hi=min(x[-1,0],y[-1,0])
 if hi-lo<c.minimum_overlap_s:return dict(score=None,status='unavailable',reason='overlap_too_short')
 t=np.linspace(lo,hi,int(round((hi-lo)/c.sampling_s))+1);a=resample(x,t);b=resample(y,t);dist=np.linalg.norm(a[:,1:3]-b[:,1:3],axis=1)
 # Differentiate the native interpolant before resampling. Do not manufacture
 # corners by linearly oversampling a coarse position sequence and deriving it.
 sx=CubicSpline(x[:,0],x[:,1:3]);sy=CubicSpline(y[:,0],y[:,1:3]);acc=sx(t,2)-sy(t,2)
 # Use the SAME overlap support for the first heading derivative. Different
 # spline endpoint conditions must not make identical recorded overlap disagree.
 n=min(11,len(t) if len(t)%2 else len(t)-1);dt=t[1]-t[0]
 yawdiff=np.abs(savgol_filter(a[:,3],n,3,deriv=1,delta=dt)-savgol_filter(b[:,3],n,3,deriv=1,delta=dt))
 position_ok=dist<=c.position_tolerance_m;yaw_ok=yawdiff<=c.yaw_rate_tolerance_radps
 return dict(score=fraction(position_ok&yaw_ok,t),strict_geometric_pass=int((position_ok&yaw_ok).all()),position_time_fraction=fraction(position_ok,t),yaw_rate_time_fraction=fraction(yaw_ok,t),position_rms_m=float(np.sqrt(np.mean(dist**2))),acceleration_rms_mps2=float(np.sqrt(np.mean(np.sum(acc**2,axis=1)))),yaw_rate_rms_radps=float(np.sqrt(np.mean(yawdiff**2))),overlap_s=float(hi-lo),status='diagnostic_only_unconditioned_on_new_hazards',acceleration_role='diagnostic_not_aggregated_derivative_sensitive_and_overlaps_comfort',time_s=t.tolist(),position_difference_m=dist.tolist(),yaw_rate_difference_radps=yawdiff.tolist(),compliant=(position_ok&yaw_ok).tolist(),config=asdict(c))

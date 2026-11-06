"""Fixed discrete LQR lateral tracker and kinematic bicycle baseline (SI)."""
from dataclasses import dataclass,asdict
from functools import lru_cache
import math
import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.interpolate import PchipInterpolator,CubicSpline
from ..benchmark.scoring import trajectory_array

@dataclass(frozen=True)
class TrackerConfig:
    reference_interpolation:str="legacy_pchip"
    dt:float=.02
    output_dt:float=.05
    wheelbase_m:float=2.8
    steering_limit_rad:float=.55
    steering_rate_radps:float=.8
    acceleration_min:float=-6.
    acceleration_max:float=3.
    speed_gain:float=1.4
    position_gain:float=.3
    lateral_cost:float=2.
    heading_cost:float=3.
    steer_cost:float=5.
    status:str='fixed_generic_geometry_not_identified_vehicle'

@lru_cache(maxsize=512)
def gain(speed,dt,wheelbase,q1,q2,r):
    a=np.array([[1.,speed*dt],[0.,1.]])
    b=np.array([[.5*speed*speed*dt*dt/wheelbase],[speed*dt/wheelbase]])
    q=np.diag([q1,q2]);rr=np.array([[r]]);p=solve_discrete_are(a,b,q,rr)
    return np.linalg.solve(rr+b.T@p@b,b.T@p@a).ravel()

def rollout(rows,initial_speed,horizon=4.,config=None):
    c=config or TrackerConfig();a=trajectory_array(rows,horizon)
    if not np.isfinite(initial_speed) or initial_speed<0:raise ValueError('invalid initial speed')
    a=a.copy();a[:,3]=np.unwrap(a[:,3]);
    if c.reference_interpolation=='native_c2':
        # Interpolate original submitted knots exactly; C2 removes feed-forward
        # acceleration jumps. Initial tangent uses observed speed and heading.
        tangent=initial_speed*np.array([math.cos(a[0,3]),math.sin(a[0,3])])
        curves=[CubicSpline(a[:,0],a[:,i],bc_type=((1,tangent[i-1]),'natural')) for i in (1,2)]
        curves.append(CubicSpline(a[:,0],a[:,3],bc_type='natural'))
    elif c.reference_interpolation=='legacy_pchip':
        curves=[PchipInterpolator(a[:,0],a[:,i]) for i in (1,2,3)]
    else:raise ValueError('unknown reference interpolation')
    x,y,yaw=a[0,1:];v=float(initial_speed);delta=0.;states=[];controls=[]
    times=np.linspace(0,horizon,round(horizon/c.dt)+1)
    for t in times:
        states.append([float(t),float(x),float(y),float(yaw)])
        ref=np.array([f(t) for f in curves]);dxy=np.array([f.derivative()(t) for f in curves[:2]]);vr=float(np.linalg.norm(dxy));yr=float(curves[2].derivative()(t));yawerr=math.atan2(math.sin(yaw-ref[2]),math.cos(yaw-ref[2]));err=np.array([x,y])-ref[:2]
        lat=-math.sin(ref[2])*err[0]+math.cos(ref[2])*err[1];along=math.cos(ref[2])*err[0]+math.sin(ref[2])*err[1]
        k=gain(round(max(v,.5),1),c.dt,c.wheelbase_m,c.lateral_cost,c.heading_cost,c.steer_cost)
        desired=float(np.clip(math.atan(c.wheelbase_m*yr/max(vr,.5))-k@[lat,yawerr],-c.steering_limit_rad,c.steering_limit_rad))
        delta+=float(np.clip(desired-delta,-c.steering_rate_radps*c.dt,c.steering_rate_radps*c.dt))
        d2xy=np.array([f.derivative(2)(t) for f in curves[:2]]);feed=float(dxy@d2xy/max(vr,.5));acc=float(np.clip(feed+c.speed_gain*(vr-v)-c.position_gain*along,c.acceleration_min,c.acceleration_max))
        controls.append({'t':float(t),'steering_rad':delta,'acceleration_mps2':acc,'speed_mps':v})
        vnext=max(0,v+acc*c.dt);vmid=(v+vnext)/2;r=vmid*math.tan(delta)/c.wheelbase_m;ymid=yaw+r*c.dt/2;x+=vmid*math.cos(ymid)*c.dt;y+=vmid*math.sin(ymid)*c.dt;yaw+=r*c.dt;v=vnext
    return {'trajectory':states,'controls':controls,'config':asdict(c),'execution_status':'fixed_kinematic_LQR_rollout'}

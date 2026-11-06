"""Versioned signal calibration and domain checks; not global vehicle identification."""
from pathlib import Path
from dataclasses import dataclass
import hashlib,json
import numpy as np

@dataclass(frozen=True)
class SteeringCalibration:
    ratio: float
    offset_deg: float
    wheelbase_m: float
    sha256: str
    source: str
    @classmethod
    def load(cls,path):
        p=Path(path);b=p.read_bytes();d=json.loads(b)
        if d.get('version')!='mohe_restricted_candidate_v1':raise ValueError('unsupported calibration version')
        if set(d['fit_runs']) & {d['heldout_run']}:raise ValueError('calibration/test overlap')
        r=float(d['effective_steering_ratio']);o=float(d['steering_wheel_zero_offset_deg']);l=float(d['wheelbase_assumed_m'])
        if not np.isfinite([r,o,l]).all() or not 5<r<40 or not 2<l<4:raise ValueError('invalid calibration')
        return cls(r,o,l,hashlib.sha256(b).hexdigest(),str(p.resolve()))
    def front_angle(self,steering_wheel_deg):return np.deg2rad(np.asarray(steering_wheel_deg)-self.offset_deg)/self.ratio
    def metadata(self):return {'sha256':self.sha256,'source':self.source,'ratio':self.ratio,'offset_deg':self.offset_deg,'assumed_wheelbase_m':self.wheelbase_m,'status':'low-slip signal conversion candidate; not full vehicle calibration'}

def causal_initial_state(frames,start,max_gap):
    past=[f for f in frames if f['t']<=start+1e-8]
    if len(past)<2:raise ValueError('insufficient past poses')
    a,b=past[-2:];dt=b['t']-a['t']
    if a['key']!=b['key'] or not 0<dt<=max_gap or abs(b['t']-start)>1e-6:raise ValueError('missing contemporaneous pose or continuous history')
    world=np.subtract(b['xy'],a['xy'])/dt;yaw=b['yaw'];co,si=np.cos(yaw),np.sin(yaw)
    rate=(b['yaw']-a['yaw']+np.pi)%(2*np.pi)-np.pi
    return dict(x=b['xy'][0],y=b['xy'][1],yaw=yaw,vx=float(co*world[0]+si*world[1]),vy=float(-si*world[0]+co*world[1]),yaw_rate=float(rate/dt))

def calibration_domain(speed,beta,rate,accel,yaw_accel):
    # Matches the fitted offline mask. Never relabel drift as comfort-exempt by itself.
    return (np.asarray(speed)>2)&(np.asarray(speed)<10)&(np.abs(beta)<np.deg2rad(3))&(np.abs(rate)<.2)&(np.abs(accel)<.5)&(np.abs(yaw_accel)<.1)

"""SI units, x forward/y left, counterclockwise yaw, CG body reference.

Planar double-track body plus four rotating wheels. Static wheel loads and a
linear tire with a combined-force friction-circle cap: deliberately not a
validated extreme-handling tire/vehicle model. No ABS/ESC or suspension.
"""
from dataclasses import dataclass, asdict, fields
from collections import deque
import hashlib,json,math
import numpy as np
from scipy.interpolate import PchipInterpolator

WHEELS=('FL','FR','RL','RR')

def wrap(a): return (a+np.pi)%(2*np.pi)-np.pi

@dataclass(frozen=True)
class Config:
    mass: float=1800.
    yaw_inertia: float=3000.
    lf: float=1.2
    lr: float=1.6
    track: float=1.6
    length: float=4.7
    width: float=1.85
    wheel_radius: float=.31
    wheel_inertia: float=1.8
    corner_stiffness: float=40000.
    longitudinal_stiffness: float=65000.
    mu: float=.85
    air_drag: float=.42
    rolling_force: float=150.
    drive_front_share: float=1.
    brake_front_share: float=.6
    max_drive_torque: float=1800.  # per wheel, N m
    max_brake_torque: float=2200.
    max_steer: float=.55           # road-wheel rad, NOT steering wheel angle
    max_steer_rate: float=.8
    steer_tau: float=.12
    torque_tau: float=.08
    command_delay: float=.06
    control_dt: float=.02
    physics_dt: float=.001
    output_dt: float=.02
    speed_kp: float=1.4
    speed_ki: float=.18
    progress_kp: float=.3
    heading_gain: float=1.15
    lateral_gain: float=.8
    yaw_damping: float=.16
    min_accel: float=-6.
    max_accel: float=3.
    parameter_status: str='illustrative_unidentified'

    def validate(self):
        for f in fields(self):
            v=getattr(self,f.name)
            if isinstance(v,(int,float)) and not np.isfinite(v):raise ValueError(f'non-finite {f.name}')
        for k in ['mass','yaw_inertia','lf','lr','track','length','width','wheel_radius','wheel_inertia','corner_stiffness','longitudinal_stiffness','mu','max_drive_torque','max_brake_torque','max_steer','max_steer_rate','steer_tau','torque_tau','control_dt','physics_dt','output_dt']:
            if getattr(self,k)<=0:raise ValueError(f'{k} must be positive')
        for k in ['drive_front_share','brake_front_share']:
            if not 0<=getattr(self,k)<=1:raise ValueError(k)
        if self.command_delay<0 or self.min_accel>=0 or self.max_accel<=0:raise ValueError('invalid limits')
        for k in ['control_dt','output_dt']:
            ratio=getattr(self,k)/self.physics_dt
            if not np.isclose(ratio,round(ratio)):raise ValueError(f'{k} must be multiple of physics_dt')
        if self.physics_dt>.002:raise ValueError('physics_dt > 2 ms is unsupported for explicit wheel integration')
        if self.length<self.lf+self.lr or self.width<self.track:raise ValueError('body must contain axles')
        return self

    def identity(self):return hashlib.sha256(json.dumps(asdict(self),sort_keys=True).encode()).hexdigest()[:16]

class Trajectory:
    """Time parameterization is retained. No silent trajectory retiming."""
    def __init__(self, points):
        a=np.asarray(points,dtype=float)
        if a.ndim!=2 or a.shape[1]!=5 or len(a)<3:raise ValueError('trajectory requires >=3 [t_s,x_m,y_m,yaw_rad,speed_mps] rows')
        if not np.isfinite(a).all():raise ValueError('trajectory contains NaN/Inf')
        if abs(a[0,0])>1e-8 or np.any(np.diff(a[:,0])<=0):raise ValueError('time starts at 0 and is strictly increasing')
        if np.any(a[:,4]<0):raise ValueError('reverse is not supported in v0.1')
        a=a.copy();a[:,3]=np.unwrap(a[:,3]);self.points=a;self.end=float(a[-1,0])
        self.curves=[PchipInterpolator(a[:,0],a[:,i],extrapolate=False) for i in range(1,5)]
    def sample(self,t):
        if t < -1e-8 or t>self.end+1e-8:raise ValueError('trajectory horizon exhausted')
        t=np.clip(t,0,self.end);x,y,yaw,v=[float(c(t)) for c in self.curves]
        acc=float(self.curves[3].derivative()(t));k=float(self.curves[2].derivative()(t))/max(v,.3)
        return np.array([x,y,yaw,v,acc,k])

@dataclass
class Command:
    steer: float
    drive: np.ndarray
    brake: np.ndarray
    acceleration_request: float=0.
    limited: bool=False
    def vector(self):return np.r_[self.steer,self.drive,self.brake]
    @staticmethod
    def zero():return Command(0.,np.zeros(4),np.zeros(4))

class Tracker:
    """Fixed transparent baseline: curvature feedforward + heading/cross-track
    feedback, and speed PI with time-aligned along-track correction. No replanner.
    """
    def __init__(self,c):self.c=c;self.integral=0.
    def command(self,state,ref):
        c=self.c;x,y,yaw,vx,vy,r=state[:6];rx,ry,ryaw,rv,ra,k=ref
        dx=rx-x;dy=ry-y
        along=math.cos(ryaw)*dx+math.sin(ryaw)*dy
        lateral=-math.sin(yaw)*dx+math.cos(yaw)*dy
        raw_steer=math.atan((c.lf+c.lr)*k)+c.heading_gain*wrap(ryaw-yaw)+math.atan2(c.lateral_gain*lateral,max(vx,0)+2)+c.yaw_damping*(rv*k-r)
        steer=float(np.clip(raw_steer,-c.max_steer,c.max_steer))
        speed_error=rv-vx
        raw_a=ra+c.speed_kp*speed_error+c.speed_ki*self.integral+c.progress_kp*along
        a=float(np.clip(raw_a,c.min_accel,c.max_accel))
        if abs(a-raw_a)<1e-9 or (raw_a>c.max_accel and speed_error<0) or (raw_a<c.min_accel and speed_error>0):
            self.integral=float(np.clip(self.integral+speed_error*c.control_dt,-10,10))
        force=c.mass*a+c.air_drag*vx*abs(vx)+c.rolling_force*math.tanh(vx/.4)
        # Hold a standstill instead of integrating controller error into creep.
        if rv<.03 and abs(vx)<.10:force=-c.mass*.5
        def distribute(total,share):return total*np.array([share/2,share/2,(1-share)/2,(1-share)/2])
        drive=distribute(max(force,0)*c.wheel_radius,c.drive_front_share)
        brake=distribute(max(-force,0)*c.wheel_radius,c.brake_front_share)
        limited=abs(steer-raw_steer)>1e-9 or abs(a-raw_a)>1e-9 or np.any(drive>c.max_drive_torque) or np.any(brake>c.max_brake_torque)
        return Command(steer,np.clip(drive,0,c.max_drive_torque),np.clip(brake,0,c.max_brake_torque),a,bool(limited))

class Actuators:
    """Causal command delay, steering lag/rate/angle limits, torque lag/limits."""
    def __init__(self,c):self.c=c;self.pending=deque();self.target=np.zeros(9);self.actual=np.zeros(9)
    def submit(self,t,cmd):
        q=cmd.vector()
        if not np.isfinite(q).all() or np.any(q[1:]<0):raise ValueError('invalid actuator command')
        q[0]=np.clip(q[0],-self.c.max_steer,self.c.max_steer)
        q[1:5]=np.clip(q[1:5],0,self.c.max_drive_torque);q[5:]=np.clip(q[5:],0,self.c.max_brake_torque)
        self.pending.append((t+self.c.command_delay,q))
    def step(self,t,dt):
        while self.pending and self.pending[0][0]<=t+1e-10:self.target=self.pending.popleft()[1]
        c=self.c
        d=(self.target[0]-self.actual[0])*(1-math.exp(-dt/c.steer_tau))
        self.actual[0]+=np.clip(d,-c.max_steer_rate*dt,c.max_steer_rate*dt)
        self.actual[1:]+=(self.target[1:]-self.actual[1:])*(1-math.exp(-dt/c.torque_tau))
        return self.actual

class Vehicle:
    def __init__(self,c):
        self.c=c
        self.wx=np.array([c.lf,c.lf,-c.lr,-c.lr]);self.wy=np.array([c.track/2,-c.track/2,c.track/2,-c.track/2])
        self.fz=c.mass*9.81*np.array([c.lr,c.lr,c.lf,c.lf])/(2*(c.lf+c.lr))
    def forces(self,z,u):
        c=self.c;vx,vy,r=z[3:6];omega=np.maximum(z[6:10],0)
        angles=np.array([u[0],u[0],0,0]);co=np.cos(angles);si=np.sin(angles)
        bx=vx-r*self.wy;by=vy+r*self.wx
        wx=co*bx+si*by;wy=-si*bx+co*by
        slip=(c.wheel_radius*omega-wx)/np.maximum(np.abs(wx),2.)
        alpha=np.arctan2(wy,np.maximum(np.abs(wx),2.))
        fx=c.longitudinal_stiffness*slip;fy=-c.corner_stiffness*alpha
        demand=np.hypot(fx,fy);scale=np.minimum(1.,c.mu*self.fz/np.maximum(demand,1e-9));fx*=scale;fy*=scale
        return fx,fy,co*fx-si*fy,si*fx+co*fy,slip,alpha,demand/(c.mu*self.fz)
    def derivative(self,z,u):
        c=self.c;fx,fy,bfx,bfy,slip,alpha,util=self.forces(z,u);vx,vy,r=z[3:6]
        drag=c.air_drag*vx*abs(vx)+c.rolling_force*math.tanh(vx/.4)
        ax=(bfx.sum()-drag)/c.mass;ay=bfy.sum()/c.mass
        mz=np.sum(self.wx*bfy-self.wy*bfx)
        spin=(u[1:5]-u[5:]-c.wheel_radius*fx)/c.wheel_inertia
        # Static brake can hold a wheel at zero, never drive it backwards.
        spin=np.where((z[6:10]<=0)&(spin<0),0,spin)
        yaw=z[2]
        return np.r_[vx*math.cos(yaw)-vy*math.sin(yaw),vx*math.sin(yaw)+vy*math.cos(yaw),r,ax+r*vy,ay-r*vx,mz/c.yaw_inertia,spin]
    def step(self,z,u,dt):
        k1=self.derivative(z,u);k2=self.derivative(z+dt*k1/2,u);k3=self.derivative(z+dt*k2/2,u);k4=self.derivative(z+dt*k3,u)
        out=z+dt*(k1+2*k2+2*k3+k4)/6;out[6:10]=np.maximum(out[6:10],0)
        # Forward-only standstill contact approximation, disclosed in protocol.
        if np.hypot(out[3],out[4])<.06 and np.max(u[1:5])<1 and np.sum(u[5:])>20:
            out[3:]=0
        if not np.isfinite(out).all() or np.max(np.abs(out))>1e6:raise FloatingPointError('vehicle integration diverged')
        if out[3]<-.5:raise FloatingPointError('reverse body motion outside prototype validity')
        return out

def rollout(scene,c,*,tracker_cls=Tracker,vehicle_cls=Vehicle):
    c.validate();tr=Trajectory(scene['trajectory']);tracker=tracker_cls(c);act=Actuators(c);plant=vehicle_cls(c)
    first=tr.sample(0);initial=scene.get('initial_state',{})
    if 'front_steer_rad' in initial:
        delta=float(initial['front_steer_rad'])
        if not np.isfinite(delta) or abs(delta)>c.max_steer:raise ValueError('initial steering outside plant limits')
        act.actual[0]=delta;act.target[0]=delta
    z=np.array([initial.get('x',first[0]),initial.get('y',first[1]),initial.get('yaw',first[2]),initial.get('vx',first[3]),initial.get('vy',0.),initial.get('yaw_rate',0.),*initial.get('wheel_radps',[initial.get('vx',first[3])/c.wheel_radius]*4)],float)
    if z.shape!=(10,) or not np.isfinite(z).all():raise ValueError('invalid initial state')
    if z[3]<0 or np.any(z[6:]<0):raise ValueError('reverse initial state unsupported')
    if hasattr(tracker,'bind'):tracker.bind(tr,act)
    n=round(tr.end/c.physics_dt)
    if not np.isclose(n*c.physics_dt,tr.end):raise ValueError('horizon must be multiple of physics_dt')
    ctrl=round(c.control_dt/c.physics_dt);output=round(c.output_dt/c.physics_dt)
    logs=[];geometry=[];cmd=Command.zero();failure=None
    def record(t):
        ref=tr.sample(t);u=act.actual;der=plant.derivative(z,u);_,_,_,_,slip,alpha,util=plant.forces(z,u)
        logs.append({'t':float(t),'x':z[0],'y':z[1],'yaw':z[2],'vx':z[3],'vy':z[4],'yaw_rate':z[5],
          'ax':der[3]-z[5]*z[4],'ay':der[4]+z[5]*z[3], 'beta':math.atan2(z[4],max(z[3],.1)),
          'ref_x':ref[0],'ref_y':ref[1],'ref_yaw':ref[2],'ref_v':ref[3],
          'steer_request':cmd.steer,'steer_actual':u[0],'command_limited':int(cmd.limited),
          **{f'{k}_{w}':v[i] for k,v in [('omega',z[6:10]),('drive_request',cmd.drive),('brake_request',cmd.brake),('drive',u[1:5]),('brake',u[5:]),('slip',slip),('force_demand_ratio',util)] for i,w in enumerate(WHEELS)}})
    for i in range(n+1):
        t=i*c.physics_dt
        if i%ctrl==0 and i<n:cmd=tracker.command(z,tr.sample(t));act.submit(t,cmd)
        if i%output==0 or i==n:record(t)
        # Safety geometry sampled at every physics step, not only log frames.
        geometry.append([t,*z[:3]])
        if i==n:break
        u=act.step(t,c.physics_dt)
        try:z=plant.step(z,u,c.physics_dt)
        except FloatingPointError as e:failure=str(e);break
    return {'logs':logs,'geometry':np.asarray(geometry),'failure':failure,'config':asdict(c),'config_id':c.identity(),'scene_id':scene['id'],'reference':tr.points.tolist()}

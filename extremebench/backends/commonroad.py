"""Adapter for the unmodified CommonRoad 29-state multibody equations.

Native inputs are steering rate and acceleration demand; the latter is converted
INSIDE the upstream model into drive/brake torque, not imposed on body motion.
"""
from dataclasses import replace
import math
import numpy as np
from scipy.integrate import solve_ivp
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.init_mb import init_mb
from vehiclemodels.vehicle_dynamics_mb import vehicle_dynamics_mb
from vehiclemodels.utils.acceleration_constraints import acceleration_constraints
from ..core import Actuators,WHEELS

class CommonRoadBackend:
    name='commonroad_mb'
    def __init__(self,ref,config):
        self.p=parameters_vehicle2();p=self.p
        # Explicit peak-coefficient scaling, not a measured road/tire calibration.
        p.tire.p_dx1=config.mu;p.tire.p_dy1=config.mu
        self.config=replace(config,mass=p.m,yaw_inertia=p.I_z,lf=p.a,lr=p.b,track=max(p.T_f,p.T_r),length=p.l,width=p.w,
            wheel_radius=p.R_w,wheel_inertia=p.I_y_w,drive_front_share=p.T_se,brake_front_share=p.T_sb,
            max_steer=min(config.max_steer,p.steering.max),max_steer_rate=min(config.max_steer_rate,p.steering.v_max),
            air_drag=0.,rolling_force=0.,physics_dt=.002,parameter_status='commonroad_vehicle2_peak_mu_scaled_not_extremedrive_calibrated')
        self.z=np.asarray(init_mb([ref[0],ref[1],0.,ref[3],ref[2],0.,0.],p));self.act=Actuators(self.config)
        self.u=np.zeros(2);self.drive=np.zeros(4);self.brake=np.zeros(4);self.derivative=np.zeros(29);self.t=0.;self.low_speed_steps=0
        self.metadata={'engine':'commonroad-vehicle-models 3.0.2','vehicle':'vehicle2 / BMW 320i parameter set',
            'native_controls':'front steering rate rad/s; longitudinal acceleration demand mapped to torques internally',
            'tire':'upstream combined-slip Magic Formula; peak p_dx1 and p_dy1 set to requested mu',
            'actuator_adapter':'aggregate lagged wheel torque -> native acceleration demand, upstream front/rear split',
            'limitations':['no individual wheel torque override','explicit bicycle/standstill guard below 0.3 m/s to avoid upstream zero-speed singularity',
             'no contact/impact response; obstacle metrics external','no independent tire-force saturation measurement']}
    def state(self):
        z=self.z;p=self.p;d=self.derivative
        # Upstream wheel-plane velocity convention retained for native slip.
        uw=np.array([(z[3]+p.T_f*z[5]/2)*math.cos(z[2])+(z[10]+p.a*z[5])*math.sin(z[2]),
                     (z[3]-p.T_f*z[5]/2)*math.cos(z[2])+(z[10]+p.a*z[5])*math.sin(z[2]),z[3]+p.T_r*z[5]/2,z[3]-p.T_r*z[5]/2])
        slip=(p.R_w*z[23:27]-uw)/np.maximum(np.abs(uw),2.)
        return {'x':float(z[0]),'y':float(z[1]),'yaw':float(z[4]),'vx':float(z[3]),'vy':float(z[10]),'yaw_rate':float(z[5]),
                'steer_actual':float(z[2]),'ax':float(d[3]-z[5]*z[10]),'ay':float(d[10]+z[5]*z[3]),'roll':float(z[6]),'pitch':float(z[8]),
                'omega':z[23:27].tolist(),'slip':slip.tolist(),'drive':self.drive.tolist(),'brake':self.brake.tolist(),
                'force_demand_ratio':[None]*4}
    def command(self,t,cmd):self.act.submit(t,cmd)
    def advance(self,t,dt):
        p=self.p;actual=self.act.step(t,dt)
        steer_rate=float(np.clip((actual[0]-self.z[2])/dt,-self.config.max_steer_rate,self.config.max_steer_rate))
        a=float((actual[1:5].sum()-actual[5:].sum())/(p.m*p.R_w));a=acceleration_constraints(self.z[3],a,p.longitudinal)
        # Match native torque split exactly for logging; not a measured CAN signal.
        total=p.m*p.R_w*a
        self.drive=max(total,0)*np.array([p.T_se,p.T_se,1-p.T_se,1-p.T_se])/2
        self.brake=max(-total,0)*np.array([p.T_sb,p.T_sb,1-p.T_sb,1-p.T_sb])/2
        self.u=np.array([steer_rate,a])
        if abs(self.z[3])<.3:
            # A disclosed hybrid guard, not a change to upstream equations.
            # Keep wheel speed consistent when stopping/restarting at zero.
            self.low_speed_steps+=1
            old=self.z.copy();v=max(0.,self.z[3]+a*dt);vm=(v+self.z[3])/2
            delta=float(np.clip(self.z[2]+steer_rate*dt,-self.config.max_steer,self.config.max_steer))
            yaw_rate=vm*math.tan(delta)/(p.a+p.b)
            self.z[0]+=vm*math.cos(self.z[4]+yaw_rate*dt/2)*dt
            self.z[1]+=vm*math.sin(self.z[4]+yaw_rate*dt/2)*dt
            self.z[2]=delta;self.z[3]=v;self.z[4]+=yaw_rate*dt;self.z[5]=yaw_rate;self.z[10]=0.;self.z[23:27]=v/p.R_w
            self.derivative=(self.z-old)/dt;self.metadata['low_speed_guard_steps']=self.low_speed_steps
            self.t=t+dt;return
        def rhs(_,z):return np.asarray(vehicle_dynamics_mb(z.copy(),self.u,self.p))
        out=solve_ivp(rhs,(t,t+dt),self.z,method='RK45',rtol=1e-7,atol=1e-9,max_step=.001)
        if not out.success:raise FloatingPointError(out.message)
        self.z=out.y[:,-1];self.z[23:27]=np.maximum(self.z[23:27],0);self.derivative=rhs(t+dt,self.z)
        if not np.isfinite(self.z).all() or abs(self.z[6])>1.2:raise FloatingPointError('multibody state outside trial validity')
        self.t=t+dt
    def close(self):pass

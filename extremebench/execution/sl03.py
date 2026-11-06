"""Unidentified SL03-sized planar 7-DOF plant, not an OEM validated model.
3 body DOF + 4 wheel spins; quasi-static load transfer, nonlinear combined slip.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.linalg import solve_discrete_are
from ..core import Config, Vehicle, Tracker, wrap

@dataclass(frozen=True)
class SL03Config(Config):
    length: float=4.82
    width: float=1.89
    lf: float=1.45
    lr: float=1.45
    track: float=1.62
    drive_front_share: float=0.
    front_corner_stiffness: float=40000. # N/rad per wheel
    rear_corner_stiffness: float=40000.
    cg_height: float=.52
    front_roll_share: float=.5
    control_dt: float=.1
    output_dt: float=.1
    parameter_status: str='SL03_dimensions_RWD_assumption_other_parameters_unidentified'
    def validate(self):
        super().validate()
        if self.front_corner_stiffness<=0 or self.rear_corner_stiffness<=0:raise ValueError('positive tire stiffness required')
        if not 0<self.cg_height<2 or not 0<=self.front_roll_share<=1:raise ValueError('invalid load transfer parameters')
        return self

class LQRTracker(Tracker):
    """Own discrete lateral LQR + longitudinal PI; NOT exact NAVSIM LQR.
    Linearized kinematic lateral error model with curvature feedforward.
    """
    def __init__(self,c):super().__init__(c);self.cache={}
    def command(self,state,ref):
        cmd=super().command(state,ref);c=self.c
        v=max(abs(float(state[3])),.5);key=round(v,1)
        if key not in self.cache:
            A=np.array([[1,key*c.control_dt],[0,1.]])
            B=np.array([[0],[key*c.control_dt/(c.lf+c.lr)]])
            Q=np.diag([2.,4.]);R=np.array([[3.]])
            P=solve_discrete_are(A,B,Q,R)
            self.cache[key]=np.linalg.solve(R+B.T@P@B,B.T@P@A)
        dx=state[0]-ref[0];dy=state[1]-ref[1]
        e=np.array([-math.sin(ref[2])*dx+math.cos(ref[2])*dy,wrap(state[2]-ref[2])])
        raw=math.atan((c.lf+c.lr)*ref[5])-float((self.cache[key]@e)[0])
        cmd.steer=float(np.clip(raw,-c.max_steer,c.max_steer));cmd.limited |= abs(raw-cmd.steer)>1e-8
        return cmd

class LoadTransferVehicle(Vehicle):
    def tire_forces(self,z,u):
        c=self.c;vx,vy,r=z[3:6]
        # Use applied wheel torque for longitudinal transfer; vy/r for lateral.
        ax=np.clip((sum(u[1:5])-sum(u[5:]))/(c.wheel_radius*c.mass),-c.mu*9.81,c.mu*9.81)
        ay=np.clip(vx*r,-c.mu*9.81,c.mu*9.81)
        front=c.mass*9.81*c.lr/(c.lf+c.lr)-c.mass*ax*c.cg_height/(c.lf+c.lr)
        front=np.clip(front,0,c.mass*9.81);rear=c.mass*9.81-front
        transfer=c.mass*ay*c.cg_height/c.track
        df=np.clip(transfer*c.front_roll_share,-front/2,front/2)
        dr=np.clip(transfer*(1-c.front_roll_share),-rear/2,rear/2)
        fz=np.array([front/2-df,front/2+df,rear/2-dr,rear/2+dr])
        # Ackermann split from virtual bicycle front steering angle.
        delta=u[0];angles=np.zeros(4)
        if abs(delta)>1e-8:
            radius=(c.lf+c.lr)/math.tan(delta)
            angles[:2]=np.arctan((c.lf+c.lr)/(radius-self.wy[:2]))
        co=np.cos(angles);si=np.sin(angles)
        bx=vx-r*self.wy;by=vy+r*self.wx
        wx=co*bx+si*by;wy=-si*bx+co*by
        slip=(c.wheel_radius*np.maximum(z[6:10],0)-wx)/np.maximum(abs(wx),2.)
        alpha=np.arctan2(wy,np.maximum(abs(wx),2.))
        fx=c.longitudinal_stiffness*slip;fy=-np.array([c.front_corner_stiffness]*2+[c.rear_corner_stiffness]*2)*alpha
        demand=np.hypot(fx,fy);capacity=c.mu*fz
        # Smooth radial saturation, retaining a combined force bound.
        ratio=demand/np.maximum(capacity,1e-9)
        scale=capacity*np.tanh(ratio)/np.maximum(demand,1e-9)
        fx*=scale;fy*=scale
        return (fx,fy,co*fx-si*fy,si*fx+co*fy,slip,alpha,ratio),fz
    def forces(self,z,u):return self.tire_forces(z,u)[0]

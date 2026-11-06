"""Unmodified pinned NAVSIM LQR feedback from four-wheel plant.
Only the actuator allocation is ours; acceleration is never imposed on body state.
"""
from pathlib import Path
import sys,subprocess,hashlib,math
import numpy as np
from ..core import Command,rollout
from .sl03 import SL03Config,LoadTransferVehicle
from .navsim_reference import PIN
def upstream():
    import json
    root=Path(__file__).resolve().parents[1]/'_vendor'
    manifest=json.loads((root/'manifest.json').read_text())
    for name,expected in manifest['packaged_sha256'].items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest()!=expected:
            raise ValueError('LQR dependency checksum mismatch: '+name)
    return manifest['original_sha256']

class OfficialTracker:
    def __init__(self,c):
        from extremebench._vendor.navsim.planning.simulation.planner.pdm_planner.simulation.batch_lqr import BatchLQRTracker
        from extremebench._vendor.nuplan.common.actor_state.vehicle_parameters import VehicleParameters
        self.c=c;self.step=0;self.raw=[]
        vehicle=VehicleParameters(c.width,c.length/2+c.lr,c.length/2-c.lr,c.lr,c.lf+c.lr,'SL03_approx','sedan')
        self.controller=BatchLQRTracker(vehicle=vehicle,discretization_time=c.control_dt)
    def bind(self,tr,act):
        from extremebench._vendor.navsim.planning.simulation.planner.pdm_planner.utils.pdm_enums import StateIndex
        self.act=act;self.tr=tr;n=round(tr.end/self.c.control_dt)
        if not np.isclose(n*self.c.control_dt,tr.end):raise ValueError('horizon must align to controller clock')
        poses=np.array([tr.sample(i*self.c.control_dt)[:3] for i in range(n+1)])
        poses[:,0]-=self.c.lr*np.cos(poses[:,2]);poses[:,1]-=self.c.lr*np.sin(poses[:,2])
        states=np.zeros((1,n+1,StateIndex.size()));states[0,:,:3]=poses;self.controller.update(states)
    def command(self,z,ref):
        from extremebench._vendor.navsim.planning.simulation.planner.pdm_planner.utils.pdm_enums import StateIndex as SI,DynamicStateIndex as DI
        from extremebench._vendor.nuplan.planning.simulation.simulation_time_controller.simulation_iteration import SimulationIteration
        from extremebench._vendor.nuplan.common.actor_state.state_representation import TimePoint
        c=self.c;s=np.zeros((1,SI.size()));yaw=z[2]
        s[0,SI.X]=z[0]-c.lr*math.cos(yaw);s[0,SI.Y]=z[1]-c.lr*math.sin(yaw);s[0,SI.HEADING]=yaw
        s[0,SI.VELOCITY_X]=z[3];s[0,SI.VELOCITY_Y]=z[4]-c.lr*z[5]
        s[0,SI.STEERING_ANGLE]=self.act.actual[0];s[0,SI.ANGULAR_VELOCITY]=z[5]
        current=SimulationIteration(TimePoint(round(self.step*c.control_dt*1e6)),self.step)
        nxt=SimulationIteration(TimePoint(round((self.step+1)*c.control_dt*1e6)),self.step+1)
        raw=self.controller.track_trajectory(current,nxt,s)[0];self.step+=1
        a0=float(raw[DI.ACCELERATION_X]);sr=float(raw[DI.STEERING_RATE])
        a=float(np.clip(a0,c.min_accel,c.max_accel));rate=float(np.clip(sr,-c.max_steer_rate,c.max_steer_rate))
        # Rate request converted to target relative to ACTUAL steering feedback.
        delta=float(np.clip(self.act.actual[0]+rate*c.control_dt,-c.max_steer,c.max_steer))
        force=c.mass*a+c.air_drag*z[3]*abs(z[3])+c.rolling_force*math.tanh(z[3]/.4)
        def distribute(total,share):return total*np.array([share,share,1-share,1-share])/2
        drive=distribute(max(force,0)*c.wheel_radius,c.drive_front_share)
        brake=distribute(max(-force,0)*c.wheel_radius,c.brake_front_share)
        limited=abs(a-a0)>1e-9 or abs(sr-rate)>1e-9 or abs(delta-(self.act.actual[0]+rate*c.control_dt))>1e-9 or np.any(drive>c.max_drive_torque) or np.any(brake>c.max_brake_torque)
        self.raw.append({'t':current.time_point.time_s,'acceleration_request_mps2':a0,'steering_rate_request_rad_s':sr,'rear_axle_state':s[0].tolist(),'steering_target_rad':delta})
        return Command(delta,np.clip(drive,0,c.max_drive_torque),np.clip(brake,0,c.max_brake_torque),a,bool(limited))

def execute_official_dynamic(scene,c=None):
    hashes=upstream();c=c or SL03Config();holder=[]
    def factory(config):
        tracker=OfficialTracker(config);holder.append(tracker);return tracker
    result=rollout(scene,c,tracker_cls=factory,vehicle_cls=LoadTransferVehicle)
    result['official_lqr_commands']=holder[0].raw
    result['provenance']={'upstream_commit':PIN,'source_sha256':hashes,'controller':'unmodified NAVSIM BatchLQRTracker','plant':'experimental SL03-sized 7DOF four wheel','allocation':'acceleration -> total wheel force with drag compensation -> drive/brake torque; steering rate -> actual-angle-relative target; actuator limits/lag/delay retained','feedback':'actual plant CG converted to rear axle, actual steering and longitudinal velocity; no custom PI or lateral correction','scope':'not official NAVSIM leaderboard; physical parameters unvalidated'}
    result['execution_status']='failed' if result['failure'] else 'official_navsim_lqr_experimental_plant'
    return result

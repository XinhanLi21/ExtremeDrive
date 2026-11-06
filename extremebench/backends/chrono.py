"""Actual PyChrono Sedan + TMeasy tires + rigid terrain, headless CPU.

Steering/drive/brake commands are adapted to the native normalized DriverInputs.
The throttle map is an explicitly approximate adapter, not direct wheel torque.
"""
from dataclasses import replace
import math
import numpy as np
import pychrono as ch
import pychrono.vehicle as veh
from ..core import Actuators

class ChronoBackend:
    name='chrono'
    def __init__(self,ref,config):
        veh.SetVehicleDataPath(ch.GetChronoDataPath()+'vehicle/')
        self.model=veh.Sedan();a=self.model
        a.SetContactMethod(ch.ChContactMethod_SMC);a.SetChassisCollisionType(veh.CollisionType_NONE)
        a.SetChassisFixed(False);a.SetInitPosition(ch.ChCoordsysd(ch.ChVector3d(0,0,.7),ch.QUNIT))
        a.SetInitFwdVel(float(ref[3]));a.SetTireType(veh.TireModelType_TMEASY);a.SetTireStepSize(.001);a.Initialize()
        self.vehicle=a.GetVehicle();self.body=a.GetChassisBody();self.system=a.GetSystem();self.system.SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)
        material=ch.ChContactMaterialSMC();material.SetFriction(float(config.mu));material.SetRestitution(.01);material.SetYoungModulus(2e7)
        self.terrain=veh.RigidTerrain(self.system);self.terrain.AddPatch(material,ch.CSYSNORM,1000.,200.);self.terrain.Initialize()
        self.inputs=veh.DriverInputs();self.inputs.m_steering=0.;self.inputs.m_throttle=0.;self.inputs.m_braking=0.
        self.native_lf=self.vehicle.GetSpindlePos(0,veh.LEFT).x-self.body.GetPos().x
        self.native_track=float(self.vehicle.GetWheeltrack(0))
        self.nominal_radius=float(self.vehicle.GetTire(0,veh.LEFT).GetRadius())
        # Settle suspension on the road while coasting. Then align evaluation
        # origin and initialize the requested speed, without teleporting later.
        for _ in range(1000):
            tm=self.system.GetChTime();self.terrain.Synchronize(tm);a.Synchronize(tm,self.inputs,self.terrain);self.terrain.Advance(.001);a.Advance(.001)
        self.wheels=[(0,veh.LEFT),(0,veh.RIGHT),(1,veh.LEFT),(1,veh.RIGHT)]
        radius=self.nominal_radius
        for body in self.system.GetBodies():
            if not body.IsFixed():body.SetPosDt(ch.ChVector3d(float(ref[3]),0,0))
        for axle,side in self.wheels:
            spindle=self.vehicle.GetWheel(axle,side).GetSpindle();spindle.SetAngVelLocal(ch.ChVector3d(0,float(ref[3])/radius,0))
        self.origin=np.array([self.body.GetPos().x,self.body.GetPos().y]);self.ref_origin=np.asarray(ref[:2]);self.heading_offset=float(ref[2]);self.base_yaw=self._native_yaw()
        wb=self.vehicle.GetWheelbase();front=self.native_lf
        lf=float(front);lr=float(wb-lf)
        self.config=replace(config,mass=float(self.vehicle.GetMass()),yaw_inertia=float(self.body.GetInertiaXX().z),lf=lf,lr=lr,
            track=self.native_track,length=4.7,width=1.9,wheel_radius=float(radius),physics_dt=.001,
            max_steer=float(self.vehicle.GetMaxSteeringAngle()),parameter_status='chrono_sedan_native_model_not_extremedrive_calibrated')
        self.act=Actuators(self.config);self.evaluation_started=False
        self.metadata={'engine':'Project Chrono / PyChrono 9.0.1','vehicle':'Sedan built-in model','tire':'TMeasy','terrain_friction':config.mu,
          'native_controls':'normalized steering, throttle and braking','adapter':'steer/max_steer; throttle=sum(drive_torque)/2500; brake=sum(brake_torque)/8000; clamped to native limits',
          'torque_channels':'measured model driveline spindle and brake torques; differ from requested torques',
          'initialization':'1 s suspension settling, then requested body/wheel speed and evaluation origin set once',
          'limitations':['throttle mapping is approximate, not a calibrated inverse powertrain','4.7x1.9 m evaluation footprint is an approximation',
                        'not matched to other backend vehicle parameters','obstacles evaluated externally, no physical impact actors']}
    def _native_yaw(self):
        axis=self.body.GetRot().Rotate(ch.ChVector3d(1,0,0));return math.atan2(axis.y,axis.x)
    def state(self):
        body=self.body;v=body.GetRot().RotateBack(body.GetPosDt());acc=body.GetRot().RotateBack(body.GetPosDt2());om=body.GetAngVelLocal()
        p=body.GetPos();offset=np.array([p.x,p.y])-self.origin;co=math.cos(self.heading_offset-self.base_yaw);si=math.sin(self.heading_offset-self.base_yaw)
        xy=np.array([[co,-si],[si,co]])@offset+self.ref_origin
        rot=body.GetRot();fwd=rot.Rotate(ch.ChVector3d(1,0,0));up=rot.Rotate(ch.ChVector3d(0,0,1))
        states=[self.vehicle.GetWheel(a,s).GetState() for a,s in self.wheels]
        tires=[self.vehicle.GetTire(a,s) for a,s in self.wheels]
        return {'x':float(xy[0]),'y':float(xy[1]),'yaw':self._native_yaw()-self.base_yaw+self.heading_offset,
           'vx':v.x,'vy':v.y,'yaw_rate':om.z,'ax':acc.x if self.evaluation_started else None,'ay':acc.y if self.evaluation_started else None,'roll':math.atan2(-up.y,up.z),'pitch':math.asin(np.clip(-fwd.z,-1,1)),
           'steer_actual':sum(self.vehicle.GetSteeringAngle(0,s) for s in [veh.LEFT,veh.RIGHT])/2,
           'omega':[s.omega for s in states],'slip':[t.GetLongitudinalSlip() for t in tires] if self.evaluation_started else [None]*4,
           'drive':[self.vehicle.GetDriveline().GetSpindleTorque(a,s) for a,s in self.wheels],
           'brake':[abs(self.vehicle.GetBrake(a,s).GetBrakeTorque()) for a,s in self.wheels],
           'force_demand_ratio':[None]*4,'native_throttle':self.inputs.m_throttle,'native_brake':self.inputs.m_braking,'native_steer':self.inputs.m_steering}
    def command(self,t,cmd):self.act.submit(t,cmd)
    def advance(self,t,dt):
        u=self.act.step(t,dt);self.inputs.m_steering=float(np.clip(u[0]/self.config.max_steer,-1,1))
        net=float(u[1:5].sum()-u[5:].sum())
        self.inputs.m_throttle=float(np.clip(net/2500,0,1));self.inputs.m_braking=float(np.clip(-net/8000,0,1))
        tm=self.system.GetChTime();self.terrain.Synchronize(tm);self.model.Synchronize(tm,self.inputs,self.terrain);self.terrain.Advance(dt);self.model.Advance(dt);self.evaluation_started=True
        if abs(self.body.GetPos().z)>5:raise FloatingPointError('Chrono vehicle left terrain')
    def close(self):pass

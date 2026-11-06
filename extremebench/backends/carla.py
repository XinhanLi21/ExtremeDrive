"""CARLA 0.9.16 / PhysX backend on a dedicated synchronous server.

Uses an isolated generated road and native steering/throttle/brake. It does not
pretend to reconstruct the recorded ExtremeDrive world or to apply wheel torque
directly. The server port is unrelated to the existing 8770 report service.
"""
from dataclasses import replace
import math
import numpy as np
import carla
from ..core import Actuators

ROAD='''<?xml version="1.0" standalone="yes"?><OpenDRIVE><header revMajor="1" revMinor="4" name="ExtremeBenchFlatTrial" version="1.00" date="2026-10-07" north="0" south="0" east="0" west="0"/>
<road name="flat_trial" length="400" id="0" junction="-1"><link/><type s="0" type="town"><speed max="50" unit="m/s"/></type>
<planView><geometry s="0" x="0" y="0" hdg="0" length="400"><line/></geometry></planView><elevationProfile><elevation s="0" a="0" b="0" c="0" d="0"/></elevationProfile><lateralProfile/>
<lanes><laneSection s="0"><center><lane id="0" type="none" level="false"><roadMark sOffset="0" type="none" weight="standard" color="standard" width="0"/></lane></center>
<right><lane id="-1" type="driving" level="false"><link/><width sOffset="0" a="30" b="0" c="0" d="0"/><roadMark sOffset="0" type="none" weight="standard" color="standard" width="0"/></lane></right></laneSection></lanes></road></OpenDRIVE>'''

class CarlaBackend:
    name='carla'
    def __init__(self,ref,config,host='127.0.0.1',port=28700):
        self.client=carla.Client(host,port);self.client.set_timeout(90.)
        self.world=self.client.generate_opendrive_world(ROAD,carla.OpendriveGenerationParameters(vertex_distance=2.,max_road_length=50.,wall_height=0.,additional_width=2.,smooth_junctions=True,enable_mesh_visibility=True))
        settings=self.world.get_settings();settings.synchronous_mode=True;settings.fixed_delta_seconds=.01;settings.substepping=True;settings.max_substep_delta_time=.002;settings.max_substeps=5;settings.no_rendering_mode=True;self.world.apply_settings(settings)
        self.vehicle=None;self.physics=None;self.snapshot_valid=False
        try:
            bp=self.world.get_blueprint_library().find('vehicle.tesla.model3');bp.set_attribute('role_name','extremebench_trial')
            self.vehicle=self.world.spawn_actor(bp,carla.Transform(carla.Location(x=20,y=15,z=1.0),carla.Rotation(yaw=0)))
            physics=self.vehicle.get_physics_control();wheels=physics.wheels
            for w in wheels:w.tire_friction=float(config.mu)
            physics.wheels=wheels;physics.steering_curve=[carla.Vector2D(0,1),carla.Vector2D(100,1)]
            self.vehicle.apply_physics_control(physics);self.physics=physics;self.control=carla.VehicleControl()
            for _ in range(100):self.world.tick()
            # A one-time velocity initialization followed by coasting lets wheel
            # states respond; no velocity override or teleport in the rollout.
            self.vehicle.set_target_velocity(carla.Vector3D(x=float(ref[3]),y=0,z=0))
            for _ in range(50):self.world.tick()
            loc=self.vehicle.get_location();self.origin=np.array([loc.x,loc.y]);self.ref_origin=np.asarray(ref[:2]);self.heading_offset=float(ref[2]);self.native_initial_yaw=math.radians(self.vehicle.get_transform().rotation.yaw)
            box=self.vehicle.bounding_box;self.native_max_steer=math.radians(max(w.max_steer_angle for w in wheels));self.brake_capacity=sum(w.max_brake_torque for w in wheels)
            # Wheel positions in physics payload are in cm in this server API.
            front=(wheels[0].position.x+wheels[1].position.x)/200.;rear=(wheels[2].position.x+wheels[3].position.x)/200.
            wheelbase=abs(front-rear)
            if not 1.5<wheelbase<5.:wheelbase=2.875
            self.config=replace(config,mass=physics.mass,lf=wheelbase*.5,lr=wheelbase*.5,track=1.58,length=2*box.extent.x,width=2*box.extent.y,
                wheel_radius=wheels[0].radius/100.,max_steer=min(config.max_steer,self.native_max_steer),physics_dt=.01,control_dt=.02,output_dt=.02,
                parameter_status='carla_model3_native_physics_not_extremedrive_calibrated')
            self.act=Actuators(self.config);self.metadata={'engine':'CARLA '+self.client.get_server_version(),'vehicle':'vehicle.tesla.model3',
                'native_controls':'steer [-1,1], throttle [0,1], brake [0,1]',
                'adapter':'canonical rad sign-flipped for CARLA; throttle positive net torque / 3500; braking negative net torque / sum(native max brake torque)',
                'frame':'CARLA x forward/y right converted to benchmark x forward/y left; yaw and steering signs flipped',
                'terrain':'generated flat 400 m road, not an ExtremeDrive digital twin','friction':'native tire_friction scalar set to requested value; not identical across tire models',
                'limitations':['approximate inverse throttle mapping','CG/reference-point and axle split not calibrated to dataset',
                    'no separate observed drive/brake torque via supported telemetry; logged unavailable',
                    'scenario obstacles scored geometrically, not spawned as impact actors','no camera re-rendering or planner replanning in this rollout']}
        except Exception:
            if self.vehicle is not None:self.vehicle.destroy()
            raise
    def state(self):
        tr=self.vehicle.get_transform();p=tr.location;native_yaw=math.radians(tr.rotation.yaw);v=self.vehicle.get_velocity();acc=self.vehicle.get_acceleration();r=self.vehicle.get_angular_velocity()
        co=math.cos(native_yaw);si=math.sin(native_yaw);vx=co*v.x+si*v.y;vy=si*v.x-co*v.y
        native_offset=np.array([p.x,p.y])-self.origin;theta=self.native_initial_yaw
        local=np.array([[math.cos(theta),math.sin(theta)],[math.sin(theta),-math.cos(theta)]])@native_offset
        co0=math.cos(self.heading_offset);si0=math.sin(self.heading_offset);xy=np.array([[co0,-si0],[si0,co0]])@local+self.ref_origin
        try:
            telemetry=self.vehicle.get_telemetry_data();wheels=telemetry.wheels
            omega=[float(w.omega) if math.isfinite(w.omega) else None for w in wheels];slip=[float(w.long_slip) if math.isfinite(w.long_slip) else None for w in wheels]
            if len(omega)!=4:omega=[None]*4;slip=[None]*4
        except RuntimeError:omega=[None]*4;slip=[None]*4
        steer=-math.radians(sum(self.vehicle.get_wheel_steer_angle(w) for w in [carla.VehicleWheelLocation.FL_Wheel,carla.VehicleWheelLocation.FR_Wheel])/2)
        return {'x':float(xy[0]),'y':float(xy[1]),'yaw':-(native_yaw-self.native_initial_yaw)+self.heading_offset,
          'vx':vx,'vy':vy,'yaw_rate':-math.radians(r.z),'steer_actual':steer,
          'ax':co*acc.x+si*acc.y,'ay':si*acc.x-co*acc.y,'roll':-math.radians(tr.rotation.roll),'pitch':math.radians(tr.rotation.pitch),
          'omega':omega,'slip':slip,'drive':[None]*4,'brake':[None]*4,'force_demand_ratio':[None]*4,
          'native_throttle':float(self.control.throttle),'native_brake':float(self.control.brake),'native_steer':float(self.control.steer)}
    def command(self,t,cmd):self.act.submit(t,cmd)
    def advance(self,t,dt):
        u=self.act.step(t,dt);net=float(u[1:5].sum()-u[5:].sum())
        self.control=carla.VehicleControl(steer=float(np.clip(-u[0]/self.native_max_steer,-1,1)),throttle=float(np.clip(net/3500,0,1)),brake=float(np.clip(-net/self.brake_capacity,0,1)))
        self.vehicle.apply_control(self.control);self.world.tick()
        if abs(self.vehicle.get_location().z)>5:raise FloatingPointError('CARLA vehicle left road surface')
    def close(self):
        if self.vehicle is not None:
            self.vehicle.destroy();self.vehicle=None

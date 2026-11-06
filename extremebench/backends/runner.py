from dataclasses import asdict
import numpy as np
from ..core import Trajectory,Tracker,WHEELS,Command

def run_backend(scene,config,name,**options):
    tr=Trajectory(scene['trajectory'])
    if name=='commonroad_mb':
        from .commonroad import CommonRoadBackend
        backend=CommonRoadBackend(tr.sample(0),config)
    elif name=='chrono':
        from .chrono import ChronoBackend
        backend=ChronoBackend(tr.sample(0),config)
    elif name=='carla':
        from .carla import CarlaBackend
        backend=CarlaBackend(tr.sample(0),config,**options)
    else:raise ValueError('unknown backend '+name)
    c=backend.config;tracker=Tracker(c);logs=[];geo=[];failure=None;cmd=Command.zero()
    dt=c.physics_dt;n=round(tr.end/dt);control=round(c.control_dt/dt);output=round(c.output_dt/dt)
    if not np.isclose(n*dt,tr.end):backend.close();raise ValueError('horizon must match backend timestep')
    try:
        for i in range(n+1):
            t=i*dt;s=backend.state();ref=tr.sample(t)
            vec=np.array([s[k] for k in ['x','y','yaw','vx','vy','yaw_rate']]+[0.]*4)
            if not np.isfinite(vec[:6]).all():failure='nonfinite backend state';break
            if i%control==0:cmd=tracker.command(vec,ref);backend.command(t,cmd)
            geo.append([t,s['x'],s['y'],s['yaw']])
            if i%output==0 or i==n:
                row={k:float(s[k]) if s.get(k) is not None else None for k in ['x','y','yaw','vx','vy','yaw_rate','steer_actual','ax','ay','roll','pitch']}
                row.update(t=t,beta=float(np.arctan2(s['vy'],max(s['vx'],.1))),ref_x=ref[0],ref_y=ref[1],ref_yaw=ref[2],ref_v=ref[3],
                    steer_request=cmd.steer,command_limited=int(cmd.limited))
                for k,vals in [('drive_request',cmd.drive),('brake_request',cmd.brake)]+[(k,s.get(k,[None]*4)) for k in ['drive','brake','omega','slip','force_demand_ratio']]:
                    row.update({k+'_'+w:float(v) if v is not None else None for w,v in zip(WHEELS,vals)})
                row.update({k:s[k] for k in ['native_throttle','native_brake','native_steer'] if k in s})
                logs.append(row)
            if i==n:break
            try:backend.advance(t,dt)
            except Exception as e:failure=f'{type(e).__name__}: {e}';break
        return {'logs':logs,'geometry':np.asarray(geo),'failure':failure,'config':asdict(c),'config_id':c.identity(),
                'scene_id':scene['id'],'reference':tr.points.tolist(),'backend':name,'backend_metadata':backend.metadata}
    finally:backend.close()

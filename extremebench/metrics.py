"""Component metrics on the EXECUTED rollout. No invented aggregate score."""
import math
import numpy as np
from shapely.geometry import Polygon
from shapely import intersects,covers,distance,polygons

def corners(x,y,yaw,length,width):
    q=np.array([[length/2,width/2],[length/2,-width/2],[-length/2,-width/2],[-length/2,width/2]])
    return q@np.array([[math.cos(yaw),math.sin(yaw)],[-math.sin(yaw),math.cos(yaw)]])+[x,y]

def footprint_array(poses,length,width):
    x,y,yaw=poses[:,0],poses[:,1],poses[:,2]
    local=np.array([[length/2,width/2],[length/2,-width/2],[-length/2,-width/2],[-length/2,width/2]])
    q=np.empty((len(poses),4,2));co=np.cos(yaw);si=np.sin(yaw)
    q[:,:,0]=x[:,None]+co[:,None]*local[:,0]-si[:,None]*local[:,1]
    q[:,:,1]=y[:,None]+si[:,None]*local[:,0]+co[:,None]*local[:,1]
    return polygons(q)

def evaluate(scene,result):
    logs=result['logs'];a=lambda k:np.array([r.get(k) for r in logs],dtype=float);t=a('t');dt=np.diff(t)
    err=np.hypot(a('x')-a('ref_x'),a('y')-a('ref_y'))
    lat=-(a('x')-a('ref_x'))*np.sin(a('ref_yaw'))+(a('y')-a('ref_y'))*np.cos(a('ref_yaw'))
    beta=np.abs(a('beta')[a('vx')>1]);slip=np.array([[r['slip_'+w] for w in ['FL','FR','RL','RR']] for r in logs],dtype=float)
    def finite_peak(v):
        v=np.asarray(v,dtype=float);v=v[np.isfinite(v)]
        return float(np.max(np.abs(v))) if len(v) else None
    saturation=np.array([[r.get('force_demand_ratio_'+w) for w in ['FL','FR','RL','RR']] for r in logs],dtype=float)
    metrics={'tracking_rmse_m':float(np.sqrt(np.mean(err**2))),'lateral_rmse_m':float(np.sqrt(np.mean(lat**2))),
        'final_tracking_error_m':float(err[-1]),'speed_rmse_mps':float(np.sqrt(np.mean((a('vx')-a('ref_v'))**2))),
        'peak_abs_beta_deg_speed_gt_1':float(np.max(beta)*180/np.pi) if len(beta) else None,
        'peak_abs_yaw_rate_radps':float(np.max(np.abs(a('yaw_rate')))),
        'peak_abs_steer_rad':finite_peak(a('steer_actual')),
        'peak_abs_steer_rate_radps':finite_peak(np.diff(a('steer_actual'))/dt) if len(dt) else 0.,
        'peak_abs_ax_mps2':finite_peak(a('ax')), 'peak_abs_ay_mps2':finite_peak(a('ay')),
        'peak_abs_slip_speed_gt_2':finite_peak(slip[a('vx')>2].ravel()) if np.any(a('vx')>2) else None,
        'command_limit_fraction':float(np.mean(a('command_limited'))),
        'tire_saturation_fraction':float(np.mean(np.max(saturation,axis=1)>1)) if np.isfinite(saturation).all() else None,
        'completed_horizon':result['failure'] is None,'simulation_failure':result['failure'],
        'collision':None,'offroad':None,'min_obstacle_clearance_m':None,'first_collision_time_s':None,
        'geometry_sampling_dt_s':result['config']['physics_dt'], 'geometry_check':'discrete_body_footprints_at_physics_steps_not_continuous_sweep',
        'official_benchmark_score':None}
    g=result['geometry'];c=result['config'];ego=footprint_array(g[:,1:4],c['length'],c['width'])
    road=scene.get('drivable_polygon')
    if road is not None:
        p=Polygon(road)
        if not p.is_valid or p.area<=0:raise ValueError('invalid drivable_polygon')
        outside=~covers(p,ego);metrics['offroad']=bool(np.any(outside));metrics['offroad_time_fraction']=float(np.mean(outside))
    # Missing annotations are unavailable, an explicit [] means verified no obstacles in this synthetic scene.
    if 'obstacles' in scene:
        collisions=np.zeros(len(g),dtype=bool);clearance=math.inf
        for obs in scene['obstacles']:
            if obs.get('length',0)<=0 or obs.get('width',0)<=0:raise ValueError('obstacle dimensions must be positive')
            path=np.asarray(obs['poses'],float)
            if path.ndim!=2 or path.shape[1]!=4 or not np.isfinite(path).all():raise ValueError('obstacle poses must be [t,x,y,yaw]')
            if len(path)==1:pose=np.tile(path[0,1:],(len(g),1))
            else:
                if np.any(np.diff(path[:,0])<=0) or path[0,0]>0 or path[-1,0]<g[-1,0]:raise ValueError('obstacle track must cover full rollout')
                path=path.copy();path[:,3]=np.unwrap(path[:,3]);pose=np.column_stack([np.interp(g[:,0],path[:,0],path[:,k]) for k in range(1,4)])
            other=footprint_array(pose,obs['length'],obs['width']);collisions|=intersects(ego,other);clearance=min(clearance,float(np.min(distance(ego,other))))
        metrics['collision']=bool(np.any(collisions));metrics['min_obstacle_clearance_m']=clearance if math.isfinite(clearance) else None
        if np.any(collisions):metrics['first_collision_time_s']=float(g[np.flatnonzero(collisions)[0],0])
    ref=np.asarray(result['reference']);d=np.diff(ref[:,1:3],axis=0);l2=np.sum(d*d,axis=1);p=a('x')[-1],a('y')[-1]
    w=np.asarray(p)-ref[:-1,1:3];frac=np.clip(np.sum(w*d,axis=1)/np.maximum(l2,1e-12),0,1)
    proj=ref[:-1,1:3]+frac[:,None]*d;j=int(np.argmin(np.linalg.norm(proj-p,axis=1)));lens=np.sqrt(l2);s0=np.r_[0,np.cumsum(lens)]
    metrics['route_progress_fraction']=float((s0[j]+frac[j]*lens[j])/s0[-1]) if s0[-1]>1e-9 else None
    return metrics

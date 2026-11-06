from .ego_geometry import ego_footprints
"""Annotation-conditional geometric safety and kinematic quality metrics.

No inferred traffic lights, legal fault, road direction, or drivable polygon.
White tolerance is distance from body edge, not a percentage of polygon area.
"""
from .marking_compliance import marking_compliance
import math
import numpy as np
from shapely.geometry import shape,Polygon,LineString,Point,box
from shapely import union_all,intersects,intersection,area,distance
from ..metrics import footprint_array
from .protocol import Protocol


def trajectory_array(rows,horizon):
    a=np.asarray(rows,float)
    if a.ndim!=2 or a.shape[1]!=4 or len(a)<3 or not np.isfinite(a).all():raise ValueError('trajectory must contain >=3 finite [t,x,y,yaw] rows')
    if abs(a[0,0])>1e-8 or abs(a[-1,0]-horizon)>1e-6 or np.any(np.diff(a[:,0])<=0):raise ValueError('trajectory must start at 0, end at exact horizon, and have increasing time')
    if np.max(np.diff(a[:,0]))>.5+1e-8:raise ValueError('trajectory spacing exceeds 0.5 s')
    return a

def resample(rows,t):
    a=np.array(rows,float,copy=True);a[:,3]=np.unwrap(a[:,3])
    return np.column_stack([t]+[np.interp(t,a[:,0],a[:,k]) for k in range(1,4)])

def event_runs(mask,t,kind,merge_gap=0.,min_duration=0.):
    mask=np.asarray(mask,bool);ids=np.flatnonzero(mask)
    if not len(ids):return []
    # Filter uninterrupted contacts before grouping; separated brief touches cannot accumulate into a violation.
    runs=np.split(ids,np.flatnonzero(np.diff(ids)>1)+1);dt=float(np.median(np.diff(t)))
    intervals=[]
    for run in runs:
        lo=float(t[run[0]]);hi=min(float(t[-1]),float(t[run[-1]])+dt)
        if hi-lo+1e-8>=min_duration:intervals.append([lo,hi])
    merged=[]
    for lo,hi in intervals:
        if merged and lo-merged[-1][1]<=merge_gap+1e-8:merged[-1][1]=hi
        else:merged.append([lo,hi])
    return [{'type':kind,'start_s':lo,'end_s':hi,'duration_s':hi-lo} for lo,hi in merged]

def marking_geometry(features,bounds):
    groups={'white':[],'yellow':[],'exclusion':[]};counts={'dashed':0,'other':0}
    for f in features:
        p=f['properties'];cl=p['class_id'];g=shape(f['geometry'])
        if not g.intersects(bounds):continue
        if p.get('non_drivable') or cl==100:kind='exclusion'
        elif p.get('road_boundary'):counts['other']+=1;continue
        elif cl==1:kind='white'
        elif cl in (8,9):kind='yellow'
        else:counts['dashed' if cl==2 else 'other']+=1;continue
        if g.geom_type in ('LineString','MultiLineString'):g=g.buffer(float(p.get('width_m',.15))/2,cap_style=2)
        groups[kind].append(g)
    return {k:union_all(v) for k,v in groups.items()},counts

def line_events(poses,t,g,kind,p):
    if g.is_empty:return [],{'contact_s':0.,'ignored_contact_s':0.,'available_marking':False}
    tol=getattr(p,kind+'_tolerance_m');duration=getattr(p,kind+'_persistence_s')
    body=ego_footprints(poses,p)
    inner=ego_footprints(poses,p,inset=tol)
    contact=area(intersection(body,g))>1e-8
    deep=area(intersection(inner,g))>1e-8
    runs=event_runs(deep,t,kind+'_solid',p.event_merge_gap_s,duration)
    # Catch a complete crossing even if it occurs between short sampled contacts.
    crossing=np.zeros(len(t),bool)
    for i in range(1,len(t)):
        if np.linalg.norm(poses[i,:2]-poses[i-1,:2])>1e-8:
            crossing[i]=LineString([poses[i-1,:2],poses[i,:2]]).crosses(g)
    for event in event_runs(crossing,t,kind+'_solid',p.event_merge_gap_s):
        if not any(event['start_s']<=r['end_s']+p.event_merge_gap_s and event['end_s']>=r['start_s']-p.event_merge_gap_s for r in runs):runs.append(event)
    dt=np.diff(t)
    return sorted(runs,key=lambda r:r['start_s']),{'contact_s':float(np.sum(dt*contact[:-1])),'ignored_contact_s':float(np.sum(dt*(contact&~deep)[:-1])),'available_marking':True}

class Actors:
    def __init__(self,frames,p):
        self.frames=frames;self.ts=np.array([f['t'] for f in frames]);self.p=p
    def at(self,time):
        idx=int(np.searchsorted(self.ts,time,side='right')-1);idx=max(0,min(idx,len(self.ts)-2))
        if idx>0 and abs(time-self.ts[idx])<1e-8 and (self.frames[idx]['key']!=self.frames[idx+1]['key'] or self.ts[idx+1]-self.ts[idx]>self.p.max_observation_gap_s):idx-=1
        a,b=self.frames[idx:idx+2];dt=b['t']-a['t']
        if time<self.ts[0]-1e-7 or time>self.ts[-1]+1e-7 or not a['dynamic_available'] or not b['dynamic_available'] or dt<=0 or dt>self.p.max_observation_gap_s+1e-8 or a['key']!=b['key']:return None
        alpha=float(np.clip((time-a['t'])/dt,0,1));left={o['id']:o for o in a['objects']};right={o['id']:o for o in b['objects']};out=[]
        for key in left.keys()|right.keys():
            x,y=left.get(key),right.get(key)
            if x and y and np.shape(x['xy'])==np.shape(y['xy']):
                q=np.asarray(x['xy']);r=np.asarray(y['xy']);velocity=(r[:-1].mean(axis=0)-q[:-1].mean(axis=0))/dt
                if np.linalg.norm(velocity)>70: # IDs may switch; do not interpolate an implausible sweep.
                    near=x if alpha<.5 else y;out.append((key,Polygon(near['xy']),None));continue
                poly=Polygon(q*(1-alpha)+r*alpha)
                if not poly.is_valid:out.append((key,Polygon((x if alpha<.5 else y)['xy']),None))
                else:out.append((key,poly,velocity))
            else:
                # A singleton is not extrapolated as a verified moving track.
                near=x if alpha<.5 else y
                if near:out.append((key,Polygon(near['xy']),None))
        return out

def score_window(scene,window,rows,p=None,execution_status='supplied_executed_trajectory'):
    p=(p or Protocol()).validate();a=trajectory_array(rows,p.horizon_s)
    n=int(math.ceil(p.horizon_s/p.sample_dt_s));t=np.linspace(0,p.horizon_s,n+1);a=resample(a,t);poses=a[:,1:].copy()
    co,si=np.cos(poses[:,2]),np.sin(poses[:,2]);poses[:,0]+=co*p.ego_center_forward_m-si*p.ego_center_left_m;poses[:,1]+=si*p.ego_center_forward_m+co*p.ego_center_left_m
    body=ego_footprints(poses,p)
    bounds=box(poses[:,0].min()-10,poses[:,1].min()-10,poses[:,0].max()+10,poses[:,1].max()+10)
    geoms,counts=marking_geometry(scene['features'],bounds)
    events=[];metrics={};details={}
    for kind in ('white','yellow'):
        e,detail=line_events(poses,t,geoms[kind],kind,p);events+=e;metrics[kind+'_compliance']=int(not e);details[kind]=detail
        metrics[kind+'_violation']=int(bool(e)) if detail['available_marking'] else None
        metrics[kind+'_violation_count']=len(e) if detail['available_marking'] else None
    exclusion=geoms['exclusion']
    if not exclusion.is_empty:
        mask=area(intersection(body,exclusion))>1e-8;e=event_runs(mask,t,'non_drivable',p.event_merge_gap_s);events+=e;metrics['annotated_exclusion_compliance']=int(not e)
    else:metrics['annotated_exclusion_compliance']=None
    frames=scene['frames'];ts=np.array([f['t'] for f in frames]);times=t+window['start_s'];actors=Actors(frames,p)
    velocity=np.gradient(poses[:,:2],t,axis=0);acc=np.gradient(velocity,t,axis=0);speed=np.linalg.norm(velocity,axis=1)
    ax=acc[:,0]*co+acc[:,1]*si;ay=-acc[:,0]*si+acc[:,1]*co;jerk=np.linalg.norm(np.gradient(acc,t,axis=0),axis=1)
    collision=np.zeros(len(t),bool);ttc_risk=np.zeros(len(t),bool);min_clearance=math.inf;min_ttc=math.inf;unavailable=0;unknown_velocity=0;actor_observations=0;collision_ids=set()
    for i,time in enumerate(times):
        objs=actors.at(time)
        if objs is None:unavailable+=1;continue
        for oid,poly,v in objs:
            actor_observations+=1
            clearance=body[i].distance(poly);min_clearance=min(min_clearance,clearance)
            if body[i].intersection(poly).area>1e-6:collision[i]=True;collision_ids.add(oid)
            if v is None:unknown_velocity+=1;continue
            # Constant-velocity rectangle forecast; actors remain non-reactive.
            # Bounding distance cheaply avoids distant actor forecasts.
            if clearance>np.linalg.norm(velocity[i]-v)*p.ttc_horizon_s+1:continue
            from shapely.affinity import translate
            for tau in np.arange(0,p.ttc_horizon_s+p.ttc_step_s/2,p.ttc_step_s):
                ego_future=translate(body[i],xoff=velocity[i,0]*tau,yoff=velocity[i,1]*tau)
                other=translate(poly,xoff=v[0]*tau,yoff=v[1]*tau)
                if ego_future.intersection(other).area>1e-6:
                    min_ttc=min(min_ttc,float(tau));ttc_risk[i]=True;break
    events+=event_runs(collision,t,'collision',p.event_merge_gap_s)
    unresolved=sum(f.get('unresolved_2d_count',0) for f in frames if window['start_s']<=f['t']<=window['end_s'])
    metrics['no_collision']=0 if collision.any() else None if unavailable or unresolved else 1
    metrics['ttc_compliance']=0 if ttc_risk.any() else None if unavailable or unknown_velocity or unresolved else 1
    # Progress is bounded, relative to logged route arc length, not arbitrary Euclidean displacement.
    ref=np.column_stack([ts-window['start_s'],[f['xy'][0] for f in frames],[f['xy'][1] for f in frames],[f['yaw'] for f in frames]])
    reference=resample(ref,t);route=LineString(reference[:,1:3]);length=route.length
    along=np.array([route.project(Point(q)) for q in poses[:,:2]]) if length>1e-6 else np.zeros(len(t))
    forward=max(0,float(along[-1]-along[0]));lateral=np.array([route.distance(Point(q)) for q in poses[:,:2]])
    # Lateral support bounds shortcut credit; legitimate adjacent lane progress remains possible.
    progress=float(np.clip(forward/length,0,1)) if length>1. else float(np.linalg.norm(poses[-1,:2]-poses[0,:2])<=1.)
    if np.any(lateral>8):progress=0.
    metrics['progress']=progress
    comfortable=(np.abs(ax)<=p.comfort_longitudinal_mps2)&(np.abs(ay)<=p.comfort_lateral_mps2)&(jerk<=p.comfort_jerk_mps3)
    metrics['comfort']=float(np.sum(np.diff(t)*comfortable[:-1])/p.horizon_s)
    limits=(p.measured_max_accel_mps2,p.measured_max_decel_mps2,p.measured_max_lateral_mps2)
    if all(v is not None for v in limits):
        dynamic=(ax>limits[0])|(ax < -limits[1])|(np.abs(ay)>limits[2]);events+=event_runs(dynamic,t,'motion_limit',p.event_merge_gap_s,.1);metrics['motion_limit_compliance']=int(not dynamic.any())
    else:metrics['motion_limit_compliance']=None
    metrics['marking_compliance']=marking_compliance(metrics)
    gate_keys=('no_collision','marking_compliance','annotated_exclusion_compliance')
    known_failure=any(metrics[k]==0 for k in gate_keys)
    core=[metrics[k] for k in ('no_collision','marking_compliance','ttc_compliance','progress','comfort')]
    score=0. if known_failure else None
    if not known_failure and all(v is not None for v in core):
        score=100*(p.progress_weight*metrics['progress']+p.ttc_weight*metrics['ttc_compliance']+p.comfort_weight*metrics['comfort'])/(p.progress_weight+p.ttc_weight+p.comfort_weight)
    # One intervention episode can have several simultaneous reasons; never count each frame as a takeover.
    event_mask=np.zeros(len(t),bool)
    for e in events:event_mask|=(t>=e['start_s']-1e-8)&(t<=e['end_s']+1e-8)
    interventions=event_runs(event_mask,t,'intervention_required',p.event_merge_gap_s)
    for e in events:
        idx=min(len(t)-1,int(np.searchsorted(t,e['start_s'])));e['xy']=poses[idx,:2].tolist();e['absolute_start_s']=window['start_s']+e['start_s'];e['absolute_end_s']=window['start_s']+e['end_s']
    unresolved=sum(f.get('unresolved_2d_count',0) for f in frames if window['start_s']<=f['t']<=window['end_s'])
    return {'window_id':window['id'],'scene_id':scene['scene_id'],'start_s':window['start_s'],'category':window.get('category'),'score':score,'dynamics_adjusted_score':(score*metrics['motion_limit_compliance']) if score is not None and metrics['motion_limit_compliance'] is not None else None,'score_name':'ExtremeDrive Marking Score (annotation-conditional, v1)','score_status':'unavailable' if score is None else 'annotation_conditional','execution_status':execution_status,'metrics':metrics,'events':events,'intervention_required_count':len(interventions),'intervention_intervals':interventions,'actual_takeover_count':None,'details':{**details,'ignored_map_counts':counts,'min_clearance_m':None if math.isinf(min_clearance) else min_clearance,'min_ttc_s':None if math.isinf(min_ttc) else min_ttc,'ttc_no_hit_within_horizon':math.isinf(min_ttc),'collision_actor_ids':sorted(collision_ids),'missing_dynamic_samples':unavailable,'actor_samples_without_velocity':unknown_velocity,'actor_samples':actor_observations,'unresolved_2d_count':unresolved,'route_length_m':length,'travel_distance_m':float(np.linalg.norm(np.diff(poses[:,:2],axis=0),axis=1).sum()),'max_route_distance_m':float(lateral.max()),'peak_accel_mps2':float(ax.max()),'peak_decel_mps2':float(-ax.min()),'peak_lateral_mps2':float(np.max(np.abs(ay))),'peak_jerk_mps3':float(jerk.max()),'drivable_area_compliance':None,'traffic_light_compliance':None,'driving_direction_compliance':None},'trajectory':a.tolist()}

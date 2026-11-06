from ..benchmark.ego_geometry import ego_footprints
from dataclasses import dataclass,asdict
import math
import numpy as np
from scipy.signal import savgol_filter
from shapely.geometry import LineString,Point,shape,box
from shapely import union_all
from ..benchmark.scoring import resample,Actors,event_runs
from ..metrics import footprint_array

@dataclass(frozen=True)
class Settings:
    version:str='extremedrive-extended-v2'
    history_s:float=1.0
    smoothing_s:float=.5
    long_accel_min:float=-4.05
    long_accel_max:float=2.4
    lateral_accel_max:float=4.89
    jerk_max:float=8.37
    yaw_rate_max:float=.95
    yaw_accel_max:float=1.93
    lane_deviation_m:float=.5
    lane_duration_s:float=2.0
    adjacent_plan_offset_s:float=.5
    ec_position_rms_m:float=.5
    ec_accel_rms_mps2:float=1.0
    ec_yaw_rate_rms_radps:float=.2
    direction_horizon_s:float=1.0
    direction_soft_m:float=2.0
    direction_hard_m:float=6.0


def signals(rows,dt=.05,smoothing=.5):
    a=np.array(rows,float);t=np.arange(a[0,0],a[-1,0]+dt*.1,dt)
    a=resample(a,t);size=min(len(t)//2*2-1,max(5,int(round(smoothing/dt))//2*2+1))
    if size<5:raise ValueError('at least five samples needed for derivatives')
    xy=savgol_filter(a[:,1:3],size,3,axis=0);yaw=savgol_filter(np.unwrap(a[:,3]),size,3)
    v=savgol_filter(a[:,1:3],size,3,deriv=1,delta=dt,axis=0)
    acc=savgol_filter(a[:,1:3],size,3,deriv=2,delta=dt,axis=0)
    jerk=savgol_filter(a[:,1:3],size,3,deriv=3,delta=dt,axis=0)
    yr=savgol_filter(np.unwrap(a[:,3]),size,3,deriv=1,delta=dt)
    ya=savgol_filter(np.unwrap(a[:,3]),size,3,deriv=2,delta=dt)
    co,si=np.cos(yaw),np.sin(yaw)
    return {'t':t,'xy':xy,'yaw':yaw,'velocity':v,'acceleration':acc,'ax':acc[:,0]*co+acc[:,1]*si,'ay':-acc[:,0]*si+acc[:,1]*co,'jerk':np.linalg.norm(jerk,axis=1),'yaw_rate':yr,'yaw_accel':ya}

def comfort(scene,w,rows,settings=None):
    s=settings or Settings();start=w['start_s'];key=w['source_key']
    history=[f for f in scene['frames'] if start-s.history_s-1e-8<=f['t']<start-1e-8 and f['key']==key]
    history_complete=bool(history and history[0]['t']<=start-s.history_s+.01 and max(np.diff([f['t'] for f in history]+[start]))<=.5)
    past=[[f['t']-start,*f['xy'],f['yaw']] for f in history]
    sig=signals(past+rows,smoothing=s.smoothing_s);mask=sig['t']>=-1e-8
    checks={'longitudinal':(sig['ax']>=s.long_accel_min)&(sig['ax']<=s.long_accel_max),'lateral':np.abs(sig['ay'])<=s.lateral_accel_max,'jerk':sig['jerk']<=s.jerk_max,'yaw_rate':np.abs(sig['yaw_rate'])<=s.yaw_rate_max,'yaw_acceleration':np.abs(sig['yaw_accel'])<=s.yaw_accel_max}
    ok=np.logical_and.reduce(list(checks.values()))[mask]
    return {'history_comfort':int(ok.all()) if history_complete else None,'comfort_time_fraction':float(ok.mean()),'history_complete':history_complete,'component_pass':{k:bool(v[mask].all()) for k,v in checks.items()},'peak_values':{k:float(np.max(np.abs(sig[k][mask]))) for k in ('ax','ay','jerk','yaw_rate','yaw_accel')},'history_samples':len(history)}

def extended_comfort(first,second,offset=.5,settings=None):
    s=settings or Settings()
    if second is None:return {'extended_comfort':None,'reason':'adjacent_plan_not_submitted'}
    x=np.asarray(first,float);y=np.asarray(second,float).copy();y[:,0]+=offset
    lo=max(x[0,0],y[0,0]);hi=min(x[-1,0],y[-1,0]);
    if offset<=0 or hi-lo<1:raise ValueError('adjacent plans require positive offset and >=1 second overlap')
    t=np.arange(lo,hi+.001,.05);a=resample(x,t);b=resample(y,t)
    sa=signals(a,smoothing=s.smoothing_s);sb=signals(b,smoothing=s.smoothing_s)
    pos=float(np.sqrt(np.mean(np.sum((a[:,1:3]-b[:,1:3])**2,axis=1))))
    acc=float(np.sqrt(np.mean(np.sum((sa['acceleration']-sb['acceleration'])**2,axis=1))))
    yaw=float(np.sqrt(np.mean((sa['yaw_rate']-sb['yaw_rate'])**2)))
    return {'extended_comfort':int(pos<=s.ec_position_rms_m and acc<=s.ec_accel_rms_mps2 and yaw<=s.ec_yaw_rate_rms_radps),'position_rms_m':pos,'acceleration_rms_mps2':acc,'yaw_rate_rms_radps':yaw,'overlap_s':hi-lo,'definition':'ExtremeDrive adjacent-plan consistency; not exact NAVSIM EC implementation'}

def lane_model(scene,w):
    """Evidence-gated cross-sections from painted boundaries; not an HD-map lane ground truth."""
    fs=[f for f in scene['frames'] if w['start_s']<=f['t']<=w['end_s']]
    x=np.array([f['xy'] for f in fs]);bb=box(x[:,0].min()-12,x[:,1].min()-12,x[:,0].max()+12,x[:,1].max()+12)
    geoms=[]
    for f in scene['features']:
        p=f['properties']
        if p.get('road_boundary') or p['class_id'] not in (1,2,8,9):continue
        g=shape(f['geometry'])
        if not g.intersects(bb):continue
        if g.geom_type.endswith('LineString'):g=g.buffer(float(p.get('width_m',.15))/2)
        geoms.append(g)
    paint=union_all(geoms);samples=[]
    for f in fs:
        q=np.array(f['xy']);normal=np.array([-math.sin(f['yaw']),math.cos(f['yaw'])]);cross=LineString([q-10*normal,q+10*normal]);hit=cross.intersection(paint)
        parts=list(hit.geoms) if hasattr(hit,'geoms') else [hit];offsets=[]
        for part in parts:
            if not part.is_empty:offsets.append(float(np.dot(np.array(part.centroid.coords[0])-q,normal)))
        left=[v for v in offsets if v>.5];right=[v for v in offsets if v<-.5]
        if not left or not right:continue
        l=min(left);r=max(right);width=l-r
        if 2.5<=width<=5.5:samples.append({'t':f['t']-w['start_s'],'center':(q+(l+r)/2*normal).tolist(),'left':(q+l*normal).tolist(),'right':(q+r*normal).tolist(),'width':width})
    usable=len(samples)>=3 and len(samples)/max(len(fs),1)>=.7 and np.ptp([v['width'] for v in samples])<=1.
    episode=next((ep for ep in scene.get('episodes',[]) if any(tag in ep.lower() for tag in ('turn','lane_change','u_turn'))),None)
    # No intersection semantics exists in v1; maneuver sources are conservatively unscored.
    if episode:usable=False
    return {'status':'derived_unverified','usable_for_proxy':bool(usable),'support_fraction':len(samples)/max(len(fs),1),'samples':samples,'reason':'maneuver_or_incomplete_or_unstable_paint' if not usable else 'paint_supported_lane_pair'}

def lane_keeping(rows,model,settings=None):
    s=settings or Settings()
    if not model.get('usable_for_proxy'):return {'lane_keeping_proxy':None,'reason':model['reason']}
    a=np.array(rows,float);samples=model['samples'];center=LineString([q['center'] for q in samples])
    covered=(a[:,0]>=samples[0]['t'])&(a[:,0]<=samples[-1]['t'])
    d=np.array([center.distance(Point(q)) for q in a[:,1:3]])
    events=event_runs((d>s.lane_deviation_m)&covered,a[:,0],'lane_deviation',0,s.lane_duration_s)
    return {'lane_keeping_proxy':int(not events),'max_offset_m':float(d[covered].max()) if covered.any() else None,'events':events,'coverage_fraction':float(covered.mean()),'not_verified_lane_topology':True}

def route_direction(scene,w,rows,settings=None):
    s=settings or Settings();a=np.array(rows,float);fs=scene['frames'];tt=np.array([f['t']-w['start_s'] for f in fs]);yaw=np.interp(a[:,0],tt,np.unwrap([f['yaw'] for f in fs]));delta=np.diff(a[:,1:3],axis=0)
    backwards=np.maximum(0,-(delta[:,0]*np.cos(yaw[:-1])+delta[:,1]*np.sin(yaw[:-1])));peak=0.
    for i in range(len(backwards)):peak=max(peak,float(backwards[(a[:-1,0]>=a[i,0]-s.direction_horizon_s)&(a[:-1,0]<=a[i,0])].sum()))
    return {'route_direction_proxy':1. if peak<s.direction_soft_m else .5 if peak<s.direction_hard_m else 0.,'max_reverse_progress_m':peak,'not_legal_lane_direction':True}

def collision_types(scene,w,rows,p,base_result):
    ids=base_result.get('details',{}).get('collision_actor_ids',[])
    if not ids:return {'at_fault_collision_proxy':base_result['metrics'].get('no_collision'),'collisions':[]}
    a=np.asarray(rows,float);v=np.gradient(a[:,1:3],a[:,0],axis=0);bodies=ego_footprints(a[:,1:],p);actors=Actors(scene['frames'],p);found=[];seen=set()
    for i,t in enumerate(a[:,0]):
        for oid,poly,vel in actors.at(t+w['start_s']) or []:
            if oid not in ids or oid in seen or bodies[i].intersection(poly).area<=1e-6:continue
            seen.add(oid);rel=np.array(poly.centroid.coords[0])-a[i,1:3];along=rel@[math.cos(a[i,3]),math.sin(a[i,3])]
            if vel is None:kind='unknown_motion';fault=None
            elif np.linalg.norm(v[i])<.05:kind='ego_stopped';fault=False
            elif np.linalg.norm(vel)<.05:kind='stopped_target';fault=True
            elif along>p.ego_length_m*.25:kind='active_front';fault=True
            elif along<-p.ego_length_m*.25:kind='rear_contact';fault=False
            else:kind='lateral_requires_lane_occupancy';fault=None
            found.append({'actor_id':oid,'time_s':float(t),'type':kind,'at_fault_proxy':fault})
    score=0 if any(f['at_fault_proxy'] is True for f in found) else None if len(seen)!=len(ids) or any(f['at_fault_proxy'] is None for f in found) else 1
    return {'at_fault_collision_proxy':score,'collisions':found,'definition':'geometry/motion proxy, no legal responsibility claim'}

def verified_map_metrics(rows,layers,p):
    """Require explicitly verified supplementary polygons and direction for formal map metrics."""
    result={'drivable_area_compliance':None,'lane_keeping':None,'driving_direction_compliance':None,'traffic_light_compliance':None}
    if not layers or layers.get('status')!='verified':return result
    a=np.asarray(rows,float);body=ego_footprints(a[:,1:],p)
    if layers.get('drivable_area'):
        road=shape(layers['drivable_area'])
        if not road.is_valid:raise ValueError('invalid verified drivable polygon')
        result['drivable_area_compliance']=int(all(road.covers(g) for g in body))
    # Lane centerlines and directions are explicit, never assigned from paint colour.
    lanes=layers.get('lanes',[]);distance=[];wrong=[];junctions=[shape(g) for g in layers.get('intersections',[])]
    for row in a:
        pt=Point(row[1:3])
        if any(g.covers(pt) for g in junctions):distance.append(0.);wrong.append(False);continue
        options=[l for l in lanes if shape(l['polygon']).covers(pt)]
        if not options:distance.append(None);wrong.append(None);continue
        lane=min(options,key=lambda l:LineString(l['centerline']).distance(pt));line=LineString(lane['centerline']);distance.append(line.distance(pt))
        pos=line.project(pt);q=np.array(line.interpolate(min(line.length,pos+.2)).coords[0])-np.array(line.interpolate(max(0,pos-.2)).coords[0])
        wrong.append(float(np.dot(q,[math.cos(row[3]),math.sin(row[3])]))<0 if lane.get('direction_verified') else None)
    if distance and all(v is not None for v in distance):result['lane_keeping']=int(not event_runs(np.array(distance)>.5,a[:,0],'lane_deviation',0,2.))
    if wrong and all(v is not None for v in wrong):
        travel=np.r_[0,np.linalg.norm(np.diff(a[:,1:3],axis=0),axis=1)];maximum=max(float(np.sum(travel[(a[:,0]>=t-1)&(a[:,0]<=t)]*np.array(wrong)[(a[:,0]>=t-1)&(a[:,0]<=t)])) for t in a[:,0]);result['driving_direction_compliance']=1. if maximum<2 else .5 if maximum<6 else 0.
    # No traffic light evaluation without an explicit, time-covered state sequence.
    signals_=layers.get('signals')
    if signals_ is not None:
        compliant=True;complete=True
        for signal in signals_:
            line=shape(signal['stop_line']);intervals=signal['states']
            for i,row in enumerate(a):
                state=next((x['state'] for x in intervals if x['start_s']<=row[0]<x['end_s']),None)
                if state not in ('red','green','yellow'):complete=False;continue
                if i and state=='red' and LineString([a[i-1,1:3],row[1:3]]).intersects(line):compliant=False
        result['traffic_light_compliance']=int(compliant) if complete else None
    return result

def reference_filter(agent,human,keys):
    out=dict(agent);waived=[]
    for k in keys:
        if human.get(k)==0 and agent.get(k) is not None:out[k]=1.;waived.append(k)
    return out,waived

def two_stage(first_score,end_pose,followups,sigma_xy=1.,sigma_yaw=.2):
    """Aggregation interface only. Followups must already have valid independently evaluated scores."""
    if first_score is None or not followups:return {'two_stage_score':None,'reason':'scored_followup_branches_missing'}
    if not isinstance(first_score,(float,int)) or not math.isfinite(first_score) or not 0<=first_score<=100:raise ValueError('invalid first stage score')
    if sigma_xy<=0 or sigma_yaw<=0:raise ValueError('invalid kernel width')
    scores=np.array([f['score'] for f in followups],float)
    if not np.isfinite(scores).all() or np.any((scores<0)|(scores>100)):raise ValueError('invalid branch score')
    starts=np.array([f['start_pose'] for f in followups],float);end=np.asarray(end_pose,float)
    if starts.shape!=(len(followups),3) or end.shape!=(3,) or not np.isfinite(starts).all() or not np.isfinite(end).all():raise ValueError('invalid branch pose')
    delta=starts-end;yaw=np.arctan2(np.sin(delta[:,2]),np.cos(delta[:,2]));logw=-.5*(np.sum(delta[:,:2]**2,axis=1)/sigma_xy**2+yaw**2/sigma_yaw**2);weights=np.exp(logw-logw.max());weights/=weights.sum()
    return {'two_stage_score':float(first_score*np.dot(weights,scores)/100),'branch_weights':weights.tolist(),'followup_weighted_score':float(np.dot(weights,scores)),'definition':'ExtremeDrive SE2 Gaussian aggregation, not off-policy camera rendering'}

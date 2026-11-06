from ..benchmark.ego_geometry import ego_footprints
from ..benchmark.marking_compliance import marking_compliance
"""User-defined road protocol. Static annotations are never mutated."""
import json, argparse, math, hashlib
from pathlib import Path
import numpy as np
from shapely.geometry import shape, mapping, LineString, Polygon, Point, box
from shapely import union_all, make_valid
from ..benchmark.dataset import Snapshot, digest
from ..metrics import footprint_array

RULES = dict(version="extremedrive-road-rules-v3", lateral_search_m=18., max_marking_gap_m=6.,
             white_edge_tolerance_m=.15, marking_bridge_m=15., yellow_separates_opposing=True,
             traffic_lights="out_of_scope", off_policy_successors="out_of_scope", exclusion_layer="apply_where_annotated")

def build_road(scene,w,p,surface_root=None):
    if surface_root is not None:
        from .reviewed_surface import load_surface
        return load_surface(scene,w,surface_root)
    fs=[f for f in scene['frames'] if f['key']==w['source_key'] and w['start_s']-2<=f['t']<=w['end_s']+2]
    poses=np.array([[*f['xy'],f['yaw']] for f in fs]);bodies=ego_footprints(poses,p)
    bounds=union_all(list(bodies)).bounds;roi=box(bounds[0]-20,bounds[1]-20,bounds[2]+20,bounds[3]+20)
    marks=[]
    for f in scene['features']:
        prop=f['properties'];cid=prop.get('class_id')
        if cid not in (1,2,8,9) or prop.get('road_boundary'):continue
        g=shape(f['geometry'])
        if not g.intersects(roi):continue
        if g.geom_type.endswith('LineString'):g=g.buffer(.075)
        marks.append((g,cid))
    sections=[];patches=[];lanes=[]
    for f in fs:
        q=np.array(f['xy']);yaw=f['yaw'];n=np.array([-math.sin(yaw),math.cos(yaw)])
        cut=LineString([q-18*n,q+18*n]);hits=[]
        for g,cid in marks:
            h=g.intersection(cut)
            for part in (list(h.geoms) if hasattr(h,'geoms') else [h]):
                if not part.is_empty:hits.append((float((np.array(part.centroid.coords[0])-q)@n),cid in (8,9)))
        # Merge both paint strokes of double yellow into one divider.
        merged=[]
        for x,y in sorted(hits):
            if merged and x-merged[-1][0]<.45:merged[-1]=((x+merged[-1][0])/2,y or merged[-1][1])
            else:merged.append((x,y))
        # Only connect neighbouring observed markings; no inferred outer lane width.
        usable=[]
        for a,b in zip(merged,merged[1:]):
            if 2<=b[0]-a[0]<=6:usable.append((a[0],b[0]))
        connected=[(-p.ego_width_m/2,p.ego_width_m/2)]
        for _ in range(len(usable)+1):
            for a,b in usable:
                if any(a<=d+.2 and b>=c-.2 for c,d in connected) and (a,b) not in connected:connected.append((a,b))
        lo=min(a for a,b in connected);hi=max(b for a,b in connected)
        yellows=[x for x,y in merged if y and lo<=x<=hi]
        sections.append(dict(t=f['t'],q=q.tolist(),yaw=yaw,lo=lo,hi=hi,yellows=yellows,bands=[list(v) for v in usable if v in connected]))
    # A dashed marking is a continuous lane boundary with intermittent paint.
    # Bridge only matching supported cross sections on BOTH sides of a gap.
    original=[dict(s) for s in sections]
    for i,s in enumerate(sections):
        candidates=[]
        for j in range(i-1,-1,-1):
            a=original[j]
            if not a['bands']:continue
            for k in range(i+1,len(original)):
                b=original[k]
                if not b['bands']:continue
                span=np.linalg.norm(np.array(b['q'])-a['q'])
                if span>15 or b['t']-a['t']>3:continue
                if any(original[z+1]['t']-original[z]['t']>.5 for z in range(j,k)):continue
                angle=abs(math.atan2(math.sin(b['yaw']-a['yaw']),math.cos(b['yaw']-a['yaw'])))
                if angle>.25 or abs(a['lo']-b['lo'])>.8 or abs(a['hi']-b['hi'])>.8:continue
                if len(a['yellows'])!=len(b['yellows']):continue
                candidates.append((min(a['hi']-a['lo'],b['hi']-b['lo']),-span,a,b))
        if candidates:
            _,_,a,b=max(candidates,key=lambda c:c[:2]);ratio=(s['t']-a['t'])/(b['t']-a['t'])
            lo=a['lo']*(1-ratio)+b['lo']*ratio;hi=a['hi']*(1-ratio)+b['hi']*ratio
            if lo<s['lo']-.1 or hi>s['hi']+.1:
                s['lo']=min(lo,s['lo']);s['hi']=max(hi,s['hi']);s['interpolated_boundary']=True
                s['yellows']=[x*(1-ratio)+y*ratio for x,y in zip(a['yellows'],b['yellows'])]
                if len(a['bands'])==len(b['bands']):s['bands']=[[x*(1-ratio)+y*ratio for x,y in zip(aa,bb)] for aa,bb in zip(a['bands'],b['bands'])]
    # Join only temporally contiguous local cross sections; no join across recording seams.
    for a,b in zip(sections,sections[1:]):
        if b['t']-a['t']>.5:continue
        qa,qb=np.array(a['q']),np.array(b['q'])
        if np.linalg.norm(qb-qa)>15:continue
        na=np.array([-math.sin(a['yaw']),math.cos(a['yaw'])]);nb=np.array([-math.sin(b['yaw']),math.cos(b['yaw'])])
        g=make_valid(Polygon([qa+a['lo']*na,qa+a['hi']*na,qb+b['hi']*nb,qb+b['lo']*nb]))
        if not g.is_empty:patches.append(g)
    # Longitudinal caps include the vehicle footprint at context endpoints.
    for s in (sections[0],sections[-1]):
        q=np.array(s['q']);u=np.array([math.cos(s['yaw']),math.sin(s['yaw'])]);n=np.array([-u[1],u[0]])
        patches.append(Polygon([q+s['lo']*n-u*p.ego_length_m/2,q+s['hi']*n-u*p.ego_length_m/2,q+s['hi']*n+u*p.ego_length_m/2,q+s['lo']*n+u*p.ego_length_m/2]))
    road=make_valid(union_all(list(bodies)+patches))
    return dict(status='user_defined_protocol',drivable_area=mapping(road),sections=sections,
                basis='observed ego swept footprint plus connected white/yellow marking bands',
                marked_section_fraction=sum(bool(s['bands']) for s in sections)/max(1,len(sections)))

def evaluate(rows,road,p):
    a=np.asarray(rows,float);g=shape(road['drivable_area']);bodies=ego_footprints(a[:,1:],p)
    # Apply the white-line graze tolerance also at the inferred road edge.
    inside=np.array([g.buffer(.15).covers(b) for b in bodies]);ss=road['sections'];xy=np.array([s['q'] for s in ss]);wrong=[]
    for r in a:
        s=ss[int(np.argmin(np.linalg.norm(xy-r[1:3],axis=1)))];q=np.array(s['q']);n=np.array([-math.sin(s['yaw']),math.cos(s['yaw'])]);lat=float((r[1:3]-q)@n)
        flips=sum(min(0,lat)<y<max(0,lat) for y in s['yellows']);sign=-1 if flips%2 else 1
        wrong.append(sign*math.cos(r[3]-s['yaw'])<0)
    travel=np.r_[0,np.linalg.norm(np.diff(a[:,1:3],axis=0),axis=1)]*wrong
    reverse=max(float(travel[(a[:,0]>=t-1)&(a[:,0]<=t)].sum()) for t in a[:,0])
    return dict(drivable_area_compliance=int(inside.all()),driving_direction_compliance=1. if reverse<2 else .5 if reverse<6 else 0.,max_wrong_way_distance_1s_m=reverse,offroad_time_fraction=float((~inside).mean()))

def total(base,ext):
    gates=[base.get('no_collision'),marking_compliance(base)]+[1 if base.get('annotated_exclusion_compliance') is None else base['annotated_exclusion_compliance']]+[ext.get('drivable_area_compliance'),ext.get('driving_direction_compliance')]
    weighted=[base.get('progress'),base.get('ttc_compliance'),ext.get('history_comfort'),ext.get('extended_comfort')]
    if any(v==0 for v in gates):return 0.
    if any(v is None for v in gates+weighted):return None
    return float(100*np.prod(gates)*np.dot([5,5,2,2],weighted)/14)

def run(snapshot,prior,out):
    snap=Snapshot(snapshot);data=json.loads(Path(prior).read_text());out=Path(out);out.mkdir(parents=True,exist_ok=True);(out/'maps').mkdir(exist_ok=True)
    if data['snapshot_sha256']!=snap.manifest['snapshot_sha256']:raise ValueError('snapshot mismatch')
    windows={w['id']:w for w in snap.manifest['windows']};cache={}
    for name,method in data['methods'].items():
        for i,r in enumerate(method['results']):
            if 'trajectory' not in r:r['road_rule_score']=0.;continue
            wid=r['window_id'];w=windows[wid]
            if wid not in cache:
                cache[wid]=build_road(snap.scene(w['scene_id']),w,snap.protocol)
                (out/'maps'/f"{wid.replace(':','_')}.json").write_text(json.dumps(cache[wid],separators=(',',':')))
            r['road_rule_metrics']=evaluate(r['trajectory'],cache[wid],snap.protocol)
            r['road_rule_score']=total(r['metrics'],{**r['extended_metrics'],**r['road_rule_metrics']})
            if (i+1)%100==0:print(name,i+1,flush=True)
    # Fixed dataset-defined cohort: prior complete-data set plus an observed adjacent window.
    from .runner import adjacent_window, confidence
    ids=[wid for wid in data['comparable_window_ids'] if adjacent_window(snap.scene(windows[wid]['scene_id']),windows[wid],.5)]
    summary={name:{'score':confidence(m['results'],'road_rule_score',set(ids)), 'windows':len(m['results']), 'road_available':sum('road_rule_metrics' in r for r in m['results'])} for name,m in data['methods'].items()}
    data.update(road_implementation_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),road_rules=RULES,road_rules_sha256=digest(RULES),road_ranking_window_ids=ids,road_summary=summary)
    data['unavailable_requirements']={};data['excluded_from_protocol']=['traffic_lights','off_policy_successors']
    (out/'results.json').write_text(json.dumps(data,separators=(',',':'),ensure_ascii=False,allow_nan=False))
    (out/'summary.json').write_text(json.dumps(dict(rules=RULES,methods=summary,snapshot_sha256=data['snapshot_sha256']),indent=2,ensure_ascii=False))
    print(summary,flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--snapshot',required=True);p.add_argument('--prior',required=True);p.add_argument('--out',required=True);a=p.parse_args();run(a.snapshot,a.prior,a.out)

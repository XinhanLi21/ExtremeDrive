"""Separate causal feature building from privileged target building."""
from pathlib import Path
import json,math,hashlib
import numpy as np
from shapely.geometry import shape,LineString
from ..benchmark.dataset import Snapshot,digest
from ..benchmark.runner import baseline
from ..benchmark_v2.agents import agent_input,ConstantVelocityAgent

FEATURE_VERSION='ego8_actors36_markings16_v1'
TIMES=np.arange(.25,4.001,.25)

def features(inputs):
    h=inputs['history'];cur=h[-1];origin=np.array(cur['xy']);yaw=cur['yaw'];c,s=math.cos(yaw),math.sin(yaw);rot=np.array([[c,-s],[s,c]])
    def velocity(a,b):return (np.array(b['xy'])-a['xy'])/max(b['t']-a['t'],1e-6)
    v=velocity(h[-2],cur) if len(h)>1 else np.zeros(2)
    acc=(v-velocity(h[-3],h[-2]))/max(cur['t']-h[-2]['t'],1e-6) if len(h)>2 else np.zeros(2)
    yr=math.atan2(math.sin(yaw-h[-2]['yaw']),math.cos(yaw-h[-2]['yaw']))/max(cur['t']-h[-2]['t'],1e-6) if len(h)>1 else 0.
    result=[*(v@rot/30),*(acc@rot/10),yr,min(1.,-h[0]['t']),float(cur['dynamic_available']),min(1.,len(cur['objects'])/20)]
    past={o['id']:o for o in h[-2]['objects']} if len(h)>1 else {}
    def center(o):return np.array(shape({'type':'Polygon','coordinates':[o['xy']]}).centroid.coords[0])
    objects=sorted(cur['objects'],key=lambda o:np.linalg.norm(center(o)-origin))[:6]
    for o in objects:
        xy=center(o);ov=(xy-center(past[o['id']]))/max(cur['t']-h[-2]['t'],1e-6) if o['id'] in past else np.zeros(2)
        result.extend([*((xy-origin)@rot/50),*(ov@rot/30),float(o['id'] in past),1.])
    result.extend([0.]*(6-len(objects))*6)
    marks=[(shape(f['geometry']),f['properties']['class_id']) for f in inputs['static_map'] if f['properties'].get('class_id') in (1,2,8,9) and not f['properties'].get('road_boundary')]
    normal=np.array([-s,c]);forward=np.array([c,s])
    for dist in (0,10,25,45):
        q=origin+dist*forward;cut=LineString([q-15*normal,q+15*normal]);slots=[15.,15.,15.,15.]
        for g,cid in marks:
            hit=g.intersection(cut)
            for part in list(hit.geoms) if hasattr(hit,'geoms') else [hit]:
                if part.is_empty:continue
                offset=float((np.array(part.centroid.coords[0])-q)@normal);index=(2 if cid in (8,9) else 0)+(1 if offset<0 else 0);slots[index]=min(slots[index],abs(offset))
        result.extend(np.array(slots)/15)
    x=np.asarray(result,np.float32)
    if x.shape!=(60,) or not np.isfinite(x).all():raise ValueError('invalid causal features')
    return np.clip(x,-20,20)

def target(scene,w,p):
    rows=np.asarray(baseline(scene,w,p,'logged_replay'));origin=rows[0,1:3];yaw=rows[0,3];rot=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    xy=np.column_stack([np.interp(TIMES,rows[:,0],rows[:,j]) for j in (1,2)])
    return ((xy-origin)@rot).astype(np.float32)

def audit(snapshot):
    snap=Snapshot(snapshot);scene_splits={};source_splits={};counts={s:0 for s in ('train','val','test')}
    for w in snap.manifest['windows']:
        split=w['split'];counts[split]+=1
        for key,table in ((w['scene_id'],scene_splits),(w['source_key'],source_splits)):
            if key in table and table[key]!=split:raise ValueError('scene/source leakage across splits')
            table[key]=split
    episode_splits={}
    for entry in snap.manifest.get('scenes',[]):
        split=entry.get('benchmark_split')
        for episode in entry.get('episodes',[]):
            if episode in episode_splits and episode_splits[episode]!=split:raise ValueError('episode leakage across splits')
            episode_splits[episode]=split
    # Verify all available accepted scene file hashes, including those too short for a window.
    for sid in scene_splits:snap.scene(sid)
    return dict(snapshot_sha256=snap.manifest['snapshot_sha256'],counts=counts,scenes={s:sum(v==s for v in scene_splits.values()) for s in counts},source_disjoint=True,scene_disjoint=True)

def cache(snapshot,out):
    report=audit(snapshot);snap=Snapshot(snapshot);out=Path(out)
    if out.exists() and any(out.iterdir()):raise ValueError('cache directory must be new')
    out.mkdir(parents=True,exist_ok=True);manifest=dict(report,feature_version=FEATURE_VERSION,feature_dim=60,target_shape=[16,2],files={})
    for split in ('train','val','test'):
        xs=[];ys=[];ids=[]
        for w in snap.manifest['windows']:
            if w['split']!=split:continue
            sc=snap.scene(w['scene_id']);xs.append(features(agent_input(sc,w)));ids.append(w['id'])
            # Public test feature cache contains no future target arrays.
            if split!='test':ys.append(target(sc,w,snap.protocol))
        payload={'features':np.array(xs,dtype=np.float32),'ids':np.array(ids)}
        if split!='test':payload['targets']=np.array(ys,dtype=np.float32)
        path=out/f'{split}.npz';np.savez_compressed(path,**payload);manifest['files'][path.name]=hashlib.sha256(path.read_bytes()).hexdigest();print('cached',split,len(ids),flush=True)
    manifest['cache_sha256']=digest(manifest);(out/'manifest.json').write_text(json.dumps(manifest,indent=2));return manifest

def load_cache(root,split):
    root=Path(root);meta=json.loads((root/'manifest.json').read_text());expected=meta.pop('cache_sha256')
    if digest(meta)!=expected:raise ValueError('cache manifest mismatch')
    meta['cache_sha256']=expected
    if meta['feature_version']!=FEATURE_VERSION:raise ValueError('feature version mismatch')
    p=root/f'{split}.npz'
    if hashlib.sha256(p.read_bytes()).hexdigest()!=meta['files'][p.name]:raise ValueError('cache file checksum mismatch')
    return np.load(p,allow_pickle=False),meta

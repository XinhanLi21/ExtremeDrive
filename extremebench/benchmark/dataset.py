"""Read-only adapter: one SQLite snapshot + original actor boxes in scene XY metres."""
import gzip,hashlib,json,sqlite3,math
from pathlib import Path
from functools import lru_cache
from collections import Counter
import numpy as np
from shapely.geometry import shape,Polygon,mapping
from shapely import union_all
from .protocol import Protocol

DEFAULT_ROOT=Path('/data/annotations_20260922/trajectory_full_10hz_20260922/extremedrive_dense_images_20260927')
DEFAULT_DB=Path('/home/lxh/project/annotation-viewer/data/extremedrive_dense_images_20260927/revisions.sqlite3')

def digest(v): return hashlib.sha256(json.dumps(v,ensure_ascii=False,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def read(path):
    path=Path(path)
    if path.suffix=='.gz':
        with gzip.open(path,'rt') as f:return json.load(f)
    return json.loads(path.read_text())
def write_gz(path,value):
    with gzip.open(path,'wt',encoding='utf8') as f:json.dump(value,f,ensure_ascii=False,separators=(',',':'),allow_nan=False)
def transform(points,tr):
    a=tr['rotation_rad'];c,s=math.cos(a),math.sin(a)
    return (np.asarray(points)@np.array([[c,s],[-s,c]])+tr['translation_xy_m']).tolist()

def build_snapshot(out,root=DEFAULT_ROOT,db=DEFAULT_DB,protocol=None):
    p=(protocol or Protocol()).validate();out=Path(out);root=Path(root)
    if out.exists() and any(out.iterdir()):raise ValueError('Snapshot output must be empty; existing releases are immutable')
    (out/'scenes').mkdir(parents=True,exist_ok=True)
    cat=read(root/'catalog.json')
    # A consistent read transaction captures concurrent editors at one instant.
    with sqlite3.connect(f'file:{Path(db).resolve()}?mode=ro',uri=True) as c:
        c.execute('BEGIN')
        rows=c.execute('SELECT document FROM revisions r WHERE revision=(SELECT MAX(revision) FROM revisions WHERE scene=r.scene)').fetchall()
    latest={d['scene_id']:d for d in map(lambda r:json.loads(r[0]),rows)}
    @lru_cache(maxsize=3)
    def part(key):
        file=root/'frames'/f'{key}.json.gz'
        return read(file if file.exists() else file.with_suffix(''))
    inventory=[];windows=[]
    for item in cat:
        sid=item['id'];rec=latest.get(sid);accepted=bool(rec and rec.get('acceptance',{}).get('status')=='accepted')
        entry={'scene_id':sid,'title':item['title'],'revision':rec['revision'] if rec else 0,'accepted':accepted,'episodes':item['episodes'],'frames':item['frames']}
        inventory.append(entry)
        if not accepted:entry['excluded_reason']='latest_revision_not_accepted';continue
        source=read(root/'scenes'/sid/'scene.json');fs=rec['features']
        bad=[];classes=Counter();clean=[];normalizations=[]
        for f in fs:
            props=f['properties'];classes[str(props['class_id'])]+=1
            g=shape(f['geometry'])
            if not g.is_valid and g.geom_type=='MultiPolygon' and all(part.is_valid for part in g.geoms):
                g=union_all(list(g.geoms))
                normalizations.append({'id':props['id'],'operation':'union_overlapping_polygon_components_same_covered_region'})
            if g.is_empty or not g.is_valid:bad.append(props['id']);continue
            keep={k:props[k] for k in ('id','class_id','kind','class_name','width_m','non_drivable','road_boundary','editor','source_episode_key') if k in props}
            clean.append({'type':'Feature','properties':keep,'geometry':mapping(g)})
        frames=[];errors=[];actor_count=0
        for row in source['frames']:
            key=row['key'];t=row['time_s'];objects=[];valid=True;unresolved=0
            try:
                raw=part(key)['frames'][row['index']]
                if raw['stamp']!=row['stamp']:raise ValueError('timestamp mismatch')
                if 'objects' not in raw:raise ValueError('objects field missing')
                tr=source['transforms'][key]
                unresolved=int(raw.get('unresolved_2d_count',0))
                for obj in raw['objects']:
                    q=transform(obj['xy'],tr)
                    if not Polygon(q).is_valid:raise ValueError('invalid object box')
                    objects.append({'id':f'{key}:{obj["id"]}','class':obj.get('class','unknown'),'xy':q,'estimated_size':obj.get('estimated_size',False)})
                actor_count+=len(objects)
            except (OSError,ValueError,KeyError,IndexError) as e:
                valid=False;errors.append({'time_s':t,'reason':str(e)})
            frames.append({'t':t,'xy':row['position'],'yaw':row['heading'],'key':key,'index':row['index'],'stamp':row['stamp'],'objects':objects,'dynamic_available':valid,'unresolved_2d_count':unresolved})
        document={'scene_id':sid,'title':item['title'],'coordinate_frame':'scene_xy_m','revision':rec['revision'],'acceptance':rec['acceptance'],'annotation_sha256':digest(fs),'features':clean,'frames':frames,'episodes':item['episodes'],'dynamic_errors':errors,'invalid_map_features':bad,'geometry_normalizations':normalizations}
        scene_hash=digest(document);file=f'scenes/{sid}.json.gz';write_gz(out/file,document)
        entry.update(file=file,sha256=scene_hash,annotation_sha256=document['annotation_sha256'],classes=dict(classes),actor_observations=actor_count,dynamic_errors=len(errors),invalid_map_features=len(bad))
        # Preserve complete scenes. Evaluation windows never cross an estimated episode join or missing interval.
        ts=np.array([f['t'] for f in frames]);start=0;excluded=Counter();nwindows=0
        for end in range(1,len(frames)+1):
            if end<len(frames) and frames[end]['key']==frames[end-1]['key'] and 0<ts[end]-ts[end-1]<=p.max_observation_gap_s:continue
            indices=list(range(start+1,end));next_t=ts[start]
            for i in indices:
                if ts[i]+1e-8<next_t or ts[i]+p.horizon_s>ts[end-1]+1e-8:continue
                j=int(np.searchsorted(ts,ts[i]+p.horizon_s,side='left'));j=min(j,end-1)
                selected=frames[i:j+1]
                reason=None
                if bad:reason='invalid_map_geometry'
                elif any(not x['dynamic_available'] for x in selected):reason='missing_dynamic_annotations'
                if reason:excluded[reason]+=1
                else:
                    source_episode=source['transforms'][frames[i]['key']].get('source_episode','')
                    category=source_episode.split('/')[1] if len(source_episode.split('/'))>1 else 'unknown'
                    windows.append({'id':f'{sid}:{i:06d}','scene_id':sid,'start_s':float(ts[i]),'end_s':float(ts[i]+p.horizon_s),'frame_start':i,'source_key':frames[i]['key'],'category':category,'unresolved_2d_count':sum(x['unresolved_2d_count'] for x in selected)})
                    nwindows+=1
                next_t=ts[i]+p.stride_s
            start=end
        entry.update(windows=nwindows,excluded_windows=dict(excluded))
        print(f'snapshot {sid} v{rec["revision"]}: {nwindows} windows',flush=True)
    # Hash by shared source episodes: scenes connected by any episode stay in one split.
    parents={s['scene_id']:s['scene_id'] for s in inventory}
    def find(x):
        while parents[x]!=x:parents[x]=parents[parents[x]];x=parents[x]
        return x
    seen={}
    for s in inventory:
        for episode in s['episodes']:
            if episode in seen:parents[find(s['scene_id'])]=find(seen[episode])
            seen[episode]=s['scene_id']
    for s in inventory:
        group=find(s['scene_id']);n=int(hashlib.sha256(group.encode()).hexdigest()[:8],16)%100
        s['split_group']=group;s['benchmark_split']='train' if n<70 else 'val' if n<85 else 'test'
    meta={s['scene_id']:s for s in inventory}
    for w in windows:w['split']=meta[w['scene_id']]['benchmark_split']
    manifest={'format':'extremebench-snapshot-v1','adapter_version':'scene-annotation-adapter-v1.1','protocol':p.__dict__,'protocol_sha256':p.identity(),'source_root':str(root),'scenes':inventory,'windows':windows,'split_policy':'scene/source-episode-disjoint hash split; independent of original train/val; not a public hidden test release','dynamic_scope':'recorded auto-labelled actor footprints; not assumed exhaustive or manually verified','map_policy':'latest accepted server revision only; unaccepted revisions never silently replaced with older accepted maps','coordinate_policy':'scene XY metres; same episode rigid transforms as editor; windows do not cross episode transforms'}
    manifest['snapshot_sha256']=digest(manifest)
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    return manifest

class Snapshot:
    def __init__(self,folder):
        self.folder=Path(folder);self.manifest=read(self.folder/'manifest.json')
        check=dict(self.manifest);claimed=check.pop('snapshot_sha256')
        if digest(check)!=claimed:raise ValueError('snapshot manifest hash mismatch')
        self.protocol=Protocol(**self.manifest['protocol']).validate()
        if self.protocol.identity()!=self.manifest['protocol_sha256']:raise ValueError('protocol hash mismatch')
        self.entries={s['scene_id']:s for s in self.manifest['scenes']}
    @lru_cache(maxsize=2)
    def scene(self,sid):
        entry=self.entries[sid];d=read(self.folder/entry['file'])
        if digest(d)!=entry['sha256']:raise ValueError('scene snapshot hash mismatch')
        return d

"""Submission contract and batch evaluation. No access to future labels for planner inputs."""
import argparse,json,math,hashlib
from pathlib import Path
from collections import defaultdict
from dataclasses import asdict,replace
import numpy as np
from shapely.geometry import shape,box
from .dataset import Snapshot,build_snapshot,digest
from .protocol import Protocol
from .scoring import score_window,resample,trajectory_array,Actors


def baseline(scene,w,p,name):
    fs=scene['frames'];ts=np.array([f['t'] for f in fs]);i=w['frame_start']
    t=np.linspace(0,p.horizon_s,17)
    if name=='logged_replay':
        full=np.array([[f['t']-w['start_s'],*f['xy'],f['yaw']] for f in fs]);return resample(full,t).tolist()
    f=fs[i]; # speed uses observed history only; no reference future leaks into this baseline.
    if i==0 or fs[i-1]['key']!=f['key']:speed=0.
    else:speed=float(np.linalg.norm(np.array(f['xy'])-fs[i-1]['xy'])/(f['t']-fs[i-1]['t']))
    return [[float(tt),f['xy'][0]+speed*tt*math.cos(f['yaw']),f['xy'][1]+speed*tt*math.sin(f['yaw']),f['yaw']] for tt in t]


def execute_plan(rows,scene,w,p):
    from ..core import Config,rollout
    a=trajectory_array(rows,p.horizon_s)
    initial=scene['frames'][w['frame_start']];i=w['frame_start'];fs=scene['frames']
    if np.linalg.norm(a[0,1:3]-initial['xy'])>.10 or abs(math.atan2(math.sin(a[0,3]-initial['yaw']),math.cos(a[0,3]-initial['yaw'])))>.05:raise ValueError('plan initial pose differs from recorded state')
    if i==0 or fs[i-1]['key']!=initial['key']:raise ValueError('controller needs an observed previous frame for initial speed')
    v0=float(np.linalg.norm(np.array(initial['xy'])-fs[i-1]['xy'])/(initial['t']-fs[i-1]['t']))
    speed=np.linalg.norm(np.gradient(a[:,1:3],a[:,0],axis=0),axis=1)
    trajectory=np.column_stack([a,speed]);c=Config(length=p.ego_length_m,width=p.ego_width_m)
    case={'id':w['id'],'trajectory':trajectory.tolist(),'initial_state':{'x':a[0,1],'y':a[0,2],'yaw':a[0,3],'vx':v0}}
    result=rollout(case,c)
    if result['failure']:raise ValueError('controller simulation failed: '+str(result['failure']))
    executed=[[r['t'],r['x'],r['y'],r['yaw']] for r in result['logs']]
    return executed,{'config_id':c.identity(),'config':asdict(c),'model_status':c.parameter_status}


def aggregate(rows,expected):
    usable=[r['score'] for r in rows if r.get('score') is not None]
    scenes=defaultdict(list);categories=defaultdict(list)
    for r in rows:
        scenes[r['scene_id']].append(r);categories[r.get('category','unknown')].append(r)
    def mean(group):
        values=[r['score'] for r in group if r.get('score') is not None];return float(np.mean(values)) if values else None
    # Deduplicate overlapping/consecutive window events on full scene time.
    event_groups=defaultdict(list)
    for r in rows:
        for e in r.get('events',[]):event_groups[r['scene_id']].append((e['absolute_start_s'],e['absolute_end_s']))
    episodes=0
    for sid,events in event_groups.items():
        end=-math.inf
        for lo,hi in sorted(events):
            if lo>end+.5:episodes+=1
            end=max(end,hi)
    eligible=[r for r in rows if r.get('ranking_eligible')]
    comparable=[r['score'] for r in eligible if r.get('score') is not None]
    metric_summary={}
    for key in ('white_compliance','yellow_compliance','no_collision','ttc_compliance','progress','comfort'):
        values=[r.get('metrics',{}).get(key) for r in rows];values=[v for v in values if v is not None]
        metric_summary[key]={'assessed_windows':len(values),'mean':float(np.mean(values)) if values else None,'zero_windows':sum(v==0 for v in values)}
    return {'metric_summary':metric_summary,'ranking_windows':len(eligible),'ranking_mean':float(np.mean(comparable)) if eligible and len(comparable)==len(eligible) else None,'expected_windows':expected,'evaluated_windows':len(rows),'scored_windows':len(usable),'unavailable_windows':len(rows)-len(usable),'failed_submissions':sum(r.get('submission_failure') is not None for r in rows),'score_coverage':len(usable)/expected if expected else 0,'benchmark_mean':float(np.mean(usable)) if len(usable)==expected and expected else None,'available_window_mean_diagnostic_only':float(np.mean(usable)) if usable else None,'scene_macro_mean_diagnostic_only':float(np.mean([mean(g) for g in scenes.values() if mean(g) is not None])) if usable else None,'intervention_required_episodes':episodes,'actual_takeover_count':None,'by_category':{k:{'windows':len(g),'scored':sum(r.get('score') is not None for r in g),'mean_diagnostic_only':mean(g)} for k,g in categories.items()}}


def evaluate(snapshot,out,names=('constant_velocity','logged_replay'),max_per_scene=None,split='all',submission=None,execution='controller'):
    snap=Snapshot(snapshot);p=snap.protocol;out=Path(out);out.mkdir(parents=True,exist_ok=True)
    windows=[w for w in snap.manifest['windows'] if split=='all' or w['split']==split]
    if max_per_scene:
        grouped=defaultdict(list)
        for w in windows:grouped[w['scene_id']].append(w)
        windows=[g[i] for g in grouped.values() for i in np.unique(np.linspace(0,len(g)-1,min(max_per_scene,len(g))).astype(int))]
    eligibility={}
    for w in windows:
        sc=snap.scene(w['scene_id']);actors=Actors(sc['frames'],p)
        valid=not any(f.get('unresolved_2d_count',0) for f in sc['frames'] if w['start_s']<=f['t']<=w['end_s'])
        for t in np.linspace(0,p.horizon_s,int(math.ceil(p.horizon_s/p.sample_dt_s))+1):
            if not valid:break
            objs=actors.at(w['start_s']+t)
            valid=objs is not None and all(v is not None for _,_,v in objs)
        eligibility[w['id']]=bool(valid)
    predictions=None
    if submission:
        doc=json.loads(Path(submission).read_text())
        if doc.get('snapshot_sha256')!=snap.manifest['snapshot_sha256'] or doc.get('protocol_sha256')!=p.identity():raise ValueError('submission snapshot/protocol mismatch')
        if doc.get('coordinate_frame')!='scene_xy_m':raise ValueError('submission must be in scene_xy_m')
        predictions=doc.get('predictions')
        if not isinstance(predictions,dict):raise ValueError('predictions must be keyed by window ID')
        if set(predictions)-{w['id'] for w in windows}:raise ValueError('unknown or out-of-scope prediction IDs')
        names=(str(doc.get('name','submitted_planner')),)
    outputs={};examples=[]
    for name in names:
        rows=[];selected_examples=0
        for n,w in enumerate(windows):
            scene=snap.scene(w['scene_id']);control=None
            try:
                if predictions is not None:
                    if w['id'] not in predictions:raise ValueError('missing prediction')
                    path=predictions[w['id']];mode=execution
                    if mode=='controller':path,control=execute_plan(path,scene,w,p)
                else:path=baseline(scene,w,p,name);mode='logged_replay_diagnostic' if name=='logged_replay' else 'constant_velocity_kinematic'
                # Enforce common starting pose for direct state submissions as well.
                first=trajectory_array(path,p.horizon_s)[0];f=scene['frames'][w['frame_start']]
                yaw_error=abs(math.atan2(math.sin(first[3]-f['yaw']),math.cos(first[3]-f['yaw'])))
                if np.linalg.norm(first[1:3]-f['xy'])>.1 or yaw_error>.05:raise ValueError('initial pose mismatch')
                result=score_window(scene,w,path,p,mode)
                if control:result['controller']=control
            except (ValueError,KeyError,TypeError,IndexError) as e:
                result={'window_id':w['id'],'scene_id':w['scene_id'],'start_s':w['start_s'],'category':w['category'],'score':0.,'score_status':'invalid_submission','submission_failure':str(e),'metrics':{},'events':[]}
            result['ranking_eligible']=eligibility[w['id']]
            m=result.get('metrics',{})
            result['map_compliance_score']=100*m['white_compliance']*m['yellow_compliance']*(m['annotated_exclusion_compliance'] if m.get('annotated_exclusion_compliance') is not None else 1) if 'white_compliance' in m else None
            rows.append(result)
            if 'trajectory' in result:
                # One representative per scene per method, including event evidence.
                if n==0 or windows[n-1]['scene_id']!=w['scene_id']:
                    a=np.array(result['trajectory']);bounds=box(a[:,1].min()-20,a[:,2].min()-20,a[:,1].max()+20,a[:,2].max()+20)
                    example={'method':name,'window':w,'result':result,'map_revision':scene['revision'],'acceptance':scene['acceptance'],'features':[f for f in scene['features'] if shape(f['geometry']).intersects(bounds)],'frames':[f for f in scene['frames'] if w['start_s']-.5<=f['t']<=w['end_s']+.5]}
                    file=f'cases/{len(examples):04d}.json';(out/'cases').mkdir(exist_ok=True);(out/file).write_text(json.dumps(example,ensure_ascii=False,separators=(',',':'),allow_nan=False));examples.append({'file':file,'method':name,'scene_id':w['scene_id'],'window_id':w['id'],'category':w['category'],'score':result['score']})
            if (n+1)%100==0:print(f'{name}: {n+1}/{len(windows)}',flush=True)
        outputs[name]={'summary':aggregate(rows,len(windows)),'results':rows}
    implementation={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in [*Path(__file__).parent.glob('*.py'),Path(__file__).parents[1]/'core.py',Path(__file__).parents[1]/'metrics.py']}
    report={'implementation_sha256':digest(implementation),'implementation_files':implementation,'ranking_window_ids':[w['id'] for w in windows if eligibility[w['id']]],'ranking_subset_sha256':digest([w['id'] for w in windows if eligibility[w['id']]]),'snapshot_sha256':snap.manifest['snapshot_sha256'],'protocol_sha256':p.identity(),'protocol':asdict(p),'total_scenes':len(snap.manifest['scenes']),'accepted_scenes':sum(s['accepted'] for s in snap.manifest['scenes']),'snapshot_windows':len(snap.manifest['windows']),'evaluation_windows':len(windows),'split':split,'window_selection':'all' if not max_per_scene else f'up to {max_per_scene} evenly spaced windows per complete scene','methods':outputs,'examples':examples,'notes':['Recorded actors are non-reactive, and their automatic annotations are not exhaustive perception ground truth.','Missing track velocity, missing 2D-to-3D targets, or observations yield unavailable safety metrics; unavailable totals are not renormalized to 100.','SL03 origin offset is an explicit unmeasured configuration assumption.','A recorded driver replay is a diagnostic reference, not a planner baseline or an automatic waiver of violations.']}
    (out/'results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False))
    import csv
    with (out/'scores.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=['method','window_id','scene_id','category','start_s','ranking_eligible','score','score_status','white_compliance','yellow_compliance','no_collision','ttc_compliance','progress','comfort','intervention_required_count','map_compliance_score','submission_failure']);writer.writeheader()
        for method,value in outputs.items():
            for row in value['results']:
                flat={**row,**row.get('metrics',{}),'method':method};writer.writerow({k:flat.get(k) for k in writer.fieldnames})
    # Summary is intentionally small; trajectories load on demand.
    brief={k:v for k,v in report.items() if k!='methods'};brief['methods']={k:v['summary'] for k,v in outputs.items()};(out/'summary.json').write_text(json.dumps(brief,ensure_ascii=False,indent=2))
    from shutil import copyfile
    copyfile(Path(__file__).with_name('viewer.html'),out/'index.html')
    print(json.dumps(brief['methods'],ensure_ascii=False,indent=2),flush=True)
    return report


def main():
    parser=argparse.ArgumentParser(description='ExtremeDrive annotation-based planning benchmark');sp=parser.add_subparsers(dest='cmd',required=True)
    b=sp.add_parser('snapshot');b.add_argument('--out',required=True);b.add_argument('--root');b.add_argument('--db');b.add_argument('--protocol')
    e=sp.add_parser('evaluate');e.add_argument('--snapshot',required=True);e.add_argument('--out',required=True);e.add_argument('--baseline',choices=['both','constant_velocity','logged_replay'],default='both');e.add_argument('--split',choices=['all','train','val','test'],default='all');e.add_argument('--max-per-scene',type=int);e.add_argument('--submission');e.add_argument('--execution',choices=['controller','executed'],default='controller')
    t=sp.add_parser('template');t.add_argument('--snapshot',required=True);t.add_argument('--out',required=True);t.add_argument('--split',choices=['all','train','val','test'],default='test')
    a=parser.parse_args()
    if a.cmd=='snapshot':
        kwargs={k:getattr(a,k) for k in ('root','db') if getattr(a,k)}
        if a.protocol:kwargs['protocol']=Protocol(**json.loads(Path(a.protocol).read_text()))
        m=build_snapshot(a.out,**kwargs);print(f"Snapshot: {m['snapshot_sha256']}; windows: {len(m['windows'])}")
    elif a.cmd=='evaluate':
        if a.max_per_scene is not None and a.max_per_scene<1:parser.error('--max-per-scene must be positive')
        evaluate(a.snapshot,a.out,('constant_velocity','logged_replay') if a.baseline=='both' else (a.baseline,),a.max_per_scene,a.split,a.submission,a.execution)
    else:
        snap=Snapshot(a.snapshot);doc={'name':'my_planner','snapshot_sha256':snap.manifest['snapshot_sha256'],'protocol_sha256':snap.protocol.identity(),'coordinate_frame':'scene_xy_m','predictions':{}}
        for w in snap.manifest['windows']:
            if a.split!='all' and w['split']!=a.split:continue
            doc['predictions'][w['id']]=baseline(snap.scene(w['scene_id']),w,snap.protocol,'constant_velocity')
        Path(a.out).write_text(json.dumps(doc,indent=2));print(a.out)
if __name__=='__main__':main()

"""Validated submissions, reference execution, and comparable leaderboards."""
import json,math,hashlib,importlib
from pathlib import Path
from dataclasses import asdict
import numpy as np
from ..benchmark.dataset import Snapshot,digest
from ..benchmark.scoring import trajectory_array,resample
from ..benchmark.runner import baseline
from ..benchmark_v2.agents import generate,ConstantVelocityAgent
from ..benchmark_v2.metrics import Settings
from ..benchmark_v2.tracker import TrackerConfig
from ..benchmark_v2.runner import adjacent_window,run as extended_run,confidence
from ..benchmark_v2.road_rules import RULES,run as road_run

def protocol_id(snap):
    return digest(dict(base=snap.protocol.identity(),extended=asdict(Settings()),tracker=asdict(TrackerConfig()),road=RULES,track='annotated_environment_planning'))

def predict(snapshot,out,checkpoint=None,split='test',agent=None):
    snap=Snapshot(snapshot)
    if checkpoint:
        from .learning import LearnedAgent
        agent=LearnedAgent(checkpoint)
        if agent.checkpoint['snapshot_sha256']!=snap.manifest['snapshot_sha256']:raise ValueError('checkpoint snapshot mismatch')
    d=generate(snapshot,out,agent or ConstantVelocityAgent(),split)
    d['sdk_protocol_sha256']=protocol_id(snap);d['track']='annotated_environment_planning'
    if checkpoint:d['checkpoint_sha256']=hashlib.sha256(Path(checkpoint).read_bytes()).hexdigest()
    Path(out).write_text(json.dumps(d,ensure_ascii=False,separators=(',',':'),allow_nan=False));validate(snapshot,out);return d

def validate(snapshot,submission):
    snap=Snapshot(snapshot);d=json.loads(Path(submission).read_text());split=d.get('split');ws=[w for w in snap.manifest['windows'] if w['split']==split]
    if split not in ('train','val','test'):raise ValueError('invalid split')
    if d.get('sdk_protocol_sha256')!=protocol_id(snap) or d.get('snapshot_sha256')!=snap.manifest['snapshot_sha256']:raise ValueError('submission protocol/snapshot mismatch')
    expected_extended=digest({'base_protocol_sha256':snap.protocol.identity(),'settings':asdict(Settings()),'tracker':asdict(TrackerConfig())})
    if d.get('extended_protocol_sha256')!=expected_extended:raise ValueError('execution protocol mismatch')
    if d.get('coordinate_frame')!='scene_xy_m' or d.get('track')!='annotated_environment_planning':raise ValueError('invalid track or coordinates')
    if set(d['predictions'])!={w['id'] for w in ws}:raise ValueError('must submit every window of the selected split exactly once')
    expected_next=set()
    for w in ws:
        sc=snap.scene(w['scene_id']);nw=adjacent_window(sc,w,.5)
        if nw:expected_next.add(w['id'])
        for key,window in (('predictions',w),('adjacent_predictions',nw)):
            if window is None:continue
            rows=d.get(key,{}).get(w['id'])
            if rows is None:raise ValueError('missing required adjacent prediction')
            a=trajectory_array(rows,4.);f=sc['frames'][window['frame_start']]
            if np.linalg.norm(a[0,1:3]-f['xy'])>.1:raise ValueError('initial position mismatch')
            dy=a[0,3]-f['yaw']
            if abs(math.atan2(math.sin(dy),math.cos(dy)))>.05:raise ValueError('initial heading mismatch')
    if set(d.get('adjacent_predictions',{}))!=expected_next:raise ValueError('unexpected adjacent prediction scope')
    return dict(valid=True,windows=len(ws),adjacent_windows=len(expected_next),protocol_sha256=protocol_id(snap),split=split)

def evaluate(snapshot,prior,submission,out):
    validation=validate(snapshot,submission);out=Path(out)
    if out.exists() and any(out.iterdir()):raise ValueError('evaluation output must be new')
    out.mkdir(parents=True,exist_ok=True)
    extended_run(snapshot,prior,out/'execution',submission=submission)
    road_run(snapshot,out/'execution/results.json',out/'road')
    snap=Snapshot(snapshot);d=json.loads((out/'road/results.json').read_text());windows={w['id']:w for w in snap.manifest['windows']}
    for name,method in d['methods'].items():
        errors=[];by_category={}
        for r in method['results']:
            if 'planned_trajectory' not in r:continue
            w=windows[r['window_id']];sc=snap.scene(w['scene_id']);ts=np.arange(.25,4.001,.25);gt=resample(np.asarray(baseline(sc,w,snap.protocol,'logged_replay')),ts);pr=resample(np.asarray(r['planned_trajectory']),ts);distance=np.linalg.norm(gt[:,1:3]-pr[:,1:3],axis=1)
            r['open_loop']={'ade_m':float(distance.mean()),'fde_m':float(distance[-1])};errors.append(r['open_loop']);by_category.setdefault(w['category'],[]).append(r)
        method['sdk_summary']={'open_loop':{k:float(np.mean([e[k] for e in errors])) if errors else None for k in ('ade_m','fde_m')},'scored_windows':len(d['road_ranking_window_ids']),'submitted_windows':len(method['results']),'category_scores':{k:confidence(rows,'road_rule_score',set(d['road_ranking_window_ids'])) for k,rows in by_category.items()},'failures':sum('submission_failure' in r for r in method['results'])}
    d.update(sdk_implementation_files={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in Path(__file__).parent.glob('*.py')},sdk_protocol_sha256=validation['protocol_sha256'],split=validation['split'],submission_sha256=hashlib.sha256(Path(submission).read_bytes()).hexdigest(),track='annotated_environment_planning')
    (out/'results.json').write_text(json.dumps(d,ensure_ascii=False,separators=(',',':'),allow_nan=False));(out/'validation.json').write_text(json.dumps(validation,indent=2));return d

def leaderboard(files,out):
    entries=[];identity=None
    for filename in files:
        d=json.loads(Path(filename).read_text());ids=sorted(d['road_ranking_window_ids']);key=(d['snapshot_sha256'],d['sdk_protocol_sha256'],d['split'],d['track'],digest(ids))
        if identity is not None and key!=identity:raise ValueError('leaderboard cannot mix protocols, splits, tracks or evaluation cohorts')
        identity=key
        for name,m in d['methods'].items():entries.append(dict(name=name,score=confidence(m['results'],'road_rule_score',set(ids)),diagnostic=name=='logged_replay',details=m.get('sdk_summary',{}),result_sha256=hashlib.sha256(Path(filename).read_bytes()).hexdigest()))
    entries.sort(key=lambda x:(x['diagnostic'],x['score']['mean'] is None,-(x['score']['mean'] or 0)))
    report=dict(identity=identity,entries=entries);Path(out).write_text(json.dumps(report,ensure_ascii=False,indent=2));return report

"""Participant inputs are exported separately from evaluator-only snapshots."""
import gzip,json,hashlib
from pathlib import Path
from dataclasses import asdict
from ..benchmark.dataset import Snapshot,digest
from ..benchmark_v2.agents import agent_input,ConstantVelocityAgent
from ..benchmark_v2.runner import adjacent_window
from ..benchmark_v2.metrics import Settings
from ..benchmark_v2.tracker import TrackerConfig
from .data import target
from .workflow import protocol_id

def export_inputs(snapshot,out):
    snap=Snapshot(snapshot);out=Path(out)
    if out.exists() and any(out.iterdir()):raise ValueError('input export must be new')
    out.mkdir(parents=True,exist_ok=True)
    meta=dict(snapshot_sha256=snap.manifest['snapshot_sha256'],sdk_protocol_sha256=protocol_id(snap),extended_protocol_sha256=digest({'base_protocol_sha256':snap.protocol.identity(),'settings':asdict(Settings()),'tracker':asdict(TrackerConfig())}),coordinate_frame='scene_xy_m',track='annotated_environment_planning',files={})
    for split in ('train','val','test'):
        path=out/f'{split}.jsonl.gz';count=0
        with gzip.open(path,'wt',encoding='utf-8') as stream:
            for w in snap.manifest['windows']:
                if w['split']!=split:continue
                sc=snap.scene(w['scene_id']);nw=adjacent_window(sc,w,.5)
                row=dict(window_id=w['id'],scene_id=w['scene_id'],input=agent_input(sc,w),adjacent_input=agent_input(sc,nw) if nw else None)
                if split!='test':row['target_local_xy']=target(sc,w,snap.protocol).tolist()
                stream.write(json.dumps(row,ensure_ascii=False,separators=(',',':'))+'\n');count+=1
        meta['files'][path.name]=dict(sha256=hashlib.sha256(path.read_bytes()).hexdigest(),windows=count);print('exported',split,count,flush=True)
    (out/'manifest.json').write_text(json.dumps(meta,indent=2));return meta

def infer(inputs,out,split='test',agent=None):
    root=Path(inputs);meta=json.loads((root/'manifest.json').read_text());p=root/f'{split}.jsonl.gz'
    if hashlib.sha256(p.read_bytes()).hexdigest()!=meta['files'][p.name]['sha256']:raise ValueError('export checksum mismatch')
    agent=agent or ConstantVelocityAgent()
    if hasattr(agent,'checkpoint') and agent.checkpoint['snapshot_sha256']!=meta['snapshot_sha256']:raise ValueError('checkpoint/input snapshot mismatch')
    d={k:v for k,v in meta.items() if k!='files'};d.update(name=type(agent).__name__,split=split,predictions={},adjacent_predictions={},input_track='past_observations_plus_local_prior_HD_map')
    with gzip.open(p,'rt',encoding='utf-8') as stream:
        for line in stream:
            r=json.loads(line);wid=r['window_id']
            # Labels never enter predict(), including on train/val exports.
            d['predictions'][wid]=agent.predict(r['input'])
            if r['adjacent_input'] is not None:d['adjacent_predictions'][wid]=agent.predict(r['adjacent_input'])
    Path(out).write_text(json.dumps(d,separators=(',',':'),allow_nan=False));return {'windows':len(d['predictions'])}

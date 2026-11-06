"""Causal planner input adapter: no future actors, future ego, or reference trajectory."""
import argparse,importlib,json,time
from pathlib import Path
from dataclasses import asdict
import numpy as np
from shapely.geometry import shape,box
from ..benchmark.dataset import Snapshot,digest
from .metrics import Settings
from .tracker import TrackerConfig
from .runner import adjacent_window

def agent_input(scene,w,history_s=1.):
    now=w['start_s'];rows=[f for f in scene['frames'] if now-history_s-1e-8<=f['t']<=now+1e-8 and f['key']==w['source_key']]
    if not rows or abs(rows[-1]['t']-now)>.01:raise ValueError('current observation missing')
    ego=rows[-1];xy=ego['xy'];region=box(xy[0]-80,xy[1]-80,xy[0]+80,xy[1]+80)
    return {'coordinate_frame':'scene_xy_m','current_time_s':now,'horizon_s':4.,'history':[{'t':f['t']-now,'xy':f['xy'],'yaw':f['yaw'],'objects':f['objects'],'dynamic_available':f['dynamic_available'],'source_image':f"frames/{f['key']}/{f['index']:03d}_raw.jpg"} for f in rows],'static_map':[f for f in scene['features'] if shape(f['geometry']).intersects(region)],'map_access':'local_prior_HD_map; separate from camera-only planning track'}

class ConstantVelocityAgent:
    def predict(self,inputs):
        rows=inputs['history'];current=rows[-1];v=0.
        if len(rows)>1:v=float(np.linalg.norm(np.array(current['xy'])-rows[-2]['xy'])/(current['t']-rows[-2]['t']))
        yaw=current['yaw'];x,y=current['xy']
        return [[float(t),float(x+v*t*np.cos(yaw)),float(y+v*t*np.sin(yaw)),yaw] for t in np.arange(0,4.001,.25)]

def generate(snapshot,out,agent=None,split='test'):
    snap=Snapshot(snapshot);agent=agent or ConstantVelocityAgent();s=Settings();times=[]
    doc={'name':type(agent).__name__,'snapshot_sha256':snap.manifest['snapshot_sha256'],'extended_protocol_sha256':digest({'base_protocol_sha256':snap.protocol.identity(),'settings':asdict(s),'tracker':asdict(TrackerConfig())}),'coordinate_frame':'scene_xy_m','split':split,'predictions':{},'adjacent_predictions':{},'input_track':'past_observations_plus_local_prior_HD_map'}
    for w in snap.manifest['windows']:
        if w['split']!=split:continue
        scene=snap.scene(w['scene_id']);inp=agent_input(scene,w);start=time.perf_counter();doc['predictions'][w['id']]=agent.predict(inp);times.append(time.perf_counter()-start)
        nw=adjacent_window(scene,w,s.adjacent_plan_offset_s)
        if nw:doc['adjacent_predictions'][w['id']]=agent.predict(agent_input(scene,nw))
    doc['inference_timing']={'mean_ms':1000*float(np.mean(times)) if times else None,'p95_ms':1000*float(np.quantile(times,.95)) if times else None,'calls':len(times),'scope':'predict_only; excludes input loading; no GPU synchronization for custom agents'}
    Path(out).write_text(json.dumps(doc,ensure_ascii=False,separators=(',',':')));return doc

def main():
    p=argparse.ArgumentParser();p.add_argument('--snapshot',required=True);p.add_argument('--out',required=True);p.add_argument('--split',choices=['train','val','test'],default='test');p.add_argument('--agent',help='module:Class with predict(inputs), instantiated without arguments');a=p.parse_args();agent=None
    if a.agent:
        module,name=a.agent.split(':',1);agent=getattr(importlib.import_module(module),name)()
    d=generate(a.snapshot,a.out,agent,a.split);print(f"Generated {len(d['predictions'])} causal predictions and {len(d['adjacent_predictions'])} adjacent predictions")
if __name__=='__main__':main()

"""Extend frozen v1 evaluations; supports fair LQR baselines and paired-plan submissions."""
import json,argparse,hashlib,math,csv
from pathlib import Path
from collections import defaultdict
from dataclasses import asdict
import numpy as np
from ..benchmark.dataset import Snapshot,digest
from ..benchmark.runner import baseline
from ..benchmark.scoring import score_window,resample,trajectory_array
from .metrics import Settings,comfort,extended_comfort,lane_model,lane_keeping,route_direction,collision_types,verified_map_metrics,reference_filter
from .tracker import rollout,TrackerConfig


def initial_speed(scene,w):
    i=w['frame_start'];fs=scene['frames']
    if i<1 or fs[i-1]['key']!=w['source_key']:raise ValueError('observed history required')
    return float(np.linalg.norm(np.array(fs[i]['xy'])-fs[i-1]['xy'])/(fs[i]['t']-fs[i-1]['t']))

def adjacent_window(scene,w,offset):
    times=np.array([f['t'] for f in scene['frames']]);target=w['start_s']+offset;idx=int(np.argmin(abs(times-target)))
    if abs(times[idx]-target)>.01:return None
    end=target+4;rows=[f for f in scene['frames'] if target-.001<=f['t']<=end+.001]
    if not rows or rows[-1]['t']<end-.01 or any(f['key']!=w['source_key'] for f in rows) or max(np.diff([f['t'] for f in rows]),default=0)>.5:return None
    return {**w,'frame_start':idx,'start_s':target,'end_s':end,'id':w['id']+':adjacent'}

def core_score(metrics,hc):
    if any(metrics.get(k)==0 for k in ('no_collision','white_compliance','yellow_compliance','annotated_exclusion_compliance')):return 0.
    values=[metrics.get(k) for k in ('no_collision','ttc_compliance','progress')]+[hc]
    if any(v is None for v in values):return None
    return 100*(5*metrics['progress']+5*metrics['ttc_compliance']+2*hc)/12

def complete_score(base,extra):
    gates=[extra.get(k) for k in ('at_fault_collision_proxy','drivable_area_compliance','driving_direction_compliance','traffic_light_compliance')]+[base.get('white_compliance'),base.get('yellow_compliance')]
    weighted=[base.get('progress'),base.get('ttc_compliance'),extra.get('lane_keeping'),extra.get('history_comfort'),extra.get('extended_comfort')]
    if any(v is None for v in gates+weighted):return None
    return float(100*np.prod(gates)*np.dot([5,5,2,2,2],weighted)/16)

def confidence(rows,field,ids,seed=20261018):
    # Scene-group bootstrap: correlated windows are sampled together.
    groups=defaultdict(list)
    selected=[r for r in rows if r['window_id'] in ids]
    if not selected or any(r.get(field) is None for r in selected):return {'mean':None,'ci95':None,'windows':len(selected),'scenes':len({r['scene_id'] for r in selected})}
    for r in selected:groups[r['scene_id']].append(r[field])
    vals=[np.array(v,float) for v in groups.values()];rng=np.random.default_rng(seed)
    means=[float(np.concatenate([vals[i] for i in rng.integers(0,len(vals),len(vals))]).mean()) for _ in range(500)]
    return {'mean':float(np.mean([r[field] for r in selected])),'ci95':np.quantile(means,[.025,.975]).tolist(),'windows':len(selected),'scenes':len(groups),'resampling_unit':'complete_scene'}

def run(snapshot,prior,out,mode='reuse',submission=None,layers=None):
    snap=Snapshot(snapshot);p=snap.protocol;s=Settings();prior=json.loads(Path(prior).read_text());out=Path(out);out.mkdir(parents=True,exist_ok=True)
    if prior['protocol_sha256']!=p.identity():raise ValueError('base protocol mismatch')
    if prior['snapshot_sha256']!=snap.manifest['snapshot_sha256']:raise ValueError('v1 results snapshot mismatch')
    windows={w['id']:w for w in snap.manifest['windows']};old_methods=prior['methods'];supplement=json.loads(Path(layers).read_text()) if layers else {}
    if supplement and supplement.get('snapshot_sha256')!=prior['snapshot_sha256']:raise ValueError('supplementary map snapshot mismatch')
    supplemental=supplement.get('windows',{})
    if submission:
        sub=json.loads(Path(submission).read_text())
        expected=digest({'base_protocol_sha256':p.identity(),'settings':asdict(s),'tracker':asdict(TrackerConfig())})
        if sub.get('snapshot_sha256')!=prior['snapshot_sha256'] or sub.get('extended_protocol_sha256')!=expected or sub.get('coordinate_frame')!='scene_xy_m':raise ValueError('submission identity/coordinate mismatch')
        predictions=sub['predictions'];next_predictions=sub.get('adjacent_predictions',{});name=sub.get('name','planner')
        split=sub.get('split','test');chosen=[w for w in windows.values() if w['split']==split]
        if split not in ('train','val','test'):raise ValueError('submission split required')
        if set(predictions)-{w['id'] for w in chosen} or set(next_predictions)-{w['id'] for w in chosen}:raise ValueError('out of scope window IDs')
        old_methods={name:{'results':[{'window_id':w['id']} for w in chosen]}};mode='lqr'
    output={};models={};all_examples=[]
    ref_cache={}
    for name,old in old_methods.items():
        results=[]
        for n,previous in enumerate(old['results']):
            w=windows[previous['window_id']];scene=snap.scene(w['scene_id']);raw_plan=None;next_plan=None;controls=None
            try:
                if submission:
                    if w['id'] not in predictions:raise ValueError('missing prediction')
                    raw_plan=predictions[w['id']]
                else:raw_plan=baseline(scene,w,p,name)
                raw_array=trajectory_array(raw_plan,p.horizon_s);f=scene['frames'][w['frame_start']]
                if np.linalg.norm(raw_array[0,1:3]-f['xy'])>.1 or abs(math.atan2(math.sin(raw_array[0,3]-f['yaw']),math.cos(raw_array[0,3]-f['yaw'])))>.05:raise ValueError('initial pose mismatch')
                if mode=='lqr':
                    roll=rollout(raw_plan,initial_speed(scene,w));path=roll['trajectory'];controls=roll['controls'];base=score_window(scene,w,path,p,roll['execution_status'])
                else:
                    if 'trajectory' not in previous:raise ValueError('prior result has no trajectory')
                    base=previous;path=base['trajectory']
                nw=adjacent_window(scene,w,s.adjacent_plan_offset_s)
                if nw:
                    if submission:next_plan=next_predictions.get(w['id'])
                    else:next_plan=baseline(scene,nw,p,name)
                    if next_plan is not None:
                        na=trajectory_array(next_plan,p.horizon_s);nf=scene['frames'][nw['frame_start']]
                        if np.linalg.norm(na[0,1:3]-nf['xy'])>.1 or abs(math.atan2(math.sin(na[0,3]-nf['yaw']),math.cos(na[0,3]-nf['yaw'])))>.05:raise ValueError('adjacent initial pose mismatch')
                if w['id'] not in models:models[w['id']]=lane_model(scene,w)
                hc=comfort(scene,w,path,s);ec=extended_comfort(raw_plan,next_plan,s.adjacent_plan_offset_s,s);lk=lane_keeping(base['trajectory'],models[w['id']],s);direction=route_direction(scene,w,base['trajectory'],s);collision=collision_types(scene,w,base['trajectory'],p,base);verified=verified_map_metrics(base['trajectory'],supplemental.get(w['id']),p)
                ext={**verified,'history_comfort':hc['history_comfort'],'extended_comfort':ec['extended_comfort'],'lane_keeping_proxy':lk['lane_keeping_proxy'],'route_direction_proxy':direction['route_direction_proxy'],'at_fault_collision_proxy':collision['at_fault_collision_proxy']}
                score=core_score(base['metrics'],ext['history_comfort'])
                # Human reference is evaluated under the same execution backend, but original events stay intact.
                if w['id'] not in ref_cache:
                    human_plan=baseline(scene,w,p,'logged_replay');human_path=rollout(human_plan,initial_speed(scene,w))['trajectory'] if mode=='lqr' else human_plan
                    human_base=score_window(scene,w,human_path,p);human_hc=comfort(scene,w,human_path,s);ref_cache[w['id']]={**human_base['metrics'],'history_comfort':human_hc['history_comfort']}
                filtered,waived=reference_filter({**base['metrics'],'history_comfort':ext['history_comfort']},ref_cache[w['id']],('no_collision','ttc_compliance','history_comfort'))
                result={**base,'v1_score':base['score'],'extended_metrics':ext,'extended_core_score':score,'full_extended_score':complete_score(base['metrics'],ext),'reference_filtered_score':core_score(filtered,filtered['history_comfort']),'reference_filter_waived':waived,'extended_details':{'comfort':hc,'consistency':ec,'lane':lk,'direction':direction,'collision':collision},'planned_trajectory':raw_plan,'adjacent_planned_trajectory':next_plan,'controls':controls}
            except (ValueError,KeyError,TypeError,IndexError) as exc:
                result={'window_id':w['id'],'scene_id':w['scene_id'],'category':w['category'],'extended_core_score':0.,'full_extended_score':0.,'reference_filtered_score':0.,'extended_metrics':{},'submission_failure':str(exc),'events':[]}
            results.append(result)
            if (n+1)%100==0:print(f'{mode}/{name}: {n+1}/{len(old["results"])}',flush=True)
        output[name]={'results':results}
    # Scope is based on source annotation and available history, not the candidate's infractions.
    comparable=set(prior['ranking_window_ids'])
    hist_ok={}
    for wid in {r['window_id'] for m in output.values() for r in m['results']}:
        w=windows[wid];sc=snap.scene(w['scene_id']);hist_ok[wid]=comfort(sc,w,baseline(sc,w,p,'logged_replay'),s)['history_complete']
    common=comparable&{wid for wid,ok in hist_ok.items() if ok}
    # All participating methods must submit the same scope. Missing submissions remain zero.
    scope=set.intersection(*[{r['window_id'] for r in m['results']} for m in output.values()])
    common&=scope
    for name,method in output.items():
        rows=method['results'];metrics={}
        for key in ('history_comfort','extended_comfort','lane_keeping_proxy','route_direction_proxy','at_fault_collision_proxy','drivable_area_compliance','lane_keeping','driving_direction_compliance','traffic_light_compliance'):
            values=[r['extended_metrics'].get(key) for r in rows if r['extended_metrics'].get(key) is not None];metrics[key]={'available':len(values),'mean':float(np.mean(values)) if values else None}
        method['summary']={'strict':confidence(rows,'extended_core_score',common),'reference_filtered':confidence(rows,'reference_filtered_score',common),'metrics':metrics,'failures':sum('submission_failure' in r for r in rows),'windows':len(rows),'scene_macro':float(np.mean([np.mean([r['extended_core_score'] for r in rows if r['window_id'] in common and r['scene_id']==sid]) for sid in {r['scene_id'] for r in rows if r['window_id'] in common}])) if common else None}
    implementation={str(f.relative_to(Path(__file__).parents[1])):hashlib.sha256(f.read_bytes()).hexdigest() for folder in [Path(__file__).parent,Path(__file__).parents[1]/'benchmark'] for f in folder.glob('*.py')}
    result={'implementation_sha256':digest(implementation),'implementation_files':implementation,'format':'extremedrive-extended-evaluation-v2','mode':mode,'snapshot_sha256':snap.manifest['snapshot_sha256'],'base_protocol_sha256':p.identity(),'settings':asdict(s),'tracker_config':asdict(TrackerConfig()),'extended_protocol_sha256':digest({'base_protocol_sha256':p.identity(),'settings':asdict(s),'tracker':asdict(TrackerConfig())}),'comparable_window_ids':sorted(common),'comparable_sha256':digest(sorted(common)),'supplementary_sha256':digest(supplement),'methods':output,'not_official_navsim_score':True,'unavailable_requirements':{'drivable_area':'verified full drivable polygons','lane_direction':'verified directed lanes/intersections','traffic_lights':'time-aligned signal state and stop-line binding','two_stage_real_scene_score':'scored off-policy followup branches and matching sensor context'}}
    (out/'results.json').write_text(json.dumps(result,ensure_ascii=False,separators=(',',':'),allow_nan=False))
    (out/'lane_models.json').write_text(json.dumps(models,ensure_ascii=False,separators=(',',':')))
    brief={k:v for k,v in result.items() if k!='methods'};brief['methods']={k:v['summary'] for k,v in output.items()};(out/'summary.json').write_text(json.dumps(brief,ensure_ascii=False,indent=2))
    with (out/'scores.csv').open('w',encoding='utf-8-sig',newline='') as file:
        fields=['method','window_id','scene_id','extended_core_score','reference_filtered_score','full_extended_score','history_comfort','extended_comfort','lane_keeping_proxy','route_direction_proxy','at_fault_collision_proxy','submission_failure'];writer=csv.DictWriter(file,fieldnames=fields);writer.writeheader()
        for name,m in output.items():
            for row in m['results']:
                flat={**row,**row['extended_metrics'],'method':name};writer.writerow({k:flat.get(k) for k in fields})
    print(json.dumps(brief['methods'],ensure_ascii=False),flush=True)
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--snapshot',required=True);p.add_argument('--prior',required=True);p.add_argument('--out',required=True);p.add_argument('--mode',choices=['reuse','lqr'],default='reuse');p.add_argument('--submission');p.add_argument('--layers');a=p.parse_args();run(a.snapshot,a.prior,a.out,a.mode,a.submission,a.layers)
if __name__=='__main__':main()

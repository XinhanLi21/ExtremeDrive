"""Fresh trajectory -> LQR -> fitted dynamics -> geometry and risk evaluation."""
import json
from dataclasses import replace
from pathlib import Path
import numpy as np
from scipy.interpolate import PchipInterpolator
from .data import load,digest,prediction_digest
from .actors import AdjacentActors
from .ttc import revise_ttc
from .friction import resolve_friction, table_identity
from ..benchmark import scoring
from ..benchmark.protocol import Protocol
from ..benchmark.road_rules import evaluate as road_evaluate
from ..benchmark.aggregation import scoped_total, road_compliance
from ..benchmark.emergency import emergency_safety
from ..execution.comfort import dynamic_comfort
from ..execution.mohe_lateral_profile import load_candidate
from ..execution.navsim_dynamic import execute_official_dynamic

def execute(dataset,pred):
    ctx=dataset.context(pred['token']);a=ctx['anchor'];t=np.r_[0.,pred['times_s']];q=np.vstack([np.zeros(3),pred['poses']]);yaw=a['yaw'];co,si=np.cos(yaw),np.sin(yaw)
    world=q[:,:2]@np.array([[co,si],[-si,co]])+a['xy'];heading=q[:,2]+yaw
    speed=np.hypot(*[PchipInterpolator(t,world[:,k]).derivative()(t) for k in (0,1)])
    state=dict(x=a['xy'][0],y=a['xy'][1],yaw=yaw,vx=float(ctx['sensor_state']['velocity'][0]),vy=float(ctx['sensor_state']['velocity'][1]),yaw_rate=ctx['measured_initial_yaw_rate'])
    friction=resolve_friction(dataset.assets,ctx,pred['token'])
    config,meta=load_candidate(dataset.assets/'mohe_profile.json',experimental=True)
    config=replace(config,mu=friction['mu']).validate()
    meta['friction']=friction
    run=execute_official_dynamic(dict(id=pred['token'],trajectory=np.column_stack([t,world,heading,speed]).tolist(),initial_state=state),config)
    run['scene_friction']=friction;run['geometry']=run['geometry'].tolist();run['candidate_profile']=meta;run['initial_state_supplied']=state;run['source_prediction']=pred
    return run

def score(dataset,pred,run):
    ctx=dataset.context(pred['token']);protocol=load(dataset.assets/'protocol.json');p=Protocol(**protocol).validate()
    identity={'scene_friction':run.get('scene_friction'),'token':pred['token'],'prediction_sha256':prediction_digest(pred),'protocol_sha256':digest(protocol),'context_sha256':digest(ctx),'input_policy':pred.get('input_policy','unspecified'),'review_policy':'automatic_only_v1'}
    if run['failure']:return {**identity,'score':0.,'status':'execution_failed','failure':run['failure']}
    plan=[[l['t'],l['x'],l['y'],l['yaw']] for l in run['logs']]
    original=scoring.Actors
    try:
        scoring.Actors=AdjacentActors
        r=scoring.score_window(ctx['scene'],ctx['window'],plan,p,execution_status=run['execution_status'])
    finally:scoring.Actors=original
    r['road_metrics']=road_evaluate(r['trajectory'],ctx['road'],p)
    r['metrics']['road_compliance']=road_compliance(r['metrics'],r['road_metrics'])
    r=revise_ttc(r,ctx,p);r['future_comfort']=dynamic_comfort(run['logs'],p.comfort)
    applicable=ctx['window']['category']!='Critical_Driving'
    r['metrics']['comfort']=r['future_comfort']['score'] if applicable else None
    r['comfort_applicability']={'applicable':applicable,'reason':'noncritical_test' if applicable else 'emergency_maneuver'}
    r['emergency_safety']=emergency_safety(r)
    r['score']=scoped_total(r['metrics'],r['road_metrics'],r['future_comfort']['score'] if applicable else None,applicable)
    r.update(identity,status='automated',manual_review_applied=False,score_name='ExtremeDrive automatic execution score',score_status='unavailable' if r['score'] is None else 'evaluated')
    return r

def summarize(records):
    fields={'score':lambda r:r.get('score'),'no_collision':lambda r:r.get('metrics',{}).get('no_collision'),'road_compliance':lambda r:road_compliance(r.get('metrics',{}),r.get('road_metrics',{})),'ttc_compliance':lambda r:r.get('metrics',{}).get('ttc_compliance'),'progress':lambda r:r.get('metrics',{}).get('progress')}
    result={'windows':len(records),'metrics':{}}
    for key,fn in fields.items():
        values=[fn(r) for r in records];known=[v for v in values if v is not None];n=len(values);hi=100. if key=='score' else 1.
        result['metrics'][key]={'mean':float(np.mean(known)) if len(known)==n and n else None,'known_mean':float(np.mean(known)) if known else None,'unknown':n-len(known),'lower':sum(known)/n if n else None,'upper':(sum(known)+(n-len(known))*hi)/n if n else None}
    for key,selected,fn in [
        ('comfort', [r for r in records if r.get('comfort_applicability',{}).get('applicable',r.get('category')!='Critical_Driving')], lambda r:r.get('future_comfort',{}).get('score')),
        ('emergency_safety_success', [r for r in records if r.get('emergency_safety',{}).get('applicable')], lambda r:r['emergency_safety'].get('success'))]:
        values=[fn(r) for r in selected];known=[float(v) for v in values if v is not None];n=len(values)
        result['metrics'][key]={'applicable_windows':n,'unknown':n-len(known),'mean':float(np.mean(known)) if n and len(known)==n else None,'lower':sum(known)/n if n else None,'upper':(sum(known)+n-len(known))/n if n else None}
    return result

def write(path,obj):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(obj,ensure_ascii=False,indent=2,allow_nan=False)+'\n')


def evaluation_identity(dataset,preds,records):
    policies={p.get('input_policy','unspecified') for p in preds.values()}
    if len(policies)!=1:raise ValueError('Mixed input policies cannot be aggregated')
    policy=next(iter(policies));complete=set(preds)==set(dataset.tokens(True))
    if any(r.get('manual_review_applied',False) for r in records):
        raise ValueError('Reviewed results cannot enter the automatic benchmark')
    return dict(mode='fresh_automated_execution',review_policy='automatic_only_v1',
                input_policy=policy,complete_cohort=complete,
                benchmark_eligible=complete and policy=='sensor_only',
                protocol_sha256=digest(load(dataset.assets/'protocol.json')),
                cohort_sha256=digest(sorted(preds)),
                asset_manifest_sha256=digest(load(dataset.assets/'manifest.json')),
                scene_friction_sha256=table_identity(dataset.assets))


def compare_summaries(summaries):
    """Reject archived, incomplete, ambiguous, or differently configured runs."""
    if not summaries:raise ValueError('No runs supplied')
    keys=('mode','review_policy','input_policy','trajectory_hz','protocol_sha256',
          'cohort_sha256','asset_manifest_sha256','scene_friction_sha256')
    first=summaries[0]
    for s in summaries:
        if not s.get('benchmark_eligible') or not s.get('complete_cohort') or s.get('mode')!='fresh_automated_execution' or s.get('review_policy')!='automatic_only_v1' or s.get('input_policy')!='sensor_only':
            raise ValueError('Only complete automatic sensor-only runs qualify for formal comparison')
        if any(k not in s or s[k]!=first.get(k) for k in keys):
            raise ValueError('Evaluation identities differ; do not mix protocols, inputs or rates')
    return summaries

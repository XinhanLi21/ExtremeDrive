"""Re-score archived predictions using verified rollouts, without review overrides."""
import argparse,json,hashlib
from pathlib import Path
from dataclasses import asdict
from concurrent.futures import ProcessPoolExecutor,as_completed
import numpy as np
from extremebench.toolkit.data import Dataset,load,read_predictions,prediction_digest,safe_token,digest
from extremebench.toolkit.evaluation import score,summarize,write,evaluation_identity
from extremebench.execution.mohe_lateral_profile import load_candidate

def job(args):
 assets,base,out,model,pred=args;ds=Dataset(assets);name=safe_token(pred['token'])+'.json'
 options=[Path(base)/p/'rollouts'/model/name for p in ['expanded_critical_20261105','critical_audit_20261103']]
 path=next((p for p in options if p.exists()),None)
 if path is None:raise FileNotFoundError((model,pred['token']))
 run=load(path);cfg,meta=load_candidate(Path(assets)/'mohe_profile.json',experimental=True)
 assert run['config']==asdict(cfg),'Vehicle configuration mismatch'
 assert run['candidate_profile']['profile_sha256']==meta['profile_sha256']
 assert prediction_digest(run['source_prediction'])==prediction_digest(pred),'Prediction mismatch'
 ctx=ds.context(pred['token']);a=ctx['anchor'];expected=dict(x=a['xy'][0],y=a['xy'][1],yaw=a['yaw'],vx=float(ctx['sensor_state']['velocity'][0]),vy=float(ctx['sensor_state']['velocity'][1]),yaw_rate=ctx['measured_initial_yaw_rate'])
 for k,v in expected.items():np.testing.assert_allclose(run['initial_state_supplied'][k],v,atol=1e-10,rtol=0)
 r=score(ds,pred,run);r.update(model=model,rollout_reused=True,rollout_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),manual_review_applied=False)
 write(Path(out)/model/'scores'/name,r);return model,r

def main():
 p=argparse.ArgumentParser();p.add_argument('--assets',type=Path,default=Path('assets'));p.add_argument('--evaluation-root',required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--workers',type=int,default=4);a=p.parse_args();ds=Dataset(a.assets);jobs=[];predictions={};records={}
 for d in sorted((a.assets/'reference').iterdir()):
  if not d.is_dir():continue
  preds=read_predictions(d/'predictions.jsonl',ds.tokens(True));policy='scenario_hint' if d.name in ('transfuser','diffusiondrive') else 'sensor_only'
  for r in preds.values():r['input_policy']=policy
  predictions[d.name]=preds;records[d.name]=[];jobs.extend((str(a.assets.resolve()),a.evaluation_root,str(a.out),d.name,r) for r in preds.values())
 with ProcessPoolExecutor(max_workers=a.workers) as pool:
  futures=[pool.submit(job,j) for j in jobs]
  for i,f in enumerate(as_completed(futures)):
   model,r=f.result();records[model].append(r)
   if i%40==0:print('rescored',i+1,len(jobs),flush=True)
 summaries={}
 for model,rs in records.items():
  result=summarize(rs);result.update(evaluation_identity(ds,predictions[model],rs),trajectory_hz=2,execution_source='verified cached rollout; automatic geometry and TTC recomputed',input_policy_basis='original adapter source audit',model=model)
  summaries[model]=result;write(a.out/model/'summary.json',result)
 write(a.out/'summary.json',summaries);print('completed',len(jobs),flush=True)
if __name__=='__main__':main()

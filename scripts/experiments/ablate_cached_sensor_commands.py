"""Paired checkpoint inference using original frozen camera/LiDAR embeddings.
Run in the original sensor experiment environment; no training or GPU needed.
Both conditions use the same checkpoint, cache, batch order and random seeds.
"""
import argparse,json,sys,hashlib,time,re
from pathlib import Path
import torch

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--experiment-root',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--model',choices=['transfuser','diffusiondrive'],required=True);a=p.parse_args()
 sys.path.insert(0,str(a.experiment_root/'scripts/paper_suite'))
 from sensor_model import build
 from common import SUITE,rows,digest
 torch.set_num_threads(2);torch.manual_seed(2026)
 model,audit=build(a.model);checkpoint=SUITE/'runs'/a.model/'best.pt';model.load_state_dict(torch.load(checkpoint,map_location='cpu',weights_only=True),strict=True);model.eval();checkpoint_hash=sha(checkpoint)
 identity=hashlib.sha256((audit['checkpoint_sha256']+digest(a.experiment_root/'scripts/paper_suite/sensor_data.py')+digest(SUITE/'data/sensor_states.json')).encode()).hexdigest()
 records=rows('test');results={k:[] for k in ('scenario_hint','sensor_only','no_maneuver_hint')};start=time.time();changed=0;maxdiff=0.;errors={k:[] for k in results}
 with torch.no_grad():
  for off in range(0,len(records),8):
   group=records[off:off+8];loaded=[]
   for row in group:
    key=hashlib.sha256((identity+row['token']).encode()).hexdigest();v=torch.load(SUITE/'sensor_features'/a.model/(key+'.pt'),map_location='cpu',weights_only=True)
    assert v['token']==row['token'] and v['identity']==identity;loaded.append(v)
   up=torch.stack([v['up'] for v in loaded]);bev=torch.stack([v['bev'] for v in loaded]);status=torch.stack([v['status'] for v in loaded]);pair={}
   model._backbone.forward=lambda camera,lidar:(up,bev,None)
   for mode in results:
    state=status.clone()
    if mode=='sensor_only':state[:,:4]=0
    elif mode=='no_maneuver_hint':
     mask=torch.tensor([bool(re.search(r'emergen|avoid|collision|evasive|obstacle',r['language'],re.I)) for r in group]);state[mask,:4]=0
    torch.manual_seed(2026+off)
    pred=model({'camera_feature':None,'lidar_feature':None,'status_feature':state})['trajectory'];assert torch.isfinite(pred).all();pair[mode]=pred
    for row,q in zip(group,pred):
     poses=q.tolist();results[mode].append(dict(token=row['token'],times_s=[.5*i for i in range(1,9)],poses=poses,input_policy=mode,model=a.model,checkpoint_sha256=checkpoint_hash))
     e=(q[:,:2]-torch.tensor(row['target'])[:,:2]).norm(dim=-1);errors[mode].append(dict(token=row['token'],ADE_m=float(e.mean()),FDE_m=float(e[-1])))
   diff=(pair['scenario_hint']-pair['sensor_only']).abs();maxdiff=max(maxdiff,float(diff.max()));changed+=int((diff.flatten(1).max(1).values>1e-6).sum())
   if off%64==0:print(a.model,off,len(records),round(time.time()-start,1),flush=True)
 a.out.mkdir(parents=True,exist_ok=True)
 for mode,rs in results.items():
  (a.out/(mode+'.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in rs))
 summary=dict(model=a.model,checkpoint_sha256=checkpoint_hash,windows=len(records),changed_predictions=changed,max_abs_pose_delta=maxdiff,paired_seed='2026 + batch_offset; reset identically for both conditions',cache_identity=identity,source_adapter_sha256=sha(a.experiment_root/'scripts/paper_suite/sensor_data.py'),scope='inference-only hint removal; checkpoint was trained with original scenario commands; not a no-hint retraining result',elapsed_s=time.time()-start,metrics={k:{'ADE_m':sum(r['ADE_m'] for r in v)/len(v),'FDE_m':sum(r['FDE_m'] for r in v)/len(v)} for k,v in errors.items()})
 (a.out/'ablation.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
if __name__=='__main__':main()

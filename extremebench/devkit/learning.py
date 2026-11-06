"""CPU-reproducible imitation-learning reference. Optional torch dependency."""
from pathlib import Path
import json,random,hashlib
import numpy as np
import torch
from torch import nn
from .data import features,load_cache,FEATURE_VERSION,TIMES

class Planner(nn.Module):
    def __init__(self):
        super().__init__();self.net=nn.Sequential(nn.Linear(60,128),nn.ReLU(),nn.Linear(128,128),nn.ReLU(),nn.Linear(128,32));nn.init.zeros_(self.net[-1].weight);nn.init.zeros_(self.net[-1].bias)
    def forward(self,x):
        residual=self.net(x).reshape(-1,16,2)*10
        cv=x[:,:2,None].transpose(1,2)*30*torch.as_tensor(TIMES,dtype=x.dtype,device=x.device)[None,:,None]
        return cv+residual

def train(cache,out,epochs=80,seed=17):
    if epochs<1:raise ValueError('epochs must be positive')
    out=Path(out)
    if out.exists() and any(out.iterdir()):raise ValueError('training output must be new')
    out.mkdir(parents=True,exist_ok=True);torch.set_num_threads(1);torch.use_deterministic_algorithms(True);torch.manual_seed(seed);np.random.seed(seed);random.seed(seed)
    tr,meta=load_cache(cache,'train');va,_=load_cache(cache,'val')
    if not len(tr['features']) or not len(va['features']):raise ValueError('nonempty train and validation splits required')
    x=torch.from_numpy(tr['features']);y=torch.from_numpy(tr['targets']);vx=torch.from_numpy(va['features']);vy=torch.from_numpy(va['targets']);model=Planner();optimizer=torch.optim.AdamW(model.parameters(),lr=.001,weight_decay=.01);best=float('inf');history=[]
    for epoch in range(1,epochs+1):
        model.train();losses=[]
        for idx in torch.randperm(len(x)).split(64):
            pred=model(x[idx]);loss=nn.functional.smooth_l1_loss(pred,y[idx]);optimizer.zero_grad();loss.backward();nn.utils.clip_grad_norm_(model.parameters(),5);optimizer.step();losses.append(loss.item())
        model.eval()
        with torch.no_grad():errors=torch.linalg.vector_norm(model(vx)-vy,dim=-1);ade=errors.mean().item();fde=errors[:,-1].mean().item()
        history.append(dict(epoch=epoch,train_huber_m=float(np.mean(losses)),val_ade_m=ade,val_fde_m=fde))
        if ade<best:
            best=ade;torch.save(dict(state_dict=model.state_dict(),feature_version=FEATURE_VERSION,snapshot_sha256=meta['snapshot_sha256'],cache_sha256=meta['cache_sha256'],epoch=epoch,seed=seed,val_ade_m=ade),out/'best.pt')
        if epoch%20==0:print('epoch',epoch,'val ADE',round(ade,3),flush=True)
    report=dict(model='MapActorMLP',seed=seed,epochs=epochs,selection='minimum validation ADE; test not used',best_val_ade_m=best,train_windows=len(x),val_windows=len(vx),history=history,checkpoint_sha256=hashlib.sha256((out/'best.pt').read_bytes()).hexdigest(),snapshot_sha256=meta['snapshot_sha256']);(out/'training.json').write_text(json.dumps(report,indent=2));return report

class LearnedAgent:
    def __init__(self,checkpoint):
        torch.set_num_threads(1);self.checkpoint=torch.load(checkpoint,map_location='cpu',weights_only=True)
        if self.checkpoint['feature_version']!=FEATURE_VERSION:raise ValueError('checkpoint feature mismatch')
        self.model=Planner();self.model.load_state_dict(self.checkpoint['state_dict']);self.model.eval()
    def predict(self,inputs):
        with torch.no_grad():xy=self.model(torch.from_numpy(features(inputs))[None])[0].numpy()
        cur=inputs['history'][-1];yaw=cur['yaw'];c,s=np.cos(yaw),np.sin(yaw);rot=np.array([[c,-s],[s,c]]);world=xy@rot.T+cur['xy'];world=np.vstack([cur['xy'],world]);delta=np.gradient(world,axis=0);head=np.arctan2(delta[:,1],delta[:,0]);head[np.linalg.norm(delta,axis=1)<.02]=yaw;head[0]=yaw
        return np.column_stack([np.r_[0,TIMES],world,head]).tolist()

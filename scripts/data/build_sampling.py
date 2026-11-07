"""Build frequency-specific labels from aligned episodes and raw sensor clocks.
Requires the original sensor records used for the existing trajectory labels.
Never derives 4 Hz labels by upsampling 2 Hz targets.
"""
import argparse,collections,hashlib,json,lzma
from pathlib import Path
import numpy as np

def main():
 p=argparse.ArgumentParser();p.add_argument('--assets',type=Path,default=Path('assets'));p.add_argument('--aligned-root',type=Path,required=True);p.add_argument('--records-root',type=Path,required=True);a=p.parse_args()
 groups=collections.defaultdict(list);episodes={};out={s:{} for s in ['train','val','test']};audit={'sources':{},'split_sha256':{},'native_capture_intervals_s':{},'policy':'true capture clocks; causal distinct historical images; sensor-integrated 4 Hz targets; original split membership'};dt=[]
 for split in out:
  src=a.assets/'splits'/f'{split}.json';audit['split_sha256'][split]=hashlib.sha256(src.read_bytes()).hexdigest()
  for r in json.loads(src.read_text()):
   ep=r['context']['source_episode']
   if ep not in episodes:
    records=[json.loads(l) for l in (a.aligned_root/ep/'metadata.jsonl').read_text().splitlines()];episodes[ep]=records;dt.extend(np.diff([x['timestamp_s'] for x in records]).tolist())
   groups[Path(r['time_alignment']['source_record']).parent.name].append((split,r))
 for source,items in groups.items():
  path=a.records_root/source/'records_10hz.jsonl'
  if not path.exists():path=Path(items[0][1]['time_alignment']['source_record'])
  vel={};head={};h=hashlib.sha256()
  with path.open('rb') as f:
   for line in f:
    h.update(line);x=json.loads(line);imu=x.get('imu',{});v=imu.get('807',{});z=imu.get('810',{})
    if all(k in v for k in ['t','VelN','VelE']):vel[v['t']]=[v['VelN'],v['VelE']]
    if all(k in z for k in ['t','AngleHeading']):head[z['t']]=z['AngleHeading']
  vt=np.array(sorted(vel));vv=np.array([vel[t] for t in vt]);ht=np.array(sorted(head));hv=np.unwrap(np.radians([head[t] for t in ht]));cum=np.vstack([np.zeros(2),np.cumsum(np.diff(vt)[:,None]*(vv[:-1]+vv[1:])/2,axis=0)])
  def pos(t):
   i=np.clip(np.searchsorted(vt,t,side='right')-1,0,len(vt)-2);d=t-vt[i];return cum[i]+vv[i]*d+.5*(vv[i+1]-vv[i])/(vt[i+1]-vt[i])*d*d
  for split,r in items:
   t=r['time_alignment']['capture_timestamp_s'];ep=episodes[r['context']['source_episode']];start=ep[0]['timestamp_s'];end=ep[-1]['timestamp_s'];et=np.array([x['timestamp_s'] for x in ep]);assert t+4<=end+1e-8
   for st in (vt,ht):
    assert st[0]<=max(start,t-1.5) and st[-1]>=t+4
    ii=max(0,np.searchsorted(st,max(start,t-1.5),side='right')-1);jj=min(len(st)-1,np.searchsorted(st,t+4));assert np.max(np.diff(st[ii:jj+1]),initial=0)<=.3
   vi=np.searchsorted(vt,t,side='right');hi=np.searchsorted(ht,t,side='right');cv=vt[:vi];ch=ht[:hi];h0=r['time_alignment']['heading_anchor_rad'];co,si=np.cos(h0),np.sin(h0)
   future=[]
   for q in t+np.arange(1,17)*.25:
    ne=pos(q)-pos(t);heading=np.interp(q,ht,hv);future.append([ne[0]*co+ne[1]*si,ne[0]*si-ne[1]*co,float(np.arctan2(np.sin(h0-heading),np.cos(h0-heading)))])
   np.testing.assert_allclose(future[1::2],r['target'],rtol=0,atol=1e-10)
   offsets=np.arange(-6,1)*.25;history=[]
   for q in np.maximum(start,t+offsets):
    grid=np.unique(np.r_[q,cv[(cv>q)&(cv<t)],t]);vels=np.column_stack([np.interp(grid,cv,vv[:vi,k]) for k in range(2)]);ne=-np.trapezoid(vels,grid,axis=0);heading=np.interp(q,ch,hv[:hi]);history.append([ne[0]*co+ne[1]*si,ne[0]*si-ne[1]*co,float(np.arctan2(np.sin(h0-heading),np.cos(h0-heading)))])
   np.testing.assert_allclose(np.asarray(history)[::2].reshape(-1),r['history'],rtol=0,atol=1e-10)
   images={}
   for rate in (2,4):
    slots=[];used=set()
    for offset in np.arange(-int(1.5*rate),1)/rate:
     wanted=t+offset;idx=int(np.searchsorted(et,wanted,side='right')-1);valid=idx>=0 and wanted-et[idx]<=1/rate and idx not in used
     if valid:
      used.add(idx);row=ep[idx];paths=[row[k]['url'] for k in ['images_2','images_1','images_3']]
      slots.append({'images':paths,'timestamp_s':float(et[idx]),'offset_s':float(et[idx]-t),'requested_offset_s':float(offset),'valid':True})
     else:slots.append({'images':None,'timestamp_s':None,'offset_s':None,'requested_offset_s':float(offset),'valid':False})
    images[str(rate)]=slots
   out[split][r['token']]={'history_4hz':np.asarray(history).reshape(-1).tolist(),'history_valid_4hz':(t+offsets>=start).tolist(),'target_4hz':future,'image_history':images}
  audit['sources'][source]=h.hexdigest();print(source,len(items),flush=True)
 target=a.assets/'sampling';target.mkdir(exist_ok=True)
 for split,records in out.items():(target/(split+'.json.xz')).write_bytes(lzma.compress(json.dumps(records,separators=(',',':'),ensure_ascii=False).encode()))
 audit['windows']={k:len(v) for k,v in out.items()};audit['episodes']=len(episodes);audit['native_capture_intervals_s']={str(q):float(np.quantile(dt,q)) for q in [0,.1,.5,.9,1]};audit['labels_match_existing_2hz_at_shared_times']=True
 (target/'audit.json').write_text(json.dumps(audit,indent=2));print(audit['windows'],audit['native_capture_intervals_s'])
if __name__=='__main__':main()

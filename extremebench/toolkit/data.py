"""Fixed-split data access. Agent inputs exclude targets and evaluation contexts."""
from pathlib import Path
import json,gzip,lzma,hashlib
import numpy as np

def load(path):
    path=Path(path)
    opener = gzip.open if path.suffix=='.gz' else lzma.open if path.suffix=='.xz' else open
    with opener(path,'rt',encoding='utf-8') as f:return json.load(f)
def digest(obj):return hashlib.sha256(json.dumps(obj,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def prediction_digest(p):return digest({k:p[k] for k in ('token','times_s','poses')})
def safe_token(token):
    if not isinstance(token,str) or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789:_-' for c in token):raise ValueError('invalid token')
    return token.replace(':','_')
class Dataset:
    def __init__(self,assets,dataroot=None,split='test'):
        if split not in ('train','val','test'):raise ValueError('invalid split')
        self.assets=Path(assets);self.dataroot=Path(dataroot) if dataroot else None;self.split=split
        self.rows={r['token']:r for r in load(self.assets/'splits'/f'{split}.json')}
        self.cohort=[r['token'] for r in load(self.assets/'cohort.json')['records']]
    def tokens(self,execution=False):return [t for t in self.rows if not execution or t in self.cohort]
    def inputs(self,token):
        r=self.rows[token];images=[]
        for name in r['images']:
            rel=name.split('/Extreme_Driving_Conditions_Dataset/',1)[-1]
            if rel.startswith('/') or '..' in Path(rel).parts:raise ValueError('unmapped sensor path')
            images.append(str(self.dataroot/rel) if self.dataroot else rel)
        return {'token':token,'images':images,'language':r['language'],'history':r['history'],'history_valid':r['history_valid']}
    def target(self,token):return self.rows[token]['target']
    def context(self,token):
        if token not in self.cohort:raise ValueError('Execution annotations released for the 131-window cohort only; use displacement evaluation for the full split.')
        path=self.assets/'contexts'/(safe_token(token)+'.json.xz')
        if not path.exists():return load(self.assets/'contexts'/(safe_token(token)+'.json.gz'))
        packed=load(path)
        if packed.get('schema')!='shared-context-v1':raise ValueError('unsupported context schema')
        def blob(ref):
            if len(ref)!=64 or any(ch not in '0123456789abcdef' for ch in ref):raise ValueError('invalid blob reference')
            data=lzma.decompress((self.assets/'context_blobs'/(ref+'.json.xz')).read_bytes())
            if hashlib.sha256(data).hexdigest()!=ref:raise ValueError('context blob checksum mismatch')
            return json.loads(data)
        result=packed['window_data']
        scene=blob(packed['scene_static']);scene['frames']=packed['frames']
        result['scene']=scene;result['road']=blob(packed['road'])
        if digest(result)!=packed['reconstructed_sha256']:raise ValueError('reconstructed context checksum mismatch')
        return result

def read_predictions(path,expected=None):
    result={}
    for line in Path(path).read_text().splitlines():
        if not line.strip():continue
        p=json.loads(line);t=p['token'];safe_token(t)
        if t in result:raise ValueError('duplicate token: '+t)
        a=np.asarray(p['poses'],dtype=float);times=np.asarray(p['times_s'],dtype=float)
        if a.shape!=(8,3) or times.shape!=(8,) or not np.isfinite(a).all() or not np.isfinite(times).all() or not np.allclose(times,np.arange(1,9)*.5,rtol=0,atol=1e-8):raise ValueError('Expected 8 finite [x,y,yaw] at 0.5,...,4 s: '+t)
        result[t]=p
    if not result:raise ValueError('empty submission')
    if expected is not None and set(result)!=set(expected):raise ValueError(f'Token coverage mismatch: missing={len(set(expected)-set(result))}, extra={len(set(result)-set(expected))}')
    return result

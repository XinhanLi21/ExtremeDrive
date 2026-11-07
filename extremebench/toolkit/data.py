"""Fixed-split data access. Agent inputs exclude targets and evaluation contexts."""
from pathlib import Path
import json,gzip,lzma,hashlib
import numpy as np
from .instructions import language_inputs, annotation, POLICIES

def load(path):
    path=Path(path)
    opener = gzip.open if path.suffix=='.gz' else lzma.open if path.suffix=='.xz' else open
    with opener(path,'rt',encoding='utf-8') as f:return json.load(f)
def digest(obj):return hashlib.sha256(json.dumps(obj,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def prediction_digest(p):return digest({k:p[k] for k in ('token','times_s','poses')})
def safe_token(token):
    if not isinstance(token,str) or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789:_-' for c in token):raise ValueError('invalid token')
    return token.replace(':','_')
def trajectory_times(hz):
    if hz not in (2,4):raise ValueError('Supported sampling rates are 2 Hz and 4 Hz')
    return np.arange(1,4*hz+1,dtype=float)/hz

def prediction_rate(pred):
    a=np.asarray(pred['poses'],dtype=float);times=np.asarray(pred['times_s'],dtype=float)
    for hz in (2,4):
        if a.shape==(4*hz,3) and times.shape==(4*hz,) and np.isfinite(a).all() and np.isfinite(times).all() and np.allclose(times,trajectory_times(hz),rtol=0,atol=1e-8):return hz
    raise ValueError('Expected finite [x,y,yaw] poses at 2 Hz (8 points) or 4 Hz (16 points), horizon 4 s')

class Dataset:
    def __init__(self,assets,dataroot=None,split='test',input_hz=2,trajectory_hz=2,input_policy='sensor_only'):
        if split not in ('train','val','test'):raise ValueError('invalid split')
        trajectory_times(input_hz);trajectory_times(trajectory_hz)
        if input_policy not in POLICIES:raise ValueError('invalid input policy')
        self.input_policy=input_policy
        self.input_hz=input_hz;self.trajectory_hz=trajectory_hz;self._sampling=None
        self.assets=Path(assets);self.dataroot=Path(dataroot) if dataroot else None;self.split=split
        self.rows={r['token']:r for r in load(self.assets/'splits'/f'{split}.json')}
        self.cohort=[r['token'] for r in load(self.assets/'cohort.json')['records']]
    def tokens(self,execution=False):return [t for t in self.rows if not execution or t in self.cohort]
    def sampling(self,token):
        if self._sampling is None:self._sampling=load(self.assets/'sampling'/(self.split+'.json.xz'))
        return self._sampling[token]
    def image_path(self,name):
        rel=name.split('/Extreme_Driving_Conditions_Dataset/',1)[-1]
        if rel.startswith('/') or '..' in Path(rel).parts:raise ValueError('unmapped sensor path')
        return str(self.dataroot/rel) if self.dataroot else rel
    def inputs(self,token):
        r=self.rows[token];sampling=self.sampling(token)
        history=r['history'] if self.input_hz==2 else sampling['history_4hz']
        valid=r['history_valid'] if self.input_hz==2 else sampling['history_valid_4hz']
        frames=[{**f,'images':[self.image_path(n) for n in f['images']] if f['images'] else None} for f in sampling['image_history'][str(self.input_hz)]]
        return {'token':token,'images':[self.image_path(n) for n in r['images']],**language_inputs(r,self.input_policy),
                'history':history,'history_valid':valid,'input_hz':self.input_hz,
                'history_times_s':(np.arange(-int(1.5*self.input_hz),1)/self.input_hz).tolist(),
                'image_history':frames,'trajectory_hz':self.trajectory_hz,'target_times_s':trajectory_times(self.trajectory_hz).tolist()}
    def instruction_annotation(self,token):
        return annotation(self.rows[token])
    def target(self,token,hz=None):
        hz=self.trajectory_hz if hz is None else hz;trajectory_times(hz)
        return self.rows[token]['target'] if hz==2 else self.sampling(token)['target_4hz']
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

def read_predictions(path,expected=None,hz=None):
    result={};rates=set();input_policies=set()
    if hz is not None:trajectory_times(hz)
    for line in Path(path).read_text().splitlines():
        if not line.strip():continue
        p=json.loads(line);t=p['token'];safe_token(t)
        if t in result:raise ValueError('duplicate token: '+t)
        rate=prediction_rate(p);rates.add(rate)
        policy=p.get('input_policy','unspecified')
        if policy not in (*POLICIES,'unspecified'):raise ValueError('invalid input policy')
        input_policies.add(policy)
        if len(input_policies)>1:raise ValueError('Mixed input policies in one submission')
        if hz is not None and rate!=hz:raise ValueError('Submission sampling rate does not match requested rate')
        if 'trajectory_hz' in p and p['trajectory_hz']!=rate:raise ValueError('trajectory_hz metadata disagrees with timestamps')
        if len(rates)>1:raise ValueError('Mixed trajectory rates in one submission are not supported')
        result[t]=p
    if not result:raise ValueError('empty submission')
    if expected is not None and set(result)!=set(expected):raise ValueError(f'Token coverage mismatch: missing={len(set(expected)-set(result))}, extra={len(set(result)-set(expected))}')
    return result

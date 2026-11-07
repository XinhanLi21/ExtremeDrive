import argparse,json,importlib,hashlib,html,csv
from pathlib import Path
import numpy as np
from .data import Dataset,read_predictions,load,safe_token,digest,prediction_digest,prediction_rate,trajectory_times

def main():
    p=argparse.ArgumentParser(description='ExtremeDrive planning and vehicle-execution toolkit')
    p.add_argument('--assets',default='assets',help='Versioned benchmark assets directory')
    sub=p.add_subparsers(dest='command',required=True)
    sub.add_parser('verify')
    q=sub.add_parser('download');q.add_argument('--dataroot',required=True);q.add_argument('--scope',choices=['execution','test','train','val','all'],default='execution');q.add_argument('--dry-run',action='store_true')
    q=sub.add_parser('inspect');q.add_argument('--input-hz',type=int,choices=[2,4],default=2);q.add_argument('--trajectory-hz',type=int,choices=[2,4],default=2);q.add_argument('--dataroot');q.add_argument('--split',default='test',choices=['train','val','test'])
    q=sub.add_parser('predict');q.add_argument('--input-hz',type=int,choices=[2,4],default=2);q.add_argument('--trajectory-hz',type=int,choices=[2,4],default=2);q.add_argument('--agent',required=True,help='module:Class implementing predict(inputs) -> Nx3 (8 or 16 points)');q.add_argument('--dataroot',required=True);q.add_argument('--split',default='test',choices=['train','val','test']);q.add_argument('--execution-only',action='store_true');q.add_argument('--out',required=True)
    for name in ['validate','displacement','evaluate']:
        q=sub.add_parser(name);q.add_argument('--trajectory-hz',type=int,choices=[2,4]);q.add_argument('--submission',required=True);q.add_argument('--out',default='results');q.add_argument('--split',default='test',choices=['train','val','test'])
        if name in ('validate','evaluate'):q.add_argument('--subset',action='store_true',help='Allow diagnostic subsets; never report as full cohort')
        if name=='evaluate':q.add_argument('--workers',type=int,default=1)
    q=sub.add_parser('reference');q.add_argument('--model',required=True);q.add_argument('--out',default='reference_results')
    q=sub.add_parser('visualize');q.add_argument('--result',required=True);q.add_argument('--out',default='bev.png')
    a=p.parse_args();assets=Path(a.assets)
    if a.command=='visualize':
        from .visualization import visualize
        visualize(Dataset(assets),a.result,a.out);result={'image':a.out}
    elif a.command=='verify':
        manifest=load(assets/'manifest.json');bad=[]
        for name,expected in manifest['sha256'].items():
            f=assets/name
            if not f.is_file() or hashlib.sha256(f.read_bytes()).hexdigest()!=expected:bad.append(name)
        if bad:raise ValueError('Asset integrity failure: '+str(bad))
        result={'verified_files':len(manifest['sha256']),'release':manifest['release']}
    elif a.command=='download':
        from .download import download
        result=download(assets,a.dataroot,a.scope,a.dry_run)
    elif a.command=='inspect':
        ds=Dataset(assets,a.dataroot,a.split,input_hz=a.input_hz,trajectory_hz=a.trajectory_hz);result={'split':a.split,'windows':len(ds.rows),'execution_windows':len(ds.tokens(True)),'example_inputs':ds.inputs(ds.tokens()[0])}
        if a.dataroot:
            paths={p for t in ds.tokens() for p in ds.inputs(t)['images']};missing=[x for x in paths if not Path(x).is_file()];result.update(sensor_files=len(paths),missing_sensor_files=len(missing),missing_examples=missing[:5])
    elif a.command=='predict':
        ds=Dataset(assets,a.dataroot,a.split,input_hz=a.input_hz,trajectory_hz=a.trajectory_hz);module,name=a.agent.split(':');agent=getattr(importlib.import_module(module),name)();dest=Path(a.out);dest.parent.mkdir(parents=True,exist_ok=True)
        with dest.open('w') as out:
            for token in ds.tokens(a.execution_only):out.write(json.dumps(dict(token=token,times_s=trajectory_times(a.trajectory_hz).tolist(),trajectory_hz=a.trajectory_hz,input_hz=a.input_hz,poses=np.asarray(agent.predict(ds.inputs(token))).tolist()),allow_nan=False)+'\n')
        result={'predictions':len(read_predictions(dest,hz=a.trajectory_hz)),'input_hz':a.input_hz,'trajectory_hz':a.trajectory_hz}
    elif a.command=='reference':
        from .evaluation import summarize,write
        if a.model not in [x.name for x in (assets/'reference').iterdir()]:raise ValueError('unknown reference model')
        records=load(assets/'reference'/a.model/'results.json.gz');preds=read_predictions(assets/'reference'/a.model/'predictions.jsonl')
        binding=load(assets/'reference_bindings.json')[a.model]
        for r in records:
            t=r['token']
            if digest(r)!=binding[t]['result_sha256'] or prediction_digest(preds[t])!=binding[t]['prediction_sha256']:raise ValueError('Reference binding mismatch')
        result=summarize(records);result['mode']='archived reviewed reference; not a new inference or simulation';write(Path(a.out)/'summary.json',result);write(Path(a.out)/'records.json',records);report(Path(a.out),records,result)
    else:
        from .evaluation import summarize,write
        ds=Dataset(assets,split=a.split)
        if a.command in ('validate','evaluate'):
            if a.split!='test':raise ValueError('Execution protocol applies to test; use displacement for train/val')
            preds=read_predictions(a.submission,None if a.subset else ds.tokens(True),hz=a.trajectory_hz)
            if not set(preds)<=set(ds.tokens(True)):raise ValueError('Unknown execution tokens')
        else:preds=read_predictions(a.submission,ds.tokens(),hz=a.trajectory_hz)
        rate=prediction_rate(next(iter(preds.values())))
        if a.command=='validate':result={'valid':True,'windows':len(preds),'complete_cohort':len(preds)==131,'trajectory_hz':rate}
        elif a.command=='displacement':
            rows=[]
            for t,r in preds.items():
                e=np.linalg.norm(np.asarray(r['poses'])[:,:2]-np.asarray(ds.target(t,hz=rate))[:,:2],axis=1);rows.append({'token':t,'ADE_m':float(e.mean()),'FDE_m':float(e[-1])})
            result={'windows':len(rows),'trajectory_hz':rate,'ADE_m':float(np.mean([r['ADE_m'] for r in rows])),'FDE_m':float(np.mean([r['FDE_m'] for r in rows]))};write(Path(a.out)/'displacement.json',dict(summary=result,records=rows))
        else:
            from concurrent.futures import ProcessPoolExecutor
            jobs=[(str(assets.resolve()),pred,str(Path(a.out).resolve())) for pred in preds.values()]
            if a.workers<1:p.error('workers must be positive')
            if a.workers==1:records=[worker(x) for x in jobs]
            else:
                with ProcessPoolExecutor(max_workers=a.workers) as pool:records=list(pool.map(worker,jobs))
            result=summarize(records);result.update(trajectory_hz=rate,mode='fresh_automated_execution',complete_cohort=len(preds)==131,submission_sha256=hashlib.sha256(Path(a.submission).read_bytes()).hexdigest())
            write(Path(a.out)/'summary.json',result);report(Path(a.out),records,result)
    print(json.dumps(result,ensure_ascii=False,indent=2))

def worker(job):
    from .evaluation import execute,score,write
    assets,pred,out=job;ds=Dataset(assets);run=execute(ds,pred);r=score(ds,pred,run);name=safe_token(pred['token'])+'.json'
    write(Path(out)/'rollouts'/name,run);write(Path(out)/'scores'/name,r)
    print('scored '+pred['token'],flush=True)
    return r

def report(out,records,summary):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    rows=[]
    for r in records:
        rows.append({'token':r['token'],'score':r.get('score'),**{k:r.get('metrics',{}).get(k) for k in ['no_collision','marking_compliance','ttc_compliance','progress']},'drivable_compliance':r.get('road_metrics',{}).get('drivable_area_compliance')})
    with (out/'metrics.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    text='<meta charset="utf-8"><title>ExtremeDrive results</title><style>body{max-width:1200px;margin:40px auto;font:16px system-ui;color:#163042}table{border-collapse:collapse;width:100%}td,th{padding:9px;border-bottom:1px solid #ddd;text-align:left}pre{background:#f0f5f7;padding:20px}</style><h1>ExtremeDrive evaluation</h1><p>Unknown metrics remain unknown. Historical reviewed references and fresh automated evaluations are reported separately.</p><pre>'+html.escape(json.dumps(summary,indent=2))+'</pre><table><tr>'+''.join('<th>'+k+'</th>' for k in rows[0])+'</tr>'
    for r in rows:text+='<tr>'+''.join('<td>'+html.escape('unknown' if v is None else str(round(v,4)) if isinstance(v,float) else str(v))+'</td>' for v in r.values())+'</tr>'
    (out/'index.html').write_text(text+'</table>')
if __name__=='__main__':main()

"""Run the same reference scenes through an actual external physics backend."""
import argparse,json
from pathlib import Path
from dataclasses import replace
from .examples import synthetic,from_metadata
from .core import Config
from .backends.runner import run_backend
from .metrics import evaluate
from .report import save_case,gallery

def main():
    p=argparse.ArgumentParser();p.add_argument('--backend',choices=['commonroad_mb','chrono','carla'],required=True);p.add_argument('--out',required=True);p.add_argument('--metadata');p.add_argument('--port',type=int,default=28700);p.add_argument('--case',action='append');args=p.parse_args()
    out=Path(args.out);out.mkdir(parents=True,exist_ok=True);scenes=synthetic()
    if args.metadata:scenes.append(from_metadata(args.metadata))
    if args.case:scenes=[s for s in scenes if s['id'] in args.case]
    items=[];summary=[]
    for original in scenes:
        scene={**original,'id':args.backend+'__'+original['id'],'source_scene_id':original['id']}
        cfg=replace(Config(),mu=scene.get('mu',.85));options={'port':args.port} if args.backend=='carla' else {}
        try:
            result=run_backend(scene,cfg,args.backend,**options);metrics=evaluate(scene,result);items.append(save_case(scene,result,metrics,out))
            status={'id':scene['id'],'backend':args.backend,'status':'completed' if result['failure'] is None else 'failed','config_id':result['config_id'],**metrics}
        except Exception as e:
            import traceback
            traceback.print_exc();status={'id':scene['id'],'backend':args.backend,'status':'initialization_failed','error':str(e)}
        summary.append(status);(out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False));print(json.dumps(status,ensure_ascii=False),flush=True)
    if items:gallery(items,out)
    print('completed',len(items),'of',len(scenes),flush=True)
if __name__=='__main__':main()

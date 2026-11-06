import argparse,json
from pathlib import Path
from dataclasses import asdict,replace
from .core import Config,rollout
from .metrics import evaluate
from .report import save_case,gallery
from .examples import synthetic,from_metadata

def main():
    p=argparse.ArgumentParser(description='ExtremeDrive actuator rollout prototype');sp=p.add_subparsers(dest='mode',required=True)
    for cmd in ['demo','run']:
        q=sp.add_parser(cmd);q.add_argument('--out',required=True);q.add_argument('--config')
        if cmd=='run':q.add_argument('--scene',required=True)
        else:q.add_argument('--metadata',help='Optional real ExtremeDrive metadata.jsonl reference')
    args=p.parse_args();cfg=Config(**json.loads(Path(args.config).read_text())) if args.config else Config();cfg.validate();out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
    if args.mode=='demo':
        scenes=synthetic()
        if args.metadata:scenes.append(from_metadata(args.metadata))
    else:scenes=[json.loads(Path(args.scene).read_text())]
    summaries=[];items=[]
    for scene in scenes:
        c=replace(cfg,mu=scene.get('mu',cfg.mu));result=rollout(scene,c);m=evaluate(scene,result);items.append(save_case(scene,result,m,out));summaries.append({'scene_id':scene['id'],'provenance':scene.get('provenance'),'config_id':c.identity(),**m})
        print(json.dumps({'scene':scene['id'],'rmse_m':round(m['tracking_rmse_m'],3),'collision':m['collision'],'offroad':m['offroad'],'failure':result['failure']},ensure_ascii=False),flush=True)
    gallery(items,out);(out/'summary.json').write_text(json.dumps(summaries,ensure_ascii=False,indent=2,allow_nan=False));(out/'config.json').write_text(json.dumps(asdict(cfg),indent=2));print(out/'index.html')
if __name__=='__main__':main()

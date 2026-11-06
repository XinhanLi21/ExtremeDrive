import argparse,json,importlib
from .data import audit,cache
from .workflow import predict,validate,evaluate,leaderboard

def main():
 p=argparse.ArgumentParser(description='ExtremeDrive training/evaluation devkit');sub=p.add_subparsers(dest='command',required=True)
 for name in ('audit','cache','export','predict','validate','evaluate'):
  q=sub.add_parser(name);q.add_argument('--snapshot',required=True)
  if name in ('cache','export','predict','evaluate'):q.add_argument('--out',required=True)
  if name in ('validate','evaluate'):q.add_argument('--submission',required=True)
  if name=='evaluate':q.add_argument('--prior',required=True)
  if name=='predict':q.add_argument('--checkpoint');q.add_argument('--split',choices=['train','val','test'],default='test');q.add_argument('--agent',help='module:Class, must implement predict(inputs)')
 q=sub.add_parser('infer');q.add_argument('--inputs',required=True);q.add_argument('--out',required=True);q.add_argument('--checkpoint');q.add_argument('--split',choices=['train','val','test'],default='test')
 q=sub.add_parser('train');q.add_argument('--cache',required=True);q.add_argument('--out',required=True);q.add_argument('--epochs',type=int,default=80);q.add_argument('--seed',type=int,default=17)
 q=sub.add_parser('detection');q.add_argument('--truth',required=True);q.add_argument('--predictions',required=True);q.add_argument('--out',required=True)
 q=sub.add_parser('forecast');q.add_argument('--input',required=True);q.add_argument('--out',required=True)
 q=sub.add_parser('leaderboard');q.add_argument('--results',nargs='+',required=True);q.add_argument('--out',required=True)
 a=p.parse_args()
 if a.command=='audit':r=audit(a.snapshot)
 elif a.command=='cache':r=cache(a.snapshot,a.out)
 elif a.command=='export':
  from .exchange import export_inputs
  r=export_inputs(a.snapshot,a.out)
 elif a.command=='infer':
  from .exchange import infer
  agent=None
  if a.checkpoint:
   from .learning import LearnedAgent
   agent=LearnedAgent(a.checkpoint)
  r=infer(a.inputs,a.out,a.split,agent)
 elif a.command=='train':
  from .learning import train
  r=train(a.cache,a.out,a.epochs,a.seed);r={k:v for k,v in r.items() if k!='history'}
 elif a.command=='predict':
  agent=None
  if a.agent:
   if a.checkpoint:p.error('choose checkpoint or agent')
   module,name=a.agent.split(':',1);agent=getattr(importlib.import_module(module),name)()
  d=predict(a.snapshot,a.out,a.checkpoint,a.split,agent);r={'predictions':len(d['predictions']),'adjacent_predictions':len(d['adjacent_predictions'])}
 elif a.command=='validate':r=validate(a.snapshot,a.submission)
 elif a.command=='evaluate':
  d=evaluate(a.snapshot,a.prior,a.submission,a.out);r=d['road_summary']
 elif a.command=='detection':
  from .tasks import detection_metrics
  from pathlib import Path
  r=detection_metrics(json.loads(Path(a.truth).read_text()),json.loads(Path(a.predictions).read_text()));Path(a.out).write_text(json.dumps(r,indent=2))
 elif a.command=='forecast':
  from .tasks import forecast_metrics
  from pathlib import Path
  d=json.loads(Path(a.input).read_text());r=forecast_metrics(d['predictions'],d['truth'],d.get('probabilities'));Path(a.out).write_text(json.dumps(r,indent=2))
 else:r=leaderboard(a.results,a.out)
 print(json.dumps(r,ensure_ascii=False,indent=2))
if __name__=='__main__':main()

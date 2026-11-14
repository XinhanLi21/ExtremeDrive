import copy,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
from extremebench.toolkit.friction import resolve_friction,table_identity
from extremebench.toolkit.evaluation import execute,compare_summaries
from extremebench.toolkit.data import Dataset,load
ROOT=Path(__file__).resolve().parents[1]

class SceneFrictionTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.assets=Path(self.tmp.name)
  self.table={'schema':'scene-friction-v1','basis':'road_condition_empirical_assignment','surface_mu':{'snow':.3,'wet':.6,'dry':.85},'records':{k:{'surface':k,'mu':v} for k,v in [('snow',.3),('wet',.6),('dry',.85)]},'execution_clips':{k:k for k in ['snow','wet','dry']}}
  self.save()
 def tearDown(self):self.tmp.cleanup()
 def save(self):(self.assets/'scene_friction.json').write_text(json.dumps(self.table))
 def ctx(self,k):return {'row':{'context':{'source_episode':k}},'audit':{'source_episode':k},'window':{'scene_id':'shared-map'}}
 def test_surface_and_shared_map_assignments(self):
  for k,v in [('snow',.3),('wet',.6),('dry',.85)]:
   result=resolve_friction(self.assets,self.ctx(k),k)
   self.assertEqual(result['mu'],v);self.assertEqual(result['table_sha256'],table_identity(self.assets))
 def test_missing_invalid_and_conflicting_assignments_fail(self):
  with self.assertRaises(ValueError):resolve_friction(self.assets,{},'absent')
  with self.assertRaises(ValueError):resolve_friction(self.assets,self.ctx('wet'),'dry')
  for bad in [None,True,-1,0,float('nan'),.85]:
   self.table['records']['snow']['mu']=bad;self.save()
   with self.assertRaises(ValueError):resolve_friction(self.assets,{},'snow')
  (self.assets/'scene_friction.json').unlink()
  with self.assertRaises(ValueError):resolve_friction(self.assets,{},'dry')
 def test_coefficient_reaches_execution_engine(self):
  # Use the real fitted configuration, replace only the expensive simulation.
  from extremebench.execution.mohe_lateral_profile import load_candidate
  config,meta=load_candidate(ROOT/'assets/mohe_profile.json',experimental=True)
  for k in ['snow','wet','dry']:
   ctx={**self.ctx(k),'anchor':{'xy':[0,0],'yaw':0},'sensor_state':{'velocity':[5,0]},'measured_initial_yaw_rate':0}
   ds=SimpleNamespace(assets=self.assets,context=lambda token:ctx)
   pred={'token':k,'times_s':[.5*i for i in range(1,9)],'poses':[[2.5*i,0,0] for i in range(1,9)]}
   with patch('extremebench.toolkit.evaluation.load_candidate',return_value=(config,copy.deepcopy(meta))),patch('extremebench.toolkit.evaluation.execute_official_dynamic',return_value={'geometry':np.zeros((1,2))}) as engine:
    run=execute(ds,pred)
   self.assertEqual(engine.call_args.args[1].mu,self.table['surface_mu'][k])
   self.assertEqual(run['scene_friction']['mu'],self.table['surface_mu'][k])
 def test_all_released_execution_and_planning_sources_covered(self):
  table=load(ROOT/'assets/scene_friction.json');cohort=load(ROOT/'assets/cohort.json')['records']
  self.assertEqual(set(table['execution_clips']),{r['token'] for r in cohort})
  for row in cohort:
   self.assertEqual(table['execution_clips'][row['token']],row['source_episode'])
   result=resolve_friction(ROOT/'assets',{'audit':{'source_episode':row['source_episode']}},row['token'])
   self.assertEqual(result['mu'],table['surface_mu'][result['surface']])
  for split in ['train','val','test']:
   for row in Dataset(ROOT/'assets',split=split).rows.values():self.assertIn(row['context']['source_episode'],table['records'])
 def test_comparisons_reject_different_friction(self):
  s=dict(benchmark_eligible=True,complete_cohort=True,mode='fresh_automated_execution',review_policy='automatic_only_v1',input_policy='sensor_only',trajectory_hz=2,protocol_sha256='p',cohort_sha256='c',asset_manifest_sha256='a',scene_friction_sha256='f')
  with self.assertRaises(ValueError):compare_summaries([s,{**s,'scene_friction_sha256':'changed'}])
if __name__=='__main__':unittest.main()

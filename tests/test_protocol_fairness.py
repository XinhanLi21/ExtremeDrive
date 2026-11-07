import copy,json,unittest,tempfile
from pathlib import Path
from dataclasses import asdict,replace
import numpy as np
from scipy.signal import savgol_filter
from extremebench.benchmark.comfort import ComfortConfig,aggregate_ratios
from extremebench.benchmark.protocol import Protocol
from extremebench.execution.comfort import dynamic_comfort
from extremebench.toolkit.data import Dataset,read_predictions
from extremebench.toolkit.instructions import language_inputs,annotation
from extremebench.toolkit.evaluation import compare_summaries
ROOT=Path(__file__).resolve().parents[1]

class ProtocolFairnessTests(unittest.TestCase):
 def test_all_default_inputs_are_neutral(self):
  for split in ['train','val','test']:
   ds=Dataset(ROOT/'assets',split=split)
   for t,r in ds.rows.items():
    x=language_inputs(r);self.assertNotIn('Instruction:',x['language']);self.assertFalse(x['navigation_valid']);self.assertIsNone(x['navigation_command'])
    a=annotation(r);self.assertIsNone(a['annotation_created_at']);self.assertIsNone(a['online_available_at_capture'])
    self.assertEqual(language_inputs(r,'scenario_hint')['language'],r['language'])
    self.assertEqual(language_inputs(r,'navigation_only')['language'],x['language'])
    expected=x['language'] if a['maneuver_hint'] else r['language']
    self.assertEqual(language_inputs(r,'no_maneuver_hint')['language'],expected)
 def test_navigation_not_taken_from_evasive_annotation(self):
  r={'language':'Speed: 10.00 m/s. Instruction: Emergency swerve to the left to avoid collision. Predict the next 4 seconds of driving.'}
  self.assertIsNone(annotation(r)['navigation_annotation']);self.assertIsNotNone(annotation(r)['maneuver_hint'])
  r['language']='Speed: 10.00 m/s. Instruction: Turn left. Predict the next 4 seconds of driving.'
  self.assertEqual(annotation(r)['navigation_annotation'],'left');self.assertIsNone(language_inputs(r)['navigation_command'])
 def test_submission_input_policy_not_mixed(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'p.jsonl';a=dict(token='a',poses=[[0,0,0]]*8,times_s=[.5*i for i in range(1,9)],input_policy='sensor_only');b={**a,'token':'b','input_policy':'scenario_hint'}
   p.write_text(json.dumps(a)+'\n'+json.dumps(b))
   with self.assertRaises(ValueError):read_predictions(p)
 def logs(self):
  return [dict(t=float(t),ax=float(3+np.sin(3*t)),ay=float(3*np.sin(t)),yaw_rate=float(.4*np.sin(t))) for t in np.arange(0,4.0001,.05)]
 def test_unchanged_effective_default_comfort(self):
  logs=self.logs();t=np.array([r['t'] for r in logs]);dt=t[1]-t[0];n=max(5,int(round(.3/dt))//2*2+1)
  f=lambda k,d=0:savgol_filter([r[k] for r in logs],n,3,deriv=d,delta=dt)
  ax,ay,r,j=f('ax'),f('ay'),f('yaw_rate'),f('ax',1)
  ratios=np.array([np.where(ax>=0,ax/2.4,ax/-4.05),abs(ay)/4.89,np.hypot(j-r*ay,f('ay',1)+r*ax)/8.37,abs(r)/.95,abs(f('yaw_rate',1))/1.93,abs(j)/4.13])
  old=aggregate_ratios(t,ratios);new=dynamic_comfort(logs,Protocol().comfort)
  for key in old:np.testing.assert_allclose(new[key],old[key],atol=1e-12,rtol=0)
 def test_protocol_threshold_actually_changes_score(self):
  logs=[dict(t=float(t),ax=3.,ay=0.,yaw_rate=0.) for t in np.arange(0,4.0001,.05)]
  c=asdict(ComfortConfig());low=dynamic_comfort(logs,c);c['acceleration_limit_mps2']=6;high=dynamic_comfort(logs,c)
  self.assertLess(low['score'],high['score']);self.assertAlmostEqual(high['score'],1.)
  self.assertEqual(high['config']['acceleration_limit_mps2'],6)
 def test_duration_weighting_and_invalid_config(self):
  r=aggregate_ratios([0,1,2],[[1,2,1]])
  self.assertAlmostEqual(r['score'],.625)
  with self.assertRaises(ValueError):replace(ComfortConfig(),inertial_jerk_limit_mps3=0).validate()
  with self.assertRaises(ValueError):Protocol(comfort={'acceleration_limit_mps2':0}).validate()
 def test_no_archive_or_policy_mixing(self):
  s=dict(benchmark_eligible=True,complete_cohort=True,mode='fresh_automated_execution',review_policy='automatic_only_v1',input_policy='sensor_only',trajectory_hz=2,protocol_sha256='p',cohort_sha256='c',asset_manifest_sha256='a')
  self.assertEqual(len(compare_summaries([s,s])),2)
  for key,value in [('mode','archived_reviewed_reference'),('input_policy','scenario_hint'),('trajectory_hz',4),('protocol_sha256','other'),('benchmark_eligible',False),('complete_cohort',False)]:
   other={**s,key:value}
   with self.assertRaises(ValueError):compare_summaries([s,other])
if __name__=='__main__':unittest.main()

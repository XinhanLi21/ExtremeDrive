import unittest,json
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
from extremebench.toolkit.data import Dataset,read_predictions,load
from extremebench.toolkit.evaluation import execute,score,summarize
ROOT=Path(__file__).resolve().parents[1]
TOKENS=['2fe2061542fdc397:dense000036:main','4a7e61429f2fb49a:dense000022:main']

class InitialStateAvailabilityTests(unittest.TestCase):
 def pred(self,t):return {'token':t,'times_s':[.5*i for i in range(1,9)],'poses':[[float(i),0.,0.] for i in range(1,9)],'input_policy':'sensor_only'}
 def test_released_negative_states_are_unavailable(self):
  ds=Dataset(ROOT/'assets');records=[]
  for token in TOKENS:
   ctx=ds.context(token);self.assertLess(ctx['sensor_state']['velocity'][0],0)
   p=self.pred(token)
   with patch('extremebench.toolkit.evaluation.execute_official_dynamic') as engine:
    run=execute(ds,p);engine.assert_not_called()
   self.assertLess(run['initial_state_supplied']['vx'],0)
   self.assertEqual(run['execution_status'],'unsupported_initial_state')
   r=score(ds,p,run);self.assertIsNone(r['score']);self.assertEqual(r['status'],'unsupported_initial_state')
   self.assertFalse(r['metrics']);self.assertIsNone(r['emergency_safety']['success'])
   json.dumps(run,allow_nan=False);json.dumps(r,allow_nan=False);records.append(r)
  summary=summarize(records)
  self.assertEqual(summary['unsupported_initial_states'],2)
  self.assertEqual(summary['metrics']['score']['unknown'],2)
  self.assertIsNone(summary['metrics']['score']['known_mean'])
  for key in ['comfort','emergency_safety_success']:
   self.assertEqual(summary['metrics'][key]['applicable_windows'],1)
   self.assertEqual(summary['metrics'][key]['unknown'],1)
 def test_zero_and_positive_speed_still_execute(self):
  ds=Dataset(ROOT/'assets');ctx=ds.context(TOKENS[0]);stub=SimpleNamespace(assets=ds.assets,context=lambda t:ctx)
  for vx in [0.,2.]:
   ctx['sensor_state']['velocity'][0]=vx
   with patch('extremebench.toolkit.evaluation.execute_official_dynamic',return_value={'geometry':np.zeros((1,2))}) as engine:
    execute(stub,self.pred(TOKENS[0]));engine.assert_called_once()
 def test_physics_failure_retains_zero_score(self):
  ds=Dataset(ROOT/'assets');r=score(ds,self.pred(TOKENS[0]),{'failure':'nonfinite_state','execution_status':'failed'})
  self.assertEqual(r['score'],0.);self.assertEqual(r['status'],'execution_failed')
if __name__=='__main__':unittest.main()

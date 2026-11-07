import json,tempfile,unittest
from pathlib import Path
import numpy as np
from extremebench.toolkit.data import Dataset,trajectory_times,read_predictions,prediction_rate
from extremebench.agents import ConstantVelocityAgent
ROOT=Path(__file__).resolve().parents[1]
class SamplingTests(unittest.TestCase):
 def test_label_and_history_equivalence(self):
  for split in ('train','val','test'):
   ds=Dataset(ROOT/'assets',split=split,input_hz=4,trajectory_hz=4)
   for token in ds.tokens():
    np.testing.assert_allclose(np.asarray(ds.target(token))[1::2],ds.rows[token]['target'],rtol=0,atol=1e-10)
    np.testing.assert_allclose(np.asarray(ds.sampling(token)['history_4hz']).reshape(7,3)[::2].reshape(-1),ds.rows[token]['history'],rtol=0,atol=1e-10)
 def test_images_are_real_distinct_and_causal(self):
  for rate in (2,4):
   ds=Dataset(ROOT/'assets',split='test',input_hz=rate,trajectory_hz=rate)
   for token in ds.tokens():
    inp=ds.inputs(token);anchor=ds.rows[token]['time_alignment']['capture_timestamp_s'];seen=set()
    self.assertEqual(len(inp['image_history']),int(1.5*rate)+1)
    self.assertEqual(len(inp['history']),3*(int(1.5*rate)+1))
    self.assertNotIn('target',inp)
    for f in inp['image_history']:
     if f['valid']:
      self.assertLessEqual(f['timestamp_s'],anchor+f['requested_offset_s']+1e-8)
      self.assertNotIn(f['timestamp_s'],seen);seen.add(f['timestamp_s'])
     else:self.assertIsNone(f['images'])
 def test_agent_supports_both_grids(self):
  for rate in (2,4):
   times=trajectory_times(rate);poses=ConstantVelocityAgent().predict({'language':'Speed: 10 m/s.','target_times_s':times.tolist()})
   self.assertEqual(poses.shape,(4*rate,3));np.testing.assert_array_equal(poses[:,0],10*times)
 def test_prediction_clock_validation(self):
  def pred(token,rate):return dict(token=token,times_s=trajectory_times(rate).tolist(),poses=np.zeros((4*rate,3)).tolist())
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'p.jsonl';p.write_text(json.dumps(pred('a',4)))
   self.assertEqual(prediction_rate(read_predictions(p)['a']),4)
   with self.assertRaises(ValueError):read_predictions(p,hz=2)
   p.write_text(json.dumps(pred('a',2))+'\n'+json.dumps(pred('b',4)))
   with self.assertRaises(ValueError):read_predictions(p)
   x=pred('a',4);x['times_s'][0]=.2;p.write_text(json.dumps(x))
   with self.assertRaises(ValueError):read_predictions(p)

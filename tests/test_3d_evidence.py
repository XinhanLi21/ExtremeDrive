import copy,unittest
import numpy as np
from extremebench.benchmark.scoring import score_window
from extremebench.benchmark.protocol import Protocol

class ActorEvidenceTests(unittest.TestCase):
 def example(self):
  frames=[dict(t=float(t),xy=[float(t),0.],yaw=0.,key='episode',objects=[],dynamic_available=True,unresolved_2d_count=3) for t in np.arange(0,4.01,.25)]
  scene=dict(scene_id='scene',features=[],frames=frames)
  window=dict(id='clip',start_s=0.,end_s=4.,category='Normal_Driving')
  plan=[[float(t),float(t),0.,0.] for t in np.arange(0,4.01,.25)]
  return scene,window,plan
 def test_unmatched_2d_does_not_invalidate_readable_3d(self):
  sc,w,plan=self.example();r=score_window(sc,w,plan,Protocol())
  self.assertEqual(r['metrics']['no_collision'],1)
  self.assertEqual(r['metrics']['ttc_compliance'],1)
  self.assertGreater(r['details']['unresolved_2d_count'],0)
 def test_missing_annotation_remains_unknown(self):
  sc,w,plan=self.example();sc['frames'][5]['dynamic_available']=False
  r=score_window(sc,w,plan,Protocol());self.assertIsNone(r['metrics']['no_collision']);self.assertIsNone(r['metrics']['ttc_compliance'])
 def test_observed_3d_collision_is_not_ignored(self):
  sc,w,plan=self.example()
  for f in sc['frames']:f['objects']=[dict(id='car',xy=[[-1,-1],[1,-1],[1,1],[-1,1],[-1,-1]],**{'class':'car'})]
  r=score_window(sc,w,plan,Protocol());self.assertEqual(r['metrics']['no_collision'],0);self.assertEqual(r['metrics']['ttc_compliance'],0)
 def test_temporal_gap_remains_unknown(self):
  sc,w,plan=self.example();sc['frames']=[sc['frames'][0],sc['frames'][-1]]
  r=score_window(sc,w,plan,Protocol());self.assertIsNone(r['metrics']['no_collision'])

 def test_3d_without_camera_availability_is_authoritative(self):
  sc,w,plan=self.example()
  for f in sc['frames']:
   f['dynamic_available']=False
   f['objects']=[dict(id='car',xy=[[-1,-1],[1,-1],[1,1],[-1,1],[-1,-1]],**{'class':'car'})]
  r=score_window(sc,w,plan,Protocol())
  self.assertEqual(r['metrics']['no_collision'],0)
  self.assertEqual(r['metrics']['ttc_compliance'],0)

 def test_invalid_3d_does_not_override_missing_observation(self):
  from extremebench.benchmark.scoring import has_3d_observation
  self.assertFalse(has_3d_observation(dict(dynamic_available=False,objects=[{'xy':[[0,0],[1,0],[2,0]]}])))

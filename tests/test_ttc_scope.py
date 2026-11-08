import unittest
from unittest.mock import patch
from shapely.geometry import box
from extremebench.toolkit.ttc import revise_ttc
from extremebench.benchmark.protocol import Protocol

class TTCScopeTests(unittest.TestCase):
 def evaluate(self, reviewed, overlapping=False, velocity=None):
  c={'scene':{'frames':[{'objects':[{'id':'a','class':'car'}]}]},'ttc_actor_scope':{'basis':'test_review','nonconflicting_ids':['a'] if reviewed else []}}
  r={'trajectory':[[0,0,0,0],[.5,0,0,0],[1,0,0,0]],'metrics':{}}
  poly=box(-1,-1,1,1) if overlapping else box(4,-1,6,1)
  with patch('extremebench.toolkit.ttc.AdjacentActors') as cls:
   cls.return_value.at.return_value=[('a',poly,velocity)]
   return revise_ttc(r,c,Protocol())
 def test_review_resolves_only_scoped_actor(self):
  self.assertIsNone(self.evaluate(False)['metrics']['ttc_compliance'])
  self.assertEqual(self.evaluate(True)['metrics']['ttc_compliance'],1)
 def test_review_never_removes_observed_overlap(self):
  self.assertEqual(self.evaluate(True,True)['metrics']['ttc_compliance'],0)
 def test_known_velocity_risk_remains(self):
  import numpy as np
  self.assertEqual(self.evaluate(True,False,np.array([-5.,0.]))['metrics']['ttc_compliance'],0)

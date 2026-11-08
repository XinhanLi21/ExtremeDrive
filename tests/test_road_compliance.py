import itertools,unittest
from extremebench.benchmark.aggregation import road_compliance,scoped_total
from extremebench.toolkit.evaluation import summarize
class RoadComplianceTests(unittest.TestCase):
 def test_joint_truth_table_and_score_equivalence(self):
  for m,d,x in itertools.product([0,1],repeat=3):
   b=dict(white_compliance=m,yellow_compliance=1,marking_compliance=m,annotated_exclusion_compliance=x,no_collision=1,progress=.8,ttc_compliance=1)
   r=dict(drivable_area_compliance=d,driving_direction_compliance=1)
   self.assertEqual(road_compliance(b,r),m*d*x)
   self.assertAlmostEqual(scoped_total(b,r,None,False),90*m*d*x)
 def test_unknown_and_optional_exclusion(self):
  self.assertEqual(road_compliance({'white_compliance':1,'yellow_compliance':1},{'drivable_area_compliance':1}),1)
  self.assertIsNone(road_compliance({'white_compliance':1,'yellow_compliance':1},{}))
  self.assertEqual(road_compliance({'white_compliance':0,'yellow_compliance':1},{}),0)
 def test_summary_exposes_one_road_metric(self):
  s=summarize([{'metrics':{'white_compliance':1,'yellow_compliance':1},'road_metrics':{'drivable_area_compliance':1}}])['metrics']
  self.assertEqual(s['road_compliance']['mean'],1)
  self.assertNotIn('marking_compliance',s)
  self.assertNotIn('drivable_compliance',s)

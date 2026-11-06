import unittest
from pathlib import Path
from extremebench.toolkit.data import load
from extremebench.benchmark.protocol import Protocol
from extremebench.benchmark.aggregation import scoped_total

class CurrentProtocolTests(unittest.TestCase):
    def test_default_matches_current_asset_protocol(self):
        assets=Path(__file__).resolve().parents[1]/'assets'
        self.assertEqual(Protocol().identity(),Protocol(**load(assets/'protocol.json')).identity())
    def test_unknown_and_failed_gate(self):
        road={'drivable_area_compliance':1,'driving_direction_compliance':1}
        metrics={'no_collision':1,'white_compliance':1,'yellow_compliance':1,'marking_compliance':1,'progress':.8,'ttc_compliance':None}
        self.assertIsNone(scoped_total(metrics,road,None,False))
        metrics['no_collision']=0
        self.assertEqual(scoped_total(metrics,road,None,False),0)
    def test_comfort_applies_only_when_requested(self):
        road={'drivable_area_compliance':1,'driving_direction_compliance':1}
        metrics={'no_collision':1,'white_compliance':1,'yellow_compliance':1,'marking_compliance':1,'progress':1,'ttc_compliance':1}
        self.assertEqual(scoped_total(metrics,road,0,False),100)
        self.assertAlmostEqual(scoped_total(metrics,road,0,True),1000/12)

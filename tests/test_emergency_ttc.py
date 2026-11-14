import itertools
import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch
import numpy as np
from shapely.geometry import box
from extremebench.benchmark.protocol import Protocol
from extremebench.benchmark.emergency import emergency_safety
from extremebench.toolkit.ttc import revise_ttc
from extremebench.toolkit.evaluation import score

class EmergencyTTCTests(unittest.TestCase):
    def row(self,n=1,r=1,t=.0):
        return dict(category='Critical_Driving',metrics=dict(no_collision=n,white_compliance=r,yellow_compliance=1,emergency_ttc_compliance=t),road_metrics=dict(drivable_area_compliance=1))
    def test_three_gates_including_unknowns(self):
        for n,r,t in itertools.product([0,1,None],repeat=3):
            expected=False if 0 in (n,r,t) else None if None in (n,r,t) else True
            self.assertIs(emergency_safety(self.row(n,r,t))['success'],expected)
    def test_no_fallback_to_composite_ttc(self):
        row=self.row(t=1);row['metrics']['ttc_compliance']=0
        self.assertTrue(emergency_safety(row)['success'])
        del row['metrics']['emergency_ttc_compliance'];row['metrics']['ttc_compliance']=1
        self.assertIsNone(emergency_safety(row)['success'])
    def test_noncritical_not_applicable(self):
        self.assertFalse(emergency_safety(dict(category='Normal_Driving'))['applicable'])
    def test_real_geometry_distinguishes_horizons(self):
        ctx={'scene':{'frames':[{'objects':[{'id':'car','class':'car'}]}]}}
        # A stationary ego and approaching vehicle conflict after 0.5 s but within 1.0 s.
        row={'trajectory':[[0,0,0,0],[.5,0,0,0],[1,0,0,0]],'metrics':{}}
        p=Protocol()
        with patch('extremebench.toolkit.ttc.AdjacentActors') as actors:
            actors.return_value.at.return_value=[('car',box(4,-1,6,1),np.array([-2.,0.]))]
            composite=revise_ttc(dict(row,metrics={}),ctx,p)
            emergency=revise_ttc(dict(row,metrics={}),ctx,replace(p,ttc_horizon_s=p.emergency_ttc_horizon_s))
        self.assertEqual(composite['metrics']['ttc_compliance'],0)
        self.assertEqual(emergency['metrics']['ttc_compliance'],1)
    def test_scoring_keeps_composite_one_second(self):
        class Dataset:
            def context(self,token):return {'scene':{},'window':{'category':'Critical_Driving'},'road':{}}
        with tempfile.TemporaryDirectory() as tmp:
            ds=Dataset();ds.assets=Path(tmp);(ds.assets/'protocol.json').write_text('{}')
            raw=dict(category='Critical_Driving',trajectory=[[0,0,0,0],[1,0,0,0]],metrics=dict(no_collision=1,white_compliance=1,yellow_compliance=1,progress=1))
            horizons=[]
            def ttc(row,ctx,p):
                horizons.append(p.ttc_horizon_s);row['metrics']['ttc_compliance']=int(p.ttc_horizon_s==.5);row['ttc_evidence']={'horizon':p.ttc_horizon_s};return row
            pred=dict(token='test',poses=[[0,0,0]]*8,times_s=[.5*i for i in range(1,9)],input_policy='sensor_only')
            run=dict(failure=None,logs=[dict(t=0,x=0,y=0,yaw=0)],execution_status='executed')
            with patch('extremebench.toolkit.evaluation.scoring.score_window',return_value=raw),patch('extremebench.toolkit.evaluation.road_evaluate',return_value={'drivable_area_compliance':1}),patch('extremebench.toolkit.evaluation.revise_ttc',side_effect=ttc),patch('extremebench.toolkit.evaluation.dynamic_comfort',return_value={'score':1}):
                result=score(ds,pred,run)
        self.assertEqual(horizons,[1.,.5])
        self.assertEqual(result['metrics']['ttc_compliance'],0)
        self.assertEqual(result['metrics']['emergency_ttc_compliance'],1)
        self.assertTrue(result['emergency_safety']['success'])
        self.assertEqual(result['score'],50.)
    def test_invalid_threshold_rejected_and_identity_changes(self):
        for invalid in [0,-1,float('nan')]:
            with self.assertRaises(ValueError):Protocol(emergency_ttc_horizon_s=invalid).validate()
        self.assertNotEqual(Protocol().identity(),Protocol(emergency_ttc_horizon_s=.75).identity())

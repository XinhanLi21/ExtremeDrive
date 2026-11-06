from ..benchmark.marking_compliance import marking_compliance
"""User-defined comfort scope. Never infer emergencies from model failures."""
import numpy as np

def comfort_applicability(split,category,source_episode=''):
    text=(category+'/'+source_episode).lower()
    if any(x in text for x in ('critical','emergency','evasive')):
        return dict(applicable=False,reason='emergency_maneuver')
    if split!='test':return dict(applicable=False,reason='not_test_split')
    return dict(applicable=True,reason='noncritical_test')

def scoped_total(base,road,comfort,consistency,applicable,*,omit_missing_ec=False,include_ec=True):
    gates=[base.get('no_collision'),marking_compliance(base)]+[1 if base.get('annotated_exclusion_compliance') is None else base['annotated_exclusion_compliance']]+[road.get('drivable_area_compliance'),road.get('driving_direction_compliance')]
    weighted=[(5,base.get('progress')),(5,base.get('ttc_compliance')),(2,consistency)]
    if not include_ec or (omit_missing_ec and consistency is None):weighted=weighted[:2]
    if applicable:weighted.append((2,comfort))
    if any(v==0 for v in gates):return 0.
    if any(v is None for v in gates) or any(v is None for _,v in weighted):return None
    return float(100*np.prod(gates)*sum(w*v for w,v in weighted)/sum(w for w,v in weighted))

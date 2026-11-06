"""Opt-in effective MoHe tire candidate; never silently promoted to default."""
from pathlib import Path
from dataclasses import replace
import json,hashlib
from .sl03 import SL03Config

def validation_gate(profile):
    if profile['heldout'] in profile['fit_runs']:raise ValueError('train/validation overlap')
    row=profile['evaluation'][profile['heldout']];old=row['rollout_before'];new=row['rollout_trajectory_fit']
    valid=new['windows']>=20 and new['windows']==old['windows']
    passed=valid and all(new[k]<=old[k] for k in ('vy_rmse_mps','yaw_rmse_deg_s'))
    return {'passed':passed,'criteria':'at least20 matched 1s windows; neither lateral-velocity nor yaw-rate RMSE may worsen','evidence_level':'conditional bicycle only; a pass would still need 7DOF validation','default_update':False}

def load_candidate(path,*,experimental=False,config=None):
    p=Path(path);data=json.loads(p.read_text());gate=validation_gate(data)
    if not experimental:raise ValueError('candidate not validated for full plant; explicit experimental=True required')
    c=config or SL03Config()
    for key,expected in [('mass',data['assumed_mass_kg']),('yaw_inertia',data['assumed_yaw_inertia_kgm2']),('lf',data['lf_m']),('lr',data['lr_m'])]:
        if abs(getattr(c,key)-expected)>1e-6:raise ValueError('candidate fit assumes '+key)
    cf,cr=data['trajectory_fit_axle_n_rad']
    c=replace(c,front_corner_stiffness=cf/2,rear_corner_stiffness=cr/2,parameter_status='MoHe_effective_lateral_candidate_not_validated_for_7DOF')
    return c,{'profile_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'gate':gate,'domain':data['domain'],'friction':'unchanged assumption, not identified','status':'experimental_only'}

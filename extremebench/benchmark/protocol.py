"""ExtremeDrive-specific protocol, inspired by PDMS, not NAVSIM compatibility."""
from dataclasses import dataclass, asdict, field
import hashlib,json,math
from .comfort import ComfortConfig

@dataclass(frozen=True)
class Protocol:
    version: str = 'extremedrive-execution-v2'
    actor_evidence_policy: str = 'annotated_3d_only'
    horizon_s: float = 4.0
    stride_s: float = 4.0
    sample_dt_s: float = 0.05
    max_observation_gap_s: float = 0.5
    white_tolerance_m: float = 0
    white_persistence_s: float = 0
    yellow_tolerance_m: float = 0
    yellow_persistence_s: float = 0
    event_merge_gap_s: float = 0.5
    ttc_horizon_s: float = 1.0
    emergency_ttc_horizon_s: float = 0.5
    ttc_step_s: float = 0.1
    comfort: dict = field(default_factory=lambda: asdict(ComfortConfig()))
    ego_outline: str = 'sl03_body_approx_v1'
    ego_length_m: float = 4.82
    ego_width_m: float = 1.89
    ego_center_forward_m: float = 0.0
    ego_center_left_m: float = 0.0
    ego_reference_status: str = 'editor_SL03_body_dimensions_origin_offset_unmeasured'
    progress_weight: float = 5.0
    ttc_weight: float = 5.0
    comfort_weight: float = 2.0
    # Optional measured limits. None means NOT assessed, never an automatic pass.
    measured_max_accel_mps2: float | None = None
    measured_max_decel_mps2: float | None = None
    measured_max_lateral_mps2: float | None = None

    def validate(self):
        if self.actor_evidence_policy != 'annotated_3d_only':raise ValueError('Unsupported actor evidence policy')
        ComfortConfig(**self.comfort).validate()
        for k,v in asdict(self).items():
            if isinstance(v,(int,float)) and not math.isfinite(v): raise ValueError(f'nonfinite {k}')
        for k in ('horizon_s','stride_s','sample_dt_s','max_observation_gap_s','ttc_horizon_s','emergency_ttc_horizon_s','ttc_step_s','ego_length_m','ego_width_m','progress_weight','ttc_weight','comfort_weight'):
            if getattr(self,k)<=0: raise ValueError(f'{k} must be positive')
        for k in ('white_tolerance_m','yellow_tolerance_m','white_persistence_s','yellow_persistence_s','event_merge_gap_s'):
            if getattr(self,k)<0: raise ValueError(f'{k} must be nonnegative')
        if max(self.white_tolerance_m,self.yellow_tolerance_m)>=min(self.ego_width_m,self.ego_length_m)/2: raise ValueError('tolerance erases vehicle')
        for k in ('measured_max_accel_mps2','measured_max_decel_mps2','measured_max_lateral_mps2'):
            if getattr(self,k) is not None and getattr(self,k)<=0: raise ValueError(f'invalid {k}')
        return self

    def identity(self):
        self.validate()
        values=asdict(self)
        if self.ego_outline=='rectangle':values.pop('ego_outline')
        return hashlib.sha256(json.dumps(values,sort_keys=True).encode()).hexdigest()

"""Execute timestamped world-CG plans; initial state must be externally supplied."""
import numpy as np
from scipy.interpolate import PchipInterpolator
from ..core import rollout
from .sl03 import SL03Config,LQRTracker,LoadTransferVehicle

def execute(plan,initial_state,config=None):
    a=np.asarray(plan,float)
    if a.ndim!=2 or a.shape[1]!=4 or len(a)<3 or not np.isfinite(a).all() or a[0,0]!=0 or np.any(np.diff(a[:,0])<=0):raise ValueError('expected ordered finite [t,x,y,yaw] plan from t=0')
    if not all(k in initial_state for k in ('x','y','yaw','vx')):raise ValueError('observed initial pose/speed required; do not derive from future plan')
    t=a[:,0];xy=[PchipInterpolator(t,a[:,j]) for j in (1,2)]
    speed=np.hypot(xy[0].derivative()(t),xy[1].derivative()(t))
    # Preserve native timestamps: controller samples the continuous reference at 10 Hz.
    scene={'id':'execution','trajectory':np.column_stack([a,speed]).tolist(),'initial_state':initial_state}
    result=rollout(scene,config or SL03Config(),tracker_cls=LQRTracker,vehicle_cls=LoadTransferVehicle)
    result['initial_state_supplied']=dict(initial_state)
    result['initial_state_assumed_fields']=[k for k in ('vy','yaw_rate','wheel_radps','front_steer_rad') if k not in initial_state]
    result['execution_status']='failed' if result['failure'] else 'experimental_unidentified_dynamics'
    result['assumptions']={'coordinate':'world CG, x forward y left','initial_unsupplied_fields':'missing vy/yaw_rate=0; missing wheel speeds=vx/r; missing front_steer_rad=0; initial drive/brake torque=0', 'traffic':'non-reactive replay; planner not rerun','controller':'custom LQR lateral + PI longitudinal, not upstream NAVSIM'}
    return result

def execute_mohe(plan,history,calibration_path,config=None):
    """Local-frame plan + measured CSV history. Reject unsupported domains.
    Last history frame is the anchor; caller must synchronize plan/telemetry.
    """
    from .calibration import SteeringCalibration,calibration_domain
    from .mohe_signals import normalize_frame
    import json
    from pathlib import Path
    c=config or SL03Config();cal=SteeringCalibration.load(calibration_path)
    if not np.isclose(c.lf+c.lr,cal.wheelbase_m):raise ValueError('wheelbase differs from calibration assumption')
    a=np.asarray(plan,float)
    if a.ndim!=2 or a.shape[1]!=4 or not np.allclose(a[0],[0,0,0,0]):raise ValueError('MoHe plan must start at local [t,x,y,yaw]=0')
    if len(history)<2:raise ValueError('two past telemetry frames required')
    n=normalize_frame(history,json.loads(Path(calibration_path).read_text()));dt=n['time_s'][-1]-n['time_s'][-2]
    if not 0<dt<=.15:raise ValueError('telemetry gap')
    beta=np.arctan2(n['vy_gps_mps'][-1],n['vx_gps_mps'][-1]);rate=n['yaw_rate_ccw_rad_s'][-1]
    dv=(n['gps_speed_mps'][-1]-n['gps_speed_mps'][-2])/dt;dr=(rate-n['yaw_rate_ccw_rad_s'][-2])/dt
    if not calibration_domain(n['gps_speed_mps'][-1],beta,rate,dv,dr):raise ValueError('outside low-slip calibration domain; drift evaluation needs separately validated plant')
    initial=dict(x=0.,y=0.,yaw=0.,vx=float(n['vx_gps_mps'][-1]),vy=float(n['vy_gps_mps'][-1]),yaw_rate=float(rate),front_steer_rad=float(n['front_steer_candidate_rad'][-1]))
    result=execute(plan,initial,c);result['signal_calibration']=cal.metadata();result['telemetry_anchor_s']=float(n['time_s'][-1]);result['assumptions']['telemetry']='GPS antenna velocity without CG lever-arm correction; candidate steering conversion; wheel angular speeds and initial torques not measured';return result

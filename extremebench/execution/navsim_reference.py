"""Pinned upstream LQR/kinematic engine; CG<->rear axle conversion explicit.
Run in hdp-navsim environment. Vehicle adaptation is NOT official leaderboard config.
"""
from pathlib import Path
import sys,subprocess
import numpy as np
from scipy.interpolate import PchipInterpolator
PIN='c0b6fb05d9060f665dc98830285fe236312dc2ba'

def execute_navsim(plan,initial,c,root='/data/lxh/hdp-navsim/navsim-v2.0'):
    root=Path(root)
    if subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()!=PIN:raise ValueError('NAVSIM revision mismatch')
    simdir='navsim/planning/simulation/planner/pdm_planner/simulation'
    if subprocess.check_output(['git','diff','HEAD','--',simdir],cwd=root,text=True).strip():raise ValueError('modified upstream simulation sources')
    sys.path.insert(0,str(root))
    from navsim.planning.simulation.planner.pdm_planner.simulation.pdm_simulator import PDMSimulator
    from navsim.planning.simulation.planner.pdm_planner.simulation.batch_lqr import BatchLQRTracker
    from navsim.planning.simulation.planner.pdm_planner.utils.pdm_enums import StateIndex
    from nuplan.common.actor_state.vehicle_parameters import VehicleParameters
    from nuplan.common.actor_state.ego_state import EgoState
    from nuplan.common.actor_state.state_representation import StateSE2,StateVector2D,TimePoint
    from nuplan.planning.simulation.trajectory.trajectory_sampling import TrajectorySampling
    a=np.array(plan,float,copy=True);a[:,3]=np.unwrap(a[:,3]);dt=.1;n=round(a[-1,0]/dt)
    if a[0,0]!=0 or not np.isclose(n*dt,a[-1,0]):raise ValueError('horizon must align with 10 Hz')
    t=np.arange(n+1)*dt;pose=PchipInterpolator(a[:,0],a[:,1:],axis=0)(t)
    # Inputs and outputs use assumed CG; upstream uses rear axle.
    pose[:,0]-=c.lr*np.cos(pose[:,2]);pose[:,1]-=c.lr*np.sin(pose[:,2])
    vehicle=VehicleParameters(c.width,c.length/2+c.lr,c.length/2-c.lr,c.lr,c.lf+c.lr,'SL03_approx','sedan')
    rear=StateSE2(initial['x']-c.lr*np.cos(initial['yaw']),initial['y']-c.lr*np.sin(initial['yaw']),initial['yaw'])
    ego=EgoState.build_from_rear_axle(rear,StateVector2D(initial['vx'],0),StateVector2D(0,0),0,TimePoint(0),vehicle)
    sim=PDMSimulator(TrajectorySampling(num_poses=n,interval_length=dt))
    # Upstream simulator sets plant vehicle but not tracker wheelbase: configure both.
    sim._tracker=BatchLQRTracker(vehicle=vehicle)
    states=np.zeros((1,n+1,StateIndex.size()));states[0,:,:3]=pose
    out=sim.simulate_proposals(states,ego)[0];cg=out[:,:3].copy()
    cg[:,0]+=c.lr*np.cos(cg[:,2]);cg[:,1]+=c.lr*np.sin(cg[:,2])
    return {'trajectory':np.column_stack([t,cg]).tolist(),'native_states':out.tolist(),'upstream_commit':PIN,'status':'upstream_engine_SL03_dimensions_not_official_leaderboard','initial_assumptions':'zero acceleration, steering, lateral velocity'}

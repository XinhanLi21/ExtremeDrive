import json,re
from datetime import datetime
from pathlib import Path
import numpy as np

def synthetic():
    scenes=[]
    t=np.arange(0,8.00001,.1);x=12*t
    u=np.clip((t-1)/3,0,1);y=3.5*(10*u**3-15*u**4+6*u**5)
    yd=np.gradient(y,t);yaw=np.arctan2(yd,12);v=np.hypot(12,yd)
    points=np.column_stack([t,x,y,yaw,v]).tolist()
    for name,mu in [('lane_change_nominal',.85),('lane_change_low_mu',.18)]:
        scenes.append({'id':name,'provenance':'synthetic_diagnostic_not_dataset_performance','mu':mu,'trajectory':points,
         'drivable_polygon':[[-10,-3.5],[110,-3.5],[110,7.0],[-10,7.0]],'obstacles':[]})
    t=np.arange(0,8.00001,.1);v=np.maximum(15-3*t,0);x=np.where(t<5,15*t-1.5*t*t,37.5)
    for name,mu in [('braking_nominal',.85),('braking_low_mu',.18)]:
        scenes.append({'id':name,'provenance':'synthetic_diagnostic_not_dataset_performance','mu':mu,
        'trajectory':np.column_stack([t,x,0*t,0*t,v]).tolist(),
        'drivable_polygon':[[-10,-3.5],[100,-3.5],[100,3.5],[-10,3.5]],
        'obstacles':[{'id':'stopped_lead_vehicle','length':4.7,'width':1.85,'poses':[[0,46,0,0]]}]})
    return scenes

def from_metadata(path):
    """Integrate documented NEXT-frame body-relative [dx,dy,dyaw].
    Uses actual frame timestamps. Does not infer CAN wheel speed/tq units.
    The pose reference/mount offset still needs calibration before formal use.
    """
    path=Path(path);rows=[json.loads(s) for s in path.read_text().splitlines() if s.strip()]
    if len(rows)<3:raise ValueError('at least 3 frames required')
    dates=[]
    for row in rows:
        name=next(v['url'] for k,v in row.items() if k.startswith('images_'))
        m=re.search(r'(\d{8}_\d{6}_\d{6})',name)
        if not m:raise ValueError('frame timestamp missing')
        dates.append(datetime.strptime(m.group(1),'%Y%m%d_%H%M%S_%f'))
    t=np.array([(d-dates[0]).total_seconds() for d in dates]);pose=np.zeros((len(rows),3))
    for i in range(len(rows)-1):
        dx,dy,dyaw=rows[i]['action'];a=pose[i,2]
        pose[i+1]=pose[i]+[np.cos(a)*dx-np.sin(a)*dy,np.sin(a)*dx+np.cos(a)*dy,dyaw]
    if np.any(np.diff(t)<=0):raise ValueError('nonmonotonic dataset timestamps')
    speed=np.r_[np.linalg.norm(np.diff(pose[:,:2],axis=0),axis=1)/np.diff(t),0.]
    speed[-1]=speed[-2]
    return {'id':'extreme_episode_189_part3' if path.parent.name=='episode_189_part3' else 'extreme_'+path.parent.name,
        'provenance':'recorded_reference_trajectory_with_unidentified_vehicle_parameters',
        'source':str(path),'timestamp_source':'released image filename timestamps',
        'reference_note':'action increments composed in local frame; first frame origin; forward x/left y assumption; pose origin provisionally treated as CG; speed derived from positions, not raw CAN',
        'environment_metrics':'unavailable: registered map and object tracks not supplied; not interpreted as safe',
        'trajectory':np.column_stack([t,pose,speed]).tolist()}

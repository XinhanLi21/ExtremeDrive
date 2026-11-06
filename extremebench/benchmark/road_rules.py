"""Compliance with reviewed drivable surfaces and road direction."""
import math
import numpy as np
from shapely.geometry import shape
from .ego_geometry import ego_footprints

def evaluate(rows,road,p):
    a=np.asarray(rows,float);g=shape(road['drivable_area']);bodies=ego_footprints(a[:,1:],p)
    # Apply the white-line graze tolerance also at the inferred road edge.
    inside=np.array([g.buffer(.15).covers(b) for b in bodies]);ss=road['sections'];xy=np.array([s['q'] for s in ss]);wrong=[]
    for r in a:
        s=ss[int(np.argmin(np.linalg.norm(xy-r[1:3],axis=1)))];q=np.array(s['q']);n=np.array([-math.sin(s['yaw']),math.cos(s['yaw'])]);lat=float((r[1:3]-q)@n)
        flips=sum(min(0,lat)<y<max(0,lat) for y in s['yellows']);sign=-1 if flips%2 else 1
        wrong.append(sign*math.cos(r[3]-s['yaw'])<0)
    travel=np.r_[0,np.linalg.norm(np.diff(a[:,1:3],axis=0),axis=1)]*wrong
    reverse=max(float(travel[(a[:,0]>=t-1)&(a[:,0]<=t)].sum()) for t in a[:,0])
    return dict(drivable_area_compliance=int(inside.all()),driving_direction_compliance=1. if reverse<2 else .5 if reverse<6 else 0.,max_wrong_way_distance_1s_m=reverse,offroad_time_fraction=float((~inside).mean()))


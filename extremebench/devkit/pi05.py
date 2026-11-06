"""Coordinate-only adapter for the native pi0.5 ten-waypoint output.
No time stretching, action integration, or future trajectory extrapolation.
"""
import math
import numpy as np

def waypoints_to_scene(anchor,times_s,waypoints):
    a=np.asarray(waypoints,float);times=np.asarray(times_s,float);origin=np.asarray(anchor,float)
    if a.shape!=(10,3) or times.shape!=(10,) or origin.shape!=(3,):raise ValueError('expected 10 absolute ego waypoints, 10 timestamps and initial pose')
    if not np.isfinite(a).all() or not np.isfinite(times).all() or not np.isfinite(origin).all():raise ValueError('nonfinite pi05 input')
    delta=np.diff(np.r_[0,times])
    if np.any(delta<=0) or np.any(delta>.5+1e-8):raise ValueError('invalid native observation times')
    x,y,yaw=origin;c,s=math.cos(yaw),math.sin(yaw);xy=a[:,:2]@np.array([[c,s],[-s,c]])+[x,y]
    return [[0.,float(x),float(y),float(yaw)]]+np.column_stack([times,xy,a[:,2]+yaw]).tolist()

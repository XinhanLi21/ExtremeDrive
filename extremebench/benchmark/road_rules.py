"""Compliance with the reviewed drivable surface."""
import numpy as np
from shapely.geometry import shape
from .ego_geometry import ego_footprints

def evaluate(rows,road,p):
    a=np.asarray(rows,float)
    surface=shape(road['drivable_area']).buffer(.15)
    bodies=ego_footprints(a[:,1:],p)
    inside=np.array([surface.covers(body) for body in bodies])
    return dict(drivable_area_compliance=int(inside.all()),offroad_time_fraction=float((~inside).mean()))

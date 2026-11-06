"""Shared footprint geometry helpers."""
import math
import numpy as np
from shapely import polygons

def corners(x,y,yaw,length,width):
    q=np.array([[length/2,width/2],[length/2,-width/2],[-length/2,-width/2],[-length/2,width/2]])
    return q@np.array([[math.cos(yaw),math.sin(yaw)],[-math.sin(yaw),math.cos(yaw)]])+[x,y]


def footprint_array(poses,length,width):
    x,y,yaw=poses[:,0],poses[:,1],poses[:,2]
    local=np.array([[length/2,width/2],[length/2,-width/2],[-length/2,-width/2],[-length/2,width/2]])
    q=np.empty((len(poses),4,2));co=np.cos(yaw);si=np.sin(yaw)
    q[:,:,0]=x[:,None]+co[:,None]*local[:,0]-si[:,None]*local[:,1]
    q[:,:,1]=y[:,None]+si[:,None]*local[:,0]+co[:,None]*local[:,1]
    return polygons(q)


"""Versioned body-shell approximation, NOT an OEM CAD/measured SL03 outline."""
import numpy as np
from shapely.geometry import Polygon
from shapely import polygons
from ..metrics import footprint_array
# Body center, forward x/left y. Maximum dimensions retained, corners tapered.
SL03_VERTICES=np.array([[2.41,0],[2.39,.55],[2.31,.77],[2.12,.89],[1.80,.935],[1.35,.945],[-1.45,.945],[-1.91,.92],[-2.23,.84],[-2.38,.64],[-2.41,.36],[-2.41,-.36],[-2.38,-.64],[-2.23,-.84],[-1.91,-.92],[-1.45,-.945],[1.35,-.945],[1.80,-.935],[2.12,-.89],[2.31,-.77],[2.39,-.55]])
def ego_footprints(poses,p,inset=0.):
 if getattr(p,'ego_outline','rectangle')=='rectangle':return footprint_array(poses,p.ego_length_m-2*inset,p.ego_width_m-2*inset)
 if p.ego_outline!='sl03_body_approx_v1':raise ValueError('unknown ego outline')
 local=SL03_VERTICES*np.array([p.ego_length_m/4.82,p.ego_width_m/1.89])
 if inset:
  g=Polygon(local).buffer(-inset,join_style=2)
  if g.is_empty or g.geom_type!='Polygon':raise ValueError('invalid inset')
  local=np.array(g.exterior.coords)[:-1]
 a=np.asarray(poses);co=np.cos(a[:,2]);si=np.sin(a[:,2]);q=np.empty((len(a),len(local),2));q[:,:,0]=a[:,0,None]+co[:,None]*local[:,0]-si[:,None]*local[:,1];q[:,:,1]=a[:,1,None]+si[:,None]*local[:,0]+co[:,None]*local[:,1]
 return polygons(q)

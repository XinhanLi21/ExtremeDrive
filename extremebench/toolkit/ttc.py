import numpy as np
from shapely.affinity import translate
from shapely.ops import unary_union
from ..benchmark import scoring
from .actors import AdjacentActors
LIMITS={'person':10.,'bicycle':18.,'motorcycle':40.,'car':55.,'truck':40.,'bus':40.}
def definitely_irrelevant(body,poly,ego_velocity,horizon,speed_cap,margin=.5):
 # Swept ego polygon covers every intermediate CV position. Any target direction up to class speed cap is permitted.
 end=translate(body,xoff=ego_velocity[0]*horizon,yoff=ego_velocity[1]*horizon)
 swept=unary_union([body,end]).convex_hull
 distance=swept.distance(poly)
 return distance>speed_cap*horizon+margin, distance


def revise_ttc(r,c,P):
 frames=c['scene']['frames'];actors=AdjacentActors(frames,P)
 a=np.array(r['trajectory']);t=a[:,0];poses=a[:,1:].copy();co=np.cos(poses[:,2]);si=np.sin(poses[:,2]);poses[:,0]+=co*P.ego_center_forward_m-si*P.ego_center_left_m;poses[:,1]+=si*P.ego_center_forward_m+co*P.ego_center_left_m
 bodies=scoring.ego_footprints(poses,P);velocity=np.gradient(poses[:,:2],t,axis=0);risk=False;missing=0;unknown=0;ignored=0;total_unknown=0;min_ttc=None;decisions=[]
 classes={o['id']:o['class'] for f in frames for o in f['objects']}
 for i,time in enumerate(t):
  obs=actors.at(time)
  if obs is None:missing+=1;continue
  for ident,poly,v in obs:
   # A current overlap establishes TTC=0 even when velocity is unavailable.
   if bodies[i].intersection(poly).area>1e-6:risk=True;min_ttc=0.
   if v is None:
    total_unknown+=1;cap=LIMITS.get(classes.get(ident),55.)
    safe,distance=definitely_irrelevant(bodies[i],poly,velocity[i],P.ttc_horizon_s,cap)
    ignored+=int(safe);unknown+=int(not safe)
    decisions.append(dict(t=float(time),id=ident,excluded=safe,distance_to_ego_sweep_m=distance,actor_reach_m=cap*P.ttc_horizon_s,margin_m=.5))
    continue
   if bodies[i].distance(poly)>np.linalg.norm(velocity[i]-v)*P.ttc_horizon_s+1:continue
   for tau in np.arange(0,P.ttc_horizon_s+P.ttc_step_s/2,P.ttc_step_s):
    ego=translate(bodies[i],xoff=velocity[i,0]*tau,yoff=velocity[i,1]*tau);other=translate(poly,xoff=v[0]*tau,yoff=v[1]*tau)
    if ego.intersection(other).area>1e-6:
     risk=True;min_ttc=float(tau) if min_ttc is None else min(min_ttc,float(tau));break
 value=0 if risk else None if missing or unknown else 1
 r["metrics"]["ttc_compliance"]=value
 r["ttc_evidence"]=dict(missing_samples=missing,relevant_unknown_samples=unknown,excluded_unreachable_samples=ignored,min_ttc_s=min_ttc,actor_evidence_policy=P.actor_evidence_policy,unknown_reasons=(["missing_temporal_3d_observation"] if missing else [])+(["risk_relevant_3d_velocity_unavailable"] if unknown else []))
 return r

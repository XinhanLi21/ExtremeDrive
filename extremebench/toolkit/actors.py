import numpy as np
from ..benchmark.scoring import Actors as Original
class AdjacentActors(Original):
 def at(self,t):
  result=super().at(t)
  if result is None:return None
  out=[]
  for ident,poly,v in result:
   # Prefer stable multi-frame fit over pairwise differencing even when both IDs match.
   candidates=[(abs(f['t']-t),o) for f in self.frames if abs(f['t']-t)<=.5 for o in f['objects'] if o['id']==ident and o.get('adjacent_velocity') is not None]
   if candidates:v=np.array(min(candidates,key=lambda z:z[0])[1]['adjacent_velocity'])
   out.append((ident,poly,v))
  return out

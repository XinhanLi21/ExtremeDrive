"""Bounded arc-length retiming of native predictions; no labels or future state."""
import numpy as np
from scipy.optimize import minimize,LinearConstraint,Bounds

def smooth_waypoints(poses,times_s,max_shift_m=.35):
    p=np.vstack([np.zeros(3),np.asarray(poses,float)]);t=np.r_[0,np.asarray(times_s,float)]
    if p.shape!=(len(t),3) or not np.isfinite(p).all() or not np.allclose(np.diff(t),.25):raise ValueError('finite native 0.25 s poses required')
    s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1))]
    if s[-1]<.1:return p[1:].tolist(),{'status':'stationary_identity'}
    n=len(s);d=np.diff(np.eye(n),n=3,axis=0)/.25**3
    # Fixed engineering regularizer, not calibrated on test outcomes.
    q=np.eye(n)/.25**2+d.T@d/4.13**2
    def fun(x):return float(np.sum(((x-s)/.25)**2)+np.sum((d@x/4.13)**2))
    def jac(x):return 2*(q@x-s/.25**2)
    lo=np.maximum(0,s-max_shift_m);hi=np.minimum(s[-1],s+max_shift_m);lo[0]=hi[0]=0;lo[-1]=hi[-1]=s[-1]
    opt=minimize(fun,s,jac=jac,bounds=Bounds(lo,hi),constraints=[LinearConstraint(np.diff(np.eye(n),axis=0),0,np.inf)],method='SLSQP',options={'maxiter':150,'ftol':1e-8})
    if not opt.success:return p[1:].tolist(),{'status':'solver_fallback','message':opt.message}
    unique=np.r_[True,np.diff(s)>1e-9];sp=s[unique];pp=p[unique];pp[:,2]=np.unwrap(pp[:,2]);out=np.column_stack([np.interp(opt.x,sp,pp[:,i]) for i in range(3)]);out[0]=p[0];out[-1]=p[-1]
    return out[1:].tolist(),dict(status='bounded_path_retiming',max_arc_shift_m=float(np.max(abs(opt.x-s))),raw_objective=fun(s),objective=fun(opt.x),endpoint_preserved=True)

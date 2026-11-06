"""Additional BEV detection and multimodal forecasting metrics.
These are ExtremeDrive tasks, not official nuScenes NDS/AMOTA.
"""
import numpy as np
from scipy.optimize import linear_sum_assignment

def forecast_metrics(predictions,truth,probabilities=None,miss_threshold_m=2.):
    pred=np.asarray(predictions,float);gt=np.asarray(truth,float)
    if pred.ndim!=3 or pred.shape[1:]!=gt.shape or gt.ndim!=2 or gt.shape[1]!=2 or not pred.shape[0]:raise ValueError('expected K,T,2 predictions and T,2 truth')
    if not np.isfinite(pred).all() or not np.isfinite(gt).all():raise ValueError('nonfinite trajectories')
    errors=np.linalg.norm(pred-gt,axis=-1);ade=errors.mean(axis=-1);fde=errors[:,-1];best=int(np.argmin(fde))
    result={'minADE_m':float(ade.min()),'minFDE_m':float(fde.min()),'miss_rate':float(fde.min()>miss_threshold_m),'modes':len(pred)}
    if probabilities is not None:
        prob=np.asarray(probabilities,float)
        if prob.shape!=(len(pred),) or not np.isfinite(prob).all() or np.any(prob<0) or not np.isclose(prob.sum(),1):raise ValueError('invalid mode probabilities')
        result['brier_minFDE']=float(fde[best]+(1-prob[best])**2)
    return result

def detection_metrics(truth,predictions,thresholds=(.5,1.,2.,4.)):
    """Frame mappings to objects {class,xy:[center x,y],score}. Unknown frames rejected.
    Greedy per-class matching sorted by confidence; 101-point interpolated AP.
    Empty frames must be explicitly present. No height/velocity/attribute score.
    """
    if set(truth)!=set(predictions):raise ValueError('prediction frame set must exactly match ground truth')
    for frames,scored in ((truth,False),(predictions,True)):
        for objects in frames.values():
            for o in objects:
                if np.asarray(o['xy']).shape!=(2,) or not np.isfinite(o['xy']).all():raise ValueError('invalid center')
                if scored and (not np.isfinite(o['score']) or not 0<=o['score']<=1):raise ValueError('invalid confidence')
    classes=sorted({o['class'] for objects in truth.values() for o in objects});report={}
    for cls in classes:
        gt={k:[o for o in obs if o['class']==cls] for k,obs in truth.items()};n=sum(map(len,gt.values()));detections=sorted([(o['score'],frame,i,o) for frame,obs in predictions.items() for i,o in enumerate(obs) if o['class']==cls],key=lambda x:(-x[0],x[1],x[2]));ap={};errors=[]
        for threshold in thresholds:
            if threshold<=0:raise ValueError('threshold must be positive')
            matched={k:set() for k in gt};tp=[]
            for _,frame,_,obj in detections:
                candidates=[(float(np.linalg.norm(np.array(obj['xy'])-o['xy'])),i) for i,o in enumerate(gt[frame]) if i not in matched[frame]]
                distance,index=min(candidates,default=(float('inf'),-1));ok=distance<=threshold;tp.append(int(ok))
                if ok:
                    matched[frame].add(index)
                    if threshold==2.:errors.append(distance)
            cum=np.cumsum(tp);precision=cum/np.arange(1,len(tp)+1);recall=cum/max(n,1)
            ap[str(threshold)]=float(np.mean([precision[recall>=r].max(initial=0) for r in np.linspace(0,1,101)])) if len(tp) else 0.
        report[cls]={'AP':ap,'translation_error_m_at_2m':float(np.mean(errors)) if errors else None,'gt_count':n}
    return {'task':'BEV center detection','classes':report,'mAP':float(np.mean([v for c in report.values() for v in c['AP'].values()])) if report else None,'not_nuscenes_nds':True}

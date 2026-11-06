"""Explicit, revision-checked road surface layer. Never silently fall back."""
import json,math
from pathlib import Path
import numpy as np
from shapely.geometry import shape,LineString
from ..benchmark.dataset import digest

def load_surface(scene,window,root):
    doc=json.loads((Path(root)/'scenes'/f"{scene['scene_id']}.json").read_text())
    if doc['scene_id']!=scene['scene_id'] or doc['annotation_sha256']!=scene['annotation_sha256']:
        raise ValueError('Manual surface does not match this annotation revision')
    if digest(doc['segments'])!=doc['surface_sha256']:
        raise ValueError('Manual surface integrity check failed')
    matches=[s for s in doc['segments'] if s['source_key']==window['source_key']]
    if len(matches)!=1:raise ValueError('No unique reviewed surface for source episode')
    seg=matches[0];sections=seg['sections']
    if window['start_s']<sections[0]['t']-1e-6 or window['end_s']>sections[-1]['t']+1e-6:
        raise ValueError('Window extends beyond reviewed temporal coverage')
    g=shape(seg['geometry'])
    if g.is_empty or not g.is_valid:raise ValueError('Invalid manual road polygon')
    yellows=[shape(f['geometry']) for f in scene['features'] if f['properties']['class_id'] in (8,9)]
    output=[]
    for s in sections:
        q=np.asarray(s['q']);n=np.array([-math.sin(s['yaw']),math.cos(s['yaw'])]);cut=LineString([q-30*n,q+30*n]);ys=[]
        for yellow in yellows:
            hit=yellow.intersection(cut)
            for part in (hit.geoms if hasattr(hit,'geoms') else [hit]):
                if not part.is_empty:ys.append(float((np.array(part.centroid.coords[0])-q)@n))
        merged=[]
        for y in sorted(ys):
            if not merged or y-merged[-1]>.45:merged.append(y)
        output.append({**s,'yellows':merged})
    return dict(status='agent_visual_review_draft',drivable_area=seg['geometry'],sections=output,
                basis='explicitly selected boundary chains and recorded manual controls',
                surface_sha256=doc['surface_sha256'],annotation_sha256=doc['annotation_sha256'],
                source_key=seg['source_key'],review_note=seg['note'])

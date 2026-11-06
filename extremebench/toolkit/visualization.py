"""Offline BEV inspection in the same map frame as evaluation."""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from shapely.geometry import shape,box
from .data import load

def visualize(ds,result_path,out):
    result=load(result_path);ctx=ds.context(result['token']);traj=np.asarray(result['trajectory']);xy=traj[:,1:3]
    bounds=box(xy[:,0].min()-12,xy[:,1].min()-12,xy[:,0].max()+12,xy[:,1].max()+12)
    fig,ax=plt.subplots(figsize=(11,8));ax.set_facecolor('#16232e')
    def draw(g,color,fill=False,alpha=1):
        if g.is_empty:return
        if hasattr(g,'geoms'):
            for part in g.geoms:draw(part,color,fill,alpha)
        elif g.geom_type=='Polygon':
            p=np.asarray(g.exterior.coords);ax.fill(p[:,0],p[:,1],color=color,alpha=alpha) if fill else ax.plot(p[:,0],p[:,1],color=color,lw=1)
        elif g.geom_type in ('LineString','LinearRing'):
            p=np.asarray(g.coords);ax.plot(p[:,0],p[:,1],color=color,lw=1)
    draw(shape(ctx['road']['drivable_area']).intersection(bounds),'#46535c',True)
    for feature in ctx['scene']['features']:
        geom=shape(feature['geometry'])
        if not geom.intersects(bounds):continue
        prop=str(feature.get('properties',{})).lower();draw(geom.intersection(bounds),'#ffcf44' if 'yellow' in prop else '#d8e5ed')
    frame=min(ctx['scene']['frames'],key=lambda f:abs(f['t']))
    for obj in frame['objects']:
        p=np.asarray(obj['xy']);ax.fill(p[:,0],p[:,1],color='#ef9749',alpha=.8)
    ax.plot(xy[:,0],xy[:,1],color='#28e3c1',lw=2,label='Executed ego trajectory')
    from ..benchmark.scoring import ego_footprints
    from ..benchmark.protocol import Protocol
    bodies=ego_footprints(traj[:,1:],Protocol(**load(ds.assets/'protocol.json')))
    for body in bodies[::10]:draw(body,'#28e3c1',False,.5)
    ax.set_xlim(bounds.bounds[0],bounds.bounds[2]);ax.set_ylim(bounds.bounds[1],bounds.bounds[3]);ax.set_aspect('equal');ax.set_xlabel('Scene x [m]');ax.set_ylabel('Scene y [m]');ax.set_title(result['token']+' | ego execution + actors at anchor');ax.legend();fig.tight_layout();fig.savefig(out,dpi=160);plt.close(fig)

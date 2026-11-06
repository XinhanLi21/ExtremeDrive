from pathlib import Path
import csv,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from .metrics import corners

def save_case(scene,result,metrics,out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);name=scene['id']
    with (out/(name+'.csv')).open('w') as f:
        wr=csv.DictWriter(f,fieldnames=list(result['logs'][0]));wr.writeheader();wr.writerows(result['logs'])
    payload={k:v for k,v in result.items() if k!='geometry'};payload.update(metrics=metrics,scene=scene)
    (out/(name+'.json')).write_text(json.dumps(payload,ensure_ascii=False,indent=2,allow_nan=False))
    a=lambda k:np.array([r.get(k) for r in result['logs']],dtype=float);t=a('t');fig,ax=plt.subplots(2,2,figsize=(12,7),layout='constrained')
    road=scene.get('drivable_polygon')
    if road:ax[0,0].add_patch(Polygon(road,color='#edf2f8'))
    for ob in scene.get('obstacles',[]):ax[0,0].add_patch(Polygon(corners(*ob['poses'][0][1:],ob['length'],ob['width']),color='#ba5b50'))
    ax[0,0].plot(a('ref_x'),a('ref_y'),'--',c='#8091a8',label='Reference');ax[0,0].plot(a('x'),a('y'),c='#145aac',label='Executed');ax[0,0].set_title('Trajectory (m)');ax[0,0].set_aspect('equal',adjustable='datalim');ax[0,0].legend(fontsize=9)
    ax[0,1].plot(t,np.rad2deg(a('steer_request')),'--',c='#8091a8',label='Requested');ax[0,1].plot(t,np.rad2deg(a('steer_actual')),c='#145aac',label='Applied');ax[0,1].set_title('Road-wheel steering (deg)');ax[0,1].legend(fontsize=9)
    if result.get('backend')=='carla':
        ax[1,0].plot(t,a('native_throttle'),c='#16866d',label='Native throttle');ax[1,0].plot(t,a('native_brake'),c='#ba5b50',label='Native brake')
        ax[1,0].set_title('Native controls [0,1]; wheel drive/brake torques unavailable')
    else:
        for key,col,lab in [('drive','#16866d','Model drive / driveline'),('brake','#ba5b50','Model brake')]:ax[1,0].plot(t,sum(a(key+'_'+w) for w in ['FL','FR','RL','RR']),c=col,label=lab)
        ax[1,0].set_title('Model wheel torque (N m)')
    ax[1,0].legend(fontsize=9)
    for w,col in zip(['FL','FR','RL','RR'],['#145aac','#e29433','#16866d','#885bb0']):ax[1,1].plot(t,a('omega_'+w),label=w,color=col)
    ax[1,1].set_title('Wheel angular velocity (rad/s)');ax[1,1].legend(fontsize=8,ncol=4)
    for a0 in ax.flat:a0.grid(alpha=.2)
    for a0 in [ax[0,1],ax[1,0],ax[1,1]]:a0.set_xlabel('Time (s)')
    fig.suptitle(name+' | '+result.get('backend','prototype')+' / uncalibrated vehicle',fontsize=14,color='#002060');fig.savefig(out/(name+'.png'),dpi=145);plt.close(fig)
    return {'scene':scene,'metrics':metrics,'config':result['config'],'config_id':result['config_id'],'backend':result.get('backend','prototype'),'backend_metadata':result.get('backend_metadata',{}),'logs':result['logs'][::2],'plot':name+'.png','csv':name+'.csv','json':name+'.json'}

def gallery(items,out):
    out=Path(out);(out/'cases.js').write_text('window.CASES='+json.dumps(items,ensure_ascii=False,allow_nan=False)+';')
    template=Path(__file__).with_name('viewer.html').read_text()
    if items and items[0].get('backend','prototype')!='prototype':
        import re,html
        metadata=items[0].get('backend_metadata',{});backend=items[0]['backend']
        template=template.replace('从规划轨迹到车辆执行','开源后端：'+html.escape(backend))
        template=template.replace('固定控制器，把同一条参考轨迹交给不同附着条件下的车辆，比较它实际跑出的结果。','实际运行外部车辆动力学引擎；沿用轨迹、控制算法形式和评分接口。各后端原生车型、执行器映射与轮胎不同，此处验证接入，不作模型优劣排名。')
        notes='引擎：'+str(metadata.get('engine'))+'。车型：'+str(metadata.get('vehicle'))+'。原生控制：'+str(metadata.get('native_controls'))+'。适配：'+str(metadata.get('adapter',metadata.get('actuator_adapter','')))+'。边界：'+'；'.join(metadata.get('limitations',[]))
        template=re.sub(r'<p class="note">这是可运行.*?</p>',lambda _: '<p class="note">'+html.escape(notes)+'</p>',template,flags=re.S)
    (out/'index.html').write_text(template)

"""Regenerate standalone backend galleries and a factual integration summary."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from .report import save_case,gallery

BACKENDS={'chrono':('Chrono::Vehicle','悬架、轮胎与四轮响应','底盘模型与控制研究'),
          'commonroad_mb':('CommonRoad MB','29 状态多体模型','轻量批量评测与对照'),
          'carla':('CARLA','同步道路场景与原生车辆','场景交互与规划闭环接入')}
SCENES={'lane_change_nominal':'正常附着 · 变道','lane_change_low_mu':'低附着 · 变道','braking_nominal':'正常附着 · 制动','braking_low_mu':'低附着 · 制动','extreme_episode_189_part3':'真实数据 · 189 part3'}

def main():
    out=Path('outputs/backend_trials');summaries=[];cards=[];rows=[]
    fig,axes=plt.subplots(1,3,figsize=(13,3.7),layout='constrained')
    for ax,(key,(label,model,role)) in zip(axes,BACKENDS.items()):
        folder=out/key;summary=json.loads((folder/'summary.json').read_text());summaries.extend(summary);items=[]
        for record in summary:
            path=folder/(record['id']+'.json')
            if path.exists():
                data=json.loads(path.read_text());items.append(save_case(data['scene'],data,data['metrics'],folder))
            source=record['id'].split('__')[-1];rmse=record.get('tracking_rmse_m');collision=record.get('collision')
            rows.append(f'<tr><td>{label}</td><td>{SCENES.get(source,source)}</td><td>{"完成" if record["status"]=="completed" else "失败"}</td><td>{rmse:.3f}</td><td>{"未评估" if collision is None else "发生" if collision else "无"}</td></tr>' if rmse is not None else f'<tr><td>{label}</td><td>{source}</td><td colspan="3">失败，见原始记录</td></tr>')
        gallery(items,folder)
        count=sum(r['status']=='completed' for r in summary)
        cards.append(f'<article><span class="badge">已实跑 {count} / {len(summary)}</span><h3>{label}</h3><p>{model}</p><p class="role">{role}</p><a class="button" href="{key}/index.html">查看轨迹与四轮响应 →</a></article>')
        for scene,col,legend in [('braking_nominal','#2364ad','Normal adhesion'),('braking_low_mu','#c35b43','Low adhesion')]:
            data=json.loads((folder/(key+'__'+scene+'.json')).read_text());logs=data['logs']
            ax.plot([r['t'] for r in logs],[r['vx'] for r in logs],c=col,label=legend)
        ax.plot([r['t'] for r in logs],[r['ref_v'] for r in logs],'--',c='#8d98a5',label='Reference')
        ax.set(title=label,xlabel='Time (s)',ylabel='Longitudinal speed (m/s)');ax.grid(alpha=.18);ax.legend(fontsize=8)
    fig.savefig(out/'braking_comparison.png',dpi=160);plt.close(fig)
    (out/'summary.json').write_text(json.dumps(summaries,ensure_ascii=False,indent=2,allow_nan=False))
    done=sum(s['status']=='completed' for s in summaries)
    page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>ExtremeDrive · 开源控制仿真</title>
<style>*{box-sizing:border-box}body{margin:0;background:#f5f7fa;color:#15283b;font:16px/1.8 system-ui,-apple-system,"Microsoft YaHei",sans-serif}header{background:#142c45;color:white;padding:60px max(6vw,20px)}header .inner,main{max-width:1200px;margin:auto}small,.eyebrow{letter-spacing:.12em;color:#87c8d2}h1{font-size:clamp(30px,4vw,48px);line-height:1.25;margin:16px 0}header p{max-width:830px;color:#d1dce7}main{padding:25px 22px 65px}section{margin:38px 0}h2{font-size:26px;margin-bottom:12px}h3{font-size:24px;margin:15px 0 2px}.cards{display:grid;grid-template-columns:repeat(3,1fr);gap:20px}article{background:white;padding:25px;border:1px solid #e0e6ed;border-radius:14px}.badge{color:#237c68;font-size:14px}.role{color:#647283}.button{display:inline-block;margin-top:14px;color:#165e97;font-weight:600;text-decoration:none}img{width:100%;border-radius:10px;background:white}a{color:#1e649c}.flow{display:flex;gap:12px;flex-wrap:wrap;align-items:center;background:#eaf0f6;padding:25px;border-radius:12px}.flow b{padding:7px 15px;background:white;border-radius:6px}.note{color:#607083;font-size:14px}.tablewrap{overflow:auto}table{border-collapse:collapse;width:100%;background:white;white-space:nowrap}td,th{padding:10px 14px;border-bottom:1px solid #e5e9ed;text-align:left}th{background:#e9eff6}footer{border-top:1px solid #dae2eb;padding-top:22px;color:#617080}@media(max-width:760px){.cards{grid-template-columns:1fr}header{padding-top:35px}h2{font-size:23px}.flow{padding:15px}}</style>
<header><div class="inner"><div class="eyebrow">EXTREMEDRIVE / CONTROL & VEHICLE DYNAMICS</div><h1>让规划轨迹真正经过车辆执行</h1><p>在现有 benchmark 中接入控制器和开源动力学引擎，观察车辆实际走出的轨迹、执行器响应与四轮状态。把“规划是否合理”推进到“车辆能否完成”。</p><small>3 个实际后端 · DONE 个试验完成 · 2026.10.07</small></div></header>
<main><section><h2>同一条评测链路，三种研究用途</h2><div class="flow"><b>带时序的轨迹</b> → <b>固定跟踪控制器</b> → <b>转向 / 驱动 / 制动</b> → <b>实际车辆响应</b> → <b>分项评分</b></div><p>车辆状态逐步反馈给控制器。当前已实现固定轨迹下的控制闭环；后续把规划器接入反馈链路，才构成完整的规划闭环。</p><div class="cards">CARDS</div></section>
<section><h2>一次实际验证：相同制动要求，不同附着条件</h2><p>每个后端内部，保留控制器和车辆，仅改变附着设置。正常附着能停在障碍前；低附着下制动距离变长，三个后端均出现车身与障碍的几何重叠。这是评测执行结果的必要性。</p><img src="braking_comparison.png" alt="三个实际开源后端的正常与低附着制动速度曲线"><p class="note">此图来自合成工况的实际运行。不同后端的原生车辆、轮胎及附着参数含义不同，不用于仿真器排名；障碍通过几何评分检查，未模拟碰撞冲击。</p></section>
<section><h2>已完成的样例</h2><p>每个后端均运行正常 / 低附着变道、正常 / 低附着制动，以及一段 ExtremeDrive 真实记录轨迹。点击上方后端可播放执行过程，查看转向和四轮响应，并下载逐时刻数据。</p><div class="tablewrap"><table><thead><tr><th>后端</th><th>输入工况</th><th>状态</th><th>位置 RMSE / m</th><th>几何碰撞</th></tr></thead><tbody>ROWS</tbody></table></div><p class="note">位置 RMSE 保留时间对齐，包含纵向进度误差。真实片段目前只接入轨迹，环境尚未配准，因此安全项为“未评估”；这些结果不是正式 benchmark 总分。</p></section>
<section><h2>对底盘极限驾驶研究的价值</h2><p>感知标注提供车道、障碍和可行区域，车辆数据提供运动与底盘响应，两者通过执行仿真联系起来：先判断规划轨迹是否避开危险，再检查它对转向、制动与轮胎能力的要求。跟踪误差、执行器限幅和车辆响应分别报告，便于区分规划与控制的贡献。</p><p>建议以 Chrono 承接底盘模型辨识，CommonRoad 支撑批量诊断，CARLA 承接场景交互。正式对比前，需要用本车四轮数据和 IMU 等信号验证车型、轮胎与执行器参数。</p><p class="note">当前使用原生示例车型，尚未标定实车。CommonRoad 停车阶段加入低于 0.3 m/s 的低速过渡保护；CARLA 的实际轮端驱动 / 制动力矩未提供。缺失通道保留为空，不按零计分。CarSim 暂无安装，本次未运行。</p></section>
<footer><a href="summary.json">下载全部分项结果</a> · <a href="BACKENDS.md">接入说明与复现步骤</a> · <a href="extremedrive-open-backends.zip">下载代码与样例</a><br>报告沿用 8770；CARLA 内部仿真服务使用独立端口，不改变网页转发。</footer></main></html>'''
    page=page.replace('DONE',str(done)).replace('CARDS',''.join(cards)).replace('ROWS',''.join(rows));(out/'index.html').write_text(page)
    (out/'BACKENDS.md').write_text(Path('BACKENDS.md').read_text())
    print(f'Regenerated {done}/{len(summaries)} completed trials')
if __name__=='__main__':main()

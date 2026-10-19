"""Standalone Chinese interactive scientific atlas; full native samples retained."""
from common import *
from scipy.spatial.transform import Rotation
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from html import escape

LABEL={'direct':'方案 A：全量 GPTR','phase':'方案 B：相位分离恢复'}

def figure_pair(a,b,title):
 titles=['GNSS 天线位置 · NED · m','GNSS 天线速度 · NED · m/s','陀螺仪 · FRD · rad/s','加速度计比力 · FRD · m/s²','导航系运动学加速度 · NED · m/s²','角加速度 · FRD · rad/s²（无直接真值）','PX4 姿态测地差异 · rad（非独立参考）','机械相位频率 · Hz','实际优化 Huber 权重 · 无量纲','原始观测向量残差 · 各自 SI 单位']
 fig=make_subplots(rows=10,cols=1,shared_xaxes=True,vertical_spacing=.025,subplot_titles=titles)
 colors=['#3366cc','#d62728','#19965c'];methods=[(a,'direct','solid'),(b,'phase','dash')]
 for run,method,dash in methods:
  for sensor,parts in [('gnss',[(1,range(3)),(2,range(3,6))]),('gyro',[(3,range(3))]),('accel',[(4,range(3))])]:
   d=pd.read_csv(run/f'{sensor}_ledger.csv');d=d[(d.participates==1)&(d.target_interior==1)];t=d.measurement_t
   for row,axes in parts:
    for j in axes:
     axis=j%3
     if method=='direct':
      obs=d[['px','py','pz','vx','vy','vz'][j] if sensor=='gnss' else ['x','y','z'][j]]
      fig.add_trace(go.Scattergl(x=t,y=obs,name=f'{sensor} 原生观测 {axis}',mode='lines',line=dict(color=colors[axis],width=1),opacity=.35,legendgroup=f'obs{sensor}{axis}'),row=row,col=1)
     fig.add_trace(go.Scattergl(x=t,y=d[f'prediction{j}'],name=f'{LABEL[method]} {sensor}{axis}',mode='lines',line=dict(color=colors[axis],dash=dash,width=1.5),legendgroup=method),row=row,col=1)
   fig.add_trace(go.Scattergl(x=t,y=d.actual_huber_weight,name=method+' '+sensor+' 优化权重',mode='lines',line=dict(dash=dash)),row=9,col=1)
   dim=6 if sensor=='gnss' else 3;err=np.linalg.norm(d[[f'residual{j}' for j in range(dim)]],axis=1)
   if sensor=='gnss':
    # Separate position and velocity; never combine metres and m/s into a physical norm.
    for label,ids in [('GNSS p (m)',range(3)),('GNSS v (m/s)',range(3,6))]:fig.add_trace(go.Scattergl(x=t,y=np.linalg.norm(d[[f'residual{j}' for j in ids]],axis=1),name=method+' '+label,mode='lines',line=dict(dash=dash)),row=10,col=1)
   else:fig.add_trace(go.Scattergl(x=t,y=err,name=method+' '+sensor,mode='lines',line=dict(dash=dash)),row=10,col=1)
  d=pd.read_csv(run/'trajectory_export.csv')
  for row,cols in [(5,['ax','ay','az']),(6,['alphax','alphay','alphaz'])]:
   for j,c in enumerate(cols):fig.add_trace(go.Scattergl(x=d.t,y=d[c],name=method+' '+c,mode='lines',line=dict(color=colors[j],dash=dash)),row=row,col=1)
  ref=pd.read_csv(run/'attitude_reference.csv');fig.add_trace(go.Scattergl(x=ref.t,y=ref.geodesic_difference_rad,name=method+' 对 PX4 姿态',mode='lines'),row=7,col=1)
 phase=pd.read_csv(Path(json.loads((a/'run_config.json').read_text())['input'])/'phase_observations.csv');fig.add_trace(go.Scattergl(x=phase.t,y=phase.phase_velocity/(2*np.pi),name='连续编码器相位导数'),row=8,col=1);fig.add_trace(go.Scattergl(x=phase.t,y=phase.logged_frequency,name='日志滤波频率（同源）'),row=8,col=1)
 fig.update_layout(title=title,height=2700,template='plotly_white',hovermode='x unified',legend=dict(groupclick='toggleitem'),margin=dict(l=65,r=25,t=80,b=40));fig.update_xaxes(title_text='PX4 HRT 时间 / s',row=10,col=1)
 return fig

def extras(a,b,title):
 fig=make_subplots(rows=4,cols=2,subplot_titles=['加速度计基频及 H2/H3 幅值','陀螺仪基频及 H2/H3 幅值','加速度计原生 Lomb 频谱','陀螺仪原生 Lomb 频谱','非独立 PX4 姿态与重建 Euler 参考','GNSS 定位等级','速度模与非周期机动','比力瞬态：观测与预测'])
 for run,method,dash in [(a,'direct','solid'),(b,'phase','dash')]:
  h=pd.read_csv(run/'harmonics.csv');s=pd.read_csv(run/'native_spectra.csv')
  for col,sensor in [(1,'accel'),(2,'gyro')]:
   for axis in range(3):
    for src in ['observation','prediction']:
     if method=='phase' and src=='observation':continue
     z=h[(h.sensor==sensor)&(h.axis==axis)&(h.source==src)];fig.add_trace(go.Scatter(x=z.harmonic,y=z.amplitude,mode='lines+markers',name=f'{method} {sensor} {axis} {src}'),row=1,col=col)
     z=s[(s.sensor==sensor)&(s.axis==axis)&(s.source==src)];fig.add_trace(go.Scatter(x=z.frequency_hz,y=z.normalized_lomb_power,name=f'{method} {sensor} {axis} {src}'),row=2,col=col)
  d=pd.read_csv(run/'trajectory_export.csv');angles=Rotation.from_quat(d[['qx','qy','qz','qw']]).as_euler('xyz',degrees=True)
  for j in range(3):fig.add_trace(go.Scattergl(x=d.t,y=angles[:,j],name=method+' Euler '+str(j),line=dict(dash=dash)),row=3,col=1)
  fig.add_trace(go.Scattergl(x=d.t,y=np.linalg.norm(d[['vx','vy','vz']],axis=1),name=method+' speed m/s'),row=4,col=1)
  ld=pd.read_csv(run/'accel_ledger.csv');ld=ld[(ld.participates==1)&(ld.target_interior==1)];ids=np.flatnonzero(abs(np.diff(ld.z.values))>=np.quantile(abs(np.diff(ld.z.values)),.95))+1
  fig.add_trace(go.Scatter(x=ld.z.values[ids],y=ld.prediction2.values[ids],mode='markers',name=method+' 观测最大 5% 跳变 z'),row=4,col=2)
  if method=='direct':
   ref=pd.read_csv(Path(json.loads((run/'run_config.json').read_text())['input'])/'attitude_init.csv');angles=Rotation.from_quat(ref[['qx','qy','qz','qw']]).as_euler('xyz',degrees=True)
   for j in range(3):fig.add_trace(go.Scattergl(x=ref.t,y=angles[:,j],name='PX4 非独立 Euler '+str(j),opacity=.4),row=3,col=1)
   gps=pd.read_csv(run/'gnss_ledger.csv');fig.add_trace(go.Scatter(x=gps.measurement_t,y=gps.fix,name='GNSS fix_type'),row=3,col=2)
 fig.update_layout(title=title,height=1500,template='plotly_white',showlegend=False);fig.update_xaxes(title_text='谐波阶数',row=1);fig.update_xaxes(title_text='Hz',row=2);fig.update_xaxes(title_text='HRT / s',row=3);fig.update_xaxes(title_text='HRT / s',row=4,col=1);fig.update_xaxes(title_text='观测 f_z / m/s²',row=4,col=2);fig.update_yaxes(title_text='预测 f_z / m/s²',row=4,col=2)
 return fig

def main():
 out=ROOT/CFG['output'];parts=['<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>完整观测六自由度轨迹重建</title><style>body{max-width:1500px;margin:auto;padding:24px;font:16px system-ui;color:#172b4d}table{border-collapse:collapse}td,th{padding:8px;border:1px solid #ccc}summary{cursor:pointer}nav{position:sticky;top:0;background:white;padding:12px;z-index:10}a{margin-right:20px}pre{white-space:pre-wrap}</style><h1>基于完整飞行观测的连续六自由度轨迹重建</h1><p>所有有效原生观测参与。方案 A：直接 GPTR；方案 B：全部有效 IMU 重拟合周期模板、固定慢变轨迹后物理恢复。姿态为非独立 PX4 参考；比力 ≠ 导航系运动学加速度；角加速度无直接真值。零杆臂、110 ms 延迟及噪声尺度均未独立标定。</p><p>图保留原生样本，可缩放及切换曲线。各模型目录提供完整逐样本 CSV、连续 knot/相位与任意时刻查询。B 的“实际权重”针对派生投影目标，原始观测残差另外显示，不能混称为同一个似然。</p><nav><a href="#A">A 航段</a><a href="#B">B 航段</a><a href="#full">完整航次</a><a href="#sensitivity">敏感性</a></nav>']
 include=True
 for tag,a,b in [('A',out/'A_direct',out/'A_phase'),('B',out/'B_direct',out/'B_phase'),('full',out/'full_log/full_0_direct',out/'full_log/full_0_phase')]:
  parts.append(f'<h2 id="{tag}">{tag}：重建与观测一致性</h2>');rows=[]
  for r in [a,b]:
   ms=json.loads((r/'observation_metrics.json').read_text())
   for m in ms:
    if m['cohort']=='target':rows.append(dict(方案=r.name,观测=m['channel'],样本=m['n'],向量RMSE=m['vector_rmse'],向量MAE=m['vector_mae'],向量P95=m['vector_p95'],逐轴均值偏差=m['axis_bias']))
  parts.append(pd.DataFrame(rows).to_html(index=False,float_format=lambda x:f'{x:.5g}'))
  for fig in [figure_pair(a,b,tag+'：原生观测与连续轨迹'),extras(a,b,tag+'：周期与机动诊断（非物理真值）')]:
   parts.append(fig.to_html(full_html=False,include_plotlyjs=True if include else False));include=False
  for r in [a,b]:parts.append('<details><summary>'+r.name+' 运动学/积分与观测假设</summary><pre>'+escape((r/'mathematics.json').read_text()+(r/'export_contract.json').read_text())+'</pre>'+pd.read_csv(r/'independent_integration.csv').to_html(index=False)+'</details>')
 parts.append('<h2 id="sensitivity">固定参数对照：A 段，非置信区间</h2><p>没有按最小拟合误差择优。两种设置都可能拟合观测，但瞬时 a/α 不同。完整航次尚未重复全部敏感性试验。</p>');s=pd.read_csv(out/'sensitivity.csv');parts.append(s.to_html(index=False,float_format=lambda x:f'{x:.5g}'))
 fig=make_subplots(rows=2,cols=2,subplot_titles=['线加速度相对主解变化 · m/s²','角加速度变化 · rad/s²','比力观测拟合 RMSE · m/s²','陀螺仪拟合 RMSE · rad/s'])
 for method in ['direct','phase']:
  z=s[s.method==method]
  for row,col,k in [(1,1,'a'),(1,2,'alpha'),(2,1,'accel_fit_rmse'),(2,2,'gyro_fit_rmse')]:fig.add_trace(go.Bar(x=z.contrast,y=z[k],name=LABEL[method]),row=row,col=col)
 fig.update_layout(height=800,template='plotly_white');parts.append(fig.to_html(full_html=False,include_plotlyjs=False));parts.append('<p>独立物理真值验证：未验证。仅适用于带假设和质量标记的研究性候选轨迹分析；不自动进入 Stage03 标签生产或神经网络训练。</p></html>')
 (out/'index.html').write_text('\n'.join(parts));print(out/'index.html')
if __name__=='__main__':main()

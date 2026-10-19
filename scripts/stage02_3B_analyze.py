#!/usr/bin/env python3
"""Fixed-mask metrics, native bandwidth diagnostics, decomposition and sensitivity."""
import json, sys, subprocess
from pathlib import Path
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from scipy.signal import lombscargle
ROOT=Path(__file__).resolve().parents[1]
CONFIG=Path(os.environ.get('STAGE02_3B_CONFIG',str(ROOT/'configs/stage02_3B_joint.json')))
OUT=ROOT/json.loads(CONFIG.read_text())['output']
sys.path.insert(0,str(ROOT/'experiments/stage02_3A'))
from continuous_phase import ContinuousPhase
sys.path.insert(0,str(ROOT/'experiments/stage02_2'))
from periodic_motion import compose, right_jacobian_and_dot

def rmse(e):return np.sqrt(np.mean(e*e,axis=0))
def vector_rmse(e):return float(np.sqrt(np.mean(np.sum(e*e,axis=1))))
def main():
 fig=OUT/'figures';fig.mkdir(exist_ok=True)
 metrics=[];cycles=[];harmonics=[];peaks=[];summaries=[];sensitivity=[];validation=[];phase_acc_terms=[]
 jobs=json.loads((OUT/'jobs.json').read_text())
 for job in jobs:
  name=job['name'];run=OUT/name
  if not (run/'solver.csv').exists():continue
  seg=job['segment'];phase=ContinuousPhase.load(ROOT/f'runs/phase_reconstruction_20261019/{seg}_phase_model.json')
  cy=pd.read_csv(ROOT/f'runs/stage02_2_20261017/regression/{seg}_cycles.csv')
  solver=pd.read_csv(run/'solver.csv').iloc[0].to_dict();summaries.append(dict(name=name,**solver))
  for sensor,dim in [('accel',3),('gyro',3),('gnss',6)]:
   d=pd.read_csv(run/f'{sensor}_predictions.csv');valid=d.valid.astype(bool);obs=d[[f'obs{i}' for i in range(dim)]].values;pred=d[[f'pred{i}' for i in range(dim)]].values;err=pred-obs
   for split,mask in [('fit',valid&~d.holdout.astype(bool)),('held',valid&d.holdout.astype(bool))]:
    if not mask.any():continue
    v=err[mask];row=dict(name=name,segment=seg,sensor=sensor,split=split,n=int(mask.sum()),vector_rmse=vector_rmse(v),p95_norm=float(np.quantile(np.linalg.norm(v,axis=1),.95)))
    row.update({f'rmse{i}':x for i,x in enumerate(rmse(v))});
    if dim==6:row.update(position_vector_rmse=vector_rmse(v[:,:3]),velocity_vector_rmse=vector_rmse(v[:,3:]))
    metrics.append(row)
   if sensor=='gnss':
    for fix,group in d[valid].groupby('fix'):
     for held,sub in group.groupby('holdout'):
      er=sub[[f'pred{i}' for i in range(6)]].values-sub[[f'obs{i}' for i in range(6)]].values
      cycles.append(dict(name=name,sensor=sensor,quality=fix,held=held,n=len(sub),position_rmse=vector_rmse(er[:,:3]),velocity_rmse=vector_rmse(er[:,3:])))
    continue
   for _,c in cy.iterrows():
    mask=valid&(d.t>=c.lo)&(d.t<c.hi)
    if mask.sum()>3:cycles.append(dict(name=name,sensor=sensor,cycle=c.cycle,t=c.mid,frequency=c.frequency,held=int(d[mask].holdout.iloc[0]),n=int(mask.sum()),rmse=vector_rmse(err[mask])))
   # Harmonics from full native valid observations/predictions, never FFT of sparse held rows.
   t=d.t[valid].values;ph=phase.query(t);design=np.column_stack([np.ones(len(t)),t-np.mean(t)]+[b for h in [1,2,3] for b in [np.sin(h*ph),np.cos(h*ph)]])
   for kind,values in [('obs',obs[valid]),('total',pred[valid]),('slow',d.loc[valid,[f'slowpred{i}' for i in range(dim)]].values),('effect',pred[valid]-d.loc[valid,[f'slowpred{i}' for i in range(dim)]].values)]:
    coef=np.linalg.lstsq(design,values,rcond=None)[0]
    for h in [1,2,3]:
     for axis in range(3):harmonics.append(dict(name=name,sensor=sensor,kind=kind,h=h,axis=axis,amplitude=np.hypot(coef[2*h,axis],coef[2*h+1,axis]),phase_rad=np.arctan2(coef[2*h+1,axis],coef[2*h,axis])))
   for axis in range(3):peaks.append(dict(name=name,sensor=sensor,axis=axis,obs_abs_max=np.max(np.abs(obs[valid,axis])),pred_abs_max=np.max(np.abs(pred[valid,axis])),obs_p99=np.quantile(np.abs(obs[valid,axis]),.99),pred_p99=np.quantile(np.abs(pred[valid,axis]),.99)))
  if name in ['A_H0','A_H1','A_H3','B_H0','B_H1','B_H3']:
   dense=pd.read_csv(run/'trajectory.csv');slow=pd.read_csv(run/'slow_trajectory.csv')
   if job['H']:
    # Independent Python implementation of the composition on saved slow states.
    coeff=pd.read_csv(run/'coefficients.csv').value.values;H=job['H'];pc=coeff[:6*H].reshape(H,2,3);rc=coeff[6*H:].reshape(H,2,3);zero=np.zeros_like(pc);errors=[]
    for i in range(7,len(dense),97):
     s=slow.iloc[i];t=s.t;x=compose(dict(p=s[['px','py','pz']].values,v=s[['vx','vy','vz']].values,a=s[['ax','ay','az']].values,R=Rotation.from_quat(s[['qx','qy','qz','qw']].values),w=s[['wx','wy','wz']].values,alpha=s[['alphax','alphay','alphaz']].values),[phase.query(t,k) for k in [0,1,2]],(pc,zero,zero),(rc,zero,zero));r=dense.iloc[i]
     eta=np.zeros(3);ep=np.zeros(3);pp=np.zeros(3);ph=phase.query(t);pdot=phase.query(t,1);pddot=phase.query(t,2)
     for h in range(1,H+1):
      basis=np.array([np.sin(h*ph),np.cos(h*ph)]);bp=h*np.array([np.cos(h*ph),-np.sin(h*ph)])
      eta+=basis@rc[h-1];ep+=bp@rc[h-1];pp+=bp@pc[h-1]
     J,_=right_jacobian_and_dot(eta,ep*pdot)
     phase_acc_terms.append(dict(name=name,t=t,phase_ddot=pddot,a_ddot_term_norm=np.linalg.norm(pp*pddot),alpha_ddot_term_norm=np.linalg.norm(J@ep*pddot)))
     errors.append([np.linalg.norm(x[k]-r[cols].values) for k,cols in [('p',['px','py','pz']),('v',['vx','vy','vz']),('a',['ax','ay','az']),('w',['wx','wy','wz']),('alpha',['alphax','alphay','alphaz'])]]+[(x['R'].inv()*Rotation.from_quat(r[['qx','qy','qz','qw']].values)).magnitude()])
    validation.append(dict(name=name,python_composition_max=np.max(errors,axis=0).tolist()))
    assert np.max(errors)<1e-7
   # Serialized query, off-grid times. Requery a saved grid subset too.
   times=pd.DataFrame({'t':np.r_[dense.t.iloc[::113].values,dense.t.iloc[7:-1:137].values+.001733]});timefile=run/'query_check_times.csv';timefile.write_text(times.to_csv(index=False,float_format='%.17g'))
   output=run/'query_check.csv'
   if not output.exists():subprocess.run([str(OUT/'build/stage02_3B_query'),str(run),str(timefile),str(output)],check=True)
   q=pd.read_csv(output);num=len(dense.iloc[::113]);delta=np.max(np.abs(q.iloc[:num].values-dense.iloc[::113].values));assert delta<1e-7
   validation.append(dict(name=name,serialized_query_grid_max=float(delta),off_grid_finite=bool(np.isfinite(q.values).all())))
  if name.startswith('A_H3_'):
   base=pd.read_csv(OUT/'A_H3/trajectory.csv');new=pd.read_csv(run/'trajectory.csv');srow={'name':name}
   for label,cols in [('p',['px','py','pz']),('v',['vx','vy','vz']),('a',['ax','ay','az']),('omega',['wx','wy','wz']),('alpha',['alphax','alphay','alphaz'])]:srow[label+'_rms']=vector_rmse(new[cols].values-base[cols].values)
   srow['R_rms_rad']=float(np.sqrt(np.mean((Rotation.from_quat(base[['qx','qy','qz','qw']].values).inv()*Rotation.from_quat(new[['qx','qy','qz','qw']].values)).magnitude()**2)))
   bc=pd.read_csv(OUT/'A_H3/coefficients.csv').value.values;nc=pd.read_csv(run/'coefficients.csv').value.values;srow['position_coeff_delta_norm']=np.linalg.norm(nc[:18]-bc[:18]);srow['rotation_coeff_delta_norm']=np.linalg.norm(nc[18:]-bc[18:]);sensitivity.append(srow)
 for name,rows in [('metrics',metrics),('cycles_and_quality',cycles),('harmonics',harmonics),('peaks',peaks),('solvers',summaries),('sensitivity',sensitivity),('phase_acceleration_terms',phase_acc_terms)]:pd.DataFrame(rows).to_csv(OUT/f'{name}.csv',index=False)
 (OUT/'query_validation.json').write_text(json.dumps(validation,indent=2)+'\n')
 # Paired held-cycle robustness, descriptive only (development-exposed cycles).
 paired=[]
 cdf=pd.DataFrame(cycles)
 for seg in ['A','B']:
  for sensor in ['accel','gyro']:
   b=cdf[(cdf.name==seg+'_H0')&(cdf.sensor==sensor)&(cdf.held==1)].set_index('cycle')
   n=cdf[(cdf.name==seg+'_H3')&(cdf.sensor==sensor)&(cdf.held==1)].set_index('cycle')
   delta=n.rmse-b.rmse
   for group,mask in [('all',np.ones(len(b),bool)),('early',b.t<(b.t.min()+b.t.max())/2),('late',b.t>=(b.t.min()+b.t.max())/2),('lower_frequency',b.frequency<b.frequency.median()),('higher_frequency',b.frequency>=b.frequency.median())]:
    paired.append(dict(segment=seg,sensor=sensor,group=group,n=int(sum(mask)),improved_cycles=int(sum(delta[mask]<0)),mean_cycle_rmse_H0=b.rmse[mask].mean(),mean_cycle_rmse_H3=n.rmse[mask].mean(),median_delta=delta[mask].median()))
 pd.DataFrame(paired).to_csv(OUT/'paired_cycles.csv',index=False)
 # Exactly matched sensor-only baseline on historical complete-cycle held points.
 aux=[]
 for seg in ['A','B']:
  for sensor in ['accel','gyro']:
   old=pd.read_csv(ROOT/f'runs/stage02_2_20261017/regression/{seg}_complete_cycle_mod5_eq2_{sensor}_fourier_global_h3.csv');d=pd.read_csv(OUT/f'{seg}_H3/{sensor}_predictions.csv');d=d[d.valid.astype(bool)&d.holdout.astype(bool)];ix=np.searchsorted(d.t.values,old.t.values-1e-8);assert np.max(np.abs(d.t.values[ix]-old.t.values))<1e-7
   aux.append(dict(segment=seg,sensor=sensor,n=len(old),sensor_only_original_phase_rmse=vector_rmse(old[[f'pred_{a}' for a in 'xyz']].values-old[[f'obs_{a}' for a in 'xyz']].values),joint_on_same_rows_rmse=vector_rmse(d.iloc[ix][[f'pred{i}' for i in range(3)]].values-d.iloc[ix][[f'obs{i}' for i in range(3)]].values)))
 pd.DataFrame(aux).to_csv(OUT/'auxiliary_sensor_regression.csv',index=False)
 # Figures: raw timestamp traces, whole-cycle errors, harmonics, native spectra, decomposition.
 colors={0:'tab:gray',1:'tab:orange',3:'tab:blue'}
 for seg in ['A','B']:
  cy=pd.read_csv(ROOT/f'runs/stage02_2_20261017/regression/{seg}_cycles.csv')
  fig1,ax=plt.subplots(2,2,figsize=(14,7));fig2,bx=plt.subplots(2,3,figsize=(15,7));fig3,cx=plt.subplots(2,3,figsize=(15,7));fig4,dx=plt.subplots(2,1,figsize=(12,6))
  for si,(sensor,axis) in enumerate([('accel',2),('gyro',1)]):
   base=pd.read_csv(OUT/f'{seg}_H0/{sensor}_predictions.csv');t0=base.t.min();mask=(base.t<t0+2)&base.valid.astype(bool)
   ax[si,0].plot(base.t[mask]-t0,base.loc[mask,f'obs{axis}'],'.',color='black',ms=3,label='native observation')
   for H in [0,1,3]:
    name=f'{seg}_H{H}';d=pd.read_csv(OUT/f'{name}/{sensor}_predictions.csv');ax[si,0].plot(d.t[mask]-t0,d.loc[mask,f'pred{axis}'],color=colors[H],label=f'H{H}');c=pd.DataFrame(cycles);c=c[(c.name==name)&(c.sensor==sensor)&(c.held==1)];ax[si,1].plot(c.t-t0,c.rmse,'o-',color=colors[H],label=f'H{H}')
    hs=pd.DataFrame(harmonics);hs=hs[(hs.name==name)&(hs.sensor==sensor)]
    for a in range(3):
     for kind,style in [('obs','k--'),('total',None)]:
      if kind=='obs' and H!=0:continue
      sub=hs[(hs.axis==a)&(hs.kind==kind)];bx[si,a].plot(sub.h,sub.amplitude,style or 'o-',color='black' if kind=='obs' else colors[H],label='observed' if kind=='obs' else f'H{H}')
     # Lomb power evaluated only from full native stream; no holdout subsequence PSD.
     tt=d.t.values-d.t.values[0];freq=np.linspace(.5,24,500);y=d[f'pred{a}'].values;power=lombscargle(tt,y-y.mean(),2*np.pi*freq,normalize=True);cx[si,a].plot(freq,power,color=colors[H],label=f'H{H}')
     if H==0:
      y=d[f'obs{a}'].values;cx[si,a].plot(freq,lombscargle(tt,y-y.mean(),2*np.pi*freq,normalize=True),'k--',label='observed')
   d=pd.read_csv(OUT/f'{seg}_H3/{sensor}_predictions.csv');dx[si].plot(d.t[mask]-t0,d.loc[mask,f'obs{axis}'],'k.',ms=3,label='observed');dx[si].plot(d.t[mask]-t0,d.loc[mask,f'pred{axis}'],label='total');dx[si].plot(d.t[mask]-t0,d.loc[mask,f'slowpred{axis}'],label='slow prediction');dx[si].plot(d.t[mask]-t0,d.loc[mask,f'pred{axis}']-d.loc[mask,f'slowpred{axis}'],label='periodic effect')
   ax[si,0].set_title(f'{seg} {sensor} axis {axis}: native time');ax[si,1].set_title(f'{seg} {sensor}: held whole-cycle vector RMSE')
   for a in range(3):bx[si,a].set_title(f'{sensor} axis {a}: harmonic amplitude');bx[si,a].set_xticks([1,2,3]);cx[si,a].set_title(f'{sensor} axis {a}: normalized native Lomb power')
   dx[si].set_title(f'{seg} {sensor} axis {axis}: allocation is not physical identification')
   for _,cyrow in cy[cy.cycle.isin(json.loads((OUT/'preparation_manifest.json').read_text())['splits'][seg]['held_cycles'])].iterrows():
    if cyrow.lo<t0+2:ax[si,0].axvspan(max(0,cyrow.lo-t0),min(2,cyrow.hi-t0),color='red',alpha=.12)
  for figure,axes,suffix in [(fig1,ax,'prediction'),(fig2,bx,'harmonics'),(fig3,cx,'spectrum'),(fig4,dx,'allocation')]:
   for axis in np.ravel(axes):axis.legend(fontsize=7);axis.grid(alpha=.2)
   figure.tight_layout();figure.savefig(fig/f'{seg}_{suffix}.png',dpi=150);plt.close(figure)
 html='<meta charset="utf-8"><title>Stage 02.3-B</title><h1>Stage 02.3-B：整翼拍留出与动态分配</h1><p>H0 原 GPTR；H1/H3 相位模型。红色阴影为整翼拍留出。图中 slow 是 GPTR 分量，不保证低频；周期分配不等于物理识别。频谱使用完整原生流，不能用来独立认证刚体运动。</p>'
 for p in sorted(fig.glob('*.png')):html+=f'<h2>{p.stem}</h2><img style="max-width:100%" src="figures/{p.name}">'
 (OUT/'index.html').write_text(html)
 print(pd.DataFrame(metrics).query("split=='held'")[['name','sensor','vector_rmse']].to_string(index=False))
if __name__=='__main__':main()

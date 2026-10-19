#!/usr/bin/env python3
"""Separate sensor addback scores from rigid forward predictions on identical masks."""
import json,sys
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import lombscargle
from scipy.spatial.transform import Rotation
from stage02_4_projection import evaluate,ContinuousPhase,fit_predict,ROOT,OUT

def vr(e):return float(np.sqrt(np.mean(np.sum(e*e,axis=1))))
def main():
 fig=OUT/'figures';fig.mkdir(exist_ok=True);metrics=[];percycle=[];harmonics=[];peak=[];solvers=[];sensitivity=[];complexity=[];proj=[]
 runs=[(f'{s}_{h}',s,ROOT/f'runs/stage02_3B_20261019/{s}_{h}') for s in ['A','B'] for h in ['H0','H3']]
 runs += [(s+'_Qian',s,OUT/(s+'_restore')) for s in ['A','B']]
 runs += [(n,'A',OUT/(n+'_restore')) for n in ['A_phase16','A_prior1','A_delay0','A_lever','A_restart']]
 for name,seg,run in runs:
  phase=ContinuousPhase.load(ROOT/f'runs/phase_reconstruction_20261019/{seg}_phase_model.json');cycles=pd.read_csv(ROOT/f'runs/stage02_2_20261017/regression/{seg}_cycles.csv');solver=pd.read_csv(run/'solver.csv').iloc[0].to_dict();solvers.append(dict(name=name,**solver))
  for sensor,dim in [('accel',3),('gyro',3),('gnss',6)]:
   d=pd.read_csv(run/f'{sensor}_predictions.csv');obs=d[[f'obs{i}' for i in range(dim)]].values;valid=d.valid.astype(bool).values;t=d.t.values;cy=np.searchsorted(cycles.lo.values,t,side='right')-1;held=d.holdout.astype(bool).values
   reference=pd.read_csv(ROOT/f'runs/stage02_3B_20261019/{seg}_H0/{sensor}_predictions.csv')
   if name in [seg+'_H0',seg+'_H3',seg+'_Qian']:
    assert np.array_equal(reference[['t','holdout','valid']].values,d[['t','holdout','valid']].values)
    assert np.array_equal(reference[[f'obs{i}' for i in range(dim)]].values,obs)
   kinds=['pred']+(['addback'] if sensor!='gnss' and 'addback0' in d else [])
   for kind in kinds:
    pred=d[[f'{kind}{i}' for i in range(dim)]].values;e=pred-obs
    for split,mask in [('fit',valid&~held),('held',valid&held)]:
     row=dict(name=name,segment=seg,sensor=sensor,output=kind,split=split,n=int(sum(mask)),vector_rmse=vr(e[mask]),**{f'rmse{i}':float(v) for i,v in enumerate(np.sqrt(np.mean(e[mask]**2,axis=0)))})
     if dim==6:row.update(position_rmse=vr(e[mask,:3]),velocity_rmse=vr(e[mask,3:]))
     metrics.append(row)
    if sensor!='gnss':
     for c in np.unique(cy[held]):
      mask=valid&held&(cy==c);percycle.append(dict(name=name,segment=seg,sensor=sensor,output=kind,cycle=int(c),t=cycles.mid.iloc[c],frequency=cycles.frequency.iloc[c],rmse=vr(e[mask])))
   if sensor=='gnss':continue
   ph=phase.query(t);train=valid&~held;query=valid;cytrain=cy[train];ids,counts=np.unique(cytrain,return_counts=True);weight=np.array([1/counts[np.searchsorted(ids,k)] for k in cytrain]);center=np.mean(t[train]);scale=np.ptp(t[train])/2
   X=np.column_stack([np.ones(len(t)),(t-center)/scale]+[f(h*ph) for h in [1,2,3] for f in [np.sin,np.cos]])
   values={'raw':obs,'total':d[[f'pred{i}' for i in range(3)]].values,'slow':d[[f'slowpred{i}' for i in range(3)]].values};values['effect']=values['total']-values['slow']
   if 'addback0' in d:values['addback']=d[[f'addback{i}' for i in range(3)]].values
   for key,y in values.items():
    # Fit projection on TRAIN only, then measure held nonperiodic residual error separately.
    b=np.linalg.lstsq(X[train]*np.sqrt(weight[:,None]),y[train]*np.sqrt(weight[:,None]),rcond=None)[0]
    for h in [1,2,3]:
     for a in range(3):harmonics.append(dict(name=name,sensor=sensor,kind=key,h=h,axis=a,amplitude=float(np.hypot(b[2*h,a],b[2*h+1,a])),phase=float(np.arctan2(b[2*h+1,a],b[2*h,a]))))
    for a in range(3):peak.append(dict(name=name,sensor=sensor,kind=key,axis=a,max_abs=float(np.max(np.abs(y[valid,a]))),p99=float(np.quantile(np.abs(y[valid,a]),.99))))
   if 'addback0' in d:
    discrepancy=values['total']-values['addback']
    complexity.append(dict(name=name,sensor=sensor,forward_vs_addback_rms=vr(discrepancy[valid]),held_forward_vs_addback_rms=vr(discrepancy[held&valid])))
  if run.parent==OUT:
   info=pd.read_csv(run/'coefficient_information.csv').drop(columns='row').values;scale=np.sqrt(np.maximum(np.diag(info),1e-30));corr=info/scale[:,None]/scale[None,:];eigen=np.linalg.eigvalsh(corr);coeff=pd.read_csv(run/'coefficients.csv').value.values
   complexity.append(dict(name=name,conditional_data_rank=int(sum(eigen>max(eigen)*1e-8)),column_normalized_condition=float(np.sqrt(max(eigen)/max(min(eigen),1e-30))),position_coeff_norm=float(np.linalg.norm(coeff[:18])),rotation_coeff_norm=float(np.linalg.norm(coeff[18:]))))
   if name.startswith('A_') and name!='A_Qian':
    base=pd.read_csv(OUT/'A_restore/trajectory.csv');alt=pd.read_csv(run/'trajectory.csv');row=dict(name=name)
    for k,cols in [('p',['px','py','pz']),('v',['vx','vy','vz']),('a',['ax','ay','az']),('omega',['wx','wy','wz']),('alpha',['alphax','alphay','alphaz'])]:row[k+'_rms']=vr(alt[cols].values-base[cols].values)
    row['R_rms']=float(np.sqrt(np.mean((Rotation.from_quat(base[['qx','qy','qz','qw']]).inv()*Rotation.from_quat(alt[['qx','qy','qz','qw']])).magnitude()**2)));sensitivity.append(row)
 for name,seg,scale in [('A','A',32),('B','B',32),('A_phase16','A',16),('A_prior1','A',32),('A_delay0','A',32),('A_lever','A',32)]:proj+=evaluate(name,seg,scale)
 for label,rows in [('metrics',metrics),('cycle_metrics',percycle),('harmonics',harmonics),('peaks',peak),('solvers',solvers),('sensitivity',sensitivity),('complexity',complexity),('slow_projections',proj)]:pd.DataFrame(rows).to_csv(OUT/f'{label}.csv',index=False)
 for seg in ['A','B']:
  f,ax=plt.subplots(2,2,figsize=(14,7));g,bx=plt.subplots(2,3,figsize=(15,7));z,zx=plt.subplots(2,2,figsize=(14,7));afigure,aa=plt.subplots(2,2,figsize=(14,7))
  for si,(sensor,axis) in enumerate([('accel',2),('gyro',1)]):
   d=pd.read_csv(OUT/f'{seg}_restore/{sensor}_predictions.csv');t0=d.t.iloc[0];near=d.t<t0+2;ax[si,0].plot(d.t[near]-t0,d.loc[near,f'obs{axis}'],'k.',ms=3,label='raw');ax[si,0].plot(d.t[near]-t0,d.loc[near,f'addback{axis}'],'--',label='sensor addback ONLY')
   for label,run,color in [('H0',ROOT/f'runs/stage02_3B_20261019/{seg}_H0','gray'),('H3',ROOT/f'runs/stage02_3B_20261019/{seg}_H3','tab:orange'),('Qian',OUT/f'{seg}_restore','tab:blue')]:
    r=pd.read_csv(run/f'{sensor}_predictions.csv');ax[si,0].plot(r.t[near]-t0,r.loc[near,f'pred{axis}'],color=color,label=label)
    cyc=pd.DataFrame(percycle);cc=cyc[(cyc.name==seg+'_'+label)&(cyc.sensor==sensor)&(cyc.output=='pred')];ax[si,1].plot(cc.t-t0,cc.rmse,'o-',color=color,label=label)
    for axis2 in range(3):
     h=pd.DataFrame(harmonics);h=h[(h.name==seg+'_'+label)&(h.sensor==sensor)&(h.axis==axis2)&(h.kind=='total')];bx[si,axis2].plot(h.h,h.amplitude,'o-',color=color,label=label)
   h=pd.DataFrame(harmonics)
   for axis2 in range(3):
    hh=h[(h.name==seg+'_Qian')&(h.sensor==sensor)&(h.axis==axis2)]
    for kind,style in [('raw','k--'),('slow','g:')]:hh2=hh[hh.kind==kind];bx[si,axis2].plot(hh2.h,hh2.amplitude,style,label=kind)
    bx[si,axis2].set_title(f'{sensor} axis {axis2}: train harmonic amplitude');bx[si,axis2].set_xticks([1,2,3])
   dec=pd.read_csv(OUT/f'observations/{seg}_phase32/{sensor}_decomposition.csv');mask=(dec.t>=t0)&(dec.t<t0+2)
   for k in ['raw','per','remaining']:zx[si,0].plot(dec.t[mask]-t0,dec.loc[mask,f'{k}{axis}'],label=k)
   dec=dec[dec.valid.astype(bool)].copy();freq=np.linspace(.5,24,400);tt=dec.t.values-dec.t.iloc[0]
   for k in ['raw','per','remaining']:
    y=dec[f'{k}{axis}'].values;zx[si,1].plot(freq,lombscargle(tt,y-y.mean(),2*np.pi*freq,normalize=False)/len(y),label=k)
   aa[si,0].plot(d.t[near]-t0,d.loc[near,f'slowpred{axis}'],label='slow physical prediction');aa[si,0].plot(d.t[near]-t0,d.loc[near,f'pred{axis}']-d.loc[near,f'slowpred{axis}'],label='restored physical effect');aa[si,0].plot(d.t[near]-t0,d.loc[near,f'addback{axis}']-d.loc[near,f'slowpred{axis}'],'--',label='learned sensor periodic term')
   aa[si,1].plot(d.t-t0,d[f'pred{axis}']-d[f'addback{axis}'],label='physical minus addback')
   unit='m/s^2' if sensor=='accel' else 'rad/s'
   cydata=pd.read_csv(ROOT/f'runs/stage02_2_20261017/regression/{seg}_cycles.csv')
   split=json.loads((OUT/f'observations/{seg}_phase32/regression.json').read_text())['split']['held_cycles']
   for _,c in cydata[cydata.cycle.isin(split)].iterrows():
    if c.lo<t0+2 and c.hi>t0:ax[si,0].axvspan(max(c.lo-t0,0),min(c.hi-t0,2),color='red',alpha=.1)
   ax[si,0].set_title(f'{seg} {sensor} axis {axis}: native predictions');ax[si,1].set_title(f'{seg} {sensor}: whole-cycle vector RMSE')
   zx[si,0].set_title(f'{seg} {sensor} axis {axis}: sensor decomposition');zx[si,1].set_title('Full native Lomb power (not calibrated PSD)')
   aa[si,0].set_title(f'{seg} {sensor}: physical vs signal allocation');aa[si,1].set_title('Physical minus sensor-addback prediction')
   for axes in [ax,zx,aa]:
    for j in [0,1]:axes[si,j].set_xlabel('time from target start (s)');axes[si,j].set_ylabel(unit)
   zx[si,1].set_xlabel('frequency (Hz)');zx[si,1].set_ylabel('Lomb power')
   for a in range(3):bx[si,a].set_xlabel('harmonic order');bx[si,a].set_ylabel(unit)
  for figure,axes,label in [(f,ax,'predictions'),(g,bx,'harmonics'),(z,zx,'separation'),(afigure,aa,'restoration')]:
   for axis in axes.ravel():axis.legend(fontsize=7);axis.grid(alpha=.2)
   figure.tight_layout();figure.savefig(fig/f'{seg}_{label}.png',dpi=150);plt.close(figure)
 # Whole-window motion and GNSS agreement, separate from high-frequency plots.
 for seg in ['A','B']:
  figure,axes=plt.subplots(2,3,figsize=(15,7));core=pd.read_csv(OUT/f'{seg}_restore/trajectory.csv');t0=core.t.iloc[0]
  gps=pd.read_csv(OUT/f'{seg}_restore/gnss_predictions.csv')
  for row,cols in enumerate([['px','py','pz'],['vx','vy','vz']]):
   for axis,col in enumerate(cols):
    ax=axes[row,axis]
    for label,path in [('H0',ROOT/f'runs/stage02_3B_20261019/{seg}_H0/trajectory.csv'),('H3',ROOT/f'runs/stage02_3B_20261019/{seg}_H3/trajectory.csv'),('Qian total',OUT/f'{seg}_restore/trajectory.csv'),('fixed slow',OUT/f'{seg}_restore/slow_trajectory.csv')]:
     d=pd.read_csv(path);ax.plot(d.t-t0,d[col],lw=1,label=label)
    for held,mark in [(False,'.'),(True,'x')]:
     mask=gps.valid.astype(bool)&(gps.holdout.astype(bool)==held)
     ax.scatter(gps.t[mask]-t0,gps.loc[mask,f'obs{3*row+axis}'],s=12,marker=mark,c='black',label='GNSS held' if held else 'GNSS fit')
    ax.set_title(f'{seg} {col}');ax.set_xlabel('time from target start (s)');ax.set_ylabel('m' if row==0 else 'm/s');ax.grid(alpha=.2)
  axes[0,0].legend(fontsize=7);figure.tight_layout();figure.savefig(fig/f'{seg}_maneuver.png',dpi=150);plt.close(figure)
 html='<meta charset="utf-8"><title>Stage 02.4</title><h1>周期观测分离与物理轨迹恢复</h1><p>raw 原生观测；per 训练翼拍学得的周期项；remaining 仅扣周期项的剩余观测。sensor addback 只是信号相加，不是运动学轨迹；Qian 是固定慢变后通过完整前向模型得到的轨迹预测。频谱使用完整原生流；谐波投影只拟合训练点。</p>'
 for p in sorted(fig.glob('*.png')):html+=f'<h2>{p.stem}</h2><img style="max-width:100%" src="figures/{p.name}">'
 (OUT/'index.html').write_text(html)
 print(pd.DataFrame(metrics).query("split=='held' and name in ['A_H0','A_H3','A_Qian','B_H0','B_H3','B_Qian']").to_string(index=False))
if __name__=='__main__':main()

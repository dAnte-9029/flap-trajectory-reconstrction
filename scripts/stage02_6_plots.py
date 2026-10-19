#!/usr/bin/env python3
"""Comparable native-time figures; no interpolation of held wingbeats."""
import numpy as np,pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from stage02_6_analyze import ROOT,OUT,SRC
D=OUT/'figures';D.mkdir(exist_ok=True)
colors={'G0':'#999999','G1':'#167ab1','G2':'#d34f29'}
def save(fig,name):fig.tight_layout();fig.savefig(D/(name+'.png'),dpi=160);fig.savefig(D/(name+'.svg'));plt.close(fig)
def shades(ax,seg,start):
 cycles=pd.read_csv(ROOT/f'runs/stage02_2_20261017/regression/{seg}_cycles.csv');d=pd.read_csv(OUT/f'{seg}_G1_accel_ledger_points.csv');ids=set(d.loc[d.holdout==1,'cycle'])
 for _,c in cycles.iterrows():
  if c.cycle in ids:ax.axvspan(c.lo-start,c.hi-start,color='#f8bd63',alpha=.22,lw=0)
for seg in 'AB':
 dfs={g:pd.read_csv(OUT/f'{seg}_{g}_accel_ledger_points.csv') for g in colors};d=dfs['G1'];start=d.t.min()
 fig,axes=plt.subplots(3,1,figsize=(13,8),sharex=True)
 for j,ax in enumerate(axes):
  y=d[f'y{j}']-d[f'P{j}'];ax.scatter(d.t-start,y,s=3,color='#777',alpha=.4,label='Remaining IMU (y-P)');ax.scatter((d.t-start)[d.holdout==1],y[d.holdout==1],s=6,color='#bd810b',label='Held IMU')
  ax.plot(d.t-start,d[f'T{j}'],'k--',lw=1,label='Fixed train trend T')
  for g in colors:ax.plot(d.t-start,dfs[g][f'S{j}'],color=colors[g],lw=.9,label=g)
  shades(ax,seg,start);ax.set_ylabel('Specific force '+ 'xyz'[j]+' [m/s2]');ax.grid(alpha=.2)
 axes[0].legend(ncol=6,fontsize=8);axes[0].set_title(seg+': remaining IMU and slow predictions; shaded = complete-wingbeat holdout');axes[-1].set_xlabel('Seconds from first core accel sample');save(fig,seg+'_native_remaining')
 cyc=pd.read_csv(OUT/'cycles.csv').query("sensor=='accel' and axis=='vector' and split=='held'");a=cyc[cyc.name==seg+'_G1'].set_index('cycle');b=cyc[cyc.name==seg+'_G2'].set_index('cycle');worst=(b.mse-a.mse).idxmax();lo=a.loc[worst,'t_lo']-.25;hi=a.loc[worst,'t_hi']+.25
 fig,axes=plt.subplots(3,1,figsize=(11,7),sharex=True)
 for j,ax in enumerate(axes):
  m=(d.t>=lo)&(d.t<=hi);ax.scatter(d.t[m]-start,(d[f'y{j}']-d[f'P{j}'])[m],s=14,color='#777',label='Remaining IMU');ax.scatter(d.t[m&(d.holdout==1)]-start,(d[f'y{j}']-d[f'P{j}'])[m&(d.holdout==1)],s=18,color='#bd810b',label='Held IMU')
  for g in ['G1','G2']:ax.plot(d.t[m]-start,dfs[g].loc[m,f'S{j}'],color=colors[g],marker='.',ms=3,label=g)
  shades(ax,seg,start);ax.set_xlim(lo-start,hi-start);ax.set_ylabel('xyz'[j]+' [m/s2]');ax.grid(alpha=.2)
 axes[0].legend(ncol=4);axes[0].set_title(f'{seg}: largest observed MSE increase, cycle {worst} (descriptive selection)');axes[-1].set_xlabel('Seconds from first core accel sample');save(fig,seg+'_largest_change')
 fig,axes=plt.subplots(2,1,figsize=(11,6),sharex=True);x=np.arange(len(a))
 for g in ['G0','G1','G2']:
  z=cyc[cyc.name==seg+'_'+g].set_index('cycle').loc[a.index];axes[0].plot(x,z.rmse,'o-',color=colors[g],label=g)
 axes[1].bar(x,b.mse-a.mse,color=np.where(b.mse>a.mse,'#d34f29','#167ab1'));axes[1].axhline(0,color='k',lw=.8);axes[1].set_xticks(x,[str(i) for i in a.index]);axes[1].set_xlabel('Held wingbeat ID');axes[0].set_ylabel('Accel vector RMSE [m/s2]');axes[1].set_ylabel('G2-G1 MSE [(m/s2)^2]');axes[0].legend();axes[0].set_title(seg+': all held wingbeats (no exclusion)');save(fig,seg+'_cycles')
 fig,axes=plt.subplots(3,2,figsize=(13,8),sharex=True)
 for g in ['G1','G2']:
  z=pd.read_csv(OUT/f'{seg}_{g}_gnss_weights.csv').query('core==1');t=z.t-start
  for j in range(3):
   for k,label in [(0,'position'),(1,'velocity')]:
    ax=axes[j,k];ax.plot(t,z[f'residual{j+3*k}'],'.-',color=colors[g],lw=.8,label=g);h=z.holdout==1;ax.scatter(t[h],z.loc[h,f'residual{j+3*k}'],facecolors='none',edgecolors=colors[g],s=40);ax.set_ylabel(f'{label} '+ 'xyz'[j]+(' [m]' if k==0 else ' [m/s]'));ax.grid(alpha=.2)
 axes[0,0].legend();axes[0,0].set_title(seg+': GNSS residuals; hollow circles = held GNSS');axes[-1,0].set_xlabel('Seconds from first core accel sample');axes[-1,1].set_xlabel('Seconds from first core accel sample');save(fig,seg+'_gnss')
fig,ax=plt.subplots(figsize=(11,5));l=pd.read_csv(OUT/'ledger.csv').query("sensor=='accel' and axis=='vector' and split=='held'");x=np.arange(len(l));ax.bar(x-.24,l.e1_mse,.24,label='e1 MSE (fixed)');ax.bar(x,l.e2_mse,.24,label='e2 MSE');ax.bar(x+.24,l.cross12,.24,label='2 E[e1 dot e2]');ax.plot(x,l.mse,'ko-',label='Total MSE (sum)');ax.set_xticks(x,l.name);ax.set_ylabel('(m/s2)^2');ax.set_title('Held prediction-error ledger: correlated terms, not additive RMSE');ax.legend();save(fig,'ledger')
html='''<!doctype html><meta charset="utf-8"><title>Stage 02.6 GNSS 速度消融</title><style>body{font:18px sans-serif;max-width:1200px;margin:30px auto;line-height:1.6}img{width:100%}section{margin:50px 0}</style><h1>GNSS 速度约束消融：同条件 G1 与 G2</h1><p>蓝线 G1 保留位置和速度；橙线 G2 只保留位置。灰线 G0 使用原六维联合 Huber。黄色背景是完整翼拍 IMU 留出区间。纵轴是比力预测，不是整机加速度真值。</p><p>结果：去除速度后，A/B 留出 RMSE 均升高，位置与速度留出误差也升高。这不证明所有 GNSS 速度都准确；只反驳当前条件下删除速度能改善缺测预测的预期。</p>'''
for name in ['ledger']+[s+'_'+k for s in 'AB' for k in ['native_remaining','largest_change','cycles','gnss']]:
 html+=f'<section><h2>{name}</h2><img src="{name}.png"></section>'
html+='<p>remaining=y-P 仍含未解释变化，不是已验证的真实慢变刚体比力。放大图按 G2-G1 均方误差增量最大翼拍选择，仅用于解释；完整翼拍图保留所有正负结果。GNSS 图的时间已按固定 110ms 假设校正。未修改留出、先验或相位模板。</p>'
(D/'index.html').write_text(html)

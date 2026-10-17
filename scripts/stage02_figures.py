#!/usr/bin/env python3
"""Readable summary figures from sealed diagnostics; no re-fitting or selection."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path('runs/stage02_20261017');out=root/'figures';out.mkdir(exist_ok=False)
s=json.loads((root/'final_validation/summary.json').read_text());primary=s['primary_runs']
phase=list(csv.DictReader((root/'final_validation/phase_harmonics.csv').open()))
def name(seg,dt,q):
    base=f'{seg}_full_dt{dt}_q{q}'
    return base+'_iter400' if base+'_iter400' in primary else base
fig,axs=plt.subplots(1,3,figsize=(13,4),sharey=True)
for ax,seg in zip(axs,'ABC'):
    for i,q in enumerate(['0.1','1','10']):
        n=name(seg,20,q);rows=sorted([r for r in phase if r['run']==n and r['sensor']=='gyro' and r['axis']=='1' and float(r['duration_s'])==16],key=lambda r:int(r['harmonic']))
        ax.bar(np.arange(1,4)+(i-1)*.22,[float(r['gain']) for r in rows],width=.22,label=f'jerk PSD x{q}')
    ax.axhline(1,color='k',lw=.7,ls='--');ax.set_xticks([1,2,3]);ax.set_xlabel('Measured wing-phase harmonic');ax.set_title(f'Segment {seg} | gyro y | 20 ms knots');ax.grid(axis='y',alpha=.2)
axs[0].set_ylabel('Prediction / observed harmonic amplitude');axs[-1].legend(fontsize=8);fig.tight_layout();fig.savefig(out/'prior_harmonic_tradeoff.png',dpi=160);plt.close(fig)
fig,axs=plt.subplots(2,1,figsize=(12,6),sharex=True)
for ax,kind,unit in zip(axs,['accel','gyro'],['m/s2','rad/s']):
    d=np.genfromtxt(root/name('A',20,'1')/(kind+'_predictions.csv'),delimiter=',',names=True);t0=101.704719;m=(d['t']>=t0)&(d['t']<t0+1);d=d[m];axis=2 if kind=='accel' else 1
    ax.plot(d['t']-t0,d[f'obs{axis}'],color='.6',lw=.8,label='All calibrated observations')
    ax.plot(d['t']-t0,d[f'pred{axis}'],label='20 ms GP sensor prediction',lw=1.2)
    hold=d['holdout']!=0;ax.scatter(d['t'][hold]-t0,d[f'obs{axis}'][hold],color='crimson',marker='x',s=30,label='Withheld before fitting',zorder=3)
    ax.set_ylabel(f'{kind} {"xyz"[axis]} ({unit})');ax.grid(alpha=.2);ax.legend(loc='upper right',fontsize=8)
axs[-1].set_xlabel('Seconds from fixed A target start');fig.suptitle('Good phase response does not imply pointwise held-out accuracy');fig.tight_layout();fig.savefig(out/'A_heldout_detail.png',dpi=160);plt.close(fig)
fig,axs=plt.subplots(1,2,figsize=(11,4))
for ax,metric,title in zip(axs,['accel','gyro'],['Specific-force vector RMSE (m/s2)','Angular-rate vector RMSE (rad/s)']):
    for j,dt in enumerate([20,50]):
        vals=[]
        for seg in 'ABC':
            r=next(r for r in s['runs'] if r['run']==name(seg,dt,'1'));vals.append(r['observations'][metric]['holdout']['vector_rmse'])
        ax.bar(np.arange(3)+(j-.5)*.3,vals,width=.3,label=f'{dt} ms knots')
    ax.set_xticks(range(3),list('ABC'));ax.set_title(title);ax.set_xlabel('Fixed segment');ax.legend();ax.grid(axis='y',alpha=.2)
fig.suptitle('Correlated held-out observations | baseline prior | not ground truth');fig.tight_layout();fig.savefig(out/'heldout_comparison.png',dpi=160);plt.close(fig)
inputs=[Path(__file__),root/'final_validation/summary.json',root/'final_validation/phase_harmonics.csv']+[root/name('A',20,'1')/(k+'_predictions.csv') for k in ['accel','gyro']]
(out/'manifest.json').write_text(json.dumps({'inputs':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},'outputs':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.glob('*.png')}},indent=2)+'\n')
print(out)

#!/usr/bin/env python3
"""Evaluation-only held-cycle projections, error identity, conditional robustness."""
import json,sys
from pathlib import Path
import numpy as np
import pandas as pd
from stage02_4_projection import ROOT,OUT,ContinuousPhase

def main():
 rows=[];paired=[];identity=[];baseline=[]
 cyc=pd.read_csv(OUT/'cycle_metrics.csv')
 for seg in ['A','B']:
  phase=ContinuousPhase.load(ROOT/f'runs/phase_reconstruction_20261019/{seg}_phase_model.json')
  for sensor in ['accel','gyro']:
   for name,path in [('H3',ROOT/f'runs/stage02_3B_20261019/{seg}_H3'),('Qian',OUT/f'{seg}_restore')]:
    d=pd.read_csv(path/f'{sensor}_predictions.csv');held=d.holdout.astype(bool)&d.valid.astype(bool);t=d.t[held].values;ph=phase.query(t);X=np.column_stack([np.ones(len(t)),t-np.mean(t)]+[f(h*ph) for h in [1,2,3] for f in [np.sin,np.cos]])
    for key in ['obs','slowpred','pred']:
     b=np.linalg.lstsq(X,d.loc[held,[f'{key}{j}' for j in range(3)]].values,rcond=None)[0]
     for axis in range(3):rows.append(dict(segment=seg,model=name,sensor=sensor,kind=key,axis=axis,held_projection_norm=float(np.linalg.norm(b[2:,axis]))))
   d=pd.read_csv(OUT/f'{seg}_restore/{sensor}_predictions.csv');held=d.holdout.astype(bool)&d.valid.astype(bool);o=d.loc[held,[f'obs{j}' for j in range(3)]].values;p=d.loc[held,[f'pred{j}' for j in range(3)]].values;a=d.loc[held,[f'addback{j}' for j in range(3)]].values;e1=a-o;e2=p-a
   identity.append(dict(segment=seg,sensor=sensor,total_mse=float(np.mean(np.sum((p-o)**2,axis=1))),remaining_slow_mse=float(np.mean(np.sum(e1**2,axis=1))),physical_projection_mse=float(np.mean(np.sum(e2**2,axis=1))),cross_term=float(2*np.mean(np.sum(e1*e2,axis=1)))))
   b=cyc[(cyc.name==seg+'_H3')&(cyc.sensor==sensor)&(cyc.output=='pred')].set_index('cycle');q=cyc[(cyc.name==seg+'_Qian')&(cyc.sensor==sensor)&(cyc.output=='pred')].set_index('cycle');diff=q.rmse-b.rmse
   for group,mask in [('all',np.ones(len(b),bool)),('early',b.t<(b.t.min()+b.t.max())/2),('late',b.t>=(b.t.min()+b.t.max())/2),('lower_frequency',b.frequency<b.frequency.median()),('higher_frequency',b.frequency>=b.frequency.median())]:paired.append(dict(segment=seg,sensor=sensor,group=group,n=int(sum(mask)),improved=int(sum(diff[mask]<0)),mean_H3=float(b.rmse[mask].mean()),mean_Qian=float(q.rmse[mask].mean())))
   decomp=pd.read_csv(OUT/f'observations/{seg}_phase32/{sensor}_decomposition.csv');train=decomp.train_regression.astype(bool);t=decomp.t.values;y=decomp[[f'raw{j}' for j in range(3)]].values;cycle=decomp.cycle.values;ids,cnt=np.unique(cycle[train],return_counts=True);w=np.array([1/cnt[np.searchsorted(ids,c)] for c in cycle[train]]);X=np.column_stack([np.ones(len(t)),t-np.mean(t[train])]);beta=np.linalg.lstsq(X[train]*np.sqrt(w[:,None]),y[train]*np.sqrt(w[:,None]),rcond=None)[0];held=decomp.holdout.astype(bool)&decomp.valid.astype(bool);null=X@beta;periodic=decomp[[f'per{j}' for j in range(3)]].values;trend=decomp[[f'trend{j}' for j in range(3)]].values
   for j in range(3):baseline.append(dict(segment=seg,sensor=sensor,axis=j,trend_only_rmse=float(np.sqrt(np.mean((null[held,j]-y[held,j])**2))),fixed_H3_rmse=float(np.sqrt(np.mean((trend[held,j]+periodic[held,j]-y[held,j])**2))),periodic_coefficient_norm=float(np.linalg.norm(np.array(json.loads((OUT/f'observations/{seg}_phase32/regression.json').read_text())['models'][sensor]['beta'])[2:,j]))))
 for label,data in [('held_projections',rows),('paired_cycles',paired),('error_decomposition',identity),('axis_separation_support',baseline)]:pd.DataFrame(data).to_csv(OUT/f'{label}.csv',index=False)
 print(pd.DataFrame(paired).query("group=='all'").to_string(index=False));print(pd.DataFrame(identity).to_string(index=False));print(pd.DataFrame(baseline).to_string(index=False))
if __name__=='__main__':main()

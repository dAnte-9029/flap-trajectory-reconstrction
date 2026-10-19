#!/usr/bin/env python3
"""Train-only cycle-balanced periodic subtraction, preserving the affine trend."""
import json,hashlib,sys,shutil
from pathlib import Path
import numpy as np
import pandas as pd
from stage02_4_paths import ROOT,OUT,CONFIG
sys.path.insert(0,str(ROOT/'experiments/stage02_2'));sys.path.insert(0,str(ROOT/'experiments/stage02_3A'))
from phase_models import fit_predict,fourier,design
from continuous_phase import ContinuousPhase

def prepare(seg,scale):
 dest=OUT/f'observations/{seg}_phase{scale}';dest.mkdir(parents=True,exist_ok=False)
 src=ROOT/f'runs/stage02_3B_20261019/observations/{seg}';cycles=pd.read_csv(ROOT/f'runs/stage02_2_20261017/regression/{seg}_cycles.csv')
 split=json.loads((ROOT/'runs/stage02_2_20261017/regression/fit_manifest.json').read_text())[f'{seg}_complete_cycle_mod5_eq2_accel_fourier_global_h3']
 pp=ROOT/'runs/phase_reconstruction_20261019';phase=ContinuousPhase.load(pp/(f'{seg}_phase_model.json' if scale==32 else f'fits/{seg}_scale_16.json'))
 model={};diagnostics=[]
 for sensor in ['accel','gyro']:
  raw=pd.read_csv(src/f'{sensor}.csv');t=raw.t.values;y=raw[['x','y','z']].values;cy=np.searchsorted(cycles.lo.values,t,side='right')-1;in_cycles=(cy>=0)&(t<cycles.hi.iloc[-1]);inside=(t>=phase.domain[0])&(t<=phase.domain[1]);ph=np.zeros(len(t));ph[inside]=phase.query(t[inside]);train=inside&in_cycles&np.isin(cy,split['train_cycles'])&raw.valid.astype(bool).values
  assert not np.any(train&raw.holdout.astype(bool).values)
  _,info=fit_predict(t,ph,y,cy,train,inside,3,False);b=np.asarray(info['beta']);periodic=fourier(ph,3)@b[2:];trend=design(t,ph,3,info['center'],info['scale'],False)[:,:2]@b[:2]
  # All statistics (including centering/scale) are fit using train cycles only.
  poisoned=y.copy();poisoned[raw.holdout.astype(bool)]+=1e6
  _,check=fit_predict(t,ph,poisoned,cy,train,inside,3,False);assert check==info
  remaining=y-periodic;residual=remaining-trend
  raw[['x','y','z']]=remaining;raw.loc[~inside,'valid']=0
  raw.to_csv(dest/f'{sensor}.csv',index=False,float_format='%.17g')
  d=pd.DataFrame({'t':t,'per0':periodic[:,0],'per1':periodic[:,1],'per2':periodic[:,2],'cycle':cy,'train_regression':train.astype(int),'holdout':raw.holdout,'valid':raw.valid})
  for label,values in [('raw',y),('remaining',remaining),('trend',trend),('unexplained',residual)]:
   for j in range(3):d[f'{label}{j}']=values[:,j]
  d.to_csv(dest/f'{sensor}_decomposition.csv',index=False,float_format='%.17g');model[sensor]=info
  # Removal must preserve exact trend, plus all non-modelled residual content.
  assert np.max(np.abs(remaining-(trend+residual)))<1e-12
  for split_name,mask in [('fit',train),('held',inside&raw.holdout.astype(bool).values)]:
   error=trend[mask]+periodic[mask]-y[mask]
   diagnostics.append({'sensor':sensor,'split':split_name,'n':int(sum(mask)),'vector_rmse':float(np.sqrt(np.mean(np.sum(error**2,axis=1)))),'axis_rmse':np.sqrt(np.mean(error**2,axis=0)).tolist()})
 for name in ['gnss.csv','attitude_init.csv','metadata.json']:shutil.copyfile(src/name,dest/name)
 shutil.copyfile(ROOT/f'runs/stage02_3B_20261019/{seg}_phase{scale}.csv',dest/'phase_pieces.csv')
 (dest/'regression.json').write_text(json.dumps({'models':model,'metrics':diagnostics,'split':split,'phase_source':str(pp),'interpretation':'remaining is raw-minus-periodic, not identified physical slow motion','held_poison_invariance':True},indent=2)+'\n')
 return diagnostics

def main():
 assert CONFIG.read_bytes()==(OUT/'frozen_config.json').read_bytes()
 result={}
 for seg,scale in [('A',32),('B',32),('A',16)]:result[f'{seg}{scale}']=prepare(seg,scale)
 (OUT/'decomposition_metrics.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()

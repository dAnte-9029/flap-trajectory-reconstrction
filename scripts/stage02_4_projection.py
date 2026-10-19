#!/usr/bin/env python3
"""Measure phase projection of slow sensor predictions, using train-only statistics."""
import json,sys
from pathlib import Path
import numpy as np
import pandas as pd
from stage02_4_paths import ROOT,OUT,CONFIG
sys.path.insert(0,str(ROOT/'experiments/stage02_2'));sys.path.insert(0,str(ROOT/'experiments/stage02_3A'))
from phase_models import fit_predict
from continuous_phase import ContinuousPhase

def evaluate(name,seg,scale=32):
 out=[];cycles=pd.read_csv(ROOT/f'runs/stage02_2_20261017/regression/{seg}_cycles.csv');pc=ROOT/'runs/phase_reconstruction_20261019';phase=ContinuousPhase.load(pc/(f'{seg}_phase_model.json' if scale==32 else f'fits/{seg}_scale_16.json'))
 for sensor in ['accel','gyro']:
  slow=pd.read_csv(OUT/f'{name}_slow/{sensor}_predictions.csv');decomp=pd.read_csv(OUT/f'observations/{seg}_phase{scale}/{sensor}_decomposition.csv');idx=np.searchsorted(decomp.t.values,slow.t.values-1e-8);assert np.max(abs(decomp.t.values[idx]-slow.t.values))<1e-7
  d=decomp.iloc[idx].reset_index(drop=True);t=slow.t.values;ph=phase.query(t);cy=d.cycle.values;train=d.train_regression.astype(bool).values&np.isin(cy,cycles[cycles.target].cycle.values);coeffs={}
  for kind,y in [('raw',d[[f'raw{j}' for j in range(3)]].values),('remaining',d[[f'remaining{j}' for j in range(3)]].values),('slow_prediction',slow[[f'pred{j}' for j in range(3)]].values)]:
   _,detail=fit_predict(t,ph,y,cy,train,train,3,False);coeffs[kind]=np.asarray(detail['beta'])[2:]
  for axis in range(3):
   for kind,b in coeffs.items():out.append(dict(name=name,segment=seg,sensor=sensor,axis=axis,kind=kind,coefficient_norm=float(np.linalg.norm(b[:,axis])),relative_to_raw=float(np.linalg.norm(b[:,axis])/max(np.linalg.norm(coeffs['raw'][:,axis]),1e-12))))
 return out
if __name__=='__main__':
 rows=sum([evaluate(s,s) for s in ['A','B']],[]);pd.DataFrame(rows).to_csv(OUT/'primary_projection_gate.csv',index=False);d=pd.DataFrame(rows);selected=d[(d.kind=='slow_prediction')&(((d.sensor=='accel')&(d.axis==2))|((d.sensor=='gyro')&(d.axis==1)))];passed=bool((selected.relative_to_raw<.2).all())
 (OUT/'projection_gate.json').write_text(json.dumps({'passed':passed,'channels':selected.to_dict('records'),'threshold':.2},indent=2)+'\n');print(selected.to_string(index=False));assert passed,'Stop before physical recovery: separation gate failed'

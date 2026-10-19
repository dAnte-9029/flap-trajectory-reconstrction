#!/usr/bin/env python3
"""Frozen six-solve ablation, with one conditional predeclared initialization pair."""
import hashlib,json,subprocess,time,os
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
CONFIG=Path(os.environ.get('STAGE02_6_CONFIG',ROOT/'configs/stage02_6_ablation.json'))
CFG=json.loads(CONFIG.read_text())
REFERENCE=json.loads((ROOT/'configs/stage02_6_ablation.json').read_text())
assert {k:v for k,v in CFG.items() if k!='output'}=={k:v for k,v in REFERENCE.items() if k!='output'},'Only output relocation allowed'
OUT=ROOT/CFG['output'];SRC=ROOT/CFG['source']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def rmse(path):
 d=pd.read_csv(path);e=d[[f'pred{i}' for i in range(3)]].values-d[[f'obs{i}' for i in range(3)]].values
 return float(np.sqrt(np.mean(np.sum(e[d.holdout==1]**2,axis=1))))
def solve(seg,g,traditional=False):
 name=f'{seg}_G{g}'+('_traditional' if traditional else '');out=OUT/name
 src=SRC/f'observations/{seg}_phase32';meta=json.loads((src/'metadata.json').read_text())['segment']
 args=[OUT/'build/stage02_6_joint',src,out,src/'phase_pieces.csv',meta['start_s'],meta['end_s'],.05,10000,1000,1.5,.05,.11,0,0,0,0,400,0,1,g,int(traditional)]
 before={p.name:sha(p) for p in src.iterdir() if p.is_file()};start=time.time()
 execution=OUT/(name+'_execution.json')
 if out.exists():
  # Resume checks only. Never launch an existing real solve a second time.
  record=json.loads(execution.read_text())
  assert record['command']==list(map(str,args)) and record['input_sha']==before and record['returncode']==0
 else:
  with (OUT/(name+'.log')).open('w') as f:r=subprocess.run(list(map(str,args)),stdout=f,stderr=subprocess.STDOUT)
  record=dict(name=name,command=list(map(str,args)),returncode=r.returncode,wall_s=time.time()-start,input_sha=before)
  execution.write_text(json.dumps(record,indent=2)+'\n')
 assert before=={p.name:sha(p) for p in src.iterdir() if p.is_file()}
 assert record['returncode']==0,record
 sol=pd.read_csv(out/'solver.csv').iloc[0];assert sol.usable==1,record
 for sensor in ['accel','gyro','gnss']:
  d=pd.read_csv(out/f'{sensor}_predictions.csv');old=pd.read_csv(SRC/f'{seg}_slow/{sensor}_predictions.csv')
  cols=['t','valid','holdout','fix']+[f'obs{i}' for i in range(6 if sensor=='gnss' else 3)]
  assert np.array_equal(d[cols].values,old[cols].values),sensor
 historical=pd.read_csv(SRC/f'{seg}_slow/solver.csv').iloc[0]
 for key in ['accel_factors','gyro_factors','gnss_factors']:assert sol[key]==historical[key]
 count=int(historical.gnss_factors)
 n=pd.read_csv(out/'gnss_components.csv').iloc[0];assert n.joint==(count if g==0 else 0) and n.position==(count if g else 0) and n.conditional_velocity==(count if g==1 else 0)
 print(name,'held accel',rmse(out/'accel_predictions.csv'),'iterations',sol.iterations,flush=True)
 return out

def main():
 assert CONFIG.read_bytes()==(OUT/'frozen_config.json').read_bytes()
 assert pd.read_csv(OUT/'synthetic.csv').iloc[0].passed==1
 checks=[]
 for seg in ['A','B']:
  for g in range(3):solve(seg,g)
  init=[sha(OUT/f'{seg}_G{g}/initial_knots.csv') for g in range(3)];assert len(set(init))==1
  # Independently check every velocity initialization against retained position slopes.
  d=pd.read_csv(SRC/f'observations/{seg}_phase32/gnss.csv');d=d[(d.valid==1)&(d.holdout==0)];ts=d.t.values-.11;p=d[['px','py','pz']].values
  k=pd.read_csv(OUT/f'{seg}_G0/initial_knots.csv');j=np.clip(np.searchsorted(ts,k.t.values,side='left')-1,0,len(ts)-2);expected=(p[j+1]-p[j])/(ts[j+1]-ts[j])[:,None]
  err=float(np.max(abs(expected-k[['vx','vy','vz']].values)));assert err<1e-10
  checks.append(dict(segment=seg,initial_sha=init[0],all_groups_equal=True,position_only_velocity_max_error=err))
  (OUT/'initialization_checks.json').write_text(json.dumps(checks,indent=2)+'\n')
  if seg=='A':
   new=pd.read_csv(OUT/'A_G0/trajectory.csv');old=pd.read_csv(SRC/'A_slow/trajectory.csv');assert np.array_equal(new.t,old.t)
   angle=(Rotation.from_quat(old[['qx','qy','qz','qw']]).inv()*Rotation.from_quat(new[['qx','qy','qz','qw']])).magnitude();rr=float(np.sqrt(np.mean(angle**2)))
   ratio=abs(rmse(OUT/'A_G0/accel_predictions.csv')/rmse(SRC/'A_slow/accel_predictions.csv')-1)
   sensitivity=ratio>.01 or rr>.01
   (OUT/'initialization_trigger.json').write_text(json.dumps(dict(relative_held_rmse_change=ratio,rotation_rms_rad=rr,triggered=sensitivity),indent=2)+'\n')
 if sensitivity:
  for g in [1,2]:solve('A',g,True)
  assert sha(OUT/'A_G1_traditional/initial_knots.csv')==sha(OUT/'A_G2_traditional/initial_knots.csv')
if __name__=='__main__':main()

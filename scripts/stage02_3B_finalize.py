#!/usr/bin/env python3
"""Verify frozen inputs, fair masks and finite outputs; seal a local manifest."""
import hashlib,json,subprocess
from pathlib import Path
import os
import numpy as np
import pandas as pd
ROOT=Path(__file__).resolve().parents[1]
CONFIG=Path(os.environ.get('STAGE02_3B_CONFIG',str(ROOT/'configs/stage02_3B_joint.json')))
OUT=ROOT/json.loads(CONFIG.read_text())['output']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def git(path,*args):return subprocess.check_output(['git','-C',str(path),*args],text=True).strip()
def main():
 prep=json.loads((OUT/'preparation_manifest.json').read_text());checks=[]
 for path,h in prep['inputs'].items():assert sha(ROOT/path)==h
 assert (OUT/'frozen_config.json').read_bytes()==CONFIG.read_bytes()
 for seg in ['A','B']:
  for sensor in ['accel','gyro','gnss']:
   a=pd.read_csv(OUT/f'{seg}_H0/{sensor}_predictions.csv')
   for H in [1,3]:
    b=pd.read_csv(OUT/f'{seg}_H{H}/{sensor}_predictions.csv');cols=['t','holdout','valid','fix']+[c for c in a if c.startswith('obs')];assert np.array_equal(a[cols].values,b[cols].values)
   checks.append(f'{seg}/{sensor}: H0/H1/H3 identical native timestamps, observations, validity and masks')
  for sensor in ['accel','gyro']:
   a=pd.read_csv(OUT/f'observations/{seg}/{sensor}.csv');mask=np.zeros(len(a),bool)
   for lo,hi in prep['splits'][seg]['intervals']:mask|=(a.t>=lo)&(a.t<hi)
   assert np.array_equal(mask,a.holdout.astype(bool));checks.append(f'{seg}/{sensor}: complete-cycle split reconstructed exactly')
 for job in json.loads((OUT/'jobs.json').read_text()):
  run=OUT/job['name'];d=pd.read_csv(run/'trajectory.csv');assert np.isfinite(d.values).all();q=d[['qx','qy','qz','qw']].values;assert np.max(np.abs(np.linalg.norm(q,axis=1)-1))<1e-9
  solver=pd.read_csv(run/'solver.csv').iloc[0];assert solver.usable==1
  jac=pd.read_csv(run/'jacobian_monitor.csv')
  if len(jac):
   assert jac.endpoint_error.max()<5e-3 and jac.coefficient_bias_error.max()<1e-5
  checks.append(job['name']+': finite unit-quaternion solution; preregistered Jacobian bounds pass where applicable')
  flags=dict(reference_point='main IMU installation point under rigid mounting assumption',target=[float(d.t.iloc[0]),float(d.t.iloc[-1])],finite=True,unit_quaternion=True,physical_acceleration_validated=False,calibrated_posterior_uncertainty=False,phase_acceleration='mathematically consistent; physically low confidence',absolute_phase_and_timing='uncalibrated, fixed assumptions',initialization='EKF attitude can indirectly expose held IMU',formal_labels=False,development_segment=True)
  (run/'quality_flags.json').write_text(json.dumps(flags,indent=2)+'\n')
 sourcepaths=list((ROOT/'experiments/stage02_3B').glob('*'))+list((ROOT/'scripts').glob('stage02_3B_*.py'))+[CONFIG,ROOT/'docs/reports/02_3B_phase_aware_joint_reconstruction.md',ROOT/'docs/STATUS.md']
 sourcepaths += [ROOT/p for p in ['experiments/stage01/query_adapter.hpp','experiments/stage02/pvt_factor.hpp','experiments/stage02_2/periodic_motion.py','experiments/stage02_3A/continuous_phase.py']]
 manifest=dict(base_commit=git(ROOT,'rev-parse','HEAD'),source_sha={str(p.relative_to(ROOT)):sha(p) for p in sourcepaths if p.is_file()},upstream_gptr_commit=git('/home/zn/gptr','rev-parse','HEAD'),upstream_gptr_status=git('/home/zn/gptr','status','--porcelain'),executable_sha={p.name:sha(p) for p in (OUT/'build').glob('stage02_3B_*') if p.is_file()},input_sha=prep['inputs'],raw_sha={s:json.loads((OUT/f'observations/{s}/metadata.json').read_text())['segment']['sha256'] for s in ['A','B']},checks=checks,artifacts={str(p.relative_to(OUT)):sha(p) for p in sorted(OUT.rglob('*')) if p.is_file() and 'build' not in p.relative_to(OUT).parts and p.name!='manifest.json'})
 (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps({'checks':checks,'artifacts':len(manifest['artifacts']),'upstream':manifest['upstream_gptr_commit'],'upstream_status':manifest['upstream_gptr_status']},indent=2))
if __name__=='__main__':main()

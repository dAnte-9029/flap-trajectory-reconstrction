#!/usr/bin/env python3
"""Fixed-gauge exchange and phase-boundary checks; no optimization."""
import json,shutil,subprocess,sys
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from stage02_4_synthetic import measure,QUERY,ROOT,OUT

def qrun(run,t,stem):
 src=run/(stem+'_times.csv');dst=run/(stem+'_queries.csv');pd.DataFrame({'t':t}).to_csv(src,index=False,float_format='%.17g')
 if not dst.exists():subprocess.run([str(QUERY),str(run),str(src),str(dst)],check=True)
 return pd.read_csv(dst)
def vr(e):return float(np.sqrt(np.mean(np.sum(e*e,axis=1))))
def main():
 limits=[];exchange=[];amplitudes=[]
 for seg in ['A','B']:
  run=OUT/(seg+'_restore');settings=pd.read_csv(run/'query_settings.csv').iloc[0];pieces=pd.read_csv(run/'phase_pieces.csv');ts=pieces.start[(pieces.start>settings.start)&(pieces.start<settings.end)].values
  times=np.concatenate([np.column_stack([ts-h,ts+h]).ravel() for h in [1e-7,1e-9,1e-11]]);q=qrun(run,times,'phase_boundary');n=len(q)//3
  for i,h in enumerate([1e-7,1e-9,1e-11]):
   z=q.iloc[i*n:(i+1)*n];a=z.iloc[::2];b=z.iloc[1::2];r=dict(segment=seg,h=h)
   for k,cols in [('p',['px','py','pz']),('v',['vx','vy','vz']),('a',['ax','ay','az']),('omega',['wx','wy','wz']),('alpha',['alphax','alphay','alphaz'])]:r[k]=np.max(np.linalg.norm(b[cols].values-a[cols].values,axis=1))
   r['R']=np.max((Rotation.from_quat(a[['qx','qy','qz','qw']]).inv()*Rotation.from_quat(b[['qx','qy','qz','qw']])).magnitude());limits.append(r)
  d=pd.read_csv(run/'trajectory.csv');slow=pd.read_csv(run/'slow_trajectory.csv');p=d[['px','py','pz']].values-slow[['px','py','pz']].values;angle=(Rotation.from_quat(slow[['qx','qy','qz','qw']]).inv()*Rotation.from_quat(d[['qx','qy','qz','qw']])).magnitude();amplitudes.append(dict(segment=seg,max_periodic_displacement_norm=float(np.linalg.norm(p,axis=1).max()),rms_periodic_displacement_norm=vr(p),max_periodic_rotation_rad=float(max(angle)),rms_periodic_rotation_rad=float(np.sqrt(np.mean(angle**2)))))
  for scale in [.8,1.2]:
   dest=OUT/f'fixed_slow_exchange/{seg}_{scale}';dest.mkdir(parents=True,exist_ok=False)
   for name in ['knots.csv','phase_pieces.csv','query_settings.csv']:shutil.copyfile(run/name,dest/name)
   coeff=pd.read_csv(run/'coefficients.csv');coeff['value']*=scale;coeff.to_csv(dest/'coefficients.csv',index=False,float_format='%.17g')
   for sensor in ['accel','gyro']:
    pred=pd.read_csv(run/f'{sensor}_predictions.csv');states=qrun(dest,pred.t.values,sensor);y=measure(states,sensor,np.zeros(3));status=pd.read_csv(run/'solver.csv').iloc[0];bias=status[['ba_x','ba_y','ba_z']].values if sensor=='accel' else status[['bg_x','bg_y','bg_z']].values;y+=bias
    exchange.append(dict(segment=seg,scale=scale,sensor=sensor,change_vector_rmse=vr(y-pred[[f'pred{j}' for j in range(3)]].values)))
 pd.DataFrame(limits).to_csv(OUT/'phase_boundary_limits.csv',index=False);pd.DataFrame(exchange).to_csv(OUT/'fixed_slow_exchange.csv',index=False);pd.DataFrame(amplitudes).to_csv(OUT/'periodic_amplitudes.csv',index=False)
 print(pd.DataFrame(exchange).to_string(index=False));print(pd.DataFrame(amplitudes).to_string(index=False))
if __name__=='__main__':main()

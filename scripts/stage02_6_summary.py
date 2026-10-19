#!/usr/bin/env python3
"""Small reviewable evidence digest derived from the frozen six runs."""
import json
import numpy as np,pandas as pd
from scipy.spatial.transform import Rotation
from stage02_6_analyze import OUT,SRC,QC,vr

def main():
 ledger=pd.read_csv(OUT/'ledger.csv');cycles=pd.read_csv(OUT/'cycles.csv');wig=pd.read_csv(OUT/'cycle_wiggles.csv');summary={}
 for seg in 'AB':
  g1=seg+'_G1';g2=seg+'_G2'
  a=ledger.query("name==@g1 and sensor=='accel' and split=='held' and axis=='vector'").iloc[0];b=ledger.query("name==@g2 and sensor=='accel' and split=='held' and axis=='vector'").iloc[0]
  ca=cycles.query("name==@g1 and sensor=='accel' and split=='held' and axis=='vector'").set_index('cycle');cb=cycles.query("name==@g2 and sensor=='accel' and split=='held' and axis=='vector'").set_index('cycle');delta=cb.mse-ca.mse
  summary[seg]=dict(held_rmse_relative_increase=float(b.rmse/a.rmse-1),held_mse_increase=float(b.mse-a.mse),e2_mse_increase=float(b.e2_mse-a.e2_mse),cross12_change=float(b.cross12-a.cross12),cycles_improved=int(sum(delta<0)),cycles_total=len(delta),cycle_mse_change={str(k):float(v) for k,v in delta.items()})
  old=pd.read_csv(SRC/f'{seg}_slow/trajectory.csv');new=pd.read_csv(OUT/f'{seg}_G0/trajectory.csv');ro=Rotation.from_quat(old[QC]);rn=Rotation.from_quat(new[QC]);eo=pd.read_csv(SRC/f'{seg}_slow/accel_predictions.csv');en=pd.read_csv(OUT/f'{seg}_G0/accel_predictions.csv');assert np.array_equal(eo.t,en.t)
  summary[seg]['historical_G0']=dict(R_rms=float(np.sqrt(np.mean((ro.inv()*rn).magnitude()**2))),accel_prediction_difference_fit=vr((en[['pred0','pred1','pred2']].values-eo[['pred0','pred1','pred2']].values)[eo.holdout==0]),accel_prediction_difference_held=vr((en[['pred0','pred1','pred2']].values-eo[['pred0','pred1','pred2']].values)[eo.holdout==1]))
  summary[seg]['held_wiggle_medians']=wig.query("name.str.startswith(@seg) and split=='held'").groupby('name')[['a_centered_rms','a_peak','jerk_rms','snap_rms']].median().to_dict('index')
 kin=[];integ=[]
 for name in [f'{s}_G{g}' for s in 'AB' for g in range(3)]:
  k=pd.read_csv(OUT/name/'kinematics.csv');k['name']=name;kin.append(k)
  i=pd.read_csv(OUT/name/'integration.csv');i['name']=name;integ.append(i)
 pd.concat(kin).to_csv(OUT/'kinematics_summary.csv',index=False);pd.concat(integ).to_csv(OUT/'integration_summary.csv',index=False)
 (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
if __name__=='__main__':main()

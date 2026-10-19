#!/usr/bin/env python3
"""Post-fit phase-knot limits and maneuver views; no new fit or parameter selection."""
import json,subprocess
from pathlib import Path
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
CONFIG=Path(os.environ.get('STAGE02_3B_CONFIG',str(ROOT/'configs/stage02_3B_joint.json')))
OUT=ROOT/json.loads(CONFIG.read_text())['output']
def main():
 limits=[];statechanges=[]
 for seg in ['A','B']:
  run=OUT/f'{seg}_H3';settings=pd.read_csv(run/'query_settings.csv').iloc[0];pieces=pd.read_csv(run/'phase_pieces.csv');times=pieces.start[(pieces.start>settings.start)&(pieces.start<settings.end)].values
  times=np.concatenate([np.ravel(np.column_stack([times-h,times+h])) for h in [1e-7,1e-9,1e-11]])
  tfile=run/'phase_boundary_times.csv';pd.DataFrame({'t':times}).to_csv(tfile,index=False,float_format='%.17g');qfile=run/'phase_boundary_queries.csv'
  if not qfile.exists():subprocess.run([str(OUT/'build/stage02_3B_query'),str(run),str(tfile),str(qfile)],check=True)
  q=pd.read_csv(qfile);count=len(q)//3
  for i,h in enumerate([1e-7,1e-9,1e-11]):
   block=q.iloc[i*count:(i+1)*count];l=block.iloc[::2];r=block.iloc[1::2];row=dict(segment=seg,h=h,n=len(l))
   for name,cols in [('p',['px','py','pz']),('v',['vx','vy','vz']),('a',['ax','ay','az']),('omega',['wx','wy','wz']),('alpha',['alphax','alphay','alphaz'])]:row[name]=np.max(np.linalg.norm(r[cols].values-l[cols].values,axis=1))
   row['R']=np.max((Rotation.from_quat(l[['qx','qy','qz','qw']].values).inv()*Rotation.from_quat(r[['qx','qy','qz','qw']].values)).magnitude());limits.append(row)
  a=pd.read_csv(OUT/f'{seg}_H0/trajectory.csv');b=pd.read_csv(run/'trajectory.csv');f,ax=plt.subplots(2,2,figsize=(13,8))
  for d,label in [(a,'H0'),(b,'H3')]:
   t=d.t-d.t.iloc[0];ax[0,0].plot(d.py,d.px,label=label);ax[0,1].plot(t,-d.pz,label=label);ax[1,0].plot(t,np.linalg.norm(d[['vx','vy','vz']],axis=1),label=label);ax[1,1].plot(t,np.linalg.norm(d[['wx','wy','wz']],axis=1),label=label)
  gps=pd.read_csv(run/'gnss_predictions.csv');ax[0,0].scatter(gps.obs1,gps.obs0,s=8,c='black',label='GNSS antenna, assumed zero lever');ax[0,1].scatter(gps.t-b.t.iloc[0],-gps.obs2,s=8,c='black',label='GNSS')
  for z,title in zip(ax.ravel(),['E/N position (m)','Up displacement (m)','Speed (m/s)','Body angular-rate norm (rad/s)']):z.set_title(seg+' '+title);z.legend(fontsize=7);z.grid(alpha=.2)
  f.tight_layout();f.savefig(OUT/f'figures/{seg}_maneuver.png',dpi=150);plt.close(f)
  for name,cols in [('p',['px','py','pz']),('v',['vx','vy','vz']),('a',['ax','ay','az']),('omega',['wx','wy','wz']),('alpha',['alphax','alphay','alphaz'])]:statechanges.append(dict(segment=seg,quantity=name,H0_H3_rms=float(np.sqrt(np.mean(np.sum((a[cols].values-b[cols].values)**2,axis=1))))))
  statechanges.append(dict(segment=seg,quantity='R_rad',H0_H3_rms=float(np.sqrt(np.mean((Rotation.from_quat(a[['qx','qy','qz','qw']].values).inv()*Rotation.from_quat(b[['qx','qy','qz','qw']].values)).magnitude()**2)))))
 pd.DataFrame(limits).to_csv(OUT/'phase_boundary_limits.csv',index=False);pd.DataFrame(statechanges).to_csv(OUT/'state_changes.csv',index=False)
 index=OUT/'index.html'
 if index.exists():
  html=index.read_text()
  for seg in ['A','B']:
   if f'{seg}_maneuver.png' not in html:html+=f'<h2>{seg}：整体轨迹及机动</h2><img style="max-width:100%" src="figures/{seg}_maneuver.png">'
  index.write_text(html)
 print(pd.DataFrame(limits).to_string(index=False))
if __name__=='__main__':main()

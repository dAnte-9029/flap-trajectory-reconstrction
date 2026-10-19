#!/usr/bin/env python3
"""Exact conditional recovery gate plus an explicit signal-template approximation case."""
import json,sys,subprocess,shutil
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from stage02_4_paths import ROOT,OUT,CONFIG
S=OUT/'synthetic'
sys.path.insert(0,str(ROOT/'experiments/stage02_2'))
from phase_models import fit_predict,fourier
QUERY=ROOT/'runs/stage02_3B_20261019/build/stage02_3B_query'
def query(run,t,name):
 inp=S/(name+'_times.csv');output=S/(name+'_query.csv');pd.DataFrame({'t':t}).to_csv(inp,index=False,float_format='%.17g');subprocess.run([str(QUERY),str(run),str(inp),str(output)],check=True);return pd.read_csv(output)
def measure(d,kind,lever):
 R=Rotation.from_quat(d[['qx','qy','qz','qw']]);a=d[['ax','ay','az']].values;w=d[['wx','wy','wz']].values
 if kind=='accel':return R.inv().apply(a+[0,0,-9.80665])
 if kind=='gyro':return w
 return np.column_stack([d[['px','py','pz']].values+R.apply(np.tile(lever,(len(d),1))),d[['vx','vy','vz']].values+R.apply(np.cross(w,lever))])
def main():
 S.mkdir(exist_ok=False)
 for name in ['slow','truth','raw','oracle','template']:(S/name).mkdir()
 rows=[];u=np.array([.31,-.17,.13]);v=np.array([-.11,.23,.09])
 for t in np.linspace(-.5,2.5,61):
  Q=Rotation.from_rotvec(v*t*t);R=Rotation.from_rotvec(u*t)*Q;w=Q.inv().apply(u)+2*t*v;alpha=-np.cross(2*t*v,Q.inv().apply(u))+2*v
  rows.append([t,t+.3*t*t,.2*t**3,-.1*t*t,1+.6*t,.6*t*t,-.2*t,.6,1.2*t,-.2,*R.as_quat(),*w,*alpha])
 header='t,px,py,pz,vx,vy,vz,ax,ay,az,qx,qy,qz,qw,wx,wy,wz,alphax,alphay,alphaz'.split(',');knots=pd.DataFrame(rows,columns=header)
 coeff=[]
 for group in [0,1]:
  for h in [1,2,3]:
   for j in range(6):coeff.append((.003/h**2 if group==0 else .012/h)*np.sin(1.7*(6*(h-1)+j+1)))
 coeff=np.asarray(coeff);ba=np.array([.12,-.07,.05]);bg=np.array([.003,-.002,.001]);lever=np.array([.21,-.09,.05])
 for name,H in [('slow',0),('truth',3)]:
  run=S/name;knots.to_csv(run/'knots.csv',index=False,float_format='%.17g');pd.DataFrame([[H,0,2,1e4,1e3,.11,*lever]],columns=['H','start','end','qt','qr','delay','lx','ly','lz']).to_csv(run/'query_settings.csv',index=False)
  pd.DataFrame([[-.5,2.5,-11.8375,23.375,.55,.1]],columns=['start','end','a0','a1','a2','a3']).to_csv(run/'phase_pieces.csv',index=False)
  pd.DataFrame({'index':np.arange(36) if H else [],'value':coeff if H else [],'prior_sigma':np.zeros(36) if H else []}).to_csv(run/'coefficients.csv',index=False,float_format='%.17g')
  pd.DataFrame([[1,0,0,0,0,0,0,0,0,0,0,*ba,*bg]],columns='usable,termination,initial_cost,final_cost,iterations,time_s,local_parameters,accel_factors,gyro_factors,gnss_factors,phase_boundary_excluded,ba_x,ba_y,ba_z,bg_x,bg_y,bg_z'.split(',')).to_csv(run/'solver.csv',index=False)
 templates={};transient={}
 for sensor,dt,offset in [('accel',.01,.003),('gyro',.0125,.007),('gnss',.2,.019)]:
  t=np.arange(-.48+offset,2.48,dt);slow=query(S/'slow',t,sensor+'_slow');truth=query(S/'truth',t,sensor+'_truth');ys=measure(slow,sensor,lever);yt=measure(truth,sensor,lever);phi=24*t+.7*t*t+.1*t**3;cy=np.floor(phi/(2*np.pi)).astype(int);held=(cy%5==2)&(t>=0)&(t<=2)
  if sensor=='gnss':
   rows=[]
   for i,tt in enumerate(t):rows.append([tt+.11,*yt[i],*np.diag([.01,.01,.02,.04,.04,.06]).ravel(),int(i%5==2),1,6,0])
   pd.DataFrame(rows,columns=['t','px','py','pz','vx','vy','vz']+[f'c{i}' for i in range(36)]+['holdout','valid','fix','utc_usec']).to_csv(S/'raw/gnss.csv',index=False,float_format='%.17g');continue
  y=yt+(ba if sensor=='accel' else bg);pd.DataFrame(dict(t=t,x=y[:,0],y=y[:,1],z=y[:,2],holdout=held.astype(int),valid=1)).to_csv(S/f'raw/{sensor}.csv',index=False,float_format='%.17g')
  per=yt-ys;pd.DataFrame(np.column_stack([t,per]),columns=['t','per0','per1','per2']).to_csv(S/f'oracle/{sensor}_decomposition.csv',index=False,float_format='%.17g')
  _,info=fit_predict(t,phi,y,cy,~held,np.ones(len(t),bool),3,False);fp=fourier(phi,3)@np.asarray(info['beta'])[2:];templates[sensor]=dict(vector_effect_rmse=float(np.sqrt(np.mean(np.sum((fp-per)**2,axis=1)))),fit=info)
  pd.DataFrame(np.column_stack([t,fp]),columns=['t','per0','per1','per2']).to_csv(S/f'template/{sensor}_decomposition.csv',index=False,float_format='%.17g')
  pulse=4*np.exp(-.5*((t-.85)/.03)**2);yp=y.copy();yp[:,0]+=pulse;_,ip=fit_predict(t,phi,yp,cy,~held,np.ones(len(t),bool),3,False);difference=pulse-(fourier(phi,3)@(np.asarray(ip['beta'])[2:]-np.asarray(info['beta'])[2:]))[:,0];transient[sensor]={'retained_peak_fraction':float(difference[np.argmax(pulse)]/max(pulse)),'outside_3sigma_leakage_max':float(np.max(np.abs(difference[np.abs(t-.85)>.09])))}
 results={}
 for case in ['oracle','template']:
  args=[OUT/'build/stage02_4_recover',S/'slow',S/'raw',S/case,S/(case+'_recovered'),0,0]
  with (S/(case+'.log')).open('w') as f:subprocess.run(list(map(str,args)),stdout=f,stderr=subprocess.STDOUT,check=True)
  estimated=pd.read_csv(S/(case+'_recovered')/'coefficients.csv').value.values;jac=pd.read_csv(S/(case+'_recovered')/'jacobian_monitor.csv').coefficient_error.max();results[case]={'position_coefficient_max_error':float(np.max(abs(estimated[:18]-coeff[:18]))),'rotation_coefficient_max_error':float(np.max(abs(estimated[18:]-coeff[18:]))),'coefficient_jacobian_max':float(jac)}
  if case=='oracle':assert max(results[case].values())<1e-5,results[case]
 (S/'validation.json').write_text(json.dumps({'passed':True,'recoveries':results,'template_forward_mismatch':templates,'transient_test':transient,'limitations':'Oracle conditional identifiability is not full trajectory physical observability. Template case deliberately measures finite-H3 sensor approximation with varying phase rate and noncommuting attitude.'},indent=2)+'\n')
 print(json.dumps(results,indent=2))
if __name__=='__main__':main()

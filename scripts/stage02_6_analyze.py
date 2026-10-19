#!/usr/bin/env python3
"""Read-only matched ablation evaluation at native times and off-grid queries."""
import json,hashlib,subprocess
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from stage02_6_run import ROOT,OUT,SRC,CFG
QUERY=ROOT/'runs/stage02_3B_20261019/build/stage02_3B_query'
QC=['qx','qy','qz','qw'];PC=['px','py','pz'];VC=['vx','vy','vz'];AC=['ax','ay','az'];WC=['wx','wy','wz'];LC=['alphax','alphay','alphaz']
def xyz(d,p,n=3):return d[[f'{p}{j}' for j in range(n)]].values
def vr(e):return float(np.sqrt(np.mean(np.sum(e*e,axis=1))))
def query(run,t,label):
 folder=OUT/'queries';folder.mkdir(exist_ok=True);tp=folder/(label+'_times.csv');rp=folder/(label+'.csv')
 if not rp.exists():
  pd.DataFrame({'t':t}).to_csv(tp,index=False,float_format='%.17g');subprocess.run([str(QUERY),str(run),str(tp),str(rp)],check=True,stdout=subprocess.DEVNULL)
 d=pd.read_csv(rp);assert np.max(abs(d.t.values-t))<1e-10
 return d

def weights(w):
 n=np.linalg.norm(w,axis=1);return np.minimum(1,3/np.maximum(n,1e-30)),.5*np.where(n<=3,n*n,6*n-9)
def ws(e,w):
 weight,cost=weights(w);n=np.linalg.norm(w,axis=1)
 return dict(n=len(e),rmse=vr(e),white_norm_median=float(np.median(n)),white_norm_p95=float(np.quantile(n,.95)),downweighted_fraction=float(np.mean(weight<1)),rho_prime_mean=float(np.mean(weight)),robust_cost=float(sum(cost)),**{f'axis{j}_rmse':float(np.sqrt(np.mean(e[:,j]**2))) for j in range(e.shape[1])},**{f'axis{j}_bias':float(np.mean(e[:,j])) for j in range(e.shape[1])})
def ledger(e1,e2,mask):
 a=e1[mask];b=e2[mask];total=a+b;m1=np.mean(a*a,axis=0);m2=np.mean(b*b,axis=0);cross=2*np.mean(a*b,axis=0);mse=np.mean(total*total,axis=0)
 assert np.max(abs(mse-m1-m2-cross))<1e-9
 rows=[]
 for ax in ['vector',0,1,2]:
  agg=lambda x:float(sum(x)) if ax=='vector' else float(x[ax])
  aa=agg(m1);bb=agg(m2);cc=agg(cross)
  se=np.sum(total*total,axis=1) if ax=='vector' else total[:,ax]**2
  rows.append(dict(axis=ax,n=len(a),rmse=np.sqrt(agg(mse)),mse=agg(mse),e1_mse=aa,e2_mse=bb,cross12=cc,delta_GP=bb+cc,normalized_dot=cc/2/np.sqrt(max(aa*bb,1e-30)),top5_sse_fraction=float(sum(np.sort(se)[-max(1,int(np.ceil(.05*len(se)))):])/sum(se)),p95_error=float(np.sqrt(np.quantile(se,.95))),max_error=float(np.sqrt(max(se)))))
 return rows

def main():
 names=[p.stem.removesuffix('_execution') for p in sorted(OUT.glob('*_execution.json'))]
 summaries=[];cycles=[];quarters=[];whites=[];costs=[];checks=[];biases=[];native={};gps={};couplings=[];state_stats=[];differences=[];increments=[];association=[];wiggles=[];covchecks=[]
 for name in names:
  seg=name[0];run=OUT/name;src=SRC/f'observations/{seg}_phase32';sol=pd.read_csv(run/'solver.csv').iloc[0];setting=pd.read_csv(run/'query_settings.csv').iloc[0];group=int(pd.read_csv(run/'gnss_components.csv').iloc[0].group)
  ba=sol[['ba_x','ba_y','ba_z']].values.astype(float);bg=sol[['bg_x','bg_y','bg_z']].values.astype(float)
  biases.append(dict(name=name,**{k:float(sol[k]) for k in sol.index},bias_cost=.5*(sum((ba/.5)**2)+sum((bg/.02)**2))))
  knots=pd.read_csv(run/'knots.csv');ph=pd.read_csv(run/'phase_pieces.csv');lo=max(knots.t.iloc[0],ph.start.iloc[0]);hi=min(knots.t.iloc[-1],ph.end.iloc[-1]);fc={}
  for sensor,dim in [('accel',3),('gyro',3),('gnss',6)]:
   raw=pd.read_csv(src/(sensor+'.csv'));t=raw.t.values-(.11 if sensor=='gnss' else 0);valid=(raw.valid==1).values&(t>=lo)&(t<=hi)&(t<knots.t.iloc[-1]);d=raw[valid].reset_index(drop=True);t=t[valid];q=query(run,t,name+'_'+sensor)
   R=Rotation.from_quat(q[QC]);pred=R.inv().apply(q[AC].values-[0,0,9.80665])+ba if sensor=='accel' else q[WC].values+bg if sensor=='gyro' else q[PC+VC].values
   obs=d[PC+VC if sensor=='gnss' else ['x','y','z']].values;e=pred-obs;held=d.holdout.values.astype(bool);core=(t>=setting.start)&(t<=setting.end)
   saved=pd.read_csv(run/f'{sensor}_predictions.csv');assert len(saved)==sum(core);err=float(np.max(abs(pred[core]-xyz(saved,'pred',dim))));assert err<1e-8
   if sensor=='gnss':
    cov=d[[f'c{i}' for i in range(36)]].values.reshape(-1,6,6);white=[];wp=[];wv=[];Klist=[]
    for c,z in zip(cov,e):
     pp=c[:3,:3];K=np.linalg.solve(pp,c[:3,3:]).T;schur=c[3:,3:]-K@c[:3,3:]
     wp.append(np.linalg.solve(np.linalg.cholesky(pp),z[:3]));wv.append(np.linalg.solve(np.linalg.cholesky(schur),z[3:]-K@z[:3]));Klist.append(K);white.append(np.linalg.solve(np.linalg.cholesky(c),z))
    white=np.array(white);wp=np.array(wp);wv=np.array(wv)
    parity=float(np.max(abs(np.sum(white**2,axis=1)-np.sum(wp**2,axis=1)-np.sum(wv**2,axis=1))));assert parity<1e-8
    covchecks.append(dict(name=name,cross_covariance_max=float(np.max(abs(cov[:,:3,3:]))),min_eigenvalue=float(np.linalg.eigvalsh(cov).min()),block_norm_max_difference=parity))
    pw,pc=weights(wp);vw,vc=weights(wv);jw,jc=weights(white);actualpw=jw if group==0 else pw
    fc['gnss']=float(sum((jc if group==0 else pc+vc if group==1 else pc)[~held]))
    fc['gnss_position']=float(sum(pc[~held])) if group else 0;fc['gnss_conditional_velocity']=float(sum(vc[~held])) if group==1 else 0
    export=pd.DataFrame(dict(t=t,publication_t=d.t,holdout=held.astype(int),core=core.astype(int),position_rho_prime=actualpw,conditional_velocity_rho_prime=vw,velocity_factor_active=int(group!=2)))
    for j in range(3):export[f'position_white{j}']=wp[:,j];export[f'velocity_conditional_white{j}']=wv[:,j]
    for region,m in [('all',np.ones(len(t),bool)),('core',core)]:
     for split,mask in [('fit',m&~held),('held',m&held)]:
      for kind,x,w in [('position',e[:,:3],wp),('velocity_conditional',e[:,3:],wv)]:
       row=dict(name=name,sensor='gnss_'+kind,region=region,split=split,**ws(x[mask],w[mask]),weight_is_actual=split=='fit' and (kind=='position' or group!=2))
       if kind=='position':row['actual_rho_prime_mean']=float(np.mean(actualpw[mask]));row['actual_downweighted_fraction']=float(np.mean(actualpw[mask]<1))
       whites.append(row)
    gps[name]=(t,e,held,core)
   else:
    white=e/(1.5 if sensor=='accel' else .05);fc[sensor]=float(sum(weights(white)[1][~held]));export=pd.DataFrame(dict(t=t,holdout=held.astype(int),core=core.astype(int),rho_prime=weights(white)[0]))
    for region,m in [('all',np.ones(len(t),bool)),('core',core)]:
     for split,mask in [('fit',m&~held),('held',m&held)]:whites.append(dict(name=name,sensor=sensor,region=region,split=split,weight_is_actual=split=='fit',**ws(e[mask],white[mask])))
    dec=pd.read_csv(src/f'{sensor}_decomposition.csv');dd=saved.merge(dec,on='t',suffixes=('','_d'),validate='one_to_one');assert len(dd)==len(saved) and np.array_equal(dd.holdout,dd.holdout_d)
    y=xyz(dd,'raw');T=xyz(dd,'trend');P=xyz(dd,'per');S=xyz(dd,'pred');e1=T+P-y;e2=S-T;total=S+P-y
    ident=float(np.max(abs(total-e1-e2)));assert ident<1e-10 and np.max(abs(xyz(dd,'obs')-(y-P)))<1e-12
    point=dd[['t','cycle','holdout','valid']].copy()
    for label,z in [('y',y),('T',T),('P',P),('S',S),('e1',e1),('e2',e2),('total',total)]:
     for j in range(3):point[f'{label}{j}']=z[:,j]
    point.to_csv(OUT/f'{name}_{sensor}_ledger_points.csv',index=False,float_format='%.17g')
    for split,mask in [('fit',dd.holdout.values==0),('held',dd.holdout.values==1)]:
     for row in ledger(e1,e2,mask):summaries.append(dict(name=name,sensor=sensor,split=split,**row))
     for quarter in range(4):
      mm=mask&(dd.t.values>=setting.start+4*quarter)&(dd.t.values<(setting.start+4*(quarter+1)))
      if mm.any():
       for row in ledger(e1,e2,mm):quarters.append(dict(name=name,sensor=sensor,split=split,quarter=quarter+1,**row))
    for cy in np.unique(dd.cycle):
     mm=dd.cycle.values==cy;assert len(np.unique(dd.holdout[mm]))==1
     for row in ledger(e1,e2,mm):cycles.append(dict(name=name,sensor=sensor,cycle=int(cy),split='held' if dd.holdout[mm].iloc[0] else 'fit',t_lo=float(dd.t[mm].min()),t_hi=float(dd.t[mm].max()),**row))
    if sensor=='accel':native[name]=(t,q,held,core,ba,dd)
    checks.append(dict(name=name,sensor=sensor,identity_max=ident))
   werr=float(np.max(abs(white[core]-xyz(saved,'white',dim))));assert werr<1e-7
   checks.append(dict(name=name,sensor=sensor,query_prediction_max=err,white_max=werr))
   for j in range(dim):export[f'residual{j}']=e[:,j];export[f'white{j}']=white[:,j];export[f'pred{j}']=pred[:,j];export[f'obs{j}']=obs[:,j]
   export.to_csv(OUT/f'{name}_{sensor}_weights.csv',index=False,float_format='%.17g')
  old=pd.read_csv(run/'cost_groups.csv').set_index('group').robust_cost;bc=.5*(sum((ba/.5)**2)+sum((bg/.02)**2));total=fc['accel']+fc['gyro']+fc['gnss']+old.slow_prior+bc
  assert abs(fc['accel']+fc['gyro']-old.imu)<.06
  assert abs(fc['gnss']-(old.gnss+old.gnss_position+old.gnss_conditional_velocity))<.01
  assert abs(total-sol.final_cost)<.06
  costs.append(dict(name=name,**fc,prior=old.slow_prior,bias=bc,total=total,solver_total=sol.final_cost,rounding_difference=total-sol.final_cost))
 # All state statistics below are curve diagnostics, not external physical errors.
 for name,(t,q,held,core,ba,dd) in native.items():
  R=Rotation.from_quat(q[QC]);a=q[AC].values;alpha=q[LC].values;grav=-R.inv().apply(np.tile([0,0,9.80665],(len(t),1)))
  for split,m in [('fit',core&~held),('held',core&held)]:state_stats.append(dict(name=name,split=split,a_rms=vr(a[m]),a_peak=float(np.linalg.norm(a[m],axis=1).max()),alpha_rms=vr(alpha[m]),alpha_peak=float(np.linalg.norm(alpha[m],axis=1).max()),omega_rms=vr(q[WC].values[m]),gravity_demean_rms=vr(grav[m]-grav[m].mean(axis=0))))
  # Curvature is finite difference of the saved continuous curve at native times;
  # compute before masking, never difference concatenated held cycles.
  jerk=np.gradient(a,t,axis=0,edge_order=2);snap=np.gradient(jerk,t,axis=0,edge_order=2);qc=q[core].reset_index(drop=True);assert np.max(abs(qc.t-dd.t))<1e-10
  for cy in np.unique(dd.cycle):
   m=dd.cycle.values==cy;aa=a[core][m]
   wiggles.append(dict(name=name,cycle=int(cy),split='held' if dd.holdout[m].iloc[0] else 'fit',n=int(sum(m)),a_centered_rms=vr(aa-aa.mean(axis=0)),a_peak=float(np.linalg.norm(aa,axis=1).max()),jerk_rms=vr(jerk[core][m]),snap_rms=vr(snap[core][m]),**{f'a_axis{j}_range':float(np.ptp(aa[:,j])) for j in range(3)}))
  gt,ge,gh,gc=gps[name];ae=xyz(dd,'pred')-xyz(dd,'obs');at=dd.t.values
  rows=[]
  for i in np.flatnonzero(gc):
   m=abs(at-gt[i])<=.1
   if m.any():rows.append(dict(name=name,t=gt[i],split='held' if gh[i] else 'fit',velocity_error_norm=float(np.linalg.norm(ge[i,3:])),nearby_accel_rmse=vr(ae[m]),nearby_held_fraction=float(dd.holdout[m].mean())))
  association.extend(rows)
 pairs=[(f'{s}_G0',f'{s}_G1','robust_structure') for s in ['A','B']]+[(f'{s}_G1',f'{s}_G2','velocity_ablation') for s in ['A','B']]
 if 'A_G1_traditional' in names:pairs += [('A_G1_traditional','A_G2_traditional','traditional_ablation'),('A_G1','A_G1_traditional','initialization'),('A_G2','A_G2_traditional','initialization')]
 for base,alt,kind in pairs:
  t,b,h,core,bab,dd=native[base];tt,a,hh,cc,baa,_=native[alt];assert np.array_equal(t,tt) and np.array_equal(h,hh)
  rb=Rotation.from_quat(b[QC]);ra=Rotation.from_quat(a[QC]);da=ra.inv().apply(a[AC].values-b[AC].values);dr=ra.inv().apply(b[AC].values-[0,0,9.80665])-rb.inv().apply(b[AC].values-[0,0,9.80665]);db=np.tile(baa-bab,(len(t),1));dg=-ra.inv().apply(np.tile([0,0,9.80665],(len(t),1)))+rb.inv().apply(np.tile([0,0,9.80665],(len(t),1)))
  gt,_,gh,_=gps[base];dist=np.min(abs(t[:,None]-gt[~gh][None,:]),axis=1)
  for split,m0 in [('fit',core&~h),('held',core&h)]:
   couplings.append(dict(base=base,alt=alt,kind=kind,split=split,prediction_change_rms=vr((da+dr+db)[m0]),a_effect_rms=vr(da[m0]),R_effect_rms=vr(dr[m0]),bias_effect_rms=vr(db[m0]),gravity_change_rms=vr(dg[m0]),cross_a_R=float(2*np.mean(np.sum(da[m0]*dr[m0],axis=1)))))
   for region,m in [('all',m0),('within_50ms_train_GNSS',m0&(dist<=.05)),('farther_than_50ms',m0&(dist>.05))]:
    differences.append(dict(base=base,alt=alt,kind=kind,split=split,region=region,n=int(sum(m)),**{label+'_rms':vr(a[cols].values[m]-b[cols].values[m]) for label,cols in [('p',PC),('v',VC),('a',AC),('omega',WC),('alpha',LC)]},R_rms=float(np.sqrt(np.mean((rb.inv()*ra).magnitude()[m]**2)))))
  # Off-grid intervals and precise cycle membership, including crossing held boundaries.
  setting=pd.read_csv(OUT/base/'query_settings.csv').iloc[0];starts=np.arange(setting.start+.01317,setting.end-.2,.03713);durations=[.02,.05,.1,.2];times=np.concatenate([starts]+[starts+dt for dt in durations]);N=len(starts)
  qb=query(OUT/base,times,base+'_intervals');qa=query(OUT/alt,times,alt+'_intervals')
  cy=pd.read_csv(ROOT/f'runs/stage02_2_20261017/regression/{base[0]}_cycles.csv');heldcycles=set(dd.loc[dd.holdout==1,'cycle']);startcy=np.searchsorted(cy.hi.values,starts,side='right');startids=cy.cycle.values[np.clip(startcy,0,len(cy)-1)];sh=np.isin(startids,list(heldcycles))
  gn=gt[~gh];near=np.min(abs(starts[:,None]-gn[None,:]),axis=1)<=.05
  for j,dt in enumerate(durations):
   b0=qb.iloc[:N];b1=qb.iloc[(j+1)*N:(j+2)*N];a0=qa.iloc[:N];a1=qa.iloc[(j+1)*N:(j+2)*N]
   diff={label:(a1[cols].values-a0[cols].values)-(b1[cols].values-b0[cols].values) for label,cols in [('dp',PC),('dv',VC),('dw',WC)]}
   diff['dR']=((Rotation.from_quat(b0[QC]).inv()*Rotation.from_quat(b1[QC])).inv()*(Rotation.from_quat(a0[QC]).inv()*Rotation.from_quat(a1[QC]))).as_rotvec()
   endcy=np.searchsorted(cy.hi.values,starts+dt,side='right');cross=startcy!=endcy
   for split,m0 in [('fit',~sh),('held',sh)]:
    for region,m in [('all',m0),('within_50ms_train_GNSS',m0&near),('farther_than_50ms',m0&~near)]:
     increments.append(dict(base=base,alt=alt,kind=kind,split=split,region=region,dt=dt,n=int(sum(m)),cross_cycle_fraction=float(np.mean(cross[m])),**{label+'_rms':vr(z[m]) for label,z in diff.items()},average_a_difference_rms=vr(diff['dv'][m]/dt)))
 for label,rows in [('ledger',summaries),('cycles',cycles),('quarters',quarters),('whitening',whites),('factor_costs',costs),('solver_biases',biases),('state_diagnostics',state_stats),('state_differences',differences),('coupling',couplings),('interval_differences',increments),('cycle_wiggles',wiggles),('gnss_velocity_accel_association',association),('covariance_checks',covchecks)]:pd.DataFrame(rows).to_csv(OUT/(label+'.csv'),index=False)
 assoc=pd.DataFrame(association);correlations=[]
 for (name,split),d in assoc.groupby(['name','split']):correlations.append(dict(name=name,split=split,n=len(d),pearson=float(np.corrcoef(d.velocity_error_norm,d.nearby_accel_rmse)[0,1])))
 pd.DataFrame(correlations).to_csv(OUT/'gnss_accel_correlations.csv',index=False)
 (OUT/'evaluation_checks.json').write_text(json.dumps(checks,indent=2)+'\n')
 print(pd.DataFrame(summaries).query("sensor=='accel' and axis=='vector'").to_string(index=False));print(pd.DataFrame(costs).to_string(index=False))
if __name__=='__main__':main()

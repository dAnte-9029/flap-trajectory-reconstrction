"""Independent native-time forward prediction, residual ledger and mathematical checks."""
from common import *
from continuous_phase import ContinuousPhase
from phase_models import design
from scipy.spatial.transform import Rotation,Slerp
from pyulog import ULog

COLS={'p':['px','py','pz'],'v':['vx','vy','vz'],'a':['ax','ay','az'],'omega':['wx','wy','wz'],'alpha':['alphax','alphay','alphaz']}
Q=['qx','qy','qz','qw']
def rms(x):return float(np.sqrt(np.mean(np.sum(np.asarray(x)**2,axis=1))))
def query(run,t,stem,build):
 t=np.asarray(t);p=run/(stem+'_times.csv');q=run/(stem+'_states.csv')
 if p.exists() or q.exists():raise FileExistsError(stem)
 pd.DataFrame(dict(t=t)).to_csv(p,index=False,float_format='%.17g');execute([build/'full_query',run,p,q],run/(stem+'_query_execution.json'))
 d=pd.read_csv(q);assert np.max(abs(d.t-t))<1e-10
 return d

def measurement(d,kind,lever,bias):
 R=Rotation.from_quat(d[Q]);v={k:d[cs].to_numpy() for k,cs in COLS.items()}
 if kind=='accel':return R.inv().apply(v['a']-[0,0,9.80665])+bias
 if kind=='gyro':return v['omega']+bias
 return np.column_stack([v['p']+R.apply(np.tile(lever,(len(d),1))),v['v']+R.apply(np.cross(v['omega'],lever))])

def metric(e):
 n=np.linalg.norm(e,axis=1)
 return dict(n=len(e),vector_rmse=rms(e),vector_mae=float(np.mean(n)),vector_p95=float(np.quantile(n,.95)),axis_rmse=np.sqrt(np.mean(e*e,axis=0)).tolist(),axis_mae=np.mean(abs(e),axis=0).tolist(),axis_bias=np.mean(e,axis=0).tolist(),axis_abs_p95=np.quantile(abs(e),.95,axis=0).tolist())

def mathematics(run,build):
 settings=pd.read_csv(run/'query_settings.csv').iloc[0];start,end=settings.start,settings.end
 base=np.arange(start+.123173,end-.13,.17313);records=[]
 for h in [1e-4,1e-5,1e-6]:
  z=query(run,np.column_stack([base-h,base,base+h]).ravel(),f'fd_{h}',build);l=z.iloc[::3];m=z.iloc[1::3];r=z.iloc[2::3]
  errors={'p_dot_v':(r[COLS['p']].values-l[COLS['p']].values)/(2*h)-m[COLS['v']].values,'v_dot_a':(r[COLS['v']].values-l[COLS['v']].values)/(2*h)-m[COLS['a']].values,'R_dot_omega':(Rotation.from_quat(l[Q]).inv()*Rotation.from_quat(r[Q])).as_rotvec()/(2*h)-m[COLS['omega']].values,'omega_dot_alpha':(r[COLS['omega']].values-l[COLS['omega']].values)/(2*h)-m[COLS['alpha']].values}
  records.append(dict(test='offgrid_derivative',step=h,**{k:float(np.max(np.linalg.norm(v,axis=1))) for k,v in errors.items()}))
 knots=pd.read_csv(run/'knots.csv').t.to_numpy();phase=pd.read_csv(run/'phase_pieces.csv').start.to_numpy()
 for name,ts in [('gp_boundary',knots),('phase_boundary',phase)]:
  ts=ts[(ts>start)&(ts<end)]
  for h in [1e-7,1e-9,1e-11]:
   d=query(run,np.column_stack([ts-h,ts+h]).ravel(),f'{name}_{h}',build);l=d.iloc[::2];r=d.iloc[1::2]
   e={k:float(np.max(np.linalg.norm(r[c].values-l[c].values,axis=1))) for k,c in COLS.items()};e['R']=float(max((Rotation.from_quat(l[Q]).inv()*Rotation.from_quat(r[Q])).magnitude()));records.append(dict(test=name,step=h,**e))
 dump(run/'mathematics.json',records)
 # Independent numerical integral at fixed non-knot starts; exact same query curve.
 ints=[];starts=np.linspace(start+.21317,end-.3,min(23,max(2,int((end-start)/.5))))
 for dur in [.02,.05,.1,.2]:
  for dt in [.002,.0005,.000125]:
   n=int(np.ceil(dur/dt));h=dur/n;ts=np.concatenate([np.r_[t,t+dur,t+(np.arange(n)+.5)*h] for t in starts]);d=query(run,ts,f'integrate_{dur}_{dt}',build);worst=np.zeros(6)
   for i,t in enumerate(starts):
    z=d.iloc[i*(n+2):(i+1)*(n+2)];a=z.iloc[0];b=z.iloc[1];mid=z.iloc[2:];R=Rotation.from_quat(a[Q].values)
    for w in mid[COLS['omega']].values:R=R*Rotation.from_rotvec(w*h)
    err=[np.linalg.norm(a[COLS['p']].values+mid[COLS['v']].values.sum(axis=0)*h-b[COLS['p']].values),np.linalg.norm(a[COLS['v']].values+mid[COLS['a']].values.sum(axis=0)*h-b[COLS['v']].values),(R.inv()*Rotation.from_quat(b[Q].values)).magnitude(),np.linalg.norm(a[COLS['omega']].values+mid[COLS['alpha']].values.sum(axis=0)*h-b[COLS['omega']].values),np.linalg.norm(a[COLS['v']].values+a[COLS['a']].values*dur-b[COLS['v']].values),np.linalg.norm(a[COLS['omega']].values+a[COLS['alpha']].values*dur-b[COLS['omega']].values)]
    worst=np.maximum(worst,err)
   ints.append(dict(duration=dur,step=h,p=worst[0],v=worst[1],R=worst[2],omega=worst[3],left_endpoint_v=worst[4],left_endpoint_omega=worst[5]))
 pd.DataFrame(ints).to_csv(run/'independent_integration.csv',index=False)
 smallest=[r for r in records if r['test']=='offgrid_derivative' and r['step']==1e-6][0]
 passed=max(v for k,v in smallest.items() if k not in ['test','step'])<5e-3
 dump(run/'math_gate.json',dict(derivative_passed=passed,tolerance=5e-3,meaning='numerical consistency only; no physical accuracy implication'))
 assert passed,smallest


def evaluate(run,build,do_math=True):
 cfg=json.loads((run/'run_config.json').read_text());raw=Path(cfg['input']);settings=cfg['config'];method=cfg['method'];status=pd.read_csv(run/'solver.csv').iloc[0];meta=json.loads((raw/'metadata.json').read_text());spec=meta['segment'];knots=pd.read_csv(run/'knots.csv');phase=ContinuousPhase.load(raw/'phase_model.json')
 u=ULog(meta['input_path'],message_name_filter_list=['vehicle_status']);vs=next(d.data for d in u.data_list if d.name=='vehicle_status' and d.multi_id==0)
 metrics=[];harmonics=[]
 for kind in ['accel','gyro','gnss']:
  d=pd.read_csv(raw/f'{kind}.csv',float_precision='round_trip');dim=6 if kind=='gnss' else 3;t=d.t.to_numpy()-(settings['gnss_delay'] if kind=='gnss' else 0)
  inside=(t>=knots.t.iloc[0])&(t<knots.t.iloc[-1])
  if method=='phase':inside&=(t>=phase.domain[0])&(t<=phase.domain[1])
  mask=inside&(d.valid==1);assert not d.holdout.any();assert sum(mask)==status[kind+'_factors']
  x=query(run,t[mask],'native_'+kind,build);bias=status[['ba_x','ba_y','ba_z'] if kind=='accel' else ['bg_x','bg_y','bg_z']].values if kind!='gnss' else np.zeros(3)
  pred=measurement(x,kind,np.array(settings['lever']),bias);obs=d.loc[mask,['px','py','pz','vx','vy','vz'] if kind=='gnss' else ['x','y','z']].values;e=pred-obs
  white=np.empty_like(e)
  if kind=='gnss':
   cov=d.loc[mask,[f'c{i}' for i in range(36)]].values.reshape(-1,6,6);white=np.linalg.solve(np.linalg.cholesky(cov),e[:,:,None])[:,:,0]
  else:white=e/(settings['sigma_accel'] if kind=='accel' else settings['sigma_gyro'])
  # Actual optimization residual differs from final raw residual in phase projection.
  objective=white.copy()
  if method=='phase' and kind!='gnss':
   slow=Path(cfg['fixed_slow']);sx=query(slow,t[mask],run.name+'_objective_'+kind,build);sy=measurement(sx,kind,np.array(settings['lever']),bias)
   dec=pd.read_csv(Path(cfg['decomposition'])/f'{kind}_decomposition.csv');assert np.max(abs(dec.t-d.t))<1e-10
   target=sy+dec.loc[mask,['per0','per1','per2']].values
   objective=(pred-target)/(settings['sigma_accel'] if kind=='accel' else settings['sigma_gyro'])
  norm=np.linalg.norm(objective,axis=1);weight=np.minimum(1,3/np.maximum(norm,1e-300));robust=.5*np.where(norm<=3,norm**2,6*norm-9)
  # Keep ALL observations including invalid/outside, and explicit participation/reasons.
  ledger=d.copy();ledger['measurement_t']=t;ledger['participates']=mask.astype(int);ledger['target_interior']=((t>=spec['start_s'])&(t<=spec['end_s'])).astype(int);ledger['exclusion']=np.where(d.valid==0,'invalid_observation',np.where(~inside,'outside_supported_context','included'))
  ix=np.maximum(0,np.searchsorted(vs['timestamp']*1e-6,t,side='right')-1);ledger['nav_state']=vs['nav_state'][ix]
  for label,arr in [('prediction',pred),('residual',e),('white',white),('objective_white',objective)]:
   for j in range(dim):ledger[f'{label}{j}']=np.nan;ledger.loc[mask,f'{label}{j}']=arr[:,j]
  for label,arr in [('actual_huber_weight',weight),('actual_robust_cost',robust)]:ledger[label]=np.nan;ledger.loc[mask,label]=arr
  ledger.to_csv(run/f'{kind}_ledger.csv',index=False,float_format='%.17g')
  lm=ledger.loc[mask];groups={'all_context':np.ones(len(lm),bool),'target':lm.target_interior.values==1}
  for k in sorted(lm.nav_state.unique()):groups[f'nav_{int(k)}']=(lm.nav_state==k).values
  bins=np.floor((lm.measurement_t-spec['start_s'])/1).astype(int)
  for k in sorted(bins.unique()):groups[f'second_{k}']=(bins==k).values
  if kind=='gnss':
   for k in sorted(lm.fix.unique()):groups[f'fix_{int(k)}']=(lm.fix==k).values
  for cohort,select in groups.items():
   if not sum(select):continue
   parts=[('position',slice(0,3)),('velocity',slice(3,6))] if kind=='gnss' else [(kind,slice(None))]
   for label,sl in parts:metrics.append(dict(channel=label,cohort=cohort,**metric(e[select,sl]),white_vector_rmse=rms(white[select,sl]),actual_huber_downweighted_fraction=float(np.mean(weight[select]<1)),actual_robust_cost_sum=float(sum(robust[select])),residual_meaning='separated sensor observation physical difference' if method=='slow' else 'raw observation physical difference',objective_meaning='derived slow+template projection' if method=='phase' and kind!='gnss' else 'direct observation'))
  if kind!='gnss':
   inside_phase=(x.t>=phase.domain[0])&(x.t<=phase.domain[1]);tt=x.t.values[inside_phase];ph=phase.query(tt);D=design(tt,ph,3,float(np.mean(tt)),max(float(np.ptp(tt)/2),.1),False)
   for label,Y in [('observation',obs[inside_phase]),('prediction',pred[inside_phase]),('residual',e[inside_phase])]:
    b=np.linalg.lstsq(D,Y,rcond=None)[0];nonperiodic=Y-D[:,2:]@b[2:]
    for h in [1,2,3]:
     sin,cos=b[2+2*(h-1):4+2*(h-1)]
     for axis in range(3):harmonics.append(dict(sensor=kind,source=label,harmonic=h,axis=axis,amplitude=float(np.hypot(sin[axis],cos[axis])),phase_rad=float(np.arctan2(cos[axis],sin[axis])),nonperiodic_axis_std=float(np.std(nonperiodic[:,axis])),peak_abs=float(np.max(abs(Y[:,axis]))),p99_abs=float(np.quantile(abs(Y[:,axis]),.99))))
 dump(run/'observation_metrics.json',metrics);pd.DataFrame(harmonics).to_csv(run/'harmonics.csv',index=False)
 # Fused attitude: native reference-only, never an independent factor.
 a=pd.read_csv(raw/'attitude_init.csv');a=a[(a.t>=spec['start_s'])&(a.t<=spec['end_s'])];x=query(run,a.t,'attitude_reference',build);angle=(Rotation.from_quat(a[Q]).inv()*Rotation.from_quat(x[Q])).magnitude();a=a.copy();a['geodesic_difference_rad']=angle;a['independent_truth']=False;a.to_csv(run/'attitude_reference.csv',index=False)
 dump(run/'reference_metrics.json',dict(attitude_rms_rad=float(np.sqrt(np.mean(angle**2))),p95_rad=float(np.quantile(angle,.95)),independent_truth=False))
 d=pd.read_csv(run/'trajectory.csv');d['timestamp_sample_us']=np.rint(d.t*1e6).astype('int64');R=Rotation.from_quat(d[Q]);ab=R.inv().apply(d[COLS['a']]);d[['a_body_x','a_body_y','a_body_z']]=ab;d['phase_rad']=phase.query(d.t);d['phase_velocity_rad_s']=phase.query(d.t,1);d['phase_valid']=1;d['interior_support']=1;d['independent_physical_validation']=False
 for kind in ['accel','gyro','gnss']:
  ld=pd.read_csv(run/f'{kind}_ledger.csv');ts=ld.measurement_t.values;j=np.clip(np.searchsorted(ts,d.t),0,len(ts)-1);jm=np.maximum(j-1,0);j=np.where(abs(ts[jm]-d.t.values)<abs(ts[j]-d.t.values),jm,j)
  d[kind+'_nearest_valid']=ld.valid.values[j];d[kind+'_nearest_dt_s']=abs(ts[j]-d.t.values)
  if kind=='gnss':d['nearest_gnss_fix']=ld.fix.values[j]
 d.to_csv(run/'trajectory_export.csv',index=False,float_format='%.17g')
 dump(run/'export_contract.json',dict(time='PX4 HRT seconds and microseconds; native asynchronous residual evaluation; GNSS observation t minus assumed delay',frames='p/v/a NED; quaternion xyzw body FRD to NED; omega/alpha FRD; a_body=R^-1 a_N, NOT specific force',reference_point=CFG['reference'],trust_interval=[spec['start_s'],spec['end_s']],query_support=[float(knots.t.iloc[0]),float(knots.t.iloc[-1])],context_quality='outside trust_interval queries have reduced boundary support',uncertainty='No calibrated state posterior; parameter contrasts only. No direct angular acceleration truth; phase absolute zero uncalibrated.',mode='offline_label_candidate',source_sha=meta['sha256']))
 if do_math:mathematics(run,build)
 print(run.name,[(m['channel'],round(m['vector_rmse'],5)) for m in metrics if m['cohort']=='target'],flush=True)
 return metrics

if __name__=='__main__':
 import argparse
 ap=argparse.ArgumentParser();ap.add_argument('run',type=Path,nargs='+');ap.add_argument('--build',type=Path,default=ROOT/CFG['output']/'build');ap.add_argument('--skip-math',action='store_true');a=ap.parse_args()
 for run in a.run:evaluate(run.resolve(),a.build.resolve(),not a.skip_math)

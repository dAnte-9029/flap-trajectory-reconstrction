"""Full native observation preparation; no inherited masks or periodic templates."""
from common import *
from continuous_phase import ContinuousPhase
from phase_models import fit_predict,fourier,design
from scipy.interpolate import PPoly
from pyulog import ULog
from stage02_extract import extract

def export_phase(model,dest):
 model.save(dest/'phase_model.json');pp=PPoly.from_spline(model.spline);rows=[]
 for i in range(len(pp.x)-1):
  if pp.x[i+1]<=pp.x[i] or pp.x[i]<0 or pp.x[i+1]>model.domain[1]-model.origin:continue
  a3,a2,a1,a0=pp.c[:,i];rows.append([pp.x[i]+model.origin,pp.x[i+1]+model.origin,a0+model.offset+model.slope*pp.x[i],a1+model.slope,a2,a3])
 pd.DataFrame(rows,columns=['start','end','a0','a1','a2','a3']).to_csv(dest/'phase_pieces.csv',index=False,float_format='%.17g')
 assert model.minimum_velocity()>0

def phase_from_ulog(path,dest,scale):
 meta=json.loads((dest/'metadata.json').read_text());s=meta['segment'];lo=s['start_s']-2;hi=s['end_s']+2
 u=ULog(str(path),message_name_filter_list=['wing_phase','encoder_count']);ds={(d.name,d.multi_id):d.data for d in u.data_list}
 w=ds['wing_phase',0];e=ds['encoder_count',0];ti=w['timestamp'];j=np.minimum(np.searchsorted(e['timestamp'],ti),len(e['timestamp'])-1)
 use=(ti*1e-6>=lo-.02)&(ti*1e-6<=hi+.02);assert np.all(e['timestamp'][j[use]]==ti[use]);t=ti[use]*1e-6;c=e['total_count'][j[use]].astype(float);r=e['position_raw'][j[use]].astype(int)
 assert np.all(w['phase_valid'][use]) and np.array_equal(np.diff(c),(np.diff(r)+2048)%4096-2048)
 ratio=u.initial_parameters['FLAP_RATIO']
 for tt,k,v in u.changed_parameters:
  if k=='FLAP_RATIO':
   if tt*1e-6<=lo:ratio=v
   elif tt*1e-6<=hi:raise ValueError('ratio change')
 y=float(w['phase_rad'][use][0])+(c-c[0])*2*np.pi/(4096*ratio)
 model=ContinuousPhase.fit(t,y,scale,[s['start_s'],s['end_s']]);export_phase(model,dest)
 pd.DataFrame(dict(t=t,encoder_phase=y,phase=model.query(t),phase_velocity=model.query(t,1),logged_frequency=w['flap_frequency_hz'][use],hall_phase=w['phase_rad'][use],phase_valid=w['phase_valid'][use])).to_csv(dest/'phase_observations.csv',index=False)
 dump(dest/'phase_audit.json',dict(ratio=float(ratio),rows=len(t),scale=scale,maximum_phase_error_rad=float(max(abs(model.query(t)-y))),min_velocity=model.minimum_velocity(),hall_zero_calibrated=False,phase_zero_resets='Hall corrections are diagnostic; continuous encoder gauge does not reset'))
 return model

def clear_masks(dest,historical=None):
 counts={}
 for sensor in ['accel','gyro','gnss']:
  p=dest/f'{sensor}.csv';d=pd.read_csv(p,float_precision='round_trip');old=d.holdout.to_numpy().copy()
  if historical is not None:old=pd.read_csv(historical/f'{sensor}.csv').holdout.to_numpy()
  d['holdout']=0
  d['historical_holdout']=old
  d.to_csv(p,index=False,float_format='%.17g')
  counts[sensor]=dict(rows=len(d),valid=int(d.valid.sum()),invalid=int((d.valid==0).sum()),historically_held_reincluded=int(((old==1)&(d.valid==1)).sum()))
 dump(dest/'full_observation.json',dict(mode='full_observation',counts=counts,exclusions='invalid finite/clipping/GNSS quality rules and explicitly recorded support boundaries only; no residual magnitude exclusion'))


def decompose(raw,dest):
 import shutil
 dest.mkdir(parents=True,exist_ok=False);phase=ContinuousPhase.load(raw/'phase_model.json');models={}
 for sensor in ['accel','gyro']:
  d=pd.read_csv(raw/f'{sensor}.csv',float_precision='round_trip');t=d.t.to_numpy();y=d[['x','y','z']].to_numpy();inside=(t>=phase.domain[0])&(t<=phase.domain[1]);assert inside.all()
  ph=phase.query(t);cycles=np.floor(ph/(2*np.pi)).astype(int);valid=d.valid.to_numpy().astype(bool)
  assert not d.holdout.any()
  _,info=fit_predict(t,ph,y,cycles,valid,np.ones(len(t),bool),3,False);per=fourier(ph,3)@np.array(info['beta'])[2:]
  d[['x','y','z']]=y-per;d.to_csv(dest/f'{sensor}.csv',index=False,float_format='%.17g')
  pd.DataFrame(dict(t=t,per0=per[:,0],per1=per[:,1],per2=per[:,2],fit_all_valid=valid.astype(int),historical_holdout=d.historical_holdout)).to_csv(dest/f'{sensor}_decomposition.csv',index=False,float_format='%.17g')
  info['n_fit']=int(valid.sum());info['historically_held_fit']=int((valid&(d.historical_holdout==1)).sum());models[sensor]=info
 for name in ['gnss.csv','attitude_init.csv','metadata.json','phase_model.json','phase_pieces.csv']:shutil.copyfile(raw/name,dest/name)
 dump(dest/'regression.json',dict(mode='full_observation',models=models,meaning='All valid samples; affine preserved. Derived sensor decomposition, not identified physical slow motion.'))


def prepare_ab(out):
 import shutil
 for seg in ['A','B']:
  dest=out/'observations'/seg;dest.mkdir(parents=True,exist_ok=False)
  src=ROOT/f'runs/stage02_3B_20261019/observations/{seg}'
  for name in ['accel.csv','gyro.csv','gnss.csv','attitude_init.csv','metadata.json']:shutil.copyfile(src/name,dest/name)
  meta=json.loads((dest/'metadata.json').read_text());assert sha(meta['input_path'])==meta['sha256']
  clear_masks(dest,src);phase_from_ulog(meta['input_path'],dest,32);decompose(dest,out/'observations'/(seg+'_separated'))


def audit_ulog(path,out):
 u=ULog(str(path));ds={(d.name,d.multi_id):d.data for d in u.data_list};events=[];topics=[]
 checks={'vehicle_land_detected':['landed'],'vehicle_status':['nav_state','arming_state'],'vehicle_attitude':['quat_reset_counter'],'vehicle_local_position':['xy_reset_counter','z_reset_counter','vxy_reset_counter','vz_reset_counter','heading_reset_counter','ref_timestamp'],'vehicle_gps_position':['fix_type','system_error','vel_ned_valid'],'wing_phase':['phase_valid'],'sensor_accel':['device_id','error_count'],'sensor_gyro':['device_id','error_count']}
 for (name,inst),d in ds.items():
  if name not in set(checks)|{'encoder_count','sensor_selection'}:continue
  t=d.get('timestamp_sample',d['timestamp'])*1e-6
  topics.append(dict(topic=name,instance=int(inst),n=len(t),start=float(t[0]),end=float(t[-1]),max_gap=float(max(np.diff(t))) if len(t)>1 else None))
  for k in checks.get(name,[]):
   if k not in d:continue
   for i in np.r_[0,np.flatnonzero(d[k][1:]!=d[k][:-1])+1]:events.append(dict(t=float(t[i]),topic=name,instance=int(inst),field=k,value=float(d[k][i])))
 for d in u.dropouts:events.append(dict(t=float(d.timestamp*1e-6),topic='logger',field='dropout_ms',value=float(d.duration)))
 for t,k,v in u.changed_parameters:
  if k.startswith(('CAL_','SENS_BOARD','TC_','FLAP_')):events.append(dict(t=float(t*1e-6),topic='parameter',field=k,value=float(v)))
 pd.DataFrame(events).to_csv(out/'log_events.csv',index=False);pd.DataFrame(topics).to_csv(out/'log_topics.csv',index=False)
 # Fixed admission: positive contiguous mechanical counts + raw modulo consistency,
 # valid phase, airborne; further strict IMU/GNSS/reset checks in existing extractor.
 w=ds['wing_phase',0];e=ds['encoder_count',0];t=w['timestamp']*1e-6;j=np.searchsorted(e['timestamp'],w['timestamp']);assert np.all(e['timestamp'][j]==w['timestamp'])
 c=e['total_count'][j].astype(float);r=e['position_raw'][j].astype(int);land=ds['vehicle_land_detected',0];li=np.searchsorted(land['timestamp'],w['timestamp'],side='right')-1
 good=(li>=0)&(~land['landed'][np.maximum(li,0)].astype(bool))&w['phase_valid'].astype(bool)
 edges=(np.diff(t)>0)&(np.diff(t)<.03)&(np.diff(c)>0)&(np.diff(c)<2048)&(np.diff(c)==(np.diff(r)+2048)%4096-2048)&good[:-1]&good[1:]
 ab=np.flatnonzero(np.diff(np.r_[False,edges,False])).reshape(-1,2);segments=[]
 for a,b in ab:
  if t[b]-t[a]<8:continue
  # Explicit 20ms extraction guard and 2s derivative context; dt multiple.
  lo=float(t[a]+.03);end=float(lo+np.floor((t[b]-.03-lo)/.05)*.05)
  segments.append(dict(start_s=lo+2,end_s=end-2,support_start=lo,support_end=end,encoder_start=float(t[a]),encoder_end=float(t[b])))
 pd.DataFrame(dict(t=t,airborne=good.astype(int),positive_contiguous_edge=np.r_[edges,False].astype(int),count=c,phase_valid=w['phase_valid'])).to_csv(out/'admission_native.csv',index=False)
 dump(out/'ulog_audit.json',dict(path=str(path),sha256=sha(path),firmware=u.msg_info_dict['ver_sw'],start=float(u.start_timestamp*1e-6),end=float(u.last_timestamp*1e-6),segments=segments,rule='minimum 8s positive phase+airborne; 30ms guard; 2s context. Strict extractor rejects gaps/resets/calibration changes; segments remain separate. Flight-mode transitions retained as cohorts if no frame reset.',unreconstructed='preflight/ground/invalid phase or nonpositive counts, short intervals, 30ms guards; context evaluated but target derivatives flagged boundary support'))
 return segments


def prepare_ulog(path,out,segments=None):
 out.mkdir(parents=True,exist_ok=False);audited=audit_ulog(path,out);cfg=json.loads((ROOT/'configs/stage02_reconstruction.json').read_text());cfg['ulog_root']=str(path.parent)
 # Disable old sample holdout before extraction, including GNSS origin selection.
 cfg['holdout_modulus']=2**62;cfg['holdout_remainder']=-1
 selected=segments or audited;result=[]
 for i,s in enumerate(selected):
  if not any(s['start_s']>=a['start_s'] and s['end_s']<=a['end_s'] for a in audited):raise ValueError('Requested segment outside audited common support')
  spec=dict(id=f'full_{i}',relative_path=path.name,sha256=sha(path),start_s=s['start_s'],end_s=s['end_s'],historical_identity_annotations='A source: historical development train, not test')
  dest=out/f'full_{i}';extract(spec,cfg,dest);clear_masks(dest);phase_from_ulog(path,dest,32);decompose(dest,out/f'full_{i}_separated');result.append(dest)
 return result

"""Check actual objective ledgers, raw-log coverage and native prediction parity."""
from common import *
from pyulog import ULog

def objective_checks(out):
 results=[]
 runs=[p.parent for p in out.rglob('run_config.json') if (p.parent/'accel_ledger.csv').exists()]
 for run in runs:
  cfg=json.loads((run/'run_config.json').read_text());status=pd.read_csv(run/'solver.csv').iloc[0];cost={};native_errors={}
  for k in ['accel','gyro','gnss']:
   ledger=pd.read_csv(run/f'{k}_ledger.csv');m=ledger.participates==1;cost[k]=float(ledger.loc[m,'actual_robust_cost'].sum())
   assert sum(m)==status[k+'_factors']
   old=pd.read_csv(run/f'{k}_predictions.csv');old=old[old.valid==1]
   # CSV time rounding only: nearest native sample must match <1ns.
   ld=ledger.loc[m];j=np.searchsorted(ld.measurement_t.values,old.t.values);j=np.clip(j,0,len(ld)-1);jm=np.maximum(j-1,0);j=np.where(abs(ld.measurement_t.values[jm]-old.t.values)<abs(ld.measurement_t.values[j]-old.t.values),jm,j)
   assert max(abs(ld.measurement_t.values[j]-old.t.values))<1e-9
   n=6 if k=='gnss' else 3;err=float(np.max(abs(ld[[f'prediction{i}' for i in range(n)]].values[j]-old[[f'pred{i}' for i in range(n)]].values)));assert err<1e-8;native_errors[k]=err
  if cfg['method']=='phase':
   c=pd.read_csv(run/'coefficients.csv');prior=float(.5*np.sum((c.value/c.prior_sigma)**2));error=abs(sum(cost.values())+prior-status.final_cost);assert error<1e-6*max(1,status.final_cost)
   result=dict(phase_objective_plus_coefficient_prior_error=error)
  else:
   groups=pd.read_csv(run/'cost_groups.csv').set_index('group').robust_cost
   errors={k:abs(v-groups[k])/max(1,abs(groups[k])) for k,v in [('imu',cost['accel']+cost['gyro']),('gnss',cost['gnss'])]};assert max(errors.values())<1e-5
   result=dict(cost_group_relative_errors=errors)
  results.append(dict(run=str(run.relative_to(out)),native_prediction_max_errors=native_errors,robust_costs=cost,**result))
 dump(out/'objective_verification.json',results)


def coverage(out):
 root=out/'full_log';audit=json.loads((root/'input/ulog_audit.json').read_text());u=ULog(audit['path']);ds={(d.name,d.multi_id):d.data for d in u.data_list};rows=[];land=ds['vehicle_land_detected',0];segments=audit['segments']
 for name,kind in [('sensor_accel','accel'),('sensor_gyro','gyro'),('vehicle_gps_position','gnss')]:
  d=ds[name,0];t=d.get('timestamp_sample',d['timestamp'])*1e-6;queryt=t-(CFG['gnss_delay'] if kind=='gnss' else 0);j=np.maximum(0,np.searchsorted(land['timestamp']*1e-6,t,side='right')-1);air=~land['landed'][j].astype(bool);inside=np.zeros(len(t),bool);core=inside.copy()
  for s in segments:inside|=(queryt>=s['support_start'])&(queryt<s['support_end']);core|=(queryt>=s['start_s'])&(queryt<=s['end_s'])
  ledger=pd.read_csv(root/f'full_0_direct/{kind}_ledger.csv');used_times=ledger.loc[ledger.participates==1,'t'].values;ji=np.searchsorted(t,used_times);jm=np.maximum(ji-1,0);ji=np.clip(ji,0,len(t)-1);ji=np.where(abs(t[jm]-used_times)<abs(t[ji]-used_times),jm,ji);assert max(abs(t[ji]-used_times))<1e-9;used=np.zeros(len(t),bool);used[ji]=True
  reason=np.where(used,np.where(core,'target_factor','context_factor'),np.where(~air,'ground_or_preflight','outside_common_continuous_phase_support_or_guard'))
  for why in np.unique(reason):rows.append(dict(topic=name,total_log_rows=len(t),classification=why,rows=int(sum(reason==why))))
  pd.DataFrame(dict(native_t=t,model_t=queryt,airborne=air.astype(int),participates=used.astype(int),reason=reason)).to_csv(root/f'{kind}_whole_log_coverage.csv',index=False)
 w=ds['wing_phase',0];ids=np.flatnonzero(np.diff(w['phase_unwrapped_rad'])<-np.pi)+1
 pd.DataFrame(dict(t=w['timestamp'][ids]*1e-6,reported_phase_unwrapped_before=w['phase_unwrapped_rad'][ids-1],reported_phase_unwrapped_after=w['phase_unwrapped_rad'][ids])).to_csv(root/'hall_zero_events.csv',index=False)
 dump(root/'raw_coverage_summary.json',rows)
 assert sha(audit['path'])==audit['sha256']

if __name__=='__main__':
 out=ROOT/CFG['output'];objective_checks(out);coverage(out);print('Objective parity, native prediction, raw coverage and source SHA verified')

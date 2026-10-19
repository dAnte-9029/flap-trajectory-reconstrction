"""Fixed sensitivity comparisons and observed harmonic/nonperiodic diagnostics."""
from analyze import *
from scipy.signal import lombscargle

def dynamic(run):
 cfg=json.loads((run/'run_config.json').read_text());raw=Path(cfg['input']);ph=ContinuousPhase.load(raw/'phase_model.json');rows=[];spectra=[]
 for sensor in ['accel','gyro']:
  ld=pd.read_csv(run/f'{sensor}_ledger.csv');ld=ld[(ld.participates==1)&(ld.target_interior==1)];t=ld.measurement_t.values;y=ld[['x','y','z']].values;yp=ld[[f'prediction{i}' for i in range(3)]].values;D=design(t,ph.query(t),3,float(np.mean(t)),float(np.ptp(t)/2),False)
  b=np.linalg.lstsq(D,y,rcond=None)[0];bp=np.linalg.lstsq(D,yp,rcond=None)[0];rest=y-D[:,2:]@b[2:];restp=yp-D[:,2:]@bp[2:];freq=np.linspace(.25,16,253)
  for j in range(3):
   transient=abs(np.diff(y[:,j]))/np.diff(t);mask=transient>=np.quantile(transient,.95);indices=np.flatnonzero(mask)+1
   rows.append(dict(sensor=sensor,axis=j,nonperiodic_rmse=float(np.sqrt(np.mean((rest[:,j]-restp[:,j])**2))),nonperiodic_correlation=float(np.corrcoef(rest[:,j],restp[:,j])[0,1]),observation_peak=float(max(abs(y[:,j]))),predicted_peak=float(max(abs(yp[:,j]))),observed_transient_subset_n=len(indices),transient_rmse=float(np.sqrt(np.mean((yp[indices,j]-y[indices,j])**2))),periodic_amplitude_ratio=float(np.linalg.norm(bp[2:,j])/max(np.linalg.norm(b[2:,j]),1e-12))))
   for label,z in [('observation',y),('prediction',yp),('residual',yp-y)]:
    # Native timestamps: a descriptive least-squares periodogram, not calibrated PSD.
    power=lombscargle(t-t[0],z[:,j]-np.mean(z[:,j]),2*np.pi*freq,normalize=True)
    spectra.extend(dict(sensor=sensor,axis=j,source=label,frequency_hz=float(f),normalized_lomb_power=float(p)) for f,p in zip(freq,power))
 pd.DataFrame(rows).to_csv(run/'dynamic_retention.csv',index=False);pd.DataFrame(spectra).to_csv(run/'native_spectra.csv',index=False)
 if cfg['method']=='phase':
  slow=Path(cfg['fixed_slow']);ratios=[]
  for sensor in ['accel','gyro']:
   ld=pd.read_csv(run/f'{sensor}_ledger.csv');ld=ld[(ld.participates==1)&(ld.target_interior==1)];t=ld.measurement_t.values;y=ld[['x','y','z']].values;xs=query(slow,t,run.name+'_separation_'+sensor,ROOT/CFG['output']/'build');status=pd.read_csv(slow/'solver.csv').iloc[0];bias=status[['ba_x','ba_y','ba_z'] if sensor=='accel' else ['bg_x','bg_y','bg_z']].values;ys=measurement(xs,sensor,np.array(cfg['config']['lever']),bias);D=design(t,ph.query(t),3,float(np.mean(t)),float(np.ptp(t)/2),False);b=np.linalg.lstsq(D,y,rcond=None)[0];bs=np.linalg.lstsq(D,ys,rcond=None)[0]
   for j in range(3):ratios.append(dict(sensor=sensor,axis=j,slow_to_raw_harmonic_norm=float(np.linalg.norm(bs[2:,j])/max(np.linalg.norm(b[2:,j]),1e-12))))
  dump(run/'separation.json',ratios)

def sensitivity(out):
 rows=[]
 for method in ['direct','phase']:
  base=pd.read_csv(out/f'A_{method}/trajectory.csv')
  for name in CFG['sensitivities_A_only']:
   run=out/f'A_{name}_{method}';d=pd.read_csv(run/'trajectory.csv');assert len(base)==len(d) and max(abs(base.t-d.t))<1e-9
   e={k:rms(d[c].values-base[c].values) for k,c in COLS.items()};e['R_rad']=float(np.sqrt(np.mean((Rotation.from_quat(base[Q]).inv()*Rotation.from_quat(d[Q])).magnitude()**2)))
   metrics=json.loads((run/'observation_metrics.json').read_text());row=dict(method=method,contrast=name,**e)
   for m in metrics:
    if m['cohort']=='target':row[m['channel']+'_fit_rmse']=m['vector_rmse']
   rows.append(row)
 pd.DataFrame(rows).to_csv(out/'sensitivity.csv',index=False)
 print(pd.DataFrame(rows).to_string(index=False))

if __name__=='__main__':
 import argparse
 ap=argparse.ArgumentParser();ap.add_argument('--sensitivity',action='store_true');ap.add_argument('runs',type=Path,nargs='*');a=ap.parse_args()
 for r in a.runs:dynamic(r.resolve())
 if a.sensitivity:sensitivity(ROOT/CFG['output'])

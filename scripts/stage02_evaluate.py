#!/usr/bin/env python3
"""Native-time residuals, measured wing-phase response and separate integrators.
No scores from neural models; plots and envelopes are diagnostics, not truth.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy import signal
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load(p): return np.genfromtxt(p,delimiter=',',names=True)
def matrix(d,names): return np.column_stack([d[n] for n in names])
def write_csv(path,rows):
    if not rows:return
    with path.open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def phase_design(t,phase):
    return np.column_stack([np.ones(len(t)),t-t.mean()]+[fn(k*phase) for k in [1,2,3] for fn in [np.sin,np.cos]])


def evaluate(run,root,out,plots):
    manifest=json.loads((run/'manifest.json').read_text());seg=manifest['segment']['id'][0]
    wing=load(root/'observations'/seg/'wing.csv');wv=(wing['valid']!=0)&np.isfinite(wing['phase']);wt=wing['t'][wv];phase=np.unwrap(wing['phase'][wv])
    solver=load(run/'solver.csv');tr=load(run/'trajectory.csv');r=Rotation.from_quat(matrix(tr,['qx','qy','qz','qw']))
    info={'run':run.name,'solver':{n:float(solver[n]) for n in solver.dtype.names},'observations':{},
          'uncertainty':'sensitivity only; no calibrated covariance; body rigidity, main-IMU point, extrinsics, delays unverified'}
    residual_rows=[];dynamic=[];spectra=[]
    fig,axes=plt.subplots(3,2,figsize=(13,8),sharex=True) if plots else (None,None)
    for kind in ['accel','gyro','gnss']:
        d=load(run/(kind+'_predictions.csv'));dim=6 if kind=='gnss' else 3
        obs=matrix(d,[f'obs{i}' for i in range(dim)]);pred=matrix(d,[f'pred{i}' for i in range(dim)]);white=matrix(d,[f'white{i}' for i in range(dim)])
        good=d['valid']!=0;info['observations'][kind]={}
        for subset in ['fit','holdout','all']:
            valid=good&((d['holdout']!=0) if subset=='holdout' else ((d['holdout']==0) if subset=='fit' else True))
            err=pred[valid]-obs[valid];we=white[valid];norm=np.linalg.norm(we,axis=1)
            metrics={'n':int(sum(valid)),'axis_rmse':np.sqrt(np.mean(err**2,axis=0)).tolist(),'axis_mean':np.mean(err,axis=0).tolist(),
                'axis_p95_abs':np.quantile(abs(err),.95,axis=0).tolist(),'vector_rmse':float(np.sqrt(np.mean(np.sum(err**2,axis=1)))),
                'normalized_vector_p50_p95':np.quantile(norm,[.5,.95]).tolist(),'robust_radius_exceed_fraction':float(np.mean(norm>3))}
            if kind=='gnss':metrics.update(vector_rmse_note='Six-vector mixes m and m/s; use separate position/velocity errors below',position_vector_rmse=float(np.sqrt(np.mean(np.sum(err[:,:3]**2,axis=1)))),velocity_vector_rmse=float(np.sqrt(np.mean(np.sum(err[:,3:]**2,axis=1)))))
            info['observations'][kind][subset]=metrics
            for j in range(dim):residual_rows.append({'run':run.name,'sensor':kind,'subset':subset,'axis':j,'n':int(sum(valid)),'rmse':metrics['axis_rmse'][j],'mean':metrics['axis_mean'][j],'p95_abs':metrics['axis_p95_abs'][j]})
        if kind=='gnss':
            info['gnss_quality']={str(int(k)):{'n':int(sum(good&(d['fix']==k))),
                'position_rmse':float(np.sqrt(np.mean(np.sum((pred[good&(d['fix']==k),:3]-obs[good&(d['fix']==k),:3])**2,axis=1)))),
                'velocity_rmse':float(np.sqrt(np.mean(np.sum((pred[good&(d['fix']==k),3:]-obs[good&(d['fix']==k),3:])**2,axis=1))))} for k in np.unique(d['fix'][good])}
            continue
        # Only phase-valid measured support; no phase used inside optimizer.
        valid=good&(d['t']>=wt[0])&(d['t']<=wt[-1]);t=d['t'][valid];o=obs[valid];p=pred[valid];ph=np.interp(t,wt,phase)
        # Windows permit varying periodic response. Phase coefficient regression is descriptive.
        for begin,finish in sorted(set([(tr['t'][0],tr['t'][-1])]+[(tr['t'][0]+k,tr['t'][0]+k+4) for k in range(0,int(tr['t'][-1]-tr['t'][0]),4)])):
            use=(t>=begin)&(t<finish)
            if sum(use)<100:continue
            design=phase_design(t[use],ph[use]);co=np.linalg.lstsq(design,o[use],rcond=None)[0];cp=np.linalg.lstsq(design,p[use],rcond=None)[0]
            hz=float(np.median(np.interp(t[use],wt,wing['frequency'][wv])))
            for harmonic in [1,2,3]:
                ix=2+2*(harmonic-1)
                for j in range(3):
                    ao=float(np.hypot(*co[ix:ix+2,j]));ap=float(np.hypot(*cp[ix:ix+2,j]));shift=np.angle(np.exp(1j*(np.arctan2(cp[ix+1,j],cp[ix,j])-np.arctan2(co[ix+1,j],co[ix,j]))))
                    dynamic.append({'run':run.name,'sensor':kind,'axis':j,'start_s':begin,'duration_s':finish-begin,'wing_hz':hz,'harmonic':harmonic,
                       'observed_amplitude':ao,'predicted_amplitude':ap,'gain':ap/ao if ao>1e-8 else np.nan,'phase_difference_rad':shift,'equivalent_lag_ms':-shift/(2*np.pi*hz*harmonic)*1000,
                       'interpretation':'sensor-point phase association, not proof of rigid-body motion; weak harmonic phase unreliable'})
        for j in range(3):
            grid=np.arange(t[0],t[-1],.01);og=np.interp(grid,t,o[:,j]);pg=np.interp(grid,t,p[:,j]);freq,po=signal.welch(og,fs=100,nperseg=min(400,len(grid)));_,pp=signal.welch(pg,fs=100,nperseg=min(400,len(grid)))
            for f,a,b in zip(freq,po,pp):spectra.append({'run':run.name,'sensor':kind,'axis':j,'frequency_hz':f,'observed_psd':a,'predicted_psd':b})
            if plots:
                ax=axes[j,0 if kind=='accel' else 1];ax.plot(t-t[0],o[:,j],alpha=.6,lw=.7,label='calibrated observation');ax.plot(t-t[0],p[:,j],lw=.9,label='GP sensor prediction');ax.set_ylabel(f'{kind} {"xyz"[j]}');ax.grid(alpha=.2)
        info['observations'][kind]['peaks']={'observed_p99_abs':np.quantile(abs(o),.99,axis=0).tolist(),'predicted_p99_abs':np.quantile(abs(p),.99,axis=0).tolist(),
           'observed_max_abs':np.max(abs(o),axis=0).tolist(),'predicted_max_abs':np.max(abs(p),axis=0).tolist()}
    if plots:
        axes[0,0].legend(fontsize=8);axes[2,0].set_xlabel('seconds');axes[2,1].set_xlabel('seconds');fig.suptitle(run.name+' | sensor predictions, not body-motion ground truth');fig.tight_layout();fig.savefig(out/(run.name+'_native.png'),dpi=140);plt.close(fig)
        fig,axs=plt.subplots(3,2,figsize=(13,8),sharex=True)
        for ax,names,title in zip(axs.flat,[['px','py','pz'],['vx','vy','vz'],['ax','ay','az'],['wx','wy','wz'],['alphax','alphay','alphaz'],None],['position NED (m)','velocity NED (m/s)','acceleration at IMU reference (m/s2)','angular rate FRD (rad/s)','angular acceleration FRD (rad/s2)','orientation rotation vector (rad)']):
            vals=matrix(tr,names) if names else r.as_rotvec();ax.plot(tr['t']-tr['t'][0],vals);ax.set_title(title);ax.grid(alpha=.2)
        fig.tight_layout();fig.savefig(out/(run.name+'_trajectory.png'),dpi=140);plt.close(fig)
        fig,axs=plt.subplots(3,2,figsize=(11,8),sharex=True)
        for j in range(3):
            for k,kind in enumerate(['accel','gyro']):
                rows=[x for x in spectra if x['sensor']==kind and x['axis']==j];ax=axs[j,k];ax.semilogy([x['frequency_hz'] for x in rows],[x['observed_psd'] for x in rows],label='observed');ax.semilogy([x['frequency_hz'] for x in rows],[x['predicted_psd'] for x in rows],label='prediction');ax.set_xlim(0,30);ax.set_ylabel(kind+' '+str(j));ax.grid(alpha=.2)
        axs[0,0].legend();fig.tight_layout();fig.savefig(out/(run.name+'_psd.png'),dpi=140);plt.close(fig)
    info['kinematics']=[{n:(str(row[n]) if n=='test' else float(row[n])) for n in row.dtype.names} for row in np.genfromtxt(run/'kinematics.csv',delimiter=',',names=True,dtype=None,encoding='utf8')]
    jm=load(run/'jacobian_monitor.csv');info['jacobian_monitor']={n:float(np.max(jm[n])) for n in ['pvt_error','imu_error','prior_error']};info['min_monitored_relative_angle']=float(np.min(jm['relative_angle']))
    # Downstream MainV2 scheme, supplied exact continuous derivatives at left endpoints.
    # No network is imported/evaluated; error solely from this discrete scheme.
    discrete=[];t=tr['t'];pos=matrix(tr,['px','py','pz']);vel=matrix(tr,['vx','vy','vz']);acc=matrix(tr,['ax','ay','az']);omega=matrix(tr,['wx','wy','wz']);alpha=matrix(tr,['alphax','alphay','alphaz'])
    for duration in [.02,.1,.2,.5,2.]:
        count=round(duration/.02);errs=[]
        for i in range(0,len(t)-4*count,40):
            x=pos[i].copy();v=vel[i].copy();w=omega[i].copy();rot=r[i]
            for j in range(count):
                k=i+4*j;h=t[k+4]-t[k]
                acc_body=r[k].inv().apply(acc[k]);acc_n=rot.apply(acc_body)
                x+=v*h+.5*acc_n*h*h;v+=acc_n*h;wn=w+alpha[k]*h;rot=rot*Rotation.from_rotvec(.5*(w+wn)*h);w=wn
            k=i+4*count;errs.append([np.linalg.norm(x-pos[k]),np.linalg.norm(v-vel[k]),np.linalg.norm((rot.inv()*r[k]).as_rotvec()),np.linalg.norm(w-omega[k])])
        a=np.array(errs);discrete.append({'duration_s':duration,'max_p_v_R_w':a.max(axis=0).tolist(),'median_p_v_R_w':np.median(a,axis=0).tolist()})
    info['downstream_left_derivative_integrator']=discrete
    # Per-time quality flags: never disguise interpolation density as measurement bandwidth.
    quality=[{'t':float(tt),'finite':int(np.isfinite(row).all()),'experimental_only':1,'inside_fixed_target':1,'rigid_mount_assumed':1,'extrinsic_delay_unknown':1,'posterior_calibrated':0,'formal_label_valid':0} for tt,row in zip(t,matrix(tr,list(tr.dtype.names)))]
    write_csv(out/(run.name+'_quality.csv'),quality)
    return info,residual_rows,dynamic,spectra


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',default='runs/stage02_20261017');ap.add_argument('--output',required=True);ap.add_argument('--runs',nargs='*');a=ap.parse_args();root=Path(a.root);out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    runs=[root/x for x in a.runs] if a.runs else sorted(p.parent for p in root.glob('*/manifest.json') if (p.parent/'trajectory.csv').exists())
    infos=[];res=[];dyn=[];spectra=[]
    for run in runs:
        info,rr,dd,ss=evaluate(run,root,out,plots=('_q1' in run.name));infos.append(info);res+=rr;dyn+=dd;spectra+=ss
    write_csv(out/'residuals.csv',res);write_csv(out/'phase_harmonics.csv',dyn);write_csv(out/'spectra.csv',spectra)
    sensitivities=[]
    primary_runs=[p for p in runs if not (root/(p.name+'_iter400')).exists()]
    for seg in ['A','B','C']:
        base=root/f'{seg}_full_dt20_q1'
        if (root/(base.name+'_iter400')).exists():base=root/(base.name+'_iter400')
        if base not in runs:continue
        b=load(base/'trajectory.csv');br=Rotation.from_quat(matrix(b,['qx','qy','qz','qw']))
        diffs=[]
        for run in primary_runs:
            if not run.name.startswith(seg+'_') or 'short' in run.name:continue
            d=load(run/'trajectory.csv');assert np.allclose(b['t'],d['t']);dr=Rotation.from_quat(matrix(d,['qx','qy','qz','qw']))
            e=np.column_stack([np.linalg.norm(matrix(d,ns)-matrix(b,ns),axis=1) for ns in [['px','py','pz'],['vx','vy','vz'],['ax','ay','az'],['wx','wy','wz'],['alphax','alphay','alphaz']]]+ [np.linalg.norm((br.inv()*dr).as_rotvec(),axis=1)])
            sensitivities.append({'run':run.name,'reference':base.name,'order':'p v a omega alpha R','rms':np.sqrt(np.mean(e**2,axis=0)).tolist(),'max':e.max(axis=0).tolist()});diffs.append(e)
        env=np.max(diffs,axis=0);np.savetxt(out/(seg+'_sensitivity_envelope.csv'),np.column_stack([b['t'],env]),delimiter=',',header='t,p,v,a,omega,alpha,R',comments='')
    (out/'summary.json').write_text(json.dumps({'runs':infos,'primary_runs':[p.name for p in primary_runs],'sensitivity':sensitivities,'scope':'experimental; held-out nearby samples correlated; heuristic sensitivity is not posterior CI'},indent=2)+'\n')
    files=[p for run in runs for p in run.glob('*.csv')]+[root/'observations/frozen_config.json',Path(__file__),Path('/home/zn/flap-system-identification/src/system_identification/models/trajectory_main_v2.py'),Path('/home/zn/flap-system-identification/src/system_identification/models/main_v2_simulator.py')]
    (out/'manifest.json').write_text(json.dumps({'inputs':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},'outputs':{str(p.name):hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file()}},indent=2)+'\n')
    print(out,len(infos),'runs')

if __name__=='__main__':main()

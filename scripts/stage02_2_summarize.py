#!/usr/bin/env python3
"""Plots, actual-cycle stability and descriptive maneuver strata; no new fits for selection."""
import csv,json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments/stage02_2'))
from phase_models import fourier,phase_at
from stage02_2_extract_phase import sha
from stage02_2_regression import load,write


def rows(p):return list(csv.DictReader(Path(p).open()))
def main():
    cfg=json.load(open('configs/stage02_2_phase_feasibility.json'));root=Path(cfg['output_root']);reg=root/'regression';out=root/'summary';out.mkdir(exist_ok=False);fig=root/'figures';fig.mkdir(exist_ok=False)
    metrics=rows(reg/'metrics.csv');harm=rows(reg/'harmonics.csv');context=[];summaries=[];peaks=[];motion_strata=[];gnss_rank=[]
    for seg in cfg['segments']:
        cycles=rows(reg/(seg+'_cycles.csv'));src=Path(cfg['source_root'])/'observations'/seg;gyro=load(src/'gyro.csv');gps=load(src/'gnss.csv');wing=load(src/'wing.csv');phi=np.unwrap(wing['phase']);s=json.load(open(src/'metadata.json'))['segment']['start_s']
        for r in cycles:
            c=int(r['cycle']);lo,hi=float(r['lo']),float(r['hi']);use=(gyro['t']>=lo)&(gyro['t']<hi);mean=np.mean(np.column_stack([gyro[k][use] for k in 'xyz']),axis=0);w_norm=np.linalg.norm(mean)
            gt=gps['t']-.11;v=np.column_stack([np.interp((lo+hi)/2,gt,gps['v'+k]) for k in 'xyz']).ravel()
            context.append(dict(segment=seg,cycle=c,mid=(lo+hi)/2,frequency=float(r['frequency']),gyro_cycle_mean_norm=w_norm,rate_group='mean_rate_lt0.3' if w_norm<.3 else 'mean_rate_ge0.3',gnss_speed=np.linalg.norm(v),gnss_up_velocity=-v[2],gnss_time_assumption_s=.11))
        # Known-phase design at existing 5Hz PVT epochs: rank is not physical observability.
        gt=gps['t']-.11;ph,ok=phase_at(gt,wing['t'],phi,wing['valid']!=0,cfg['phase_gap_limit_s']);ok&=(gt>=s)&(gt<=s+16)&(gps['valid']!=0)
        u=(gt[ok]-(s+8))/8;D=np.column_stack([np.ones(sum(ok)),u,u*u,fourier(ph[ok],3)]);sv=np.linalg.svd(D,compute_uv=False)
        gnss_rank.append(dict(segment=seg,n=int(sum(ok)),columns=D.shape[1],rank=int(np.linalg.matrix_rank(D)),condition=float(sv[0]/sv[-1]),interpretation='Algebraic basis only; low sample rate alone does not make a known-phase model unidentifiable. No PVT amplitude fit or delay inference.'))
        for sensor in ['accel','gyro']:
            for split in cfg['splits']:
                for model in ['global_mean_trend','phase_bin_mean','fourier_global_h3','fourier_local_affine_h3','fixed_frequency_local_affine_h3','cycle_scrambled_phase_local_affine_h3']:
                    d=load(reg/('_'.join([seg,split,sensor,model])+'.csv'))
                    for j,k in enumerate('xyz'):
                        o=d['obs_'+k];p=d['pred_'+k];peaks.append(dict(segment=seg,split=split,sensor=sensor,model=model,axis=j,observed_abs_p95=np.quantile(abs(o),.95),predicted_abs_p95=np.quantile(abs(p),.95),observed_abs_max=max(abs(o)),predicted_abs_max=max(abs(p))))
                    for group in ['mean_rate_lt0.3','mean_rate_ge0.3']:
                        ids=[r['cycle'] for r in context if r['segment']==seg and r['rate_group']==group];use=np.isin(d['cycle'],ids)
                        if sum(use):motion_strata.append(dict(segment=seg,split=split,sensor=sensor,model=model,group=group,n=int(sum(use)),**{f'rmse_{k}':float(np.sqrt(np.mean((d['pred_'+k][use]-d['obs_'+k][use])**2))) for k in 'xyz'}))
                    for axis in range(3):
                        for h in [1,2,3]:
                            a=[r for r in harm if r['segment']==seg and r['sensor']==sensor and r['split']==split and r['model']==model and r['axis']==str(axis) and r['harmonic']==str(h)]
                            if not a:continue
                            angle=np.array([float(r['observed_phase_rad']) for r in a]);phase_err=np.array([float(r['phase_error_rad']) for r in a]);amp=np.array([float(r['observed_amplitude']) for r in a])
                            summaries.append(dict(segment=seg,split=split,sensor=sensor,model=model,axis=axis,harmonic=h,cycles=len(a),observed_amplitude_median=np.median(amp),observed_amplitude_p10=np.quantile(amp,.1),observed_amplitude_p90=np.quantile(amp,.9),observed_phase_resultant=abs(np.mean(np.exp(1j*angle))),observed_phase_mean=float(np.angle(np.mean(np.exp(1j*angle)))),median_gain=np.median([float(r['gain']) for r in a]),median_abs_phase_error_rad=np.median(abs(phase_err)),median_equivalent_lag_ms=np.median([float(r['phase_lag_ms']) for r in a])))
    write(out/'cycle_motion_context.csv',context);write(out/'maneuver_strata.csv',motion_strata);write(out/'harmonic_summary.csv',summaries);write(out/'peaks.csv',peaks);write(out/'gnss_phase_design.csv',gnss_rank)
    split='complete_cycle_mod5_eq2';selected=['global_mean_trend','native_linear_interpolation','phase_bin_mean','fourier_global_h3','fourier_local_affine_h3','fixed_frequency_local_affine_h3','cycle_scrambled_phase_local_affine_h3'];labels=['trend','time interp','phase bins','Fourier fixed','Fourier local','clock phase','scrambled']
    f,axs=plt.subplots(2,2,figsize=(13,7))
    for col,seg in enumerate('AB'):
        for row,sensor in enumerate(['accel','gyro']):
            ax=axs[row,col];sub={r['model']:r for r in metrics if r['segment']==seg and r['sensor']==sensor and r['split']==split};x=np.arange(len(selected));
            for j,k in enumerate('xyz'):ax.bar(x+(j-1)*.25,[float(sub[m]['rmse_'+k]) for m in selected],width=.25,label=k)
            ax.set_xticks(x,labels,rotation=20,ha='right');ax.set_title(seg+' '+sensor+' whole-cycle holdout');ax.set_ylabel('m/s²' if sensor=='accel' else 'rad/s');ax.legend()
    f.tight_layout();f.savefig(fig/'cycle_holdout_errors.png',dpi=150);plt.close(f)
    f,axs=plt.subplots(2,2,figsize=(13,7))
    for col,seg in enumerate('AB'):
        for row,(sensor,k) in enumerate([('accel','z'),('gyro','y')]):
            ax=axs[row,col]
            for model,color in zip(['fourier_global_h3','fourier_local_affine_h3','fixed_frequency_local_affine_h3'],['#247ba0','#f25f5c','#7a5195']):
                d=load(reg/('_'.join([seg,split,sensor,model])+'.csv'));cy=np.unique(d['cycle'])[:3];use=np.isin(d['cycle'],cy)
                if model=='fourier_global_h3':ax.scatter(d['phase'][use],d['obs_'+k][use],s=9,alpha=.6,label='observed: first 3 held cycles')
                for c in cy:
                    u=d['cycle']==c;ix=np.argsort(d['phase'][u]);ax.plot(d['phase'][u][ix],d['pred_'+k][u][ix],lw=1,alpha=.7,color=color,label=model if c==cy[0] else None)
            ax.set_title(seg+' '+sensor+'-'+k);ax.set_ylabel('m/s²' if sensor=='accel' else 'rad/s');ax.set_xlabel('Measured mechanical phase (rad)');ax.legend(fontsize=6)
    f.tight_layout();f.savefig(fig/'held_phase_patterns.png',dpi=150);plt.close(f)
    f,axs=plt.subplots(2,2,figsize=(11,7))
    for col,seg in enumerate('AB'):
        for row,(sensor,axis) in enumerate([('accel',2),('gyro',1)]):
            ax=axs[row,col]
            for model in ['fourier_global_h3','fourier_local_affine_h3']:
                sub=[r for r in summaries if r['segment']==seg and r['sensor']==sensor and r['axis']==axis and r['split']==split and r['model']==model]
                ax.plot([r['harmonic'] for r in sub],[r['median_gain'] for r in sub],'o-',label=model)
            ax.axhline(1,color='k',ls=':');ax.set_xticks([1,2,3]);ax.set_title(seg+' '+sensor+' harmonic gain, held cycles');ax.set_xlabel('Mechanical-phase harmonic order (not FFT of sparse samples)');ax.legend(fontsize=7)
    f.tight_layout();f.savefig(fig/'harmonic_retention.png',dpi=150);plt.close(f)
    f,axs=plt.subplots(2,2,figsize=(12,7))
    for col,seg in enumerate('AB'):
        d=load(reg/(seg+'_phase_steps.csv'));t=d['t']-d['t'][0];u=t<2
        for key in ['phase_rate_hz','count_rate_hz','logged_frequency_hz']:axs[0,col].plot(t[u],d[key][u],label=key,lw=1)
        axs[0,col].set_title(seg+' first 2 s: phase derivative / encoder / filtered f');axs[0,col].set_xlabel('Time (s)');axs[0,col].set_ylabel('Hz');axs[0,col].legend(fontsize=7)
        axs[1,col].plot(t,d['phase_step_correction_rad'],lw=.7);axs[1,col].set_title(seg+' phase step minus encoder count increment');axs[1,col].set_xlabel('Time (s)');axs[1,col].set_ylabel('rad')
    f.tight_layout();f.savefig(fig/'phase_time_quality.png',dpi=150);plt.close(f)
    # Native all-sample observed spectra are reused from Stage02.1; no FFT of disjoint held cycles.
    source=Path('runs/stage02_1_20261017/existing_residuals/native_spectra.csv');spec=rows(source)
    f,axs=plt.subplots(1,2,figsize=(11,4))
    for ax,seg in zip(axs,'AB'):
        for axis in ['0','1','2']:
            a=[r for r in spec if r['run']==seg+'_full_dt20_q1' and r['axis']==axis and r['signal']=='observed'];ax.semilogy([float(r['frequency_hz']) for r in a],[float(r['psd']) for r in a],label='accel '+axis)
        ax.set_xlim(0,45);ax.set_title(seg+' reused full-native observed spectrum');ax.set_xlabel('Hz');ax.set_ylabel('(m/s²)²/Hz');ax.legend()
    f.tight_layout();f.savefig(fig/'native_observation_spectra.png',dpi=150);plt.close(f)
    # Independent synthetic observability counterexample.
    d=load(root/'synthetic/ambiguity_curve.csv');f,axs=plt.subplots(2,1,figsize=(10,5),sharex=True);u=d['t']<1
    axs[0].plot(d['t'][u],d['delta_p'][u]*1000,label='rigid-periodic minus no-motion position');axs[0].scatter(np.arange(0,1,.2),np.zeros(5),label='5Hz PVT epochs');axs[0].set_ylabel('mm');axs[0].legend()
    axs[1].plot(d['t'][u],d['delta_a'][u],label='same IMU signal: motion OR nuisance');axs[1].set_xlabel('Time (s)');axs[1].set_ylabel('m/s²');axs[1].legend();f.tight_layout();f.savefig(fig/'ambiguity_counterexample.png',dpi=150);plt.close(f)
    (out/'manifest.json').write_text(json.dumps({'script_sha256':sha(__file__),'reused_spectra_sha256':sha(source),'no_real_trajectory_optimization':True,'strata':'Descriptive; gyro mean uses observations for grouping only, never coefficient fitting or sample selection','outputs':{str(p):sha(p) for d in [out,fig] for p in d.glob('*') if p.is_file()}},indent=2)+'\n')
    print('Summary/figures complete. GNSS known-phase design:',gnss_rank)

if __name__=='__main__':main()

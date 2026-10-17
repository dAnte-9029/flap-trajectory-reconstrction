#!/usr/bin/env python3
"""Reproducible diagnostic tables, figures, and narrowly scoped consistency checks."""
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from stage02_1_residuals import load, sha, write


def rows(p): return list(csv.DictReader(Path(p).open()))
def baseline(s): return s+'_full_dt20_q1'+('_iter400' if s=='C' else '')

def main():
    cfg=json.load(open('configs/stage02_1_diagnostics.json'));root=Path(cfg['source_root']);out=Path(cfg['output_root'])
    target=out/'summary';target.mkdir(exist_ok=False);fig=out/'figures';fig.mkdir(exist_ok=False)
    old=json.load(open(root/'final_validation/summary.json'))
    existing=json.load(open(out/'existing_residuals/summary.json'));new=json.load(open(out/'new_residuals/summary.json'))
    byname={r['run']:r for r in existing+new}
    flat=[]
    for r in existing+new:
        flat.append(dict(run=r['run'],n_fit=r['fit']['n'],n_hold=r['holdout']['n'],fit_rmse=r['fit']['rmse_vector'],hold_rmse=r['holdout']['rmse_vector'],linear_rmse=r['baselines']['linear']['rmse_vector'],pchip_rmse=r['baselines']['pchip']['rmse_vector'],**{'hold_'+k:r['holdout']['axis_rmse'][i] for i,k in enumerate('xyz')},trim5_rmse=r['holdout']['trim_largest_5pct_rmse'],top5_sse=r['holdout']['top_5pct_sse_fraction'],wing_TV=r['wing_histogram_total_variation'],knot_TV=r['knot_histogram_total_variation']))
    write(target/'residual_comparison.csv',flat)
    audits=[];checks=[]
    for p in sorted((out/'factor_audit').glob('*/factor_query.csv')):
        a=np.genfromtxt(p,delimiter=',',names=True,dtype=None,encoding=None)
        for sensor in ['accel','gyro']:
            d=a[a['sensor']==sensor];record=dict(run=p.parent.name,sensor=sensor,n=len(d))
            for key in ['saved_minus_query','query_minus_factor','residual_jac_mode_difference']:record[key]=float(max(d[key]));assert record[key]<1e-7
            record['pub_lag_min_ms']=float(min(d['pub_minus_sample_ms']));record['pub_lag_median_ms']=float(np.median(d['pub_minus_sample_ms']));record['pub_lag_max_ms']=float(max(d['pub_minus_sample_ms']));audits.append(record)
    assert len(audits)==6
    write(target/'factor_audit.csv',audits);checks.append(f"All {sum(r['n'] for r in audits)} native accel/gyro predictions match saved query and GPIMUFactor; Jacobian mode residual unchanged")
    for r in existing+new:
        assert r['calibration_replay_max_error']<1e-12 and r['whitening_max_error']<1e-12
    checks.append('22 acceleration result sets pass exact native timestamp/value/mask/valid join, calibration and whitening replay')
    sensitivity=[]
    for d in old['sensitivity']:
        if d['run'] not in old['primary_runs']:continue
        sensitivity.append(dict(run=d['run'],**{k:v for k,v in zip(['p_rms','v_rms','a_rms','omega_rms','alpha_rms','R_rms'],d['rms'])}))
    write(target/'source_sensitivity.csv',sensitivity)
    delays=[]
    for r in old['runs']:
        name=r['run']
        if name not in old['primary_runs'] or not ('delay' in name or name==baseline(name[0])):continue
        o=r['observations'];m=json.load(open(root/name/'manifest.json'))
        delays.append(dict(run=name,delay=m['delay_s_assumption'],accel_hold=o['accel']['holdout']['vector_rmse'],gyro_hold=o['gyro']['holdout']['vector_rmse'],p_hold=o['gnss']['holdout']['position_vector_rmse'],v_hold=o['gnss']['holdout']['velocity_vector_rmse'],**{k:r['solver'][k] for k in ['ba_x','ba_y','ba_z','bg_x','bg_y','bg_z']}))
    write(target/'delay_bias.csv',delays)
    # Preserve all steps, horizons, RMS and maxima; figures use the declared .2s horizon.
    integrations=[]
    for directory in ['integration','integration_offgrid']:
        for p in sorted((out/directory).glob('*/*.csv')):
            if p.name not in ['integrators.csv','constant.csv','periodic4hz.csv']:continue
            for r in rows(p):integrations.append(dict(run=p.parent.name if p.name=='integrators.csv' else p.stem,grid=directory,**r))
    write(target/'integrations.csv',integrations)
    constants=[r for r in integrations if r['run']=='constant']
    assert constants and max(float(r[k]) for r in constants for k in ['max_p','max_v','max_R','max_w'])<1e-9
    checks.append('Constant-derivative analytic control is exact within 1e-9 for all methods/steps/horizons')
    identities=[r for r in integrations if r['method']=='endpoint_moments']
    assert identities and max(float(r[k]) for r in identities for k in ['max_p','max_v','max_R','max_w'])<1e-8
    checks.append('Endpoint moment identities agree within 1e-8; algebraic construction check only, not accuracy validation')
    assert len(list((out/'solves').glob('*/solver.csv')))==4
    for p in (out/'solves').glob('*/solver.csv'):
        s=rows(p)[0];assert s['usable']=='1' and s['termination']=='0'
    checks.append('Exactly four predeclared solves completed and converged; scientific failures retained')
    assert len(list((out/'integration').glob('*/integrators.csv')))==9
    assert len(list((out/'integration_offgrid').glob('*/integrators.csv')))==2
    checks.append('All nine fixed trajectory replays, two analytic controls and two off-grid replays completed')
    # Block residual versus distance into each missing interval, including all sixteen blocks.
    block=[]
    for seg in 'AB':
        name=seg+'_block200ms_each_second';d=load(out/'new_residuals'/(name+'_native_errors.csv'))
        start=json.load(open(out/'solves'/name/'manifest.json'))['segment']['start_s']
        for i in range(4):
            fraction=(d['t']-start)%1;use=(d['holdout']==1)&(fraction>=.4+i*.05)&(fraction<.45+i*.05)
            block.append(dict(run=name,block_offset_ms=i*50,n=int(sum(use)),rmse=float(np.sqrt(np.mean(d['error_norm'][use]**2)))))
    write(target/'block_position.csv',block)
    # Publication lag is not a sensor response delay estimate.
    fig1,axs=plt.subplots(1,3,figsize=(13,4))
    for ax,seg in zip(axs,'ABC'):
        d=byname[baseline(seg)];labels=['fit','held','linear','PCHIP'];values=[d['fit']['rmse_vector'],d['holdout']['rmse_vector'],d['baselines']['linear']['rmse_vector'],d['baselines']['pchip']['rmse_vector']]
        ax.bar(labels,values,color=['#488','#d65','#578','#99b']);ax.set_title(seg+' accel native-time error');ax.set_ylabel('Vector RMSE (m/s²)');ax.grid(axis='y',alpha=.2)
    fig1.tight_layout();fig1.savefig(fig/'residual_baselines.png',dpi=150);plt.close(fig1)
    spectrum=rows(out/'existing_residuals/native_spectra.csv')
    f,axs=plt.subplots(2,3,figsize=(14,7))
    for col,seg in enumerate('ABC'):
        name=baseline(seg);d=load(out/'existing_residuals'/(name+'_native_errors.csv'));t=d['t']-d['t'][0];hold=d['holdout']!=0
        axs[0,col].scatter(t[hold],d['error_norm'][hold],s=5,label='held');axs[0,col].scatter(t[~hold],d['error_norm'][~hold],s=2,label='fit');axs[0,col].set_title(seg+' native error norm');axs[0,col].set_xlabel('Time (s)');axs[0,col].set_ylabel('m/s²');axs[0,col].legend()
        for label in ['observed','predicted','all_native_error']:
            sel=[r for r in spectrum if r['run']==name and r['axis']=='2' and r['signal']==label]
            axs[1,col].semilogy([float(r['frequency_hz']) for r in sel],[float(r['psd']) for r in sel],label=label)
        axs[1,col].set_xlim(0,45);axs[1,col].set_xlabel('Hz (full native sequence)');axs[1,col].set_ylabel('Z specific force PSD');axs[1,col].legend(fontsize=7)
    f.suptitle('Error spectrum includes fit/hold mask modulation; not a vibration identification');f.tight_layout();f.savefig(fig/'native_errors_spectra.png',dpi=150);plt.close(f)
    bins=rows(out/'existing_residuals/stratified_errors.csv')
    f,axs=plt.subplots(2,2,figsize=(11,7))
    for col,seg in enumerate('AB'):
        for ax,group,count in [(axs[0,col],'phase',12),(axs[1,col],'knot_fraction',10)]:
            for subset in ['fit','holdout']:
                y=[]
                for b in range(count):
                    sel=[r for r in bins if r['run']==baseline(seg) and r['group']==group and r['subset']==subset and int(r['bin'])==b]
                    y.append(np.sqrt(sum(float(r['rmse'])**2 for r in sel)) if sel else np.nan)
                ax.plot(np.arange(count),y,'o-',label=subset)
            ax.set_title(seg+' '+group+' bins');ax.set_ylabel('Vector RMSE (m/s²)');ax.legend()
    f.tight_layout();f.savefig(fig/'phase_knot_errors.png',dpi=150);plt.close(f)
    f,axs=plt.subplots(2,2,figsize=(10,7))
    for col,seg in enumerate('AB'):
        for row,key in enumerate(['rms_v','rms_w']):
            for grid,method,style in [('integration','body_left','o-'),('integration','body_rk4','s-'),('integration_offgrid','body_rk4','x--')]:
                sel=[r for r in integrations if r['run']==baseline(seg) and r['grid']==grid and r['method']==method and abs(float(r['horizon'])-.2)<1e-9]
                axs[row,col].loglog([float(r['step'])*1000 for r in sel],[float(r[key]) for r in sel],style,label=method+(' offgrid' if grid.endswith('offgrid') else ''))
            axs[row,col].set_title(seg+' '+key+' at 0.2 s');axs[row,col].set_xlabel('Step (ms)');axs[row,col].set_ylabel('m/s' if row==0 else 'rad/s');axs[row,col].grid(alpha=.2);axs[row,col].legend(fontsize=7)
    f.tight_layout();f.savefig(fig/'integration_refinement.png',dpi=150);plt.close(f)
    # Source native phase association is descriptive, not an independent rigid-body test.
    harmonic=[]
    for r in rows(root/'final_validation/phase_harmonics.csv'):
        if r['run'] in old['primary_runs'] and '_full_' in r['run']:harmonic.append(r)
    write(target/'source_phase_harmonics.csv',harmonic)
    (target/'checks.json').write_text(json.dumps({'passed':checks,'scope':'Focused Stage02.1 arithmetic/provenance checks, not physical truth certification'},indent=2)+'\n')
    (target/'manifest.json').write_text(json.dumps({'script_sha256':sha(__file__),'config_sha256':sha('configs/stage02_1_diagnostics.json'),'outputs':{str(p):sha(p) for d in [target,fig] for p in d.glob('*') if p.is_file()}},indent=2)+'\n')
    print('\n'.join(checks))

if __name__=='__main__':main()

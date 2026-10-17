#!/usr/bin/env python3
"""Stage02.1 existing-result attribution, never read raw logs or fit a trajectory."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy import signal


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load(p):return np.genfromtxt(p,delimiter=',',names=True)
def mat(d,names):return np.column_stack([d[n] for n in names])
def write(path,rows):
    if rows:
        with path.open('w') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def metrics(e):
    sq=np.sum(e**2,axis=1);v=np.linalg.norm(e,axis=1)
    return {'n':len(e),'rmse_vector':float(np.sqrt(np.mean(sq))),'axis_rmse':np.sqrt(np.mean(e**2,axis=0)).tolist(),
            'axis_bias':np.mean(e,axis=0).tolist(),'norm_quantiles':np.quantile(v,[.5,.9,.95,.99,1]).tolist(),
            'top_1pct_sse_fraction':float(np.sort(sq)[-max(1,int(np.ceil(.01*len(sq)))):].sum()/sq.sum()),
            'top_5pct_sse_fraction':float(np.sort(sq)[-max(1,int(np.ceil(.05*len(sq)))):].sum()/sq.sum()),
            'trim_largest_5pct_rmse':float(np.sqrt(np.mean(np.sort(sq)[:int(.95*len(sq))])))}

def inspect(root,run,out,obsdir=None):
    manifest=json.load(open(run/'manifest.json'));seg=manifest['segment']['id'][0];start=manifest['segment']['start_s'];dt=manifest['dt_s']
    obsdir=obsdir or root/'observations'/seg
    obs=load(obsdir/'accel.csv');d=load(run/'accel_predictions.csv');idx=np.searchsorted(obs['t'],d['t']);o=mat(d,['obs0','obs1','obs2']);p=mat(d,['pred0','pred1','pred2']);e=p-o
    assert np.array_equal(obs['t'][idx],d['t']);assert np.array_equal(mat(obs,['x','y','z'])[idx],o)
    assert np.array_equal(obs['holdout'][idx],d['holdout']) and np.array_equal(obs['valid'][idx],d['valid'])
    assert np.all(d['valid']==1)
    meta=json.load(open(obsdir/'metadata.json'));cal=meta['calibration']['accel'];raw=mat(obs,['raw_x','raw_y','raw_z'])
    replay=((raw-np.array(cal['offset']))*np.array(cal['scale']))@np.array(cal['rotation']).T
    calibration_error=float(np.max(abs(replay-mat(obs,['x','y','z']))));assert calibration_error<1e-12
    weights=mat(d,['white0','white1','white2']);white_error=float(np.max(abs(weights-e/1.5)));assert white_error<1e-12
    hold=d['holdout']!=0;wing=load(root/'observations'/seg/'wing.csv');assert np.all(wing['valid'])
    ph=np.interp(d['t'],wing['t'],np.unwrap(wing['phase']));frac=((d['t']-(start-2))/dt)%1
    result={'run':run.name,'alignment':'exact native timestamp/value/valid/holdout join passed','calibration_replay_max_error':calibration_error,'whitening_max_error':white_error,
      'fit':metrics(e[~hold]),'holdout':metrics(e[hold]),'native_rate_hz':float(1/np.median(np.diff(d['t']))),'holdout_rate_hz':float(1/np.median(np.diff(d['t'][hold]))),'baselines':{}}
    train=(obs['valid']!=0)&(obs['holdout']==0);ttrain=obs['t'][train];ytrain=mat(obs,['x','y','z'])[train]
    assert ttrain[0]<d['t'][0] and ttrain[-1]>d['t'][-1]
    for kind in ['linear','pchip']:
        pred=np.column_stack([np.interp(d['t'],ttrain,ytrain[:,j]) for j in range(3)]) if kind=='linear' else PchipInterpolator(ttrain,ytrain,axis=0)(d['t'])
        err=pred-o;result['baselines'][kind]=metrics(err[hold])
    # A trained known-phase template is diagnostic only; not an alternative GP solver.
    phtrain=np.interp(ttrain,wing['t'],np.unwrap(wing['phase']))
    design=lambda t,phase:np.column_stack([np.ones(len(t)),t-start]+[f(k*phase) for k in [1,2,3] for f in [np.sin,np.cos]])
    c=np.linalg.lstsq(design(ttrain,phtrain),ytrain,rcond=None)[0];pred=design(d['t'],ph)@c
    result['baselines']['train_phase_template']=metrics((pred-o)[hold])
    bins=[];native=[]
    for label,bindex,number in [('second',np.floor(d['t']-start).astype(int),16),('phase',np.floor(ph%(2*np.pi)/(2*np.pi)*12).astype(int),12),('knot_fraction',np.minimum((frac*10).astype(int),9),10)]:
        for subset,mask in [('fit',~hold),('holdout',hold)]:
            total=np.sum(e[mask]**2)
            for b in range(number):
                use=mask&(bindex==b)
                if not sum(use):continue
                for j in range(3):bins.append({'run':run.name,'group':label,'bin':b,'subset':subset,'axis':j,'n':int(sum(use)),
                  'rmse':float(np.sqrt(np.mean(e[use,j]**2))),'bias':float(np.mean(e[use,j])),'sse_fraction_of_subset_allaxes':float(np.sum(e[use,j]**2)/total)})
    for label,angle in [('wing',ph),('knot',frac*2*np.pi)]:
        result[label+'_circular_moments']={s:[float(abs(np.mean(np.exp(1j*k*angle[m])))) for k in [1,2,3]] for s,m in [('fit',~hold),('holdout',hold)]}
        counts=[np.histogram(angle[m]%(2*np.pi),bins=12,range=(0,2*np.pi))[0] for m in [~hold,hold]]
        result[label+'_histogram_total_variation']=float(.5*np.sum(abs(counts[0]/sum(counts[0])-counts[1]/sum(counts[1]))))
    result['knot_distance']= {s: {'distance_ms_quantiles':np.quantile(np.minimum(frac[m],1-frac[m])*dt*1000,[0,.25,.5,.75,1]).tolist(),
      'norm_error_correlation':float(np.corrcoef(np.minimum(frac[m],1-frac[m]),np.linalg.norm(e[m],axis=1))[0,1])} for s,m in [('fit',~hold),('holdout',hold)]}
    # Full ~100 Hz native sequence, never a Welch PSD of heldout or zero-filled masks.
    grid=np.arange(d['t'][0],d['t'][-1],.01);spectra=[];bands=[];reg=[]
    for j in range(3):
        signals={'observed':o[:,j],'predicted':p[:,j],'all_native_error':e[:,j]}
        for label,y in signals.items():
            ygrid=np.interp(grid,d['t'],y);freq,power=signal.welch(ygrid,fs=100,nperseg=400)
            for f,pow in zip(freq,power):spectra.append({'run':run.name,'axis':j,'signal':label,'frequency_hz':f,'psd':pow})
            total=np.trapz(power,freq)
            for lo,hi in [[0,2],[2,6],[6,10],[10,16],[16,30],[30,45]]:
                use=(freq>=lo)&(freq<=hi);energy=np.trapz(power[use],freq[use]);bands.append({'run':run.name,'axis':j,'signal':label,'low_hz':lo,'high_hz':hi,'power':float(energy),'fraction_total':float(energy/total)})
        for subset,m in [('all',np.ones(len(d),bool)),('fit',~hold),('holdout',hold)]:
            X=design(d['t'][m],ph[m]);co=np.linalg.lstsq(X,o[m,j],rcond=None)[0];cp=np.linalg.lstsq(X,p[m,j],rcond=None)[0];ce=np.linalg.lstsq(X,e[m,j],rcond=None)[0]
            result.setdefault('phase_error_explained',{})[subset+'_'+str(j)]=float(1-np.mean((e[m,j]-X@ce)**2)/np.mean(e[m,j]**2))
            for h in [1,2,3]:
                k=2+2*(h-1);ao=np.hypot(*co[k:k+2]);ap=np.hypot(*cp[k:k+2]);delta=np.angle(np.exp(1j*(np.arctan2(cp[k+1],cp[k])-np.arctan2(co[k+1],co[k]))))
                reg.append({'run':run.name,'axis':j,'subset':subset,'harmonic':h,'observed_amplitude':ao,'predicted_amplitude':ap,'error_amplitude':np.hypot(*ce[k:k+2]),'gain':ap/ao,'phase_difference_rad':delta,'design_condition':np.linalg.cond(X),
                   'interpretation':'Known-phase parametric regression; sparse holdout harmonics are alias-ambiguous, not an independent spectrum'})
    for i in range(len(d)):
        native.append(dict(t=float(d['t'][i]),holdout=int(hold[i]),wing_phase=float(ph[i]%(2*np.pi)),knot_fraction=float(frac[i]),
          error_x=e[i,0],error_y=e[i,1],error_z=e[i,2],error_norm=float(np.linalg.norm(e[i]))))
    write(out/(run.name+'_native_errors.csv'),native)
    return result,bins,spectra,bands,reg


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',default='configs/stage02_1_diagnostics.json');ap.add_argument('--new',action='store_true');a=ap.parse_args();cfg=json.load(open(a.config));root=Path(cfg['source_root']);out=Path(cfg['output_root'])/('new_residuals' if a.new else 'existing_residuals');out.mkdir(exist_ok=False)
    summary=json.load(open(root/'final_validation/summary.json'));runs=[root/n for n in summary['primary_runs'] if '_full_' in n]
    if a.new:runs=sorted((Path(cfg['output_root'])/'solves').glob('*'))
    results=[];bins=[];spectra=[];bands=[];phase=[];inputs={}
    for run in runs:
        ob=Path(cfg['output_root'])/'observations'/run.name if a.new else None
        rr,bb,ss,dd,pp=inspect(root,run,out,ob);results.append(rr);bins+=bb;spectra+=ss;bands+=dd;phase+=pp
        for p in [run/'manifest.json',run/'accel_predictions.csv'] :inputs[str(p)]=sha(p)
    write(out/'stratified_errors.csv',bins);write(out/'native_spectra.csv',spectra);write(out/'band_powers.csv',bands);write(out/'phase_regression.csv',phase)
    (out/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
    (out/'manifest.json').write_text(json.dumps({'inputs':inputs,'script_sha256':sha(__file__),'config_sha256':sha(a.config),'outputs':{p.name:sha(p) for p in out.iterdir() if p.is_file()}},indent=2)+'\n')
    for r in results:print(r['run'],'fit',round(r['fit']['rmse_vector'],3),'hold',round(r['holdout']['rmse_vector'],3),'linear',round(r['baselines']['linear']['rmse_vector'],3),'pchip',round(r['baselines']['pchip']['rmse_vector'],3))

if __name__=='__main__':main()

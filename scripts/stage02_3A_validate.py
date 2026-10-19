#!/usr/bin/env python3
"""Synthetic truth checks and fixed Stage02.2 H3 cycle protocol comparison."""
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.signal import lombscargle

from stage02_3A_reconstruct import ContinuousPhase, load_data, dump, rms, sha
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'experiments/stage02_2'))
from phase_models import phase_at, fit_predict, fourier


def synthetic(cfg, out):
    rng=np.random.default_rng(23019)
    t=np.arange(2000)*.01+rng.uniform(-20e-6,20e-6,2000)
    t-=t[0]
    step=2*np.pi/(4096*7.909090995788574)
    def truth(x,k):
        w1,w3=2*np.pi*4,2*np.pi*12
        if k==0:return 2*np.pi*(4*x+.01*x*x)+.02*np.sin(w1*x)+.003*np.sin(w3*x)
        if k==1:return 2*np.pi*(4+.02*x)+.02*w1*np.cos(w1*x)+.003*w3*np.cos(w3*x)
        return np.full_like(x,2*np.pi*.02)-.02*w1*w1*np.sin(w1*x)-.003*w3*w3*np.sin(w3*x)
    y=np.round(truth(t,0)/step)*step
    hall=truth(t,0)+np.deg2rad(12)*((t>=8)&(t<9))
    records=[];mask=(t>2)&(t<18)
    for scale in cfg['cutoff_scale_hz']:
        m=ContinuousPhase.fit(t,y,scale,[2,18])
        values=[m.query(t[mask],k)-truth(t[mask],k) for k in (0,1,2)]
        records.append(dict(scale_hz=scale,phase_rmse_rad=rms(values[0]),velocity_rmse_rad_s=rms(values[1]),
            acceleration_rmse_rad_s2=rms(values[2]),minimum_velocity=m.minimum_velocity()))
        pd.DataFrame(dict(t=t,truth_phase=truth(t,0),observed_count_phase=y,hall_with_steps=hall,
            fitted_phase=m.query(t),truth_velocity=truth(t,1),fitted_velocity=m.query(t,1),
            truth_acceleration=truth(t,2),fitted_acceleration=m.query(t,2))).to_csv(out/f'synthetic_{scale}.csv',index=False)
    m=ContinuousPhase.fit(t,y,32,[2,18])
    assert records[-1]['phase_rmse_rad']<5e-4 and records[-1]['velocity_rmse_rad_s']<.03
    assert records[-1]['acceleration_rmse_rad_s2']<3
    assert m.minimum_velocity()>0
    # A constant gauge change must not alter either derivative.
    alt=ContinuousPhase.fit(t,y+.2,32,[2,18])
    assert max(abs(m.query(t,1)-alt.query(t,1)))<1e-7
    assert max(abs(m.query(t,2)-alt.query(t,2)))<1e-5
    rejected=[]
    for name,tt,yy in [('gap',np.r_[t[:50],t[60:]],np.r_[y[:50],y[60:]]),
                       ('count_reset',t,np.r_[y[:100],y[100:]-10]),
                       ('invalid',t,np.r_[y[:100],np.nan,y[101:]]),
                       ('duplicate_time',np.r_[t[:100],t[99],t[101:]],y)]:
        try:ContinuousPhase.fit(tt,yy,32,[2,18])
        except ValueError:rejected.append(name)
    assert len(rejected)==4
    try:m.query(-1)
    except ValueError:rejected.append('extrapolation')
    assert len(rejected)==5
    # Finite differences independently check the derivative query API.
    q=np.linspace(2.001234,17.99973,300);h=1e-5
    errors=[max(abs((m.query(q+h,k)-m.query(q-h,k))/(2*h)-m.query(q,k+1))) for k in (0,1)]
    assert errors[0]<1e-5 and errors[1]<1e-3
    pd.DataFrame(records).to_csv(out/'synthetic_metrics.csv',index=False)
    dump(out/'synthetic_checks.json',dict(passed=True,invalid_inputs_rejected=rejected,
         derivative_max_errors=errors,constant_gauge_invariance=True,
         hall_step_note='Encoder fit never sees the synthetic Hall step; suppression is structural, not evidence of correct physical zero',
         truth='mechanical phase only, not joint angle'))


def cycle_comparison(cfg,out):
    metrics=[];cycle_rows=[];fits={};harmonic=[]
    oldroot=Path(cfg['source'])/'regression'
    previous=json.load(open(oldroot/'fit_manifest.json'))
    for seg in cfg['segments']:
        d=load_data(cfg,seg);m=ContinuousPhase.load(out/f'{seg}_phase_model.json')
        cytable=pd.read_csv(oldroot/f'{seg}_cycles.csv')
        edges=np.r_[cytable.lo.to_numpy(),cytable.hi.iloc[-1]]
        wing=np.genfromtxt(Path(cfg['observations'])/seg/'wing.csv',delimiter=',',names=True)
        for sensor in ('accel','gyro'):
            obs=np.genfromtxt(Path(cfg['observations'])/seg/(sensor+'.csv'),delimiter=',',names=True)
            ph,ok=phase_at(obs['t'],wing['t'],np.unwrap(wing['phase']),wing['valid']!=0,.03)
            cycle=np.searchsorted(edges,obs['t'],side='right')-1
            ok&=(cycle>=0)&(cycle<len(cytable))&(obs['valid']!=0)
            t=obs['t'][ok];ph=ph[ok];cycle=cycle[ok];y=np.column_stack([obs[k][ok] for k in 'xyz'])
            for split in ('complete_cycle_mod5_eq2','last8_complete_cycles_forecast'):
                key=f'{seg}_{split}_{sensor}_fourier_global_h3'
                manifest=previous[key];train=np.isin(cycle,manifest['train_cycles']);held=np.isin(cycle,manifest['held_cycles'])
                assert not np.any(train&held)
                reference=pd.read_csv(oldroot/(key+'.csv'))
                assert np.allclose(t[held],reference.t,atol=1e-12,rtol=0)
                for name,phase in [('hall_indexed',ph),('continuous_encoder',m.query(t))]:
                    pred,detail=fit_predict(t,phase,y,cycle,train,held,3,False)
                    if name=='hall_indexed':
                        assert np.max(abs(pred-reference[['pred_x','pred_y','pred_z']].to_numpy()))<1e-8
                    err=pred-y[held];axis=np.sqrt(np.mean(err**2,axis=0))
                    metrics.append(dict(segment=seg,split=split,sensor=sensor,phase_model=name,n=sum(held),
                        rmse_vector=float(np.linalg.norm(axis)),**{f'rmse_{k}':axis[j] for j,k in enumerate('xyz')}))
                    fits[f'{key}_{name}']=detail
                    rows=dict(t=t[held],cycle=cycle[held],phase=phase[held])
                    rows.update({f'obs_{k}':y[held,j] for j,k in enumerate('xyz')})
                    rows.update({f'pred_{k}':pred[:,j] for j,k in enumerate('xyz')})
                    pd.DataFrame(rows).to_csv(out/f'{key}_{name}.csv',index=False)
                    for cy in manifest['held_cycles']:
                        ix=cycle[held]==cy;ec=err[ix]
                        cycle_rows.append(dict(segment=seg,split=split,sensor=sensor,phase_model=name,cycle=cy,
                            mid=float(cytable.mid.iloc[cy]),frequency=float(cytable.frequency.iloc[cy]),
                            rmse_vector=float(np.sqrt(np.mean(np.sum(ec**2,axis=1)))),
                            **{f'rmse_{k}':rms(ec[:,j]) for j,k in enumerate('xyz')}))
                        # Same old phase basis for both prediction curves: amplitude
                        # differences cannot be an artifact of changing evaluation coordinates.
                        F=np.column_stack([np.ones(sum(ix)),fourier(ph[held][ix],3)])
                        bo=np.linalg.lstsq(F,y[held][ix],rcond=None)[0]
                        bp=np.linalg.lstsq(F,pred[ix],rcond=None)[0]
                        for h in (1,2,3):
                            k=2*h-1
                            for j in range(3):
                                ao=np.hypot(bo[k,j],bo[k+1,j]);ap=np.hypot(bp[k,j],bp[k+1,j])
                                pe=np.angle(np.exp(1j*(np.arctan2(bp[k+1,j],bp[k,j])-np.arctan2(bo[k+1,j],bo[k,j]))))
                                harmonic.append(dict(segment=seg,split=split,sensor=sensor,phase_model=name,cycle=cy,
                                    harmonic=h,axis=j,observed_amplitude=ao,predicted_amplitude=ap,phase_error_rad=pe))
    pd.DataFrame(metrics).to_csv(out/'cycle_metrics.csv',index=False)
    pd.DataFrame(cycle_rows).to_csv(out/'per_cycle_metrics.csv',index=False)
    pd.DataFrame(harmonic).to_csv(out/'cycle_harmonics.csv',index=False)
    dump(out/'cycle_fit_manifest.json',fits)


def frequency_spectra(cfg,out):
    harmonic=[]
    for seg in cfg['segments']:
        d=load_data(cfg,seg);t=d['t'];tm=(t[:-1]+t[1:])/2;dt=np.diff(t)
        use=(tm>=d['core'][0])&(tm<=d['core'][1]);x=tm[use]
        selected=ContinuousPhase.load(out/f'{seg}_phase_model.json')
        phase=selected.query(x);basis=np.column_stack([np.ones(len(x)),x-x.mean(),fourier(phase,3)])
        spectra={};freqs=np.linspace(.5,45,1000)
        signals={'encoder_interval':np.diff(d['y'])/(2*np.pi*dt)}
        for scale in cfg['cutoff_scale_hz']:
            m=ContinuousPhase.load(out/'fits'/f'{seg}_scale_{scale}.json')
            signals[f'spline_{scale}_interval']=np.diff(m.query(t))/(2*np.pi*dt)
        signals['px4_lowpass_endpoint']=d['freq'].frequency_hz.to_numpy()[1:]
        for name,v in signals.items():
            y=v[use]
            slow=np.column_stack([np.ones(len(x)),x-x.mean()]);yd=y-slow@np.linalg.lstsq(slow,y,rcond=None)[0]
            spectra[name]=lombscargle(x-x[0],yd,2*np.pi*freqs,normalize=False)*4/len(x)
            beta=np.linalg.lstsq(basis,y,rcond=None)[0]
            for h in (1,2,3):
                k=2*h
                harmonic.append(dict(segment=seg,signal=name,harmonic=h,
                    amplitude_hz=float(np.hypot(beta[k],beta[k+1])),phase_rad=float(np.arctan2(beta[k+1],beta[k]))))
        pd.DataFrame(dict(frequency_hz=freqs,**spectra)).to_csv(out/f'{seg}_frequency_spectrum.csv',index=False)
    pd.DataFrame(harmonic).to_csv(out/'frequency_harmonics.csv',index=False)


def main():
    cfg=json.load(open('configs/stage02_3A_phase.json'));out=Path(cfg['output'])
    assert not (out/'validation_manifest.json').exists()
    synthetic(cfg,out)
    cycle_comparison(cfg,out)
    frequency_spectra(cfg,out)
    dump(out/'validation_manifest.json',dict(script_sha=sha(__file__),synthetic_passed=True,
         stage02_2_old_predictions_reproduced=True,exact_same_old_cycle_memberships=True,
         no_imu_tuning=True,phase_count_observations='all available; conditional sensor prediction, not independent test'))
    print('Synthetic, cycle comparison and native-time frequency spectra completed.')


if __name__=='__main__':main()

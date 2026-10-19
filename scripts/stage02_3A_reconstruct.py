#!/usr/bin/env python3
"""Frozen encoder-only phase fitting/selection, Hall diagnostics and derivative validation."""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.integrate import quad

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'experiments/stage02_3A'))
from continuous_phase import ContinuousPhase


def sha(p):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for b in iter(lambda: f.read(1024*1024), b''):
            h.update(b)
    return h.hexdigest()


def dump(p, d):
    Path(p).write_text(json.dumps(d, indent=2, ensure_ascii=False)+'\n')


def rms(x):
    return float(np.sqrt(np.mean(np.asarray(x)**2)))


def load_data(cfg, seg):
    root = Path(cfg['source'])/'phase_raw'/seg
    meta = json.load(open(root/'metadata.json'))
    inputs = {}
    for name in ('wing_phase_0.csv', 'encoder_count_0.csv', 'flap_frequency_0.csv'):
        inputs[str(root/name)] = sha(root/name)
        assert inputs[str(root/name)] == meta['topics'][name]['sha256']
    w = pd.read_csv(root/'wing_phase_0.csv', float_precision='round_trip')
    enc = pd.read_csv(root/'encoder_count_0.csv', float_precision='round_trip')
    freq = pd.read_csv(root/'flap_frequency_0.csv', float_precision='round_trip')
    assert np.array_equal(w.timestamp, enc.timestamp) and np.array_equal(w.timestamp, freq.timestamp)
    assert np.array_equal(w.encoder_total_count, enc.total_count) and np.all(w.phase_valid == 1)
    t = w.timestamp.to_numpy()*1e-6
    c = enc.total_count.to_numpy()
    step = 2*np.pi/(4096*meta['parameters']['FLAP_RATIO'])
    y = float(w.phase_rad.iloc[0])+(c-c[0])*step
    core = [meta['segment']['start_s'], meta['segment']['end_s']]
    return dict(w=w, enc=enc, freq=freq, t=t, c=c, step=step, y=y, core=core, meta=meta, inputs=inputs)


def main():
    cfg_path = Path('configs/stage02_3A_phase.json'); cfg = json.load(open(cfg_path))
    out = Path(cfg['output']); assert json.load(open(out/'frozen_config.json')) == cfg
    fitroot = out/'fits'; fitroot.mkdir(exist_ok=False)
    data, metrics, audit = {}, [], {}
    # Only sparse count scores are inspected during this selection pass.
    for seg in cfg['segments']:
        d = load_data(cfg, seg); data[seg] = d
        t, y, c, step = [d[k] for k in ('t', 'y', 'c', 'step')]
        dt, dc = np.diff(t), np.diff(c)
        rawpos = d['enc'].position_raw.to_numpy().astype(np.int64)
        rawdiff = (np.diff(rawpos)+2048) % 4096-2048
        assert np.array_equal(rawdiff, dc)
        assert np.all(dc > 0) and max(dc) < 2048 and np.all((dt > 0) & (dt < cfg['gap_limit_s']))
        assert np.array_equal(d['w'].flap_frequency_hz, d['freq'].frequency_hz)
        fcount = dc*step/(2*np.pi*dt)
        replay = d['freq'].frequency_hz.to_numpy().copy()
        for i in range(1, len(t)):
            replay[i] = .8*replay[i-1]+.2*fcount[i-1]
        qsigma = step/np.sqrt(12)
        audit[seg] = dict(source_sha=d['meta']['source_sha256'], firmware=d['meta']['firmware'],
            rows=len(t), count_increment_min_max=[int(min(dc)), int(max(dc))],
            dt_min_median_max_s=np.quantile(dt, [0,.5,1]).tolist(),
            encoder_raw_wraps=int(sum(np.diff(rawpos)<-2048)),
            discrete_direction='positive; no observed resets/gaps or unwrap mismatch',
            hidden_alias_caveat='More than half a motor turn per unobserved interval cannot be excluded solely by a modulo sensor',
            count_step_rad=step, ideal_quantization_phase_std_rad=qsigma,
            ideal_quantization_first_difference_std_rad_s=np.sqrt(2)*qsigma/np.median(dt),
            ideal_quantization_second_difference_std_rad_s2=np.sqrt(6)*qsigma/np.median(dt)**2,
            timestamp_scenario_phase_std_rad=2*np.pi*4*cfg['perturbation_scenarios']['timestamp_uniform_half_width_s']/np.sqrt(3),
            frequency_filter_replay_rmse_hz=rms(replay-d['freq'].frequency_hz),
            frequency_filter_replay_max_abs_hz=float(max(abs(replay-d['freq'].frequency_hz))),
            original_sha_verification='CSV hashes rechecked; raw SHA checked separately in parameter audit',
            input_hashes=d['inputs'])
        sparse = np.arange(len(t)) % 5 == 2; sparse[:2] = sparse[-2:] = False
        d['sparse'] = sparse
        for scale in cfg['cutoff_scale_hz']:
            model = ContinuousPhase.fit(t, y, scale, d['core'], ~sparse)
            pred = model.query(t)
            err = (pred-y)/step
            metrics.append(dict(segment=seg, split='sparse_selection', scale_hz=scale, lambda_value=model.metadata['lambda'],
                                held_rows=int(sum(sparse)), held_rmse_count=rms(err[sparse]), fit_rmse_count=rms(err[~sparse]),
                                max_held_abs_count=float(max(abs(err[sparse])))))
            pd.DataFrame(dict(t=t, held=sparse, observed_phase=y, predicted_phase=pred, residual_count=err)).to_csv(
                fitroot/f'{seg}_sparse_{scale}.csv', index=False)
        lin = np.interp(t[sparse], t[~sparse], y[~sparse])
        metrics.append(dict(segment=seg, split='sparse_selection', scale_hz='linear', lambda_value=None,
                            held_rows=int(sum(sparse)), held_rmse_count=rms((lin-y[sparse])/step)))
    means = {s:float(np.mean([r['held_rmse_count'] for r in metrics if r['scale_hz']==s])) for s in cfg['cutoff_scale_hz']}
    selected = min(s for s,v in means.items() if v <= 1.02*min(means.values()))
    dump(out/'selection.json', dict(common_scale_hz=selected, sparse_mean_rmse_count=means,
         rule=cfg['selection'], no_block_or_imu_scores_used=True))
    print('Frozen selection:', selected, means, flush=True)

    checks, sensitivity, frequency, zero_events = [], [], [], []
    for seg, d in data.items():
        t,y,c,step = [d[k] for k in ('t','y','c','step')]; rel=t-t[0]
        dt=np.diff(t);tm=(t[1:]+t[:-1])/2
        block=np.zeros(len(t),bool)
        for center in cfg['block_centers_s_from_context_start']:
            block |= abs(rel-center)<cfg['block_width_s']/2
        pd.DataFrame(dict(t=t,sparse_held=d['sparse'],block_held=block)).to_csv(out/f'{seg}_count_masks.csv',index=False)
        models={}
        for scale in cfg['cutoff_scale_hz']:
            model=ContinuousPhase.fit(t,y,scale,d['core']);models[scale]=model
            model.save(fitroot/f'{seg}_scale_{scale}.json')
            gapmodel=ContinuousPhase.fit(t,y,scale,d['core'],~block)
            err=(gapmodel.query(t)-y)/step
            metrics.append(dict(segment=seg,split='block_validation',scale_hz=scale,
                held_rows=int(sum(block)),held_rmse_count=rms(err[block]),fit_rmse_count=rms(err[~block]),max_held_abs_count=float(max(abs(err[block])))))
            pd.DataFrame(dict(t=t,held=block,observed_phase=y,predicted_phase=gapmodel.query(t),residual_count=err)).to_csv(fitroot/f'{seg}_block_{scale}.csv',index=False)
            residual=(model.query(t)-y)/step
            metrics.append(dict(segment=seg,split='all_observation_refit',scale_hz=scale,
                held_rows=0,fit_rmse_count=rms(residual),max_fit_abs_count=float(max(abs(residual)))))
            q=np.linspace(t[0],t[-1],len(t)*5)
            vals={'t':q,'phase_rad':model.query(q),'phase_velocity_rad_s':model.query(q,1),
                  'phase_acceleration_rad_s2':model.query(q,2),'frequency_hz':model.query(q,1)/(2*np.pi),**model.flags(q)}
            pd.DataFrame(vals).to_csv(fitroot/f'{seg}_query_{scale}.csv',index=False)
            native=pd.DataFrame(dict(t=t,encoder_phase=y,hall_phase_unwrapped=np.unwrap(d['w'].phase_rad),
                fitted_phase=model.query(t),velocity=model.query(t,1),acceleration=model.query(t,2),
                residual_count=residual,logged_frequency=d['freq'].frequency_hz,**model.flags(t)))
            native.to_csv(fitroot/f'{seg}_native_{scale}.csv',index=False)
            avgfreq=np.diff(model.query(t))/(2*np.pi*dt);rawfreq=np.diff(y)/(2*np.pi*dt)
            core=(tm>=d['core'][0])&(tm<=d['core'][1])
            frequency.append(dict(segment=seg,scale_hz=scale,min_velocity_rad_s=model.minimum_velocity(),
                interval_frequency_rmse_hz=rms(avgfreq[core]-rawfreq[core]),
                interval_frequency_centered_std_gain=float(np.std(avgfreq[core])/np.std(rawfreq[core])),
                core_acceleration_rms_rad_s2=rms(model.query(tm[core],2))))
            assert model.minimum_velocity()>0
        lin=np.interp(t[block],t[~block],y[~block])
        metrics.append(dict(segment=seg,split='block_validation',scale_hz='linear',held_rows=int(sum(block)),held_rmse_count=rms((lin-y[block])/step)))
        m=models[selected];m.save(out/f'{seg}_phase_model.json')
        loaded=ContinuousPhase.load(out/f'{seg}_phase_model.json')
        assert max(abs(loaded.query(t)-m.query(t)))==0
        interior=(t>=d['core'][0])&(t<=d['core'][1])
        for scale,model in models.items():
            sensitivity.append(dict(segment=seg,kind='smoothing_scale',scenario=scale,
                phase_rms_rad=rms(model.query(t[interior])-m.query(t[interior])),
                frequency_rms_hz=rms((model.query(t[interior],1)-m.query(t[interior],1))/(2*np.pi)),
                acceleration_rms_rad_s2=rms(model.query(t[interior],2)-m.query(t[interior],2))))
        rng=np.random.default_rng(cfg['perturbation_scenarios']['seed'])
        for kind in ('quantization','timestamp'):
            for trial in range(cfg['perturbation_scenarios']['repeats']):
                tp=t.copy();yp=y.copy()
                if kind=='quantization':
                    yp += rng.uniform(-.5,.5,len(t))*step
                else:
                    tp += rng.uniform(-1,1,len(t))*cfg['perturbation_scenarios']['timestamp_uniform_half_width_s']
                pm=ContinuousPhase.fit(tp,yp,selected,d['core'])
                sensitivity.append(dict(segment=seg,kind=kind,scenario=trial,
                    phase_rms_rad=rms(pm.query(t[interior])-m.query(t[interior])),
                    frequency_rms_hz=rms((pm.query(t[interior],1)-m.query(t[interior],1))/(2*np.pi)),
                    acceleration_rms_rad_s2=rms(pm.query(t[interior],2)-m.query(t[interior],2))))
        hall=np.unwrap(d['w'].phase_rad.to_numpy());ids=np.flatnonzero(np.diff(d['w'].phase_unwrapped_rad)<-np.pi)+1
        corrections=np.diff(hall)-np.diff(y)
        for i in ids:
            zero_events.append(dict(segment=seg,interval_start=t[i-1],interval_end=t[i],
                correction_deg=np.rad2deg(corrections[i-1]),phase_reference_minus_encoder_deg=np.rad2deg(hall[i]-y[i]),
                phase_reference_minus_fit_deg=np.rad2deg(hall[i]-m.query(t[i])),
                is_large=abs(np.rad2deg(corrections[i-1]))>5))
        # Choose regimes by observed phase/encoder data, not fitted errors.
        normal=ids[np.argmin(abs(abs(corrections[ids-1])-np.median(abs(corrections[ids-1]))))]
        large=ids[np.argmax(abs(corrections[ids-1]))]
        changed=int(np.argmax(abs(np.diff(np.diff(y)/dt))))+1
        centers={'ordinary_reset':t[normal],'large_reset':t[large],
                 'max_encoder_rate_change':t[changed],'ordinary_interior':(d['core'][0]+d['core'][1])/2}
        for label,center in centers.items():
            start=max(t[0]+.01,center-.1);end=min(t[-1]-.01,center+.1)
            # Non-knot grid avoids optimistic checks only at spline knots.
            q=np.linspace(start+.000173,end-.000139,157)
            for h in (1e-3,1e-4,1e-5):
                e1=(m.query(q+h)-m.query(q-h))/(2*h)-m.query(q,1)
                e2=(m.query(q+h,1)-m.query(q-h,1))/(2*h)-m.query(q,2)
                checks.append(dict(segment=seg,regime=label,test='centered_difference',step=h,
                    velocity_max_abs=float(max(abs(e1))),acceleration_max_abs=float(max(abs(e2)))))
            for n in (20,80,320,1280):
                edges=np.linspace(start,end,n+1);mid=(edges[1:]+edges[:-1])/2
                ep=sum(m.query(mid,1)*np.diff(edges))-(m.query(end)-m.query(start))
                ev=sum(m.query(mid,2)*np.diff(edges))-(m.query(end,1)-m.query(start,1))
                checks.append(dict(segment=seg,regime=label,test='independent_midpoint',subintervals=n,
                    phase_error_rad=float(abs(ep)),velocity_error_rad_s=float(abs(ev))))
            knots=np.unique(m.spline.t)+m.origin;knots=knots[(knots>start)&(knots<end)]
            for order in (0,1):
                val,err=quad(lambda x:float(m.query(x,order+1)),start,end,points=knots,epsabs=1e-9,limit=100)
                closure=abs(val-(m.query(end,order)-m.query(start,order)))
                assert closure<1e-7
                checks.append(dict(segment=seg,regime=label,test='independent_quad',order=order,closure=float(closure),quad_error_estimate=err))
        # Cubic natural spline is C2; jerk is not required continuous.
        knots=np.unique(m.spline.t)+m.origin;knots=knots[(knots>t[0])&(knots<t[-1])]
        for eps in (1e-6,1e-8,1e-10):
            jumps=[float(max(abs(m.query(knots+eps,k)-m.query(knots-eps,k)))) for k in (0,1,2)]
            checks.append(dict(segment=seg,test='knot_limits',epsilon_s=eps,phase=jumps[0],velocity=jumps[1],acceleration=jumps[2]))
    pd.DataFrame(metrics).to_csv(out/'observation_metrics.csv',index=False)
    pd.DataFrame(checks).to_csv(out/'kinematic_checks.csv',index=False)
    pd.DataFrame(sensitivity).to_csv(out/'sensitivity.csv',index=False)
    pd.DataFrame(frequency).to_csv(out/'frequency_checks.csv',index=False)
    pd.DataFrame(zero_events).to_csv(out/'hall_diagnostics.csv',index=False)
    dump(out/'encoder_audit.json',audit)
    dump(out/'reconstruction_manifest.json',dict(config_sha=sha(cfg_path),script_sha=sha(__file__),
         module_sha=sha('experiments/stage02_3A/continuous_phase.py'),selected=selected,
         heldout_note='Same-source correlated development count holdouts; block validation was not used for selection',
         params_used=cfg))
    print('Reconstruction, holdouts, sensitivity and kinematic checks completed.',flush=True)


if __name__=='__main__':
    main()

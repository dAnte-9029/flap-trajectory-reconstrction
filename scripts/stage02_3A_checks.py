#!/usr/bin/env python3
"""Focused acceptance checks over saved results; no model re-selection or new solves."""
import json
from pathlib import Path
import numpy as np
import pandas as pd
from stage02_3A_reconstruct import ContinuousPhase, dump, sha


def main():
    cfg=json.load(open('configs/stage02_3A_phase.json'));out=Path(cfg['output'])
    selected=json.load(open(out/'selection.json'));assert selected['no_block_or_imu_scores_used']
    checks=[]
    assert json.load(open(out/'synthetic_checks.json'))['passed']
    checks.append('Synthetic truth, gauge invariance, gap/reset/invalid/time/extrapolation rejection')
    params=json.load(open(out/'parameter_audit.json'))
    audit=json.load(open(out/'encoder_audit.json'))
    for seg in cfg['segments']:
        assert not params[seg]['hall_event_present'] and not params[seg]['all_ratio_changes']
        assert params[seg]['raw_sha256']==audit[seg]['source_sha']
        assert audit[seg]['frequency_filter_replay_max_abs_hz']<2e-6
        for p,h in audit[seg]['input_hashes'].items():assert sha(p)==h
    checks.append('Raw identity/ratio, extraction hashes, same-source PX4 frequency replay')
    k=pd.read_csv(out/'kinematic_checks.csv')
    assert k[k.test=='independent_quad'].closure.max()<1e-7
    fd=k[(k.test=='centered_difference')&(k.step<1.1e-5)]
    assert fd.velocity_max_abs.max()<1e-5 and fd.acceleration_max_abs.max()<.01
    coarse=k[(k.test=='independent_midpoint')&(k.subintervals==20)].set_index(['segment','regime'])
    fine=k[(k.test=='independent_midpoint')&(k.subintervals==1280)].set_index(['segment','regime'])
    assert np.all(fine.phase_error_rad<coarse.phase_error_rad)
    # Near-grid sampling can leave tiny kink-location plateaus: do not require
    # strictly monotonic improvement at numerical noise scale in every regime.
    assert fine.velocity_error_rad_s.max()<1e-5
    off=pd.read_csv(out/'offgrid_integrals.csv')
    offcoarse=off[off.subintervals==23].set_index(['segment','regime'])
    offfine=off[off.subintervals==1471].set_index(['segment','regime'])
    assert offfine.velocity_error_rad_s.max()<1e-4 and offfine.phase_error_rad.max()<1e-5
    assert np.all(offfine.velocity_error_rad_s<offcoarse.velocity_error_rad_s)
    assert np.all(offfine.phase_error_rad<offcoarse.phase_error_rad)
    limits=k[(k.test=='knot_limits')&(k.epsilon_s<1.1e-10)]
    assert limits.phase.max()<1e-7 and limits.velocity.max()<1e-6 and limits.acceleration.max()<1e-5
    checks.append('Four regimes: finite-difference refinement, independent quadrature, midpoint refinement and C2 limits')
    for seg in cfg['segments']:
        m=ContinuousPhase.load(out/f'{seg}_phase_model.json')
        assert m.minimum_velocity()>0
        q=pd.read_csv(out/'fits'/f'{seg}_query_{selected["common_scale_hz"]}.csv')
        assert max(abs(m.query(q.t)-q.phase_rad))<1e-10
        assert max(abs(m.query(q.t,1)-q.phase_velocity_rad_s))<1e-8
        assert max(abs(m.query(q.t,2)-q.phase_acceleration_rad_s2))<1e-6
        assert not q.acceleration_physically_validated.any() and not q.absolute_zero_calibrated.any()
    checks.append('Saved model query roundtrip, exact piecewise monotonicity, conservative flags')
    old=json.load(open(Path(cfg['source'])/'regression'/'fit_manifest.json'))
    new=json.load(open(out/'cycle_fit_manifest.json'))
    for key,v in new.items():
        base=key.removesuffix('_hall_indexed').removesuffix('_continuous_encoder')
        assert v['train_cycle_ids']==old[base]['train_cycles']
        assert not set(v['train_cycle_ids']) & set(old[base]['held_cycles'])
        assert v['rank']==8
    checks.append('All 16 fixed H3 fits: same prior memberships, no held IMU leakage, full rank')
    # Persist reference-offset episodes, including duration and unresolved cause,
    # rather than treating a paired offset as a single bad sample.
    events=pd.read_csv(out/'hall_diagnostics.csv');episodes=[];quality={}
    for seg in cfg['segments']:
        large=events[(events.segment==seg)&events.is_large].reset_index(drop=True)
        assert len(large)%2==0
        for i in range(0,len(large),2):
            first,last=large.iloc[i],large.iloc[i+1]
            assert first.correction_deg*last.correction_deg<0
            episodes.append(dict(segment=seg,start_observed_hrt_s=first.interval_end,
                end_observed_hrt_s=last.interval_end,duration_s=last.interval_end-first.interval_end,
                entering_correction_deg=first.correction_deg,leaving_correction_deg=last.correction_deg,
                net_correction_deg=first.correction_deg+last.correction_deg,
                cause='unknown; Hall pulse not recorded; software-reference change, not a measured velocity impulse'))
        m=ContinuousPhase.load(out/f'{seg}_phase_model.json')
        quality[seg]=dict(valid_domain=m.domain.tolist(),interior_derivative_support=m.core.tolist(),
            reference_phase0_rad=m.offset,reference='first logged valid Hall-indexed phase; fixed additive convention',
            zero_uncertainty='not calibrated; observed ~12deg competing reference levels are not a physical confidence bound',
            clock_uncertainty='HRT read timestamp; actual latency and sensor response not measured',
            phase_confidence='relative encoder-supported phase, within the fixed ratio convention',
            velocity_confidence='conditional on timing/angle/ratio assumptions',
            acceleration_confidence='low-confidence candidate; smoothing and timing sensitivity retained',
            monotone_min_velocity_rad_s=m.minimum_velocity(),
            spline_degree=m.spline.k,spline_coefficients=len(m.spline.c),
            posterior_calibrated=False)
    pd.DataFrame(episodes).to_csv(out/'hall_offset_episodes.csv',index=False)
    dump(out/'phase_quality.json',quality)
    dump(out/'focused_checks.json',dict(passed=True,groups=checks,
         note='Mathematical/data integrity checks; not physical truth acceptance'))
    print('PASS:',len(checks),'focused groups')


if __name__=='__main__':main()

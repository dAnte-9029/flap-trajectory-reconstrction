#!/usr/bin/env python3
"""Verify Stage 00 delivery invariants and emit compact, reproducible summaries."""
from pathlib import Path
import csv
import json
import subprocess
import numpy as np
import pandas as pd
import stage00_audit as a

cfg=json.loads((a.ROOT/'configs/stage00_audit.json').read_text());root=a.ROOT/cfg['output'];run=root.parent
out=run/'validation';out.mkdir(exist_ok=False)
m=json.loads((root/'manifest.json').read_text());inv=pd.read_csv(root/'inventory.csv');t=pd.read_csv(root/'topic_coverage.csv');f=pd.read_csv(root/'field_statistics.csv');l=pd.read_csv(root/'per_log.csv');q=pd.read_csv(root/'quality.csv');d=pd.read_csv(run/'diagnostics_final/diagnostics.csv');s=pd.read_csv(run/'diagnostics_final/spectral_windows.csv')
checks={}
checks['all_discovered_september_candidates_accounted']=len(inv)==m['counts']['september_candidates']
checks['one_row_per_unique_raw_log']=l.sha256.nunique()==len(l)==m['counts']['unique_ulogs']
checks['all_raw_ulog_hashes_covered']=set(l.sha256)==set(inv[inv.kind=='ulog'].sha256)
checks['one_row_per_topic_instance']=not t.duplicated(['sha256','topic','instance']).any()
checks['all_topic_fields_accounted']=sum(len(json.loads(x)) for x in t.fields)==len(f)
checks['bounded_coverage_monotonic_streams']=bool(((t.supported_s>=0)&((t.supported_s<=t.span_s+1e-5)|(t.backwards>0))&(t.airborne_supported_s<=t.supported_s+1e-5)).all())
nonmonotonic=t[t.backwards>0].merge(l[['sha256','log_span_s']],on='sha256')
checks['nonmonotonic_support_within_log_span']=bool((nonmonotonic.supported_s<=nonmonotonic.log_span_s+1e-5).all())
checks['gap_sensitivity_monotonic']=bool(((t.supported_s_gap2<=t.supported_s_gap3+1e-6)&(t.supported_s_gap3<=t.supported_s_gap5+1e-6)).all())
checks['all_final_spectra_finite']=bool(np.isfinite(s[['flap_hz','phase_derivative_hz','phase_coherence_at_f0','flap_band_power_fraction']]).all().all())
checks['independent_spectral_pass_matches']=pd.read_csv(root/'spectral_windows.csv').equals(s)
checks['diagnostics_cover_all_logs']=set(d.sha256)==set(l.sha256)
checks['actual_interval_coverage_not_logging_span']=bool((q[q.metric.isin(['delta_angle_dt','delta_velocity_dt'])].log_fraction<.02).all())
checks['zero_raw_gps_sample_clock_handled']=bool((t[t.topic=='sensor_gps'].median_hz>0).all())
checks['raw_sources_unchanged']=all(a.sha(row.path)==row.sha256 for row in inv.itertuples())
checks['output_hashes_match']=all(a.sha(root/path)==h for path,h in m['outputs'].items())
checks['executed_audit_code_matches']=a.sha(a.ROOT/'scripts/stage00_audit.py')==m['code_sha256']
dm=json.loads((run/'diagnostics_final/manifest.json').read_text());checks['executed_diagnostic_code_matches']=a.sha(a.ROOT/'scripts/stage00_diagnostics.py')==dm['code_sha256']
checks['no_optimization_or_performance_access']=m['optimization_run'] is False and m['model_performance_accessed'] is False
# Each final result is keyed by raw hash; day totals use unique recording units.
dates=[]
for date,g in l.groupby('date'):
 ids=set(g.sha256);gq=q[q.sha256.isin(ids)&(q.topic=='vehicle_gps_position')];gp=gq[gq.metric=='pvt'];gf=gq[gq.metric=='rtk_fixed']
 dates.append(dict(date=date,unique_logs=len(g),recorded_s=g.log_span_s.sum(),marked_airborne_s=g.marked_airborne_s.sum(),gnss_logs=int(t[(t.sha256.isin(ids))&(t.topic=='vehicle_gps_position')].sha256.nunique()),joint_imu_gnss_airborne_s=g.main_accel_gyro_vehicle_gps_pvt_airborne_s.sum(),joint_continuous_logs=int((g.main_accel_gyro_vehicle_gps_pvt_longest_airborne_s>0).sum()),gnss_pvt_airborne_s=gp.airborne_s.sum(),rtk_fixed_airborne_s=gf.airborne_s.sum(),rtk_fixed_fraction_of_pvt=gf.airborne_s.sum()/gp.airborne_s.sum()))
a.write_csv(out/'date_summary.csv',dates)
columns=['topic','instance','unique_logs','supported_s','airborne_supported_s','hz_min','hz_median','hz_max']
core=pd.read_csv(root/'date_topic_summary.csv');core=core[(core.date=='ALL')&(core.topic.isin(a.CORE+['hall_event','rpm','sensor_baro','sensor_mag','airspeed','differential_pressure']))][columns];core.to_csv(out/'core_topic_summary.csv',index=False)
# Machine-readable judgments are scope-qualified analysis, not measured accuracies.
a.write_csv(root/'observability_matrix.csv',[
 dict(quantity='position',judgment='CONDITIONAL',support='GNSS PVT/RTK 90 of 91 logs, fused local state 91',gap='one no-GNSS log, variable RTK, antenna lever arm and latency unknown',evidence='quality.csv; topic_coverage.csv'),
 dict(quantity='velocity',judgment='CONDITIONAL',support='GNSS NED velocity ~5 Hz plus IMU and fused state',gap='subcycle translation not independently resolved by GNSS; CAN s_variance semantics',evidence='quality.csv; reviewed PX4 gnss.cpp'),
 dict(quantity='orientation',judgment='CONDITIONAL',support='fused quaternion ~50 Hz and main gyro ~100 Hz',gap='correlated EKF prior; absolute heading not validated; reset boundaries',evidence='field_statistics.csv; boundary_events.csv'),
 dict(quantity='angular_velocity',judgment='CONDITIONAL',support='main gyro and bias corrected filtered body rate ~100 Hz; phase-linked dynamics',gap='sensor selection/calibration, lowpass response, decimation and aliasing',evidence='spectral_windows.csv; diagnostics.csv'),
 dict(quantity='linear_acceleration',judgment='CONDITIONAL',support='main accelerometer specific force ~100 Hz with clear flap fundamental',gap='not kinematic a until bias/rotation/gravity/lever-arm handled; body COM and high harmonics uncertain',evidence='spectral_windows.csv; source provenance'),
 dict(quantity='angular_acceleration',judgment='CONDITIONAL_LOW_CONFIDENCE',support='gyro temporal structure and PX4 filtered xyz_derivative',gap='derivative amplifies noise; filtered derivative not independent observation; above-band motion prior dominated',evidence='field_statistics.csv; spectral_windows.csv')])
a.write_csv(root/'method_requirements.csv',[
 dict(method='GPTR',priority=1,status='CONDITIONAL',reuse='GaussianProcess.hpp; GPMotionPriorTwoKnotsFactor.h; GPIMUFactor.h',adapter='ULog clocks/calibration; separate accel/gyro timestamps; GNSS PVT factor; NED/gravity/lever arm',hard_limit='cannot reconstruct unobserved bandwidth or absolute position without anchors'),
 dict(method='gnssFGO_loose',priority=2,status='CONDITIONAL',reuse='GNSSLCIntegrator; GPInterpolatedPVTFactor; PVTFactor; Full WNOJ as comparison',adapter='ULog PVT/IMU; preintegration from calibrated sampled IMU; frame/derivative checks',hard_limit='continuous delta stream unavailable; sampled IMU integration requires validated bandwidth assumption'),
 dict(method='gnssFGO_tight',priority=3,status='BLOCKED_CURRENT_INPUTS',reuse='none directly from current logs',adapter='per-satellite GNSS raw input absent',hard_limit='PVT/RTK status cannot replace pseudorange/Doppler observations'),
 dict(method='GNSS_INS_using_logged_vehicle_imu_deltas',priority=3,status='BLOCKED_CURRENT_INPUTS',reuse='none as contiguous increment series',adapter='switch to sampled main IMU, conditional',hard_limit='actual increment time coverage about one percent')])
summary=dict(counts=m['counts'],recorded_s=float(l.log_span_s.sum()),marked_airborne_s=float(l.marked_airborne_s.sum()),joint_imu_gnss_airborne_s=float(l.main_accel_gyro_vehicle_gps_pvt_airborne_s.sum()),spectral_rows=len(s),spectral_unique_logs=s.sha256.nunique(),flap_window_frequency_quantiles=s.flap_hz.quantile([0,.05,.5,.95,1]).to_dict(),checks=checks)
a.dump(out/'checks.json',summary)
print(json.dumps(a.clean(summary),ensure_ascii=False,indent=2));assert all(checks.values()),checks

#!/usr/bin/env python3
"""Seal Stage 02.4 evidence without treating conditioned estimates as truth."""
import hashlib
import json
import subprocess
import sys
from pathlib import Path
import numpy as np
import pandas as pd
from stage02_4_paths import ROOT, OUT, CONFIG


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(*args, cwd=ROOT):
    return subprocess.check_output(['git', *args], cwd=cwd, text=True).strip()


def main():
    assert CONFIG.read_bytes() == (OUT / 'frozen_config.json').read_bytes()
    prior_dir = ROOT / 'runs/stage02_3B_20261019'
    prior = json.loads((prior_dir / 'manifest.json').read_text())
    for rel, digest in prior['artifacts'].items():
        assert sha(prior_dir / rel) == digest, rel
    for rel, digest in prior['input_sha'].items():
        assert sha(ROOT / rel) == digest, rel
    for name, digest in prior['executable_sha'].items():
        assert sha(prior_dir / 'build' / name) == digest, name
    executions = sorted(OUT.glob('*_execution.json'))
    assert len(executions) == 13
    for path in executions:
        assert json.loads(path.read_text())['returncode'] == 0, path
    checks = ['303 inherited artifacts and inherited input/binary hashes unchanged',
              '13 real solves completed; no additional parameter search',
              'fixed config bytes unchanged']
    jmax = 0.
    times = 0.
    for path in sorted(OUT.glob('*_restore')):
        name = path.name.removesuffix('_restore')
        slow = OUT / (('A' if name == 'A_restart' else name) + '_slow')
        assert sha(path / 'knots.csv') == sha(slow / 'knots.csv')
        status = pd.read_csv(path / 'solver.csv').iloc[0]
        slowstatus = pd.read_csv(slow / 'solver.csv').iloc[0]
        biascols = ['ba_x', 'ba_y', 'ba_z', 'bg_x', 'bg_y', 'bg_z']
        assert np.array_equal(status[biascols].values, slowstatus[biascols].values)
        assert status.usable == 1 and status.termination == 0
        jm = pd.read_csv(path / 'jacobian_monitor.csv').coefficient_error.max()
        jmax = max(jmax, float(jm)); assert jm < 1e-5
        d = pd.read_csv(path / 'trajectory.csv')
        assert np.isfinite(d.values).all()
        assert np.max(abs(np.linalg.norm(d[['qx','qy','qz','qw']], axis=1)-1)) < 1e-10
        assert (np.diff(d.t) > 0).all()
        flags = pd.DataFrame({'t': d.t, 'query_valid': 1, 'development_exposed': 1,
                              'reference_is_main_imu_point': 1,
                              'rigid_mounting_assumed': 1, 'physical_acceleration_validated': 0,
                              'formal_label_eligible': 0})
        flags.to_csv(path / 'quality_flags.csv', index=False, float_format='%.17g')
        (path / 'uncertainty.json').write_text(json.dumps({
            'calibrated_posterior_available': False,
            'coefficient_information': 'Conditional on fixed slow curve, template, phase, bias, timing and rigid mounting; NOT physical covariance',
            'evidence': 'A sensitivities in sensitivity.csv; no transfer of A sensitivity magnitudes to B',
            'phase_acceleration': 'Mathematically consistent, physically low confidence',
            'absolute_phase_delay_extrinsics': 'Uncalibrated; no retuning',
            'slow_fixed_exactly': True, 'bias_fixed_exactly': True,
            'joint_sensor_nuisance_identified': False,
        }, indent=2) + '\n')
    for path in list(OUT.glob('*_slow')) + list(OUT.glob('*_restore')):
        status = pd.read_csv(path / 'solver.csv').iloc[0]
        assert status.usable == 1 and status.termination == 0
        times += float(status.time_s)
    for seg in ['A', 'B']:
        for sensor, dim in [('accel',3), ('gyro',3), ('gnss',6)]:
            cols = ['t','holdout','valid'] + [f'obs{i}' for i in range(dim)]
            new = pd.read_csv(OUT / f'{seg}_restore/{sensor}_predictions.csv')
            for kind in ['H0', 'H3']:
                old = pd.read_csv(prior_dir / f'{seg}_{kind}/{sensor}_predictions.csv')
                assert np.array_equal(new[cols].values, old[cols].values)
    checks += ['Every fixed slow knot and bias exactly retained during recovery',
               'H0/H3/new exact native timestamps, original values, validity and holdout masks',
               'Finite exported states, unit quaternions; conditional coefficient Jacobians < 1e-5']
    assert json.loads((OUT / 'synthetic/validation.json').read_text())['passed']
    assert json.loads((OUT / 'projection_gate.json').read_text())['passed']
    upstream = Path('/home/zn/gptr')
    assert git('rev-parse','HEAD',cwd=upstream) == prior['upstream_gptr_commit']
    assert git('status','--porcelain',cwd=upstream) == ''
    paths = list((ROOT/'experiments/stage02_4').glob('*')) + list((ROOT/'scripts').glob('stage02_4_*.py'))
    paths += [CONFIG, ROOT/'docs/STATUS.md', ROOT/'docs/reports/02_4_qian_style_separation.md']
    # Reused mathematical/factor sources are part of this run's identity too.
    for folder, pattern in [('experiments/stage01','*.hpp'),('experiments/stage02','*.hpp'),
                            ('experiments/stage02_3B','*.hpp'),('experiments/stage02_2','phase_models.py'),
                            ('experiments/stage02_3A','continuous_phase.py')]:
        paths += list((ROOT/folder).glob(pattern))
    inputs = dict(prior['input_sha'])
    for p in (prior_dir/'observations').rglob('*'):
        if p.is_file(): inputs[str(p.relative_to(ROOT))] = sha(p)
    for p in (ROOT/'runs/stage02_2_20261017/regression').glob('*_cycles.csv'):
        inputs[str(p.relative_to(ROOT))] = sha(p)
    review = json.loads((OUT/'reference_review.json').read_text())
    manifest = dict(stage='02.4', base_commit=git('rev-parse','HEAD'),
                    verdict='CONDITIONAL; not formal-label acceptance',
                    source_sha={str(p.relative_to(ROOT)):sha(p) for p in paths if p.is_file()},
                    input_sha=inputs, raw_sha=prior['raw_sha'], reference_review=review,
                    source_manifest_sha=sha(prior_dir/'manifest.json'),
                    upstream_gptr_commit=git('rev-parse','HEAD',cwd=upstream),
                    upstream_gptr_status=git('status','--porcelain',cwd=upstream),
                    executable_sha={str(p.relative_to(ROOT)):sha(p) for p in [OUT/'build/stage02_4_recover',prior_dir/'build/stage02_3B_joint',prior_dir/'build/stage02_3B_query']},
                    checks=checks, real_solves=13, synthetic_recoveries=2,
                    total_real_solver_seconds=times, max_coefficient_jacobian_error=jmax,
                    python=sys.version, numpy=np.__version__, pandas=pd.__version__)
    manifest['artifacts'] = {str(p.relative_to(OUT)):sha(p) for p in sorted(OUT.rglob('*'))
                             if p.is_file() and 'build' not in p.relative_to(OUT).parts
                             and p.name not in ['manifest.json','git_delivery.json','finalize.log']}
    (OUT/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps({k:manifest[k] for k in ['checks','real_solves','synthetic_recoveries','total_real_solver_seconds','max_coefficient_jacobian_error']},indent=2))
    print('Sealed',len(manifest['artifacts']),'artifacts')

if __name__ == '__main__':
    main()

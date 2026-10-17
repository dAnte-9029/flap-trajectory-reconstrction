#!/usr/bin/env python3
"""Record Stage 01 source/build evidence without opening any raw flight data."""
import argparse
import csv
import hashlib
import json
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def command(*args):
    p = subprocess.run(args, text=True, capture_output=True, check=False)
    return {'argv': list(args), 'returncode': p.returncode,
            'stdout': p.stdout, 'stderr': p.stderr}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='configs/stage01_feasibility.json')
    parser.add_argument('--completed', action='store_true',
                        help='Record the completed synthetic validation, preserving the initial blocked manifest')
    args = parser.parse_args()
    cfg = json.loads(Path(args.config).read_text())
    out = Path(cfg['output_root'])
    target = out / ('manifest_completed.json' if args.completed else 'manifest.json')
    if target.exists():
        raise FileExistsError(target)
    gptr_files = ['README.md', 'LICENSE', 'gptr/CMakeLists.txt', 'gptr/package.xml',
                  'gptr/include/GaussianProcess.hpp', 'gptr/include/SE3JQ.hpp',
                  'gptr/include/SE3JQ.cpp', 'gptr/include/utility.h',
                  'gptr/include/factor/GPIMUFactor.h',
                  'gptr/include/factor/GPMotionPriorTwoKnotsFactor.h',
                  'gptr/src/GPUI.hpp', 'gptr/src/GPVICalib.cpp']
    fgo_files = ['LICENSE', 'online_fgo/include/factor/gnss/GPInterpolatedPVTFactor.h',
                 'online_fgo/include/factor/gnss/PVTFactor.h',
                 'online_fgo/src/integrator/GNSSLCIntegrator.cpp']
    upstream = {}
    for name, files in [('gptr', gptr_files), ('gnssfgo', fgo_files)]:
        root = Path(cfg[name+'_root'])
        head = command('git', '-C', str(root), 'rev-parse', 'HEAD')
        status = command('git', '-C', str(root), 'status', '--porcelain')
        assert head['returncode'] == 0 and head['stdout'].strip() == cfg[name+'_commit']
        assert status['returncode'] == 0 and status['stdout'] == '', status
        upstream[name] = {
            'root': str(root), 'head': head, 'status': status,
            'remotes': command('git', '-C', str(root), 'remote', '-v'),
            'submodules': command('git', '-C', str(root), 'submodule', 'status'),
            'file_sha256': {f: digest(root / f) for f in files},
        }
    # Source consulted at logged firmware, without checking out or changing PX4.
    px4_rev = 'a644ca7868f07004e45658d4ae9a7a8c880c6bb5'
    px4 = {}
    for f in ['src/lib/sensor_calibration/Accelerometer.hpp',
              'src/lib/sensor_calibration/Accelerometer.cpp']:
        content = subprocess.check_output(['git', '-C', '/home/zn/PX4-Autopilot',
                                           'show', f'{px4_rev}:{f}'])
        px4[f] = hashlib.sha256(content).hexdigest()
    local_files = [Path(args.config), Path('docs/reports/01_method_feasibility.md'),
                   Path('docs/STATUS.md'), Path('docs/contracts/01_upstream_feasibility.md')]
    local_files.append(Path('configs/environment.stage01.yml'))
    local_files += sorted(Path('experiments/stage01').glob('*'))
    local_files += sorted(Path('scripts').glob('stage01_*.py'))
    artifacts = [out / n for n in ['build_initial.log', 'convention_checks.json',
                                   'prototype_segments.json']]
    import numpy
    import scipy
    result = {
        'stage': '01', 'status': 'incomplete_build_blocked',
        'recorded_utc': datetime.now(timezone.utc).isoformat(),
        'repo_head_before_uncommitted_stage01': command('git', 'rev-parse', 'HEAD'),
        'repo_status_at_recording': command('git', 'status', '--short'),
        'environment': {'platform': platform.platform(), 'python': sys.version,
                        'executable': sys.executable, 'numpy': numpy.__version__,
                        'scipy': scipy.__version__, 'compiler': command('g++', '--version'),
                        'cmake': command('cmake', '--version')},
        'upstream': upstream, 'additional_px4_source': {'commit': px4_rev, 'sha256': px4},
        'local_source_sha256': {str(p): digest(p) for p in local_files if p.is_file()},
        'artifact_sha256': {str(p): digest(p) for p in artifacts},
        'scope': {'raw_ulog_read': False, 'stage00_statistics_rerun': False,
                  'model_performance_read': False, 'dependencies_installed': False,
                  'upstream_modified': False, 'gptr_binary_built': False,
                  'gptr_numeric_checks_executed': False, 'stage02_executed': False},
        'executed': ['CMake configure: stopped at missing Eigen3',
                     '10 independent analytic convention checks: passed',
                     '3 fixed 16 s prototype proposals checked against existing Stage 00: passed'],
        'not_executed': ['C++ probes and Ceres solve', 'GPTR/gnssFGO binary validation',
                         'real-flight optimization or formal labels'],
    }
    if args.completed:
        results = {}
        for name in ['original', 'adapted']:
            path = out / f'probe_{name}_final.csv'
            with path.open() as handle:
                checks = list(csv.DictReader(handle))
            assert checks and all(r['status'] != 'FAIL' for r in checks), path
            results[name] = checks
        fixed = {r['test']: r for r in results['adapted']}
        for name in ['patched_mixer_submillisecond_acceleration_error',
                     'patched_mixer_1ms_acceleration_continuity',
                     'ceres_joint_state_solution', 'adapter_manifold_plus_jacobian']:
            assert fixed[name]['status'] == 'PASS', name
        extra = [out / name for name in [
            'probe_original_final.csv', 'probe_original_final.stderr',
            'probe_adapted_final.csv', 'probe_adapted_final.stderr',
            'configure_original_v2.log', 'configure_adapted.log',
            'build_original_final.log', 'build_adapted_final.log',
            'ctest_adapted.log', 'upstream_overlay.diff',
            'convention_checks_conda.json', 'convention_checks_conda.log',
            'environment_review/create_retry.log',
            'environment_review/conda-explicit-md5.txt',
            'build_original/stage01_probe', 'build_adapted/stage01_probe',
            'build_adapted/upstream_overlay/GaussianProcess.hpp',
        ]]
        result['artifact_sha256'].update({str(p): digest(p) for p in extra})
        result['status'] = 'stage01_complete_synthetic_feasibility_with_documented_limits'
        result['numerical_checks'] = results
        result['scope'].update(dependencies_installed=True, gptr_binary_built=True,
                               gptr_numeric_checks_executed=True)
        result['scope']['build_local_upstream_header_patch'] = 'PSI endpoint shortcut only: Dtau < 0.001 changed to Dtau == 0.0; original fork unchanged'
        result['executed'] += ['Conda environment creation recovered after interrupted download',
                               'Pinned upstream core compiled with middleware utility bridge',
                               'Original and build-local adapted C++ numerical probes executed',
                               'Ceres bias-only and jointly optimized synthetic states',
                               'Adapted CTest passed; original defects and small-angle approximation limits retained']
        result['not_executed'] = ['full ROS GPTR/gnssFGO pipelines', 'gnssFGO numerical verification',
                                  'real-log calibration adapter and GNSS Ceres factor',
                                  'real-flight optimization or formal labels', 'Stage 02']
        result['environment']['prefix'] = sys.prefix
        result['environment']['active_cxx'] = command(str(Path(sys.prefix)/'bin/c++'), '--version')
        packages = []
        for path in sorted((Path(sys.prefix)/'conda-meta').glob('*.json')):
            meta = json.loads(path.read_text())
            packages.append({key: meta.get(key) for key in
                             ['name', 'version', 'build', 'channel', 'url', 'md5', 'sha256']})
        assert packages, 'Run completed provenance with the flap-recon Python'
        result['environment']['conda_packages'] = packages
    target.write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    print(target)


if __name__ == '__main__':
    main()

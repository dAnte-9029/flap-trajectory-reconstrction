#!/usr/bin/env python3
"""Validate fixed proposed segments against Stage 00 CSVs; never open a ULog.

This is provenance/interval selection, not a repeat of the raw observation audit.
No data-driven ranking, optimization, or model-performance access occurs here.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='configs/stage01_feasibility.json')
    args = parser.parse_args()
    cfg = json.loads(Path(args.config).read_text())
    root = Path(cfg['stage00_root'])
    sources = [
        'audit_final/inventory.csv', 'audit_final/per_log.csv',
        'audit_final/continuous_segments.csv', 'audit_final/quality.csv',
        'diagnostics_final/diagnostics.csv', 'diagnostics_final/boundary_events.csv',
        'diagnostics_final/spectral_windows.csv',
    ]
    tables = {}
    for name in sources:
        with (root / name).open() as handle:
            tables[name] = list(csv.DictReader(handle))
    results = []
    for spec in cfg['prototype_segments']:
        digest, start, end = spec['sha256'], spec['start_s'], spec['end_s']
        relevant = {name: [r for r in rows if r['sha256'] == digest]
                    for name, rows in tables.items()}
        inventory = next(r for r in relevant['audit_final/inventory.csv']
                         if r['duplicate'] == 'False')
        roles = json.loads(inventory['historical_identity_annotations'])
        assert roles and all(r['role'] == 'train' for r in roles), roles
        assert end - start == 16
        # Include two seconds of context at either end in the evidence check.
        joint = [r for r in relevant['audit_final/continuous_segments.csv']
                 if r['combination'] == 'main_accel_gyro_vehicle_gps_pvt'
                 and r['scope'] == 'airborne'
                 and float(r['start_s']) <= start - 2 and float(r['end_s']) >= end + 2]
        assert joint, (spec['id'], 'missing continuous airborne context')
        boundary_topics = {'sensor_accel', 'sensor_gyro', 'vehicle_attitude',
                           'vehicle_local_position', 'vehicle_gps_position', 'wing_phase'}
        boundaries = [r for r in relevant['diagnostics_final/boundary_events.csv']
                      if r['topic'] in boundary_topics and r['instance'] == '0'
                      and float(r['start_s']) < end + 2 and float(r['end_s']) > start - 2]
        assert not boundaries, boundaries
        spectra = [r for r in relevant['diagnostics_final/spectral_windows.csv']
                   if float(r['start_s']) < end
                   and float(r['start_s']) + float(r['duration_s']) > start]
        gyro_y = [r for r in spectra if r['topic'] == 'sensor_gyro' and r['axis'] == '1'
                  and float(r['start_s']) >= start - 1e-6]
        assert len(gyro_y) == 2, (spec['id'], gyro_y)
        quality = [r for r in relevant['audit_final/quality.csv']
                   if r['topic'] == 'vehicle_gps_position']
        sensors = [r for r in relevant['diagnostics_final/diagnostics.csv']
                   if r['metric'] == 'sensor_quality']
        results.append(dict(spec, relative_path=inventory['relative_path'],
                            historical_identity_annotations=roles,
                            joint_context=joint, boundary_events=boundaries,
                            flight_quality_not_window_statistics=quality,
                            flight_sensor_quality=sensors, overlapping_spectral_windows=spectra))
    out = Path(cfg['output_root']) / 'prototype_segments.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists():
        raise FileExistsError(out)
    out.write_text(json.dumps({
        'status': cfg['prototype_status'], 'time_basis': cfg['prototype_time_basis'],
        'raw_logs_read': False, 'selection_uses_model_performance': False,
        'config_sha256': sha(Path(args.config)),
        'input_sha256': {str(root / s): sha(root / s) for s in sources},
        'segments': results,
    }, indent=2, ensure_ascii=False) + '\n')
    print(f'Validated {len(results)} fixed 16 s proposals plus 2 s context; {out}')


if __name__ == '__main__':
    main()

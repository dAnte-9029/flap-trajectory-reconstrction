#!/usr/bin/env python3
"""Four predeclared mask experiments, using the unmodified Stage02 binary."""
import csv
import json
from pathlib import Path
import shutil
import subprocess
import time
import numpy as np
from stage02_1_residuals import sha


def main():
    config = Path('configs/stage02_1_diagnostics.json')
    cfg = json.loads(config.read_text())
    root, out = Path(cfg['source_root']), Path(cfg['output_root'])
    assert sha(config) == sha(out/'frozen_config.json')
    audits = list((out/'factor_audit').glob('*/factor_query.csv'))
    assert len(audits) == 3, 'Complete the baseline factor audit first'
    for p in audits:
        a = np.genfromtxt(p, delimiter=',', names=True, dtype=None, encoding=None)
        for k in ['saved_minus_query', 'query_minus_factor', 'residual_jac_mode_difference']:
            assert np.max(a[k]) < 1e-7, (p, k)
    old = json.loads((root/'observations/frozen_config.json').read_text())
    exe = root/'build/stage02_reconstruct'
    for spec in cfg['new_solves']:
        seg, mask = spec['segment'], spec['mask']
        name = seg+'_'+mask
        src = root/'observations'/seg
        ob, run = out/'observations'/name, out/'solves'/name
        assert not ob.exists() and not run.exists()
        shutil.copytree(src, ob)
        meta = json.loads((ob/'metadata.json').read_text())
        start = meta['segment']['start_s']
        baseline = json.loads((root/(seg+'_full_dt20_q1')/'manifest.json').read_text())
        assert baseline['binary_sha256'] == sha(exe)
        with (ob/'accel.csv').open() as f:
            reader = csv.DictReader(f); names = reader.fieldnames; rows = list(reader)
        previous = np.array([int(r['holdout']) for r in rows])
        t = np.array([float(r['t']) for r in rows])
        new = np.roll(previous, 2) if mask == 'shift_plus2' else (((t-start)%1 >= .4) & ((t-start)%1 < .6)).astype(int)
        for r, h in zip(rows, new): r['holdout'] = str(h)
        with (ob/'accel.csv').open('w') as f:
            writer = csv.DictWriter(f, fieldnames=names); writer.writeheader(); writer.writerows(rows)
        meta['stage02_1_mask'] = {'rule':mask,'old_count':int(previous.sum()),'new_count':int(new.sum()),'source_accel_sha256':sha(src/'accel.csv'),'other_sensors':'Original masks unchanged; conditional acceleration prediction'}
        (ob/'metadata.json').write_text(json.dumps(meta, indent=2)+'\n')
        # Only mask strings changed; preserve every time/value string exactly.
        original = list(csv.DictReader((src/'accel.csv').open()))
        assert all(all(r[k] == q[k] for k in names if k != 'holdout') for r,q in zip(rows,original))
        for p in src.glob('*.csv'):
            if p.name != 'accel.csv': assert sha(p) == sha(ob/p.name)
        args = [exe, ob, run, start, start+16, .02, old['translation_jerk_psd_m2_s5'],old['rotation_jerk_psd_rad2_s5'],old['accel_effective_sigma_m_s2'],old['gyro_effective_sigma_rad_s'],old['gnss_base_delay_s'],*old['antenna_lever_base_body_m'],0,cfg['solver_iterations'],old['accel_bias_prior_sigma_m_s2'],old['gyro_bias_prior_sigma_rad_s']]
        args = list(map(str,args)); run.parent.mkdir(exist_ok=True)
        begin = time.time(); print('START', name, flush=True)
        with (out/(name+'.log')).open('x') as f: result = subprocess.run(args,stdout=f,stderr=subprocess.STDOUT)
        run.mkdir(exist_ok=True)
        manifest = dict(command=args,exit_code=result.returncode,elapsed_s=time.time()-begin,segment=meta['segment'],dt_s=.02,prior_multiplier=1,mask=mask,base_git_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),binary_sha256=sha(exe),upstream_commit=old['gptr_commit'],config_sha256=sha(config),script_sha256=sha(__file__),input_hashes={str(p):sha(p) for p in ob.glob('*') if p.is_file()},outputs={p.name:sha(p) for p in run.glob('*') if p.is_file()},formal_labels=False)
        (run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        print('END',name,result.returncode,round(time.time()-begin,1),flush=True)
        if result.returncode: raise RuntimeError('Failed solve preserved; no substitution or parameter changes')

if __name__ == '__main__': main()

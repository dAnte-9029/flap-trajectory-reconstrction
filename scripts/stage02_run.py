#!/usr/bin/env python3
"""Frozen finite experiment runner. No model metrics and no best-run cherry picking."""
import argparse
import hashlib
import json
import subprocess
import time
from pathlib import Path


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def run(root, segment, dt, mult=1., delay=None, lever=None, yaw=0., short=False, tag=None, maxiter=None):
    cfg=json.loads((root/'observations/frozen_config.json').read_text())
    # These are fixed in the focused C++ harness, not silently configurable.
    for key,value in {'context_s':2.,'gravity_m_s2':9.80665,
                      'huber_normalized_radius':3.,'solver_threads':2,
                      'output_dt_s':.005}.items():
        assert cfg[key]==value, ('unsupported harness configuration',key)
    meta=json.loads((root/'observations'/segment/'metadata.json').read_text());spec=meta['segment']
    delay=cfg['gnss_base_delay_s'] if delay is None else delay
    lever=cfg['antenna_lever_base_body_m'] if lever is None else lever
    name=tag or f'{segment}_{"short" if short else "full"}_dt{int(dt*1000)}_q{mult:g}'
    out=root/name;assert not out.exists(),out
    exe=root/'build/stage02_reconstruct';end=spec['start_s']+(cfg['short_A_duration_s'] if short else 16.)
    args=[str(exe),str(root/'observations'/segment),str(out),spec['start_s'],end,dt,
          cfg['translation_jerk_psd_m2_s5']*mult,cfg['rotation_jerk_psd_rad2_s5']*mult,
          cfg['accel_effective_sigma_m_s2'],cfg['gyro_effective_sigma_rad_s'],delay,*lever,yaw,
          cfg['max_iterations'] if maxiter is None else maxiter,cfg['accel_bias_prior_sigma_m_s2'],cfg['gyro_bias_prior_sigma_rad_s']]
    args=list(map(str,args));t=time.time()
    source_paths=list(Path('experiments/stage02').glob('*'))+list(Path('experiments/stage01').glob('*.hpp'))+list(Path('scripts').glob('stage02_*.py'))+[Path('configs/stage02_reconstruction.json')]
    provenance={str(p):sha(p) for p in source_paths if p.is_file()}
    print('START',name,flush=True)
    with (root/(name+'.log')).open('x') as f: r=subprocess.run(args,stdout=f,stderr=subprocess.STDOUT)
    manifest={'command':args,'exit_code':r.returncode,'elapsed_s':time.time()-t,'segment':spec,'dt_s':dt,'prior_multiplier':mult,
       'delay_s_assumption':delay,'antenna_lever_body_m_assumption':lever,'initial_yaw_offset_deg':yaw,
       'base_git_commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
       'source_hashes':provenance,'binary_sha256':sha(exe),'observations_manifest_sha256':sha(root/'observations/manifest.json'),
       'frozen_config_sha256':sha(root/'observations/frozen_config.json'),
       'upstream_commit':cfg['gptr_commit'],'outputs':{p.name:sha(p) for p in out.glob('*') if p.is_file()},
       'iteration_budget_override':maxiter,'formal_labels':False,'posterior_covariance':'not calibrated/computed; sensitivity envelope only; output includes IMU-point rigid-body assumption'}
    out.mkdir(exist_ok=True);(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('END',name,'exit',r.returncode,'seconds',round(time.time()-t,1),flush=True)
    if r.returncode:raise RuntimeError(name+' failed; preserved evidence')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',default='runs/stage02_20261017');ap.add_argument('--mode',choices=['short','A','BC','sensitivity','convergence'],required=True);a=ap.parse_args();root=Path(a.root)
    cfg=json.loads((root/'observations/frozen_config.json').read_text())
    if a.mode=='short': run(root,'A',.02,short=True);return
    if a.mode=='convergence':
        import csv
        for path in sorted(root.glob('*/manifest.json')):
            if '_iter400' in path.parent.name or not (path.parent/'solver.csv').exists(): continue
            m=json.loads(path.read_text());s=list(csv.DictReader((path.parent/'solver.csv').open()))[0]
            name=path.parent.name+'_iter400'
            if s['termination']=='0' or (root/name).exists():continue
            run(root,m['segment']['id'][0],m['dt_s'],m['prior_multiplier'],delay=m['delay_s_assumption'],
                lever=m['antenna_lever_body_m_assumption'],yaw=m['initial_yaw_offset_deg'],
                short='short' in path.parent.name,tag=name,maxiter=400)
        return
    if a.mode in ('A','BC'):
        for segment in (['A'] if a.mode=='A' else ['B','C']):
            for mult in cfg['prior_multipliers']:
                for dt in cfg['knot_dt_s']:run(root,segment,dt,mult)
    else:
        for segment in ['A','B','C']:
            for delay in cfg['gnss_delay_sensitivity_s']:run(root,segment,.02,delay=delay,tag=f'{segment}_delay{int(delay*1000)}')
            run(root,segment,.02,lever=cfg['antenna_lever_sensitivity_body_m'],tag=f'{segment}_lever')
            run(root,segment,.02,yaw=cfg['initial_yaw_sensitivity_deg'],tag=f'{segment}_yawinit')


if __name__=='__main__':main()

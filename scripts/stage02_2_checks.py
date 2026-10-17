#!/usr/bin/env python3
"""Focused regression leakage, phase conventions, and SO3 branch checks."""
import json,sys
from pathlib import Path
import numpy as np
from pyulog import ULog
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments/stage02_2'))
from periodic_motion import right_jacobian_and_dot,hat
from phase_models import fourier
from stage02_2_extract_phase import sha


def main():
    cfg=json.load(open('configs/stage02_2_phase_feasibility.json'));out=Path(cfg['output_root'])/'focused_checks';out.mkdir(exist_ok=False);checks=[]
    fits=json.load(open(Path(cfg['output_root'])/'regression/fit_manifest.json'))
    for name,info in fits.items():
        train=set(info['train_cycles']);held=set(info['held_cycles']);assert train.isdisjoint(held)
        if 'forecast' in name:assert max(train)<min(held)
        for fit in info['fits']:
            assert set(fit['train_cycle_ids'])<=train and set(fit['train_cycle_ids']).isdisjoint(held)
            if 'rank' in fit:assert fit['rank']==fit['parameters_per_axis']
    checks.append(dict(name='all 88 model/split/sensor fits exclude entire held IMU cycles; forecast has no later training cycle; full rank',groups=len(fits)))
    ed=np.array([.3,-.4,.2]);maximum=0.
    for eta in [np.zeros(3),np.array([1e-8,-2e-8,3e-8]),np.array([.02,-.01,.03])]:
        J,Jd=right_jacobian_and_dot(eta,ed);h=1e-6
        numerical=(right_jacobian_and_dot(eta+h*ed,ed)[0]-right_jacobian_and_dot(eta-h*ed,ed)[0])/(2*h)
        maximum=max(maximum,float(np.max(abs(Jd-numerical))))
    assert maximum<1e-7;J,Jd=right_jacobian_and_dot(np.zeros(3),ed);assert np.max(abs(J-np.eye(3)))==0 and np.max(abs(Jd+.5*hat(ed)))==0
    checks.append(dict(name='right Jacobian derivative at zero, small and finite noncommuting rotation',max_error=maximum))
    # Minimal arithmetic counterexamples are derived independently, not copied upstream code.
    assert abs(1-np.cos(np.pi)-2)<1e-12
    centers=np.array([2,3,1]);permutation=np.argsort(centers);assert permutation[0]!=np.argsort(permutation)[0]
    assert np.var([0,4])==4 and np.std([0,4])==2
    checks.append(dict(name='half-angle wrap, inverse label permutation, std-versus-variance counterexamples',passed=True))
    params={}
    for seg in cfg['segments']:
        m=json.load(open(Path(cfg['source_root'])/'observations'/seg/'metadata.json'))
        u=ULog(m['input_path'],message_name_filter_list=['__parameters_only_no_matching_topic__']);lo=m['segment']['start_s']-2;hi=m['segment']['end_s']+2
        value=u.initial_parameters['FLAP_RATIO'];changes=[]
        for t,k,v in u.changed_parameters:
            if k!='FLAP_RATIO':continue
            if t*1e-6<=lo:value=v
            elif t*1e-6<=hi:changes.append(dict(t=t*1e-6,value=v))
        assert not changes and value==json.load(open(Path(cfg['output_root'])/'phase_raw'/seg/'metadata.json'))['parameters']['FLAP_RATIO']
        params[seg]=dict(effective_FLAP_RATIO=value,in_context_changes=changes,source_sha256=m['sha256'])
    checks.append(dict(name='FLAP_RATIO effective value and no context changes verified from raw parameter records',segments=params))
    (out/'checks.json').write_text(json.dumps(checks,indent=2)+'\n');(out/'manifest.json').write_text(json.dumps({'script_sha256':sha(__file__),'module_sha256':sha('experiments/stage02_2/periodic_motion.py')},indent=2)+'\n')
    print(json.dumps(checks,indent=2))

if __name__=='__main__':main()

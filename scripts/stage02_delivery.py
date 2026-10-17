#!/usr/bin/env python3
"""Seal evidence with integrity checks, not physical-accuracy assertions."""
import argparse
import csv
import hashlib
import importlib.metadata
import json
import subprocess
from pathlib import Path
import numpy as np


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(2**20),b''):h.update(block)
    return h.hexdigest()


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',default='runs/stage02_20261017');ap.add_argument('--check-only',action='store_true');a=ap.parse_args();root=Path(a.root)
    destination=root/'delivery_manifest.json';assert not destination.exists(),destination
    checks=[]
    def checked(name,condition):
        checks.append({'name':name,'passed':bool(condition)})
        assert condition,name
    om=json.loads((root/'observations/manifest.json').read_text())
    checked('frozen observation adapter artifacts unchanged',all(sha(root/'observations'/p)==h for p,h in om['files'].items()))
    for p in (root/'observations').glob('*/metadata.json'):
        m=json.loads(p.read_text());checked('original raw bytes unchanged '+p.parent.name,sha(m['input_path'])==m['sha256'])
    validation=json.loads((root/'final_validation/summary.json').read_text())
    for p in root.glob('*/manifest.json'):
        m=json.loads(p.read_text())
        if 'outputs' in m:
            checked('artifact hashes '+p.parent.name,all(sha(p.parent/k)==v for k,v in m['outputs'].items()))
    checked('primary numerical solves converged',all(r['solver']['termination']==0 for r in validation['runs'] if r['run'] in validation['primary_runs']))
    checked('all experimental state exports finite',all(
        np.isfinite(np.loadtxt(root/r['run']/'trajectory.csv',delimiter=',',skiprows=1)).all()
        for r in validation['runs']))
    factor=list(csv.DictReader((root/'pvt_synthetic.csv').open()));checked('PVT synthetic checks',all(r['status']=='PASS' for r in factor))
    py=json.loads((root/'adapter_checks.json').read_text());checked('independent adapter geometry checks',all(v['passed'] for v in py.values()))
    repos={}
    for name,path,expected in [('gptr','/home/zn/gptr','2c0285a4f3141b131cc7b3aa77b3c6d979c0a2c2'),('gnssFGO','/home/zn/gnssFGO','16030fb6a24263a4b927d2ec5d1da266340ddb9b')]:
        head=subprocess.check_output(['git','-C',path,'rev-parse','HEAD'],text=True).strip()
        status=subprocess.check_output(['git','-C',path,'status','--porcelain'],text=True)
        checked(name+' pinned and clean',head==expected and not status);repos[name]={'head':head,'status':status}
    if a.check_only:
        print(len(checks),'integrity checks passed; no manifest written')
        return
    # Numeric accuracy is summarized separately. Integrity checks do not certify physical truth.
    sources=[*Path('experiments/stage02').glob('*'),*Path('experiments/stage01').glob('*.hpp'),Path('experiments/stage01/CMakeLists.txt'),*Path('scripts').glob('stage02_*.py'),Path('configs/stage02_reconstruction.json'),Path('docs/reports/02_trajectory_reconstruction.md'),Path('docs/STATUS.md')]
    artifacts=[p for p in root.rglob('*') if p.is_file() and 'build' not in p.relative_to(root).parts]
    manifest={'stage':'02','scope':'41 solves: 31 planned configurations including short run and 10 same-data iteration-budget retries; no Stage03',
      'scientific_decision':'Prototype completed; NOT ready for formal Stage03 labels. See report for holdout/dynamic/observability limitations.',
      'git_head':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
      'git_status_at_seal':subprocess.check_output(['git','status','--porcelain'],text=True),
      'code_and_report_sha256':{str(p):sha(p) for p in sources if p.is_file()},
      'upstream':repos,'python_packages':{n:importlib.metadata.version(n) for n in ['numpy','scipy','pyulog','matplotlib']},
      'artifact_sha256':{str(p.relative_to(root)):sha(p) for p in artifacts},'integrity_checks':checks,
      'binaries':{str(p.relative_to(root)):sha(p) for p in (root/'build').glob('stage02_*') if p.is_file()},
      'build_configuration_sha256':{str(p.relative_to(root)):sha(p) for p in [root/'build/CMakeCache.txt',root/'build/stage01/upstream_overlay/GaussianProcess.hpp']},
      'primary_runs':validation['primary_runs'],'large_artifacts_tracked':False,'neural_model_evaluation_accessed':False}
    destination.write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n');print(destination,len(checks),'integrity checks')

if __name__=='__main__':main()

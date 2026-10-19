#!/usr/bin/env python3
"""Seal phase artifacts and verify previous deliveries remain byte-identical."""
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path
import numpy as np
import scipy
import pandas as pd
import matplotlib
from stage02_3A_reconstruct import sha,dump


def main():
    cfg=json.load(open('configs/stage02_3A_phase.json'));out=Path(cfg['output'])
    previous={}
    for root in ['runs/stage02_20261017','runs/stage02_1_20261017','runs/stage02_2_20261017']:
        delivery=json.load(open(Path(root)/'delivery_manifest.json'))
        for p,h in delivery['artifact_sha256'].items():
            path=Path(p) if Path(p).is_file() else Path(root)/p
            assert sha(path)==h,str(path)
        previous[root]=len(delivery['artifact_sha256'])
    jump=Path(cfg['jump_source']);jm=json.load(open(jump/'manifest.json'))
    for name,h in jm['outputs'].items():assert sha(jump/name)==h
    previous[str(jump)]=len(jm['outputs'])
    paths=[Path('configs/stage02_3A_phase.json'),Path('docs/STATUS.md'),Path('docs/reports/02_3A_continuous_phase_reconstruction.md'),
           Path('scripts/phase_jump_diagnostics.py'),Path('docs/reports/02_2_phase_jump_diagnostics.md')]
    paths+=list(Path('scripts').glob('stage02_3A_*.py'))+list(Path('experiments/stage02_3A').glob('*.py'))+list(Path('experiments/stage02_3A').glob('*.md'))
    paths+=[Path('experiments/stage02_2/phase_models.py')]
    paths+=[Path('configs/stage02_3A_integration_followup.json')]
    artifacts={str(p):sha(p) for p in sorted(out.rglob('*')) if p.is_file() and p.name!='delivery_manifest.json'}
    upstream=json.load(open('runs/stage02_2_20261017/delivery_manifest.json'))['upstream']
    dump(out/'delivery_manifest.json',dict(stage='02.3-A',decision='CONDITIONAL',
         sealed_utc=datetime.now(timezone.utc).isoformat(),
         git_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
         git_status=subprocess.check_output(['git','status','--short'],text=True),
         code_and_report_sha256={str(p):sha(p) for p in paths},artifact_sha256=artifacts,
         verified_previous_artifacts=previous,source_parameter_audit=json.load(open(out/'parameter_audit.json')),
         upstream_inherited_not_executed=upstream,
         packages=dict(numpy=np.__version__,scipy=scipy.__version__,pandas=pd.__version__,matplotlib=matplotlib.__version__),
         tests=json.load(open(out/'focused_checks.json')),scope='A/B mechanical phase only; no GPTR solves, labels, training or Stage03'))
    print('Sealed',len(artifacts),'artifacts; previous verified:',previous)


if __name__=='__main__':main()

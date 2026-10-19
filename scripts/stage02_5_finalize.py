#!/usr/bin/env python3
"""Verify immutable inputs, focused audit checks, and seal Stage 02.5 provenance."""
import argparse
import datetime
import scipy
from stage02_5_common import *


def verify_history():
    m=json.loads((SRC/'manifest.json').read_text())
    for rel,h in m['artifacts'].items():assert sha(SRC/rel)==h,rel
    for rel,h in m['input_sha'].items():assert sha(ROOT/rel)==h,rel
    for rel,h in m['executable_sha'].items():assert sha(ROOT/rel)==h,rel
    return m


def firmware():
    dest=OUT/'firmware_reference';records=[]
    for seg in ['A','B']:
        m=json.loads((ROOT/f'runs/stage02_3B_20261019/observations/{seg}/metadata.json').read_text())
        for rel,h in m['source_hashes'].items():
            data=subprocess.check_output(['git','show',m['firmware']+':'+rel],cwd='/home/zn/PX4-Autopilot');assert hashlib.sha256(data).hexdigest()==h
            p=dest/seg/rel;p.parent.mkdir(parents=True,exist_ok=True)
            if p.exists():assert sha(p)==h
            else:p.write_bytes(data)
            records.append(dict(segment=seg,firmware=m['firmware'],path=rel,sha256=h))
    (OUT/'firmware_verification.json').write_text(json.dumps(records,indent=2)+'\n')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--init',action='store_true');args=ap.parse_args();prior=verify_history()
    if args.init:
        OUT.mkdir(exist_ok=False,parents=True);(OUT/'frozen_config.json').write_bytes(CONFIG.read_bytes());(OUT/'window_probe_frozen.json').write_bytes((ROOT/'configs/stage02_5_window_probe.json').read_bytes());firmware()
        (OUT/'source_verification.json').write_text(json.dumps({'verified_stage02_4_artifacts':len(prior['artifacts']),'source_manifest_sha256':sha(SRC/'manifest.json'),'raw_sha':prior['raw_sha']},indent=2)+'\n');return
    assert CONFIG.read_bytes()==(OUT/'frozen_config.json').read_bytes()
    assert (ROOT/'configs/stage02_5_window_probe.json').read_bytes()==(OUT/'window_probe_frozen.json').read_bytes()
    firmware();ident=json.loads((OUT/'identity_checks.json').read_text());queries=json.loads((OUT/'query_checks.json').read_text());assert len(ident)==4 and len(queries)==18
    assert max(x['max_abs'] for x in ident)<CFG['identity_absolute_tolerance']
    assert max(x['prediction_max_error'] for x in queries)<1e-8
    assert max(x['whitening_max_error'] for x in queries)<1e-7
    ledgerdata=pd.read_csv(OUT/'ledger.csv');reconstructed=ledgerdata[[f'e{i}_mse' for i in [1,2,3]]+['c12','c13','c23']].sum(axis=1);mse_error=float(max(abs(reconstructed-ledgerdata.total_mse)));assert mse_error<1e-9
    costs=pd.read_csv(OUT/'factor_costs.csv');assert max(abs(costs.serialization_difference))<.06
    for name in CFG['slow_cases']:
        sol=pd.read_csv(SRC/f'{name}_slow/solver.csv').iloc[0]
        for sensor in ['accel','gyro','gnss']:
            d=pd.read_csv(OUT/f'{name}_{sensor}_weights.csv');assert sum(d.holdout==0)==sol[sensor+'_factors']
    upstream=Path('/home/zn/gptr');commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=upstream,text=True).strip();status=subprocess.check_output(['git','status','--porcelain'],cwd=upstream,text=True).strip();assert commit==prior['upstream_gptr_commit'] and status==''
    sources=list((ROOT/'scripts').glob('stage02_5_*.py'))+list((ROOT/'experiments/stage02_5').glob('*'))+list((ROOT/'configs').glob('stage02_5_*.json'))+[ROOT/'docs/STATUS.md',ROOT/'docs/reports/02_5_residual_attribution.md',ROOT/'experiments/stage02_3B/query.cpp',ROOT/'experiments/stage02_3B/phase_motion.hpp',ROOT/'experiments/stage02_3A/continuous_phase.py',ROOT/'scripts/stage02_extract.py']
    manifest=dict(stage='02.5',scope='Read-only saved-curve diagnostics; zero new trajectory optimizations; no formal labels',timestamp_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),base_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),source_manifest_sha256=sha(SRC/'manifest.json'),inherited_artifacts_verified=len(prior['artifacts']),raw_sha=prior['raw_sha'],upstream_gptr_commit=commit,upstream_gptr_status=status,query_binary_sha256=sha(QUERY),config_sha256=sha(OUT/'frozen_config.json'),window_probe_config_sha256=sha(OUT/'window_probe_frozen.json'),source_sha={str(p.relative_to(ROOT)):sha(p) for p in sources if p.is_file()},input_sha=prior['input_sha'],python=sys.version,numpy=np.__version__,scipy=scipy.__version__,pandas=pd.__version__,checks={'identity_max':max(x['max_abs'] for x in ident),'mse_reconstruction_max':mse_error,'query_prediction_max':max(x['prediction_max_error'] for x in queries),'query_whitening_max':max(x['whitening_max_error'] for x in queries),'solver_cost_rounding_max':float(max(abs(costs.serialization_difference))),'training_factor_counts_exact':True,'firmware_snapshots_verified':18},new_solves=0)
    manifest['artifacts']={str(p.relative_to(OUT)):sha(p) for p in sorted(OUT.rglob('*')) if p.is_file() and p.name not in ['manifest.json','git_delivery.json','finalize.log']}
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(manifest['checks'],indent=2));print('Sealed',len(manifest['artifacts']),'artifacts; verified',len(prior['artifacts']),'historical artifacts')
if __name__=='__main__':main()

#!/usr/bin/env python3
"""Initialize a fresh replication directory or verify and seal this ablation."""
import argparse,datetime,json,subprocess,sys
import numpy as np,pandas as pd,scipy
from stage02_6_run import ROOT,OUT,SRC,CFG,CONFIG,sha

def history():
 records=[]
 for path in [SRC,ROOT/'runs/stage02_5_20261019']:
  manifest=json.loads((path/'manifest.json').read_text())
  for rel,h in manifest['artifacts'].items():assert sha(path/rel)==h,(path,rel)
  for rel,h in manifest.get('input_sha',{}).items():assert sha(ROOT/rel)==h,rel
  records.append(dict(path=str(path.relative_to(ROOT)),artifacts_verified=len(manifest['artifacts']),manifest_sha256=sha(path/'manifest.json')))
 return records

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--init',action='store_true');args=ap.parse_args();prior=history()
 if args.init:
  OUT.mkdir(parents=True,exist_ok=False);(OUT/'frozen_config.json').write_bytes(CONFIG.read_bytes());(OUT/'source_verification.json').write_text(json.dumps(prior,indent=2)+'\n');return
 assert CONFIG.read_bytes()==(OUT/'frozen_config.json').read_bytes()
 executions=[json.loads(p.read_text()) for p in sorted(OUT.glob('*_execution.json'))];assert len(executions) in [6,8] and all(x['returncode']==0 for x in executions)
 assert [x['name'] for x in executions if not x['name'].endswith('_traditional')]==CFG['primary_order']
 for e in executions:
  src=SRC/f"observations/{e['name'][0]}_phase32";assert e['input_sha']=={p.name:sha(p) for p in src.iterdir() if p.is_file()}
  assert pd.read_csv(OUT/e['name']/'solver.csv').iloc[0].usable==1
 synthetic=pd.read_csv(OUT/'synthetic_with_jacobians.csv').iloc[0];assert synthetic.passed==1
 assert np.array_equal(pd.read_csv(OUT/'synthetic.csv').values,pd.read_csv(OUT/'synthetic_with_jacobians.csv').values)
 j=pd.read_csv(OUT/'synthetic_with_jacobians.csv.jacobians.csv');fd=float(np.max(abs(j.analytic-j.numerical)/(1+abs(j.numerical))));assert fd<1e-5 and len(j)==4536
 ledger=pd.read_csv(OUT/'ledger.csv');merr=float(np.max(abs(ledger.mse-ledger.e1_mse-ledger.e2_mse-ledger.cross12)));assert merr<1e-9
 checks=json.loads((OUT/'evaluation_checks.json').read_text());assert len(checks)==5*len(executions)
 assert max(x.get('query_prediction_max',0) for x in checks)<1e-8 and max(x.get('white_max',0) for x in checks)<1e-7
 assert max(x.get('identity_max',0) for x in checks)<1e-10
 costs=pd.read_csv(OUT/'factor_costs.csv');assert max(abs(costs.rounding_difference))<.06
 init=json.loads((OUT/'initialization_checks.json').read_text());assert len(init)==2 and all(x['all_groups_equal'] for x in init)
 assert max(x['position_only_velocity_max_error'] for x in init)<1e-10
 trigger=json.loads((OUT/'initialization_trigger.json').read_text());assert len(executions)==(8 if trigger['triggered'] else 6)
 upstream=Path('/home/zn/gptr')
 commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=upstream,text=True).strip();status=subprocess.check_output(['git','status','--porcelain'],cwd=upstream,text=True).strip();assert commit=='2c0285a4f3141b131cc7b3aa77b3c6d979c0a2c2' and not status
 prior24=json.loads((SRC/'manifest.json').read_text())
 sources=list((ROOT/'experiments/stage02_6').glob('*'))+list((ROOT/'scripts').glob('stage02_6_*.py'))+[ROOT/'configs/stage02_6_ablation.json',ROOT/'docs/STATUS.md',ROOT/'docs/reports/02_6_gnss_velocity_ablation.md',ROOT/'experiments/stage02/pvt_factor.hpp',ROOT/'experiments/stage02_3B/phase_motion.hpp',ROOT/'experiments/stage02_3B/query.cpp',ROOT/'experiments/stage01/CMakeLists.txt']
 manifest=dict(stage='02.6',timestamp_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),scope='Six fixed GNSS-factor ablation slow GPTR solves; no physical-periodic restoration, new labels, training or Stage03',base_commit='62d5dea893994578260070c60992ca0d9e254114',head_at_seal=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),history=prior,raw_sha=prior24['raw_sha'],upstream_gptr_commit=commit,upstream_gptr_status=status,config_sha=sha(OUT/'frozen_config.json'),input_sha=prior24['input_sha'],source_sha={str(p.relative_to(ROOT)):sha(p) for p in sources if p.is_file()},binary_sha={str(p.relative_to(ROOT)):sha(p) for p in [OUT/'build/stage02_6_joint',OUT/'build/stage02_6_synthetic',ROOT/'runs/stage02_3B_20261019/build/stage02_3B_query']},python=sys.version,numpy=np.__version__,scipy=scipy.__version__,pandas=pd.__version__,new_solves=len(executions),optional_initialization_pair=trigger,checks=dict(jacobian_rows=len(j),jacobian_relative_max=fd,mse_identity_max=merr,prediction_max=max(x.get('query_prediction_max',0) for x in checks),white_max=max(x.get('white_max',0) for x in checks),cost_serialization_max=float(max(abs(costs.rounding_difference))),identical_initial_states=True),failures='Runner expected A GNSS count80 for B actual79; resumed verification and only unexecuted solves. Summary pandas string-query exception fixed. Logs retained; zero solver retries, zero scientific-parameter changes.')
 manifest['artifacts']={str(p.relative_to(OUT)):sha(p) for p in sorted(OUT.rglob('*')) if p.is_file() and 'build' not in p.relative_to(OUT).parts and p.name not in ['manifest.json','git_delivery.json','finalize.log']}
 (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(manifest['checks'],indent=2));print('Sealed',len(manifest['artifacts']),'artifacts')
if __name__=='__main__':
 from pathlib import Path
 main()

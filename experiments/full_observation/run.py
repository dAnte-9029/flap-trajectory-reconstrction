#!/usr/bin/env python3
"""Explicit full-observation solver and reusable single-ULog CLI."""
from common import *
from prepare import prepare_ulog,phase_from_ulog,decompose
import argparse,shutil

def solve_pair(raw,out,name,build,overrides=None):
 cfg={**CFG,**(overrides or {})};meta=json.loads((raw/'metadata.json').read_text())['segment'];dec=raw.parent/(raw.name+'_separated')
 if cfg['phase_scale']!=32:
  variant=out/'observations'/f'{name}_phase{cfg["phase_scale"]}';shutil.copytree(raw,variant)
  phase_from_ulog(json.loads((raw/'metadata.json').read_text())['input_path'],variant,cfg['phase_scale']);raw=variant;dec=variant.parent/(variant.name+'_separated');decompose(raw,dec)
 env=dict(os.environ,FO_BIAS_SCALE=str(cfg['bias_scale']))
 for method,src in [('direct',raw),('slow',dec)]:
  dest=out/f'{name}_{method}'
  args=[build/'full_solve',src,dest,src/'phase_pieces.csv',meta['start_s'],meta['end_s'],cfg['knot_dt'],cfg['qt'],cfg['qr'],cfg['sigma_accel'],cfg['sigma_gyro'],cfg['gnss_delay'],*cfg['lever'],0,cfg['max_iterations'],0,1,cfg['gnss_factor_group'],0,'full_observation']
  before={p.name:sha(p) for p in src.iterdir() if p.is_file()};execute(args,out/f'{name}_{method}_execution.json',env)
  assert before=={p.name:sha(p) for p in src.iterdir() if p.is_file()}
  dump(dest/'run_config.json',dict(config=cfg,input=str(src),input_hashes=before,method=method))
  verify_counts(src,dest,cfg,False)
  print(name,method,pd.read_csv(dest/'solver.csv').to_dict('records'),flush=True)
 dest=out/f'{name}_phase';execute([build/'full_recover',out/f'{name}_slow',raw,dec,dest,0,1,'full_observation'],out/f'{name}_phase_execution.json',env)
 dump(dest/'run_config.json',dict(config=cfg,input=str(raw),decomposition=str(dec),method='phase',fixed_slow=str(out/f'{name}_slow')))
 assert sha(dest/'knots.csv')==sha(out/f'{name}_slow/knots.csv')
 verify_counts(raw,dest,cfg,True)
 return [out/f'{name}_direct',dest]

def verify_counts(src,dest,cfg,phase_gate):
 status=pd.read_csv(dest/'solver.csv').iloc[0];knots=pd.read_csv(dest/'knots.csv');pieces=pd.read_csv(dest/'phase_pieces.csv');counts={}
 assert status.usable==1
 for kind in ['accel','gyro','gnss']:
  d=pd.read_csv(src/f'{kind}.csv');assert not d.holdout.any();t=d.t.to_numpy()-(cfg['gnss_delay'] if kind=='gnss' else 0)
  mask=(d.valid==1)&(t>=knots.t.iloc[0])&(t<knots.t.iloc[-1])
  if phase_gate:mask&=(t>=pieces.start.iloc[0])&(t<=pieces.end.iloc[-1])
  actual=int(status[kind+'_factors']);assert actual==sum(mask),(kind,actual,sum(mask))
  counts[kind]=dict(actual=actual,expected=int(sum(mask)),historical_valid_reincluded=int((mask&(d.historical_holdout==1)).sum()),valid_outside_support=int(((d.valid==1)&~mask).sum()),invalid=int((d.valid==0).sum()))
 dump(dest/'factor_audit.json',dict(mode='full_observation',counts=counts))

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--ulog',type=Path);ap.add_argument('--segments',type=Path,help='JSON list of start_s/end_s inside audited support');ap.add_argument('--output',type=Path,required=True);ap.add_argument('--build',type=Path,default=ROOT/CFG['output']/'build');ap.add_argument('--ab',action='store_true');ap.add_argument('--sensitivity',action='store_true');a=ap.parse_args();out=a.output.resolve()
 if a.ulog:
  out.mkdir(parents=True,exist_ok=False);dump(out/'frozen_config.json',{**CFG,'output':str(out)})
  inputs=prepare_ulog(a.ulog.resolve(),out/'input',json.loads(a.segments.read_text()) if a.segments else None)
  from analyze import evaluate
  from seal import seal
  for raw in inputs:
   for run in solve_pair(raw,out,raw.name,a.build.resolve()):evaluate(run,a.build.resolve())
  seal(out)
 elif a.ab:
  for seg in ['A','B']:solve_pair(out/'observations'/seg,out,seg,a.build.resolve())
 elif a.sensitivity:
  for name,changes in CFG['sensitivities_A_only'].items():
   if name=='phase16':
    # Direct GP does not use mechanical phase; retain one identical control solve to verify that.
    pass
   solve_pair(out/'observations/A',out,'A_'+name,a.build.resolve(),changes)
 else:ap.error('Choose --ulog, --ab or --sensitivity')
if __name__=='__main__':main()

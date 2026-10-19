"""Immutable artifact manifest; no raw data or large output enters Git."""
from common import *

def seal(out,verify_history=False):
 out=Path(out).resolve();manifest=out/'manifest.json'
 if manifest.exists():raise FileExistsError(manifest)
 sources={}
 for p in sorted((ROOT/'experiments/full_observation').glob('*')):
  if p.is_file():sources[str(p.relative_to(ROOT))]=sha(p)
 for directory in ['experiments/stage01','experiments/stage02','experiments/stage02_2','experiments/stage02_3A','experiments/stage02_3B','experiments/stage02_6']:
  for p in sorted((ROOT/directory).glob('*')):
   if p.is_file():sources[str(p.relative_to(ROOT))]=sha(p)
 sources['scripts/stage02_extract.py']=sha(ROOT/'scripts/stage02_extract.py')
 externals={}
 for name in ['gptr','gnssFGO','PX4-Autopilot']:
  path=Path('/home/zn')/name;externals[name]=dict(commit=subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip(),working_tree=subprocess.check_output(['git','-C',str(path),'status','--porcelain'],text=True),usage='read only; GPTR compiled core, gnssFGO reference only; PX4 logged-firmware sources read via git show')
 raw={}
 for m in out.rglob('metadata.json'):
  d=json.loads(m.read_text())
  if 'input_path' in d:
   actual=sha(d['input_path']);assert actual==d['sha256'];raw[d['input_path']]=actual
 history=None
 if verify_history:
  before=json.loads((out/'historical_before.json').read_text());after=historical_snapshot();assert before==after
  history=dict(artifact_count=len(before),all_hashes_unchanged=True,before_file='historical_before.json');dump(out/'historical_verification.json',history)
 info=dict(task='Full-Observation Kinematically Consistent 6-DoF Reconstruction',mode='full_observation',base_commit=CFG['base_commit'],branch=subprocess.check_output(['git','branch','--show-current'],text=True).strip(),worktree_parent_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),code_sha256=sources,raw_sha256=raw,externals=externals,historical_preservation=history,config_sha256=sha(out/'frozen_config.json'),limits=dict(independent_physical_validation='unverified',held_out_observation_prediction='not run in this task; historical diagnostics unchanged',posterior_uncertainty='not calibrated',downstream_use='research candidate only, Stage03 not authorized'),files={str(p.relative_to(out)):dict(sha256=sha(p),bytes=p.stat().st_size) for p in sorted(out.rglob('*')) if p.is_file() and not p.is_symlink() and p!=manifest})
 dump(manifest,info);return info
if __name__=='__main__':
 import argparse
 ap=argparse.ArgumentParser();ap.add_argument('output',type=Path);ap.add_argument('--verify-history',action='store_true');a=ap.parse_args();m=seal(a.output,a.verify_history);print(len(m['files']),'artifacts sealed')

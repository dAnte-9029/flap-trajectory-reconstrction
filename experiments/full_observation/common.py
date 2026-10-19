"""Shared paths, immutable execution records, raw identities."""
import hashlib,json,os,subprocess,time,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'experiments/stage02_3A'),str(ROOT/'experiments/stage02_2')]
import numpy as np
import pandas as pd
CFG=json.loads((ROOT/'configs/full_observation_reconstruction.json').read_text())
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(2**20),b''):h.update(b)
 return h.hexdigest()
def dump(p,obj):Path(p).write_text(json.dumps(obj,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def execute(args,record,env=None):
 record=Path(record)
 if record.exists():raise FileExistsError(record)
 start=time.time()
 with record.with_suffix('.log').open('w') as f:r=subprocess.run(list(map(str,args)),stdout=f,stderr=subprocess.STDOUT,env=env)
 dump(record,dict(command=list(map(str,args)),executable_sha256=sha(args[0]),returncode=r.returncode,wall_s=time.time()-start,environment_overrides={k:v for k,v in (env or {}).items() if k.startswith('FO_')}))
 if r.returncode:raise RuntimeError(f'{record}: exit {r.returncode}; see log')
def historical_snapshot():
 # Every established artifact, including binaries; full byte preservation audit.
 return {str(p.relative_to(ROOT)):sha(p) for d in sorted((ROOT/'runs').iterdir()) if d.is_dir() and not d.name.startswith('full_observation') for p in sorted(d.rglob('*')) if p.is_file() and not p.is_symlink()}

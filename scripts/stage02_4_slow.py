#!/usr/bin/env python3
"""Reuse the pinned Stage02.3-B H0 executable for separated observations."""
import json,subprocess,time,sys
from pathlib import Path
from stage02_4_paths import ROOT,OUT,CONFIG

def solve(name,seg='A',scale=32,q=.1,delay=.11,lever=(0,0,0)):
 src=OUT/f'observations/{seg}_phase{scale}';meta=json.loads((src/'metadata.json').read_text())['segment'];output=OUT/(name+'_slow')
 args=[ROOT/'runs/stage02_3B_20261019/build/stage02_3B_joint',src,output,src/'phase_pieces.csv',meta['start_s'],meta['end_s'],.05,1e5*q,1e4*q,1.5,.05,delay,*lever,0,400,0,1]
 before=time.time()
 with (OUT/(name+'_slow.log')).open('w') as f:r=subprocess.run(list(map(str,args)),stdout=f,stderr=subprocess.STDOUT)
 record=dict(name=name,command=list(map(str,args)),returncode=r.returncode,wall_s=time.time()-before)
 (OUT/(name+'_slow_execution.json')).write_text(json.dumps(record,indent=2)+'\n')
 if r.returncode:raise RuntimeError(record)
 return output
if __name__=='__main__':
 if len(sys.argv)==1:
  for s in ['A','B']:print(solve(s,seg=s),flush=True)
 else:
  for name,kw in [('A_phase16',dict(scale=16)),('A_prior1',dict(q=1.)),('A_delay0',dict(delay=0.)),('A_lever',dict(lever=(.1,.05,.03)))]:print(solve(name,**kw),flush=True)

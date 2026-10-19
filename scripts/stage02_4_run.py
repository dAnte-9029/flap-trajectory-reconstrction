#!/usr/bin/env python3
"""Bounded fixed-slow restoration after both frozen gates; no H0/H3 reruns."""
import json,subprocess,time
from pathlib import Path
from stage02_4_slow import solve,ROOT,OUT,CONFIG

def recover(name,seg='A',scale=32,slow_name=None,init=0):
 sr=OUT/((slow_name or name)+'_slow');raw=ROOT/f'runs/stage02_3B_20261019/observations/{seg}';decomp=OUT/f'observations/{seg}_phase{scale}';output=OUT/(name+'_restore');args=[OUT/'build/stage02_4_recover',sr,raw,decomp,output,init,1]
 t=time.time()
 with (OUT/(name+'_restore.log')).open('w') as f:r=subprocess.run(list(map(str,args)),stdout=f,stderr=subprocess.STDOUT)
 (OUT/(name+'_restore_execution.json')).write_text(json.dumps(dict(command=list(map(str,args)),returncode=r.returncode,wall_s=time.time()-t),indent=2)+'\n')
 if r.returncode:raise RuntimeError(name)
 print('DONE',name,flush=True)
if __name__=='__main__':
 assert json.loads((OUT/'projection_gate.json').read_text())['passed']
 assert json.loads((OUT/'synthetic/validation.json').read_text())['passed']
 assert CONFIG.read_bytes()==(OUT/'frozen_config.json').read_bytes()
 for seg in ['A','B']:recover(seg,seg=seg)
 for name,kwargs in [('A_phase16',dict(scale=16)),('A_prior1',dict(q=1.)),('A_delay0',dict(delay=0.)),('A_lever',dict(lever=(.1,.05,.03)))]:
  solve(name,**kwargs);recover(name,scale=kwargs.get('scale',32))
 recover('A_restart',slow_name='A',init=.1)

#!/usr/bin/env python3
"""Run exactly the preregistered comparison after the synthetic gate."""
import json, subprocess, time
from pathlib import Path
import os
ROOT=Path(__file__).resolve().parents[1]
CONFIG=Path(os.environ.get('STAGE02_3B_CONFIG',str(ROOT/'configs/stage02_3B_joint.json')))
def main():
 cfg=json.loads(CONFIG.read_text());out=ROOT/cfg['output'];exe=out/'build/stage02_3B_joint'
 assert (out/'frozen_config.json').read_bytes()==CONFIG.read_bytes()
 with (out/'synthetic.csv').open('w') as f:subprocess.run([str(out/'build/stage02_3B_synthetic')],stdout=f,stderr=subprocess.STDOUT,check=True)
 jobs=[dict(name=f'A_short_H{h}',segment='A',H=h,duration=4) for h in [0,3]]
 jobs += [dict(name=f'{s}_H{h}',segment=s,H=h) for s in ['A','B'] for h in [0,1,3]]
 jobs += [dict(name='A_H3_'+d['name'],segment='A',H=3,**{k:v for k,v in d.items() if k!='name'}) for d in cfg['sensitivity_A_H3_only']]
 (out/'jobs.json').write_text(json.dumps(jobs,indent=2)+'\n');ledger=[]
 for j in jobs:
  s=j['segment'];meta=json.loads((out/f'observations/{s}/metadata.json').read_text());start=meta['segment']['start_s'];end=start+j.get('duration',16);prior=j.get('prior_multiplier',.1);lever=j.get('lever_body_m',[0,0,0]);delay=j.get('gnss_delay_s',.11);scale=j.get('phase_scale_hz',32)
  args=[exe,out/f'observations/{s}',out/j['name'],out/f'{s}_phase{scale}.csv',start,end,.05,1e5*prior,1e4*prior,1.5,.05,delay,*lever,j['H'],400,j.get('coefficient_initialization_scale',0),1]
  print('START',j['name'],flush=True);t=time.time()
  with (out/(j['name']+'.log')).open('w') as f:r=subprocess.run(list(map(str,args)),stdout=f,stderr=subprocess.STDOUT)
  ledger.append(dict(job=j,command=list(map(str,args)),returncode=r.returncode,wall_s=time.time()-t));(out/'execution.json').write_text(json.dumps(ledger,indent=2)+'\n')
  print('DONE',j['name'],r.returncode,round(time.time()-t,2),flush=True)
  if r.returncode:raise RuntimeError('Gate stopped after failed solve; retain all output')
if __name__=='__main__':main()

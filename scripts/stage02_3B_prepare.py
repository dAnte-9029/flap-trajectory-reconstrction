#!/usr/bin/env python3
"""Materialize the frozen whole-cycle split and exact C2 phase pieces. No refit."""
import hashlib, json, shutil, sys
from pathlib import Path
import os
import numpy as np
import pandas as pd
from scipy.interpolate import PPoly
ROOT=Path(__file__).resolve().parents[1]
CONFIG=Path(os.environ.get('STAGE02_3B_CONFIG',str(ROOT/'configs/stage02_3B_joint.json')))
sys.path.insert(0,str(ROOT/'experiments/stage02_3A'))
from continuous_phase import ContinuousPhase

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 cfg=json.loads(CONFIG.read_text());out=ROOT/cfg['output']
 assert (out/'frozen_config.json').read_bytes()==CONFIG.read_bytes()
 manifest={'inputs':{},'splits':{},'phase_export_max_error':{}}
 source=ROOT/cfg['observation_source'];cs=ROOT/cfg['cycle_source'];ps=ROOT/cfg['phase_source']
 fits=json.loads((cs/'fit_manifest.json').read_text())
 for seg in ['A','B']:
  dest=out/'observations'/seg;dest.mkdir(parents=True,exist_ok=False)
  cycles=pd.read_csv(cs/f'{seg}_cycles.csv');held=fits[f'{seg}_complete_cycle_mod5_eq2_accel_fourier_global_h3']['held_cycles'];intervals=cycles[cycles.cycle.isin(held)][['lo','hi']].values
  manifest['splits'][seg]={'held_cycles':held,'intervals':intervals.tolist(),'counts':{}}
  for name in ['accel.csv','gyro.csv','gnss.csv','attitude_init.csv','metadata.json']:
   src=source/seg/name;manifest['inputs'][str(src.relative_to(ROOT))]=sha(src)
   if name in ['accel.csv','gyro.csv']:
    d=pd.read_csv(src);heldmask=np.zeros(len(d),bool)
    for lo,hi in intervals:heldmask|=(d.t>=lo)&(d.t<hi)
    d['holdout']=heldmask.astype(int);d.to_csv(dest/name,index=False,float_format='%.17g')
    # Serialization must not change calibrated observations or timestamps.
    check=pd.read_csv(dest/name);original=pd.read_csv(src)
    assert np.allclose(check.drop(columns='holdout'),original.drop(columns='holdout'),rtol=1e-14,atol=1e-12,equal_nan=True)
    manifest['splits'][seg]['counts'][name]={'rows':len(d),'held':int(heldmask.sum())}
   else:shutil.copyfile(src,dest/name)
  for scale in [32,16]:
   src=ps/(f'{seg}_phase_model.json' if scale==32 else f'fits/{seg}_scale_16.json');manifest['inputs'][str(src.relative_to(ROOT))]=sha(src)
   model=ContinuousPhase.load(src);pp=PPoly.from_spline(model.spline);rows=[]
   for i in range(len(pp.x)-1):
    if pp.x[i+1]<=pp.x[i] or pp.x[i]<0 or pp.x[i+1]>model.domain[1]-model.origin:continue
    a3,a2,a1,a0=pp.c[:,i];a0+=model.offset+model.slope*pp.x[i];a1+=model.slope
    rows.append([pp.x[i]+model.origin,pp.x[i+1]+model.origin,a0,a1,a2,a3])
   data=np.asarray(rows);pd.DataFrame(data,columns=['start','end','a0','a1','a2','a3']).to_csv(out/f'{seg}_phase{scale}.csv',index=False,float_format='%.17g')
   t=np.linspace(model.domain[0],model.domain[1],10031);j=np.minimum(np.searchsorted(data[:,0],t,side='right')-1,len(data)-1);x=t-data[j,0];a0,a1,a2,a3=data[j,2:].T
   values=[a0+x*(a1+x*(a2+x*a3)),a1+x*(2*a2+3*x*a3),2*a2+6*x*a3]
   errors=[float(np.max(np.abs(v-model.query(t,k)))) for k,v in enumerate(values)];assert max(errors)<1e-7
   manifest['phase_export_max_error'][f'{seg}{scale}']=errors
   assert model.domain[0]<model.core[0] and model.domain[1]>model.core[1]
 manifest['inputs'][str((cs/'fit_manifest.json').relative_to(ROOT))]=sha(cs/'fit_manifest.json')
 (out/'preparation_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
 print(json.dumps(manifest['splits'],indent=2))
if __name__=='__main__':main()

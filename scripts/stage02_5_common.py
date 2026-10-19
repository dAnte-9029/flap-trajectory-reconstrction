"""Shared paths and exact native-time algebra for the read-only Stage 02.5 audit."""
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
CONFIG = Path(os.environ.get('STAGE02_5_CONFIG', ROOT/'configs/stage02_5_diagnostics.json'))
CFG = json.loads(CONFIG.read_text())
_reference = json.loads((ROOT/'configs/stage02_5_diagnostics.json').read_text())
assert {k:v for k,v in CFG.items() if k!='output'} == {k:v for k,v in _reference.items() if k!='output'}, 'Only output relocation is supported for the frozen audit'
OUT = ROOT/CFG['output']
SRC = ROOT/CFG['source']
QUERY = ROOT/'runs/stage02_3B_20261019/build/stage02_3B_query'
sys.path.insert(0,str(ROOT/'experiments/stage02_3A'))
from continuous_phase import ContinuousPhase

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def vr(e): return float(np.sqrt(np.mean(np.sum(np.asarray(e)**2,axis=1))))
def xyz(d,p): return d[[f'{p}{j}' for j in range(3)]].values

def query(run,t,label):
    folder=OUT/'queries';folder.mkdir(exist_ok=True)
    times=folder/(label+'_times.csv'); result=folder/(label+'.csv')
    if not result.exists():
        pd.DataFrame({'t':t}).to_csv(times,index=False,float_format='%.17g')
        subprocess.run([str(QUERY),str(run),str(times),str(result)],check=True)
    d=pd.read_csv(result)
    assert np.max(np.abs(d.t.values-t))<1e-10
    return d

def phase(seg): return ContinuousPhase.load(ROOT/f'runs/phase_reconstruction_20261019/{seg}_phase_model.json')
def cycles(seg): return pd.read_csv(ROOT/f'runs/stage02_2_20261017/regression/{seg}_cycles.csv')
def joined(seg,sensor):
    pred=pd.read_csv(SRC/f'{seg}_restore/{sensor}_predictions.csv')
    dec=pd.read_csv(SRC/f'observations/{seg}_phase32/{sensor}_decomposition.csv')
    # Exact timestamp join: no resampling, nearest-neighbour tolerance or duplicate keys.
    assert pred.t.is_unique and dec.t.is_unique
    d=pred.merge(dec,on='t',validate='one_to_one',suffixes=('','_decomp'))
    assert len(d)==len(pred) and np.array_equal(d.t,pred.t)
    assert np.array_equal(d.holdout,d.holdout_decomp) and np.array_equal(d.valid,d.valid_decomp)
    assert np.max(abs(xyz(d,'obs')-xyz(d,'raw')))<1e-12  # CSV decimal round-trip only
    y,T,P,S,Y=[xyz(d,p) for p in ['obs','trend','per','slowpred','pred']]
    e={'e1':T+P-y,'e2':S-T,'e3':Y-S-P,'total':Y-y,'slow_error':S+P-y}
    assert np.max(abs(e['total']-e['e1']-e['e2']-e['e3']))<CFG['identity_absolute_tolerance']
    assert np.max(abs(xyz(d,'addback')-(S+P)))<1e-10
    return d,e

def ledger(e,mask):
    es=[e[k][mask] for k in ['e1','e2','e3']]
    ms=[np.mean(x*x,axis=0) for x in es]
    cross=[2*np.mean(es[i]*es[j],axis=0) for i,j in [(0,1),(0,2),(1,2)]]
    total=np.mean(e['total'][mask]**2,axis=0)
    assert np.max(abs(total-sum(ms)-sum(cross)))<1e-9
    rows=[]
    for axis in ['vector',0,1,2]:
        agg=lambda x:float(sum(x)) if axis=='vector' else float(x[axis])
        r={'axis':axis,'n':int(sum(mask)),'total_mse':agg(total),'total_rmse':np.sqrt(agg(total))}
        for i,x in enumerate(ms):r[f'e{i+1}_mse']=agg(x);r[f'e{i+1}_rmse']=np.sqrt(agg(x))
        for k,x in zip(['c12','c13','c23'],cross):r[k]=agg(x)
        r['gp_replacement_mse_change']=r['e2_mse']+r['c12']
        r['physical_restoration_mse_change']=r['e3_mse']+r['c13']+r['c23']
        rows.append(r)
    return rows

def predict(d,sensor,ba,bg,lever):
    R=Rotation.from_quat(d[['qx','qy','qz','qw']]);a=d[['ax','ay','az']].values;w=d[['wx','wy','wz']].values
    if sensor=='accel':return R.inv().apply(a-[0,0,9.80665])+ba
    if sensor=='gyro':return w+bg
    return np.column_stack([d[['px','py','pz']].values+R.apply(np.tile(lever,(len(d),1))),d[['vx','vy','vz']].values+R.apply(np.cross(w,lever))])

"""Tests that full mode rejects residual masks and uses historical held observations."""
from common import *
from prepare import decompose
import shutil

def main():
 out=ROOT/CFG['output'];tests=out/'checks';tests.mkdir(exist_ok=False);src=out/'observations/A';bad=tests/'poison_mask';shutil.copytree(src,bad)
 d=pd.read_csv(bad/'accel.csv');d.loc[0,'holdout']=1;d.to_csv(bad/'accel.csv',index=False)
 meta=json.loads((src/'metadata.json').read_text())['segment'];args=[out/'build/full_solve',bad,tests/'rejected',bad/'phase_pieces.csv',meta['start_s'],meta['end_s'],.05,10000,1000,1.5,.05,.11,0,0,0,0,1,0,1,0,0,'full_observation']
 r=subprocess.run(list(map(str,args)),capture_output=True,text=True);assert r.returncode==2 and 'rejects ALL residual holdout' in r.stderr
 dump(tests/'mask_rejection.json',dict(command=list(map(str,args)),returncode=r.returncode,stderr=r.stderr,passed=True))
 # A positive influence test: formerly held observations MUST affect the refit.
 pos=tests/'poison_values';shutil.copytree(src,pos);d=pd.read_csv(pos/'accel.csv');m=(d.historical_holdout==1)&(d.valid==1);assert m.any();d.loc[m,'z']+=20;d.to_csv(pos/'accel.csv',index=False);decompose(pos,tests/'refit')
 b0=np.array(json.loads((out/'observations/A_separated/regression.json').read_text())['models']['accel']['beta']);b1=np.array(json.loads((tests/'refit/regression.json').read_text())['models']['accel']['beta']);delta=float(np.linalg.norm(b1-b0));assert delta>.01
 dump(tests/'all_valid_regression_influence.json',dict(rows_perturbed=int(sum(m)),coefficient_change=delta,passed=True))
 print('Full mode rejection and historical-held influence checks passed')
if __name__=='__main__':main()

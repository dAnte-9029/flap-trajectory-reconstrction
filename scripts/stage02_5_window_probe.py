#!/usr/bin/env python3
"""Frozen, no-solve measurement-window perturbation on already saved curves."""
from stage02_5_common import *

def main():
    assert (ROOT/'configs/stage02_5_window_probe.json').read_bytes()==(OUT/'window_probe_frozen.json').read_bytes()
    rows=[];weights=np.array([.5,1,1,1,1,.5])/5
    for seg in ['A','B']:
        run=SRC/f'{seg}_restore';s=pd.read_csv(run/'solver.csv').iloc[0];ba=s[['ba_x','ba_y','ba_z']].values.astype(float);bg=s[['bg_x','bg_y','bg_z']].values.astype(float)
        for sensor in ['accel','gyro']:
            d,e=joined(seg,sensor);t=d.t.values;obs=xyz(d,'obs');point=xyz(d,'pred')
            for mode,offsets in [('centred',np.linspace(-.00125,.00125,6)),('trailing',np.linspace(-.0025,0,6))]:
                ts=(t[:,None]+offsets).ravel();q=query(run,ts,f'{seg}_{sensor}_{mode}_window');pred=predict(q,sensor,ba,bg,np.zeros(3)).reshape(len(t),6,3);average=np.einsum('nka,k->na',pred,weights)
                for split,mask in [('fit',d.valid.astype(bool)&~d.holdout.astype(bool)),('held',d.valid.astype(bool)&d.holdout.astype(bool))]:
                    rows.append(dict(segment=seg,sensor=sensor,mode=mode,split=split,n=int(sum(mask)),instant_rmse=vr(point[mask]-obs[mask]),window_rmse=vr(average[mask]-obs[mask]),prediction_change_rms=vr(average[mask]-point[mask]),prediction_change_max=float(max(np.linalg.norm(average[mask]-point[mask],axis=1))),**{f'axis{i}_rmse':float(x) for i,x in enumerate(np.sqrt(np.mean((average[mask]-obs[mask])**2,axis=0)))}))
    pd.DataFrame(rows).to_csv(OUT/'window_probe.csv',index=False);print(pd.DataFrame(rows).query("split=='held'").to_string(index=False))
if __name__=='__main__':main()

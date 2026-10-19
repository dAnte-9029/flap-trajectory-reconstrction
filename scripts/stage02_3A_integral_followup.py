#!/usr/bin/env python3
"""Off-grid quadrature diagnostic after a near-grid refinement plateau, no refitting."""
import json
from pathlib import Path
import numpy as np
import pandas as pd
from stage02_3A_reconstruct import ContinuousPhase,load_data,sha,dump


def main():
    cfg=json.load(open('configs/stage02_3A_phase.json'));out=Path(cfg['output'])
    assert (out/'integration_followup_plan.json').exists()
    rows=[]
    for seg in cfg['segments']:
        d=load_data(cfg,seg);t=d['t'];y=d['y'];dt=np.diff(t)
        hall=np.unwrap(d['w'].phase_rad.to_numpy());ids=np.flatnonzero(np.diff(d['w'].phase_unwrapped_rad)<-np.pi)+1
        corrections=np.diff(hall)-np.diff(y)
        normal=ids[np.argmin(abs(abs(corrections[ids-1])-np.median(abs(corrections[ids-1]))))]
        large=ids[np.argmax(abs(corrections[ids-1]))]
        changed=int(np.argmax(abs(np.diff(np.diff(y)/dt))))+1
        centers=dict(ordinary_reset=t[normal],large_reset=t[large],max_encoder_rate_change=t[changed],ordinary_interior=np.mean(d['core']))
        m=ContinuousPhase.load(out/f'{seg}_phase_model.json')
        for label,center in centers.items():
            start=max(t[0]+.01,center-.1)+.00317;end=min(t[-1]-.01,center+.1)-.00143
            for n in (23,91,367,1471):
                edges=np.linspace(start,end,n+1);mid=(edges[:-1]+edges[1:])/2
                errors=[abs(sum(m.query(mid,k+1)*np.diff(edges))-(m.query(end,k)-m.query(start,k))) for k in (0,1)]
                rows.append(dict(segment=seg,regime=label,subintervals=n,start=start,end=end,phase_error_rad=errors[0],velocity_error_rad_s=errors[1]))
    pd.DataFrame(rows).to_csv(out/'offgrid_integrals.csv',index=False)
    dump(out/'integration_followup_manifest.json',dict(script_sha=sha(__file__),plan_sha=sha(out/'integration_followup_plan.json'),refit=False))


if __name__=='__main__':main()

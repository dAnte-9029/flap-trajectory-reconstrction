#!/usr/bin/env python3
"""Exact error ledger and native-time descriptive diagnostics, without refitting trajectories."""
import json
import numpy as np
import pandas as pd
from scipy.signal import lombscargle
from stage02_5_common import *


def fit_phase(t,ph,y,cy,mask):
    center=float(np.mean(t[mask]));scale=float(np.ptp(t[mask]));X=np.column_stack([np.ones(len(t)),(t-center)/scale]+[f(h*ph) for h in [1,2,3] for f in [np.sin,np.cos]])
    ids,counts=np.unique(cy[mask],return_counts=True);w=np.array([1/counts[np.searchsorted(ids,c)] for c in cy[mask]])
    b=np.linalg.lstsq(X[mask]*np.sqrt(w[:,None]),y[mask]*np.sqrt(w[:,None]),rcond=None)[0]
    return X,b


def main():
    assert CONFIG.read_bytes()==(OUT/'frozen_config.json').read_bytes()
    folder=OUT/'samples';folder.mkdir(exist_ok=True)
    accounts=[];tails=[];phase_rows=[];spec=[];bands=[];windows=[];rough=[];gap=[];sampling=[];identity=[]
    freq=np.arange(.125,45.0001,.0625)
    for seg in CFG['segments']:
        cydata=cycles(seg);start=float(pd.read_csv(SRC/f'{seg}_slow/query_settings.csv').start.iloc[0])
        for sensor in ['accel','gyro']:
            d,e=joined(seg,sensor);t=d.t.values;held=d.holdout.astype(bool).values;valid=d.valid.astype(bool).values;ph=phase(seg).query(t);cy=d.cycle.values
            e['remaining']=xyz(d,'raw')-xyz(d,'per');e['S']=xyz(d,'slowpred');e['T']=xyz(d,'trend')
            identity.append(dict(segment=seg,sensor=sensor,raw_csv_roundtrip_max=float(np.max(abs(xyz(d,'obs')-xyz(d,'raw')))),max_abs=float(np.max(abs(e['total']-e['e1']-e['e2']-e['e3'])))))
            out=pd.DataFrame(dict(t=t,holdout=held.astype(int),valid=valid.astype(int),cycle=cy,phase_rad=ph))
            for key,y in {**{k:xyz(d,p) for k,p in [('y','raw'),('P','per'),('Y','pred')]},**e}.items():
                for j in range(3):out[f'{key}{j}']=y[:,j]
            all_dec=pd.read_csv(SRC/f'observations/{seg}_phase32/{sensor}_decomposition.csv');trt=all_dec.t[all_dec.valid.astype(bool)&~all_dec.holdout.astype(bool)].values
            pos=np.searchsorted(trt,t);dist=np.minimum(abs(t-trt[np.maximum(pos-1,0)]),abs(t-trt[np.minimum(pos,len(trt)-1)]))
            frac=(t-cydata.lo.values[cy])/(cydata.hi.values[cy]-cydata.lo.values[cy]);out['nearest_retained_s']=dist;out['cycle_fraction']=frac
            out.to_csv(folder/f'{seg}_{sensor}.csv',index=False,float_format='%.17g')
            sampling.append(dict(segment=seg,sensor=sensor,n=len(t),duration=t[-1]-t[0],dt_median=np.median(np.diff(t)),dt_min=np.min(np.diff(t)),dt_max=np.max(np.diff(t)),held_points=int(sum(held)),held_gap_max=float(max(dist[held])),phase_rate_min=float(min(phase(seg).query(t,1)/(2*np.pi))),phase_rate_max=float(max(phase(seg).query(t,1)/(2*np.pi)))))
            for split,mask in [('fit',valid&~held),('held',valid&held)]:
                groups=[('all',0,mask)]
                for typ,labels in [('cycle',cy),('second',np.floor(t-start).astype(int)),('quarter',np.floor((t-start)/4).astype(int))]:
                    for label in np.unique(labels[mask]):groups.append((typ,int(label),mask&(labels==label)))
                for typ,label,m in groups:
                    for r in ledger(e,m):accounts.append(dict(segment=seg,sensor=sensor,split=split,group=typ,group_id=label,**r))
                for term in ['e1','e2','e3','total','slow_error']:
                    y=e[term][mask];norm=np.linalg.norm(y,axis=1);sq=norm**2;row=dict(segment=seg,sensor=sensor,split=split,term=term,n=len(y),rmse=vr(y))
                    for q in [.5,.9,.95,.99,1]:row[f'q{q}']=float(np.quantile(norm,q))
                    for fraction in [.01,.05,.1]:
                        k=max(1,int(np.ceil(len(sq)*fraction)));order=np.sort(sq);row[f'top{fraction}_sse_share']=float(sum(order[-k:])/sum(order));row[f'trim{fraction}_rmse']=float(np.sqrt(np.mean(order[:-k])))
                    tails.append(row)
                for term in ['remaining','S','e1','e2','e3','total']:
                    # Differences only on actually adjacent native samples, never across gaps.
                    pair=mask[1:]&mask[:-1]&(np.diff(t)<=.02)
                    dy=np.diff(e[term],axis=0)[pair];dt=np.diff(t)[pair]
                    rough.append(dict(segment=seg,sensor=sensor,split=split,term=term,n=int(sum(pair)),rms=vr(e[term][mask]),increment_rms=vr(dy),difference_rate_rms=vr(dy/dt[:,None])))
                for quart in range(4):
                    m=mask&(np.clip((frac*4).astype(int),0,3)==quart)
                    for term in ['e1','e2','total','slow_error']:gap.append(dict(segment=seg,sensor=sensor,split=split,cycle_quarter=quart,term=term,n=int(sum(m)),rmse=vr(e[term][m]),nearest_train_mean=float(np.mean(dist[m]))))
            train=valid&~held;test=valid&held
            for term in ['e1','e2','e3','total','slow_error']:
                y=e[term];X,b=fit_phase(t,ph,y,cy,train);Xh,bh=fit_phase(t,ph,y,cy,test)
                for j in range(3):
                    z=y[test,j];trend_h=Xh[test,:2]@np.linalg.lstsq(Xh[test,:2],z,rcond=None)[0];full_h=Xh[test]@bh[:,j]
                    # Held-fit is descriptive only; transferable prediction uses training coefficients.
                    row=dict(segment=seg,sensor=sensor,term=term,axis=j,held_mse=float(np.mean(z*z)),train_mean=float(np.mean(y[train,j])),held_mean=float(np.mean(z)),train_projection_held_mse=float(np.mean((z-X[test]@b[:,j])**2)),held_descriptive_phase_reduction=float(np.mean((z-trend_h)**2)-np.mean((z-full_h)**2)))
                    for h in [1,2,3]:
                        row[f'train_H{h}_amp']=float(np.hypot(b[2*h,j],b[2*h+1,j]));row[f'held_H{h}_amp']=float(np.hypot(bh[2*h,j],bh[2*h+1,j]));row[f'train_H{h}_phase']=float(np.arctan2(b[2*h+1,j],b[2*h,j]));row[f'held_H{h}_phase']=float(np.arctan2(bh[2*h+1,j],bh[2*h,j]))
                    phase_rows.append(row)
                for q in range(4):
                    m=valid&(t>=start+4*q)&(t<start+4*(q+1));_,bq=fit_phase(t,ph,y,cy,m)
                    for j in range(3):
                        for h in [1,2,3]:windows.append(dict(segment=seg,sensor=sensor,term=term,quarter=q,axis=j,h=h,amp=float(np.hypot(bq[2*h,j],bq[2*h+1,j])),phase=float(np.arctan2(bq[2*h+1,j],bq[2*h,j]))))
                for split,m in [('full',valid),('held_gapped',test)]:
                    ts=t[m]-t[0]
                    for j in range(3):
                        z=y[m,j];power=lombscargle(ts,z-np.mean(z),2*np.pi*freq,normalize=True)
                        for f,p in zip(freq,power):spec.append(dict(segment=seg,sensor=sensor,term=term,axis=j,split=split,hz=f,power=p))
                        for lo,hi in [(.125,2),(2,6),(6,10),(10,16),(16,30),(30,45)]:
                            mask=(freq>=lo)&(freq<hi);area=np.trapz(power[mask],freq[mask]);bands.append(dict(segment=seg,sensor=sensor,term=term,axis=j,split=split,lo=lo,hi=hi,relative_lomb_area=float(area/np.trapz(power,freq)),peak_hz=float(freq[np.argmax(power)])))
            for split,m in [('full',valid),('held_gapped',test)]:
                power=abs(np.mean(np.exp(2j*np.pi*freq[:,None]*(t[m]-t[0])[None,:]),axis=1))**2
                pd.DataFrame({'hz':freq,'window_power':power}).to_csv(OUT/f'{seg}_{sensor}_{split}_spectral_window.csv',index=False)
    for name,rows in [('ledger',accounts),('tails',tails),('phase_projection',phase_rows),('spectra',spec),('spectral_bands',bands),('phase_windows',windows),('roughness',rough),('gap_quarters',gap),('sampling',sampling)]:pd.DataFrame(rows).to_csv(OUT/f'{name}.csv',index=False)
    (OUT/'identity_checks.json').write_text(json.dumps(identity,indent=2)+'\n')
    print(pd.DataFrame(accounts).query("group=='all' and axis=='vector'").to_string(index=False))
if __name__=='__main__':main()

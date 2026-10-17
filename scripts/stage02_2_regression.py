#!/usr/bin/env python3
"""Mechanical phase diagnostics and complete-cycle held-out sensor regression."""
import csv,json,sys,time
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments/stage02_2'))
from phase_models import fourier,fit_predict,phase_at
from stage02_2_extract_phase import sha


def load(p):return np.genfromtxt(p,delimiter=',',names=True)
def write(p,data):
    if not data:return
    with Path(p).open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
def quant(x):return np.quantile(x,[0,.05,.5,.95,1]).tolist()
def rmse(e):return np.sqrt(np.mean(e*e,axis=0))


def prepare(cfg,seg,out):
    root=Path(cfg['source_root']);src=root/'observations'/seg;raw=Path(cfg['output_root'])/'phase_raw'/seg
    m=json.load(open(src/'metadata.json'));s,e=m['segment']['start_s'],m['segment']['end_s']
    w=load(src/'wing.csv');r=load(raw/'wing_phase_0.csv');enc=load(raw/'encoder_count_0.csv');freq=load(raw/'flap_frequency_0.csv')
    pt=w['t'];valid=(w['valid']!=0)&np.isfinite(w['phase']);phi=np.unwrap(w['phase'])
    assert np.all(valid) and np.all(np.diff(pt)>0) and max(np.diff(pt))<cfg['phase_gap_limit_s'], 'Invalid phase must be segmented before regression'
    assert np.all(np.diff(phi)>0), 'Do not force nonmonotonic phase into cycles'
    assert np.max(abs(r['timestamp']*1e-6-pt))<1e-10
    assert np.array_equal(r['encoder_total_count'],enc['total_count']) and np.array_equal(r['timestamp'],enc['timestamp'])
    cpc=4096*json.load(open(raw/'metadata.json'))['parameters']['FLAP_RATIO'];fcount=np.diff(enc['total_count'])/cpc/np.diff(pt);fp=np.diff(phi)/(2*np.pi)/np.diff(pt)
    jump=np.diff(phi)-2*np.pi*np.diff(enc['total_count'])/cpc;reset=np.diff(r['phase_unwrapped_rad'])<0
    turns=np.arange(np.ceil(phi[0]/(2*np.pi)),np.floor(phi[-1]/(2*np.pi))+1)*2*np.pi;edges=np.interp(turns,phi,pt)
    records=[]
    for i in range(len(edges)-1):
        lo,hi=edges[i:i+2];mask=(pt>=lo)&(pt<hi);target=lo>=s and hi<=e
        records.append(dict(cycle=i,lo=lo,hi=hi,mid=(lo+hi)/2,target=bool(target),frequency=1/(hi-lo),logged_frequency=float(np.mean(w['frequency'][mask]))))
    write(out/(seg+'_cycles.csv'),records)
    phase_info=dict(hall_event_recorded=False,boundary='Interpolated phase crossing, not logged Hall event',rows=len(w),valid_rows=int(sum(valid)),dt_s_quantiles=quant(np.diff(pt)),wrapped_crossings=int(sum(np.diff(w['phase'])< -np.pi)),hall_relative_unwrapped_resets=int(sum(reset)),phase_rate_hz_quantiles=quant(fp),count_rate_hz_quantiles=quant(fcount),logged_frequency_hz_quantiles=quant(w['frequency']),phase_frequency_rmse_hz=float(np.sqrt(np.mean((fp-w['frequency'][1:])**2))),count_frequency_rmse_hz=float(np.sqrt(np.mean((fcount-w['frequency'][1:])**2))),phase_minus_encoder_step_rad_quantiles=quant(jump),nonreset_step_error_max_rad=float(max(abs(jump[~reset]))),reset_step_error_max_rad=float(max(abs(jump[reset]))),complete_cycles=len(records),target_complete_cycles=sum(r['target'] for r in records))
    write(out/(seg+'_phase_steps.csv'),[dict(t=pt[i+1],phase_rate_hz=fp[i],count_rate_hz=fcount[i],logged_frequency_hz=w['frequency'][i+1],reset=int(reset[i]),phase_step_correction_rad=jump[i]) for i in range(len(fp))])
    sensors={}
    for sensor in ['accel','gyro']:
        d=load(src/(sensor+'.csv'));t=d['t'];ph,ok=phase_at(t,pt,phi,valid,cfg['phase_gap_limit_s']);cy=np.searchsorted(edges,t,side='right')-1;ok&=(cy>=0)&(cy<len(records))&(d['valid']!=0)
        phase_info[sensor+'_unsupported_rows']=int(sum(~ok));nearest=np.minimum(abs(t-np.interp(t,pt,pt)),1e10)
        j=np.clip(np.searchsorted(pt,t),1,len(pt)-1);delta=np.minimum(abs(t-pt[j]),abs(t-pt[j-1]));phase_info[sensor+'_nearest_phase_ms_quantiles']=quant(1000*delta)
        sensors[sensor]=dict(t=t[ok],phi=ph[ok],cycle=cy[ok],y=np.column_stack([d[k][ok] for k in 'xyz']))
    (out/(seg+'_phase_quality.json')).write_text(json.dumps(phase_info,indent=2)+'\n')
    return records,sensors,phase_info


def predictions(model,data,train_cycles,test_cycles,records,cfg,offset):
    t,phi,cy,y=[data[k] for k in ['t','phi','cycle','y']];train=np.isin(cy,train_cycles);test=np.isin(cy,test_cycles);pred=np.full_like(y,np.nan);details=[]
    ids,counts=np.unique(cy[train],return_counts=True);weights=np.array([1/counts[np.searchsorted(ids,i)] for i in cy[train]])
    if model=='native_linear_interpolation':
        for j in range(3):pred[test,j]=np.interp(t[test],t[train],y[train,j])
        return pred,details  # constant endpoint extrapolation in forecast, explicitly reported
    if model=='global_mean_trend':
        x=np.column_stack([np.ones(sum(train)),t[train]-np.mean(t[train])]);b=np.linalg.lstsq(x*np.sqrt(weights[:,None]),y[train]*np.sqrt(weights[:,None]),rcond=None)[0];pred[test]=np.column_stack([np.ones(sum(test)),t[test]-np.mean(t[train])])@b
        return pred,[dict(parameters_per_axis=2,train_cycle_ids=list(map(int,train_cycles)))]
    if model=='phase_bin_mean':
        center=np.mean(t[train]);x=np.column_stack([np.ones(sum(train)),t[train]-center]);b=np.linalg.lstsq(x*np.sqrt(weights[:,None]),y[train]*np.sqrt(weights[:,None]),rcond=None)[0];res=y[train]-x@b
        bins=cfg['phase_mean_bins'];ix=np.floor((phi[train]%(2*np.pi))/(2*np.pi)*bins).astype(int)
        means=np.array([np.average(res[ix==i],axis=0,weights=weights[ix==i]) for i in range(bins)]);assert np.all(np.isfinite(means))
        phase_centers=(np.arange(bins)+.5)/bins*2*np.pi;xt=np.r_[phase_centers[-1]-2*np.pi,phase_centers,phase_centers[0]+2*np.pi]
        vals=np.vstack([means[-1],means,means[0]]);pred[test]=np.column_stack([np.ones(sum(test)),t[test]-center])@b
        for j in range(3):pred[test,j]+=np.interp(phi[test]%(2*np.pi),xt,vals[:,j])
        return pred,[dict(parameters_per_axis=bins+2,train_cycle_ids=list(map(int,train_cycles)))]
    order=int(model[-1]);local='local' in model;affine=local
    used_phi=phi.copy()
    if model.startswith('cycle_scrambled'):used_phi+=offset[cy]
    if model.startswith('fixed_frequency'):
        f=np.median([records[i]['frequency'] for i in train_cycles]);used_phi=2*np.pi*f*(t-records[0]['lo'])
    if not local:
        pred[test],detail=fit_predict(t,used_phi,y,cy,train,test,order,False);details.append(detail)
    else:
        for c in test_cycles:
            chosen=sorted(train_cycles,key=lambda i:abs(records[i]['mid']-records[c]['mid']))[:cfg['local_training_cycles']]
            tm=np.isin(cy,chosen);qm=cy==c
            assert not (set(chosen)&set(test_cycles))
            pred[qm],detail=fit_predict(t,used_phi,y,cy,tm,qm,order,True);detail['query_cycle']=int(c);details.append(detail)
    return pred,details


def main():
    config=Path('configs/stage02_2_phase_feasibility.json');cfg=json.load(config.open());root=Path(cfg['output_root']);out=root/'regression';out.mkdir(exist_ok=False)
    assert sha(config)==sha(root/'frozen_config_phase_proxy.json')
    metrics=[];cycles_out=[];harmonic=[];strata=[];phase_all={};model_provenance={};start=time.time()
    models=['global_mean_trend','native_linear_interpolation','phase_bin_mean']+[f'fourier_{kind}_h{h}' for kind in ['global','local_affine'] for h in cfg['harmonics']]+['fixed_frequency_local_affine_h3','cycle_scrambled_phase_local_affine_h3']
    for seg in cfg['segments']:
        records,sensors,quality=prepare(cfg,seg,out);phase_all[seg]=quality;target=np.array([i for i,r in enumerate(records) if r['target']]);allids=np.arange(len(records));offset=np.random.default_rng(cfg['random_seed']).uniform(0,2*np.pi,len(records))
        for split in cfg['splits']:
            test=target[np.arange(len(target))%5==2] if split.startswith('complete') else target[-8:]
            # No future IMU in forecast; context is otherwise included only as whole cycles.
            train=allids[~np.isin(allids,test)] if split.startswith('complete') else allids[allids<min(test)]
            for sensor,data in sensors.items():
                t,phi,cy,y=[data[k] for k in ['t','phi','cycle','y']];held=np.isin(cy,test);mean=np.mean(y[np.isin(cy,train)],axis=0)
                for model in models:
                    pred,details=predictions(model,data,train,test,records,cfg,offset);err=pred[held]-y[held];assert np.all(np.isfinite(err))
                    key='_'.join([seg,split,sensor,model]);model_provenance[key]=dict(train_cycles=train.tolist(),held_cycles=test.tolist(),fits=details)
                    axis=rmse(err);expl=1-np.sum(err**2,axis=0)/np.sum((y[held]-mean)**2,axis=0)
                    metrics.append(dict(segment=seg,split=split,sensor=sensor,model=model,held_cycles=len(test),held_samples=int(sum(held)),rmse_vector=float(np.linalg.norm(axis)),**{f'rmse_{k}':axis[j] for j,k in enumerate('xyz')},**{f'explained_{k}':expl[j] for j,k in enumerate('xyz')},max_parameters_per_axis=max([d['parameters_per_axis'] for d in details] or [len(t[np.isin(cy,train)])]),local_fits=len(details),max_condition=max([d.get('condition',0) for d in details] or [0])))
                    rows=[]
                    for i in np.flatnonzero(held):rows.append(dict(t=t[i],cycle=int(cy[i]),phase=phi[i]%(2*np.pi),**{f'obs_{k}':y[i,j] for j,k in enumerate('xyz')},**{f'pred_{k}':pred[i,j] for j,k in enumerate('xyz')}))
                    write(out/(key+'.csv'),rows)
                    for c in test:
                        use=cy==c;errc=pred[use]-y[use];r=records[c];X=np.column_stack([np.ones(sum(use)),fourier(phi[use],3)]);bo=np.linalg.lstsq(X,y[use],rcond=None)[0];bp=np.linalg.lstsq(X,pred[use],rcond=None)[0]
                        cyc=dict(segment=seg,split=split,sensor=sensor,model=model,cycle=int(c),mid=r['mid'],frequency=r['frequency'],rmse_vector=float(np.linalg.norm(rmse(errc))),**{f'rmse_{k}':rmse(errc)[j] for j,k in enumerate('xyz')});cycles_out.append(cyc)
                        for h in [1,2,3]:
                            k=1+2*(h-1)
                            for j in range(3):
                                ao=np.hypot(bo[k,j],bo[k+1,j]);ap=np.hypot(bp[k,j],bp[k+1,j]);phase=np.angle(np.exp(1j*(np.arctan2(bp[k+1,j],bp[k,j])-np.arctan2(bo[k+1,j],bo[k,j]))))
                                harmonic.append(dict(segment=seg,split=split,sensor=sensor,model=model,cycle=int(c),axis=j,harmonic=h,observed_amplitude=ao,predicted_amplitude=ap,gain=ap/ao if ao>1e-9 else np.nan,phase_error_rad=phase,observed_phase_rad=np.arctan2(bo[k+1,j],bo[k,j]),phase_lag_ms=-phase/(2*np.pi*h*r['frequency'])*1000))
                    for label,selcycles in [('first_half',[c for c in test if records[c]['mid']<(records[target[0]]['lo']+records[target[-1]]['hi'])/2]),('second_half',[c for c in test if records[c]['mid']>=(records[target[0]]['lo']+records[target[-1]]['hi'])/2]),('freq_lt4',[c for c in test if records[c]['frequency']<4]),('freq_4to5',[c for c in test if 4<=records[c]['frequency']<5]),('freq_ge5',[c for c in test if records[c]['frequency']>=5])]:
                        use=np.isin(cy,selcycles)
                        if sum(use):strata.append(dict(segment=seg,split=split,sensor=sensor,model=model,stratum=label,n=int(sum(use)),cycles=len(selcycles),**{f'rmse_{k}':rmse(pred[use]-y[use])[j] for j,k in enumerate('xyz')}))
                print(seg,split,sensor,'cycles',len(test),'samples',sum(held),flush=True)
    write(out/'metrics.csv',metrics);write(out/'cycle_metrics.csv',cycles_out);write(out/'harmonics.csv',harmonic);write(out/'strata.csv',strata)
    (out/'phase_quality.json').write_text(json.dumps(phase_all,indent=2)+'\n');(out/'fit_manifest.json').write_text(json.dumps(model_provenance,indent=2)+'\n')
    (out/'manifest.json').write_text(json.dumps({'config_sha256':sha(config),'script_sha256':sha(__file__),'module_sha256':sha('experiments/stage02_2/phase_models.py'),'seconds':time.time()-start,'evaluation':'Complete cycle development holdout, not independent physical test'},indent=2)+'\n')

if __name__=='__main__':main()

#!/usr/bin/env python3
"""Read-only saved-curve factor evaluation, coupling and interval sensitivity."""
import json
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from stage02_5_common import *


def white_stats(e,white):
    n=np.linalg.norm(white,axis=1);s=n*n;weight=np.minimum(1,3/np.maximum(n,1e-30));rho=np.where(n<=3,s,6*n-9)
    return dict(n=len(n),rmse=vr(e) if e.shape[1]==3 else np.nan,position_rmse=vr(e[:,:3]) if e.shape[1]==6 else np.nan,velocity_rmse=vr(e[:,3:]) if e.shape[1]==6 else np.nan,white_norm_median=float(np.median(n)),white_norm_p95=float(np.quantile(n,.95)),downweighted_fraction=float(np.mean(n>3)),rho_prime_mean=float(np.mean(weight)),rho_prime_p05=float(np.quantile(weight,.05)),robust_cost=float(.5*sum(rho)),unrobust_cost=float(.5*sum(s)))


def main():
    assert CONFIG.read_bytes()==(OUT/'frozen_config.json').read_bytes()
    stats=[];costs=[];biases=[];checks=[];coupling=[];states=[];intervals=[]
    all_native={}
    for name in CFG['slow_cases']:
        seg=name[0];scale=16 if name.endswith('phase16') else 32
        run=SRC/f'{name}_slow';setting=pd.read_csv(run/'query_settings.csv').iloc[0];sol=pd.read_csv(run/'solver.csv').iloc[0]
        ba=sol[['ba_x','ba_y','ba_z']].values.astype(float);bg=sol[['bg_x','bg_y','bg_z']].values.astype(float);lever=setting[['lx','ly','lz']].values.astype(float)
        biases.append(dict(name=name,**{k:float(sol[k]) for k in ['ba_x','ba_y','ba_z','bg_x','bg_y','bg_z']},accel_prior_norm=float(np.linalg.norm(ba/.5)),gyro_prior_norm=float(np.linalg.norm(bg/.02))))
        knots=pd.read_csv(run/'knots.csv');ph=pd.read_csv(run/'phase_pieces.csv');lo=max(knots.t.iloc[0],ph.start.iloc[0]);hi=min(knots.t.iloc[-1],ph.end.iloc[-1]);factorcost={}
        for sensor,dim in [('accel',3),('gyro',3),('gnss',6)]:
            raw=pd.read_csv(SRC/f'observations/{seg}_phase{scale}/{sensor}.csv');ts=raw.t.values-(setting.delay if sensor=='gnss' else 0)
            valid=raw.valid.astype(bool).values&(ts>=lo)&(ts<=hi)&(ts<knots.t.iloc[-1]);d=raw[valid].reset_index(drop=True);t=ts[valid];q=query(run,t,name+'_'+sensor+'_native')
            pred=predict(q,sensor,ba,bg,lever);obs=d[['px','py','pz','vx','vy','vz'] if sensor=='gnss' else ['x','y','z']].values;e=pred-obs
            if sensor=='gnss':
                cov=d[[f'c{i}' for i in range(36)]].values.reshape(-1,6,6);white=np.array([np.linalg.solve(np.linalg.cholesky(c),v) for c,v in zip(cov,e)])
            else:white=e/(1.5 if sensor=='accel' else .05)
            held=d.holdout.astype(bool).values;core=(t>=setting.start)&(t<=setting.end)
            saved=pd.read_csv(run/f'{sensor}_predictions.csv');assert len(saved)==sum(core)
            err=float(np.max(abs(pred[core]-saved[[f'pred{i}' for i in range(dim)]].values)));werr=float(np.max(abs(white[core]-saved[[f'white{i}' for i in range(dim)]].values)))
            assert err<1e-8 and werr<1e-7
            checks.append(dict(name=name,sensor=sensor,prediction_max_error=err,whitening_max_error=werr))
            for region,mask in [('context_all',np.ones(len(t),bool)),('core',core)]:
                for split,m in [('fit',mask&~held),('held',mask&held)]:
                    stats.append(dict(name=name,sensor=sensor,region=region,split=split,weight_is_actual=split=='fit',**white_stats(e[m],white[m])))
            factorcost[sensor]=white_stats(e[~held],white[~held])['robust_cost']
            n=np.linalg.norm(white,axis=1)
            export=pd.DataFrame({'t':t,'holdout':held.astype(int),'core':core.astype(int),'white_norm':n,'rho_prime':np.minimum(1,3/np.maximum(n,1e-30))})
            for i in range(dim):export[f'residual{i}']=e[:,i];export[f'white{i}']=white[:,i]
            export.to_csv(OUT/f'{name}_{sensor}_weights.csv',index=False,float_format='%.17g')
            if sensor=='accel':all_native[name]=(t,q,held,core,ba,bg)
        old=pd.read_csv(run/'cost_groups.csv').set_index('group').robust_cost
        biascost=.5*(sum((ba/.5)**2)+sum((bg/.02)**2));reconstructed=factorcost['accel']+factorcost['gyro']+factorcost['gnss']+old.slow_prior+biascost
        # Old group exports have six significant digits. Exact native IMU/GNSS
        # recomputation is checked against this known serialization precision.
        assert abs(factorcost['accel']+factorcost['gyro']-old.imu)<.06
        assert abs(factorcost['gnss']-old.gnss)<.006
        assert abs(reconstructed-sol.final_cost)<.06
        costs.append(dict(name=name,accel=factorcost['accel'],gyro=factorcost['gyro'],gnss=factorcost['gnss'],prior=float(old.slow_prior),bias=biascost,reconstructed_total=reconstructed,solver_total=sol.final_cost,serialization_difference=reconstructed-sol.final_cost))
    for name,(t,q,held,core,ba,bg) in all_native.items():
        R=Rotation.from_quat(q[['qx','qy','qz','qw']]);a=q[['ax','ay','az']].values;gravity=-R.inv().apply(np.tile([0,0,9.80665],(len(t),1)));linear=R.inv().apply(a)
        for split,mask in [('fit',core&~held),('held',core&held)]:
            states.append(dict(name=name,split=split,a_rms=vr(a[mask]),alpha_rms=vr(q.loc[mask,['alphax','alphay','alphaz']].values),omega_rms=vr(q.loc[mask,['wx','wy','wz']].values),body_linear_demean_rms=vr(linear[mask]-np.mean(linear[mask],axis=0)),gravity_demean_rms=vr(gravity[mask]-np.mean(gravity[mask],axis=0))))
        if name in ['A','B']:continue
        t0,base,h0,c0,babase,bgbase=all_native['A'];assert np.array_equal(t,t0)
        Rb=Rotation.from_quat(base[['qx','qy','qz','qw']]);ab=base[['ax','ay','az']].values
        da=R.inv().apply(a-ab);dR=R.inv().apply(ab-[0,0,9.80665])-Rb.inv().apply(ab-[0,0,9.80665]);db=np.tile(ba-babase,(len(t),1));delta=linear+gravity+ba-(Rb.inv().apply(ab-[0,0,9.80665])+babase)
        assert np.max(abs(delta-da-dR-db))<1e-10
        dg=gravity+Rb.inv().apply(np.tile([0,0,9.80665],(len(t),1)))
        for split,mask in [('fit',core&~held),('held',core&held)]:
            coupling.append(dict(name=name,split=split,total_rms=vr(delta[mask]),acceleration_effect_rms=vr(da[mask]),orientation_effect_rms=vr(dR[mask]),bias_effect_rms=vr(db[mask]),gravity_projection_change_rms=vr(dg[mask]),cross_a_R=float(2*np.mean(np.sum(da[mask]*dR[mask],axis=1))),cross_a_b=float(2*np.mean(np.sum(da[mask]*db[mask],axis=1))),cross_R_b=float(2*np.mean(np.sum(dR[mask]*db[mask],axis=1)))))
    # Exact queries of saved curves; interval differences are mathematical
    # functionals, not new labels and not independent physical measurements.
    s=pd.read_csv(SRC/'A_restore/query_settings.csv').iloc[0];starts=np.arange(s.start+.01317,s.end-.2,.03713);durations=CFG['intervals_s'];times=np.concatenate([starts]+[starts+h for h in durations]);N=len(starts)
    for kind in ['slow','restore']:
        base=query(SRC/f'A_{kind}',times,'A_'+kind+'_interval')
        for name in ['A_prior1','A_delay0','A_lever','A_phase16']:
            alt=query(SRC/f'{name}_{kind}',times,name+'_'+kind+'_interval')
            pcols=['px','py','pz'];vcols=['vx','vy','vz'];wcols=['wx','wy','wz'];acols=['ax','ay','az'];alcols=['alphax','alphay','alphaz'];qc=['qx','qy','qz','qw']
            states.append(dict(name=name,split='sensitivity_'+kind,**{k+'_rms':vr(alt[c].values[:N]-base[c].values[:N]) for k,c in [('p',pcols),('v',vcols),('a',acols),('omega',wcols),('alpha',alcols)]},R_rms=float(np.sqrt(np.mean((Rotation.from_quat(base[qc].values[:N]).inv()*Rotation.from_quat(alt[qc].values[:N])).magnitude()**2)))))
            for i,h in enumerate(durations):
                stop=slice((i+1)*N,(i+2)*N);b0=base.iloc[:N];b1=base.iloc[stop];a0=alt.iloc[:N];a1=alt.iloc[stop];diffs={}
                for k,c in [('dp',pcols),('dv',vcols),('dw',wcols)]:diffs[k]=(a1[c].values-a0[c].values)-(b1[c].values-b0[c].values)
                diffs['position_moment']=diffs['dp']-h*(a0[vcols].values-b0[vcols].values)
                rb=Rotation.from_quat(b0[qc]).inv()*Rotation.from_quat(b1[qc]);ra=Rotation.from_quat(a0[qc]).inv()*Rotation.from_quat(a1[qc]);rd=(rb.inv()*ra).as_rotvec()
                intervals.append(dict(name=name,kind=kind,h=h,n=N,**{k+'_rms':vr(v) for k,v in diffs.items()},dR_rms=vr(rd),average_a_difference_rms=vr(diffs['dv']/h),average_alpha_component_difference_rms=vr(diffs['dw']/h),average_rotation_rate_difference_rms=vr(rd/h)))
    for label,rows in [('whitening',stats),('factor_costs',costs),('biases',biases),('coupling',coupling),('state_diagnostics',states),('interval_sensitivity',intervals)]:pd.DataFrame(rows).to_csv(OUT/f'{label}.csv',index=False)
    (OUT/'query_checks.json').write_text(json.dumps(checks,indent=2)+'\n')
    print(pd.DataFrame(costs).to_string(index=False));print(pd.DataFrame(stats).query("region=='core'").to_string(index=False));print(pd.DataFrame(coupling).to_string(index=False))
if __name__=='__main__':main()

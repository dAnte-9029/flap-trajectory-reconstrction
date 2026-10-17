#!/usr/bin/env python3
"""Focused algebra/kinematics/observability tests; no real trajectory optimization."""
import csv,json,sys
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments/stage02_2'))
from phase_models import fourier,phase_at,fit_predict
from periodic_motion import compose,right_jacobian_and_dot,hat
from stage02_2_extract_phase import sha


def main():
    cfg=json.load(open('configs/stage02_2_phase_feasibility.json'));out=Path(cfg['output_root'])/'synthetic';out.mkdir(exist_ok=False)
    # Gate is evaluated before constructing a more involved kinematic example.
    data=list(csv.DictReader(open(Path(cfg['output_root'])/'regression/metrics.csv')));gates=[]
    for seg in 'AB':
        for sensor,axis in [('accel','z'),('gyro','y')]:
            subset={r['model']:r for r in data if r['segment']==seg and r['sensor']==sensor and r['split']=='complete_cycle_mod5_eq2'}
            x=float(subset['fourier_local_affine_h3']['rmse_'+axis]);controls=[float(subset[m]['rmse_'+axis]) for m in ['global_mean_trend','fixed_frequency_local_affine_h3','cycle_scrambled_phase_local_affine_h3']]
            assert all(x<c for c in controls);gates.append(dict(segment=seg,sensor=sensor,axis=axis,rmse=x,controls=controls))
    (out/'gate.json').write_text(json.dumps(gates,indent=2)+'\n')
    checks=[];rng=np.random.default_rng(cfg['random_seed']);phi=np.linspace(0,2*np.pi,43,endpoint=False);F=fourier(phi,3);X=np.sqrt(2)*F;y=np.sin(phi)+.2*np.cos(3*phi);noise=.3
    dual=X@X.T@np.linalg.solve(X@X.T+noise**2*np.eye(len(phi)),y)
    primal=X@np.linalg.solve(X.T@X+noise**2*np.eye(6),X.T@y)
    err=float(max(abs(dual-primal)));assert err<cfg['synthetic_tolerances']['kernel_max_abs'];checks.append(dict(name='finite Fourier kernel primal-dual mean equivalence',max_error=err))
    # A constant phase shift cannot be used as a negative control for a full sin/cos basis.
    shift=fourier(phi+.93,3);p1=F@np.linalg.lstsq(F,y,rcond=None)[0];p2=shift@np.linalg.lstsq(shift,y,rcond=None)[0]
    assert max(abs(p1-p2))<1e-12;checks.append(dict(name='global phase-offset invariance',max_error=float(max(abs(p1-p2)))))
    _,valid=phase_at(np.array([.005,.015,.08]),np.array([0,.01,.02,.1]),np.array([0,.2,.4,2.]),np.array([True,False,True,True]),.03)
    assert not valid.any();checks.append(dict(name='invalid and gapped phase interpolation rejected',passed=True))
    # Algebraic counterexamples to reviewed upstream expressions, not execution of ROS code.
    reviews=dict(half_angle_distance_same_phase_0_2pi=float(1-np.cos((0-2*np.pi)/2)),cluster_center_example=[2,3,1],sorted_index=[2,0,1],original_label0_wrong_new=2,correct_new=1,variance_of_0_4=4,code_covariance_diagonal_for_0_4=2,negative_center_change_can_pass_one_sided_convergence=True,labels_appended_without_clear_reuse_first_iteration=True,posterior_variance_subtraction_present=True)
    (out/'upstream_arithmetic_counterexamples.json').write_text(json.dumps(reviews,indent=2)+'\n')
    knots=np.arange(0,4.01,1.);tr=rng.normal(size=(len(knots),3,2,3))*.004;rot=rng.normal(size=tr.shape)*.009
    for h in range(1,4):tr[:,h-1]/=h*h;rot[:,h-1]/=h*h
    cs=CubicSpline(knots,tr,axis=0);cr=CubicSpline(knots,rot,axis=0);axis=np.array([.12,-.08,.06])
    def query(t):
        phase=(2*np.pi*(4*t+.05*t*t+.04*np.sin(.7*t)),2*np.pi*(4+.1*t+.028*np.cos(.7*t)),2*np.pi*(.1-.0196*np.sin(.7*t)))
        slow=dict(p=np.array([.1*t**3,t+.03*t*t,.2*np.sin(.5*t)]),v=np.array([.3*t*t,1+.06*t,.1*np.cos(.5*t)]),a=np.array([.6*t,.06,-.05*np.sin(.5*t)]),R=Rotation.from_rotvec(axis*(t+.1*t*t)),w=axis*(1+.2*t),alpha=axis*.2)
        return compose(slow,phase,[cs(t,i) for i in range(3)],[cr(t,i) for i in range(3)])
    errors=[]
    for h in [1e-3,1e-4,1e-5]:
        vals=[]
        for t in np.r_[np.linspace(.13,3.87,101),[1,2,3]]:
            x=query(t);lo=query(t-h);hi=query(t+h);R=x['R'].as_matrix();Rd=(hi['R'].as_matrix()-lo['R'].as_matrix())/(2*h);W=R.T@Rd;wv=np.array([W[2,1]-W[1,2],W[0,2]-W[2,0],W[1,0]-W[0,1]])/2
            vals.append([np.linalg.norm((hi['p']-lo['p'])/(2*h)-x['v']),np.linalg.norm((hi['v']-lo['v'])/(2*h)-x['a']),np.linalg.norm(wv-x['w']),np.linalg.norm((hi['w']-lo['w'])/(2*h)-x['alpha'])])
        errors.append(dict(step=h,p_to_v=float(np.max(vals,axis=0)[0]),v_to_a=float(np.max(vals,axis=0)[1]),R_to_w=float(np.max(vals,axis=0)[2]),w_to_alpha=float(np.max(vals,axis=0)[3])))
    assert max(errors[-1][k] for k in ['p_to_v','v_to_a'])<cfg['synthetic_tolerances']['linear_fd_max_abs']
    assert max(errors[-1][k] for k in ['R_to_w','w_to_alpha'])<cfg['synthetic_tolerances']['rotation_fd_max_abs']
    checks.append(dict(name='variable phase/amplitude noncommuting SO3 derivative finite differences',results=errors))
    # Refining one-sided limits across every amplitude spline knot (C2, not C3).
    limits=[]
    for eps in [1e-4,1e-6,1e-8]:
        v=[]
        for t in [1.,2.,3.]:
            a,b=query(t-eps),query(t+eps);v.append([np.linalg.norm(a[k]-b[k]) for k in ['p','v','a','w','alpha']]+[np.linalg.norm((a['R'].inv()*b['R']).as_rotvec())])
        limits.append(dict(offset=eps,max_difference=np.max(v,axis=0).tolist()))
    assert max(limits[-1]['max_difference'])<1e-3;checks.append(dict(name='C2 coefficient-knot one-sided convergence',results=limits))
    # Independently integrate the composite body kinematic acceleration and alpha.
    def pack(x):return np.r_[x['p'],x['v'],x['R'].as_quat(),x['w']]
    def rhs(t,z):
        truth=query(t);q=z[6:10]/np.linalg.norm(z[6:10]);R=Rotation.from_quat(q);w=z[10:13];dq=.5*np.r_[q[3]*w+np.cross(q[:3],w),-q[:3]@w]
        return np.r_[z[3:6],R.apply(truth['R'].inv().apply(truth['a'])),dq,truth['alpha']]
    x0=query(.137);end=2.873;answer=query(end);integrations=[]
    for step in [.02,.01,.005]:
        sol=solve_ivp(rhs,[.137,end],pack(x0),method='RK45',max_step=step,rtol=1e-2,atol=1e-2);z=sol.y[:,-1]
        integrations.append(dict(max_step=step,evaluations=sol.nfev,p=float(np.linalg.norm(z[:3]-answer['p'])),v=float(np.linalg.norm(z[3:6]-answer['v'])),R=float(np.linalg.norm((Rotation.from_quat(z[6:10]).inv()*answer['R']).as_rotvec())),w=float(np.linalg.norm(z[10:]-answer['w']))))
    sol=solve_ivp(rhs,[.137,end],pack(x0),method='DOP853',max_step=.005,rtol=1e-11,atol=1e-12);z=sol.y[:,-1]
    err=max(np.linalg.norm(z[:3]-answer['p']),np.linalg.norm(z[3:6]-answer['v']),np.linalg.norm((Rotation.from_quat(z[6:10]).inv()*answer['R']).as_rotvec()),np.linalg.norm(z[10:]-answer['w']))
    assert err<cfg['synthetic_tolerances']['integration_end_norm'];checks.append(dict(name='independent coupled integration across coefficient knots',max_error=float(err),refinement=integrations))
    # Exact counterexample: one phase-locked translational mode is invisible in BOTH
    # GNSS position and velocity at 5Hz, while an IMU detects it at 100Hz.
    t=np.arange(0,4,.01);tg=np.arange(0,4,.2);freq=5.;amp=10/(2*np.pi*freq)**2
    dp=lambda tt:amp*(np.cos(2*np.pi*freq*tt)-1)
    dv=lambda tt:-amp*2*np.pi*freq*np.sin(2*np.pi*freq*tt)
    da=lambda tt:-10*np.cos(2*np.pi*freq*tt)
    # Model A: moving IMU point, no nuisance. Model B: straight trajectory and da nuisance.
    amb=dict(frequency_hz=freq,gnss_rate_hz=5,imu_rate_hz=100,max_position_difference_at_gnss=float(max(abs(dp(tg)))),max_velocity_difference_at_gnss=float(max(abs(dv(tg)))),identical_predicted_specific_force=True,accel_difference_rms=float(np.sqrt(np.mean(da(t)**2))),peak_to_peak_hidden_position_m=float(np.ptp(dp(t))),scope='Constructive commensurate-frequency counterexample, not proof all actual A/B frequencies unobservable')
    assert amb['max_position_difference_at_gnss']<1e-12 and amb['max_velocity_difference_at_gnss']<1e-12
    (out/'ambiguity.json').write_text(json.dumps(amb,indent=2)+'\n');checks.append(dict(name='motion-versus-phase-nuisance exact PVT/IMU equivalence counterexample',passed=True))
    np.savetxt(out/'ambiguity_curve.csv',np.column_stack([t,dp(t),dv(t),da(t)]),delimiter=',',header='t,delta_p,delta_v,delta_a',comments='')
    samples=[]
    for t in np.arange(0,4,.005):
        x=query(t);samples.append(np.r_[t,x['p'],x['v'],x['a'],x['R'].as_quat(),x['w'],x['alpha']])
    np.savetxt(out/'synthetic_motion.csv',samples,delimiter=',',header='t,px,py,pz,vx,vy,vz,ax,ay,az,qx,qy,qz,qw,wx,wy,wz,alphax,alphay,alphaz',comments='')
    (out/'checks.json').write_text(json.dumps(checks,indent=2)+'\n')
    (out/'manifest.json').write_text(json.dumps({'config_sha256':sha('configs/stage02_2_phase_feasibility.json'),'source_sha256':{p:sha(p) for p in [__file__,'experiments/stage02_2/periodic_motion.py','experiments/stage02_2/phase_models.py']},'real_joint_optimization':False},indent=2)+'\n')
    for c in checks:print(c['name'],c.get('max_error','PASS'))

if __name__=='__main__':main()

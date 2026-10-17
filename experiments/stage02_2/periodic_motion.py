"""Kinematic composition only: a small periodic perturbation of a supplied slow state.

No trajectory estimator, GP solver, or physical attribution is implemented here.
The caller supplies C2 phase and coefficient functions with their derivatives.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def hat(x):
    x,y,z=x
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def right_jacobian_and_dot(eta,eta_dot):
    s=float(eta@eta);sd=2*float(eta@eta_dot);K=hat(eta);Kd=hat(eta_dot)
    if s<1e-8:
        a=.5-s/24+s*s/720-s**3/40320;b=1/6-s/120+s*s/5040-s**3/362880
        ad=(-1/24+s/360-s*s/13440)*sd;bd=(-1/120+s/2520-s*s/120960)*sd
    else:
        z=np.sqrt(s);zd=sd/(2*z);a=(1-np.cos(z))/s;b=(z-np.sin(z))/z**3
        ad=(z*np.sin(z)-2*(1-np.cos(z)))/z**3*zd
        bd=(z*(1-np.cos(z))-3*(z-np.sin(z)))/z**4*zd
    return np.eye(3)-a*K+b*K@K, -ad*K-a*Kd+bd*K@K+b*(Kd@K+K@Kd)


def harmonic_motion(phi,phi_dot,phi_ddot,coeff,coeff_dot,coeff_ddot):
    """coeff[order, sin_or_cos, xyz]; all time/amplitude chain-rule terms."""
    value=np.zeros(3);velocity=np.zeros(3);accel=np.zeros(3)
    for h in range(1,len(coeff)+1):
        s,c=np.sin(h*phi),np.cos(h*phi)
        b=np.array([s,c]);bd=h*phi_dot*np.array([c,-s])
        bdd=h*phi_ddot*np.array([c,-s])-(h*phi_dot)**2*b
        value+=b@coeff[h-1]
        velocity+=b@coeff_dot[h-1]+bd@coeff[h-1]
        accel+=b@coeff_ddot[h-1]+2*bd@coeff_dot[h-1]+bdd@coeff[h-1]
    return value,velocity,accel


def compose(slow,phase,translation,rotation):
    dp,dv,da=harmonic_motion(*phase,*translation)
    eta,ed,edd=harmonic_motion(*phase,*rotation)
    Q=Rotation.from_rotvec(eta);J,Jd=right_jacobian_and_dot(eta,ed)
    wq=J@ed;aq=Jd@ed+J@edd;ws=Q.inv().apply(slow['w'])
    return dict(p=slow['p']+dp,v=slow['v']+dv,a=slow['a']+da,R=slow['R']*Q,
                w=ws+wq,alpha=Q.inv().apply(slow['alpha'])-np.cross(wq,ws)+aq)

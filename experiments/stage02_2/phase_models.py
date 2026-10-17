"""Small finite Fourier regressors; not a GP trajectory solver or physical classifier."""
import numpy as np


def fourier(phi, order):
    return np.column_stack([f(k*phi) for k in range(1,order+1) for f in (np.sin,np.cos)])


def design(t,phi,order,center,scale,affine):
    u=(t-center)/scale;F=fourier(phi,order)
    return np.column_stack([np.ones(len(t)),u,F]+([u[:,None]*F] if affine else []))


def fit_predict(t,phi,y,cycle,train,query,order=3,affine=False):
    center=float(np.mean(t[train]));scale=max(float(np.ptp(t[train])/2),.1)
    X=design(t[train],phi[train],order,center,scale,affine)
    Xq=design(t[query],phi[query],order,center,scale,affine)
    ids,counts=np.unique(cycle[train],return_counts=True);w=np.array([1/counts[np.searchsorted(ids,i)] for i in cycle[train]])
    beta,_,rank,singular=np.linalg.lstsq(X*np.sqrt(w[:,None]),y[train]*np.sqrt(w[:,None]),rcond=None)
    return Xq@beta,dict(parameters_per_axis=X.shape[1],rank=int(rank),condition=float(singular[0]/singular[-1]),train_cycle_ids=ids.tolist(),center=center,scale=scale,beta=beta.tolist())


def phase_at(t,phase_t,unwrapped,valid,gap=.03):
    """No extrapolation or interpolation across invalid/gapped endpoints."""
    i=np.searchsorted(phase_t,t,side='right')-1;j=i+1
    inside=(i>=0)&(j<len(phase_t));i=np.clip(i,0,len(phase_t)-1);j=np.clip(j,0,len(phase_t)-1)
    good=inside&valid[i]&valid[j]&((phase_t[j]-phase_t[i])<=gap)
    out=np.full(len(t),np.nan);out[good]=np.interp(t[good],phase_t,unwrapped)
    return out,good

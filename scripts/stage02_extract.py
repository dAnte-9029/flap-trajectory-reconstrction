#!/usr/bin/env python3
"""Selected raw ULog -> native-time calibrated observations. No audit rerun.

Fail closed on unsupported calibration, resets or gaps in the chosen context.
Firmware source hashes and assumptions are saved; original ULogs are read-only.
"""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pyulog import ULog


def sha(p):
    h = hashlib.sha256()
    with Path(p).open('rb') as f:
        for block in iter(lambda: f.read(2**20), b''): h.update(block)
    return h.hexdigest()


def ecef(lla):
    lat, lon = np.deg2rad(lla[:, :2]).T
    h = lla[:, 2]; a = 6378137.; e2 = 6.6943799901413165e-3
    n = a / np.sqrt(1-e2*np.sin(lat)**2)
    return np.array([(n+h)*np.cos(lat)*np.cos(lon), (n+h)*np.cos(lat)*np.sin(lon), (n*(1-e2)+h)*np.sin(lat)]).T


def ned_matrix(lat_deg, lon_deg):
    a,b = np.deg2rad([lat_deg,lon_deg]); s,c = np.sin(a),np.cos(a); u,v = np.sin(b),np.cos(b)
    return np.array([[-s*v,-s*u,c],[-u,v,0],[-c*v,-c*u,-s]])


def calibration(p, kind, device):
    prefix = 'CAL_ACC' if kind == 'accel' else 'CAL_GYRO'
    ids = [i for i in range(4) if p.get(f'{prefix}{i}_ID') == device]
    assert len(ids) == 1, ('device calibration',device,ids)
    key = f'{prefix}{ids[0]}_'
    # Audited internal SPI IMUs only; no generic external-rotation guess.
    assert device in (4259850,4325386,2490378) and p[key+'ROT'] == -1
    assert p['SENS_BOARD_ROT'] == 8 and p['TC_A_ENABLE'] == p['TC_G_ENABLE'] == 0
    level = Rotation.from_euler('xyz',[p['SENS_BOARD_'+a+'_OFF'] for a in 'XYZ'],degrees=True).as_matrix()
    rot = level @ np.diag([1.,-1.,-1.])
    off = np.array([p[key+a+'OFF'] for a in 'XYZ'])
    scale = np.array([p[key+a+'SCALE'] for a in 'XYZ']) if kind == 'accel' else np.ones(3)
    return rot,off,scale


def extract(spec, cfg, out):
    path = Path(cfg['ulog_root'])/spec['relative_path']; assert sha(path) == spec['sha256']
    u = ULog(str(path)); ds = {(d.name,d.multi_id): d.data for d in u.data_list}
    lo,hi = spec['start_s']-cfg['context_s'],spec['end_s']+cfg['context_s']
    out.mkdir(parents=True,exist_ok=False)
    meta = {'segment':spec,'input_path':str(path),'sha256':spec['sha256'], 'firmware':u.msg_info_dict['ver_sw'],
            'time_basis':'native PX4 HRT seconds; IMU timestamp_sample; GNSS publication minus explicit assumed delay',
            'parameters':{},'topics':{},'excluded':{},'source_hashes':{},'calibration':{},
            'uncertainty':'No calibrated posterior. Unknown mount rigidity, physical extrinsics and absolute latency. Reference point is main IMU under rigid mounting assumption.'}
    sources = ['src/lib/sensor_calibration/'+f for f in ['Accelerometer.cpp','Accelerometer.hpp','Gyroscope.cpp','Gyroscope.hpp','Utilities.cpp']]
    sources += ['src/lib/drivers/accelerometer/PX4Accelerometer.cpp','src/lib/drivers/gyroscope/PX4Gyroscope.cpp','src/drivers/uavcan/sensors/gnss.cpp','src/lib/conversion/rotation.h']
    for f in sources:
        data = subprocess.check_output(['git','-C','/home/zn/PX4-Autopilot','show',meta['firmware']+':'+f])
        meta['source_hashes'][f]=hashlib.sha256(data).hexdigest()
    p = dict(u.initial_parameters)
    relevant = ('CAL_ACC','CAL_GYRO','TC_A','TC_G','SENS_BOARD','EKF2_GPS','EKF2_IMU_POS')
    for t,k,v in u.changed_parameters:
        if t*1e-6 <= lo: p[k]=v
        elif t*1e-6 <= hi and k.startswith(relevant): raise ValueError(('calibration parameter change',t,k))
    meta['parameters']={k:v for k,v in p.items() if k.startswith(relevant)}
    for d in u.dropouts:
        assert not (d.duration > 0 and d.timestamp*1e-6 < hi and (d.timestamp+d.duration*1000)*1e-6 > lo), 'logger dropout'
    def select(name, instance=0):
        d=ds[name,instance]; ts=d.get('timestamp_sample',d['timestamp']).astype(float)*1e-6
        assert np.all(ts>0), ('invalid sample time',name)
        use=(ts>=lo)&(ts<=hi); idx=np.where(use)[0]; t=ts[use]
        assert len(t)>1 and np.all(np.diff(t)>0), ('time order',name)
        delta=np.diff(t); med=float(np.median(delta))
        assert np.max(delta)<max(3*med,0.04) if name!='vehicle_gps_position' else np.max(delta)<0.65
        meta['topics'][f'{name}[{instance}]']={'rows':len(t),'median_dt_s':med,'max_gap_s':float(max(delta)),
            'first_s':float(t[0]),'last_s':float(t[-1]),'max_publication_minus_sample_s':float(np.max(d['timestamp'][use]*1e-6-t))}
        return d,idx,t
    def save(name,cols,header):
        np.savetxt(out/name,np.column_stack(cols),delimiter=',',header=header,comments='',fmt='%.17g')
    for kind in ['accel','gyro']:
        d,idx,t=select('sensor_'+kind); device=np.unique(d['device_id'][idx]);assert len(device)==1
        rot,off,scale=calibration(p,kind,int(device[0]))
        raw=np.column_stack([d[a][idx] for a in 'xyz']); xyz=((raw-off)*scale)@rot.T
        valid=np.isfinite(xyz).all(axis=1)
        clips=np.column_stack([d[f'clip_counter[{i}]'][idx] for i in range(3)])
        valid &= np.all(clips==0,axis=1)  # conservative recorded clipping; do not turn spikes into an exclusion rule
        assert np.all(d['error_count'][idx]==d['error_count'][idx[0]]), 'sensor error count change'
        hold=idx%cfg['holdout_modulus']==cfg['holdout_remainder']
        meta['excluded'][kind]=int(sum(~valid))
        meta['calibration'][kind]={'device_id':int(device[0]),'rotation':rot.tolist(),'offset':off.tolist(),'scale':scale.tolist(),
           'thermal_offset':'zero: logged TC_ENABLE=0; no sensor_correction topic','driver_rotation':'already in raw topic; not applied twice',
           'residual_bias':'one constant body-frame bias per segment, zero-centred heuristic prior; EKF bias not used'}
        save(kind+'.csv',[t,xyz,hold.astype(int),valid.astype(int),d['timestamp'][idx]*1e-6,raw], 't,x,y,z,holdout,valid,t_publication,raw_x,raw_y,raw_z')
    d,idx,t=select('vehicle_gps_position')
    lla=np.column_stack([d[k][idx] for k in ['latitude_deg','longitude_deg','altitude_ellipsoid_m']])
    hold=idx%cfg['holdout_modulus']==cfg['holdout_remainder']
    valid=(d['fix_type'][idx]>=3)&d['vel_ned_valid'][idx].astype(bool)&np.isfinite(lla).all(axis=1)
    valid &= (abs(lla[:,0])<=90)&(abs(lla[:,1])<=180)&(d['eph'][idx]>0)&(d['epv'][idx]>0)&(d['s_variance_m_s'][idx]>0)
    valid &= (d['system_error'][idx]==0)
    epochs=d['time_utc_usec'][idx]; seen=set()
    for i,e in enumerate(epochs):
        if int(e) in seen or e<=0: valid[i]=False
        seen.add(int(e))
    origin=lla[np.where(valid&~hold)[0][0]]; n0=ned_matrix(*origin[:2]); pos=(ecef(lla)-ecef(origin[None]))@n0.T
    vel=np.column_stack([d[k][idx] for k in ['vel_n_m_s','vel_e_m_s','vel_d_m_s']]);cov=[]
    for i in range(len(t)):
        tr=n0@ned_matrix(*lla[i,:2]).T;vel[i]=tr@vel[i]
        cp=tr@np.diag([d['eph'][idx[i]]**2]*2+[d['epv'][idx[i]]**2])@tr.T
        cp+=np.diag(np.array(cfg['gnss_position_sigma_floor_m'])**2)
        cv=np.eye(3)*(d['s_variance_m_s'][idx[i]]+cfg['gnss_velocity_sigma_floor_m_s']**2)
        c=np.zeros((6,6));c[:3,:3]=cp;c[3:,3:]=cv;cov.append(c.ravel())
    valid &= np.isfinite(vel).all(axis=1)&np.isfinite(cov).all(axis=1)
    save('gnss.csv',[t,pos,vel,np.array(cov),hold.astype(int),valid.astype(int),d['fix_type'][idx],epochs],
         't,px,py,pz,vx,vy,vz,'+','.join(f'c{i}' for i in range(36))+',holdout,valid,fix,utc_usec')
    meta['origin_lla_deg_m']=origin.tolist();meta['excluded']['gnss']=int(sum(~valid))
    meta['gnss_fix_counts']={str(k):int(sum(d['fix_type'][idx]==k)) for k in np.unique(d['fix_type'][idx])}
    d,idx,t=select('vehicle_attitude');assert np.unique(d['quat_reset_counter'][idx]).size==1,'attitude reset'
    quat=np.column_stack([d[f'q[{i}]'][idx] for i in [1,2,3,0]])
    assert np.max(abs(np.linalg.norm(quat,axis=1)-1))<1e-3
    save('attitude_init.csv',[t,quat],'t,qx,qy,qz,qw')
    if ('vehicle_local_position',0) in ds:
        d=ds['vehicle_local_position',0]; idx=np.where((d['timestamp']*1e-6>=lo)&(d['timestamp']*1e-6<=hi))[0]
        for k in ['xy_reset_counter','z_reset_counter','vxy_reset_counter','vz_reset_counter','heading_reset_counter','ref_timestamp']:
            if k in d: assert np.unique(d[k][idx]).size<=1,('EKF reset/reference',k)
    d,idx,t=select('wing_phase'); valid=d['phase_valid'][idx].astype(bool)&np.isfinite(d['phase_rad'][idx])
    save('wing.csv',[t,d['phase_rad'][idx],d['flap_frequency_hz'][idx],valid.astype(int)],'t,phase,frequency,valid')
    meta['selection_history']={k:v.tolist() for k,v in ds['sensor_selection',0].items()}
    # Correlated diagnostic state is never an observation factor.
    for name in ['vehicle_angular_velocity','vehicle_acceleration']:
        d,idx,t=select(name);save(name+'.csv',[t,np.column_stack([d[f'xyz[{i}]'][idx] for i in range(3)])],'t,x,y,z')
    assert sha(path)==spec['sha256'];meta['outputs']={x.name:sha(x) for x in out.glob('*.csv')}
    (out/'metadata.json').write_text(json.dumps(meta,indent=2,ensure_ascii=False)+'\n')
    print(spec['id'],meta['topics'],flush=True)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);ap.add_argument('--config',default='configs/stage02_reconstruction.json');a=ap.parse_args()
    cfg=json.loads(Path(a.config).read_text());root=Path(a.output);root.mkdir(parents=True,exist_ok=False)
    (root/'frozen_config.json').write_text(json.dumps(cfg,indent=2)+'\n')
    proposals=json.loads(Path(cfg['stage01_segments']).read_text())
    for s in proposals['segments']: extract(s,cfg,root/s['id'][0])
    (root/'manifest.json').write_text(json.dumps({'config_sha256':sha(a.config),'extract_code_sha256':sha(__file__),
      'stage01_proposals_sha256':sha(cfg['stage01_segments']),'source':'only three selected raw ULogs; no dataset or model performance',
      'files':{str(p.relative_to(root)):sha(p) for p in root.rglob('*') if p.is_file()}},indent=2)+'\n')


if __name__=='__main__': main()

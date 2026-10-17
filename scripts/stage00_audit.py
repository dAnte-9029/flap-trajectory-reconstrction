#!/usr/bin/env python3
"""Read-only raw-ULog observation inventory. No model datasets or evaluation inputs.

Coverage is supported elapsed time between consecutive valid samples, with gap
sensitivity 2/3/5 times the stream's median positive dt. This is a descriptive
logging-continuity measure, NOT a physical observability threshold. Event topics
must not be interpreted as missing periodic measurements. No tail extrapolation.
"""
import argparse
import csv
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from datetime import datetime, timezone
import numpy as np
from pyulog import ULog
from scipy.signal import welch, coherence

ROOT = Path(__file__).resolve().parents[1]
MAGIC = b'ULog\x01\x12\x35'
IMPORTANT = re.compile(r'^(sensor_|vehicle_(imu|acceleration|angular_velocity|attitude|local_position|global_position|gps_position|land_detected)|estimator_(sensor_bias|selector_status|status|gps_status|event_flags|aid_src_gnss)|wing_phase|encoder_count|flap_frequency|airspeed|differential_pressure|actuator_armed)')
CORE = ['sensor_accel','sensor_gyro','sensor_combined','vehicle_imu','vehicle_acceleration','vehicle_angular_velocity','sensor_gps','vehicle_gps_position','sensor_gnss_relative','vehicle_global_position','vehicle_local_position','vehicle_attitude','wing_phase','encoder_count','flap_frequency']

def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def clean(x):
    if isinstance(x,dict): return {str(k):clean(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)): return [clean(v) for v in x]
    if isinstance(x,np.ndarray): return clean(x.tolist())
    if isinstance(x,np.generic):return clean(x.item())
    if isinstance(x,float) and not np.isfinite(x):return None
    return x

def dump(p,x):p.write_text(json.dumps(clean(x),ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def write_csv(p,rows):
    if not rows:p.write_text('');return
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with p.open('w') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader()
        for r in rows:w.writerow({k:json.dumps(clean(v),ensure_ascii=False) if isinstance(v,(dict,list,tuple)) else clean(v) for k,v in r.items()})
def stats(x):
    x=np.asarray(x,dtype=float);good=x[np.isfinite(x)]
    return dict(n=len(x),finite_fraction=float(len(good)/len(x)) if len(x) else None,
                **(dict(zip(['min','p05','p50','p95','max'],np.quantile(good,[0,.05,.5,.95,1]).tolist())) if len(good) else {}))
def intervals(t,valid=None,mult=3):
    t=np.asarray(t,dtype=float);valid=np.ones(len(t),bool) if valid is None else np.asarray(valid,bool)
    if len(t)<2:return np.empty((0,2))
    dt=np.diff(t);pos=dt[dt>0]
    if not len(pos):return np.empty((0,2))
    ok=(dt>0)&(dt<=mult*np.median(pos))&valid[:-1]&valid[1:]&(t[:-1]>0)
    return merge(np.column_stack((t[:-1][ok],t[1:][ok])))
def merge(a):
    if not len(a):return np.empty((0,2))
    a=sorted(a,key=lambda x:x[0]);out=[list(a[0])]
    for l,r in a[1:]:
        if l<=out[-1][1]:out[-1][1]=max(r,out[-1][1])
        else:out.append([l,r])
    return np.asarray(out,float)
def intersect(a,b):
    out=[];i=j=0
    while i<len(a) and j<len(b):
        l=max(a[i,0],b[j,0]);r=min(a[i,1],b[j,1])
        if r>l:out.append([l,r])
        if a[i,1]<b[j,1]:i+=1
        else:j+=1
    return np.asarray(out,float).reshape(-1,2)
def seconds(a):return float(np.sum(a[:,1]-a[:,0])) if len(a) else 0.
def longest(a):return float(np.max(a[:,1]-a[:,0])) if len(a) else 0.
def time_basis(d):
    if 'timestamp_sample' not in d:return 'timestamp'
    return 'timestamp_sample' if np.any(d['timestamp_sample']>0) else 'timestamp (all sample timestamps zero)'

def time_data(d):
    key='timestamp_sample' if time_basis(d)=='timestamp_sample' else 'timestamp'
    return np.asarray(d[key],float)*1e-6

def mask_for(name,d):
    n=len(d['timestamp']);m=np.ones(n,bool);reason='finite primary fields; no independent accuracy validation'
    keys=[]
    if name in ['sensor_accel','sensor_gyro','sensor_mag']:keys=['x','y','z']
    elif name=='vehicle_imu':keys=[f'{v}[{i}]' for v in ['delta_angle','delta_velocity'] for i in range(3)];m&=(d['delta_angle_dt']>0)&(d['delta_velocity_dt']>0)
    elif name=='sensor_combined':keys=[f'{v}[{i}]' for v in ['gyro_rad','accelerometer_m_s2'] for i in range(3)];m&=d['accelerometer_timestamp_relative']!=2147483647
    elif name in ['vehicle_acceleration','vehicle_angular_velocity']:keys=[f'xyz[{i}]' for i in range(3)]
    elif name in ['sensor_gps','vehicle_gps_position']:
        keys=[k for k in ['latitude_deg','longitude_deg','altitude_msl_m','lat','lon','alt','vel_n_m_s','vel_e_m_s','vel_d_m_s'] if k in d]
        m&=np.isin(d['fix_type'],[3,4,5,6]) & d['vel_ned_valid'].astype(bool);reason='fix_type 3/4/5/6, vel_ned_valid, finite PVT; extrapolated type 8 excluded'
    elif name=='sensor_gnss_relative':
        keys=[f'position[{i}]' for i in range(3)];m&=d['gnss_fix_ok'].astype(bool)&d['relative_position_valid'].astype(bool);reason='gnss_fix_ok and relative_position_valid; heading separate'
    elif name=='vehicle_attitude':
        keys=[f'q[{i}]' for i in range(4)];q=np.array([d[k] for k in keys]);m&=np.abs(np.linalg.norm(q,axis=0)-1)<.01;reason='finite quaternion, norm within .01; not independent truth'
    elif name=='vehicle_local_position':
        keys=['x','y','z','vx','vy','vz'];
        for k in ['xy_valid','z_valid','v_xy_valid','v_z_valid']:m&=d[k].astype(bool)
        reason='all position/velocity validity flags and finite fields; reset edges removed'
    elif name=='vehicle_global_position':
        keys=['lat','lon','alt']
        for k in ['lat_lon_valid','alt_valid']:
            if k in d:m&=d[k].astype(bool)
    elif name=='wing_phase':keys=['phase_rad','phase_unwrapped_rad','flap_frequency_hz'];m&=d['phase_valid'].astype(bool)
    elif name=='flap_frequency':keys=['frequency_hz']
    elif name=='encoder_count':keys=['total_count','position_raw']
    else:reason='timestamp support only; semantic validity not asserted'
    for k in keys:
        if k in d:m&=np.isfinite(d[k])
    # Never bridge a known estimator reset or origin update.
    for k,v in d.items():
        if 'reset_counter' in k or k=='ref_timestamp':
            changed=np.flatnonzero(np.diff(np.asarray(v,dtype=float))!=0)+1;m[changed]=False
    return m,reason

def held_flag(d,t,key,value):
    if d is None:return np.zeros(len(t),bool)
    ts=np.asarray(d['timestamp'],float)*1e-6;ix=np.searchsorted(ts,t,side='right')-1;dt=np.diff(ts);pos=dt[dt>0]
    if not len(pos):return np.zeros(len(t),bool)
    safe=np.clip(ix,0,len(ts)-1)
    return (ix>=0)&(t-ts[safe]<=5*np.median(pos))&(d[key][safe]==value)

def discovery(root):
    records=[]
    for folder,dirs,files in os.walk(root):
        dirs[:]=sorted(d for d in dirs if d!='.git')
        for name in sorted(files):
            p=Path(folder)/name;rel=p.relative_to(root)
            # Header inspection also catches ULogs with a nonstandard extension.
            with p.open('rb') as f:head=f.read(16)
            if p.suffix.lower()!='.ulg' and not head.startswith(MAGIC):continue
            match=re.search(r'2026[-.]0?9[-.](\d{1,2})',str(rel))
            folder_match=re.match(r'9\.(\d{1,2})(?:\D|$)',rel.parts[0])
            day=int(match[1]) if match else int(folder_match[1]) if folder_match else None
            selected=day is not None and 1<=day<=30
            kind='ulog' if head.startswith(MAGIC) else 'appledouble' if head[:4]==b'\x00\x05\x16\x07' else 'lfs_pointer' if head.startswith(b'version https://') else 'not_ulog'
            records.append(dict(path=str(p),relative_path=str(rel),folder=rel.parts[0],bytes=p.stat().st_size,kind=kind,september_candidate=selected,date=f'2026-09-{day:02d}' if selected else '',date_basis='filename_or_year_directory' if match else 'September directory (year archive context)' if folder_match else 'outside scope',sha256=sha(p) if selected else ''))
    return records

def unwrap_finite_phase(values):
    values=np.asarray(values,float);result=np.full_like(values,np.nan)
    finite=np.isfinite(values);edges=np.flatnonzero(np.diff(np.r_[False,finite,False]))
    for start,end in zip(edges[::2],edges[1::2]):result[start:end]=np.unwrap(values[start:end])
    return result

def spectral(datasets,air,outbase,window):
    rows=[];phase=datasets.get(('wing_phase',0))
    if phase is None:return rows
    pt=time_data(phase);pm,_=mask_for('wing_phase',phase)
    active=pm&(phase['flap_frequency_hz']>0)
    pi=intersect(intervals(pt,active),air)
    for name in ['sensor_accel','sensor_gyro','vehicle_angular_velocity']:
        d=datasets.get((name,0))
        if d is None:continue
        t=time_data(d);mask,_=mask_for(name,d);iv=intersect(intervals(t,mask),pi)
        for l,r in iv:
            for start in np.arange(l,r-window,window):
                ix=(t>=start)&(t<start+window);tt=t[ix]
                if len(tt)<32:continue
                dt=np.diff(tt);fs=1/np.median(dt);grid=np.arange(tt[0],tt[-1],1/fs)
                ph=np.interp(grid,pt,unwrap_finite_phase(phase['phase_rad']));freq=np.interp(grid,pt,phase['flap_frequency_hz']);f0=float(np.median(freq))
                # Compare logged frequency with phase derivative, not a priori flap band.
                phase_rate=np.diff(ph)/np.diff(grid)/(2*np.pi)
                for axis in range(3):
                    field='xyz[%d]'%axis if name=='vehicle_angular_velocity' else 'xyz'[axis]
                    y=np.interp(grid,tt,d[field][ix]);nseg=min(len(y),max(16,int(fs*2)))
                    f,p=welch(y,fs,nperseg=nseg);_,c=coherence(y,np.sin(ph),fs,nperseg=nseg)
                    band=(f>=.8*f0)&(f<=1.2*f0);positive=f>0
                    peak=float(f[np.flatnonzero(band)[np.argmax(p[band])]]) if band.any() else None
                    rows.append(dict(**outbase,topic=name,axis=axis,start_s=float(start),duration_s=float(grid[-1]-grid[0]),sample_hz=fs,jitter_cv=float(np.std(dt)/np.mean(dt)),flap_hz=f0,phase_derivative_hz=float(np.median(phase_rate)),samples_per_cycle=fs/f0,max_gap_cycle_fraction=float(max(dt)*max(freq)),flap_band_peak_hz=peak,flap_band_power_fraction=float(p[band].sum()/p[positive].sum()) if p[positive].sum()>0 else None,phase_coherence_at_f0=float(c[np.argmin(abs(f-f0))]),signal_std=float(np.std(y))))
    return rows

def audit_one(rec,out,multipliers,window):
    path=Path(rec['path']);u=ULog(str(path));h=rec['sha256'];base=dict(sha256=h,date=rec['date']);datasets={(d.name,d.multi_id):d.data for d in u.data_list}
    land=datasets.get(('vehicle_land_detected',0));armed=datasets.get(('actuator_armed',0))
    # Report marked airborne only where both recorded conditions are supported.
    if land is not None:
        lt=time_data(land);am=(land['landed']==0)&held_flag(armed,lt,'armed',1);air=intervals(lt,am)
    else:air=np.empty((0,2))
    topics=[];fields=[];timings=[];qualities=[];supports={};clipping=[]
    logspan=(u.last_timestamp-u.start_timestamp)*1e-6
    for topic in u.data_list:
        name=topic.name;instance=topic.multi_id;d=topic.data;ident=dict(**base,topic=name,instance=instance)
        t=time_data(d);pub=np.asarray(d['timestamp'],float)*1e-6
        if name=='sensor_gnss_relative':t=pub.copy()  # CAN remote sample clock has no verified HRT transform
        basis='timestamp (relative GNSS remote clock unverified)' if name=='sensor_gnss_relative' else time_basis(d)
        dt=np.diff(t);positive=dt[dt>0];med=np.median(positive) if len(positive) else np.nan
        valid,rule=mask_for(name,d);valid&=t>0
        iv=intervals(t,valid);supports[(name,instance)]=iv
        ivair=intersect(iv,air)
        missing=float(np.maximum(np.rint(positive/med)-1,0).sum()) if len(positive) else 0
        row=dict(**ident,rows=len(t),fields=list(d),time_basis=basis,first_s=float(t[0]),last_s=float(t[-1]),span_s=float(t[-1]-t[0]),median_hz=1/med if med>0 else None,mean_unique_hz=float(len(positive)/(t[-1]-t[0])) if t[-1]>t[0] else None,dt_p50_s=med,dt_p95_s=float(np.quantile(positive,.95)) if len(positive) else None,max_gap_s=float(max(positive)) if len(positive) else None,duplicates=int((dt==0).sum()),backwards=int((dt<0).sum()),zero_timestamps=int((t==0).sum()),estimated_missing_slots=missing,estimated_missing_fraction=missing/(missing+len(t)),valid_rows=int(valid.sum()),valid_fraction=float(valid.mean()),validity_rule=rule,supported_s=seconds(iv),airborne_supported_s=seconds(ivair),longest_supported_s=longest(iv),longest_airborne_s=longest(ivair),span_fraction_of_log=float((t[-1]-t[0])/logspan),supported_fraction_of_log=seconds(iv)/logspan)
        for mult in multipliers:row[f'supported_s_gap{mult}']=seconds(intervals(t,valid,mult))
        topics.append(row)
        raw_sample=np.asarray(d.get('timestamp_sample',[]),float)*1e-6
        timing=dict(**ident,time_basis=basis,publication_dt=stats(np.diff(pub)),selected_clock_dt=stats(dt),raw_sample_dt=stats(np.diff(raw_sample)),raw_sample_zero_rows=int((raw_sample==0).sum()),publication_minus_sample_s=stats((pub-raw_sample)[raw_sample>0]) if len(raw_sample) else stats([]),negative_latency_rows=int(((pub<raw_sample)&(raw_sample>0)).sum()) if len(raw_sample) else 0)
        for field in ['timestamp_time_relative','accelerometer_timestamp_relative','time_utc_usec']:
            if field in d:timing[field]=stats(d[field])
        timings.append(timing)
        for field,v in d.items():
            fr=dict(**ident,field=field,dtype=str(v.dtype),finite_fraction=float(np.isfinite(v).mean()))
            if IMPORTANT.match(name):
                fr.update(stats(v))
                if v.dtype.kind in 'bu' and len(np.unique(v))<=24:fr['counts']={str(k):int(n) for k,n in zip(*np.unique(v,return_counts=True))}
                if any(z in field for z in ['reset_counter','calibration_count','device_id']) or field=='ref_timestamp':fr['transitions']=int(np.count_nonzero(np.diff(v.astype(float))))
            fields.append(fr)
        for field in ['delta_angle_dt','delta_velocity_dt','gyro_integral_dt','accelerometer_integral_dt']:
            if field in d:
                duration=np.asarray(d[field],float)*1e-6;ends=t.copy()
                if field=='accelerometer_integral_dt':ends+=np.asarray(d['accelerometer_timestamp_relative'],float)*1e-6
                good=(duration>0)&(ends>duration)&valid
                integration=merge(np.column_stack((ends[good]-duration[good],ends[good])))
                qualities.append(dict(**ident,metric=field,value=stats(duration),union_integration_s=seconds(integration),log_fraction=seconds(integration)/logspan,airborne_integration_s=seconds(intersect(integration,air)),median_dt_to_logged_spacing_ratio=float(np.median(duration)/med),note='union of actual [sample_end-integration_dt,sample_end]; NOT gaps filled'))
        if name in ['sensor_gps','vehicle_gps_position']:
            for label,m in [('pvt',valid),('rtk_fixed',valid&(d['fix_type']==6)),('rtk_float',valid&(d['fix_type']==5)),('heading_finite',valid&np.isfinite(d['heading']))]:
                qiv=intervals(t,m);qualities.append(dict(**ident,metric=label,rows=int(m.sum()),supported_s=seconds(qiv),airborne_s=seconds(intersect(qiv,air)),longest_s=longest(qiv)))
            for field in ['eph','epv','s_variance_m_s','hdop','vdop','heading_accuracy','satellites_used']:
                qualities.append(dict(**ident,metric=field,value=stats(d[field][valid]),airborne_value=stats(d[field][valid&held_flag(land,t,'landed',0)&held_flag(armed,t,'armed',1)])))
        if name=='sensor_gnss_relative':
            for label,m in [('relative_valid',valid),('heading_valid',valid&d['heading_valid'].astype(bool)&np.isfinite(d['heading'])),('carrier_fixed',valid&d['carrier_solution_fixed'].astype(bool))]:
                qiv=intervals(t,m);qualities.append(dict(**ident,metric=label,rows=int(m.sum()),supported_s=seconds(qiv),airborne_s=seconds(intersect(qiv,air)),longest_s=longest(qiv)))
        if name in ['wing_phase','flap_frequency']:
            ffield='flap_frequency_hz' if name=='wing_phase' else 'frequency_hz';act=valid&(d[ffield]>0);actair=act&held_flag(land,t,'landed',0)&held_flag(armed,t,'armed',1)
            qualities.append(dict(**ident,metric='active_frequency_hz',value=stats(d[ffield][act]),airborne_value=stats(d[ffield][actair]),supported_s=seconds(intervals(t,act)),airborne_s=seconds(intersect(intervals(t,act),air))))
        if name=='vehicle_attitude':qualities.append(dict(**ident,metric='quaternion_norm',value=stats(np.linalg.norm(np.array([d[f'q[{i}]'] for i in range(4)]),axis=0))))
    # Exact support intersection; keep every interval for traceable coverage claims.
    combinations={
        'main_accel_gyro': [('sensor_accel',0),('sensor_gyro',0)],
        'main_accel_gyro_gnss_pvt':[('sensor_accel',0),('sensor_gyro',0),('sensor_gps',0)],
        'main_accel_gyro_vehicle_gps_pvt':[('sensor_accel',0),('sensor_gyro',0),('vehicle_gps_position',0)],
        'state_R_omega_pv':[('vehicle_attitude',0),('vehicle_angular_velocity',0),('vehicle_local_position',0)]}
    overlap={};segments=[]
    for label,keys in combinations.items():
        iv=supports.get(keys[0],np.empty((0,2)))
        for key in keys[1:]:iv=intersect(iv,supports.get(key,np.empty((0,2))))
        ai=intersect(iv,air);overlap[label]=dict(supported_s=seconds(iv),longest_s=longest(iv),airborne_s=seconds(ai),longest_airborne_s=longest(ai),segments=len(iv))
        for scope,ints in [('all',iv),('airborne',ai)]:
            for l,r in ints:segments.append(dict(**base,combination=label,scope=scope,start_s=l,end_s=r,duration_s=r-l))
    for left,right in [('vehicle_attitude','vehicle_angular_velocity'),('vehicle_local_position','vehicle_attitude'),('sensor_accel','sensor_gyro'),('sensor_gps','vehicle_local_position'),('sensor_gps','vehicle_gps_position'),('vehicle_gps_position','vehicle_local_position'),('wing_phase','sensor_gyro')]:
        a=datasets.get((left,0));b=datasets.get((right,0))
        if a is None or b is None:continue
        ta=time_data(a);tb=time_data(b);inside=(ta>=tb[0])&(ta<=tb[-1]);ta=ta[inside];idx=np.searchsorted(tb,ta);idx=np.clip(idx,1,len(tb)-1)
        nearest=np.where(abs(ta-tb[idx-1])<=abs(ta-tb[idx]),idx-1,idx)
        timings.append(dict(**base,topic=left,instance=0,comparison=right,nearest_sample_offset_s=stats(ta-tb[nearest]),previous_sample_age_s=stats(ta-tb[np.maximum(0,np.searchsorted(tb,ta,side='right')-1)]),note='within overlapping span; descriptive nearest timing, no same-row synchronization assumed'))
    pars={k:v for k,v in u.initial_parameters.items() if any(k.startswith(z) for z in ['CAL_','SENS_','IMU_','EKF2_','GPS_','SDLOG_','FW_WING','FLAP','ENC','WING'])}
    meta=dict(**base,path=str(path),log_span_s=logspan,marked_airborne_s=seconds(air),airborne_definition='landed==false AND armed==true; recorded flag support, not independent flight truth',topics=len(u.data_list),info=u.msg_info_dict,parameters=pars,changed_parameters=[x for x in u.changed_parameters if x[1] in pars],dropouts=[dict(timestamp=x.timestamp,duration_ms=x.duration) for x in u.dropouts],has_sync=getattr(u,'_has_sync',None),overlap=overlap,missing_core=[k for k in CORE if (k,0) not in datasets],raw_sha_after=sha(path))
    assert meta['raw_sha_after']==h, f'Raw source changed: {path}'
    dump(out/'logs'/f'{h}.json',meta)
    return topics,fields,timings,qualities,segments,spectral(datasets,air,base,window),meta

def git_info(path):
    def g(*args):return subprocess.check_output(['git','-C',str(path),*args],text=True).strip()
    return dict(path=str(path),commit=g('rev-parse','HEAD'),branch=g('branch','--show-current'),status=g('status','--porcelain'),remotes=g('remote','-v'))

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',default='configs/stage00_audit.json');args=ap.parse_args();cfgpath=Path(args.config);cfg=json.loads(cfgpath.read_text());out=ROOT/cfg['output'];out.mkdir(parents=True,exist_ok=False);(out/'logs').mkdir()
    start=datetime.now(timezone.utc).isoformat();found=discovery(Path(cfg['ulog_root']));write_csv(out/'discovery.csv',found)
    inv=[r for r in found if r['september_candidate']];histpath=Path(cfg['historical_identity_csv']);history=list(csv.DictReader(histpath.open()));byhash={}
    for r in history:byhash.setdefault(r['sha256'],[]).append({k:r[k] for k in ['relative_path','registered_role','role','flight_id','current_status'] if k in r})
    unique={}
    for r in inv:
        r['historical_identity_annotations']=byhash.get(r['sha256'],[]);r['exposure']='raw observation audit authorized 2026-10-17; model performance not accessed'
        if r['kind']=='ulog':unique.setdefault(r['sha256'],r)
        r['canonical_path']=unique[r['sha256']]['path'] if r['kind']=='ulog' else '';r['duplicate']=r['kind']=='ulog' and r['path']!=r['canonical_path']
    write_csv(out/'inventory.csv',inv)
    tables={k:[] for k in ['topic_coverage','field_statistics','timing','quality','continuous_segments','spectral_windows']};metas=[]
    for i,r in enumerate(unique.values()):
        print(f'{i+1}/{len(unique)} {r["relative_path"]}',flush=True)
        result=audit_one(r,out,cfg['gap_median_multipliers'],cfg['spectral_window_seconds'])
        for k,rows in zip(tables,result[:-1]):tables[k].extend(rows)
        metas.append(result[-1])
    for k,rows in tables.items():write_csv(out/f'{k}.csv',rows)
    perlog=[]
    for m in metas:
        row={k:m[k] for k in ['sha256','date','path','log_span_s','marked_airborne_s','topics','missing_core']};row['firmware']=m['info'].get('ver_sw');row['ulog_dropout_count']=len(m['dropouts']);row['ulog_dropout_total_ms']=sum(x['duration_ms'] for x in m['dropouts'])
        for k,v in m['overlap'].items():row.update({f'{k}_{a}':b for a,b in v.items()})
        perlog.append(row)
    write_csv(out/'per_log.csv',perlog)
    import pandas as pd
    tf=pd.DataFrame(tables['topic_coverage']);summ=[]
    for date in ['ALL',*sorted(set(tf.date))]:
        sub=tf if date=='ALL' else tf[tf.date==date]
        for (topic,instance),g in sub.groupby(['topic','instance']):summ.append(dict(date=date,topic=topic,instance=int(instance),unique_logs=len(g),logs_with_valid_support=int((g.supported_s>0).sum()),logs_with_airborne_support=int((g.airborne_supported_s>0).sum()),rows=int(g.rows.sum()),supported_s=float(g.supported_s.sum()),airborne_supported_s=float(g.airborne_supported_s.sum()),hz_min=float(g.median_hz.min()),hz_median=float(g.median_hz.median()),hz_max=float(g.median_hz.max()),worst_gap_s=float(g.max_gap_s.max())))
    write_csv(out/'date_topic_summary.csv',summ)
    lf=pd.DataFrame(perlog);lf.groupby('date').agg(unique_logs=('sha256','size'),log_span_s=('log_span_s','sum'),marked_airborne_s=('marked_airborne_s','sum'),imu_gnss_supported_s=('main_accel_gyro_vehicle_gps_pvt_supported_s','sum'),imu_gnss_airborne_s=('main_accel_gyro_vehicle_gps_pvt_airborne_s','sum')).to_csv(out/'date_summary.csv')
    manifest=dict(start_utc=start,end_utc=datetime.now(timezone.utc).isoformat(),command=sys.argv,configuration=cfg,code_sha256=sha(Path(__file__)),config_sha256=sha(cfgpath),versions={k:importlib.metadata.version(k) for k in ['pyulog','numpy','scipy','pandas']},python=sys.version,repositories={name:git_info(path) for name,path in [('audit',ROOT),('logs',cfg['ulog_root']),('gptr','/home/zn/gptr'),('gnssFGO','/home/zn/gnssFGO'),('px4','/home/zn/PX4-Autopilot')]},historical_identity=dict(path=str(histpath),sha256=sha(histpath),usage='annotation only; not authoritative new split; no performance columns read'),counts=dict(discovered_log_named_or_magic_files=len(found),september_candidates=len(inv),ulog_paths=sum(r['kind']=='ulog' for r in inv),unique_ulogs=len(unique),non_ulogs=sum(r['kind']!='ulog' for r in inv),parsed_logs=len(metas),topic_instances=len(tf)),raw_hashes_verified_after_parse=True,model_performance_accessed=False,optimization_run=False,papers={str(p.relative_to(ROOT)):sha(p) for p in (ROOT/'docs/papers').glob('*.pdf')},outputs={str(p.relative_to(out)):sha(p) for p in sorted(out.rglob('*')) if p.is_file()})
    dump(out/'manifest.json',manifest);print(json.dumps(manifest['counts']),flush=True)
if __name__=='__main__':main()

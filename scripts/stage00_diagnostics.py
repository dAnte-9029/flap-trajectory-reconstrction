#!/usr/bin/env python3
"""Raw observation-only follow-up: timestamp epochs, selected sensor, rest scatter,
cycle bandwidth and source overlap. Uses discovery SHA list, never model data.
No optimization, labels, or precision claims. Original cycle-reset phase is
unwrapped on S1 before temporal interpolation for descriptive spectra.
"""
import argparse
import json
from pathlib import Path
import csv
from datetime import datetime,timezone
import numpy as np
from pyulog import ULog
import stage00_audit as a


def run(config):
    cp=Path(config);cfg=json.loads(cp.read_text());root=a.ROOT/cfg['output'];out=root.parent/'diagnostics_final';out.mkdir(exist_ok=False)
    inventory=list(csv.DictReader((root/'inventory.csv').open()));logs={r['sha256']:r for r in inventory if r['kind']=='ulog'}
    rows=[];spectra=[];segments=[];boundaries=[]
    for i,(h,r) in enumerate(logs.items()):
        print(f'{i+1}/{len(logs)} {r["relative_path"]}',flush=True)
        u=ULog(r['path']);ds={(d.name,d.multi_id):d.data for d in u.data_list};base=dict(sha256=h,date=r['date'])
        for (name,instance),data in ds.items():
            if name not in a.CORE:continue
            tt=a.time_data(data) if name!='sensor_gnss_relative' else np.asarray(data['timestamp'],float)*1e-6
            dd=np.diff(tt);positive=dd[dd>0]
            if len(positive):
                for j in np.flatnonzero((dd>3*np.median(positive))|(dd<=0)):
                    boundaries.append(dict(**base,topic=name,instance=instance,event='clock_or_gap',field='selected_time',start_s=tt[j],end_s=tt[j+1],dt_s=dd[j]))
            for key,val in data.items():
                if 'reset_counter' in key or key=='ref_timestamp':
                    for j in np.flatnonzero(np.diff(np.asarray(val,float))!=0):
                        boundaries.append(dict(**base,topic=name,instance=instance,event='reset_or_origin_change',field=key,start_s=tt[j],end_s=tt[j+1],before=float(val[j]),after=float(val[j+1])))
        status=ds.get(('vehicle_status',0))
        if status is not None:
            rows.append(dict(**base,topic='vehicle_status',metric='flight_modes',nav_state_counts={str(k):int(n) for k,n in zip(*np.unique(status['nav_state'],return_counts=True))},nav_state_transitions=int(np.count_nonzero(np.diff(status['nav_state'].astype(float))))))
        land=ds.get(('vehicle_land_detected',0));armed=ds.get(('actuator_armed',0));air=np.empty((0,2))
        if land is not None:
            lt=a.time_data(land);air=a.intervals(lt,(land['landed']==0)&a.held_flag(armed,lt,'armed',1))
        gps=ds.get(('vehicle_gps_position',0));rawgps=ds.get(('sensor_gps',0))
        if gps is not None:
            t=a.time_data(gps);m,_=a.mask_for('vehicle_gps_position',gps);iv=a.intervals(t,m)
            for name in ['sensor_accel','sensor_gyro']:
                d=ds[(name,0)];sm,_=a.mask_for(name,d);iv=a.intersect(iv,a.intervals(a.time_data(d),sm))
            ai=a.intersect(iv,air)
            rows.append(dict(**base,topic='joint_main_imu_vehicle_gps',metric='support',supported_s=a.seconds(iv),airborne_s=a.seconds(ai),longest_airborne_s=a.longest(ai),longest_s=a.longest(iv)))
            for scope,ints in [('all',iv),('airborne',ai)]:
                for l,e in ints:segments.append(dict(**base,scope=scope,start_s=l,end_s=e,duration_s=e-l))
        for name in ['sensor_gps','vehicle_gps_position','sensor_gnss_relative']:
            d=ds.get((name,0))
            if d is None:continue
            if 'heading' in d:
                hd=np.asarray(d['heading'],float);ha=np.asarray(d['heading_accuracy'],float);bounded=np.isfinite(hd)&(abs(hd)<=(2*np.pi if name=='sensor_gnss_relative' else np.pi)+1e-5);plausible=bounded&np.isfinite(ha)&(ha>0)&(ha<=np.pi)
                rows.append(dict(**base,topic=name,metric='heading_sanity',rows=len(hd),finite_heading_rows=int(np.isfinite(hd).sum()),bounded_heading_rows=int(bounded.sum()),positive_bounded_accuracy_rows=int(plausible.sum()),heading_valid_true_rows=int(np.count_nonzero(d.get('heading_valid',[]))),positive_accuracy=a.stats(ha[plausible]),relative_flag_counts={key:int(np.count_nonzero(d[key])) for key in ['gnss_fix_ok','relative_position_valid','carrier_solution_fixed','heading_valid'] if key in d},note='sanity screen, not proof of valid heading or calibrated covariance'))
            t=a.time_data(d);utc=np.asarray(d['time_utc_usec'],float);dt=np.diff(utc);nonzero=utc>0
            rows.append(dict(**base,topic=name,metric='utc_epoch',nonzero_rows=int(nonzero.sum()),distinct_nonzero_epochs=len(np.unique(utc[nonzero])),repeated_adjacent_epochs=int((dt==0).sum()),backward_epochs=int((dt<0).sum()),utc_step_s=a.stats(dt[dt>0]*1e-6),utc_min=int(utc[nonzero].min()) if nonzero.any() else None,utc_max=int(utc[nonzero].max()) if nonzero.any() else None))
        if gps is not None and rawgps is not None:
            gt=a.time_data(gps);rt=a.time_data(rawgps);idx=np.searchsorted(gt,rt);idx=np.clip(idx,1,len(gt)-1);idx=np.where(abs(rt-gt[idx-1])<abs(rt-gt[idx]),idx-1,idx)
            keys=['latitude_deg','longitude_deg','altitude_msl_m','vel_n_m_s','vel_e_m_s','vel_d_m_s']
            rows.append(dict(**base,topic='sensor_gps_vs_vehicle_gps_position',metric='nearest_sample_comparison',device_ids_equal=bool(np.array_equal(np.unique(gps['device_id']),np.unique(rawgps['device_id']))),sample_offset_s=a.stats(rt-gt[idx]),utc_exact_fraction=float(np.mean(gps['time_utc_usec'][idx]==rawgps['time_utc_usec'])),field_equal_fraction={k:float(np.mean(np.isclose(gps[k][idx],rawgps[k],atol=0,rtol=0,equal_nan=True))) for k in keys if k in gps and k in rawgps}))
        phase=ds.get(('wing_phase',0));freq=ds.get(('flap_frequency',0))
        if phase is not None:
            t=a.time_data(phase);v,_=a.mask_for('wing_phase',phase);airmask=a.held_flag(land,t,'landed',0)&a.held_flag(armed,t,'armed',1)
            # Per-cycle reset makes phase_unwrapped_rad unsuitable for direct interpolation.
            dp=np.diff(np.asarray(phase['phase_unwrapped_rad'],float));rows.append(dict(**base,topic='wing_phase',metric='phase_resets',backward_jumps_gt_pi=int((dp<-np.pi).sum()),forward_jumps_gt_pi=int((dp>np.pi).sum()),valid_fraction=float(v.mean()),airborne_valid_fraction=float(v[airmask].mean()) if airmask.any() else None))
            # spectral() unwraps each finite circular phase run separately.
            for name in ['sensor_accel','sensor_gyro','vehicle_angular_velocity','vehicle_attitude','vehicle_local_position','vehicle_gps_position','sensor_gps']:
                d=ds.get((name,0))
                if d is None:continue
                st=a.time_data(d);sf=np.interp(st,t,phase['flap_frequency_hz']);mask=a.held_flag(land,st,'landed',0)&a.held_flag(armed,st,'armed',1)&(sf>0)&a.held_flag({'timestamp':phase['timestamp'],'phase_valid':v},st,'phase_valid',1)
                dt=np.diff(st);ok=mask[:-1]&mask[1:]&(dt>0)
                rows.append(dict(**base,topic=name,metric='airborne_cycle_sampling',flap_hz=a.stats(sf[mask]),dt_in_cycles=a.stats(dt[ok]*sf[1:][ok]),samples_per_cycle=a.stats(1/(dt[ok]*sf[1:][ok]))))
            spectra.extend(a.spectral(ds,air,base,cfg['spectral_window_seconds']))
        sel=ds.get(('sensor_selection',0))
        for name in ['sensor_accel','sensor_gyro']:
            d=ds[(name,0)];t=a.time_data(d);valid,_=a.mask_for(name,d)
            rest=a.held_flag(land,t,'at_rest',1) if land is not None and 'at_rest' in land else np.zeros(len(t),bool)
            if freq is not None:rest&=a.held_flag(freq,t,'frequency_hz',0)
            rest&=valid
            clip=np.any(np.array([d[f'clip_counter[{j}]']>0 for j in range(3)]),axis=0)
            match=None
            if sel is not None:
                sk='accel_device_id' if name=='sensor_accel' else 'gyro_device_id';st=a.time_data(sel);ii=np.searchsorted(st,t,side='right')-1;ok=ii>=0;match=float(np.mean(d['device_id'][ok]==sel[sk][np.maximum(ii[ok],0)])) if ok.any() else None
            rows.append(dict(**base,topic=name,metric='sensor_quality',rest_proxy_rows=int(rest.sum()),rest_proxy_supported_s=a.seconds(a.intervals(t,rest)),rest_proxy_std={axis:float(np.std(d[axis][rest])) if rest.any() else None for axis in 'xyz'},rest_proxy_robust_scatter={axis:float(1.4826*np.median(abs(d[axis][rest]-np.median(d[axis][rest])))) if rest.any() else None for axis in 'xyz'},clipped_rows=int(clip.sum()),clipped_fraction=float(clip.mean()),hardware_error_count_change=int(d['error_count'][-1])-int(d['error_count'][0]),samples_per_logged_message=a.stats(d['samples']),selected_device_match_fraction=match,note='Rest proxy scatter includes vibration/handling and bias drift; not calibrated white-noise density. Selection last-held event has no timeout.'))
        assert a.sha(r['path'])==h
    a.write_csv(out/'boundary_events.csv',boundaries);a.write_csv(out/'diagnostics.csv',rows);a.write_csv(out/'spectral_windows.csv',spectra);a.write_csv(out/'joint_imu_gnss_segments.csv',segments)
    a.dump(out/'manifest.json',dict(completed_utc=datetime.now(timezone.utc).isoformat(),configuration=cfg,code_sha256=a.sha(Path(__file__)),audit_library_sha256=a.sha(Path(a.__file__)),raw_hashes=list(logs),all_raw_hashes_rechecked=True,phase_method='unwrap phase_rad on circle before interpolation; unwrap finite runs separately, no interpolation across invalid phase support; exploratory runs superseded',outputs={p.name:a.sha(p) for p in out.glob('*.csv')}))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--config',default='configs/stage00_audit.json');run(p.parse_args().config)

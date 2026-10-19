#!/usr/bin/env python3
"""Freeze protocol, read pinned PX4 sources, verify known ULog identity and ratio."""
import json
import subprocess
from pathlib import Path
from pyulog import ULog
from stage02_3A_reconstruct import sha, dump


def main():
    cfg=json.load(open('configs/stage02_3A_phase.json'));out=Path(cfg['output'])
    out.mkdir(exist_ok=False);(out/'reference').mkdir()
    dump(out/'frozen_config.json',cfg)
    # Subsequent reproductions include the documented post-first-run numerical
    # integration check; it does not change fitting or parameter selection.
    followup=Path('configs/stage02_3A_integration_followup.json')
    (out/'integration_followup_plan.json').write_text(followup.read_text())
    files=['src/drivers/encoder/as5600/AS5600.cpp','src/drivers/encoder/as5600/AS5600Math.hpp',
           'src/modules/wing_phase/WingPhase.cpp','src/modules/wing_phase/WingPhaseMath.cpp',
           'src/modules/wing_phase/WingPhaseMath.hpp']
    refs=[];params={}
    for seg in cfg['segments']:
        meta=json.load(open(Path(cfg['source'])/'phase_raw'/seg/'metadata.json'))
        for filename in files:
            content=subprocess.check_output(['git','-C',cfg['px4_repo'],'show',meta['firmware']+':'+filename])
            p=out/'reference'/(seg+'_'+Path(filename).name);p.write_bytes(content)
            refs.append(dict(segment=seg,commit=meta['firmware'],path=filename,snapshot=str(p),sha256=sha(p)))
        actual_sha=sha(meta['input_path']);assert actual_sha==meta['source_sha256']
        u=ULog(meta['input_path'],message_name_filter_list=['encoder_count','wing_phase','hall_event','flap_frequency'])
        changes=[dict(timestamp=int(t),value=float(v)) for t,k,v in u.changed_parameters if k=='FLAP_RATIO']
        lo=meta['segment']['start_s']-2;hi=meta['segment']['end_s']+2;r=u.initial_parameters['FLAP_RATIO']
        for change in changes:
            if change['timestamp']*1e-6<=lo:r=change['value']
        assert r==meta['parameters']['FLAP_RATIO']
        assert not any(lo<change['timestamp']*1e-6<=hi for change in changes)
        params[seg]=dict(raw_sha256=actual_sha,effective_ratio=r,all_ratio_changes=changes,
                        hall_event_present=any(d.name=='hall_event' for d in u.data_list),
                        read_scope='two known logs, metadata and four phase topics only; no new broad statistics')
    dump(out/'reference'/'manifest.json',refs);dump(out/'parameter_audit.json',params)


if __name__=='__main__':main()

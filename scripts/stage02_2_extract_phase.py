#!/usr/bin/env python3
"""Targeted mechanical phase evidence only, preserving the Stage02 observations."""
import csv,json,hashlib
from pathlib import Path
import numpy as np
from pyulog import ULog


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()

def main():
    cfg=json.load(open('configs/stage02_2_phase_feasibility.json'));root=Path(cfg['source_root']);out=Path(cfg['output_root'])/'phase_raw';out.mkdir(exist_ok=False)
    metadata={}
    for seg in cfg['segments']:
        m=json.load(open(root/'observations'/seg/'metadata.json'));assert sha(m['input_path'])==m['sha256']
        u=ULog(m['input_path'],message_name_filter_list=['wing_phase','encoder_count','hall_event','flap_frequency'])
        dest=out/seg;dest.mkdir();meta={'source_sha256':m['sha256'],'input_path':m['input_path'],'firmware':m['firmware'],'segment':m['segment'],'topics':{},'parameters':{k:v for k,v in u.initial_parameters.items() if 'FLAP' in k or 'HALL' in k}}
        for d in u.data_list:
            t=d.data['timestamp']*1e-6;use=(t>=m['segment']['start_s']-2)&(t<=m['segment']['end_s']+2)
            fields=list(d.data);name=d.name+'_'+str(d.multi_id)+'.csv'
            with (dest/name).open('w') as f:
                w=csv.writer(f);w.writerow(fields)
                for i in np.flatnonzero(use):w.writerow([d.data[k][i] for k in fields])
            meta['topics'][name]={'rows':int(sum(use)),'fields':fields,'sha256':sha(dest/name)}
        (dest/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n');metadata[seg]=meta
        print(seg,[(k,v['rows']) for k,v in meta['topics'].items()])
    (out/'manifest.json').write_text(json.dumps({'sources':metadata,'script_sha256':sha(__file__),'config_sha256':sha('configs/stage02_2_phase_feasibility.json')},indent=2)+'\n')

if __name__=='__main__':main()

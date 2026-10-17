#!/usr/bin/env python3
"""Hash the Stage02.2 evidence and verify prior deliveries and read-only inputs."""
import datetime,importlib.metadata,json,subprocess
from pathlib import Path
from stage02_2_extract_phase import sha


def git(path,*args):return subprocess.check_output(['git','-C',str(path),*args],text=True).strip()


def main():
    cfg=json.load(open('configs/stage02_2_phase_feasibility.json'));out=Path(cfg['output_root']);assert git('.','branch','--show-current')=='reconstruction-audit'
    assert sha('configs/stage02_2_phase_feasibility.json')==sha(out/'frozen_config_phase_proxy.json')
    previous={};inputs={};upstream={}
    for stage in ['stage02_20261017','stage02_1_20261017']:
        root=Path('runs')/stage;m=json.load(open(root/'delivery_manifest.json'))
        for path,h in m['artifact_sha256'].items():assert sha(root/path)==h,(stage,path)
        previous[stage]={'manifest_sha256':sha(root/'delivery_manifest.json'),'git_head':m['git_head'],'verified_unchanged_artifacts':len(m['artifact_sha256'])}
    for seg in cfg['segments']:
        src=Path(cfg['source_root'])/'observations'/seg;meta=json.load(open(src/'metadata.json'));assert sha(meta['input_path'])==meta['sha256']
        inputs[meta['input_path']]=meta['sha256']
        for p in src.glob('*'):
            if p.is_file():inputs[str(p)]=sha(p)
    paper=json.load(open(out/'reference/paper.json'));assert sha(paper['path'])==paper['sha256'];inputs[paper['path']]=paper['sha256']
    old=json.load(open(Path(cfg['source_root'])/'delivery_manifest.json'))
    for name in ['gptr','gnssFGO']:
        path=Path('/home/zn')/name;upstream[name]={'head':git(path,'rev-parse','HEAD'),'status':git(path,'status','--porcelain')};assert upstream[name]==old['upstream'][name]
    public=json.load(open(out/'reference/upstream_commit.json'));upstream['Qian']={'head':public['commit'],'repository':public['url'],'access':'Read-only local snapshots; no known redistribution license'}
    for r in json.load(open(out/'reference/upstream_sources.json')):assert sha(out/'reference/upstream'/r['path'])==r['sha256']
    fits=json.load(open(out/'regression/fit_manifest.json'))
    assert len(fits)==88
    for name,m in fits.items():
        assert set(m['train_cycles']).isdisjoint(m['held_cycles'])
        if 'forecast' in name:assert max(m['train_cycles'])<min(m['held_cycles'])
    paths=list(Path('experiments/stage02_2').glob('*.py'))+[Path('experiments/stage02_2/README.md')]+list(Path('scripts').glob('stage02_2_*.py'))+[Path('configs/stage02_2_phase_feasibility.json'),Path('docs/STATUS.md'),Path('docs/reports/02_2_phase_aware_reconstruction.md')]
    remote=json.load(open(out/'remote_verification.json')) if (out/'remote_verification.json').exists() else None
    manifest={'stage':'02.2','decision':'CONDITIONAL for phase-aware prototype; NO-GO for direct whole-body labels/Stage03','scope':'88 sensor-regression combinations, targeted A/B mechanical phase extraction, synthetic composition and ambiguity checks; zero new GPTR solves',
      'sealed_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'git_head':git('.','rev-parse','HEAD'),'git_status_at_seal':git('.','status','--porcelain'),'code_and_report_sha256':{str(p):sha(p) for p in paths},
      'input_sha256':inputs,'previous_deliveries':previous,'upstream':upstream,'python_packages':{p:importlib.metadata.version(p) for p in ['numpy','scipy','matplotlib','pyulog','requests']},
      'synthetic_checks':json.load(open(out/'synthetic/checks.json')),'focused_checks':json.load(open(out/'focused_checks/checks.json')),'current_fit_membership_check':{'groups':len(fits),'passed':True},
      'baseline_correction':json.load(open(out/'baseline_weight_correction.json')),'paper_preserved_untracked':paper['path'],'remote_verification':remote,
      'artifact_sha256':{str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file() and p.name!='delivery_manifest.json'},
      'formal_labels':False,'neural_training':False,'real_phase_joint_optimization':False,'large_artifacts_tracked':False}
    (out/'delivery_manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
    print('Sealed',len(manifest['artifact_sha256']),'local files;',previous,'; commit',manifest['git_head'])

if __name__=='__main__':main()

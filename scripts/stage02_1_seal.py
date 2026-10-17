#!/usr/bin/env python3
"""Seal local evidence and verify immutable Stage02 inputs; never edit source data."""
import importlib.metadata
import json
from pathlib import Path
import subprocess
from stage02_1_residuals import sha


def git(path, *args):
    return subprocess.check_output(['git','-C',str(path),*args],text=True).strip()


def main():
    cfg=json.load(open('configs/stage02_1_diagnostics.json'));root=Path(cfg['source_root']);out=Path(cfg['output_root'])
    original=json.load(open(root/'delivery_manifest.json'))
    assert git('.', 'branch','--show-current')=='reconstruction-audit'
    assert sha(out/'frozen_config.json')==sha('configs/stage02_1_diagnostics.json')
    assert sha(out/'frozen_offgrid_check.json')==sha('configs/stage02_1_offgrid_check.json')
    for path,digest in original['artifact_sha256'].items():assert sha(root/path)==digest,path
    for path,digest in original['binaries'].items():assert sha(root/path)==digest,path
    # The adapters actually reused here are unchanged from the Stage02 delivery.
    for path,digest in original['code_and_report_sha256'].items():
        if path.startswith('experiments/stage01/') or path.startswith('experiments/stage02/'):
            assert sha(path)==digest,path
    sources=list(Path('experiments/stage02_1').glob('*'))+list(Path('scripts').glob('stage02_1_*.py'))+list(Path('configs').glob('stage02_1_*.json'))+[Path('docs/STATUS.md'),Path('docs/reports/02_1_reconstruction_diagnostics.md')]
    upstream={}
    for name,path in [('gptr','/home/zn/gptr'),('gnssFGO','/home/zn/gnssFGO')]:
        upstream[name]={'head':git(path,'rev-parse','HEAD'),'status':git(path,'status','--porcelain')}
        assert upstream[name]==original['upstream'][name],name
    provenance=out/'provenance';provenance.mkdir(exist_ok=True)
    # Recover the precise initial diagnostic source by undoing only the later
    # mode/offset extension. Initial binary hash was not captured before rebuild.
    final=Path('experiments/stage02_1/diagnostics.cpp').read_text()
    initial=final.replace('const std::string& file,double offset=.1) {','const std::string& file) {').replace('double t=start+offset;','double t=start+.1;').replace('if(mode=="integrate"||mode=="integrate_offgrid") {integrate(query,start,end,outdir+"/integrators.csv",mode=="integrate"?.1:.1037);return 0;}\n  if(mode!="audit")throw std::runtime_error("unknown mode");','if(mode=="integrate") {integrate(query,start,end,outdir+"/integrators.csv");return 0;}')
    (provenance/'diagnostics_initial.cpp').write_text(initial)
    (provenance/'diagnostics_final.cpp').write_text(final)
    dynamics=Path('/home/zn/flap-system-identification')
    external=[dynamics/'src/system_identification/models/trajectory_main_v2.py',dynamics/'src/system_identification/models/main_v2_simulator.py']
    inputs={str(root/'delivery_manifest.json'):sha(root/'delivery_manifest.json')}
    # Full old artifact index is carried as a reference, not duplicated CSV data.
    for p in (root/'observations').rglob('*'):
        if p.is_file(): inputs[str(p)]=sha(p)
    for p in [root/'final_validation/summary.json',root/'final_validation/phase_harmonics.csv']:
        inputs[str(p)]=sha(p)
    scalar_checks=json.load(open(out/'scalar_psd_query_check/checks.json'))
    assert len(scalar_checks)==6 and max(r['max_saved_query_error'] for r in scalar_checks)<1e-7
    manifest={'stage':'02.1','scope':'Reuse 18 final grid results and old sensitivities; four fixed mask solves; nine saved-curve replays, two analytic controls, two off-grid replays. No Stage03 or labels.',
      'scientific_decision':'Formal labels NOT READY; experimental research export conditional; acceleration not whole-body truth',
      'git_head':git('.','rev-parse','HEAD'),'git_status_at_seal':git('.','status','--porcelain'),
      'code_and_report_sha256':{str(p):sha(p) for p in sources if p.is_file()},'upstream':upstream,
      'source_stage02_git_head':original['git_head'],'source_stage02_verified_artifacts':len(original['artifact_sha256']),
      'source_stage02_artifact_sha256':original['artifact_sha256'],'input_sha256':inputs,
      'dynamics_source_sha256':{str(p):sha(p) for p in external},'dynamics_git_head':git(dynamics,'rev-parse','HEAD'),
      'python_packages':{p:importlib.metadata.version(p) for p in ['numpy','scipy','matplotlib']},
      'binaries':{str(out/'build/stage02_1_diagnostics'):sha(out/'build/stage02_1_diagnostics'),str(root/'build/stage02_reconstruct'):sha(root/'build/stage02_reconstruct')},
      'scalar_psd_query_check':scalar_checks,'initial_diagnostic_binary_sha256':None,'initial_binary_provenance_note':'Not captured before off-grid option rebuild. Initial source snapshot reconstructed by reversing only documented CLI extension; numerical default path unchanged. Final binary hash recorded.',
      'build_configuration_sha256':{str(p):sha(p) for p in [out/'build/CMakeCache.txt',out/'build/build.ninja']},
      'focused_checks':json.load(open(out/'summary/checks.json')),'new_solves':{p.parent.name:json.load(open(p))['exit_code'] for p in (out/'solves').glob('*/manifest.json')},
      'artifact_sha256':{str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file() and 'build' not in p.relative_to(out).parts and p.name!='delivery_manifest.json'},
      'large_artifacts_tracked':False,'neural_model_evaluation_accessed':False}
    assert len(manifest['new_solves'])==4 and all(v==0 for v in manifest['new_solves'].values())
    (out/'delivery_manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
    print('Sealed',len(manifest['artifact_sha256']),'new local artifacts;',len(original['artifact_sha256']),'Stage02 artifacts unchanged; git',manifest['git_head'])

if __name__=='__main__':main()

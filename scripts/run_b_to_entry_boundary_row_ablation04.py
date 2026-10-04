#!/usr/bin/env python3
"""Freeze boundary staging, then execute only four B_ENTRY_STAGE rollouts."""
import argparse
import json
from pathlib import Path
import shutil
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts'), str(ROOT/'scripts/isaac')]
import numpy as np
import yaml
import run_relative_factor_multisource01 as runtime
import run_b_to_entry_vector_bridge_execution03 as prior
from reconciliation.join_source03 import read, save, sha
from reconciliation.boundary_row_ablation04 import *
from reconciliation.relative_factor_multisource import geometry, reference_safety
from reconciliation.robotless_online import stamp
from run_osa03_native_continuation import git, plain
CONFIG = ROOT/'configs/b_to_entry_boundary_row_ablation_04.yaml'
RESULTS = ROOT/'results/b_to_entry_boundary_row_ablation_04'
DOC = ROOT/'docs/B_TO_ENTRY_BOUNDARY_ROW_ABLATION_04.md'
FILES = [CONFIG, ROOT/'src/reconciliation/boundary_row_ablation04.py',
    *[ROOT/'scripts'/f'{s}_b_to_entry_boundary_row_ablation04.py' for s in ['run','validate','report']],
    ROOT/'tests/test_b_to_entry_boundary_row_ablation04.py']


def authenticate(cfg, deep=False):
    path=ROOT/cfg['vector_result'];assert sha(path)==cfg['vector_result_sha256']
    assert git('show',cfg['vector_result_commit']+':'+cfg['vector_result'])==path.read_text().strip()
    result=read(path);run=Path(result['run']);prior.verify(run)
    assert result['summary']['scientific_freeze_sha']==cfg['vector_freeze_sha']
    assert sha(run/'result_hashes.json')==result['result_hashes_sha256']
    for p,digest in read(run/'result_hashes.json').items():assert sha(run/p)==digest,p
    for p,digest in result['table_hashes'].items():assert sha(path.parent/p)==digest,p
    if deep:
        from validate_b_to_entry_vector_bridge_execution03 import validate
        s,v=validate(run);assert s==result['summary'] and v['valid'] and v['all_historical_validators']
    assert cfg['sources']==SOURCE_IDS and cfg['order']==ORDER and cfg['new_method']==STAGE
    assert cfg['optimizer_calls']==0 and cfg['new_rollouts']==4 and cfg['integration_steps']==180 and cfg['primary_intervals']==54
    assert cfg['scalar_atol']==ATOL and cfg['time_tolerance']=='integration_dt_s - 1e-9'
    return result


def prepare(run):
    with no_reconciliation_optimizer():
        cfg=yaml.safe_load(CONFIG.read_text());h=authenticate(cfg,deep=True)
        run.mkdir(parents=True,exist_ok=False);save(run/'protocol.json',cfg);selected=[]
        for spec in h['summary']['selected_sources']:
            old=Path(spec['folder']);sid=spec['id'];folder=run/'sources'/spec['label'];(folder/'references').mkdir(parents=True)
            copies={}
            for name in ['common_state.json','source_manifest.json','schedule.json','scenario.json','metric_protocol.json',
                         'recorded_old_to_B.npy','source_audit.json','entry.json','suffix_native_installed.npy']:
                shutil.copyfile(old/name,folder/name);copies[name]=dict(path=str(old/name),sha256=sha(old/name))
            save(folder/'protocol.json',cfg)
            refs=read(old/'references.json');hist=h['summary']['sources'][sid]
            methods={n:dict(folder=hist['method_folders'][n],reference=refs[n],
                           hashes_sha256=sha(Path(hist['method_folders'][n])/'hashes.json')) for n in HISTORICAL}
            save(folder/'reuse.json',dict(copies=copies,methods=methods,historical_folder=str(old)))
            c=read(folder/'common_state.json');entry=read(folder/'entry.json')
            native=np.asarray(read(Path(methods[NATIVE]['folder'])/'restoration.json')['installed_world'])
            raw=np.load(refs[NATIVE]['local_path']);before=raw.tobytes()
            w,local,labels=staged_reference(native,raw,c['fresh_capture_pose'],c['B'],entry,np.load(folder/'suffix_native_installed.npy'))
            assert raw.tobytes()==before
            paths={}
            for frame,arr in [('world',w),('local',local)]:
                p=folder/'references'/f'{STAGE}_{frame}.npy'
                with p.open('xb') as stream:np.save(stream,arr,allow_pickle=False)
                paths.update({frame+'_path':str(p),frame+'_sha256':sha(p)})
            safe=reference_safety(w,geometry(folder))
            # Legacy safety-query wording says "no B connector"; annotate actual checked input explicitly.
            ref=dict(**paths,labels=labels,safety=safe,status='REFERENCE_GEOMETRY_SAFE' if safe['clearance_valid'] else 'REFERENCE_GEOMETRY_UNSAFE')
            save(folder/'prepared_references.json',{STAGE:ref});save(folder/'references.json',{**refs,STAGE:ref})
            shutil.copyfile(refs[NATIVE]['world_path'],folder/'references/M0_NATIVE_world.npy')
            labels_by_method={NATIVE:[f'F_{i}' for i in range(len(native))],ENTRY:['E*',*[f'F_{i}' for i in entry['row_original_identities'][1:]]],
                              HERMITE:refs[HERMITE]['labels'],VECTOR:refs[VECTOR]['labels'],STAGE:labels}
            save(folder/'identity_mapping.json',{n:identity_mapping(lab,entry,native) for n,lab in labels_by_method.items()})
            install=dict(performed=False,numerical_MPC_calls=0)
            if safe['clearance_valid']:
                runtime.prior.preflight(folder);pre=read(folder/'restoration_preflight.json');installed=np.asarray(pre['rows'][0]['installed_world'])
                ids=entry['row_original_identities'][1:];assert installed[2:].tobytes()==native[ids].tobytes()
                ip=folder/'installed_preflight_world.npy'
                with ip.open('xb') as stream:np.save(stream,installed,allow_pickle=False)
                actual_safe=reference_safety(installed,geometry(folder));assert actual_safe['clearance_valid']
                install=dict(performed=True,numerical_MPC_calls=0,labels=labels,installed_path=str(ip),installed_sha256=sha(ip),
                    installed_safety=actual_safe,derived_row_roundoff_max_abs=float(np.max(abs(installed[:2]-w[:2]))),downstream_bit_exact=True,
                    actual_checked_polyline='B -> E* -> original suffix; B-to-entry edge included')
            save(folder/'installation.json',install)
            selected.append(dict(spec,folder=str(folder),historical_folder=str(old)))
        save(run/'selected_sources.json',selected)
        save(run/'authentication.json',dict(V2_and_all_historical_validators=True,optimizer_calls=0,numerical_MPC_preflight_calls=0,
            historical_rollouts_reused=16,new_reference_structure='B,E*,unchanged original suffix'))
        print(json.dumps(dict(prepared=str(run),optimizer_calls=0,numerical_MPC_calls=0)))


def freeze(run):
    cfg=yaml.safe_load(CONFIG.read_text());h=authenticate(cfg);assert cfg==read(run/'protocol.json')
    files=list(dict.fromkeys([Path(p) for p in read(Path(h['run'])/'freeze.json')['files']]+FILES))
    with (run/'protocol_document.md').open('xb') as f:f.write(DOC.read_bytes())
    save(run/'freeze.json',dict(files={str(p):sha(p) for p in files},inputs={str(p):sha(p) for p in run.rglob('*') if p.is_file()},
        optimizer_calls=0,maximum_B_ENTRY_rollouts=4,retries=0))
    RESULTS.mkdir(parents=True,exist_ok=True)
    save(RESULTS/'freeze_summary.json',dict(run=str(run),freeze=read(run/'freeze.json'),authentication=read(run/'authentication.json'),
        selected_sources=read(run/'selected_sources.json'),sources={s['label']:{k:read(Path(s['folder'])/(k+'.json')) for k in
        ['common_state','schedule','references','identity_mapping','installation']} for s in read(run/'selected_sources.json')}))


def verify(run,pushed=False):
    authenticate(read(run/'protocol.json'));f=read(run/'freeze.json')
    for group in ['files','inputs']:
        for p,digest in f[group].items():assert sha(p)==digest,p
    if pushed:
        assert git('rev-parse','HEAD')==git('ls-remote','origin','refs/heads/main').split()[0]
        for p in f['files']:assert git('rev-parse','HEAD:'+str(Path(p).relative_to(ROOT)))==git('hash-object',p),p
        assert git('rev-parse','HEAD:'+str(DOC.relative_to(ROOT)))==git('hash-object',str(run/'protocol_document.md'))
        assert not git('diff','HEAD','--',str(RESULTS/'freeze_summary.json'))


def execute(run):
    with no_reconciliation_optimizer():
        verify(run,pushed=True)
        save(run/'execution_start.json',dict(sha=git('rev-parse','HEAD'),started=stamp(),optimizer_calls=0,retries=0))
        for spec in read(run/'selected_sources.json'):
            folder=Path(spec['folder']);ref=read(folder/'references.json')[STAGE]
            if ref['safety']['clearance_valid']:runtime.run_method(folder,STAGE)
            else:save(folder/'methods'/STAGE/'skipped.json',dict(status=ref['status'],metrics=None,rollouts=0))
        verify(run)
        save(run/'completion.json',dict(completed=stamp(),optimizer_calls=0,historical_reruns=0,V3_rollouts=0,LightNav=0,RGB=0,Isaac=0,retries=0))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--mode',choices=['prepare','freeze','execute'],required=True)
    a=p.parse_args();globals()[a.mode](a.run.resolve())

#!/usr/bin/env python3
"""Authenticate saved solutions, prepend fixed B, freeze, execute only new wrappers."""
import argparse
from pathlib import Path
import shutil
import sys
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
from reconciliation.canonical_graph_execution01 import *
from reconciliation.join_source03 import read,save,sha
from reconciliation.relative_factor_multisource import geometry,reference_safety
from reconciliation.robotless_online import stamp
from run_local_se2_reconciliation_formulation01 import git,plain
import run_relative_factor_multisource01 as runtime
from run_direct_transition_hard_eval02 import record_reference
from run_canonical_se2_graph_formulation_audit01 import np_save
CONFIG=ROOT/'configs/canonical_graph_common_b_execution_01.yaml'
RESULTS=ROOT/'results/canonical_graph_common_b_execution_01'
DOC=ROOT/'docs/CANONICAL_GRAPH_COMMON_B_EXECUTION_01.md'


def authenticate(cfg,deep=False):
    out={};files={}
    for key in ['canonical','historical']:
        a=cfg[key];p=ROOT/a['summary'];assert sha(p)==a['summary_sha256']
        assert git('show',a['result_commit']+':'+a['summary'])==p.read_text().strip()
        run=ROOT/a['run'];result=read(p)
        if key=='canonical':
            from run_canonical_se2_graph_formulation_audit01 import verify
            verify(run)
            assert read(run/'execution_start.json')['scientific_freeze_sha']==a['freeze_commit']
            from validate_canonical_se2_graph_formulation_audit01 import validate,check_exports
            if deep:s,v=validate(run);assert v['valid'];check_exports(run,s)
            ledger=read(run/'result_hashes.json')
            authority=read(p.parent/'result_hashes.json')
            assert ledger==authority['files'] and sha(run/'result_hashes.json')==authority['sha256']
        else:
            from run_direct_transition_hard_eval02 import verify
            verify(run)
            assert read(run/'execution_start.json')['sha']==a['freeze_commit']
            assert sha(run/'result_hashes.json')==result['result_hashes_sha256']
            ledger=read(run/'result_hashes.json')
            if deep:
                from validate_direct_transition_hard_eval02 import validate
                s,v=validate(run,deep=False);assert v['valid'] and s==read(run/'summary.json')
            for name,h in result['table_hashes'].items():assert sha(p.parent/name)==h
        for rel,h in ledger.items():
            path=run/rel;assert sha(path)==h,str(path);files[str(path)]=h
        out[key]=dict(result_commit=a['result_commit'],freeze_commit=a['freeze_commit'],summary_sha256=sha(p),
            run=str(run),saved_validation=deep,artifact_count=len(ledger))
        files[str(p)]=sha(p)
    return dict(authorities=out,files=files,new_scientific_calls=0)


def prepare(run):
    cfg=yaml.safe_load(CONFIG.read_text())
    assert cfg['sources']==SOURCE_IDS and cfg['new_methods']==NEW and cfg['figures']==PNGS
    history=authenticate(cfg,deep=True)
    run.mkdir(parents=True,exist_ok=False);save(run/'protocol.json',cfg);save(run/'authentication.json',history)
    selected=[];counts=dict(rollouts=0,MPC_solves=0,logical_releases=0,withheld_results=0,executed_intervals=0)
    with execution_guard():
        for i,sid in enumerate(SOURCE_IDS,1):
            label=f'E{i}';old=ROOT/cfg['historical']['run']/'sources'/label;canon=ROOT/cfg['canonical']['run']/'sources'/label
            f=run/'sources'/label;(f/'references').mkdir(parents=True)
            for n in ['common_state.json','source_manifest.json','schedule.json','scenario.json','source_audit.json','metric_protocol.json','entry.json','recorded_old_to_B.npy','native_installed.npy']:
                shutil.copyfile(old/n,f/n)
            save(f/'protocol.json',cfg);c=read(f/'common_state.json');ctx=read(canon/'context.json');assert ctx['id']==sid
            assert ctx['A']==c['fresh_capture_pose'] and ctx['B']==c['B']
            assert sha(f/'common_state.json')==ctx['common_state_sha256'] and sha(f/'schedule.json')==ctx['schedule_sha256']
            native=read(old/'native_reference.json')['M0_NATIVE']
            for frame in ['world','local']:
                assert sha(native[frame+'_path'])==native[frame+'_sha256']
                shutil.copyfile(native[frame+'_path'],f/'references'/f'M0_NATIVE_{frame}.npy')
            for name in ['canonical_optimized.npy','canonical_original_A_local.npy','result.json','context.json']:
                shutil.copyfile(canon/name,f/name)
            assert sha(f/'canonical_optimized.npy')==sha(canon/'canonical_optimized.npy')
            fresh=np.load(native['world_path']);raw=np.load(native['local_path'])
            assert fresh.tobytes()==np.load(canon/'raw_fresh.npy').tobytes() and raw.tobytes()==np.load(canon/'raw_observation_local.npy').tobytes()
            historical=read(old/'references.json');refs={ENTRY:historical[ENTRY]};prepared={}
            for name,prefix,w,l in [(RAW,'F',fresh,raw),(CANONICAL,'X',np.load(f/'canonical_optimized.npy'),np.load(f/'canonical_original_A_local.npy'))]:
                rw,rl,labels=boundary_wrapper(c['fresh_capture_pose'],c['B'],w,l,prefix)
                prepared[name]=record_reference(f,name,rw,rl,labels)
            save(f/'prepared_references.json',prepared)
            runtime.prior.preflight(f)
            pre=read(f/'restoration_preflight.json');assert pre['passed'] and pre['numerical_MPC_calls']==0
            installed={r['identity']:r for r in pre['rows']};installation={}
            for name,rec in prepared.items():
                actual=np.array(installed[name]['installed_world']);world=np.load(rec['world_path'])
                np.testing.assert_allclose(actual,world,rtol=0,atol=1e-12)
                safe=reference_safety(actual,geometry(f))
                valid=rec['safety']['clearance_valid'] and safe['clearance_valid']
                rec.update(status='REFERENCE_GEOMETRY_SAFE' if valid else 'SKIPPED_REFERENCE_INVALID',may_execute=bool(valid))
                np_save(f/'references'/f'{name}_installed.npy',actual)
                installation[name]=dict(safety=safe,max_roundtrip_abs=float(np.max(abs(actual-world))),
                    explicit_B_connector_checked=True,planning_world_bits_exact=True,local_source_bits_exact=True)
                if name==RAW:assert actual[1:].tobytes()==np.load(f/'native_installed.npy').tobytes()
                if valid:
                    sched=read(f/'schedule.json');pairs=len(sched['pairs']);apps=len(sched['full']['applications'])
                    for k,v in dict(rollouts=1,MPC_solves=pairs,logical_releases=apps,withheld_results=pairs-apps,executed_intervals=cfg['integration_steps']).items():counts[k]+=v
                refs[name]=rec
            save(f/'installation.json',installation);save(f/'references.json',refs)
            save(f/'identity_mapping.json',{n:identities(r['labels'],read(f/'entry.json'),fresh) for n,r in refs.items()})
            reuse={n:dict(folder=str(old/'methods'/n),reference=historical[n],hashes_sha256=sha(old/'methods'/n/'hashes.json')) for n in [ENTRY,*CONTEXT]}
            save(f/'reuse.json',reuse)
            selected.append(dict(id=sid,label=label,folder=str(f),historical_folder=str(old),canonical_folder=str(canon)))
    save(run/'selected_sources.json',selected)
    save(run/'expected_calls.json',dict(**counts,canonical_optimizer=0,V2_optimizer=0,historical_rollouts_reused=16,
        conditional='Exact full-cap count if all eligible rollouts reach cap; unsafe references skipped; safety abort prefix retained without retry'))
    print(read(run/'expected_calls.json'),flush=True)


def freeze(run):
    assert read(run/'protocol.json')==yaml.safe_load(CONFIG.read_text())
    files=[ROOT/p for p in git('ls-files','src','scripts','tests').splitlines() if (ROOT/p).is_file()]
    files += [CONFIG,*[ROOT/p for p in git('ls-files','--others','--exclude-standard','src','scripts','tests').splitlines() if 'canonical_graph' in p]]
    shutil.copyfile(DOC,run/'protocol_document.md')
    save(run/'freeze.json',dict(files={str(p):sha(p) for p in files},inputs={str(p):sha(p) for p in run.rglob('*') if p.is_file()}))
    RESULTS.mkdir(parents=True,exist_ok=True)
    for n in ['protocol.json','selected_sources.json','expected_calls.json','authentication.json']:shutil.copyfile(run/n,RESULTS/n)
    save(RESULTS/'freeze.json',dict(run=str(run),freeze=read(run/'freeze.json')))


def verify(run,pushed=False):
    assert read(run/'protocol.json')==yaml.safe_load(CONFIG.read_text())
    assert read(run/'freeze.json')==read(RESULTS/'freeze.json')['freeze']
    for group in read(run/'freeze.json').values():
        for p,h in group.items():assert sha(p)==h,p
    for p,h in read(run/'authentication.json')['files'].items():assert sha(p)==h,p
    if pushed:
        assert git('rev-parse','HEAD')==git('ls-remote','origin','refs/heads/main').split()[0]
        for p in read(run/'freeze.json')['files']:
            assert git('rev-parse','HEAD:'+str(Path(p).relative_to(ROOT)))==git('hash-object',p),p
        assert git('rev-parse','HEAD:'+str(DOC.relative_to(ROOT)))==git('hash-object',str(run/'protocol_document.md'))
        assert not git('diff','HEAD','--',str(RESULTS))


def execute(run):
    verify(run,pushed=True)
    # Exclusive creation makes retries and duplicate executions fail closed.
    with (run/'execution.lock').open('x') as stream:stream.write(git('rev-parse','HEAD')+'\n')
    save(run/'execution_start.json',dict(sha=git('rev-parse','HEAD'),started=stamp()))
    with execution_guard():
        for s in read(run/'selected_sources.json'):
            f=Path(s['folder']);refs=read(f/'references.json')
            for n in NEW:
                if refs[n]['may_execute']:
                    print('EXECUTE '+s['label']+' '+n,flush=True);runtime.run_method(f,n)
                else:
                    out=f/'methods'/n;out.mkdir(parents=True,exist_ok=False)
                    save(out/'skipped.json',dict(status='SKIPPED_REFERENCE_INVALID',metrics=None))
    verify(run)
    save(run/'completion.json',dict(completed=stamp(),canonical_optimizer_calls=0,V2_optimizer_calls=0,
         LightNav=0,RGB=0,Isaac=0,new_source_acquisitions=0,historical_scientific_reruns=0,retries=0))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    p.add_argument('--mode',choices=['prepare','freeze','execute'],required=True);a=p.parse_args()
    globals()[a.mode](a.run.resolve())

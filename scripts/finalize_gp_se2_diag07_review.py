#!/usr/bin/env python3
"""Close an isolated post-render log hash issue; preserve original evidence.

The frozen validator recomputed scientific records successfully, but captured
plot_console.log before the plotter's last stdout write. This helper explicitly
allows only that exact metadata error, independently rechecks plans/execution
and all numeric sidecars, and writes new authoritative verification artifacts.
No primary artifacts, frozen code, controller or scientific outcomes are edited.
"""
from __future__ import annotations
import argparse
from pathlib import Path
import sys
import time
import zipfile
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
import yaml
from run_gp_se2_diag07 import verify,read,write,digest,clean,check_hashes,ONLINE
from reconciliation.gp_se2_diag07_lateral_execution import HARD,support_reference,plan_gate,validate_rollout_records,execution_diagnostics,deduplicate
from reconciliation.gp_se2_diagnostics import load_frozen_case
from reconciliation.gp_se2_diag_acceptance import check_full_candidate as original_full_check
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_evaluation import evaluate_rollout
from reconciliation.gp_se2_rollout import load_frozen_context
from plot_gp_se2_diag07 import plot_data,numeric,PLOTS


def reporting_error_gate(initial,actual_mismatches):
    expected=['artifact hash plot_console.log']
    if initial['errors']!=expected or actual_mismatches!=['plot_console.log']:
        raise ValueError('unexpected validation failure; reporting completion cannot waive it')
    if initial['valid'] or not all(initial[k] for k in ('no_new_MPC','no_new_GP','no_new_VLA','no_new_rollout')):
        raise ValueError('initial validation scope mismatch')


def finalize(run):
    target=run/'verification';target.mkdir(exist_ok=False);begin=time.perf_counter()
    source=verify(run,frozen=True);initial=read(run/'validation.json');artifacts=read(run/'artifact_hashes.json')
    mismatches=[p for p,h in artifacts.items() if digest(run/p)!=h]
    reporting_error_gate(initial,mismatches)
    completed_log=(run/'plot_console.log').read_text()
    if completed_log.strip()!=str(dict(plots=10,zip_bytes=(run/'review_bundle.zip').stat().st_size)):
        raise ValueError('plot log is not exactly the terminal plot summary')
    # Record the originally empty log hash and final complete log without altering either manifest.
    import hashlib
    if artifacts['plot_console.log']!=hashlib.sha256(b'').hexdigest():raise ValueError('unexpected captured log prefix')
    errors=[];checks=0
    def check(ok,label):
        nonlocal checks
        checks+=1
        if not bool(ok):errors.append(label)
    env=HospitalEnvironment.load(source['environment_path']);config=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    manifest=read(run/'reference_manifest.json');unique,alias=deduplicate(manifest['records'])
    check(unique==manifest['unique_references'] and alias==read(run/'provenance_aliases.json'),'dedup')
    for r in manifest['records']:
        saved=read(Path(r['source_path']));case=load_frozen_case(source['primary_source'],r['case_id'],r['method'],environment=env)
        z,poses,twists,ref=support_reference(case['problem'],saved)
        full=clean(original_full_check(case['problem'],z,case))
        plan=read(Path(r['plan_recheck_path']))
        check(full==plan['original_full_check'],'full original plan '+r['reference_id'])
        check(plan_gate(full,hard=r['case_id']==HARD,changed_from_seed=not np.array_equal(z,saved['initial_vector']))==plan['admission'],'unchanged admission')
        check(np.array_equal(ref,np.load(r['world_reference_path'],allow_pickle=False)),'exact support reference')
    for r in unique:
        uid=r['unique_reference_id'];folder=run/'rollouts'/uid;roll=read(folder/'rollout.json')
        ctx=read(run/'contexts'/(r['case_id'].replace('/','__')+'.json'));frozen=load_frozen_context(ONLINE,*r['case_id'].split('/'))
        check(frozen==ctx['context'],'source acquisition '+uid)
        check(not validate_rollout_records(roll,frozen,np.load(r['world_reference_path'],allow_pickle=False)),'all integration/selector records '+uid)
        evaluation=clean(evaluate_rollout(roll,ctx['goal_route'],env,config))
        check(evaluation==read(folder/'full_execution_evaluation.json'),'independent execution '+uid)
        diagnostics=execution_diagnostics(roll,evaluation,frozen);saved=read(folder/'diagnostics.json')
        check(all(clean(v)==saved[k] for k,v in diagnostics.items()),'command/selection diagnostics '+uid)
    data=plot_data(run);check(data==read(run/'plot_data.json'),'full plot numeric source')
    for name in PLOTS:
        d=read(run/'plots'/(name+'.json'));check(d['numeric']==numeric(name,data),'original plot numeric '+name)
    for r in data['records']:
        for name in PLOTS:
            p=run/'presentation'/r['id']/(name+'.json');d=read(p)
            check(d['numeric']==numeric(name,{**data,'records':[r]}),'readable plot data '+r['id']+name)
            check(d['image_sha256']==digest(p.with_suffix('.png')),'readable PNG '+r['id']+name)
            check(d['parent_sha256']==digest(run/'plot_data.json'),'readable source '+r['id']+name)
    check_hashes({str(run/'presentation'/p):h for p,h in read(run/'presentation/artifact_hashes.json').items()})
    with zipfile.ZipFile(run/'review_bundle_readable.zip') as z:
        for name in z.namelist():
            path=run/name if name.startswith('aggregate/') or name in ['review_source.json','protocol.json','reference_manifest.json'] else run/'presentation'/name
            check(hashlib.sha256(z.read(name)).hexdigest()==digest(path),'review ZIP member '+name)
    verify(run,frozen=True)
    correction=dict(scope='report-only; no numerical or execution rerun',reason='plotter hashed an empty redirected stdout log before its final print',
        original_validator_path=str(run/'validation.json'),original_validator_sha256=digest(run/'validation.json'),
        original_check_count=initial['check_count'],original_scientific_checks_passed=True,
        sole_original_error=initial['errors'][0],original_log_sha256=artifacts['plot_console.log'],complete_log_sha256=digest(run/'plot_console.log'),
        original_artifact_manifest_preserved=True,readable_presentation='separate per-reference plots fix overlapping initial multi-panel titles; identical numeric sources',
        actual_primary_reruns=0,new_MPC=0,new_GP=0,new_rollout=0)
    write(target/'reporting_completion.json',correction)
    write(target/'output_hashes.json',{str(p.relative_to(run)):digest(p) for p in sorted(run.rglob('*')) if p.is_file() and p.name not in ['completion.json'] and not p.is_relative_to(target)})
    result=dict(valid=not errors,authoritative=True,errors=errors,check_count=checks,
        frozen_independent_validator_checks=initial['check_count'],wall_s=time.perf_counter()-begin,
        source_files_preserved=len(source['preserved_hashes']),full_plan_execution_recomputed=True,
        all_numeric_sidecars_checked=True,all_sources_outputs_checked=True,
        no_new_MPC=True,no_new_GP=True,no_new_VLA=True,no_new_rollout=True,
        supersedes_only_post_render_log_hash_error=str(run/'validation.json'))
    write(target/'validation.json',result);print(result,flush=True)
    if errors:raise SystemExit('final report verification failed')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',required=True,type=Path);finalize(p.parse_args().run.resolve())

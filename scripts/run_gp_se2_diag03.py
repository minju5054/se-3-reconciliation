#!/usr/bin/env python3
"""Freeze, audit, and independently validate nine saved GP vectors. No solves."""
from __future__ import annotations
import argparse
import csv
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import zipfile

for name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):
    os.environ[name]='1'
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
from run_gp_se2_ref01 import read,write,copy_new,digest,utc,revision,table,clean,array
from reconciliation.gp_se2_diag03_audit import PROTOCOL,HARD,BENIGN,record_specs,restore_record,audit_record,summarize,compare_tree
from reconciliation.gp_se2_diagnostics import load_frozen_case
from reconciliation.gp_se2_environment import HospitalEnvironment

STARTING_SHA='da5b93151aec62ce88d4fa6bda58ca8ae5b7dd86'
PRIMARY=ROOT/'data/robotless_gp_se2_02/primary_20260919T024000Z'
OUTPUT=ROOT/'data/robotless_gp_se2_diag_03'
OWN_FILES=['src/reconciliation/gp_se2_diag03_audit.py','src/reconciliation/gp_se2_diag03_intervals.py',
    'src/reconciliation/gp_se2_diag03_derivatives.py','scripts/run_gp_se2_diag03.py','scripts/plot_gp_se2_diag03.py',
    'tests/test_gp_se2_diag03_audit.py','tests/test_gp_se2_diag03_intervals.py',
    'tests/test_gp_se2_diag03_derivatives.py','tests/test_gp_se2_diag03_plots.py']
USER_FILES=['configs/stage0_jackal_controller_validation.yaml','configs/stage0_lightnav_single_chunk.yaml']


def check_hashes(expected):
    return [p for p,h in expected.items() if not Path(p).is_file() or digest(p)!=h]


def prohibit_optimization():
    """Fail closed if this saved-query path accidentally attempts optimization."""
    from unittest.mock import patch
    from contextlib import ExitStack
    import reconciliation.gp_se2_formulation as formulation
    def forbidden(*args,**kwargs):
        raise RuntimeError('DIAG03 saved audit forbids new optimization/rollout')
    stack=ExitStack()
    for name in ('minimize','solve_gp','solve_rigid','_minimize_budgeted'):
        if hasattr(formulation,name):stack.enter_context(patch.object(formulation,name,forbidden))
    stack.enter_context(patch('scipy.optimize.minimize',forbidden))
    stack.enter_context(patch('reconciliation.gp_se2_rollout.counterfactual_rollout',forbidden))
    return stack


def prepare(run):
    if run.exists() or not run.is_relative_to(OUTPUT):raise FileExistsError('exclusive new DIAG03 run; no overwrite')
    begin=time.perf_counter();src=read(PRIMARY/'source.json');validation=read(PRIMARY/'validation.json')
    if not validation['valid'] or validation['errors']:raise ValueError('authoritative GP-SE2-02 validation failed')
    freeze=read(PRIMARY/'experiment_freeze.json')
    if freeze['experiment_git_sha']!='bd273b22ed9ddcb35a3f2eadcf550895adee46b9':raise ValueError('historical execution SHA mismatch')
    expected={str(ROOT/p):h for p,h in src['preserved_core_sha256'].items()}
    expected.update({str(Path(src['environment_path'])/r['path']):r['sha256'] for r in src['original_environment_file_hashes']})
    expected.update({p:r['sha256'] for p,r in src['authoritative_validations'].items()})
    expected.update({str(PRIMARY/p):h for p,h in read(PRIMARY/'artifact_manifest.json')['files'].items()})
    if digest(PRIMARY/'config_snapshot.yaml')!=src['original_config_sha256']:raise ValueError('original config mismatch')
    manifest=read(PRIMARY/'case_manifest.json');records=record_specs();contexts={}
    for record in records:
        case=next(r for r in manifest['selected'] if r['case_id']==record['case_id'])
        folder=PRIMARY/'cases'/case['case_directory'];start=folder/'methods'/record['method']/'starts'/record['initialization']
        record.update(case_directory=str(folder),solver_path=str(start/'solver_result.json'),
            post_checks_path=str(start/'post_full_checks.json'),
            initial_path=str(folder/'initializations'/f"{record['initialization']}.npy"),
            initial_metadata_path=str(folder/'initializations'/f"{record['initialization']}.json"),
            initial_full_path=str(folder/'initializations'/f"initial_full_{record['method']}_{record['initialization']}.json"))
        context=read(folder/'input_context.json')
        for row in context['source_files']:expected[str(Path(context['source_root'])/row['path'])]=row['sha256']
        for p in folder.rglob('*'):
            if p.is_file():expected.setdefault(str(p),digest(p))
        contexts[record['case_id']]=dict(case_directory=str(folder),context=context,goal_route=read(folder/'goal_route.json'))
    mismatch=check_hashes(expected)
    if mismatch:raise ValueError('SOURCE_INTEGRITY_BLOCKER: '+str(mismatch))
    # Read-only hashes, not copies or re-evaluation of unrelated historical corpora.
    expected.update({str(p):digest(p) for p in PRIMARY.iterdir() if p.is_file()})
    run.mkdir(parents=True);code={p:digest(ROOT/p) for p in OWN_FILES}
    write(run/'protocol.json',dict(PROTOCOL,frozen_utc=utc(),audit_git_sha=revision(),audit_code_sha256=code))
    write(run/'record_manifest.json',dict(records=records,selected_before_numerical_audit=True,case_selection='user-fixed'))
    copy_new(PRIMARY/'config_snapshot.yaml',run/'config_snapshot.yaml')
    write(run/'source.json',dict(experiment='GP-SE2-DIAG-03',starting_git_sha=STARTING_SHA,audit_git_sha=revision(),
        historical_execution_sha=freeze['experiment_git_sha'],primary_source=str(PRIMARY),
        authoritative_validation=dict(path=str(PRIMARY/'validation.json'),sha256=digest(PRIMARY/'validation.json'),valid=True),
        environment_path=src['environment_path'],environment_export_source=src['environment_export_source'],
        original_config_sha256=src['original_config_sha256'],core_sha256=src['preserved_core_sha256'],
        audit_code_sha256=code,contexts=contexts,source_input_hashes=expected,
        user_config_hashes={str(ROOT/p):digest(ROOT/p) for p in USER_FILES},
        versions={p:importlib.metadata.version(p) for p in ('numpy','scipy','shapely','matplotlib','pytest','PyYAML')},
        preparation_wall_s=time.perf_counter()-begin,frame=src['frame'],GUI_runtime='NOT_RUN_STATIC_DIAGNOSTIC'))
    print(json.dumps(dict(prepared=str(run),records=len(records),preserved_files=len(expected))),flush=True)


def compute(run, *, verbose=False):
    source=read(run/'source.json');start=time.perf_counter();env=HospitalEnvironment.load(source['environment_path'])
    timings=dict(environment_loading_s=time.perf_counter()-start);results=[];contexts={};config=None
    with prohibit_optimization():
        for spec in read(run/'record_manifest.json')['records']:
            start=time.perf_counter()
            case=load_frozen_case(PRIMARY,spec['case_id'],spec['method'],environment=env);config=case['config']
            solver=read(spec['solver_path'])
            record=restore_record(spec,case,solver,np.load(spec['initial_path'],allow_pickle=False),read(spec['initial_metadata_path']))
            record=audit_record(record,case,solver,read(spec['post_checks_path']),read(spec['initial_full_path']))
            results.append(record);timings[spec['record_id']]=time.perf_counter()-start
            context=case['context'];rawfiles=context['source_files']
            worldfiles=[r['path'] for r in rawfiles if r['path'].endswith('/world.npy')]
            contexts['hard' if spec['case_id']==HARD else 'benign']=dict(B_world=context['B_world'],
                old_world=np.load(Path(context['source_root'])/worldfiles[0],allow_pickle=False),
                fresh_world=np.load(Path(spec['case_directory'])/'F_native.npy',allow_pickle=False),
                goal_world=case['goal_route']['goal_world'])
            if verbose:print(json.dumps(dict(record=spec['record_id'],acceptance=record.get('acceptance'),wall_s=timings[spec['record_id']])),flush=True)
    return dict(records=results,contexts=contexts,formulation_config=config['formulation'],summary=summarize(results),timing=timings),env


def tables(audit):
    result={k:[] for k in ('original_reproduction','final_acceptance_matrix','constraint_time_map','interval_extrema',
                           'violation_brackets','derivative_consistency','grid_detection_comparison')}
    for r in audit['records']:
        tag={k:r[k] for k in ('record_id','role','method','initialization')}
        if not r['available']:
            result['original_reproduction'].append(dict(tag,status=r['status']));continue
        result['original_reproduction'].append(dict(tag,available=True,solver_termination=r['solver_termination'],
            candidate_available=r['candidate_available'],vector_sha256=r['vector_sha256'],objective=r['objective'],
            **r['acceptance'],reproduced=r['original_reproduction']['reproduced'],literal_equal=r['original_reproduction']['all_literal_equal'],
            support_reconstruction_consistent=r['reconstruction']['consistent']))
        if r['phase']=='final':result['final_acceptance_matrix'].append(dict(result['original_reproduction'][-1],
            **r['derivatives']['summary']))
        mappings={'constraint_time_map':r['intervals']['constraint_time_map']['rows'],
                  'interval_extrema':r['interval_extrema'],'violation_brackets':r['intervals']['violation_brackets'],
                  'grid_detection_comparison':r['grid_detection'],'derivative_consistency':r['derivatives']['derivative_checks']}
        for name,rows in mappings.items():result[name].extend(dict(tag,**row) for row in rows)
    return result


def audit(run):
    if (run/'audit.json').exists() or (run/'audit_started.json').exists():raise FileExistsError('audit already started; preserve run')
    source=read(run/'source.json')
    if check_hashes(source['source_input_hashes']):raise ValueError('source changed since freeze')
    if any(digest(ROOT/p)!=h for p,h in source['audit_code_sha256'].items()):raise ValueError('audit implementation changed since freeze')
    write(run/'audit_started.json',dict(utc=utc(),audit_git_sha=revision()))
    result,env=compute(run,verbose=True)
    write(run/'audit.json',result)
    for record in result['records']:
        folder=run/'records'/record['record_id'];folder.mkdir(parents=True)
        if record['available']:
            for key in ('vector','support_poses','support_twists'):array(folder/(key+'.npy'),record[key])
            write(folder/'original_full_acceptance.json',record['original_full'])
            write(folder/'derivative_checks.json',record['derivatives'])
            write(folder/'interval_analysis.json',record['intervals'])
        write(folder/'provenance.json',{k:v for k,v in record.items() if k in record_specs()[0] or k in ('solver_path','source_vector_key','vector_sha256','available','reconstruction')})
    (run/'aggregate').mkdir();(run/'original_reproduction').mkdir()
    for name,rows in tables(result).items():table(run/'aggregate'/(name+'.csv'),rows)
    for r in result['records']:
        if r['available']:write(run/'original_reproduction'/(r['record_id']+'.json'),r['original_reproduction'])
    write(run/'aggregate/summary.json',result['summary']);write(run/'aggregate/timing.json',result['timing'])
    from plot_gp_se2_diag03 import generate
    generate(run,audit=clean(result),environment=env)
    write(run/'audit_completed.json',dict(utc=utc(),summary=result['summary']))


def validate(run):
    if (run/'validation.json').exists():raise FileExistsError('validation exists; no overwrite')
    start=time.perf_counter();source=read(run/'source.json');saved=read(run/'audit.json');errors=[]
    mismatch=check_hashes(source['source_input_hashes']);errors.extend('source hash '+p for p in mismatch)
    errors.extend('user edit '+p for p in check_hashes(source['user_config_hashes']))
    for p,h in source['audit_code_sha256'].items():
        if digest(ROOT/p)!=h:errors.append('frozen code '+p)
    # Independent reloading/reconstruction and original checker invocation; no solver.
    recomputed,_=compute(run,verbose=True);checks=0
    for original,current in zip(saved['records'],clean(recomputed)['records']):
        for key in ('vector','support_poses','support_twists','original_collocation','original_full','intervals','derivatives','acceptance'):
            if key not in current:continue
            rows=compare_tree(current[key],original[key],original['record_id']+'.'+key);checks+=len(rows)
            errors.extend(r['field'] for r in rows if not r['numerical_agreement'])
        if original['available']:
            for key in ('vector','support_poses','support_twists'):
                checks+=1
                if not np.array_equal(np.load(run/'records'/original['record_id']/(key+'.npy'),allow_pickle=False),current[key]):errors.append('derived array '+key)
    for name,rows in tables(recomputed).items():
        # Exact canonical CSV serialization checks every table cell without new solves.
        import io
        buf=io.StringIO(newline='');rows=clean(rows);fields=list(dict.fromkeys(k for row in rows for k in row)) or ['status']
        writer=csv.DictWriter(buf,fields);writer.writeheader()
        for row in rows:writer.writerow({k:json.dumps(v) if isinstance(v,(dict,list)) else v for k,v in row.items()})
        checks+=len(rows)
        if (run/'aggregate'/(name+'.csv')).read_bytes()!=buf.getvalue().encode():errors.append('table '+name)
    from plot_gp_se2_diag03 import expected_numeric
    expected=expected_numeric(saved)
    for name,numbers in expected.items():
        side=read(run/'plots'/(name+'.json'));checks+=1
        # Sidecar schema belongs to the focused plotter.
        if side['numeric_data']!=numbers:errors.append('plot numeric '+name)
        if digest(run/'plots'/(name+'.png'))!=side['image_sha256']:errors.append('plot image '+name)
        errors.extend('plot source '+p for p in check_hashes(side['source_hashes']))
    with zipfile.ZipFile(run/'review_bundle.zip') as z:
        for name in z.namelist():
            checks+=1
            member=run/'review_bundle'/name
            if not member.is_file() or z.read(name)!=member.read_bytes():errors.append('ZIP '+name)
    checks+=1
    if summarize(recomputed['records'])!=saved['summary']:errors.append('summary')
    result=dict(valid=not errors,authoritative=True,errors=errors,check_count=checks,
        validation_wall_s=time.perf_counter()-start,source_hashes_preserved=not mismatch,
        new_optimization=0,new_MPC_solve=0,new_rollout=0,
        scientific_failure_is_artifact_failure=False,scope='saved source, reconstruction, original acceptance, every interval/derivative/table and plot numbers; no new solves')
    write(run/'validation.json',result);print(json.dumps(result),flush=True)
    if errors:raise SystemExit(1)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=['prepare','audit','validate']);parser.add_argument('--run',type=Path,required=True)
    args=parser.parse_args();globals()[args.action](args.run.resolve())

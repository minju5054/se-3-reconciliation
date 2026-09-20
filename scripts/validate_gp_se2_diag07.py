#!/usr/bin/env python3
"""Independently recheck saved DIAG-07 records without any solver or simulator."""
from __future__ import annotations
import argparse
import csv
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
import yaml
from run_gp_se2_diag07 import verify,read,write,digest,clean,D6,ONLINE
from reconciliation.gp_se2_diag07_lateral_execution import (
    HARD,BENIGN,array_hash,support_reference,plan_gate,deduplicate,
    validate_rollout_records,execution_diagnostics,experiment_interpretation)
from reconciliation.gp_se2_diagnostics import load_frozen_case
from reconciliation.gp_se2_diag_acceptance import check_full_candidate as original_full_check
from reconciliation.gp_se2_rollout import load_frozen_context
from reconciliation.gp_se2_evaluation import evaluate_rollout,environment_trace
from reconciliation.gp_se2_environment import HospitalEnvironment
from plot_gp_se2_diag07 import plot_data,numeric,PLOTS


def validate(run):
    if (run/'validation.json').exists():raise FileExistsError('validation output already exists')
    begin=time.perf_counter();source=verify(run,frozen=True);env=HospitalEnvironment.load(source['environment_path']);config=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    errors=[];count=0
    def check(ok,label):
        nonlocal count
        count+=1
        if not bool(ok):errors.append(label)
    manifest=read(run/'reference_manifest.json');rows=manifest['records'];unique,aliases=deduplicate(rows)
    check(len(rows)==5 and sum(r['case_id']==HARD for r in rows)==4 and sum(r['case_id']==BENIGN for r in rows)==1,'five fixed provenance labels')
    check(unique==manifest['unique_references'] and aliases==read(run/'provenance_aliases.json'),'exact dedup and aliases')
    check(manifest['unique_reference_count']==len(unique),'unique count')
    summary=read(run/'aggregate/summary.json');independent=[]
    for r in rows:
        saved=read(Path(r['source_path']));case=load_frozen_case(source['primary_source'],r['case_id'],r['method'],environment=env)
        z,p,v,ref=support_reference(case['problem'],saved)
        check(digest(r['source_path'])==r['source_sha256'],'source record '+r['reference_id'])
        check(array_hash(z)==r['support_vector_sha256'] and array_hash(p)==r['support_poses_sha256'] and array_hash(v)==r['support_twists_sha256'],'support identity '+r['reference_id'])
        check(np.array_equal(ref,np.load(r['world_reference_path'],allow_pickle=False)) and array_hash(ref)==r['world_reference_sha256'],'reference exact support[1:] '+r['reference_id'])
        full=clean(original_full_check(case['problem'],z,case));recheck=read(Path(r['plan_recheck_path']))
        check(full==recheck['original_full_check'],'independent full check literal '+r['reference_id'])
        admission=plan_gate(full,hard=r['case_id']==HARD,changed_from_seed=not np.array_equal(z,saved['initial_vector']))
        check(admission==recheck['admission'],'admission '+r['reference_id'])
        check(all(r[k]==v for k,v in admission.items()),'manifest admission '+r['reference_id'])
        if r['case_id']==HARD:check(saved['candidate_vector'] is None and not saved['candidate_found'],'DIAG06 hard selection remains N/A')
        else:check(np.array_equal(saved['candidate_vector'],z),'benign optimized latest/retained')
    for r in unique:
        uid=r['unique_reference_id'];folder=run/'rollouts'/uid;roll=read(folder/'rollout.json')
        frozen=load_frozen_context(ONLINE,*r['case_id'].split('/'));ctx=read(run/'contexts'/(r['case_id'].replace('/','__')+'.json'))
        check(frozen==ctx['context'],'frozen source context '+uid)
        record_errors=validate_rollout_records(roll,frozen,np.load(r['world_reference_path'],allow_pickle=False))
        for error in record_errors:check(False,uid+': '+error)
        check(not record_errors,'complete actual record '+uid)
        evaluation=clean(evaluate_rollout(roll,ctx['goal_route'],env,config));check(evaluation==read(folder/'full_execution_evaluation.json'),'independent execution predicate '+uid)
        check(evaluation['execution']==read(folder/'execution_metrics.json'),'execution metrics '+uid)
        diag=execution_diagnostics(roll,evaluation,frozen);stored=read(folder/'diagnostics.json')
        check(all(clean(v)==stored[k] for k,v in diag.items()),'command/selection diagnostics '+uid)
        for s,pr in zip(roll['controller_reference_selections'],stored['prediction_geometry']):
            if s['prediction_world'] is None:check(not pr['available'],'missing prediction stays missing')
            else:
                actual=clean(environment_trace(env,np.asarray(s['prediction_world']),s['time_s']+np.arange(6)*.1,config))
                check(actual==pr['environment'],'saved prediction geometry '+uid+str(s['solve_index']))
        row=next(o for o in summary['outcomes'] if o['unique_reference_id']==uid)
        check(row['plan_valid']==r['plan_valid'] and not row['deployment_candidate'],'no execution promotion '+uid)
        check(row['primary_success']==evaluation['primary_success'] and row['failure_reasons']==evaluation['failure_reasons'],'primary failure preservation '+uid)
        check(row['failure_taxonomy']==diag['failure_taxonomy'],'execution failure taxonomy '+uid)
        for key,value in evaluation['execution'].items():
            if key in row:check(value==row[key],'execution metric '+uid+key)
        for key in ('first_command','first_delta_from_physical','final_command','achieved_sampled_terminal_dwell_s'):
            check(row[key]==diag[key],'command/dwell '+uid+key)
        check(row['minimum_clearance_m']==evaluation['minimum_clearance_m'],'clearance '+uid)
        check(row['lateral_invalid_execution_success']==bool(r['case_id']==HARD and evaluation['primary_success']),'semantic endpoint '+uid)
        independent.append(dict(case_id=r['case_id'],primary_success=evaluation['primary_success']))
        for name,key in [('states','states'),('commands','commands'),('selections','controller_reference_selections')]:check(read(folder/(name+'.json'))==roll[key],'saved stream '+uid+name)
        for path,h in read(folder/'hashes.json').items():check(digest(folder/path)==h,'worker output hash '+uid+path)
    actual=read(run/'rollouts/summary.json');audit=read(run/'historical_audits/summary.json')
    check(len(actual['records'])==len(unique) and actual['mpc_solves']==30*len(unique),'unique primary coverage')
    check([r['unique_reference_id'] for r in actual['records']]==[r['unique_reference_id'] for r in unique],'once-only frozen execution order')
    check(audit['mpc_solves']==2 and audit['passed'],'two historical audits')
    for case_id in (HARD,BENIGN):
        a=read(run/'historical_audits'/(case_id.replace('/','__')+'.json'));c=load_frozen_context(ONLINE,*case_id.split('/'));old=c['historical_first_fresh_solve']
        check(a['passed'] and np.max(np.abs(np.asarray(a['result']['command'])-old['command']))<=1e-6,'historical actual command agreement '+case_id)
        check(a['result']['selection']['indices']==old['selection']['indices'],'historical indices '+case_id)
    check(summary['interpretation']==experiment_interpretation(independent),'independent experiment interpretation')
    check(summary['primary_rollouts']==len(unique) and summary['primary_MPC_solves']==30*len(unique) and summary['total_MPC_solves']==30*len(unique)+2,'runtime totals')
    for key in ['new_GP','new_rigid','new_VLA','new_RGB','new_GUI','new_online_episode']:check(summary[key]==0,'forbidden runtime '+key)
    # Recompute source-linked numeric sidecars, not just their checksums.
    data=plot_data(run);check(data==read(run/'plot_data.json'),'plot data reconstruction')
    for name in PLOTS:
        p=run/'plots'/(name+'.json');d=read(p)
        check(d['numeric']==numeric(name,data),'numeric plot parity '+name)
        check(digest(p.with_suffix('.png'))==d['image_sha256'],'PNG hash '+name)
    with (run/'aggregate/execution_outcomes.csv').open() as f:csvrows=list(csv.DictReader(f))
    check(len(csvrows)==len(unique),'outcome table coverage')
    for row,c in zip(summary['outcomes'],csvrows):
        for key,value in row.items():
            # Existing table writer serializes lists as JSON and booleans literally.
            if isinstance(value,(dict,list)):
                import json
                check(json.loads(c[key])==value,'CSV structured '+key)
            else:check(c[key]==('' if value is None else str(value)),'CSV scalar '+key)
    with (run/'aggregate/plan_vs_execution.csv').open() as f:two=list(csv.DictReader(f))
    check(len(two)==2*len(unique) and [r['layer'] for r in two]==['PLAN','EXECUTION']*len(unique),'two independent layer rows')
    for rel,h in read(run/'artifact_hashes.json').items():check(digest(run/rel)==h,'artifact hash '+rel)
    verify(run,frozen=True)
    result=dict(valid=not errors,authoritative=True,check_count=count,errors=errors,wall_s=time.perf_counter()-begin,
        no_new_MPC=True,no_new_GP=True,no_new_VLA=True,no_new_rollout=True,
        scientific_execution_failure_is_artifact_failure=False,source_hash_count=len(source['preserved_hashes']))
    write(run/'validation.json',result);print(result,flush=True)
    if errors:raise SystemExit('artifact validation failed')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',required=True,type=Path);validate(p.parse_args().run.resolve())

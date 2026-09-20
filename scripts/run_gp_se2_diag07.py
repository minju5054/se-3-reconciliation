#!/usr/bin/env python3
"""Frozen DIAG-07 prepare/audit/freeze/execute/evaluate; no GP optimization."""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):
    os.environ[key]='1'
os.environ['PYTHONDONTWRITEBYTECODE']='1'
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
import yaml
from run_gp_se2_ref01 import read,write,digest,clean,table,utc,revision
from reconciliation.gp_se2_diag07_lateral_execution import (
    HARD,BENIGN,array_hash,support_reference,plan_gate,deduplicate,
    execution_diagnostics,experiment_interpretation,validate_rollout_records)
from reconciliation.gp_se2_diagnostics import load_frozen_case
from reconciliation.gp_se2_diag_acceptance import check_full_candidate as original_full_check
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_evaluation import evaluate_rollout
from reconciliation.gp_se2_rollout import load_frozen_context,verify_source_records
from reconciliation.online_mpc_adapter import PINNED_LIGHTNAV_SHA,PINNED_MPC_SHA256
D6=ROOT/'data/robotless_gp_se2_diag_06/primary_20260920T083200Z'
OUTPUT=ROOT/'data/robotless_gp_se2_diag_07'
ONLINE=ROOT/'data/robotless_online_handoffs_v1/primary_20260915T091900Z'
EXTERNAL=Path('/home/gpuadmin/Workspace/external/LightNav-0-official-demo')
PYTHON=EXTERNAL/'mujoco_demo/.venv/bin/python'
STARTING_SHA='f454a3de86a18160ec182cccab3f4857f1b92bf0'
CODE=['src/reconciliation/gp_se2_diag07_lateral_execution.py','scripts/run_gp_se2_diag07.py',
      'scripts/lightnav/gp_se2_diag07_mpc_rollout.py','scripts/plot_gp_se2_diag07.py',
      'scripts/validate_gp_se2_diag07.py','tests/test_gp_se2_diag07.py']
CORE=['se2.py','gp_se2.py','gp_se2_formulation.py','gp_se2_diagnostics.py','gp_se2_diag_acceptance.py',
      'gp_se2_rollout.py','gp_se2_evaluation.py','gp_se2_reference.py','gp_se2_environment.py',
      'online_mpc_adapter.py','robotless_online.py']

def external_state():
    sha=subprocess.check_output(['git','-C',str(EXTERNAL),'rev-parse','HEAD'],text=True).strip()
    status=subprocess.check_output(['git','-C',str(EXTERNAL),'status','--porcelain'],text=True).strip()
    path=EXTERNAL/'mujoco_demo/vln_mujoco/mpc.py'
    if sha!=PINNED_LIGHTNAV_SHA or status or digest(path)!=PINNED_MPC_SHA256:
        raise ValueError('official MPC source mismatch')
    return dict(checkout=str(EXTERNAL),sha=sha,git_status=status,mpc_path=str(path),mpc_sha256=digest(path),python=str(PYTHON))

def check_hashes(mapping):
    bad=[p for p,h in mapping.items() if not Path(p).is_file() or digest(p)!=h]
    if bad:raise ValueError('SOURCE_HASH_MISMATCH: '+str(bad[:12]))
    return len(mapping)

def verify(run,*,frozen=False):
    s=read(run/'source.json');check_hashes(s['preserved_hashes']);check_hashes(s['user_config_hashes'])
    external_state()
    if frozen:
        f=read(run/'execution_freeze.json');check_hashes(f['file_sha256'])
    return s

def source_inventory():
    s=read(D6/'source.json');expected=dict(s['preserved_hashes'])
    expected.update({str(ROOT/p):h for p,h in s['experiment_code_sha256'].items()})
    expected.update({str(D6/p):h for p,h in s['frozen_input_sha256'].items()})
    check_hashes(expected);check_hashes(s['user_config_hashes'])
    expected.update({str(p):digest(p) for p in D6.rglob('*') if p.is_file()})
    primary=Path(s['primary_source']);p=read(primary/'source.json')
    for key in ('input_sha256','preserved_core_sha256'):
        for path,h in p[key].items():
            path=Path(path);path=path if path.is_absolute() else ROOT/path
            expected[str(path)]=h
    for record in p['original_environment_file_hashes']:
        expected[str(Path(p['environment_path'])/record['path'])]=record['sha256']
    for path,record in p['authoritative_validations'].items():
        expected[path]=record['sha256']
    authority=[D6/'validation.json',primary/'validation.json',ONLINE/'validation.json']
    for path in authority:
        if not read(path)['valid']:raise ValueError('AUTHORITATIVE_VALIDATION_FAILED: '+str(path))
        expected[str(path)]=digest(path)
    # The original online validator authenticates its entire acquisition manifest.
    # Rehash all referenced raw files without rerunning any inference/rollout.
    for completion in sorted((ONLINE/'episodes').glob('*/completion.json')):
        expected[str(completion)]=digest(completion)
        for record in read(completion)['raw_manifest']['files']:
            path=completion.parent/record['path'];expected[str(path)]=record['sha256']
    for name in CORE:
        path=ROOT/'src/reconciliation'/name;expected.setdefault(str(path),digest(path))
    check_hashes(expected)
    return s,primary,expected,authority

def prepare(run):
    if run.exists() or not run.is_relative_to(OUTPUT):raise FileExistsError('new exclusive DIAG07 directory required')
    begin=time.perf_counter();old,primary,expected,authority=source_inventory();official=external_state()
    env=HospitalEnvironment.load(old['environment_path']);rows=[];contexts={}
    scheduled=read(D6/'experiment_manifest.json')['starts']
    if [(r['case_id'],r['method'],r['initialization']) for r in scheduled] != [
        (HARD,'M2_GP_NO_OBSTACLE','I0_FRESH'),(HARD,'M2_GP_NO_OBSTACLE','I1_DECEL'),
        (HARD,'M3_GP_CONSTRAINED','I0_FRESH'),(HARD,'M3_GP_CONSTRAINED','I1_DECEL'),
        (BENIGN,'M3_GP_CONSTRAINED','I1_DECEL')]:raise ValueError('five frozen source labels changed')
    run.mkdir(parents=True);shutil.copyfile(D6/'config_snapshot.yaml',run/'config_snapshot.yaml')
    for k,row in enumerate(scheduled):
        label=('hard' if k<4 else 'benign')+'_'+row['method'][:2].lower()+'_'+row['initialization'][:2].lower()
        path=D6/'solves'/row['solve_id']/'solver_result.json';saved=read(path)
        case=load_frozen_case(primary,row['case_id'],row['method'],environment=env)
        context=load_frozen_context(ONLINE,*row['case_id'].split('/'))
        for key in ('B_world','u_minus','previous_control','original_capture_pose_world'):
            if context[key]!=case['context'][key]:raise ValueError('historical/current input mismatch: '+key)
        contexts[row['case_id']]=dict(context=context,goal_route=case['goal_route'])
        z,poses,twists,reference=support_reference(case['problem'],saved)
        t=time.perf_counter();full=clean(original_full_check(case['problem'],z,case));check_s=time.perf_counter()-t
        original=next(c['full_acceptance'] for c in saved['candidate_checks'] if c['iterate']=='latest_iterate')
        if full!=original:raise ValueError('DIAG06 full-check literal reproduction failed: '+label)
        if k<4 and (saved['candidate_vector'] is not None or saved['candidate_found']):raise ValueError('historical rejected status changed')
        admission=plan_gate(full,hard=k<4,changed_from_seed=not np.array_equal(z,saved['initial_vector']))
        if k==4 and not np.array_equal(z,saved['candidate_vector']):raise ValueError('benign latest/retained identity differs')
        write(run/'plan_recheck'/f'{label}.json',dict(admission=admission,original_full_check=full,
            matches_DIAG06_literally=True,wall_s=check_s))
        folder=run/'references'/label;folder.mkdir(parents=True)
        for name,value in [('latest_vector',z),('support_poses',poses),('support_twists',twists),('world_reference',reference)]:
            with (folder/(name+'.npy')).open('xb') as f:np.save(f,value,allow_pickle=False)
        state={key:context[key] for key in ('case_id','B_world','u_minus','previous_control','original_capture_pose_world')}
        state['goal_route']=case['goal_route'];state['config_sha256']=digest(run/'config_snapshot.yaml')
        statehash=hashlib.sha256(json.dumps(state,sort_keys=True,separators=(',',':')).encode()).hexdigest()
        worldpath=folder/'world_reference.npy'
        rows.append(dict(reference_id=label,case_id=row['case_id'],case_role=row['case_role'],method=row['method'],
            initialization=row['initialization'],source_path=str(path),source_sha256=digest(path),source_field='latest_iterate',
            support_vector_sha256=array_hash(z),support_poses_sha256=array_hash(poses),support_twists_sha256=array_hash(twists),
            world_reference_path=str(worldpath),world_reference_sha256=array_hash(reference),world_reference_file_sha256=digest(worldpath),
            state_identity_sha256=statehash,shape=list(reference.shape),dtype=str(reference.dtype),
            plan_recheck_path=str(run/'plan_recheck'/f'{label}.json'),all_full_check_flags=full['additional_grid']['flags'],
            max_absolute_plan_vy_mps=full['additional_grid']['maximum_absolute_lateral_velocity_m_s'],
            plan_recheck_wall_s=check_s,reference_extraction='saved latest right-chart unpack; support_poses[1:] exactly; no new interpolation',**admission))
        expected.update(case['hashes']);expected[str(path)]=digest(path)
        for rec in context['source_files']:expected[str(ONLINE/rec['path'])]=rec['sha256']
        print(json.dumps(dict(reference=label,plan_valid=admission['plan_valid'],failed=[k for k,v in full['additional_grid']['flags'].items() if not v])),flush=True)
    unique,aliases=deduplicate(rows)
    for case_id,value in contexts.items():write(run/'contexts'/(case_id.replace('/','__')+'.json'),value)
    write(run/'reference_manifest.json',dict(provenance_record_count=5,hard_provenance_count=4,
        unique_reference_count=len(unique),unique_hard_reference_count=sum(r['case_id']==HARD for r in unique),
        records=rows,unique_references=unique,deduplication='exact float64 C-order reference bytes plus exact frozen state; no approximate merging'))
    write(run/'provenance_aliases.json',aliases)
    protocol=dict(experiment='GP-SE2-DIAG-07',execution_order=[r['unique_reference_id'] for r in unique],
        reference_count=5,unique_rollouts=len(unique),primary_MPC_solves=30*len(unique),historical_audit_solves=2,
        horizon_s=3.,control_hz=10.,integration_hz=60.,horizon_rows=5,MPC_dt_s=.1,
        selector='unchanged official nearest weighted XY/wrapped-yaw; next rows; original tie break',
        full_plan_acceptance_unchanged=True,hard_admission='explicit plan-invalid diagnostic authorization only',
        gp_body_velocity_feed_forward=False,retry=False,source_progress_selector=False,
        plan_lateral_tolerance_mps=case['config']['formulation']['equality_tolerance'],
        world_roundtrip_tolerance=1e-11,historical_command_atol=1e-6,
        no_runtime=['VLA','GP optimization','rigid optimization','RGB capture','Isaac GUI','online episode'],
        prediction_diagnostic='saved Euler prediction polyline; no unrecorded control-sequence reconstruction',
        prediction_snapshots_s=[0.,1.,2.],missing='N/A, never zero',frame=old['source_frame'])
    write(run/'protocol.json',protocol)
    historical=[]
    for method in ('M0_NATIVE','M0_ADAPTER','M1_RIGID'):
        path=primary/'cases'/HARD.replace('/','__')/'methods'/method/'metrics.json'
        oldmetrics=read(path);expected[str(path)]=digest(path)
        historical.append(dict(method=method,provenance='HISTORICAL CONTEXT — NOT NEW ROLLOUT',
            path=str(path),sha256=digest(path),metrics=oldmetrics))
    write(run/'historical_context/gp_se2_02_summary.json',dict(case_id=HARD,records=historical,rerun=False))
    request=dict(source_root=str(ONLINE),unique_references=unique,context_paths={c:str(run/'contexts'/(c.replace('/','__')+'.json')) for c in contexts},protocol=protocol)
    write(run/'mpc_request.json',request)
    write(run/'source.json',dict(experiment='GP-SE2-DIAG-07',starting_sha=STARTING_SHA,preparation_sha=revision(),
        diag06=str(D6),primary_source=str(primary),online_source=str(ONLINE),environment_path=old['environment_path'],
        environment_export_source=old['environment_export_source'],official=official,
        preserved_hashes=expected,user_config_hashes=old['user_config_hashes'],
        authoritative_validations={str(p):digest(p) for p in authority},preparation_wall_s=time.perf_counter()-begin))
    print(json.dumps(dict(prepared=str(run),unique=len(unique),source_files=len(expected))),flush=True)

def worker(run,stage):
    verify(run,frozen=stage=='execute')
    subprocess.run([str(PYTHON),str(ROOT/'scripts/lightnav/gp_se2_diag07_mpc_rollout.py'),
                    '--run',str(run),'--stage',stage],check=True,cwd=ROOT,env=os.environ.copy())

def freeze(run):
    verify(run)
    if not read(run/'historical_audits/summary.json')['passed']:raise ValueError('historical audit gate failed')
    files={str(run/p):digest(run/p) for p in ['source.json','protocol.json','reference_manifest.json','provenance_aliases.json','mpc_request.json','config_snapshot.yaml']}
    files.update({str(ROOT/p):digest(ROOT/p) for p in CODE})
    files.update({str(p):digest(p) for parent in ['references','contexts','plan_recheck','historical_audits','historical_context'] for p in (run/parent).rglob('*') if p.is_file()})
    for p in CODE:
        head=subprocess.check_output(['git','show','HEAD:'+p],cwd=ROOT)
        if hashlib.sha256(head).hexdigest()!=digest(ROOT/p):raise ValueError('execution code not committed: '+p)
    write(run/'execution_freeze.json',dict(utc=utc(),execution_sha=revision(),file_sha256=files,all_pre_execution_gates_passed=True))

def evaluate(run):
    s=verify(run,frozen=True);begin=time.perf_counter();env=HospitalEnvironment.load(s['environment_path'])
    config=yaml.safe_load((run/'config_snapshot.yaml').read_text());manifest=read(run/'reference_manifest.json');out=[]
    for r in manifest['unique_references']:
        uid=r['unique_reference_id'];folder=run/'rollouts'/uid;rollout=read(folder/'rollout.json');ctx=read(run/'contexts'/(r['case_id'].replace('/','__')+'.json'))
        errors=validate_rollout_records(rollout,ctx['context'],np.load(r['world_reference_path'],allow_pickle=False))
        if errors:raise ValueError(str(errors))
        t=time.perf_counter();evaluation=clean(evaluate_rollout(rollout,ctx['goal_route'],env,config));duration=time.perf_counter()-t
        diag=execution_diagnostics(rollout,evaluation,ctx['context'])
        # Evaluate all saved predictions spatially, not as executed trajectories.
        from reconciliation.gp_se2_evaluation import environment_trace
        prediction_reports=[]
        for solve in rollout['controller_reference_selections']:
            pred=solve['prediction_world']
            if pred is None:
                prediction_reports.append(dict(issue_time_s=solve['time_s'],available=False,reason='controller prediction unavailable'))
            else:
                q=np.asarray(pred);tt=solve['time_s']+np.arange(6)*.1
                report=clean(environment_trace(env,q,tt,config))
                prediction_reports.append(dict(issue_time_s=solve['time_s'],available=True,semantics='SAVED_DISCRETE_PREDICTION_POLYLINE',environment=report))
        diag['prediction_geometry']=prediction_reports
        write(folder/'full_execution_evaluation.json',evaluation);write(folder/'execution_metrics.json',evaluation['execution']);write(folder/'diagnostics.json',diag)
        row=dict(unique_reference_id=uid,provenance_labels=r['provenance_labels'],case_id=r['case_id'],case_role=r['case_role'],
            reference_kind=r['reference_kind'],plan_valid=r['plan_valid'],deployment_candidate=False,
            plan_failure_reason=r['plan_failure_reason'],diagnostic_execution_only=True,
            primary_success=evaluation['primary_success'],failure_reasons=evaluation['failure_reasons'],failure_taxonomy=diag['failure_taxonomy'],
            lateral_invalid_execution_success=bool(not r['plan_valid'] and evaluation['primary_success']),
            plan_max_absolute_vy_mps=r['max_absolute_plan_vy_mps'],
            minimum_clearance_m=evaluation['minimum_clearance_m'],clearance_pass=evaluation['environment']['clearance_valid'],
            physical_overlap=evaluation['environment']['physical_overlap'],workspace_pass=evaluation['environment']['workspace_known'],route_pass=evaluation['route']['valid'],
            **{k:evaluation['execution'][k] for k in ['time_to_goal_s','terminal_position_error_m','terminal_yaw_error_rad',
                'terminal_goal_dwell_pass','motion_limits_pass','controller_failure_count','path_length_m',
                'command_total_variation_v_mps','command_total_variation_omega_radps',
                'control_grid_max_abs_acceleration_v_mps2','control_grid_max_abs_acceleration_omega_radps2']},
            first_command=diag['first_command'],first_delta_from_physical=diag['first_delta_from_physical'],
            final_command=diag['final_command'],achieved_sampled_terminal_dwell_s=diag['achieved_sampled_terminal_dwell_s'],
            first_final_reference_row_in_horizon_s=diag['first_final_row_in_horizon_s'],
            final_reference_row_maintained=diag['final_row_maintained_after_first'],
            actual_lateral_command_dimension='nonexistent',rollout_wall_s=rollout['rollout_wall_s'],
            mpc_solve_wall_s=sum(t for t in rollout['official_mpc_solve_wall_s'] if t is not None),
            evaluation_wall_s=duration)
        out.append(row);print(json.dumps(row),flush=True)
    (run/'aggregate').mkdir(exist_ok=False)
    table(run/'aggregate/execution_outcomes.csv',out)
    table(run/'aggregate/unique_references.csv',manifest['unique_references'])
    two=[]
    for r in out:
        two.extend([dict(r,layer='PLAN',layer_pass=r['plan_valid']),dict(r,layer='EXECUTION',layer_pass=r['primary_success'])])
    table(run/'aggregate/plan_vs_execution.csv',two)
    table(run/'aggregate/command_metrics.csv',[{k:v for k,v in r.items() if k in ['unique_reference_id','first_command','first_delta_from_physical','final_command','actual_lateral_command_dimension'] or 'acceleration' in k or 'variation' in k} for r in out])
    table(run/'aggregate/selection_metrics.csv',[{k:r[k] for k in ['unique_reference_id','first_final_reference_row_in_horizon_s','final_reference_row_maintained','terminal_yaw_error_rad']} for r in out])
    table(run/'aggregate/timing.csv',[{k:r[k] for k in ['unique_reference_id','rollout_wall_s','mpc_solve_wall_s','evaluation_wall_s']} for r in out])
    summary=dict(operational_status='NUMERICAL_EXECUTION_COMPLETE_PENDING_ARTIFACT_VALIDATION',
        interpretation=experiment_interpretation(out),provenance_count=5,unique_reference_count=len(out),unique_hard_count=sum(r['case_id']==HARD for r in out),
        hard_execution_successes=sum(r['lateral_invalid_execution_success'] for r in out),
        benign_execution_preserved=next(r['primary_success'] for r in out if r['case_id']==BENIGN),
        primary_rollouts=len(out),primary_MPC_solves=30*len(out),historical_audit_MPC_solves=2,total_MPC_solves=30*len(out)+2,
        new_GP=0,new_rigid=0,new_VLA=0,new_RGB=0,new_GUI=0,new_online_episode=0,
        plan_acceptance_unchanged=True,hard_deployment_candidates=0,evaluation_stage_wall_s=time.perf_counter()-begin,
        outcomes=out,limitations=['one hard event; four starts are not independent handoffs','plan validity is unchanged by execution',
        'failure does not establish causal necessity of lateral rejection','robotless exact unicycle; no physical collision response',
        'offline synchronous MPC; not online latency','no new observations; no tolerance relaxation'])
    write(run/'aggregate/summary.json',summary)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('stage',choices=['prepare','audit','freeze','execute','evaluate']);p.add_argument('--run',type=Path,required=True)
    a=p.parse_args();run=a.run.resolve()
    if a.stage in ('audit','execute'):worker(run,a.stage)
    else:globals()[a.stage](run)
if __name__=='__main__':main()

#!/usr/bin/env python3
"""Exclusive G4 freeze, five GP starts, eligible fixed-interface executions."""
from __future__ import annotations
import argparse
import copy
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):os.environ[key]='1'
os.environ['JAX_PLATFORMS']='cpu';os.environ['JAX_ENABLE_X64']='true';os.environ['PYTHONDONTWRITEBYTECODE']='1'
os.environ.setdefault('XLA_FLAGS','--xla_cpu_multi_thread_eigen=false intra_op_parallelism_threads=1')
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
import yaml
import run_gp_se2_diag06 as d6
import run_gp_se2_diag07 as d7
from run_gp_se2_ref01 import read,write,copy_new,digest,clean,table,utc,revision
from reconciliation.gp_se2_diag08_endpoint_margin import (
    G4,DEFINITION,G4EndpointMarginView,G4EndpointMarginDerivatives,reserve_report,
    execution_admission,endpoint_geometry,classify,validate_execution_records)
from reconciliation.gp_se2_diag07_lateral_execution import (
    HARD,BENIGN,array_hash,support_reference,deduplicate,execution_diagnostics)
from reconciliation.gp_se2_diag06_validation import PROTOCOL as D6_PROTOCOL,verify_point as verify_G3
from reconciliation.gp_se2_diag02_validation import error_report
from reconciliation.gp_se2_diag02_derivatives import DerivativeError
from reconciliation.gp_se2_diag04_constraints import constraint_row_metadata
from reconciliation.gp_se2_diag04_solver import run_refined
from reconciliation.gp_se2_diag03_intervals import audit_intervals
from reconciliation.gp_se2_diag06_witnesses import extract_runs
from reconciliation.gp_se2_diag_acceptance import check_full_candidate
from reconciliation.gp_se2_diagnostics import load_frozen_case
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_evaluation import evaluate_rollout
from reconciliation.gp_se2_formulation import _constraint_report
D6=d7.D6
D7=ROOT/'data/robotless_gp_se2_diag_07/primary_20260920T103000Z'
OUTPUT=ROOT/'data/robotless_gp_se2_diag_08'
STARTING_SHA='1fa1e1e83074218b44456e4a00c64947a2591760'
LABELS=['hard_m2_i0','hard_m2_i1','hard_m3_i0','hard_m3_i1','benign_m3_i1']
CODE=['src/reconciliation/gp_se2_diag08_endpoint_margin.py','scripts/run_gp_se2_diag08.py',
 'scripts/lightnav/gp_se2_diag08_mpc_rollout.py','scripts/plot_gp_se2_diag08.py',
 'scripts/validate_gp_se2_diag08.py','tests/test_gp_se2_diag08.py']


def save_array(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('xb') as f:np.save(f,np.asarray(value,dtype=np.float64),allow_pickle=False)


def verify(run, frozen=False):
    source=read(run/'source.json')
    for key in ('preserved_hashes','user_config_hashes','code_hashes','input_hashes'):d7.check_hashes(source[key])
    if d7.external_state()!=source['official']:raise ValueError('external MPC changed')
    if frozen:d7.check_hashes(read(run/'execution_freeze.json')['file_sha256'])
    return source


def prepare(run):
    if run.exists() or not run.is_relative_to(OUTPUT):raise FileExistsError('new exclusive DIAG08 run required')
    begin=time.perf_counter();prior=d7.verify(D7,frozen=True);old=d6.verify_frozen(D6)
    authority=[D6/'validation.json',D7/'verification/validation.json',D7/'delivery_validation.json']
    if not all(read(p)['valid'] for p in authority):raise ValueError('authoritative validation failed')
    d7.check_hashes({str(D7/p):h for p,h in read(D7/'verification/output_hashes.json').items()})
    preserved=dict(prior['preserved_hashes'])
    preserved.update({str(p):digest(p) for parent in [D6,D7] for p in parent.rglob('*') if p.is_file()})
    # Preserve imported numerical, checker, wrapper and runner dependencies.
    preserved.update({str(p):digest(p) for folder in [ROOT/'src/reconciliation',ROOT/'scripts']
                      for p in folder.rglob('*.py') if str(p.relative_to(ROOT)) not in CODE})
    run.mkdir(parents=True)
    copy_new(D6/'config_snapshot.yaml',run/'config_snapshot.yaml')
    copy_new(D6/'witness_selection/frozen_union.json',run/'margin/frozen_g3_witness_union.json')
    if len(read(run/'margin/frozen_g3_witness_union.json'))!=3:raise ValueError('frozen three-row union required')
    write(run/'margin/definition.json',DEFINITION)
    oldrows=read(D6/'experiment_manifest.json')['starts'];historical=read(D7/'reference_manifest.json')
    rows=[]
    for i,(oldrow,label,hist) in enumerate(zip(oldrows,LABELS,historical['records'])):
        if (oldrow['case_id'],oldrow['method'],oldrow['initialization'])!=(hist['case_id'],hist['method'],hist['initialization']):raise ValueError('paired manifest mismatch')
        result_path=D6/'solves'/oldrow['solve_id']/'solver_result.json';result=read(result_path)
        original=Path(oldrow['original_seed_path']);seed=D6/oldrow['derived_seed_path']
        if digest(seed)!=oldrow['seed_sha256'] or digest(seed)!=digest(original):raise ValueError('saved seed bytes changed')
        z=np.load(seed,allow_pickle=False)
        if array_hash(z)!=array_hash(result['initial_vector']):raise ValueError('initial vector differs')
        target=run/'initializations'/(label+'.npy');copy_new(seed,target)
        rows.append(dict(pair_index=i,solve_id=label,case_id=oldrow['case_id'],case_role=oldrow['case_role'],
            method=oldrow['method'],initialization=oldrow['initialization'],grid=G4,
            seed_path=str(target),seed_sha256=digest(target),seed_vector_sha256=array_hash(z),
            original_seed_path=str(original),g3_seed_path=str(seed),g3_result_path=str(result_path),
            g3_result_sha256=digest(result_path),historical_reference=hist,
            historical_uid=read(D7/'provenance_aliases.json')[label]))
    if len(rows)!=5:raise ValueError('exact five starts required')
    for caseid in [HARD,BENIGN]:
        copy_new(D7/'contexts'/(caseid.replace('/','__')+'.json'),run/'contexts'/(caseid.replace('/','__')+'.json'))
    points=copy.deepcopy(read(D6/'derivative_checks/point_manifest.json'))
    for p in points['points']:
        if p['kind']=='saved_G2_latest':
            result=read(Path(rows[p['pair_index']]['g3_result_path']))
            p.update(kind='saved_G3_latest',name=f"pair_{p['pair_index']}__G3_latest",vector=result['latest_iterate'],
                vector_sha256=array_hash(result['latest_iterate']),used_as_optimization_seed=False)
    write(run/'margin/point_manifest.json',points)
    write(run/'experiment_manifest.json',dict(starts=rows,planned_new_GP_solves=5,execution_order=LABELS))
    write(run/'historical_g3/diag07_manifest.json',dict(source=str(D7),records=historical['records'],
        aliases=read(D7/'provenance_aliases.json'),rerun=False,authoritative_validation=str(authority[1])))
    protocol=dict(experiment='GP-SE2-DIAG-08',definition=DEFINITION,
        expected_dimensions={'M2':[150,30,1297],'M3':[150,30,1387]},execution_order=LABELS,
        planned_GP_solves=5,max_iterations=200,ftol=1e-7,prepared_budget_s=30.,single_thread_BLAS=True,
        retention_policy='unchanged run_refined: grid+dense, minimum original objective, original full check never reselects',
        execution_reference='latest_iterate only for all five; selected retained candidate separately reported',
        execution_admission='strict endpoint distance <= 0.11 AND original full valid or hard lateral-only invalid',
        nonlateral_supplemental_motion_required=True,original_checker_unchanged=True,
        derivative_protocol={k:D6_PROTOCOL[k] for k in ['directional_fd_steps','primal_tolerance','constraint_derivative_tolerance']},
        derivative_points='5 original seeds, 5 saved G3 latest, 5 unchanged perturbations; 3 original directions',
        margin_tuning=False,retry=False,historical_reruns=0,world_frame=old['source_frame'],
        horizon_s=3.,control_hz=10.,integration_hz=60.,MPC_horizon=5,MPC_dt_s=.1,
        new_VLA=0,new_RGB=0,new_GUI=0,new_online_episode=0,meaningful_error_improvement_m=1e-6,
        historical_audits='one per eligible source event at original recorded solve state; separately counted',
        deduplication='exact float64 reference bytes plus frozen state/config; no approximate merging',
        software={p:importlib.metadata.version(p) for p in ['numpy','scipy','jax','jaxlib','shapely','matplotlib']})
    write(run/'protocol.json',protocol)
    inputs={str(p):digest(p) for p in run.rglob('*') if p.is_file()}
    write(run/'source.json',dict(starting_sha=STARTING_SHA,preparation_sha=revision(),experiment='GP-SE2-DIAG-08',
        primary_source=old['primary_source'],environment_path=old['environment_path'],
        environment_export_source=old['environment_export_source'],diag06=str(D6),diag07=str(D7),
        official=d7.external_state(),authoritative_validations={str(p):digest(p) for p in authority},
        preserved_hashes=preserved,user_config_hashes=old['user_config_hashes'],input_hashes=inputs,
        code_hashes={str(ROOT/p):digest(ROOT/p) for p in CODE},
        preparation_wall_s=time.perf_counter()-begin))
    print(json.dumps(dict(prepared=str(run),preserved_files=len(preserved),planned_GP_solves=5)),flush=True)


def providers(base,env,rows):
    inherited=d6.providers(base,env,rows)
    view=G4EndpointMarginView(inherited[-2]);provider=G4EndpointMarginDerivatives(view,inherited[-1])
    return inherited,view,provider


def verify_margin_point(point,case,inherited,view,provider,directions):
    base=case['problem'];z=np.asarray(point['vector']);geo,original,g2,p2,g3,p3=inherited
    prior,arrays=verify_G3(point['name'],z,base,geo,original,g2,p2,g3,p3,directions)
    provider.warmup(z);a=g3.evaluate(z);b=view.evaluate(z);n=len(a['inequality'])
    jac=provider.inequality_jacobian(z);oldjac=p3.inequality_jacobian(z)
    parity=dict(objective=b['objective']==a['objective'],equality=np.array_equal(b['equality'],a['equality']),
        inequality_prefix=np.array_equal(b['inequality'][:-1],a['inequality']),
        objective_gradient=np.array_equal(provider.objective_gradient(z),p3.objective_gradient(z)),
        equality_jacobian=np.array_equal(provider.equality_jacobian(z),p3.equality_jacobian(z)),
        inequality_jacobian_prefix=np.array_equal(jac[:-1],oldjac),
        new_row_equals_original_goal_jacobian=np.array_equal(jac[-1],oldjac[view.goal_row]))
    expected=.11**2-np.sum((base.unpack(z)[0][-1,:2]-base.goal_pose[:2])**2)
    primal=error_report(provider.values(z)['endpoint_margin'][:,None],np.array([[expected]]),D6_PROTOCOL['primal_tolerance'])
    directional=[]
    for h in D6_PROTOCOL['directional_fd_steps']:
        fd=np.array([(view.evaluate(z+h*d)['endpoint_margin'][0]-view.evaluate(z-h*d)['endpoint_margin'][0])/(2*h) for d in directions])
        pred=jac[-1]@directions.T
        directional.append(dict(step=h,AD=pred.tolist(),FD=fd.tolist(),
            **error_report(pred[None,:],fd[None,:],D6_PROTOCOL['constraint_derivative_tolerance'])))
    shape=(b['inequality'].shape==(n+1,) and jac.shape==(n+1,150) and provider.equality_jacobian(z).shape==(30,150))
    rows=constraint_row_metadata(view,z)['rows']
    goal=[r for r in rows if r['family']=='goal_position' and r['source']=='original_base']
    source_row_ok=len(goal)==1 and goal[0]['row_index']==view.goal_row
    valid=prior['valid'] and all(parity.values()) and shape and source_row_ok and primal['passed'] and all(r['passed'] for r in directional[-2:])
    arrays.update(endpoint_jacobian=jac[-1],endpoint_primal_numpy=np.array([expected]),endpoint_primal_AD=provider.values(z)['endpoint_margin'])
    return dict(name=point['name'],valid=bool(valid),G3_validation=prior,parity=parity,
        dimensions=view.dimensions(z),shapes_valid=shape,goal_source_row_verified=source_row_ok,
        primal=primal,directional=directional,no_FD_fallback=True),arrays


def derivatives(run):
    source=verify(run);folder=run/'margin/derivative_checks';write(folder/'started.json',dict(utc=utc()))
    begin=time.perf_counter();env=HospitalEnvironment.load(source['environment_path']);cache={};reports=[]
    points=read(run/'margin/point_manifest.json');witnesses=read(run/'margin/frozen_g3_witness_union.json')
    for point in points['points']:
        key=(point['case_id'],point['method'])
        if key not in cache:
            case=load_frozen_case(Path(source['primary_source']),*key,environment=env)
            cache[key]=(case,*providers(case['problem'],env,witnesses))
        try:
            report,arrays=verify_margin_point(point,*cache[key],np.asarray(points['directions']))
            with (folder/(point['name']+'.npz')).open('xb') as f:np.savez_compressed(f,**arrays)
        except (DerivativeError,ValueError,FloatingPointError) as exc:
            report=dict(name=point['name'],valid=False,error=str(exc),status='DERIVATIVE_VALIDATION_FAILED')
        write(folder/(point['name']+'.json'),report);reports.append(report)
        print(json.dumps(dict(point=point['name'],valid=report['valid'])),flush=True)
    valid=len(reports)==15 and all(r['valid'] for r in reports)
    write(run/'margin/derivative_validation.json',dict(valid=valid,point_count=len(reports),
        status='VERIFIED_WITH_DECLARED_BRANCH_LIMITATIONS' if valid else 'DERIVATIVE_VALIDATION_FAILED',
        wall_s=time.perf_counter()-begin,all_prefix_parities=all(all(r.get('parity',{}).values()) for r in reports),new_GP_solves=0))
    if not valid:raise SystemExit('derivative gate failed; primary prohibited')


def freeze(run):
    verify(run)
    if not read(run/'margin/derivative_validation.json')['valid']:raise ValueError('derivative gate failed')
    for path in CODE:
        if hashlib.sha256(subprocess.check_output(['git','show','HEAD:'+path],cwd=ROOT)).hexdigest()!=digest(ROOT/path):
            raise ValueError('code not committed: '+path)
    paths={str(p):digest(p) for p in run.rglob('*') if p.is_file() and p.suffix!='.log'}
    paths.update({str(ROOT/p):digest(ROOT/p) for p in CODE})
    write(run/'execution_freeze.json',dict(execution_sha=revision(),utc=utc(),file_sha256=paths,
        all_pre_solve_gates_passed=True,reserve_m=.04,planning_radius_m=.11,execution_radius_m=.15))


def solve(run):
    source=verify(run,True)
    if revision()!=read(run/'execution_freeze.json')['execution_sha']:raise ValueError('frozen execution SHA required')
    write(run/'optimization_started.json',dict(utc=utc(),order=LABELS,retry=False))
    begin=time.perf_counter();t=time.perf_counter();env=HospitalEnvironment.load(source['environment_path']);env_s=time.perf_counter()-t
    witnesses=read(run/'margin/frozen_g3_witness_union.json');actual=0
    for row in read(run/'experiment_manifest.json')['starts']:
        folder=run/'solves'/row['solve_id'];write(folder/'started.json',dict(row,utc=utc()));t=time.perf_counter()
        case=load_frozen_case(Path(source['primary_source']),row['case_id'],row['method'],environment=env)
        z=np.load(row['seed_path'],allow_pickle=False)
        if digest(row['seed_path'])!=row['seed_sha256'] or array_hash(z)!=row['seed_vector_sha256']:raise ValueError('seed identity')
        loading=time.perf_counter()-t;t=time.perf_counter();_,view,provider=providers(case['problem'],env,witnesses);construction=time.perf_counter()-t
        t=time.perf_counter();provider.warmup(z);warmup=time.perf_counter()-t;stats=provider.stats();dims=view.dimensions(z)
        if (dims['variable_count'],dims['equality_count'],dims['inequality_count'])!=(150,30,1297 if row['method'].startswith('M2') else 1387):raise ValueError('G4 dimensions')
        write(folder/'constraint_rows.json',constraint_row_metadata(view,z));provider.reset_stats();view.clear_cache()
        result=run_refined(view,z,derivative_provider=provider,case=case,initialization_name=row['initialization'])
        result.update(solve_id=row['solve_id'],case_id=row['case_id'],method=row['method'],provenance='NEW_DIAG08_G4',
            input_loading_wall_s=loading,provider_construction_wall_s=construction,compilation_warmup_wall_s=warmup,
            warmup_stats=stats,constraint_dimensions=dims,
            cold_setup_solve_validation_s=loading+construction+warmup+result['total_setup_solve_post_wall_time_s'])
        write(folder/'solver_result.json',result);actual+=result['minimize_invocations']
        print(json.dumps(dict(solve=row['solve_id'],status=result['solver_status'],iterations=result['iterations'],selected_full=result['selected_full_feasible'])),flush=True)
    write(run/'optimization_completed.json',dict(actual_new_GP_solves=actual,planned_new_GP_solves=5,
        wall_s=time.perf_counter()-begin,environment_loading_s=env_s,new_MPC_solves=0,retry=False))


def analyze_plans(run):
    source=verify(run,True);write(run/'plan_analysis_started.json',dict(utc=utc()));begin=time.perf_counter()
    env=HospitalEnvironment.load(source['environment_path']);rows=[]
    witnesses=read(run/'margin/frozen_g3_witness_union.json')
    for row in read(run/'experiment_manifest.json')['starts']:
        t=time.perf_counter();folder=run/'solves'/row['solve_id'];result=read(folder/'solver_result.json')
        case=load_frozen_case(Path(source['primary_source']),row['case_id'],row['method'],environment=env)
        base=case['problem'];view=G4EndpointMarginView(d6.WitnessView(base,witnesses));checks=[];cache={}
        for check in result['candidate_checks']:
            z=check['vector'];identity=array_hash(z)
            if identity not in cache:
                iv=audit_intervals(base,z);runs=extract_runs(iv['finest_trace'],base.config,row['solve_id'])
                compact=d6.d5.compact_intervals(iv);write(folder/'intervals'/(identity+'.json'),compact)
                cache[identity]=dict(reserve=reserve_report(base,z),motion_violating_runs=runs,
                    supplemental_motion_pass=not runs,interval_path=str(folder/'intervals'/(identity+'.json')))
            checks.append(dict(iterate=check['iterate'],vector_sha256=identity,
                grid_feasible=check['collocation']['feasible'],original_dense_feasible=check['constraint_report']['feasible'],
                original_full_feasible=check['original_full_feasible'],**cache[identity]))
        z,poses,twists,reference=support_reference(base,result)
        latest=next(c for c in result['candidate_checks'] if c['iterate']=='latest_iterate')
        full=clean(check_full_candidate(base,z,case))
        if full!=latest['full_acceptance']:raise ValueError('post-solve full reproduction mismatch')
        reserve=reserve_report(base,z);admission=execution_admission(full,reserve,hard=row['case_role']=='HARD')
        extra=cache[array_hash(z)]
        if extra['motion_violating_runs']:
            admission.update(eligible_for_execution=False,diagnostic_execution_authorized=False)
            admission['ineligible_reasons']=list(admission['ineligible_reasons'])+['supplemental_motion_violation']
        write(run/'plan_recheck'/(row['solve_id']+'.json'),dict(original_full_check=full,reserve=reserve,
            admission=admission,source='latest_iterate fixed before solve',candidate_checks=checks,
            selected_source=result['selected_iterate'],selected_full_feasible=result['selected_full_feasible'],wall_s=time.perf_counter()-t))
        folder=run/'references'/row['solve_id']
        for name,value in [('latest_vector',z),('support_poses',poses),('support_twists',twists),('world_reference',reference)]:save_array(folder/(name+'.npy'),value)
        h=row['historical_reference'];path=folder/'world_reference.npy'
        record=dict(reference_id=row['solve_id'],case_id=row['case_id'],case_role=row['case_role'],method=row['method'],
            initialization=row['initialization'],world_reference_path=str(path),world_reference_file_sha256=digest(path),
            world_reference_sha256=array_hash(reference),state_identity_sha256=h['state_identity_sha256'],
            shape=list(reference.shape),dtype=str(reference.dtype),plan_recheck_path=str(run/'plan_recheck'/(row['solve_id']+'.json')),
            historical_uid=row['historical_uid'],source_path=str(run/'solves'/row['solve_id']/'solver_result.json'),
            source_sha256=digest(run/'solves'/row['solve_id']/'solver_result.json'),source_field='latest_iterate',**admission)
        rows.append(record)
        print(json.dumps(dict(reference=row['solve_id'],eligible=admission['eligible_for_execution'],plan_valid=admission['plan_valid'],
            endpoint=reserve['endpoint_position_error_m'],ineligible=admission['ineligible_reasons'])),flush=True)
    unique,aliases=deduplicate([r for r in rows if r['eligible_for_execution']])
    write(run/'reference_manifest.json',dict(records=rows,unique_references=unique,aliases=aliases,
        planned_provenance_count=5,eligible_provenance_count=sum(r['eligible_for_execution'] for r in rows),unique_rollouts=len(unique)))
    contexts={r['case_id']:str(run/'contexts'/(r['case_id'].replace('/','__')+'.json')) for r in unique}
    request=dict(source_root=str(d7.ONLINE),context_paths=contexts,unique_references=unique,
        protocol=read(run/'protocol.json'),planned_primary_MPC_solves=len(unique)*30,planned_audit_solves=len(contexts))
    write(run/'mpc_request.json',request)
    files=[run/'mpc_request.json',run/'reference_manifest.json']+[Path(r['world_reference_path']) for r in rows]+[Path(r['plan_recheck_path']) for r in rows]
    write(run/'execution_request_freeze.json',dict(file_sha256={str(p):digest(p) for p in files},utc=utc(),
        policy_frozen_before_GP=True,unique_reference_count=len(unique)))
    write(run/'plan_analysis_completed.json',dict(wall_s=time.perf_counter()-begin,unique_rollouts=len(unique)))


def worker(run,stage):
    verify(run,True);d7.check_hashes(read(run/'execution_request_freeze.json')['file_sha256'])
    subprocess.run([str(d7.PYTHON),str(ROOT/'scripts/lightnav/gp_se2_diag08_mpc_rollout.py'),
        '--run',str(run),'--stage',stage],check=True,cwd=ROOT,env=os.environ.copy())


def plan_metrics(result,full,reserve):
    a=full['additional_grid'];p=full['original_plan_report'];latest=result['candidate_checks'][1]
    return dict(solver_status=result['solver_status'],termination=result['termination'],iterations=result['iterations'],
        objective=latest['objective'],solver_grid_feasible=latest['collocation']['feasible'],
        original_dense_feasible=full['original_dense_feasible'],original_full_feasible=full['full_feasible'],
        selected_full_feasible=result['selected_full_feasible'],selected_source=result['selected_iterate'],
        endpoint_position_error_m=p['goal']['position_error_m'],endpoint_yaw_error_rad=p['goal']['yaw_error_rad'],
        planning_reserve_m=.15-p['goal']['position_error_m'],G4_reserve=reserve,
        extrema={k:v for k,v in a.items() if k.startswith(('maximum_absolute','minimum_linear','maximum_linear'))},
        flags=a['flags'],deformation=p['deformation'],solve_wall_s=result['solve_wall_time_s'],
        selected_objective=result['candidate_objective'],initial_objective=result['candidate_checks'][0]['objective'])


def assemble(run,env=None):
    source=read(run/'source.json');env=env or HospitalEnvironment.load(source['environment_path'])
    config=yaml.safe_load((run/'config_snapshot.yaml').read_text());manifest=read(run/'reference_manifest.json')
    aliases=manifest['aliases'];records=[];cache={}
    for row,record in zip(read(run/'experiment_manifest.json')['starts'],manifest['records']):
        label=row['solve_id'];context=read(run/'contexts'/(row['case_id'].replace('/','__')+'.json'))
        goal=context['goal_route']['goal_world'] if 'goal_world' in context['goal_route'] else None
        old_result=read(Path(row['g3_result_path']));old_full=old_result['candidate_checks'][1]['full_acceptance']
        oldroll=read(D7/'rollouts'/row['historical_uid']/'rollout.json');oldeval=read(D7/'rollouts'/row['historical_uid']/'full_execution_evaluation.json')
        olddiag=read(D7/'rollouts'/row['historical_uid']/'diagnostics.json');goal=oldeval['goal']
        result=read(run/'solves'/label/'solver_result.json');plan=read(run/'plan_recheck'/(label+'.json'))
        reference=np.load(record['world_reference_path'],allow_pickle=False);newroll=neweval=newdiag=None;uid=aliases.get(label)
        if uid is not None:
            if uid not in cache:
                rollout=read(run/'rollouts'/uid/'rollout.json')
                errors=validate_execution_records(rollout,context['context'],reference,plan['admission'])
                if errors:raise ValueError(str(errors))
                evaluation=clean(evaluate_rollout(rollout,context['goal_route'],env,config))
                diagnostics=clean(execution_diagnostics(rollout,evaluation,context['context']))
                cache[uid]=(rollout,evaluation,diagnostics)
            newroll,neweval,newdiag=cache[uid]
        oldgeom=endpoint_geometry(goal,oldroll['reference_world'][-1],oldroll['states'][-1]['pose_world'])
        newgeom=None if newroll is None else endpoint_geometry(goal,reference[-1],newroll['states'][-1]['pose_world'])
        classification=classify(hard=row['case_role']=='HARD',admission=plan['admission'],execution=neweval,
            historical_error=oldeval['execution']['terminal_position_error_m'])
        records.append(dict(reference_id=label,case_id=row['case_id'],method=row['method'],initialization=row['initialization'],
            classification=classification,admission=plan['admission'],unique_reference_id=uid,
            G3=dict(provenance='HISTORICAL_DIAG06_PLAN_DIAG07_EXECUTION_NOT_RERUN',plan=plan_metrics(old_result,old_full,None),
                geometry=oldgeom,evaluation=oldeval,diagnostics=olddiag),
            G4=dict(provenance='NEW_DIAG08',plan=plan_metrics(result,plan['original_full_check'],plan['reserve']),
                geometry=newgeom,evaluation=neweval,diagnostics=newdiag)))
    return records,cache


def evaluate(run):
    verify(run,True);begin=time.perf_counter();records,cache=assemble(run)
    for uid,(_,evaluation,diagnostics) in cache.items():
        write(run/'rollouts'/uid/'full_execution_evaluation.json',evaluation)
        write(run/'rollouts'/uid/'diagnostics.json',diagnostics)
    (run/'aggregate').mkdir(exist_ok=False)
    planrows=[];execrows=[];geomrows=[];commandrows=[];selectionrows=[];pairs=[];timing=[]
    for r in records:
        for grid in ['G3','G4']:
            v=r[grid];e=v['evaluation'];d=v['diagnostics'];p=v['plan']
            base=dict(reference_id=r['reference_id'],case_id=r['case_id'],grid=grid,provenance=v['provenance'])
            planrows.append(dict(base,**p))
            geomrows.append(dict(base,available=v['geometry'] is not None,**(v['geometry'] or {})))
            execrows.append(dict(base,performed=e is not None,primary_success=None if e is None else e['primary_success'],
                failure_reasons=r['admission']['ineligible_reasons'] if e is None else e['failure_reasons'],
                **({} if e is None else e['execution'])))
            if e is not None:
                commandrows.append(dict(base,**{k:v for k,v in e['execution'].items() if 'acceleration' in k or 'variation' in k},
                    first_command=d['first_command'],final_command=d['final_command']))
                selectionrows.append(dict(base,first_goal_region_target_s=d['first_goal_region_in_horizon_s'],
                    first_final_row_s=d['first_final_row_in_horizon_s'],final_row_maintained=d['final_row_maintained_after_first'],
                    selected_indices=d['selection']['selected_indices'],nearest_indices=d['selection']['nearest_indices']))
        pairs.append(dict(reference_id=r['reference_id'],classification=r['classification'],
            G3_plan_error_m=r['G3']['plan']['endpoint_position_error_m'],G4_plan_error_m=r['G4']['plan']['endpoint_position_error_m'],
            G3_execution_error_m=r['G3']['geometry']['execution_error_m'],G4_execution_error_m=None if r['G4']['geometry'] is None else r['G4']['geometry']['execution_error_m'],
            G3_success=r['G3']['evaluation']['primary_success'],G4_success=None if r['G4']['evaluation'] is None else r['G4']['evaluation']['primary_success'],
            G4_plan_valid=r['admission']['plan_valid'],G4_reserve_pass=r['admission']['planning_endpoint_reserve_pass']))
        result=read(run/'solves'/r['reference_id']/'solver_result.json')
        timing.append(dict(reference_id=r['reference_id'],**{k:result[k] for k in ['input_loading_wall_s','provider_construction_wall_s',
            'compilation_warmup_wall_s','solve_wall_time_s','post_solve_validation_time_s','cold_setup_solve_validation_s',
            'objective_evaluations','equality_evaluations','inequality_evaluations','derivative_calls']},
            mpc_rollout_wall_s=None if r['unique_reference_id'] is None else cache[r['unique_reference_id']][0]['rollout_wall_s'],
            mpc_solve_wall_s=None if r['unique_reference_id'] is None else sum(cache[r['unique_reference_id']][0]['official_mpc_solve_wall_s'])))
    for name,rows in [('g3_vs_g4',pairs),('plan_metrics',planrows),('endpoint_geometry',geomrows),('execution_outcomes',execrows),
        ('command_metrics',commandrows),('selection_metrics',selectionrows),('timing',timing)]:table(run/'aggregate'/(name+'.csv'),rows)
    hard=records[:4];outcomes=[r['classification'] for r in hard]
    interpretation=('ENDPOINT_RESERVE_RECOVERS_FIXED_EVENT' if 'MARGIN_EXECUTION_RECOVERY' in outcomes else
        'ENDPOINT_RESERVE_REDUCES_ERROR_ONLY' if 'MARGIN_IMPROVES_GOAL_BUT_NOT_SUCCESS' in outcomes else
        'ENDPOINT_RESERVE_NO_RECOVERY' if any(r['G4']['evaluation'] is not None for r in hard) else 'ENDPOINT_RESERVE_NOT_PLAN_FEASIBLE')
    summary=dict(operational_status='PENDING_ARTIFACT_VALIDATION',interpretation=interpretation,
        planned_GP_solves=5,actual_GP_solves=read(run/'optimization_completed.json')['actual_new_GP_solves'],
        eligible_provenance_references=sum(r['admission']['eligible_for_execution'] for r in records),
        unique_rollouts=len(cache),primary_MPC_solves=sum(len(v[0]['controller_reference_selections']) for v in cache.values()),
        historical_audit_MPC_solves=read(run/'historical_audits/summary.json')['mpc_solves'],
        hard_execution_recovered_starts=outcomes.count('MARGIN_EXECUTION_RECOVERY'),hard_event_count=1,benign_event_count=1,
        benign_outcome=records[-1]['classification'],original_lateral_tolerance_unchanged=True,
        new_VLA=0,new_RGB=0,new_GUI=0,new_online_episode=0,historical_G3_reruns=0,retry=False,
        pairs=pairs,evaluation_wall_s=time.perf_counter()-begin,
        limitations=['four hard starts are one development event','entire optimized reference changes; endpoint is not uniquely isolated',
            'strict 0.11 m admission distinct from original squared-margin solver allowance','offline exact-unicycle execution, not real robot or online',
            'plan lateral invalidity is not changed by execution success','no continuous-time proof'])
    write(run/'aggregate/summary.json',summary);write(run/'comparison_records.json',records)
    print(json.dumps({k:v for k,v in summary.items() if k not in ['pairs','limitations']}),flush=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['prepare','derivatives','freeze','solve','plans','audit','execute','evaluate'])
    p.add_argument('--run',required=True,type=Path);a=p.parse_args();run=a.run.resolve()
    if a.stage in ['audit','execute']:worker(run,a.stage)
    else:globals()[{'plans':'analyze_plans'}.get(a.stage,a.stage)](run)
if __name__=='__main__':main()

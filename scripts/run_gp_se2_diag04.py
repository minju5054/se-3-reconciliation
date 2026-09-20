#!/usr/bin/env python3
"""One frozen ten-start quarter-motion refinement comparison; no MPC runtime."""
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
import tempfile
import zipfile

for name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):
    os.environ[name]='1'
os.environ['JAX_PLATFORMS']='cpu';os.environ['JAX_ENABLE_X64']='true'
os.environ.setdefault('XLA_FLAGS','--xla_cpu_multi_thread_eigen=false intra_op_parallelism_threads=1')
os.environ.setdefault('MPLCONFIGDIR','/tmp/gp_se2_diag04_mpl')
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
import yaml
from run_gp_se2_ref01 import read,write,copy_new,digest,utc,revision,table,array,clean
from reconciliation.gp_se2_diag04_validation import PROTOCOL,PAIR_SPECS,GRIDS,schedule,vector_hash,directions_and_perturbation,verify_point
from reconciliation.gp_se2_diag04_constraints import ConstraintView,RefinedDerivativeProvider,constraint_row_metadata
from reconciliation.gp_se2_diag04_solver import run_refined,equality_rank_diagnostics
from reconciliation.gp_se2_diag02_derivatives import DerivativeProvider,DerivativeError
from reconciliation.gp_se2_diag02_environment import EnvironmentDerivatives
from reconciliation.gp_se2_diag_acceptance import check_full_candidate
from reconciliation.gp_se2_diag03_intervals import audit_intervals
from reconciliation.gp_se2_diag03_audit import compare_tree
from reconciliation.gp_se2_formulation import _constraint_report
from reconciliation.gp_se2_diagnostics import load_frozen_case
from reconciliation.gp_se2_environment import HospitalEnvironment

STARTING_SHA='9f6031bf46f4b885af91433ab023a1552e222599'
PRIMARY=ROOT/'data/robotless_gp_se2_02/primary_20260919T024000Z'
DIAG03=ROOT/'data/robotless_gp_se2_diag_03/audit_20260920T010000Z'
OUTPUT=ROOT/'data/robotless_gp_se2_diag_04'
CODE=['src/reconciliation/gp_se2_diag04_constraints.py','src/reconciliation/gp_se2_diag04_solver.py',
      'src/reconciliation/gp_se2_diag04_validation.py','scripts/run_gp_se2_diag04.py','scripts/plot_gp_se2_diag04.py',
      'tests/test_gp_se2_diag04_constraints.py','tests/test_gp_se2_diag04_solver.py',
      'tests/test_gp_se2_diag04_validation.py','tests/test_gp_se2_diag04_plots.py']


def check_hashes(expected):
    return [p for p,h in expected.items() if not Path(p).is_file() or digest(p)!=h]


def verify_frozen(run):
    source=read(run/'source.json')
    failed=check_hashes(source['preserved_hashes'])+check_hashes(source['user_config_hashes'])
    failed += [p for p,h in source['experiment_code_sha256'].items() if digest(ROOT/p)!=h]
    failed += [p for p,h in source['frozen_input_sha256'].items() if digest(run/p)!=h]
    if failed:raise ValueError('SOURCE_OR_IMPLEMENTATION_MISMATCH: '+str(failed))
    return source


def prepare(run):
    if run.exists() or not run.is_relative_to(OUTPUT):raise FileExistsError('new exclusive DIAG04 run required')
    began=time.perf_counter();previous=read(DIAG03/'source.json');original=read(PRIMARY/'source.json')
    for p in (PRIMARY/'validation.json',DIAG03/'validation.json'):
        if not read(p)['valid']:raise ValueError('authoritative validation fails: '+str(p))
    from reconciliation.gp_se2_02_transfer import verify_diag02_authority
    authority=verify_diag02_authority(Path(original['previous_diag02']))
    expected=dict(previous['source_input_hashes'])
    expected['/home/gpuadmin/Workspace/external/LightNav-0-official-demo/mujoco_demo/vln_mujoco/mpc.py']='2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1'
    expected.update({str(p):digest(p) for p in DIAG03.rglob('*') if p.is_file()})
    expected.update({str(ROOT/p):h for p,h in previous['audit_code_sha256'].items()})
    if check_hashes(expected):raise ValueError('source integrity failed')
    config=yaml.safe_load((PRIMARY/'config_snapshot.yaml').read_text());c=config['formulation']
    for k,v in dict(horizon_s=3.,support_dt_s=.1,max_iterations=200,wall_time_s=30.,ftol=1e-7,equality_tolerance=1e-5,inequality_tolerance=1e-5).items():
        if c[k]!=v:raise ValueError('authoritative configuration differs: '+k)
    run.mkdir(parents=True);copy_new(PRIMARY/'config_snapshot.yaml',run/'config_snapshot.yaml')
    rows=schedule();cases=read(PRIMARY/'case_manifest.json')['selected'];directions,perturbation=directions_and_perturbation();points=[]
    for pair,(cid,role,method,seed) in enumerate(PAIR_SPECS):
        folder=PRIMARY/'cases'/next(r['case_directory'] for r in cases if r['case_id']==cid)
        seed_path=folder/'initializations'/(seed+'.npy');z=np.load(seed_path,allow_pickle=False)
        if z.dtype!=np.float64:raise ValueError('source seed precision differs')
        solver_path=folder/'methods'/method/'starts'/seed/'solver_result.json';sr=read(solver_path)
        if z.tobytes()!=np.asarray(sr['initial_vector'],np.float64).tobytes():raise ValueError('saved seed differs from primary source')
        if read(folder/'goal_route.json')['gates']:raise ValueError('frozen cases unexpectedly contain gate constraints')
        copy_new(seed_path,run/'initializations'/f'pair_{pair}.npy')
        for row in rows:
            if row['pair_index']==pair:row.update(seed_path=str(seed_path),seed_file_sha256=digest(seed_path),seed_vector_sha256=vector_hash(z),
                derived_seed_path=f'initializations/pair_{pair}.npy',case_directory=str(folder),historical_solver_path=str(solver_path))
        for kind,vector in [('seed',z),('historical_final',np.asarray(sr['latest_iterate'],np.float64)),('fixed_seed_perturbation',z+perturbation)]:
            points.append(dict(name=f'pair_{pair}__{kind}',pair_index=pair,case_id=cid,method=method,initialization=seed,
                kind=kind,vector=vector,vector_sha256=vector_hash(vector),used_as_optimization_seed=kind=='seed'))
    write(run/'protocol.json',dict(PROTOCOL,frozen_utc=utc(),execution_order=[r['solve_id'] for r in rows],
        primary_G0_reexecution=True,rank_diagnostics_after_all_solves=True,
        software={p:importlib.metadata.version(p) for p in ('numpy','scipy','jax','jaxlib','shapely','matplotlib')}))
    write(run/'experiment_manifest.json',dict(starts=rows,planned_solves=10,planned_order_frozen=True))
    write(run/'derivative_checks/point_manifest.json',dict(points=points,directions=directions,perturbation=perturbation,
        point_count=len(points),thresholds=PROTOCOL,optimizer_executed=False))
    write(run/'source.json',dict(experiment='GP-SE2-DIAG-04',starting_git_sha=STARTING_SHA,experiment_git_sha=revision(),
        primary_source=str(PRIMARY),previous_audit=str(DIAG03),environment_path=original['environment_path'],
        environment_export_source=original['environment_export_source'],original_config_sha256=original['original_config_sha256'],
        preserved_hashes=expected,user_config_hashes=previous['user_config_hashes'],
        experiment_code_sha256={p:digest(ROOT/p) for p in CODE},core_sha256=original['preserved_core_sha256'],
        frozen_input_sha256={str(p.relative_to(run)):digest(p) for p in run.rglob('*') if p.is_file()},
        derivative_authority=authority,source_preparation_wall_s=time.perf_counter()-began,
        authoritative_validation_hashes={str(p):digest(p) for p in (PRIMARY/'validation.json',DIAG03/'validation.json')},
        source_frame=original['frame'],new_reference_preparation=False,
        citations=['https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html',
          'https://docs.scipy.org/doc/scipy/reference/optimize.minimize-slsqp.html','https://docs.jax.dev/en/latest/_autosummary/jax.jacfwd.html']))
    print(json.dumps(dict(prepared=str(run),planned_solves=10,verification_points=15,preserved_files=len(expected))),flush=True)


def providers(base,environment):
    geometry=EnvironmentDerivatives(environment,radius=.20)
    original=DerivativeProvider(base,geometry)
    v0,v1=[ConstraintView(base,g) for g in GRIDS]
    p0,p1=[RefinedDerivativeProvider(v,original) for v in (v0,v1)]
    return geometry,original,v0,v1,p0,p1


def derivatives(run):
    source=verify_frozen(run);folder=run/'derivative_checks'
    if (folder/'started.json').exists():raise FileExistsError('derivative attempt already started')
    write(folder/'started.json',dict(utc=utc()));began=time.perf_counter();env=HospitalEnvironment.load(source['environment_path'])
    frozen=read(folder/'point_manifest.json');dirs=np.asarray(frozen['directions']);cache={};reports=[]
    for point in frozen['points']:
        key=(point['case_id'],point['method'])
        if key not in cache:
            case=load_frozen_case(PRIMARY,*key,environment=env);cache[key]=(case,providers(case['problem'],env))
        case,(geometry,original,v0,v1,p0,p1)=cache[key]
        try:
            report,arrays=verify_point(point['name'],point['vector'],case['problem'],geometry,original,v0,v1,p0,p1,dirs)
            with (folder/(point['name']+'.npz')).open('xb') as stream:np.savez_compressed(stream,**arrays)
        except DerivativeError as exc:
            report=dict(name=point['name'],valid=False,status='DERIVATIVE_UNSUPPORTED',error=str(exc),reason_code=exc.reason_code)
        write(folder/(point['name']+'.json'),report);reports.append(report)
        print(json.dumps(dict(point=point['name'],valid=report['valid'],status=report['status'])),flush=True)
    result=dict(valid=all(r['valid'] for r in reports),point_count=len(reports),reports=[dict(name=r['name'],valid=r['valid'],status=r['status']) for r in reports],
        wall_s=time.perf_counter()-began,no_optimizer_executed=True,code_hashes=source['experiment_code_sha256'])
    write(folder/'validation.json',result)
    if not result['valid']:raise SystemExit('DERIVATIVE_VALIDATION_FAILED: primary optimization blocked')


def solve(run):
    source=verify_frozen(run)
    if not read(run/'derivative_checks/validation.json')['valid']:raise ValueError('derivative verification not passed')
    if (run/'optimization_started.json').exists():raise FileExistsError('primary optimization already started; no retry')
    write(run/'optimization_started.json',dict(utc=utc(),experiment_git_sha=revision(),schedule=read(run/'experiment_manifest.json')['starts']))
    began=time.perf_counter();t=time.perf_counter();env=HospitalEnvironment.load(source['environment_path']);environment_s=time.perf_counter()-t
    for row in read(run/'experiment_manifest.json')['starts']:
        folder=run/'solves'/row['solve_id'];folder.mkdir(parents=True)
        write(folder/'started.json',dict(row,utc=utc()));t=time.perf_counter()
        case=load_frozen_case(PRIMARY,row['case_id'],row['method'],environment=env)
        z=np.load(run/row['derived_seed_path'],allow_pickle=False);input_s=time.perf_counter()-t
        if vector_hash(z)!=row['seed_vector_sha256']:raise ValueError('frozen seed altered')
        view=ConstraintView(case['problem'],row['grid']);t=time.perf_counter()
        geometry=EnvironmentDerivatives(env,radius=case['config']['footprint']['radius_m'])
        base_provider=DerivativeProvider(case['problem'],geometry);provider=RefinedDerivativeProvider(view,base_provider)
        construction_s=time.perf_counter()-t;t=time.perf_counter();provider.warmup(z);warmup_s=time.perf_counter()-t
        setup_stats=provider.stats();dimensions=view.dimensions(z)
        expected_eq=90 if row['grid']=='G1_QUARTER' else 30
        expected_iq=(903 if row['method']=='M3_GP_CONSTRAINED' else 813)+(480 if row['grid']=='G1_QUARTER' else 0)
        if dimensions['equality_count']!=expected_eq or dimensions['inequality_count']!=expected_iq:raise ValueError('constraint dimensions mismatch')
        write(folder/'constraint_rows.json',constraint_row_metadata(view,z))
        provider.reset_stats();view.reset_stats();view.clear_cache()
        result=run_refined(view,z,derivative_provider=provider,case=case,initialization_name=row['initialization'])
        result.update(row,input_loading_wall_s=input_s,provider_construction_wall_s=construction_s,compilation_warmup_wall_s=warmup_s,
            warmup_provider_stats=setup_stats,constraint_dimensions=dimensions,
            cold_setup_solve_validation_s=input_s+construction_s+warmup_s+result['total_setup_solve_post_wall_time_s'])
        write(folder/'solver_result.json',result)
        print(json.dumps(dict(solve=row['solve_id'],termination=result['termination'],iterations=result['iterations'],
            candidate=result['candidate_found'],selected_full=result['selected_full_feasible'],latest_full=result['latest_full_feasible'],solve_s=result['solve_wall_time_s'])),flush=True)
    actual_count=sum(read(run/'solves'/r['solve_id']/'solver_result.json')['minimize_invocations'] for r in read(run/'experiment_manifest.json')['starts'])
    write(run/'optimization_completed.json',dict(utc=utc(),GP_solves=actual_count,environment_loading_s=environment_s,
        total_optimization_phase_wall_s=time.perf_counter()-began,new_MPC_solves=0,new_rollouts=0))


def sample_record(base,vector,full,objective):
    interval=audit_intervals(base,vector);trace=dict(interval['finest_trace']);trace['local_u']=trace['local_fraction']
    return dict(samples=trace,interval_extrema=interval['interval_extrema'],violation_brackets=interval['violation_brackets'],
        full_feasible=full.get('full_feasible'),objective=objective,vector_sha256=vector_hash(vector),
        physical_flags=full.get('additional_grid',{}).get('flags'),full_report=full)


def analyze(run):
    source=verify_frozen(run)
    if not (run/'optimization_completed.json').exists():raise ValueError('all planned starts must complete before analysis')
    if (run/'plot_input.json').exists():raise FileExistsError('analysis already exists')
    began=time.perf_counter();env=HospitalEnvironment.load(source['environment_path']);solves=[];contexts={};provider_cache={}
    rows=read(run/'experiment_manifest.json')['starts'];rank_setup=0.;rank_times=0.
    for row in rows:
        folder=run/'solves'/row['solve_id'];result=read(folder/'solver_result.json');key=(row['case_id'],row['method'])
        if key not in provider_cache:
            case=load_frozen_case(PRIMARY,*key,environment=env);t=time.perf_counter();ps=providers(case['problem'],env)
            ps[-2].warmup(result['initial_vector']);ps[-1].warmup(result['initial_vector']);rank_setup+=time.perf_counter()-t
            provider_cache[key]=(case,ps)
        case,ps=provider_cache[key];base=case['problem'];checks=result['candidate_checks'];latest=checks[1]
        rank=equality_rank_diagnostics(ps[-2],ps[-1],dict(initial=result['initial_vector'],latest_iterate=result['latest_iterate'],selected=result['candidate_vector']))
        rank_times+=rank['wall_s'];write(folder/'conditioning.json',rank)
        lat=sample_record(base,result['latest_iterate'],latest['full_acceptance'],latest['objective'])
        selected=None
        if result['candidate_vector'] is not None:
            selected=sample_record(base,result['candidate_vector'],result['selected_full_acceptance'],result['candidate_objective'])
            selected['source']=result['selected_iterate']
        write(folder/'motion_analysis.json',dict(latest=lat,selected=selected))
        byhash={c['vector_sha256']:c for c in checks};history=[]
        for snap in result['callback_snapshots']:
            check=byhash.get(vector_hash(snap['vector']));report=snap['collocation']
            history.append(dict(elapsed_s=snap['elapsed_s'],objective=snap['objective'],
                equality_residual=None if report is None else report['maximum_equality_residual'],
                inequality_violation=None if report is None else report['maximum_inequality_violation'],
                full_feasible=None if check is None else check['grid_and_full_feasible'],source=f"callback_{snap['iteration']:04d}",
                feasibility_checked_after_solve=check is not None))
        history.insert(0,dict(elapsed_s=0.,objective=checks[0]['objective'],equality_residual=checks[0]['collocation'].get('maximum_equality_residual'),
            inequality_violation=checks[0]['collocation'].get('maximum_inequality_violation'),full_feasible=checks[0]['grid_and_full_feasible'],source='initial',feasibility_checked_after_solve=True))
        conditioning={}
        for r in rank['records']:
            if not r['available']:continue
            label='latest' if r['source']=='latest_iterate' else r['source']
            conditioning.setdefault(label,{})[r['grid']]=dict(raw_singular_values=r['matrices']['raw']['singular_values'],
                scaled_singular_values=r['matrices']['column_scaled']['singular_values'])
        timing={k:result[k] for k in ('input_loading_wall_s','provider_construction_wall_s','compilation_warmup_wall_s',
            'solve_wall_time_s','post_solve_validation_time_s','cold_setup_solve_validation_s')}
        timing.update(rank_diagnostics_s=rank['wall_s'],prepared_solve_s=result['solve_wall_time_s'],
            compile_warmup_s=result['compilation_warmup_wall_s'],candidate_postcheck_s=result['post_solve_validation_time_s'])
        counts=dict(objective_calls=result['objective_evaluations'],constraint_calls=result['equality_evaluations']+result['inequality_evaluations'],
            derivative_calls=sum(result['derivative_calls'].values()),primal_cache_misses=result['profiling']['evaluator_cache_misses'],
            equality_calls=result['equality_evaluations'],inequality_calls=result['inequality_evaluations'],iterations=result['iterations'])
        solves.append(dict(row,termination=result['termination'],latest=lat,selected=selected,history=history,timing=timing,counts=counts,
            conditioning=conditioning,solver_result=result))
        context=case['context'];world=[x['path'] for x in context['source_files'] if x['path'].endswith('/world.npy')]
        contexts[row['case_role']]=dict(B_world=context['B_world'],old_world=np.load(Path(context['source_root'])/world[0],allow_pickle=False),
            fresh_world=np.load(Path(row['case_directory'])/'F_native.npy',allow_pickle=False),goal_world=case['goal_route']['goal_world'])
    summary=aggregate(run,solves)
    for item in solves:
        item.pop('solver_result')
        for phase in ('latest','selected'):
            if item[phase] is not None:
                for key in ('interval_extrema','violation_brackets','full_report'):item[phase].pop(key,None)
    audit=dict(solves=solves,contexts=contexts,formulation_config=case['config']['formulation'],summary=summary,
        rank_setup_warmup_s=rank_setup,rank_SVD_and_derivative_query_s=rank_times,analysis_wall_s=time.perf_counter()-began)
    write(run/'plot_input.json',audit);write(run/'aggregate/summary.json',summary)
    write(run/'analysis_completed.json',dict(utc=utc(),rank_setup_warmup_s=rank_setup,rank_query_SVD_s=rank_times,wall_s=time.perf_counter()-began))
    from plot_gp_se2_diag04 import generate
    generate(run,audit=clean(audit),environment=env)


def failure_labels(result,grid):
    labels=[]
    if result['selected_full_feasible']:return labels
    if result['termination']=='TIMEOUT':labels.append('TIMEOUT_WITHOUT_VALID_CANDIDATE')
    if result['termination']=='NUMERICAL_FAILURE' or result['solver_status'] in (5,6,7):labels.append('SOLVER_NUMERICAL_FAILURE')
    if result['recorded_derivative_errors']:labels.append('DERIVATIVE_UNSUPPORTED')
    if grid=='G1_QUARTER' and result['candidate_checks'][1]['collocation']['feasible'] and result['latest_full_feasible'] is False:
        labels.append('REFINED_GRID_PASS_FULL_FAIL')
    labels.append('NO_FEASIBLE_CANDIDATE')
    return labels


def aggregate(run,solves,output_folder=None):
    folder=Path(output_folder) if output_folder is not None else run/'aggregate';folder.mkdir(exist_ok=output_folder is not None);outcomes=[];extrema=[];conditioning=[]
    for s in solves:
        r=s['solver_result'];c=r['candidate_checks'][1];flags=c['full_acceptance'].get('additional_grid',{}).get('flags',{})
        outcomes.append(dict(**{k:s[k] for k in ('solve_id','case_role','method','initialization','grid')},
            **{k:r[k] for k in ('termination','solver_success','solver_status','iterations','candidate_found','selected_iterate','returned_initial_unchanged','selected_full_feasible','latest_full_feasible','candidate_objective')},
            latest_solver_grid_feasible=c['collocation']['feasible'],latest_original_dense_feasible=c['constraint_report']['feasible'],
            latest_original_full_feasible=c['original_full_feasible'],latest_objective=c['objective'],
            latest_failure_families=[k for k,v in flags.items() if not v],
            inspected_full_feasible_iterate_exists=r['inspected_full_feasible_iterate_exists']))
        for phase,report in [('latest',c['full_acceptance']),('selected',r['selected_full_acceptance'])]:
            values={} if report is None else report.get('additional_grid',{})
            for key in ('maximum_absolute_lateral_velocity_m_s','minimum_linear_speed_m_s','maximum_linear_speed_m_s',
                'maximum_absolute_angular_speed_rad_s','maximum_absolute_linear_acceleration_m_s2','maximum_absolute_angular_acceleration_rad_s2'):
                outcomes[-1][phase+'_'+key]=values.get(key)
            for key in ('original_goal','original_route','original_workspace','original_obstacle_clearance'):
                outcomes[-1][phase+'_'+key]=values.get('flags',{}).get(key)
        outcomes[-1]['failure_classification']=failure_labels(r,s['grid'])
        for phase in ('latest','selected'):
            if s[phase] is not None:extrema.extend(dict(solve_id=s['solve_id'],phase=phase,**x) for x in s[phase]['interval_extrema'] if x['grid']=='SUPPLEMENTAL_0.001_OFFSET')
        rank=read(run/'solves'/s['solve_id']/'conditioning.json')
        for row in rank['records']:
            if row['available']:
                for scale,matrix in row['matrices'].items():conditioning.append(dict(solve_id=s['solve_id'],source=row['source'],grid=row['grid'],scale=scale,**matrix))
            else:conditioning.append(dict(solve_id=s['solve_id'],**row))
    paired=[]
    for i in range(5):
        a,b=solves[2*i:2*i+2];ar,br=a['solver_result'],b['solver_result']
        recovered=not ar['selected_full_feasible'] and br['selected_full_feasible']
        paired.append(dict(pair_index=i,case_role=a['case_role'],method=a['method'],initialization=a['initialization'],
            G0_full=ar['selected_full_feasible'],G1_full=br['selected_full_feasible'],feasibility_recovered=recovered,
            feasibility_regression=ar['selected_full_feasible'] and not br['selected_full_feasible'],
            G0_objective=ar['candidate_objective'],G1_objective=br['candidate_objective'],
            same_seed_vector=ar['initial_vector_sha256']==br['initial_vector_sha256'],
            G1_failure_classification=failure_labels(br,b['grid']),
            G1_status='FEASIBILITY_RECOVERED' if recovered else ('REFINED_GRID_PASS_FULL_FAIL' if br['candidate_checks'][1]['collocation']['feasible'] and br['latest_full_feasible'] is False else 'NO_FEASIBLE_CANDIDATE' if not br['selected_full_feasible'] else 'BENIGN_FEASIBILITY_PRESERVED' if a['case_role']=='BENIGN_CONTROL' else 'FEASIBILITY_PRESERVED')))
    historical=[]
    for s in solves:
        if s['grid']!=GRIDS[0]:continue
        old=read(s['historical_solver_path']);new=s['solver_result']
        historical.append(dict(solve_id=s['solve_id'],historical_path=s['historical_solver_path'],
            historical_termination=old['termination'],new_termination=new['termination'],historical_iterations=old['iterations'],new_iterations=new['iterations'],
            latest_literal_equal=np.array_equal(old['latest_iterate'],new['latest_iterate']),
            latest_max_absolute_chart_difference=float(np.max(np.abs(np.asarray(old['latest_iterate'])-new['latest_iterate']))),
            historical_objective=old['candidate_checks'][1]['objective'],new_objective=new['candidate_checks'][1]['objective'],
            preserved_core_and_seeds=True,new_harness='same SLSQP options and retention; module-global inner profiling hooks disabled in BOTH grids'))
    table(folder/'historical_G0_context.csv',historical)
    table(folder/'all_starts.csv',outcomes);table(folder/'paired_feasibility.csv',paired)
    table(folder/'constraint_dimensions.csv',[dict(solve_id=s['solve_id'],**s['solver_result']['constraint_dimensions']) for s in solves])
    table(folder/'motion_extrema.csv',extrema);table(folder/'conditioning.csv',conditioning)
    table(folder/'timing.csv',[dict(solve_id=s['solve_id'],**s['timing'],**s['counts']) for s in solves])
    recoveries=sum(p['feasibility_recovered'] for p in paired[:4]);regression=paired[-1]['feasibility_regression']
    return dict(operational_status='GP_SE2_DIAG_04_COMPLETED_WITH_LIMITATIONS',planned_GP_solves=10,actual_GP_solves=sum(s['solver_result']['minimize_invocations'] for s in solves),
        hard_start_pair_recoveries=recoveries,hard_event_count=1,benign_event_count=1,
        benign_feasibility_preserved=paired[-1]['G1_full'],benign_regression=regression,
        numerical_outcome='MIXED_FEASIBILITY_RESULT' if regression and recoveries else 'FEASIBILITY_RECOVERY_OBSERVED' if recoveries else 'NO_FEASIBILITY_RECOVERY',
        new_VLA_inference=0,new_MPC_solve=0,new_rollout=0,new_GUI_runtime=0,
        physical_acceptance_changed=False,continuous_time_proof=False,navigation_improvement_claimed=False,
        outcomes=outcomes,paired=paired)


def validate(run):
    if (run/'validation.json').exists():raise FileExistsError('validation exists')
    began=time.perf_counter();source=verify_frozen(run);env=HospitalEnvironment.load(source['environment_path']);errors=[];checks=0
    reconstructed=[];rank_providers={}
    manifest=read(run/'experiment_manifest.json')['starts'];audit=read(run/'plot_input.json')
    if len(manifest)!=10 or [r['solve_id'] for r in manifest]!=[r['solve_id'] for r in schedule()]:errors.append('planned coverage/order')
    for row in manifest:
        result=read(run/'solves'/row['solve_id']/'solver_result.json');case=load_frozen_case(PRIMARY,row['case_id'],row['method'],environment=env)
        view=ConstraintView(case['problem'],row['grid']);seed=np.load(run/row['derived_seed_path'],allow_pickle=False)
        if result['minimize_invocations']!=1:errors.append(row['solve_id']+' primary invocation count')
        if result['config']!=case['problem'].config:errors.append(row['solve_id']+' configuration')
        if view.dimensions(seed)!=result['constraint_dimensions']:errors.append(row['solve_id']+' constraint dimensions')
        metadata=constraint_row_metadata(view,seed)
        if clean(metadata)!=read(run/'solves'/row['solve_id']/'constraint_rows.json'):errors.append(row['solve_id']+' row metadata')
        if vector_hash(result['initial_vector'])!=vector_hash(seed):errors.append(row['solve_id']+' seed')
        cache={};choices=[]
        for record in result['candidate_checks']:
            z=np.asarray(record['vector']);key=vector_hash(z)
            if key not in cache:
                e=view.evaluate(z);full=check_full_candidate(case['problem'],z,case)
                cache[key]=dict(objective=e['objective'],collocation=_constraint_report(e['equality'],e['inequality'],case['problem'].config),
                    constraint_report=full['original_dense_report'],full_acceptance=full)
            calculated=cache[key]
            for field in calculated:
                compared=compare_tree(calculated[field],record[field],row['solve_id']+'.'+record['iterate']+'.'+field);checks+=len(compared)
                errors.extend(r['field'] for r in compared if not r['numerical_agreement'])
            if calculated['collocation']['feasible'] and calculated['constraint_report']['feasible']:choices.append((calculated['objective'],record['iterate'],key))
        selected=min(choices) if choices else None
        if (None if selected is None else selected[1])!=result['selected_iterate']:errors.append(row['solve_id']+' selection')
        expected_labels=['initial','latest_iterate']+[f"callback_{s['iteration']:04d}" for s in result['callback_snapshots'] if s['collocation'] is not None and s['collocation']['feasible']]
        if expected_labels!=[c['iterate'] for c in result['candidate_checks']]:errors.append(row['solve_id']+' retention')
        for snap in result['callback_snapshots']:
            if snap['evaluation_complete']:
                e=view.evaluate(snap['vector']);checks+=1
                if not np.isclose(e['objective'],snap['objective'],atol=1e-8,rtol=1e-10):errors.append('callback objective')
                for k in ('equality','inequality'):
                    field='equality_residuals' if k=='equality' else 'inequality_margins'
                    if not np.allclose(e[k],snap[field],atol=1e-8,rtol=1e-10):errors.append('callback '+k)
        plotted=next(s for s in audit['solves'] if s['solve_id']==row['solve_id'])
        motion=read(run/'solves'/row['solve_id']/'motion_analysis.json')
        reconstructed.append(dict(plotted,solver_result=result,**motion))
        key=(row['case_id'],row['method'])
        if key not in rank_providers:
            rank_providers[key]=providers(case['problem'],env)
            rank_providers[key][-2].warmup(seed);rank_providers[key][-1].warmup(seed)
        rank=equality_rank_diagnostics(*rank_providers[key][-2:],dict(initial=result['initial_vector'],latest_iterate=result['latest_iterate'],selected=result['candidate_vector']))
        saved_rank=read(run/'solves'/row['solve_id']/'conditioning.json')
        if len(rank['records'])!=len(saved_rank['records']):errors.append('rank coverage')
        for a,b in zip(rank['records'],saved_rank['records']):
            compared=compare_tree(clean({k:v for k,v in a.items() if k!='diagnostic_wall_s'}),{k:v for k,v in b.items() if k!='diagnostic_wall_s'})
            checks+=len(compared);errors.extend(row['solve_id']+' rank '+r['field'] for r in compared if not r['numerical_agreement'])
        for phase,z in [('latest',result['latest_iterate']),('selected',result['candidate_vector'])]:
            if z is None:
                if plotted[phase] is not None:errors.append('fabricated selected')
                continue
            interval=audit_intervals(case['problem'],z);trace=dict(interval['finest_trace']);trace['local_u']=trace['local_fraction']
            compared=compare_tree(clean(trace),plotted[phase]['samples']);checks+=len(compared)
            if any(not r['numerical_agreement'] for r in compared):errors.append(row['solve_id']+' plot trace '+phase)
            for field in ('interval_extrema','violation_brackets'):
                compared=compare_tree(clean(interval[field]),motion[phase][field]);checks+=len(compared)
                errors.extend(row['solve_id']+' '+field+' '+r['field'] for r in compared if not r['numerical_agreement'])
    with tempfile.TemporaryDirectory(prefix='diag04_saved_validator_') as tmp:
        summary=aggregate(run,reconstructed,output_folder=tmp)
        compared=compare_tree(clean(summary),read(run/'aggregate/summary.json'));checks+=len(compared)
        errors.extend('summary '+r['field'] for r in compared if not r['numerical_agreement'])
        for calculated in Path(tmp).glob('*.csv'):
            checks+=1
            if calculated.read_bytes()!=(run/'aggregate'/calculated.name).read_bytes():errors.append('aggregate table '+calculated.name)
    if not read(run/'derivative_checks/validation.json')['valid']:errors.append('derivative gate')
    from plot_gp_se2_diag04 import expected_numeric
    for name,numeric in expected_numeric(audit).items():
        side=read(run/'plots'/(name+'.json'));checks+=1
        if side['numeric_data']!=numeric:errors.append('plot numeric '+name)
        if side['image_sha256']!=digest(run/'plots'/(name+'.png')):errors.append('plot PNG '+name)
        errors.extend('plot source '+p for p in check_hashes(side['source_hashes']))
    with zipfile.ZipFile(run/'review_bundle.zip') as archive:
        for name in archive.namelist():
            checks+=1
            if archive.read(name)!=(run/'review_bundle'/name).read_bytes():errors.append('ZIP '+name)
    write(run/'validation.json',dict(valid=not errors,authoritative=True,errors=errors,check_count=checks,
        wall_s=time.perf_counter()-began,no_new_optimization=True,no_MPC_or_rollout=True,
        scientific_failure_is_validation_failure=False))
    print(json.dumps(read(run/'validation.json')),flush=True)
    if errors:raise SystemExit(1)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=['prepare','derivatives','solve','analyze','validate']);parser.add_argument('--run',type=Path,required=True)
    args=parser.parse_args();globals()[args.action](args.run.resolve())

#!/usr/bin/env python3
"""One frozen G2-extremum union, five G3 solves, unchanged full acceptance."""
from __future__ import annotations
import argparse
import copy
import importlib.metadata
import os
from pathlib import Path
import sys
import time
import tempfile
import zipfile
for name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):os.environ[name]='1'
os.environ['JAX_PLATFORMS']='cpu';os.environ['JAX_ENABLE_X64']='true'
os.environ.setdefault('XLA_FLAGS','--xla_cpu_multi_thread_eigen=false intra_op_parallelism_threads=1')
os.environ.setdefault('MPLCONFIGDIR','/tmp/gp_se2_diag06_mpl')
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
import yaml
import run_gp_se2_diag05 as d5
from run_gp_se2_ref01 import read,write,copy_new,digest,utc,revision,table,clean
from reconciliation.gp_se2_diag06_witnesses import finest_trace,canonical_grid,extract_runs,witness_union,validate_schedule
from reconciliation.gp_se2_diag06_constraints import WitnessView,WitnessDerivatives,G3
from reconciliation.gp_se2_diag06_validation import PROTOCOL,schedule,verify_point,equality_diagnostics,witness_diagnostics,classify
from reconciliation.gp_se2_diag04_constraints import constraint_row_metadata
from reconciliation.gp_se2_diag05_validation import vector_hash
from reconciliation.gp_se2_diag04_solver import run_refined
from reconciliation.gp_se2_diag02_derivatives import DerivativeError
from reconciliation.gp_se2_diag_acceptance import check_full_candidate
from reconciliation.gp_se2_diag03_intervals import audit_intervals
from reconciliation.gp_se2_diag03_audit import compare_tree
from reconciliation.gp_se2_formulation import _constraint_report
from reconciliation.gp_se2_diagnostics import load_frozen_case
from reconciliation.gp_se2_environment import HospitalEnvironment
D5=ROOT/'data/robotless_gp_se2_diag_05/primary_20260920T064500Z'
PRIMARY=d5.PRIMARY;OUTPUT=ROOT/'data/robotless_gp_se2_diag_06'
STARTING_SHA='f86adffc4d54ae09a3581f557a9fc3eb77c07ed4'
CODE=['src/reconciliation/gp_se2_diag06_witnesses.py','src/reconciliation/gp_se2_diag06_constraints.py',
      'src/reconciliation/gp_se2_diag06_validation.py','scripts/run_gp_se2_diag06.py',
      'scripts/plot_gp_se2_diag06.py','tests/test_gp_se2_diag06.py']


def verify_frozen(run):
    s=read(run/'source.json');failed=d5.d4.check_hashes(s['preserved_hashes'])+d5.d4.check_hashes(s['user_config_hashes'])
    failed += [p for p,h in s['experiment_code_sha256'].items() if digest(ROOT/p)!=h]
    failed += [p for p,h in s['frozen_input_sha256'].items() if digest(run/p)!=h]
    if failed:raise ValueError('SOURCE_OR_IMPLEMENTATION_MISMATCH: '+str(failed))
    return s


def select_witnesses(env,rows,oldplot):
    records=[];sources=[];grid=None
    for row in rows[:4]:
        saved=read(D5/'solves'/row['solve_id']/'solver_result.json')
        z=saved['latest_iterate'];case=load_frozen_case(PRIMARY,row['case_id'],row['method'],environment=env)
        trace=finest_trace(case['problem'],z);actual=canonical_grid(trace)
        item=next(s for s in oldplot['solves'] if s['solve_id']==row['solve_id'])
        historical=item['latest']['samples']
        if actual!=canonical_grid(historical):raise ValueError('canonical DIAG05 sample identity mismatch')
        for field in ('body_twists','body_accelerations','poses_world'):
            if not np.array_equal(trace[field],historical[field]):raise ValueError('DIAG05 saved trace mismatch: '+field)
        if grid is None:grid=actual
        elif grid!=actual:raise ValueError('hard starts have different canonical grids')
        records.extend(extract_runs(trace,case['problem'].config,row['solve_id']))
        sources.append(dict(solve_id=row['solve_id'],solver_path=str(D5/'solves'/row['solve_id']/'solver_result.json'),
            solver_sha256=digest(D5/'solves'/row['solve_id']/'solver_result.json'),vector_sha256=vector_hash(z),
            source_field='latest_iterate',optimization_seed=False))
    union=witness_union(records);validate_schedule(union,grid)
    return dict(grid=grid,records=records,sources=sources,union=union)


def prepare(run):
    if run.exists() or not run.is_relative_to(OUTPUT):raise FileExistsError('new exclusive DIAG06 output required')
    t=time.perf_counter();old=d5.verify_frozen(D5)
    authority=[D5/'validation.json',d5.D4/'validation.json',PRIMARY/'validation.json']
    if not all(read(p)['valid'] for p in authority):raise ValueError('authoritative validation fails')
    expected=dict(old['preserved_hashes']);expected.update({str(p):digest(p) for p in D5.rglob('*') if p.is_file()})
    expected.update({str(ROOT/p):h for p,h in old['experiment_code_sha256'].items()})
    env=HospitalEnvironment.load(old['environment_path']);previous=read(D5/'experiment_manifest.json')['starts']
    oldplot=read(D5/'plot_input.json');selection=select_witnesses(env,previous,oldplot)
    run.mkdir(parents=True);copy_new(D5/'config_snapshot.yaml',run/'config_snapshot.yaml')
    starts=schedule();historical=read(D5/'historical_baselines/diag04_g0_g1_manifest.json')['starts']
    for row,h in zip(starts,previous):
        path=D5/'solves'/h['solve_id']/'solver_result.json';r=read(path)
        seed=D5/h['derived_seed_path'];z=np.load(seed,allow_pickle=False)
        if digest(seed)!=h['seed_sha256'] or digest(seed)!=digest(h['original_seed_path']) or vector_hash(z)!=vector_hash(r['initial_vector']):raise ValueError('original seed mismatch')
        target=f'initializations/pair_{row["pair_index"]}.npy';copy_new(seed,run/target)
        row.update(derived_seed_path=target,seed_sha256=digest(seed),seed_vector_sha256=vector_hash(z),
            original_seed_path=h['original_seed_path'],diag05_seed_path=str(seed),case_directory=h['case_directory'],
            common_witness_path='witness_selection/frozen_union.json')
        historical.append(dict(h,solver_path=str(path),solver_sha256=digest(path),provenance='HISTORICAL_DIAG05_NOT_RERUN'))
    write(run/'experiment_manifest.json',dict(starts=starts,planned_new_solves=5,historical_start_count=15))
    write(run/'historical_baselines/manifest.json',dict(starts=historical,rerun=False))
    for name,value in [('source_vectors',selection['sources']),('canonical_grid',selection['grid']),
        ('per_source_violations',selection['records']),('frozen_union',selection['union'])]:write(run/'witness_selection'/(name+'.json'),value)
    write(run/'witness_selection/validation.json',dict(valid=True,source_count=4,run_count=len(selection['records']),
        witness_count=len(selection['union']),matches_saved_DIAG05_trace_literally=True,
        exact_continuous_extrema=False,lateral_witnesses=0,common_union_for_all_starts=True))
    points=read(D5/'derivative_checks/point_manifest.json')
    for point in points['points']:
        if point['kind']=='historical_final':
            h=previous[point['pair_index']];r=read(D5/'solves'/h['solve_id']/'solver_result.json')
            point.update(vector=r['latest_iterate'],vector_sha256=vector_hash(r['latest_iterate']),kind='saved_G2_latest',
                name=f"pair_{point['pair_index']}__G2_latest",used_as_optimization_seed=False)
    # The field naming is source-checked instead of silently keeping GP02 finals.
    if sum(p['kind']=='saved_G2_latest' for p in points['points'])!=5:raise ValueError('five G2 latest derivative probes required')
    write(run/'derivative_checks/point_manifest.json',points)
    write(run/'protocol.json',dict(PROTOCOL,frozen_utc=utc(),execution_order=[r['solve_id'] for r in starts],
        witness_count=len(selection['union']),witness_union_sha256=digest(run/'witness_selection/frozen_union.json'),
        software={p:importlib.metadata.version(p) for p in ('numpy','scipy','jax','jaxlib','shapely','matplotlib')}))
    write(run/'source.json',dict(experiment='GP-SE2-DIAG-06',starting_git_sha=STARTING_SHA,preparation_git_sha=revision(),
        historical_diag05_execution_sha=old['experiment_git_sha'],primary_source=str(PRIMARY),historical_baseline=str(D5),
        environment_path=old['environment_path'],environment_export_source=old['environment_export_source'],
        original_config_sha256=old['original_config_sha256'],source_frame=old['source_frame'],preserved_hashes=expected,
        user_config_hashes=old['user_config_hashes'],authoritative_validation_hashes={str(p):digest(p) for p in authority},
        experiment_code_sha256={p:digest(ROOT/p) for p in CODE},
        frozen_input_sha256={str(p.relative_to(run)):digest(p) for p in run.rglob('*') if p.is_file()},
        preparation_wall_s=time.perf_counter()-t,no_historical_arrays_copied=True))
    print(dict(prepared=str(run),witness_count=len(selection['union']),preserved_files=len(expected)),flush=True)


def providers(base,env,rows):
    geometry,original,g2,p2=d5.providers(base,env);view=WitnessView(base,rows);p3=WitnessDerivatives(view,p2)
    return geometry,original,g2,p2,view,p3


def derivatives(run):
    s=verify_frozen(run);folder=run/'derivative_checks'
    if (folder/'started.json').exists():raise FileExistsError('derivative gate already attempted')
    write(folder/'started.json',dict(utc=utc()));start=time.perf_counter();env=HospitalEnvironment.load(s['environment_path'])
    frozen=read(folder/'point_manifest.json');rows=read(run/'witness_selection/frozen_union.json');cache={};reports=[]
    for point in frozen['points']:
        key=(point['case_id'],point['method'])
        if key not in cache:
            case=load_frozen_case(PRIMARY,*key,environment=env);cache[key]=(case,providers(case['problem'],env,rows))
        case,(geometry,original,g2,p2,view,p3)=cache[key]
        try:
            report,arrays=verify_point(point['name'],point['vector'],case['problem'],geometry,original,g2,p2,view,p3,np.asarray(frozen['directions']))
            with (folder/(point['name']+'.npz')).open('xb') as f:np.savez_compressed(f,**arrays)
        except (DerivativeError,ValueError,FloatingPointError) as exc:
            report=dict(name=point['name'],valid=False,status='DERIVATIVE_VALIDATION_FAILED',error=str(exc))
        write(folder/(point['name']+'.json'),report);reports.append(report);print(dict(point=point['name'],valid=report['valid']),flush=True)
    valid=len(reports)==15 and all(r['valid'] for r in reports)
    write(folder/'validation.json',dict(valid=valid,point_count=len(reports),wall_s=time.perf_counter()-start,
        status='VERIFIED_WITH_DECLARED_BRANCH_LIMITATIONS' if valid else 'DERIVATIVE_VALIDATION_FAILED',new_optimizer_calls=0))
    if not valid:raise SystemExit('DERIVATIVE_VALIDATION_FAILED: no primary solves')


def freeze(run):
    verify_frozen(run)
    if not read(run/'derivative_checks/validation.json')['valid']:raise ValueError('derivative gate failed')
    if (run/'execution_freeze.json').exists():raise FileExistsError('freeze exists')
    import subprocess
    if subprocess.check_output(['git','status','--porcelain','--',*CODE],cwd=ROOT,text=True):raise ValueError('code must be committed before execution')
    write(run/'execution_freeze.json',dict(execution_git_sha=revision(),utc=utc(),source_sha256=digest(run/'source.json'),
        derivative_gate_sha256=digest(run/'derivative_checks/validation.json'),witness_union_sha256=digest(run/'witness_selection/frozen_union.json')))


def solve(run):
    s=verify_frozen(run);frozen=read(run/'execution_freeze.json')
    if not read(run/'derivative_checks/validation.json')['valid'] or frozen['execution_git_sha']!=revision():raise ValueError('verified frozen commit required')
    if (run/'optimization_started.json').exists():raise FileExistsError('no retry')
    rows=read(run/'experiment_manifest.json')['starts'];witnesses=read(run/'witness_selection/frozen_union.json')
    write(run/'optimization_started.json',dict(utc=utc(),schedule=rows,execution_git_sha=revision()))
    start=time.perf_counter();t=time.perf_counter();env=HospitalEnvironment.load(s['environment_path']);environment_s=time.perf_counter()-t;actual=0
    for row in rows:
        folder=run/'solves'/row['solve_id'];write(folder/'started.json',dict(row,utc=utc()));t=time.perf_counter()
        case=load_frozen_case(PRIMARY,row['case_id'],row['method'],environment=env);z=np.load(run/row['derived_seed_path'],allow_pickle=False)
        if digest(run/row['derived_seed_path'])!=row['seed_sha256'] or vector_hash(z)!=row['seed_vector_sha256']:raise ValueError('seed mismatch')
        loading=time.perf_counter()-t;t=time.perf_counter();_,original,g2,p2,view,provider=providers(case['problem'],env,witnesses);construction=time.perf_counter()-t
        t=time.perf_counter();provider.warmup(z);warmup=time.perf_counter()-t;setup=provider.stats();dims=view.dimensions(z)
        expected=(1383 if row['method']=='M3_GP_CONSTRAINED' else 1293)+len(witnesses)
        if (dims['variable_count'],dims['equality_count'],dims['inequality_count'])!=(150,30,expected):raise ValueError('G3 dimensions mismatch')
        if not np.array_equal(provider.equality_jacobian(z),original.equality_jacobian(z)):raise ValueError('equality mismatch')
        write(folder/'constraint_rows.json',constraint_row_metadata(view,z));provider.reset_stats();view.clear_cache()
        result=run_refined(view,z,derivative_provider=provider,case=case,initialization_name=row['initialization'])
        result.update(row,provenance='NEW_DIAG06_G3',input_loading_wall_s=loading,provider_construction_wall_s=construction,
            compilation_warmup_wall_s=warmup,warmup_stats=setup,constraint_dimensions=dims,
            common_witness_sha256=digest(run/'witness_selection/frozen_union.json'),
            cold_setup_solve_validation_s=loading+construction+warmup+result['total_setup_solve_post_wall_time_s'])
        write(folder/'solver_result.json',result);actual+=result['minimize_invocations']
        print(dict(solve=row['solve_id'],status=result['solver_status'],iterations=result['iterations'],full=result['selected_full_feasible']),flush=True)
    write(run/'optimization_completed.json',dict(utc=utc(),actual_new_GP_solves=actual,planned_new_GP_solves=5,
        wall_s=time.perf_counter()-start,environment_loading_s=environment_s,new_MPC_solves=0,new_rollouts=0,new_GUI=0,new_VLA=0))


def analyze(run):
    s=verify_frozen(run)
    if not (run/'optimization_completed.json').exists():raise ValueError('five starts required')
    if (run/'analysis_started.json').exists():raise FileExistsError('no analysis overwrite')
    write(run/'analysis_started.json',dict(utc=utc()));start=time.perf_counter();env=HospitalEnvironment.load(s['environment_path']);cache={}
    witnesses=read(run/'witness_selection/frozen_union.json')
    for row in read(run/'experiment_manifest.json')['starts']:
        folder=run/'solves'/row['solve_id'];r=read(folder/'solver_result.json');t=time.perf_counter();key=(row['case_id'],row['method'])
        if key not in cache:
            case=load_frozen_case(PRIMARY,*key,environment=env);ps=providers(case['problem'],env,witnesses);ps[-1].warmup(r['initial_vector']);cache[key]=(case,ps)
        case,(_,original,g2,p2,view,provider)=cache[key]
        vectors=dict(initial=r['initial_vector'],latest_iterate=r['latest_iterate'],selected=r['candidate_vector'])
        rank=equality_diagnostics(original,provider,vectors);unique={};samples={};margins={};remaining={}
        for check in r['candidate_checks']:
            z=check['vector'];identity=vector_hash(z)
            if identity not in unique:
                iv=audit_intervals(case['problem'],z);unique[identity]=iv
                write(folder/'intervals'/(identity+'.json'),d5.compact_intervals(iv))
            for phase,v in vectors.items():
                if v is not None and vector_hash(v)==identity and phase not in samples:
                    iv=unique[identity];samples[phase]=d5.sample_result(iv,check['full_acceptance'],check['objective'],z,phase)
                    margins[phase]=witness_diagnostics(view,z,iv['finest_trace'])
                    remaining[phase]=extract_runs(iv['finest_trace'],view.config,row['solve_id'])
                    for failure in remaining[phase]:
                        failure['overlaps_old_observed_run']=any(
                            failure['family']==old['family'] and failure['first_sample_index']<=old['last_sample_index']
                            and failure['last_sample_index']>=old['first_sample_index']
                            for w in witnesses for old in w['source_violations'])
        old_row=read(D5/'experiment_manifest.json')['starts'][row['pair_index']]
        old_result=read(D5/'solves'/old_row['solve_id']/'solver_result.json')
        old_z=old_result['latest_iterate']
        historical_witness_margins=witness_diagnostics(view,old_z,finest_trace(case['problem'],old_z))
        classification=classify(r,remaining['latest_iterate'],margins['latest_iterate'],benign=row['case_role']=='BENIGN_CONTROL')
        write(folder/'analysis.json',dict(rank=rank,initial=samples['initial'],latest=samples['latest_iterate'],selected=samples.get('selected'),
            witness_margins=margins,historical_G2_witness_margins=historical_witness_margins,remaining_motion_runs=remaining,classification=classification,
            candidate_interval_checks=[dict(source=c['iterate'],vector_sha256=c['vector_sha256']) for c in r['candidate_checks']],
            unique_vectors=len(unique),wall_s=time.perf_counter()-t))
    data,tables=assemble(run)
    for name,rows in tables.items():table(run/'aggregate'/(name+'.csv'),rows)
    write(run/'aggregate/summary.json',data['summary']);write(run/'plot_input.json',data)
    write(run/'analysis_completed.json',dict(utc=utc(),wall_s=time.perf_counter()-start))
    from plot_gp_se2_diag06 import generate
    t=time.perf_counter();generate(run);write(run/'rendering_completed.json',dict(utc=utc(),wall_s=time.perf_counter()-t))


def assemble(run):
    old=read(D5/'plot_input.json');new=[];outcomes=[];extrema=[];witnesses=[];remaining=[];timing=[];paired=[];ranks=[]
    frozen=read(run/'witness_selection/frozen_union.json')
    for row in read(run/'experiment_manifest.json')['starts']:
        group=[copy.deepcopy(s) for s in old['solves'] if s['pair_index']==row['pair_index']]
        for s in group:
            if s['grid'].startswith('G2'):s['provenance']='HISTORICAL_DIAG05_NOT_RERUN'
            s['outcome']['provenance']=s['provenance']
            new.append(s);outcomes.append(s['outcome'])
        folder=run/'solves'/row['solve_id'];r=read(folder/'solver_result.json');a=read(folder/'analysis.json')
        o=d5.result_row({k:row[k] for k in ('pair_index','solve_id','case_role','method','initialization','grid')},r)
        o.update(provenance='NEW_DIAG06_G3',classification=a['classification'])
        byhash={c['vector_sha256']:c for c in r['candidate_checks']};history=[]
        for snap in [dict(elapsed_s=0.,objective=r['candidate_checks'][0]['objective'],vector=r['initial_vector'])]+r['callback_snapshots']:
            checked=byhash.get(vector_hash(snap['vector']));history.append(dict(elapsed_s=snap['elapsed_s'],objective=snap['objective'],
                full_feasible=None if checked is None else checked['grid_and_full_feasible'],certification='post-solve'))
        item=dict(row,provenance='NEW_DIAG06_G3',outcome=o,latest=a['latest'],selected=a['selected'],history=history,
            witness_margins=a['witness_margins'],historical_G2_witness_margins=a['historical_G2_witness_margins'],remaining_motion_runs=a['remaining_motion_runs'])
        new.append(item);outcomes.append(o)
        for phase,v in [('initial',r['initial_vector']),('latest',r['latest_iterate']),('selected',r['candidate_vector'])]:
            if v is None:continue
            iv=read(folder/'intervals'/(vector_hash(v)+'.json'))
            extrema.extend(dict(solve_id=row['solve_id'],phase=phase,**e) for e in iv['interval_extrema'] if e['grid']=='SUPPLEMENTAL_0.001_OFFSET')
        for phase,values in a['witness_margins'].items():
            witnesses.extend(dict(solve_id=row['solve_id'],phase=phase,paired_G2_margin=a['historical_G2_witness_margins'][w['witness_row_index']]['new_margin'],**w) for w in values)
        for phase,values in a['remaining_motion_runs'].items():
            remaining.extend(dict(solve_id=row['solve_id'],phase=phase,**v) for v in values)
        for rr in a['rank']['records']:
            if rr['available']:
                ranks.extend(dict(solve_id=row['solve_id'],phase=rr['source'],grid=rr['grid'],scale=k,**v) for k,v in rr['matrices'].items())
        timing.append(dict(solve_id=row['solve_id'],provenance='NEW_DIAG06_G3',**{k:r[k] for k in (
            'input_loading_wall_s','provider_construction_wall_s','compilation_warmup_wall_s','solve_wall_time_s',
            'post_solve_validation_time_s','cold_setup_solve_validation_s','objective_evaluations','equality_evaluations',
            'inequality_evaluations','derivative_calls')},analysis_wall_s=a['wall_s'],rank_wall_s=a['rank']['wall_s'],
            primal_cache_misses=r['profiling']['evaluator_cache_misses']))
        g2=group[-1]['outcome'];paired.append(dict(pair_index=row['pair_index'],case_role=row['case_role'],method=row['method'],
            initialization=row['initialization'],G2_status=g2['solver_status'],G3_status=o['solver_status'],
            G2_full=g2['selected_full_feasible'],G3_full=o['selected_full_feasible'],
            G2_min_vx=g2['minimum_linear_speed_m_s'],G3_min_vx=o['minimum_linear_speed_m_s'],
            G2_max_abs_ax=g2['maximum_absolute_linear_acceleration_m_s2'],G3_max_abs_ax=o['maximum_absolute_linear_acceleration_m_s2'],
            G2_max_abs_vy=g2['maximum_absolute_lateral_velocity_m_s'],G3_max_abs_vy=o['maximum_absolute_lateral_velocity_m_s'],**a['classification']))
    hard=paired[:4];recovered=sum(p['G3_full'] for p in hard)
    if recovered:interpretation='FULL_FEASIBILITY_RECOVERY_OBSERVED'
    elif all(p['outcome']=='INEQUALITY_GAP_CLOSED_LATERAL_ONLY_REMAINS' for p in hard):interpretation='NONLATERAL_GAP_CLOSED_LATERAL_REMAINS'
    elif any(p['outcome']=='VIOLATION_RELOCATED' for p in hard):interpretation='VIOLATION_RELOCATES'
    else:interpretation='NUMERICAL_OR_FEASIBILITY_FAILURE'
    summary=dict(operational_status='GP_SE2_DIAG_06_COMPLETED_WITH_LIMITATIONS',interpretation=interpretation,
        planned_new_GP_solves=5,actual_new_GP_solves=read(run/'optimization_completed.json')['actual_new_GP_solves'],
        historical_starts=15,historical_reruns=0,witness_count=len(frozen),hard_full_valid_starts=recovered,
        hard_event_count=1,benign_event_count=1,benign_outcome=paired[-1]['outcome'],pairs=paired,
        new_VLA=0,new_MPC=0,new_rollout=0,new_GUI=0,physical_acceptance_unchanged=True,
        continuous_feasibility_proof=False,navigation_claim=False,second_refinement_rounds=0)
    data=dict(solves=new,contexts=old['contexts'],formulation_config=old['formulation_config'],frozen_witnesses=frozen,summary=summary)
    tables=dict(all_methods=outcomes,paired_g2_g3=paired,witness_before_after=witnesses,motion_extrema=extrema,
        remaining_violations=remaining,equality_rank=ranks,timing=timing)
    return data,tables


def validate(run):
    if (run/'validation.json').exists():raise FileExistsError('validation exists')
    start=time.perf_counter();s=verify_frozen(run);env=HospitalEnvironment.load(s['environment_path']);errors=[];count=0;cache={}
    def compare(a,b,label):
        nonlocal count
        records=compare_tree(clean(a),b,label);count+=len(records);errors.extend(r['field'] for r in records if not r['numerical_agreement'])
    # Re-extract from historical source records, never from new G3 outcomes.
    selection=select_witnesses(env,read(D5/'experiment_manifest.json')['starts'],read(D5/'plot_input.json'))
    for name,key in [('source_vectors','sources'),('canonical_grid','grid'),('per_source_violations','records'),('frozen_union','union')]:
        compare(selection[key],read(run/'witness_selection'/(name+'.json')),'frozen selection '+name)
    witness_rows=read(run/'witness_selection/frozen_union.json');manifest=read(run/'experiment_manifest.json')['starts']
    if [r['solve_id'] for r in manifest]!=[r['solve_id'] for r in schedule()]:errors.append('schedule coverage')
    invokes=0
    for row in manifest:
        folder=run/'solves'/row['solve_id'];r=read(folder/'solver_result.json');analysis=read(folder/'analysis.json');key=(row['case_id'],row['method'])
        if key not in cache:
            case=load_frozen_case(PRIMARY,*key,environment=env);ps=providers(case['problem'],env,witness_rows);ps[-1].warmup(r['initial_vector']);cache[key]=(case,ps)
        case,(_,original,g2,p2,view,provider)=cache[key];z0=np.load(run/row['derived_seed_path'],allow_pickle=False)
        invokes+=r['minimize_invocations']
        if vector_hash(z0)!=vector_hash(r['initial_vector']) or r['minimize_invocations']!=1:errors.append('seed/invocation')
        if digest(run/row['derived_seed_path'])!=digest(row['original_seed_path']):errors.append('seed bytes')
        if r['common_witness_sha256']!=digest(run/'witness_selection/frozen_union.json'):errors.append('common union')
        compare(view.dimensions(z0),r['constraint_dimensions'],'dimensions');compare(constraint_row_metadata(view,z0),read(folder/'constraint_rows.json'),'rows')
        compare(case['problem'].config,r['config'],'config');checked={};choices=[]
        for c in r['candidate_checks']:
            z=c['vector'];identity=vector_hash(z)
            if identity not in checked:
                e=view.evaluate(z);full=check_full_candidate(case['problem'],z,case);iv=audit_intervals(case['problem'],z)
                checked[identity]=dict(objective=e['objective'],collocation=_constraint_report(e['equality'],e['inequality'],view.config),
                    constraint_report=full['original_dense_report'],full_acceptance=full)
                compare(d5.compact_intervals(iv),read(folder/'intervals'/(identity+'.json')),'intervals')
                for phase,vector in [('initial',r['initial_vector']),('latest_iterate',r['latest_iterate']),('selected',r['candidate_vector'])]:
                    if vector is None or identity!=vector_hash(vector):continue
                    keyphase='latest' if phase=='latest_iterate' else phase
                    compare(d5.sample_result(iv,full,e['objective'],z,phase),analysis[keyphase],'plot '+phase)
                    compare(witness_diagnostics(view,z,iv['finest_trace']),analysis['witness_margins'][phase],'witness margins')
                    recomputed=extract_runs(iv['finest_trace'],view.config,row['solve_id'])
                    saved=[{k:v for k,v in q.items() if k!='overlaps_old_observed_run'} for q in analysis['remaining_motion_runs'][phase]]
                    compare(recomputed,saved,'remaining motion runs')
            for field,value in checked[identity].items():compare(value,c[field],c['iterate']+'.'+field)
            if c['collocation']['feasible'] and c['constraint_report']['feasible']:choices.append((c['objective'],c['iterate']))
        if (min(choices)[1] if choices else None)!=r['selected_iterate']:errors.append('candidate selection')
        labels=['initial','latest_iterate']+[f"callback_{c['iteration']:04d}" for c in r['callback_snapshots'] if c['collocation'] is not None and c['collocation']['feasible']]
        if labels!=[c['iterate'] for c in r['candidate_checks']]:errors.append('retention')
        for c in r['callback_snapshots']:
            if c['evaluation_complete']:
                e=view.evaluate(c['vector']);compare(e['objective'],c['objective'],'callback objective');compare(e['equality'],c['equality_residuals'],'callback eq');compare(e['inequality'],c['inequality_margins'],'callback ineq')
        old_row=read(D5/'experiment_manifest.json')['starts'][row['pair_index']]
        old_z=read(D5/'solves'/old_row['solve_id']/'solver_result.json')['latest_iterate']
        compare(witness_diagnostics(view,old_z,finest_trace(case['problem'],old_z)),analysis['historical_G2_witness_margins'],'historical witness margins')
        rank=equality_diagnostics(original,provider,dict(initial=r['initial_vector'],latest_iterate=r['latest_iterate'],selected=r['candidate_vector']))
        compare(rank['equality_parity'],analysis['rank']['equality_parity'],'equality parity')
        for a,b in zip(rank['records'],analysis['rank']['records']):
            compare({k:v for k,v in a.items() if k!='diagnostic_wall_s'},{k:v for k,v in b.items() if k!='diagnostic_wall_s'},'rank')
        compare(classify(r,analysis['remaining_motion_runs']['latest_iterate'],analysis['witness_margins']['latest_iterate'],benign=row['case_role']=='BENIGN_CONTROL'),analysis['classification'],'classification')
    if invokes!=5 or read(run/'optimization_completed.json')['actual_new_GP_solves']!=5:errors.append('exactly five solves')
    audit,tables=assemble(run);compare(audit,read(run/'plot_input.json'),'plot payload');compare(audit['summary'],read(run/'aggregate/summary.json'),'summary')
    with tempfile.TemporaryDirectory(prefix='diag06_validate_') as tmp:
        for name,rows in tables.items():
            path=Path(tmp)/(name+'.csv');table(path,rows);count+=1
            if path.read_bytes()!=(run/'aggregate'/path.name).read_bytes():errors.append('table '+name)
    from plot_gp_se2_diag06 import PLOTS,expected_numeric
    for name in PLOTS:
        side=read(run/'plots'/(name+'.json'));count+=1
        if side['numeric_data']!=expected_numeric(name,audit):errors.append('plot numbers '+name)
        if side['image_sha256']!=digest(run/'plots'/(name+'.png')):errors.append('PNG '+name)
        errors.extend(d5.d4.check_hashes(side['source_hashes']))
    with zipfile.ZipFile(run/'review_bundle.zip') as archive:
        for name in archive.namelist():
            count+=1
            if archive.read(name)!=(run/'review_bundle'/name).read_bytes():errors.append('ZIP '+name)
    if not read(run/'derivative_checks/validation.json')['valid']:errors.append('derivative gate')
    write(run/'validation.json',dict(valid=not errors,authoritative=True,errors=errors,check_count=count,wall_s=time.perf_counter()-start,
        new_optimization=0,new_MPC=0,new_rollout=0,scientific_failure_is_artifact_failure=False))
    print(read(run/'validation.json'),flush=True)
    if errors:raise SystemExit(1)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','derivatives','freeze','solve','analyze','validate']);p.add_argument('--run',type=Path,required=True)
    args=p.parse_args();globals()[args.action](args.run.resolve())

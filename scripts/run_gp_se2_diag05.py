#!/usr/bin/env python3
"""Frozen five-start inequality-only ablation; G0/G1 are saved baselines."""
from __future__ import annotations
import argparse
import copy
import csv
import importlib.metadata
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import zipfile
for name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):
    os.environ[name]='1'
os.environ['JAX_PLATFORMS']='cpu';os.environ['JAX_ENABLE_X64']='true'
os.environ.setdefault('XLA_FLAGS','--xla_cpu_multi_thread_eigen=false intra_op_parallelism_threads=1')
os.environ.setdefault('MPLCONFIGDIR','/tmp/gp_se2_diag05_mpl')
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
import yaml
from run_gp_se2_ref01 import read,write,copy_new,digest,utc,revision,table,clean
import run_gp_se2_diag04 as d4
from reconciliation.gp_se2_diag05_constraints import InequalityOnlyView,InequalityOnlyDerivatives,G2,margin_diagnostics,constraint_row_metadata
from reconciliation.gp_se2_diag05_validation import PROTOCOL,schedule,verify_point,equality_diagnostics,classify,vector_hash
from reconciliation.gp_se2_diag04_solver import run_refined
from reconciliation.gp_se2_diag02_derivatives import DerivativeProvider,DerivativeError
from reconciliation.gp_se2_diag02_environment import EnvironmentDerivatives
from reconciliation.gp_se2_diag_acceptance import check_full_candidate
from reconciliation.gp_se2_diag03_intervals import audit_intervals
from reconciliation.gp_se2_diag03_audit import compare_tree
from reconciliation.gp_se2_formulation import _constraint_report
from reconciliation.gp_se2_diagnostics import load_frozen_case
from reconciliation.gp_se2_environment import HospitalEnvironment
D4=ROOT/'data/robotless_gp_se2_diag_04/primary_20260920T015400Z'
PRIMARY=d4.PRIMARY
OUTPUT=ROOT/'data/robotless_gp_se2_diag_05'
STARTING_SHA='070f2ca5cff44fcd3027e86fd198e8ebb1addd57'
CODE=['src/reconciliation/gp_se2_diag05_constraints.py','src/reconciliation/gp_se2_diag05_validation.py',
      'scripts/run_gp_se2_diag05.py','scripts/plot_gp_se2_diag05.py','tests/test_gp_se2_diag05.py']


def verify_frozen(run):
    s=read(run/'source.json');failed=d4.check_hashes(s['preserved_hashes'])+d4.check_hashes(s['user_config_hashes'])
    failed += [p for p,h in s['experiment_code_sha256'].items() if digest(ROOT/p)!=h]
    failed += [p for p,h in s['frozen_input_sha256'].items() if digest(run/p)!=h]
    if failed:raise ValueError('SOURCE_OR_IMPLEMENTATION_MISMATCH: '+str(failed))
    return s


def prepare(run):
    if run.exists() or not run.is_relative_to(OUTPUT):raise FileExistsError('new exclusive DIAG05 output required')
    start=time.perf_counter();old=d4.verify_frozen(D4)
    authority=[D4/'validation.json',d4.DIAG03/'validation.json',PRIMARY/'validation.json']
    if not all(read(p)['valid'] for p in authority):raise ValueError('authoritative validation fails')
    expected=dict(old['preserved_hashes'])
    expected.update({str(p):digest(p) for p in D4.rglob('*') if p.is_file()})
    expected.update({str(ROOT/p):h for p,h in old['experiment_code_sha256'].items()})
    c=yaml.safe_load((D4/'config_snapshot.yaml').read_text())['formulation']
    for k,v in dict(horizon_s=3.,support_dt_s=.1,max_iterations=200,wall_time_s=30.,ftol=1e-7,
        v_max=.8,w_max=3.,a_v_max=2.,a_w_max=5.,equality_tolerance=1e-5,inequality_tolerance=1e-5).items():
        if c[k]!=v:raise ValueError('unexpected authoritative setting: '+k)
    run.mkdir(parents=True);copy_new(D4/'config_snapshot.yaml',run/'config_snapshot.yaml')
    historical=read(D4/'experiment_manifest.json')['starts'];rows=schedule();baseline=[]
    for row in rows:
        pair=[r for r in historical if r['pair_index']==row['pair_index']]
        if len(pair)!=2 or [r['grid'] for r in pair]!=['G0_ORIGINAL','G1_QUARTER']:raise ValueError('historical pair missing')
        seed=D4/pair[0]['derived_seed_path'];z=np.load(seed,allow_pickle=False)
        if z.dtype!=np.float64 or z.shape!=(150,):raise ValueError('seed precision/chart mismatch')
        for h in pair:
            path=D4/'solves'/h['solve_id']/'solver_result.json';r=read(path)
            if digest(D4/h['derived_seed_path'])!=digest(seed) or vector_hash(r['initial_vector'])!=vector_hash(z):raise ValueError('paired initialization mismatch')
            baseline.append(dict(h,solver_path=str(path),solver_sha256=digest(path),provenance='HISTORICAL_DIAG04_NOT_RERUN'))
        if digest(seed)!=digest(pair[0]['seed_path']):raise ValueError('original GP02 initialization changed')
        target=f'initializations/pair_{row["pair_index"]}.npy';copy_new(seed,run/target)
        row.update(derived_seed_path=target,original_seed_path=pair[0]['seed_path'],diag04_seed_path=str(seed),
            seed_sha256=digest(seed),seed_vector_sha256=vector_hash(z),case_directory=pair[0]['case_directory'])
    write(run/'experiment_manifest.json',dict(starts=rows,planned_new_solves=5,historical_start_count=10))
    write(run/'historical_baselines/diag04_g0_g1_manifest.json',dict(primary=str(D4),starts=baseline,rerun=False))
    copy_new(D4/'derivative_checks/point_manifest.json',run/'derivative_checks/point_manifest.json')
    write(run/'protocol.json',dict(PROTOCOL,frozen_utc=utc(),execution_order=[r['solve_id'] for r in rows],
        verification_point_manifest='literal DIAG04 file, including original recorded thresholds',
        software={p:importlib.metadata.version(p) for p in ('numpy','scipy','jax','jaxlib','shapely','matplotlib')}))
    write(run/'source.json',dict(experiment='GP-SE2-DIAG-05',starting_git_sha=STARTING_SHA,experiment_git_sha=revision(),
        historical_diag04_execution_sha=old['experiment_git_sha'],historical_diag04_final_sha=STARTING_SHA,
        primary_source=str(PRIMARY),historical_baseline=str(D4),environment_path=old['environment_path'],
        environment_export_source=old['environment_export_source'],original_config_sha256=old['original_config_sha256'],
        source_frame=old['source_frame'],preserved_hashes=expected,user_config_hashes=old['user_config_hashes'],
        authoritative_validation_hashes={str(p):digest(p) for p in authority},
        experiment_code_sha256={p:digest(ROOT/p) for p in CODE},
        frozen_input_sha256={str(p.relative_to(run)):digest(p) for p in run.rglob('*') if p.is_file()},
        preparation_wall_s=time.perf_counter()-start,no_historical_arrays_copied=True))
    print(json.dumps(dict(prepared=str(run),new_solves=5,historical_starts=10,verification_points=15,preserved_files=len(expected))),flush=True)


def providers(base,env):
    geometry=EnvironmentDerivatives(env,radius=.20);original=DerivativeProvider(base,geometry)
    view=InequalityOnlyView(base);provider=InequalityOnlyDerivatives(view,original)
    return geometry,original,view,provider


def derivatives(run):
    s=verify_frozen(run);folder=run/'derivative_checks'
    if (folder/'started.json').exists():raise FileExistsError('verification already attempted; no overwrite')
    write(folder/'started.json',dict(utc=utc()));start=time.perf_counter()
    env=HospitalEnvironment.load(s['environment_path']);frozen=read(folder/'point_manifest.json');cache={};reports=[]
    for point in frozen['points']:
        key=(point['case_id'],point['method'])
        if key not in cache:
            case=load_frozen_case(PRIMARY,*key,environment=env);cache[key]=(case,providers(case['problem'],env))
        case,(geometry,original,view,provider)=cache[key]
        try:
            report,arrays=verify_point(point['name'],point['vector'],case['problem'],geometry,original,view,provider,np.asarray(frozen['directions']))
            with (folder/(point['name']+'.npz')).open('xb') as f:np.savez_compressed(f,**arrays)
        except DerivativeError as exc:
            report=dict(name=point['name'],valid=False,status='DERIVATIVE_VALIDATION_FAILED',reason_code=exc.reason_code,error=str(exc))
        write(folder/(point['name']+'.json'),report);reports.append(report)
        print(json.dumps(dict(point=point['name'],valid=report['valid'])),flush=True)
    valid=len(reports)==15 and all(r['valid'] for r in reports)
    write(folder/'validation.json',dict(valid=valid,point_count=len(reports),wall_s=time.perf_counter()-start,
        status='VERIFIED_WITH_DECLARED_BRANCH_LIMITATIONS' if valid else 'DERIVATIVE_VALIDATION_FAILED',new_optimizer_calls=0))
    if not valid:raise SystemExit('DERIVATIVE_VALIDATION_FAILED: primary blocked')


def solve(run):
    s=verify_frozen(run)
    if not read(run/'derivative_checks/validation.json')['valid']:raise ValueError('DERIVATIVE_VALIDATION_FAILED')
    if (run/'optimization_started.json').exists():raise FileExistsError('no retry')
    rows=read(run/'experiment_manifest.json')['starts'];write(run/'optimization_started.json',dict(utc=utc(),schedule=rows))
    begin=time.perf_counter();t=time.perf_counter();env=HospitalEnvironment.load(s['environment_path']);environment_s=time.perf_counter()-t
    actual=0
    for row in rows:
        folder=run/'solves'/row['solve_id'];write(folder/'started.json',dict(row,utc=utc()));t=time.perf_counter()
        case=load_frozen_case(PRIMARY,row['case_id'],row['method'],environment=env);z=np.load(run/row['derived_seed_path'],allow_pickle=False)
        if vector_hash(z)!=row['seed_vector_sha256'] or digest(run/row['derived_seed_path'])!=row['seed_sha256']:raise ValueError('seed mismatch')
        loading=time.perf_counter()-t;t=time.perf_counter();_,original,view,provider=providers(case['problem'],env);construction=time.perf_counter()-t
        t=time.perf_counter();provider.warmup(z);warmup=time.perf_counter()-t;setup=provider.stats();dimensions=view.dimensions(z)
        expected=1383 if row['method']=='M3_GP_CONSTRAINED' else 1293
        if dimensions['equality_count']!=30 or dimensions['inequality_count']!=expected:raise ValueError('G2 dimensions mismatch')
        if not np.array_equal(provider.equality_jacobian(z),original.equality_jacobian(z)):raise ValueError('equality parity failure')
        write(folder/'constraint_rows.json',constraint_row_metadata(view,z));provider.reset_stats();view.clear_cache()
        result=run_refined(view,z,derivative_provider=provider,case=case,initialization_name=row['initialization'])
        result.update(row,provenance='NEW_DIAG05_G2',input_loading_wall_s=loading,provider_construction_wall_s=construction,
            compilation_warmup_wall_s=warmup,warmup_stats=setup,constraint_dimensions=dimensions,
            cold_setup_solve_validation_s=loading+construction+warmup+result['total_setup_solve_post_wall_time_s'])
        write(folder/'solver_result.json',result);actual+=result['minimize_invocations']
        print(json.dumps(dict(solve=row['solve_id'],termination=result['termination'],status=result['solver_status'],iterations=result['iterations'],
            candidate=result['candidate_found'],full=result['selected_full_feasible'],latest_full=result['latest_full_feasible'])),flush=True)
    write(run/'optimization_completed.json',dict(utc=utc(),actual_new_GP_solves=actual,planned_new_GP_solves=5,
        wall_s=time.perf_counter()-begin,environment_loading_s=environment_s,new_MPC_solves=0,new_rollouts=0,new_GUI=0,new_VLA=0))


def compact_intervals(value):
    return {k:value[k] for k in ('interval_extrema','violation_brackets','representative_intervals','grid_summary','knot_limits')}


def sample_result(interval,full,objective,z,source):
    trace=dict(interval['finest_trace']);trace['local_u']=trace['local_fraction']
    return dict(samples=trace,full_feasible=full.get('full_feasible'),objective=objective,vector_sha256=vector_hash(z),source=source)


def result_row(row,r):
    latest=r['candidate_checks'][1];full=latest['full_acceptance'];a=full.get('additional_grid',{});flags=a.get('flags',{})
    d=dict(row,**{k:r[k] for k in ('termination','solver_status','solver_success','iterations','candidate_found','selected_iterate',
        'returned_initial_unchanged','latest_full_feasible','selected_full_feasible','candidate_objective','solve_wall_time_s')},
        latest_grid_feasible=latest['collocation']['feasible'],latest_dense_feasible=latest['constraint_report']['feasible'],
        latest_original_full_feasible=latest['original_full_feasible'],latest_objective=latest['objective'],
        initial_objective=r['candidate_checks'][0]['objective'],failure_flags=[k for k,v in flags.items() if not v],
        classification=classify(r,benign=row['case_role']=='BENIGN_CONTROL'))
    for key in ('maximum_absolute_lateral_velocity_m_s','minimum_linear_speed_m_s','maximum_linear_speed_m_s',
        'maximum_absolute_angular_speed_rad_s','maximum_absolute_linear_acceleration_m_s2','maximum_absolute_angular_acceleration_rad_s2'):
        d[key]=a.get(key)
    for key in ('original_goal','original_route','original_workspace','original_obstacle_clearance'):d[key]=flags.get(key)
    return d


def assemble(run):
    """Saved data only: one consistent table/plot payload used again by validator."""
    oldplot=read(D4/'plot_input.json');historical=read(run/'historical_baselines/diag04_g0_g1_manifest.json')['starts']
    items=[];outcomes=[];extrema=[];ranks=[];margins=[];timing=[];pairs=[]
    for row in read(run/'experiment_manifest.json')['starts']:
        group=[]
        for h in [h for h in historical if h['pair_index']==row['pair_index']]:
            r=read(h['solver_path']);o=result_row({k:h[k] for k in ('pair_index','solve_id','case_role','method','initialization','grid','provenance')},r)
            item=copy.deepcopy(next(s for s in oldplot['solves'] if s['solve_id']==h['solve_id']))
            # Keep only plotted samples, not repeated full checker reports/arrays.
            for phase in ('latest','selected'):
                if item.get(phase) is not None:
                    item[phase]={k:v for k,v in item[phase].items() if k in ('samples','full_feasible','objective','vector_sha256','source')}
            oldmotion=read(Path(h['solver_path']).with_name('motion_analysis.json'))
            for phase,value in oldmotion.items():
                if value is not None:
                    extrema.extend(dict(solve_id=h['solve_id'],phase=phase,provenance='HISTORICAL_DIAG04_NOT_RERUN',**e)
                        for e in value['interval_extrema'] if e['grid']=='SUPPLEMENTAL_0.001_OFFSET')
            oldrank=read(Path(h['solver_path']).with_name('conditioning.json'))
            for record in oldrank['records']:
                if record['available'] and record['grid']==h['grid']:
                    for scale,m in record['matrices'].items():
                        ranks.append(dict(solve_id=h['solve_id'],source=record['source'],grid=record['grid'],scale=scale,
                            provenance='HISTORICAL_DIAG04_NOT_RERUN',**m))
            item.update(provenance='HISTORICAL_DIAG04_NOT_RERUN',outcome=o)
            items.append(item);outcomes.append(o);group.append(o)
        folder=run/'solves'/row['solve_id'];r=read(folder/'solver_result.json');analysis=read(folder/'analysis.json')
        o=result_row({k:row[k] for k in ('pair_index','solve_id','case_role','method','initialization','grid')},r);o['provenance']='NEW_DIAG05_G2'
        byhash={c['vector_sha256']:c for c in r['candidate_checks']};history=[]
        for snap in [dict(elapsed_s=0.,objective=r['candidate_checks'][0]['objective'],vector=r['initial_vector'])]+r['callback_snapshots']:
            checked=byhash.get(vector_hash(snap['vector']));history.append(dict(elapsed_s=snap['elapsed_s'],objective=snap['objective'],
                full_feasible=None if checked is None else checked['grid_and_full_feasible'],certification='post-solve'))
        items.append(dict(row,provenance='NEW_DIAG05_G2',outcome=o,latest=analysis['latest'],selected=analysis['selected'],
            history=history,conditioning=analysis['conditioning'],margins=analysis['margins']))
        outcomes.append(o);group.append(o)
        for phase,vector in [('latest',r['latest_iterate']),('selected',r['candidate_vector'])]:
            if vector is not None:
                value=read(folder/'intervals'/(vector_hash(vector)+'.json'))
                extrema.extend(dict(solve_id=row['solve_id'],phase=phase,provenance='NEW_DIAG05_G2',**e) for e in value['interval_extrema'] if e['grid']=='SUPPLEMENTAL_0.001_OFFSET')
        for record in analysis['rank']['records']:
            if record['available']:
                for scale,m in record['matrices'].items():ranks.append(dict(solve_id=row['solve_id'],source=record['source'],grid=record['grid'],scale=scale,provenance='NEW_DIAG05_G2',**m))
            else:ranks.append(dict(solve_id=row['solve_id'],**record))
        for phase,rows in analysis['margins'].items():margins.extend(dict(solve_id=row['solve_id'],phase=phase,**m) for m in rows)
        timing.append(dict(solve_id=row['solve_id'],provenance='NEW_DIAG05_G2',**{k:r[k] for k in ('input_loading_wall_s','provider_construction_wall_s',
            'compilation_warmup_wall_s','solve_wall_time_s','post_solve_validation_time_s','cold_setup_solve_validation_s',
            'objective_evaluations','equality_evaluations','inequality_evaluations','derivative_calls')},
            post_rank_s=analysis['rank']['wall_s'],analysis_wall_s=analysis['wall_s'],primal_cache_misses=r['profiling']['evaluator_cache_misses']))
        g0,g1,g2=group;pairs.append(dict(pair_index=row['pair_index'],case_role=row['case_role'],method=row['method'],initialization=row['initialization'],
            G0_status=g0['solver_status'],G1_status=g1['solver_status'],G2_status=g2['solver_status'],
            G0_full=g0['selected_full_feasible'],G1_full=g1['selected_full_feasible'],G2_full=g2['selected_full_feasible'],
            G1_to_G2_convergence_recovered=not g1['solver_success'] and g2['solver_success'],**g2['classification']))
    hard=pairs[:4];benign=pairs[4];recovered=sum(p['G2_full'] for p in hard)
    if recovered:interpretation='FEASIBILITY_RECOVERY_OBSERVED' if not benign['outcome']=='BENIGN_REGRESSION' else 'MIXED_INEQUALITY_ONLY_RESULT'
    elif all(p['outcome']=='NUMERICAL_RECOVERY_LATERAL_REMAINS' for p in hard) and benign['outcome']=='BENIGN_OPTIMIZATION_RECOVERED':
        interpretation='EQUALITY_CAUSED_NUMERICAL_REGRESSION_WITH_LATERAL_GAP_REMAINING'
    elif all(p['outcome']=='NO_NUMERICAL_RECOVERY' for p in hard):interpretation='NO_FEASIBILITY_OR_NUMERICAL_RECOVERY'
    elif any(p['outcome']=='SOURCE_OR_DERIVATIVE_BLOCKER' for p in pairs):interpretation='INSUFFICIENT_EVIDENCE'
    else:interpretation='MIXED_INEQUALITY_ONLY_RESULT'
    summary=dict(operational_status='GP_SE2_DIAG_05_COMPLETED_WITH_LIMITATIONS',interpretation=interpretation,
        planned_new_GP_solves=5,actual_new_GP_solves=read(run/'optimization_completed.json')['actual_new_GP_solves'],
        historical_starts=10,historical_reruns=0,hard_full_valid_starts=recovered,hard_event_count=1,benign_event_count=1,
        benign_outcome=benign['outcome'],pairs=pairs,new_VLA=0,new_MPC=0,new_rollout=0,new_GUI=0,
        physical_acceptance_unchanged=True,continuous_feasibility_proof=False,navigation_claim=False)
    tables={'all_methods':outcomes,'paired_outcomes':pairs,'motion_extrema':extrema,'constraint_margins':margins,'equality_rank':ranks,'timing':timing}
    return dict(solves=items,contexts=oldplot['contexts'],formulation_config=oldplot['formulation_config'],summary=summary),tables


def analyze(run):
    s=verify_frozen(run)
    if not (run/'optimization_completed.json').exists():raise ValueError('five starts required')
    if (run/'analysis_started.json').exists():raise FileExistsError('no analysis overwrite')
    write(run/'analysis_started.json',dict(utc=utc()));start=time.perf_counter();env=HospitalEnvironment.load(s['environment_path']);cache={}
    for row in read(run/'experiment_manifest.json')['starts']:
        folder=run/'solves'/row['solve_id'];r=read(folder/'solver_result.json');t=time.perf_counter();key=(row['case_id'],row['method'])
        if key not in cache:
            case=load_frozen_case(PRIMARY,*key,environment=env);ps=providers(case['problem'],env);ps[-1].warmup(r['initial_vector']);cache[key]=(case,ps)
        case,(_,original,view,provider)=cache[key]
        vectors=dict(initial=r['initial_vector'],latest_iterate=r['latest_iterate'],selected=r['candidate_vector'])
        rank=equality_diagnostics(original,provider,vectors);margins={k:margin_diagnostics(view,provider,z) for k,z in vectors.items() if z is not None}
        unique={};selected=None;latest=None
        for check in r['candidate_checks']:
            z=check['vector'];identity=vector_hash(z)
            if identity not in unique:
                interval=audit_intervals(case['problem'],z);unique[identity]=interval
                write(folder/'intervals'/(identity+'.json'),compact_intervals(interval))
            if check['iterate']=='latest_iterate':latest=sample_result(unique[identity],check['full_acceptance'],check['objective'],z,'latest_iterate')
            if check['iterate']==r['selected_iterate']:selected=sample_result(unique[identity],check['full_acceptance'],check['objective'],z,check['iterate'])
        conditioning={}
        for rr in rank['records']:
            if rr['available']:
                label='latest' if rr['source']=='latest_iterate' else rr['source']
                conditioning.setdefault(label,{})[rr['grid']]=dict(raw_singular_values=rr['matrices']['raw']['singular_values'],scaled_singular_values=rr['matrices']['column_scaled']['singular_values'])
        write(folder/'analysis.json',dict(rank=rank,margins=margins,latest=latest,selected=selected,conditioning=conditioning,
            candidate_interval_checks=[dict(source=c['iterate'],vector_sha256=c['vector_sha256']) for c in r['candidate_checks']],
            unique_vectors=len(unique),wall_s=time.perf_counter()-t))
    audit,tables=assemble(run);(run/'aggregate').mkdir()
    for name,rows in tables.items():table(run/'aggregate'/(name+'.csv'),rows)
    write(run/'aggregate/summary.json',audit['summary']);write(run/'plot_input.json',audit)
    write(run/'analysis_completed.json',dict(utc=utc(),wall_s=time.perf_counter()-start))
    from plot_gp_se2_diag05 import generate
    t=time.perf_counter();generate(run)
    write(run/'rendering_completed.json',dict(utc=utc(),wall_s=time.perf_counter()-t))


def validate(run):
    if (run/'validation.json').exists():raise FileExistsError('validation exists')
    start=time.perf_counter();s=verify_frozen(run);env=HospitalEnvironment.load(s['environment_path']);errors=[];count=0;cache={}
    def compare(a,b,label):
        nonlocal count
        records=compare_tree(clean(a),b,label);count+=len(records);errors.extend(r['field'] for r in records if not r['numerical_agreement'])
    manifest=read(run/'experiment_manifest.json')['starts']
    if [r['solve_id'] for r in manifest]!=[r['solve_id'] for r in schedule()]:errors.append('schedule coverage')
    for row in manifest:
        folder=run/'solves'/row['solve_id'];r=read(folder/'solver_result.json');analysis=read(folder/'analysis.json');key=(row['case_id'],row['method'])
        if key not in cache:
            case=load_frozen_case(PRIMARY,*key,environment=env);ps=providers(case['problem'],env);ps[-1].warmup(r['initial_vector']);cache[key]=(case,ps)
        case,(_,original,view,provider)=cache[key];z0=np.load(run/row['derived_seed_path'],allow_pickle=False)
        if vector_hash(z0)!=vector_hash(r['initial_vector']) or r['minimize_invocations']!=1:errors.append('seed/invocation')
        compare(view.dimensions(z0),r['constraint_dimensions'],'dimensions');compare(constraint_row_metadata(view,z0),read(folder/'constraint_rows.json'),'row metadata')
        compare(case['problem'].config,r['config'],'original config')
        checked={};choices=[]
        for c in r['candidate_checks']:
            z=c['vector'];identity=vector_hash(z)
            if identity not in checked:
                e=view.evaluate(z);full=check_full_candidate(case['problem'],z,case);iv=audit_intervals(case['problem'],z)
                checked[identity]=dict(objective=e['objective'],collocation=_constraint_report(e['equality'],e['inequality'],view.config),
                    constraint_report=full['original_dense_report'],full_acceptance=full)
                compare(compact_intervals(iv),read(folder/'intervals'/(identity+'.json')),'interval audit')
                if identity==vector_hash(r['latest_iterate']):compare(sample_result(iv,full,e['objective'],z,'latest_iterate'),analysis['latest'],'latest plot')
                if r['candidate_vector'] is not None and identity==vector_hash(r['candidate_vector']):
                    compare(sample_result(iv,full,e['objective'],z,r['selected_iterate']),analysis['selected'],'selected plot')
            for field,value in checked[identity].items():compare(value,c[field],c['iterate']+'.'+field)
            if c['collocation']['feasible'] and c['constraint_report']['feasible']:choices.append((c['objective'],c['iterate']))
        if (min(choices)[1] if choices else None)!=r['selected_iterate']:errors.append('candidate selection')
        labels=['initial','latest_iterate']+[f"callback_{c['iteration']:04d}" for c in r['callback_snapshots'] if c['collocation'] is not None and c['collocation']['feasible']]
        if labels!=[c['iterate'] for c in r['candidate_checks']]:errors.append('retention')
        for c in r['callback_snapshots']:
            if c['evaluation_complete']:
                e=view.evaluate(c['vector']);compare(e['objective'],c['objective'],'callback objective')
                compare(e['equality'],c['equality_residuals'],'callback eq');compare(e['inequality'],c['inequality_margins'],'callback ineq')
        vectors=dict(initial=r['initial_vector'],latest_iterate=r['latest_iterate'],selected=r['candidate_vector'])
        rank=equality_diagnostics(original,provider,vectors)
        compare(rank['equality_parity'],analysis['rank']['equality_parity'],'equality parity')
        for a,b in zip(rank['records'],analysis['rank']['records']):compare({k:v for k,v in a.items() if k!='diagnostic_wall_s'},{k:v for k,v in b.items() if k!='diagnostic_wall_s'},'rank')
        compare({k:margin_diagnostics(view,provider,z) for k,z in vectors.items() if z is not None},analysis['margins'],'margins')
    audit,tables=assemble(run);compare(audit,read(run/'plot_input.json'),'plot payload');compare(audit['summary'],read(run/'aggregate/summary.json'),'summary')
    with tempfile.TemporaryDirectory(prefix='diag05_validate_') as tmp:
        for name,rows in tables.items():
            path=Path(tmp)/(name+'.csv');table(path,rows);count+=1
            if path.read_bytes()!=(run/'aggregate'/path.name).read_bytes():errors.append('table '+name)
    from plot_gp_se2_diag05 import PLOTS,expected_numeric
    for name in PLOTS:
        side=read(run/'plots'/(name+'.json'));count+=1
        if side['numeric_data']!=expected_numeric(name,audit):errors.append('plot data '+name)
        if side['image_sha256']!=digest(run/'plots'/(name+'.png')):errors.append('PNG '+name)
        errors.extend(d4.check_hashes(side['source_hashes']))
    with zipfile.ZipFile(run/'review_bundle.zip') as archive:
        for name in archive.namelist():
            count+=1
            if archive.read(name)!=(run/'review_bundle'/name).read_bytes():errors.append('ZIP '+name)
    if not read(run/'derivative_checks/validation.json')['valid']:errors.append('derivative gate')
    write(run/'validation.json',dict(valid=not errors,authoritative=True,errors=errors,check_count=count,wall_s=time.perf_counter()-start,
        new_optimization=0,new_MPC=0,new_rollout=0,scientific_failure_is_artifact_failure=False))
    print(json.dumps(read(run/'validation.json')),flush=True)
    if errors:raise SystemExit(1)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','derivatives','solve','analyze','validate']);p.add_argument('--run',type=Path,required=True)
    args=p.parse_args();globals()[args.action](args.run.resolve())

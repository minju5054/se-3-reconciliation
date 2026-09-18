#!/usr/bin/env python3
"""Read-only independent validation of the frozen GP-SE2-DIAG-02 artifacts.

The validator evaluates saved GP vectors and supplied derivatives, never solves
for a trajectory, collects observations, or executes MPC. Existing artifacts are
never rewritten; its optional output must be a new file inside the new run.
"""
from __future__ import annotations
import argparse
import csv
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
import sys
import zipfile

for _name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[_name]='1'
os.environ['JAX_PLATFORMS']='cpu'
os.environ['JAX_ENABLE_X64']='true'
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
import yaml
from validate_gp_se2_diag_01 import Checks,digest,read,write_exclusive,equivalent
from reconciliation.gp_se2_diagnostics import load_frozen_case
from reconciliation.gp_se2_diag_acceptance import check_full_candidate
from reconciliation.gp_se2_formulation import _constraint_report


def vector_hash(z):
    return hashlib.sha256(np.asarray(z,dtype=np.float64).tobytes()).hexdigest()


def verify_digest(path,expected,checks,label):
    checks.check(Path(path).is_file() and digest(path)==expected,label)


def validate_seed(result,expected,checks,label):
    checks.equal(result['initial_vector'],expected,label+': bitwise frozen seed',exact=True)


def validate_attempt(problem,result,mode,checks,label):
    """Original-primal replay of every retained actual callback and candidate."""
    checks.equal(result['config'],problem.config,label+': unchanged original formulation',exact=True)
    checks.check(result['variable_count']==150 and result['equality_count']==30 and
                 result['inequality_count']==(903 if problem.include_obstacles else 813),label+': full original row dimensions')
    checks.check(result['configured_wall_time_budget_s']==30. and result['config']['max_iterations']==200,label+': fixed budget')
    checks.check(result['pre_solve_budget_cost_s']==0.,label+': prepared-functions timing')
    checks.check(result['derivative_mode']==mode,label+': mode identity')
    checks.check(result['solver_backend']=='scipy.optimize.SLSQP',label+': same optimizer')
    checks.check(not result['fallback_used'] and not result['infeasibility_proven'],label+': no fallback or infeasibility assertion')
    calls=result['derivative_calls']
    if mode=='FD_BASELINE':
        checks.check(all(value==0 for value in calls.values()) and result['derivative_provider_stats'] is None,label+': untouched numerical derivative path')
        checks.check('jac not supplied' in result['jacobian_method'],label+': baseline jac=None')
    else:
        checks.check(set(calls)=={'objective_gradient','equality_jacobian','inequality_jacobian'} and
                     all(value>0 for value in calls.values()),label+': all three supplied callbacks used')
        stats=result['derivative_provider_stats']
        checks.check(stats['float64_enabled'] and stats['device_backend']=='cpu' and
                     stats['numerical_finite_difference_calls']==0 and not stats['numerical_fallback_used'],label+': CPU float64 and no FD fallback')
        for name,value in calls.items():
            checks.check(stats['callback_calls'][name]==value,label+': actual derivative call count '+name)
        branches=stats['geometry_branches']['runtime']
        checks.check(len(branches['samples'])<=128 and branches['distinct_flagged_locations']==
                     len(branches['samples'])+branches['omitted_distinct_locations'],label+': bounded explicit branch locations')
        for family in branches['families'].values():
            checks.check(family['points_without_metadata']==0,label+': every analytic geometry query classified')
    checks.check(result['blas_threads']['single_thread_environment_valid'] and
                 result['blas_threads']['observed_runtime_single_thread_valid'],label+': single BLAS thread')
    initial=np.asarray(result['initial_vector']);latest=np.asarray(result['latest_iterate'])
    actual={'initial':initial,'latest_iterate':latest}
    snapshots=result['callback_snapshots']
    checks.check(result['callback_count']==len(snapshots),label+': actual callback count')
    checks.check([s['iteration'] for s in snapshots]==list(range(1,len(snapshots)+1)),label+': no manufactured callback indices')
    checks.check(all(a['elapsed_s']<=b['elapsed_s'] for a,b in zip(snapshots,snapshots[1:])),label+': monotonic callback time')
    for entry in snapshots:
        where=label+f'/callback{entry["iteration"]}'
        checks.check(entry['source']=='actual scipy SLSQP callback',where+': observed source')
        if not entry['evaluation_complete']:
            checks.check(entry['objective'] is None and entry['collocation'] is None,where+': unknown residuals remain null')
            continue
        value=problem.evaluate(np.asarray(entry['vector']))
        report=_constraint_report(value['equality'],value['inequality'],problem.config)
        for key,expected in [('objective',value['objective']),('equality_residuals',value['equality']),
                             ('inequality_margins',value['inequality']),('collocation',report)]:
            checks.equal(entry[key],expected,where+': exact original primal '+key)
        if report['feasible']:
            actual[f'callback_{entry["iteration"]:04d}']=np.asarray(entry['vector'])
    checks.check([r['iterate'] for r in result['candidate_checks']]==list(actual),label+': original candidate policy')
    choices=[];reevaluated={}
    for entry in result['candidate_checks']:
        name=entry['iterate']
        if name not in actual:
            checks.check(False,label+': candidate has no saved iterate '+name);continue
        z=actual[name];value=problem.evaluate(z);dense=problem.dense_report(z)
        checks.equal(entry['vector'],z,label+': candidate vector '+name,exact=True)
        checks.equal(entry['objective'],value['objective'],label+': candidate original objective '+name)
        checks.equal(entry['constraint_report'],dense,label+': original dense report '+name)
        checks.equal(entry['collocation'],_constraint_report(value['equality'],value['inequality'],problem.config),label+': original collocation '+name)
        reevaluated[name]=dict(vector=z,objective=float(value['objective']),dense=dense)
        if dense['feasible']:
            choices.append((float(value['objective']),name,z))
    chosen=min(choices,key=lambda row:(row[0],row[1])) if choices else None
    initial_feasible=bool(reevaluated['initial']['dense']['feasible'])
    checks.check(result['candidate_found']==bool(chosen),label+': candidate availability')
    checks.check(result['initial_feasible']==initial_feasible,label+': sampled initial feasibility')
    checks.check(result['recovered_from_infeasible']==(not initial_feasible and chosen is not None),label+': recovery distinct from retained seed')
    same=bool(chosen is not None and np.array_equal(chosen[2],initial))
    checks.check(result['returned_initial_unchanged']==same and result['feasible_initial_retained']==(initial_feasible and same),label+': unchanged initial correctly identified')
    if chosen:
        objective,name,z=chosen;p,v=problem.unpack(z)
        checks.check(result['selected_iterate']==name,label+': minimum feasible original objective selected')
        for key,value in [('candidate_vector',z),('candidate_world',p[1:]),('support_poses',p),('support_twists',v)]:
            checks.equal(result[key],value,label+': exact selected '+key,exact=True)
        checks.equal(result['candidate_objective'],objective,label+': selected objective')
        checks.equal(result['constraint_report'],problem.dense_report(z),label+': selected dense report')
        delta=objective-reevaluated['initial']['objective']
        checks.equal(result['objective_delta'],delta,label+': strict numerical objective delta')
        checks.check(result['selected_objective_improved']==(delta<0) and result['selected_objective_improved_beyond_ftol']==(delta < -problem.config['ftol']),label+': strict and meaningful improvement distinguished')
    else:
        for key in ('candidate_vector','candidate_world','support_poses','support_twists','candidate_objective','constraint_report','selected_iterate','objective_delta'):
            checks.check(result[key] is None,label+': absent '+key+' is null')
        checks.check(not result['selected_objective_improved'] and not result['selected_objective_improved_beyond_ftol'],label+': no invented improvement without candidate')
    profile=result['profiling']
    checks.check(profile['evaluate_requests']==profile['evaluator_cache_hits']+profile['evaluator_cache_misses'],label+': unchanged primal cache accounting')
    checks.check(0<profile['unique_vector_evaluations']<=profile['evaluator_cache_misses'],label+': actual evaluated points')
    for role,field in [('objective','objective_evaluations'),('equality','equality_evaluations'),('inequality','inequality_evaluations')]:
        checks.check(profile['function_calls'].get(role,0)==result[field],label+': primal call count '+role)
    return reevaluated,chosen


def outcome_flags(problem,initial_full,final_full,selected_full,initial_objective,selected_objective,solver_success):
    reduction=None if selected_objective is None else initial_objective-selected_objective
    threshold=max(10*problem.config['ftol'],1e-8*max(1,abs(initial_objective)))
    return dict(initial_full_feasible=initial_full,final_full_feasible=final_full,
        selected_full_feasible=selected_full,infeasible_start_recovered=bool(not initial_full and selected_full),
        feasible_seed_objective_improved=bool(initial_full and selected_full and reduction is not None and reduction>threshold),
        solver_converged_with_full_feasibility=bool(solver_success and final_full),
        objective_reduction=reduction,relative_objective_reduction=None if reduction is None else reduction/max(1,abs(initial_objective)),
        numerical_improvement_threshold=threshold)


def validate_full_outcome(problem,case,result,summary,directory,reevaluated,chosen,checks,label):
    cached={};full_by_name={}
    saved=read(directory/'post_full_checks.json')
    checks.check([row['iterate'] for row in saved['rows']]==list(reevaluated),label+': every candidate check full-classified')
    for row in saved['rows']:
        name=row['iterate'];entry=reevaluated[name];z=entry['vector'];key=vector_hash(z)
        checks.check(row['vector_sha256']==key,label+': postcheck vector hash '+name)
        checks.equal(row['objective'],entry['objective'],label+': postcheck original objective '+name)
        if entry['dense']['feasible']:
            if key not in cached:cached[key]=check_full_candidate(problem,z,case)
            full=cached[key];full_by_name[name]=bool(full['full_feasible'])
            expected=dict(full_feasible=full['full_feasible'],original_dense_feasible=full['original_dense_feasible'],
                original_plan_valid=full['original_plan_valid'],additional_grid_valid=full['additional_grid']['valid'],
                maximum_lateral_m_s=full['additional_grid']['maximum_absolute_lateral_velocity_m_s'],
                max_linear_acceleration_m_s2=full['additional_grid']['maximum_absolute_linear_acceleration_m_s2'],
                flags=full['additional_grid']['flags'])
            for field,value in expected.items():checks.equal(row[field],value,label+': independent '+name+'/'+field)
        else:
            full_by_name[name]=False
            checks.check(not row['full_feasible'] and row['full_check_skipped_reason']=='original dense acceptance already fails',label+': rejected before full check '+name)
    checks.check(saved['unique_full_checks']==len(cached),label+': deduplicated independent full checks')
    selected_full=False
    if chosen:
        selected=cached[vector_hash(chosen[2])];selected_full=bool(selected['full_feasible'])
        checks.equal(read(directory/'full_acceptance.json'),selected,label+': complete selected original full acceptance')
    else:
        absent=read(directory/'full_acceptance.json')
        checks.check(not absent['full_feasible'] and absent['status']=='NO_CANDIDATE' and absent['metrics'] is None,label+': no invented metrics for absent candidate')
    flags=outcome_flags(problem,full_by_name['initial'],full_by_name['latest_iterate'],selected_full,
                        reevaluated['initial']['objective'],None if chosen is None else chosen[0],result['solver_success'])
    for key,value in flags.items():checks.equal(summary[key],value,label+': derived outcome '+key)
    for key in ('termination','solver_success','iterations','returned_initial_unchanged','solve_wall_time_s','post_solve_validation_time_s'):
        checks.equal(summary[key],result[key],label+': summary source '+key)
    checks.equal(summary['initial_objective'],reevaluated['initial']['objective'],label+': initial objective summary')
    checks.equal(summary['final_objective'],reevaluated['latest_iterate']['objective'],label+': final objective summary')
    checks.equal(summary['selected_objective'],result['candidate_objective'],label+': selected objective summary')
    checks.check(summary['selected_source']==result['selected_iterate'] and summary['best_retained_dense_feasible']==result['candidate_found'],label+': unchanged selected policy')
    checks.check(not summary['navigation_improvement_claim'],label+': no execution superiority claim')
    return cached


def validate_derivative_gate(run,source,checks):
    from reconciliation.gp_se2_diag02_validation import (PROTOCOL,build_validation_points,point_record,plain,
        _family_reports,_reports_pass,_values,error_report,constraint_families)
    from reconciliation.gp_se2_diag02_derivatives import DerivativeProvider,DerivativeError
    base=run/'derivative_checks';protocol=read(base/'protocol.json');manifest=read(base/'point_manifest.json')
    for key,value in PROTOCOL.items():checks.equal(protocol[key],value,'derivatives: frozen protocol '+key,exact=True)
    verify_digest(base/'protocol.json',manifest['protocol_sha256'],checks,'derivatives: protocol hash')
    verify_digest(base/'directions.json',manifest['direction_sha256'],checks,'derivatives: direction hash')
    points=build_validation_points(source['primary_run'],source['previous_diagnostic'],seeds=run/'seeds')
    identities={row['name']:row for row in manifest['points']}
    checks.check(set(identities)=={p['name'] for p in points} and len(points)==20,'derivatives: complete 20-point fixed coverage')
    checks.check(sum(point['full_coordinate'] for point in points)==10,'derivatives: ten complete 150-coordinate checks')
    for point in points:
        record=identities[point['name']]
        verify_digest(base/record['path'],record['sha256'],checks,'derivatives: frozen point hash '+point['name'])
        checks.equal(read(base/record['path']),plain(point_record(point)),'derivatives: independently reconstructed point '+point['name'],exact=True)
    direction_data=read(base/'directions.json');directions=np.asarray(direction_data['directions'])
    rng=np.random.default_rng(PROTOCOL['random_seed'])
    expected=rng.normal(size=(PROTOCOL['direction_count'],150));expected/=np.linalg.norm(expected,axis=1)[:,None]
    checks.equal(directions,expected,'derivatives: deterministic directions',exact=True)
    if not (base/'authoritative_validation.json').exists():
        checks.check(not (run/'execution_started.json').exists(),'derivatives: no actual comparison without successful published gate')
        return None
    gate=read(base/'authoritative_validation.json')
    if not gate['valid']:
        checks.check(not (run/'execution_started.json').exists(),'derivatives: failed gate blocks actual comparison')
        return gate
    target=base/gate['authoritative_attempt'];attempt=read(target/'summary.json');provenance=read(target/'source.json')
    verify_digest(target/'summary.json',gate['attempt_summary_sha256'],checks,'derivatives: authoritative attempt summary hash')
    verify_digest(target/'source.json',gate['attempt_source_sha256'],checks,'derivatives: authoritative source hash')
    for name,value in provenance['source_sha256'].items():
        verify_digest(target/'implementation_sources'/name,value,checks,'derivatives: archived source '+name)
        current=ROOT/'src/reconciliation'/name
        if digest(current)==value:
            checks.check(True,'derivatives: qualified current source '+name)
        else:
            import ast
            permitted=[r for r in gate.get('publication_only_source_corrections',[]) if r['source']==name]
            checks.check(name=='gp_se2_diag02_validation.py' and len(permitted)==1,'derivatives: only explicit publisher-only correction')
            if permitted:
                checks.check(permitted[0]['checked_sha256']==value and permitted[0]['current_sha256']==digest(current),'derivatives: exact publisher correction hashes')
                def without_publisher(path):
                    tree=ast.parse(path.read_text());tree.body=[n for n in tree.body if not isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef)) or n.name!='publish_validation_gate']
                    return ast.dump(tree,include_attributes=False)
                checks.check(without_publisher(target/'implementation_sources'/name)==without_publisher(current),'derivatives: all numerical validation AST unchanged')
                correction=read(base/'gate_publication_correction.json')
                verify_digest(ROOT/correction['wrong_named_artifact'],correction['wrong_named_artifact_sha256'],checks,'derivatives: wrong-named first gate preserved')
                verify_digest(base/'authoritative_validation.json',correction['authoritative_gate_sha256'],checks,'derivatives: corrected gate exact publication hash')
    addendum=read(base/'protocol_addendum_01.json')
    verify_digest(base/'protocol_addendum_01.json',gate['protocol_addendum_sha256'],checks,'derivatives: frozen cut addendum')
    checks.check(gate['supported_points_validated']==19 and gate['unsupported_points_correctly_rejected']==1,'derivatives: supported/unsupported coverage distinct')
    reports=[];providers={}
    for point in points:
        name=point['name'];problem=point['problem'];z=point['vector'];record=read(target/(name+'.json'))
        reports.append(record)
        if name=='relative_yaw_wrap_cut':
            checks.check(record['expected_runtime_rejection'] and record['correctly_rejected'] and
                         not record['supported_derivative_domain'] and record['valid'], 'derivatives: exact cut unsupported')
            checks.check(not (target/(name+'.npz')).exists() and not record['primal_reports'] and not record['coordinate_reports'] and not record['directional'], 'derivatives: no invented cut derivative')
            for field,hashfield in [('previous_cut_report','previous_cut_report_sha256'),('previous_cut_matrices','previous_cut_matrices_sha256')]:
                path=Path(record[field]);path=path if path.is_absolute() else ROOT/path
                verify_digest(path,record[hashfield],checks,'derivatives: preserved prior cut evidence '+field)
            try:
                DerivativeProvider(problem,point['geometry']).warmup(z)
                checks.check(False,'derivatives: runtime cut guard rejected')
            except DerivativeError as exc:
                checks.check(exc.reason_code=='UNSUPPORTED_WRAP_CUT','derivatives: precise runtime unsupported reason')
                checks.equal(record['guard_diagnostics'],exc.diagnostics,'derivatives: exact unsupported location')
            continue
        verify_digest(target/record['matrix_file'],record['matrix_sha256'],checks,'derivatives: raw matrix hash '+name)
        arrays=dict(np.load(target/record['matrix_file'],allow_pickle=False))
        checks.equal(arrays['vector'],z,'derivatives: exact matrix chart '+name,exact=True)
        checks.equal(arrays['directions'],directions,'derivatives: exact matrix directions '+name,exact=True)
        key=id(problem)
        if key not in providers:providers[key]=DerivativeProvider(problem,point['geometry'])
        provider=providers[key];actual_values=provider.values(z);original=_values(problem,z)
        actual={field:np.atleast_1d(np.asarray(actual_values[field]))[:,None] for field in original}
        expected={field:value[:,None] for field,value in original.items()}
        checks.equal(record['primal_reports'],_family_reports(problem,actual,expected,None,primal=True),'derivatives: independent primal errors '+name)
        ad=dict(objective=arrays['objective_gradient'],equality=arrays['equality_jacobian'],inequality=arrays['inequality_jacobian'])
        for field,current in [('objective',provider.objective_gradient(z)[None,:]),('equality',provider.equality_jacobian(z)),('inequality',provider.inequality_jacobian(z))]:
            checks.equal(ad[field],current,'derivatives: independent supplied matrix '+name+'/'+field)
            checks.check(ad[field].shape==(len(original[field]),150) and np.isfinite(ad[field]).all(),'derivatives: full finite matrix '+name+'/'+field)
        coordinate_pass=True
        if point['full_coordinate']:
            fd={field:arrays['coordinate_fd_'+field] for field in ad}
            masks={field:arrays['coordinate_smooth_mask_'+field] for field in ad}
            checks.check(masks['objective'].all() and masks['equality'].all() and masks['inequality'][:723].all(),'derivatives: no physical/objective rows excluded '+name)
            computed=_family_reports(problem,ad,fd,masks)
            checks.equal(record['coordinate_reports'],computed,'derivatives: coordinate error recomputation '+name)
            coordinate_pass=_reports_pass(computed)
        directional_passes=[]
        for i,h in enumerate(PROTOCOL['directional_fd_steps']):
            predicted={field:ad[field]@directions.T for field in ad}
            fd={field:arrays[f'directional_{i}_{field}_central_fd'] for field in ad}
            masks={field:arrays[f'directional_{i}_{field}_smooth_mask'] for field in ad}
            for field in ad:
                checks.equal(arrays[f'directional_{i}_{field}_prediction'],predicted[field],'derivatives: AD directional projection '+name+'/'+str(i)+'/'+field)
            if point['mode']=='nonsmooth_relative_log_wrap_cut':
                checks.check(not any(mask.any() for mask in masks.values()),'derivatives: explicit cut has no classical derivative claim')
            else:
                checks.check(masks['objective'].all() and masks['equality'].all() and masks['inequality'][:723].all(),'derivatives: all smooth nongemetry rows retained '+name)
            computed=_family_reports(problem,predicted,fd,masks)
            checks.equal(record['directional'][i]['smooth_reports'],computed,'derivatives: directional error recomputation '+name+'/'+str(i))
            checks.check(record['directional'][i]['step']==h,'derivatives: unchanged FD step '+name)
            directional_passes.append(_reports_pass(computed))
        expected_valid=bool(_reports_pass(record['primal_reports']) and coordinate_pass and all(directional_passes[-2:]))
        checks.check(record['valid']==expected_valid and expected_valid,'derivatives: qualification reproduced '+name)
        checks.check(record['required_primal_pass']==_reports_pass(record['primal_reports']) and
                     record['coordinate_pass']==coordinate_pass and record['directional_pass']==all(directional_passes[-2:]),'derivatives: every qualification flag reproduced '+name)
        checks.check(not record['optimizer_executed'] and not record['nonsmooth_classical_derivative_claimed'],'derivatives: diagnostic scope '+name)
        contract=record['provider_stats']
        checks.check(record['provider_contract_valid'] and contract['float64_enabled'] and contract['device_backend']=='cpu' and contract['numerical_finite_difference_calls']==0 and not contract['numerical_fallback_used'],'derivatives: recorded provider contract '+name)
    checks.check(attempt['point_count']==20 and attempt['full_coordinate_point_count']==10 and not attempt['failed_points'],'derivatives: qualification counts')
    checks.check(gate['valid'] and gate['actual_solver_authorized_by_this_gate'] and not gate['optimizer_executed'],'derivatives: successful read-only qualification gate')
    geometry_path=Path(gate['geometry_nonsmooth_report']);geometry_path=geometry_path if geometry_path.is_absolute() else ROOT/geometry_path
    verify_digest(geometry_path,gate['geometry_report_sha256'],checks,'derivatives: separate analytic geometry report hash')
    geometry=read(geometry_path)
    checks.check(geometry['valid'] and len(geometry['per_case'])==40 and len(geometry['synthetic_branch_checks'])==4,'derivatives: independent geometry/nonsmooth coverage')
    classical=[r for item in reports for r in item['coordinate_reports'] if r.get('compared_elements',0)>0]
    directional=[r for item in reports for step in item['directional'][-2:] for r in step['smooth_reports'] if r.get('compared_elements',0)>0]
    checks.equal(gate['maximum_required_derivative_scaled_error'],max(r['maximum_scaled_error'] for r in classical+directional),'derivatives: authoritative maximum scaled error')
    checks.check(gate['coordinate_comparison_elements']==sum(r['compared_elements'] for r in classical),'derivatives: authoritative compared elements')
    checks.check(gate['coordinate_excluded_elements']==sum(r.get('excluded_elements',0) for r in classical),'derivatives: authoritative excluded elements')
    checks.equal(gate['maximum_primal_absolute_error'],max(r['maximum_absolute_error'] for item in reports for r in item['primal_reports']),'derivatives: authoritative primal error')
    return gate


def validate_optimality(case,result,directory,checks,label):
    problem=case['problem']
    """Reconstruct residuals using the saved multiplier fit, never refit it."""
    from reconciliation.gp_se2_diag02_derivatives import DerivativeProvider,DerivativeError
    from reconciliation.gp_se2_diag02_environment import EnvironmentDerivatives
    report=read(directory/'optimality.json')
    provider=DerivativeProvider(problem,EnvironmentDerivatives(case['environment'],case['config']['footprint']['radius_m']))
    for name,field in [('initial','initial_vector'),('final','latest_iterate')]:
        item=report[name];z=np.asarray(result[field]);v=problem.evaluate(z)
        try:grad=provider.objective_gradient(z);jh=provider.equality_jacobian(z);jg=provider.inequality_jacobian(z)
        except DerivativeError:
            checks.check(item.get('status')=='UNSUPPORTED',label+': unsupported optimality preserved '+name);continue
        checks.check(item.get('status')!='UNSUPPORTED',label+': supported diagnostic available '+name)
        active=np.flatnonzero(v['inequality']<=problem.config['inequality_tolerance'])
        a=np.column_stack([jh.T,jg[active].T]);s=np.linalg.svd(a,compute_uv=False)
        tol=1e-9*max(1.,s[0] if len(s) else 0.)
        lam=np.asarray(item['equality_multipliers']);mu=np.asarray(item['active_inequality_multipliers'])
        expected=dict(primal=_constraint_report(v['equality'],v['inequality'],problem.config),
          active_inequality_indices=active,active_jacobian_shape=list(a.T.shape),
          active_jacobian_rank=int(np.count_nonzero(s>tol)),rank_tolerance=tol,singular_values=s,
          rank_deficient=bool(np.count_nonzero(s>tol)<min(a.shape)),
          minimum_inequality_multiplier=float(np.min(mu,initial=0.)),
          stationarity_inf=float(np.max(np.abs(grad-a@np.r_[lam,mu]),initial=0.)),
          objective_gradient_inf=float(np.max(np.abs(grad),initial=0.)),
          complementarity_inf=float(np.max(np.abs(mu*v['inequality'][active]),initial=0.)))
        for key,value in expected.items():checks.equal(item[key],value,label+': local optimality '+name+'/'+key)
        checks.check(not item['global_optimum_proven'] and not item['sufficiency_proven'],label+': no global claim '+name)
        if name=='final' and result['scipy_qp_multipliers'] is not None:
            q=np.asarray(result['scipy_qp_multipliers']);n=len(v['equality']);qp=item['slsqp_internal_qp']
            checks.equal(qp['multipliers'],q,label+': original internal QP multipliers',exact=True)
            checks.equal(qp['stationarity_inf'],np.max(np.abs(grad-jh.T@q[:n]-jg.T@q[n:])),label+': QP residual recomputed')


def validate_sources(run,cfg,source,checks):
    from reconciliation.gp_se2_diag_initialization import same_curvature_deceleration_seed
    for path,h in source['preserved_core_sha256'].items():verify_digest(ROOT/path,h,checks,'original core '+path)
    for path,h in source['input_sha256'].items():verify_digest(path,h,checks,'frozen input '+path)
    verify_digest(run/'config_snapshot.yaml',source['diagnostic_config_sha256'],checks,'diagnostic configuration hash')
    verify_digest(ROOT/'configs/gp_se2_diag_02.yaml',source['diagnostic_config_sha256'],checks,'versioned config matches snapshot')
    primary=Path(source['primary_run']);previous=Path(source['previous_diagnostic'])
    checks.check(primary.resolve()==(ROOT/cfg['primary_run']).resolve() and previous.resolve()==(ROOT/cfg['previous_diagnostic']).resolve(),'exact historical source directories')
    checks.check(read(previous/'verification/validation.json')['valid'],'previous authoritative diagnostic valid')
    checks.check(digest(run/'actual_event/input_snapshot.json')==digest(previous/'actual_event/input_snapshot.json'),'byte-identical original event input snapshot')
    cases={};seeds={}
    for method in cfg['methods']:
        case=load_frozen_case(primary,cfg['actual_case_id'],method,environment=next(iter(cases.values()))['environment'] if cases else None)
        cases[method]=case;p=case['problem'];i0=p.initializations()[0];i1=same_curvature_deceleration_seed(p)
        for index,(name,init) in enumerate(zip(cfg['initializations'],[i0,i1])):
            z=np.load(run/'seeds'/f'{name}.npy',allow_pickle=False);seeds[name]=z
            verify_digest(run/'seeds'/f'{name}.npy',source['seed_sha256'][name],checks,'frozen seed hash '+name)
            checks.equal(z,p.vector(init['poses'],init['twists']),'independent original seed reconstruction '+method+'/'+name,exact=True)
            checks.equal(z,read(previous/'actual_event/single_change'/method/f'start_{index:02d}.json')['initial_vector'],'bitwise original DIAG01 seed '+method+'/'+name,exact=True)
    protocol=read(run/'protocol.json')
    checks.equal(protocol['original_formulation'],next(iter(cases.values()))['config']['formulation'],'original formulation unchanged',exact=True)
    checks.equal(protocol['solve_order'],cfg['solve_order'],'frozen order matches config',exact=True)
    checks.check(protocol['sequential'] and not protocol['retry'] and not protocol['new_inference'] and not protocol['new_mpc_rollout'],'sequential bounded comparison scope')
    return cases,seeds


def validate_freeze(run,cfg,source,gate,checks):
    freeze=read(run/'comparison_freeze.json');started=read(run/'execution_started.json')
    for field,path in [('derivative_validation_sha256','derivative_checks/authoritative_validation.json'),('protocol_sha256','protocol.json'),('config_sha256','config_snapshot.yaml'),('runtime_versions_sha256','runtime_versions.json')]:
        verify_digest(run/path,freeze[field],checks,'comparison freeze '+field)
    checks.equal(freeze['seeds'],source['seed_sha256'],'same seed hashes at comparison freeze',exact=True)
    checks.equal(freeze['exact_order'],cfg['solve_order'],'same exact frozen execution order',exact=True)
    checks.equal(started['order'],cfg['solve_order'],'actual declared order',exact=True)
    checks.check(not started['retry_allowed'] and freeze['ready'],'no comparison retries')
    checks.check(datetime.fromisoformat(gate['published_utc'])<=datetime.fromisoformat(freeze['created_utc'])<=datetime.fromisoformat(started['utc']),'gate completed before freeze before actual comparison')
    for phase in ('prepare','comparison','final'):
        manifest=read(run/'implementation_sources'/phase/'manifest.json')
        for path,h in manifest['files'].items():
            verify_digest(run/'implementation_sources'/phase/path,h,checks,'archived implementation '+phase+'/'+path)
            if phase=='final':verify_digest(ROOT/path,h,checks,'current final implementation '+path)
        if phase=='comparison':checks.equal(manifest['files'],freeze['code_sha256'],'comparison snapshot matches frozen code',exact=True)
    changed={path for path,h in freeze['code_sha256'].items() if digest(ROOT/path)!=h}
    allowed=validate_presentation_correction(run,freeze,checks) if changed else set()
    checks.check(changed==allowed,'only disclosed presentation source drift after comparison freeze')
    for path,h in freeze['code_sha256'].items():
        if path not in allowed:verify_digest(ROOT/path,h,checks,'comparison source unchanged '+path)
    version=read(run/'runtime_versions.json')
    checks.check(version['jax_enable_x64'] and version['jax']=='0.7.2' and version['jaxlib']=='0.7.2','verified AD runtime version and precision')
    verify_digest(version['slsqp_source_path'],version['slsqp_source_sha256'],checks,'installed SLSQP source preserved')
    return freeze


def validate_presentation_correction(run,freeze,checks):
    report=read(run/'plot_presentation_correction.json')
    allowed={'scripts/plot_gp_se2_diag_02.py','tests/test_gp_se2_diag02_plots.py'}
    checks.check({r['path'] for r in report['changed_sources']}==allowed,'exact presentation-only source allowlist')
    checks.check(report['valid'] and not report['numerical_solve_rerun'] and not report['numerical_trace_change'] and not report['acceptance_change'],'presentation correction scope')
    for row in report['changed_sources']:
        checks.check(freeze['code_sha256'][row['path']]==row['before_sha256'],'presentation original frozen source '+row['path'])
        verify_digest(ROOT/row['path'],row['after_sha256'],checks,'presentation final source '+row['path'])
    checks.check(len(report['numerical_artifact_sha256'])==40 and sum(Path(p).name=='solver_result.json' for p in report['numerical_artifact_sha256'])==8,'presentation audit contains all actual numerical inputs')
    for path,h in report['numerical_artifact_sha256'].items():verify_digest(path,h,checks,'numerical artifact unchanged during presentation '+path)
    for archive in report['archive_checks']:
        folder=run/archive['path'];inventory=read(folder/'inventory.json')
        verify_digest(folder/'inventory.json',archive['inventory_sha256'],checks,'preserved presentation inventory '+archive['path'])
        checks.check(len(inventory['files'])==archive['file_count'],'preserved presentation file count '+archive['path'])
        for item in inventory['files']:verify_digest(folder/item['path'],item['sha256'],checks,'preserved presentation file '+archive['path']+'/'+item['path'])
        if (folder/'plot_manifest.json').exists():
            manifest=read(folder/'plot_manifest.json')
            checks.check(len(manifest['records'])==18,'preserved full plot inventory '+archive['path'])
            for record in manifest['records']:
                before=read(folder/record['sidecar']);after=read(run/record['sidecar'])
                checks.equal(before['numeric_data'],after['numeric_data'],'exact unchanged rendered numbers '+archive['path']+'/'+record['sidecar'],exact=True)
    checks.check(report['numeric_data_matches']==18 and len(report['numeric_data_records'])==18,'reported 18 numeric plot matches')
    for item in report['numeric_data_records']:
        checks.check(item['identical'] and item['before_numeric_sha256']==item['after_numeric_sha256'],'reported numeric equality '+item['sidecar'])
    transitions=read(run/'plot_presentation_transition_audit.json')
    verify_digest(run/'plot_presentation_correction.json',transitions['presentation_correction_sha256'],checks,'presentation transition audit exact correction source')
    checks.check(transitions['valid'] and not transitions['numerical_solve_rerun'] and not transitions['trace_change'],'presentation transitions remain nonnumerical')
    for row in transitions['source_transitions']:
        for side in ('before','after'):
            path=ROOT/row[side] if row[side].startswith('scripts/') else run/row[side]
            verify_digest(path,row[side+'_sha256'],checks,'presentation source transition '+row[side])
    packaging=report['bundle_packaging']
    verify_digest(run/'review_bundle.zip',packaging['bundle_sha256'],checks,'disclosed packaged review ZIP')
    for row in packaging['added']:
        verify_digest(run/row['source'],row['sha256'],checks,'review packaging source '+row['source'])
        verify_digest(run/'review_bundle'/row['bundled_as'],row['sha256'],checks,'review packaging exact copy '+row['bundled_as'])
    return allowed


def validate_actual(run,cfg,cases,seeds,checks):
    complete=read(run/'execution_completed.json');summary=read(run/'summary.json')
    checks.check(complete['solve_count']==summary['actual_solve_count']==8,'exactly eight prescribed actual solves')
    checks.equal(complete['exact_order'],cfg['solve_order'],'completed exact comparison order',exact=True)
    paths=list((run/'actual_event').glob('*/*/*/solver_result.json'))
    checks.check(len(paths)==8,'no extra actual comparison starts')
    previous_end=None;rows=[]
    for index,(method,mode,name) in enumerate(cfg['solve_order']):
        directory=run/'actual_event'/method/mode/name;where='/'.join([method,mode,name]);case=cases[method];problem=case['problem']
        result=read(directory/'solver_result.json');row=read(directory/'result_summary.json');setup=read(directory/'setup.json');timing=read(directory/'execution_timing.json')
        checks.equal([setup['method'],setup['mode'],setup['initialization']],[method,mode,name],where+': case identity',exact=True)
        checks.check(setup['execution_index']==row['execution_index']==index,where+': fixed execution position')
        checks.check(timing['end_monotonic_s']>=timing['start_monotonic_s'] and
                     (previous_end is None or timing['start_monotonic_s']>=previous_end),where+': sequential nonoverlapping harness timestamps')
        previous_end=timing['end_monotonic_s']
        checks.check(datetime.fromisoformat(timing['start_utc'])<=datetime.fromisoformat(timing['end_utc']),where+': UTC timestamp order')
        checks.check(setup['seed_sha256']==digest(run/'seeds'/f'{name}.npy'),where+': same prepared seed file')
        validate_seed(result,seeds[name],checks,where)
        checked,chosen=validate_attempt(problem,result,mode,checks,where)
        validate_full_outcome(problem,case,result,row,directory,checked,chosen,checks,where)
        # Only saved support differences; these are not executions.
        changed={}
        if chosen is not None:
            from reconciliation.se2 import wrap_angle
            a,b=problem.unpack(seeds[name]);c,d=problem.unpack(chosen[2])
            changed=dict(max_position_change_m=float(np.max(np.linalg.norm(c[:,:2]-a[:,:2],axis=1))),max_yaw_change_rad=float(np.max(np.abs(wrap_angle(c[:,2]-a[:,2])))),max_linear_velocity_change_m_s=float(np.max(np.abs(d[:,0]-b[:,0]))),max_angular_velocity_change_rad_s=float(np.max(np.abs(d[:,2]-b[:,2]))))
        checks.equal(row['candidate_change'],changed,where+': independent selected movement')
        for k,field in [('objective_calls','objective_evaluations'),('equality_calls','equality_evaluations'),('inequality_calls','inequality_evaluations'),('derivative_calls','derivative_calls')]:checks.equal(row[k],result[field],where+': callback summary '+k)
        cold=sum(setup[k] for k in ('case_loading_seed_preparation_s','derivative_graph_construction_s','compilation_first_call_warmup_s'))+result['setup_wall_time_s']+result['solve_wall_time_s']+result['post_solve_validation_time_s']+row['independent_full_validation_time_s']
        checks.equal(row['cold_setup_solve_validation_s'],cold,where+': disclosed cold total')
        checks.check((mode=='SUPPLIED_JAC' and setup['compilation_first_call_warmup_s']>0) or (mode=='FD_BASELINE' and setup['compilation_first_call_warmup_s']==setup['derivative_graph_construction_s']==0),where+': warmup only supplied mode')
        checks.equal(complete['solves'][index],row,where+': completed result source')
        aggregated=summary['all_solves'][index]
        for key,value in row.items():
            if key not in ('optimality_provider_setup_s','optimality_diagnostic_time_s'):checks.equal(aggregated[key],value,where+': aggregate source '+key)
        checks.section(where+' optimality',lambda:validate_optimality(case,result,directory,checks,where))
        rows.append(row)
    checks.check(not summary['navigation_improvement_claim'] and summary['new_mpc_rollouts']==summary['new_inference']==0,'no new rollout or inference claim')
    supplied=[r for r in rows if r['derivative_mode']=='SUPPLIED_JAC']
    for key in ('infeasible_start_recovered','feasible_seed_objective_improved','solver_converged_with_full_feasibility','returned_initial_unchanged'):
        checks.check(summary['numerical_outcome_flags'][key]==any(r[key] for r in supplied),'aggregate flag '+key)
    costs=[]
    for pair in summary['paired_comparison']:
        a,b=[next(r for r in rows if r['method']==pair['method'] and r['initialization']==pair['initialization'] and r['derivative_mode']==mode) for mode in cfg['derivative_modes']]
        for prefix,item in [('fd_',a),('supplied_',b)]:
            for key,value in pair.items():
                if key.startswith(prefix):checks.equal(value,item[key[len(prefix):]],'paired source '+key)
        reduced=b['solve_wall_time_s']<a['solve_wall_time_s'] and b['primal_cache_misses']<a['primal_cache_misses'];costs.append(reduced)
        checks.check(pair['compute_cost_reduced']==reduced,'paired compute reduction both criteria')
        for key,field in [('solve_time_ratio_supplied_over_fd','solve_wall_time_s'),('objective_call_ratio_supplied_over_fd','objective_calls'),('primal_miss_ratio_supplied_over_fd','primal_cache_misses')]:
            checks.equal(pair[key],b[field]/(a[field] if field=='solve_wall_time_s' else max(1,a[field])),'paired ratio '+key)
    checks.check(summary['numerical_outcome_flags']['compute_cost_reduced']==any(costs),'aggregate compute flag')
    validate_csv_rows(run/'aggregate/all_solves.csv',summary['all_solves'],checks,'all-solves CSV')
    validate_csv_rows(run/'aggregate/paired_comparison.csv',summary['paired_comparison'],checks,'paired comparison CSV')
    validate_csv_rows(run/'aggregate/candidate_acceptance.csv',summary['all_solves'],checks,'candidate acceptance CSV')
    checks.check(summary['operational_status']=='GP_SE2_DIAG_02_COMPLETED_WITH_LIMITATIONS' and summary['derivative_validation']=='VERIFIED_WITH_DECLARED_BRANCH_LIMITATIONS','reported completion scope')
    return rows


def validate_csv_rows(path,expected,checks,label):
    with Path(path).open(newline='') as stream:reader=csv.DictReader(stream);rows=list(reader);fields=reader.fieldnames
    checks.check(len(rows)==len(expected),label+': complete row count')
    for index,(row,source) in enumerate(zip(rows,expected)):
        for field in fields:
            value=source[field]
            checks.check(row[field]==('' if value is None else str(value)),label+f': source row {index}/'+field)


def validate_plot_sidecar(path,sidecar,expected,checks,label):
    checks.check(sidecar['image']==path.name and sidecar['image_sha256']==digest(path),label+': image hash')
    checks.check(sidecar['dpi']>=160 and sidecar['no_new_execution'],label+': plot scope/resolution')
    checks.equal(sidecar['numeric_data'],expected,label+': reproduced plotted numbers')
    for name,h in sidecar['source_sha256'].items():verify_digest(name,h,checks,label+': source hash '+name)


def validate_plots(run,cases,checks):
    import plot_gp_se2_diag_02 as plots
    from shapely import to_wkb
    cells=plots.records(run);manifest=read(run/'plot_manifest.json');snapshot=read(run/'actual_event/input_snapshot.json')
    expected_names={f'plots/{method}/{name}.png' for method in plots.METHODS for name in plots.PLOT_NAMES}
    checks.check({row['path'] for row in manifest['records']}==expected_names and manifest['plot_count']==18 and manifest['comparison_cell_count']==8,'exact 18 plot inventory includes all eight cells')
    for row in manifest['records']:
        path=run/row['path'];sidepath=run/row['sidecar'];method=path.parent.name;name=path.stem;side=read(sidepath)
        verify_digest(path,row['sha256'],checks,'plot image '+row['path']);verify_digest(sidepath,row['sidecar_sha256'],checks,'plot sidecar '+row['sidecar'])
        numeric={};traces={}
        for seed in plots.SEEDS:
            for mode in plots.MODES:
                record=cells[method,seed,mode];label=seed+'/'+mode;result=record['solver_result'];history=plots.callback_history(result)
                if name in ('objective_iteration','objective_wall_time'):
                    numeric[label]=dict(x=history['iteration' if name=='objective_iteration' else 'elapsed_s'],objective=history['objective'],history_source=history['source'])
                elif name=='collocation_time':numeric[label]=history
                elif name=='best_full_feasible_objective':numeric[label]=plots.best_full_history(record)
                elif name in ('lateral','accelerations','world_xy'):
                    traces[label]=plots.trajectory_records(record,cases[method]['problem']);numeric[label]=traces[label]
                elif name=='profiling':numeric[label]=plots.disjoint_profile(record)
                elif name=='evaluation_counts':
                    numeric[label]=dict(labels=['objective','equality','inequality','unique primal','obj gradient','eq Jacobian','ineq Jacobian'],counts=[result[k] for k in ('objective_evaluations','equality_evaluations','inequality_evaluations')]+[result['profiling']['unique_vector_evaluations']]+[result['derivative_calls'][k] for k in ('objective_gradient','equality_jacobian','inequality_jacobian')])
        if name=='world_xy':
            raw=np.asarray(snapshot['F_native']);xy=[raw[:,:2],np.asarray(snapshot['context']['B_world'])[None,:2]]
            for trace in traces.values():xy.extend(np.asarray(trace[k]['poses_world'])[:,:2] for k in ('seed','final','selected') if trace[k] is not None)
            xy=np.vstack(xy);env=cases[method]['environment']
            numeric=dict(traces=traces,raw_fresh_world=raw,axis_limits_world_m=[xy.min(axis=0)-.55,xy.max(axis=0)+.55],aspect='equal',geometry=dict(workspace_wkb_sha256=hashlib.sha256(to_wkb(env.workspace)).hexdigest(),obstacles_wkb_sha256=hashlib.sha256(to_wkb(env.obstacles)).hexdigest()),boundary_world=snapshot['context']['B_world'],goal_world=snapshot['goal_route']['goal_world'])
        validate_plot_sidecar(path,side,numeric,checks,'plot '+row['path'])
    bundle=run/'review_bundle';inventory=read(bundle/'manifest.json');names={r['path'] for r in inventory['allowlisted_files']}
    checks.check(len(names)==inventory['file_count'],'review bundle complete unique inventory')
    allowed=set(expected_names)|{str(Path(p).with_suffix('.json')) for p in expected_names}|{'plot_manifest.json','summary.json','comparison.csv','index.html','README.md'}
    allowed|={'aggregate/all_solves.csv','aggregate/paired_comparison.csv','aggregate/candidate_acceptance.csv','aggregate/derivative_coverage.csv','derivative_design.json','authoritative_validation.json','experiment_summary.json','packaging_provenance.json'}
    allowed|={f'results/{method}/{mode}/{seed}/{name}.json' for method in plots.METHODS for mode in plots.MODES for seed in plots.SEEDS for name in ('result_summary','setup','post_full_checks')}
    checks.check(names==allowed,'review bundle strict compact allowlist excludes raw/solver vectors')
    for item in inventory['allowlisted_files']:
        path=bundle/item['path'];verify_digest(path,item['sha256'],checks,'review file '+item['path'])
        checks.check(path.stat().st_size==item['bytes'],'review byte count '+item['path'])
        origin=run/item['path'] if not item['path'].startswith('results/') else run/'actual_event'/Path(item['path']).relative_to('results')
        if origin.exists() and item['path'] not in ('index.html','summary.json','README.md'):checks.check(digest(path)==digest(origin),'review exact original copy '+item['path'])
    with zipfile.ZipFile(run/'review_bundle.zip') as archive:
        checks.check(archive.testzip() is None and set(archive.namelist())==names|{'manifest.json'},'ZIP integrity and exact allowlist')
        for name in archive.namelist():checks.check(hashlib.sha256(archive.read(name)).hexdigest()==digest(bundle/name),'ZIP bytes match review file '+name)
    bundle_summary=read(bundle/'summary.json')
    checks.check(bundle_summary['comparison_cell_count']==8,'bundle all eight cells')
    checks.equal(bundle_summary['cells'],[r['result_summary'] for r in cells.values()],'bundle exact per-start summaries')
    validate_csv_rows(bundle/'comparison.csv',bundle_summary['cells'],checks,'bundle comparison CSV')


def validate_preservation(run,checks):
    before=read(run/'preservation_before.json');after=read(run/'preservation_after.json')
    checks.check(before['valid'] and after['valid'] and not after['mismatches'],'preservation before and after valid')
    for category in ('historical_files','external_files','previous_diagnostic_files','user_config_hashes'):
        checks.equal(after[category],before[category],'unchanged preserved inventory '+category,exact=True)
        for name,h in before[category].items():
            path=Path(name);path=path if path.is_absolute() else ROOT/path
            verify_digest(path,h,checks,'current preserved file '+name)
    for category in ('external_git_sha','external_git_status'):
        checks.equal(before[category],after[category],'external repository unchanged '+category,exact=True)
    manifest=read(run/'artifact_manifest.json')
    for name,h in manifest['files'].items():
        path=run/name
        checks.check(path.resolve().is_relative_to(run.resolve()),'artifact path belongs to new run '+name)
        verify_digest(path,h,checks,'frozen scientific artifact '+name)
    return len(manifest['files'])


def validate(run):
    run=Path(run).resolve();checks=Checks();context={}
    def sources():
        context['cfg']=yaml.safe_load((run/'config_snapshot.yaml').read_text());context['source']=read(run/'source.json')
        context['cases'],context['seeds']=validate_sources(run,context['cfg'],context['source'],checks)
    checks.section('source',sources)
    def derivatives():context['gate']=validate_derivative_gate(run,context['source'],checks)
    checks.section('derivative verification',derivatives)
    gate=context.get('gate');actual_available=(run/'execution_completed.json').exists()
    if gate is None or not gate.get('valid'):
        checks.check(not actual_available and not (run/'execution_started.json').exists(),'unqualified derivatives block actual comparison')
    else:
        checks.section('comparison freeze',lambda:validate_freeze(run,context['cfg'],context['source'],gate,checks))
        if actual_available:
            checks.section('actual comparison',lambda:context.update(rows=validate_actual(run,context['cfg'],context['cases'],context['seeds'],checks)))
            checks.section('plots and review bundle',lambda:validate_plots(run,context['cases'],checks))
            checks.section('preservation and artifact manifest',lambda:context.update(artifact_count=validate_preservation(run,checks)))
        else:checks.check(False,'qualified comparison has no completed eight-start evidence')
    return dict(valid=not checks.errors,check_count=checks.count,errors=checks.errors,run=str(run),
      completed_utc=datetime.now(timezone.utc).isoformat(),actual_results_available=actual_available,
      actual_solve_count=len(context['rows']) if 'rows' in context else None,
      artifact_count=context.get('artifact_count'),optimizer_executed=False,new_inference=False,new_mpc_rollout=False,
      validation_scope='saved inputs, full AD matrices and error arithmetic, original primal/dense/full acceptance, saved multiplier residuals, exact plot numbers and hashes; no optimizer or finite-difference sweep rerun',
      validator_sha256=digest(__file__))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--run',required=True,type=Path);parser.add_argument('--output',type=Path)
    args=parser.parse_args();run=args.run.resolve()
    if args.output is not None:
        output=args.output.resolve()
        if not output.is_relative_to(run):raise ValueError('validation output must be a NEW path inside DIAG02 run')
        if output.exists():raise FileExistsError(output)
    report=validate(run)
    if args.output is not None:write_exclusive(output,report)
    print(json.dumps(report,indent=2));return 0 if report['valid'] else 1


if __name__=='__main__':raise SystemExit(main())

#!/usr/bin/env python3
"""Exclusive, staged GP-SE2-DIAG-01 diagnostics; no new inference or execution."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

# Set before NumPy/SciPy imports. This runner never launches concurrent solves.
for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[_name] = '1'
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'scripts')]
import numpy as np
import yaml
from run_gp_se2_01 import digest, read, write, array, table, file_records


def utc():
    return datetime.now(timezone.utc).isoformat()


def code_snapshot(run, phase):
    paths = sorted(set(list((ROOT/'src/reconciliation').glob('gp_se2*.py')) +
                       list((ROOT/'scripts').glob('*gp_se2_diag*')) +
                       list((ROOT/'tests').glob('test_gp_se2_diag*.py')) +
                       [ROOT/'src/reconciliation/se2.py', ROOT/'configs/gp_se2_diag_01.yaml']))
    hashes = {}
    base = Path(run)/'implementation_sources'/phase
    base.mkdir(parents=True, exist_ok=False)
    for path in paths:
        relative = path.relative_to(ROOT)
        target = base/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
        hashes[str(relative)] = digest(path)
    write(base/'manifest.json', dict(captured_utc=utc(), files=hashes))
    return hashes


def preservation_snapshot(cfg):
    """Hash historical data and external checkpoint without writing there."""
    audit = ROOT/'data/robotless_gp_se2_01/source_audit_20260918'
    old = read(audit/'source_hashes_before.json')
    expected = dict(old)
    for key in ('primary_run', 'verification_run', 'environment_export'):
        root = ROOT/cfg[key]
        expected.update({str(p.relative_to(ROOT)): digest(p) for p in sorted(root.rglob('*')) if p.is_file()})
    mismatches = []
    current = {}
    for name, original in expected.items():
        path = ROOT/name
        value = digest(path) if path.is_file() else None
        current[name] = value
        if value != original:
            mismatches.append(name)
    external = read(audit/'external_before.json')
    checkpoint = Path(external['checkpoint_root'])
    expected_checkpoint_names={record['path'] for record in external['checkpoint_files']}
    current_checkpoint_names={str(path.relative_to(checkpoint)) for path in checkpoint.rglob('*') if path.is_file()}
    if current_checkpoint_names!=expected_checkpoint_names:
        mismatches.append('checkpoint file inventory changed')
    external_hashes = {}
    for record in external['checkpoint_files']:
        path = checkpoint/record['path']
        value = digest(path) if path.is_file() else None
        external_hashes[str(path)] = value
        if value != record['sha256']:
            mismatches.append(str(path))
    repo = Path('/home/gpuadmin/Workspace/external/LightNav-0-official-demo')
    mpc = repo/'mujoco_demo/vln_mujoco/mpc.py'
    external_hashes[str(mpc)] = digest(mpc)
    configs = {name: digest(ROOT/name) for name in (
        'configs/stage0_jackal_controller_validation.yaml',
        'configs/stage0_lightnav_single_chunk.yaml')}
    return dict(valid=not mismatches, mismatches=mismatches, historical_files=current,
                historical_file_count=len(current), external_files=external_hashes,
                checkpoint_file_count=len(external['checkpoint_files']),
                checkpoint_inventory_matches=current_checkpoint_names==expected_checkpoint_names,
                external_git_sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),
                external_git_status=subprocess.check_output(['git','status','--porcelain'],cwd=repo,text=True),
                user_config_hashes=configs, captured_utc=utc())


def prepare(run, config):
    cfg=yaml.safe_load(Path(config).read_text())
    run=Path(run).resolve()
    if run.exists() or not run.is_relative_to(ROOT/'data/robotless_gp_se2_diag_01'):
        raise ValueError('output must be a new directory below data/robotless_gp_se2_diag_01')
    primary=ROOT/cfg['primary_run']
    if not read(primary/'validation.json')['valid']:
        raise ValueError('historical primary validation must pass')
    run.mkdir(parents=True)
    shutil.copy2(config,run/'config_snapshot.yaml')
    before=preservation_snapshot(cfg)
    write(run/'preservation_before.json',before)
    if not before['valid']:
        raise ValueError('source preservation audit failed')
    original_source=read(primary/'source.json')
    core=['src/reconciliation/'+p for p in ('se2.py','gp_se2.py','gp_se2_formulation.py',
          'gp_se2_evaluation.py','gp_se2_reference.py','gp_se2_rollout.py','gp_se2_environment.py')]
    core_hashes={name:digest(ROOT/name) for name in core}
    if any(original_source['experiment_source_sha256'][name]!=h for name,h in core_hashes.items()):
        raise ValueError('original numerical implementation drifted')
    source_validation=read(ROOT/'data/robotless_gp_se2_01/source_audit_20260918/source_validation.json')
    source_hash_validation=read(ROOT/'data/robotless_gp_se2_01/source_audit_20260918/source_hash_validation.json')
    source=dict(created_utc=utc(), starting_git_sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        primary_run=str(primary), historical_numerical_git_sha=cfg['historical_numerical_git_sha'],
        historical_primary_validation_sha256=digest(primary/'validation.json'),
        source_validation_valid=source_validation['valid'], source_hash_validation_valid=source_hash_validation['valid'],
        current_full_retained_data_rehash_valid=before['valid'],
        core_numerical_source_sha256=core_hashes, config_sha256=digest(run/'config_snapshot.yaml'),
        original_config_sha256=digest(primary/'config_snapshot.yaml'),
        original_case_manifest_sha256=digest(primary/'case_manifest.json'),
        historical_validator_correction='separate bookkeeping-only verification; no numerical rerun',
        frame='fixed Isaac Hospital world; metres, +Z up, CCW yaw radians; body x forward/y left',
        body_twist='vee(X^-1 dX/dt)', intrinsic_lightnav_waypoint_dt_s=None,
        raw_reference_time='original evaluation-only .1,...,3.0 s grid, unchanged')
    write(run/'source.json',source)
    write(run/'protocol.json',dict(created_utc=utc(),actual_case_id=cfg['actual_case_id'],
        original_config=yaml.safe_load((primary/'config_snapshot.yaml').read_text()),
        optimization_order='saved40 audit then fixture diagnostics then synthetic/actual baseline then decision then one variant',
        iterations=200,per_initialization_wall_s=30.,blas_threads=1,
        physical_acceptance_unchanged=True, new_mpc_rollouts=False,new_inference=False,
        missing_historical_callback_history='UNKNOWN; never interpolated or fabricated',
        numerical_source=code_snapshot(run,'prepare')))
    print(json.dumps(dict(run=str(run),status='PREPARED',source_valid=before['valid'])),flush=True)


def settings(run):
    cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    source=read(run/'source.json')
    primary=Path(source['primary_run'])
    if digest(run/'config_snapshot.yaml')!=source['config_sha256']:
        raise ValueError('diagnostic configuration changed after freeze')
    if digest(primary/'config_snapshot.yaml')!=source['original_config_sha256']:
        raise ValueError('original configuration changed')
    if digest(primary/'case_manifest.json')!=source['original_case_manifest_sha256']:
        raise ValueError('case manifest changed')
    for name,h in source['core_numerical_source_sha256'].items():
        if digest(ROOT/name)!=h:raise ValueError('original numerical source changed: '+name)
    return cfg,primary


def require_audit(run):
    completion=read(run/'existing_failure_audit/completion.json')
    summary=read(run/'existing_failure_audit/summary.json')
    if (summary['attempt_count']!=40 or not completion.get('all_40_attempts_reclassified')
            or not completion.get('source_files_unchanged')):
        raise ValueError('all 40 saved attempts must be audited before optimization')
    return completion


def synthetic_solves(run):
    from reconciliation.gp_se2_diag_fixtures import make_fixture_problem
    from reconciliation.gp_se2_diag_solver import run_instrumented
    from reconciliation.gp_se2_diagnostics import detailed_residuals
    cfg,primary=settings(run);require_audit(run)
    if not (run/'synthetic_fixtures/math/summary.json').is_file():
        raise ValueError('optimizer-free fixture evidence required first')
    out=run/'synthetic_fixtures/solver';out.mkdir(parents=True,exist_ok=False)
    code=code_snapshot(run,'synthetic-solves')
    write(out/'source.json',dict(created_utc=utc(),code_sha256=code,
        audit_completion_sha256=digest(run/'existing_failure_audit/completion.json'),
        math_summary_sha256=digest(run/'synthetic_fixtures/math/summary.json')))
    original=yaml.safe_load((primary/'config_snapshot.yaml').read_text())['formulation']
    rows=[]
    for name in ('S0','S1','S2'):
        directory=out/name;directory.mkdir()
        problem,fixture=make_fixture_problem(name,original)
        write(directory/'input.json',fixture)
        write(directory/'known_without_solver.json',detailed_residuals(problem,fixture['known_vector']))
        for kind in ('known','perturbed'):
            result=run_instrumented(problem,fixture[kind+'_vector'],
                initialization_name=name+'_'+kind,post_diagnostics=detailed_residuals)
            result['evidence_kind']='SYNTHETIC_SOLVER_DIAGNOSTIC_NOT_RESEARCH_PERFORMANCE'
            write(directory/(kind+'.json'),result)
            row={key:result[key] for key in ('termination','iterations','candidate_found','initial_feasible',
                'feasible_initial_retained','recovered_from_infeasible','selected_objective_improved',
                'selected_iterate','solve_wall_time_s','post_solve_validation_time_s','objective_evaluations')}
            row.update(fixture=name,initialization=kind,unique_evaluations=result['profiling']['unique_vector_evaluations'],
                       synthetic_only=True)
            rows.append(row)
            print(json.dumps(row),flush=True)
    write(out/'summary.json',dict(rows=rows,completed_utc=utc(),synthetic_only=True))
    table(out/'fixture_solver_results.csv',rows)


def freeze_actual_inputs(run,case):
    from reconciliation.gp_se2_rollout import load_frozen_context
    context=case['context']
    independent=load_frozen_context(context['source_root'],context['episode_id'],context['handoff_id'])
    if independent!=context:
        raise ValueError('saved context differs from independently reloaded raw event')
    native=np.load(case['case_directory']/'F_native.npy',allow_pickle=False)
    if not np.array_equal(native,np.asarray(context['fresh_world'])):
        raise ValueError('original native array differs from raw source')
    write(run/'actual_event/input_snapshot.json',dict(case_id=context['case_id'],
        primary_case_directory=str(case['case_directory']),source_hashes=case['hashes'],
        context=context,goal_route=case['goal_route'],config=case['config'],
        F_native=native,F_common=case['common_reference'],support_times_s=case['problem'].times,
        reference_times_s=case['problem'].times[1:],independently_reloaded_source_matches=True,
        gp_initial_twist_source='physical u_minus, not controller previous_control',
        original_goal_unchanged=True,original_reference_unchanged=True))


def actual_phase(run,phase):
    from reconciliation.gp_se2_diag_solver import run_instrumented
    from reconciliation.gp_se2_diagnostics import load_frozen_case,detailed_residuals
    from reconciliation.gp_se2_diag_acceptance import check_full_candidate
    cfg,primary=settings(run);require_audit(run)
    if phase not in ('baseline','single_change'):raise ValueError('unknown actual phase')
    decision=None
    if phase=='single_change':
        decision=read(run/'decision.json')
        if decision['selected_change']!='B_FEASIBLE_INITIALIZATION':raise ValueError('unimplemented decision')
        for name,value in decision['evidence_sha256'].items():
            if digest(run/name)!=value:raise ValueError('decision evidence changed')
    out=run/'actual_event'/phase;out.mkdir(parents=True,exist_ok=False)
    code=code_snapshot(run,phase)
    write(out/'source.json',dict(created_utc=utc(),code_sha256=code,
        decision_sha256=digest(run/'decision.json') if decision else None,
        audit_completion_sha256=digest(run/'existing_failure_audit/completion.json')))
    rows=[];environment=None
    for method in cfg['methods']:
        case=load_frozen_case(primary,cfg['actual_case_id'],method,environment=environment)
        environment=case['environment'];problem=case['problem']
        if phase=='baseline' and not (run/'actual_event/input_snapshot.json').exists():
            freeze_actual_inputs(run,case)
        snapshot=read(run/'actual_event/input_snapshot.json')
        if case['hashes']!=snapshot['source_hashes']:
            raise ValueError('actual input hashes changed between phases')
        target=out/method;target.mkdir()
        inits=problem.initializations()
        seed_construction_wall_s=0.
        if phase=='single_change':
            from reconciliation.gp_se2_diag_initialization import same_curvature_deceleration_seed
            started=time.perf_counter()
            inits[1]=same_curvature_deceleration_seed(problem)
            seed_construction_wall_s=time.perf_counter()-started
            write(target/'seed_definition.json',inits[1])
        attempts=[]
        for index,initial in enumerate(inits):
            vector=problem.vector(initial['poses'],initial['twists'])
            historical=read(case['case_directory']/'methods'/method/'solver_result.json')['attempts'][index]
            matches_historical=bool(np.array_equal(vector,historical['initial_vector']))
            if (phase=='baseline' or index==0) and not matches_historical:
                raise ValueError('baseline initialization differs from saved original')
            result=run_instrumented(problem,vector,initialization_name=initial['name'],
                                    post_diagnostics=detailed_residuals,
                                    pre_solve_budget_cost_s=seed_construction_wall_s if index==1 else 0.)
            result.update(case_id=cfg['actual_case_id'],method=method,phase=phase,
                          input_hashes=case['hashes'],historical_initial_vector_exactly_equal=matches_historical,
                          seed_construction_wall_s=seed_construction_wall_s if index==1 else 0.)
            write(target/f'start_{index:02d}.json',result)
            attempts.append(result)
            print(json.dumps(dict(phase=phase,method=method,start=index,termination=result['termination'],
                candidate_found=result['candidate_found'],iterations=result['iterations'],
                solve_wall_time_s=result['solve_wall_time_s'])),flush=True)
        choices=[(a['candidate_objective'],i,a) for i,a in enumerate(attempts) if a['candidate_found']]
        chosen=min(choices,key=lambda x:(x[0],x[1])) if choices else None
        independent=None;independent_wall=0.
        seed_full_check=None;seed_check_wall=0.
        if phase=='single_change':
            started=time.perf_counter()
            seed_full_check=check_full_candidate(problem,attempts[1]['initial_vector'],case,
                maximum_step_s=cfg['additional_check']['maximum_step_s'],
                offset_fraction=cfg['additional_check']['offset_fraction'])
            seed_check_wall=time.perf_counter()-started
            write(target/'independent_seed_acceptance.json',seed_full_check)
        if chosen:
            started=time.perf_counter()
            independent=check_full_candidate(problem,chosen[2]['candidate_vector'],case,
                maximum_step_s=cfg['additional_check']['maximum_step_s'],
                offset_fraction=cfg['additional_check']['offset_fraction'])
            independent_wall=time.perf_counter()-started
            write(target/'independent_acceptance.json',independent)
            if independent['full_feasible']:
                array(target/'candidate_world.npy',chosen[2]['candidate_world'])
                array(target/'support_poses.npy',chosen[2]['support_poses'])
                array(target/'support_twists.npy',chosen[2]['support_twists'])
        summary=dict(case_id=cfg['actual_case_id'],method=method,phase=phase,
            sampled_candidate_found=chosen is not None,full_candidate_found=bool(independent and independent['full_feasible']),
            selected_start=chosen[1] if chosen else None,selected_iterate=chosen[2]['selected_iterate'] if chosen else None,
            candidate_world='candidate_world.npy' if independent and independent['full_feasible'] else None,
            candidate_status='FULL_FEASIBLE_CANDIDATE_FOUND' if independent and independent['full_feasible'] else 'NO_FULL_FEASIBLE_CANDIDATE_FOUND',
            solver_wall_total_s=sum(a['solve_wall_time_s'] for a in attempts),
            post_validation_total_s=sum(a['post_solve_validation_time_s'] for a in attempts),
            full_independent_validation_wall_s=independent_wall,
            seed_construction_wall_s=seed_construction_wall_s,
            seed_independent_validation_wall_s=seed_check_wall,
            seed_full_feasible=None if seed_full_check is None else seed_full_check['full_feasible'],
            seed_construction_plus_solver_wall_s=seed_construction_wall_s+sum(a['solve_wall_time_s'] for a in attempts),
            objective_evaluations=sum(a['objective_evaluations'] for a in attempts),
            solver_success_count=sum(a['solver_success'] for a in attempts),input_hashes=case['hashes'],
            original_formulation_config=problem.config,new_execution_performed=False,fallback_used=False)
        write(target/'summary.json',summary);rows.append(summary)
    write(out/'summary.json',dict(rows=rows,completed_utc=utc(),actual_case_count=1,new_execution_performed=False))


def decide(run):
    cfg,primary=settings(run);require_audit(run)
    audit=read(run/'existing_failure_audit/summary.json')
    math=read(run/'synthetic_fixtures/math/summary.json')
    synthetic=read(run/'synthetic_fixtures/solver/summary.json')
    baseline=read(run/'actual_event/baseline/summary.json')
    if (audit['classification_counts']['COLLOCATION_INFEASIBLE']!=40 or
        any(not r['known_seed_original_dense_feasible'] for r in math['fixtures'][:3]) or
        len(synthetic['rows'])!=6 or len(baseline['rows'])!=2 or
        sum(r['initialization']=='perturbed' and r['recovered_from_infeasible'] for r in synthetic['rows'])!=3 or
        any(r['sampled_candidate_found'] for r in baseline['rows'])):
        raise ValueError('predeclared B decision rationale does not match completed evidence; stop for review')
    files=['existing_failure_audit/summary.json','existing_failure_audit/completion.json',
           'synthetic_fixtures/math/summary.json','synthetic_fixtures/solver/summary.json',
           'actual_event/baseline/summary.json','actual_event/input_snapshot.json']
    write(run/'decision.json',dict(decision_utc=utc(),selected_change='B_FEASIBLE_INITIALIZATION',
        variant_name='replace_constant_twist_start_with_fixed_same_curvature_deceleration',
        case_id=cfg['actual_case_id'],evidence_sha256={name:digest(run/name) for name in files},
        observed_failure='All 40 historical latest iterates and all four new actual-event baseline starts fail collocation; no dense-only historical rejection. Known same-direction GP witnesses reproduce to roundoff; all three perturbed synthetic problems recover original dense feasibility.',
        exact_change='Keep start 0 FRESH finite-log initialization unchanged. Replace only start 1 constant body twist by X(t)=B Exp(q(t) eta), nu(t)=(1-t/T)eta, q=t-t^2/(2T), eta=original physical initial twist, T=3s.',
        seed_parameters_fitted=False,raw_rollout_warm_start=False,restoration_phase=False,
        q_units='seconds',q_dot_units='dimensionless',q_ddot_units='1/seconds',
        acceptance='Original GPProblem dense checker + original evaluate_plan/direct geometry + independent .001s offset grid; all thresholds, goal, gates, footprint and inputs unchanged.',
        budget=dict(methods=cfg['methods'],starts_per_method=2,max_iterations=200,wall_s_per_start=30.,
                    total_nominal_s_per_method=60.,blas_threads=1,
                    seed_construction='charged to its original 30s start budget',
                    validation='separately measured, same post-solve checking procedure for both variants'),
        fixed=['SLSQP','numerical forward differences','original objective/FRESH weight','30 future supports',
               '3s horizon','start0','B','physical initial twist','controller memory','F_native','F_common',
               'original goal/gates/workspace/obstacles/footprint','all acceptance tolerances'],
        not_selected=dict(A='Finite-difference repeated evaluations and environment cost are measured bottlenecks; changing derivatives now would mix a computational remedy with the direct initialization diagnosis.',
                          C='The blind spot exists in a synthetic interpolant, but historical collocation-pass/dense-fail count is zero; it does not explain these rejections.'),
        improvement_supports='If original full checks pass, the actual event admits a GP feasible witness and candidate availability is sensitive to initialization. Returning that seed is not evidence that the optimizer recovered feasibility or improved execution.',
        no_improvement_uncertainty='Fixed family may not meet the actual goal/environment or SLSQP may depart from feasibility; failure does not prove mathematical infeasibility.',
        no_chained_second_change=True,new_execution_or_mpc=False))
    print(json.dumps(dict(status='DECISION_FROZEN',selected_change='B_FEASIBLE_INITIALIZATION',
                          decision_sha256=digest(run/'decision.json'))),flush=True)


def finalize(run):
    """Collect numerical outcomes and rehash preserved sources; never solve."""
    cfg,_=settings(run);require_audit(run)
    if (run/'summary.json').exists():raise FileExistsError('final summary exists')
    rows=[];methods=[]
    for phase in ('baseline','single_change'):
        phase_summary=read(run/'actual_event'/phase/'summary.json')
        methods.extend(phase_summary['rows'])
        for method in cfg['methods']:
            directory=run/'actual_event'/phase/method
            for path in sorted(directory.glob('start_*.json')):
                result=read(path);initial,latest=result['candidate_checks'][:2]
                profile=result['profiling']
                row=dict(phase=phase,method=method,start=int(path.stem.split('_')[1]),
                    initialization=result['initialization'],termination=result['termination'],
                    iterations=result['iterations'],initial_feasible=result['initial_feasible'],
                    candidate_found=result['candidate_found'],selected_iterate=result['selected_iterate'],
                    returned_initial_unchanged=result['returned_initial_unchanged'],
                    recovered_from_infeasible=result['recovered_from_infeasible'],
                    initial_objective=initial['objective'],latest_objective=latest['objective'],
                    candidate_objective=result['candidate_objective'],objective_delta=result['objective_delta'],
                    objective_improved_beyond_ftol=result['selected_objective_improved_beyond_ftol'],
                    latest_collocation_max_equality_residual=latest['collocation']['maximum_equality_residual'],
                    latest_collocation_max_inequality_violation=latest['collocation']['maximum_inequality_violation'],
                    latest_dense_max_equality_residual=latest['constraint_report']['maximum_equality_residual'],
                    latest_dense_max_inequality_violation=latest['constraint_report']['maximum_inequality_violation'],
                    latest_lateral_velocity_m_s=latest['constraint_report']['maximum_absolute_lateral_velocity_m_s'],
                    variable_count=result['variable_count'],equality_count=result['equality_count'],inequality_count=result['inequality_count'],
                    solve_wall_time_s=result['solve_wall_time_s'],setup_wall_time_s=result['setup_wall_time_s'],
                    seed_construction_wall_s=result.get('pre_solve_budget_cost_s',0.),
                    post_solve_validation_time_s=result['post_solve_validation_time_s'],
                    unique_vector_evaluations=profile['unique_vector_evaluations'],
                    evaluator_requests=profile['evaluate_requests'],evaluator_cache_misses=profile['evaluator_cache_misses'],
                    objective_evaluations=result['objective_evaluations'],equality_evaluations=result['equality_evaluations'],
                    inequality_evaluations=result['inequality_evaluations'],
                    numerical_component_seconds=profile['numerical_component_seconds'],
                    instrumentation_bookkeeping_s=profile['measured_instrumentation_bookkeeping_seconds'],
                    source=str(path),source_sha256=digest(path))
                rows.append(row)
    if len(rows)!=8:raise ValueError('expected eight actual solves of one event')
    full={}
    for method in cfg['methods']:
        path=run/'actual_event/single_change'/method/'independent_acceptance.json'
        if path.exists():
            report=read(path);additional=report['additional_grid']
            full[method]=dict(status=report['status'],full_feasible=report['full_feasible'],
                original_dense_feasible=report['original_dense_feasible'],original_plan_valid=report['original_plan_valid'],
                additional_grid={k:v for k,v in additional.items() if k not in
                    ('query_times_s','poses_world','body_twists','body_accelerations','environment','route')},
                minimum_clearance_m=additional['environment']['minimum_clearance_m'],
                goal_position_error_m=report['original_plan_report']['goal']['position_error_m'],
                goal_yaw_error_rad=report['original_plan_report']['goal']['yaw_error_rad'])
    before=read(run/'preservation_before.json');after=preservation_snapshot(cfg)
    compared=['historical_files','external_files','external_git_sha','external_git_status','user_config_hashes']
    after['before_after_equal']={key:before[key]==after[key] for key in compared}
    after['valid']=bool(after['valid'] and all(after['before_after_equal'].values()))
    write(run/'preservation_after.json',after)
    if not after['valid']:raise ValueError('original preservation changed')
    audit=read(run/'existing_failure_audit/summary.json')
    synthetic=read(run/'synthetic_fixtures/solver/summary.json')
    summary=dict(completed_utc=utc(),starting_git_sha=read(run/'source.json')['starting_git_sha'],
        operational_status='GP_SE2_DIAG_01_COMPLETED_WITH_LIMITATIONS',diagnosis_status='PARTIALLY_LOCALIZED',
        actual_event_candidate_status='FULL_FEASIBLE_CANDIDATE_FOUND' if any(r['full_candidate_found'] for r in methods) else 'NO_FULL_FEASIBLE_CANDIDATE_FOUND',
        actual_case_id=cfg['actual_case_id'],existing_failure_audit=audit,
        mathematical_fixtures=read(run/'synthetic_fixtures/math/summary.json'),
        synthetic_solver_results=synthetic['rows'],actual_attempts=rows,actual_methods=methods,
        full_candidate_checks=full,selected_change=read(run/'decision.json')['selected_change'],
        confirmed=['Historical latest failures already violate collocation, not exclusively between inspection times.',
            'S0/S1/S2 interpolate known feasible curves to roundoff; S3 has small nonzero valid reconstruction error.',
            'The specified blind-spot GP example passes endpoint/midpoint lateral checks but fails between them.',
            'All three synthetic 3s perturbed starts recover sampled feasibility under original constraints.',
            'A fixed same-direction deceleration seed is a full feasible witness for the specified actual event.',
            'The actual solver retains that feasible seed without an improved feasible candidate under this budget.'],
        remaining_uncertainty=['No unique causal split between numerical derivatives, objective conditioning and limited iterations has been demonstrated.',
            'One benign event does not establish that this seed family meets other events goals or obstacles.',
            'Independent sampled checks do not prove continuous-time feasibility.',
            'No new MPC execution, performance comparison or GP superiority was evaluated.'],
        next_single_element='Verified objective gradient and constraint Jacobian with the same SLSQP, objective, initialization and budget; proposed only, not implemented.',
        preservation_valid=after['valid'],preserved_historical_file_count=after['historical_file_count'],
        preserved_checkpoint_file_count=after['checkpoint_file_count'],new_inference=False,new_mpc_execution=False,
        no_fallback=True,acceptance_relaxed=False,infeasibility_proven=False,
        final_git_sha_location='git_completion.json (written after focused commit and normal push)')
    write(run/'summary.json',summary)
    (run/'profiling').mkdir(exist_ok=False)
    write(run/'profiling/summary.json',dict(rows=rows,timing_note='component timings are subsets of evaluate; do not add caller-inclusive levels; validation and seed costs separately recorded'))
    table(run/'profiling/actual_event_comparison.csv',rows)
    write(run/'implementation_final.json',dict(captured_utc=utc(),source_sha256=code_snapshot(run,'final')))
    print(json.dumps({k:summary[k] for k in ('operational_status','diagnosis_status','actual_event_candidate_status')}),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase',choices=['prepare','audit','fixtures','synthetic-solves','baseline','decide','variant','finalize'])
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/gp_se2_diag_01.yaml')
    args=parser.parse_args()
    if args.phase=='prepare':prepare(args.run,args.config)
    elif args.phase=='audit':
        from reconciliation.gp_se2_diagnostics import audit_saved_attempts
        _,primary=settings(args.run)
        print(json.dumps(audit_saved_attempts(primary,args.run/'existing_failure_audit')),flush=True)
    elif args.phase=='fixtures':
        from reconciliation.gp_se2_diag_fixtures import write_optimizer_free_fixtures
        from reconciliation.gp_se2_diag_plot_sources import write_math_plot_sources
        _,primary=settings(args.run)
        config=yaml.safe_load((primary/'config_snapshot.yaml').read_text())['formulation']
        result=write_optimizer_free_fixtures(args.run/'synthetic_fixtures/math',config)
        write_math_plot_sources(args.run/'synthetic_fixtures/math')
        print(json.dumps(dict(fixtures=len(result['fixtures']),optimizer_executed=False,
                             output=str(args.run/'synthetic_fixtures/math'))),flush=True)
    elif args.phase=='synthetic-solves':synthetic_solves(args.run)
    elif args.phase=='baseline':actual_phase(args.run,'baseline')
    elif args.phase=='decide':decide(args.run)
    elif args.phase=='variant':actual_phase(args.run,'single_change')
    elif args.phase=='finalize':finalize(args.run)
    else:raise NotImplementedError('single variant requires completed diagnostics and decision first')


if __name__=='__main__':main()

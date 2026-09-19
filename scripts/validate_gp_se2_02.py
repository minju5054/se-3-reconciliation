#!/usr/bin/env python3
"""Read-only GP-SE2-02 artifact audit; never optimize or run a controller.

Saved trajectories are evaluated with the original primal/dense/full checkers.
Saved commands are integrated and original execution outcomes recomputed. The
only optional write is an exclusive validation report in the new output run.
"""
from __future__ import annotations
import argparse
import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import time
import zipfile

for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_name] = '1'
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
import numpy as np
import yaml
from validate_gp_se2_diag_01 import Checks, digest, read, write_exclusive, equivalent
from validate_gp_se2_diag_02 import (validate_attempt, validate_full_outcome,
    validate_seed, verify_digest)
from reconciliation.gp_se2_02_protocol import CASES, GP_METHODS, INITIALIZATIONS, execution_order, select_retained_start
from reconciliation.gp_se2_02_evaluation import METHODS, plan_for_method, evaluate_method, comparisons, make_mpc_request
from reconciliation.gp_se2_diagnostics import load_frozen_case
from reconciliation.gp_se2_diag_acceptance import check_full_candidate


def without_elapsed(value):
    """Only newly measured audit durations differ when recomputing outcomes."""
    result = dict(value)
    result.pop('independent_rollout_evaluation_wall_s', None)
    if 'timing' in result:
        result['timing'] = dict(result['timing'])
        result['timing'].pop('evaluation_wall_s', None)
    return result


def validate_csv_rows(path, expected, checks, label):
    """Match GP01 table serialization: JSON containers and blank absent columns."""
    with Path(path).open(newline='') as stream:
        reader=csv.DictReader(stream);rows=list(reader);fields=reader.fieldnames or []
    checks.check(len(rows)==len(expected),label+': complete row count')
    for index,(row,source) in enumerate(zip(rows,expected)):
        for field in fields:
            value=source.get(field)
            rendered='' if value is None else json.dumps(value) if isinstance(value,(dict,list)) else str(value)
            checks.check(row[field]==rendered,label+f': source row {index}/'+field)


def validate_method_selection(results, aggregate, checks, label):
    selected = select_retained_start(results)
    checks.equal(aggregate['selected_initialization'], selected, label+': same minimum-dense-cost start selection', exact=True)
    chosen = None if selected is None else results[selected]
    for key in ('candidate_world', 'candidate_vector', 'candidate_objective', 'support_poses', 'support_twists', 'selected_iterate', 'constraint_report'):
        checks.equal(aggregate[key], None if chosen is None else chosen[key], label+': unchanged selected '+key, exact=True)
    checks.check(not aggregate['fallback_used'], label+': no replacement after full-check failure')
    return selected


def validate_unsupported(result, seed, summary, checks, label):
    validate_seed(result, seed, checks, label)
    checks.check(not result['solver_executed'] and not result['candidate_found'], label+': unsupported start never optimized')
    checks.check(result['termination'] in ('DERIVATIVE_UNSUPPORTED', 'DERIVATIVE_VALIDATION_FAILED', 'UNSUPPORTED_GATE_CASE'), label+': precise unsupported status')
    for field in ('candidate_vector', 'candidate_world', 'candidate_objective', 'latest_iterate'):
        checks.check(result[field] is None, label+': unsupported absent '+field)
    checks.check(result['callback_snapshots'] == [] and result['candidate_checks'] == [], label+': no fabricated unsupported history')
    checks.check(not summary['selected_full_feasible'] and summary['final_full_feasible'] is None, label+': unsupported not accepted')


def validate_runtime_failure(result, checks, label):
    failures = result.get('recorded_derivative_errors', [])
    expected = 'DERIVATIVE_UNSUPPORTED' if failures else result['termination']
    checks.equal(result['effective_termination'], expected, label+': exact effective termination')
    if failures:
        checks.check(result['termination'] == 'NUMERICAL_FAILURE', label+': raw harness numerical failure preserved')
        for error in failures:
            checks.check(bool(error['reason_code']) and np.asarray(error['vector']).shape == (150,), label+': exact unsupported vector and reason retained')


class RuntimeGuardChecks:
    """A legitimate first-callback guard can prevent later Jacobian calls.

    All ordinary DIAG-02 validation is reused. Only its assertion that all three
    derivative callbacks have positive counts is replaced for a recorded runtime
    unsupported failure; no zero-count successful solve is accepted.
    """
    def __init__(self, checks, result):
        self.parent, self.result = checks, result
    def equal(self, *args, **kwargs):
        self.parent.equal(*args, **kwargs)
    def check(self, condition, label):
        if label.endswith(': all three supplied callbacks used') and self.result.get('recorded_derivative_errors'):
            calls = self.result['derivative_calls']
            condition = set(calls) == {'objective_gradient', 'equality_jacobian', 'inequality_jacobian'} and all(v >= 0 for v in calls.values()) and any(v > 0 for v in calls.values())
            label += ' or later callbacks preempted by preserved unsupported guard'
        self.parent.check(condition, label)


def rigid_value(case, vector, times):
    """Independent read-only transcription of the unchanged three-DOF evaluator."""
    from reconciliation.gp_se2_formulation import _pose_reference, _goal_inequalities, _query, _gate_rows, _constraint_report
    from reconciliation.se2 import compose_poses, se2_exp, se2_log, relative_pose
    p = case['problem']; c = p.config; f = p.common_reference
    support = p.times[1:]
    g = se2_exp(vector); candidate = compose_poses(g, f)
    entry = compose_poses(p.boundary_pose, se2_exp(support[0]*p.initial_twist))
    entry_r = se2_log(relative_pose(entry, candidate[0]))/np.asarray(c['fresh_std'])
    fresh = se2_log(relative_pose(f, candidate))/np.asarray(c['fresh_std'])
    at = lambda q: compose_poses(g, _pose_reference(f, support, q))
    eq, gates = _gate_rows(at, p.gates, support[0], support[-1]); poses = at(times)
    inequality = [_goal_inequalities(candidate[-1], p.goal_pose, c), gates,
                  _query(p.obstacle_clearance, poses, 'obstacle_clearance')-c['required_clearance'],
                  _query(p.workspace_margin, poses, 'workspace_margin')]
    entry_cost, fresh_cost = float(entry_r@entry_r), float(np.mean(np.sum(fresh**2, axis=1)))
    report = _constraint_report(eq, np.concatenate(inequality), c)
    return dict(objective=c['entry_weight']*entry_cost+c['lambda_fresh']*fresh_cost,
                candidate=candidate, transform_pose=g, report=report,
                factor_costs={'entry': entry_cost, 'fresh_uniform': fresh_cost, 'total': c['entry_weight']*entry_cost+c['lambda_fresh']*fresh_cost})


def validate_rigid(case, result, checks, label):
    from reconciliation.se2 import compose_poses, se2_exp, se2_log, inverse_pose
    p = case['problem']; c = p.config
    checks.equal(result['config'], c, label+': original rigid formulation', exact=True)
    checks.check(len(result['attempts']) == 2, label+': exactly two original rigid starts')
    entry = compose_poses(p.boundary_pose, se2_exp(p.times[1]*p.initial_twist))
    initial = [np.zeros(3), se2_log(compose_poses(entry, inverse_pose(p.common_reference[0])))]
    times = np.linspace(p.times[1], p.times[-1], int(np.ceil((p.times[-1]-p.times[1])/c['dense_dt_s']))+1)
    choices = []
    for index, attempt in enumerate(result['attempts']):
        checks.equal(attempt['initial_vector'], initial[index], label+': exact original rigid initialization '+str(index), exact=True)
        checks.check(attempt['initialization'] == ('identity', 'finite_lookahead_alignment')[index], label+': rigid initialization identity')
        vectors = {'initial': initial[index], 'latest_iterate': np.asarray(attempt['latest_iterate'])}
        if attempt['best_feasible_iterate'] is not None:
            vectors['best_feasible_intermediate'] = np.asarray(attempt['best_feasible_iterate'])
        checks.equal([r['iterate'] for r in attempt['candidate_checks']], list(vectors), label+': original rigid candidate coverage', exact=True)
        for row in attempt['candidate_checks']:
            name = row['iterate']; value = rigid_value(case, vectors[name], times)
            checks.equal(row['objective'], value['objective'], label+': rigid original objective '+name)
            for key, expected in value['report'].items():
                checks.equal(row['constraint_report'][key], expected, label+': rigid dense '+name+'/'+key)
            if value['report']['feasible']:
                choices.append((value['objective'], index, name, value))
    selected = min(choices, key=lambda r: r[:3]) if choices else None
    checks.equal(result.get('selected_initialization'), None if selected is None else selected[1], label+': rigid minimum-cost selection', exact=True)
    checks.equal(result['candidate_world'], None if selected is None else selected[3]['candidate'], label+': rigid candidate reconstruction')
    if selected:
        checks.equal(result['selected_iterate'], selected[2], label+': rigid selected source', exact=True)
        checks.equal(result['factor_costs'], selected[3]['factor_costs'], label+': rigid separate objective')


def validate_sources(run, checks):
    from reconciliation.gp_se2_environment import HospitalEnvironment
    from reconciliation.gp_se2_diag_initialization import same_curvature_deceleration_seed
    source, freeze, protocol = [read(run/name) for name in ('source.json', 'experiment_freeze.json', 'protocol.json')]
    config = yaml.safe_load((run/'config_snapshot.yaml').read_text())
    checks.check(digest(run/'config_snapshot.yaml') == source['original_config_sha256'], 'unchanged original configuration bytes')
    for path, value in {**source['preserved_core_sha256'], **source['input_sha256']}.items():
        verify_digest(ROOT/path, value, checks, 'preserved original source '+path)
    for path, record in source['authoritative_validations'].items():
        verify_digest(path, record['sha256'], checks, 'authoritative validation '+path)
        checks.check(read(path)['valid'], 'authoritative PASS '+path)
    for filename, field in [('protocol.json', 'protocol_sha256'), ('config_snapshot.yaml', 'config_sha256'),
                             ('experiment_config.yaml', 'experiment_config_sha256'), ('case_manifest.json', 'case_manifest_sha256'),
                             ('derivative_transfer_checks/summary.json', 'transfer_summary_sha256')]:
        verify_digest(run/filename, freeze[field], checks, 'primary freeze '+filename)
    for path, expected in freeze['code_sha256'].items():
        verify_digest(ROOT/path, expected, checks, 'primary code unchanged '+path)
        verify_digest(run/'implementation_sources/experiment'/path, expected, checks, 'archived primary code '+path)
    checks.equal(protocol['exact_execution_order'], execution_order(), 'fixed execution order', exact=True)
    checks.check(protocol['sequential'] and not protocol['retry'] and not protocol['replace_cases'], 'no concurrent retries or case replacement')
    checks.check(protocol['prepared_solve_budget_s'] == 30. and protocol['max_iterations'] == 200 and protocol['blas_threads'] == 1, 'fixed primary budgets')
    checks.equal(protocol['original_formulation'], config, 'same original formulation in protocol', exact=True)
    manifest = read(run/'case_manifest.json')['selected']
    checks.equal([(r['case_id'], r['case_role']) for r in manifest], list(CASES), 'exact four cases in fixed order', exact=True)
    env = HospitalEnvironment.load(source['environment_path'])
    for record in source['original_environment_file_hashes']:
        verify_digest(Path(source['environment_path'])/record['path'], record['sha256'], checks, 'unchanged environment '+record['path'])
    cases = {}
    for row in manifest:
        case = load_frozen_case(source['primary_run'], row['case_id'], environment=env)
        cases[row['case_id']] = case; folder = run/'cases'/row['case_directory']
        checks.check(not case['goal_route']['gates'], 'frozen no-gate scope '+row['case_id'])
        for name in ('input_context.json', 'goal_route.json', 'F_native.npy', 'F_common.npy', 'reference_preparation.json', 'source_metrics.json'):
            checks.check(digest(folder/name) == digest(case['case_directory']/name), 'exact original case copy '+row['case_id']+'/'+name)
        p = case['problem']
        seeds = [p.initializations()[0], same_curvature_deceleration_seed(p)]
        for name, seed in zip(INITIALIZATIONS, seeds):
            file = folder/'initializations'/(name+'.npy'); z = np.load(file, allow_pickle=False)
            verify_digest(file, row['seed_sha256'][name], checks, 'frozen seed '+row['case_id']+'/'+name)
            checks.equal(z, p.vector(seed['poses'], seed['twists']), 'same unfit seed reconstruction '+row['case_id']+'/'+name, exact=True)
            poses, twists = p.unpack(z)
            checks.equal(poses[0], p.boundary_pose, 'bitwise B '+row['case_id']+'/'+name, exact=True)
            checks.equal(twists[0], [case['context']['u_minus'][0], 0., case['context']['u_minus'][1]], 'physical twist distinct from memory '+row['case_id']+'/'+name, exact=True)
    checks.check(datetime.fromisoformat(read(run/'derivative_transfer_checks/summary.json')['completed_utc']) <= datetime.fromisoformat(freeze['frozen_utc']) <= datetime.fromisoformat(read(run/'optimization_started.json')['utc']), 'smoke then code freeze then primary execution')
    return source, config, manifest, cases


def validate_starts(run, manifest, cases, checks):
    all_starts, intervals = [], []
    completion = read(run/'optimization_completed.json')
    checks.equal(completion['exact_order'], execution_order(), 'completed fixed order', exact=True)
    checks.check(not completion['retries'], 'no primary optimization retry')
    for row in manifest:
        case = cases[row['case_id']]; base = run/'cases'/row['case_directory']
        rigid = read(base/'methods/M1_RIGID/solver_result.json')
        validate_rigid(case, rigid, checks, row['case_id']+'/M1')
        timing = read(base/'methods/M1_RIGID/execution_timing.json')
        intervals.append((timing['start_utc'], timing['end_utc']))
        for index, attempt in enumerate(rigid['attempts']):
            all_starts.append(dict(case_id=row['case_id'], case_role=row['case_role'], method='M1_RIGID', initialization=attempt['initialization'], initialization_index=index,
                termination=attempt['termination'], solver_success=attempt['solver_success'], iterations=attempt['iterations'], solve_wall_time_s=attempt['wall_time_s'], solver_executed=True,
                candidate_selection='unchanged solve_rigid internal policy'))
        for method in GP_METHODS:
            current = load_frozen_case(case['case_directory'].parents[1], row['case_id'], method, environment=case['environment'])
            p = current['problem']; results = []
            for name in INITIALIZATIONS:
                folder = base/'methods'/method/'starts'/name
                result, summary = read(folder/'solver_result.json'), read(folder/'result_summary.json')
                setup = read(folder/'setup.json'); seed = np.load(base/'initializations'/(name+'.npy'), allow_pickle=False)
                verify_digest(base/'initializations'/(name+'.npy'), setup['seed_sha256'], checks, 'start seed '+str(folder))
                validate_seed(result, seed, checks, str(folder))
                if result['solver_executed']:
                    replay, chosen = validate_attempt(p, result, 'SUPPLIED_JAC', RuntimeGuardChecks(checks, result), str(folder))
                    validate_full_outcome(p, current, result, summary, folder, replay, chosen, checks, str(folder))
                    validate_runtime_failure(result, checks, str(folder))
                else:
                    validate_unsupported(result, seed, summary, checks, str(folder))
                initial_full = check_full_candidate(p, seed, current)
                checks.equal(read(base/'initializations'/f'initial_full_{method}_{name}.json'), initial_full, 'independent initial acceptance '+str(folder))
                timing = read(folder/'execution_timing.json'); intervals.append((timing['start_utc'], timing['end_utc']))
                cold = setup['case_loading_seed_preparation_s']+setup['derivative_graph_construction_s']+setup['compilation_first_call_warmup_s']+result['setup_wall_time_s']+result['solve_wall_time_s']+result['post_solve_validation_time_s']+summary['independent_full_validation_time_s']
                checks.equal(summary['cold_setup_solve_validation_s'], cold, 'complete cold cost '+str(folder))
                all_starts.append(summary); results.append(result)
            aggregate = read(base/'methods'/method/'solver_result.json')
            selected = validate_method_selection(results, aggregate, checks, row['case_id']+'/'+method)
            if selected is not None:
                checks.equal(read(base/'methods'/method/'full_acceptance.json'), read(base/'methods'/method/'starts'/INITIALIZATIONS[selected]/'full_acceptance.json'), 'selected full check not swapped '+method)
            checks.equal(aggregate['sum_all_start_cold_wall_s'], sum(read(base/'methods'/method/'starts'/name/'result_summary.json')['cold_setup_solve_validation_s'] for name in INITIALIZATIONS), 'both initialization costs '+method)
        seed_result = read(base/'methods/SEED_ONLY/solver_result.json')
        seed = np.load(base/'initializations/I1_DECEL.npy', allow_pickle=False)
        poses, twists = case['problem'].unpack(seed)
        checks.equal(seed_result['candidate_vector'], seed, 'seed-only exact deceleration '+row['case_id'], exact=True)
        checks.equal(seed_result['candidate_world'], poses[1:], 'seed-only no goal fitting '+row['case_id'], exact=True)
        checks.check(not seed_result['optimization_performed'] and not seed_result['attempts'] and not seed_result['fallback_used'], 'seed-only has no optimizer/fallback '+row['case_id'])
    checks.check(len(all_starts) == 24 and sum(r['method'] in GP_METHODS for r in all_starts) == 16, '16 GP plus eight rigid ledger records')
    for left, right in zip(intervals, intervals[1:]):
        checks.check(datetime.fromisoformat(left[1]) <= datetime.fromisoformat(right[0]), 'sequential nonoverlapping solver intervals')
    checks.equal(completion['starts'], all_starts, 'full start ledger matches per-start files', exact=True)
    validate_csv_rows(run/'aggregate/all_starts.csv', [{k:v for k,v in r.items() if not isinstance(v, (dict,list))} for r in all_starts], checks, 'all starts CSV')
    checks.check(completion['actual_GP_solver_calls'] == sum(r['solver_executed'] for r in all_starts if r['method'] in GP_METHODS), 'unsupported starts not counted as solves')
    return all_starts


def validate_methods(run, manifest, cases, checks):
    output = run/'mpc_output'; provenance = read(output/'provenance.json')
    verify_digest(run/'mpc_request.json', provenance['request_sha256'], checks, 'official MPC exact request')
    verify_digest(provenance['mpc_source'], provenance['mpc_source_sha256'], checks, 'official MPC source unchanged')
    for record in read(output/'output_hashes.json')['files']:
        verify_digest(output/record['path'], record['sha256'], checks, 'MPC output '+record['path'])
    rows, entries = [], []
    for row in manifest:
        case = cases[row['case_id']]; base = run/'cases'/row['case_directory']
        historical = read(output/row['case_directory']/'historical_solve_audit.json')
        checks.check(historical['passed'] and not historical['B_counterfactual_equality_required'], 'historical input audit separate '+row['case_id'])
        checks.equal(historical['historical_input_pose_world'], case['context']['historical_first_fresh_solve']['input_pose'], 'historical input preserved '+row['case_id'], exact=True)
        entry = dict(context=case['context'], goal_route=case['goal_route'], methods={})
        for method in METHODS:
            target = base/'methods'/method
            solver, plan, metrics = [read(target/(name+'.json')) for name in ('solver_result', 'plan_validation', 'metrics')]
            candidate = solver['candidate_world']; full = read(target/'full_acceptance.json') if (target/'full_acceptance.json').exists() else None
            expected_plan = plan_for_method(case, method, candidate, solver, full)
            checks.equal(plan, expected_plan, 'original plan policy '+row['case_id']+'/'+method)
            path = target/'candidate_world.npy'
            if candidate is not None:
                checks.equal(np.load(path, allow_pickle=False), candidate, 'exact saved candidate '+row['case_id']+'/'+method, exact=True)
            else:
                checks.check(not path.exists(), 'absent candidate has no fallback file '+row['case_id']+'/'+method)
            path = target/'rollout/rollout.json'; rollout = read(path) if path.exists() else None
            if rollout is not None:
                checks.check(digest(path) == digest(output/row['case_directory']/method/'rollout.json'), 'rollout from fresh worker '+row['case_id']+'/'+method)
                verify_digest(target/'candidate_world.npy', rollout['candidate_source']['sha256'], checks, 'executed candidate hash '+row['case_id']+'/'+method)
            expected = evaluate_method(case, method, candidate, plan, solver, rollout)
            checks.equal(without_elapsed(metrics), without_elapsed(expected), 'recomputed original rollout outcome '+row['case_id']+'/'+method)
            rows.append(dict(case_id=row['case_id'], case_role=row['case_role'], method=method,
                candidate_available=metrics['candidate_available'], plan_valid=metrics['plan_valid'], rollout_performed=metrics['rollout_performed'],
                rollout_success=metrics['rollout_success'], primary_success=metrics['rollout_success'], termination_reason=metrics['termination_reason'],
                failure_reasons=metrics['failure_reasons'], optimizer_wall_s=solver['total_optimization_and_check_wall_s'], deformation=plan.get('deformation'), rollout_metrics=metrics))
            entry['methods'][method] = dict(candidate_path=str((target/'candidate_world.npy').resolve()) if candidate is not None else None, plan=plan, solver_result=solver)
        entries.append(entry)
        checks.check(read(base/'validation.json')['method_count'] == 6, 'all six methods per case '+row['case_id'])
    source = read(run/'source.json')
    checks.equal(read(run/'mpc_request.json'), make_mpc_request(source['source_run'], entries, cases[CASES[0][0]]['config']), 'request eligibility preserves invalid-plan/no-candidate statuses', exact=True)
    checks.equal(read(run/'aggregate/method_results.json'), rows, 'all 24 original outcomes', exact=True)
    expected = comparisons(rows); summary = read(run/'aggregate/summary.json')
    for key, value in expected.items():
        checks.equal(summary[key], value, 'failure-first paired comparison '+key)
    checks.check(summary['actual_rollout_count'] == sum(r['rollout_performed'] for r in rows), 'actual rollout count')
    validate_csv_rows(run/'aggregate/primary_outcomes.csv', [{k:v for k,v in r.items() if k!='rollout_metrics'} for r in rows], checks, 'primary CSV')
    validate_csv_rows(run/'aggregate/candidate_acceptance.csv', [{k:r[k] for k in ('case_id','case_role','method','candidate_available','plan_valid','rollout_performed')} for r in rows], checks, 'candidate CSV')
    for name in ('regressions', 'paired_metrics'):
        validate_csv_rows(run/'aggregate'/(name+'.csv'), expected[name], checks, name+' CSV')
    timing = [dict(case_id=r['case_id'], method=r['method'], total_optimizer_all_starts_wall_s=r['optimizer_wall_s'],
        rollout_evaluation_wall_s=r['rollout_metrics']['independent_rollout_evaluation_wall_s'],
        mpc_solve_count=(r['rollout_metrics']['execution'] or {}).get('mpc_solve_count_total'),
        mpc_official_solve_total_s=(r['rollout_metrics']['execution'] or {}).get('mpc_official_solve_total_s'),
        mpc_submit_wait_poll_total_s=(r['rollout_metrics']['execution'] or {}).get('mpc_submit_wait_poll_total_s')) for r in rows]
    validate_csv_rows(run/'aggregate/timing.csv', timing, checks, 'timing CSV')
    return rows


def validate_preservation(run, checks):
    before, after = read(run/'preservation_before.json'), read(run/'preservation_after.json')
    checks.check(before['valid'] and after['valid'], 'before/after preservation PASS')
    for key in ('historical_files', 'external_files', 'user_config_hashes', 'previous_diag01_files', 'previous_diag02_files'):
        checks.equal(before[key], after[key], 'unchanged preservation '+key, exact=True)
        for path, expected in before[key].items():
            verify_digest(ROOT/path, expected, checks, 'preserved file '+path)
    for key in ('external_git_sha', 'external_git_status'):
        checks.equal(before[key], after[key], 'external Git unchanged '+key, exact=True)
    manifest = read(run/'artifact_manifest.json')['files']
    for name, expected in manifest.items():
        checks.check((run/name).resolve().is_relative_to(run), 'artifact path inside run '+name)
        verify_digest(run/name, expected, checks, 'artifact hash '+name)
    return len(manifest)


def validate_transfer(run, checks):
    from reconciliation.gp_se2_02_transfer_validation import validate_saved_transfer
    validate_saved_transfer(run/'derivative_transfer_checks', read(run/'source.json')['primary_run'], checks)


def plot_numbers(case, config):
    """Reconstruct numbers only from saved inputs; never render or write a PNG."""
    import plot_gp_se2_02 as p
    from reconciliation.gp_se2_evaluation import goal_trace
    context, route = read(case/'input_context.json'), read(case/'goal_route.json')
    methods = p.load_methods(case); past = read(case/'actual_past_execution.json')
    xy = p.xy_numeric(context, route, methods, past); bounds = p.common_limits(xy)
    b = np.asarray(context['B_world'])
    numbers = {name: dict(**xy, axes_world_m=([b[0]-.75,b[0]+.75,b[1]-.75,b[1]+.75] if name=='boundary_zoom' else bounds)) for name in p.PLOT_NAMES[:3]}
    numbers['clearance_vs_time'] = {m: None if item['metrics']['environment'] is None else
        dict(times_s=item['metrics']['dense_times_s'], clearance_m=item['metrics']['environment']['clearance_samples_m']) for m,item in methods.items()}
    for component, name in enumerate(('linear_command_vs_time','angular_command_vs_time')):
        numbers[name] = {m:p.command_numeric(item,context,component) for m,item in methods.items()}
    numbers['goal_error_vs_time'] = {}
    for method,item in methods.items():
        points = item['metrics'].get('dense_poses_world')
        trace = None
        if points:
            distance,yaw,_ = goal_trace(points,route['goal_world'])
            trace = dict(times_s=item['metrics']['dense_times_s'],position_error_m=distance,yaw_error_deg=np.degrees(yaw))
        numbers['goal_error_vs_time'][method] = trace
    numbers['candidate_feasibility_summary'] = {m:p.availability(item) for m,item in methods.items()}
    history,traces,cost = {},{},{}
    for method in GP_METHODS:
        for seed,start in methods[method]['starts'].items():
            key=method+'/'+seed; r,s = start['solver_result'],start['setup']
            history[key] = p.best_full_history(start)
            traces[key] = dict(seed=p.sample_support(start['seed_metadata']['chart_reconstructed_poses'],start['seed_metadata']['chart_reconstructed_twists'],config['formulation']),
                final_full_feasible=start['result_summary'].get('final_full_feasible'), seed_full_feasible=start['result_summary'].get('initial_full_feasible'),
                final=p.sample_support(r.get('latest_support_poses'),r.get('latest_support_twists'),config['formulation']),
                selected=p.sample_support(r.get('support_poses'),r.get('support_twists'),config['formulation']),selected_full_feasible=bool(start['full_acceptance'].get('full_feasible')))
            parts=dict(case_seed_loading=s.get('case_loading_seed_preparation_s',0.),solver_setup=r.get('setup_wall_time_s',0.),
                graph=s.get('derivative_graph_construction_s',0.),warmup=s.get('compilation_first_call_warmup_s',0.),solve=r.get('solve_wall_time_s',0.),
                original_post=r.get('post_solve_validation_time_s',0.),full_validation=start['post_full_checks'].get('full_validation_wall_time_s',0.))
            count=dict(objective=r.get('objective_evaluations'),primal_misses=r.get('profiling',{}).get('evaluator_cache_misses'),objective_jac=r.get('derivative_calls',{}).get('objective_gradient'))
            cost[key]=dict(disjoint_seconds=parts,counts=count,setup=s,termination=r.get('termination'))
    numbers.update(feasible_objective_history=history,lateral_velocity=traces,planned_acceleration=traces,solver_cost=cost)
    return numbers


def validate_plot_record(image, side, expected, checks, label):
    checks.check(side['image'] == image.name and side['image_sha256'] == digest(image), label+': image hash')
    checks.check(side['dpi'] == 160 and side['label'] == 'OFFLINE COUNTERFACTUAL HARD-HANDOFF COMPARISON', label+': real metric scope')
    checks.check(not side['new_inference'] and not side['new_solver'] and not side['new_execution'], label+': saved values only')
    checks.equal(side['numeric_data'], expected, label+': exact plotted numbers')
    verify_digest(ROOT/'scripts/plot_gp_se2_02.py', side['plotter_sha256'], checks, label+': plotter hash')
    for path, expected_hash in side['source_hashes'].items():
        verify_digest(path, expected_hash, checks, label+': input hash '+path)


def validate_past(case, checks):
    """Historical line must end at B, without any future counterfactual sample."""
    past, context = read(case/'actual_past_execution.json'), read(case/'input_context.json')
    source = Path(context['source_root'])/'episodes'/context['episode_id']/'execution.csv'
    verify_digest(source,past['source_sha256'],checks,'past execution original CSV')
    with source.open() as stream:
        rows=list(csv.DictReader(stream))
    boundary=context['switch_sim_time_s']
    rows=[r for r in rows if boundary-3. <= float(r['sim_time_s']) <= boundary and int(r['state_id']) <= context['switch_state_id']]
    checks.equal(past['state_ids'],[int(r['state_id']) for r in rows],'actual past state selection',exact=True)
    checks.equal(past['times_relative_to_B_s'],[float(r['sim_time_s'])-boundary for r in rows],'actual past time convention',exact=True)
    checks.equal(past['poses_world'],[[float(r[k]) for k in ('x','y','yaw')] for r in rows],'actual past untouched world states',exact=True)
    checks.check(past['historical'] and not past['newly_executed'] and not past['future_samples_used'],'past does not become execution evidence')


def validate_gui(bundle, run, checks):
    from PIL import Image
    runtime=read(bundle/'gui/runtime_validation.json')
    checks.check(runtime['status']=='GP_SE2_02_COMPARISON_REPLAY_RUNTIME_VALIDATED' and Path(runtime['experiment_run']).resolve()==run,'GUI actual new-run renderer validation')
    checks.check(runtime['label']=='OFFLINE COUNTERFACTUAL HARD-HANDOFF COMPARISON','GUI correct offline label')
    expected={(case,method) for case,_ in CASES[:2] for method in METHODS}
    checks.check({(r['case_id'],r['method']) for r in runtime['captures']}==expected and len(runtime['captures'])==12,'GUI all six methods on benign and first hard')
    checks.check(runtime['display_playback_only'] and runtime['new_mpc_solves']==runtime['generated_execution_states']==runtime['new_inference_count']==0,'GUI no manufactured new execution')
    for record in runtime['captures']:
        image=bundle/'gui'/record['path'];side=read(image.with_suffix('.json'))
        verify_digest(image,record['sha256'],checks,'actual GUI screenshot '+record['path'])
        checks.check(side['image_sha256']==record['sha256'] and side['display_playback_only'],'GUI sidecar image and playback '+record['path'])
        for source in side['source_hashes']:
            verify_digest(source['path'],source['sha256'],checks,'GUI immutable input '+source['path'])
        with Image.open(image) as pixels:
            checks.check(min(pixels.size)>=480 and np.asarray(pixels).std()>1,'GUI nonblank renderer image '+record['path'])
        sample=side['saved_state'];folder=run/'cases'/record['case_id'].replace('/','__')/'methods'/record['method']
        rollout_file=folder/'rollout/rollout.json'
        if rollout_file.exists():
            rollout=read(rollout_file)
            checks.equal(sample['pose_world'],rollout['states'][sample['saved_row']]['pose_world'],'GUI saved pose matches actual rollout '+record['path'])
        else:
            context=read(folder.parents[1]/'input_context.json')
            checks.equal(sample['pose_world'],context['B_world'],'GUI unavailable execution remains at B '+record['path'])
        from reconciliation.se2 import wrap_angle
        difference=np.asarray(sample['usd_pose_readback'])-sample['pose_world']
        checks.check(np.max(np.abs(difference[:2])) <= 1e-10 and abs(float(wrap_angle(difference[2]))) <= 1e-10,'GUI USD pose readback '+record['path'])
    return {str(Path('gui')/r['path']) for r in runtime['captures']} | {str(Path('gui')/Path(r['path']).with_suffix('.json')) for r in runtime['captures']} | {'gui/runtime_validation.json'}


def validate_bundle_allowlist(bundle, allowed, checks):
    manifest=read(bundle/'manifest.json');records=manifest['allowlisted_files']
    checks.check(len({r['path'] for r in records})==len(records),'review has no duplicate paths')
    checks.check({r['path'] for r in records}==allowed,'exact compact review allowlist')
    for record in records:
        path=bundle/record['path']
        checks.check(path.resolve().is_relative_to(bundle.resolve()),'review safe path '+record['path'])
        verify_digest(path,record['sha256'],checks,'review content hash '+record['path'])
        checks.check(path.stat().st_size==record['bytes'],'review exact byte count '+record['path'])
        if record['source']!='generated review text':
            verify_digest(record['source'],record['sha256'],checks,'review original source '+record['path'])
    with zipfile.ZipFile(bundle.parent/'review_bundle.zip') as archive:
        checks.check(archive.testzip() is None and set(archive.namelist())==allowed|{'manifest.json'},'review ZIP exact allowlist and CRC')
        for name in archive.namelist():
            checks.check(archive.read(name)==(bundle/name).read_bytes(),'review ZIP exact content '+name)


def validate_plots_and_bundle(run, manifest, config, checks):
    from PIL import Image
    import plot_gp_se2_02 as p
    inventory=read(run/'plot_manifest.json')
    expected={str(Path('cases')/r['case_directory']/'plots'/(name+'.png')) for r in manifest for name in p.PLOT_NAMES}
    records=inventory['images']
    checks.check({r['path'] for r in records}==expected and len(records)==48,'all 48 required case figures retained')
    for row in manifest:
        case=run/'cases'/row['case_directory'];validate_past(case,checks);numbers=plot_numbers(case,config)
        for name,values in numbers.items():
            image=case/'plots'/(name+'.png');side=read(image.with_suffix('.json'))
            validate_plot_record(image,side,values,checks,str(image.relative_to(run)))
            with Image.open(image) as pixels:
                checks.check(min(pixels.size)>=900,'scientific figure export resolution '+image.name)
    for record in records:
        verify_digest(run/record['path'],record['sha256'],checks,'plot inventory '+record['path'])
        verify_digest((run/record['path']).with_suffix('.json'),record['sidecar_sha256'],checks,'plot numeric sidecar inventory '+record['path'])
    bundle=run/'review_bundle';bundle_manifest=read(bundle/'manifest.json')
    allowed={'source.json','protocol.json','config_snapshot.yaml','experiment_config.yaml','case_manifest.json','plot_manifest.json','README.md','index.html','final_report.json','RESULTS.md'}
    allowed|={'aggregate/'+n for n in ('all_starts.csv','candidate_acceptance.csv','primary_outcomes.csv','paired_metrics.csv','regressions.csv','timing.csv','summary.json')}
    allowed|={'aggregate/outcome_matrix.csv','aggregate/compute_summary.json'}
    allowed|={'history_supplement_manifest.json','history_supplement_visual_review.json','plot_visual_review.json'}
    allowed|={str(Path('cases')/r['case_directory']/'plots'/('feasible_objective_history_full_time'+ext)) for r in manifest for ext in ('.png','.json')}
    allowed|={str(Path('cases')/r['case_directory']/'plots'/(name+ext)) for r in manifest for name in ('candidate_world_overlay','actual_rollout_overlay','boundary_zoom','candidate_feasibility_summary') for ext in ('.png','.json')}
    if bundle_manifest['gui_runtime_validated']:
        allowed|=validate_gui(bundle,run,checks)
    else:
        checks.check('GUI_RUNTIME_NOT_VALIDATED' in (bundle/'index.html').read_text(),'GUI unavailable explicitly reported without synthetic evidence')
    validate_bundle_allowlist(bundle,allowed,checks)
    package=read(run/'review_bundle_manifest.json')
    verify_digest(run/'review_bundle.zip',package['bundle_sha256'],checks,'review ZIP publication hash')
    checks.check(package['bundle_bytes']==(run/'review_bundle.zip').stat().st_size,'review ZIP publication bytes')
    completion=read(run/'review_completion.json')
    verify_digest(run/'review_base/review_bundle.zip',completion['base_package_sha256'],checks,'frozen original packager output preserved')
    checks.check(completion['base_package_preserved'] and not completion['numerical_artifacts_changed'] and not completion['new_optimization'] and not completion['new_rollout'],'completion adds reporting only')
    checks.equal(completion['final_publication'],package,'completion final package identity',exact=True)
    supplemental=read(run/'history_supplement_manifest.json')
    checks.check(supplemental['image_count']==4 and supplemental['original_48_plots_unchanged'] and not supplemental['solver_rerun'],'four reporting-only supplements preserve primary figures')
    verify_digest(run/'plot_manifest.json',supplemental['original_plot_manifest_sha256'],checks,'original figure inventory preserved')
    for row in supplemental['images']:
        path=run/row['path'];side=read(path.with_suffix('.json'))
        verify_digest(path,row['sha256'],checks,'supplemental image '+row['path'])
        verify_digest(path.with_suffix('.json'),row['sidecar_sha256'],checks,'supplemental sidecar '+row['path'])
        original=read(side['original_sidecar'])
        checks.equal(side['numeric_data'],original['numeric_data'],'identical supplemental numeric history '+row['path'],exact=True)
        verify_digest(side['original_sidecar'],side['original_sidecar_sha256'],checks,'preserved original history sidecar')
        checks.check(not side['numerical_values_changed'] and not side['original_plots_changed'] and not side['solver_rerun'] and not side['acceptance_or_selection_changed'],'display-only supplement scope')
        case=path.parents[1]
        times={key:read(case/'methods'/key.split('/')[0]/'starts'/key.split('/')[1]/'solver_result.json')['solve_wall_time_s'] for key in side['numeric_data']}
        costs=[float(p['best_full_objective']) for record in side['numeric_data'].values() for p in record['points'] if p['best_full_objective'] is not None]
        costs.extend(float(record['seed_objective']) for record in side['numeric_data'].values() if record['seed_full_feasible'])
        checks.equal(side['prepared_solve_end_s'],times,'supplement actual solve ends')
        checks.equal(side['axes']['xlim_s'],[0.,max(times.values()) or 1.],'common full elapsed-time axis')
        checks.equal(side['axes']['ylim_original_objective'],[0.,max(costs)*1.05 if costs and max(costs)>0 else 1.],'common unscaled objective axis includes seed')
        old=run/'history_supplement_attempt01'/row['path']
        if old.with_suffix('.json').exists():
            prior=read(old.with_suffix('.json'))
            checks.equal(side['numeric_data'],prior['numeric_data'],'legend-only supplement numeric preservation',exact=True)
            checks.equal(side['axes'],prior['axes'],'legend-only supplement axes preservation',exact=True)
    visual=read(run/'history_supplement_visual_review.json')
    checks.check(visual['valid'] and visual['original48imagehashes_preserved'] and not visual['solver_rerun'],'supplement visual review scope')
    for record in visual['checks']:
        verify_digest(run/record['image'],record['sha256'],checks,'visually reviewed current supplement')


def expected_final_numerical(run, summary):
    starts=read(run/'optimization_completed.json')['starts']
    gp=[r for r in starts if r['method'] in GP_METHODS]
    methods={}
    for method in GP_METHODS:
        rows=[r for r in gp if r['method']==method and r['case_role']!='BENIGN_CONTROL']
        group=summary['groups']['hard'][method]
        methods[method]=dict(events=3,candidate_available=group['candidates'],plan_valid=group['plan_valid'],
            converged_starts=sum(r['solver_success'] for r in rows),
            full_feasible_final_starts=sum(bool(r.get('final_full_feasible')) for r in rows))
    return dict(gp_start_count=len(gp),rigid_start_count=len(starts)-len(gp),
        actual_gp_solver_calls=sum(r['solver_executed'] for r in gp),
        gp_solver_converged_starts=sum(r['solver_success'] for r in gp),hard_gp_methods=methods)


def validate_final_report(run, checks):
    report=read(run/'final_report.json');summary=read(run/'aggregate/summary.json')
    freeze=read(run/'experiment_freeze.json')
    checks.check(report['schema_version']==1 and report['experiment']=='GP-SE2-02','final report identity')
    checks.check(report['operational_status'] in ('GP_SE2_02_COMPLETED','GP_SE2_02_COMPLETED_WITH_LIMITATIONS'),'final operational completion distinguished from research finding')
    checks.equal(report['execution_interpretation'],summary['execution_interpretation'],'final execution conclusion from original success transitions',exact=True)
    checks.equal(report['actual_rollout_count'],summary['actual_rollout_count'],'final actual execution coverage',exact=True)
    checks.equal(report['numerical'],expected_final_numerical(run,summary),'final numerical convergence/availability separate',exact=True)
    checks.check(report['validation_requirement']=='authoritative validation.json must pass','final completion conditional on authoritative artifact audit')
    checks.equal(report['original_summary_operational_status'],summary['operational_status'],'interim summary preserved and identified',exact=True)
    checks.equal(report['experiment_git_sha'],freeze['experiment_git_sha'],'final experiment SHA',exact=True)
    verify_digest(run/'aggregate/summary.json',report['aggregate_summary_sha256'],checks,'final original numeric summary hash')
    verify_digest(run/'experiment_freeze.json',report['experiment_freeze_sha256'],checks,'final experiment freeze hash')
    verify_digest(run/'aggregate/outcome_matrix.csv',report['outcome_matrix_sha256'],checks,'final expanded outcome matrix hash')
    verify_digest(run/'aggregate/compute_summary.json',report['compute_summary_sha256'],checks,'final complete compute accounting hash')
    gui=report['gui']
    if gui['status']=='GUI_RUNTIME_NOT_VALIDATED':
        checks.check(gui['runtime_validation_path'] is None and gui['sha256'] is None,'unavailable GUI remains null')
        checks.check(report['operational_status']=='GP_SE2_02_COMPLETED_WITH_LIMITATIONS','GUI limitation explicit in operational result')
    else:
        verify_digest(gui['runtime_validation_path'],gui['sha256'],checks,'final actual GUI report hash')
        runtime=read(gui['runtime_validation_path'])
        checks.check(runtime['status']==gui['status']=='GP_SE2_02_COMPARISON_REPLAY_RUNTIME_VALIDATED','final actual renderer status')
    checks.check(not list(run.glob('technical_failure_*.json')),'completed primary has no hidden technical failure restart')


def expanded_outcomes(rows):
    """Numeric goal thresholds independently check the reporting reason mapping."""
    expected=[]
    for row in rows:
        m=row['rollout_metrics'];e=m.get('execution') or {};env=m.get('environment') or {}
        performed=row['rollout_performed']
        expected.append(dict(case=row['case_id'],case_role=row['case_role'],method=row['method'],
            candidate_available=row['candidate_available'],plan_valid=row['plan_valid'],rollout_performed=performed,
            diagnostic_rollout_plan_invalid=m['diagnostic_rollout_plan_invalid'],
            collision=env.get('physical_overlap'),clearance_valid=env.get('clearance_valid'),known_workspace=env.get('workspace_known'),
            motion_valid=e.get('motion_limits_pass'),controller_valid=e.get('controller_validity_pass'),
            goal_position_valid=None if not performed else e['terminal_position_error_m']<=e['goal_position_tolerance_m'],
            goal_yaw_valid=None if not performed else e['terminal_yaw_error_rad']<=e['goal_yaw_tolerance_rad'],
            goal_dwell_valid=e.get('terminal_goal_dwell_pass'),terminal_position_error_m=e.get('terminal_position_error_m'),
            terminal_yaw_error_rad=e.get('terminal_yaw_error_rad'),minimum_clearance_m=m.get('minimum_clearance_m'),
            rollout_success=row['rollout_success'],termination_reasons=';'.join(m['termination_reasons']),status_reasons=';'.join(m['status_reasons'])))
    return expected


def validate_derived_reporting(run, checks):
    rows=read(run/'aggregate/method_results.json')
    validate_csv_rows(run/'aggregate/outcome_matrix.csv',expanded_outcomes(rows),checks,'expanded full outcome matrix')
    saved=read(run/'aggregate/compute_summary.json');optimization=read(run/'optimization_completed.json')
    source=read(run/'source.json');summary=read(run/'aggregate/summary.json');manifest=read(run/'case_manifest.json')
    starts=[s for s in optimization['starts'] if s['method'] in GP_METHODS]
    results=[read(run/'cases'/s['case_id'].replace('/','__')/'methods'/s['method']/'starts'/s['initialization']/'solver_result.json') for s in starts]
    for key in ('case_loading_seed_preparation_s','derivative_graph_construction_s','compilation_first_call_warmup_s',
                'solve_wall_time_s','post_solve_validation_time_s','independent_full_validation_time_s','cold_setup_solve_validation_s'):
        checks.equal(saved['gp_all_16_starts'][key],sum(s[key] for s in starts),'all planned GP costs '+key)
    checks.equal(saved['gp_all_16_starts']['harness_setup_s'],sum(r['setup_wall_time_s'] for r in results),'harness setup separated from initial solve evaluation')
    counts={key:sum(r[key] for r in results) for key in ('iterations','objective_evaluations','equality_evaluations','inequality_evaluations','callback_count')}
    counts.update({key:sum(r['derivative_calls'][key] for r in results) for key in ('objective_gradient','equality_jacobian','inequality_jacobian')})
    counts.update(primal_cache_misses=sum(r['profiling']['evaluator_cache_misses'] for r in results),
        primal_environment_query_calls=sum(r['profiling']['numerical_component_calls']['environment_query'] for r in results),
        primal_GP_interpolation_calls=sum(r['profiling']['numerical_component_calls']['collocation_gp_interpolation'] for r in results),
        derivative_environment_query_calls=sum(r['derivative_provider_stats']['phases']['runtime']['environment_calls'] for r in results),
        derivative_core_ad_evaluations=sum(r['derivative_provider_stats']['phases']['runtime']['core_ad_evaluations'] for r in results))
    checks.equal(saved['gp_evaluation_counts'],counts,'all planned actual evaluator counts',exact=True)
    checks.equal(saved['environment_loading_s'],dict(preparation=source['shared_preparation_environment_loading_s'],optimization=optimization['shared_environment_loading_s'],evaluation=summary['shared_evaluation_environment_loading_s']),'all environment loading phases')
    checks.equal(saved['method_costs'],optimization['method_timings'],'method total includes both starts and checks',exact=True)
    for key,field in [('seed_construction_s','seed_construction_s'),('initial_full_checks_s','initial_full_checks_s')]:
        checks.equal(saved[key],sum(r['preparation_timing'][field] for r in manifest['selected']),'shared preparation '+key)
    phases=dict(offline_optimization_phase_wall_s=optimization['offline_optimization_phase_wall_s'],
        official_mpc_batch_wall_s=read(run/'mpc_batch_timing.json')['wall_seconds'],outcome_evaluation_wall_s=summary['evaluation_phase_wall_s'])
    for key,value in phases.items():
        checks.equal(saved[key],value,'disjoint phase timing '+key)
    checks.equal(saved['optimization_mpc_evaluation_phase_wall_sum_s'],sum(phases.values()),'disjoint numerical phase wall sum')
    execution=[r['rollout_metrics']['execution'] for r in rows if r['rollout_performed']]
    checks.equal(saved['official_mpc_rollout_solve_s'],sum(e['mpc_official_solve_total_s'] for e in execution),'actual controller solve total nested separately')
    checks.equal(saved['official_mpc_rollout_solve_count'],sum(e['mpc_solve_count_total'] for e in execution),'exact actual controller solve count',exact=True)
    for path,value in saved['source_hashes'].items():
        verify_digest(run/path,value,checks,'reporting source hash '+path)


def validate(run):
    run = Path(run).resolve(); checks = Checks(); state = {}; started = time.perf_counter()
    checks.section('sources', lambda: state.update(zip(('source','config','manifest','cases'), validate_sources(run, checks))))
    if 'cases' in state:
        checks.section('transfer derivative evidence', lambda: validate_transfer(run, checks))
        checks.section('all solver starts and acceptance', lambda: validate_starts(run, state['manifest'], state['cases'], checks))
        checks.section('MPC and method outcomes', lambda: state.update(rows=validate_methods(run, state['manifest'], state['cases'], checks)))
        checks.section('plots and review evidence', lambda: validate_plots_and_bundle(run, state['manifest'], state['config'], checks))
        checks.section('final scientific and operational report', lambda: validate_final_report(run, checks))
        checks.section('expanded outcomes and compute accounting', lambda: validate_derived_reporting(run, checks))
        checks.section('preservation', lambda: state.update(artifact_count=validate_preservation(run, checks)))
    return dict(valid=not checks.errors, check_count=checks.count, errors=checks.errors,
                experiment='GP-SE2-02', artifact_count=state.get('artifact_count'),
                created_utc=datetime.now(timezone.utc).isoformat(), independent_validation_wall_s=time.perf_counter()-started,
                validation_scope='saved source/seed/order, original primal/dense/full acceptance, exact held-command execution and original goal/dwell, complete outcomes, plots and GUI hashes; no optimizer or MPC rerun',
                no_optimization=True, no_new_rollout=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True, type=Path); parser.add_argument('--output', type=Path)
    args = parser.parse_args(); result = validate(args.run)
    if args.output:
        output = args.output.resolve()
        if not output.is_relative_to(args.run.resolve()):
            raise ValueError('validation report must be inside the new run')
        write_exclusive(output, result)
    print(json.dumps(result, indent=2)); raise SystemExit(0 if result['valid'] else 1)


if __name__ == '__main__':
    main()

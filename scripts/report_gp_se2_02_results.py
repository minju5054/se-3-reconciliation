#!/usr/bin/env python3
"""Derive completion tables from saved GP-SE2-02 results; never run a solver."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path


def read(path):
    return json.loads(Path(path).read_text())


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def outcome_matrix(rows):
    result = []
    for row in rows:
        m = row['rollout_metrics']
        e, env = m.get('execution') or {}, m.get('environment') or {}
        performed = row['rollout_performed']
        reasons = m['status_reasons']
        result.append(dict(
            case=row['case_id'], case_role=row['case_role'], method=row['method'],
            candidate_available=row['candidate_available'], plan_valid=row['plan_valid'],
            rollout_performed=performed, diagnostic_rollout_plan_invalid=m['diagnostic_rollout_plan_invalid'],
            collision=env.get('physical_overlap'), clearance_valid=env.get('clearance_valid'),
            known_workspace=env.get('workspace_known'), motion_valid=e.get('motion_limits_pass'),
            controller_valid=e.get('controller_validity_pass'),
            goal_position_valid=None if not performed else 'GOAL_POSITION_FAILURE' not in reasons,
            goal_yaw_valid=None if not performed else 'GOAL_YAW_FAILURE' not in reasons,
            goal_dwell_valid=e.get('terminal_goal_dwell_pass'),
            terminal_position_error_m=e.get('terminal_position_error_m'),
            terminal_yaw_error_rad=e.get('terminal_yaw_error_rad'),
            minimum_clearance_m=m.get('minimum_clearance_m'),
            rollout_success=row['rollout_success'],
            termination_reasons=';'.join(m['termination_reasons']),
            status_reasons=';'.join(reasons)))
    return result


def compute_summary(run, rows):
    run = Path(run)
    optimization = read(run/'optimization_completed.json')
    starts = [s for s in optimization['starts'] if s['method'].startswith(('M2_', 'M3_'))]
    saved = [read(p) for p in sorted(run.glob('cases/*/methods/M[23]*/starts/*/solver_result.json'))]
    manifest = read(run/'case_manifest.json')
    source, aggregate = read(run/'source.json'), read(run/'aggregate/summary.json')
    gp_cost_fields = ('case_loading_seed_preparation_s', 'derivative_graph_construction_s',
        'compilation_first_call_warmup_s', 'solve_wall_time_s', 'post_solve_validation_time_s',
        'independent_full_validation_time_s', 'cold_setup_solve_validation_s')
    gp_costs = {k: sum(s[k] for s in starts) for k in gp_cost_fields}
    gp_costs['harness_setup_s'] = sum(s['setup_wall_time_s'] for s in saved)
    counts = {k: sum(s[k] for s in saved) for k in ('iterations', 'objective_evaluations',
        'equality_evaluations', 'inequality_evaluations', 'callback_count')}
    counts.update({k: sum(s['derivative_calls'][k] for s in saved) for k in (
        'objective_gradient', 'equality_jacobian', 'inequality_jacobian')})
    counts['primal_cache_misses'] = sum(s['profiling']['evaluator_cache_misses'] for s in saved)
    counts['primal_environment_query_calls'] = sum(s['profiling']['numerical_component_calls']['environment_query'] for s in saved)
    counts['primal_GP_interpolation_calls'] = sum(s['profiling']['numerical_component_calls']['collocation_gp_interpolation'] for s in saved)
    counts['derivative_environment_query_calls'] = sum(s['derivative_provider_stats']['phases']['runtime']['environment_calls'] for s in saved)
    counts['derivative_core_ad_evaluations'] = sum(s['derivative_provider_stats']['phases']['runtime']['core_ad_evaluations'] for s in saved)
    execution = [r['rollout_metrics']['execution'] for r in rows if r['rollout_performed']]
    mpc_wall = read(run/'mpc_batch_timing.json')['wall_seconds']
    seed_checks = sum(r['preparation_timing']['initial_full_checks_s'] for r in manifest['selected'])
    seed_construct = sum(r['preparation_timing']['seed_construction_s'] for r in manifest['selected'])
    return dict(
        measured_scope='saved measured numerical phases; excludes human/tool idle time, plotting, GUI, tests and independent artifact audit',
        environment_loading_s=dict(preparation=source['shared_preparation_environment_loading_s'],
            optimization=optimization['shared_environment_loading_s'], evaluation=aggregate['shared_evaluation_environment_loading_s']),
        seed_construction_s=seed_construct, initial_full_checks_s=seed_checks,
        gp_all_16_starts=gp_costs, gp_evaluation_counts=counts,
        method_costs=optimization['method_timings'],
        offline_optimization_phase_wall_s=optimization['offline_optimization_phase_wall_s'],
        official_mpc_batch_wall_s=mpc_wall,
        official_mpc_rollout_solve_s=sum(e['mpc_official_solve_total_s'] for e in execution),
        official_mpc_rollout_solve_count=sum(e['mpc_solve_count_total'] for e in execution),
        outcome_evaluation_wall_s=aggregate['evaluation_phase_wall_s'],
        optimization_mpc_evaluation_phase_wall_sum_s=optimization['offline_optimization_phase_wall_s']+mpc_wall+aggregate['evaluation_phase_wall_s'],
        compilation_policy='fresh CPU float64 provider per start; no compilation reuse; warmup outside each 30 s prepared solve budget',
        seed_only_cost='no optimization; construction and original full acceptance costs included in shared preparation, not zero computational cost',
        accounting='method totals include two starts, compilation, validation and persistence; GP phase subtotals nest within those totals; MPC solve time nests within MPC batch; do not add nested levels',
        source_hashes={name:digest(run/name) for name in ('source.json','case_manifest.json',
            'optimization_completed.json','mpc_batch_timing.json','aggregate/method_results.json','aggregate/summary.json')})


def report(run, gui=None):
    # The independent validator also recomputes this report's required counts.
    from validate_gp_se2_02 import expected_final_numerical
    run = Path(run).resolve()
    summary = read(run/'aggregate/summary.json')
    rows = read(run/'aggregate/method_results.json')
    matrix = outcome_matrix(rows)
    with (run/'aggregate/outcome_matrix.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(matrix[0]))
        writer.writeheader()
        writer.writerows(matrix)
    write(run/'aggregate/compute_summary.json', compute_summary(run, rows))
    gui_record = dict(status='GUI_RUNTIME_NOT_VALIDATED', runtime_validation_path=None, sha256=None)
    if gui is not None:
        path = Path(gui).resolve()/'runtime_validation.json'
        evidence = read(path)
        if evidence['status'] != 'GP_SE2_02_COMPARISON_REPLAY_RUNTIME_VALIDATED' or Path(evidence['experiment_run']).resolve() != run:
            raise ValueError('renderer evidence does not validate this saved experiment')
        gui_record = dict(status=evidence['status'], runtime_validation_path=str(path), sha256=digest(path))
    value = dict(schema_version=1, experiment='GP-SE2-02',
        operational_status='GP_SE2_02_COMPLETED' if gui is not None else 'GP_SE2_02_COMPLETED_WITH_LIMITATIONS',
        execution_interpretation=summary['execution_interpretation'],
        actual_rollout_count=summary['actual_rollout_count'],
        aggregate_summary_sha256=digest(run/'aggregate/summary.json'),
        experiment_freeze_sha256=digest(run/'experiment_freeze.json'),
        experiment_git_sha=read(run/'experiment_freeze.json')['experiment_git_sha'],
        original_summary_operational_status=summary['operational_status'],
        validation_requirement='authoritative validation.json must pass',
        numerical=expected_final_numerical(run, summary), gui=gui_record,
        outcome_matrix_sha256=digest(run/'aggregate/outcome_matrix.csv'),
        compute_summary_sha256=digest(run/'aggregate/compute_summary.json'),
        interim_status_explanation='aggregate/summary.json preserves the evaluation-stage status before renderer/artifact validation; this completion report supersedes only that status, not numerical outcomes',
        inference_scope='offline development-corpus diagnostic transfer; no online latency or held-out navigation generalization claim',
        next_single_experiment='isolate F_native versus F_common pose-reference preparation on the fixed large-turn event with unchanged controller and original goal; do not optimize again')
    write(run/'final_report.json', value)
    return value


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True, type=Path)
    parser.add_argument('--gui', type=Path)
    args = parser.parse_args()
    print(json.dumps(report(args.run, args.gui), indent=2))

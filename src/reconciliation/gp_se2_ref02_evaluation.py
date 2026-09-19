"""REF-02 independent execution outcomes with unchanged physical acceptance.

The MPC calculation is unchanged; C changes reference selection. This module
contains no trajectory optimizer or MPC solve and does not disguise method IDs.
"""
from __future__ import annotations
import time
import numpy as np
from .gp_se2_evaluation import evaluate_rollout
from .gp_se2_02_evaluation import validate_counterfactual, rollout_failure_reasons
from .gp_se2_rollout import dense_rollout_samples
from .gp_se2_ref01_evaluation import CONTRAST_METRICS, REPRODUCTION_TOLERANCES, reproduction_check, selection_diagnostics
from .se2 import wrap_angle

METHODS = ('A_NATIVE', 'B_DENSE_ROW_STEP', 'C_DENSE_SOURCE_PROGRESS')
FIXED_CASES = ('episode_014_repeat_01/handoff_026', 'episode_001_repeat_01/handoff_002')

def evaluate_method(method, rollout, context, reference, goal_route, env, config):
    """Use unchanged rollout conjunction; add separately named diagnostic metrics."""
    if method not in METHODS:
        raise ValueError('unknown REF-02 method')
    begin = time.perf_counter()
    validation = validate_counterfactual(rollout, context, reference)
    outcome = evaluate_rollout(rollout, goal_route, env, config)
    times, poses = dense_rollout_samples(rollout)
    goal = np.asarray(goal_route['goal_world'], float)
    signed_yaw = wrap_angle(poses[:, 2]-goal[2])
    unwrapped = np.unwrap(poses[:, 2])
    solves = rollout['controller_reference_selections']
    execution = outcome['execution']
    # Report achieved terminal dwell independently of the REQUIRED .20s field.
    inside = (np.linalg.norm(poses[:, :2]-goal[:2], axis=1) <= execution['goal_position_tolerance_m']) & (
        np.abs(signed_yaw) <= execution['goal_yaw_tolerance_rad'])
    trailing = 0
    for flag in inside[::-1]:
        if not flag:
            break
        trailing += 1
    achieved = float(times[-1]-times[len(times)-trailing]) if trailing else 0.
    selected = selection_diagnostics(solves)
    summary = {key: execution.get(key) for key in CONTRAST_METRICS}
    summary.update(
        case_id=context['case_id'], method=method, primary_success=bool(outcome['primary_success']),
        rollout_success=bool(outcome['primary_success']),
        terminal_goal_dwell_pass=execution['terminal_goal_dwell_pass'],
        terminal_goal_dwell_required_s=execution['terminal_goal_dwell_s'],
        terminal_achieved_goal_dwell_sampled_s=achieved,
        terminal_achieved_goal_dwell_definition='duration of final consecutive in-goal dense samples; sampled diagnostic, not a changed success predicate',
        terminal_signed_yaw_error_rad=float(signed_yaw[-1]),
        terminal_yaw_error_deg=float(np.rad2deg(execution['terminal_yaw_error_rad'])),
        first_goal_entry_time_s=execution['time_to_goal_s'],
        minimum_clearance_m=outcome['minimum_clearance_m'],
        motion_limits_pass=execution['motion_limits_pass'],
        motion_violation=not execution['motion_limits_pass'],
        controller_failure_count=execution['controller_failure_count'],
        actual_accumulated_yaw_rad=float(unwrapped[-1]-unwrapped[0]),
        actual_yaw_total_variation_rad=float(np.abs(np.diff(unwrapped)).sum()),
        final_control_input_goal_yaw_error_rad=float(wrap_angle(solves[-1]['input_pose_world'][2]-goal[2])),
        final_applied_v_mps=float(solves[-1]['command'][0]),
        final_applied_omega_radps=float(solves[-1]['command'][1]),
        primary_mpc_solve_count=len(solves),
        failure_reasons=rollout_failure_reasons(outcome),
        final_target_first_in_horizon_time_s=selected['final_target_first_in_horizon_time_s'],
        first_horizon_xy_arc_length_m=selected['first_horizon_xy_arc_length_m'],
        mean_horizon_xy_arc_length_m=selected['mean_horizon_xy_arc_length_m'],
        mean_pose_to_last_target_distance_m=selected['mean_pose_to_last_target_distance_m'],
    )
    outcome.update(case_id=context['case_id'], method=method, rollout_success=outcome['primary_success'],
                   rollout_input_validation=validation, summary=summary, selection_diagnostics=selected,
                   terminal_achieved_goal_dwell_sampled_s=achieved,
                   independent_evaluation_wall_s=time.perf_counter()-begin,
                   GP_or_rigid_optimization_performed=False, new_VLA_updates=0)
    return outcome


def paired_comparisons(rows):
    """All fixed pairs remain visible; absent times produce absent contrasts."""
    cases = {}
    for item in rows:
        row = item.get('summary', item)
        case = cases.setdefault(row['case_id'], {})
        if row['method'] not in METHODS or row['method'] in case:
            raise ValueError('unknown or duplicate method')
        case[row['method']] = row
    if set(cases) != set(FIXED_CASES) or any(set(v) != set(METHODS) for v in cases.values()):
        raise ValueError('all two cases and three methods must remain present')
    result = []
    for case_id in FIXED_CASES:
        case = cases[case_id]
        for baseline, method in [('B_DENSE_ROW_STEP', 'C_DENSE_SOURCE_PROGRESS'),
                                 ('A_NATIVE', 'C_DENSE_SOURCE_PROGRESS'), ('A_NATIVE', 'B_DENSE_ROW_STEP')]:
            a, b = case[baseline], case[method]
            for metric in CONTRAST_METRICS:
                x, y = a[metric], b[metric]
                result.append(dict(case_id=case_id, baseline=baseline, method=method,
                    baseline_success=a['primary_success'], method_success=b['primary_success'],
                    both_success=bool(a['primary_success'] and b['primary_success']), metric=metric,
                    baseline_value=x, method_value=y, difference=None if x is None or y is None else float(y-x),
                    interpretation='fixed-event descriptive difference; a failure is not a quality improvement'))
    return result


def selection_mechanism(records):
    result = selection_diagnostics(records)
    rows = [r.get('selection_diagnostic', r) for r in records]
    final = [bool(r['final_goal_row_in_horizon']) for r in rows]
    first = next((i for i, flag in enumerate(final) if flag), None)
    result.update(goal_in_horizon=final,
        final_target_stays_after_first=None if first is None else all(final[first:]),
        final_target_absent_after_first_count=None if first is None else sum(not x for x in final[first:]),
        endpoint_repetition_counts=[r['endpoint_repetition_count'] for r in rows],
        nearest_indices=[r['nearest_index'] for r in rows],
        nearest_original_progress=[r['nearest_original_fractional_row_coordinate'] for r in rows])
    return result


def compare_selections(case_id, records, *, matched):
    """Compare installed-row targets, never assume Native and dense nearest agree."""
    result = []
    for baseline, method in [('B_DENSE_ROW_STEP', 'C_DENSE_SOURCE_PROGRESS'),
                             ('A_NATIVE', 'C_DENSE_SOURCE_PROGRESS'), ('A_NATIVE', 'B_DENSE_ROW_STEP')]:
        for i in range(30):
            if matched:
                a, b = records[i]['methods'][baseline], records[i]['methods'][method]
                t = records[i]['time_s']; command_difference = pose_difference = None
            else:
                first, second = records[baseline][i], records[method][i]
                a, b = first['selection_diagnostic'], second['selection_diagnostic']; t = first['time_s']
                command_difference = (np.asarray(second['command'])-first['command']).tolist()
                pose_difference = np.asarray(second['input_pose_world'])-first['input_pose_world']
                pose_difference[2] = wrap_angle(pose_difference[2]); pose_difference = pose_difference.tolist()
            x, y = np.asarray(a['reference_world']), np.asarray(b['reference_world'])
            delta = y-x; delta[:, 2] = wrap_angle(delta[:, 2])
            progress = np.asarray(b['selected_original_fractional_row_coordinates'])-a['selected_original_fractional_row_coordinates']
            nearest_equal = a['nearest_index'] == b['nearest_index'] and a['nearest_original_fractional_row_coordinate'] == b['nearest_original_fractional_row_coordinate']
            costs_equal = a['weighted_total_pose_distances'] == b['weighted_total_pose_distances']
            if matched and baseline == 'B_DENSE_ROW_STEP' and not (nearest_equal and costs_equal):
                raise ValueError('B/C matched-state nearest or cost changed')
            result.append(dict(case_id=case_id, baseline=baseline, method=method, time_s=t,
                same_input_pose=matched, same_nearest_index_and_source_progress=nearest_equal,
                same_all_nearest_costs=costs_equal,
                selected_indices_baseline=a['indices'], selected_indices_method=b['indices'],
                max_selected_xy_difference_m=float(np.max(np.linalg.norm(delta[:, :2], axis=1))),
                max_selected_yaw_difference_rad=float(np.max(np.abs(delta[:, 2]))),
                selected_original_progress_difference=progress.tolist(),
                baseline_goal_in_horizon=a['final_goal_row_in_horizon'], method_goal_in_horizon=b['final_goal_row_in_horizon'],
                selection_changed=bool(np.max(np.abs(delta)) > 1e-12 or np.max(np.abs(progress)) > 1e-12),
                command_difference=command_difference, input_pose_difference=pose_difference,
                interpretation='same-state selector intervention' if matched else 'closed-loop includes feedback state divergence'))
    return result


def first_command_divergences(case_id, actual, *, atol=1e-6):
    result = []
    for baseline, method in [('B_DENSE_ROW_STEP', 'C_DENSE_SOURCE_PROGRESS'), ('A_NATIVE', 'C_DENSE_SOURCE_PROGRESS')]:
        a, b = actual[baseline], actual[method]
        delta = np.asarray([r['command'] for r in b])-np.asarray([r['command'] for r in a])
        first = lambda mask: next((float(row['time_s']) for row, yes in zip(a, mask) if yes), None)
        result.append(dict(case_id=case_id, baseline=baseline, method=method,
            atol=atol, first_divergent_command_time_s=first(np.any(np.abs(delta) > atol, axis=1)),
            first_divergent_linear_command_time_s=first(np.abs(delta[:, 0]) > atol),
            first_divergent_angular_command_time_s=first(np.abs(delta[:, 1]) > atol),
            differences=delta.tolist()))
    return result


def result_flags(outcomes, reproductions, matched_pairs, *, fixed_dense_preserved, selection_validated):
    by = {(r['case_id'], r['method']): r for r in outcomes}
    if set(by) != {(c, m) for c in FIXED_CASES for m in METHODS}:
        raise ValueError('missing planned outcome')
    baseline_reproduced = all(reproductions[c]['passed'] for c in FIXED_CASES)
    pairs = [r for r in matched_pairs if r['baseline'] == 'B_DENSE_ROW_STEP']
    nearest = len(pairs) == 60 and all(r['same_nearest_index_and_source_progress'] and r['same_all_nearest_costs'] for r in pairs)
    prereqs = baseline_reproduced and fixed_dense_preserved and selection_validated and nearest
    safety = ('COLLISION', 'CLEARANCE_VIOLATION', 'UNKNOWN_WORKSPACE', 'MOTION_VIOLATION', 'CONTROLLER_FAILURE')
    regressions = []
    for case in FIXED_CASES:
        b, c = by[case, METHODS[1]], by[case, METHODS[2]]
        new = [reason for reason in c['failure_reasons'] if reason in safety and reason not in b['failure_reasons']]
        if new:
            regressions.append(dict(case_id=case, reasons=new))
    b, c = [by[FIXED_CASES[0], m] for m in METHODS[1:]]
    recovered = bool(prereqs and not b['primary_success'] and c['primary_success'])
    if not prereqs:
        status = 'INTERVENTION_NOT_CONFIRMED'
    elif any(r['case_id'] == FIXED_CASES[0] for r in regressions):
        status = 'REGRESSION_OR_TRADEOFF'
    elif recovered:
        status = 'FULL_FIXED_EVENT_RECOVERY'
    elif c['terminal_yaw_error_rad'] < b['terminal_yaw_error_rad']-1e-12:
        status = 'PARTIAL_RECOVERY'
    else:
        status = 'NO_RECOVERY_OBSERVED'
    return dict(baseline_reproduced=baseline_reproduced,
        fixed_dense_reference_preserved=bool(fixed_dense_preserved), matched_state_nearest_preserved=nearest,
        source_progress_selection_validated=bool(selection_validated), large_turn_success_recovered=recovered,
        benign_success_preserved=bool(prereqs and by[FIXED_CASES[1], METHODS[1]]['primary_success'] and by[FIXED_CASES[1], METHODS[2]]['primary_success']),
        new_safety_or_motion_regression=bool(regressions), safety_motion_regressions=regressions,
        large_turn_result=status,
        mechanism_evidence_level='FIXED_GEOMETRY_MATCHED_NEAREST_SELECTION_COMMAND_OUTCOME' if prereqs else 'INTERVENTION_NOT_CONFIRMED',
        scope='fixed-event offline reference-selector intervention; not GP or general navigation improvement')

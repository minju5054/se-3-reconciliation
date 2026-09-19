"""REF-01 outcomes and descriptive contrasts; original success predicates reused.

This module contains no controller or trajectory optimizer. Rows, not their
interpolation timestamps, are submitted to the unchanged official MPC.
"""
from __future__ import annotations

from copy import deepcopy
import time

import numpy as np

from .gp_se2_evaluation import evaluate_rollout
from .gp_se2_02_evaluation import validate_counterfactual, rollout_failure_reasons
from .gp_se2_rollout import dense_rollout_samples
from .se2 import wrap_angle

VARIANTS = ('R00_NATIVE', 'R10_SUFFIX_ONLY', 'R01_RESAMPLE_ONLY', 'R11_CURRENT_ADAPTER')
FIXED_CASES = ('episode_014_repeat_01/handoff_026', 'episode_001_repeat_01/handoff_002')
REPRODUCTION_TOLERANCES = {
    'selected_reference_atol': 1e-12,
    'command_atol': 1e-6,
    'state_atol': 1e-6,
    'endpoint_metric_atol': 1e-6,
    'rtol': 0.,
    'rationale': ('selected references and commands reuse the original historical-audit '
                  '1e-12/1e-6 tolerances; state and endpoint diagnostics additionally use '
                  '1e-6 absolute units (metres/radians), frozen before primary. These '
                  'diagnostic comparisons do not change physical acceptance thresholds.'),
}
CONTRAST_METRICS = (
    'terminal_position_error_m', 'terminal_signed_yaw_error_rad',
    'terminal_yaw_error_rad', 'terminal_yaw_error_deg', 'time_to_goal_s',
    'minimum_clearance_m', 'command_total_variation_v_mps',
    'command_total_variation_omega_radps', 'first_command_delta_v_mps',
    'first_command_delta_omega_radps', 'control_grid_max_abs_acceleration_v_mps2',
    'control_grid_max_abs_acceleration_omega_radps2', 'actual_accumulated_yaw_rad',
    'actual_yaw_total_variation_rad', 'path_length_m',
    'final_target_first_in_horizon_time_s', 'first_horizon_xy_arc_length_m',
    'mean_horizon_xy_arc_length_m', 'mean_pose_to_last_target_distance_m',
    'terminal_achieved_goal_dwell_sampled_s',
)


def selection_diagnostics(records):
    """Summarize measured next-row targets without calling an optimizer."""
    rows = [r.get('selection_diagnostic', r) for r in records]
    times = [float(r.get('time_s', i/10)) for i, r in enumerate(records)]
    if not rows:
        raise ValueError('selection records must be nonempty')
    def val(row, *keys):
        for key in keys:
            if key in row:
                return row[key]
        raise KeyError(keys[0])
    first_progress = np.asarray([val(r, 'selected_original_fractional_row_coordinates')[0] for r in rows])
    last_progress = np.asarray([val(r, 'selected_original_fractional_row_coordinates')[-1] for r in rows])
    final = [bool(val(r, 'final_goal_row_in_horizon')) for r in rows]
    arcs = [float(val(r, 'horizon_xy_arc_length_m')) for r in rows]
    distances = [float(val(r, 'selected_last_target_distance_m')) for r in rows]
    progress_steps = np.diff(first_progress)
    return {
        'final_target_first_in_horizon_time_s': next((t for t, yes in zip(times, final) if yes), None),
        'final_target_absent_times_s': [t for t, yes in zip(times, final) if not yes],
        'selected_first_original_progress': first_progress.tolist(),
        'selected_last_original_progress': last_progress.tolist(),
        'selected_first_progress_deltas': progress_steps.tolist(),
        'backward_progress_count': int(np.count_nonzero(progress_steps < -1e-12)),
        'maximum_forward_progress_jump': float(max(0., np.max(progress_steps))) if len(progress_steps) else 0.,
        'maximum_backward_progress_jump': float(min(0., np.min(progress_steps))) if len(progress_steps) else 0.,
        'first_horizon_xy_arc_length_m': arcs[0],
        'mean_horizon_xy_arc_length_m': float(np.mean(arcs)),
        'mean_pose_to_last_target_distance_m': float(np.mean(distances)),
        'time_s': times,
        'horizon_xy_arc_lengths_m': arcs,
        'horizon_yaw_spans_rad': [float(val(r, 'horizon_yaw_span_rad')) for r in rows],
        'definition': 'original fractional row coordinate is row-order progress, not time or distance',
    }


def evaluate_variant(variant, rollout, context, reference, goal_route, env, config):
    """Use unchanged rollout conjunction; add separately named diagnostic metrics."""
    if variant not in VARIANTS:
        raise ValueError('unknown REF-01 variant')
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
        case_id=context['case_id'], variant=variant, primary_success=bool(outcome['primary_success']),
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
    outcome.update(case_id=context['case_id'], variant=variant, rollout_success=outcome['primary_success'],
                   rollout_input_validation=validation, summary=summary, selection_diagnostics=selected,
                   terminal_achieved_goal_dwell_sampled_s=achieved,
                   independent_evaluation_wall_s=time.perf_counter()-begin,
                   GP_or_rigid_optimization_performed=False, new_VLA_updates=0)
    return outcome


def _array_comparison(a, b, atol, *, wrapped_last=False):
    left, right = np.asarray(a), np.asarray(b)
    if left.shape != right.shape:
        return dict(shape_match=False, bitwise_equal=False, numerical_agreement=False,
                    maximum_absolute_error=None, atol=atol)
    error = left.astype(float)-right.astype(float)
    if wrapped_last and error.size:
        error[..., -1] = wrap_angle(error[..., -1])
    maximum = float(np.max(np.abs(error))) if error.size else 0.
    return dict(shape_match=True, bitwise_equal=bool(left.dtype == right.dtype and left.tobytes() == right.tobytes()),
                numerical_agreement=maximum <= atol, maximum_absolute_error=maximum, atol=atol)


def reproduction_check(new_rollout, old_rollout, new_metrics, old_metrics):
    """Compare first primary execution to historical evidence, without retries."""
    checks = {}
    for key in ('initial_pose_world', 'initial_physical_command', 'initial_previous_control', 'candidate_world'):
        checks[key] = _array_comparison(new_rollout[key], old_rollout[key], 0.)
    for key in ('horizon_s', 'control_hz', 'integration_hz'):
        checks[key] = dict(numerical_agreement=new_rollout[key] == old_rollout[key],
                           bitwise_equal=new_rollout[key] == old_rollout[key])
    for key in ('official_settings_sha256', 'resolved_limits', 'official_gains', 'previous_control_policy', 'failure_policy'):
        if key in new_rollout or key in old_rollout:
            same = new_rollout.get(key) == old_rollout.get(key)
            checks[key] = dict(numerical_agreement=same, bitwise_equal=same)
    capture_new = new_rollout['candidate_frame_transform']['capture_pose_world']
    capture_old = old_rollout['candidate_frame_transform']['capture_pose_world']
    checks['original_capture_pose_world'] = _array_comparison(capture_new, capture_old, 0.)
    for field, tolerance in (('input_pose_world', 'state_atol'), ('previous_control', 'command_atol')):
        checks['solve_'+field] = _array_comparison(
            [r[field] for r in new_rollout['controller_reference_selections']],
            [r[field] for r in old_rollout['controller_reference_selections']],
            REPRODUCTION_TOLERANCES[tolerance], wrapped_last=field == 'input_pose_world')
    checks['solve_success_flags'] = _array_comparison(
        [r['success'] for r in new_rollout['controller_reference_selections']],
        [r['success'] for r in old_rollout['controller_reference_selections']], 0.)
    ns, os = [r['selection'] for r in new_rollout['controller_reference_selections']], [r['selection'] for r in old_rollout['controller_reference_selections']]
    checks['selected_indices'] = _array_comparison([r['indices'] for r in ns], [r['indices'] for r in os], 0.)
    checks['nearest_indices'] = _array_comparison([r['nearest_index'] for r in ns], [r['nearest_index'] for r in os], 0.)
    checks['selected_reference_world_modulo_yaw'] = _array_comparison([r['reference_world'] for r in ns], [r['reference_world'] for r in os], REPRODUCTION_TOLERANCES['selected_reference_atol'], wrapped_last=True)
    checks['selected_reference_world_literal_unwrapped'] = _array_comparison([r['reference_world'] for r in ns], [r['reference_world'] for r in os], REPRODUCTION_TOLERANCES['selected_reference_atol'])
    checks['control_commands'] = _array_comparison([r['command'] for r in new_rollout['controller_reference_selections']], [r['command'] for r in old_rollout['controller_reference_selections']], REPRODUCTION_TOLERANCES['command_atol'])
    checks['held_commands'] = _array_comparison([r['command'] for r in new_rollout['commands']], [r['command'] for r in old_rollout['commands']], REPRODUCTION_TOLERANCES['command_atol'])
    checks['state_sequence'] = _array_comparison([r['pose_world'] for r in new_rollout['states']], [r['pose_world'] for r in old_rollout['states']], REPRODUCTION_TOLERANCES['state_atol'], wrapped_last=True)
    predictions = [r['prediction_world'] for r in new_rollout['controller_reference_selections']], [r['prediction_world'] for r in old_rollout['controller_reference_selections']]
    if any(p is None for side in predictions for p in side):
        checks['prediction_sequence'] = dict(numerical_agreement=predictions[0] == predictions[1], bitwise_equal=predictions[0] == predictions[1], comparison='missing prediction preserved')
    else:
        checks['prediction_sequence'] = _array_comparison(*predictions, REPRODUCTION_TOLERANCES['state_atol'], wrapped_last=True)
    for key in ('terminal_position_error_m', 'terminal_yaw_error_rad', 'time_to_goal_s'):
        a, b = new_metrics['execution'][key], old_metrics['execution'][key]
        checks[key] = (dict(numerical_agreement=a is b, bitwise_equal=a is b, missing_value_preserved=True)
                       if a is None or b is None else _array_comparison([a], [b], REPRODUCTION_TOLERANCES['endpoint_metric_atol']))
    for key in ('goal_reached', 'terminal_goal_dwell_pass', 'motion_limits_pass', 'controller_failure_count'):
        same = new_metrics['execution'][key] == old_metrics['execution'][key]
        checks[key] = dict(numerical_agreement=same, bitwise_equal=same)
    for family, key in (('environment', 'physical_overlap'), ('environment', 'clearance_valid'),
                        ('environment', 'workspace_known'), ('route', 'valid')):
        same = new_metrics[family][key] == old_metrics[family][key]
        checks[family+'_'+key] = dict(numerical_agreement=same, bitwise_equal=same)
    same = new_metrics['primary_success'] == old_metrics['primary_success']
    checks['primary_success'] = dict(numerical_agreement=same, bitwise_equal=same)
    return dict(passed=all(c['numerical_agreement'] for c in checks.values()),
                all_compared_values_bitwise_equal=all(c['bitwise_equal'] for c in checks.values()),
                tolerances=deepcopy(REPRODUCTION_TOLERANCES), checks=checks,
                repetitions=0, comparison='first prescribed primary rollout versus immutable GP-SE2-02 evidence',
                physical_acceptance_tolerances_changed=False)


def factor_contrasts(rows):
    """Fixed-event descriptive 2x2 contrasts, retaining missing values as null."""
    cases = {}
    for raw in rows:
        row = raw.get('summary', raw)
        case = cases.setdefault(row['case_id'], {})
        if row['variant'] not in VARIANTS or row['variant'] in case:
            raise ValueError('unknown or duplicate variant')
        case[row['variant']] = row
    output = []
    for case_id, variants in cases.items():
        if set(variants) != set(VARIANTS):
            raise ValueError('all four planned variants required')
        for metric in CONTRAST_METRICS:
            y = [variants[v].get(metric) for v in VARIANTS]
            def contrast(indices, weights):
                values = [y[i] for i in indices]
                return None if any(x is None for x in values) else float(sum(w*x for w, x in zip(weights, values)))
            output.append(dict(case_id=case_id, metric=metric,
                **{v: value for v, value in zip(VARIANTS, y)},
                suffix_without_resampling=contrast([1, 0], [1, -1]),
                resampling_without_suffix=contrast([2, 0], [1, -1]),
                suffix_with_resampling=contrast([3, 2], [1, -1]),
                resampling_with_suffix=contrast([3, 1], [1, -1]),
                interaction=contrast([3, 1, 2, 0], [1, -1, -1, 1]),
                interpretation='fixed-event descriptive contrast; no population significance'))
    return output

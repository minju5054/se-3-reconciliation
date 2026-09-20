"""DIAG-07 fixed-reference diagnostic, with unchanged historical checkers.

No optimizer or controller implementation lives here. A hard reference always
remains plan-invalid, regardless of its downstream execution result.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from .gp_se2_rollout import candidate_in_capture_frame, counterfactual_rollout
from .online_mpc_adapter import selection_audit
from .robotless_online import integrate_unicycle
from .se2 import local_trajectory_to_world, wrap_angle

HARD = 'episode_013_repeat_01/handoff_024'
BENIGN = 'episode_001_repeat_01/handoff_002'
HARD_LABEL = 'PLAN INVALID: LATERAL ONLY | DIAGNOSTIC EXECUTION | NOT A DEPLOYMENT CANDIDATE'
EXECUTION_LABEL = 'OFFLINE COUNTERFACTUAL | NOT ONLINE NAVIGATION'
REQUIRED_FLAGS = {'boundary_pose', 'initial_twist', 'pose_body_derivative_identity',
                  'lateral_velocity', 'linear_speed', 'angular_speed',
                  'linear_acceleration', 'angular_acceleration', 'original_goal',
                  'original_route', 'original_workspace', 'original_obstacle_clearance'}


def array_hash(value):
    """Exact float64 C-order bytes; caller records shape/dtype separately."""
    return hashlib.sha256(np.ascontiguousarray(value, dtype=np.float64).tobytes()).hexdigest()


def support_reference(problem, saved):
    """Use saved latest, never the retained/selected result for hard records."""
    z = np.asarray(saved['latest_iterate'], dtype=np.float64)
    if z.shape != (150,) or not np.isfinite(z).all():
        raise ValueError('MISSING_OR_INVALID_SAVED_LATEST_VECTOR')
    poses, twists = problem.unpack(z)
    if not np.array_equal(poses, saved['latest_support_poses']) or not np.array_equal(twists, saved['latest_support_twists']):
        raise ValueError('SAVED_SUPPORT_RECONSTRUCTION_MISMATCH')
    if poses.shape != (31, 3) or twists.shape != (31, 3):
        raise ValueError('support dimensions changed')
    return z, poses, twists, poses[1:].copy()


def plan_gate(full, *, hard, changed_from_seed=True):
    flags = full['additional_grid']['flags']
    if set(flags) != REQUIRED_FLAGS:
        raise ValueError('unexpected full-check coverage')
    failed = sorted(k for k, v in flags.items() if not v)
    motion = full['original_plan_report']['motion']['violations']
    failed_motion = sorted(k for k, v in motion.items() if v)
    if hard:
        if failed != ['lateral_velocity'] or failed_motion != ['lateral_velocity'] or full['full_feasible']:
            raise ValueError('NOT_LATERAL_ONLY_ELIGIBLE')
        return dict(reference_kind='LATERAL_ONLY_PLAN_INVALID_REFERENCE', plan_valid=False,
                    plan_failure_reason='lateral_velocity', deployment_candidate=False,
                    diagnostic_execution_authorized=True, diagnostic_execution_only=True,
                    selected_source_in_DIAG06=None, label=HARD_LABEL)
    if not full['full_feasible'] or failed or failed_motion or not changed_from_seed:
        raise ValueError('BENIGN_PLAN_CONTROL_GATE_FAILED')
    return dict(reference_kind='PLAN_VALID_OPTIMIZED_CONTROL_REFERENCE', plan_valid=True,
                plan_failure_reason=None, deployment_candidate=False,
                diagnostic_execution_authorized=True, diagnostic_execution_only=True,
                label='PLAN VALID: BENIGN OPTIMIZED CONTROL')


def deduplicate(rows):
    """Only exact reference bytes AND exact frozen event state may alias."""
    unique, aliases, seen = [], {}, {}
    for row in rows:
        key = (row['case_id'], row['state_identity_sha256'], row['world_reference_sha256'],
               tuple(row['shape']), row['dtype'])
        if key not in seen:
            uid = f'ref_{len(unique):02d}'
            seen[key] = uid
            unique.append(dict(row, unique_reference_id=uid, provenance_labels=[]))
        uid = seen[key]
        aliases[row['reference_id']] = uid
        next(r for r in unique if r['unique_reference_id'] == uid)['provenance_labels'].append(row['reference_id'])
    return unique, aliases


def diagnostic_rollout(module, frozen, reference_world, admission):
    """Compose the unchanged official rollout and rename legacy storage fields.

    This is the ONLY new counterfactual call site. No projection, resampling,
    optimizer or GP velocity feed-forward is performed.
    """
    if not admission['diagnostic_execution_authorized'] or admission['deployment_candidate']:
        raise ValueError('diagnostic admission required')
    result = counterfactual_rollout(module, frozen, reference_world,
                                    horizon_s=3., control_hz=10., integration_hz=60.)
    for old, new in [('candidate_world', 'reference_world'),
                     ('candidate_capture_local', 'reference_capture_local'),
                     ('candidate_frame_transform', 'reference_frame_transform')]:
        result[new] = result.pop(old)
    result.update(admission)
    result['kind'] = EXECUTION_LABEL
    result['actual_lateral_command_dimension'] = 'nonexistent; unicycle command [v, omega]'
    result['GP_body_velocity_feed_forward'] = False
    result['installed_reference_world'] = module.project_body_to_world(
        result['reference_capture_local'], frozen['original_capture_pose_world']).tolist()
    result['installed_reference_provenance'] = 'same pure official projection used by unmodified set_body_path; rollout verifies installation roundtrip'
    return result


def validate_rollout_records(rollout, context, reference):
    """Saved-record check. Never instantiates a tracker or invokes a solve."""
    errors = []
    def check(value, message):
        if not bool(value):
            errors.append(message)
    check(rollout['horizon_s'] == 3. and rollout['control_hz'] == 10. and rollout['integration_hz'] == 60., 'rates')
    check(np.array_equal(rollout['initial_pose_world'], context['B_world']), 'B')
    check(np.array_equal(rollout['initial_physical_command'], context['u_minus']), 'physical command')
    check(np.array_equal(rollout['initial_previous_control'], context['previous_control']), 'controller memory')
    check(array_hash(rollout['reference_world']) == array_hash(reference), 'reference identity')
    local, transform = candidate_in_capture_frame(reference, context['original_capture_pose_world'])
    check(np.array_equal(local, rollout['reference_capture_local']), 'capture inverse')
    check(not transform['B_reanchoring'], 'reanchoring')
    recovered = local_trajectory_to_world(context['original_capture_pose_world'], local)
    check(np.max(np.abs(recovered[:, :2]-np.asarray(reference)[:, :2])) <= 1e-11, 'world XY roundtrip')
    check(np.max(np.abs(wrap_angle(recovered[:, 2]-np.asarray(reference)[:, 2]))) <= 1e-11, 'world yaw roundtrip')
    states, commands, solves = [rollout[k] for k in ('states', 'commands', 'controller_reference_selections')]
    check((len(states), len(commands), len(solves)) == (181, 180, 30), 'coverage')
    if (len(states), len(commands), len(solves)) != (181, 180, 30):
        return errors
    installed = np.asarray(rollout['installed_reference_world'])
    check(np.max(np.abs(installed[:, :2]-np.asarray(reference)[:, :2])) <= 1e-11, 'installed XY')
    check(np.max(np.abs(wrap_angle(installed[:, 2]-np.asarray(reference)[:, 2]))) <= 1e-11, 'installed yaw')
    for k, state in enumerate(states):
        check(state['tick'] == k and state['time_s'] == k/60., f'state time {k}')
    for k, command in enumerate(commands):
        check(command['time_s'] == k/60. and command['end_time_s'] == (k+1)/60., f'command time {k}')
        check(command['solve_index'] == k//6 and command['held'] == bool(k % 6), f'hold schedule {k}')
        check(np.array_equal(command['command'], solves[k//6]['command']), f'held command {k}')
        exact = integrate_unicycle(states[k]['pose_world'], command['command'], 1/60.)
        check(np.array_equal(exact, states[k+1]['pose_world']), f'exact integration {k}')
    for k, solve in enumerate(solves):
        check(solve['tick'] == 6*k and solve['time_s'] == k/10., f'solve time {k}')
        check(np.array_equal(solve['input_pose_world'], states[6*k]['pose_world']), f'solve input {k}')
        previous = context['previous_control'] if k == 0 else solves[k-1]['previous_control_after']
        check(np.array_equal(previous, solve['previous_control']), f'memory chain {k}')
        check(solve['simulation_time_advanced_during_solve_s'] == 0., f'simulation clock {k}')
        try:
            audit = selection_audit(installed, solve['input_pose_world'], solve['selection']['reference_world'],
                                    horizon=5, weights=rollout['official_gains']['Q_WEIGHTS'])
            check(audit == solve['selection'], f'actual target audit {k}')
        except ValueError as exc:
            errors.append(f'target {k}: {exc}')
        pred = solve['prediction_world']
        if pred is not None:
            check(np.shape(pred) == (6, 3), f'prediction shape {k}')
            check(np.max(np.abs(np.asarray(pred)[0, :2]-solve['input_pose_world'][:2])) <= 1e-6, f'prediction start {k}')
    check(rollout['controller_failure_count'] == sum(not s['success'] for s in solves), 'failure ledger')
    check(rollout['new_lightnav_updates'] == 0 and rollout['pose_snaps'] == 0, 'forbidden runtime')
    check(not rollout['deployment_candidate'] and rollout['diagnostic_execution_only'], 'no promotion')
    return errors


def execution_diagnostics(rollout, evaluation, context):
    e = evaluation['execution']
    solves = rollout['controller_reference_selections']
    times = np.asarray(evaluation['dense_times_s'])
    poses = np.asarray(evaluation['dense_poses_world'])
    goal = np.asarray(evaluation['goal'])
    cfg = e
    position = np.linalg.norm(poses[:, :2]-goal[:2], axis=1)
    yaw = wrap_angle(poses[:, 2]-goal[2])
    inside = (position <= cfg['goal_position_tolerance_m']) & (np.abs(yaw) <= cfg['goal_yaw_tolerance_rad'])
    outside = np.flatnonzero(~inside)
    start = None if not inside[-1] else (0 if not len(outside) else int(outside[-1]+1))
    achieved = 0. if start is None else float(times[-1]-times[start])
    inclusion = [len(rollout['reference_world'])-1 in s['selection']['indices'] for s in solves]
    first = next((s['time_s'] for s, yes in zip(solves, inclusion) if yes), None)
    targets = np.asarray([s['selection']['reference_world'] for s in solves])
    cmd = np.asarray([s['command'] for s in solves])
    delta = np.diff(np.vstack([context['u_minus'], cmd]), axis=0)
    failures = []
    for condition, name in [(not e['goal_reached'], 'EXECUTION_GOAL_FAILURE'),
            (not e['terminal_goal_dwell_pass'], 'EXECUTION_DWELL_FAILURE'),
            (not e['motion_limits_pass'], 'EXECUTION_MOTION_FAILURE'),
            (not evaluation['environment']['clearance_valid'], 'EXECUTION_CLEARANCE_FAILURE'),
            (not evaluation['route']['valid'], 'EXECUTION_ROUTE_FAILURE'),
            (not evaluation['environment']['workspace_known'], 'EXECUTION_UNKNOWN_WORKSPACE'),
            (bool(e['controller_failure_count']), 'EXECUTION_CONTROLLER_FAILURE')]:
        if condition:
            failures.append(name)
    return dict(failure_taxonomy=failures, cause_attribution='not identified by this fixed-reference diagnostic',
        first_command=cmd[0].tolist(), first_delta_from_physical=delta[0].tolist(),
        first_delta_from_controller_memory=(cmd[0]-context['previous_control']).tolist(),
        final_command=cmd[-1].tolist(), achieved_sampled_terminal_dwell_s=achieved,
        required_terminal_dwell_s=.20, first_goal_entry_s=float(times[np.flatnonzero(inside)[0]]) if inside.any() else None,
        first_final_row_in_horizon_s=first, final_row_maintained_after_first=all(inclusion[int(round(first*10)):]) if first is not None else None,
        final_goal_pose_equals_reference_endpoint=bool(np.array_equal(goal,rollout['reference_world'][-1])),
        first_goal_region_in_horizon_s=next((s['time_s'] for s,t in zip(solves,targets) if np.any(
            (np.linalg.norm(t[:,:2]-goal[:2],axis=1)<=cfg['goal_position_tolerance_m']) &
            (np.abs(wrap_angle(t[:,2]-goal[2]))<=cfg['goal_yaw_tolerance_rad']))),None),
        endpoint_note='goal stays original raw FRESH endpoint; row inclusion refers to final GP reference row',
        selection=dict(times_s=[s['time_s'] for s in solves], nearest_indices=[s['selection']['nearest_index'] for s in solves],
            selected_indices=[s['selection']['indices'] for s in solves], target_world=targets.tolist(),
            final_reference_row_in_horizon=inclusion,
            nearest_backward_jumps=sum(b<a for a,b in zip([s['selection']['nearest_index'] for s in solves][:-1], [s['selection']['nearest_index'] for s in solves][1:])),
            selected_yaw_span_rad=np.ptp(targets[:, :, 2], axis=1).tolist()),
        command_times_s=[s['time_s'] for s in solves], command_values=cmd.tolist(),
        control_grid_acceleration=(delta/.1).tolist(),
        error_times_s=times.tolist(), position_error_m=position.tolist(), signed_goal_yaw_error_rad=yaw.tolist(),
        actual_lateral_command_dimension='nonexistent; not a measured zero lateral channel')


def experiment_interpretation(rows):
    hard = [r for r in rows if r['case_id'] == HARD]
    benign = [r for r in rows if r['case_id'] == BENIGN]
    if len(benign) != 1 or not benign[0]['primary_success']:
        return 'EXECUTION_HARNESS_OR_CONTROL_REGRESSION'
    passed = sum(r['primary_success'] for r in hard)
    if not hard:
        return 'INSUFFICIENT_SAVED_EVIDENCE'
    if passed == 0:
        return 'LATERAL_INVALID_REFERENCES_EXECUTION_FAIL'
    if passed != len(hard):
        return 'MIXED_LATERAL_EXECUTION_RESULT'
    return 'STRICT_PLAN_LATERAL_REJECTION_CONSERVATIVE_ON_FIXED_EVENT'

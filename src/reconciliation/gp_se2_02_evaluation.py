"""Frozen hard-transfer evaluation using the unchanged GP-SE2-01 MPC policy.

No optimizer, derivative, reference adapter, controller, or acceptance rule is
implemented here. This layer keeps candidate availability, plan validity and
newly computed execution outcomes separate, including diagnostic baseline
rollouts whose spatial reference fails the independent plan check.
"""
from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import time

import numpy as np

from .gp_se2_evaluation import evaluate_plan, evaluate_rollout
from .gp_se2_rollout import candidate_in_capture_frame
from .robotless_online import integrate_unicycle
from .se2 import wrap_angle


METHODS = ('M0_NATIVE', 'M0_ADAPTER', 'M1_RIGID',
           'M2_GP_NO_OBSTACLE', 'M3_GP_CONSTRAINED', 'SEED_ONLY')
GP_METHODS = METHODS[3:5]
FULL_CHECK_METHODS = (*GP_METHODS, 'SEED_ONLY')
FIXED_CASES = {
    'episode_001_repeat_01/handoff_002': 'BENIGN_CONTROL',
    'episode_013_repeat_01/handoff_024': 'HARD_POSITION_AND_DIRECTION',
    'episode_014_repeat_01/handoff_026': 'HARD_LARGE_TURN',
    'episode_000_repeat_01/handoff_019': 'HARD_POSITION_STRAIGHT',
}
PAIRS = [('M0_NATIVE', 'M0_ADAPTER')]
PAIRS += [(baseline, gp) for gp in GP_METHODS
          for baseline in ('M0_NATIVE', 'M0_ADAPTER', 'M1_RIGID', 'SEED_ONLY')]
PAIRS += [('M2_GP_NO_OBSTACLE', 'M3_GP_CONSTRAINED')]


def case_scope(case):
    """Report a gate as unsupported; never drop it or replace the fixed case."""
    case_id = case['context']['case_id']
    if case_id not in FIXED_CASES:
        raise ValueError('event is outside the four fixed GP-SE2-02 cases')
    gates = case['goal_route']['gates']
    return dict(case_id=case_id, case_role=FIXED_CASES[case_id],
                status='UNSUPPORTED_GATE_CASE' if gates else 'NO_GATE_SUPPORTED_SCOPE',
                required_gate_count=len(gates), gates_preserved=deepcopy(gates),
                case_replacement=False, held_out_benchmark=False)


def _method(method):
    if method not in METHODS:
        raise ValueError('unknown comparison method: '+str(method))


def plan_for_method(case, method, candidate, solver_result=None, full_acceptance=None):
    """Reuse spatial baseline checks or an already computed original full check.

    A seed is checked as the same continuous-time GP representation, not as a
    raw spatial reference. A full-check failure is preserved even if a simpler
    plan subcheck passes. This function never selects a replacement candidate.
    """
    _method(method)
    problem = case['problem']
    if candidate is None:
        return dict(plan_valid=False, status='NO_CANDIDATE', candidate_available=False,
                    motion_applicable=method in FULL_CHECK_METHODS, environment=None,
                    route=None, goal=None, deformation=None, full_acceptance=None,
                    acceptance_source='candidate absent; no RAW fallback')
    candidate = np.asarray(candidate, dtype=np.float64)
    if candidate.ndim != 2 or candidate.shape[1] != 3 or not np.isfinite(candidate).all():
        raise ValueError('candidate must be finite Nx3 world poses')
    if method == 'M0_NATIVE':
        native = np.load(Path(case['case_directory'])/'F_native.npy', allow_pickle=False)
        if not np.array_equal(native, candidate):
            raise ValueError('M0_NATIVE must preserve the original F_native array')
    if method == 'M0_ADAPTER' and not np.array_equal(candidate, case['common_reference']):
        raise ValueError('M0_ADAPTER must preserve the original F_common array')
    if method in FULL_CHECK_METHODS:
        if full_acceptance is None:
            raise ValueError('GP and SEED_ONLY require the unchanged full acceptance report')
        required = bool(full_acceptance['original_dense_feasible']
                        and full_acceptance['original_plan_valid']
                        and full_acceptance['additional_grid']['valid'])
        if bool(full_acceptance['full_feasible']) != required:
            raise ValueError('inconsistent original full acceptance report')
        plan = deepcopy(full_acceptance['original_plan_report'])
        plan.update(plan_valid=required,
                    status='FULL_FEASIBLE_CANDIDATE_FOUND' if required else 'PLAN_INVALID',
                    full_acceptance=deepcopy(full_acceptance),
                    acceptance_source='unchanged gp_se2_diag_acceptance.check_full_candidate')
    else:
        plan = evaluate_plan(method, candidate, solver_result, case['common_reference'],
                             problem.boundary_pose, problem.initial_twist, case['goal_route'],
                             case['environment'], case['config'])
        plan['acceptance_source'] = 'unchanged GP-SE2-01 spatial reference plan check'
    plan.update(candidate_available=True, method=method,
                motion_state_semantics=('GP body velocity is a planned state; never an MPC feed-forward command'
                                        if method in FULL_CHECK_METHODS else 'spatial reference; continuous GP motion check inapplicable'))
    return plan


def _terminations(solver_result):
    result = solver_result or {}
    starts = result.get('attempts') or [result]
    return list(dict.fromkeys(str(row.get('effective_termination', row.get('termination', row.get('status', 'NOT_OPTIMIZED'))))
                              for row in starts))


def candidate_failure_reasons(candidate_available, solver_result):
    if candidate_available:
        return []
    result = solver_result or {}
    terminations = _terminations(result)
    reasons = ['NO_CANDIDATE']
    if any('DERIVATIVE_UNSUPPORTED' in t for t in terminations):
        reasons.append('DERIVATIVE_UNSUPPORTED')
    if any(t == 'TIMEOUT' for t in terminations):
        reasons.append('TIMEOUT_WITHOUT_VALID_CANDIDATE')
    if any('UNSUPPORTED_GATE_CASE' in t for t in terminations):
        reasons.append('UNSUPPORTED_GATE_CASE')
    return reasons


def rollout_submission(method, candidate, plan, solver_result=None):
    """Only original baselines may use an explicitly diagnostic invalid plan."""
    _method(method)
    available = candidate is not None
    valid = bool(available and plan.get('plan_valid'))
    reasons = candidate_failure_reasons(available, solver_result)
    if available and not valid:
        reasons.append('PLAN_INVALID')
    allowed = bool(available and (valid or method not in FULL_CHECK_METHODS))
    return dict(candidate_available=available, plan_valid=valid,
                rollout_allowed=allowed, diagnostic_rollout_plan_invalid=bool(allowed and not valid),
                missing_reason=None if allowed else ';'.join(reasons),
                status_reasons=reasons, fallback_used=False,
                policy=('full acceptance required before GP/seed rollout' if method in FULL_CHECK_METHODS
                        else 'unchanged GP-SE2-01 baseline policy; available reference may be diagnostic'))


def make_mpc_request(source_root, entries, config):
    """Build input for the unchanged isolated official-MPC batch worker.

    Each entry has context, goal_route and six method mappings containing
    candidate_path, plan and solver_result. Eligibility is extra metadata that
    the original worker ignores; nulls remain null, never a native fallback.
    """
    schedule = config['rollout']
    expected = {'horizon_s': 3., 'control_hz': 10., 'integration_hz': 60.}
    if any(float(schedule[key]) != value for key, value in expected.items()):
        raise ValueError('original fixed counterfactual schedule changed')
    request = dict(source_root=str(Path(source_root).resolve()), audit_only=False, **expected,
                   goal_position_tolerance_m=config['formulation']['goal_position_tolerance'],
                   goal_yaw_tolerance_rad=config['formulation']['goal_yaw_tolerance'],
                   goal_dwell_s=config['evaluation']['terminal_goal_dwell_s'], cases=[])
    seen = set()
    for entry in entries:
        context = entry['context']
        if context['case_id'] in seen:
            raise ValueError('duplicate MPC case')
        seen.add(context['case_id'])
        if set(entry['methods']) != set(METHODS):
            raise ValueError('every planned method must remain in the MPC request')
        methods, eligibility = {}, {}
        for method in METHODS:
            record = entry['methods'][method]
            path = record['candidate_path']
            if path is not None and not Path(path).is_file():
                raise FileNotFoundError(path)
            eligibility[method] = rollout_submission(method, path, record['plan'], record.get('solver_result'))
            methods[method] = str(Path(path).resolve()) if eligibility[method]['rollout_allowed'] else None
        request['cases'].append(dict(episode_id=context['episode_id'], handoff_id=context['handoff_id'],
                                     goal=entry['goal_route']['goal_world'], methods=methods,
                                     rollout_eligibility=eligibility))
    if seen != set(FIXED_CASES):
        raise ValueError('MPC batch must retain exactly the four predetermined cases')
    return request


def validate_counterfactual(rollout, context, candidate):
    """Independently enforce fixed inputs, frame roundtrip and command schedule."""
    for target, source in [('initial_pose_world', 'B_world'), ('initial_physical_command', 'u_minus'),
                           ('initial_previous_control', 'previous_control')]:
        if not np.array_equal(rollout[target], context[source]):
            raise ValueError('counterfactual initial condition changed: '+source)
    candidate = np.asarray(candidate, dtype=np.float64)
    if not np.array_equal(rollout['candidate_world'], candidate):
        raise ValueError('rollout does not use the submitted candidate')
    local, roundtrip = candidate_in_capture_frame(candidate, context['original_capture_pose_world'])
    if not np.allclose(rollout['candidate_capture_local'], local, atol=1e-11, rtol=0):
        raise ValueError('candidate was not converted through the original observation frame')
    transform = rollout['candidate_frame_transform']
    if transform['B_reanchoring'] or not np.array_equal(transform['capture_pose_world'], context['original_capture_pose_world']):
        raise ValueError('candidate reanchoring is forbidden')
    if [rollout[k] for k in ('horizon_s', 'control_hz', 'integration_hz')] != [3., 10., 60.]:
        raise ValueError('fixed counterfactual schedule changed')
    states, commands = rollout['states'], rollout['commands']
    solves = rollout['controller_reference_selections']
    if (len(states), len(commands), len(solves)) != (181, 180, 30):
        raise ValueError('counterfactual must have 181 states, 180 held ticks and 30 fresh MPC solves')
    np.testing.assert_array_equal([s['time_s'] for s in states], np.arange(181)/60)
    np.testing.assert_array_equal([s['time_s'] for s in solves], np.arange(30)*6/60)
    if not np.array_equal(states[0]['pose_world'], context['B_world']):
        raise ValueError('execution does not start at original B')
    if not np.array_equal(solves[0]['previous_control'], context['previous_control']):
        raise ValueError('first MPC solve did not restore the recorded controller memory')
    for i, command in enumerate(commands):
        if command['time_s'] != i/60 or command['end_time_s'] != (i+1)/60:
            raise ValueError('held command time changed')
        if not np.array_equal(command['command'], solves[i//6]['command']):
            raise ValueError('recorded state command is not the actual held MPC command')
        expected_pose = integrate_unicycle(states[i]['pose_world'], command['command'], 1/60)
        delta = np.asarray(states[i+1]['pose_world'])-expected_pose
        delta[2] = wrap_angle(delta[2])
        if np.max(np.abs(delta)) > 1e-11:
            raise ValueError('execution is not exact held-command integration')
    if any(s['simulation_time_advanced_during_solve_s'] != 0 for s in solves):
        raise ValueError('offline solve wall time was added to simulation time')
    if rollout['new_lightnav_updates'] or rollout['pose_snaps']:
        raise ValueError('new updates or pose snapping are outside the protocol')
    return dict(valid=True, original_inputs_preserved=True, schedule_preserved=True,
                original_observation_frame_roundtrip=roundtrip,
                physical_u_minus_and_controller_memory_separately_preserved=True,
                gp_velocity_feedforward=False, execution_command_source='new official MPC solve results')


def rollout_failure_reasons(outcome):
    """Detailed taxonomy without changing the original success conjunction."""
    env, execution, route = outcome['environment'], outcome['execution'], outcome['route']
    reasons = []
    if execution['controller_failure_count']:
        reasons.append('CONTROLLER_FAILURE')
    if env['physical_overlap']:
        reasons.append('COLLISION')
    if not env['clearance_valid']:
        reasons.append('CLEARANCE_VIOLATION')
    if not env['workspace_known']:
        reasons.append('UNKNOWN_WORKSPACE')
    if not execution['motion_limits_pass']:
        reasons.append('MOTION_VIOLATION')
    if execution['terminal_position_error_m'] > execution['goal_position_tolerance_m']:
        reasons.append('GOAL_POSITION_FAILURE')
    if execution['terminal_yaw_error_rad'] > execution['goal_yaw_tolerance_rad']:
        reasons.append('GOAL_YAW_FAILURE')
    if not execution['terminal_goal_dwell_pass']:
        reasons.append('GOAL_DWELL_FAILURE')
    if not route['valid']:
        reasons.append('UNKNOWN_ROUTE' if route['status'] == 'UNKNOWN' else 'ROUTE_VIOLATION')
    return reasons


def evaluate_method(case, method, candidate, plan, solver_result=None, rollout=None):
    """Evaluate one method; absent/invalid GP plans never acquire fake traces."""
    begin = time.perf_counter()
    eligible = rollout_submission(method, candidate, plan, solver_result)
    if rollout is not None and not eligible['rollout_allowed']:
        raise ValueError('GP/seed rollout is forbidden without full acceptance')
    if rollout is None and eligible['rollout_allowed']:
        raise ValueError('missing required independently generated MPC rollout')
    available, valid = eligible['candidate_available'], eligible['plan_valid']
    missing = candidate_failure_reasons(available, solver_result)
    plan_reasons = ['PLAN_INVALID'] if available and not valid else []
    if rollout is None:
        outcome = dict(primary_success=False, failure_reasons=[s.lower() for s in missing+plan_reasons],
                       execution=None, environment=None, route=None, minimum_clearance_m=None,
                       dense_times_s=None, dense_poses_world=None, rollout_input_validation=None)
        execution_reasons = []
    else:
        validation = validate_counterfactual(rollout, case['context'], candidate)
        outcome = evaluate_rollout(rollout, case['goal_route'], case['environment'], case['config'])
        outcome['rollout_input_validation'] = validation
        execution_reasons = rollout_failure_reasons(outcome)
        # A diagnostic observable only. It does not change original goal/dwell acceptance.
        b = np.asarray(case['context']['B_world'])[:2]
        g = np.asarray(case['goal_route']['goal_world'])[:2]
        length = float(np.linalg.norm(g-b))
        poses = np.asarray(outcome['dense_poses_world'])[:, :2]
        outcome['execution']['goal_segment_max_overshoot_m'] = (
            None if length <= 1e-12 else float(max(0., np.max((poses-g)@((g-b)/length)))))
        outcome['execution']['overshoot_definition'] = 'positive projection beyond the original goal along B-to-goal chord; descriptive only'
    reasons = list(dict.fromkeys(missing+plan_reasons+execution_reasons))
    result = solver_result or {}
    execution = outcome['execution'] or {}
    outcome.update(case_id=case['context']['case_id'], case_role=FIXED_CASES.get(case['context']['case_id']),
                   method=method, candidate_available=available, candidate_found=available, plan_valid=valid,
                   rollout_performed=rollout is not None, rollout_success=bool(outcome['primary_success']),
                   diagnostic_rollout_plan_invalid=eligible['diagnostic_rollout_plan_invalid'],
                   candidate_failure_reasons=missing, plan_failure_reasons=plan_reasons,
                   rollout_failure_reasons=execution_reasons, status_reasons=reasons,
                   termination_reason=(reasons[0] if reasons else 'ROLLOUT_SUCCESS'), termination_reasons=reasons,
                   solver_terminations=_terminations(result), fallback_used=False,
                   execution_kind='OFFLINE COUNTERFACTUAL HARD-HANDOFF COMPARISON',
                   candidate_velocity_is_actual_command=False,
                   timing=dict(optimization_total_wall_s=result.get('total_optimization_and_check_wall_s'),
                               rollout_wall_s=None if rollout is None else rollout['rollout_wall_s'],
                               mpc_official_solve_total_s=execution.get('mpc_official_solve_total_s'),
                               mpc_submit_wait_poll_total_s=execution.get('mpc_submit_wait_poll_total_s'),
                               evaluation_wall_s=time.perf_counter()-begin))
    return outcome


def comparisons(rows):
    """Complete fixed matrix, original success pairs, no pseudo-episode pooling."""
    by = {(r['case_id'], r['method']): r for r in rows}
    expected = {(case, method) for case in FIXED_CASES for method in METHODS}
    if len(by) != len(rows) or set(by) != expected:
        raise ValueError('comparison requires every fixed case/method exactly once')
    success = lambda row: bool(row.get('rollout_success', row.get('primary_success', False)))
    metrics_of = lambda row: row.get('rollout_metrics', row.get('metrics', row))
    regressions, improvements, paired = [], [], []
    keys = ['time_to_goal_s', 'terminal_position_error_m', 'terminal_yaw_error_rad',
            'path_length_m', 'minimum_clearance_m', 'first_command_delta_v_mps',
            'first_command_delta_omega_radps', 'command_total_variation_v_mps',
            'command_total_variation_omega_radps', 'control_grid_max_abs_acceleration_v_mps2',
            'control_grid_max_abs_acceleration_omega_radps2', 'goal_distance_improvement_m',
            'net_displacement_m', 'goal_reentries_after_exit', 'goal_segment_max_overshoot_m',
            'mpc_official_solve_total_s', 'mpc_official_solve_median_s', 'mpc_official_solve_max_s']
    for case_id, role in FIXED_CASES.items():
        for baseline, method in PAIRS:
            a, b = by[case_id, baseline], by[case_id, method]
            info = dict(case_id=case_id, case_role=role, baseline=baseline, method=method,
                        baseline_rollout_success=success(a), method_rollout_success=success(b))
            if success(a) and not success(b):
                regressions.append(dict(info, reasons=metrics_of(b).get('status_reasons', b.get('termination_reasons', []))))
            if not success(a) and success(b):
                improvements.append(dict(info, baseline_reasons=metrics_of(a).get('status_reasons', a.get('termination_reasons', []))))
            if success(a) and success(b):
                item = dict(info, both_primary_success=True)
                for key in keys:
                    av = metrics_of(a).get('minimum_clearance_m') if key == 'minimum_clearance_m' else metrics_of(a)['execution'].get(key)
                    bv = metrics_of(b).get('minimum_clearance_m') if key == 'minimum_clearance_m' else metrics_of(b)['execution'].get(key)
                    item[key+'_baseline'], item[key+'_method'] = av, bv
                    item[key+'_difference'] = None if av is None or bv is None else bv-av
                for label, row in [('baseline', a), ('method', b)]:
                    item['optimization_total_wall_s_'+label] = metrics_of(row).get('timing', {}).get('optimization_total_wall_s')
                    item['fresh_deformation_'+label] = row.get('deformation')
                paired.append(item)
    summaries = {}
    for group in ('benign', 'hard'):
        cases = [c for c, role in FIXED_CASES.items() if (role == 'BENIGN_CONTROL') == (group == 'benign')]
        summaries[group] = {method: dict(
            events=len(cases), candidates=sum(bool(by[c, method].get('candidate_available', by[c, method].get('candidate_found'))) for c in cases),
            plan_valid=sum(bool(by[c, method]['plan_valid']) for c in cases),
            rollouts=sum(bool(by[c, method]['rollout_performed']) for c in cases),
            rollout_success=sum(success(by[c, method]) for c in cases)) for method in METHODS}
    relevant_regressions = [r for r in regressions if r['baseline'] in ('M0_NATIVE', 'M0_ADAPTER') and r['method'] in GP_METHODS]
    relevant_improvements = [r for r in improvements if r['baseline'] in ('M0_NATIVE', 'M0_ADAPTER') and r['method'] in GP_METHODS]
    gp_executed = any(row['method'] in GP_METHODS and row['rollout_performed'] for row in rows)
    interpretation = ('INSUFFICIENT_EXECUTION_EVIDENCE' if not gp_executed else
                      'MIXED_LOCAL_EXECUTION_EVIDENCE' if relevant_regressions and relevant_improvements else
                      'POSITIVE_LOCAL_EXECUTION_EVIDENCE' if relevant_improvements else
                      'NO_ADDITIONAL_EXECUTION_BENEFIT')
    return dict(groups=summaries, regressions=regressions, improvements=improvements, paired_metrics=paired,
                native_success_to_gp_failure=[r for r in relevant_regressions if r['baseline'] == 'M0_NATIVE'],
                adapter_success_to_gp_failure=[r for r in relevant_regressions if r['baseline'] == 'M0_ADAPTER'],
                native_failure_to_gp_success=[r for r in relevant_improvements if r['baseline'] == 'M0_NATIVE'],
                execution_interpretation=interpretation,
                interpretation_rule='original rollout-success transitions versus native/adapter; paired quality metrics remain separate; no objective-based navigation claim',
                experimental_units='four dependent development-corpus events: one benign control and three hard cases; starts/methods are not independent episodes')

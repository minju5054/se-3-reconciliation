"""Synthetic lifecycle/aggregation fixtures; none are handoff performance data."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from reconciliation.gp_se2_02_evaluation import (
    FIXED_CASES, METHODS, case_scope, comparisons, evaluate_method,
    make_mpc_request, plan_for_method, rollout_submission, validate_counterfactual,
)
from reconciliation.gp_se2_rollout import counterfactual_rollout
from test_gp_se2_evaluation import environment, route
from test_gp_se2_rollout import mock_module


@pytest.fixture
def case(tmp_path):
    config = yaml.safe_load((Path(__file__).resolve().parents[1]/'configs/gp_se2_01.yaml').read_text())
    candidate = np.column_stack((np.linspace(.02, .6, 30), np.zeros((30, 2))))
    context = dict(case_id=next(iter(FIXED_CASES)), episode_id='episode_001_repeat_01',
                   handoff_id='handoff_002', B_world=[0., 0., 0.], u_minus=[.2, 0.],
                   previous_control=[.4, .1], original_capture_pose_world=[-1., .2, .4],
                   source_files=[])
    np.save(tmp_path/'F_native.npy', candidate)
    return dict(context=context, config=config, case_directory=tmp_path,
                common_reference=candidate, environment=environment(),
                problem=SimpleNamespace(boundary_pose=np.zeros(3), initial_twist=np.array([.2, 0., 0.])),
                goal_route=route([.6, 0., 0.]))


def plan(case, method='M0_ADAPTER'):
    return plan_for_method(case, method, case['common_reference'])


def full_report(case, valid=True):
    original = plan(case)
    return dict(original_dense_feasible=valid, original_plan_valid=True,
                additional_grid={'valid': True}, full_feasible=valid,
                original_plan_report=original)


def rollout(case, command=(.2, 0.), module=None):
    return counterfactual_rollout(module or mock_module(command), case['context'], case['common_reference'])


def test_exact_scope_and_no_gate_does_not_delete_gate(case):
    assert list(FIXED_CASES.values()) == ['BENIGN_CONTROL', 'HARD_POSITION_AND_DIRECTION', 'HARD_LARGE_TURN', 'HARD_POSITION_STRAIGHT']
    assert case_scope(case)['status'] == 'NO_GATE_SUPPORTED_SCOPE'
    case['goal_route']['gates'] = [{'gate_id': 'synthetic retained'}]
    assert case_scope(case)['status'] == 'UNSUPPORTED_GATE_CASE'
    assert case_scope(case)['gates_preserved'] == case['goal_route']['gates']
    case['context']['case_id'] = 'episode_017_repeat_00/handoff_007'
    with pytest.raises(ValueError, match='outside'):
        case_scope(case)


@pytest.mark.parametrize('method', ['M0_NATIVE', 'M0_ADAPTER'])
def test_native_adapter_arrays_never_modified_or_reanchored(case, method):
    candidate = case['common_reference'].copy()
    assert plan_for_method(case, method, candidate)['plan_valid']
    candidate[0, 0] += .01
    with pytest.raises(ValueError, match='preserve'):
        plan_for_method(case, method, candidate)


@pytest.mark.parametrize('method', ['M2_GP_NO_OBSTACLE', 'M3_GP_CONSTRAINED', 'SEED_ONLY'])
def test_gp_and_seed_require_full_checker_and_never_fallback(case, method):
    candidate = case['common_reference']
    with pytest.raises(ValueError, match='full acceptance'):
        plan_for_method(case, method, candidate)
    good = plan_for_method(case, method, candidate, full_acceptance=full_report(case))
    assert good['plan_valid']
    assert rollout_submission(method, candidate, good)['rollout_allowed']
    bad = plan_for_method(case, method, candidate, full_acceptance=full_report(case, False))
    assert not bad['plan_valid']
    assert not rollout_submission(method, candidate, bad)['rollout_allowed']
    outcome = evaluate_method(case, method, candidate, bad)
    assert not outcome['rollout_performed'] and not outcome['rollout_success']
    assert outcome['candidate_available'] and outcome['plan_failure_reasons'] == ['PLAN_INVALID']
    assert outcome['execution'] is None and outcome['dense_poses_world'] is None
    with pytest.raises(ValueError, match='forbidden'):
        evaluate_method(case, method, candidate, bad, rollout=rollout(case))


def test_inconsistent_full_report_cannot_bless_seed(case):
    report = full_report(case, False)
    report['full_feasible'] = True
    with pytest.raises(ValueError, match='inconsistent'):
        plan_for_method(case, 'SEED_ONLY', case['common_reference'], full_acceptance=report)


def test_baseline_invalid_plan_remains_diagnostic_but_rollout_success_separate(case):
    candidate = case['common_reference']
    invalid = dict(plan(case), plan_valid=False, status='PLAN_INVALID')
    outcome = evaluate_method(case, 'M0_ADAPTER', candidate, invalid, rollout=rollout(case))
    assert outcome['primary_success'] and outcome['rollout_success']
    assert not outcome['plan_valid']
    assert outcome['diagnostic_rollout_plan_invalid']
    assert outcome['status_reasons'] == ['PLAN_INVALID']
    assert outcome['rollout_failure_reasons'] == []
    assert outcome['failure_reasons'] == []  # original evaluation unchanged


def test_no_candidate_unsupported_and_timeout_remain_distinct(case):
    result = {'attempts': [{'termination': 'DERIVATIVE_UNSUPPORTED'}, {'termination': 'TIMEOUT'}]}
    empty = plan_for_method(case, 'M3_GP_CONSTRAINED', None)
    outcome = evaluate_method(case, 'M3_GP_CONSTRAINED', None, empty, result)
    assert outcome['candidate_failure_reasons'] == ['NO_CANDIDATE', 'DERIVATIVE_UNSUPPORTED', 'TIMEOUT_WITHOUT_VALID_CANDIDATE']
    assert outcome['rollout_failure_reasons'] == []
    assert outcome['minimum_clearance_m'] is None
    assert not outcome['fallback_used']


def test_effective_derivative_unsupported_preserves_raw_harness_termination(case):
    result = {'attempts': [{'termination': 'NUMERICAL_FAILURE', 'effective_termination': 'DERIVATIVE_UNSUPPORTED'}]}
    empty = plan_for_method(case, 'M3_GP_CONSTRAINED', None)
    outcome = evaluate_method(case, 'M3_GP_CONSTRAINED', None, empty, result)
    assert outcome['candidate_failure_reasons'] == ['NO_CANDIDATE', 'DERIVATIVE_UNSUPPORTED']
    assert result['attempts'][0]['termination'] == 'NUMERICAL_FAILURE'


def test_valid_reference_requires_new_independent_rollout(case):
    with pytest.raises(ValueError, match='missing required'):
        evaluate_method(case, 'M0_ADAPTER', case['common_reference'], plan(case))
    module = mock_module()
    a, b = rollout(case, module=module), rollout(case, module=module)
    assert len(module.instances) == 2
    assert validate_counterfactual(a, case['context'], case['common_reference'])['valid']
    assert validate_counterfactual(b, case['context'], case['common_reference'])['valid']
    a['states'][1]['pose_world'][0] += 1.
    with pytest.raises(ValueError, match='exact held'):
        validate_counterfactual(a, case['context'], case['common_reference'])
    assert validate_counterfactual(b, case['context'], case['common_reference'])['valid']


def test_physical_motion_memory_and_capture_frame_preserved(case):
    actual = rollout(case)
    report = validate_counterfactual(actual, case['context'], case['common_reference'])
    assert report['physical_u_minus_and_controller_memory_separately_preserved']
    assert not report['gp_velocity_feedforward']
    actual['initial_previous_control'] = actual['initial_physical_command']
    with pytest.raises(ValueError, match='previous_control'):
        validate_counterfactual(actual, case['context'], case['common_reference'])


@pytest.mark.parametrize('mutation', ['schedule', 'capture', 'command', 'snapping', 'simulation_time'])
def test_changed_schedule_reanchoring_commands_or_time_rejected(case, mutation):
    actual = rollout(case)
    if mutation == 'schedule':
        actual['control_hz'] = 20.
    elif mutation == 'capture':
        actual['candidate_frame_transform']['capture_pose_world'] = case['context']['B_world']
    elif mutation == 'command':
        actual['commands'][1]['command'] = [0., 0.]
    elif mutation == 'snapping':
        actual['pose_snaps'] = 1
    else:
        actual['controller_reference_selections'][0]['simulation_time_advanced_during_solve_s'] = .1
    with pytest.raises(ValueError):
        validate_counterfactual(actual, case['context'], case['common_reference'])


def test_original_goal_and_dwell_failures_are_not_zero_error(case):
    actual = rollout(case, command=(0., 0.))
    outcome = evaluate_method(case, 'M0_ADAPTER', case['common_reference'], plan(case), rollout=actual)
    assert outcome['rollout_failure_reasons'] == ['GOAL_POSITION_FAILURE', 'GOAL_DWELL_FAILURE']
    assert outcome['execution']['terminal_position_error_m'] == pytest.approx(.6)
    assert outcome['execution']['time_to_goal_s'] is None
    # Motion limits use the physical incoming .2 m/s, not controller-memory .4.
    assert outcome['execution']['control_grid_max_abs_acceleration_v_mps2'] == pytest.approx(2.)


def test_terminal_pose_pass_is_not_terminal_goal_dwell_pass(case):
    case['goal_route']['goal_world'] = [.74, 0., 0.]
    outcome = evaluate_method(case, 'M0_ADAPTER', case['common_reference'], plan(case), rollout=rollout(case))
    assert outcome['execution']['terminal_position_error_m'] < .15
    assert outcome['execution']['goal_reached']
    assert not outcome['execution']['terminal_goal_dwell_pass']
    assert outcome['rollout_failure_reasons'] == ['GOAL_DWELL_FAILURE']
    assert not outcome['rollout_success']


def test_collision_clearance_and_controller_failure_all_retained(case):
    from shapely.geometry import LineString
    case['environment'] = environment(LineString([(.3, -1.), (.3, 1.)]))
    actual = rollout(case, module=mock_module(command=(.8, 0.), failure_at=1))
    outcome = evaluate_method(case, 'M0_ADAPTER', case['common_reference'], plan(case), rollout=actual)
    assert {'CONTROLLER_FAILURE', 'COLLISION', 'CLEARANCE_VIOLATION', 'MOTION_VIOLATION'} <= set(outcome['rollout_failure_reasons'])
    assert outcome['environment']['first_overlap_time_bracket_s'] is not None
    assert actual['states'][-1]['time_s'] == 3.  # original continue-through-failure policy


def test_goal_yaw_failure_separate_and_seed_velocity_is_not_command(case):
    case['goal_route']['goal_world'][2] = .5
    candidate = case['common_reference']
    valid_report = full_report(case)
    valid_report['original_plan_report']['plan_valid'] = True
    seed_plan = plan_for_method(case, 'SEED_ONLY', candidate, full_acceptance=valid_report)
    result = evaluate_method(case, 'SEED_ONLY', candidate, seed_plan,
                             {'total_optimization_and_check_wall_s': 7.}, rollout(case))
    assert 'GOAL_YAW_FAILURE' in result['rollout_failure_reasons']
    assert 'GOAL_DWELL_FAILURE' in result['rollout_failure_reasons']
    assert not result['candidate_velocity_is_actual_command']
    assert result['timing']['optimization_total_wall_s'] == 7.
    assert result['timing']['mpc_official_solve_total_s'] == 3000.


def request_entries(case, tmp_path):
    candidate_path = tmp_path/'candidate.npy'
    np.save(candidate_path, case['common_reference'])
    entries = []
    for case_id in FIXED_CASES:
        episode, handoff = case_id.split('/')
        context = dict(case['context'], case_id=case_id, episode_id=episode, handoff_id=handoff)
        methods = {m: {'candidate_path': candidate_path, 'plan': {'plan_valid': m != 'SEED_ONLY'}} for m in METHODS}
        entries.append(dict(context=context, goal_route=case['goal_route'], methods=methods))
    return entries


def test_request_covers_fixed_matrix_and_invalid_seed_null_with_no_fallback(case, tmp_path):
    entries = request_entries(case, tmp_path)
    request = make_mpc_request(tmp_path/'source', entries, case['config'])
    assert len(request['cases']) == 4
    assert all(len(r['methods']) == 6 for r in request['cases'])
    assert all(r['methods']['SEED_ONLY'] is None for r in request['cases'])
    assert all(r['rollout_eligibility']['SEED_ONLY']['missing_reason'] == 'PLAN_INVALID' for r in request['cases'])
    assert request['horizon_s'] == 3. and request['goal_dwell_s'] == .2
    with pytest.raises(ValueError, match='exactly the four'):
        make_mpc_request(tmp_path/'source', entries[:-1], case['config'])
    entries[0]['methods'].pop('SEED_ONLY')
    with pytest.raises(ValueError, match='every planned method'):
        make_mpc_request(tmp_path/'source', entries, case['config'])


def result_rows(case):
    outcome = evaluate_method(case, 'M0_ADAPTER', case['common_reference'], plan(case), rollout=rollout(case))
    rows = []
    for case_id, role in FIXED_CASES.items():
        for method in METHODS:
            rows.append(dict(case_id=case_id, case_role=role, method=method,
                             candidate_available=True, plan_valid=True, rollout_performed=True,
                             rollout_success=True, rollout_metrics=deepcopy(outcome)))
    return rows


def test_case_method_coverage_and_no_duplicate_episode_counts(case):
    rows = result_rows(case)
    result = comparisons(rows)
    assert result['groups']['benign']['M3_GP_CONSTRAINED']['events'] == 1
    assert result['groups']['hard']['M3_GP_CONSTRAINED']['events'] == 3
    assert result['groups']['hard']['M3_GP_CONSTRAINED']['rollout_success'] == 3
    assert len(result['paired_metrics']) == 40
    assert result['execution_interpretation'] == 'NO_ADDITIONAL_EXECUTION_BENEFIT'
    with pytest.raises(ValueError, match='exactly once'):
        comparisons(rows[:-1])
    with pytest.raises(ValueError, match='exactly once'):
        comparisons(rows+[rows[0]])


def test_regressions_prioritized_and_paired_quality_requires_success(case):
    rows = result_rows(case)
    hard = list(FIXED_CASES)[1]
    for row in rows:
        if row['case_id'] == hard and row['method'] == 'M3_GP_CONSTRAINED':
            row.update(candidate_available=False, plan_valid=False, rollout_performed=False, rollout_success=False,
                       rollout_metrics={'status_reasons': ['NO_CANDIDATE'], 'execution': None})
    result = comparisons(rows)
    assert len(result['native_success_to_gp_failure']) == 1
    assert len(result['adapter_success_to_gp_failure']) == 1
    assert len(result['regressions']) == 5  # native/adapter/rigid/seed/M2
    assert len(result['paired_metrics']) == 35
    assert not any(r['case_id'] == hard and r['method'] == 'M3_GP_CONSTRAINED' for r in result['paired_metrics'])
    assert result['native_success_to_gp_failure'][0]['reasons'] == ['NO_CANDIDATE']


def test_mixed_execution_transitions_are_not_objective_improvement(case):
    rows = result_rows(case)
    hard1, hard2 = list(FIXED_CASES)[1:3]
    for row in rows:
        fail = (row['case_id'] == hard1 and row['method'] == 'M0_NATIVE') or (row['case_id'] == hard2 and row['method'] == 'M3_GP_CONSTRAINED')
        if fail:
            row['rollout_success'] = False
            row['rollout_metrics'] = {'status_reasons': ['GOAL_DWELL_FAILURE'], 'execution': None}
    result = comparisons(rows)
    assert result['execution_interpretation'] == 'MIXED_LOCAL_EXECUTION_EVIDENCE'
    assert len(result['native_failure_to_gp_success']) == 2  # paired M2 and M3, explicitly not two events
    assert {r['case_id'] for r in result['native_failure_to_gp_success']} == {hard1}

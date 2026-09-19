"""Synthetic REF-01 orchestration fixtures, never actual handoff evidence."""
from copy import deepcopy
import importlib.util
from pathlib import Path

import numpy as np
import pytest
import yaml

from reconciliation.gp_se2_reference import prepare_reference
from reconciliation.gp_se2_ref01_reference import build_variants
from reconciliation.gp_se2_ref01_evaluation import (
    FIXED_CASES, VARIANTS, evaluate_variant, reproduction_check,
    factor_contrasts, selection_diagnostics,
)
from reconciliation.gp_se2_rollout import counterfactual_rollout
from reconciliation.se2 import local_trajectory_to_world
from test_gp_se2_evaluation import environment, route
from test_gp_se2_rollout import mock_module

spec = importlib.util.spec_from_file_location('ref01_worker_test', Path(__file__).resolve().parents[1]/'scripts/lightnav/gp_se2_ref01_mpc.py')
worker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(worker)


@pytest.fixture
def fixture():
    native = np.array([[0., 0., 0.], [.2, 0., .05], [.4, .02, .1], [.6, .05, .15]])
    context = dict(case_id=FIXED_CASES[0], B_world=[.01, 0., 0.], u_minus=[.1, .02],
                   previous_control=[.3, -.1], original_capture_pose_world=[-1., .2, .4], source_files=[])
    common = prepare_reference(native, context['B_world'], [10, 10, 1])['common_world']
    prepared = build_variants(native, context['B_world'], [10, 10, 1], expected_common=common)
    module = mock_module(command=(.2, .05))
    module.project_body_to_world = local_trajectory_to_world_wrapper
    module.MPC_DT_S = .1
    config = yaml.safe_load((Path(__file__).resolve().parents[1]/'configs/gp_se2_01.yaml').read_text())
    return context, native, prepared, module, config


def local_trajectory_to_world_wrapper(rows, capture):
    return local_trajectory_to_world(capture, rows)


def make_rollout(fixture, name='R00_NATIVE'):
    context, native, prepared, module, config = fixture
    record = prepared['variants'][name]
    rollout = counterfactual_rollout(module, context, record['reference_world'])
    worker.enrich_rollout(module, rollout, record['reference_world'], record['row_provenance'], len(native)-1, name)
    outcome = evaluate_variant(name, rollout, context, record['reference_world'], route(native[-1]), environment(), config)
    return rollout, outcome


def test_probes_only_selector_no_new_tracker_or_solve_same_states(fixture):
    context, native, prepared, module, _ = fixture
    historical = counterfactual_rollout(module, context, native)
    before = len(module.instances)
    refs = {k: v['reference_world'] for k, v in prepared['variants'].items()}
    lineages = {k: v['row_provenance'] for k, v in prepared['variants'].items()}
    original = deepcopy(historical)
    probe = worker.matched_state_probes(module, historical, refs, lineages, len(native)-1)
    assert len(module.instances) == before
    assert probe['new_mpc_solves'] == 0 and probe['variant_selection_count'] == 120
    assert not probe['state_integration_performed'] and not probe['controller_memory_modified']
    assert historical == original
    for i, row in enumerate(probe['states']):
        assert row['input_pose_world'] == historical['controller_reference_selections'][i]['input_pose_world']
        assert tuple(row['variants']) == VARIANTS
        assert all(r['input_pose_world'] == row['input_pose_world'] for r in row['variants'].values())


def test_probes_reject_wrong_historical_schedule(fixture):
    context, native, prepared, module, _ = fixture
    history = counterfactual_rollout(module, context, native)
    history['controller_reference_selections'][1]['time_s'] = .11
    with pytest.raises(ValueError, match='original 30'):
        worker.matched_state_probes(module, history, {}, {}, 3)


def test_variants_use_independent_tracker_and_original_capture_memory(fixture):
    context, native, prepared, module, _ = fixture
    outputs = [make_rollout(fixture, name)[0] for name in VARIANTS]
    assert len(module.instances) == 4
    assert all(instance.closed for instance in module.instances)
    for name, result in zip(VARIANTS, outputs):
        assert len(result['states']) == 181 and len(result['commands']) == 180
        assert len(result['controller_reference_selections']) == 30
        assert result['initial_pose_world'] == context['B_world']
        assert result['initial_physical_command'] != result['initial_previous_control']
        assert result['initial_previous_control'] == context['previous_control']
        assert not result['candidate_frame_transform']['B_reanchoring']
        assert result['candidate_frame_transform']['capture_pose_world'] == context['original_capture_pose_world']
        assert result['variant'] == name
        assert not result['GP_or_rigid_optimization_performed']
        for solve in result['controller_reference_selections']:
            assert solve['installed_path_sha256'] == result['installed_path_sha256']
            assert solve['selection_diagnostic']['reference_world'] == solve['selection']['reference_world']
            assert len(solve['selection_diagnostic']['selected_source_rows']) == 5
            assert solve['prediction_world'] is not None
            assert solve['simulation_time_advanced_during_solve_s'] == 0


def test_changed_initial_memory_or_snap_rejected(fixture):
    context, native, prepared, module, config = fixture
    rollout, _ = make_rollout(fixture)
    for mutation in ('memory', 'snap', 'state'):
        changed = deepcopy(rollout)
        if mutation == 'memory': changed['initial_previous_control'] = context['u_minus']
        elif mutation == 'snap': changed['pose_snaps'] = 1
        else: changed['states'][20]['pose_world'][0] += .05
        with pytest.raises(ValueError):
            evaluate_variant('R00_NATIVE', changed, context, native, route(native[-1]), environment(), config)


def test_original_outcome_and_dwell_definition_preserved(fixture):
    from reconciliation.gp_se2_evaluation import evaluate_rollout
    context, native, _, _, config = fixture
    rollout, outcome = make_rollout(fixture)
    original = evaluate_rollout(rollout, route(native[-1]), environment(), config)
    assert outcome['primary_success'] == original['primary_success']
    assert outcome['execution'] == original['execution']
    assert outcome['environment'] == original['environment']
    assert outcome['summary']['terminal_goal_dwell_required_s'] == .2
    assert outcome['summary']['terminal_achieved_goal_dwell_sampled_s'] != .2
    assert outcome['summary']['primary_mpc_solve_count'] == 30
    assert not outcome['GP_or_rigid_optimization_performed']


def test_original_command_failure_hold_policy_unchanged(fixture):
    context, native, prepared, _, config = fixture
    module = mock_module(command=(.2, .05), failure_at=1)
    module.project_body_to_world = local_trajectory_to_world_wrapper
    rollout = counterfactual_rollout(module, context, native)
    worker.enrich_rollout(module, rollout, native, prepared['variants']['R00_NATIVE']['row_provenance'], 3, 'R00_NATIVE')
    outcome = evaluate_variant('R00_NATIVE', rollout, context, native, route(native[-1]), environment(), config)
    assert outcome['summary']['controller_failure_count'] == 1
    assert not outcome['primary_success']
    assert 'CONTROLLER_FAILURE' in outcome['summary']['failure_reasons']
    assert rollout['commands'][6]['command'] == [0., 0.]
    assert rollout['controller_reference_selections'][2]['previous_control'] == [0., 0.]
    assert rollout['states'][-1]['time_s'] == 3.


def test_reproduction_distinguishes_bitwise_and_tolerance(fixture):
    rollout, outcome = make_rollout(fixture)
    equal = reproduction_check(rollout, rollout, outcome, outcome)
    assert equal['passed'] and equal['all_compared_values_bitwise_equal']
    changed = deepcopy(rollout)
    changed['states'][-1]['pose_world'][0] += 1e-8
    report = reproduction_check(changed, rollout, outcome, outcome)
    assert report['passed'] and not report['all_compared_values_bitwise_equal']
    changed['states'][-1]['pose_world'][0] += 1e-4
    assert not reproduction_check(changed, rollout, outcome, outcome)['passed']


def test_reproduction_fails_changed_selection_and_success(fixture):
    rollout, outcome = make_rollout(fixture)
    changed = deepcopy(rollout)
    changed['controller_reference_selections'][0]['selection']['indices'][0] += 1
    assert not reproduction_check(changed, rollout, outcome, outcome)['passed']
    metrics = deepcopy(outcome)
    metrics['primary_success'] = not outcome['primary_success']
    assert not reproduction_check(rollout, rollout, metrics, outcome)['passed']


def test_reproduction_preserves_unavailable_goal_time(fixture):
    rollout, outcome = make_rollout(fixture)
    a, b = deepcopy(outcome), deepcopy(outcome)
    a['execution']['time_to_goal_s'] = None
    b['execution']['time_to_goal_s'] = None
    assert reproduction_check(rollout, rollout, a, b)['passed']
    b['execution']['time_to_goal_s'] = 0.
    assert not reproduction_check(rollout, rollout, a, b)['passed']


def test_factor_contrasts_direction_and_null_not_zero():
    rows = [dict(case_id='synthetic', variant=v, terminal_position_error_m=value,
                 time_to_goal_s=None if i == 3 else value) for i, (v, value) in enumerate(zip(VARIANTS, [2, 5, 7, 13]))]
    contrasts = {r['metric']: r for r in factor_contrasts(rows)}
    row = contrasts['terminal_position_error_m']
    assert [row[k] for k in ('suffix_without_resampling', 'resampling_without_suffix', 'suffix_with_resampling', 'resampling_with_suffix', 'interaction')] == [3, 5, 6, 8, 3]
    assert contrasts['time_to_goal_s']['interaction'] is None
    assert contrasts['time_to_goal_s']['suffix_without_resampling'] == 3
    with pytest.raises(ValueError, match='all four'):
        factor_contrasts(rows[:-1])
    with pytest.raises(ValueError, match='duplicate'):
        factor_contrasts(rows+[rows[0]])


def test_selection_progress_is_original_fractional_not_local_index(fixture):
    rollout, outcome = make_rollout(fixture, 'R11_CURRENT_ADAPTER')
    first = rollout['controller_reference_selections'][0]['selection_diagnostic']
    diag = selection_diagnostics(rollout['controller_reference_selections'])
    assert diag['selected_first_original_progress'][0] == first['selected_original_fractional_row_coordinates'][0]
    assert diag['selected_first_original_progress'][0] != first['indices'][0]
    assert diag['first_horizon_xy_arc_length_m'] == first['horizon_xy_arc_length_m']
    no_goal = deepcopy(rollout['controller_reference_selections'])
    for row in no_goal: row['selection_diagnostic']['final_goal_row_in_horizon'] = False
    assert selection_diagnostics(no_goal)['final_target_first_in_horizon_time_s'] is None


def test_fixed_request_case_variant_order_no_fallback():
    request = dict(cases=[dict(episode_id=c.split('/')[0], handoff_id=c.split('/')[1], variants={v: {} for v in VARIANTS}) for c in FIXED_CASES])
    assert worker.verify_request(request)
    for mutation in ('order', 'variant', 'fallback', 'horizon'):
        bad = deepcopy(request)
        if mutation == 'order': bad['cases'].reverse()
        elif mutation == 'variant': bad['cases'][0]['variants']['M0_NATIVE'] = bad['cases'][0]['variants'].pop('R00_NATIVE')
        elif mutation == 'fallback': bad['cases'][0]['variants']['R00_NATIVE'] = None
        else: bad['horizon_s'] = 4.
        with pytest.raises(ValueError): worker.verify_request(bad)


def test_worker_write_refuses_overwrite(tmp_path):
    path = tmp_path/'record.json'
    worker.write_new(path, {'first': True})
    with pytest.raises(FileExistsError): worker.write_new(path, {'overwrite': True})

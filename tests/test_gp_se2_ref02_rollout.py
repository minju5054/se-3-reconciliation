"""Synthetic REF-02 runtime fixtures; these are not actual handoff evidence.

Load only pure pinned official runtime classes/functions via AST. Replace the
numerical MPCController with a deterministic fixture; no optimizer is run here.
"""
from __future__ import annotations

import ast
from concurrent.futures import Future, ThreadPoolExecutor
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import importlib.util
import math
from pathlib import Path
import sys
import time
from types import ModuleType
from typing import Sequence

import numpy as np
import pytest

from reconciliation.gp_se2_ref01_evaluation import FIXED_CASES
from reconciliation.gp_se2_ref01_reference import build_variants
from reconciliation.gp_se2_ref02_reference import METHODS
from reconciliation.gp_se2_ref02_rollout import (
    counterfactual_rollout, diagnostic_synchronous_solve, make_diagnostic_tracker,
)
from reconciliation.gp_se2_reference import prepare_reference
from reconciliation.gp_se2_rollout import counterfactual_rollout as original_rollout
from reconciliation.online_mpc_adapter import PINNED_MPC_SHA256
from reconciliation.robotless_online import integrate_unicycle

OFFICIAL = Path('/home/gpuadmin/Workspace/external/LightNav-0-official-demo/mujoco_demo/vln_mujoco/mpc.py')
spec = importlib.util.spec_from_file_location('ref02_worker_test', Path(__file__).resolve().parents[1]/'scripts/lightnav/gp_se2_ref02_mpc.py')
worker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(worker)


def fixture_runtime(failure_at=None):
    if not OFFICIAL.exists():
        pytest.skip('read-only pinned official runtime source unavailable')
    source = OFFICIAL.read_bytes()
    assert hashlib.sha256(source).hexdigest() == PINNED_MPC_SHA256
    wanted = {'wrap_angle', 'project_body_to_world', 'project_world_to_local',
              'project_local_to_world', 'build_pose_aligned_reference', 'MpcSolveResult', 'MpcTracker'}
    tree = ast.parse(source)
    selected = [node for node in tree.body if isinstance(node, ast.Assign)
                or isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in wanted]
    module = ModuleType('ref02_pinned_runtime_fixture')
    sys.modules[module.__name__] = module
    module.__dict__.update(np=np, math=math, time=time, Future=Future,
        ThreadPoolExecutor=ThreadPoolExecutor, dataclass=dataclass, Sequence=Sequence)

    class MPCController:
        def __init__(self, **settings):
            self.settings, self.calls = settings, []

        def solve(self, pose, reference, previous, v_max):
            self.calls.append((np.asarray(pose).copy(), np.asarray(reference).copy(), tuple(previous), v_max))
            if len(self.calls)-1 == failure_at:
                raise RuntimeError('synthetic controller failure')
            # Dependent on actual targets; no optimization, merely a runtime fixture.
            command = (float(np.clip(np.mean(reference[:, 0]), 0, .15)),
                       float(np.clip(np.mean(reference[:, 2]), -.3, .3)))
            return command, np.vstack([pose, reference])

    module.MPCController = MPCController
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(OFFICIAL), 'exec'), module.__dict__)
    return module


@pytest.fixture
def fixture():
    module = fixture_runtime()
    native = np.array([[3.1, -1.9, 2.8], [3.2, -1.9, 3.0], [3.25, -1.8, -3.0],
                       [3.4, -1.7, -2.8], [3.5, -1.7, -2.6], [3.6, -1.6, -2.4]])
    frozen = dict(case_id=FIXED_CASES[0], B_world=[3., -2., 2.7], u_minus=[.1, .2],
                  previous_control=[.3, -.1], original_capture_pose_world=[2., -3., -2.9], source_files=[])
    common = prepare_reference(native, frozen['B_world'], module.Q_WEIGHTS)['common_world']
    variants = build_variants(native, frozen['B_world'], module.Q_WEIGHTS, expected_common=common)['variants']
    mappings = ('R00_NATIVE', 'R11_CURRENT_ADAPTER', 'R11_CURRENT_ADAPTER')
    references = {m: variants[v]['reference_world'] for m, v in zip(METHODS, mappings)}
    lineages = {m: variants[v]['row_provenance'] for m, v in zip(METHODS, mappings)}
    return module, frozen, references, lineages, len(native)-1


def perform(fixture, method):
    module, frozen, references, lineages, goal_index = fixture
    return counterfactual_rollout(module, frozen, references[method], lineages[method], method,
        original_goal_row_index=goal_index)


@pytest.mark.parametrize('method', METHODS[:2])
def test_official_wrapper_reference_command_prediction_state_parity(fixture, method):
    module, frozen, references, _, _ = fixture
    baseline = original_rollout(module, frozen, references[method])
    wrapped = perform(fixture, method)
    for key in ('candidate_world', 'candidate_capture_local', 'candidate_frame_transform',
                'states', 'commands', 'initial_physical_command', 'initial_previous_control',
                'resolved_limits', 'official_gains', 'failure_policy'):
        assert wrapped[key] == baseline[key]
    for a, b in zip(wrapped['controller_reference_selections'], baseline['controller_reference_selections']):
        for key in ('input_pose_world', 'previous_control', 'command', 'previous_control_after',
                    'success', 'error', 'prediction_world'):
            assert a[key] == b[key]
        for key in a['selection']:
            assert a['selection'][key] == b['selection'][key]
    assert all(wrapped['wrapper_identity'].values())


def test_per_instance_selector_private_namespace_and_no_global_patch(fixture):
    module, _, references, lineages, goal_index = fixture
    original_selector = module.build_pose_aligned_reference
    original_globals = dict(module.MpcTracker.submit.__globals__)
    a = make_diagnostic_tracker(module, METHODS[1], references[METHODS[1]], lineages[METHODS[1]], goal_index)
    c = make_diagnostic_tracker(module, METHODS[2], references[METHODS[2]], lineages[METHODS[2]], goal_index)
    try:
        assert a.submit.__func__.__code__ is c.submit.__func__.__code__ is module.MpcTracker.submit.__code__
        assert a.submit.__func__.__globals__ is not c.submit.__func__.__globals__
        assert module.build_pose_aligned_reference is original_selector
        assert all(module.MpcTracker.submit.__globals__[k] is v for k, v in original_globals.items())
        for tracker in (a, c):
            tracker.set_body_path(references[METHODS[1]], [0, 0, 0])
            tracker.submit(references[METHODS[1]][0])
        for tracker in (a, c):
            tracker._future.result()
            assert tracker.poll() is not None
        assert a._ref02_capture['selector_calls'][0]['diagnostic']['nearest_index'] == c._ref02_capture['selector_calls'][0]['diagnostic']['nearest_index']
        assert a._ref02_capture['selector_calls'][0]['diagnostic']['indices'] != c._ref02_capture['selector_calls'][0]['diagnostic']['indices']
        assert a._ref02_capture['selector_calls'][0]['reference'] is not c._ref02_capture['selector_calls'][0]['reference']
    finally:
        a.close()
        c.close()


def test_original_outstanding_future_and_generation_poll_behavior(fixture):
    module, _, refs, lineage, goal = fixture
    tracker = make_diagnostic_tracker(module, METHODS[2], refs[METHODS[2]], lineage[METHODS[2]], goal)
    try:
        tracker.set_body_path(refs[METHODS[2]], [0, 0, 0])
        outstanding = Future()
        tracker._future = outstanding
        tracker.submit(refs[METHODS[2]][0])
        assert tracker._future is outstanding
        assert not tracker._ref02_capture['selector_calls']
        tracker._future = None
        tracker.submit(refs[METHODS[2]][0])
        tracker._future.result()
        tracker._generation += 1
        assert tracker.poll() is None
        assert tracker.previous_command == (0., 0.)
    finally:
        tracker.close()


def test_actual_controller_input_capture_matches_selected_existing_rows(fixture):
    module, _, _, _, _ = fixture
    result = perform(fixture, METHODS[2])
    installed = np.array(result['installed_reference_world'])
    for row in result['controller_reference_selections']:
        selected = row['selection']['indices']
        ref = np.asarray(row['actual_submitted_reference_world'])
        np.testing.assert_array_equal(ref[:, :2], installed[selected, :2])
        np.testing.assert_allclose(np.sin(ref[:, 2]-installed[selected, 2]), 0., atol=1e-12)
        np.testing.assert_array_equal(row['actual_controller_reference_local'], module.project_world_to_local(ref, row['input_pose_world']))
        assert row['result_reference_world'] == row['actual_submitted_reference_world']
        assert row['actual_reference_capture_complete']
        assert row['actual_controller_previous_control'] == row['previous_control']
        assert row['selection_diagnostic']['reference_world'] == row['selection']['reference_world']


def test_actual_controller_input_mismatch_is_not_silently_relabelled(fixture):
    module, frozen, refs, lineage, goal = fixture
    tracker = make_diagnostic_tracker(module, METHODS[2], refs[METHODS[2]], lineage[METHODS[2]], goal)
    try:
        tracker.set_body_path(refs[METHODS[2]], [0, 0, 0])
        namespace = tracker.submit.__func__.__globals__
        good = namespace['build_pose_aligned_reference']
        def wrong(*args, **kwargs):
            actual = good(*args, **kwargs).copy()
            actual[0, 0] += .01
            return actual
        namespace['build_pose_aligned_reference'] = wrong
        with pytest.raises(ValueError, match='actual MPCController.solve reference'):
            diagnostic_synchronous_solve(module, tracker, frozen['B_world'])
    finally:
        tracker.close()


def test_fixed_capture_frame_memory_timing_and_exact_integration(fixture):
    _, frozen, references, _, _ = fixture
    result = perform(fixture, METHODS[2])
    assert result['initial_pose_world'] == frozen['B_world']
    assert result['initial_physical_command'] == frozen['u_minus']
    assert result['initial_previous_control'] == frozen['previous_control'] != frozen['u_minus']
    assert result['candidate_frame_transform']['capture_pose_world'] == frozen['original_capture_pose_world']
    assert not result['candidate_frame_transform']['B_reanchoring']
    assert result['new_lightnav_updates'] == result['pose_snaps'] == 0
    assert not result['GP_or_rigid_optimization_performed']
    assert len(result['states']) == 181 and len(result['commands']) == 180
    assert len(result['controller_reference_selections']) == result['actual_selector_calls'] == result['actual_controller_calls'] == 30
    assert result['controller_reference_selections'][0]['time_s'] == 0.
    assert result['controller_reference_selections'][-1]['time_s'] == 2.9
    assert result['states'][-1]['time_s'] == 3.
    for state, command, next_state in zip(result['states'], result['commands'], result['states'][1:]):
        np.testing.assert_array_equal(next_state['pose_world'], integrate_unicycle(state['pose_world'], command['command'], 1/60))
    for solve in result['controller_reference_selections']:
        assert solve['simulation_time_advanced_during_solve_s'] == 0.
    np.testing.assert_array_equal(result['candidate_world'], references[METHODS[2]])


def test_controller_failure_keeps_original_zero_hold_memory_and_continues(fixture):
    _, frozen, refs, lineage, goal = fixture
    module = fixture_runtime(failure_at=1)
    result = counterfactual_rollout(module, frozen, refs[METHODS[2]], lineage[METHODS[2]], METHODS[2], original_goal_row_index=goal)
    assert result['controller_failure_count'] == 1
    assert result['commands'][6]['command'] == [0., 0.]
    assert result['controller_reference_selections'][2]['previous_control'] == [0., 0.]
    failed = result['controller_reference_selections'][1]
    assert failed['actual_reference_capture_complete']
    assert failed['result_reference_world'] is None and failed['prediction_world'] is None
    assert result['states'][-1]['time_s'] == 3.


def test_matched_probes_no_tracker_optimization_or_state_memory_update(fixture):
    module, frozen, refs, lineages, goal = fixture
    history = original_rollout(module, frozen, refs[METHODS[0]])
    stored = deepcopy(history)
    old_tracker = module.MpcTracker
    module.MpcTracker = lambda: pytest.fail('probe constructed tracker')
    try:
        probes = worker.matched_state_probes(module, history, refs, lineages, goal)
    finally:
        module.MpcTracker = old_tracker
    assert probes['probe_count'] == 30 and probes['method_selection_count'] == 90
    assert probes['new_mpc_solves'] == 0
    assert not probes['state_integration_performed'] and not probes['controller_memory_modified']
    assert history == stored
    assert all(row['B_C_nearest_exactly_equal'] for row in probes['states'])
    assert all(tuple(row['methods']) == METHODS for row in probes['states'])


def test_saved_probe_input_mismatch_and_B_C_geometry_changes_rejected(fixture):
    module, frozen, refs, lineages, goal = fixture
    history = original_rollout(module, frozen, refs[METHODS[0]])
    saved = dict(states=[dict(time_s=s['time_s'], input_pose_world=s['input_pose_world']) for s in history['controller_reference_selections']])
    saved['states'][1]['time_s'] = .11
    with pytest.raises(ValueError, match='REF-01 matched-state'):
        worker.matched_state_probes(module, history, refs, lineages, goal, saved_ref01_probes=saved)
    different = {method: value.copy() for method, value in refs.items()}
    different[METHODS[2]][2, 0] += .01
    with pytest.raises(ValueError, match='B/C dense'):
        worker.matched_state_probes(module, history, different, lineages, goal)


def test_six_rollout_coverage_is_two_times_three_not_retry(fixture):
    module, frozen, refs, lineages, goal = fixture
    outputs = []
    for case in FIXED_CASES:
        context = {**frozen, 'case_id': case}
        for method in METHODS:
            outputs.append(counterfactual_rollout(module, context, refs[method], lineages[method], method, original_goal_row_index=goal))
    assert len(outputs) == 6
    assert sum(o['actual_controller_calls'] for o in outputs) == 180
    assert sum(o['actual_selector_calls'] for o in outputs) == 180
    assert all(o['independent_tracker_instance'] for o in outputs)
    outputs[0]['states'][0]['pose_world'][0] += 100
    assert outputs[1]['states'][0]['pose_world'][0] != outputs[0]['states'][0]['pose_world'][0]


def test_fixed_order_schedule_stride_no_fallback_and_no_overwrite(tmp_path):
    request = dict(cases=[dict(episode_id=c.split('/')[0], handoff_id=c.split('/')[1], methods={m: {} for m in METHODS}) for c in FIXED_CASES])
    assert worker.verify_request(request)
    for mutation in ('case', 'method', 'fallback', 'stride', 'horizon'):
        bad = deepcopy(request)
        if mutation == 'case': bad['cases'].reverse()
        elif mutation == 'method': bad['cases'][0]['methods'] = dict(reversed(list(bad['cases'][0]['methods'].items())))
        elif mutation == 'fallback': bad['cases'][0]['methods'][METHODS[2]] = None
        elif mutation == 'stride': bad['source_progress_stride'] = 2.
        else: bad['horizon_s'] = 4.
        with pytest.raises(ValueError): worker.verify_request(bad)
    path = tmp_path/'record.json'
    worker.write_new(path, {'value': 1})
    with pytest.raises(FileExistsError): worker.write_new(path, {'value': 2})


def test_matched_probes_use_exact_original_capture_installed_world(fixture):
    from reconciliation.gp_se2_rollout import candidate_in_capture_frame
    from reconciliation.gp_se2_ref02_rollout import array_sha256
    module, frozen, refs, lineages, goal = fixture
    history = original_rollout(module, frozen, refs[METHODS[0]])
    original = {key: value.copy() for key, value in refs.items()}
    probe = worker.matched_state_probes(module, history, refs, lineages, goal)
    capture = frozen['original_capture_pose_world']
    roundtrip_differs_from_source_bitwise = False
    for method in METHODS:
        local, transform = candidate_in_capture_frame(refs[method], capture)
        expected = module.project_body_to_world(local, capture)
        roundtrip_differs_from_source_bitwise |= not np.array_equal(expected, refs[method])
        recorded = probe['installation'][method]
        np.testing.assert_array_equal(recorded['installed_reference_world'], expected)
        np.testing.assert_array_equal(recorded['candidate_capture_local'], local)
        assert recorded['candidate_frame_transform'] == transform
        assert recorded['source_reference_array_sha256'] == array_sha256(refs[method])
        assert recorded['installed_path_sha256'] == array_sha256(expected)
        np.testing.assert_array_equal(refs[method], original[method])
        for row in probe['states']:
            selected = row['methods'][method]
            np.testing.assert_array_equal(np.asarray(selected['reference_world'])[:, :2], expected[selected['indices'], :2])
    assert roundtrip_differs_from_source_bitwise  # Fixture explicitly exercises floating-point roundtrip.
    assert probe['installation'][METHODS[1]]['installed_reference_world'] == probe['installation'][METHODS[2]]['installed_reference_world']

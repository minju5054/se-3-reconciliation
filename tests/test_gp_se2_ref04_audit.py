"""Saved-record audit checks; analytic fixtures are not experiment evidence.

No tracker is constructed and no controller, GP, VLA or new rollout is run.
"""
from copy import deepcopy
import ast
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

from reconciliation import gp_se2_ref04_audit as audit
from reconciliation.robotless_online import integrate_unicycle
from reconciliation.se2 import wrap_angle

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'data/robotless_gp_se2_ref_03/primary_20260919T141000Z'
CASE = SOURCE / 'controls/episode_017_repeat_00__handoff_007'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def analytic_saved_commands():
    """Explicit fixed-command state records, with no execution/controller API."""
    pose = np.array([1., -2., math.pi-.02])
    initial = pose.copy()
    states = [dict(tick=0, time_s=0., pose_world=pose.tolist())]
    commands = []
    for tick in range(180):
        command = np.array([.2, .4]) if tick < 60 else np.array([.1, -.3])
        commands.append(dict(tick=tick, time_s=tick/60., end_time_s=(tick+1)/60.,
                             command=command.tolist(), solve_index=tick//6, held=bool(tick % 6)))
        pose = integrate_unicycle(pose, command, 1/60.)
        states.append(dict(tick=tick+1, time_s=(tick+1)/60., pose_world=pose.tolist()))
    return dict(horizon_s=3., control_hz=10., integration_hz=60., initial_pose_world=initial.tolist(),
                states=states, commands=commands)


def prediction_record(time_s=.7, pose=(1., 2., 3.13), command=(.2, .4)):
    poses = [np.asarray(pose, float)]
    for _ in range(5):
        p = poses[-1]
        poses.append(p + .1*np.array([command[0]*math.cos(p[2]),
                                     command[0]*math.sin(p[2]), command[1]]))
    return dict(time_s=time_s, input_pose_world=list(pose), prediction_world=np.asarray(poses).tolist(),
                actual_submitted_reference_world=np.asarray(poses[1:]).tolist(), command=list(command),
                success=True, error=None)


def test_prediction_has_current_node_plus_five_future_times_without_truncation():
    record = prediction_record(time_s=2.9)
    original = deepcopy(record)
    result = audit.prediction_data(record)
    assert result['available'] and result['current_node_matches_input']
    assert np.asarray(result['poses_world']).shape == (6, 3)
    np.testing.assert_array_equal(result['poses_world'], record['prediction_world'])
    np.testing.assert_allclose(result['node_times_s'], 2.9+np.arange(6)*.1, atol=1e-15, rtol=0)
    assert result['node_times_s'][-1] > 3.
    # Yaw is the solver's continuous prediction, not a visual wrap artifact.
    assert np.asarray(result['poses_world'])[-1, 2] > np.pi
    assert record == original


def test_missing_prediction_is_explicit_na_without_reference_substitution():
    record = prediction_record()
    record['prediction_world'] = None
    result = audit.prediction_data(record)
    assert result['available'] is False and result['reason']
    assert result['poses_world'] is None and result['node_times_s'] is None


@pytest.mark.parametrize('bad', [[], [[0., 0., 0.]]*5, [[0., 0.]]*6,
                               [[0., 0., float('nan')]]*6, [[0., 0., float('inf')]]*6])
def test_malformed_prediction_never_becomes_fabricated_six_node_path(bad):
    record = prediction_record()
    record['prediction_world'] = bad
    result = audit.prediction_data(record)
    assert not result['available'] and result['reason']
    assert result['poses_world'] is None and result['node_times_s'] is None


def test_current_prediction_node_preserves_logged_unwrapped_origin_and_reports_mismatch():
    record = prediction_record()
    result = audit.prediction_data(record)
    assert result['current_node_matches_input'] and result['current_node_bitwise_equal']
    # The official projection stores this node at the exact solve origin.
    # An equivalent but rewritten yaw is not the originally logged state.
    record['prediction_world'][0][2] += 2*np.pi
    result = audit.prediction_data(record)
    assert not result['current_node_matches_input']
    assert not result['current_node_bitwise_equal']
    record = prediction_record()
    record['prediction_world'][0][0] += .01
    result = audit.prediction_data(record)
    assert not result['current_node_matches_input']


def test_execution_reconstruction_uses_saved_held_commands_and_refuses_extrapolation():
    rollout = analytic_saved_commands()
    original = deepcopy(rollout)
    for t in (0., .1, .725, 1., 1.002, 2.999, 3.):
        if t in (0., 3.):
            expected = rollout['states'][0 if t == 0. else -1]['pose_world']
        else:
            tick = min(int(np.searchsorted([r['time_s'] for r in rollout['commands']], t,
                                         side='right')-1), 179)
            elapsed = t-rollout['commands'][tick]['time_s']
            expected = rollout['states'][tick]['pose_world'] if elapsed == 0 else integrate_unicycle(
                rollout['states'][tick]['pose_world'], rollout['commands'][tick]['command'], elapsed)
        np.testing.assert_allclose(audit.execution_at(rollout, t), expected, atol=1e-10, rtol=0)
    for t in (-.01, 3.0001, float('nan'), float('inf')):
        with pytest.raises(ValueError):
            audit.execution_at(rollout, t)
    assert rollout == original


def test_euler_prediction_and_exact_executed_arc_are_separate_with_same_command():
    pose = np.array([2., -1., 1.2]); command = np.array([.7, -.8])
    expected = pose + .1*np.array([command[0]*np.cos(pose[2]), command[0]*np.sin(pose[2]), command[1]])
    euler = np.asarray(audit.euler_endpoint(pose, command))
    exact = integrate_unicycle(pose, command, .1)
    np.testing.assert_allclose(euler, expected, atol=1e-15, rtol=0)
    assert np.linalg.norm(exact[:2]-euler[:2]) > .001
    assert abs(wrap_angle(exact[2]-euler[2])) < 1e-14
    # Recovered controls use Euler displacement along the preceding heading;
    # they are a reconstruction, not a claimed saved optimized control array.
    delta = euler-pose
    recovered = [np.dot(delta[:2], [np.cos(pose[2]), np.sin(pose[2])])/.1, delta[2]/.1]
    np.testing.assert_allclose(recovered, command, atol=1e-14, rtol=0)


def test_audit_source_has_no_solver_constructor_new_rollout_or_process_call():
    tree = ast.parse(Path(audit.__file__).read_text())
    forbidden_modules = {'casadi', 'jax', 'subprocess', 'multiprocessing'}
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            assert not forbidden_modules.intersection(name.name.split('.')[0] for name in node.names)
        elif isinstance(node, ast.ImportFrom):
            assert (node.module or '').split('.')[0] not in forbidden_modules
        elif isinstance(node, ast.Call):
            name = node.func.id if isinstance(node.func, ast.Name) else node.func.attr if isinstance(node.func, ast.Attribute) else ''
            assert name not in {'MpcTracker', 'MPCController', 'counterfactual_rollout',
                                'historical_solve_audit', 'solve_gp', 'solve_rigid',
                                'solve', 'minimize', 'submit', 'poll', 'Popen'}


def forbid_new_execution(monkeypatch):
    """Even indirect legacy execution entry points must remain unused."""
    from reconciliation import gp_se2_rollout, gp_se2_ref02_rollout

    def forbidden(*args, **kwargs):
        raise AssertionError('saved audit attempted a new solve or rollout')

    for module, names in (
        (gp_se2_rollout, ('counterfactual_rollout', 'historical_solve_audit', '_synchronous_solve')),
        (gp_se2_ref02_rollout, ('counterfactual_rollout', 'make_diagnostic_tracker', 'diagnostic_synchronous_solve')),
    ):
        for name in names:
            monkeypatch.setattr(module, name, forbidden)


def test_execution_guard_patch_is_scoped_and_restores_original_entry_points(monkeypatch):
    from reconciliation import gp_se2_rollout, gp_se2_ref02_rollout
    original = (gp_se2_rollout.counterfactual_rollout, gp_se2_ref02_rollout.make_diagnostic_tracker)
    with monkeypatch.context() as scoped:
        forbid_new_execution(scoped)
        assert gp_se2_rollout.counterfactual_rollout is not original[0]
        assert gp_se2_ref02_rollout.make_diagnostic_tracker is not original[1]
    assert gp_se2_rollout.counterfactual_rollout is original[0]
    assert gp_se2_ref02_rollout.make_diagnostic_tracker is original[1]


@pytest.fixture
def saved_source():
    if not (SOURCE/'validation.json').exists():
        pytest.skip('authenticated local REF-03 saved artifacts are unavailable; no synthetic replacement')
    from reconciliation.gp_se2_environment import HospitalEnvironment
    read = lambda p: json.loads(p.read_text())
    source = read(SOURCE/'source.json')
    assert read(SOURCE/'validation.json')['valid']
    official = source['official_mpc']
    assert digest(official['mpc_source']) == official['mpc_source_sha256']
    config = yaml.safe_load((SOURCE/'config_snapshot.yaml').read_text())
    env = HospitalEnvironment.load(source['environment_path'])
    route = read(CASE/'goal_route.json')
    files = [SOURCE/'source.json', SOURCE/'validation.json', SOURCE/'config_snapshot.yaml',
             CASE/'goal_route.json', CASE/'input_context.json', Path(official['mpc_source'])]
    files += [ROOT/'src/reconciliation'/name for name in
              ('gp_se2_ref02_rollout.py', 'gp_se2_rollout.py', 'robotless_online.py', 'gp_se2_evaluation.py')]
    methods = {}
    for method in audit.METHODS:
        folder = CASE/'methods'/method
        files += [folder/'rollout.json', folder/'metrics.json', folder/'reference_world.npy', folder/'row_provenance.json']
        methods[method] = (read(folder/'rollout.json'), read(folder/'metrics.json'))
    files += [Path(source['environment_path'])/row['path'] for row in source['environment_file_hashes']]
    before = {str(p): digest(p) for p in files}
    yield methods, route, env, config
    assert {str(p): digest(p) for p in files} == before


def test_saved_stress_records_reproduce_original_acceptance_without_new_execution(monkeypatch, saved_source):
    forbid_new_execution(monkeypatch)
    methods, route, env, config = saved_source
    for method, (rollout, stored) in methods.items():
        original = deepcopy(rollout)
        result = audit.audit_method(rollout, stored, route, env, config)
        # This asserts preservation, not an expected B/C scientific outcome.
        assert result['original_outcome_reproduced']
        assert all(result['original_outcome_field_matches'].values())
        assert result['original_outcome']['primary_success'] == stored['primary_success']
        assert result['original_outcome']['route'] == stored['route']
        assert result['stored_counts'] == dict(states=181, integration_commands=180, control_solves=30)
        assert result['reconstruction']['max_xy_error_m'] <= 1e-10
        assert result['reconstruction']['max_periodic_yaw_error_rad'] <= 1e-10
        assert result['stored_initial_physical_command'] == rollout['initial_physical_command']
        assert result['stored_initial_previous_control'] == rollout['initial_previous_control']
        assert len(result['per_solve']) == 30
        beyond_horizon = []
        for k, row in enumerate(result['per_solve']):
            saved = rollout['controller_reference_selections'][k]
            assert row['solve_index'] == k and row['issue_time_s'] == saved['time_s']
            assert row['selected_reference']['poses_world'] == saved['actual_submitted_reference_world']
            assert row['selected_reference']['actual_local_reference_max_error'] <= 1e-8
            assert row['selected_reference']['installed_target_xy_max_error_m'] <= 1e-8
            assert row['selected_reference']['installed_target_periodic_yaw_max_error_rad'] <= 1e-8
            assert row['selected_reference']['connections_are_diagnostic_not_execution']
            pred = row['prediction']; prefix = row['applied_prefix']
            assert pred['available'] and pred['current_node_matches_input']
            assert pred['poses_world'] == saved['prediction_world']
            assert not pred['future_solved_controls_available'] and not pred['preclipping_control_available']
            assert prefix['applied_command'] == saved['command']
            assert prefix['previous_control'] == saved['previous_control']
            assert prefix['model_discrepancy']['exact_vs_saved_actual']['xy_m'] <= 1e-10
            assert prefix['model_discrepancy']['exact_vs_saved_actual']['yaw_rad'] <= 1e-10
            for future in pred['future_actual_comparison']:
                if not future['actual_available']:
                    beyond_horizon.append(future)
                    assert future['actual_pose_world'] is None and future['discrepancy'] is None
                    assert future['absolute_time_s'] > 3.
                    assert future['missing_reason'] == 'BEYOND_RECORDED_EXECUTION_HORIZON'
                else:
                    np.testing.assert_allclose(future['actual_pose_world'], audit.execution_at(
                        rollout, future['absolute_time_s']), atol=1e-10, rtol=0)
        assert len(beyond_horizon) == 10
        assert rollout == original


def test_missing_saved_prediction_stays_na_and_wrong_historical_outcome_is_detected(monkeypatch, saved_source):
    forbid_new_execution(monkeypatch)
    methods, route, env, config = saved_source
    rollout, stored = deepcopy(methods[audit.METHODS[0]])
    # Deliberate corruption fixture in memory only; original records remain intact.
    rollout['controller_reference_selections'][0]['prediction_world'] = None
    stored['primary_success'] = not stored['primary_success']
    result = audit.audit_method(rollout, stored, route, env, config)
    assert not result['original_outcome_reproduced']
    assert result['original_outcome_field_matches']['primary_success'] is False
    first = result['per_solve'][0]
    assert first['prediction']['available'] is False
    assert first['prediction']['poses_world'] is first['prediction']['geometry'] is None
    assert first['prediction']['future_actual_comparison'] == []
    assert first['applied_prefix']['model_discrepancy'] is None
    assert first['applied_prefix']['geometry'] is not None
    assert first['evaluation_completeness'] == 'MISSING_PREDICTION'
    # A deliberately duplicated in-memory summary fixture checks status logic;
    # it is not a new B/C comparison or experimental observation.
    grouped = audit.assemble_audit({method: deepcopy(result) for method in audit.METHODS})
    assert grouped['summary']['diagnosis_status'] == 'PARTIALLY_LOCALIZED'
    assert all(row['prediction_records_available'] == 29 for row in grouped['summary']['methods'].values())
    for key in ('new_MPC_solves', 'new_GP_or_rigid_solves', 'new_VLA_inferences', 'new_rollouts'):
        assert grouped['summary'][key] == 0


def test_bad_actual_target_shape_is_rejected_without_replacing_saved_reference(monkeypatch, saved_source):
    forbid_new_execution(monkeypatch)
    methods, route, env, config = saved_source
    rollout, stored = deepcopy(methods[audit.METHODS[0]])
    rollout['controller_reference_selections'][0]['actual_submitted_reference_world'].pop()
    with pytest.raises(ValueError, match='5 finite target poses'):
        audit.audit_method(rollout, stored, route, env, config)


def test_runner_refuses_existing_run_and_audit_before_consuming_or_overwriting(tmp_path, monkeypatch):
    # Loading this script configures its process environment. Keep those import
    # effects scoped to the fixture as well as the explicit execution guards.
    monkeypatch.setattr(sys, 'path', sys.path.copy())
    for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
        monkeypatch.setenv(name, os.environ.get(name, '1'))
    spec = importlib.util.spec_from_file_location('ref04_runner_test', ROOT/'scripts/run_gp_se2_ref04.py')
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    run = tmp_path/'existing'; run.mkdir()
    preserved = run/'audit.json'; preserved.write_text('{"preserved": true}\n')
    before = preserved.read_bytes()

    def forbidden(*args, **kwargs):
        raise AssertionError('existing output must be rejected before reading sources or recomputing')

    monkeypatch.setattr(runner, 'compute', forbidden)
    monkeypatch.setattr(runner, 'read', forbidden)
    with pytest.raises(FileExistsError, match='no overwrite'):
        runner.prepare(run)
    with pytest.raises(FileExistsError, match='no overwrite'):
        runner.audit(run)
    assert preserved.read_bytes() == before

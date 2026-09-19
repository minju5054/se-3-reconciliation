"""Frozen runner integration tests; all optimizer outputs below are unit stubs."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from reconciliation.gp_se2_02_protocol import (
    CASES, GP_METHODS, INITIALIZATIONS, METHODS, RecordingDerivatives,
    classify_derivative_error, execution_order, select_retained_start, validate_config,
)
from reconciliation.gp_se2_diag02_derivatives import DerivativeError
from reconciliation.gp_se2_diag_fixtures import make_fixture_problem

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def runner():
    spec = importlib.util.spec_from_file_location('gp02_runner_unit_test', ROOT/'scripts/run_gp_se2_02.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_original_fixed_order_has_16_gp_starts_and_unchanged_rigid_pairs():
    order = execution_order()
    assert len(order) == 20
    assert [row['case_id'] for row in order[::5]] == [case for case, _ in CASES]
    for case_index, (case, role) in enumerate(CASES):
        group = order[case_index*5:(case_index+1)*5]
        assert all(row['case_id'] == case and row['case_role'] == role for row in group)
        assert group[0]['method'] == 'M1_RIGID'
        assert group[0]['initializations'] == ['identity', 'finite_lookahead_alignment']
        assert [(row['method'], row['initialization']) for row in group[1:]] == [
            (method, init) for method in GP_METHODS for init in INITIALIZATIONS]


@pytest.mark.parametrize('field,value', [
    ('prepared_solve_seconds_per_gp_start', 31), ('max_iterations', 201),
    ('blas_threads', 2), ('solver_retry', True), ('new_inference', True),
    ('new_environment_export', True), ('new_initialization', True),
    ('gp_initializations', ['I0_FRESH', 'constant_twist']),
    ('methods', ['M0_NATIVE']),
])
def test_config_rejects_changed_methods_seed_solver_and_runtime_budget(field, value):
    config = yaml.safe_load((ROOT/'configs/gp_se2_02.yaml').read_text())
    validate_config(config)
    config[field] = value
    with pytest.raises(ValueError): validate_config(config)


def test_config_rejects_outcome_selected_case_or_role_or_order():
    base = yaml.safe_load((ROOT/'configs/gp_se2_02.yaml').read_text())
    for operation in ('case', 'role', 'order'):
        config = deepcopy(base)
        if operation == 'case': config['cases'][0]['case_id'] = 'episode_017_repeat_00/handoff_007'
        if operation == 'role': config['cases'][0]['role'] = 'HARD'
        if operation == 'order': config['cases'].reverse()
        with pytest.raises(ValueError, match='fixed cases'): validate_config(config)


def candidate(cost, full_valid=True):
    return dict(candidate_found=True, candidate_vector=np.arange(150, dtype=float),
                candidate_objective=cost, constraint_report={'feasible': True},
                selected_iterate='callback_0001', selected_full_feasible=full_valid)


def test_across_start_selection_minimizes_dense_objective_without_full_invalid_substitution():
    lower_invalid, higher_valid = candidate(1, False), candidate(2, True)
    assert select_retained_start([lower_invalid, higher_valid]) == 0
    assert select_retained_start([higher_valid, lower_invalid]) == 1
    assert select_retained_start([candidate(1), candidate(1)]) == 0
    assert select_retained_start([{'candidate_found': False}, {'candidate_found': False}]) is None
    assert select_retained_start([{'termination': 'DERIVATIVE_UNSUPPORTED'}, candidate(3)]) == 1


@pytest.mark.parametrize('change', [dict(candidate_vector=None), dict(candidate_objective=None),
                                    dict(candidate_objective=float('nan')), dict(constraint_report={'feasible': False})])
def test_selection_cannot_invent_or_accept_invalid_retained_record(change):
    bad = candidate(1); bad.update(change)
    with pytest.raises(ValueError): select_retained_start([bad, candidate(2)])


@pytest.mark.parametrize('callback', ['objective_gradient', 'equality_jacobian', 'inequality_jacobian'])
def test_recording_provider_forwards_exact_error_and_vector_without_fallback(callback):
    error = DerivativeError('original exact cause', reason_code='UNSUPPORTED_WRAP_CUT', diagnostics={'support_index': 7})
    class Provider:
        def __getattr__(self, name):
            def call(z): raise error
            return call
    provider = RecordingDerivatives(Provider())
    vector = np.arange(150, dtype=float)
    with pytest.raises(DerivativeError) as raised: getattr(provider, callback)(vector)
    assert raised.value is error
    vector[0] = -123.
    assert provider.errors[0]['callback'] == callback
    assert provider.errors[0]['reason_code'] == error.reason_code
    assert provider.errors[0]['classification'] == 'DERIVATIVE_UNSUPPORTED'
    assert provider.errors[0]['diagnostics'] == error.diagnostics
    assert provider.errors[0]['vector'][0] == 0.


def test_recording_provider_forwards_unmodified_values_and_stats():
    expected = np.arange(150.)
    class Provider:
        def objective_gradient(self, z): return expected
        def stats(self): return {'verified': True}
    provider = RecordingDerivatives(Provider())
    assert provider.objective_gradient(np.zeros(150)) is expected
    assert provider.stats() == {'verified': True}
    assert not provider.errors


def test_numerical_provider_failure_is_not_falsely_called_unsupported_domain():
    from reconciliation.gp_se2_diag02_environment import UnsupportedEnvironmentDerivative
    failure = DerivativeError('nonfinite AD result')
    assert classify_derivative_error(failure) == 'NUMERICAL_FAILURE'
    failure.__cause__ = UnsupportedEnvironmentDerivative('missing boundary feature')
    assert classify_derivative_error(failure) == 'DERIVATIVE_UNSUPPORTED'


def test_settings_preserves_original_snapshot_bytes_and_detects_source_drift(runner, tmp_path):
    config = yaml.safe_load((ROOT/'configs/gp_se2_02.yaml').read_text())
    runner.write(tmp_path/'case_manifest_inputs.json', {'selected': []})
    runner.write(tmp_path/'source.json', dict(original_config_sha256=runner.digest(ROOT/'configs/gp_se2_01.yaml'),
        experiment_config_sha256=runner.digest(ROOT/'configs/gp_se2_02.yaml'), preserved_core_sha256={}, input_sha256={},
        fixed_manifest_inputs_sha256=runner.digest(tmp_path/'case_manifest_inputs.json')))
    (tmp_path/'config_snapshot.yaml').write_bytes((ROOT/'configs/gp_se2_01.yaml').read_bytes())
    (tmp_path/'experiment_config.yaml').write_bytes((ROOT/'configs/gp_se2_02.yaml').read_bytes())
    loaded, _ = runner.settings(tmp_path)
    assert loaded == config
    original = yaml.safe_load((tmp_path/'config_snapshot.yaml').read_text())
    assert original['experiment'] == 'GP-SE2-01'
    assert original['formulation']['wall_time_s'] == 30.
    assert original['formulation']['horizon_s'] == 3.
    (tmp_path/'config_snapshot.yaml').write_text((tmp_path/'config_snapshot.yaml').read_text()+'\n')
    with pytest.raises(ValueError, match='original configuration changed'): runner.settings(tmp_path)


@pytest.mark.parametrize('target', ['input_context.json', 'initializations/I0_FRESH.npy'])
def test_settings_rejects_changed_copied_inputs_or_seed_before_solver(runner, tmp_path, target):
    folder = tmp_path/'cases'/'unit_case'
    (folder/'initializations').mkdir(parents=True)
    (folder/'input_context.json').write_text('{"unit_only":true}')
    runner.array(folder/'initializations/I0_FRESH.npy', np.zeros(150))
    runner.write(tmp_path/'case_manifest_inputs.json', {'selected': [dict(case_directory='unit_case',
        copied_input_sha256={'input_context.json': runner.digest(folder/'input_context.json')},
        initialization_artifact_sha256={'I0_FRESH.npy': runner.digest(folder/'initializations/I0_FRESH.npy')})]})
    (tmp_path/'config_snapshot.yaml').write_bytes((ROOT/'configs/gp_se2_01.yaml').read_bytes())
    (tmp_path/'experiment_config.yaml').write_bytes((ROOT/'configs/gp_se2_02.yaml').read_bytes())
    runner.write(tmp_path/'source.json', dict(original_config_sha256=runner.digest(tmp_path/'config_snapshot.yaml'),
        experiment_config_sha256=runner.digest(tmp_path/'experiment_config.yaml'), preserved_core_sha256={}, input_sha256={},
        fixed_manifest_inputs_sha256=runner.digest(tmp_path/'case_manifest_inputs.json')))
    runner.settings(tmp_path)
    (folder/target).write_bytes((folder/target).read_bytes()+b'corrupted')
    with pytest.raises(ValueError, match='changed'): runner.settings(tmp_path)


def test_unavailable_start_preserves_original_vector_and_never_adds_raw_fallback(runner):
    vector = np.arange(150.)
    result = runner.unavailable_start('I0_FRESH', vector, 'DERIVATIVE_UNSUPPORTED', {'precise': 'cut'})
    assert result['initial_vector'] is vector
    assert result['termination'] == 'DERIVATIVE_UNSUPPORTED'
    assert not result['candidate_found'] and result['candidate_world'] is None
    assert result['unsupported_detail'] == {'precise': 'cut'}
    assert not result['solver_executed'] and not result['fallback_used']
    assert result['solve_wall_time_s'] == 0


@pytest.mark.parametrize('first_status', ['VERIFIED', 'DERIVATIVE_UNSUPPORTED'])
def test_four_case_optimize_orchestration_without_real_solver_or_rollout(runner, monkeypatch, tmp_path, first_status):
    """Exercise complete file/schema/control flow using labelled unit-only stubs."""
    import reconciliation.gp_se2_environment as environment
    import reconciliation.gp_se2_diag02_environment as derivatives_environment
    import reconciliation.gp_se2_diag02_derivatives as derivatives
    import reconciliation.gp_se2_diag02_solver as instrumented
    import reconciliation.gp_se2_formulation as formulation
    import reconciliation.gp_se2_02_evaluation as evaluation
    import run_gp_se2_diag_02 as prior_runner
    from reconciliation.gp_se2_02_transfer import seed_vectors
    config = yaml.safe_load((ROOT/'configs/gp_se2_02.yaml').read_text())
    original_config = yaml.safe_load((ROOT/'configs/gp_se2_01.yaml').read_text())
    source = dict(environment_path='unit_environment', primary_run='unit_primary', source_run=str(tmp_path/'raw_unit_source'))
    monkeypatch.setattr(runner, 'verify_frozen', lambda run: (config, source))
    monkeypatch.setattr(environment.HospitalEnvironment, 'load', lambda path: object())
    monkeypatch.setattr(derivatives_environment, 'EnvironmentDerivatives', lambda *a, **k: object())
    case_map, manifests, lookup, order, vectors_seen = {}, [], {}, [], {}
    full_true = dict(full_feasible=True, original_dense_feasible=True, original_plan_valid=True,
                     additional_grid={'valid': True}, original_plan_report={'plan_valid': True})
    for case_id, role in CASES:
        directory = case_id.replace('/', '__')
        folder = tmp_path/'cases'/directory
        (folder/'initializations').mkdir(parents=True)
        problem, _ = make_fixture_problem('S0')
        problem.config.update(original_config['formulation'])
        problem.unit_case_id = case_id
        common = problem.common_reference.copy()
        # Native and adapter are intentionally different arrays in this test.
        native = common.copy(); native[:, 0] += .002
        runner.array(folder/'F_native.npy', native); runner.array(folder/'F_common.npy', common)
        for name, vector in seed_vectors(problem).items():
            runner.array(folder/'initializations'/f'{name}.npy', vector)
            for method in GP_METHODS:
                lookup[case_id+'/'+method+'/'+name] = 'VERIFIED'
                runner.write(folder/'initializations'/f'initial_full_{method}_{name}.json', full_true)
        episode, handoff = case_id.split('/')
        context = dict(case_id=case_id, episode_id=episode, handoff_id=handoff,
                       B_world=problem.boundary_pose, u_minus=[.3, .1], previous_control=[.25, -.2])
        case_map[case_id] = dict(problem=problem, config=original_config, environment=object(),
            case_directory=folder, common_reference=common, context=context,
            goal_route={'goal_world': problem.goal_pose, 'gates': []})
        manifests.append(dict(case_id=case_id, case_role=role, case_directory=directory))
    lookup[CASES[0][0]+'/M2_GP_NO_OBSTACLE/I0_FRESH'] = first_status
    runner.write(tmp_path/'case_manifest.json', {'selected': manifests})
    runner.write(tmp_path/'derivative_transfer_checks/summary.json', {'start_lookup': lookup})
    def load(primary, case_id, method='M3_GP_CONSTRAINED', environment=None):
        case = case_map[case_id]
        case['problem'].include_obstacles = method == 'M3_GP_CONSTRAINED'
        case['problem'].unit_method = method
        return case
    monkeypatch.setattr(runner, 'load_frozen_case', load)
    class UnitProvider:
        def __init__(self, *args): pass
        def warmup(self, z): pass
        def reset_stats(self, **kwargs): pass
        def stats(self): return {'unit_stub': True}
    monkeypatch.setattr(derivatives, 'DerivativeProvider', UnitProvider)
    def rigid(boundary, twist, common, goal, config, **kwargs):
        # The real rigid solver is never called. Preserve its two-start schema.
        order.append(('M1_RIGID', None))
        return dict(status='CANDIDATE_FOUND', candidate_world=common, attempts=[dict(
            initialization=name, termination='CONVERGED', solver_success=True, iterations=1, wall_time_s=0.)
            for name in ('identity', 'finite_lookahead_alignment')])
    monkeypatch.setattr(formulation, 'solve_rigid', rigid)
    def solve(problem, z, initialization_name, derivative_provider):
        assert isinstance(derivative_provider, RecordingDerivatives)
        order.append((problem.unit_method, initialization_name))
        vectors_seen[(problem.unit_case_id, problem.unit_method, initialization_name)] = z.copy()
        pp, vv = problem.unpack(z)
        # I0 is lower cost. For first M2 only, full-invalid I0 must still be
        # selected and blocked, never substituted with higher-cost valid I1.
        full_valid = not (problem.unit_case_id == CASES[0][0] and
                         problem.unit_method == 'M2_GP_NO_OBSTACLE' and initialization_name == 'I0_FRESH')
        return dict(candidate(1 if initialization_name == 'I0_FRESH' else 2, full_valid),
            candidate_world=pp[1:], support_poses=pp, support_twists=vv, latest_iterate=z,
            termination='CONVERGED', solver_success=True, iterations=1,
            setup_wall_time_s=0., solve_wall_time_s=0., post_solve_validation_time_s=0.,
            initial_vector=z, callback_snapshots=[], candidate_checks=[], scope='UNIT_STUB_NOT_RESEARCH')
    monkeypatch.setattr(instrumented, 'run_instrumented', solve)
    def post(case, result, out):
        full = deepcopy(full_true)
        if not result['selected_full_feasible']:
            full.update(full_feasible=False, original_plan_valid=False, original_plan_report={'plan_valid': False})
        runner.write(out/'full_acceptance.json', full)
        return dict(termination=result['termination'], solver_success=True, iterations=1,
                    selected_full_feasible=full['full_feasible'], selected_objective=result['candidate_objective'],
                    independent_full_validation_time_s=0., solve_wall_time_s=0.)
    monkeypatch.setattr(prior_runner, 'postprocess', post)
    def plan(case, method, candidate, solver_result=None, full_acceptance=None):
        if method == 'M0_NATIVE': np.testing.assert_array_equal(candidate, np.load(case['case_directory']/'F_native.npy'))
        if method == 'M0_ADAPTER': np.testing.assert_array_equal(candidate, case['common_reference'])
        return dict(plan_valid=full_acceptance['full_feasible'] if full_acceptance else True,
                    candidate_available=candidate is not None)
    monkeypatch.setattr(evaluation, 'plan_for_method', plan)
    runner.optimize(tmp_path)
    skipped = int(first_status != 'VERIFIED')
    assert len(order) == 20-skipped
    expected_order = [(row['method'], row.get('initialization')) for i, row in enumerate(execution_order())
                      if not (skipped and i == 1)]
    assert order == expected_order
    assert len(vectors_seen) == 16-skipped
    for case_id, _ in CASES:
        for name in INITIALIZATIONS:
            if (case_id, GP_METHODS[0], name) in vectors_seen:
                assert vectors_seen[case_id, GP_METHODS[0], name].tobytes() == vectors_seen[case_id, GP_METHODS[1], name].tobytes()
    request = runner.read(tmp_path/'mpc_request.json')
    assert len(request['cases']) == 4
    if not skipped:
        assert request['cases'][0]['methods']['M2_GP_NO_OBSTACLE'] is None
        assert request['cases'][0]['rollout_eligibility']['M2_GP_NO_OBSTACLE']['missing_reason'] == 'PLAN_INVALID'
    else:
        assert request['cases'][0]['methods']['M2_GP_NO_OBSTACLE'] is not None
    first = runner.read(tmp_path/'cases'/CASES[0][0].replace('/', '__')/'methods/M2_GP_NO_OBSTACLE/solver_result.json')
    assert first['selected_initialization'] == skipped
    assert first['selected_full_feasible'] == bool(skipped)
    assert first['candidate_world'] is not None
    complete = runner.read(tmp_path/'optimization_completed.json')
    assert complete['actual_GP_solver_calls'] == 16-skipped and complete['rigid_starts'] == 8
    assert len(complete['starts']) == 24
    if skipped:
        record = next(row for row in complete['starts'] if row['case_id'] == CASES[0][0]
                      and row['method'] == 'M2_GP_NO_OBSTACLE' and row['initialization'] == 'I0_FRESH')
        assert record['termination'] == 'DERIVATIVE_UNSUPPORTED' and not record['solver_executed']
    assert not (tmp_path/'mpc_output').exists()

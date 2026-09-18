"""Harness accounting/selection tests use fake optimizers, not research runs."""
from types import SimpleNamespace

import numpy as np
import pytest

from reconciliation import gp_se2_diag_solver as diagnostic
from reconciliation import gp_se2_formulation as original
from reconciliation.se2 import compose_poses, se2_exp


def free(points):
    return np.full(np.asarray(points).shape[:-1], 2.)


def problem_and_witness():
    boundary = np.array([.8, -.3, .4])
    twist = np.array([.3, 0., .4])
    time = np.linspace(0., 3., 31)
    poses = compose_poses(boundary, se2_exp(time[:, None]*twist))
    velocity = np.tile(twist, (31, 1))
    problem = original.GPProblem(boundary, twist, poses[1:], poses[-1], {}, free, free)
    return problem, problem.vector(poses, velocity)


def fake_result(x, success=True, nit=1, nfev=1):
    return SimpleNamespace(x=x.copy(), success=success, status=0 if success else 9,
                           message='unit-test fake optimizer', nit=nit, nfev=nfev)


def test_scoped_profiling_preserves_exact_original_values_cache_and_restores_hooks():
    problem, vector = problem_and_witness()
    vector = vector + np.random.default_rng(912).normal(size=vector.shape)*.001
    baseline = problem.evaluate(vector)
    problem._last_x = None
    problem._last_eval = None
    names = ('gp_residual', 'solve_triangular', 'interpolate_interval', 'sample_gp', '_query')
    functions = {name: getattr(original, name) for name in names}
    profile = diagnostic._Profile(problem)
    with profile.hooks():
        value = profile.evaluate(vector, 'objective')
        again = profile.evaluate(vector.copy(), 'equality')
    assert value is again
    for key in baseline:
        np.testing.assert_array_equal(value[key], baseline[key])
    assert profile.cache_misses == profile.cache_hits == 1
    assert len(profile.unique_hashes) == 1
    assert profile.part_calls['environment_query'] == 2
    assert all(getattr(original, name) is function for name, function in functions.items())
    report = profile.report(0.)
    assert report['measured_instrumentation_bookkeeping_seconds'] >= 0.
    assert all(seconds >= 0. for seconds in report['numerical_component_seconds'].values())


def test_known_feasible_return_is_not_misclassified_as_recovery(monkeypatch):
    problem, vector = problem_and_witness()
    originals = [problem.boundary_pose.copy(), problem.initial_twist.copy(),
                 problem.common_reference.copy(), problem.goal_pose.copy(), dict(problem.config)]

    def optimizer(fun, x, **kw):
        assert kw['method'] == 'SLSQP'
        assert 'jac' not in kw
        assert kw['options'] == {'maxiter': 200, 'ftol': 1e-7, 'disp': False}
        assert all('jac' not in item for item in kw['constraints'])
        fun(x)
        for item in kw['constraints']:
            item['fun'](x)
        kw['callback'](x)
        return fake_result(x)

    monkeypatch.setattr(diagnostic, 'minimize', optimizer)
    result = diagnostic.run_instrumented(problem, vector, require_single_blas=False)
    assert result['candidate_found'] and result['initial_feasible']
    assert result['returned_initial_unchanged'] and result['feasible_initial_retained']
    assert not result['recovered_from_infeasible']
    assert not result['selected_objective_improved']
    assert not result['selected_objective_improved_beyond_ftol']
    assert result['objective_delta'] == 0.
    assert result['variable_count'] == 150
    assert result['equality_count'] == 30
    assert result['inequality_count'] == 903
    assert result['objective_evaluations'] == result['equality_evaluations'] == result['inequality_evaluations'] == 1
    assert result['profiling']['unique_vector_evaluations'] == 1
    assert result['callback_count'] == 1
    assert result['callback_snapshots'][0]['source'] == 'actual scipy SLSQP callback'
    for before, after in zip(originals[:-1], [problem.boundary_pose, problem.initial_twist,
                                            problem.common_reference, problem.goal_pose]):
        np.testing.assert_array_equal(before, after)
    assert originals[-1] == problem.config


def test_feasible_intermediate_survives_infeasible_latest_and_is_recovery(monkeypatch):
    problem, witness = problem_and_witness()
    perturbed = witness.copy()
    perturbed[6] += .01
    calls_inside_optimizer = []
    real_dense = problem.dense_report
    in_optimizer = False

    def dense(x):
        calls_inside_optimizer.append(in_optimizer)
        return real_dense(x)

    problem.dense_report = dense

    def optimizer(fun, initial, **kw):
        nonlocal in_optimizer
        in_optimizer = True
        fun(initial)
        kw['callback'](witness)
        kw['callback'](perturbed)
        in_optimizer = False
        return fake_result(perturbed, success=False, nit=2)

    monkeypatch.setattr(diagnostic, 'minimize', optimizer)
    result = diagnostic.run_instrumented(problem, perturbed, require_single_blas=False,
                                        post_diagnostics=lambda p, x: {'recorded': True})
    assert not result['initial_feasible']
    assert result['candidate_found'] and result['recovered_from_infeasible']
    assert result['selected_iterate'] == 'callback_0001'
    assert not result['returned_initial_unchanged']
    assert result['selected_objective_improved']
    assert result['selected_objective_improved_beyond_ftol']
    assert result['objective_delta'] < -result['objective_improvement_absolute_threshold']
    assert not result['solver_success_matches_sampled_candidate']
    assert result['termination'] == 'SOLVER_FAILURE'
    assert not any(calls_inside_optimizer)
    assert all(item['independent_diagnostics']['recorded'] for item in result['candidate_checks'])
    np.testing.assert_array_equal(result['candidate_vector'], witness)


def test_timeout_keeps_latest_and_no_fallback(monkeypatch):
    problem, witness = problem_and_witness()
    invalid = witness.copy()
    invalid[11] += .01

    def optimizer(fun, initial, **kw):
        fun(initial)
        kw['callback'](initial)
        raise original._WallTimeExceeded('unit test timeout')

    monkeypatch.setattr(diagnostic, 'minimize', optimizer)
    result = diagnostic.run_instrumented(problem, invalid, require_single_blas=False)
    assert result['termination'] == 'TIMEOUT'
    assert not result['candidate_found']
    assert result['candidate_vector'] is result['candidate_world'] is result['candidate_objective'] is None
    assert not result['infeasibility_proven'] and not result['fallback_used']
    assert result['callback_count'] == 1
    np.testing.assert_array_equal(result['latest_iterate'], invalid)
    assert result['full_candidate_acceptance'] == 'NOT_EVALUATED_BY_THIS_SOLVER_HARNESS'


def test_numerical_failure_is_separate_from_known_feasible_initial(monkeypatch):
    problem, witness = problem_and_witness()
    saved_hook = original.interpolate_interval

    def optimizer(*args, **kwargs):
        raise FloatingPointError('unit test numerical failure')

    monkeypatch.setattr(diagnostic, 'minimize', optimizer)
    result = diagnostic.run_instrumented(problem, witness, require_single_blas=False)
    assert result['termination'] == 'NUMERICAL_FAILURE'
    assert result['candidate_found'] and result['feasible_initial_retained']
    assert not result['solver_success']
    assert original.interpolate_interval is saved_hook


def test_single_thread_environment_is_checked_without_modifying_it(monkeypatch):
    for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
        monkeypatch.setenv(name, '9')
    with pytest.raises(ValueError, match='BLAS thread'):
        diagnostic.blas_thread_state(True)
    report = diagnostic.blas_thread_state(False)
    assert report['environment']['OPENBLAS_NUM_THREADS'] == '9'
    assert not report['single_thread_environment_valid']


@pytest.mark.parametrize('bad', [np.zeros(149), np.full(150, np.nan)])
def test_invalid_chart_never_starts_solver(bad, monkeypatch):
    problem, _ = problem_and_witness()
    monkeypatch.setattr(diagnostic, 'minimize', lambda *a, **kw: pytest.fail('must not optimize'))
    with pytest.raises(ValueError, match='initial'):
        diagnostic.run_instrumented(problem, bad, require_single_blas=False)


@pytest.mark.parametrize('cost', [-1., np.nan, np.inf])
def test_invalid_pre_solve_cost_rejected(cost):
    problem, witness = problem_and_witness()
    with pytest.raises(ValueError, match='pre_solve_budget_cost_s'):
        diagnostic.run_instrumented(problem, witness, pre_solve_budget_cost_s=cost,
                                    require_single_blas=False)


def test_pre_solve_work_is_charged_against_original_budget(monkeypatch):
    clock = [0.]

    def now():
        clock[0] += .00001
        return clock[0]

    def optimizer(fun, x, **kw):
        clock[0] += 1.
        fun(x)
        return fake_result(x)

    monkeypatch.setattr(diagnostic.time, 'perf_counter', now)
    monkeypatch.setattr(diagnostic, 'minimize', optimizer)
    problem, witness = problem_and_witness()
    baseline = diagnostic.run_instrumented(problem, witness, require_single_blas=False)
    problem, witness = problem_and_witness()
    charged = diagnostic.run_instrumented(problem, witness, require_single_blas=False,
                                         pre_solve_budget_cost_s=29.5)
    assert baseline['termination'] == 'CONVERGED'
    assert charged['termination'] == 'TIMEOUT'
    assert charged['config']['wall_time_s'] == baseline['config']['wall_time_s'] == 30.
    assert charged['remaining_solver_budget_s'] == .5
    assert charged['solve_and_pre_solve_wall_time_s'] > 30.
    assert charged['pre_solve_budget_cost_s'] == 29.5


def test_failed_callback_evaluation_preserves_actual_vector_and_unknown_residuals(monkeypatch):
    problem, witness = problem_and_witness()
    original_evaluate = diagnostic._Profile.evaluate

    def evaluate(self, x, caller):
        if caller == 'callback':
            raise FloatingPointError('unit test callback evaluation failure')
        return original_evaluate(self, x, caller)

    def optimizer(fun, x, **kw):
        kw['callback'](x)
        pytest.fail('callback should have raised')

    monkeypatch.setattr(diagnostic._Profile, 'evaluate', evaluate)
    monkeypatch.setattr(diagnostic, 'minimize', optimizer)
    result = diagnostic.run_instrumented(problem, witness, require_single_blas=False)
    assert result['termination'] == 'NUMERICAL_FAILURE'
    assert result['callback_count'] == 1
    assert result['completed_callback_evaluations'] == 0
    snapshot = result['callback_snapshots'][0]
    np.testing.assert_array_equal(snapshot['vector'], witness)
    assert snapshot['objective'] is snapshot['collocation'] is None

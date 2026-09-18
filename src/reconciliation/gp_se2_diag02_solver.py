"""DIAG-02 paired harness, derived literally from DIAG-01 with explicit Jacobians.

The original evaluator, primal cache, callback retention and post-solve candidate
selection are identical in both modes. Historical harness and core are unchanged.
Only the three supplied derivative callbacks and their timing/multiplier capture
are added. This module is sequential and uses the existing scoped primal profiler.
"""
from __future__ import annotations
import time
from typing import Callable
import numpy as np
from scipy import __version__ as scipy_version
from scipy.optimize import minimize
from . import gp_se2_formulation as original
from .gp_se2_diag_solver import _Profile, _timer_calibration, blas_thread_state, _NUMERICAL_ERRORS
from .gp_se2_diag02_derivatives import DerivativeError

_NUMERICAL_ERRORS = _NUMERICAL_ERRORS + (DerivativeError,)

def run_instrumented(problem: original.GPProblem, initial: np.ndarray, *,
                     initialization_name: str = 'provided_support_states',
                     post_diagnostics: Callable[[original.GPProblem, np.ndarray], dict] | None = None,
                     pre_solve_budget_cost_s: float = 0.,
                     require_single_blas: bool = True,
                     derivative_provider=None) -> dict:
    """Run one original SLSQP start; return actual history and separate checks.

    ``post_diagnostics`` may add independent residual/time/unit diagnostics. Its
    result cannot change original sampled candidate acceptance. Full independent
    environment acceptance remains the caller's responsibility, exactly as in
    GP-SE2-01. ``pre_solve_budget_cost_s`` charges measured initialization or
    restoration work against the same frozen wall budget. There is no fallback
    and no retry.
    """
    if not isinstance(problem, original.GPProblem):
        raise TypeError('problem must be the original GPProblem')
    initial = np.array(initial, dtype=float, copy=True)
    expected = 5*(len(problem.times)-1)
    if initial.shape != (expected,) or not np.all(np.isfinite(initial)):
        raise ValueError('initial must be a finite original optimization-chart vector')
    if not np.isfinite(pre_solve_budget_cost_s) or pre_solve_budget_cost_s < 0:
        raise ValueError('pre_solve_budget_cost_s must be finite and nonnegative')
    pre_solve_budget_cost_s = float(pre_solve_budget_cost_s)
    setup_start = time.perf_counter()
    threads = blas_thread_state(require_single_blas)
    timer_pair = _timer_calibration()
    profile = _Profile(problem)
    # Reset only transient memoization, never the source inputs/configuration.
    problem._last_x = None
    problem._last_eval = None
    setup_seconds = time.perf_counter()-setup_start
    config = problem.config
    latest = initial.copy()
    snapshots = []
    iterations = 0
    result_nfev = None
    initial_e = None
    multipliers = None
    derivative_calls = dict(objective_gradient=0, equality_jacobian=0, inequality_jacobian=0)
    derivative_seconds = dict(objective_gradient=0., equality_jacobian=0., inequality_jacobian=0.)
    solve_start = time.perf_counter()

    def check_budget():
        if pre_solve_budget_cost_s+time.perf_counter()-solve_start >= config['wall_time_s']:
            raise original._WallTimeExceeded('frozen per-initialization wall-time budget reached')

    def requested(x, caller, field):
        check_budget()
        return profile.evaluate(x, caller)[field]

    def derivative(x, name):
        check_budget()
        begin = time.perf_counter()
        derivative_calls[name] += 1
        try:
            value = np.asarray(getattr(derivative_provider, name)(x), dtype=np.float64)
            if not np.all(np.isfinite(value)):
                raise FloatingPointError('nonfinite supplied derivative: '+name)
            return value
        finally:
            derivative_seconds[name] += time.perf_counter()-begin
            check_budget()

    def callback(x):
        nonlocal latest, iterations
        if not np.all(np.isfinite(x)):
            raise FloatingPointError('solver produced a nonfinite iterate')
        begin = profile.now()
        latest = np.array(x, copy=True)
        iterations += 1
        # Preserve the actual callback even if its entry reaches the budget.
        # The original implementation also updates latest before this check.
        elapsed = time.perf_counter()-solve_start
        snapshot = dict(iteration=iterations, elapsed_s=elapsed, vector=latest.copy(),
                        objective=None, collocation=None, equality_residuals=None,
                        inequality_margins=None, evaluation_complete=False,
                        source='actual scipy SLSQP callback')
        snapshots.append(snapshot)
        profile.bookkeeping_seconds += profile.now()-begin
        check_budget()
        value = profile.evaluate(x, 'callback')
        begin = profile.now()
        report = original._constraint_report(value['equality'], value['inequality'], config)
        snapshot.update(objective=float(value['objective']), collocation=report,
                        equality_residuals=value['equality'].copy(),
                        inequality_margins=value['inequality'].copy(), evaluation_complete=True)
        profile.bookkeeping_seconds += profile.now()-begin
        check_budget()

    with profile.hooks():
        try:
            # The historical harness evaluates the initial point before its
            # first wall-budget check; retain that convention.
            initial_e = profile.evaluate(initial, 'initial')
            constraints = []
            if len(initial_e['equality']):
                constraints.append({'type': 'eq', 'fun': lambda x: requested(x, 'equality', 'equality')})
            if len(initial_e['inequality']):
                constraints.append({'type': 'ineq', 'fun': lambda x: requested(x, 'inequality', 'inequality')})
            jacobian = None
            if derivative_provider is not None:
                jacobian = lambda x: derivative(x, 'objective_gradient')
                for item in constraints:
                    name = 'equality_jacobian' if item['type']=='eq' else 'inequality_jacobian'
                    item['jac'] = lambda x, name=name: derivative(x, name)
            result = minimize(lambda x: requested(x, 'objective', 'objective'), initial,
                              jac=jacobian, method='SLSQP', constraints=constraints, callback=callback,
                              options={'maxiter': int(config['max_iterations']),
                                       'ftol': config['ftol'], 'disp': False})
            if np.all(np.isfinite(result.x)):
                latest = np.array(result.x, copy=True)
            result_nfev = int(result.nfev)
            multipliers = np.asarray(result.multipliers).copy() if hasattr(result, 'multipliers') else None
            status = dict(termination='CONVERGED' if result.success else 'SOLVER_FAILURE',
                          solver_success=bool(result.success), solver_status=int(result.status),
                          message=str(result.message), iterations=int(result.nit))
        except original._WallTimeExceeded as exc:
            status = dict(termination='TIMEOUT', solver_success=False, solver_status=None,
                          message=str(exc), iterations=iterations)
        except _NUMERICAL_ERRORS as exc:
            status = dict(termination='NUMERICAL_FAILURE', solver_success=False, solver_status=None,
                          message=f'{type(exc).__name__}: {exc}', iterations=iterations)
    solve_seconds = time.perf_counter()-solve_start
    profile_report = profile.report(timer_pair)

    # Heavy sampled validation is timed separately and cannot consume solver
    # iteration budget. No artificial intermediate iterate is manufactured.
    post_start = time.perf_counter()
    probes = [('initial', initial), ('latest_iterate', latest)]
    probes.extend((f'callback_{s["iteration"]:04d}', s['vector']) for s in snapshots
                  if s['collocation'] is not None and s['collocation']['feasible'])
    checks, choices = [], []
    original_check_seconds = independent_check_seconds = 0.
    for label, vector in probes:
        entry = dict(iterate=label, vector=np.array(vector, copy=True))
        begin = time.perf_counter()
        try:
            value = problem.evaluate(vector)
            dense = problem.dense_report(vector)
            entry.update(objective=float(value['objective']),
                         collocation=original._constraint_report(value['equality'], value['inequality'], config),
                         constraint_report=dense)
            if dense['feasible']:
                choices.append((float(value['objective']), label, vector.copy(), dense))
        except _NUMERICAL_ERRORS as exc:
            entry.update(objective=None, collocation=None,
                         constraint_report=dict(feasible=False, error=f'{type(exc).__name__}: {exc}'))
        original_check_seconds += time.perf_counter()-begin
        if post_diagnostics is not None:
            begin = time.perf_counter()
            try:
                entry['independent_diagnostics'] = post_diagnostics(problem, vector.copy())
            except _NUMERICAL_ERRORS as exc:
                entry['independent_diagnostics'] = dict(error=f'{type(exc).__name__}: {exc}')
            independent_check_seconds += time.perf_counter()-begin
        checks.append(entry)
    selected = min(choices, key=lambda row: (row[0], row[1])) if choices else None
    initial_feasible = bool(checks[0]['constraint_report']['feasible'])
    initial_objective = checks[0]['objective']
    improved = bool(selected is not None and initial_objective is not None
                    and selected[0] < initial_objective)
    objective_delta = (None if selected is None or initial_objective is None
                       else float(selected[0]-initial_objective))
    selected_same_initial = bool(selected is not None and np.array_equal(selected[2], initial))
    post_seconds = time.perf_counter()-post_start
    poses, twists = problem.unpack(latest)
    status.update(
        initialization=initialization_name, solver_backend='scipy.optimize.SLSQP',
        solver_version=scipy_version, jacobian_method=('SciPy numerical forward two-point finite differences; jac not supplied' if derivative_provider is None else 'JAX float64 AD plus analytic environment gradients; all three callbacks supplied'),
        derivative_mode='FD_BASELINE' if derivative_provider is None else 'SUPPLIED_JAC',
        derivative_calls=derivative_calls, derivative_callback_seconds=derivative_seconds,
        derivative_provider_stats=None if derivative_provider is None else derivative_provider.stats(),
        scipy_qp_multipliers=multipliers,
        optimization_chart='right-local SE(2) pose delta (m,m,rad), body vx (m/s), omega (rad/s) per future support',
        variable_count=expected,
        equality_count=None if initial_e is None else len(initial_e['equality']),
        inequality_count=None if initial_e is None else len(initial_e['inequality']),
        config=dict(config), blas_threads=threads, initial_vector=initial,
        latest_iterate=latest, latest_support_poses=poses, latest_support_twists=twists,
        objective_evaluations=profile.calls.get('objective', 0), scipy_result_nfev=result_nfev,
        equality_evaluations=profile.calls.get('equality', 0),
        inequality_evaluations=profile.calls.get('inequality', 0),
        callback_snapshots=snapshots, callback_count=len(snapshots),
        completed_callback_evaluations=sum(s['evaluation_complete'] for s in snapshots),
        callback_history_scope='actual callbacks only; internal line-search and FD probes are counted, not trajectory history',
        callback_dense_checks_deferred=True,
        timing_schedule_differs_from_historical=True,
        candidate_checks=checks, candidate_found=selected is not None,
        selected_iterate=None if selected is None else selected[1],
        candidate_vector=None if selected is None else selected[2],
        candidate_objective=None if selected is None else selected[0],
        constraint_report=None if selected is None else selected[3],
        initial_feasible=initial_feasible, returned_initial_unchanged=selected_same_initial,
        feasible_initial_retained=initial_feasible and selected_same_initial,
        recovered_from_infeasible=not initial_feasible and selected is not None,
        selected_objective_improved=improved,
        selected_objective_improvement_definition='strict floating-point comparison; also report beyond-ftol criterion',
        objective_delta=objective_delta,
        objective_delta_definition='selected candidate objective minus initial objective',
        selected_objective_improved_beyond_ftol=bool(objective_delta is not None and objective_delta < -config['ftol']),
        objective_improvement_absolute_threshold=float(config['ftol']),
        solver_success_matches_sampled_candidate=bool(status['solver_success']) == bool(selected is not None),
        independent_environment_check_required=True,
        full_candidate_acceptance='NOT_EVALUATED_BY_THIS_SOLVER_HARNESS',
        setup_wall_time_s=setup_seconds, solve_wall_time_s=solve_seconds,
        pre_solve_budget_cost_s=pre_solve_budget_cost_s,
        configured_wall_time_budget_s=float(config['wall_time_s']),
        remaining_solver_budget_s=max(0., float(config['wall_time_s'])-pre_solve_budget_cost_s),
        solve_and_pre_solve_wall_time_s=pre_solve_budget_cost_s+solve_seconds,
        wall_time_s=solve_seconds, post_solve_validation_time_s=post_seconds,
        original_dense_check_time_s=original_check_seconds,
        independent_diagnostics_time_s=independent_check_seconds,
        total_setup_solve_post_wall_time_s=setup_seconds+solve_seconds+post_seconds,
        total_pre_setup_solve_post_wall_time_s=pre_solve_budget_cost_s+setup_seconds+solve_seconds+post_seconds,
        profiling=profile_report, infeasibility_proven=False, fallback_used=False,
        candidate_selection='minimum original objective among original dense-feasible initial/latest/actual collocation-feasible callbacks',
    )
    if selected is not None:
        support, velocity = problem.unpack(selected[2])
        status.update(support_poses=support, support_twists=velocity,
                      candidate_world=support[1:].copy())
    else:
        status.update(support_poses=None, support_twists=None, candidate_world=None)
    return status

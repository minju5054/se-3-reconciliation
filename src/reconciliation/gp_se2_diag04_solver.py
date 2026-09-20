"""Bounded DIAG-04 SLSQP adapter and post-solve equality-rank diagnostics.

The DIAG-02 callback, budget, and dense-first retention policy is retained. A
separate constraint view is necessary because the historical harness requires a
GPProblem and its dense report would otherwise omit the appended quarter rows.
No historical class, evaluator, or module global is changed. Expensive original
full checks and SVD diagnostics run only after the prepared solve has ended.
"""
from __future__ import annotations

from hashlib import sha256
import time

import numpy as np
from scipy import __version__ as scipy_version
from scipy.optimize import minimize

from . import gp_se2_formulation as original
from .gp_se2_diag_acceptance import check_full_candidate
from .gp_se2_diag_solver import _Profile, _timer_calibration, blas_thread_state
from .gp_se2_diag02_derivatives import DerivativeError


NUMERICAL_ERRORS = (ValueError, FloatingPointError, np.linalg.LinAlgError, DerivativeError)
RANK_PROTOCOL = {
    "column_scales_per_future_support": [1., 1., 1., .8, 3.],
    "column_units": ["m", "m", "rad", "m/s", "rad/s"],
    "scaling_definition": "J_scaled = J_raw @ diag(column_scales); diagnostic only",
    "machine_rank_threshold": "max(matrix.shape) * float64_epsilon * largest_singular_value",
    "relative_cutoffs": [1e-12, 1e-10, 1e-8, 1e-6],
    "near_zero_row_norm_absolute_threshold": 1e-12,
    "interpretation": "numerical sensitivity diagnostic; not a proof of infeasibility",
    "locations": ["initial", "latest_iterate", "selected"],
    "timing": "outside prepared solve; never in callback",
}


def vector_hash(vector):
    return sha256(np.asarray(vector, dtype=np.float64).tobytes()).hexdigest()


def _error(exc):
    return {"error": f"{type(exc).__name__}: {exc}",
            "reason_code": getattr(exc, "reason_code", None),
            "diagnostics": getattr(exc, "diagnostics", None)}


def _post_checks(view, initial, latest, snapshots, case):
    """Check only initial/latest and actual solver-grid-feasible callbacks.

    Full results never change dense-first selection. Thus a selected candidate
    which fails full acceptance remains selected and rejected, even if another
    inspected vector passes the full checker. Duplicate vectors share numerical
    checks, but all source labels and their actual discovery times are retained.
    """
    base = view.base_problem
    probes = [("initial", initial, 0.), ("latest_iterate", latest, None)]
    probes.extend((f'callback_{s["iteration"]:04d}', s["vector"], s["elapsed_s"])
                  for s in snapshots if s["collocation"] is not None
                  and s["collocation"]["feasible"])
    checks, choices, cache = [], [], {}
    dense_s = full_s = grid_s = 0.
    for label, vector, elapsed in probes:
        identity = vector_hash(vector)
        cached = identity in cache
        if not cached:
            entry = {}
            began = time.perf_counter()
            try:
                value = view.evaluate(vector)
                grid = original._constraint_report(value["equality"], value["inequality"], base.config)
                entry.update(objective=float(value["objective"]), collocation=grid,
                             equality_residuals=value["equality"].copy(),
                             inequality_margins=value["inequality"].copy())
            except NUMERICAL_ERRORS as exc:
                entry.update(objective=None, collocation={"feasible": False, **_error(exc)})
            grid_s += time.perf_counter()-began
            began = time.perf_counter()
            try:
                # Always the unchanged original problem, never the constraint view.
                entry["constraint_report"] = base.dense_report(vector)
            except NUMERICAL_ERRORS as exc:
                entry["constraint_report"] = {"feasible": False, **_error(exc)}
            dense_s += time.perf_counter()-began
            entry["candidate_eligible"] = bool(entry["collocation"]["feasible"]
                and entry["constraint_report"]["feasible"])
            began = time.perf_counter()
            try:
                entry["full_acceptance"] = check_full_candidate(base, vector, case)
                entry["full_acceptance_available"] = True
            except NUMERICAL_ERRORS as exc:
                entry["full_acceptance"] = {"full_feasible": None, **_error(exc)}
                entry["full_acceptance_available"] = False
            full_s += time.perf_counter()-began
            entry["original_full_feasible"] = (bool(entry["full_acceptance"]["full_feasible"])
                if entry["full_acceptance_available"] else None)
            entry["grid_and_full_feasible"] = (bool(entry["collocation"]["feasible"]
                and entry["original_full_feasible"]) if entry["full_acceptance_available"] else None)
            cache[identity] = entry
        check = dict(cache[identity], iterate=label, vector=np.array(vector, copy=True),
                     vector_sha256=identity, reused_vector_check=cached,
                     discovered_callback_elapsed_s=elapsed,
                     certification="post-solve; discovery time is not certification time")
        checks.append(check)
        if check["candidate_eligible"]:
            choices.append((check["objective"], label, np.array(vector, copy=True), check))
    selected = min(choices, key=lambda row: (row[0], row[1])) if choices else None
    return checks, selected, {"original_dense_check_time_s": dense_s,
        "independent_full_check_time_s": full_s, "post_grid_check_time_s": grid_s,
        "unique_candidate_vectors_checked": len(cache), "candidate_source_labels_checked": len(checks)}


def run_refined(view, initial, *, derivative_provider, case,
                initialization_name="provided_saved_initialization", require_single_blas=True):
    """One prepared, supplied-Jacobian SLSQP solve, with no retry or fallback."""
    base = view.base_problem
    if not isinstance(base, original.GPProblem):
        raise TypeError("view.base_problem must be the unchanged original GPProblem")
    if derivative_provider is None:
        raise ValueError("both DIAG-04 grids require all three supplied derivative callbacks")
    initial = np.array(initial, dtype=np.float64, copy=True)
    expected = 5*(len(base.times)-1)
    if initial.shape != (expected,) or not np.all(np.isfinite(initial)):
        raise ValueError("initial must be finite original-chart vector")
    began = time.perf_counter()
    threads = blas_thread_state(require_single_blas)
    timer_pair = _timer_calibration()
    profile = _Profile(view)
    view.clear_cache()
    if hasattr(view, "reset_stats"):
        view.reset_stats()
    setup_s = time.perf_counter()-began
    config = base.config
    latest, snapshots = initial.copy(), []
    iterations = 0
    minimize_invocations = 0
    initial_e = None
    result_nfev = multipliers = None
    derivative_calls = dict(objective_gradient=0, equality_jacobian=0, inequality_jacobian=0)
    derivative_seconds = {name: 0. for name in derivative_calls}
    derivative_errors = []
    solve_start = time.perf_counter()

    def check_budget():
        if time.perf_counter()-solve_start >= config["wall_time_s"]:
            raise original._WallTimeExceeded("frozen per-initialization wall-time budget reached")

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
                raise FloatingPointError("nonfinite supplied derivative: "+name)
            return value
        finally:
            derivative_seconds[name] += time.perf_counter()-begin
            check_budget()

    def callback(x):
        nonlocal latest, iterations
        if not np.all(np.isfinite(x)):
            raise FloatingPointError("solver produced a nonfinite iterate")
        begin = profile.now()
        latest = np.array(x, copy=True)
        iterations += 1
        snapshot = dict(iteration=iterations, elapsed_s=time.perf_counter()-solve_start,
            vector=latest.copy(), objective=None, collocation=None, equality_residuals=None,
            inequality_margins=None, evaluation_complete=False, source="actual scipy SLSQP callback")
        snapshots.append(snapshot)
        profile.bookkeeping_seconds += profile.now()-begin
        check_budget()
        value = profile.evaluate(x, "callback")
        begin = profile.now()
        snapshot.update(objective=float(value["objective"]),
            collocation=original._constraint_report(value["equality"], value["inequality"], config),
            equality_residuals=value["equality"].copy(), inequality_margins=value["inequality"].copy(),
            evaluation_complete=True)
        profile.bookkeeping_seconds += profile.now()-begin
        check_budget()

    # Unlike the historical detailed profiler, no module-global hooks are used.
    # The same inclusive evaluator profiler and callback policy apply to G0/G1.
    try:
        initial_e = profile.evaluate(initial, "initial")
        constraints = []
        for kind, field, name in (("eq", "equality", "equality_jacobian"),
                                  ("ineq", "inequality", "inequality_jacobian")):
            if len(initial_e[field]):
                constraints.append({"type": kind,
                    "fun": lambda x, f=field: requested(x, f, f),
                    "jac": lambda x, n=name: derivative(x, n)})
        minimize_invocations += 1
        result = minimize(lambda x: requested(x, "objective", "objective"), initial,
            jac=lambda x: derivative(x, "objective_gradient"), method="SLSQP",
            constraints=constraints, callback=callback,
            options={"maxiter": int(config["max_iterations"]), "ftol": config["ftol"], "disp": False})
        if np.all(np.isfinite(result.x)):
            latest = np.array(result.x, copy=True)
        result_nfev = int(result.nfev)
        multipliers = np.asarray(result.multipliers).copy() if hasattr(result, "multipliers") else None
        status = dict(termination="CONVERGED" if result.success else "SOLVER_FAILURE",
            solver_success=bool(result.success), solver_status=int(result.status),
            message=str(result.message), iterations=int(result.nit))
    except original._WallTimeExceeded as exc:
        status = dict(termination="TIMEOUT", solver_success=False, solver_status=None,
                      message=str(exc), iterations=iterations)
    except NUMERICAL_ERRORS as exc:
        if isinstance(exc, DerivativeError):
            derivative_errors.append(_error(exc))
        status = dict(termination="NUMERICAL_FAILURE", solver_success=False, solver_status=None,
                      message=f"{type(exc).__name__}: {exc}", iterations=iterations)
    solve_s = time.perf_counter()-solve_start
    profiling = profile.report(timer_pair)
    profiling["component_hooks"] = "disabled in both grids; no module-global mutation"
    profiling["numerical_component_instrumentation_available"] = False
    profiling["numerical_component_seconds"] = None
    profiling["numerical_component_calls"] = None
    profiling["other_evaluate_inclusive_seconds"] = None
    profiling["component_cost_unavailability_reason"] = "inner GP/environment functions not separately instrumented"
    profiling["unique_vector_definition"] = "distinct float64 vector bytes; includes actual line-search evaluation probes"
    provider_stats = derivative_provider.stats()
    view_stats = view.stats() if hasattr(view, "stats") else None
    post_start = time.perf_counter()
    checks, selected, check_timing = _post_checks(view, initial, latest, snapshots, case)
    post_s = time.perf_counter()-post_start
    latest_poses, latest_twists = base.unpack(latest)
    selected_entry = None if selected is None else selected[3]
    initial_feasible = checks[0]["candidate_eligible"]
    objective_delta = None if selected is None or checks[0]["objective"] is None else float(selected[0]-checks[0]["objective"])
    same_initial = bool(selected is not None and np.array_equal(initial, selected[2]))
    selected_full = bool(selected_entry is not None and selected_entry["grid_and_full_feasible"])
    status.update(grid=view.grid_name, initialization=initialization_name,
        solver_backend="scipy.optimize.SLSQP", solver_version=scipy_version,
        minimize_invocations=minimize_invocations,
        derivative_mode="SUPPLIED_JAC", derivative_calls=derivative_calls,
        derivative_callback_seconds=derivative_seconds, derivative_provider_stats=provider_stats,
        constraint_view_stats=view_stats,
        recorded_derivative_errors=derivative_errors, scipy_qp_multipliers=multipliers,
        variable_count=expected, equality_count=None if initial_e is None else len(initial_e["equality"]),
        inequality_count=None if initial_e is None else len(initial_e["inequality"]),
        config=dict(config), blas_threads=threads, initial_vector=initial, initial_vector_sha256=vector_hash(initial),
        latest_iterate=latest, latest_support_poses=latest_poses, latest_support_twists=latest_twists,
        objective_evaluations=profile.calls.get("objective", 0), scipy_result_nfev=result_nfev,
        equality_evaluations=profile.calls.get("equality", 0), inequality_evaluations=profile.calls.get("inequality", 0),
        callback_snapshots=snapshots, callback_count=len(snapshots),
        callback_history_scope="actual callbacks only; internal line-search probes counted, not saved as callback history",
        completed_callback_evaluations=sum(s["evaluation_complete"] for s in snapshots),
        callback_dense_checks_deferred=True, candidate_checks=checks, candidate_found=selected is not None,
        selected_iterate=None if selected is None else selected[1],
        candidate_vector=None if selected is None else selected[2],
        candidate_objective=None if selected is None else selected[0],
        constraint_report=None if selected is None else selected_entry["constraint_report"],
        selected_grid_report=None if selected is None else selected_entry["collocation"],
        selected_full_acceptance=None if selected is None else selected_entry["full_acceptance"],
        selected_full_feasible=selected_full, initial_feasible=bool(initial_feasible),
        initial_full_feasible=checks[0]["grid_and_full_feasible"], latest_full_feasible=checks[1]["grid_and_full_feasible"],
        inspected_full_feasible_iterate_exists=any(c["grid_and_full_feasible"] for c in checks),
        returned_initial_unchanged=same_initial, feasible_initial_retained=bool(initial_feasible and same_initial),
        recovered_from_infeasible=bool(not initial_feasible and selected is not None),
        selected_objective_improved=bool(objective_delta is not None and objective_delta < 0.),
        selected_objective_improved_beyond_ftol=bool(objective_delta is not None and objective_delta < -config["ftol"]),
        objective_delta=objective_delta, objective_delta_definition="selected minus initial original objective",
        setup_wall_time_s=setup_s, solve_wall_time_s=solve_s, configured_wall_time_budget_s=float(config["wall_time_s"]),
        pre_solve_budget_cost_s=0., wall_time_s=solve_s, post_solve_validation_time_s=post_s,
        total_setup_solve_post_wall_time_s=setup_s+solve_s+post_s, profiling=profiling,
        candidate_selection="minimum original objective, then lexical source label, among grid-and-original-dense feasible initial/latest/actual grid-feasible callbacks",
        full_check_does_not_reselect=True, independent_full_checker_receives_original_base=True,
        infeasibility_proven=False, fallback_used=False, new_mpc_solves=0, new_rollouts=0,
        **check_timing)
    if selected is not None:
        p, v = base.unpack(selected[2])
        status.update(support_poses=p, support_twists=v, candidate_world=p[1:].copy())
    else:
        status.update(support_poses=None, support_twists=None, candidate_world=None)
    return status


def equality_rank_diagnostics(g0_provider, g1_provider, vectors):
    """Analyze frozen seed/latest/selected vectors after solve; never optimize."""
    started = time.perf_counter()
    records, cache = [], {}
    for label, vector in vectors.items():
        if label not in RANK_PROTOCOL["locations"]:
            raise ValueError("rank diagnostics are restricted to initial/latest_iterate/selected")
        if vector is None:
            records.append(dict(source=label, available=False, reason="NO_SELECTED_CANDIDATE"))
            continue
        z = np.asarray(vector, dtype=np.float64)
        if z.ndim != 1 or len(z) % 5 or not np.all(np.isfinite(z)):
            raise ValueError("finite original five-column chart required for rank diagnostic")
        identity = vector_hash(z)
        scales = np.tile(RANK_PROTOCOL["column_scales_per_future_support"], len(z)//5)
        for grid, provider in (("G0_ORIGINAL", g0_provider), ("G1_QUARTER", g1_provider)):
            key = (grid, identity)
            cached = key in cache
            if not cached:
                begin = time.perf_counter()
                try:
                    jac = np.asarray(provider.equality_jacobian(z), dtype=np.float64)
                    if jac.ndim != 2 or jac.shape[1] != len(z) or not np.all(np.isfinite(jac)):
                        raise ValueError("nonfinite or incompatible equality Jacobian")
                    matrices = {}
                    for name, matrix in (("raw", jac), ("column_scaled", jac*scales)):
                        singular = np.linalg.svd(matrix, compute_uv=False)
                        largest = float(singular[0]) if len(singular) else 0.
                        machine = max(matrix.shape)*np.finfo(np.float64).eps*largest
                        ranks = {str(cutoff): int(np.count_nonzero(singular > largest*cutoff))
                                 for cutoff in RANK_PROTOCOL["relative_cutoffs"]}
                        row_norms = np.linalg.norm(matrix, axis=1)
                        full_row_rank = bool(np.count_nonzero(singular > machine) == matrix.shape[0])
                        matrices[name] = dict(shape=list(matrix.shape), singular_values=singular,
                            matrix_sha256=vector_hash(matrix), machine_threshold=machine,
                            machine_rank=int(np.count_nonzero(singular > machine)), relative_cutoff_ranks=ranks,
                            near_zero_row_indices=np.flatnonzero(row_norms <= RANK_PROTOCOL["near_zero_row_norm_absolute_threshold"]),
                            row_norms=row_norms, smallest_to_largest_ratio=None if largest == 0 else float(singular[-1]/largest),
                            full_row_rank=full_row_rank,
                            full_row_rank_condition_number=(float(largest/singular[-1])
                                if full_row_rank and len(singular) and singular[-1] > 0 else None))
                    entry = dict(available=True, matrices=matrices, column_scales=scales)
                except NUMERICAL_ERRORS as exc:
                    entry = dict(available=False, **_error(exc))
                entry["diagnostic_wall_s"] = time.perf_counter()-begin
                cache[key] = entry
            records.append(dict(cache[key], source=label, grid=grid, vector_sha256=identity,
                                reused_vector_diagnostic=cached, infeasibility_proven=False))
    return dict(protocol=RANK_PROTOCOL, records=records, unique_grid_vector_diagnostics=len(cache),
                wall_s=time.perf_counter()-started, solver_scaling_changed=False,
                equality_rows_removed=False, infeasibility_proven=False)

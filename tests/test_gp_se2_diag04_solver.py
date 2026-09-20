"""Mock-SLSQP policy tests; fixtures are not handoff outcome evidence."""
from types import SimpleNamespace

import numpy as np
import pytest

from reconciliation import gp_se2_diag04_solver as refined
from reconciliation import gp_se2_diag02_solver as historical
from reconciliation import gp_se2_formulation as original
from reconciliation.gp_se2_diag02_derivatives import DerivativeError


class MockBase(original.GPProblem):
    """Small five-variable policy fixture, with no physical research meaning."""
    def __init__(self):
        self.times = np.array([0., .1])
        self.config = dict(wall_time_s=30., max_iterations=200, ftol=1e-7,
                           equality_tolerance=1e-5, inequality_tolerance=1e-5)
        self._last_x = self._last_eval = None
        self.dense_calls = []

    def evaluate(self, z):
        if self._last_x is not None and np.array_equal(z, self._last_x):
            return self._last_eval
        self._last_x = np.array(z, copy=True)
        self._last_eval = dict(objective=float(z[0]), equality=np.array([0.]),
                               inequality=np.array([z[0]]))
        return self._last_eval

    def unpack(self, z):
        poses = np.zeros((2, 3))
        poses[1, 0] = z[0]
        return poses, np.zeros_like(poses)

    def dense_report(self, z):
        self.dense_calls.append(np.array(z, copy=True))
        return dict(feasible=bool(z[0] >= 0.))


class MockView:
    def __init__(self, base, grid="G0_ORIGINAL"):
        self.base_problem = base
        self.grid_name = grid
        self._last_x = self._last_eval = None

    def clear_cache(self):
        self._last_x = self._last_eval = None
        self.base_problem._last_x = self.base_problem._last_eval = None

    def evaluate(self, z):
        if self._last_x is not None and np.array_equal(z, self._last_x):
            return self._last_eval
        value = dict(self.base_problem.evaluate(z))
        if self.grid_name == "G1_QUARTER":
            value["equality"] = np.r_[value["equality"], z[2]]
            value["inequality"] = np.r_[value["inequality"], z[1]]
        self._last_x, self._last_eval = np.array(z, copy=True), value
        return value


class MockProvider:
    def __init__(self, eq=1, ineq=1):
        self.eq, self.ineq = eq, ineq
        self.calls = []

    def objective_gradient(self, z):
        self.calls.append("objective_gradient")
        return np.array([1., 0., 0., 0., 0.])

    def equality_jacobian(self, z):
        self.calls.append("equality_jacobian")
        return np.zeros((self.eq, len(z)))

    def inequality_jacobian(self, z):
        self.calls.append("inequality_jacobian")
        return np.zeros((self.ineq, len(z)))

    def stats(self):
        return dict(calls=list(self.calls))


def fake_slsqp(callback_first=(2., -.5), final=-1., *, success=True, code=0):
    def solve(fun, x, *, jac, method, constraints, callback, options):
        assert method == "SLSQP"
        assert options == dict(maxiter=200, ftol=1e-7, disp=False)
        fun(x)
        jac(x)
        assert [c["type"] for c in constraints] == ["eq", "ineq"]
        for constraint in constraints:
            constraint["fun"](x)
            constraint["jac"](x)
        for number in callback_first:
            y = x.copy()
            y[0] = number
            callback(y)
        z = x.copy()
        z[0] = final
        return SimpleNamespace(x=z, nfev=1, success=success, status=code,
                               message="mock outcome", nit=len(callback_first))
    return solve


@pytest.fixture
def full_spy(monkeypatch):
    calls = []
    def check(base, vector, case):
        assert isinstance(base, MockBase)
        assert case["case"] == "synthetic-policy-fixture"
        calls.append((base, np.array(vector, copy=True)))
        return dict(full_feasible=bool(vector[0] >= 3.), unchanged_original_base=True)
    monkeypatch.setattr(refined, "check_full_candidate", check)
    return calls


def run(view, provider=None):
    return refined.run_refined(view, np.array([4., 1., 0., 0., 0.]),
        derivative_provider=provider or MockProvider(),
        case={"case": "synthetic-policy-fixture"}, require_single_blas=False)


def test_invalid_low_objective_not_selected_and_full_failure_does_not_reselect(monkeypatch, full_spy):
    monkeypatch.setattr(refined, "minimize", fake_slsqp())
    provider = MockProvider()
    result = run(MockView(MockBase()), provider)
    assert result["solver_success"]
    assert result["minimize_invocations"] == 1
    assert result["selected_iterate"] == "callback_0001"
    assert result["candidate_objective"] == 2.
    assert result["latest_iterate"][0] == -1.
    assert result["candidate_vector"][0] == 2.
    assert not result["selected_full_feasible"]
    assert result["inspected_full_feasible_iterate_exists"]  # Initial only.
    assert [c["iterate"] for c in result["candidate_checks"]] == ["initial", "latest_iterate", "callback_0001"]
    assert len(full_spy) == 3
    assert set(provider.calls) == {"objective_gradient", "equality_jacobian", "inequality_jacobian"}
    assert result["candidate_checks"][2]["discovered_callback_elapsed_s"] >= 0.
    assert result["full_check_does_not_reselect"]
    assert result["new_mpc_solves"] == result["new_rollouts"] == 0
    assert result["profiling"]["numerical_component_calls"] is None


def test_g1_grid_is_required_even_if_original_dense_and_full_pass(monkeypatch, full_spy):
    monkeypatch.setattr(refined, "minimize", fake_slsqp(callback_first=(4.,), final=4.))
    initial = np.array([4., -1., 0., 0., 0.])
    result = refined.run_refined(MockView(MockBase(), "G1_QUARTER"), initial,
        derivative_provider=MockProvider(eq=2, ineq=2), case={"case": "synthetic-policy-fixture"},
        require_single_blas=False)
    assert result["candidate_vector"] is None
    assert not result["candidate_found"]
    assert result["candidate_checks"][0]["original_full_feasible"]
    assert not result["candidate_checks"][0]["grid_and_full_feasible"]
    assert result["candidate_checks"][0]["constraint_report"]["feasible"]
    assert not result["candidate_checks"][0]["collocation"]["feasible"]
    assert result["equality_count"] == result["inequality_count"] == 2


def test_duplicate_vectors_preserve_sources_and_lexical_tie_break(monkeypatch, full_spy):
    monkeypatch.setattr(refined, "minimize", fake_slsqp(callback_first=(4.,), final=4.))
    result = run(MockView(MockBase()))
    assert len(full_spy) == 1
    assert len(result["candidate_checks"]) == 3
    assert result["candidate_source_labels_checked"] == 3
    assert result["unique_candidate_vectors_checked"] == 1
    assert result["selected_iterate"] == "callback_0001"
    assert result["returned_initial_unchanged"]


def test_g0_historical_mock_runtime_parity_options_callbacks_selection(monkeypatch, full_spy):
    solve = fake_slsqp()
    monkeypatch.setattr(refined, "minimize", solve)
    monkeypatch.setattr(historical, "minimize", solve)
    initial = np.array([4., 1., 0., 0., 0.])
    before = historical.run_instrumented(MockBase(), initial,
        derivative_provider=MockProvider(), require_single_blas=False)
    after = run(MockView(MockBase()))
    for key in ("termination", "solver_success", "solver_status", "iterations", "candidate_found",
                "selected_iterate", "candidate_objective", "returned_initial_unchanged", "equality_count",
                "inequality_count", "derivative_calls", "callback_count", "objective_evaluations",
                "equality_evaluations", "inequality_evaluations", "scipy_result_nfev"):
        assert after[key] == before[key]
    for key in ("initial_vector", "latest_iterate", "candidate_vector"):
        np.testing.assert_array_equal(after[key], before[key])
    for a, b in zip(after["callback_snapshots"], before["callback_snapshots"]):
        for key in ("iteration", "objective", "collocation", "evaluation_complete"):
            assert a[key] == b[key]
        np.testing.assert_array_equal(a["vector"], b["vector"])


def test_timeout_preserves_actual_latest_and_callback_ledger(monkeypatch, full_spy):
    def timeout(fun, x, *, callback, **kwargs):
        y = x.copy()
        y[0] = 3.
        callback(y)
        raise original._WallTimeExceeded("mock bounded timeout")
    monkeypatch.setattr(refined, "minimize", timeout)
    result = run(MockView(MockBase()))
    assert result["termination"] == "TIMEOUT"
    assert not result["solver_success"]
    assert result["latest_iterate"][0] == 3.
    assert len(result["callback_snapshots"]) == 1
    assert result["selected_full_feasible"]
    assert not result["fallback_used"]


@pytest.mark.parametrize("failure", ["unsupported", "nonfinite"])
def test_derivative_failure_preserved_no_finite_difference_fallback(monkeypatch, full_spy, failure):
    monkeypatch.setattr(refined, "minimize", fake_slsqp())
    provider = MockProvider()
    def invalid(z):
        if failure == "unsupported":
            raise DerivativeError("fixture cut", reason_code="UNSUPPORTED_WRAP_CUT")
        return np.full(len(z), np.nan)
    provider.objective_gradient = invalid
    result = run(MockView(MockBase()), provider)
    assert result["termination"] == "NUMERICAL_FAILURE"
    assert result["derivative_mode"] == "SUPPLIED_JAC"
    assert result["callback_count"] == 0
    assert result["returned_initial_unchanged"]
    if failure == "unsupported":
        assert result["recorded_derivative_errors"][0]["reason_code"] == "UNSUPPORTED_WRAP_CUT"


def test_solver_numerical_status_preserved_even_with_retained_candidate(monkeypatch, full_spy):
    monkeypatch.setattr(refined, "minimize", fake_slsqp(callback_first=(), final=4., success=False, code=6))
    result = run(MockView(MockBase()))
    assert result["termination"] == "SOLVER_FAILURE"
    assert result["solver_status"] == 6
    assert result["selected_full_feasible"]
    assert result["infeasibility_proven"] is False


def test_supplied_derivatives_required_before_solver(monkeypatch):
    monkeypatch.setattr(refined, "minimize", lambda *a, **k: pytest.fail("must not solve"))
    with pytest.raises(ValueError, match="all three"):
        refined.run_refined(MockView(MockBase()), np.zeros(5), derivative_provider=None, case={})


def test_initial_evaluation_failure_has_zero_solver_invocations(monkeypatch, full_spy):
    monkeypatch.setattr(refined, "minimize", lambda *a, **k: pytest.fail("must not enter SLSQP"))
    view = MockView(MockBase())
    def invalid(z):
        raise ValueError("mock initial primal failure")
    view.evaluate = invalid
    result = run(view)
    assert result["termination"] == "NUMERICAL_FAILURE"
    assert result["minimize_invocations"] == 0
    assert result["callback_count"] == 0


def test_rank_sensitivity_scaled_matrix_and_duplicate_label_preservation():
    class Provider:
        def __init__(self, matrix):
            self.matrix = matrix
            self.calls = 0
        def equality_jacobian(self, z):
            self.calls += 1
            return self.matrix
    g0 = Provider(np.array([[1., 0., 0., 0., 0.], [0., 1e-11, 0., 0., 0.]]))
    g1 = Provider(np.array([[1., 0., 0., 0., 0.], [2., 0., 0., 0., 0.], [0., 0., 0., 0., 0.]]))
    result = refined.equality_rank_diagnostics(g0, g1,
        {"initial": np.zeros(5), "latest_iterate": np.zeros(5), "selected": None})
    assert g0.calls == g1.calls == 1
    assert result["unique_grid_vector_diagnostics"] == 2
    raw = result["records"][0]["matrices"]["raw"]
    assert raw["machine_rank"] == 2
    assert raw["relative_cutoff_ranks"]["1e-12"] == 2
    assert raw["relative_cutoff_ranks"]["1e-10"] == 1
    deficient = result["records"][1]["matrices"]["column_scaled"]
    assert deficient["machine_rank"] == 1
    np.testing.assert_array_equal(deficient["near_zero_row_indices"], [2])
    assert result["records"][-1]["available"] is False
    assert result["infeasibility_proven"] is False
    assert result["solver_scaling_changed"] is False
    assert result["equality_rows_removed"] is False
    assert result["protocol"]["column_scales_per_future_support"] == [1., 1., 1., .8, 3.]


def test_rank_diagnostic_failure_remains_unavailable():
    class Provider:
        def equality_jacobian(self, z):
            return np.full((1, len(z)), np.nan)
    result = refined.equality_rank_diagnostics(Provider(), Provider(), {"initial": np.zeros(5)})
    assert all(not row["available"] for row in result["records"])
    assert all(row["infeasibility_proven"] is False for row in result["records"])


def test_rank_column_scale_multiplies_speed_columns_only_for_diagnostic():
    matrix = np.array([[0., 0., 0., 1., 1.]])
    class Provider:
        def equality_jacobian(self, z):
            return matrix
    result = refined.equality_rank_diagnostics(Provider(), Provider(), {"initial": np.zeros(5)})
    matrices = result["records"][0]["matrices"]
    np.testing.assert_allclose(matrices["raw"]["singular_values"], [np.sqrt(2.)])
    np.testing.assert_allclose(matrices["column_scaled"]["singular_values"], [np.sqrt(.8**2+3.**2)])
    np.testing.assert_array_equal(matrix, [[0., 0., 0., 1., 1.]])


def test_rank_rejects_callback_scope():
    with pytest.raises(ValueError, match="restricted"):
        refined.equality_rank_diagnostics(None, None, {"callback_0001": np.zeros(5)})


def test_missing_full_check_not_reported_as_measured_physical_failure(monkeypatch):
    monkeypatch.setattr(refined, "minimize", fake_slsqp(callback_first=(), final=4.))
    def invalid(*args, **kwargs):
        raise ValueError("fixture missing geometry")
    monkeypatch.setattr(refined, "check_full_candidate", invalid)
    result = run(MockView(MockBase()))
    assert result["candidate_found"]
    assert result["initial_full_feasible"] is None
    assert result["latest_full_feasible"] is None
    assert not result["candidate_checks"][0]["full_acceptance_available"]
    assert result["candidate_checks"][0]["full_acceptance"]["full_feasible"] is None
    assert "missing geometry" in result["candidate_checks"][0]["full_acceptance"]["error"]

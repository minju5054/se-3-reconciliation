"""Synthetic implementation checks only; no actual-event optimization evidence."""
import inspect

import numpy as np
import pytest

pytest.importorskip("jax")

from reconciliation import gp_se2_diag04_constraints as module
from reconciliation.gp_se2 import interpolate_interval
from reconciliation.gp_se2_diag02_derivatives import DerivativeError, DerivativeProvider
from reconciliation.gp_se2_diag04_constraints import (
    ConstraintView, RefinedDerivativeProvider, G0_ORIGINAL, G1_QUARTER,
    constraint_row_metadata, quarter_motion_values,
)
from reconciliation.gp_se2_diag_fixtures import make_fixture_problem
from reconciliation.gp_se2_formulation import GPProblem, _motion_inequalities
from reconciliation.se2 import compose_poses, se2_exp


class AffineEnvironment:
    def __init__(self):
        self.calls = {"workspace": 0, "obstacle": 0}

    def workspace(self, xy):
        self.calls["workspace"] += 1
        return 3.+.3*xy[:, 0]-.1*xy[:, 1], np.tile([.3, -.1], (len(xy), 1)), {"family": "workspace"}

    def obstacle(self, xy):
        self.calls["obstacle"] += 1
        return 2.-.2*xy[:, 0]+.4*xy[:, 1], np.tile([-.2, .4], (len(xy), 1)), {"family": "obstacle"}


def fixture(name="S1", *, obstacles=True):
    original, data = make_fixture_problem(name)
    env = AffineEnvironment()
    frame = np.array([2., -3., 1.1])
    boundary = compose_poses(frame, original.boundary_pose)
    reference = compose_poses(frame, original.common_reference)
    goal = compose_poses(frame, original.goal_pose)
    problem = GPProblem(boundary, original.initial_twist, reference, goal, original.config,
                        lambda xy: env.obstacle(xy)[0], lambda xy: env.workspace(xy)[0], [], obstacles)
    vector = problem.vector(np.vstack((boundary, reference)), data["support_twists"])
    return problem, vector, env


@pytest.fixture(scope="module")
def ready():
    problem, vector, env = fixture()
    vector = vector+np.random.default_rng(104).normal(size=150)*np.tile([.003, .002, .002, .003, .002], 30)
    view = ConstraintView(problem, G1_QUARTER)
    base = DerivativeProvider(problem, env)
    provider = RefinedDerivativeProvider(view, base)
    provider.warmup(vector)
    return view, vector, provider, env


@pytest.mark.parametrize("obstacles,expected", [(False, 1293), (True, 1383)])
def test_original_objective_rows_environment_and_chart_are_unchanged(obstacles, expected):
    problem, vector, env = fixture(obstacles=obstacles)
    g0, g1 = ConstraintView(problem), ConstraintView(problem, G1_QUARTER)
    original = problem.evaluate(vector)
    assert g0.evaluate(vector) is original
    counts = dict(env.calls)
    refined = g1.evaluate(vector)
    assert env.calls == counts  # Quarter evaluation adds no environment query.
    assert refined["objective"] == original["objective"]
    for key in set(original)-{"equality", "inequality"}:
        np.testing.assert_array_equal(refined[key], original[key])
    np.testing.assert_array_equal(refined["equality"][:30], original["equality"])
    np.testing.assert_array_equal(refined["inequality"][:expected-480], original["inequality"])
    dimensions = g1.dimensions(vector)
    assert dimensions["equality_count"] == 90
    assert dimensions["inequality_count"] == expected
    assert dimensions["variable_count"] == 150 and dimensions["support_count"] == 31
    assert dimensions["appended_equality_count"] == 60
    assert dimensions["appended_inequality_count"] == 480
    assert dimensions["added_environment_rows"] == dimensions["added_goal_rows"] == 0
    assert not hasattr(g1, "dense_report") and not isinstance(g1, GPProblem)
    assert g1.base is problem


def test_quarter_rows_have_original_interpolation_family_order_sign_and_units(ready):
    view, vector, _, _ = ready
    p, v = view.unpack(vector)
    _, vv, aa = interpolate_interval(p[:-1, None], v[:-1, None], p[1:, None], v[1:, None], .1, np.array([.25, .75]))
    q = quarter_motion_values(view.base, vector)
    np.testing.assert_array_equal(q["quarter_equality"], vv[:, :, 1].ravel())
    np.testing.assert_array_equal(q["quarter_inequality"], _motion_inequalities(vv.reshape(-1, 3), aa.reshape(-1, 3), view.config))
    rows = view.appended_row_metadata(vector)
    assert len(rows) == 540
    assert {r["local_fraction"] for r in rows} == {.25, .75}
    assert {r["family"] for r in rows} == {"lateral"} | {r[0] for r in module.MOTION_ROWS}
    assert rows[0]["row_index"] == 30 and rows[1]["local_fraction"] == .75
    assert rows[60]["row_index"] == 903
    result = view.evaluate(vector)
    for row in rows:
        assert row["signed_residual_or_margin"] == result[row["constraint_kind"]][row["row_index"]]
        assert row["trajectory_time_s"] == view.times[row["interval_index"]]+.1*row["local_fraction"]
    full = constraint_row_metadata(view, vector)
    assert full["equality_count"] == 90 and full["inequality_count"] == 1383
    assert sum(r["family"] == "workspace" for r in full["rows"]) == 90
    assert sum(r["family"] == "obstacle" for r in full["rows"]) == 90
    assert sum(r["family"].startswith("goal_") for r in full["rows"]) == 3


def test_g0_provider_is_literal_original_delegation_and_no_quarter_graph(ready):
    view, z, provider, _ = ready
    base = provider.base_provider
    g0 = RefinedDerivativeProvider(ConstraintView(view.base), base)
    assert g0._jitted is None and g0._compiled is None
    for method in ("values", "objective_gradient", "equality_jacobian", "inequality_jacobian"):
        actual, expected = getattr(g0, method)(z), getattr(base, method)(z)
        if isinstance(actual, dict):
            for key in actual:
                np.testing.assert_array_equal(actual[key], expected[key])
        else:
            np.testing.assert_array_equal(actual, expected)
    assert g0.stats()["quarter_provider"]["compile_count"] == 0


def test_refined_ad_primal_and_all_original_derivative_rows_are_unchanged(ready):
    view, z, provider, _ = ready
    primal, ad = view.evaluate(z), provider.values(z)
    for key in ("objective", "equality", "inequality", "quarter_equality", "quarter_inequality"):
        np.testing.assert_allclose(ad[key], primal[key], atol=1e-8, rtol=1e-10)
    base = provider.base_provider
    np.testing.assert_array_equal(provider.objective_gradient(z), base.objective_gradient(z))
    np.testing.assert_array_equal(provider.equality_jacobian(z)[:30], base.equality_jacobian(z))
    np.testing.assert_array_equal(provider.inequality_jacobian(z)[:903], base.inequality_jacobian(z))
    assert provider.equality_jacobian(z).shape == (90, 150)
    assert provider.inequality_jacobian(z).shape == (1383, 150)


def test_nonzero_right_chart_quarter_jacobian_coordinate_fd(ready):
    view, z, provider, _ = ready
    columns = []
    step = 2e-6
    for j in range(150):
        offset = np.zeros(150); offset[j] = step
        minus, plus = quarter_motion_values(view.base, z-offset), quarter_motion_values(view.base, z+offset)
        columns.append(np.r_[plus["quarter_equality"]-minus["quarter_equality"],
                             plus["quarter_inequality"]-minus["quarter_inequality"]]/(2*step))
    actual = np.vstack((provider.equality_jacobian(z)[30:], provider.inequality_jacobian(z)[903:]))
    expected = np.stack(columns, axis=-1)
    np.testing.assert_allclose(actual, expected, atol=2e-5, rtol=3e-5)
    # Last 240 appended rows include both linear/angular acceleration signs.
    assert np.max(np.abs(actual[-240:, 2::5])) > 1.


def test_multistep_directional_quarter_derivative_trend(ready):
    view, z, provider, _ = ready
    jac = np.vstack((provider.equality_jacobian(z)[30:], provider.inequality_jacobian(z)[903:]))
    random = np.random.default_rng(20260919)
    for _ in range(3):
        direction = random.normal(size=150); direction /= np.linalg.norm(direction)
        errors = []
        for step in (2e-4, 2e-5, 2e-6):
            a, b = quarter_motion_values(view.base, z+step*direction), quarter_motion_values(view.base, z-step*direction)
            fd = np.r_[a["quarter_equality"]-b["quarter_equality"],
                       a["quarter_inequality"]-b["quarter_inequality"]]/(2*step)
            errors.append(float(np.max(np.abs(fd-jac@direction))))
            np.testing.assert_allclose(fd, jac@direction, atol=2e-5, rtol=3e-5)
        assert np.isfinite(errors).all()  # Trends retained; roundoff need not decrease monotonically.


def test_exact_vector_cache_and_no_additional_environment_derivatives(ready):
    view, z, provider, env = ready
    provider.reset_stats(clear_cache=True)
    before = dict(env.calls)
    provider.objective_gradient(z)
    counts = dict(env.calls)
    provider.equality_jacobian(z.copy()); provider.inequality_jacobian(z.copy())
    assert env.calls == counts and counts != before
    stats = provider.stats()
    assert stats["quarter_provider"]["phases"]["runtime"]["ad_evaluations"] == 1
    assert stats["quarter_provider"]["phases"]["runtime"]["cache_hits"] == 1
    assert stats["base_provider"]["phases"]["runtime"]["core_ad_evaluations"] == 1
    assert stats["numerical_finite_difference_calls"] == 0
    assert not stats["numerical_fallback_used"]
    values = provider.quarter_values(z); values["quarter_equality"][:] = np.nan
    assert np.isfinite(provider.quarter_values(z)["quarter_equality"]).all()
    view.clear_cache(); view.reset_stats()
    view.evaluate(z); view.evaluate(z.copy())
    assert view.stats()["quarter_primal_interpolation_calls"] == 1


@pytest.mark.parametrize("angle", [0., 1e-12, .999e-4, 1.001e-4, .999e-3, 1.001e-3])
def test_quarter_small_angle_and_world_wrap_primal_ad(angle):
    import jax
    import jax.numpy as jnp
    boundary = np.array([3., -2., np.pi-.0001])
    twist = np.array([0., 0., angle/.1])
    poses = compose_poses(boundary, se2_exp(np.linspace(0., 3., 31)[:, None]*twist))
    problem = GPProblem(boundary, twist, poses[1:], poses[-1], {}, None, None, [], False)
    z = problem.vector(poses, np.tile(twist, (31, 1)))
    core = module.make_quarter_core(problem)
    values = np.asarray(core(jnp.asarray(z))[0])
    npvalues = quarter_motion_values(problem, z)
    np.testing.assert_allclose(values, np.r_[npvalues["quarter_equality"], npvalues["quarter_inequality"]], atol=1e-9, rtol=1e-10)
    jac = np.asarray(jax.jacfwd(core, has_aux=True)(jnp.asarray(z))[0])
    assert np.isfinite(jac).all()


def test_unsupported_wrap_cut_nonfinite_and_wrong_base_fail_without_fallback(ready):
    view, z, provider, env = ready
    bad = z.copy(); bad[12] = np.pi
    with pytest.raises(DerivativeError, match="wrap cut"):
        provider.quarter_values(bad)
    with pytest.raises(DerivativeError, match="finite"):
        provider.equality_jacobian(np.full(150, np.nan))
    other, _, _ = fixture()
    with pytest.raises(ValueError, match="identical"):
        RefinedDerivativeProvider(ConstraintView(other), provider.base_provider)
    with pytest.raises(ValueError, match="unsupported fixed"):
        ConstraintView(view.base, "adaptive")


def test_constraint_view_does_not_change_dense_or_one_sided_acceleration(ready):
    view, z, _, _ = ready
    before = view.base.dense_report(z)
    view.evaluate(z)
    after = view.base.dense_report(z)
    for field in before:
        if isinstance(before[field], np.ndarray):
            np.testing.assert_array_equal(after[field], before[field])
        else:
            assert after[field] == before[field]
    p, v = view.base.unpack(z)
    _, _, both = interpolate_interval(p[:-1, None], v[:-1, None], p[1:, None], v[1:, None], .1, [0., 1.])
    assert both.shape == (30, 2, 3)
    assert np.any(np.abs(both[:-1, 1]-both[1:, 0]) > 1e-5)


def test_no_solver_or_numerical_fallback_is_defined_here():
    source = inspect.getsource(module)
    assert "scipy.optimize" not in source
    assert "minimize(" not in source and "run_instrumented(" not in source
    assert "approx_derivative(" not in source

"""Synthetic analytic derivative checks; these are not experimental evidence."""
import numpy as np
import pytest
from shapely.geometry import LineString, Polygon, box
from shapely.ops import unary_union

from reconciliation.gp_se2_diag02_environment import (
    EnvironmentDerivatives, UnsupportedEnvironmentDerivative,
)
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_formulation import _query


def environment(workspace=None, *, radius=.2):
    grid = dict(origin=np.array([-2., -3.]), resolution=np.array([.5, .25]),
                distances=np.array([[.3, .7, 1.1, 1.8], [.5, .9, 1.9, 2.1],
                                    [1.0, 1.2, 2.5, 2.7], [1.4, 1.8, 2.8, 3.1]]))
    env = HospitalEnvironment(LineString([(0., -10.), (0., 10.)]),
                              box(-4., -4., 4., 4.) if workspace is None else workspace,
                              grid=grid, metadata={"grid_distance_absolute_error_bound_m": .03125})
    return env, EnvironmentDerivatives(env, radius)


def central_gradient(query, point, step=1e-6):
    return np.array([(query(point + step * axis) - query(point - step * axis)) / (2 * step)
                     for axis in np.eye(2)])


@pytest.mark.parametrize("point", [[-1.83, -2.91], [-1.32, -2.61], [-.77, -2.42]])
def test_bilinear_world_chain_central_and_original_values(point):
    env, derivative = environment()
    point = np.asarray(point)
    value, gradient, metadata = derivative.obstacle(point)
    assert value == env.optimizer_clearance(point)
    np.testing.assert_allclose(gradient, central_gradient(env.optimizer_clearance, point), atol=2e-9)
    assert metadata["points"][0]["smooth"]
    assert metadata["points"][0]["derivative_semantics"] == "classical_gradient"


def test_bilinear_grid_boundary_positive_cell_one_sided_not_central():
    env, derivative = environment()
    point = np.array([-1.5, -2.875])
    value, gradient, metadata = derivative.obstacle(point)
    entry = metadata["points"][0]
    assert not entry["smooth"]
    assert entry["grid_boundary_axes"] == [0]
    assert entry["cell_xy"] == [1, 0]
    assert entry["derivative_semantics"] == "positive_cell_one_sided_gradient"
    step = 1e-7
    right = (env.optimizer_clearance(point + [step, 0]) - value) / step
    left = (value - env.optimizer_clearance(point - [step, 0])) / step
    assert gradient[0] == pytest.approx(right, abs=2e-8)
    assert abs(right - left) > .1
    assert gradient[1] == pytest.approx(central_gradient(env.optimizer_clearance, point)[1], abs=1e-8)


def test_grid_corner_and_minimum_domain_boundary_report_branch_only():
    env, derivative = environment()
    for point in (np.array([-1.5, -2.75]), np.array([-2., -3.])):
        value, gradient, metadata = derivative.obstacle(point)
        entry = metadata["points"][0]
        assert entry["grid_boundary_axes"] == [0, 1]
        assert not entry["smooth"]
        direction = np.array([.6, .8])
        assert (env.optimizer_clearance(point + 1e-7 * direction) - value) / 1e-7 == pytest.approx(
            gradient @ direction, abs=1e-6)
    assert entry["domain_boundary"]
    assert entry["derivative_semantics"] == "selected_branch_only_possible_discontinuity"


@pytest.mark.parametrize("point,boundary", [([-2.1, -2.8], False), ([-.5, -2.8], True),
                                          ([-1.4, -2.25], True), ([10., 10.], False)])
def test_outside_grid_original_query_penalty_zero_branch_derivative(point, boundary):
    env, derivative = environment()
    value, gradient, metadata = derivative.obstacle(point)
    original = _query(env.optimizer_clearance, np.array([[*point, 0.]]), "obstacle_clearance")
    assert value == original[0] == -1e6
    np.testing.assert_array_equal(gradient, [0., 0.])
    entry = metadata["points"][0]
    assert entry["invalid_query"]
    assert entry["domain_boundary"] == boundary
    assert entry["derivative_semantics"] == "selected_branch_only_possible_discontinuity"


def test_unknown_grid_cell_preserves_nonfinite_query_sanitization():
    env, _ = environment()
    env.grid["distances"][1, 1] = np.nan
    derivative = EnvironmentDerivatives(env)
    point = np.array([-1.8, -2.9])
    value, gradient, metadata = derivative.obstacle(point)
    assert np.isnan(env.optimizer_clearance(point))
    assert value == -1e6
    np.testing.assert_array_equal(gradient, [0., 0.])
    assert metadata["points"][0]["invalid_query"]


@pytest.mark.parametrize("point", [[-2.9, .3], [1.1, 2.8], [4.7, 1.], [4.7, 5.2], [4., .7]])
def test_workspace_smooth_signed_distance_interior_exterior_vertex_projection_boundary(point):
    env, derivative = environment()
    point = np.asarray(point)
    value, gradient, metadata = derivative.workspace(point)
    assert value == env.workspace_margin(point)
    np.testing.assert_allclose(gradient, central_gradient(env.workspace_margin, point), atol=1e-8)
    assert metadata["points"][0]["smooth"]
    assert np.linalg.norm(gradient) == pytest.approx(1.)


def test_workspace_nearest_feature_reselected_across_medial_axis():
    env, derivative = environment()
    _, gradients, metadata = derivative.workspace([[3., .1], [.1, 3.]])
    np.testing.assert_allclose(gradients, [[-1., 0.], [0., -1.]])
    assert metadata["points"][0]["selected_segment_id"] != metadata["points"][1]["selected_segment_id"]
    value, gradient, metadata = derivative.workspace([0., 0.])
    entry = metadata["points"][0]
    assert not entry["smooth"]
    assert len(entry["active_segment_ids"]) == 4
    assert entry["selected_segment_id"] == min(entry["active_segment_ids"])
    # Move toward the selected boundary: it becomes uniquely nearest.
    direction = -gradient
    assert (env.workspace_margin(1e-7 * direction) - value) / 1e-7 == pytest.approx(
        gradient @ direction, abs=1e-8)
    np.testing.assert_allclose(central_gradient(env.workspace_margin, np.zeros(2)), [0., 0.], atol=1e-9)


def test_concave_workspace_interior_vertex_projection_has_positive_signed_radial_gradient():
    floor = Polygon([(0., 0.), (3., 0.), (3., 1.), (1., 1.), (1., 3.), (0., 3.)])
    env, derivative = environment(floor)
    point = np.array([.8, .8])
    _, gradient, metadata = derivative.workspace(point)
    np.testing.assert_allclose(gradient, [-np.sqrt(.5), -np.sqrt(.5)], atol=1e-14)
    np.testing.assert_allclose(gradient, central_gradient(env.workspace_margin, point), atol=1e-8)
    assert metadata["points"][0]["smooth"]
    assert metadata["points"][0]["kind"] == "nearest_vertex"


def test_sloped_boundary_world_gradient_including_exact_boundary():
    env, derivative = environment(Polygon([(0., 0.), (4., 4.), (0., 4.)]))
    points = np.array([[1.1, 1.1], [1., 1.2], [1.2, 1.]])
    _, gradients, metadata = derivative.workspace(points)
    for point, gradient in zip(points, gradients):
        np.testing.assert_allclose(gradient, [-np.sqrt(.5), np.sqrt(.5)], atol=1e-14)
        np.testing.assert_allclose(gradient, central_gradient(env.workspace_margin, point), atol=1e-8)
    assert all(entry["smooth"] for entry in metadata["points"])


@pytest.mark.parametrize("point", [[4., 4.], [-4., 4.], [-4., -4.], [4., -4.]])
def test_workspace_boundary_vertex_reports_active_limiting_gradient(point):
    env, derivative = environment()
    point = np.asarray(point)
    value, gradient, metadata = derivative.workspace(point)
    entry = metadata["points"][0]
    assert entry["kind"] == "boundary_vertex"
    assert not entry["smooth"]
    assert entry["derivative_semantics"] == "active_limiting_gradient"
    # Leave the workspace along the selected edge's outward normal; the
    # vertex remains nearest and its directional derivative is -1.
    direction = -gradient
    assert (env.workspace_margin(point + 1e-7 * direction) - value) / 1e-7 == pytest.approx(-1., abs=1e-8)


@pytest.mark.parametrize("reverse", [False, True])
def test_holes_unknown_space_and_ring_orientation(reverse):
    outer = [(-4., -4.), (4., -4.), (4., 4.), (-4., 4.)]
    hole = [(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)]
    if reverse:
        outer, hole = outer[::-1], hole[::-1]
    env, derivative = environment(Polygon(outer, holes=[hole]))
    points = np.array([[1.4, .2], [.6, .2], [1., .2], [1., 1.]])
    values, gradients, metadata = derivative.workspace(points)
    np.testing.assert_array_equal(values, env.workspace_margin(points))
    np.testing.assert_allclose(gradients[:3], [[1., 0.]] * 3)
    assert values[1] < -.2  # Hole interior is explicitly unknown ground.
    assert not metadata["points"][1]["covered_by_workspace"]
    assert all(entry["hole"] for entry in metadata["points"])
    for point, gradient in zip(points[:3], gradients[:3]):
        np.testing.assert_allclose(gradient, central_gradient(env.workspace_margin, point), atol=1e-8)
    assert not metadata["points"][3]["smooth"]


def test_disconnected_workspace_outside_tie_is_dynamic_signed_distance():
    workspace = unary_union([box(-3., -1., -1., 1.), box(1., -1., 3., 1.)])
    env, derivative = environment(workspace)
    value, gradient, metadata = derivative.workspace([0., .2])
    assert value == env.workspace_margin([0., .2]) == -1.2
    assert not metadata["points"][0]["smooth"]
    assert not metadata["points"][0]["covered_by_workspace"]
    direction = gradient
    assert (env.workspace_margin(np.array([0., .2]) + 1e-7 * direction) - value) / 1e-7 == pytest.approx(
        gradient @ direction, abs=1e-8)


def test_shapes_empty_batches_and_footprint_applied_once():
    env, derivative = environment(radius=.31)
    points = np.array([[[-1.83, -2.91], [-1.32, -2.61]]])
    for family, original in (("workspace", env.workspace_margin), ("obstacle", env.optimizer_clearance)):
        values, gradients, metadata = derivative.evaluate(points, family)
        assert values.shape == (1, 2)
        assert gradients.shape == points.shape
        np.testing.assert_array_equal(values, original(points, .31))
        assert len(metadata["points"]) == 2
        values, gradients, metadata = derivative.evaluate(np.empty((0, 2)), family)
        assert values.shape == (0,) and gradients.shape == (0, 2)
        assert metadata["points"] == []


@pytest.mark.parametrize("point", [[np.nan, 1.], [np.inf, 1.], [1., -np.inf], [1., 2., 3.], 0.])
def test_nonfinite_inputs_and_wrong_shape_raise_without_fallback(point):
    _, derivative = environment()
    for family in ("workspace", "obstacle"):
        with pytest.raises(ValueError, match="finite world XY"):
            derivative.evaluate(point, family)


def test_unsupported_geometry_no_grid_and_unknown_family_fail_explicitly():
    env, _ = environment()
    env.grid = None
    derivative = EnvironmentDerivatives(env)
    with pytest.raises(UnsupportedEnvironmentDerivative, match="original bilinear grid"):
        derivative.obstacle([1., 1.])
    with pytest.raises(ValueError, match="family"):
        derivative.evaluate([1., 1.], "other")
    env.workspace = LineString([(0., 0.), (1., 1.)])
    with pytest.raises(UnsupportedEnvironmentDerivative, match="polygonal workspace"):
        EnvironmentDerivatives(env)


@pytest.mark.parametrize("radius", [-.1, np.nan, np.inf])
def test_invalid_radius_rejected(radius):
    with pytest.raises(ValueError, match="footprint radius"):
        environment(radius=radius)

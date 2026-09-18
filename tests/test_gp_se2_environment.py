"""Synthetic oracle-geometry tests, never performance evidence."""
import json

import numpy as np
import pytest
from shapely.geometry import LineString, Polygon, box
from shapely.ops import unary_union

from reconciliation.gp_se2_environment import (
    HospitalEnvironment, build_environment, clip_height, project_surface,
    transform_points, triangulate_face,
    filled_cross_section,
    _transform_validation_path,
)


def test_transform_nested_scale_and_stage_units():
    parent = np.array([[0., -2., 0., 100.], [2., 0., 0., 200.], [0., 0., 3., 0.], [0., 0., 0., 1.]])
    child = np.eye(4)
    child[:3, 3] = [1., 2., 3.]
    actual = transform_points([[4., 5., 6.]], parent @ child, .01)
    np.testing.assert_allclose(actual, [[.86, 2.10, .27]])


def test_nonconvex_mesh_face_preserves_door_not_aabb():
    points = np.array([[0., 0., 0.], [3., 0., 0.], [3., 3., 0.], [2., 3., 0.],
                       [2., 1., 0.], [1., 1., 0.], [1., 3., 0.], [0., 3., 0.]])
    triangles = triangulate_face(points)
    result = unary_union([Polygon(x[:, :2]) for x in triangles])
    assert result.area == pytest.approx(7.)
    assert not result.covers(box(1.2, 1.2, 1.8, 2.8))


@pytest.mark.parametrize("z, expected", [(-.1, False), (0., False), (.049, False), (.05, True), (.3, True), (.65, True), (.651, False), (2., False)])
def test_height_band_inclusion_excludes_floor_and_overhead(z, expected):
    triangle = np.array([[0., 0., z], [1., 0., z], [0., 1., z]])
    assert bool(len(clip_height(triangle, .05, .65))) is expected


def test_thin_vertical_surface_survives_projection():
    triangle = np.array([[0., -1., 0.], [0., 1., 0.], [0., 1., 1.]])
    projection = project_surface(clip_height(triangle, .05, .65))
    assert projection.geom_type == "LineString"
    assert projection.length > 1.8


@pytest.fixture
def environment():
    obstacles = unary_union([LineString([(0., -3.), (0., -.5)]),
                             LineString([(0., .5), (0., 3.)]), box(2., 1., 3., 2.)])
    return HospitalEnvironment(obstacles, box(-4., -4., 4., 4.))


def test_free_overlap_boundary_and_single_inflation(environment):
    assert environment.query([-1., 1.])["status"] == "CLEARANCE_VALID"
    assert environment.query([0., 1.])["status"] == "PHYSICAL_OVERLAP"
    assert environment.clearance([.25, 1.]) == pytest.approx(.05)
    assert environment.query([.25, 1.])["status"] == "BORDERLINE"
    assert environment.query([.225, 1.])["status"] == "CLEARANCE_VIOLATION"
    assert environment.query([.30, 1.])["status"] == "CLEARANCE_VALID"


def test_actual_doorway_and_swept_segment(environment):
    assert environment.check_polyline([[-1., 0.], [1., 0.]])["clearance_valid"]
    report = environment.check_polyline([[-1., 1.], [1., 1.]])
    assert report["physical_overlap"]
    assert report["minimum_clearance_m"] == pytest.approx(-.20)
    # Both endpoints alone pass, proving endpoint-only checking is insufficient.
    assert environment.query([-1., 1.])["status"] == "CLEARANCE_VALID"
    assert environment.query([1., 1.])["status"] == "CLEARANCE_VALID"


def test_unknown_workspace_includes_whole_footprint(environment):
    assert environment.query([4.2, 0.])["status"] == "UNKNOWN_WORKSPACE"
    assert environment.query([3.9, 0.])["status"] == "UNKNOWN_WORKSPACE"
    assert environment.workspace_status([3.7, 0.])
    assert environment.check_polyline([[3.7, 0.], [4.1, 0.]])["status"] == "UNKNOWN_WORKSPACE"


def test_unknown_ground_hole_is_not_free():
    floor = Polygon([(-4., -4.), (4., -4.), (4., 4.), (-4., 4.)], holes=[[(-.5, -.5), (.5, -.5), (.5, .5), (-.5, .5)]])
    env = HospitalEnvironment(LineString([(2., -4.), (2., 4.)]), floor)
    assert env.query([0., 0.])["status"] == "UNKNOWN_WORKSPACE"
    assert env.check_polyline([[-1., 0.], [1., 0.]])["status"] == "UNKNOWN_WORKSPACE"


def test_time_grid_is_explicit_and_rejects_duplicate_time(environment):
    report = environment.check_trajectory([0., .01, .02], [[-1., 0., 0.], [0., 0., 0.], [1., 0., 0.]])
    assert report["max_time_step_s"] == .01
    assert report["validation_level"] == "SAMPLED_CLEARANCE_VALIDATION"
    with pytest.raises(ValueError):
        environment.check_trajectory([0., 0.], [[-1., 0., 0.], [1., 0., 0.]])


def test_build_grid_bound_exact_checker_and_overwrite_refusal(tmp_path):
    (tmp_path / "geometry").mkdir()
    floor = np.array([[[-4., -4., 0.], [4., -4., 0.], [4., 4., 0.]], [[-4., -4., 0.], [4., 4., 0.], [-4., 4., 0.]]])
    wall = np.array([[[0., -3., 0.], [0., 3., 0.], [0., 3., 1.]], [[0., -3., 0.], [0., 3., 1.], [0., -3., 1.]]])
    np.savez_compressed(tmp_path / "geometry/world_triangles.npz", triangles=np.concatenate([floor, wall]), mesh_ids=[0, 0, 1, 1])
    (tmp_path / "geometry/meshes.json").write_text("[]")
    (tmp_path / "scene_provenance.json").write_text(json.dumps({"ground_z_m": 0., "height_band_ground_relative_m": [.05, .65], "composition_errors": [], "missing_geometry": [], "unsupported_geometry": []}))
    before = (tmp_path / "geometry/world_triangles.npz").read_bytes()
    env = build_environment(tmp_path, grid_resolution=.1)
    assert env.workspace.area == pytest.approx(64.)
    points = np.random.default_rng(11).uniform([-3.8, -3.8], [3.8, 3.8], (1000, 2))
    exact, approximate = env.obstacle_distance(points), env.optimizer_distance(points, conservative=False)
    bound = env.metadata["grid_distance_absolute_error_bound_m"]
    assert np.max(np.abs(exact - approximate)) <= bound
    assert np.all(env.optimizer_distance(points) <= exact + 1e-12)
    assert np.isnan(env.optimizer_distance([8., 8.]))
    assert (tmp_path / "geometry/world_triangles.npz").read_bytes() == before
    with pytest.raises(FileExistsError):
        build_environment(tmp_path)


def test_unresolved_geometry_is_not_treated_as_free(tmp_path):
    (tmp_path / "geometry").mkdir()
    (tmp_path / "scene_provenance.json").write_text(json.dumps({"composition_errors": ["missing.usd"]}))
    with pytest.raises(ValueError, match="Unresolved"):
        build_environment(tmp_path)


def _vertical_box_sides(lo, hi, bottom=0., top=2.):
    ring = np.array([[lo, lo], [hi, lo], [hi, hi], [lo, hi]])
    triangles = []
    for a, b in zip(ring, np.roll(ring, -1, axis=0)):
        triangles.extend([[[*a, bottom], [*b, bottom], [*b, top]], [[*a, bottom], [*b, top], [*a, top]]])
    return np.asarray(triangles)


def test_tall_closed_box_interior_is_occupied_below_top():
    from shapely.geometry import Point
    triangle = _vertical_box_sides(-2., 2.)
    surfaces = unary_union([project_surface(clip_height(t, .05, .65)) for t in triangle])
    assert surfaces.distance(Point(0., 0.)) == pytest.approx(2.)
    occupied, unknown, _ = filled_cross_section(triangle, .05)
    assert occupied.covers(Point(0., 0.))
    assert occupied.area == pytest.approx(16.)
    assert unknown.is_empty
    assert HospitalEnvironment(unary_union([surfaces, occupied]), box(-4., -4., 4., 4.)).query([0., 0.])["status"] == "PHYSICAL_OVERLAP"


def test_nested_closed_section_preserves_cavity_hole():
    from shapely.geometry import Point
    triangles = np.concatenate([_vertical_box_sides(-2., 2.), _vertical_box_sides(-1., 1.)])
    occupied, unknown, _ = filled_cross_section(triangles, .05)
    assert occupied.area == pytest.approx(12.)
    assert not occupied.covers(Point(0., 0.))
    assert occupied.covers(Point(1.5, 0.))
    assert unknown.is_empty


def test_open_bent_shell_interior_is_unknown_not_free():
    triangles = _vertical_box_sides(-2., 2.)[:-2]
    occupied, unknown, _ = filled_cross_section(triangles, .05)
    assert occupied.is_empty
    assert unknown.area > 0.


@pytest.mark.parametrize("name", ["transform_and_layer_validation.json", "actual_corner_passage_transform_and_layer_validation.json", "hospital_corridor_checkpoints_transform_and_layer_validation.json"])
def test_transform_validation_current_and_technical_names(tmp_path, name):
    evidence = tmp_path / "evidence"
    evidence.mkdir()
    (evidence / name).write_text("{}")
    assert _transform_validation_path(tmp_path) == evidence / name


def test_transform_validation_missing_is_explicit(tmp_path):
    with pytest.raises(FileNotFoundError, match="No independent"):
        _transform_validation_path(tmp_path)

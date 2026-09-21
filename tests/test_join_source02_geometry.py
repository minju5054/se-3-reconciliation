"""Synthetic geometry fixtures only; no LightNav obstacle-response evidence."""
import hashlib

import numpy as np
import pytest
from shapely.geometry import LineString, Point, box

from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.join_source02_geometry import (
    compose_environment, project_prop, projection_from_metadata,
)


def sides(lo=-.4, hi=.4, bottom=0., top=1.):
    ring = np.array([[lo, lo], [hi, lo], [hi, hi], [lo, hi]])
    triangles = []
    for a, b in zip(ring, np.roll(ring, -1, axis=0)):
        triangles.extend([[[*a, bottom], [*b, bottom], [*b, top]],
                          [[*a, bottom], [*b, top], [*a, top]]])
    return np.asarray(triangles, dtype=np.float64)


@pytest.fixture
def base():
    wall = LineString([(4., -5.), (4., 5.)])
    result = HospitalEnvironment(wall, box(-5., -5., 5., 5.),
        metadata=dict(numerical_tolerance_m=1e-7, height_band_world_m=[.05, .65]))
    result.gates = [dict(id="original_gate", start=[1., 1.], end=[1., 2.])]
    return result


def test_projection_preserves_exact_input_hash_units_and_single_offsets():
    triangles = sides()
    original = triangles.tobytes()
    p = project_prop(triangles)
    assert triangles.tobytes() == original
    assert p["metadata"]["triangles_float64_c_sha256"] == hashlib.sha256(original).hexdigest()
    assert p["metadata"]["height_band_world_m"] == [.05, .65]
    assert p["obstacle_geometry"].area == pytest.approx(.64)
    assert p["unknown_geometry"].is_empty
    assert p["metadata"]["footprint_radius_applied_in_projection"] is False
    assert p["metadata"]["clearance_applied_in_projection"] is False


def test_off_on_off_activation_does_not_modify_original(base):
    old_obstacles, old_workspace = base.obstacles.wkb, base.workspace.wkb
    p = project_prop(sides())
    off = compose_environment(base, p, False)
    on = compose_environment(base, p, True)
    sham = compose_environment(base, p, False)
    assert off.obstacles.wkb == sham.obstacles.wkb == old_obstacles
    assert off.workspace.wkb == sham.workspace.wkb == old_workspace
    assert base.obstacles.wkb == old_obstacles and base.workspace.wkb == old_workspace
    assert off.query([0., 0.])["status"] == "CLEARANCE_VALID"
    assert on.query([0., 0.])["status"] == "PHYSICAL_OVERLAP"
    assert on.clearance([.7, 0.]) == pytest.approx(.10)
    assert on.grid is None and off.grid is None
    assert on.gates == base.gates and on.gates is not base.gates
    assert on.metadata["runtime_obstacle_present"] is True
    assert off.metadata["runtime_obstacle_present"] is False


def test_entire_swept_segments_checked_no_row_deletion(base):
    on = compose_environment(base, project_prop(sides()), True)
    raw = np.array([[-1., 0.], [1., 0.]])
    before = raw.copy()
    assert all(on.query(x)["status"] == "CLEARANCE_VALID" for x in raw)
    assert not on.check_polyline(raw)["clearance_valid"]
    assert on.check_polyline(raw)["physical_overlap"]
    np.testing.assert_array_equal(raw, before)


def test_closed_nested_mesh_opening_preserved_not_bbox(base):
    p = project_prop(np.concatenate([sides(-2., 2.), sides(-1., 1.)]))
    assert not p["obstacle_geometry"].covers(Point(0., 0.))
    assert p["obstacle_geometry"].area == pytest.approx(12.)
    assert p["metadata"]["bbox_extra_area_m2"] == pytest.approx(4.)
    assert p["metadata"]["bbox_is_oracle"] is False
    assert compose_environment(base, p, True).query([0., 0.])["status"] == "CLEARANCE_VALID"


def test_nonclosing_bent_mesh_unknown_is_not_promoted_to_free(base):
    p = project_prop(sides(-2., 2.)[:-2])
    assert p["unknown_geometry"].area > 0.
    assert compose_environment(base, p, True).query([0., 0.])["status"] == "UNKNOWN_WORKSPACE"
    assert compose_environment(base, p, False).query([0., 0.])["status"] == "CLEARANCE_VALID"


def test_floor_and_overhead_excluded_thin_slab_walls_retained():
    plane = np.array([[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]]])
    assert project_prop(plane)["obstacle_geometry"].is_empty
    assert project_prop(plane + [0., 0., .7])["obstacle_geometry"].is_empty
    vertical = np.array([[[0., -1., 0.], [0., 1., 0.], [0., 1., 1.]]])
    p = project_prop(vertical)
    assert p["obstacle_geometry"].length > 1.8
    assert p["obstacle_geometry"].area == 0.


def test_saved_metadata_reconstruction_exact_no_new_runtime(base):
    p = project_prop(sides())
    saved = projection_from_metadata(p["metadata"])
    assert saved["obstacle_geometry"].wkb == p["obstacle_geometry"].wkb
    assert saved["unknown_geometry"].wkb == p["unknown_geometry"].wkb
    assert compose_environment(base, p, True).query([1., 0.]) == compose_environment(base, saved, True).query([1., 0.])


@pytest.mark.parametrize("bad", [np.zeros((0, 3, 3)), np.zeros((3, 3)), np.full((1, 3, 3), np.nan)])
def test_bad_mesh_rejected_not_unknown_as_free(bad):
    with pytest.raises(ValueError):
        project_prop(bad)


def test_semantic_changes_and_nonboolean_activation_rejected(base):
    with pytest.raises(ValueError, match="height band"):
        project_prop(sides(), height_band=(0., .7))
    p = project_prop(sides())
    with pytest.raises(TypeError, match="boolean"):
        compose_environment(base, p, "on")
    base.metadata["height_band_world_m"] = [0., .7]
    with pytest.raises(ValueError, match="height band"):
        compose_environment(base, p, True)

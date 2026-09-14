"""Synthetic geometry checks, never experimental evidence."""

import numpy as np
import pytest

from reconciliation.robotless_projection_handoff import characterize_projection, projection_geometry
from reconciliation.robotless_controlled_staleness import characterize


@pytest.mark.parametrize("boundary,path,j,alpha,q,location", [
    ([1, 1, 0], [[0, 0, 0], [2, 0, 0]], 0, .5, [1, 0], "interior"),
    ([-1, 1, 0], [[0, 0, 0], [2, 0, 0]], 0, 0, [0, 0], "start_endpoint"),
    ([3, 1, 0], [[0, 0, 0], [2, 0, 0]], 0, 1, [2, 0], "end_endpoint"),
    ([3, 1, 0], [[0, 0, 0], [2, 0, 0], [2, 2, 0]], 1, .5, [2, 1], "interior"),
    ([1, 1, 0], [[0, 0, 0], [2, 0, 0], [2, 2, 0]], 0, .5, [1, 0], "interior"),
    ([0, 0, 0], [[-1, -1, 0], [1, 1, 0], [-1, 1, 0], [1, -1, 0]], 0, .5, [0, 0], "interior"),
])
def test_projection_and_deterministic_ties(boundary, path, j, alpha, q, location):
    row = projection_geometry(boundary, path)
    assert row["segment_index"] == j
    assert row["alpha"] == pytest.approx(alpha)
    assert row["Q_xy_world_m"] == pytest.approx(q)
    assert row["e_perp_m"] == pytest.approx(np.linalg.norm(np.array(boundary[:2]) - q))
    assert row["projection_location"] == location
    assert row["projection_is_interior"] == (location == "interior")


def test_progress_across_prior_segments():
    row = projection_geometry([3, 1, 0], [[0, 0, 0], [2, 0, 0], [2, 2, 0]])
    assert row["s_Q_m"] == 3
    assert row["normalized_progress"] == .75
    assert row["total_FRESH_arc_length_m"] == 4


def test_degenerate_winner_is_point_with_unavailable_tangent():
    row = projection_geometry([0, 1, 0], [[0, 0, .3], [0, 0, 1.2], [1, 0, 2.4]])
    assert row["segment_index"] == 0 and row["alpha"] == 0
    assert row["e_perp_m"] == 1 and row["Q_xy_world_m"] == [0, 0]
    assert row["tangent_available"] is False
    for key in ("phi_F_rad", "e_dir_rad", "abs_e_dir_rad", "abs_e_dir_deg"):
        assert row[key] is None
    assert row["tangent_unavailable_reason"]
    assert row["theta_Q_rad"] == pytest.approx(.3)
    assert row["s_Q_m"] == row["normalized_progress"] == 0
    assert row["projection_location"] == "degenerate_point"


def test_zero_length_preceding_segment_adds_no_progress():
    row = projection_geometry([.5, 1, 0], [[0, 0, 0], [0, 0, 1], [1, 0, 2]])
    assert row["segment_index"] == 1 and row["s_Q_m"] == .5
    assert row["normalized_progress"] == .5


@pytest.mark.parametrize("path", [[[0, 0, 0]], [[0, 0, 0], [0, 0, 1]], [[1, 2, 0]] * 5])
def test_zero_total_length_explicitly_rejected(path):
    with pytest.raises(ValueError, match="arc length"):
        projection_geometry([0, 0, 0], path)


@pytest.mark.parametrize("dx,dy,expected", [(1, 0, 0), (0, 1, np.pi/2), (-1, 0, np.pi), (0, -1, -np.pi/2)])
def test_tangent_atan2(dx, dy, expected):
    row = projection_geometry([dx/2, dy/2, 0], [[0, 0, 0], [dx, dy, 0]])
    assert row["phi_F_rad"] == pytest.approx(expected)


def test_tangent_and_pose_yaw_are_distinct():
    row = projection_geometry([.5, .1, np.pi/2], [[0, 0, np.pi/4], [1, 0, np.pi/4]])
    assert row["phi_F_rad"] == 0
    assert row["theta_Q_rad"] == pytest.approx(np.pi/4)
    assert row["e_dir_rad"] == pytest.approx(-np.pi/2)
    assert row["abs_e_dir_deg"] == pytest.approx(90)
    assert row["e_yaw_rad"] == pytest.approx(-np.pi/4)
    assert row["abs_e_yaw_deg"] == pytest.approx(45)


@pytest.mark.parametrize("sign", [1, -1])
def test_shortest_yaw_interpolation_crosses_pi(sign):
    path = [[0, 0, sign*np.deg2rad(179)], [1, 0, -sign*np.deg2rad(179)]]
    row = projection_geometry([.5, 1, 0], path)
    assert row["theta_Q_rad"] == pytest.approx(-np.pi)
    assert row["e_yaw_rad"] == pytest.approx(-np.pi)


def test_direction_wrap_across_pi_and_positive_sign():
    angle = np.deg2rad(-179)
    row = projection_geometry([0, 0, np.deg2rad(179)], [[0, 0, angle], [np.cos(angle), np.sin(angle), angle]])
    assert row["e_dir_rad"] == pytest.approx(np.deg2rad(2))
    assert row["e_yaw_rad"] == pytest.approx(np.deg2rad(2))


@pytest.mark.parametrize("n", [2, 3, 10, 37, 101])
def test_arbitrary_n_and_source_preservation(n):
    fresh = np.column_stack((np.linspace(0, 1, n), np.zeros(n), np.linspace(0, .5, n)))
    before = fresh.copy()
    fresh.setflags(write=False)
    row = projection_geometry([.3, .2, 0], fresh)
    assert row["s_Q_m"] == pytest.approx(.3)
    assert row["normalized_progress"] == pytest.approx(.3)
    assert np.array_equal(fresh, before)


@pytest.mark.parametrize("b,f", [([0, np.nan, 0], [[0, 0, 0], [1, 0, 0]]),
    ([0, 0, np.inf], [[0, 0, 0], [1, 0, 0]]), ([0, 0], [[0, 0, 0], [1, 0, 0]]),
    ([0, 0, 0], [[0, 0, 0], [np.inf, 0, 0]]), ([0, 0, 0], [[0, 0, 0], [1, 0, np.nan]]), ([0, 0, 0], [])])
def test_invalid_inputs_rejected(b, f):
    with pytest.raises(ValueError):
        projection_geometry(b, f)


def test_previous_polyline_distance_consistency_and_saved_B():
    fresh = np.array([[.15, 0, 0], [.5, 0, .1], [1, 0, .2]])
    motion = {"tau_s": [0, .2, .5, 1], "v_mps": .25, "omega_radps": 0}
    boundaries, previous = characterize([0, 0, 0], fresh, **motion)
    rows = characterize_projection(boundaries, fresh, previous, motion, distance_atol_m=1e-12)
    assert all(r["e_perp_minus_previous_d_poly_m"] == 0 for r in rows)
    assert rows[-1]["projection_is_interior"] is True
    previous[-1]["d_poly_m"] = .1
    with pytest.raises(ValueError, match="previous d_poly"):
        characterize_projection(boundaries, fresh, previous, motion, distance_atol_m=1e-12)


@pytest.mark.parametrize("v,w", [(0, 0), (-1, 0), (.25, .1), (np.nan, 0), (.25, np.inf)])
def test_incoming_direction_requires_positive_forward_translation(v, w):
    with pytest.raises(ValueError):
        characterize_projection([[0, 0, 0]], [[0, 0, 0], [1, 0, 0]], [],
            {"v_mps": v, "omega_radps": w, "tau_s": [0]}, distance_atol_m=1e-12)

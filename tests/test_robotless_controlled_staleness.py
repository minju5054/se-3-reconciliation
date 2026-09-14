"""Deterministic synthetic geometry tests, never experimental evidence."""

import numpy as np
import pytest

from reconciliation.robotless_controlled_staleness import (
    characterize, controlled_boundaries, csv_row, motion_parameters, point_to_polyline,
)


@pytest.mark.parametrize("pose", [[0, 0, 0], [3, -2, np.pi / 2], [1, 2, .1], [1, 2, -2.8]])
def test_zero_delay_exact_and_yaw_unchanged(pose):
    boundaries = controlled_boundaries(pose, [0, .2, .5, 1], .25)
    assert np.array_equal(boundaries[0], pose)
    assert np.array_equal(boundaries[:, 2], np.repeat(pose[2], 4))
    assert np.linalg.norm(boundaries[:, :2] - boundaries[0, :2], axis=1) == pytest.approx([0, .05, .125, .25])


@pytest.mark.parametrize("yaw, expected", [(0, [1.25, 2]), (np.pi / 2, [1, 2.25]),
                                          (np.pi, [.75, 2]), (-np.pi / 2, [1, 1.75])])
def test_forward_motion_rotates_into_world(yaw, expected):
    assert controlled_boundaries([1, 2, yaw], [0, 1], .25)[1, :2] == pytest.approx(expected)


def test_zero_speed_and_immutable_inputs():
    pose, times = np.array([1., 2., .5]), np.array([0., 1.])
    before_pose, before_times = pose.copy(), times.copy()
    result = controlled_boundaries(pose, times, 0)
    assert np.array_equal(result, np.tile(pose, (2, 1)))
    assert np.array_equal(pose, before_pose) and np.array_equal(times, before_times)
    result[0, 0] += 1
    assert np.array_equal(pose, before_pose)


@pytest.mark.parametrize("times", [[], [1], [-1, 0], [0, -.2], [0, .5, .2], [0, 0], [0, np.nan], [0, np.inf], [[0]], [False, True], [0, True], ["0", "1"]])
def test_invalid_delays_rejected(times):
    with pytest.raises(ValueError):
        motion_parameters(times, .25, 0)


@pytest.mark.parametrize("velocity", [-.1, np.nan, np.inf, True, "0.25", None])
def test_invalid_velocity_rejected(velocity):
    with pytest.raises(ValueError):
        controlled_boundaries([0, 0, 0], [0, 1], velocity)


@pytest.mark.parametrize("omega", [.01, -.1, np.nan, np.inf, True, "0"])
def test_nonzero_or_invalid_omega_rejected(omega):
    with pytest.raises(ValueError):
        controlled_boundaries([0, 0, 0], [0, 1], .25, omega)


@pytest.mark.parametrize("pose", [[0, 0], [0, np.nan, 0], [0, 0, np.inf], [True, False, True]])
def test_invalid_pose_rejected(pose):
    with pytest.raises(ValueError):
        controlled_boundaries(pose, [0, 1], .25)


def test_overflow_displacement_rejected():
    with pytest.raises(ValueError, match="finite"):
        controlled_boundaries([0, 0, 0], [0, 1e308], 1e308)


@pytest.mark.parametrize("point,path,distance,index,fraction,closest", [
    ([1, 1], [[0, 0], [2, 0]], 1, 0, .5, [1, 0]),
    ([-1, 1], [[0, 0], [2, 0]], np.sqrt(2), 0, 0, [0, 0]),
    ([3, 0], [[0, 0], [2, 0]], 1, 0, 1, [2, 0]),
    ([3, 1], [[0, 0], [2, 0], [2, 2]], 1, 1, .5, [2, 1]),
    ([1, 0], [[0, 0], [0, 0], [2, 0]], 0, 1, .5, [1, 0]),
    ([1, 0], [[0, 0], [0, 0]], 1, 0, 0, [0, 0]),
    ([1, 1], [[0, 0]], np.sqrt(2), None, 0, [0, 0]),
    ([1, 1], [[0, 0], [2, 0], [2, 2]], 1, 0, .5, [1, 0]),
])
def test_segment_distance_cases(point, path, distance, index, fraction, closest):
    p, line = np.array(point), np.array(path)
    before = line.copy()
    result = point_to_polyline(p, line)
    assert result["distance_m"] == pytest.approx(distance)
    assert result["segment_index"] == index
    assert result["segment_fraction"] == pytest.approx(fraction)
    assert result["nearest_xy_world_m"] == pytest.approx(closest)
    assert np.array_equal(line, before)


@pytest.mark.parametrize("point,path", [([0], [[0, 0]]), ([0, 0], []), ([0, 0], [[0, 0, 0]]),
                                       ([np.inf, 0], [[0, 0]]), ([0, 0], [[np.nan, 0]]), (["a", "b"], [[0, 0]])])
def test_invalid_polyline_rejected(point, path):
    with pytest.raises(ValueError):
        point_to_polyline(point, path)


def test_entry_and_polyline_distances_diverge_after_passing_entry():
    # Synthetic straight path: boundary enters segment without touching first row.
    fresh = np.array([[.15, 0, 0], [.5, 0, 0], [1, 0, 0]])
    original = fresh.copy()
    fresh.setflags(write=False)
    _, rows = characterize([0, 0, 0], fresh, [0, .2, .5, 1], .25)
    assert [r["d_entry_m"] for r in rows] == pytest.approx([.15, .1, .025, .1])
    assert [r["d_poly_m"] for r in rows] == pytest.approx([.15, .1, .025, 0])
    assert rows[0]["delta_d_entry_m"] == rows[0]["delta_d_poly_m"] == 0.0
    assert rows[-1]["delta_d_entry_m"] == pytest.approx(-.05)
    assert rows[-1]["delta_d_poly_m"] == pytest.approx(-.15)
    assert np.array_equal(fresh, original)
    # Varying conditions never transforms/reanchors the frozen world input.
    for stop in [.2, .5, 1, 2]:
        characterize([0, 0, 0], fresh, [0, stop], .25)
        assert np.array_equal(fresh, original)


def test_polyline_excludes_observation_to_first_waypoint_connector():
    _, rows = characterize([0, 0, 0], [[1, 0, 0], [2, 0, 0]], [0, 1], .25)
    assert rows[0]["d_poly_m"] == 1  # Would be zero with an invented connector.


def test_log_residual_is_not_plain_relative_translation():
    # B=(0,0,0), F0=(1,0,pi/2); analytic V^-1 * [1,0] = [pi/4,-pi/4].
    _, rows = characterize([0, 0, 0], [[1, 0, np.pi / 2]], [0], .25)
    row = rows[0]
    assert row["entry_relative_pose_B_frame"] == pytest.approx([1, 0, np.pi / 2])
    assert row["r_entry_log_B_frame"] == pytest.approx([np.pi / 4, -np.pi / 4, np.pi / 2])
    assert row["r_entry_log_translation_norm_m"] == pytest.approx(np.pi / np.sqrt(8))
    assert row["d_entry_m"] == 1
    assert csv_row(row)["r_entry_log_x_m"] == pytest.approx(np.pi / 4)


def test_log_residual_in_rotated_boundary_frame():
    _, rows = characterize([1, 2, np.pi / 2], [[1, 3, np.pi / 2]], [0, 1], .25)
    assert rows[1]["r_entry_log_B_frame"] == pytest.approx([.75, 0, 0], abs=1e-12)
    assert rows[1]["delta_B_relative_pose"] == pytest.approx([.25, 0, 0], abs=1e-12)


@pytest.mark.parametrize("fresh", [[], [[1, 2]], [[1, np.nan, 0]], [[1, 2, np.inf]]])
def test_invalid_fresh_rejected(fresh):
    with pytest.raises(ValueError):
        characterize([0, 0, 0], fresh, [0, 1], .25)

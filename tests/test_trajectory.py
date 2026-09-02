import numpy as np
import pytest

from reconciliation.trajectory import (
    MotionSegment,
    generate_reference_trajectory,
    validate_se2_trajectory,
)


def segment(
    *,
    name: str = "motion",
    duration: float = 1.0,
    linear: float = 1.0,
    angular: float = 0.0,
) -> MotionSegment:
    return MotionSegment(name, duration, linear, angular)


def test_straight_trajectory_generation() -> None:
    trajectory = generate_reference_trajectory([0, 0, 0], [segment()], 0.25)
    assert trajectory.poses.shape == (5, 3)
    np.testing.assert_allclose(trajectory.poses[-1], [1.0, 0.0, 0.0])
    np.testing.assert_allclose(trajectory.times_s, [0, 0.25, 0.5, 0.75, 1.0])


def test_turning_trajectory_generation() -> None:
    trajectory = generate_reference_trajectory(
        [0, 0, 0],
        [segment(duration=1.0, linear=1.0, angular=np.pi / 2.0)],
        0.5,
    )
    np.testing.assert_allclose(trajectory.poses[1], [0.5, 0.0, np.pi / 4.0])
    np.testing.assert_allclose(
        trajectory.poses[-1],
        [0.5 + 0.5 / np.sqrt(2), 0.5 / np.sqrt(2), np.pi / 2],
    )


def test_arbitrary_n_by_three_shape() -> None:
    trajectory = generate_reference_trajectory(
        [0, 0, 0],
        [segment(duration=2.3)],
        0.1,
    )
    assert trajectory.poses.shape == (24, 3)


def test_yaw_wraps_during_generation() -> None:
    trajectory = generate_reference_trajectory(
        [0, 0, np.pi - 0.05],
        [segment(duration=0.2, linear=0.0, angular=1.0)],
        0.1,
    )
    assert -np.pi <= trajectory.poses[-1, 2] < np.pi
    assert trajectory.poses[-1, 2] == pytest.approx(-np.pi + 0.15)


def test_generation_is_deterministic() -> None:
    segments = [segment(duration=0.5), segment(name="turn", duration=0.5, angular=0.3)]
    first = generate_reference_trajectory([1, 2, 0.2], segments, 0.1)
    second = generate_reference_trajectory([1, 2, 0.2], segments, 0.1)
    np.testing.assert_array_equal(first.poses, second.poses)
    np.testing.assert_array_equal(first.linear_velocity_mps, second.linear_velocity_mps)


def test_zero_motion_keeps_pose() -> None:
    trajectory = generate_reference_trajectory(
        [1.0, -2.0, 0.4],
        [segment(duration=1.0, linear=0.0, angular=0.0)],
        0.2,
    )
    np.testing.assert_allclose(trajectory.poses, np.tile([1.0, -2.0, 0.4], (6, 1)))


@pytest.mark.parametrize("dt", [0.0, -0.1, np.nan])
def test_invalid_dt_rejected(dt: float) -> None:
    with pytest.raises(ValueError):
        generate_reference_trajectory([0, 0, 0], [segment()], dt)


def test_non_integral_duration_rejected_without_hidden_interpolation() -> None:
    with pytest.raises(ValueError, match="integer multiple"):
        generate_reference_trajectory([0, 0, 0], [segment(duration=1.05)], 0.1)


def test_nan_rejected() -> None:
    with pytest.raises(ValueError, match="finite"):
        validate_se2_trajectory([[0, 0, 0], [np.nan, 1, 0]])
    with pytest.raises(ValueError, match="finite"):
        MotionSegment("bad", 1.0, np.nan, 0.0)

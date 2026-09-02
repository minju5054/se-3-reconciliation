import numpy as np
import pytest

from reconciliation.controllers.trajectory_follower import (
    FollowerConfig,
    TrajectoryFollower,
    lookahead_index,
    nearest_progress_index,
)


def config() -> FollowerConfig:
    return FollowerConfig(
        lookahead_distance_m=0.25,
        position_gain=1.0,
        heading_gain=2.0,
        cross_track_gain=1.0,
        max_linear_velocity_mps=0.6,
        max_angular_velocity_rps=1.5,
        goal_position_tolerance_m=0.05,
        goal_yaw_tolerance_rad=0.05,
        rotate_in_place_threshold_rad=0.3,
        nearest_search_window=10,
    )


def test_nearest_progress_is_monotonic_and_not_time_driven() -> None:
    trajectory = np.column_stack((np.arange(8, dtype=float), np.zeros(8), np.zeros(8)))
    assert nearest_progress_index(trajectory, [3.1, 0.0, 0.0], 2, 4) == 3
    assert nearest_progress_index(trajectory, [0.0, 0.0, 0.0], 3, 4) == 3


def test_lookahead_uses_path_distance_for_arbitrary_horizon() -> None:
    cumulative = np.array([0.0, 0.0, 0.1, 0.4, 0.9, 1.4])
    assert lookahead_index(cumulative, 1, 0.25) == 3


def test_follower_rotates_when_target_is_behind_heading() -> None:
    trajectory = np.array([[0.0, 0.0, np.pi], [-1.0, 0.0, np.pi]])
    follower = TrajectoryFollower(trajectory, config())
    command = follower.forward([0.0, 0.0, 0.0])
    assert command.linear_velocity_mps == 0.0
    assert abs(command.angular_velocity_rps) > 0.0


def test_terminal_heading_wrap_across_pi() -> None:
    trajectory = np.array([[0.0, 0.0, np.pi - 0.02]])
    values = config()
    follower = TrajectoryFollower(
        trajectory,
        FollowerConfig(
            **{
                field: getattr(values, field)
                for field in values.__dataclass_fields__
                if field != "goal_yaw_tolerance_rad"
            },
            goal_yaw_tolerance_rad=0.01,
        ),
    )
    command = follower.forward([0.0, 0.0, -np.pi + 0.02])
    assert command.linear_velocity_mps == 0.0
    assert command.angular_velocity_rps == pytest.approx(-0.08)
    assert command.goal_reached is False


def test_follower_reaches_goal_without_fixed_horizon() -> None:
    trajectory = np.column_stack((np.linspace(0.0, 1.0, 37), np.zeros(37), np.zeros(37)))
    follower = TrajectoryFollower(trajectory, config())
    command = follower.forward([1.0, 0.0, 0.0])
    assert command.goal_reached is True
    assert command.target_index == 36


@pytest.mark.parametrize(
    "trajectory",
    [np.array([[0.0, np.nan, 0.0]]), np.array([[0.0, 0.0, np.inf]])],
)
def test_follower_rejects_nonfinite_trajectory(trajectory) -> None:
    with pytest.raises(ValueError):
        TrajectoryFollower(trajectory, config())

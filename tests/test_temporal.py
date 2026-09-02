import numpy as np

from reconciliation.temporal import (
    first_usable_waypoint_index,
    last_reached_waypoint_index,
    stale_prefix_length,
    waypoint_execution_times,
)
from reconciliation.types import TrajectoryChunk


def chunk_with_latency(latency: float, dt: float = 0.5, horizon: int = 5) -> TrajectoryChunk:
    return TrajectoryChunk(
        poses_local=np.zeros((horizon, 3)),
        observation_time=10.0,
        ready_time=10.0 + latency,
        inference_latency=latency,
        waypoint_dt=dt,
        robot_pose_at_observation=[0.0, 0.0, 0.0],
        frame="robot_local_at_observation",
        source="synthetic-test-fixture",
    )


def test_waypoint_times_start_after_one_interval() -> None:
    np.testing.assert_allclose(waypoint_execution_times(chunk_with_latency(0.0)), [10.5, 11, 11.5, 12, 12.5])


def test_zero_latency_keeps_first_waypoint() -> None:
    assert first_usable_waypoint_index(chunk_with_latency(0.0)) == 0


def test_latency_smaller_than_one_interval_keeps_first_waypoint() -> None:
    assert first_usable_waypoint_index(chunk_with_latency(0.2)) == 0


def test_latency_spanning_multiple_intervals_discards_strictly_stale_prefix() -> None:
    chunk = chunk_with_latency(1.1)
    assert first_usable_waypoint_index(chunk) == 2
    assert stale_prefix_length(chunk) == 2


def test_waypoint_exactly_at_ready_time_is_usable() -> None:
    assert first_usable_waypoint_index(chunk_with_latency(1.0)) == 1


def test_all_waypoints_can_be_stale() -> None:
    chunk = chunk_with_latency(3.0)
    assert first_usable_waypoint_index(chunk) == chunk.horizon


def test_last_reached_waypoint_is_discrete_and_inclusive() -> None:
    chunk = chunk_with_latency(0.0)
    assert last_reached_waypoint_index(chunk, 10.49) == -1
    assert last_reached_waypoint_index(chunk, 10.5) == 0
    assert last_reached_waypoint_index(chunk, 99.0) == chunk.horizon - 1

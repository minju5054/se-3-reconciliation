import numpy as np
import pytest

from reconciliation.metrics import analyze_raw_switch, compute_transition_metrics
from reconciliation.types import TrajectoryChunk


def test_boundary_metric_sanity() -> None:
    metrics = compute_transition_metrics(
        old_previous_pose=[0.0, 0.0, 0.1],
        old_boundary_pose=[1.0, 0.0, 0.2],
        new_first_pose=[1.3, 0.4, 0.5],
        new_second_pose=[1.3, 2.4, 0.9],
    )
    assert metrics.translation_gap_m == pytest.approx(0.5)
    assert metrics.yaw_gap_rad == pytest.approx(0.3)
    assert metrics.previous_old_segment_translation_m == pytest.approx(1.0)
    assert metrics.first_new_segment_translation_m == pytest.approx(2.0)
    assert metrics.translational_motion_jump_m == pytest.approx(1.0)
    assert metrics.previous_old_yaw_increment_rad == pytest.approx(0.1)
    assert metrics.first_new_yaw_increment_rad == pytest.approx(0.4)
    assert metrics.yaw_motion_jump_rad == pytest.approx(0.3)


def test_yaw_motion_jump_wraps_across_pi() -> None:
    metrics = compute_transition_metrics(
        [0, 0, 0],
        [1, 0, np.pi - 0.05],
        [1, 0, 0],
        [2, 0, -np.pi + 0.05],
    )
    expected = abs(float(((-np.pi + 0.05) - (np.pi - 0.05) + np.pi) % (2 * np.pi) - np.pi))
    assert metrics.yaw_motion_jump_rad == pytest.approx(expected)


def test_raw_switch_uses_old_progress_and_new_observation_pose() -> None:
    old = TrajectoryChunk(
        poses_local=np.array([[1, 0, 0], [2, 0, 0], [3, 0, 0]], dtype=float),
        observation_time=0.0,
        ready_time=0.0,
        inference_latency=0.0,
        waypoint_dt=1.0,
        robot_pose_at_observation=[0, 0, 0],
        frame="robot_local_at_observation",
        source="synthetic-test-fixture-old",
    )
    new = TrajectoryChunk(
        poses_local=np.array([[1, 0, 0], [2.5, 0, 0.2], [4.5, 0, 0.5]], dtype=float),
        observation_time=1.0,
        ready_time=2.2,
        inference_latency=1.2,
        waypoint_dt=1.0,
        robot_pose_at_observation=[1, 0, 0],
        frame="robot_local_at_observation",
        source="synthetic-test-fixture-new",
    )
    analysis = analyze_raw_switch(old, new)
    assert analysis.old_boundary_index == 1
    assert analysis.new_first_usable_index == 1
    np.testing.assert_allclose(analysis.old_boundary_pose_world, [2, 0, 0])
    np.testing.assert_allclose(analysis.new_first_pose_world, [3.5, 0, 0.2])
    assert analysis.metrics.translation_gap_m == pytest.approx(1.5)
    assert analysis.metrics.translational_motion_jump_m == pytest.approx(1.0)
    assert analysis.metrics.yaw_motion_jump_rad == pytest.approx(0.3)


def test_raw_switch_requires_two_usable_new_waypoints() -> None:
    common = dict(
        observation_time=0,
        ready_time=0,
        inference_latency=0,
        waypoint_dt=1,
        robot_pose_at_observation=[0, 0, 0],
        frame="robot_local_at_observation",
        source="synthetic-test-fixture",
    )
    old = TrajectoryChunk(poses_local=[[1, 0, 0], [2, 0, 0]], **common)
    new = TrajectoryChunk(poses_local=[[1, 0, 0]], **common)
    with pytest.raises(ValueError, match="at least two"):
        analyze_raw_switch(old, new)


def test_raw_switch_rejects_silent_frame_mismatch() -> None:
    common = dict(
        poses_local=[[1, 0, 0], [2, 0, 0]],
        observation_time=0,
        ready_time=0,
        inference_latency=0,
        waypoint_dt=1,
        robot_pose_at_observation=[0, 0, 0],
        source="synthetic-test-fixture",
    )
    old = TrajectoryChunk(frame="robot_local_a", **common)
    new = TrajectoryChunk(frame="robot_local_b", **common)
    with pytest.raises(ValueError, match="explicit conversion"):
        analyze_raw_switch(old, new)

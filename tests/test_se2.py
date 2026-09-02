import numpy as np

from reconciliation.se2 import (
    compose_poses,
    inverse_pose,
    local_trajectory_to_world,
    relative_pose,
    wrap_angle,
)


def test_se2_identity() -> None:
    pose = np.array([2.0, -1.0, 0.7])
    identity = np.zeros(3)
    np.testing.assert_allclose(compose_poses(identity, pose), pose)
    np.testing.assert_allclose(compose_poses(pose, identity), pose)


def test_compose_inverse_consistency() -> None:
    pose = np.array([1.2, -3.4, 2.2])
    np.testing.assert_allclose(compose_poses(pose, inverse_pose(pose)), np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(relative_pose(pose, pose), np.zeros(3), atol=1e-12)


def test_yaw_wrap_across_plus_minus_pi() -> None:
    assert np.isclose(wrap_angle(np.pi + 0.1), -np.pi + 0.1)
    assert np.isclose(wrap_angle(-np.pi - 0.1), np.pi - 0.1)
    assert np.isclose(wrap_angle(np.pi), -np.pi)


def test_local_trajectory_to_world_uses_observation_pose() -> None:
    robot_world = np.array([10.0, 2.0, np.pi / 2.0])
    local = np.array([[1.0, 0.0, 0.0], [1.0, 2.0, np.pi / 2.0]])
    world = local_trajectory_to_world(robot_world, local)
    expected = np.array([[10.0, 3.0, np.pi / 2.0], [8.0, 3.0, -np.pi]])
    np.testing.assert_allclose(world, expected, atol=1e-12)


def test_compose_broadcasts_over_arbitrary_horizon() -> None:
    local = np.zeros((24, 3))
    world = local_trajectory_to_world(np.array([1.0, 2.0, 0.3]), local)
    assert world.shape == (24, 3)

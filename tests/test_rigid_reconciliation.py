import numpy as np
import pytest

from reconciliation.rigid_reconciliation import analytic_rigid_reconciliation
from reconciliation.se2 import compose_poses, relative_pose, wrap_angle


def fixture_suffix(n=7):
    poses = [np.array([1.0, 0.5, 0.2])]
    for _ in range(n - 1):
        poses.append(compose_poses(poses[-1], [0.2, 0.0, 0.05]))
    return np.asarray(poses)


def test_rigid_target_translation_and_yaw() -> None:
    previous = np.array([-0.2, 0.0, -0.1])
    boundary = np.array([0.0, 0.0, 0.0])
    suffix = fixture_suffix()
    result = analytic_rigid_reconciliation(
        previous, boundary, suffix, minimum_translation_m=1e-6
    )
    radius = np.linalg.norm(suffix[0, :2] - boundary[:2])
    assert result.target_entry_world[:2] == pytest.approx([radius, 0.0])
    assert result.target_entry_world[2] == pytest.approx(0.1)
    assert result.trajectory_world[0] == pytest.approx(result.target_entry_world)


def test_complete_suffix_and_relative_motions_are_preserved() -> None:
    suffix = fixture_suffix(n=11)
    original = suffix.copy()
    result = analytic_rigid_reconciliation(
        [-0.3, 0.0, 0.0], [0.0, 0.0, 0.05], suffix, minimum_translation_m=1e-6
    )
    assert result.trajectory_world.shape == suffix.shape
    for index in range(len(suffix) - 1):
        assert relative_pose(
            result.trajectory_world[index], result.trajectory_world[index + 1]
        ) == pytest.approx(relative_pose(suffix[index], suffix[index + 1]), abs=1e-12)
    assert np.array_equal(suffix, original)


def test_arbitrary_horizon_and_wrapped_yaw_target() -> None:
    for count in (2, 4, 24):
        suffix = fixture_suffix(count)
        result = analytic_rigid_reconciliation(
            [-0.1, 0.0, np.pi - 0.02],
            [0.0, 0.0, -np.pi + 0.03],
            suffix,
            minimum_translation_m=1e-6,
        )
        incoming_yaw = relative_pose(
            [-0.1, 0.0, np.pi - 0.02], [0.0, 0.0, -np.pi + 0.03]
        )[2]
        assert relative_pose([0.0, 0.0, -np.pi + 0.03], result.target_entry_world)[2] == pytest.approx(
            incoming_yaw
        )
        assert -np.pi <= result.target_entry_world[2] < np.pi


def test_undefined_incoming_or_entry_direction_rejected() -> None:
    suffix = fixture_suffix()
    with pytest.raises(ValueError, match="incoming translation"):
        analytic_rigid_reconciliation([0, 0, 0], [0, 0, 0], suffix, minimum_translation_m=1e-6)
    suffix[0, :2] = 0.0
    with pytest.raises(ValueError, match="boundary-to-entry"):
        analytic_rigid_reconciliation([-0.1, 0, 0], [0, 0, 0], suffix, minimum_translation_m=1e-6)

import numpy as np
import pytest

from reconciliation.se2 import (
    compose_poses,
    inverse_pose,
    retract_pose,
    se2_exp,
    se2_log,
    wrap_angle,
)


@pytest.mark.parametrize(
    "tangent",
    (
        [0.0, 0.0, 0.0],
        [0.4, -0.2, 0.0],
        [0.0, 0.0, 0.7],
        [0.2, 0.1, 1e-10],
        [0.3, -0.4, np.pi - 1e-6],
        [-0.1, 0.5, -np.pi + 1e-6],
    ),
)
def test_exp_log_round_trip(tangent) -> None:
    recovered = se2_log(se2_exp(tangent))
    assert recovered[:2] == pytest.approx(tangent[:2], abs=1e-9)
    assert recovered[2] == pytest.approx(wrap_angle(tangent[2]), abs=1e-9)


@pytest.mark.parametrize(
    "pose",
    ([0.0, 0.0, 0.0], [1.2, -0.8, 0.0], [0.0, 0.0, 1.1], [0.7, 0.4, -2.8]),
)
def test_log_exp_round_trip(pose) -> None:
    assert se2_exp(se2_log(pose)) == pytest.approx(pose, abs=1e-10)


def test_retract_is_right_local_compose() -> None:
    pose = np.array([1.0, 2.0, np.pi / 2])
    delta = np.array([0.2, 0.0, 0.1])
    assert retract_pose(pose, delta) == pytest.approx(compose_poses(pose, se2_exp(delta)))


def test_lie_operations_preserve_compose_inverse_consistency() -> None:
    pose = se2_exp([0.4, -0.1, 0.2])
    assert compose_poses(pose, inverse_pose(pose)) == pytest.approx([0.0, 0.0, 0.0], abs=1e-12)


def test_lie_operations_reject_nonfinite() -> None:
    with pytest.raises(ValueError, match="finite"):
        se2_exp([np.nan, 0.0, 0.0])
    with pytest.raises(ValueError, match="finite"):
        se2_log([0.0, np.inf, 0.0])

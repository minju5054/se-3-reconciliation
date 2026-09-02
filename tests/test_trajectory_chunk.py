import numpy as np
import pytest

from reconciliation.types import TrajectoryChunk


def make_chunk(**overrides) -> TrajectoryChunk:
    values = {
        "poses_local": np.zeros((4, 3)),
        "observation_time": 10.0,
        "ready_time": 10.5,
        "inference_latency": 0.5,
        "waypoint_dt": 0.25,
        "robot_pose_at_observation": [1.0, 2.0, 0.1],
        "frame": "robot_local_at_observation",
        "source": "synthetic-test-fixture",
    }
    values.update(overrides)
    return TrajectoryChunk(**values)


def test_accepts_arbitrary_n_by_three_horizon() -> None:
    assert make_chunk(poses_local=np.zeros((24, 3))).horizon == 24


@pytest.mark.parametrize("shape", [(3,), (0, 3), (4, 2), (2, 3, 1)])
def test_rejects_invalid_pose_dimensions(shape) -> None:
    with pytest.raises(ValueError, match="poses_local"):
        make_chunk(poses_local=np.zeros(shape))


def test_rejects_ready_time_before_observation() -> None:
    with pytest.raises(ValueError, match="ready_time"):
        make_chunk(ready_time=9.9, inference_latency=-0.1)


def test_rejects_latency_inconsistent_with_timestamps() -> None:
    with pytest.raises(ValueError, match="must equal"):
        make_chunk(inference_latency=0.4)


def test_rejects_nonpositive_waypoint_dt() -> None:
    with pytest.raises(ValueError, match="greater than zero"):
        make_chunk(waypoint_dt=0.0)


def test_arrays_are_copied_and_read_only() -> None:
    original = np.zeros((2, 3))
    chunk = make_chunk(poses_local=original)
    original[0, 0] = 99.0
    assert chunk.poses_local[0, 0] == 0.0
    with pytest.raises(ValueError):
        chunk.poses_local[0, 0] = 1.0


def test_json_round_trip() -> None:
    chunk = make_chunk()
    restored = TrajectoryChunk.from_dict(chunk.to_dict())
    np.testing.assert_array_equal(restored.poses_local, chunk.poses_local)
    assert restored.to_dict() == chunk.to_dict()

import json

import numpy as np

from reconciliation.io import build_chunk, load_chunk, load_poses, save_chunk


def metadata() -> dict:
    return {
        "observation_time": 1.0,
        "ready_time": 1.2,
        "inference_latency": 0.2,
        "waypoint_dt": 0.1,
        "robot_pose_at_observation": [0, 0, 0],
        "frame": "robot_local_at_observation",
        "source": "synthetic-test-fixture",
    }


def test_load_npy_and_build_chunk_without_modifying_raw(tmp_path) -> None:
    raw = tmp_path / "raw.npy"
    poses = np.arange(12, dtype=float).reshape(4, 3)
    np.save(raw, poses)
    before = raw.read_bytes()
    chunk = build_chunk(raw, metadata())
    assert raw.read_bytes() == before
    np.testing.assert_array_equal(chunk.poses_local, poses)


def test_load_json_pose_list(tmp_path) -> None:
    path = tmp_path / "poses.json"
    path.write_text(json.dumps([[0, 0, 0], [1, 0, 0]]), encoding="utf-8")
    assert load_poses(path).shape == (2, 3)


def test_save_and_load_chunk(tmp_path) -> None:
    raw = tmp_path / "raw.npy"
    np.save(raw, np.zeros((3, 3)))
    destination = tmp_path / "derived" / "chunk.json"
    save_chunk(build_chunk(raw, metadata()), destination)
    loaded = load_chunk(destination)
    assert loaded.horizon == 3
    assert loaded.source == "synthetic-test-fixture"

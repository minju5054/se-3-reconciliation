"""Pure contracts; synthetic fixtures do not constitute Isaac runtime evidence."""

from copy import deepcopy
import json

import numpy as np
import pytest

from reconciliation.robotless_single_chunk import (
    FRAME_CONVENTIONS,
    frame_sanity_fixtures,
    load_config,
    make_observation_time,
    observation_to_world,
    save_json_exclusive,
    save_npy_exclusive,
    sha256_file,
    validate_observation_metadata,
    validate_observation_time,
    validate_scene_frame,
    validate_waypoints,
)


@pytest.mark.parametrize(
    "pose, local, expected",
    [
        ([0, 0, 0], [[1, 0, 0]], [[1, 0, 0]]),
        ([2, 3, 0], [[1, 0, 0]], [[3, 3, 0]]),
        ([0, 0, np.pi / 2], [[1, 0, 0]], [[0, 1, np.pi / 2]]),
        ([0, 0, np.pi / 2], [[0, 1, 0]], [[-1, 0, np.pi / 2]]),
        ([0, 0, np.pi / 2], [[0, 0, np.pi / 2]], [[0, 0, -np.pi]]),
        ([0, 0, -np.pi / 2], [[0, 0, -np.pi]], [[0, 0, np.pi / 2]]),
        ([0, 0, 0], [[0, 0, 5 * np.pi]], [[0, 0, -np.pi]]),
    ],
)
def test_observation_transform(pose, local, expected):
    np.testing.assert_allclose(observation_to_world(pose, local), expected, atol=1e-12)


@pytest.mark.parametrize("count", [1, 2, 7, 10, 24, 137])
def test_arbitrary_length_preserves_all_waypoints(count):
    raw = np.column_stack((np.arange(count), np.zeros(count), np.zeros(count)))
    result = observation_to_world([2, 3, np.pi / 2], raw)
    assert result.shape == (count, 3)
    np.testing.assert_allclose(result[:, 0], 2, atol=1e-12)
    np.testing.assert_allclose(result[:, 1], 3 + np.arange(count), atol=1e-12)
    np.testing.assert_allclose(result[:, 2], np.pi / 2, atol=1e-12)


def test_transform_keeps_raw_bytes_and_pose_immutable():
    raw = np.array([[1, 2, 9], [-3, 0, -7]], dtype=np.float32)
    pose = np.array([2, 3, np.pi / 2])
    before = raw.tobytes()
    pose_before = pose.copy()
    raw.flags.writeable = False
    result = observation_to_world(pose, raw)
    result[:] = 999
    assert raw.tobytes() == before
    np.testing.assert_array_equal(pose, pose_before)
    assert not np.shares_memory(result, raw)


def test_validation_returns_an_independent_float64_copy():
    raw = np.array([[1, 2, 3]], dtype=np.float64)
    validated = validate_waypoints(raw)
    assert validated.dtype == np.float64
    assert not np.shares_memory(validated, raw)
    validated[:] = 0
    np.testing.assert_array_equal(raw, [[1, 2, 3]])


@pytest.mark.parametrize(
    "raw",
    [
        [], [1, 2, 3], [[1, 2]], [[1, 2, 3, 4]], np.empty((0, 3)),
        np.zeros((1, 2, 3)), [[np.nan, 0, 0]], [[0, np.inf, 0]],
        [[0, 0, -np.inf]], [["1", "2", "3"]], [[True, False, True]],
        [[1j, 0, 0]], np.array([[1, 2, 3]], dtype=object),
    ],
)
def test_rejects_invalid_trajectory(raw):
    with pytest.raises(ValueError):
        validate_waypoints(raw)


@pytest.mark.parametrize("pose", [[], [1, 2], [[1, 2, 3]], [1, np.nan, 0], ["1", "2", "3"]])
def test_rejects_invalid_observation_pose(pose):
    with pytest.raises(ValueError):
        observation_to_world(pose, [[1, 0, 0]])


def test_frame_sanity_fixtures_are_independent_and_explicitly_synthetic():
    fixtures = frame_sanity_fixtures()
    assert [item["name"] for item in fixtures] == ["yaw_0", "yaw_90"]
    for fixture in fixtures:
        assert fixture["passed"] is True
        assert fixture["synthetic_only"] is True
        assert fixture["point_labels"] == ["forward", "left"]
        np.testing.assert_allclose(
            fixture["actual_world_waypoints"], fixture["expected_world_waypoints"], atol=1e-12
        )
    fixtures[0]["local_waypoints"][0][0] = 100
    assert fixtures[1]["local_waypoints"][0][0] == 1
    assert frame_sanity_fixtures()[0]["local_waypoints"][0][0] == 1


def test_exclusive_artifact_writes_preserve_existing_raw_bytes(tmp_path):
    response = tmp_path / "raw/response.json"
    trajectory = tmp_path / "raw/waypoints.npy"
    raw = np.array([[1, 2, 3]], dtype=np.float32)
    save_json_exclusive(response, {"waypoints": raw.tolist()})
    save_npy_exclusive(trajectory, raw)
    hashes = [sha256_file(response), sha256_file(trajectory)]
    with pytest.raises(FileExistsError):
        save_json_exclusive(response, {"waypoints": [[9, 9, 9]]})
    with pytest.raises(FileExistsError):
        save_npy_exclusive(trajectory, [[9, 9, 9]])
    assert hashes == [sha256_file(response), sha256_file(trajectory)]
    assert json.loads(response.read_text())["waypoints"] == raw.tolist()
    restored = np.load(trajectory, allow_pickle=False)
    assert restored.dtype == raw.dtype
    assert restored.tobytes() == raw.tobytes()


def test_invalid_serialization_does_not_create_artifact(tmp_path):
    with pytest.raises(ValueError):
        save_json_exclusive(tmp_path / "bad.json", {"value": float("nan")})
    with pytest.raises(ValueError):
        save_npy_exclusive(tmp_path / "bad.npy", np.array([object()], dtype=object))
    assert list(tmp_path.iterdir()) == []


def test_sha256_matches_known_bytes(tmp_path):
    path = tmp_path / "input"
    path.write_bytes(b"abc")
    assert sha256_file(path) == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"


def test_load_config_requires_mapping(tmp_path):
    path = tmp_path / "config.yaml"
    path.write_text("agent:\n  pose_world: [2, 3, 0]\n")
    assert load_config(path) == {"agent": {"pose_world": [2, 3, 0]}}
    path.write_text("[]\n")
    with pytest.raises(ValueError, match="mapping"):
        load_config(path)


def _metadata():
    return {
        "observation_time": {
            "utc": "2026-09-14T09:00:00Z", "monotonic_ns": 123, "simulation_time_s": 0.0,
        },
        "agent_pose_world": [2, 3, 0],
        "camera": {"extrinsic": {"frame": "logical_agent"}, "intrinsics": {"fov_deg": 112}},
        "instruction": "move forward",
        "scene": {"asset_path": "configured/scene.usd"},
        "coordinate_conventions": deepcopy(FRAME_CONVENTIONS),
        "research_git_sha": "a" * 40,
        "lightnav_git_sha": "b" * 40,
        "model_checkpoint_identifier": "external/LightNav-0",
        "rgb": {"path": "raw/observation.jpg", "sha256": "c" * 64, "resolution_width_height": [480, 270]},
        "execution_time": None,
    }


def test_metadata_accepts_static_capture_without_execution():
    metadata = _metadata()
    before = deepcopy(metadata)
    validate_observation_metadata(metadata)
    assert metadata == before


def test_observation_clock_domains_are_named_and_not_assigned_to_waypoints(monkeypatch):
    monkeypatch.setattr("reconciliation.robotless_single_chunk.time.monotonic_ns", lambda: 12345)
    stamp = make_observation_time(3.25)
    validate_observation_time(stamp)
    assert stamp["monotonic_ns"] == 12345
    assert stamp["simulation_time_s"] == 3.25
    assert stamp["utc"].endswith("Z")
    assert set(stamp) == {"utc", "monotonic_ns", "simulation_time_s"}


@pytest.mark.parametrize(
    "key, value",
    [
        ("utc", "2026-09-14T09:00:00"), ("utc", "2026-09-14T09:00:00+09:00"),
        ("utc", "2026-09-14"), ("utc", "not-a-date"), ("monotonic_ns", -1),
        ("monotonic_ns", 1.2), ("monotonic_ns", True), ("simulation_time_s", -1),
        ("simulation_time_s", np.inf), ("simulation_time_s", None), ("simulation_time_s", True),
    ],
)
def test_invalid_clock_conventions_are_rejected(key, value):
    metadata = _metadata()
    metadata["observation_time"][key] = value
    with pytest.raises(ValueError):
        validate_observation_metadata(metadata)


@pytest.mark.parametrize("value", [-1, np.inf, np.nan, True, "0"])
def test_invalid_simulation_clock_cannot_be_stamped(value):
    with pytest.raises(ValueError):
        make_observation_time(value)


@pytest.mark.parametrize(
    "key, value",
    [
        ("observation_time", None), ("agent_pose_world", [0, 0, np.inf]),
        ("camera", {}), ("scene", "scene"), ("coordinate_conventions", {}),
        ("instruction", ""), ("research_git_sha", "short"), ("lightnav_git_sha", "z" * 40),
        ("model_checkpoint_identifier", ""), ("rgb", {}), ("execution_time", 0),
    ],
)
def test_metadata_requires_provenance_and_static_scope(key, value):
    metadata = _metadata()
    metadata[key] = value
    with pytest.raises(ValueError):
        validate_observation_metadata(metadata)


@pytest.mark.parametrize("resolution", [[480], [480, 270, 3], [0, 270], [480.0, 270], [True, 270]])
def test_metadata_requires_positive_integer_width_and_height(resolution):
    metadata = _metadata()
    metadata["rgb"]["resolution_width_height"] = resolution
    with pytest.raises(ValueError, match="resolution"):
        validate_observation_metadata(metadata)


def test_optional_readiness_is_a_host_event_and_has_no_pose_or_latency():
    metadata = _metadata()
    metadata["inference_ready_time"] = {"utc": "2026-09-14T09:00:01+00:00", "monotonic_ns": 124}
    validate_observation_metadata(metadata)
    metadata["inference_ready_time"]["monotonic_ns"] = "124"
    with pytest.raises(ValueError, match="monotonic_ns"):
        validate_observation_metadata(metadata)


@pytest.mark.parametrize("units", [1, 1.0])
def test_scene_frame_accepts_only_explicit_metres_and_z_up(units):
    validate_scene_frame({"stage_units_in_meters": units, "up_axis": "Z"})


@pytest.mark.parametrize("scene", [
    None, {}, {"stage_units_in_meters": 0.01, "up_axis": "Z"},
    {"stage_units_in_meters": True, "up_axis": "Z"},
    {"stage_units_in_meters": "1.0", "up_axis": "Z"},
    {"stage_units_in_meters": float("nan"), "up_axis": "Z"},
    {"stage_units_in_meters": 1.0, "up_axis": "Y"},
    {"stage_units_in_meters": 1.0},
])
def test_scene_frame_rejects_unsupported_units_without_silent_conversion(scene):
    with pytest.raises(ValueError):
        validate_scene_frame(scene)

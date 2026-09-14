"""Synthetic SE(2)/timing contracts only, not actual model or Isaac evidence."""

from copy import deepcopy

import numpy as np
import pytest

from reconciliation.robotless_successive_chunks import (
    SUCCESSIVE_FRAME_CONVENTIONS, round_trip_latency_ms, successive_observation_poses,
    successive_to_world, validate_successive_observation_metadata,
)


@pytest.mark.parametrize("R0,delta,expected", [
    ([2, 3, 0], [.30, 0, 0], [2.30, 3, 0]),
    ([2, 3, np.pi / 2], [.30, 0, 0], [2, 3.30, np.pi / 2]),
    ([2, 3, -np.pi / 2], [.30, 0, 0], [2, 2.70, -np.pi / 2]),
    ([2, 3, np.pi / 2], [.30, .1, np.pi / 2], [1.90, 3.30, -np.pi]),
])
def test_scripted_displacement_is_local_se2_composition(R0, delta, expected):
    before = deepcopy((R0, delta))
    r0, r1 = successive_observation_poses(R0, delta)
    np.testing.assert_allclose(r0, R0, atol=1e-12)
    np.testing.assert_allclose(r1, expected, atol=1e-12)
    assert (R0, delta) == before


@pytest.mark.parametrize("invalid", [[], [1, 2], [[1, 2, 3]], [np.nan, 0, 0], [0, np.inf, 0],
                                    [0, 0, -np.inf], [True, False, True], ["1", "0", "0"]])
@pytest.mark.parametrize("which", [0, 1])
def test_invalid_pose_and_displacement_rejected(invalid, which):
    args = [[0, 0, 0], [.3, 0, 0]]
    args[which] = invalid
    with pytest.raises(ValueError):
        successive_observation_poses(*args)


def test_old_uses_r0_fresh_uses_r1_without_endpoint_anchor_or_cumulative_integration():
    r0, r1 = successive_observation_poses([10, 20, np.pi / 2], [.3, 0, 0])
    raw = np.array([[1, 0, 0], [2, 0, np.pi / 2]])
    old, fresh = successive_to_world(r0, r1, raw, raw)
    np.testing.assert_allclose(old, [[10, 21, np.pi / 2], [10, 22, -np.pi]], atol=1e-12)
    np.testing.assert_allclose(fresh, [[10, 21.3, np.pi / 2], [10, 22.3, -np.pi]], atol=1e-12)
    assert not np.allclose(old, fresh)
    assert not np.allclose(fresh[0, :2], old[-1, :2] + [0, 1])


@pytest.mark.parametrize("counts", [(1, 137), (137, 1), (7, 24), (10, 10)])
def test_independent_arbitrary_chunk_lengths_and_raw_immutability(counts):
    raw = [np.column_stack((np.arange(n), np.zeros(n), np.zeros(n))).astype(np.float32) for n in counts]
    before = [value.tobytes() for value in raw]
    for value in raw:
        value.flags.writeable = False
    r0 = np.array([2., 3., np.pi / 2])
    r1 = np.array([2., 3.3, np.pi / 2])
    pose_before = [r0.copy(), r1.copy()]
    world = successive_to_world(r0, r1, *raw)
    for seq, (result, n) in enumerate(zip(world, counts, strict=True)):
        assert result.shape == (n, 3)
        assert not np.shares_memory(result, raw[seq])
        np.testing.assert_allclose(result[:, 1], np.arange(n) + (3 if seq == 0 else 3.3))
        result[:] = -999
        assert raw[seq].tobytes() == before[seq]
    np.testing.assert_array_equal(r0, pose_before[0])
    np.testing.assert_array_equal(r1, pose_before[1])


@pytest.mark.parametrize("invalid", [[], [1, 2, 3], [[1, 2]], [[1, 2, 3, 4]], np.empty((0, 3)),
                                    [[np.nan, 0, 0]], [[0, np.inf, 0]], [[0, 0, -np.inf]],
                                    [["1", "2", "3"]], [[True, False, True]]])
@pytest.mark.parametrize("which", [0, 1])
def test_invalid_old_or_fresh_rejected(invalid, which):
    chunks = [[[1, 0, 0]], [[2, 0, 0]]]
    chunks[which] = invalid
    with pytest.raises(ValueError):
        successive_to_world([0, 0, 0], [.3, 0, 0], *chunks)


def event(ns, utc="2026-09-14T10:00:00Z"):
    return {"utc": utc, "monotonic_ns": ns}


def test_round_trip_uses_only_host_monotonic_nanoseconds_to_milliseconds():
    # UTC can move backwards under synchronization without changing this duration.
    sent = event(9_000_000_000, "2026-09-14T10:00:05Z")
    received = event(9_001_234_567, "2026-09-14T10:00:00Z")
    assert round_trip_latency_ms(sent, received) == 1.234567
    assert round_trip_latency_ms(sent, sent) == 0.0
    assert set(sent) == {"utc", "monotonic_ns"}  # no execution or waypoint time added


@pytest.mark.parametrize("send,receive", [
    (event(2), event(1)), (event(-1), event(2)), (event(True), event(2)),
    (event(1), event(2.5)), (event(1, "2026-09-14T10:00:00"), event(2)),
    (event(1), event(2, "2026-09-14T10:00:00+09:00")),
])
def test_invalid_round_trip_clock_domain_or_order_rejected(send, receive):
    with pytest.raises(ValueError):
        round_trip_latency_ms(send, receive)


def metadata_fixture():
    r0, r1 = successive_observation_poses([2, 3, np.pi / 2], [.3, 0, 0])
    metadata = {
        "R0": r0.tolist(), "R1": r1.tolist(), "configured_local_displacement": [.3, 0, 0],
        "coordinate_conventions": deepcopy(SUCCESSIVE_FRAME_CONVENTIONS), "execution_time": None,
        "instruction": "Go forward.", "research_git_sha": "a" * 40, "lightnav_git_sha": "b" * 40,
        "model_checkpoint_identifier": "external/model", "scene": {"asset": "synthetic-test.usd"},
        "observations": [],
    }
    for seq, pose in enumerate((r0, r1)):
        metadata["observations"].append({
            "observation_id": f"observation_{seq:03d}", "seq": seq, "label": ["OLD", "FRESH"][seq],
            "observation_time": {**event(seq + 1), "simulation_time_s": 0.0},
            "agent_pose_world": pose.tolist(), "instruction": metadata["instruction"],
            "rgb": {"path": f"raw/observation_{seq:03d}.jpg", "sha256": "c" * 64,
                    "resolution_width_height": [480, 270]},
            "camera": {"configured": {"fov": 112}, "actual_intrinsics": {"fov": 112},
                       "T_agent_camera": np.eye(4).tolist()},
        })
    return metadata


def test_observation_metadata_accepts_two_distinct_poses_without_mutation():
    metadata = metadata_fixture()
    before = deepcopy(metadata)
    validate_successive_observation_metadata(metadata)
    assert metadata == before


def test_measured_camera_extrinsic_roundoff_allowed_but_change_rejected():
    metadata = metadata_fixture()
    matrix = metadata["observations"][1]["camera"]["T_agent_camera"]
    matrix[0][3] += 1e-12
    validate_successive_observation_metadata(metadata)
    matrix[0][3] += 1e-5
    with pytest.raises(ValueError, match="T_agent_camera"):
        validate_successive_observation_metadata(metadata)


@pytest.mark.parametrize("mutation", [
    lambda m: m.update(R1=m["R0"]),
    lambda m: m.update(configured_local_displacement=[.3, 0, .5]),
    lambda m: m.update(execution_time=123),
    lambda m: m.update(coordinate_conventions={}),
    lambda m: m.update(observations=m["observations"][:1]),
    lambda m: m["observations"][1].update(seq=0),
    lambda m: m["observations"][1].update(label="OLD"),
    lambda m: m["observations"][1].update(observation_id="observation_000"),
    lambda m: m["observations"][1].update(observation_id=""),
    lambda m: m["observations"][1].update(instruction="changed"),
    lambda m: m["observations"][1].update(agent_pose_world=m["R0"]),
    lambda m: m["observations"][1]["observation_time"].update(monotonic_ns=1),
    lambda m: m["observations"][1]["camera"].update(configured={"fov": 90}),
    lambda m: m["observations"][1]["camera"].update(actual_intrinsics={"fov": 90}),
])
def test_invalid_successive_metadata_rejected(mutation):
    metadata = metadata_fixture()
    mutation(metadata)
    with pytest.raises(ValueError):
        validate_successive_observation_metadata(metadata)

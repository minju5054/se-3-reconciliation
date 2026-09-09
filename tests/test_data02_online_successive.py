from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np
import pytest

from reconciliation.data02_online_successive import (
    SuccessiveEpisodeState,
    TransitionTiming,
    chunk_content_sha256,
    classify_transition,
    difficulty_bin,
    episode_pair_components,
    geometry_bin,
    isolated_split,
    observation_anchored_paths,
    ordered_pair_sha256,
    path_geometry,
    readiness_decision,
    validate_raw_chunk,
    validate_transition_artifact,
    variant_pose,
)
from reconciliation.online_switch import sha256_file


def trajectory(n: int = 4) -> np.ndarray:
    return np.column_stack((np.linspace(.1, 1.0, n), np.linspace(0, .2, n), np.linspace(0, .3, n))).astype(np.float32)


@pytest.mark.parametrize("count", [1, 3, 10, 17])
def test_arbitrary_n_raw_and_observation_transform_without_b(count: int) -> None:
    raw = trajectory(count)
    before = raw.copy()
    observation = np.array([3.0, -2.0, np.pi / 2])
    local, world = observation_anchored_paths(raw, observation)
    assert local.shape == (count, 3)
    assert world.shape == (count, 3)
    assert np.array_equal(raw, before)
    expected_first = observation[:2] + np.array([-local[0, 1], local[0, 0]])
    assert np.allclose(world[0, :2], expected_first)
    assert not np.array_equal(world[0], observation)  # B/anchor was not prepended.


@pytest.mark.parametrize("bad", [np.array([[np.nan, 0, 0]]), np.array([[np.inf, 0, 0]]), np.zeros((2, 2)), np.zeros((0, 3))])
def test_raw_validation_rejects_nonfinite_and_wrong_shapes(bad: np.ndarray) -> None:
    with pytest.raises(ValueError):
        validate_raw_chunk(bad)


def test_chunk_and_ordered_pair_identity_are_order_sensitive() -> None:
    old, fresh = trajectory(), trajectory() + np.float32(.01)
    old_hash, fresh_hash = chunk_content_sha256(old), chunk_content_sha256(fresh)
    assert old_hash == chunk_content_sha256(old.copy())
    assert old_hash != fresh_hash
    assert ordered_pair_sha256(old_hash, fresh_hash) != ordered_pair_sha256(fresh_hash, old_hash)


def test_variant_offsets_use_robot_forward_and_left_axes() -> None:
    pose = variant_pose([1, 2, np.pi / 2], {"longitudinal_offset_m": .15, "lateral_offset_m": .09, "yaw_offset_rad": .05})
    assert np.allclose(pose[:2], [.91, 2.15])
    assert np.isclose(pose[2], np.pi / 2 + .05)


def test_timing_reconstruction_and_request_switch_distinction() -> None:
    timing = TransitionTiming(2.0, 1_000_000_000, 1_500_000_000, 2.48, 2.50, .5, .44, .96)
    result = timing.validate([.9, 1.1])
    assert result["valid"]
    assert result["tau_effective_s"] == pytest.approx(.5)
    assert result["observation_to_ready_sim_s"] == pytest.approx(.48)
    assert result["ready_to_switch_sim_s"] == pytest.approx(.02)
    assert result["t_request_host_monotonic_ns"] != result["t_switch_sim_s"]


def test_timing_rejects_order_and_rtf() -> None:
    timing = TransitionTiming(2.0, 2_000, 1_000, 1.9, 2.1, 1e-6, .4, .2)
    result = timing.validate([.9, 1.1])
    assert not result["valid"]
    assert not result["checks"]["host_order"]
    assert not result["checks"]["simulation_order"]


def test_p_to_b_semantics_are_separate_from_request(monkeypatch) -> None:
    from reconciliation.data02_online_successive import transition_metrics
    old = trajectory().astype(float); fresh = (trajectory() + .1).astype(float)
    result = transition_metrics(old, fresh, [0, 0, 0], [.2, 0, 0], [.23, 0, .01], [.2, .1], [.3, -.1])
    assert result["p_to_b_translation_m"] == pytest.approx(.03)
    assert result["p_to_b_yaw_rad"] == pytest.approx(.01)
    assert result["waypoint_time_semantics"].startswith("none")


def test_model_ready_motion_is_separate_from_effective_latency_motion() -> None:
    from reconciliation.data02_online_successive import transition_metrics
    old = trajectory().astype(float); fresh = (trajectory() + .1).astype(float)
    result = transition_metrics(
        old, fresh, [0, 0, 0], [.2, 0, 0], [.3, 0, 0], [.2, 0], [.3, 0],
        model_ready_pose=[.25, 0, 0],
    )
    assert result["observation_to_model_ready_translation_m"] == pytest.approx(.25)
    assert result["observation_to_boundary_translation_m"] == pytest.approx(.30)


def test_successive_state_promotes_fresh_without_reset() -> None:
    state = SuccessiveEpisodeState("episode_1")
    state.reset(); state.bootstrap("chunk_00"); state.start_request()
    assert state.promote_fresh("chunk_01") == ("chunk_00", "chunk_01")
    state.start_request()
    assert state.promote_fresh("chunk_02") == ("chunk_01", "chunk_02")
    assert state.reset_count == 1 and state.transition_index == 2
    with pytest.raises(RuntimeError):
        state.reset()


def test_state_rejects_two_inflight_requests() -> None:
    state = SuccessiveEpisodeState("episode_1"); state.reset(); state.bootstrap("chunk_00"); state.start_request()
    with pytest.raises(RuntimeError): state.start_request()


@pytest.mark.parametrize(
    "kwargs,expected",
    [
        ({}, "ELIGIBLE_MOVING"), ({"model_stop": True}, "MODEL_STOP"),
        ({"old_exhausted": True}, "OLD_EXHAUSTED"), ({"exhausted_before_trigger": True}, "CHUNK_EXHAUSTED_BEFORE_TRIGGER"),
        ({"timing_valid": False}, "TIMING_INVALID"), ({"finite_model_output": False}, "NONFINITE_MODEL_OUTPUT"),
        ({"collision": True}, "EXECUTION_COLLISION"), ({"out_of_envelope": True}, "EXECUTION_OUT_OF_ENVELOPE"),
        ({"technical_valid": False}, "TECHNICAL_INVALID"),
    ],
)
def test_exact_transition_statuses(kwargs, expected) -> None:
    assert classify_transition(**kwargs) == expected


def descriptor(yaw: float, lateral: float) -> dict:
    return {"signed_net_yaw_rad": yaw, "endpoint_lateral_m": lateral}


@pytest.mark.parametrize("value,expected", [(descriptor(.09,.09), "STRAIGHT_LIKE"), (descriptor(.15,0), "POSITIVE_TURNING"), (descriptor(0,.15), "POSITIVE_TURNING"), (descriptor(-.15,0), "NEGATIVE_TURNING"), (descriptor(0,-.15), "NEGATIVE_TURNING"), (descriptor(.12,.12), "OTHER")])
def test_frozen_geometry_bins(value, expected) -> None:
    assert geometry_bin(value) == expected


@pytest.mark.parametrize("dv,domega,expected", [(0.05,.1,"BENIGN"), (.15,0,"CHALLENGING"), (0,.3,"CHALLENGING"), (.08,.2,"INTERMEDIATE")])
def test_frozen_difficulty_bins(dv, domega, expected) -> None:
    assert difficulty_bin(dv, domega) == expected


def test_geometry_has_required_untimed_descriptors() -> None:
    result = path_geometry(trajectory(10))
    for key in ("number_of_poses", "path_length_m", "endpoint_forward_m", "endpoint_lateral_m", "signed_net_yaw_rad", "cumulative_abs_pose_yaw_change_rad", "max_abs_pose_yaw_increment_rad", "signed_tangent_turn_rad", "cumulative_abs_tangent_turn_rad", "maximum_lateral_excursion_m", "near_zero_segment_fraction"):
        assert key in result
    assert result["intrinsic_waypoint_time_base"] is False


def record(identifier: str, episode: str, pair: str, template: str = "H00") -> dict:
    digest = hashlib_sha(pair)
    return {"transition_id": identifier, "episode_id": episode, "ordered_raw_pair_sha256": digest, "status": "ELIGIBLE_MOVING", "template_id": template, "old_raw_sha256": hashlib_sha("old"+identifier), "fresh_raw_sha256": hashlib_sha("fresh"+identifier), "fresh_geometry_bin": "STRAIGHT_LIKE", "difficulty_bin": "BENIGN"}


def hashlib_sha(value: str) -> str:
    import hashlib
    return hashlib.sha256(value.encode()).hexdigest()


def test_connectivity_split_prevents_episode_and_pair_leakage() -> None:
    rows = [record("t0", "e0", "shared", "H0"), record("t1", "e1", "shared", "H1"), record("t2", "e2", "unique2", "H0"), record("t3", "e3", "unique3", "H1")]
    components = episode_pair_components(rows)
    assert any({row["episode_id"] for row in group} == {"e0", "e1"} for group in components)
    split = isolated_split(rows, .25)
    assert split["no_episode_leakage"] and split["no_ordered_raw_pair_leakage"]
    assert split == isolated_split(rows, .25)


def test_split_deterministically_seeds_two_sided_scenario_coverage() -> None:
    rows = []
    for scenario in ("H0", "H1", "H2"):
        for variant in range(4):
            identifier = f"{scenario}_t{variant}"
            rows.append(record(identifier, f"{scenario}_e{variant}", f"{scenario}_pair{variant}", scenario))
    split = isolated_split(rows, .25)
    assert split["scenario_coverage"]
    assert split["development_scenarios"] == ["H0", "H1", "H2"]
    assert split["heldout_scenarios"] == ["H0", "H1", "H2"]
    assert len(split["heldout_transition_ids"]) == 3


def test_readiness_does_not_weaken_failed_gates() -> None:
    rows = [record("t0", "e0", "pair")]
    split = isolated_split(rows)
    criteria = {"minimum_eligible_moving":300,"minimum_unique_ordered_raw_pairs":75,"maximum_largest_pair_fraction":.2,"minimum_unique_old_chunks":25,"minimum_unique_fresh_chunks":25,"minimum_straight_like":30,"minimum_positive_turning":20,"minimum_negative_turning":20,"minimum_benign":20,"minimum_challenging":20,"minimum_heldout_eligible":60}
    result = readiness_decision(rows, split, criteria, artifacts_valid=True, template_count=12)
    assert result["status"] == "DATA02_COLLECTED_BUT_DIVERSITY_INSUFFICIENT"
    assert not result["checks"]["A_minimum_eligible_moving"]


def write_transition_fixture(root: Path) -> Path:
    transition = root / "transition_00"; (transition / "raw").mkdir(parents=True); (transition / "derived").mkdir()
    old = trajectory(); fresh = trajectory() + .01; actual = trajectory().astype(float)
    for relative, value in (("raw/old_actions.npy", old), ("raw/fresh_actions.npy", fresh), ("derived/old_world.npy", old.astype(float)), ("derived/fresh_world.npy", fresh.astype(float)), ("actual.npy", actual)):
        with (transition / relative).open("xb") as stream: np.save(stream, value, allow_pickle=False)
    with (transition / "telemetry.csv").open("x", newline="") as stream: csv.writer(stream).writerow(["sim_time_s"]); csv.writer(stream).writerow([0.1])
    artifacts = {relative: sha256_file(transition / relative) for relative in ("raw/old_actions.npy","raw/fresh_actions.npy","derived/old_world.npy","derived/fresh_world.npy","actual.npy","telemetry.csv")}
    document = {"status":"ELIGIBLE_MOVING","waypoint_dt":None,"hashes":{"old_raw_sha256":chunk_content_sha256(old),"fresh_raw_sha256":chunk_content_sha256(fresh)},"artifact_sha256":artifacts}
    (transition / "transition.json").write_text(json.dumps(document))
    return transition


def test_strict_artifact_accepts_then_rejects_modified_raw(tmp_path: Path) -> None:
    transition = write_transition_fixture(tmp_path)
    validate_transition_artifact(transition)
    with (transition / "raw/old_actions.npy").open("wb") as stream: np.save(stream, trajectory()+1, allow_pickle=False)
    with pytest.raises(ValueError): validate_transition_artifact(transition)


def test_strict_artifact_rejects_missing_stream(tmp_path: Path) -> None:
    transition = write_transition_fixture(tmp_path)
    (transition / "telemetry.csv").unlink()
    with pytest.raises(ValueError): validate_transition_artifact(transition)


def test_save_npy_exclusive_prevents_raw_overwrite(tmp_path: Path) -> None:
    from reconciliation.lightnav_adapter import save_npy_exclusive
    destination = tmp_path / "raw.npy"
    save_npy_exclusive(destination, trajectory())
    with pytest.raises(FileExistsError): save_npy_exclusive(destination, trajectory()+1)

import csv
import json
from pathlib import Path

import numpy as np
import pytest

from reconciliation.lightnav_adapter import lightnav_local_to_world
from reconciliation.online_switch import (
    MeasuredReadyTiming,
    aggregate_experiment,
    analyze_online_ready_switch,
    apply_reporting_thresholds,
    save_json_exclusive,
    save_npy_exclusive,
    sha256_file,
    timing_validity,
    validate_experiment_output,
    validate_trial_output,
)


def analysis():
    timing = MeasuredReadyTiming(10.0, 10.6, 1_000_000_000, 1_600_000_000)
    observation = np.array([1.0, 2.0, np.pi - 0.05])
    local = np.array([[0.2, 0.0, 0.1], [0.5, 0.0, 0.2]])
    world = lightnav_local_to_world(local, observation)
    return analyze_online_ready_switch(
        actual_pose_before_ready=[0.86, 2.0, -np.pi + 0.01],
        actual_pose_at_ready=[0.90, 2.0, -np.pi + 0.02],
        robot_pose_at_new_observation=observation,
        new_world_trajectory=world,
        timing=timing,
        old_progress_at_observation=2,
        old_progress_at_ready=4,
        old_exhausted_before_new_ready=False,
    )


def test_online_raw_switch_gap_yaw_wrap_and_motion_jump() -> None:
    result = analysis()
    assert result.timing.real_time_factor == pytest.approx(1.0)
    assert result.observation_to_ready_translation_m == pytest.approx(0.1)
    assert result.observation_to_ready_yaw_rad == pytest.approx(0.07)
    assert result.transition.translation_gap_m == pytest.approx(0.10024963632952164)
    assert result.transition.previous_old_segment_translation_m == pytest.approx(0.04)
    assert result.transition.first_new_segment_translation_m == pytest.approx(0.3)
    assert result.transition.translational_motion_jump_m == pytest.approx(0.26)
    assert result.transition.yaw_gap_rad == pytest.approx(0.03)
    assert result.transition.yaw_motion_jump_rad == pytest.approx(0.09)


def test_ready_before_observation_and_invalid_host_timing_rejected() -> None:
    with pytest.raises(ValueError, match="after observation"):
        MeasuredReadyTiming(2.0, 2.0, 1, 2)
    with pytest.raises(ValueError, match="monotonic"):
        MeasuredReadyTiming(2.0, 2.1, 3, 2)


def test_old_progress_cannot_regress() -> None:
    base = analysis()
    with pytest.raises(ValueError, match="must not regress"):
        analyze_online_ready_switch(
            actual_pose_before_ready=base.actual_pose_before_ready,
            actual_pose_at_ready=base.robot_pose_at_new_ready,
            robot_pose_at_new_observation=base.robot_pose_at_new_observation,
            new_world_trajectory=[[1.0, 2.0, 0.0], [1.2, 2.0, 0.0]],
            timing=base.timing,
            old_progress_at_observation=4,
            old_progress_at_ready=3,
            old_exhausted_before_new_ready=False,
        )


def test_timing_validity_checks_rtf_motion_and_old_exhaustion() -> None:
    base = analysis()
    valid = timing_validity(base, acceptable_rtf_range=[0.9, 1.1], motion_noise_floor_m=1e-4)
    assert valid["valid"] is True
    exhausted = analyze_online_ready_switch(
        actual_pose_before_ready=base.actual_pose_before_ready,
        actual_pose_at_ready=base.robot_pose_at_new_ready,
        robot_pose_at_new_observation=base.robot_pose_at_new_observation,
        new_world_trajectory=[[1.0, 2.0, 0.0], [1.2, 2.0, 0.0]],
        timing=base.timing,
        old_progress_at_observation=2,
        old_progress_at_ready=4,
        old_exhausted_before_new_ready=True,
    )
    assert timing_validity(
        exhausted, acceptable_rtf_range=[0.9, 1.1], motion_noise_floor_m=1e-4
    )["valid"] is False


def test_thresholds_are_descriptive() -> None:
    flags = apply_reporting_thresholds(
        analysis(),
        {
            "translation_gap_m": 0.05,
            "yaw_gap_rad": 0.05,
            "translational_motion_jump_m": 0.05,
            "yaw_motion_jump_rad": 0.05,
        },
    )
    assert flags == {
        "exceeds_translation_threshold": True,
        "exceeds_yaw_threshold": False,
        "exceeds_translational_motion_jump_threshold": True,
        "exceeds_yaw_motion_jump_threshold": True,
    }


def test_online_api_has_no_waypoint_dt_dependency() -> None:
    payload = analysis().to_dict()
    assert payload["intrinsic_waypoint_time_base"] is False
    assert "waypoint_dt" not in json.dumps(payload)
    assert payload["raw_switch_policy"].startswith("use NEW row 0")


def test_aggregate_summary_keeps_raw_lists_and_counts() -> None:
    base = analysis().to_dict()
    flags = apply_reporting_thresholds(
        analysis(),
        {
            "translation_gap_m": 0.05,
            "yaw_gap_rad": 0.05,
            "translational_motion_jump_m": 0.05,
            "yaw_motion_jump_rad": 0.05,
        },
    )
    rows = [
        {**base, "valid": True, "threshold_exceedance": flags},
        {**base, "valid": True, "threshold_exceedance": flags},
        {**base, "valid": False, "threshold_exceedance": flags},
    ]
    result = aggregate_experiment(rows)
    assert result["valid_trial_count"] == 2
    assert result["invalid_trial_count"] == 1
    expected_gap = analysis().transition.translation_gap_m
    assert result["raw_lists"]["translation_gap_m"] == pytest.approx(
        [expected_gap, expected_gap]
    )
    assert result["statistics"]["translation_gap_m"]["median"] == pytest.approx(expected_gap)
    assert result["threshold_exceedance_counts"]["exceeds_translation_threshold"] == 2


def build_trial(root: Path) -> dict:
    for folder in ("raw", "derived", "results"):
        (root / folder).mkdir(parents=True, exist_ok=True)
    old = np.array([[0.1, 0.0, 0.0], [0.2, 0.0, 0.0]], dtype=np.float32)
    new = np.array([[0.2, 0.0, 0.1], [0.5, 0.0, 0.2]], dtype=np.float32)
    old_anchor = [0.0, 0.0, 0.0]
    new_anchor = [1.0, 2.0, 0.0]
    old_world = lightnav_local_to_world(old, old_anchor)
    new_world = lightnav_local_to_world(new, new_anchor)
    ready = [1.1, 2.0, 0.02]
    save_npy_exclusive(root / "raw/old_actions.npy", old)
    save_npy_exclusive(root / "raw/new_actions.npy", new)
    (root / "raw/old_raw_text.txt").write_text("old", encoding="utf-8")
    (root / "raw/new_raw_text.txt").write_text("new", encoding="utf-8")
    (root / "raw/new_observation_rgb.png").write_bytes(b"fixture")
    save_npy_exclusive(root / "derived/old_world.npy", old_world)
    save_npy_exclusive(root / "derived/new_world.npy", new_world)
    save_npy_exclusive(root / "derived/new_controller_reference.npy", np.vstack((ready, new_world)))
    save_npy_exclusive(root / "derived/actual_trajectory.npy", [[0, 0, 0], ready])
    with (root / "derived/timeline.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=(
                "sim_time_s", "host_monotonic_ns", "phase", "actual_x", "actual_y",
                "actual_yaw", "commanded_v", "commanded_omega", "active_chunk",
                "active_reference_index", "new_inference_in_flight", "rgb_frame_index",
            ),
        )
        writer.writeheader()
        writer.writerow({key: 0 for key in writer.fieldnames})
    event_names = (
        "episode_start", "old_observation", "old_ready", "old_execution_start",
        "new_observation", "new_request_sent", "new_ready", "raw_switch",
        "new_execution_start", "episode_end",
    )
    save_json_exclusive(root / "raw/event_log.json", [{"event": name} for name in event_names])
    measured = analyze_online_ready_switch(
        actual_pose_before_ready=[1.05, 2.0, 0.01],
        actual_pose_at_ready=ready,
        robot_pose_at_new_observation=new_anchor,
        new_world_trajectory=new_world,
        timing=MeasuredReadyTiming(1.0, 1.6, 1_000_000_000, 1_600_000_000),
        old_progress_at_observation=0,
        old_progress_at_ready=1,
        old_exhausted_before_new_ready=False,
    )
    payload = measured.to_dict()
    payload["valid"] = True
    payload["threshold_exceedance"] = apply_reporting_thresholds(
        measured,
        {
            "translation_gap_m": 0.05, "yaw_gap_rad": 0.05,
            "translational_motion_jump_m": 0.05, "yaw_motion_jump_rad": 0.05,
        },
    )
    save_json_exclusive(root / "results/switch_metrics.json", payload)
    save_json_exclusive(root / "results/timing.json", {"valid": True})
    save_json_exclusive(
        root / "metadata.json",
        {
            "action_horizon": 2,
            "intrinsic_waypoint_time_base": False,
            "robot_pose_at_old_observation": old_anchor,
            "raw_sha256": {
                "old_actions.npy": sha256_file(root / "raw/old_actions.npy"),
                "new_actions.npy": sha256_file(root / "raw/new_actions.npy"),
            },
        },
    )
    return payload


def test_raw_derived_separation_strict_validation_and_immutable_output(tmp_path: Path) -> None:
    payload = build_trial(tmp_path)
    result = validate_trial_output(tmp_path)
    assert result["valid_output"] is True
    assert result["research_valid"] is True
    assert result["old_shape"] == [2, 3]
    with pytest.raises(FileExistsError):
        save_json_exclusive(tmp_path / "metadata.json", {})

    experiment = tmp_path.parent / "experiment"
    trial_root = experiment / "trial_000"
    build_trial(trial_root)
    aggregate = aggregate_experiment([payload])
    save_json_exclusive(experiment / "summary.json", {"aggregate": aggregate})
    save_json_exclusive(
        experiment / "metadata.json",
        {"trial_count": 1, "intrinsic_waypoint_time_base": False},
    )
    assert validate_experiment_output(experiment)["research_valid_trial_count"] == 1

    retry_experiment = tmp_path.parent / "retry_experiment"
    retry_trial_root = retry_experiment / "trial_000"
    build_trial(retry_trial_root)
    save_json_exclusive(retry_experiment / "summary.json", {"aggregate": aggregate})
    save_json_exclusive(
        retry_experiment / "metadata.json",
        {"attempted_trial_count": 1, "intrinsic_waypoint_time_base": False},
    )
    assert validate_experiment_output(retry_experiment)["trial_count"] == 1


def test_strict_json_and_fabricated_timing_rejected(tmp_path: Path) -> None:
    build_trial(tmp_path)
    metadata_path = tmp_path / "metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata["waypoint_dt_seconds"] = 0.25
    metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
    with pytest.raises(ValueError, match="forbidden"):
        validate_trial_output(tmp_path)

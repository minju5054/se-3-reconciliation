import copy
import csv
from pathlib import Path

import numpy as np
import pytest

from reconciliation.exp01b_extension import (
    aggregate_extension,
    classify_attempt,
    descriptive_statistics,
    enumerate_condition_attempts,
    geometry_descriptors,
    is_stop_actions,
    select_representative_samples,
    timeline_inference_activity,
    validate_extension_config,
    write_csv_exclusive,
)


def config():
    return {
        "extension_cohort": {
            "target_valid_per_condition": 6,
            "max_attempts_per_condition": 10,
            "conditions": [
                {
                    "id": f"condition_{letter}",
                    "description": f"fixture {letter}",
                    "initial_pose_se2": [0.0, offset, yaw],
                    "new_observation_delay_s": delay,
                }
                for letter, offset, yaw, delay in (
                    ("A", 0.0, 0.0, 0.5),
                    ("B", 0.15, 0.1, 0.5),
                    ("C", -0.15, -0.1, 0.5),
                    ("D", 0.0, 0.0, 0.75),
                )
            ],
        }
    }


def record(condition="condition_A", attempt=0, *, valid=True, stop=False, jump=0.1):
    checks = {
        "ready_after_observation": True,
        "rtf_in_range": valid,
        "robot_moved_during_inference": True,
        "old_not_exhausted_before_ready": True,
        "old_progress_nonregressing": True,
        "within_configured_new_wait": True,
    }
    classification = classify_attempt(validity_checks=checks, stop_output=stop)
    return {
        "condition_id": condition,
        "attempt_index": attempt,
        "global_episode_index": attempt,
        "classification": classification,
        "timing_valid": valid,
        "stop_output": stop,
        "artifact_path": f"data/fixture/{condition}/attempt_{attempt:03d}",
        "timing": {
            "request_response_wall_s": 0.55,
            "simulation_observation_to_ready_s": 0.53,
            "real_time_factor": 0.963636363636,
        },
        "lightnav_predict_host_latency_ms": 540.0,
        "lightnav_reported_latency_ms": 500.0,
        "observation_to_ready_translation_m": 0.16,
        "observation_to_ready_yaw_rad": -0.02,
        "old_progress_delta": 2,
        "inference_activity": {
            "in_flight_timeline_samples": 31,
            "in_flight_nonzero_old_command_fraction": 1.0,
        },
        "metrics": {
            "translation_gap_m": 0.06,
            "yaw_gap_rad": 0.02,
            "translational_motion_jump_m": jump,
            "yaw_motion_jump_rad": 0.01,
        },
        "geometry": {
            "old_incoming_motion_m": 0.04,
            "fresh_first_motion_m": 0.14,
            "old_fresh_initial_heading_disagreement_abs_rad": 0.2,
            "fresh_old_motion_mismatch_abs_m": 0.1,
        },
        "threshold_exceedance": {
            "exceeds_translation_threshold": True,
            "exceeds_yaw_threshold": False,
            "exceeds_translational_motion_jump_threshold": True,
            "exceeds_yaw_motion_jump_threshold": False,
        },
    }


def test_condition_validation_and_enumeration_are_frozen_and_deterministic() -> None:
    conditions = validate_extension_config(config())
    assert [item["id"] for item in conditions] == [
        "condition_A", "condition_B", "condition_C", "condition_D"
    ]
    plan = enumerate_condition_attempts(config())
    assert len(plan) == 40
    assert plan[:2] == [("condition_A", 0), ("condition_A", 1)]
    assert plan[-1] == ("condition_D", 9)


@pytest.mark.parametrize("bad_index", (-1, 4))
def test_condition_count_and_invalid_pose_rejected(bad_index: int) -> None:
    source = config()
    if bad_index == -1:
        source["extension_cohort"]["conditions"].pop()
    else:
        source["extension_cohort"]["conditions"][0]["initial_pose_se2"][0] = np.nan
    with pytest.raises(ValueError):
        validate_extension_config(source)


def test_attempt_classification_preserves_stop_and_failure_reasons() -> None:
    checks = {
        "ready_after_observation": True,
        "rtf_in_range": True,
        "robot_moved_during_inference": True,
        "old_not_exhausted_before_ready": True,
        "old_progress_nonregressing": True,
        "within_configured_new_wait": True,
    }
    assert classify_attempt(validity_checks=checks, stop_output=False) == "VALID"
    assert classify_attempt(validity_checks=checks, stop_output=True) == "MODEL_STOP_OUTPUT"
    for key, expected in (
        ("rtf_in_range", "TIMING_INVALID"),
        ("old_not_exhausted_before_ready", "OLD_EXHAUSTED"),
        ("within_configured_new_wait", "NEW_TIMEOUT"),
    ):
        changed = {**checks, key: False}
        assert classify_attempt(validity_checks=changed, stop_output=False) == expected
    assert classify_attempt(validity_checks=checks, stop_output=False, completed=False) == (
        "OTHER_PROTOCOL_FAILURE"
    )


def test_stop_classification_supports_arbitrary_horizon_without_mutation() -> None:
    for horizon in (1, 10, 24):
        actions = np.zeros((horizon, 3), dtype=np.float32)
        original = actions.copy()
        assert is_stop_actions(actions)
        assert np.array_equal(actions, original)
    moving = np.zeros((7, 3), dtype=np.float64)
    moving[-1, 0] = 0.1
    assert not is_stop_actions(moving)
    with pytest.raises(ValueError):
        is_stop_actions(np.array([[np.inf, 0.0, 0.0]]))


def test_geometry_descriptors_and_heading_wrap() -> None:
    result = geometry_descriptors(
        actual_pose_before_ready=[0.0, 0.0, np.pi - 0.01],
        actual_pose_at_ready=[0.1, 0.0, -np.pi + 0.01],
        fresh_world=[[0.2, 0.0, 0.0], [0.2, 0.2, 0.1]],
    )
    assert result["old_incoming_motion_m"] == pytest.approx(0.1)
    assert result["old_incoming_heading_rad"] == pytest.approx(0.0)
    assert result["fresh_first_heading_rad"] == pytest.approx(np.pi / 2)
    assert result["old_fresh_initial_heading_disagreement_rad"] == pytest.approx(np.pi / 2)
    assert result["old_incoming_yaw_increment_rad"] == pytest.approx(0.02)


def test_zero_length_motion_is_explicitly_undefined() -> None:
    result = geometry_descriptors(
        actual_pose_before_ready=[0.0, 0.0, 0.0],
        actual_pose_at_ready=[0.0, 0.0, 0.0],
        fresh_world=[[1.0, 1.0, 0.0], [1.0, 1.0, 0.0]],
    )
    assert result["old_incoming_heading_defined"] is False
    assert result["fresh_first_heading_defined"] is False
    assert result["old_fresh_initial_heading_disagreement_rad"] is None
    assert result["fresh_to_old_motion_ratio"] is None


def test_timeline_activity_counts_nonzero_old_control() -> None:
    rows = [
        {"new_inference_in_flight": 1, "commanded_v": 0.2, "commanded_omega": 0.0},
        {"new_inference_in_flight": 1, "commanded_v": 0.0, "commanded_omega": 0.0},
        {"new_inference_in_flight": 0, "commanded_v": 0.2, "commanded_omega": 0.0},
    ]
    result = timeline_inference_activity(rows)
    assert result["in_flight_timeline_samples"] == 2
    assert result["in_flight_nonzero_old_command_samples"] == 1
    assert result["in_flight_nonzero_old_command_fraction"] == pytest.approx(0.5)


def test_statistics_include_requested_percentiles_and_reject_invalid() -> None:
    result = descriptive_statistics([1.0, 2.0, 3.0, 4.0])
    assert result["mean"] == pytest.approx(2.5)
    assert result["p25"] == pytest.approx(1.75)
    assert result["p75"] == pytest.approx(3.25)
    assert result["p90"] == pytest.approx(3.7)
    with pytest.raises(ValueError):
        descriptive_statistics([1.0, np.nan])


def test_aggregate_reports_conditions_stop_subsets_and_combined_thresholds() -> None:
    records = [
        record(condition, index, stop=(condition == "condition_B" and index == 0))
        for condition in ("condition_A", "condition_B", "condition_C", "condition_D")
        for index in range(2)
    ]
    result = aggregate_extension(
        records,
        condition_ids=["condition_A", "condition_B", "condition_C", "condition_D"],
    )
    assert result["overall"]["timing_valid"] == 8
    assert result["overall"]["stop"] == 1
    assert result["overall"]["excluding_stop"]["count"] == 7
    assert result["overall"]["including_stop"]["statistics"]["translation_gap_m"][
        "median"
    ] == pytest.approx(0.06)
    assert result["overall"]["including_stop"]["threshold_exceedance"][
        "translation_inconsistent"
    ]["count"] == 8


def test_representative_selection_is_median_nearest_and_deterministic() -> None:
    records = [
        record("condition_A", 0, jump=0.2),
        record("condition_A", 1, jump=0.1),
        record("condition_A", 2, jump=0.3),
        record("condition_A", 3, stop=True, jump=0.0),
    ]
    selected = select_representative_samples(records, ["condition_A"])
    assert selected["condition_A"].endswith("attempt_000")


def test_csv_schema_and_immutable_output(tmp_path: Path) -> None:
    source = record()
    before = copy.deepcopy(source)
    output = tmp_path / "attempts.csv"
    write_csv_exclusive(output, [source])
    with output.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert rows[0]["condition_id"] == "condition_A"
    assert "heading_disagreement_rad" in rows[0]
    assert source == before
    with pytest.raises(FileExistsError):
        write_csv_exclusive(output, [source])

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from reconciliation.closed_loop_execution_validation import (
    CONTROLLED_IDS,
    ClosedLoopTelemetry,
    compare_modes,
    compute_closed_loop_metrics,
    evaluate_controlled_group,
    evaluate_exp02b_group,
    evaluate_scenario,
    final_platform_decision,
    generate_controlled_references,
    load_frozen_stage0d_candidate,
    save_closed_loop_trial,
    validate_closed_loop_trial,
    validate_controlled_geometry,
    validate_stage0e_config,
)
from reconciliation.online_switch import sha256_file


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def load_config() -> dict:
    return yaml.safe_load(
        (REPOSITORY_ROOT / "configs/stage0_closed_loop_execution_validation.yaml").read_text(
            encoding="utf-8"
        )
    )


def telemetry_fixture() -> ClosedLoopTelemetry:
    reference = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    actual = reference.copy()
    return ClosedLoopTelemetry(
        reference,
        actual,
        np.array([0.0, 1.0, 2.0]),
        np.array([-1, 0, 1]),
        np.array([0, 1, 2]),
        np.array([0, 1, 2]),
        np.array([False, False, True]),
        np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 0.0]]),
        np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 0.0]]),
        np.zeros((3, 4)),
        np.zeros((3, 4)),
        np.zeros(3),
        np.zeros(3),
        np.zeros(3, dtype=bool),
        np.zeros(3, dtype=bool),
    )


def passing_trial(**overrides: object) -> dict:
    row = {
        "position_rmse_m": 0.05,
        "yaw_rmse_rad": 0.04,
        "final_position_error_m": 0.06,
        "final_yaw_error_rad": 0.03,
        "goal_reached": True,
        "numerically_stable": True,
        "active_saturation_fraction": 0.0,
        "progress_monotonic": True,
        "progress_completion_ratio": 1.0,
        "desired_v_vs_measured_v_rmse_mps": 0.02,
        "desired_omega_vs_measured_omega_rmse_rps": 0.03,
        "wheel_target_vs_measured_rmse_rad_s": 0.2,
    }
    row.update(overrides)
    return row


def test_controlled_references_have_deterministic_lengths_and_geometry() -> None:
    references = generate_controlled_references(load_config())
    assert tuple(references) == CONTROLLED_IDS
    validation = validate_controlled_geometry(references)
    assert validation["sample_counts"] == {
        "straight": 41,
        "gentle_left": 46,
        "gentle_right": 46,
        "strong_left": 36,
        "strong_right": 36,
        "s_curve": 51,
        "straight_turn_straight": 56,
    }
    assert validation["path_lengths_m"] == pytest.approx(
        {
            "straight": 1.4,
            "gentle_left": 1.44,
            "gentle_right": 1.44,
            "strong_left": 0.84,
            "strong_right": 0.84,
            "s_curve": 1.5,
            "straight_turn_straight": 1.525,
        }
    )
    assert all(validation["checks"].values())


def test_metrics_separate_reference_body_wheel_and_controller_levels() -> None:
    metrics = compute_closed_loop_metrics(telemetry_fixture())
    assert metrics["position_rmse_m"] == pytest.approx(0.0)
    assert metrics["yaw_rmse_rad"] == pytest.approx(0.0)
    assert metrics["desired_v_vs_measured_v_rmse_mps"] == pytest.approx(0.0)
    assert metrics["executed_v_vs_measured_v_rmse_mps"] == pytest.approx(0.0)
    assert metrics["wheel_target_vs_measured_rmse_rad_s"] == pytest.approx(0.0)
    assert metrics["goal_reached"] is True
    assert metrics["progress_completion_ratio"] == pytest.approx(1.0)
    assert "not time-aligned" in metrics["metric_semantics"]["trajectory"]


def test_telemetry_copies_sources_and_rejects_nonfinite() -> None:
    fixture = telemetry_fixture()
    reference = fixture.reference_trajectory.copy()
    actual = fixture.actual_trajectory.copy()
    telemetry = ClosedLoopTelemetry(
        reference,
        actual,
        fixture.sim_times_s,
        fixture.control_indices,
        fixture.nearest_indices,
        fixture.target_indices,
        fixture.goal_reached,
        fixture.desired_body,
        fixture.executed_body,
        fixture.target_wheels,
        fixture.measured_wheels,
        fixture.pi_correction_rps,
        fixture.integral_error_rad,
        fixture.saturated,
        fixture.sign_protection_events,
    )
    reference[:] = 99.0
    actual[:] = 88.0
    assert telemetry.reference_trajectory[0, 0] == pytest.approx(0.0)
    assert telemetry.actual_trajectory[0, 0] == pytest.approx(0.0)
    bad = telemetry_fixture()
    values = bad.actual_trajectory.copy()
    values[1, 0] = np.nan
    with pytest.raises(ValueError, match="finite"):
        ClosedLoopTelemetry(
            bad.reference_trajectory,
            values,
            bad.sim_times_s,
            bad.control_indices,
            bad.nearest_indices,
            bad.target_indices,
            bad.goal_reached,
            bad.desired_body,
            bad.executed_body,
            bad.target_wheels,
            bad.measured_wheels,
            bad.pi_correction_rps,
            bad.integral_error_rad,
            bad.saturated,
            bad.sign_protection_events,
        )


def test_repeatability_acceptance_group_and_platform_selection() -> None:
    criteria = load_config()["acceptance_criteria"]
    passed = evaluate_scenario([passing_trial() for _ in range(3)], criteria)
    assert passed["passed"] is True
    assert passed["repeatability"]["position_rmse_m"]["std"] == pytest.approx(0.0)
    failed = evaluate_scenario(
        [passing_trial(yaw_rmse_rad=0.2) for _ in range(3)], criteria
    )
    assert failed["passed"] is False
    scenarios = {key: passed for key in CONTROLLED_IDS}
    geometry = {
        "straight": "straight",
        "gentle_left": "left",
        "gentle_right": "right",
        "strong_left": "left",
        "strong_right": "right",
        "s_curve": "direction_change",
        "straight_turn_straight": "direction_change",
    }
    assert evaluate_controlled_group(scenarios, geometry, criteria)["passed"] is True
    exp = {key: passed for key in ("a", "b", "c")}
    assert evaluate_exp02b_group(exp, criteria)["passed"] is True
    assert final_platform_decision(nominal_passed=True, calibrated_passed=False)[
        "status"
    ] == "NOMINAL_PLATFORM_SELECTED"
    assert final_platform_decision(nominal_passed=True, calibrated_passed=True)[
        "status"
    ] == "BOTH_PASS_NOMINAL_SELECTED_FOR_SIMPLICITY"
    assert final_platform_decision(nominal_passed=False, calibrated_passed=False)[
        "status"
    ] == "EXECUTION_PLATFORM_NOT_READY"


def test_mode_comparison_reports_calibrated_minus_nominal() -> None:
    nominal = passing_trial()
    calibrated = passing_trial(yaw_rmse_rad=0.02)
    comparison = compare_modes({"trials": [nominal]}, {"trials": [calibrated]})
    assert comparison["yaw_rmse_rad"]["calibrated_minus_nominal"] == pytest.approx(-0.02)


def test_config_freezes_gates_and_forbids_held_out_tuning() -> None:
    config = load_config()
    validation = validate_stage0e_config(config)
    assert validation["criteria_frozen"] is True
    assert validation["held_out_controller_tuning_permitted"] is False
    assert validation["historical_commands_reused"] is False
    config["controlled_paths"]["repetitions"] = 2
    with pytest.raises(ValueError, match="at least three"):
        validate_stage0e_config(config)


def test_frozen_stage0d_parameter_loading_and_hash_validation(tmp_path: Path) -> None:
    model = {
        "model_type": "fixture",
        "selected_parameters": {"yaw_rate_kp": 3.0},
        "feedback_selection": {"selected_candidate_id": "pi_strong"},
        "held_out_data_used_for_fitting_or_tuning": False,
        "runtime_controller_config": {"maximum_abs_executed_omega_rps": 8.0},
    }
    metadata = {"run_id": "fixture", "smoke": False}
    final = {"decision": {"status": "EXECUTION_LAYER_NOT_YET_VALIDATED"}}
    for name, value in (
        ("calibration_model.json", model),
        ("metadata.json", metadata),
        ("final_execution_layer_validation.json", final),
    ):
        (tmp_path / name).write_text(json.dumps(value), encoding="utf-8")
    (tmp_path / "config_snapshot.yaml").write_text("stage: fixture\n", encoding="utf-8")
    provenance = {
        "run_id": "fixture",
        "selected_candidate_id": "pi_strong",
        "calibration_model_sha256": sha256_file(tmp_path / "calibration_model.json"),
        "metadata_sha256": sha256_file(tmp_path / "metadata.json"),
        "config_snapshot_sha256": sha256_file(tmp_path / "config_snapshot.yaml"),
    }
    loaded = load_frozen_stage0d_candidate(tmp_path, provenance)
    assert loaded["selected_parameters"] == {"yaw_rate_kp": 3.0}
    assert loaded["held_out_data_used_for_fitting_or_tuning"] is False
    provenance["metadata_sha256"] = "0" * 64
    with pytest.raises(ValueError, match="provenance mismatch"):
        load_frozen_stage0d_candidate(tmp_path, provenance)


def test_immutable_trial_output_and_validation(tmp_path: Path) -> None:
    telemetry = telemetry_fixture()
    metrics = compute_closed_loop_metrics(telemetry)
    metadata = {
        "mode": "nominal",
        "scenario": "fixture",
        "repetition": 0,
        "evaluation_semantics": "reference-matched closed-loop evaluation",
    }
    destination = tmp_path / "trial"
    save_closed_loop_trial(destination, telemetry, metrics, metadata)
    assert validate_closed_loop_trial(destination)["valid"] is True
    with pytest.raises(FileExistsError):
        save_closed_loop_trial(destination, telemetry, metrics, metadata)
    with pytest.raises(ValueError, match="non-finite"):
        save_closed_loop_trial(
            tmp_path / "bad", telemetry, {"bad": float("nan")}, metadata
        )

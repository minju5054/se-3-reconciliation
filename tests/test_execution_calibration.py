from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
import yaml

from reconciliation.execution_calibration import (
    ExecutionTelemetry,
    assert_disjoint_condition_sets,
    compute_trial_metrics,
    evaluate_composite_acceptance,
    evaluate_exp02b_replay_acceptance,
    final_validation_decision,
    fit_effective_width_model,
    response_dependence,
    save_execution_trial,
    summarize_conditions,
    validate_execution_calibration_config,
    validate_execution_trial,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def telemetry_fixture(*, measured_omega: float = 0.2) -> ExecutionTelemetry:
    times = np.arange(0.0, 5.0 + 0.1, 0.1)
    count = times.size
    phases = tuple(
        "INITIAL_STOP" if t <= 0.5 else "ACTIVE" if t <= 4.5 else "FINAL_STOP"
        for t in times
    )
    desired = np.zeros((count, 2), dtype=float)
    active = np.asarray(phases) == "ACTIVE"
    desired[active] = [0.2, 0.4]
    poses = np.zeros((count, 3), dtype=float)
    for index in range(1, count):
        if active[index]:
            poses[index, 0] = poses[index - 1, 0] + 0.15 * np.cos(poses[index - 1, 2]) * 0.1
            poses[index, 1] = poses[index - 1, 1] + 0.15 * np.sin(poses[index - 1, 2]) * 0.1
            poses[index, 2] = poses[index - 1, 2] + measured_omega * 0.1
        else:
            poses[index] = poses[index - 1]
    target = np.zeros((count, 4), dtype=float)
    target[active] = [1.0, 3.0, 1.0, 3.0]
    measured = np.zeros((count, 4), dtype=float)
    measured[active] = [1.0, 2.0, 1.0, 2.0]
    return ExecutionTelemetry(
        actual_trajectory=poses,
        sim_times_s=times,
        control_indices=np.arange(-1, count - 1),
        phases=phases,
        desired_body=desired,
        executed_body=desired,
        ideal_left_right=np.column_stack((target[:, 0], target[:, 1])),
        target_wheels=target,
        measured_wheels=measured,
        saturated=np.zeros(count, dtype=bool),
    )


def criteria_fixture() -> dict:
    return {
        "steady_state_sign_correct_fraction": 1.0,
        "per_condition_omega_mean_abs_error_floor_rps": 0.06,
        "per_condition_omega_mean_abs_error_fraction": 0.2,
        "aggregate_omega_rmse_rps": 0.12,
        "aggregate_v_rmse_mps": 0.06,
        "per_condition_repeatability_std_omega_rps": 0.05,
        "maximum_active_saturation_fraction": 0.2,
        "maximum_sustained_sign_alternations": 4,
    }


def test_trial_metrics_separate_body_wheels_and_effective_width() -> None:
    telemetry = telemetry_fixture()
    before = telemetry.actual_trajectory.copy()
    metrics = compute_trial_metrics(
        telemetry,
        wheel_radius_m=0.1,
        steady_state_tail_s=1.5,
        minimum_abs_measured_omega_rps=0.02,
    )
    assert metrics["steady_measured_v_mean_mps"] == pytest.approx(0.15, abs=1e-9)
    assert metrics["steady_measured_omega_mean_rps"] == pytest.approx(0.2)
    assert metrics["omega_steady_state_gain"] == pytest.approx(0.5)
    assert metrics["wheel_target_measured_rmse_rad_s"] == pytest.approx(np.sqrt(0.5))
    assert metrics["effective_track_width_mean_m"] == pytest.approx(0.5)
    np.testing.assert_array_equal(telemetry.actual_trajectory, before)


def test_near_zero_omega_makes_effective_width_undefined() -> None:
    metrics = compute_trial_metrics(
        telemetry_fixture(measured_omega=0.0),
        wheel_radius_m=0.1,
        steady_state_tail_s=1.5,
        minimum_abs_measured_omega_rps=0.02,
    )
    assert metrics["effective_track_width_sample_count"] == 0
    assert metrics["effective_track_width_mean_m"] is None


def fit_rows() -> list[dict]:
    rows = []
    for condition, desired, measured in (
        ("left", 0.3, 0.15),
        ("right", -0.3, -0.15),
        ("left_fast", 0.6, 0.3),
        ("right_fast", -0.6, -0.3),
    ):
        rows.append(
            {
                "condition_id": condition,
                "desired_omega_rps": desired,
                "steady_measured_omega_mean_rps": measured,
            }
        )
    return rows


def test_effective_width_fit_is_reproducible() -> None:
    assessment = {
        "leave_one_condition_out_omega_rmse_rps": 0.01,
        "maximum_condition_mean_abs_error_rps": 0.01,
        "minimum_sign_correct_fraction": 1.0,
        "maximum_group_gain_range": 0.01,
    }
    first = fit_effective_width_model(
        fit_rows(),
        physical_wheel_separation_m=0.4,
        maximum_feedforward_scale=10.0,
        assessment=assessment,
    )
    second = fit_effective_width_model(
        fit_rows(),
        physical_wheel_separation_m=0.4,
        maximum_feedforward_scale=10.0,
        assessment=assessment,
    )
    assert first == second
    assert first["sufficient"] is True
    assert first["calibrated_effective_wheel_separation_m"] == pytest.approx(0.8)


def test_held_out_overlap_is_rejected() -> None:
    with pytest.raises(ValueError, match="held-out leakage"):
        assert_disjoint_condition_sets([(0.0, 0.3)], [(0.0, 0.3)])


def test_left_right_asymmetry_metric() -> None:
    rows = [
        {
            "desired_v_mps": 0.2,
            "desired_omega_rps": 0.3,
            "omega_steady_state_gain": 0.5,
            "steady_measured_omega_mean_rps": 0.15,
        },
        {
            "desired_v_mps": 0.2,
            "desired_omega_rps": -0.3,
            "omega_steady_state_gain": 0.4,
            "steady_measured_omega_mean_rps": -0.12,
        },
    ]
    result = response_dependence(rows)
    assert result["maximum_abs_left_right_response_difference_rps"] == pytest.approx(0.03)


def test_config_validation_and_leakage() -> None:
    with (REPOSITORY_ROOT / "configs/stage0_jackal_execution_calibration.yaml").open() as stream:
        config = yaml.safe_load(stream)
    result = validate_execution_calibration_config(config)
    assert result["calibration_condition_count"] == 21
    assert result["held_out_condition_count"] == 12
    invalid = deepcopy(config)
    invalid["held_out_validation"]["linear_velocity_mps"] = [0.0]
    invalid["held_out_validation"]["angular_velocity_rps"] = [0.3]
    with pytest.raises(ValueError, match="held-out leakage"):
        validate_execution_calibration_config(invalid)


def test_validation_decisions_cover_pass_partial_fail() -> None:
    assert final_validation_decision(
        calibration_grid_passed=True,
        held_out_passed=True,
        composite_passed=True,
        representative_replay_passed=True,
        held_out_leakage=False,
    )["status"] == "EXECUTION_LAYER_CALIBRATED_AND_VALIDATED"
    assert final_validation_decision(
        calibration_grid_passed=True,
        held_out_passed=False,
        composite_passed=True,
        representative_replay_passed=False,
        held_out_leakage=False,
    )["status"] == "PARTIAL_VALIDATION"
    assert final_validation_decision(
        calibration_grid_passed=False,
        held_out_passed=False,
        composite_passed=False,
        representative_replay_passed=False,
        held_out_leakage=True,
    )["status"] == "EXECUTION_LAYER_NOT_YET_VALIDATED"


def test_composite_and_replay_acceptance() -> None:
    nominal = {
        "goal_reached": True,
        "position_rmse_m": 0.1,
        "final_position_error_m": 0.1,
        "yaw_rmse_rad": 0.08,
        "final_yaw_error_rad": 0.08,
        "desired_omega_rmse_rps": 0.3,
    }
    calibrated = {**nominal, "position_rmse_m": 0.09, "desired_omega_rmse_rps": 0.2}
    criteria = {
        "final_position_error_m": 0.2,
        "position_rmse_m": 0.15,
        "final_yaw_error_rad": 0.1,
        "yaw_rmse_rad": 0.1,
        "non_degradation_tolerance": 1e-6,
    }
    assert evaluate_composite_acceptance(nominal, calibrated, criteria)["passed"] is True
    replay_nominal = {
        "body_and_wheel_execution_metrics": {
            "omega_command_vs_measured_rmse_rps": 0.6
        },
        "old_reference_spatial_metrics": {"old_reference_distance_rms_m": 0.08},
    }
    replay_calibrated = {
        "body_and_wheel_execution_metrics": {
            "omega_command_vs_measured_rmse_rps": 0.3
        },
        "old_reference_spatial_metrics": {"old_reference_distance_rms_m": 0.09},
    }
    replay_criteria = {
        "minimum_representative_omega_rmse_reduction_fraction": 0.3,
        "maximum_spatial_rms_ratio": 1.25,
        "maximum_spatial_rms_absolute_increase_m": 0.02,
    }
    assert evaluate_exp02b_replay_acceptance(
        replay_nominal, replay_calibrated, replay_criteria
    )["passed"] is True


def test_immutable_trial_output_and_nonfinite_rejection(tmp_path: Path) -> None:
    telemetry = telemetry_fixture()
    destination = tmp_path / "trial"
    metrics = {"finite": 1.0}
    metadata = {"mode": "nominal", "condition_id": "fixture", "repetition": 0}
    save_execution_trial(destination, telemetry, metrics, metadata)
    assert validate_execution_trial(destination)["valid"] is True
    with pytest.raises(FileExistsError):
        save_execution_trial(destination, telemetry, metrics, metadata)
    with pytest.raises(ValueError, match="non-finite"):
        save_execution_trial(tmp_path / "bad", telemetry, {"bad": float("nan")}, metadata)

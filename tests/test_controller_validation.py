from pathlib import Path

import numpy as np
import pytest

from reconciliation.controller_validation import (
    ControllerTelemetry,
    compute_controller_metrics,
    estimate_body_velocities,
    save_controller_run,
    validate_controller_run,
)


def telemetry_fixture() -> ControllerTelemetry:
    reference = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.1], [2.0, 0.0, 0.2]])
    actual = np.array([[0.0, 0.0, 0.0], [0.8, 0.0, 0.05], [1.8, 0.0, 0.15]])
    return ControllerTelemetry(
        reference,
        actual,
        np.array([0.0, 1.0, 2.0]),
        np.array([[0.0, 0.0], [1.0, 0.1], [1.0, 0.1]]),
        np.ones((3, 4)),
        np.full((3, 4), 0.9),
        np.array([0, 1, 2]),
        ("fl", "fr", "rl", "rr"),
    )


def test_body_velocity_and_yaw_rate_estimation() -> None:
    poses = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.1], [2.0, 0.0, 0.2]])
    linear, angular = estimate_body_velocities(poses, [0.0, 1.0, 2.0])
    np.testing.assert_allclose(linear, [0.0, 1.0, np.cos(0.1)])
    np.testing.assert_allclose(angular, [0.0, 0.1, 0.1])


def test_yaw_rate_wraps_across_pi_boundary() -> None:
    poses = np.array([[0.0, 0.0, np.pi - 0.05], [0.0, 0.0, -np.pi + 0.05]])
    _, angular = estimate_body_velocities(poses, [0.0, 0.5])
    assert angular[1] == pytest.approx(0.2)


def test_controller_metrics_cover_pose_body_and_wheel_errors() -> None:
    metrics = compute_controller_metrics(telemetry_fixture())
    assert metrics["final_position_error_m"] == pytest.approx(0.2)
    assert metrics["final_yaw_error_rad"] == pytest.approx(0.05)
    assert metrics["position_rmse_m"] > 0.0
    assert metrics["yaw_rmse_rad"] > 0.0
    assert metrics["wheel_velocity_tracking_rmse_rad_s"] == pytest.approx(0.1)


def test_telemetry_rejects_nan_and_invalid_shape() -> None:
    telemetry = telemetry_fixture()
    invalid = telemetry.actual_wheel_velocities.copy()
    invalid[1, 2] = np.nan
    with pytest.raises(ValueError):
        ControllerTelemetry(
            telemetry.reference_trajectory,
            telemetry.actual_trajectory,
            telemetry.sim_times_s,
            telemetry.commanded_body,
            telemetry.target_wheel_velocities,
            invalid,
            telemetry.reference_indices,
            telemetry.wheel_names,
        )


def test_controller_run_serialization_and_overwrite_protection(tmp_path: Path) -> None:
    telemetry = telemetry_fixture()
    metrics = compute_controller_metrics(telemetry)
    destination = tmp_path / "straight"
    save_controller_run(
        destination,
        telemetry,
        metrics,
        {"scenario": "straight", "controller": "official_differential_open_loop"},
    )
    validation = validate_controller_run(destination)
    assert validation["valid"] is True
    assert validation["reference_shape"] == [3, 3]
    with pytest.raises(FileExistsError):
        save_controller_run(destination, telemetry, metrics, {})

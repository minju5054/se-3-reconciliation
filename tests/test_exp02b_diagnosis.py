from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest

from reconciliation.exp02b_diagnosis import (
    DiagnosticTelemetry,
    execution_tracking_metrics,
    interpretation_status,
    nearest_polyline_samples,
    save_diagnostic_output,
    spatial_reference_metrics,
    validate_diagnosis_summary,
    validate_diagnostic_output,
)


def telemetry_fixture() -> DiagnosticTelemetry:
    return DiagnosticTelemetry(
        actual_trajectory=np.array(
            [[0.0, 0.0, 0.0], [0.1, 0.0, 0.1], [0.2, 0.01, 0.2]], dtype=float
        ),
        sim_times_s=np.array([0.0, 0.1, 0.2]),
        commanded_body=np.array([[0.0, 0.0], [1.0, 1.0], [1.0, 1.0]]),
        target_wheel_velocities=np.array(
            [[0.0] * 4, [2.0, 2.0, 2.0, 2.0], [3.0, 3.0, 3.0, 3.0]]
        ),
        measured_wheel_velocities=np.array(
            [[0.0] * 4, [1.9, 1.9, 1.9, 1.9], [2.8, 2.8, 2.8, 2.8]]
        ),
    )


def summary_fixture() -> dict:
    return {
        "experiment": "EXP-02B GUI execution diagnosis",
        "source_trial_path": "/immutable/source",
        "source_provenance": {"sha256": "a" * 64},
        "case_id": "case_high_delta_omega",
        "entry_index": 3,
        "method": "raw_k",
        "saved_boundary_se2": [0.0, 0.0, 0.0],
        "reproduced_boundary_se2": [0.001, 0.0, 0.01],
        "pre_reset_error": {"translation_m": 0.001, "yaw_rad": 0.01},
        "exact_reset": {"occurred": True},
        "old_reference_spatial_metrics": {"old_reference_distance_rms_m": 0.1},
        "old_body_command_vs_measured_metrics": {"v_command_vs_measured_rmse_mps": 0.2},
        "old_wheel_target_vs_measured_metrics": {
            "wheel_target_vs_measured_rmse_rad_s": 0.1
        },
        "post_switch_controller_metrics": {"delta_v_abs_mps": 0.1},
        "post_switch_execution_metrics": {"v_command_vs_measured_rmse_mps": 0.1},
        "interpretation_status": {"labels": ["CONTROLLER_REFERENCE_DEVIATION_VISIBLE"]},
    }


def test_nearest_point_to_polyline_is_spatial_and_preserves_inputs() -> None:
    reference = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, np.pi / 2]])
    actual = np.array([[0.25, 0.2, 0.0], [1.2, 0.75, np.pi / 2]])
    before_reference = reference.copy()
    before_actual = actual.copy()
    samples = nearest_polyline_samples(reference, actual)
    np.testing.assert_allclose(samples["distance_m"], [0.2, 0.2])
    np.testing.assert_array_equal(samples["segment_index"], [0, 1])
    metrics = spatial_reference_metrics(reference, actual)
    assert metrics["old_reference_distance_rms_m"] == pytest.approx(0.2)
    assert "not time-aligned" in metrics["metric_semantics"]
    np.testing.assert_array_equal(reference, before_reference)
    np.testing.assert_array_equal(actual, before_actual)


def test_nearest_segment_yaw_interpolation_wraps_at_pi() -> None:
    reference = np.array(
        [[0.0, 0.0, np.pi - 0.1], [1.0, 0.0, -np.pi + 0.1]], dtype=float
    )
    actual = np.array([[0.5, 0.0, -np.pi]], dtype=float)
    # validate_se2_trajectory requires at least two actual poses.
    actual = np.vstack((actual, actual))
    samples = nearest_polyline_samples(reference, actual)
    assert abs(samples["yaw_error_rad"][0]) == pytest.approx(0.0, abs=1e-12)


def test_body_and_per_wheel_command_tracking_rmse() -> None:
    metrics = execution_tracking_metrics(telemetry_fixture())
    assert metrics["v_command_vs_measured_rmse_mps"] < 0.01
    assert metrics["omega_command_vs_measured_rmse_rps"] == pytest.approx(0.0)
    assert metrics["wheel_target_vs_measured_rmse_rad_s"] == pytest.approx(
        np.sqrt((0.1**2 + 0.2**2) / 2)
    )
    assert set(metrics["wheel_target_vs_measured_rmse_by_corner_rad_s"]) == {
        "front_left",
        "front_right",
        "rear_left",
        "rear_right",
    }


def test_telemetry_and_summary_reject_nan_and_missing_fields() -> None:
    telemetry = telemetry_fixture()
    bad_wheels = telemetry.measured_wheel_velocities.copy()
    bad_wheels[1, 0] = np.nan
    with pytest.raises(ValueError, match="finite"):
        DiagnosticTelemetry(
            telemetry.actual_trajectory,
            telemetry.sim_times_s,
            telemetry.commanded_body,
            telemetry.target_wheel_velocities,
            bad_wheels,
        )
    invalid = summary_fixture()
    invalid["pre_reset_error"]["translation_m"] = float("inf")
    with pytest.raises(ValueError, match="non-finite"):
        validate_diagnosis_summary(invalid)
    missing = summary_fixture()
    del missing["exact_reset"]
    with pytest.raises(ValueError, match="missing fields"):
        validate_diagnosis_summary(missing)


def test_interpretation_supports_multiple_noncausal_labels() -> None:
    result = interpretation_status(
        old_spatial={"old_reference_distance_rms_m": 0.2},
        old_execution={
            "v_command_vs_measured_rmse_mps": 0.2,
            "omega_command_vs_measured_rmse_rps": 0.3,
            "wheel_target_vs_measured_rmse_rad_s": 0.4,
        },
        reset_translation_m=0.01,
        reset_yaw_rad=0.02,
        thresholds={
            "reference_rms_m": 0.05,
            "body_v_rmse_mps": 0.05,
            "body_omega_rmse_rps": 0.1,
            "wheel_rmse_rad_s": 0.25,
            "reset_translation_m": 0.001,
            "reset_yaw_rad": 0.001,
        },
    )
    assert result["labels"] == [
        "CONTROLLER_REFERENCE_DEVIATION_VISIBLE",
        "BODY_EXECUTION_MISMATCH_VISIBLE",
        "WHEEL_TRACKING_MISMATCH_VISIBLE",
        "REPLAY_RESET_ARTIFACT_RELEVANT",
    ]
    assert "none identifies a causal root cause" in result["semantics"]


def test_diagnostic_output_round_trip_and_overwrite_protection(tmp_path: Path) -> None:
    telemetry = telemetry_fixture()
    destination = tmp_path / "diagnosis"
    progress = [(0, 1, False), (0, 1, False), (1, 2, True)]
    save_diagnostic_output(
        destination,
        metadata={"diagnostic_only": True},
        summary=summary_fixture(),
        old_telemetry=telemetry,
        old_command_indices=[-1, 0, 1],
        post_telemetry=telemetry,
        post_command_indices=[-1, 0, 1],
        post_progress=progress,
    )
    validation = validate_diagnostic_output(destination)
    assert validation["valid"] is True
    assert validation["old_sample_count"] == 3
    before = deepcopy(summary_fixture())
    with pytest.raises(FileExistsError):
        save_diagnostic_output(
            destination,
            metadata={},
            summary=before,
            old_telemetry=telemetry,
            old_command_indices=[-1, 0, 1],
            post_telemetry=telemetry,
            post_command_indices=[-1, 0, 1],
            post_progress=progress,
        )

import json

import numpy as np
import pytest

from reconciliation.stage0_jackal import (
    SAMPLE_COLUMNS,
    Stage0Recording,
    save_stage0_run,
    validate_stage0_output,
)


def recording() -> Stage0Recording:
    reference = np.array([[0, 0, 0], [1, 0, 0.1], [2, 0.1, 0.2]], dtype=float)
    actual = np.array([[0, 0, 0], [0.9, 0.0, 0.08], [1.8, 0.12, 0.18]], dtype=float)
    return Stage0Recording(reference, actual, [0.0, 0.1, 0.2])


def metadata(run_id: str = "synthetic-stage0-test") -> dict:
    return {
        "stage": "stage0_jackal_trajectory_smoke_test",
        "run_id": run_id,
        "creation_time": "2026-09-02T12:00:00+09:00",
        "git_commit_sha": "deadbeef",
        "isaac_sim_version": "test-only",
        "robot_asset": {"relative_path": "Clearpath/Jackal/jackal.usd"},
        "robot_prim_path": "/World/Jackal",
        "actual_dof_names": ["runtime-name-0", "runtime-name-1", "runtime-name-2", "runtime-name-3"],
        "physics_dt": 0.01,
        "sample_dt": 0.1,
        "trajectory_convention": "test fixture",
        "pose_frame": "world",
        "yaw_convention": "world +Z, radians, [-pi, pi)",
        "motion_profile": [],
        "wheel_parameters": {"radius_m": 0.1, "separation_m": 0.4},
        "visualization_height_m": 0.05,
    }


def test_trajectory_serialization_and_validation(tmp_path) -> None:
    destination = tmp_path / "run"
    save_stage0_run(destination, recording(), metadata())
    result = validate_stage0_output(destination)
    assert result["valid"] is True
    assert result["reference_shape"] == [3, 3]
    assert result["actual_shape"] == [3, 3]
    np.testing.assert_array_equal(
        np.load(destination / "reference_trajectory.npy", allow_pickle=False),
        recording().reference_trajectory,
    )
    np.testing.assert_array_equal(
        np.load(destination / "actual_trajectory.npy", allow_pickle=False),
        recording().actual_trajectory,
    )
    header = (destination / "samples.csv").read_text(encoding="utf-8").splitlines()[0]
    assert tuple(header.split(",")) == SAMPLE_COLUMNS
    assert json.loads((destination / "metadata.json").read_text(encoding="utf-8"))["run_id"] == "synthetic-stage0-test"


def test_output_overwrite_protection(tmp_path) -> None:
    destination = tmp_path / "same-run"
    save_stage0_run(destination, recording(), metadata())
    with pytest.raises(FileExistsError):
        save_stage0_run(destination, recording(), metadata())


def test_mismatched_lengths_rejected_without_interpolation() -> None:
    with pytest.raises(ValueError, match="does not interpolate"):
        Stage0Recording(np.zeros((3, 3)), np.zeros((2, 3)), [0, 1, 2])


def test_nan_actual_trajectory_rejected() -> None:
    actual = np.zeros((3, 3))
    actual[1, 0] = np.nan
    with pytest.raises(ValueError, match="finite"):
        Stage0Recording(np.zeros((3, 3)), actual, [0, 1, 2])


def test_validator_detects_csv_array_disagreement(tmp_path) -> None:
    destination = tmp_path / "corrupt"
    save_stage0_run(destination, recording(), metadata())
    lines = (destination / "samples.csv").read_text(encoding="utf-8").splitlines()
    cells = lines[1].split(",")
    cells[2] = "999.0"
    lines[1] = ",".join(cells)
    (destination / "samples.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="differ"):
        validate_stage0_output(destination)


def test_validator_rejects_failed_recorded_smoke_checks(tmp_path) -> None:
    destination = tmp_path / "failed-check"
    failed_metadata = metadata()
    failed_metadata["smoke_success_checks"] = {"passed": False}
    save_stage0_run(destination, recording(), failed_metadata)
    with pytest.raises(ValueError, match="did not pass"):
        validate_stage0_output(destination)

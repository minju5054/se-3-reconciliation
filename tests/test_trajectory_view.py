from __future__ import annotations

from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from reconciliation.trajectory_view import (
    export_trajectory_csv,
    format_trajectory_report,
    load_trajectory_npy,
    summarize_trajectory,
)


def test_load_and_summarize_trajectory_npy(tmp_path: Path) -> None:
    trajectory = np.array([[0.0, 0.0, 3.0], [3.0, 4.0, -3.0]])
    source = tmp_path / "trajectory.npy"
    np.save(source, trajectory)

    loaded = load_trajectory_npy(source)
    summary = summarize_trajectory(loaded)

    np.testing.assert_array_equal(loaded, trajectory)
    assert summary.sample_count == 2
    assert summary.endpoint_displacement_m == pytest.approx(5.0)
    assert summary.path_length_m == pytest.approx(5.0)
    assert summary.wrapped_yaw_change_rad == pytest.approx(0.28318530717958623)


@pytest.mark.parametrize(
    "value",
    [np.zeros((3, 2)), np.array([[0.0, np.nan, 0.0]])],
)
def test_load_rejects_noncanonical_trajectory(tmp_path: Path, value: np.ndarray) -> None:
    source = tmp_path / "invalid.npy"
    np.save(source, value)
    with pytest.raises(ValueError):
        load_trajectory_npy(source)


def test_report_marks_omitted_rows() -> None:
    trajectory = np.column_stack((np.arange(6), np.zeros(6), np.zeros(6)))
    report = format_trajectory_report("test.npy", trajectory, rows=2)

    assert "shape: (6, 3)" in report
    assert "       0:" in report
    assert "  ..." in report
    assert "       5:" in report


def test_csv_export_preserves_rows_and_protects_existing_file(tmp_path: Path) -> None:
    trajectory = np.array([[1.0, 2.0, 0.1], [3.0, 4.0, 0.2]])
    destination = export_trajectory_csv(tmp_path / "trajectory.csv", trajectory)

    assert destination.read_text(encoding="utf-8").splitlines() == [
        "x,y,yaw",
        "1.0,2.0,0.1",
        "3.0,4.0,0.2",
    ]
    with pytest.raises(FileExistsError):
        export_trajectory_csv(destination, trajectory)


def test_cli_prints_saves_and_exports_without_gui(tmp_path: Path) -> None:
    repository = Path(__file__).resolve().parents[1]
    source = tmp_path / "actual_trajectory.npy"
    plot = tmp_path / "trajectory.png"
    csv_dir = tmp_path / "csv"
    np.save(source, np.array([[0.0, 0.0, 0.0], [1.0, 0.2, 0.1]]))

    result = subprocess.run(
        [
            sys.executable,
            str(repository / "scripts/view_trajectory_npy.py"),
            str(source),
            "--no-show",
            "--save",
            str(plot),
            "--csv-dir",
            str(csv_dir),
        ],
        cwd=repository,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "shape: (2, 3)" in result.stdout
    assert plot.stat().st_size > 0
    assert (csv_dir / "actual_trajectory.csv").is_file()

import csv
from pathlib import Path

import numpy as np
import pytest

from reconciliation.exp02b_diagnosis import DiagnosticTelemetry, OLD_TELEMETRY_COLUMNS, telemetry_csv_rows
from reconciliation.exp02b_failure_demo import (
    checked_file, read_old_telemetry, saved_sample_index, verify_metric_subset,
)
from reconciliation.online_switch import sha256_file


def test_saved_time_does_not_display_future_samples_or_interpolate():
    times = np.array([0.0, 0.1, 0.3, 0.4])
    # 4 wall seconds represent 0.4 saved seconds, including unequal sampling intervals.
    assert [saved_sample_index(times, t, 4) for t in (0, 0.99, 1, 2.9, 3, 4, 8)] == [0, 0, 1, 1, 2, 3, 3]
    # Holding elapsed time is a pause; restarting chooses the exact original first pose.
    assert saved_sample_index(times, 2, 4) == saved_sample_index(times, 2, 4) == 1
    assert saved_sample_index(times, 0, 4) == 0


@pytest.mark.parametrize("times,elapsed,duration", [
    ([0, 0], 0, 4), ([1, 2], 0, 4), ([0, np.nan], 0, 4),
    ([0, 1], -1, 4), ([0, 1], np.inf, 4), ([0, 1], 0, 0), ([0, 1], 0, np.nan),
])
def test_invalid_playback_clocks_rejected(times, elapsed, duration):
    with pytest.raises(ValueError):
        saved_sample_index(np.array(times), elapsed, duration)


def telemetry_csv(tmp_path: Path):
    # Synthetic test fixture only: a wrap-crossing turn over unequal intervals.
    poses = np.array([[0, 0, 3.13], [0, 0, -3.13], [0, 0, -3.10]])
    t = DiagnosticTelemetry(poses, np.array([0, 0.1, 0.3]), np.array([[0, 0], [0, 0.2], [0, 0.15]]),
                            np.zeros((3, 4)), np.zeros((3, 4)))
    columns, rows = telemetry_csv_rows(t, command_indices=[-1, 0, 1])
    path = tmp_path / "old.csv"
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(rows)
    return path, poses


def test_saved_measurement_uses_ending_interval_and_preserves_source(tmp_path):
    path, poses = telemetry_csv(tmp_path)
    before = path.read_bytes()
    telemetry, measured = read_old_telemetry(path, poses)
    assert measured[0, 1] == 0
    assert measured[1, 1] == pytest.approx((2*np.pi-6.26)/0.1)
    assert measured[2, 1] == pytest.approx(0.03/0.2)
    assert not telemetry.actual_trajectory.flags.writeable
    assert not measured.flags.writeable
    assert path.read_bytes() == before


def test_mismatched_pose_or_measured_telemetry_is_rejected(tmp_path):
    path, poses = telemetry_csv(tmp_path)
    changed = poses.copy()
    changed[1, 0] += 0.01
    with pytest.raises(ValueError, match="CSV poses"):
        read_old_telemetry(path, changed)
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    rows[1]["measured_body_omega_rps"] = "0.5"
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=OLD_TELEMETRY_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    with pytest.raises(ValueError, match="ending-interval"):
        read_old_telemetry(path, poses)


def test_archived_hash_and_summary_tampering_are_rejected(tmp_path):
    path = tmp_path / "evidence"
    path.write_bytes(b"immutable fixture")
    digest = sha256_file(path)
    assert checked_file(path, digest) == path
    path.write_bytes(b"changed fixture")
    with pytest.raises(ValueError, match="hash mismatch"):
        checked_file(path, digest)
    with pytest.raises(ValueError, match="does not reproduce"):
        verify_metric_subset({"omega_mean": 0.139}, {"omega_mean": 0.755})

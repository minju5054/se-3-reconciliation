from pathlib import Path

import numpy as np
import pytest

from reconciliation.exp02b import CASE_IDS
from reconciliation.exp02b_calibrated_reeval import (
    ENTRY_INDICES,
    command_level_metrics,
    compare_method_pairs,
    final_interpretation,
    first_command_invariant,
    historical_nominal_post_metrics,
    measured_transition_without_reset_crossing,
    verify_file_hash,
    write_json_exclusive,
)
from reconciliation.online_switch import sha256_file


def test_frozen_candidate_hash_verification_detects_changed_bytes(tmp_path: Path) -> None:
    candidate = tmp_path / "candidate.npy"
    np.save(candidate, np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]))
    digest = sha256_file(candidate)
    assert verify_file_hash(candidate, digest, "candidate") == digest
    np.save(candidate, np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]]))
    with pytest.raises(ValueError, match="frozen provenance mismatch"):
        verify_file_hash(candidate, digest, "candidate")


def test_historical_metric_extraction_is_spatial_and_preserves_sources() -> None:
    candidate = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    actual = np.array(
        [[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.2, 0.0, 0.0], [0.3, 0.0, 0.0]]
    )
    desired = np.array([[1.0, 0.0], [1.0, 0.0], [1.0, 0.0]])
    before = (candidate.copy(), actual.copy(), desired.copy())
    result = historical_nominal_post_metrics(
        candidate=candidate, actual=actual, desired=desired, control_dt_s=0.1
    )
    assert result["interval_count"] == 3
    assert result["desired_v_vs_measured_v_rmse_mps"] == pytest.approx(0.0)
    assert result["position_rmse_m"] == pytest.approx(0.0)
    for value, expected in zip((candidate, actual, desired), before, strict=True):
        np.testing.assert_array_equal(value, expected)


def test_first_desired_command_invariant_has_strict_pass_and_fail() -> None:
    passed = first_command_invariant([0.2, -0.4], [0.2, -0.4], tolerance=1e-12)
    assert passed["passed"] is True
    failed = first_command_invariant(
        [0.2, -0.4], [0.2, -0.4 + 2e-12], tolerance=1e-12
    )
    assert failed["passed"] is False
    with pytest.raises(ValueError, match="finite"):
        first_command_invariant([0.2, np.nan], [0.2, 0.0], tolerance=1e-12)


def test_desired_executed_and_measured_metrics_remain_separate() -> None:
    metrics = command_level_metrics(
        old_desired=[0.1, 0.2],
        post_desired=[[0.2, 0.4], [0.3, 0.3], [0.4, 0.2]],
        post_executed=[[0.2, 0.8], [0.3, 0.6], [0.4, 0.4]],
        old_measured=[0.05, 0.1],
        post_measured=[[0.1, 0.15], [0.2, 0.2], [0.3, 0.25]],
        control_dt_s=0.1,
        window_size=3,
    )
    assert metrics["desired"]["delta_omega_signed"] == pytest.approx(0.2)
    assert metrics["executed"]["delta_omega_signed"] == pytest.approx(0.6)
    assert metrics["measured"]["delta_omega_signed"] == pytest.approx(0.05)
    assert metrics["desired_to_executed"]["first_correction_v_omega"] == pytest.approx(
        [0.0, 0.4]
    )


def test_measured_transition_never_differentiates_across_reset() -> None:
    result = measured_transition_without_reset_crossing(
        old_interval_start=[100.0, 100.0, 0.0],
        old_interval_end=[100.1, 100.0, 0.1],
        old_interval_duration_s=0.1,
        post_control_poses=[
            [0.0, 0.0, 0.0],
            [0.02, 0.0, 0.01],
            [0.04, 0.0, 0.02],
            [0.06, 0.0, 0.03],
        ],
        control_dt_s=0.1,
    )
    # A reset-crossing derivative would be roughly -1000 m/s. The actual result
    # compares the separately measured 1.0 m/s OLD interval with 0.2 m/s post-reset.
    assert result["pre_switch_v_omega"][0] == pytest.approx(1.0)
    assert result["post_switch_first_v_omega"][0] == pytest.approx(0.2)
    assert result["delta_v_signed"] == pytest.approx(-0.8)
    assert "no derivative spans" in result["interval_semantics"]


def _pairwise_rows() -> list[dict]:
    rows = []
    for case in CASE_IDS:
        for k in ENTRY_INDICES:
            rows.extend(
                [
                    {"case_id": case, "entry_index": k, "method": "raw_k", "metric": 2.0},
                    {"case_id": case, "entry_index": k, "method": "rigid", "metric": 1.5},
                    {"case_id": case, "entry_index": k, "method": "graph", "metric": 1.0},
                ]
            )
    return rows


def test_branch_pairwise_comparison_reports_all_nine_pairs() -> None:
    result = compare_method_pairs(
        _pairwise_rows(),
        method="graph",
        baseline="raw_k",
        metric_paths=(("metric",),),
        tie_tolerance=1e-12,
    )
    assert result["pair_count"] == 9
    assert result["metric_comparison_counts"] == {"better": 9, "worse": 0, "tie": 0}


def test_final_decision_logic_prioritizes_invariant_and_consistency() -> None:
    failed = final_interpretation(
        invariant_passed=False,
        graph_vs_raw_consistent=True,
        graph_vs_rigid_consistent=True,
        physical_tracking_improved=True,
        physical_method_ranking_materially_changed=True,
    )
    assert failed["primary_status"] == "PROTOCOL_INVARIANT_FAILURE"
    unchanged = final_interpretation(
        invariant_passed=True,
        graph_vs_raw_consistent=False,
        graph_vs_rigid_consistent=False,
        physical_tracking_improved=True,
        physical_method_ranking_materially_changed=False,
    )
    assert unchanged["primary_status"] == "EXP02B_FORMULATION_CONCLUSION_UNCHANGED"
    assert "CALIBRATION_IMPROVES_PHYSICAL_EXECUTION_BUT_NOT_FORMULATION" in unchanged[
        "secondary_status"
    ]


def test_output_is_exclusive_and_non_finite_json_is_rejected(tmp_path: Path) -> None:
    output = tmp_path / "result.json"
    write_json_exclusive(output, {"value": 1.0})
    with pytest.raises(FileExistsError):
        write_json_exclusive(output, {"value": 2.0})
    with pytest.raises(ValueError, match="non-finite"):
        write_json_exclusive(tmp_path / "bad.json", {"value": float("inf")})


def test_command_metrics_reject_non_finite_input() -> None:
    with pytest.raises(ValueError, match="finite"):
        command_level_metrics(
            old_desired=[0.0, 0.0],
            post_desired=[[0.0, 0.0], [0.0, np.nan], [0.0, 0.0]],
            post_executed=np.zeros((3, 2)),
            old_measured=[0.0, 0.0],
            post_measured=np.zeros((3, 2)),
            control_dt_s=0.1,
            window_size=3,
        )

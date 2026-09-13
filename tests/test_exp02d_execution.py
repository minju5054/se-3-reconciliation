"""Synthetic bookkeeping fixtures; not physical or research evidence."""

from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from reconciliation.exp02d_execution import safe_run_directory, summarize_execution


@pytest.fixture
def criteria():
    return yaml.safe_load(Path("configs/stage0_closed_loop_execution_validation.yaml").read_text())["acceptance_criteria"]


def trials():
    metrics = {"goal_reached": True, "position_rmse_m": 0.01, "yaw_rmse_rad": 0.01,
               "final_position_error_m": 0.01, "final_yaw_error_rad": 0.01,
               "numerically_stable": True, "active_saturation_fraction": 0.0,
               "progress_monotonic": True, "progress_completion_ratio": 1.0}
    return [{"case": case, "method": method, "repetition": rep, "metrics": deepcopy(metrics)}
            for case in ["S1", "S2", "F1"] for method in ["M0_RAW", "M3_LOOKAHEAD"] for rep in range(3)]


def summarize(rows, criteria):
    return summarize_execution(rows, ["S1", "S2", "F1"], ["M0_RAW", "M3_LOOKAHEAD"], criteria)


def test_old_failure_label_does_not_force_physical_failure(criteria):
    result = summarize(trials(), criteria)
    assert result["outcomes"]["F1"]["M3_LOOKAHEAD"]["physical_tracking_status"] == "PASS"
    assert result["gui_selection"] == {"success": "S2", "failure": None}


def test_goal_reached_is_not_sufficient_for_tracking_pass(criteria):
    rows = trials()
    for row in rows:
        if row["case"] == "F1" and row["method"] == "M3_LOOKAHEAD":
            row["metrics"]["yaw_rmse_rad"] = 0.2
    result = summarize(rows, criteria)
    assert result["gui_selection"] == {"success": "S2", "failure": "F1"}
    assert result["outcomes"]["F1"]["M3_LOOKAHEAD"]["failed_checks"] == ["yaw_rmse"]


def test_no_success_is_reported_honestly(criteria):
    rows = trials()
    for row in rows:
        row["metrics"]["goal_reached"] = False
    assert summarize(rows, criteria)["gui_selection"]["success"] is None


@pytest.mark.parametrize("mutation", ["missing", "duplicate", "unexpected"])
def test_reject_incomplete_or_duplicate_comparison(criteria, mutation):
    rows = trials()
    if mutation == "missing":
        rows.pop()
    elif mutation == "duplicate":
        rows.append(deepcopy(rows[0]))
    else:
        rows[0]["case"] = "invented"
    with pytest.raises(ValueError, match="missing, duplicate or unexpected"):
        summarize(rows, criteria)


@pytest.mark.parametrize("name", ["", ".", "..", "../escape", "/absolute", "nested/run"])
def test_run_id_cannot_escape_output_root(tmp_path, name):
    with pytest.raises(ValueError):
        safe_run_directory(tmp_path, name)

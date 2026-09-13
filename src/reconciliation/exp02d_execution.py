"""Post-hoc physical execution bookkeeping, downstream of frozen EXP-02D.

Original S/F identities describe command scores. Physical outcomes are computed
independently using the existing Stage 0-E gates; labels are never forced.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping, Sequence

from reconciliation.closed_loop_execution_validation import evaluate_scenario


def summarize_execution(trials: Sequence[Mapping[str, Any]], cases, methods, criteria):
    expected = {(case, method, repetition) for case in cases for method in methods
                for repetition in range(int(criteria["repetitions"]))}
    actual = [(row["case"], row["method"], row["repetition"]) for row in trials]
    if len(actual) != len(set(actual)) or set(actual) != expected:
        raise ValueError("physical comparison has missing, duplicate or unexpected trials")
    outcomes = {}
    for case in cases:
        outcomes[case] = {}
        for method in methods:
            rows = [row["metrics"] for row in trials
                    if row["case"] == case and row["method"] == method]
            result = evaluate_scenario(rows, criteria)
            outcomes[case][method] = {
                **result,
                "physical_tracking_status": "PASS" if result["passed"] else "FAIL",
                "failed_checks": [key for key, passed in result["checks"].items() if not passed],
            }
    successful = [case for case in cases if outcomes[case]["M3_LOOKAHEAD"]["passed"]]
    failed = [case for case in cases if not outcomes[case]["M3_LOOKAHEAD"]["passed"]]
    # Prefer the pre-existing challenging example if it actually passes; never
    # infer a physical outcome from its historical S2/F1 name.
    success = next((case for case in ("S2", "S1", "F1") if case in successful), None)
    failure = next((case for case in ("F1", "S2", "S1") if case in failed), None)
    return {"outcomes": outcomes,
            "gui_selection": {"success": success, "failure": failure},
            "selection_rule": "prefer S2/S1/F1 among actual M3 passes; F1/S2/S1 among actual failures; absent class remains null",
            "claim_scope": "reset-state Hospital simulation tracking only; historical command-score labels are separate"}


def safe_run_directory(root: Path, run_id: str) -> Path:
    if not run_id or run_id in (".", "..") or Path(run_id).name != run_id:
        raise ValueError("run ID must be a single directory name")
    return root / run_id

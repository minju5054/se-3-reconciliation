#!/usr/bin/env python3
"""Read-only audit of the complete curved-path exclusive-success search."""

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "scripts")]
import numpy as np
from reconciliation.exp02d_turning_search import FrozenArchive, path_turning, actual_is_turning, METHODS
from reconciliation.closed_loop_execution_validation import evaluate_scenario, validate_closed_loop_trial
from reconciliation.online_switch import sha256_file
from summarize_exp02d_physical_execution import validate_run


def audit(run):
    run = Path(run).resolve()
    protocol = json.loads((run / "protocol.json").read_text())
    summary = json.loads((run / "scan_summary.json").read_text())
    selection = json.loads((run / "selection.json").read_text())
    archive = FrozenArchive(protocol["source_run"])
    config = protocol["config"]
    identifiers = [row["identifier"] for row in summary["cases"]]
    if identifiers != protocol["scan_order"] or len(identifiers) != len(set(identifiers)):
        raise ValueError("search order/grid is incomplete or duplicated")
    matches, goal_only = [], []
    method_pass = {method: 0 for method in METHODS}
    method_goal = {method: 0 for method in METHODS}
    for row in summary["cases"]:
        evidence = archive.load(row["identifier"])
        expected = protocol["input_hashes"][row["identifier"]]
        if evidence.source_sha256 != expected["source"] or evidence.result_sha256 != expected["result"]:
            raise ValueError("frozen search input changed")
        if set(row["trials"]) != set(METHODS):
            raise ValueError("case has missing methods")
        for method, trial in row["trials"].items():
            directory = Path(trial["directory"])
            if not directory.is_relative_to(run):
                raise ValueError("trial outside search")
            checked = validate_closed_loop_trial(directory)
            if checked["metrics"] != trial["metrics"]:
                raise ValueError("trial metrics differ from saved summary")
            metadata = json.loads((directory / "metadata.json").read_text())
            if metadata["protocol_sha256"] != sha256_file(run / "protocol.json"):
                raise ValueError("trial protocol changed")
            reference = Path(metadata["reference_path"])
            if metadata["reference_sha256"] != sha256_file(reference) or not np.array_equal(np.load(reference), evidence.method_candidates[method]):
                raise ValueError("trial reference differs from frozen candidate")
            for name, digest in trial["raw_sha256"].items():
                if sha256_file(directory / name) != digest:
                    raise ValueError("trial raw bytes changed")
            geometry = path_turning(np.load(directory / "raw/actual_trajectory.npy"), config["tangent_minimum_segment_m"])
            if geometry != trial["actual_geometry"]:
                raise ValueError("measured turning does not reproduce")
            outcome = evaluate_scenario([trial["metrics"]], protocol["scan_criteria"])
            if outcome != row["outcomes"][method]:
                raise ValueError("screening outcome does not reproduce")
            method_pass[method] += outcome["passed"]
            method_goal[method] += trial["metrics"]["goal_reached"]
        outcomes = row["outcomes"]
        matched = (outcomes["M3_LOOKAHEAD"]["passed"] and not outcomes["M0_RAW"]["passed"]
                   and not outcomes["M1_HISTORICAL_M4"]["passed"]
                   and actual_is_turning(row["trials"]["M3_LOOKAHEAD"]["actual_geometry"], config))
        if matched != row["provisional_match"]:
            raise ValueError("exclusive-success classification changed")
        if matched:
            matches.append(row["identifier"])
        if (row["trials"]["M3_LOOKAHEAD"]["metrics"]["goal_reached"] and
                not any(row["trials"][m]["metrics"]["goal_reached"] for m in METHODS[:2])):
            goal_only.append(row["identifier"])
    if selection["scan_trial_count"] != len(identifiers) * len(METHODS) or selection["provisional_match_count"] != len(matches):
        raise ValueError("search counts changed")
    if selection["selected"] is not None:
        validate_run(Path(selection["selected"]["confirmation"]))
        if selection["selected"]["identifier"] not in matches:
            raise ValueError("selected case was not an eligible success")
    return {"validated_scan_trials": len(identifiers) * len(METHODS), "eligible_cases": len(identifiers),
        "method_pass_counts": method_pass, "method_goal_counts": method_goal,
        "exclusive_success_candidates": matches, "M3_only_goal_cases_before_quality_gates": goal_only,
        "selected": None if selection["selected"] is None else selection["selected"]["identifier"],
        "population_claim": "none; post-hoc examples from development data"}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    print(json.dumps(audit(args.run), indent=2))

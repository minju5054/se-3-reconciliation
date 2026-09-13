#!/usr/bin/env python3
"""Search all frozen curved candidates, then confirm an exclusive M3 success."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
import numpy as np
import yaml
from reconciliation.exp02d_turning_search import FrozenArchive, path_turning, reference_is_turning, actual_is_turning, METHODS
from reconciliation.exp02d_gui import load_exp02d_gui_case
from reconciliation.exp02d_execution import safe_run_directory, summarize_execution
from reconciliation.closed_loop_execution_validation import (
    evaluate_scenario, compute_closed_loop_metrics, load_frozen_stage0d_candidate,
    save_closed_loop_trial, validate_closed_loop_trial,
)
from reconciliation.online_switch import sha256_file
from exp02d_physical_execution import resolve, utc, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/exp02d_turning_search.yaml")
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    archive = FrozenArchive(resolve(config["source_run"]))
    eligible, inventory = [], []
    for identifier, record in archive.corpus.items():
        paths, hashes = archive.candidates(identifier)
        geometry = path_turning(paths["M3_LOOKAHEAD"], config["tangent_minimum_segment_m"])
        included = reference_is_turning(geometry, config)
        row = {"identifier": identifier, "geometry": geometry, "included": included}
        inventory.append(row)
        if included:
            a = record["analysis"]
            eligible.append((min(a["j_cmd_raw"], a["j_cmd_m1"]) - a["j_cmd_m3"], identifier))
    eligible.sort(key=lambda row: (-row[0], row[1]))
    evidence = {identifier: archive.load(identifier) for _, identifier in eligible}
    first = load_exp02d_gui_case(archive.run, config["scene_initialization_case"])
    source = yaml.safe_load((first.active_old.run / "config_snapshot.yaml").read_text())
    for item in evidence.values():
        other = yaml.safe_load((item.active_old.run / "config_snapshot.yaml").read_text())
        if any(source[k] != other[k] for k in ("simulation", "follower", "robot", "environment", "stage0d_candidate_provenance")):
            raise ValueError("scan contexts have different physics/controller systems")
    criteria = yaml.safe_load(resolve(config["stage0e_config"]).read_text())["acceptance_criteria"]
    if config["confirmation_repetitions"] != criteria["repetitions"]:
        raise ValueError("retain frozen repetition criterion")
    scan_criteria = {**criteria, "repetitions": 1, "minimum_goal_successes": 1}
    candidate = load_frozen_stage0d_candidate(resolve(source["paths"]["stage0d_run"]),
        source["stage0d_candidate_provenance"], repository_root=ROOT)
    output = safe_run_directory(resolve(config["output_root"]), args.run_id)
    output.mkdir(parents=True, exist_ok=False)
    files = [Path(__file__), ROOT / "src/reconciliation/exp02d_turning_search.py",
        ROOT / "scripts/isaac/exp02d_execution_runtime.py", ROOT / "scripts/isaac/closed_loop_execution_runtime.py",
        ROOT / "src/reconciliation/exp02d_execution.py", ROOT / "src/reconciliation/controllers/trajectory_follower.py",
        ROOT / "src/reconciliation/controllers/jackal_execution_controller.py"]
    code_hashes = {str(p.relative_to(ROOT)): sha256_file(p) for p in files}
    protocol = {"started_utc": utc(), "config": config, "config_sha256": sha256_file(args.config),
        "git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "source_run": str(archive.run), "source_system": source, "candidate": candidate,
        "acceptance_criteria": criteria, "scan_criteria": scan_criteria, "code_sha256": code_hashes,
        "scene_initialization_pose_world_se2": first.active_old.boundary_pose_world_se2.tolist(),
        "coordinate_frame": "Isaac world XY metres, +Z CCW yaw radians; identity transform; no resampling or B insertion",
        "timing": "1/60 s physics; 0.1 s control; row zero after 1 s settling; later row command applies to ending interval; reset at B without original velocity/PI history",
        "input_hashes": {key: {"source": value.source_sha256, "result": value.result_sha256} for key, value in evidence.items()},
        "scan_order": [identifier for _, identifier in eligible], "inventory": inventory,
        "selection_scope": "post-hoc development search; no population success-rate claim"}
    write_json(output / "protocol.json", protocol)
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        from exp02d_execution_runtime import HospitalExecutionRuntime, loop_configs
        from closed_loop_execution_runtime import run_closed_loop
        runtime = HospitalExecutionRuntime(app, source, candidate, np.asarray(protocol["scene_initialization_pose_world_se2"]))
        loop, follower = loop_configs(source, config["maximum_duration_s"])

        def execute(item, method, repetition, destination, protocol_path):
            started = utc()
            telemetry = run_closed_loop(runtime, loop, follower, item.method_candidates[method],
                item.active_old.boundary_pose_world_se2, "calibrated")
            runtime.world.pause()
            metrics = compute_closed_loop_metrics(telemetry)
            geometry = path_turning(telemetry.actual_trajectory, config["tangent_minimum_segment_m"])
            reference = destination.parent / "reference.npy"
            if not reference.exists():
                reference.parent.mkdir(parents=True, exist_ok=True)
                with reference.open("xb") as stream:
                    np.save(stream, item.method_candidates[method])
            hashes = save_closed_loop_trial(destination, telemetry, metrics, {
                "stage": config["experiment"], "evaluation_semantics": "reference-matched closed-loop evaluation",
                "mode": "calibrated", "case": item.case, "method": method, "repetition": repetition,
                "scenario": f"{item.case}_{method}", "corpus_transition_id": item.corpus_transition_id,
                "started_utc": started, "completed_utc": utc(), "protocol_path": str(protocol_path),
                "protocol_sha256": sha256_file(protocol_path), "reference_path": str(reference),
                "reference_sha256": sha256_file(reference), "coordinate_frame": protocol["coordinate_frame"],
                "timing": protocol["timing"], "requested_initial_pose_world_se2": item.active_old.boundary_pose_world_se2.tolist(),
                "settled_initial_pose_world_se2": telemetry.actual_trajectory[0].tolist(),
                "source_observation_sim_time_s": item.active_old.observation_sim_time_s,
                "source_readiness_sim_time_s": item.active_old.model_ready_sim_time_s,
                "source_switch_sim_time_s": item.active_old.switch_sim_time_s})
            validate_closed_loop_trial(destination)
            return {"case": item.case, "method": method, "repetition": repetition,
                "directory": str(destination), "metrics": metrics, "raw_sha256": hashes, "actual_geometry": geometry}

        scan = []
        matches = []
        for index, (_, identifier) in enumerate(eligible):
            item = evidence[identifier]
            trials, outcomes = {}, {}
            for method in METHODS:
                row = execute(item, method, 0, output / "scan" / item.case / method / "repetition_00", output / "protocol.json")
                trials[method] = row
                outcomes[method] = evaluate_scenario([row["metrics"]], scan_criteria)
            passes = {method: bool(outcomes[method]["passed"]) for method in METHODS}
            match = (passes["M3_LOOKAHEAD"] and not passes["M0_RAW"] and not passes["M1_HISTORICAL_M4"]
                     and actual_is_turning(trials["M3_LOOKAHEAD"]["actual_geometry"], config))
            record = {"identifier": identifier, "trials": trials, "outcomes": outcomes, "provisional_match": match}
            write_json(output / "scan" / item.case / "result.json", record)
            scan.append(record)
            if match:
                missing_goals = sum(not trials[m]["metrics"]["goal_reached"] for m in METHODS[:2])
                matches.append((-missing_goals, identifier))
            goals = [trials[m]["metrics"]["goal_reached"] for m in METHODS]
            print(f"TURN_SCAN={index+1}/{len(eligible)} {identifier} pass={list(passes.values())} goals={goals} match={match}", flush=True)
        write_json(output / "scan_summary.json", {"cases": scan, "provisional_matches": matches, "completed_utc": utc()})
        selected = None
        for attempt, (_, identifier) in enumerate(sorted(matches)):
            item = archive.load(identifier, case="R1")
            confirmation = output / f"confirmation_{attempt:02d}"
            confirmation.mkdir()
            final_config = {**config, "cases": ["R1"], "methods": list(METHODS),
                "repetitions": criteria["repetitions"], "transition_ids": {"R1": identifier}}
            final_protocol = {**protocol, "config": final_config,
                "input_hashes": {"R1": {"source": item.source_sha256, "result": item.result_sha256}}}
            write_json(confirmation / "protocol.json", final_protocol)
            with (confirmation / "config_snapshot.yaml").open("x") as stream:
                yaml.safe_dump(final_config, stream)
            write_json(confirmation / "references/R1/input_reference.json", item.input_reference)
            for method in METHODS:
                with (confirmation / "references/R1" / f"{method}.npy").open("xb") as stream:
                    np.save(stream, item.method_candidates[method])
            rows = [execute(item, method, rep, confirmation / "trials/R1" / method / f"repetition_{rep:02d}", confirmation / "protocol.json")
                    for method in METHODS for rep in range(criteria["repetitions"])]
            result = summarize_execution(rows, ["R1"], list(METHODS), criteria)
            write_json(confirmation / "summary.json", {**result, "trials": rows, "trial_count": len(rows), "completed_utc": utc()})
            outcomes = result["outcomes"]["R1"]
            match = (outcomes["M3_LOOKAHEAD"]["passed"] and not outcomes["M0_RAW"]["passed"] and
                     not outcomes["M1_HISTORICAL_M4"]["passed"] and all(actual_is_turning(row["actual_geometry"], config)
                     for row in rows if row["method"] == "M3_LOOKAHEAD"))
            if match:
                selected = {"identifier": identifier, "confirmation": str(confirmation), "outcomes": outcomes}
                break
        for key, before in evidence.items():
            after = archive.load(key)
            if after.source_sha256 != before.source_sha256 or after.result_sha256 != before.result_sha256:
                raise ValueError("source changed during search")
        if any(sha256_file(ROOT / name) != digest for name, digest in code_hashes.items()):
            raise ValueError("execution code changed during search")
        write_json(output / "selection.json", {"selected": selected, "eligible_case_count": len(eligible),
            "scan_trial_count": len(scan) * len(METHODS), "provisional_match_count": len(matches),
            "source_hashes_rechecked": True, "completed_utc": utc()})
        print(f"TURN_SEARCH_COMPLETE={output} selected={identifier if selected else None}", flush=True)
    except Exception:
        import traceback
        traceback.print_exc()
        print("TURN_SEARCH_FAILED", flush=True)
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()

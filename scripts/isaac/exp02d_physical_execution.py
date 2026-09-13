#!/usr/bin/env python3
"""Physically execute frozen RAW/M3 EXP-02D references in the source Hospital."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
import yaml
from reconciliation.exp02d_gui import load_exp02d_gui_case
from reconciliation.exp02d_execution import safe_run_directory, summarize_execution
from reconciliation.online_switch import sha256_file
from reconciliation.closed_loop_execution_validation import (
    compute_closed_loop_metrics, load_frozen_stage0d_candidate,
    save_closed_loop_trial, validate_closed_loop_trial,
)


def utc():
    return datetime.now(timezone.utc).isoformat()


def resolve(path):
    value = Path(path)
    return value if value.is_absolute() else ROOT / value


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/exp02d_physical_execution.yaml")
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    if config["mode"] != "calibrated":
        raise ValueError("this comparison freezes the current calibrated controller")
    source_run = resolve(config["source_run"])
    evidence = {case: load_exp02d_gui_case(source_run, case) for case in config["cases"]}
    sources = {case: yaml.safe_load((item.active_old.run / "config_snapshot.yaml").read_text())
               for case, item in evidence.items()}
    source = next(iter(sources.values()))
    for other in sources.values():
        if any(source[key] != other[key] for key in ("simulation", "robot", "environment", "follower")):
            raise ValueError("representatives require different simulation systems")
    base = yaml.safe_load(resolve(config["stage0e_config"]).read_text())
    criteria = base["acceptance_criteria"]
    if config["repetitions"] != criteria["repetitions"]:
        raise ValueError("retain the existing three-repetition acceptance convention")
    candidate = load_frozen_stage0d_candidate(resolve(source["paths"]["stage0d_run"]),
                    source["stage0d_candidate_provenance"], repository_root=ROOT)
    output = safe_run_directory(resolve(config["output_root"]), args.run_id)
    output.mkdir(parents=True, exist_ok=False)
    with (output / "config_snapshot.yaml").open("x") as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
    code = [Path(__file__), ROOT / "scripts/isaac/exp02d_execution_runtime.py",
            ROOT / "scripts/isaac/closed_loop_execution_runtime.py",
            ROOT / "src/reconciliation/exp02d_execution.py",
            ROOT / "src/reconciliation/controllers/jackal_execution_controller.py",
            ROOT / "src/reconciliation/controllers/trajectory_follower.py"]
    code_hashes = {str(path.relative_to(ROOT)): sha256_file(path) for path in code}
    protocol = {"started_utc": utc(), "git_commit": subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "config": config, "config_sha256": sha256_file(args.config),
        "stage0e_config_sha256": sha256_file(resolve(config["stage0e_config"])),
        "acceptance_criteria": criteria, "source_system": source,
        "candidate": candidate, "code_sha256": code_hashes,
        "source_run": str(source_run),
        "coordinate_frame": "Isaac world X/Y metres, yaw CCW radians about +Z; identity transform; no path scaling, B insertion or resampling",
        "timing": "1/60 s physics and 0.1 s control; row zero is settled observation; later rows hold the command over the ending interval; feedback from previous control interval; no waypoint time",
        "input_hashes": {case: {"source": dict(item.source_sha256), "result": dict(item.result_sha256)} for case, item in evidence.items()},
    }
    write_json(output / "protocol.json", protocol)
    for case, item in evidence.items():
        write_json(output / "references" / case / "input_reference.json", dict(item.input_reference))
        import numpy as np
        for method in config["methods"]:
            with (output / "references" / case / f"{method}.npy").open("xb") as stream:
                np.save(stream, item.method_candidates[method])
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        from exp02d_execution_runtime import HospitalExecutionRuntime, loop_configs
        from closed_loop_execution_runtime import run_closed_loop
        runtime = HospitalExecutionRuntime(app, source, candidate,
                        next(iter(evidence.values())).active_old.boundary_pose_world_se2)
        loop, follower = loop_configs(source, config["maximum_duration_s"])
        write_json(output / "runtime.json", {"hospital": str(runtime.hospital), "robot": runtime.asset,
                    "wheel_runtime": runtime.wheels, "additional_ground_plane": False})
        rows = []
        for case, item in evidence.items():
            for method in config["methods"]:
                for repetition in range(config["repetitions"]):
                    started = utc()
                    telemetry = run_closed_loop(runtime, loop, follower, item.method_candidates[method],
                                      item.active_old.boundary_pose_world_se2, "calibrated")
                    runtime.world.pause()
                    metrics = compute_closed_loop_metrics(telemetry)
                    directory = output / "trials" / case / method / f"repetition_{repetition:02d}"
                    reference_path = output / "references" / case / f"{method}.npy"
                    metadata = {"stage": config["experiment"], "mode": "calibrated",
                        "evaluation_semantics": "reference-matched closed-loop evaluation",
                        "case": case, "method": method, "repetition": repetition,
                        "scenario": f"{case}_{method}",
                        "corpus_transition_id": item.corpus_transition_id,
                        "started_utc": started, "completed_utc": utc(),
                        "protocol_path": str(output / "protocol.json"),
                        "protocol_sha256": sha256_file(output / "protocol.json"),
                        "reference_path": str(reference_path), "reference_sha256": sha256_file(reference_path),
                        "requested_initial_pose_world_se2": item.active_old.boundary_pose_world_se2.tolist(),
                        "settled_initial_pose_world_se2": telemetry.actual_trajectory[0].tolist(),
                        "source_observation_sim_time_s": item.active_old.observation_sim_time_s,
                        "source_readiness_sim_time_s": item.active_old.model_ready_sim_time_s,
                        "source_switch_sim_time_s": item.active_old.switch_sim_time_s,
                        "new_inference_readiness": None,
                        "original_online_velocity_integral_contact_history_restored": False,
                        "controller_tuned": False, "candidate_optimized_again": False,
                        "coordinate_frame": protocol["coordinate_frame"], "timing": protocol["timing"]}
                    hashes = save_closed_loop_trial(directory, telemetry, metrics, metadata)
                    validate_closed_loop_trial(directory)
                    rows.append({"case": case, "method": method, "repetition": repetition,
                                 "directory": str(directory), "metrics": metrics, "raw_sha256": hashes})
                    print(f"EXP02D_PHYSICAL_TRIAL={case} {method} {repetition} "
                          f"goal={metrics['goal_reached']} position_RMS={metrics['position_rmse_m']:.6f} "
                          f"yaw_RMS={metrics['yaw_rmse_rad']:.6f}", flush=True)
        for case, item in evidence.items():
            after = load_exp02d_gui_case(source_run, case)
            if dict(after.source_sha256) != dict(item.source_sha256) or dict(after.result_sha256) != dict(item.result_sha256):
                raise ValueError("source or optimized candidate changed during physical execution")
        if any(sha256_file(ROOT / path) != digest for path, digest in code_hashes.items()):
            raise ValueError("runtime code changed during execution")
        summary = summarize_execution(rows, config["cases"], config["methods"], criteria)
        write_json(output / "summary.json", {**summary, "trials": rows, "trial_count": len(rows),
                   "completed_utc": utc(), "source_hashes_verified_after_execution": True})
        print(f"EXP02D_PHYSICAL_COMPLETE={output} " + json.dumps(summary["gui_selection"]), flush=True)
    except Exception:
        import traceback
        traceback.print_exc()
        print("EXP02D_PHYSICAL_FAILED", flush=True)
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()

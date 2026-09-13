#!/usr/bin/env python3
"""Compare the current frozen correction with nominal execution, without tuning."""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--config", type=Path, default=ROOT / "configs/controller_effect_check.yaml")
parser.add_argument("--run-id", required=True)
args = parser.parse_args()

from isaacsim import SimulationApp

APP = SimulationApp({"headless": True})

import numpy as np
import yaml

from closed_loop_execution_runtime import ClosedLoopRuntime, follower_config, run_closed_loop
from lightnav_stage0c_runtime import canonical_wheel_values, se2_from_world_pose
from reconciliation.closed_loop_execution_validation import (
    compute_closed_loop_metrics, load_frozen_stage0d_candidate,
    save_closed_loop_trial, validate_closed_loop_trial,
)
from reconciliation.controller_effect_check import record_rotation, rotation_drift_metrics
from reconciliation.data02_active_old import load_active_old_interval
from reconciliation.execution_calibration import (
    compute_trial_metrics, save_execution_trial, validate_execution_trial,
)
from reconciliation.online_switch import sha256_file


def resolve(value):
    path = Path(value)
    return path if path.is_absolute() else ROOT / path


def read_yaml(path):
    return yaml.safe_load(Path(path).read_text())


def utc():
    return datetime.now(timezone.utc).isoformat()


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


class RotationRuntime(ClosedLoopRuntime):
    def read_pose(self):
        return se2_from_world_pose(self.robot)

    def read_wheels(self):
        return canonical_wheel_values(self.robot.get_joint_velocities(), self.wheels)

    def ideal_sides(self, desired):
        return np.asarray(self.differential.forward(desired), dtype=float)

    def canonical_targets(self, target):
        return canonical_wheel_values(target, self.wheels)


def main():
    config = read_yaml(args.config)
    base_path = resolve(config["stage0e_config"])
    base = read_yaml(base_path)
    follower_path = resolve(base["paths"]["stage0b_config"])
    stage0b = read_yaml(follower_path)
    stage0b["closed_loop"]["maximum_duration_s"] = config["saved_reference_maximum_duration_s"]
    candidate = load_frozen_stage0d_candidate(
        resolve(base["paths"]["stage0d_run"]), base["stage0d_candidate_provenance"],
        repository_root=ROOT,
    )
    data02 = resolve(config["data02_run"])
    source_protocol = json.loads((data02 / "protocol.json").read_text())
    follower_values = asdict(follower_config(stage0b))
    if any(source_protocol["system"]["follower"][key] != value
           for key, value in follower_values.items()):
        raise ValueError("current follower differs from the frozen DATA-02 follower")
    cases = [(value, load_active_old_interval(data02, value["episode"], value["transition"]))
             for value in config["saved_old_cases"]]
    output = resolve(config["output_root"]) / args.run_id
    output.mkdir(parents=True, exist_ok=False)
    with (output / "config_snapshot.yaml").open("x") as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
    code_paths = [Path(__file__), ROOT / "src/reconciliation/controller_effect_check.py",
                  ROOT / "scripts/isaac/closed_loop_execution_runtime.py",
                  ROOT / "src/reconciliation/controllers/jackal_execution_controller.py",
                  ROOT / "src/reconciliation/controllers/trajectory_follower.py"]
    code_hashes = {str(path.relative_to(ROOT)): sha256_file(path) for path in code_paths}
    protocol = {
        "started_utc": utc(), "git_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "code_sha256": code_hashes, "config_sha256": sha256_file(args.config),
        "base_config_sha256": sha256_file(base_path),
        "stage0b_input_sha256": sha256_file(follower_path),
        "simulation": base["simulation"], "follower": follower_values,
        "candidate": candidate, "semantics": config["semantics"],
        "coordinates": "Isaac world x/y metres, yaw CCW about +Z in radians; no transform",
        "timing": "row zero is post-settling observation; later rows describe commands applied over the ending physics interval; controller samples every 0.1 s; no waypoint timestamps",
        "saved_sources": {f"{value['episode']}/{value['transition']}": dict(case.source_sha256)
                          for value, case in cases},
    }
    write_json(output / "protocol.json", protocol)
    runtime = RotationRuntime(APP, base, stage0b, candidate, render=False)
    rows = []
    common = {
        "stage": config["stage"], "protocol_path": str(output / "protocol.json"),
        "protocol_sha256": sha256_file(output / "protocol.json"),
        "environment": "Stage 0-E default flat ground; Hospital is not loaded",
        "controller_tuned": False, "raw_vla_modified": False,
        "coordinate_frame": protocol["coordinates"], "timing": protocol["timing"],
        "jackal_asset": runtime.asset, "wheel_runtime": runtime.wheels,
        "controller_model_sha256": candidate["model_sha256"],
    }
    for value, case in cases:
        scenario = f"{value['episode']}_transition_{value['transition']:02d}"
        reference_path = output / "references" / f"{scenario}.npy"
        reference_path.parent.mkdir(exist_ok=True)
        with reference_path.open("xb") as stream:
            np.save(stream, case.old_world)
        for mode in ("nominal", "calibrated"):
            for repeat in range(config["repetitions"]):
                started = utc()
                telemetry = run_closed_loop(runtime, base, stage0b, case.old_world,
                                            case.activation_pose_world_se2, mode)
                metrics = compute_closed_loop_metrics(telemetry)
                destination = output / "saved_old" / scenario / mode / f"repetition_{repeat:02d}"
                metadata = {
                    **common, "mode": mode, "scenario": scenario, "repetition": repeat,
                    "evaluation_semantics": "reference-matched closed-loop evaluation",
                    "started_utc": started, "completed_utc": utc(),
                    "reference_path": str(reference_path),
                    "reference_sha256": sha256_file(reference_path),
                    "requested_initial_pose_world_se2": case.activation_pose_world_se2.tolist(),
                    "settled_initial_pose_world_se2": telemetry.actual_trajectory[0].tolist(),
                    "original_activation_sim_time_s": case.activation_sim_time_s,
                    "original_observation_sim_time_s": case.observation_sim_time_s,
                    "original_switch_sim_time_s": case.switch_sim_time_s,
                    "original_history_velocity_integral_restored": False,
                    "maximum_execution_duration_s": config["saved_reference_maximum_duration_s"],
                    "meaning": "same saved OLD and activation pose; reset-state flat-ground comparison, not original online episode reproduction",
                }
                hashes = save_closed_loop_trial(destination, telemetry, metrics, metadata)
                validate_closed_loop_trial(destination)
                rows.append({"kind": "saved_old", "scenario": scenario, "mode": mode,
                             "repetition": repeat, "directory": str(destination),
                             "raw_sha256": hashes, "metrics": metrics})
                print(f"CONTROLLER_EFFECT_TRIAL={scenario} {mode} {repeat} "
                      f"position_RMS={metrics['position_rmse_m']:.6f} "
                      f"goal={metrics['goal_reached']}", flush=True)
    for omega in config["rotation"]["desired_omega_rps"]:
        scenario = f"rotate_{'p' if omega > 0 else 'm'}{abs(omega):.2f}"
        for mode in ("nominal", "calibrated"):
            for repeat in range(config["repetitions"]):
                started = utc()
                telemetry, states = record_rotation(runtime, config["rotation"], mode, omega)
                metrics = compute_trial_metrics(
                    telemetry, wheel_radius_m=float(runtime.wheels["radius_m"]),
                    steady_state_tail_s=config["rotation"]["steady_tail_s"],
                    minimum_abs_measured_omega_rps=config["rotation"]["minimum_measured_omega_rps"],
                )
                metrics.update(rotation_drift_metrics(telemetry))
                destination = output / "rotation" / scenario / mode / f"repetition_{repeat:02d}"
                metadata = {
                    **common, "mode": mode, "scenario": scenario, "repetition": repeat,
                    "condition_id": scenario,
                    "started_utc": started, "completed_utc": utc(),
                    "source_kind": "synthetic engineering command fixture, not research evidence",
                    "rotation_config": config["rotation"], "desired_omega_rps": omega,
                }
                hashes = save_execution_trial(destination, telemetry, metrics, metadata)
                write_json(destination / "raw/controller_states.json", states)
                hashes["raw/controller_states.json"] = sha256_file(destination / "raw/controller_states.json")
                validate_execution_trial(destination)
                rows.append({"kind": "rotation", "scenario": scenario, "mode": mode,
                             "repetition": repeat, "directory": str(destination),
                             "raw_sha256": hashes, "metrics": metrics})
                print(f"CONTROLLER_EFFECT_TRIAL={scenario} {mode} {repeat} "
                      f"omega_RMSE={metrics['omega_rmse_rps']:.6f} "
                      f"active_drift={metrics['active_end_xy_drift_m']:.6f}", flush=True)
    for path in code_paths:
        if sha256_file(path) != code_hashes[str(path.relative_to(ROOT))]:
            raise ValueError("runtime source changed during execution")
    for value, original in cases:
        after = load_active_old_interval(data02, value["episode"], value["transition"])
        if dict(after.source_sha256) != dict(original.source_sha256):
            raise ValueError("original DATA-02 source changed")
    write_json(output / "summary.json", {"completed_utc": utc(), "trial_count": len(rows),
               "source_hashes_verified_after_execution": True, "trials": rows})
    print(f"CONTROLLER_EFFECT_COMPLETE={output} trials={len(rows)}", flush=True)


try:
    main()
except Exception:
    # Isaac close may terminate the process; emit the failure before closing.
    import traceback
    traceback.print_exc()
    print("CONTROLLER_EFFECT_FAILED", flush=True)
    raise
finally:
    APP.close()

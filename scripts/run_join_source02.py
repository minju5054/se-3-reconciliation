#!/usr/bin/env python3
"""Freeze and run bounded JOIN-SOURCE-02 paired development screening.

Only ``screen`` invokes the existing official inference worker. It never starts
a server or constructs a controller/optimizer. ``prepare`` and ``analyze`` are
saved-record operations. Online acquisition and confirmation are separate.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "scripts/lightnav")]
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.join_source02 import (
    BRANCHES, REQUIRED_SCREENING_CHECKS, declared_conditions, evaluate_screening,
    observation_anchored_world, validate_development_order, validate_paired_inputs,
)
from reconciliation.robotless_single_chunk import save_json_exclusive, sha256_file
from reconciliation.join_source02_geometry import compose_environment, project_prop, projection_from_metadata
from join_source02_paired import declared_condition_ids, wire_parity

MAPPING = dict(A_OFF="A", B_ON="B", A_SHAM="SHAM")


def read(path):
    return json.loads(Path(path).read_text())


def resolve(path, base=ROOT):
    path = Path(path)
    return path.resolve() if path.is_absolute() else (base / path).resolve()


def git(*args):
    return subprocess.check_output(["git", "-C", str(ROOT), *args], text=True).strip()


def source_files():
    fixed = ["src/reconciliation/online_history.py", "src/reconciliation/robotless_online.py",
             "src/reconciliation/gp_se2_environment.py", "src/reconciliation/se2.py",
             "src/reconciliation/robotless_projection_handoff.py", "src/reconciliation/robotless_single_chunk.py",
             "src/reconciliation/robotless_controlled_staleness.py", "src/reconciliation/online_mpc_adapter.py",
             "scripts/online_lightnav_worker.py", "scripts/online_mpc_worker.py",
             "scripts/isaac/robotless_online_handoffs.py", "scripts/isaac/robotless_runtime.py",
             "scripts/isaac/robotless_old_consistent_observation.py",
             "scripts/lightnav/robotless_online_server.py", "scripts/lightnav/robotless_successive_server.py",
             "scripts/lightnav/robotless_single_frame_inference.py", "configs/robotless_online_handoffs.yaml"]
    paths = {ROOT / p for p in fixed}
    for parent, pattern in (("src/reconciliation", "join_source02*.py"), ("scripts", "*join_source02*.py"),
                            ("scripts/lightnav", "*join_source02*.py"), ("scripts/isaac", "*join_source02*.py"),
                            ("tests", "test_join_source02*.py"), ("configs", "join_source_02.yaml")):
        paths.update((ROOT / parent).glob(pattern))
    return {str(p.relative_to(ROOT)): sha256_file(p) for p in sorted(paths)}


def prepare(run, config_path):
    """Freeze protocol/code before live history acquisition; no fabricated manifests."""
    config_path = resolve(config_path)
    experiment = yaml.safe_load(config_path.read_text())
    base = yaml.safe_load(resolve(experiment["base_config"]).read_text())
    config = {**base, **experiment}
    run.mkdir(parents=True, exist_ok=True)
    snapshot = run / "config_snapshot.yaml"
    if snapshot.exists():
        if yaml.safe_load(snapshot.read_text()) not in (base, config):
            raise ValueError("existing config snapshot differs; overwrite prohibited")
    else:
        with snapshot.open("x") as stream:
            yaml.safe_dump(config, stream, sort_keys=False)
    declaration = declared_conditions([x["id"] for x in config["locations"]],
                                      [x["id"] for x in config["placements"]])
    protocol_path = run / "protocol.json"
    if not protocol_path.exists():
        save_json_exclusive(protocol_path, {
            "experiment": "JOIN-SOURCE-02", "development": {"condition_order": declaration,
                "branches": ["A", "B", "SHAM"], "first_complete_qualifier_stops_development": True,
                "maximum_conditions": 12, "maximum_terminal_predictions": 36},
            "confirmation": config["confirmation"], "qualification": config["qualification"],
            "source_capture": "separate live acquisition; no saved screening RGB in online confirmation",
            "history": config["history"], "placement_design": config["placement_design"],
            "generation_environment": config["generation_environment"],
            "immutable_scenario": {"locations": config["locations"], "placements": config["placements"],
                                   "prop": config["prop"], "camera": config["camera"]},
            "no_GP_MPC_reconciliation_comparison": True,
        })
    protocol = read(protocol_path)
    if declared_condition_ids(protocol) != [r["condition_id"] for r in declaration]:
        raise ValueError("protocol order does not match source-only frozen config")
    environment = resolve(config["environment"])
    environment_files = [environment / name for name in ("validation.json", "scene_provenance.json",
        "geometry/world_triangles.npz", "geometry/meshes.json", "gate_catalog.json")]
    environment_files.extend(sorted((environment / "geometry/projected").iterdir()))
    save_json_exclusive(run / "development_freeze.json", {
        "preparation_git_sha": git("rev-parse", "HEAD"), "branch": git("branch", "--show-current"),
        "config_source": str(config_path), "config_source_sha256": sha256_file(config_path),
        "config_snapshot_sha256": sha256_file(snapshot), "protocol_sha256": sha256_file(protocol_path),
        "source_sha256": source_files(), "input_manifests_required_at_freeze": False,
        "environment_source_sha256": {str(p): sha256_file(p) for p in environment_files if p.is_file()},
        "input_policy": "live histories/OLD-derived placements acquired only after protocol/code commit; preserve all attempts",
        "new_inference_calls": 0, "new_MPC_calls": 0,
    })
    return {"run": str(run), "declared_conditions": len(declaration), "status": "PRE_PRIMARY_FROZEN"}


def verify_freeze(run, *, require_pushed=False):
    freeze = read(run / "development_freeze.json")
    for name, key in (("config_snapshot.yaml", "config_snapshot_sha256"), ("protocol.json", "protocol_sha256")):
        if sha256_file(run / name) != freeze[key]:
            raise ValueError(f"frozen {name} changed")
    for name, digest in freeze["source_sha256"].items():
        if sha256_file(ROOT / name) != digest:
            raise ValueError(f"frozen implementation/core changed: {name}")
    for name, digest in freeze.get("environment_source_sha256", {}).items():
        if sha256_file(Path(name)) != digest:
            raise ValueError(f"frozen environment changed: {name}")
    if require_pushed:
        if git("rev-parse", "HEAD") != git("rev-parse", "@{upstream}"):
            raise ValueError("commit and normal push the pre-primary implementation before screening")
        changed = git("status", "--porcelain", "--", *freeze["source_sha256"])
        if changed:
            raise ValueError("frozen implementation/config has uncommitted changes")
    return freeze


def load_environments(placement):
    import shapely
    off = HospitalEnvironment.load(resolve(placement["environment_export"]))
    if "projection" in placement:
        metadata = placement["projection"].get("metadata", placement["projection"])
        projection = projection_from_metadata(metadata)
        if "triangles_path" in placement:
            triangles = np.load(resolve(placement["triangles_path"]), allow_pickle=False)["triangles"]
            recomputed = project_prop(triangles)
            for key in ("triangles_float64_c_sha256", "obstacle_wkb_hex", "unknown_wkb_hex"):
                if recomputed["metadata"][key] != metadata[key]:
                    raise ValueError(f"runtime prop saved geometry mismatch: {key}")
        return off, compose_environment(off, projection, True)
    if "obstacle_wkb_hex" in placement:
        obstacle = shapely.from_wkb(bytes.fromhex(placement["obstacle_wkb_hex"]))
    else:
        path = resolve(placement["obstacle_geometry_path"])
        if path.suffix.lower() == ".json":
            record = read(path)
            obstacle = shapely.from_wkb(bytes.fromhex(record["obstacle_wkb_hex"]))
        else:
            obstacle = shapely.from_wkb(path.read_bytes())
    if obstacle.is_empty or not obstacle.is_valid:
        raise ValueError("invalid or empty actual-prop obstacle geometry")
    on = HospitalEnvironment(off.obstacles.union(obstacle), off.workspace,
        parts=[*off.parts, obstacle], metadata=off.metadata.copy())
    on.gates = list(off.gates)
    return off, on


def evaluate_saved(run, manifest):
    """Re-read original response/NPY/wire records; no inference or controller call."""
    output = run / "development" / manifest["condition_id"]
    worker = read(output / "summary.json")
    startup = read(output / "official_startup.json")
    config = yaml.safe_load((run / "config_snapshot.yaml").read_text())
    placement = read(resolve(manifest["placement_path"]))
    source_ok = True
    errors = []
    branches, paths, stops, completed = {}, {}, {}, []
    all_wire = True
    counts = dict(terminal_predictions=0, buffer_only=0)
    for name in BRANCHES:
        alias = MAPPING[name]
        branch_dir = output / "branches" / alias
        result = read(branch_dir / "paired_result.json")
        frames = read(branch_dir / "replayed_inputs.json")["frames"]
        parity = wire_parity(branch_dir, frames)
        all_wire = all_wire and parity["valid"]
        counts["terminal_predictions"] += parity.get("actual_terminal_predictions_sent", 0)
        counts["buffer_only"] += parity.get("actual_next_requests_sent", 0)-parity.get("actual_terminal_predictions_sent", 0)
        # A failed response or history delivery remains a completed branch
        # attempt. Actual terminal calls are counted from sent wire records.
        completed.append(name)
        terminal = result.get("terminal_prediction")
        if terminal is None or terminal.get("status") not in ("PREDICTION_READY", "MODEL_STOP"):
            errors.append(f"{name}: {result['status']}")
            source_ok = False
            continue
        raw = np.load(branch_dir / "chunks/terminal/raw_local.npy", allow_pickle=False)
        world = np.load(branch_dir / "chunks/terminal/world.npy", allow_pickle=False)
        response = read(branch_dir / "chunks/terminal/response.json")["data"]
        source_frame = manifest["final_frames"][alias]
        exact_raw = np.array_equal(raw, np.asarray(response["actions"]["actions"], dtype=np.float64))
        exact_world = np.allclose(world, observation_anchored_world(raw, source_frame["pose_world"]), atol=1e-12, rtol=0)
        source_ok = source_ok and exact_raw and exact_world
        if not exact_raw or not exact_world:
            errors.append(f"{name}: raw-response or observation-transform mismatch")
        branches[name] = {"session_id": result["session_close"]["connection_id"],
            "instruction": manifest["instruction"], "camera_config": config["camera"],
            "model_config": config["lightnav"], "history": deepcopy(manifest["history"]),
            "final_frame": deepcopy(source_frame)}
        paths[name] = world
        stops[name] = response["stop"]
    flags = {key: placement.get("technical_checks", {}).get(key) is True for key in REQUIRED_SCREENING_CHECKS}
    flags.update(source_integrity=bool(source_ok and flags["source_integrity"]), official_contract=bool(flags["official_contract"]
                 and worker.get("source_unchanged") and worker.get("checkpoint_stat_unchanged")
                 and startup.get("config_sha256") == sha256_file(run / "config_snapshot.yaml")),
                 wire_jpeg_parity=bool(all_wire), paired_inputs=False)
    paired = None
    if len(branches) == 3:
        try:
            paired = validate_paired_inputs(branches)
            flags["paired_inputs"] = paired["valid"]
        except Exception as exc:
            errors.append(f"paired inputs: {type(exc).__name__}: {exc}")
    common = {"condition_id": manifest["condition_id"], "availability": "AVAILABLE",
              "completed_branches": completed, "call_counts": counts,
              "worker_output_path": str(output), "input_manifest_sha256": sha256_file(resolve(manifest["_manifest_path"])),
              "placement_sha256": sha256_file(resolve(manifest["placement_path"])),
              "paired_inputs_validation": paired, "technical_errors": errors,
              "worker_wall_time_s": worker.get("wall_time_s"),
              "new_optimizer_or_controller_outcomes_used": False}
    if len(paths) != 3:
        return {**common, "label": "TECHNICAL_INVALID", "qualified": False,
                "technical_checks": flags, "reason": "missing/error terminal source; no geometric output fabricated"}
    off, on = load_environments(placement)
    result = evaluate_screening(paths["A_OFF"], paths["B_ON"], paths["A_SHAM"],
        observation_pose_world=manifest["final_frames"]["A"]["pose_world"],
        target_xy=placement["target_xy"], environment_off=off, environment_on=on,
        influence=placement["influence"], bypass_plane=placement["bypass_plane"],
        technical_checks=flags, stops=stops)
    return {**result, **common}


def ledger_prefix(run, ids):
    ledger = []
    gap = False
    for condition_id in ids:
        directory = run / "development" / condition_id
        path = directory / "evaluation.json"
        if path.exists():
            if gap:
                raise ValueError("completed condition appears after a missing declared predecessor")
            row = read(path)
            if row.get("condition_id") != condition_id:
                raise ValueError("saved condition ID differs from directory")
            ledger.append(row)
        else:
            if directory.exists():
                raise ValueError(f"partial condition preserved; no retry allowed: {directory}")
            gap = True
    return ledger


def screen(run, *, only=None):
    freeze = verify_freeze(run, require_pushed=True)
    protocol = read(run / "protocol.json")
    ids = declared_condition_ids(protocol)
    declaration = [{"condition_id": x} for x in ids]
    aggregate = run / "aggregate"
    aggregate.mkdir(exist_ok=True)
    final_path = aggregate / "development_ledger.json"
    if final_path.exists():
        raise FileExistsError("completed development ledger exists; no repeat/overwrite")
    ledger = ledger_prefix(run, ids)
    checked = validate_development_order(declaration, ledger)
    if checked["selected_condition_id"]:
        raise ValueError("first qualifying condition already recorded; development must remain stopped")
    if only is not None and (len(ledger) == len(ids) or only != ids[len(ledger)]):
        raise ValueError("only the next predeclared condition may run")
    execution_path = run / "development_execution.json"
    if not execution_path.exists():
        save_json_exclusive(execution_path, {"git_sha": git("rev-parse", "HEAD"),
            "branch": git("branch", "--show-current"), "freeze_sha256": sha256_file(run / "development_freeze.json"),
            "prepared_git_sha": freeze["preparation_git_sha"], "no_automatic_server_launch": True})
    started = time.monotonic()
    for condition_id in ids[len(ledger):]:
        validate_development_order(declaration, ledger)
        manifest_path = run / "development/input_manifests" / f"{condition_id}.json"
        if not manifest_path.exists():
            raise FileNotFoundError(f"declared condition has no availability/source manifest: {manifest_path}")
        manifest = read(manifest_path)
        if manifest["condition_id"] != condition_id:
            raise ValueError("manifest/deterministic-order identity mismatch")
        manifest["_manifest_path"] = str(manifest_path)
        output = run / "development" / condition_id
        availability = manifest.get("availability", "AVAILABLE")
        if availability != "AVAILABLE":
            if availability not in ("HISTORY_UNAVAILABLE", "PLACEMENT_UNAVAILABLE", "TECHNICAL_INVALID"):
                raise ValueError("unsupported availability reason")
            output.mkdir(exist_ok=False)
            row = {"condition_id": condition_id, "availability": availability, "label": availability,
                   "completed_branches": [], "qualified": False, "reason": manifest.get("reason"),
                   "call_counts": {"terminal_predictions": 0, "buffer_only": 0},
                   "input_manifest_sha256": sha256_file(manifest_path)}
        else:
            if output.exists():
                raise ValueError("partial run exists; no branch retry")
            if sha256_file(resolve(manifest["config_path"])) != sha256_file(run / "config_snapshot.yaml"):
                raise ValueError("paired worker config differs from frozen snapshot")
            config = yaml.safe_load((run / "config_snapshot.yaml").read_text())
            python = resolve(config["paths"]["lightnav_checkout"]) / ".venv/bin/python"
            logs = run / "development/worker_logs"
            logs.mkdir(exist_ok=True)
            argv = [str(python), str(ROOT / "scripts/lightnav/join_source02_paired.py"),
                    "--run", str(run), "--condition", str(manifest_path)]
            with (logs / f"{condition_id}.log").open("xb") as stream:
                process = subprocess.run(argv, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
            if not (output / "summary.json").exists():
                raise RuntimeError(f"paired worker technical failure exit={process.returncode}; artifacts preserved; no retry")
            row = evaluate_saved(run, manifest)
            row["worker_exit_code"] = process.returncode
        save_json_exclusive(output / "evaluation.json", row)
        ledger.append(row)
        checked = validate_development_order(declaration, ledger)
        if checked["selected_condition_id"] or only is not None:
            break
    finished = bool(checked["selected_condition_id"] or len(ledger) == len(ids))
    summary = {"records": ledger, "order_validation": checked,
        "selected_condition_id": checked["selected_condition_id"], "planned_conditions": len(ids),
        "actual_terminal_prediction_requests": sum(x.get("call_counts", {}).get("terminal_predictions", 0) for x in ledger),
        "actual_buffer_only_requests": sum(x.get("call_counts", {}).get("buffer_only", 0) for x in ledger),
        "wall_time_s_this_invocation": time.monotonic()-started,
        "status": "DEVELOPMENT_RESPONSE_ONLY" if checked["selected_condition_id"] else
                  "NO_QUALIFYING_DEVELOPMENT_SCENARIO" if finished else "DEVELOPMENT_IN_PROGRESS",
        "confirmation_attempted": False, "new_GP_solves": 0, "new_MPC_solves_in_this_runner": 0,
        "other_model_call_categories": "startup warmup/live history acquisition/confirmation/test calls recorded separately",
        "complete_declared_phase": finished}
    if finished:
        save_json_exclusive(final_path, summary)
        save_json_exclusive(aggregate / "actual_call_counts_development.json", {
            "scope": "paired development only; excludes live history acquisition, warmup, confirmation, tests",
            "terminal_predictions": summary["actual_terminal_prediction_requests"],
            "buffer_only_requests": summary["actual_buffer_only_requests"], "retries": 0,
            "GP_solves": 0, "MPC_solves": 0})
    return summary


def analyze(run, condition_id):
    """Independent derived re-evaluation of one complete saved paired condition."""
    verify_freeze(run)
    if condition_id not in declared_condition_ids(read(run / "protocol.json")):
        raise ValueError("analyze requires one declared --condition")
    path = run / "development/input_manifests" / f"{condition_id}.json"
    manifest = read(path)
    manifest["_manifest_path"] = str(path)
    result = evaluate_saved(run, manifest)
    result.update(saved_record_analysis=True, new_inference_calls=0, new_MPC_calls=0)
    output = run / "development" / condition_id / "saved_analysis.json"
    save_json_exclusive(output, result)
    return {"output": str(output), "label": result["label"], "qualified": result["qualified"],
            "new_inference_calls": 0, "new_MPC_calls": 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["prepare", "verify", "screen", "analyze"], required=True)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/join_source_02.yaml")
    parser.add_argument("--condition", help="screen only the next frozen ID; never skip/retry")
    args = parser.parse_args()
    run = args.run.resolve()
    if args.mode == "prepare":
        result = prepare(run, args.config)
    elif args.mode == "verify":
        verify_freeze(run, require_pushed=True)
        result = {"valid": True, "git_sha": git("rev-parse", "HEAD"), "new_inference_calls": 0, "new_MPC_calls": 0}
    elif args.mode == "analyze":
        result = analyze(run, args.condition)
    else:
        result = screen(run, only=args.condition)
    print(json.dumps({k: v for k, v in result.items() if k != "records"}, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()

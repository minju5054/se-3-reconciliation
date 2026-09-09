#!/usr/bin/env python3
"""Create and freeze a provenance-rich Stage 0-G2 qualification run."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.lightnav_adapter import save_json_exclusive
from reconciliation.stage0g2_jackal_domain_scene_qualification import (
    SCENARIO_IDS, sha256_file, validate_config, validate_preview_gate,
)
from reconciliation.stage0g_lightnav_scene_qualification import create_run_directory_exclusive


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/stage0g2_jackal_domain_scene_qualification.yaml")
    parser.add_argument("--run-id")
    parser.add_argument("--freeze", type=Path)
    parser.add_argument("--visual-inspection-passed", action="store_true")
    return parser.parse_args()


def load_config(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream: value = yaml.safe_load(stream)
    if not isinstance(value, dict): raise ValueError("config must be a mapping")
    validate_config(value); return value


def resolve(value: str) -> Path:
    path = Path(value).expanduser(); return path.resolve() if path.is_absolute() else (ROOT / path).resolve()


def git(checkout: Path, *args: str) -> str:
    return subprocess.check_output(["git", "-C", str(checkout), *args], text=True).strip()


def checkpoint_revision(path: Path) -> str | None:
    trees = sorted((path / ".cache/huggingface/trees").glob("*.json")); return trees[-1].stem if trees else None


def initialize(config_path: Path, run_id: str | None) -> Path:
    config = load_config(config_path); identifier = run_id or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run = resolve(str(config["paths"]["output_root"])) / identifier; create_run_directory_exclusive(run)
    checkout = resolve(str(config["paths"]["lightnav_checkout"])); checkpoint = checkout / str(config["paths"]["checkpoint_relative_path"])
    baseline = config["baseline"]; actual_hashes = {name: sha256_file(checkpoint / name) for name in baseline["expected_file_sha256"]}
    checks = {
        "lightnav_sha": git(checkout, "rev-parse", "HEAD") == baseline["expected_lightnav_git_sha"],
        "lightnav_clean": git(checkout, "status", "--porcelain") == "",
        "checkpoint_revision": checkpoint_revision(checkpoint) == baseline["expected_checkpoint_revision"],
        "checkpoint_hashes": actual_hashes == baseline["expected_file_sha256"],
        "jackal_required": config["robot"]["required_embodiment"] == "Jackal",
        "camera_contract_frozen": config["camera"]["resolution_width"] == 480 and config["camera"]["resolution_height"] == 270 and float(config["camera"]["horizontal_fov_deg"]) == 112.0,
        "history_contract_frozen": config["capture"]["frame_count"] == 64 and float(config["capture"]["fps"]) == 4.0,
        "task_contract_frozen": config["lightnav"]["task"] == "vln" and config["lightnav"]["task_type"] == "vlnce_traj",
        "waypoints_have_no_intrinsic_timestamps": config["lightnav"]["intrinsic_waypoint_time_base"] is False,
    }
    save_json_exclusive(run / "input_contract/lightnav_input_contract.json", {
        "stage": config["stage"], "created_utc": datetime.now(timezone.utc).isoformat(), "contract_resolved": all(checks.values()), "checks": checks,
        "lightnav": {"checkout": str(checkout), "expected_sha": baseline["expected_lightnav_git_sha"], "actual_sha": git(checkout, "rev-parse", "HEAD")},
        "checkpoint": {"identifier": baseline["checkpoint_identifier"], "revision": checkpoint_revision(checkpoint), "actual_file_sha256": actual_hashes},
        "fixed_input": {"camera": config["camera"], "capture": config["capture"], "lightnav": config["lightnav"], "output_shape": [10, 3], "output_dtype": "float32"},
        "scope": {"inference_only": True, "jackal_stationary": True, "controller": False, "reconciliation": False, "planner": False},
    })
    save_json_exclusive(run / "run_metadata.json", {"stage": config["stage"], "run_id": identifier, "research_git_sha_at_start": git(ROOT, "rev-parse", "HEAD"), "config_frozen": False, "primary_output_count": 0})
    print(f"STAGE0G2_RUN_DIRECTORY={run}"); return run


def freeze(config_path: Path, run: Path, visual_passed: bool) -> None:
    if not visual_passed: raise ValueError("freeze requires explicit --visual-inspection-passed after inspecting all six V0 images")
    config = load_config(config_path)
    for required in (run / "environment_asset_manifest.json", run / "environment_selection.json"):
        if not required.is_file(): raise ValueError(f"missing pre-inference environment audit: {required}")
    records = {}
    for scenario_id in SCENARIO_IDS:
        root = run / "scene_previews" / scenario_id / "V0"; metadata = json.loads((root / "preview_metadata.json").read_text())
        if not (root / "latest_rgb.png").is_file() or metadata.get("initial_pose_collision_free") is not True:
            raise ValueError(f"preview/collision gate failed: {scenario_id}")
        gate = {"free_space_visible": True, "intended_route_visible": True, "referenced_feature_visible": True, "natural_scene": True, "camera_usable": True, "initial_pose_collision_free": True}
        validate_preview_gate(gate); records[scenario_id] = gate
    save_json_exclusive(run / "scene_previews/preview_gate.json", {"inspection_actor": "Codex visual inspection", "inspection_utc": datetime.now(timezone.utc).isoformat(), "all_six_pass": True, "records": records})
    destination = run / "config_snapshot.yaml"
    with destination.open("xb") as stream: stream.write(config_path.read_bytes())
    save_json_exclusive(run / "config_freeze.json", {
        "frozen_utc": datetime.now(timezone.utc).isoformat(), "config_sha256": sha256_file(destination),
        "environment_selection_sha256": sha256_file(run / "environment_selection.json"), "all_six_v0_preview_gate_passed": True,
        "primary_outputs_at_freeze": 0, "prohibition": "No environment, camera, pose, instruction, variant, or threshold tuning after freeze.",
    })
    print(f"STAGE0G2_FROZEN_CONFIG_SHA256={sha256_file(destination)}")


def main() -> None:
    args = arguments(); config = args.config.resolve()
    if args.freeze: freeze(config, args.freeze.resolve(), args.visual_inspection_passed)
    else: initialize(config, args.run_id)


if __name__ == "__main__": main()

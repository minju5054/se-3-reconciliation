#!/usr/bin/env python3
"""Create and freeze a paired Stage 0-G3 moving-history run."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.lightnav_adapter import save_json_exclusive, save_npy_exclusive
from reconciliation.stage0g_lightnav_scene_qualification import create_run_directory_exclusive
from reconciliation.stage0g3_moving_history_qualification import (
    SCENARIO_IDS,
    VARIANT_IDS,
    approach_history,
    load_yaml,
    resolved_scientific_config,
    sha256_file,
    validate_config,
    validate_frozen_control,
    validate_history_poses,
)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/stage0g3_moving_history_qualification.yaml")
    parser.add_argument("--run-id")
    parser.add_argument("--freeze", type=Path)
    parser.add_argument("--visual-inspection-passed", action="store_true")
    return parser.parse_args()


def resolve(value: str) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (ROOT / path).resolve()


def git(*args: str) -> str:
    return subprocess.check_output(["git", "-C", str(ROOT), *args], text=True).strip()


def initialize(config_path: Path, run_id: str | None) -> Path:
    config = load_yaml(config_path)
    validate_config(config, ROOT)
    control = validate_frozen_control(config, ROOT)
    identifier = run_id or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run = resolve(str(config["paths"]["output_root"])) / identifier
    create_run_directory_exclusive(run)
    save_json_exclusive(run / "stage0g2_control_reference.json", {
        **control,
        "validated_utc": datetime.now(timezone.utc).isoformat(),
        "control_was_not_regenerated": True,
        "control_artifacts_are_read_only_inputs": True,
    })
    maximum = float(config["history"]["maximum_common_distance_m"])
    proposed_root = run / "proposed_histories" / f"distance_{maximum:.3f}m"
    for scenario_id in SCENARIO_IDS:
        for variant_id in VARIANT_IDS:
            key = f"{scenario_id}/{variant_id}"
            final = control["cases"][key]["final_pose_se2"]
            poses = approach_history(final, maximum)
            validate_history_poses(poses, final, maximum)
            save_npy_exclusive(proposed_root / scenario_id / variant_id / "history_pose_se2.npy", poses)
    save_json_exclusive(run / "run_metadata.json", {
        "stage": config["stage"],
        "run_id": identifier,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "research_git_sha_at_start": git("rev-parse", "HEAD"),
        "config_frozen": False,
        "primary_output_count": 0,
        "single_changed_variable": "stationary 64-frame visual history -> deterministic moving egocentric 64-frame visual history",
        "scripted_history_is_not_controller_execution": True,
    })
    print(f"STAGE0G3_RUN_DIRECTORY={run}")
    print(f"STAGE0G3_CONTROL_CASES={control['case_count']} config_sha256={control['config_sha256']}")
    print(f"STAGE0G3_PROPOSED_DISTANCE_M={maximum:.3f}")
    return run


def freeze(config_path: Path, run: Path, visual_inspection_passed: bool) -> None:
    config = load_yaml(config_path)
    validate_config(config, ROOT)
    control = validate_frozen_control(config, ROOT)
    feasibility_path = run / "approach_feasibility.json"
    if not feasibility_path.is_file():
        raise ValueError("run collision/scene feasibility before freeze")
    feasibility = json.loads(feasibility_path.read_text(encoding="utf-8"))
    if feasibility.get("status") != "FEASIBLE" or feasibility.get("all_30_histories_feasible") is not True:
        raise ValueError("one common Stage 0-G3 approach distance was not feasible")
    selected = float(feasibility["selected_common_distance_m"])
    history = config["history"]
    if not float(history["minimum_common_distance_m"]) <= selected <= float(history["maximum_common_distance_m"]):
        raise ValueError("selected approach distance is outside the frozen policy")
    cases = feasibility.get("selected_case_results", {})
    if set(cases) != {f"{q}/{v}" for q in SCENARIO_IDS for v in VARIANT_IDS} or not all(item.get("feasible") is True for item in cases.values()):
        raise ValueError("approach feasibility does not cover all 30 paired cases")
    if list(run.glob("qualification/*/*/raw/lightnav.npy")) or list(run.glob("qualification/*/*/input/history/frame_*.png")):
        raise ValueError("config must be frozen before RGB capture or LightNav inference")
    if not visual_inspection_passed:
        raise ValueError("freeze requires explicit feasibility-path visual inspection")
    destination = run / "config_snapshot.yaml"
    with destination.open("xb") as stream:
        stream.write(config_path.read_bytes())
    resolved = resolved_scientific_config(config, ROOT)
    save_json_exclusive(run / "resolved_scientific_contract.json", {
        "stage": config["stage"],
        "single_changed_variable": "history.stationary_history true -> moving_scripted",
        "g2_config_sha256": control["config_sha256"],
        "selected_common_distance_m": selected,
        "robot": resolved["robot"],
        "environment": resolved["environment"],
        "camera": resolved["camera"],
        "capture": resolved["capture"],
        "lightnav": resolved["lightnav"],
        "baseline": resolved["baseline"],
        "scenarios": resolved["scenarios"],
        "variants": resolved["variants"],
    })
    save_json_exclusive(run / "config_freeze.json", {
        "frozen_utc": datetime.now(timezone.utc).isoformat(),
        "config_sha256": sha256_file(destination),
        "g2_config_sha256": control["config_sha256"],
        "g2_control_reference_sha256": sha256_file(run / "stage0g2_control_reference.json"),
        "approach_feasibility_sha256": sha256_file(feasibility_path),
        "selected_common_distance_m": selected,
        "primary_outputs_at_freeze": 0,
        "visual_approach_path_inspection_passed": True,
        "prohibition": "No pose, approach, camera, scene, instruction, model, variant, or threshold tuning after freeze.",
    })
    print(f"STAGE0G3_FROZEN_CONFIG_SHA256={sha256_file(destination)}")
    print(f"STAGE0G3_SELECTED_COMMON_DISTANCE_M={selected:.3f}")


def main() -> None:
    args = arguments()
    config = args.config.resolve()
    if args.freeze:
        freeze(config, args.freeze.resolve(), args.visual_inspection_passed)
    else:
        initialize(config, args.run_id)


if __name__ == "__main__":
    main()

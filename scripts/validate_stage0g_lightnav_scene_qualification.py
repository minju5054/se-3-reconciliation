#!/usr/bin/env python3
"""Strictly validate a completed Stage 0-G input/scene qualification run."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from PIL import Image
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.lightnav_adapter import save_json_exclusive
from reconciliation.stage0g_lightnav_scene_qualification import FINAL_STATUSES, SCENARIO_IDS, VARIANT_IDS, load_json, sha256_file, validate_case, validate_config


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    args = parser.parse_args()
    run = args.run_directory.resolve()
    with (run / "config_snapshot.yaml").open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("config snapshot must be a mapping")
    validate_config(config)
    contract = load_json(run / "input_contract/lightnav_input_contract.json")
    if not isinstance(contract.get("checks"), dict) or not all(contract["checks"].values()):
        raise ValueError("input contract has unresolved checks")
    freeze = load_json(run / "config_freeze.json")
    config_hash = sha256_file(run / "config_snapshot.yaml")
    if config_hash != freeze.get("config_sha256"):
        raise ValueError("frozen config hash mismatch")
    for scenario_id in SCENARIO_IDS:
        preview = run / "scene_previews" / scenario_id / "V0/latest_rgb.png"
        with Image.open(preview) as image:
            image.verify()
    case_paths = list(run.glob("qualification/Q*/V*/raw/lightnav.npy"))
    if len(case_paths) != 30:
        raise ValueError(f"expected exactly 30 primary outputs, found {len(case_paths)}")
    validations = {}
    for scenario_id in SCENARIO_IDS:
        for variant_id in VARIANT_IDS:
            case = run / "qualification" / scenario_id / variant_id
            validations[f"{scenario_id}/{variant_id}"] = validate_case(case, int(config["lightnav"]["expected_horizon"]))
            for figure in ("camera_and_trajectory.png", "trajectory_world_xy.png"):
                with Image.open(case / figure) as image:
                    image.verify()
    for name in ("all_scenarios_camera_and_trajectory.png", "all_variants_by_scenario.png", "pairwise_trajectory_distance.png"):
        with Image.open(run / "overview" / name) as image:
            image.verify()
    summary = load_json(run / "summary.json")
    result = load_json(run / "qualification_result.json")
    if summary.get("primary_output_count") != 30:
        raise ValueError("summary primary output count is not 30")
    if result.get("status") not in FINAL_STATUSES:
        raise ValueError("invalid Stage 0-G final status")
    validation = {
        "valid": True, "stage": config["stage"], "config_sha256": config_hash,
        "input_contract_resolved": contract["contract_resolved"], "preview_count": 6,
        "primary_output_count": len(validations), "case_validations": validations,
        "summary_status": result["status"], "no_waypoint_intrinsic_time_base": True,
        "raw_outputs_immutable_and_hash_verified": True,
    }
    save_json_exclusive(run / "validation.json", validation)
    print(f"STAGE0G_VALIDATION=PASS cases={len(validations)} status={result['status']}")


if __name__ == "__main__":
    main()

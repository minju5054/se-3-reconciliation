#!/usr/bin/env python3
"""Strictly validate a completed Stage 0-G2 qualification run."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np
from PIL import Image
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.lightnav_adapter import save_json_exclusive
from reconciliation.stage0g2_jackal_domain_scene_qualification import (
    FINAL_STATUSES, SCENARIO_IDS, VARIANT_IDS, dominant_exact_raw_fraction,
    load_json, sha256_file, validate_asset_manifest, validate_case, validate_config, validate_preview_gate,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument("run_directory", type=Path); parser.add_argument("--check-only", action="store_true", help="validate without creating validation.json"); args = parser.parse_args(); run = args.run_directory.resolve()
    with (run / "config_snapshot.yaml").open(encoding="utf-8") as stream: config = yaml.safe_load(stream)
    validate_config(config); freeze = load_json(run / "config_freeze.json")
    if sha256_file(run / "config_snapshot.yaml") != freeze.get("config_sha256"): raise ValueError("frozen config hash mismatch")
    contract = load_json(run / "input_contract/lightnav_input_contract.json")
    if not contract.get("contract_resolved") or not all(contract.get("checks", {}).values()): raise ValueError("input contract unresolved")
    selection = load_json(run / "environment_selection.json"); manifest = load_json(run / "environment_asset_manifest.json")
    validate_asset_manifest(manifest)
    if not selection.get("selected_before_primary_inference") or selection.get("selected_family") != config["environment"]["family"]: raise ValueError("environment selection provenance mismatch")
    if not manifest.get("audit_before_primary_inference") or len(manifest.get("candidates", [])) < 4: raise ValueError("asset audit manifest incomplete")
    preview = load_json(run / "scene_previews/preview_gate.json")
    if not preview.get("all_six_pass") or set(preview.get("records", {})) != set(SCENARIO_IDS): raise ValueError("preview gate incomplete")
    for q in SCENARIO_IDS:
        validate_preview_gate(preview["records"][q]); metadata = load_json(run / "scene_previews" / q / "V0/preview_metadata.json")
        if metadata.get("initial_pose_collision_free") is not True: raise ValueError(f"collision gate failed for {q}")
        with Image.open(run / "scene_previews" / q / "V0/latest_rgb.png") as image: image.verify()
    named = {}; validations = {}
    for q in SCENARIO_IDS:
        for v in VARIANT_IDS:
            case = run / "qualification" / q / v; validations[f"{q}/{v}"] = validate_case(case, int(config["lightnav"]["expected_horizon"])); named[f"{q}/{v}"] = np.load(case / "raw/lightnav.npy", allow_pickle=False)
            for figure in ("camera_and_trajectory.png", "trajectory_world_xy.png"):
                with Image.open(case / figure) as image: image.verify()
    if len(named) != 30: raise ValueError("expected exactly 30 primary outputs")
    for name in ("all_scenarios_camera_and_trajectory.png", "all_variants_by_scenario.png", "stage0g_vs_stage0g2_comparison.png", "pairwise_trajectory_distance.png"):
        with Image.open(run / "overview" / name) as image: image.verify()
    summary = load_json(run / "summary.json"); result = load_json(run / "qualification_result.json"); comparison = load_json(run / "stage0g_vs_stage0g2_summary.json")
    if summary.get("primary_output_count") != 30 or result.get("status") not in FINAL_STATUSES: raise ValueError("summary/result invalid")
    actual_dominant = dominant_exact_raw_fraction(named)
    if not np.isclose(float(summary["dominant_exact_raw_fraction"]), actual_dominant): raise ValueError("dominant fraction mismatch")
    if not comparison.get("comparison_is_descriptive_only") or not comparison.get("no_causal_statistical_claim"): raise ValueError("comparison claim limit missing")
    validation = {"valid": True, "stage": config["stage"], "config_sha256": freeze["config_sha256"], "preview_count": 6, "primary_output_count": 30,
                  "case_validations": validations, "summary_status": result["status"], "selected_environment": selection["selected_family"], "jackal_required": True,
                  "no_waypoint_intrinsic_time_base": True, "raw_outputs_immutable_and_hash_verified": True, "comparison_descriptive_only": True}
    if not args.check_only: save_json_exclusive(run / "validation.json", validation)
    print(f"STAGE0G2_VALIDATION=PASS cases=30 status={result['status']} check_only={str(args.check_only).lower()}")


if __name__ == "__main__": main()

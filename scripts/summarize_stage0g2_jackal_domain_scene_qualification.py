#!/usr/bin/env python3
"""Summarize Stage 0-G2 and compare it descriptively with frozen Stage 0-G."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.lightnav_adapter import save_json_exclusive
from reconciliation.stage0g2_jackal_domain_scene_qualification import (
    SCENARIO_IDS, VARIANT_IDS, dominant_exact_raw_fraction, doorway_distinct_status,
    duplicate_groups, load_json, pairwise_trajectory_distance, sha256_file,
    qualification_status, validate_case, validate_config,
)


def local_plot(axis, trajectory: np.ndarray, title: str) -> None:
    points = np.vstack((np.zeros((1, 3)), trajectory)); axis.plot(points[:, 0], points[:, 1], "o-", color="#06a9c8", linewidth=2, markersize=3)
    axis.scatter([0], [0], marker="s", color="#1b9e77"); axis.scatter([trajectory[-1, 0]], [trajectory[-1, 1]], marker="*", color="#d95f02", s=55)
    axis.axhline(0, color=".7", linewidth=.7); axis.set_aspect("equal", adjustable="datalim"); axis.grid(True, alpha=.3); axis.set_title(title, fontsize=8)
    axis.set_xlabel("forward +x [m]"); axis.set_ylabel("left +y [m]")


def case_figures(case: Path, name: str, raw: np.ndarray, world: np.ndarray) -> None:
    figure, axes = plt.subplots(1, 2, figsize=(10, 4.2), constrained_layout=True); axes[0].imshow(Image.open(case / "input/latest_rgb.png")); axes[0].axis("off"); axes[0].set_title("frozen RGB")
    local_plot(axes[1], raw, name); figure.savefig(case / "camera_and_trajectory.png", dpi=140); plt.close(figure)
    figure, axis = plt.subplots(figsize=(5, 4.5), constrained_layout=True); axis.plot(world[:, 0], world[:, 1], "o-", color="#6a3d9a"); axis.set_aspect("equal", adjustable="datalim"); axis.grid(True, alpha=.3); axis.set_title(name + " world")
    axis.set_xlabel("world x [m]"); axis.set_ylabel("world y [m]"); figure.savefig(case / "trajectory_world_xy.png", dpi=140); plt.close(figure)


def stage0g_reference(config: dict) -> tuple[dict, dict[str, list[dict]]]:
    reference = (ROOT / str(config["paths"]["stage0g_reference_run"])).resolve(); summary = load_json(reference / "summary.json"); per = defaultdict(list); hashes = []
    for path in sorted(reference.glob("qualification/Q*/V*/descriptors.json")):
        d = load_json(path); p = load_json(path.parent / "provenance.json"); d["raw_sha256"] = p["raw_lightnav_sha256"]; per[path.parent.parent.name].append(d); hashes.append(p["raw_lightnav_sha256"])
    summary = dict(summary); summary["dominant_exact_raw_fraction"] = max(Counter(hashes).values()) / len(hashes)
    return summary, per


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument("run_directory", type=Path); parser.add_argument("--visual-inspection-passed", action="store_true"); args = parser.parse_args(); run = args.run_directory.resolve()
    with (run / "config_snapshot.yaml").open(encoding="utf-8") as stream: config = yaml.safe_load(stream)
    validate_config(config); criteria = config["qualification"]; scenarios = {item["id"]: item for item in config["scenarios"]}
    named = {}; per = defaultdict(list); records = []
    for q in SCENARIO_IDS:
        for v in VARIANT_IDS:
            case = run / "qualification" / q / v; validate_case(case, int(config["lightnav"]["expected_horizon"])); raw = np.load(case / "raw/lightnav.npy", allow_pickle=False); world = np.load(case / "derived/trajectory_world.npy", allow_pickle=False)
            d = load_json(case / "descriptors.json"); p = load_json(case / "provenance.json"); record = {"case": f"{q}/{v}", "scenario_id": q, "variant_id": v, "raw_sha256": p["raw_lightnav_sha256"], **d}
            named[record["case"]] = raw; per[q].append(record); records.append(record); case_figures(case, record["case"], raw, world)
    overview = run / "overview"; overview.mkdir(exist_ok=False)
    figure, axes = plt.subplots(6, 2, figsize=(12, 19), constrained_layout=True)
    for row, q in enumerate(SCENARIO_IDS):
        axes[row, 0].imshow(Image.open(run / "qualification" / q / "V0/input/latest_rgb.png")); axes[row, 0].axis("off"); axes[row, 0].set_title(q + ": " + scenarios[q]["instruction"], fontsize=8); local_plot(axes[row, 1], named[f"{q}/V0"], "V0 output")
    figure.savefig(overview / "all_scenarios_camera_and_trajectory.png", dpi=140); plt.close(figure)
    figure, axes = plt.subplots(6, 5, figsize=(18, 19), constrained_layout=True)
    for row, q in enumerate(SCENARIO_IDS):
        for col, v in enumerate(VARIANT_IDS): local_plot(axes[row, col], named[f"{q}/{v}"], f"{q}/{v}\nmatch={per[q][col]['matches_intended_geometry']}")
    figure.savefig(overview / "all_variants_by_scenario.png", dpi=140); plt.close(figure)
    pairwise = pairwise_trajectory_distance(named); save_json_exclusive(overview / "pairwise_trajectory_distance.json", pairwise)
    matrix = np.asarray(pairwise["matrix"]); figure, axis = plt.subplots(figsize=(10, 9), constrained_layout=True); im = axis.imshow(matrix, cmap="viridis"); figure.colorbar(im, ax=axis); axis.set_title("G2 pairwise descriptive trajectory distance"); figure.savefig(overview / "pairwise_trajectory_distance.png", dpi=140); plt.close(figure)
    scenario_summary = {}
    for q in SCENARIO_IDS:
        values = per[q]; scenario_summary[q] = {
            "valid_count": sum(bool(x["valid"]) for x in values), "matching_intended_geometry_count": sum(bool(x["matches_intended_geometry"]) for x in values),
            "unique_raw_output_count": len({x["raw_sha256"] for x in values}), "stop_or_nonmoving_count": sum(bool(x["nonmoving"]) for x in values),
            "endpoint_lateral_range_m": [min(x["endpoint_lateral_m"] for x in values), max(x["endpoint_lateral_m"] for x in values)],
            "signed_net_yaw_range_rad": [min(x["signed_net_yaw_rad"] for x in values), max(x["signed_net_yaw_rad"] for x in values)],
        }; scenario_summary[q]["qualified"] = scenario_summary[q]["matching_intended_geometry_count"] >= int(criteria["required_matches_per_scenario"])
    hashes = {x["raw_sha256"] for x in records}; signatures = {x["geometry_signature"] for x in records}; dominant = dominant_exact_raw_fraction(named)
    doorway = {x["raw_sha256"] for x in per["G2_Q3_DOORWAY"]}; straight = {x["raw_sha256"] for x in per["G2_Q0_STRAIGHT"]}; unrelated = {x["raw_sha256"] for q in ("G2_Q1_LEFT_TURN", "G2_Q2_RIGHT_TURN") for x in per[q]}
    distinct = doorway_distinct_status(doorway, straight, unrelated)
    summary = {"stage": config["stage"], "config_sha256": sha256_file(run / "config_snapshot.yaml"), "primary_output_count": len(records), "scenario_results": scenario_summary,
               "unique_raw_output_count": len(hashes), "dominant_exact_raw_fraction": dominant, "unique_geometry_signature_count": len(signatures), "geometry_signatures": sorted(signatures),
               "exact_duplicate_groups": duplicate_groups(named), "doorway_distinct_status": distinct, "visual_inspection": {"all_six_v0_inspected": bool(args.visual_inspection_passed), "three_overviews_inspected": bool(args.visual_inspection_passed)}}
    old, old_per = stage0g_reference(config); old_ids = ("Q0_STRAIGHT", "Q1_LEFT_TURN", "Q2_RIGHT_TURN", "Q3_DOORWAY", "Q4_DETOUR_LEFT", "Q5_DETOUR_RIGHT")
    comparison = {"comparison_is_descriptive_only": True, "no_causal_statistical_claim": True,
                  "unique_raw_output_count": {"stage0g": old["unique_raw_output_count"], "stage0g2": len(hashes)},
                  "dominant_exact_raw_fraction": {"stage0g": old["dominant_exact_raw_fraction"], "stage0g2": dominant},
                  "unique_geometry_signature_count": {"stage0g": old["unique_geometry_signature_count"], "stage0g2": len(signatures)}, "scenario_comparison": {}}
    for old_id, q in zip(old_ids, SCENARIO_IDS):
        comparison["scenario_comparison"][q] = {"stage0g_match_count": old["scenario_results"][old_id]["matching_intended_geometry_count"], "stage0g2_match_count": scenario_summary[q]["matching_intended_geometry_count"],
            "stage0g_endpoint_lateral_range_m": [min(x["endpoint_lateral_m"] for x in old_per[old_id]), max(x["endpoint_lateral_m"] for x in old_per[old_id])], "stage0g2_endpoint_lateral_range_m": scenario_summary[q]["endpoint_lateral_range_m"],
            "stage0g_signed_net_yaw_range_rad": [min(x["signed_net_yaw_rad"] for x in old_per[old_id]), max(x["signed_net_yaw_rad"] for x in old_per[old_id])], "stage0g2_signed_net_yaw_range_rad": scenario_summary[q]["signed_net_yaw_range_rad"]}
    save_json_exclusive(run / "stage0g_vs_stage0g2_summary.json", comparison)
    figure, axes = plt.subplots(1, 3, figsize=(13, 4), constrained_layout=True); axes[0].bar(["G", "G2"], [old["unique_raw_output_count"], len(hashes)]); axes[0].set_title("unique exact raw"); axes[1].bar(["G", "G2"], [old["dominant_exact_raw_fraction"], dominant]); axes[1].set_title("dominant fraction"); axes[2].bar(["G", "G2"], [old["unique_geometry_signature_count"], len(signatures)]); axes[2].set_title("geometry signatures"); figure.suptitle("Descriptive only; no causal statistical claim"); figure.savefig(overview / "stage0g_vs_stage0g2_comparison.png", dpi=150); plt.close(figure)
    conditions = {"preview_gate_passed": load_json(run / "scene_previews/preview_gate.json")["all_six_pass"], "minimum_five_unique_raw_outputs": len(hashes) >= int(criteria["global"]["minimum_unique_raw_outputs"]),
                  "dominant_exact_raw_group_within_limit": dominant <= float(criteria["global"]["maximum_dominant_exact_raw_fraction"]), "left_turn_qualifies": scenario_summary["G2_Q1_LEFT_TURN"]["qualified"],
                  "right_turn_qualifies": scenario_summary["G2_Q2_RIGHT_TURN"]["qualified"], "doorway_or_detour_qualifies": any(scenario_summary[q]["qualified"] for q in SCENARIO_IDS[3:]),
                  "doorway_distinct_intent_demonstrated": distinct == "DISTINCT_INTENT_DEMONSTRATED", "no_invalid_stop_dominance": sum(x["nonmoving"] for x in records) <= int(criteria["global"]["maximum_stop_or_nonmoving_count"]), "visual_inspection_complete": bool(args.visual_inspection_passed)}
    status = qualification_status(conditions)
    save_json_exclusive(run / "summary.json", summary); save_json_exclusive(run / "qualification_result.json", {"status": status, "conditions": conditions, "claim_boundary": "Scene/input qualification only; descriptive comparison cannot establish causality or navigation performance.", "next_step_if_ready": "Collect genuine same-episode successive OLD/FRESH transitions with frozen Stage 0-G2."})
    print(f"STAGE0G2_PRIMARY_OUTPUTS={len(records)} unique={len(hashes)} dominant={dominant:.3f} signatures={len(signatures)}")
    for q in SCENARIO_IDS: print(f"STAGE0G2_SCENARIO {q} match={scenario_summary[q]['matching_intended_geometry_count']}/5 unique={scenario_summary[q]['unique_raw_output_count']} qualified={scenario_summary[q]['qualified']}")
    print(status)


if __name__ == "__main__": main()

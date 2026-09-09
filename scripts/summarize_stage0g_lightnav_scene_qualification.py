#!/usr/bin/env python3
"""Summarize and visualize a complete frozen Stage 0-G qualification run."""

from __future__ import annotations

import argparse
from collections import defaultdict
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.lightnav_adapter import save_json_exclusive
from reconciliation.stage0g_lightnav_scene_qualification import (
    SCENARIO_IDS, VARIANT_IDS, duplicate_groups, geometry_signature, load_json,
    pairwise_trajectory_distance, sha256_file, validate_case, validate_config,
)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--visual-inspection-passed", action="store_true")
    return parser.parse_args()


def local_plot(axis, trajectory: np.ndarray, title: str) -> None:
    points = np.vstack((np.zeros((1, 3)), trajectory))
    axis.plot(points[:, 0], points[:, 1], "o-", color="#06a9c8", linewidth=2, markersize=3)
    axis.scatter([0.0], [0.0], marker="s", color="#1b9e77", s=35, label="observation")
    axis.scatter([trajectory[-1, 0]], [trajectory[-1, 1]], marker="*", color="#d95f02", s=65, label="endpoint")
    for pose in trajectory[::2]:
        axis.arrow(pose[0], pose[1], 0.08 * np.cos(pose[2]), 0.08 * np.sin(pose[2]), width=0.004, color="#e6ab02")
    axis.axhline(0.0, color="0.75", linewidth=0.7)
    axis.set_xlabel("forward +x [m]"); axis.set_ylabel("lateral-left +y [m]")
    axis.set_aspect("equal", adjustable="datalim"); axis.grid(True, alpha=0.3); axis.set_title(title, fontsize=9)


def save_case_figures(case: Path, scenario_id: str, variant_id: str, raw: np.ndarray, world: np.ndarray) -> None:
    rgb = np.asarray(Image.open(case / "input/latest_rgb.png").convert("RGB"))
    figure, axes = plt.subplots(1, 2, figsize=(10, 4.2), constrained_layout=True)
    axes[0].imshow(rgb); axes[0].set_title("final stationary RGB input"); axes[0].axis("off")
    local_plot(axes[1], raw, f"{scenario_id}/{variant_id} LightNav local output")
    figure.savefig(case / "camera_and_trajectory.png", dpi=150); plt.close(figure)
    figure, axis = plt.subplots(figsize=(5.5, 5.0), constrained_layout=True)
    axis.plot(world[:, 0], world[:, 1], "o-", color="#6a3d9a", linewidth=2)
    axis.scatter(world[0, 0], world[0, 1], marker="s", color="#1b9e77", label="first decoded pose")
    axis.scatter(world[-1, 0], world[-1, 1], marker="*", color="#d95f02", s=80, label="endpoint")
    axis.set_xlabel("world x [m]"); axis.set_ylabel("world y [m]"); axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, alpha=0.3); axis.legend(); axis.set_title(f"{scenario_id}/{variant_id}: observation-pose anchored")
    figure.savefig(case / "trajectory_world_xy.png", dpi=150); plt.close(figure)


def main() -> None:
    options = arguments()
    run = options.run_directory.resolve()
    with (run / "config_snapshot.yaml").open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("config snapshot must be a mapping")
    validate_config(config)
    criteria = config["qualification"]
    scenario_configs = {item["id"]: item for item in config["scenarios"]}
    named_raw = {}
    records = []
    per_scenario: dict[str, list[dict]] = defaultdict(list)
    for scenario_id in SCENARIO_IDS:
        for variant_id in VARIANT_IDS:
            case = run / "qualification" / scenario_id / variant_id
            validate_case(case, expected_horizon=int(config["lightnav"]["expected_horizon"]))
            raw = np.load(case / "raw/lightnav.npy", allow_pickle=False)
            world = np.load(case / "derived/trajectory_world.npy", allow_pickle=False)
            descriptor = load_json(case / "descriptors.json")
            provenance = load_json(case / "provenance.json")
            name = f"{scenario_id}/{variant_id}"
            named_raw[name] = raw
            record = {"case": name, "scenario_id": scenario_id, "variant_id": variant_id, "raw_sha256": provenance["raw_lightnav_sha256"], **descriptor}
            records.append(record); per_scenario[scenario_id].append(record)
            save_case_figures(case, scenario_id, variant_id, raw, world)

    overview = run / "overview"
    overview.mkdir(exist_ok=False)
    figure, axes = plt.subplots(6, 2, figsize=(12, 19), constrained_layout=True)
    for row, scenario_id in enumerate(SCENARIO_IDS):
        case = run / "qualification" / scenario_id / "V0"
        axes[row, 0].imshow(Image.open(case / "input/latest_rgb.png")); axes[row, 0].axis("off")
        axes[row, 0].set_title(f"{scenario_id} V0: {scenario_configs[scenario_id]['instruction']}", fontsize=9)
        local_plot(axes[row, 1], named_raw[f"{scenario_id}/V0"], "V0 local trajectory")
    figure.savefig(overview / "all_scenarios_camera_and_trajectory.png", dpi=140); plt.close(figure)
    figure, axes = plt.subplots(6, 5, figsize=(18, 19), constrained_layout=True)
    for row, scenario_id in enumerate(SCENARIO_IDS):
        for column, variant_id in enumerate(VARIANT_IDS):
            record = next(item for item in per_scenario[scenario_id] if item["variant_id"] == variant_id)
            local_plot(axes[row, column], named_raw[f"{scenario_id}/{variant_id}"], f"{scenario_id}/{variant_id}\nmatch={record['matches_intended_geometry']}")
    figure.savefig(overview / "all_variants_by_scenario.png", dpi=140); plt.close(figure)

    pairwise = pairwise_trajectory_distance(named_raw)
    save_json_exclusive(overview / "pairwise_trajectory_distance.json", pairwise)
    matrix = np.asarray(pairwise["matrix"])
    figure, axis = plt.subplots(figsize=(12, 10), constrained_layout=True)
    image = axis.imshow(matrix, cmap="viridis"); figure.colorbar(image, ax=axis, label="descriptive mixed-unit RMS")
    axis.set_title("Pairwise LightNav trajectory distance"); axis.set_xlabel("case index"); axis.set_ylabel("case index")
    figure.savefig(overview / "pairwise_trajectory_distance.png", dpi=140); plt.close(figure)

    scenario_summary = {}
    for scenario_id in SCENARIO_IDS:
        values = per_scenario[scenario_id]
        numerical = ("path_length_m", "endpoint_forward_m", "endpoint_lateral_m", "max_left_lateral_excursion_m", "max_right_lateral_excursion_m", "signed_net_yaw_rad")
        scenario_summary[scenario_id] = {
            "valid_count": sum(bool(item["valid"]) for item in values),
            "matching_intended_geometry_count": sum(bool(item["matches_intended_geometry"]) for item in values),
            "stop_or_nonmoving_count": sum(bool(item["nonmoving"]) for item in values),
            "unique_raw_output_count": len({item["raw_sha256"] for item in values}),
            "descriptor_ranges": {name: [float(min(item[name] for item in values)), float(max(item[name] for item in values))] for name in numerical},
        }
        scenario_summary[scenario_id]["qualified"] = scenario_summary[scenario_id]["matching_intended_geometry_count"] >= int(criteria["required_matches_per_scenario"])
    hashes = {record["raw_sha256"] for record in records}
    signatures = {geometry_signature(record) for record in records}
    duplicates = duplicate_groups(named_raw)
    input_contract = load_json(run / "input_contract/lightnav_input_contract.json")
    q = scenario_summary
    stop_count = sum(item["stop_or_nonmoving_count"] for item in q.values())
    straight_hashes = {item["raw_sha256"] for item in per_scenario["Q0_STRAIGHT"]}
    nonstraight_hashes = {item["raw_sha256"] for scenario in SCENARIO_IDS[1:] for item in per_scenario[scenario]}
    conditions = {
        "input_contract_resolved": bool(input_contract["contract_resolved"]),
        "camera_input_profile_frozen": sha256_file(run / "config_snapshot.yaml") == load_json(run / "config_freeze.json")["config_sha256"],
        "q0_straight_qualifies": bool(q["Q0_STRAIGHT"]["qualified"]),
        "left_turn_family_qualifies": bool(q["Q1_LEFT_TURN"]["qualified"]),
        "right_turn_family_qualifies": bool(q["Q2_RIGHT_TURN"]["qualified"]),
        "detour_or_doorway_family_qualifies": any(q[name]["qualified"] for name in SCENARIO_IDS[3:]),
        "no_systematic_invalid_stop_behavior": all(item["valid_count"] == 5 for item in q.values()) and stop_count <= int(criteria["global"]["maximum_stop_or_nonmoving_count"]),
        "straight_not_identical_to_all_nonstraight": not nonstraight_hashes.issubset(straight_hashes),
        "minimum_five_unique_raw_outputs": len(hashes) >= int(criteria["global"]["minimum_unique_raw_outputs"]),
        "reproducible_left_geometry": bool(q["Q1_LEFT_TURN"]["qualified"]),
        "reproducible_right_geometry": bool(q["Q2_RIGHT_TURN"]["qualified"]),
        "reproducible_doorway_or_detour_geometry": any(q[name]["qualified"] for name in SCENARIO_IDS[3:]),
        "scenes_visually_legible": bool(options.visual_inspection_passed),
    }
    diversity_pass = all(conditions[name] for name in (
        "straight_not_identical_to_all_nonstraight", "minimum_five_unique_raw_outputs",
        "reproducible_left_geometry", "reproducible_right_geometry", "reproducible_doorway_or_detour_geometry",
    ))
    conditions["cross_scenario_diversity_passes"] = diversity_pass
    ready = all(conditions.values())
    status = "STAGE0G_READY_FOR_DATA_COLLECTION" if ready else (
        "STAGE0G_INPUT_CONTRACT_NOT_RESOLVED" if not input_contract["contract_resolved"] else "STAGE0G_SCENE_QUALIFICATION_FAILED"
    )
    summary = {
        "stage": config["stage"], "config_sha256": sha256_file(run / "config_snapshot.yaml"),
        "primary_output_count": len(records), "maximum_primary_output_count": 30,
        "scenario_results": scenario_summary, "stop_or_nonmoving_count": stop_count,
        "unique_raw_output_count": len(hashes), "unique_geometry_signature_count": len(signatures),
        "geometry_signatures": sorted(signatures), "exact_duplicate_groups": duplicates,
        "pairwise_trajectory_distance": "overview/pairwise_trajectory_distance.json",
        "visual_inspection": {"all_six_v0_inspected": bool(options.visual_inspection_passed), "both_overviews_inspected": bool(options.visual_inspection_passed)},
        "criteria_are_engineering_qualification_only": True,
    }
    result = {
        "status": status, "conditions": conditions,
        "claim_boundary": "Scene/input qualification only; no navigation, controller, reconciliation, or real-world performance claim.",
        "next_step_if_ready": "Collect genuine same-episode successive OLD/FRESH transitions using this frozen Stage0G-qualified input/scene setup.",
    }
    save_json_exclusive(run / "summary.json", summary)
    save_json_exclusive(run / "qualification_result.json", result)
    print(f"STAGE0G_PRIMARY_OUTPUTS={len(records)}")
    print(f"STAGE0G_UNIQUE_RAW_OUTPUTS={len(hashes)}")
    for scenario_id in SCENARIO_IDS:
        value = scenario_summary[scenario_id]
        print(f"STAGE0G_SCENARIO {scenario_id} match={value['matching_intended_geometry_count']}/5 stop={value['stop_or_nonmoving_count']} unique={value['unique_raw_output_count']} qualified={value['qualified']}")
    print(status)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Summarize paired stationary-versus-moving Stage 0-G3 qualification."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.lightnav_adapter import save_json_exclusive
from reconciliation.stage0g3_moving_history_qualification import (
    SCENARIO_IDS,
    QUALIFICATION_CONDITION_KEYS,
    VARIANT_IDS,
    class_transition,
    dominant_exact_raw_fraction,
    load_yaml,
    paired_trajectory_metrics,
    qualification_status,
    sha256_file,
    validate_config,
    validate_frozen_control,
    validate_history_poses,
)


CONTACT_FRAMES = (0, 16, 32, 48, 63)


def args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--visual-inspection-passed", action="store_true")
    return parser.parse_args()


def local_plot(axis, stationary: np.ndarray, moving: np.ndarray, title: str, legend: bool = False) -> None:
    origin = np.zeros((1, 3), dtype=float)
    old = np.vstack((origin, stationary)); new = np.vstack((origin, moving))
    axis.plot(old[:, 0], old[:, 1], "o-", color="#06a9c8", linewidth=1.5, markersize=2.5, label="G2 stationary")
    axis.plot(new[:, 0], new[:, 1], "o-", color="#e7298a", linewidth=1.8, markersize=2.5, label="G3 moving")
    axis.scatter([0], [0], marker="s", color="#1b9e77", s=25)
    axis.axhline(0, color=".75", linewidth=.6); axis.grid(True, alpha=.25)
    axis.set_aspect("equal", adjustable="datalim"); axis.set_title(title, fontsize=8)
    axis.set_xlabel("forward [m]"); axis.set_ylabel("left [m]")
    if legend: axis.legend(fontsize=7)


def contact_sheet(case: Path, title: str, destination: Path) -> None:
    images = [Image.open(case / f"input/history/frame_{index:06d}.png").convert("RGB") for index in CONTACT_FRAMES]
    figure, axes = plt.subplots(1, 5, figsize=(15, 2.5), constrained_layout=True)
    for axis, image, index in zip(axes, images, CONTACT_FRAMES):
        axis.imshow(image); axis.axis("off"); axis.set_title(f"frame {index}\nt={index / 4:.2f}s", fontsize=8)
    figure.suptitle(title + " — scripted moving egocentric history")
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=130); plt.close(figure)


def distribution(values: list[float]) -> dict[str, float]:
    array = np.asarray(values, dtype=np.float64)
    return {"min": float(np.min(array)), "median": float(np.median(array)), "mean": float(np.mean(array)), "max": float(np.max(array))}


def main() -> None:
    options = args(); run = options.run_directory.resolve()
    config = load_yaml(run / "config_snapshot.yaml"); validate_config(config, ROOT)
    freeze = json.loads((run / "config_freeze.json").read_text(encoding="utf-8"))
    if sha256_file(run / "config_snapshot.yaml") != freeze.get("config_sha256"):
        raise ValueError("frozen Stage 0-G3 config hash mismatch")
    control = validate_frozen_control(config, ROOT); g2_run = Path(control["primary_run"])
    g2_summary = json.loads((g2_run / "summary.json").read_text(encoding="utf-8"))
    selected_distance = float(freeze["selected_common_distance_m"])
    named_moving = {}; per_scenario = defaultdict(list); records = []
    final_translation_errors = []; final_yaw_errors = []; history_distances = []
    consecutive_rgb = []; first_final_rgb = []
    for scenario_id in SCENARIO_IDS:
        for variant_id in VARIANT_IDS:
            name = f"{scenario_id}/{variant_id}"; case = run / "qualification" / scenario_id / variant_id
            g2_case = g2_run / "qualification" / scenario_id / variant_id
            stationary = np.load(g2_case / "raw/lightnav.npy", allow_pickle=False)
            moving = np.load(case / "raw/lightnav.npy", allow_pickle=False)
            metrics = paired_trajectory_metrics(stationary, moving)
            g2_descriptor = json.loads((g2_case / "descriptors.json").read_text(encoding="utf-8"))
            g3_descriptor = json.loads((case / "descriptors.json").read_text(encoding="utf-8"))
            transition = class_transition(g2_descriptor["geometry_signature"], g3_descriptor["geometry_signature"])
            capture = json.loads((case / "input/capture_metadata.json").read_text(encoding="utf-8"))
            poses = np.load(case / "input/history_pose_se2.npy", allow_pickle=False)
            motion = validate_history_poses(
                poses, np.asarray(control["cases"][name]["final_pose_se2"]), selected_distance,
                float(config["history"]["final_translation_tolerance_m"]), float(config["history"]["final_yaw_tolerance_rad"]),
            )
            record = {
                "case": name, "scenario_id": scenario_id, "variant_id": variant_id,
                "stationary_raw_sha256": sha256_file(g2_case / "raw/lightnav.npy"),
                "moving_raw_sha256": sha256_file(case / "raw/lightnav.npy"),
                **metrics,
                "stationary_geometry_class": transition["stationary_class"],
                "moving_geometry_class": transition["moving_class"],
                "stationary_intended_match": bool(g2_descriptor["matches_intended_geometry"]),
                "moving_intended_match": bool(g3_descriptor["matches_intended_geometry"]),
                "class_transition": transition["class_transition"],
                "moving_endpoint_lateral_m": float(g3_descriptor["endpoint_lateral_m"]),
                "moving_signed_net_yaw_rad": float(g3_descriptor["signed_net_yaw_rad"]),
                "moving_nonmoving": bool(g3_descriptor["nonmoving"]),
                "final_translation_mismatch_m": float(motion["final_translation_mismatch_m"]),
                "final_yaw_mismatch_rad": float(motion["final_yaw_mismatch_rad"]),
                "history_translation_m": float(motion["history_translation_m"]),
                "mean_interframe_translation_m": float(motion["mean_interframe_translation_m"]),
                "first_to_final_rgb_mae_uint8": float(capture["rgb_diagnostics"]["first_to_final_rgb_mae_uint8"]),
                "mean_consecutive_rgb_mae_uint8": float(capture["rgb_diagnostics"]["mean_consecutive_rgb_mae_uint8"]),
            }
            named_moving[name] = moving; records.append(record); per_scenario[scenario_id].append(record)
            final_translation_errors.append(record["final_translation_mismatch_m"]); final_yaw_errors.append(record["final_yaw_mismatch_rad"])
            history_distances.append(record["history_translation_m"]); consecutive_rgb.append(record["mean_consecutive_rgb_mae_uint8"]); first_final_rgb.append(record["first_to_final_rgb_mae_uint8"])
            save_json_exclusive(case / "paired_metrics.json", record)
            plots = case / "plots"; contact_sheet(case, name, plots / "contact_sheet.png")
            figure, axis = plt.subplots(figsize=(5.2, 4.5), constrained_layout=True); local_plot(axis, stationary, moving, name, True); figure.savefig(plots / "stationary_vs_moving.png", dpi=140); plt.close(figure)
            figure, axis = plt.subplots(figsize=(5.2, 4.5), constrained_layout=True); local_plot(axis, moving, moving, name + " moving output"); axis.lines[0].set_visible(False); figure.savefig(plots / "local_trajectory.png", dpi=140); plt.close(figure)

    hashes = [record["moving_raw_sha256"] for record in records]
    moving_hash_sets = {q: {record["moving_raw_sha256"] for record in per_scenario[q]} for q in SCENARIO_IDS}
    unrelated_doorway = moving_hash_sets[SCENARIO_IDS[0]] | moving_hash_sets[SCENARIO_IDS[1]] | moving_hash_sets[SCENARIO_IDS[2]]
    scenario_summary = {}
    for scenario_id in SCENARIO_IDS:
        values = per_scenario[scenario_id]
        doorway_distinct_count = sum(item["moving_intended_match"] and item["moving_raw_sha256"] not in unrelated_doorway for item in values) if scenario_id == "G2_Q3_DOORWAY" else None
        scenario_summary[scenario_id] = {
            "stage0g2_stationary_intended_matches": int(g2_summary["scenario_results"][scenario_id]["matching_intended_geometry_count"]),
            "stage0g3_moving_intended_matches": sum(item["moving_intended_match"] for item in values),
            "moving_unique_raw_count": len({item["moving_raw_sha256"] for item in values}),
            "moving_endpoint_lateral_range_m": [min(item["moving_endpoint_lateral_m"] for item in values), max(item["moving_endpoint_lateral_m"] for item in values)],
            "moving_signed_net_yaw_range_rad": [min(item["moving_signed_net_yaw_rad"] for item in values), max(item["moving_signed_net_yaw_rad"] for item in values)],
            "moving_stop_or_nonmoving_count": sum(item["moving_nonmoving"] for item in values),
            "doorway_distinct_intended_match_count": doorway_distinct_count,
        }
        scenario_summary[scenario_id]["qualified"] = scenario_summary[scenario_id]["stage0g3_moving_intended_matches"] >= int(config["qualification"]["required_matches_per_scenario"])
    signatures = sorted({json.loads((run / "qualification" / r["scenario_id"] / r["variant_id"] / "descriptors.json").read_text())["geometry_signature"] for r in records})
    dominant = dominant_exact_raw_fraction(named_moving)
    stop_count = sum(record["moving_nonmoving"] for record in records)
    doorway_distinct = int(scenario_summary["G2_Q3_DOORWAY"]["doorway_distinct_intended_match_count"] or 0)
    q = scenario_summary
    conditions = {
        "stage0g2_control_validated": bool(control["valid"]),
        "all_final_observation_poses_equal": max(final_translation_errors) <= float(config["history"]["final_translation_tolerance_m"]) and max(final_yaw_errors) <= float(config["history"]["final_yaw_tolerance_rad"]),
        "all_histories_verified_nonzero_movement": all(value > 0.0 for value in history_distances),
        "q0_straight_at_least_4_of_5": q["G2_Q0_STRAIGHT"]["qualified"],
        "q1_left_at_least_4_of_5": q["G2_Q1_LEFT_TURN"]["qualified"],
        "q2_right_at_least_4_of_5": q["G2_Q2_RIGHT_TURN"]["qualified"],
        "doorway_distinct_or_detour_family_at_least_4_of_5": doorway_distinct >= 4 or q["G2_Q4_DETOUR_LEFT"]["qualified"] or q["G2_Q5_DETOUR_RIGHT"]["qualified"],
        "minimum_five_unique_moving_raw_outputs": len(set(hashes)) >= int(config["qualification"]["minimum_unique_raw_outputs"]),
        "dominant_exact_raw_group_within_limit": dominant <= float(config["qualification"]["maximum_dominant_exact_raw_fraction"]),
        "no_invalid_or_stop_dominance": stop_count <= int(config["qualification"]["maximum_stop_or_nonmoving_count"]),
    }
    if set(conditions) != QUALIFICATION_CONDITION_KEYS:
        raise AssertionError("Stage 0-G3 qualification conditions no longer match frozen A-J contract")
    status = qualification_status(conditions)
    fail_to_pass = sum(not r["stationary_intended_match"] and r["moving_intended_match"] for r in records)
    pass_to_fail = sum(r["stationary_intended_match"] and not r["moving_intended_match"] for r in records)
    same_class = sum(r["stationary_geometry_class"] == r["moving_geometry_class"] for r in records)
    comparison = {
        "stage": config["stage"], "comparison_is_paired_descriptive_only": True,
        "single_changed_variable": "stationary versus deterministic moving egocentric visual history",
        "case_count": 30,
        "exact_raw_changed_count": sum(not r["exact_raw_equal"] for r in records),
        "exact_raw_unchanged_count": sum(r["exact_raw_equal"] for r in records),
        "per_case_hash_change_rate": sum(not r["exact_raw_equal"] for r in records) / 30.0,
        "translation_rms_difference_m_distribution": distribution([r["translation_rms_difference_m"] for r in records]),
        "wrapped_yaw_rms_difference_rad_distribution": distribution([r["wrapped_yaw_rms_difference_rad"] for r in records]),
        "delta_endpoint_forward_m_distribution": distribution([r["delta_endpoint_forward_m"] for r in records]),
        "delta_endpoint_lateral_m_distribution": distribution([r["delta_endpoint_lateral_m"] for r in records]),
        "delta_endpoint_yaw_rad_distribution": distribution([r["delta_endpoint_yaw_rad"] for r in records]),
        "fail_to_pass_count": fail_to_pass, "pass_to_fail_count": pass_to_fail,
        "same_geometry_class_count": same_class, "changed_geometry_class_count": 30 - same_class,
        "stage0g2_unique_raw_count": g2_summary["unique_raw_output_count"], "stage0g3_unique_raw_count": len(set(hashes)),
        "stage0g2_dominant_fraction": g2_summary["dominant_exact_raw_fraction"], "stage0g3_dominant_fraction": dominant,
        "stage0g2_geometry_signature_count": g2_summary["unique_geometry_signature_count"], "stage0g3_geometry_signature_count": len(signatures),
        "scenario_results": scenario_summary, "cases": records,
    }
    save_json_exclusive(run / "stage0g2_vs_stage0g3_summary.json", comparison)
    save_json_exclusive(run / "stage0g2_vs_stage0g3_cases.json", {"cases": records})
    with (run / "stage0g2_vs_stage0g3_cases.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0])); writer.writeheader(); writer.writerows(records)
    summary = {
        "stage": config["stage"], "config_sha256": sha256_file(run / "config_snapshot.yaml"),
        "primary_output_count": 30, "stage0g2_control_config_sha256": control["config_sha256"],
        "selected_common_approach_distance_m": selected_distance,
        "max_final_translation_mismatch_m": max(final_translation_errors), "max_final_yaw_mismatch_rad": max(final_yaw_errors),
        "history_translation_m_distribution": distribution(history_distances),
        "mean_interframe_translation_m_distribution": distribution([r["mean_interframe_translation_m"] for r in records]),
        "mean_consecutive_rgb_mae_uint8_distribution": distribution(consecutive_rgb),
        "first_to_final_rgb_mae_uint8_distribution": distribution(first_final_rgb),
        "scenario_results": scenario_summary, "unique_raw_output_count": len(set(hashes)),
        "dominant_exact_raw_fraction": dominant, "unique_geometry_signature_count": len(signatures),
        "geometry_signatures": signatures, "stop_or_nonmoving_count": stop_count,
        "visual_inspection": {"all_six_v0_contact_sheets": bool(options.visual_inspection_passed), "all_six_v0_stationary_vs_moving": bool(options.visual_inspection_passed), "all_overviews": bool(options.visual_inspection_passed)},
        "waypoint_rows_have_intrinsic_timestamps": False,
    }
    save_json_exclusive(run / "summary.json", summary)
    save_json_exclusive(run / "qualification_result.json", {
        "status": status, "conditions": conditions,
        "claim_boundary": "Paired inference-only association with temporal visual history; not physical following, navigation success, causality, obstacle avoidance, controller, or reconciliation evidence.",
        "next_step_if_ready": "Collect genuine same-episode successive OLD/FRESH chunks while actual Jackal executes OLD under the frozen Hospital setup." if status == "STAGE0G3_READY_FOR_SUCCESSIVE_DATA_COLLECTION" else None,
    })

    overview = run / "overview"; overview.mkdir(exist_ok=False)
    figure, axes = plt.subplots(6, 5, figsize=(15, 14), constrained_layout=True)
    for row, scenario_id in enumerate(SCENARIO_IDS):
        case = run / "qualification" / scenario_id / "V0"
        for col, index in enumerate(CONTACT_FRAMES):
            axes[row, col].imshow(Image.open(case / f"input/history/frame_{index:06d}.png")); axes[row, col].axis("off")
            axes[row, col].set_title(f"{scenario_id}\nf{index}" if col == 0 else f"f{index}", fontsize=7)
    figure.suptitle("Stage 0-G3 V0 scripted moving-history examples"); figure.savefig(overview / "history_examples.png", dpi=130); plt.close(figure)
    figure, axes = plt.subplots(3, 2, figsize=(11, 13), constrained_layout=True)
    for axis, scenario_id in zip(axes.flat, SCENARIO_IDS):
        local_plot(axis, np.load(g2_run / "qualification" / scenario_id / "V0/raw/lightnav.npy"), named_moving[f"{scenario_id}/V0"], scenario_id, True)
    figure.savefig(overview / "all_scenarios_stationary_vs_moving.png", dpi=140); plt.close(figure)
    figure, axes = plt.subplots(6, 5, figsize=(18, 19), constrained_layout=True)
    for row, scenario_id in enumerate(SCENARIO_IDS):
        for col, variant_id in enumerate(VARIANT_IDS):
            local_plot(axes[row, col], np.load(g2_run / "qualification" / scenario_id / variant_id / "raw/lightnav.npy"), named_moving[f"{scenario_id}/{variant_id}"], f"{scenario_id}/{variant_id}")
    figure.savefig(overview / "all_variants_by_scenario.png", dpi=140); plt.close(figure)
    figure, axis = plt.subplots(figsize=(11, 5), constrained_layout=True)
    x = np.arange(6); width = .36
    axis.bar(x - width/2, [q[s]["stage0g2_stationary_intended_matches"] for s in SCENARIO_IDS], width, label="G2 stationary", color="#06a9c8")
    axis.bar(x + width/2, [q[s]["stage0g3_moving_intended_matches"] for s in SCENARIO_IDS], width, label="G3 moving", color="#e7298a")
    axis.set_xticks(x, [s.replace("G2_", "") for s in SCENARIO_IDS], rotation=20, ha="right"); axis.set_ylim(0, 5.5); axis.set_ylabel("intended matches /5"); axis.legend(); axis.grid(axis="y", alpha=.25)
    figure.savefig(overview / "intended_match_comparison.png", dpi=150); plt.close(figure)
    print(f"STAGE0G3_PRIMARY_OUTPUTS=30 changed={comparison['exact_raw_changed_count']}/30 unique={len(set(hashes))} dominant={dominant:.3f}")
    for scenario_id in SCENARIO_IDS:
        value = q[scenario_id]; print(f"STAGE0G3_SCENARIO {scenario_id} stationary={value['stage0g2_stationary_intended_matches']}/5 moving={value['stage0g3_moving_intended_matches']}/5 qualified={str(value['qualified']).lower()}")
    print(status)


if __name__ == "__main__": main()

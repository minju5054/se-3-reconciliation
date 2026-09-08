#!/usr/bin/env python3
"""Validate Stage 0-F execution, decide coverage, and generate frozen plots."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import sys
from typing import Any, Mapping

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from reconciliation.closed_loop_execution_validation import validate_closed_loop_trial  # noqa: E402
from reconciliation.lightnav_execution_envelope import (  # noqa: E402
    final_envelope_decision,
    rigid_transform_trajectory,
    severity_key,
)
from reconciliation.online_switch import load_strict_json, sha256_file  # noqa: E402
from reconciliation.se2 import inverse_pose  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--validate-only", action="store_true")
    return parser.parse_args()


def write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(dict(value), stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def condition_path(
    root: Path,
    kind: str,
    reference_id: str,
    mode: str,
    repetition: int,
) -> Path:
    if mode == "calibrated":
        base = root / "execution" / kind.lower() / reference_id
    else:
        base = root / "nominal_representatives" / kind.lower() / reference_id
    return base / mode / f"repetition_{repetition:02d}"


def validate_run(root: Path) -> dict[str, Any]:
    required = (
        "config_snapshot.yaml",
        "preparation_metadata.json",
        "execution_metadata.json",
        "summary/source_inventory.json",
        "summary/stage0e_fixture_geometry.json",
        "summary/old_geometry_summary.json",
        "summary/fresh_geometry_summary.json",
        "summary/fresh_suffix_geometry_summary.json",
        "summary/geometry_fixture_comparison.json",
        "summary/representative_selection.json",
        "summary/old_execution_summary.json",
        "summary/fresh_execution_summary.json",
        "summary/representative_mode_comparison.json",
    )
    missing = [name for name in required if not (root / name).is_file()]
    if missing:
        raise ValueError(f"Stage 0-F run is missing: {missing}")
    preparation = load_strict_json(root / "preparation_metadata.json")
    execution = load_strict_json(root / "execution_metadata.json")
    inventory = load_strict_json(root / "summary/source_inventory.json")
    comparison = load_strict_json(root / "summary/geometry_fixture_comparison.json")
    selection = load_strict_json(root / "summary/representative_selection.json")
    for relative, key in (
        ("config_snapshot.yaml", "config_snapshot_sha256"),
        ("summary/source_inventory.json", "source_inventory_sha256"),
        ("summary/stage0e_fixture_geometry.json", "stage0e_fixture_geometry_sha256"),
        ("summary/geometry_fixture_comparison.json", "geometry_fixture_comparison_sha256"),
        ("summary/representative_selection.json", "representative_selection_sha256"),
    ):
        if sha256_file(root / relative) != preparation[key]:
            raise ValueError(f"prepared artifact hash changed: {relative}")
    if execution.get("historical_controller_commands_reused") is not False:
        raise ValueError("historical controller commands were reused")
    if execution.get("controller_or_physics_tuned") is not False:
        raise ValueError("controller or physics tuning flag changed")
    repetitions = int(execution["repetitions"])
    geometry_rows = comparison["references"]
    validated = []

    def validate_bound_trial(
        trial_path: Path,
        row: Mapping[str, Any],
        reference_digest: str,
        provenance_digest: str,
    ) -> dict[str, Any]:
        result = validate_closed_loop_trial(trial_path)
        metadata = load_strict_json(trial_path / "metadata.json")
        expected = {
            "reference_derived_copy_sha256": reference_digest,
            "source_world_trajectory_sha256": row["world_trajectory_sha256"],
            "source_provenance_sha256": provenance_digest,
            "stage0d_candidate_model_sha256": execution["stage0d_candidate"][
                "model_sha256"
            ],
        }
        for key, value in expected.items():
            if metadata.get(key) != value:
                raise ValueError(f"trial provenance binding changed: {key}")
        return result

    for row in geometry_rows:
        reference_path = (
            root
            / "geometry"
            / row["kind"].lower()
            / row["reference_id"]
            / "reference_world.npy"
        )
        provenance = load_strict_json(reference_path.parent / "source_provenance.json")
        if sha256_file(reference_path) != provenance["derived_copy_sha256"]:
            raise ValueError("execution reference hash changed")
        reference_digest = sha256_file(reference_path)
        provenance_digest = sha256_file(
            reference_path.parent / "source_provenance.json"
        )
        for repetition in range(repetitions):
            validated.append(
                validate_bound_trial(
                    condition_path(
                        root,
                        row["kind"],
                        row["reference_id"],
                        "calibrated",
                        repetition,
                    ),
                    row,
                    reference_digest,
                    provenance_digest,
                )
            )
    selected = {
        (row["kind"], row["world_trajectory_sha256"])
        for row in selection["references"]
    }
    selected_rows = [
        row
        for row in geometry_rows
        if (row["kind"], row["world_trajectory_sha256"]) in selected
    ]
    for row in selected_rows:
        reference_path = (
            root
            / "geometry"
            / row["kind"].lower()
            / row["reference_id"]
            / "reference_world.npy"
        )
        reference_digest = sha256_file(reference_path)
        provenance_digest = sha256_file(
            reference_path.parent / "source_provenance.json"
        )
        for repetition in range(repetitions):
            validated.append(
                validate_bound_trial(
                    condition_path(
                        root,
                        row["kind"],
                        row["reference_id"],
                        "nominal",
                        repetition,
                    ),
                    row,
                    reference_digest,
                    provenance_digest,
                )
            )
    expected = int(execution["calibrated_trial_count"]) + int(
        execution["nominal_trial_count"]
    )
    if len(validated) != expected:
        raise ValueError("validated trial count differs from execution metadata")
    if len(geometry_rows) != inventory["unique_execution_reference_count"]:
        raise ValueError("geometry and inventory unique counts differ")
    return {
        "valid": True,
        "run_id": preparation["run_id"],
        "validated_trial_count": len(validated),
        "calibrated_reference_count": len(geometry_rows),
        "nominal_representative_count": len(selected_rows),
    }


def condition_rows(root: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    old = load_strict_json(root / "summary/old_execution_summary.json")
    fresh = load_strict_json(root / "summary/fresh_execution_summary.json")

    def flatten(summary: Mapping[str, Any]) -> list[dict[str, Any]]:
        return [
            {
                "reference_id": reference_id,
                "kind": row["kind"],
                "passed": bool(row["passed"]),
                "fixture_label": row["fixture_comparison"]["label"],
                "fixture_comparison": row["fixture_comparison"],
                "descriptor": row["descriptor"],
                "source_geometry_ids": row["source_geometry_ids"],
                "source_latency_ids": row["source_latency_ids"],
                "metrics": {
                    name: float(np.mean([trial[name] for trial in row["trial_metrics"]]))
                    for name in (
                        "position_rmse_m",
                        "yaw_rmse_rad",
                        "final_position_error_m",
                        "final_yaw_error_rad",
                        "desired_abs_omega_p95_rps",
                        "desired_abs_omega_max_rps",
                        "desired_omega_vs_measured_omega_rmse_rps",
                        "executed_omega_vs_measured_omega_rmse_rps",
                        "wheel_target_vs_measured_rmse_rad_s",
                        "active_saturation_fraction",
                    )
                },
            }
            for reference_id, row in summary["conditions"].items()
        ]

    return flatten(old), flatten(fresh)


def save_figure(fig, path: Path) -> None:
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def reference_and_actual(
    root: Path, row: Mapping[str, Any], mode: str
) -> tuple[np.ndarray, np.ndarray]:
    reference = np.load(
        root
        / "geometry"
        / row["kind"].lower()
        / row["reference_id"]
        / "reference_world.npy",
        allow_pickle=False,
    )
    actual = np.load(
        condition_path(root, row["kind"], row["reference_id"], mode, 0)
        / "raw/actual_trajectory.npy",
        allow_pickle=False,
    )
    return reference, actual


def generate_plots(
    root: Path,
    fixture: Mapping[str, Any],
    rows: list[dict[str, Any]],
    severity_order: list[str],
) -> list[Path]:
    output = root / "plots"
    output.mkdir(parents=True, exist_ok=False)
    paths: list[Path] = []
    fixtures = fixture["fixtures"]
    colors = {"OLD": "#1565c0", "FRESH": "#7b1fa2"}

    def distribution_plot(field: str, title: str, ylabel: str, filename: str) -> None:
        fig, ax = plt.subplots(figsize=(8.5, 4.8))
        for index, kind in enumerate(("OLD", "FRESH")):
            values = [row["descriptor"][field] for row in rows if row["kind"] == kind]
            ax.scatter(
                np.full(len(values), index),
                values,
                color=colors[kind],
                alpha=0.7,
                label=kind,
            )
        fixture_values = [row["descriptor"][field] for row in fixtures]
        ax.scatter(
            np.full(len(fixture_values), 2),
            fixture_values,
            c=["#2e7d32" if row["calibrated_passed"] else "#c62828" for row in fixtures],
            marker="D",
            label="Stage 0-E fixtures",
        )
        ax.set_xticks([0, 1, 2], ["OLD", "FRESH", "Stage 0-E"])
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True, axis="y", alpha=0.25)
        ax.legend()
        path = output / filename
        save_figure(fig, path)
        paths.append(path)

    distribution_plot(
        "tangent_curvature_p95_abs_per_m",
        "Untimed spatial tangent-curvature p95",
        "p95 |tangent curvature proxy| [1/m]",
        "01_tangent_curvature_distribution.png",
    )
    distribution_plot(
        "cumulative_abs_pose_yaw_change_rad",
        "Untimed cumulative absolute pose-yaw change",
        "cumulative |pose yaw change| [rad]",
        "02_cumulative_pose_yaw_distribution.png",
    )
    distribution_plot(
        "max_abs_pose_yaw_increment_rad",
        "Maximum pose-yaw increment",
        "max |pose yaw increment| [rad/row]",
        "03_max_pose_yaw_increment_distribution.png",
    )

    fig, ax = plt.subplots(figsize=(8.5, 5.2))
    for kind in ("OLD", "FRESH"):
        selected = [row for row in rows if row["kind"] == kind]
        ax.scatter(
            [row["descriptor"]["path_length_m"] for row in selected],
            [
                row["descriptor"]["tangent_curvature_p95_abs_per_m"]
                for row in selected
            ],
            label=f"LightNav {kind}",
            color=colors[kind],
            alpha=0.75,
        )
    for row in fixtures:
        ax.scatter(
            row["descriptor"]["path_length_m"],
            row["descriptor"]["tangent_curvature_p95_abs_per_m"],
            marker="D",
            color="#2e7d32" if row["calibrated_passed"] else "#c62828",
        )
        ax.annotate(
            row["fixture_id"],
            (
                row["descriptor"]["path_length_m"],
                row["descriptor"]["tangent_curvature_p95_abs_per_m"],
            ),
            fontsize=7,
        )
    ax.set_xlabel("path length [m]")
    ax.set_ylabel("p95 |tangent curvature proxy| [1/m]")
    ax.set_title("Stage 0-E fixtures and LightNav spatial geometry")
    ax.grid(True, alpha=0.25)
    ax.legend()
    path = output / "04_path_length_vs_tangent_curvature.png"
    save_figure(fig, path)
    paths.append(path)

    fig, ax = plt.subplots(figsize=(10.0, 4.8))
    ordered = sorted(rows, key=lambda row: (row["kind"], row["reference_id"]))
    ax.bar(
        np.arange(len(ordered)),
        [row["fixture_comparison"]["nearest_fixture_distance"] for row in ordered],
        color=[colors[row["kind"]] for row in ordered],
    )
    ax.axhline(
        fixture["distance_model"]["outside_radius"],
        color="#c62828",
        linestyle="--",
        label="fixture-only outside radius",
    )
    ax.set_xlabel("unique LightNav reference")
    ax.set_ylabel("normalized nearest-fixture distance [unitless]")
    ax.set_title("Descriptive Stage 0-E fixture distance")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend()
    path = output / "05_nearest_fixture_distance.png"
    save_figure(fig, path)
    paths.append(path)

    severity = sorted(
        rows,
        key=lambda row: (
            severity_key(row["descriptor"], severity_order),
            row["kind"],
            row["reference_id"],
        ),
    )
    fig, ax = plt.subplots(figsize=(9.0, 4.8))
    ax.scatter(
        np.arange(1, len(severity) + 1),
        [row["metrics"]["yaw_rmse_rad"] for row in severity],
        c=[colors[row["kind"]] for row in severity],
    )
    ax.axhline(0.10, color="#c62828", linestyle="--", label="Stage 0-E yaw gate")
    ax.set_xlabel("geometry severity rank [easy to hard]")
    ax.set_ylabel("calibrated yaw RMSE [rad]")
    ax.set_title("LightNav geometry severity versus execution yaw RMSE")
    ax.grid(True, alpha=0.25)
    ax.legend()
    path = output / "06_severity_rank_vs_yaw_rmse.png"
    save_figure(fig, path)
    paths.append(path)

    fig, ax = plt.subplots(figsize=(8.0, 5.0))
    for kind in ("OLD", "FRESH"):
        selected = [row for row in rows if row["kind"] == kind]
        ax.scatter(
            [row["metrics"]["desired_abs_omega_p95_rps"] for row in selected],
            [row["metrics"]["yaw_rmse_rad"] for row in selected],
            color=colors[kind],
            label=kind,
        )
    ax.axhline(0.10, color="#c62828", linestyle="--", label="yaw gate")
    ax.set_xlabel("follower desired p95 |omega| [rad/s]")
    ax.set_ylabel("calibrated yaw RMSE [rad]")
    ax.set_title("Desired angular command demand versus yaw tracking")
    ax.grid(True, alpha=0.25)
    ax.legend()
    path = output / "07_desired_omega_vs_yaw_rmse.png"
    save_figure(fig, path)
    paths.append(path)

    groups: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        for group in row["source_geometry_ids"]:
            groups[group].append(row)
    labels = sorted(groups)
    passed = [sum(row["passed"] for row in groups[label]) for label in labels]
    failed = [len(groups[label]) - passed[index] for index, label in enumerate(labels)]
    fig, ax = plt.subplots(figsize=(8.0, 4.8))
    ax.bar(labels, passed, label="PASS", color="#2e7d32")
    ax.bar(labels, failed, bottom=passed, label="FAIL", color="#c62828")
    ax.set_ylabel("unique reference memberships [count]")
    ax.set_title("Calibrated execution by frozen LightNav geometry group")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend()
    path = output / "08_execution_by_geometry_group.png"
    save_figure(fig, path)
    paths.append(path)

    fig, axes = plt.subplots(2, 3, figsize=(14.0, 8.0))
    for row_index, kind in enumerate(("OLD", "FRESH")):
        kind_rows = sorted(
            [row for row in rows if row["kind"] == kind],
            key=lambda row: (
                severity_key(row["descriptor"], severity_order), row["reference_id"]
            ),
        )
        selected = (
            kind_rows[0],
            kind_rows[(len(kind_rows) - 1) // 2],
            kind_rows[-1],
        )
        for column, (label, row) in enumerate(
            zip(("low", "median", "high"), selected, strict=True)
        ):
            reference, actual = reference_and_actual(root, row, "calibrated")
            transform = inverse_pose(reference[0])
            reference_local = rigid_transform_trajectory(reference, transform)
            actual_local = rigid_transform_trajectory(actual, transform)
            ax = axes[row_index, column]
            ax.plot(reference_local[:, 0], reference_local[:, 1], label="reference", color="#1565c0")
            ax.plot(actual_local[:, 0], actual_local[:, 1], label="calibrated", color="#2e7d32")
            ax.set_title(f"{kind} {label}: {row['reference_id'][-6:]}")
            ax.set_xlabel("canonical x [m]")
            ax.set_ylabel("canonical y [m]")
            ax.axis("equal")
            ax.grid(True, alpha=0.25)
            ax.legend(fontsize=8)
    fig.suptitle("Display-only first-pose canonicalized representative overlays")
    path = output / "09_low_median_high_xy.png"
    save_figure(fig, path)
    paths.append(path)

    hardest = severity[-1]
    strong = [
        row for row in fixtures if row["fixture_id"] in ("strong_left", "strong_right")
    ]
    descriptor_names = (
        "tangent_curvature_p95_abs_per_m",
        "cumulative_abs_tangent_turn_rad",
        "max_abs_pose_yaw_increment_rad",
    )
    x = np.arange(len(descriptor_names))
    width = 0.25
    fig, ax = plt.subplots(figsize=(9.0, 4.8))
    for index, row in enumerate([hardest, *strong]):
        descriptor = row["descriptor"]
        label = row.get("reference_id", row.get("fixture_id"))
        ax.bar(
            x + (index - 1) * width,
            [descriptor[name] for name in descriptor_names],
            width,
            label=label,
        )
    ax.set_xticks(x, ["p95 tangent κ [1/m]", "cumulative tangent turn [rad]", "max pose Δyaw [rad]"])
    ax.set_title("Hardest LightNav reference versus Stage 0-E strong-turn fixtures")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(fontsize=8)
    path = output / "10_hardest_vs_strong_turn_descriptors.png"
    save_figure(fig, path)
    paths.append(path)
    return paths


def main() -> None:
    args = parse_args()
    root = args.run_directory.resolve()
    validation = validate_run(root)
    old_rows, fresh_rows = condition_rows(root)
    decision = final_envelope_decision(
        old_results=old_rows, fresh_results=fresh_rows
    )
    all_rows = old_rows + fresh_rows
    worst = sorted(
        all_rows,
        key=lambda row: (
            row["metrics"]["yaw_rmse_rad"],
            row["metrics"]["position_rmse_m"],
            row["reference_id"],
        ),
        reverse=True,
    )
    decision.update(
        {
            "fixture_label_counts": dict(
                sorted(Counter(row["fixture_label"] for row in all_rows).items())
            ),
            "failed_references": [
                {
                    "reference_id": row["reference_id"],
                    "kind": row["kind"],
                    "fixture_label": row["fixture_label"],
                    "nearest_fixture": row["fixture_comparison"]["nearest_fixture"],
                    "metrics": row["metrics"],
                    "descriptor": row["descriptor"],
                }
                for row in all_rows
                if not row["passed"]
            ],
            "worst_by_yaw_rmse": [
                {
                    "reference_id": row["reference_id"],
                    "kind": row["kind"],
                    "passed": row["passed"],
                    "fixture_label": row["fixture_label"],
                    "yaw_rmse_rad": row["metrics"]["yaw_rmse_rad"],
                    "position_rmse_m": row["metrics"]["position_rmse_m"],
                }
                for row in worst[:5]
            ],
            "fresh_start_semantics": "observation state, not ready/switch state",
            "fresh_suffix_execution_evaluated": False,
            "reconciliation_solved": False,
        }
    )
    if not (root / "summary/final_lightnav_execution_envelope_decision.json").exists():
        write_json(
            root / "summary/final_lightnav_execution_envelope_decision.json",
            decision,
        )
    else:
        existing = load_strict_json(
            root / "summary/final_lightnav_execution_envelope_decision.json"
        )
        if existing != decision:
            raise ValueError("existing final decision does not reconstruct")
    plot_count = 0
    if not args.validate_only:
        fixture = load_strict_json(root / "summary/stage0e_fixture_geometry.json")
        import yaml

        with (root / "config_snapshot.yaml").open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        paths = generate_plots(
            root, fixture, all_rows, config["geometry"]["severity_order"]
        )
        plot_count = len(paths)
        write_json(
            root / "plots/plot_manifest.json",
            {
                "plot_count": plot_count,
                "plots": [
                    {"path": str(path), "sha256": sha256_file(path)}
                    for path in paths
                ],
                "coordinate_note": (
                    "XY paths are separate or display-only first-pose canonicalized; "
                    "source world frames are never overlaid as if common"
                ),
            },
        )
    print(
        "STAGE0F_VALIDATION="
        + json.dumps(
            {
                **validation,
                "decision": decision["status"],
                "old_pass": decision["unique_old_pass_count"],
                "old_count": decision["unique_old_count"],
                "fresh_pass": decision["unique_fresh_pass_count"],
                "fresh_count": decision["unique_fresh_count"],
                "failed_count": len(decision["failed_references"]),
                "plot_count": plot_count,
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()

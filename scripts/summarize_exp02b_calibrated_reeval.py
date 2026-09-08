#!/usr/bin/env python3
"""Validate and summarize the frozen EXP-02B-R calibrated re-evaluation."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any, Mapping

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

from reconciliation.exp02b import CASE_IDS
from reconciliation.exp02b_calibrated_reeval import (
    ENTRY_INDICES,
    REEVAL_METHODS,
    compare_method_pairs,
    final_interpretation,
    historical_nominal_post_metrics,
    transition_change_metrics,
    validate_branch_output,
    validate_reeval_config,
    verify_file_hash,
    write_json_exclusive,
)
from reconciliation.online_switch import load_strict_json, sha256_file


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CASE_SHORT = {
    "case_high_delta_v": "high-dv",
    "case_high_delta_omega": "high-domega",
    "case_benign_delayed": "benign",
}
METHOD_COLORS = {"raw_k": "#4C78A8", "rigid": "#F58518", "graph": "#B279A2"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--validate-only", action="store_true")
    return parser.parse_args()


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a mapping")
    return value


def read_body_csv(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    old = next(row for row in rows if row["segment"] == "OLD_PRE_RESET_FINAL_INTERVAL")
    post = [row for row in rows if row["segment"] == "POST_RESET_CANDIDATE_INTERVAL"]
    return (
        np.asarray([float(old["measured_v_mps"]), float(old["measured_omega_rps"])]),
        np.asarray(
            [[float(row["measured_v_mps"]), float(row["measured_omega_rps"])] for row in post]
        ),
    )


def read_command_csv(path: Path, prefix: str) -> np.ndarray:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    return np.asarray(
        [[float(row[f"{prefix}_v_mps"]), float(row[f"{prefix}_omega_rps"])] for row in rows]
    )


def write_csv_exclusive(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        raise ValueError("cannot write an empty summary CSV")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def validate_run(run: Path) -> dict[str, Any]:
    config = load_yaml(run / "config_snapshot.yaml")
    validate_reeval_config(config)
    metadata = load_strict_json(run / "metadata.json")
    for filename, key in (
        ("config_snapshot.yaml", "config_snapshot_sha256"),
        ("source_provenance.json", "source_provenance_sha256"),
        ("historical_metric_snapshot.json", "historical_metric_snapshot_sha256"),
        ("source_snapshot/candidate_manifest.json", "candidate_manifest_sha256"),
    ):
        verify_file_hash(run / filename, metadata[key], f"prepared {filename}")
    execution = load_strict_json(run / "execution_summary.json")
    snapshot = load_strict_json(run / "historical_metric_snapshot.json")
    if execution["branch_count"] != 27 or snapshot["branch_count"] != 27:
        raise ValueError("EXP-02B-R requires exactly 27 branches")
    if not execution["all_comparability_gates_passed"]:
        raise ValueError("one or more OLD replay comparability gates failed")
    if not execution["all_first_desired_invariants_passed"]:
        raise ValueError("one or more first desired invariants failed")
    seen = set()
    for row in execution["branches"]:
        key = (row["case_id"], int(row["entry_index"]), row["method"])
        expected = config["candidate_sha256"][key[0]][f"k_{key[1]}"][key[2]]
        validate_branch_output(row["branch_directory"], expected)
        verify_file_hash(
            Path(load_strict_json(Path(row["branch_directory"]) / "metadata.json")["candidate_path"]),
            expected,
            f"current candidate {key}",
        )
        if key in seen:
            raise ValueError(f"duplicate EXP-02B-R branch: {key}")
        seen.add(key)
    expected_keys = {
        (case, k, method)
        for case in CASE_IDS
        for k in ENTRY_INDICES
        for method in REEVAL_METHODS
    }
    if seen != expected_keys:
        raise ValueError("EXP-02B-R branch set is incomplete")
    summary_exists = (run / "summary/calibrated_reeval_summary.json").is_file()
    plot_count = 0
    if summary_exists:
        final = load_strict_json(run / "summary/final_interpretation.json")
        if final["invariant_check"]["passed_count"] != 27:
            raise ValueError("final interpretation does not retain all invariants")
        manifest = load_strict_json(run / "plots/plot_manifest.json")
        for name in manifest["files"]:
            path = run / "plots" / name
            if not path.is_file() or path.stat().st_size == 0:
                raise ValueError(f"missing plot: {name}")
        plot_count = len(manifest["files"])
    return {
        "valid": True,
        "branch_count": len(seen),
        "all_invariants_passed": True,
        "summary_present": summary_exists,
        "plot_count": plot_count,
    }


def make_plots(
    run: Path,
    rows: list[dict[str, Any]],
    snapshot_by_key: Mapping[tuple[str, int, str], Mapping[str, Any]],
) -> list[str]:
    plots = run / "plots"
    plots.mkdir(parents=True, exist_ok=False)
    labels = [f"{CASE_SHORT[row['case_id']]} k{row['entry_index']} {row['method']}" for row in rows]
    x = np.arange(len(rows))
    files = []

    fig, axes = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
    v_differences = [row["invariant_abs_v"] for row in rows]
    omega_differences = [row["invariant_abs_omega"] for row in rows]
    axes[0].plot(x, v_differences, "o", color="#4C78A8", ms=4)
    axes[1].plot(x, omega_differences, "o", color="#E45756", ms=4)
    for ax in axes:
        ax.axhline(0.0, color="black", linewidth=0.8)
        ax.set_ylim(-1e-12, 1e-12)
        ax.grid(axis="y", alpha=0.25)
    axes[0].set_ylabel("|desired v difference| [m/s]")
    axes[1].set_ylabel("|desired omega difference| [rad/s]")
    axes[1].set_xticks(x, labels, rotation=90, fontsize=7)
    axes[0].set_title(
        "Historical vs calibrated-run first desired command invariant: 27/27 PASS"
    )
    fig.tight_layout()
    name = "01_first_desired_invariant.png"
    fig.savefig(plots / name, dpi=160)
    plt.close(fig)
    files.append(name)

    for index, metric, ylabel, filename in (
        (2, "measured_delta_omega_abs", "first measured |delta omega| [rad/s]", "02_measured_delta_omega.png"),
        (3, "measured_delta_v_abs", "first measured |delta v| [m/s]", "03_measured_delta_v.png"),
    ):
        del index
        fig, ax = plt.subplots(figsize=(12, 5))
        width = 0.24
        groups = [(case, k) for case in CASE_IDS for k in ENTRY_INDICES]
        gx = np.arange(len(groups))
        by_key = {(row["case_id"], row["entry_index"], row["method"]): row for row in rows}
        for offset, method in enumerate(REEVAL_METHODS):
            ax.bar(
                gx + (offset - 1) * width,
                [by_key[(case, k, method)][metric] for case, k in groups],
                width,
                label=method,
                color=METHOD_COLORS[method],
            )
        ax.set_xticks(gx, [f"{CASE_SHORT[case]}\nk{k}" for case, k in groups])
        ax.set_ylabel(ylabel)
        ax.legend()
        ax.grid(axis="y", alpha=0.25)
        fig.tight_layout()
        fig.savefig(plots / filename, dpi=160)
        plt.close(fig)
        files.append(filename)

    fig, ax = plt.subplots(figsize=(12, 5))
    width = 0.38
    ax.bar(
        x - width / 2,
        [row["historical_desired_omega_tracking_rmse"] for row in rows],
        width,
        label="historical nominal",
        color="#F58518",
    )
    ax.bar(
        x + width / 2,
        [row["calibrated_desired_omega_tracking_rmse"] for row in rows],
        width,
        label="calibrated",
        color="#54A24B",
    )
    ax.set_ylabel("desired omega vs measured omega RMSE [rad/s]")
    ax.set_xticks(x, labels, rotation=90, fontsize=7)
    ax.legend()
    fig.tight_layout()
    name = "04_desired_omega_tracking_rmse.png"
    fig.savefig(plots / name, dpi=160)
    plt.close(fig)
    files.append(name)

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    width = 0.38
    axes[0].bar(x - width / 2, [row["historical_position_rmse_m"] for row in rows], width, color="#F58518", label="historical nominal")
    axes[0].bar(x + width / 2, [row["calibrated_position_rmse_m"] for row in rows], width, color="#54A24B", label="calibrated")
    axes[1].bar(x - width / 2, [row["historical_yaw_rmse_rad"] for row in rows], width, color="#F58518")
    axes[1].bar(x + width / 2, [row["calibrated_yaw_rmse_rad"] for row in rows], width, color="#54A24B")
    axes[0].set_ylabel("position RMS [m]")
    axes[1].set_ylabel("yaw RMS [rad]")
    axes[1].set_xticks(x, labels, rotation=90, fontsize=7)
    axes[0].legend()
    fig.tight_layout()
    name = "05_candidate_tracking_rms.png"
    fig.savefig(plots / name, dpi=160)
    plt.close(fig)
    files.append(name)

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for method, linestyle in (("raw_k", "-"), ("graph", "--")):
        row = next(
            item for item in rows
            if item["case_id"] == "case_benign_delayed" and item["entry_index"] == 0 and item["method"] == method
        )
        root = Path(row["branch_directory"])
        desired = read_command_csv(root / "desired_commands.csv", "desired")
        executed = read_command_csv(root / "executed_commands.csv", "executed")
        _, measured = read_body_csv(root / "measured_body.csv")
        time = np.arange(min(len(desired), 10)) * 0.1
        axes[0].plot(time, desired[: len(time), 0], linestyle, color=METHOD_COLORS[method], label=f"{method} desired")
        axes[0].plot(time, executed[: len(time), 0], ":", color=METHOD_COLORS[method], label=f"{method} executed")
        axes[0].plot(time, measured[: len(time), 0], "o", ms=3, color=METHOD_COLORS[method], label=f"{method} measured")
        axes[1].plot(time, desired[: len(time), 1], linestyle, color=METHOD_COLORS[method], label=f"{method} desired")
        axes[1].plot(time, executed[: len(time), 1], ":", color=METHOD_COLORS[method], label=f"{method} executed")
        axes[1].plot(time, measured[: len(time), 1], "o", ms=3, color=METHOD_COLORS[method], label=f"{method} measured")
    axes[0].set_ylabel("v [m/s]")
    axes[1].set_ylabel("omega [rad/s]")
    axes[1].set_xlabel("time after switch [s]")
    for ax in axes:
        ax.legend(ncol=3, fontsize=8)
        ax.grid(alpha=0.25)
    axes[0].set_title("Benign Case C k=0: desired / executed / measured")
    fig.tight_layout()
    name = "06_benign_k0_command_trace.png"
    fig.savefig(plots / name, dpi=160)
    plt.close(fig)
    files.append(name)

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    for ax, method in zip(axes, REEVAL_METHODS, strict=True):
        key = ("case_high_delta_omega", 3, method)
        row = next(item for item in rows if (item["case_id"], item["entry_index"], item["method"]) == key)
        frozen = snapshot_by_key[key]
        candidate = np.load(frozen["candidate_path"], allow_pickle=False)
        nominal = np.load(frozen["historical_actual_path"], allow_pickle=False)
        calibrated = np.load(Path(row["branch_directory"]) / "actual_trajectory.npy", allow_pickle=False)
        ax.plot(candidate[:, 0], candidate[:, 1], "-o", ms=3, color="#4C78A8", label="candidate")
        ax.plot(nominal[:, 0], nominal[:, 1], color="#F58518", label="historical nominal")
        ax.plot(calibrated[:, 0], calibrated[:, 1], color="#54A24B", label="calibrated")
        ax.set_title(method)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("world x [m]")
        ax.grid(alpha=0.25)
    axes[0].set_ylabel("world y [m]")
    axes[0].legend(fontsize=8)
    fig.suptitle("case_high_delta_omega k=3: same frozen candidates")
    fig.tight_layout()
    name = "07_high_delta_omega_k3_xy.png"
    fig.savefig(plots / name, dpi=160)
    plt.close(fig)
    files.append(name)
    write_json_exclusive(
        plots / "plot_manifest.json",
        {"plot_count": len(files), "files": files, "coordinate_frame": "Isaac Sim world XY"},
    )
    return files


def main() -> None:
    args = parse_args()
    run = args.run_directory.resolve()
    initial_validation = validate_run(run)
    if args.validate_only:
        print("EXP02B_R_VALIDATION=" + json.dumps(initial_validation, sort_keys=True))
        return
    summary_root = run / "summary"
    if summary_root.exists() or (run / "plots").exists():
        raise FileExistsError("EXP-02B-R summary/plots already exist")
    config = load_yaml(run / "config_snapshot.yaml")
    execution = load_strict_json(run / "execution_summary.json")
    snapshot = load_strict_json(run / "historical_metric_snapshot.json")
    snapshot_by_key = {
        (row["case_id"], int(row["entry_index"]), row["method"]): row
        for row in snapshot["branches"]
    }
    branch_rows = []
    csv_rows = []
    for row in execution["branches"]:
        key = (row["case_id"], int(row["entry_index"]), row["method"])
        root = Path(row["branch_directory"])
        frozen = snapshot_by_key[key]
        controller = load_strict_json(root / "controller_metrics.json")
        tracking = load_strict_json(root / "tracking_metrics.json")
        old_measured, calibrated_measured = read_body_csv(root / "measured_body.csv")
        desired = read_command_csv(root / "desired_commands.csv", "desired")
        actual = np.load(root / "actual_trajectory.npy", allow_pickle=False)
        physics_steps = int(
            round(config["simulation"]["control_dt_s"] / config["simulation"]["physics_dt_s"])
        )
        control_actual = actual[::physics_steps]
        calibrated_comparable = historical_nominal_post_metrics(
            candidate=np.load(frozen["candidate_path"], allow_pickle=False),
            actual=control_actual,
            desired=desired,
            control_dt_s=float(config["simulation"]["control_dt_s"]),
        )
        calibrated_comparable["sampling_semantics"] = "calibrated EXP-02B-R control-time poses and desired commands at 0.1 s"
        historical_measured = np.asarray(
            frozen["historical_nominal_post_metrics"]["measured_body_first3_v_omega"]
        )
        historical_transition = transition_change_metrics(
            old_measured,
            historical_measured,
            control_dt_s=float(config["simulation"]["control_dt_s"]),
            window_size=int(config["protocol"]["command_window"]),
        )
        result = {
            "case_id": key[0],
            "entry_index": key[1],
            "method": key[2],
            "branch_directory": str(root),
            "candidate_sha256": frozen["candidate_sha256"],
            "first_desired_invariant": controller["first_desired_invariant"],
            "controller_metrics": controller,
            "tracking_metrics": tracking,
            "historical_nominal_post_metrics": frozen["historical_nominal_post_metrics"],
            "calibrated_control_time_post_metrics": calibrated_comparable,
            "historical_nominal_measured_transition": historical_transition,
        }
        branch_rows.append(result)
        csv_rows.append(
            {
                "case_id": key[0],
                "entry_index": key[1],
                "method": key[2],
                "candidate_sha256": frozen["candidate_sha256"],
                "historical_desired_delta_v_abs_mps": frozen["historical_controller_metrics"]["delta_v_abs_mps"],
                "reeval_desired_delta_v_abs_mps": controller["desired"]["delta_v_abs"],
                "historical_desired_delta_omega_abs_rps": frozen["historical_controller_metrics"]["delta_omega_abs_rps"],
                "reeval_desired_delta_omega_abs_rps": controller["desired"]["delta_omega_abs"],
                "invariant_abs_v": controller["first_desired_invariant"]["absolute_difference_v_omega"][0],
                "invariant_abs_omega": controller["first_desired_invariant"]["absolute_difference_v_omega"][1],
                "invariant_passed": controller["first_desired_invariant"]["passed"],
                "historical_measured_delta_v_abs_mps": historical_transition["delta_v_abs"],
                "calibrated_measured_delta_v_abs_mps": controller["measured"]["delta_v_abs"],
                "historical_measured_delta_omega_abs_rps": historical_transition["delta_omega_abs"],
                "calibrated_measured_delta_omega_abs_rps": controller["measured"]["delta_omega_abs"],
                "historical_desired_v_tracking_rmse_mps": frozen["historical_nominal_post_metrics"]["desired_v_vs_measured_v_rmse_mps"],
                "calibrated_desired_v_tracking_rmse_mps": calibrated_comparable["desired_v_vs_measured_v_rmse_mps"],
                "historical_desired_omega_tracking_rmse_rps": frozen["historical_nominal_post_metrics"]["desired_omega_vs_measured_omega_rmse_rps"],
                "calibrated_desired_omega_tracking_rmse_rps": calibrated_comparable["desired_omega_vs_measured_omega_rmse_rps"],
                "historical_position_rmse_m": frozen["historical_nominal_post_metrics"]["position_rmse_m"],
                "calibrated_position_rmse_m": calibrated_comparable["position_rmse_m"],
                "historical_yaw_rmse_rad": frozen["historical_nominal_post_metrics"]["yaw_rmse_rad"],
                "calibrated_yaw_rmse_rad": calibrated_comparable["yaw_rmse_rad"],
                "calibrated_executed_v_tracking_rmse_mps": tracking["executed_v_vs_measured_v_rmse_mps"],
                "calibrated_executed_omega_tracking_rmse_rps": tracking["executed_omega_vs_measured_omega_rmse_rps"],
                "calibrated_wheel_rmse_rad_s": tracking["wheel_target_vs_measured_rmse_rad_s"],
                "calibrated_saturation_fraction": tracking["active_saturation_fraction"],
                "branch_directory": str(root),
            }
        )

    metric_paths = (
        ("controller_metrics", "measured", "delta_v_abs"),
        ("controller_metrics", "measured", "delta_omega_abs"),
        ("controller_metrics", "measured", "max3_delta_v_abs"),
        ("controller_metrics", "measured", "max3_delta_omega_abs"),
        ("calibrated_control_time_post_metrics", "desired_v_vs_measured_v_rmse_mps"),
        ("calibrated_control_time_post_metrics", "desired_omega_vs_measured_omega_rmse_rps"),
        ("calibrated_control_time_post_metrics", "position_rmse_m"),
        ("calibrated_control_time_post_metrics", "yaw_rmse_rad"),
        ("tracking_metrics", "wheel_target_vs_measured_rmse_rad_s"),
    )
    graph_raw = compare_method_pairs(
        branch_rows,
        method="graph",
        baseline="raw_k",
        metric_paths=metric_paths,
        tie_tolerance=float(config["protocol"]["pairwise_tie_tolerance"]),
    )
    graph_rigid = compare_method_pairs(
        branch_rows,
        method="graph",
        baseline="rigid",
        metric_paths=metric_paths,
        tie_tolerance=float(config["protocol"]["pairwise_tie_tolerance"]),
    )
    pairwise_csv = []
    for comparison in (graph_raw, graph_rigid):
        for item in comparison["rows"]:
            output = {
                "case_id": item["case_id"],
                "entry_index": item["entry_index"],
                "method": item["method"],
                "baseline": item["baseline"],
            }
            for name, metric in item["metrics"].items():
                safe = name.replace(".", "__")
                output[f"{safe}__method"] = metric["method"]
                output[f"{safe}__baseline"] = metric["baseline"]
                output[f"{safe}__difference"] = metric["method_minus_baseline"]
                output[f"{safe}__outcome"] = metric["outcome_lower_is_better"]
            pairwise_csv.append(output)

    aggregate_names = (
        "desired_v_tracking_rmse_mps",
        "desired_omega_tracking_rmse_rps",
        "position_rmse_m",
        "yaw_rmse_rad",
    )
    aggregate = {}
    for name in aggregate_names:
        historical_key = f"historical_{name}"
        calibrated_key = f"calibrated_{name}"
        historical_mean = float(np.mean([float(row[historical_key]) for row in csv_rows]))
        calibrated_mean = float(np.mean([float(row[calibrated_key]) for row in csv_rows]))
        aggregate[name] = {
            "historical_nominal_mean": historical_mean,
            "calibrated_mean": calibrated_mean,
            "calibrated_minus_historical": calibrated_mean - historical_mean,
            "improved_lower_is_better": calibrated_mean < historical_mean,
        }
    physical_tracking_improved = all(
        aggregate[name]["improved_lower_is_better"]
        for name in ("desired_omega_tracking_rmse_rps", "position_rmse_m", "yaw_rmse_rad")
    )

    def consistently_better(comparison: Mapping[str, Any]) -> bool:
        primary = (
            "controller_metrics.measured.delta_v_abs",
            "controller_metrics.measured.delta_omega_abs",
        )
        outcomes = [
            row["metrics"][name]["outcome_lower_is_better"]
            for row in comparison["rows"]
            for name in primary
        ]
        return all(value in ("better", "tie") for value in outcomes) and "better" in outcomes

    measured_graph_raw_consistent = consistently_better(graph_raw)
    measured_graph_rigid_consistent = consistently_better(graph_rigid)
    desired_graph_raw_consistent = all(
        row["controller_metrics"]["desired"]["delta_v_abs"]
        <= next(
            other for other in branch_rows
            if other["case_id"] == row["case_id"]
            and other["entry_index"] == row["entry_index"]
            and other["method"] == "raw_k"
        )["controller_metrics"]["desired"]["delta_v_abs"] + 1e-12
        and row["controller_metrics"]["desired"]["delta_omega_abs"]
        <= next(
            other for other in branch_rows
            if other["case_id"] == row["case_id"]
            and other["entry_index"] == row["entry_index"]
            and other["method"] == "raw_k"
        )["controller_metrics"]["desired"]["delta_omega_abs"] + 1e-12
        for row in branch_rows if row["method"] == "graph"
    )
    decision = final_interpretation(
        invariant_passed=True,
        graph_vs_raw_consistent=desired_graph_raw_consistent and measured_graph_raw_consistent,
        graph_vs_rigid_consistent=measured_graph_rigid_consistent,
        physical_tracking_improved=physical_tracking_improved,
        physical_method_ranking_materially_changed=False,
    )
    decision.update(
        {
            "invariant_check": {
                "passed_count": sum(bool(row["invariant_passed"]) for row in csv_rows),
                "branch_count": len(csv_rows),
                "maximum_absolute_v_difference_mps": max(float(row["invariant_abs_v"]) for row in csv_rows),
                "maximum_absolute_omega_difference_rps": max(float(row["invariant_abs_omega"]) for row in csv_rows),
            },
            "physical_tracking_aggregate": aggregate,
            "measured_graph_vs_raw_consistently_better": measured_graph_raw_consistent,
            "measured_graph_vs_rigid_consistently_better": measured_graph_rigid_consistent,
            "desired_graph_vs_raw_consistently_better": desired_graph_raw_consistent,
            "physical_method_ranking_materially_changed": False,
            "physical_ranking_label_basis": "all pairwise directions are reported; no post-result materiality threshold was invented",
            "historical_result_rewritten": False,
            "controller_or_reconciliation_tuned": False,
            "claim_boundary": "same frozen candidates, B, follower, nominal OLD replay, and deterministic Isaac environment; calibrated post-switch only",
        }
    )
    summary_root.mkdir(parents=True, exist_ok=False)
    write_csv_exclusive(summary_root / "historical_vs_calibrated.csv", csv_rows)
    write_csv_exclusive(summary_root / "method_pairwise_comparison.csv", pairwise_csv)
    write_json_exclusive(
        summary_root / "calibrated_reeval_summary.json",
        {
            "experiment": "EXP-02B-R",
            "branch_count": len(branch_rows),
            "branches": branch_rows,
            "physical_tracking_aggregate": aggregate,
            "graph_vs_raw_k": graph_raw,
            "graph_vs_rigid": graph_rigid,
        },
    )
    write_json_exclusive(summary_root / "final_interpretation.json", decision)
    plot_rows = [
        {
            **row,
            "invariant_abs_v": float(row["invariant_abs_v"]),
            "invariant_abs_omega": float(row["invariant_abs_omega"]),
            "measured_delta_v_abs": float(row["calibrated_measured_delta_v_abs_mps"]),
            "measured_delta_omega_abs": float(row["calibrated_measured_delta_omega_abs_rps"]),
            "historical_desired_omega_tracking_rmse": float(row["historical_desired_omega_tracking_rmse_rps"]),
            "calibrated_desired_omega_tracking_rmse": float(row["calibrated_desired_omega_tracking_rmse_rps"]),
        }
        for row in csv_rows
    ]
    files = make_plots(run, plot_rows, snapshot_by_key)
    final_validation = validate_run(run)
    print(
        "EXP02B_R_SUMMARY="
        + json.dumps(
            {
                "branch_count": len(branch_rows),
                "decision": decision["primary_status"],
                "secondary": decision["secondary_status"],
                "plot_count": len(files),
                "validation": final_validation,
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()

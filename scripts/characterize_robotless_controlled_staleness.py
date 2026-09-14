#!/usr/bin/env python3
"""Characterize saved FRESH geometry with controlled boundaries; no inference."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.robotless_controlled_staleness import characterize, csv_row, validate_config
from reconciliation.robotless_single_chunk import load_config, make_observation_time, save_json_exclusive, save_npy_exclusive, sha256_file
from robotless_controlled_staleness_artifacts import (
    SOURCE_ARRAYS, file_record, git, processing_sources, read_json, resolve_source,
    snapshot_source, source_arrays, verify_source,
)


METRIC_DEFINITIONS = {
    "frames": "Isaac world +Z up; agent +X forward,+Y left; metres, radians; yaw CCW",
    "boundary": "B(tau)=R_obs * [v*tau,0,0]; translation-only controlled surrogate",
    "delta_B": "R_obs^-1 * B; ordinary SE(2) relative pose components",
    "r_entry_log_B_frame": "Log(B^-1 * F_0): tangent [dx_m,dy_m,dyaw_rad], using principal yaw [-pi,pi)",
    "entry_relative_pose_B_frame": "B^-1 * F_0: ordinary [x_m,y_m,yaw_rad]; distinct from Log translation when yaw != 0",
    "r_entry_log_translation_norm_m": "norm of Log translation; separate from Euclidean d_entry",
    "d_entry_m": "Euclidean norm B_xy-F_0_xy",
    "d_poly_m": "minimum point-to-segment XY distance on FRESH rows only; no R_obs-to-F_0 connector",
    "delta_d_entry_m": "d_entry(tau)-d_entry(0), signed; can be negative",
    "delta_d_poly_m": "d_poly(tau)-d_poly(0), signed; can be negative",
    "tau_semantics": "controlled simulation variable; not LightNav RTT, execution or switch latency",
    "baseline_scope": "tau=0 is pre-motion boundary-to-FRESH geometry; OLD is context, not an OLD/FRESH pair-distance metric",
}


def plot_metrics(run: Path, config: dict) -> None:
    """Plots consume serialized derived metrics rather than recomputing geometry."""
    metrics_path = run / "derived/metrics.json"
    metrics = read_json(metrics_path)
    times = [row["tau_s"] for row in metrics["conditions"]]
    records = []
    for key, suffix, label in (("d_entry_m", "entry_gap", "Boundary to FRESH entry"),
                               ("d_poly_m", "polyline_gap", "Boundary to FRESH polyline")):
        values = np.array([row[key] for row in metrics["conditions"]])
        increments = np.array([row[f"delta_{key}"] for row in metrics["conditions"]])
        fig, axes = plt.subplots(2, 1, figsize=config["plots"]["figsize_inches"], sharex=True, layout="constrained")
        for ax in axes:
            ax.grid(alpha=0.22)
            ax.spines[["top", "right"]].set_visible(False)
            ax.set_xticks(times)
        axes[0].plot(times, values, "o-", color="#9634b5", label="Fixed FRESH, moving B")
        axes[0].axhline(values[0], color="#777777", linestyle="--", label="tau=0 baseline")
        axes[0].set_ylabel("Distance (m)")
        axes[0].set_ylim(bottom=0, top=max(values) * 1.22 + 1e-6)
        axes[0].legend(loc="upper right", fontsize=9)
        axes[1].plot(times, increments, "o-", color="#137d7a")
        axes[1].axhline(0, color="#777777", linestyle="--")
        axes[1].set_ylim(min(increments) - 0.035, max(increments) + 0.04)
        axes[1].set_ylabel("Change from tau=0 (m)")
        axes[1].set_xlabel(f"Controlled delay tau (s); v={config['motion']['v_mps']:g} m/s, omega=0")
        for ax, ys in zip(axes, (values, increments), strict=True):
            for x, y in zip(times, ys, strict=True):
                ax.annotate(f"{y:.6f}", (x, y), xytext=(0, 10), textcoords="offset points", ha="center", fontsize=9)
        fig.suptitle(f"{label}\nSaved model output + controlled motion surrogate", fontsize=13)
        path = run / f"evidence/latency_vs_{suffix}.png"
        with path.open("xb") as stream:
            fig.savefig(stream, format="png", dpi=config["plots"]["dpi"])
        plt.close(fig)
        records.append({**file_record(path, run), "metric": key, "tau_s": times,
                        "distance_m": values.tolist(), "increment_m": increments.tolist()})
    save_json_exclusive(run / "plot_manifest.json", {
        "metrics_input": file_record(metrics_path, run), "config_sha256": sha256_file(run / "config_snapshot.yaml"),
        "plots": records, "interpretation": "controlled tau, not measured LightNav latency", "matplotlib_version": matplotlib.__version__,
    })


def build_run(config_path: Path, run: Path) -> dict:
    config = load_config(config_path)
    validate_config(config)
    source = resolve_source(config["source_run"])
    run = run.resolve()
    if run.is_relative_to(source) or source.is_relative_to(run):
        raise ValueError("output must be separate from the immutable source run")
    if run.exists():
        raise FileExistsError("controlled output run must be new; existing runs are immutable")
    record = snapshot_source(source)  # Existing validator runs before creating outputs.
    arrays = source_arrays(source)
    metadata = read_json(source / "metadata.json")
    R_obs = metadata["R1"]
    motion = config["motion"]
    boundaries, rows = characterize(R_obs, arrays["FRESH_world"], motion["tau_s"], motion["v_mps"], motion["omega_radps"])
    run.mkdir(parents=True)
    (run / "derived").mkdir()
    (run / "evidence").mkdir()
    with (run / "config_snapshot.yaml").open("xb") as stream:
        stream.write(config_path.read_bytes())
    record.update({
        "schema_version": 1, "stage": config["stage"], "created_time": make_observation_time(0.0),
        "research_git_sha": git("rev-parse", "HEAD"), "research_git_status": git("status", "--porcelain"),
        "source_research_capture_git_sha": metadata["research_git_sha"],
        "source_reviewed_git_sha": read_json(source / "git_completion.json")["final_git_sha"],
        "R_obs": R_obs, "source_observation_time": metadata["observations"][1]["observation_time"],
        "configuration": file_record(run / "config_snapshot.yaml", run),
        "array_inputs": {key: file_record(source / path, source) for key, path in SOURCE_ARRAYS.items()},
        "research_source_sha256": processing_sources(), "new_lightnav_inference_count": 0,
        "actual_robot_execution_time": None, "actual_old_execution": False,
        "controlled_motion_is_surrogate": True, "inference_readiness_time": None,
        "time_convention": "created_time is host provenance; simulation_time_s=0 for static scene; tau is a separate controlled variable",
    })
    save_json_exclusive(run / "source.json", record)
    save_npy_exclusive(run / "derived/boundaries.npy", boundaries)
    fresh_ref = record["array_inputs"]["FRESH_world"]
    for row in rows:
        row["fixed_fresh_world_input"] = fresh_ref.copy()
    metrics = {"schema_version": 1, "definitions": METRIC_DEFINITIONS, "R_obs": R_obs,
               "motion": motion, "source_json_sha256": sha256_file(run / "source.json"),
               "boundaries_input": file_record(run / "derived/boundaries.npy", run),
               "configuration": record["configuration"], "conditions": rows,
               "fresh_reanchored": False, "old_role": "unchanged context only; not an executed path"}
    save_json_exclusive(run / "derived/metrics.json", metrics)
    flattened = [csv_row(row) for row in rows]
    with (run / "derived/metrics.csv").open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flattened[0]))
        writer.writeheader()
        writer.writerows(flattened)
    plot_metrics(run, config)
    verify_source(record)
    save_json_exclusive(run / "analysis_validation.json", {
        "source_hashes_unchanged": True, "source_json_sha256": sha256_file(run / "source.json"),
        "new_lightnav_inference_count": 0, "no_robot_controller": True,
        "fresh_fixed_all_conditions": True, "condition_count": len(rows),
        "outputs": [file_record(run / rel, run) for rel in ("derived/boundaries.npy", "derived/metrics.json", "derived/metrics.csv", "plot_manifest.json")],
    })
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/robotless_controlled_staleness.yaml")
    args = parser.parse_args()
    result = build_run(args.config.resolve(), args.run_directory)
    for row in result["conditions"]:
        print(f"tau={row['tau_s']:g} B={row['B_world']} d_entry={row['d_entry_m']:.9f} d_poly={row['d_poly_m']:.9f}")


if __name__ == "__main__":
    main()

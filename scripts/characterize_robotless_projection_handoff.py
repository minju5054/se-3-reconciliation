#!/usr/bin/env python3
"""Descriptive projection metrics and plots from a validated frozen source."""

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import robotless_projection_artifacts as artifacts
from reconciliation.robotless_projection_handoff import DEFINITIONS, characterize_projection, csv_row
from reconciliation.robotless_single_chunk import make_observation_time, save_json_exclusive, sha256_file


PLOTS = (
    ("e_perp_m", "cross_track", "Cross-track distance (m)"),
    ("abs_e_dir_deg", "direction", "Absolute tangent-direction mismatch (deg)"),
    ("abs_e_yaw_deg", "yaw", "Absolute pose-yaw mismatch (deg)"),
    ("s_Q_m", "progress", "Projection arc-length progress (m)"),
)


def make_plots(run: Path, config: dict) -> None:
    path = run / "derived/projection_metrics.json"
    metrics = artifacts.read_json(path)
    rows = metrics["conditions"]
    tau = [r["tau_s"] for r in rows]
    records = []
    for key, name, label in PLOTS:
        values = [r[key] for r in rows]
        numeric = np.array([np.nan if v is None else v for v in values])
        fig, ax = plt.subplots(figsize=config["plots"]["figsize_inches"], layout="constrained")
        ax.plot(tau, numeric, "o-", color="#8250ad")
        ax.set(xlabel="Controlled delay tau (s)", ylabel=label, xticks=tau)
        ax.set_ylim(0, max(float(np.max(numeric[np.isfinite(numeric)], initial=0)) * 1.35, 1e-5))
        ax.grid(alpha=.22)
        ax.spines[["top", "right"]].set_visible(False)
        for x, value in zip(tau, values, strict=True):
            if value is None:
                ax.text(x, .1, "Unavailable", transform=ax.get_xaxis_transform(), ha="center")
            else:
                ax.annotate(f"{value:.6f}", (x, value), xytext=(0, 10), textcoords="offset points", ha="center", fontsize=9)
        ax.set_title("Fixed FRESH: continuous projection geometry\nConfigured controlled body-forward motion", fontsize=12)
        image_path = run / f"evidence/tau_vs_{name}.png"
        with image_path.open("xb") as stream:
            fig.savefig(stream, format="png", dpi=config["plots"]["dpi"])
        plt.close(fig)
        records.append({**artifacts.file_record(image_path, run), "metric": key, "tau_s": tau, "values": values})
    save_json_exclusive(run / "plot_manifest.json", {"metrics_input": artifacts.file_record(path, run),
        "configuration": artifacts.file_record(run / "config_snapshot.yaml", run), "plots": records,
        "matplotlib_version": matplotlib.__version__, "x_axis": "controlled delay; not actual LightNav latency"})


def build_run(config_path: Path, output: Path) -> dict:
    config = artifacts.config_read(config_path)
    source = artifacts.previous.resolve_source(config["source_run"])
    run = output.resolve()
    if run.exists():
        raise FileExistsError("output run must be new")
    if run.is_relative_to(source) or source.is_relative_to(run):
        raise ValueError("output must be separate from source")
    record = artifacts.snapshot_source(source)
    _, previous_config, previous_record, successive, metadata, arrays, boundaries, previous_metrics = artifacts.load_source(record)
    if run.is_relative_to(successive) or successive.is_relative_to(run):
        raise ValueError("output must be separate from the successive source")
    rows = characterize_projection(boundaries, arrays["FRESH_world"], previous_metrics["conditions"],
        previous_config["motion"], distance_atol_m=config["numerical_validation"]["previous_d_poly_atol_m"])
    (run / "derived").mkdir(parents=True)
    (run / "evidence").mkdir()
    with (run / "config_snapshot.yaml").open("xb") as stream:
        stream.write(config_path.read_bytes())
    record.update({"schema_version": 1, "research_git_sha": artifacts.git("rev-parse", "HEAD"),
        "created_time": make_observation_time(0.0), "configuration": artifacts.file_record(run / "config_snapshot.yaml", run),
        "R_obs": metadata["R1"], "motion": previous_config["motion"],
        "observation_time": previous_record["source_observation_time"],
        "source_reviewed_git_sha": artifacts.read_json(source / "git_completion.json")["final_git_sha"],
        "source_inputs": {k: artifacts.file_record(source / p, source) for k, p in (
            ("boundaries", "derived/boundaries.npy"), ("previous_metrics", "derived/metrics.json"), ("provenance", "source.json"))},
        "successive_source_run": str(successive), "trajectory_inputs": previous_record["array_inputs"],
        "processing_source_sha256": artifacts.source_hashes(), "new_lightnav_inference_count": 0,
        "actual_execution_time": None, "new_inference_readiness_time": None,
        "incoming_direction_definition": DEFINITIONS["phi_in_rad"], "controlled_motion_is_surrogate": True})
    save_json_exclusive(run / "source.json", record)
    metrics = {"schema_version": 1, "definitions": DEFINITIONS, "conditions": rows,
        "source_json_sha256": sha256_file(run / "source.json"), "configuration": record["configuration"],
        "fixed_fresh_world_input": record["trajectory_inputs"]["FRESH_world"], "fresh_reanchored": False,
        "old_role": "context only; unused by projection metrics"}
    save_json_exclusive(run / "derived/projection_metrics.json", metrics)
    flat = [csv_row(row) for row in rows]
    with (run / "derived/projection_metrics.csv").open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flat[0]))
        writer.writeheader()
        writer.writerows(flat)
    make_plots(run, config)
    artifacts.load_source(record)
    save_json_exclusive(run / "analysis_validation.json", {"source_hashes_unchanged": True,
        "source_json_sha256": sha256_file(run / "source.json"), "new_lightnav_inference_count": 0,
        "outputs": [artifacts.file_record(run / p, run) for p in ("derived/projection_metrics.json", "derived/projection_metrics.csv", "plot_manifest.json")]})
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--config", type=Path, default=artifacts.ROOT / "configs/robotless_projection_handoff.yaml")
    args = parser.parse_args()
    result = build_run(args.config.resolve(), args.run_directory)
    for r in result["conditions"]:
        print(f"tau={r['tau_s']:g} j={r['segment_index']} alpha={r['alpha']:.12f} Q={r['Q_xy_world_m']} e_perp={r['e_perp_m']:.12f} s={r['s_Q_m']:.12f}")


if __name__ == "__main__":
    main()

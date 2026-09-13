#!/usr/bin/env python3
"""Validate and visualize new physical executions of frozen EXP-02D candidates."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from reconciliation.closed_loop_execution_validation import validate_closed_loop_trial
from reconciliation.exp02d_execution import summarize_execution
from reconciliation.online_switch import sha256_file


def validate_run(run):
    protocol = json.loads((run / "protocol.json").read_text())
    summary = json.loads((run / "summary.json").read_text())
    for row in summary["trials"]:
        trial = Path(row["directory"])
        if not trial.resolve().is_relative_to(run.resolve()):
            raise ValueError("trial escapes run directory")
        result = validate_closed_loop_trial(trial)
        if result["metrics"] != row["metrics"]:
            raise ValueError("trial and summary metrics differ")
        metadata = json.loads((trial / "metadata.json").read_text())
        if metadata["protocol_sha256"] != sha256_file(run / "protocol.json"):
            raise ValueError("protocol hash mismatch")
        if metadata["reference_sha256"] != sha256_file(Path(metadata["reference_path"])):
            raise ValueError("candidate reference changed")
        for name, digest in row["raw_sha256"].items():
            if sha256_file(trial / name) != digest:
                raise ValueError("raw artifact changed")
    config = protocol["config"]
    derived = summarize_execution(summary["trials"], config["cases"], config["methods"], protocol["acceptance_criteria"])
    if any(summary[key] != value for key, value in derived.items()):
        raise ValueError("physical outcomes do not reproduce from trials")
    if summary["trial_count"] != len(summary["trials"]):
        raise ValueError("trial count mismatch")
    return protocol, summary


def main():
    if len(sys.argv) not in (2, 3):
        raise SystemExit("usage: summarize_exp02d_physical_execution.py RUN [--validate-only|--gui-plots]")
    run = Path(sys.argv[1]).resolve()
    protocol, summary = validate_run(run)
    option = sys.argv[2] if len(sys.argv) == 3 else None
    if option and option not in ("--validate-only", "--gui-plots"):
        raise ValueError("unknown argument")
    if option == "--validate-only":
        print(json.dumps({"valid": True, "trial_count": summary["trial_count"], "gui_selection": summary["gui_selection"]}))
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    if option == "--gui-plots":
        output = run / "gui_plots"
        output.mkdir(exist_ok=False)
        plots = {}
        for case in protocol["config"]["cases"]:
            reference = np.load(run / "references" / case / "M3_LOOKAHEAD.npy", allow_pickle=False)
            actual = np.load(run / "trials" / case / "M3_LOOKAHEAD/repetition_00/raw/actual_trajectory.npy", allow_pickle=False)
            fig, ax = plt.subplots(figsize=(5, 3.2), layout="constrained")
            ax.plot(reference[:, 0], reference[:, 1], "--", c="#c02da5", lw=2, label="M3 corrected path")
            ax.plot(actual[:, 0], actual[:, 1], c="#168a4e", lw=1.8, label="Primary measured motion")
            ax.scatter(*actual[0, :2], c="#f7bd21", edgecolors="black", s=35, zorder=4, label="Start / end X")
            ax.scatter(*actual[-1, :2], c="#168a4e", marker="x", s=45, zorder=4)
            ax.set_aspect("equal", adjustable="datalim")
            ax.ticklabel_format(useOffset=False, style="plain")
            ax.set(xlabel="World X [m]", ylabel="World Y [m]", title=f"{case}: primary physical trajectory")
            ax.grid(alpha=.2)
            ax.legend(fontsize=7)
            path = output / f"{case}_xy.png"
            fig.savefig(path, dpi=140)
            plt.close(fig)
            plots[path.name] = sha256_file(path)
        with (output / "manifest.json").open("x") as stream:
            json.dump({"source_summary_sha256": sha256_file(run / "summary.json"),
                       "processing_script_sha256": sha256_file(Path(__file__)), "plots": plots,
                       "semantics": "primary repetition 00; world XY metres unchanged; not live GUI telemetry"}, stream, indent=2)
        print(json.dumps({"gui_plots": str(output), "plots": plots}))
        return
    output = run / "plots"
    output.mkdir(exist_ok=False)
    colors = {"M0_RAW": "#df7825", "M3_LOOKAHEAD": "#168a4e"}
    reference_colors = {"M0_RAW": "#8a8a8a", "M3_LOOKAHEAD": "#c02da5"}
    labels = {"M0_RAW": "RAW", "M3_LOOKAHEAD": "M3 corrected"}
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    plots = {}
    for case in protocol["config"]["cases"]:
        fig, axes = plt.subplots(2, 2, figsize=(13, 9), layout="constrained")
        info = json.loads((run / "references" / case / "input_reference.json").read_text())
        b = info["B_world_se2"]
        for method in protocol["config"]["methods"]:
            reference = np.load(run / "references" / case / f"{method}.npy", allow_pickle=False)
            axes[0, 0].plot(reference[:, 0], reference[:, 1], "--", c=reference_colors[method], lw=2,
                            label=f"{labels[method]} reference")
            for row in summary["trials"]:
                if row["case"] != case or row["method"] != method:
                    continue
                trial = Path(row["directory"])
                actual = np.load(trial / "raw/actual_trajectory.npy", allow_pickle=False)
                with (trial / "raw/telemetry.csv").open() as stream:
                    data = list(csv.DictReader(stream))
                col = lambda key: np.asarray([float(item[key]) for item in data])
                label = f"{labels[method]} actual" if row["repetition"] == 0 else None
                axes[0, 0].plot(actual[:, 0], actual[:, 1], c=colors[method], lw=1.6, label=label)
                if row["repetition"] == 0:
                    axes[0, 0].scatter(*actual[-1, :2], c=colors[method], marker="x", s=60)
                times = col("sim_time_s")
                axes[0, 1].plot(times, col("nearest_reference_distance_m") * 100, c=colors[method], label=label)
                axes[1, 0].plot(times, np.abs(col("nearest_reference_yaw_error_rad")), c=colors[method], label=label)
        axes[0, 0].scatter(*b[:2], marker="o", c="#f7bd21", edgecolors="black", s=60, label="Saved switch B")
        axes[0, 0].set_aspect("equal", adjustable="datalim")
        axes[0, 0].ticklabel_format(useOffset=False, style="plain")
        axes[0, 0].set(xlabel="World X [m]", ylabel="World Y [m]", title="Paths and physical Jackal motion")
        axes[0, 0].legend(fontsize=8)
        axes[0, 1].set(xlabel="Execution time [s]", ylabel="Distance to own reference [cm]", title="Spatial tracking error")
        axes[1, 0].set(xlabel="Execution time [s]", ylabel="Absolute yaw error [rad]", title="Heading at nearest reference point")
        for ax in (axes[0, 0], axes[0, 1], axes[1, 0]):
            ax.grid(alpha=.2)
        axes[1, 1].axis("off")
        text = [info["corpus_transition_id"], "", "Three repetitions; goal or 18 s timeout", ""]
        for method in protocol["config"]["methods"]:
            outcome = summary["outcomes"][case][method]
            rows = outcome["trial_metrics"]
            text.extend([f"{labels[method]}: {outcome['physical_tracking_status']}",
                         f"  Goal: {outcome['goal_successes']}/3; position RMS: {100*np.mean([r['position_rmse_m'] for r in rows]):.2f} cm",
                         f"  Yaw RMS: {np.mean([r['yaw_rmse_rad'] for r in rows]):.3f} rad",
                         f"  Maximum deviation: {100*max(r['max_position_error_m'] for r in rows):.2f} cm",
                         "  Failed gates: " + (", ".join(outcome["failed_checks"]) or "none"), ""])
        text.extend(["Hospital / current frozen controller", "Reset at B; prior velocity and PI history not restored",
                     "Physical tracking status differs from old S/F labels"])
        axes[1, 1].text(0, 1, "\n".join(text), va="top", fontsize=10)
        status = summary["outcomes"][case]["M3_LOOKAHEAD"]["physical_tracking_status"]
        fig.suptitle(f"EXP02D post-hoc physical execution | {case} | M3 tracking: {status}", fontsize=15)
        path = output / f"{case}_physical_comparison.png"
        fig.savefig(path, dpi=160)
        plt.close(fig)
        plots[path.name] = sha256_file(path)
    manifest = {"source_summary_sha256": sha256_file(run / "summary.json"),
                "processing_script_sha256": sha256_file(Path(__file__)), "plots": plots,
                "coordinate_units": "world XY metres; plot cm = m*100; no transform",
                "gui_selection": summary["gui_selection"], "validated_trial_count": summary["trial_count"]}
    with (output / "manifest.json").open("x") as stream:
        json.dump(manifest, stream, indent=2)
        stream.write("\n")
    print(json.dumps(manifest))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Generate the ten frozen EXP-02D aggregate plots from completed results."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402


METHODS = ("M0_RAW", "M1_HISTORICAL_M4", "M2_NO_DIRECTION", "M3_LOOKAHEAD")
METHOD_LABELS = ("RAW", "M1 historical", "M2 no direction", "M3 lookahead")
METHOD_COLORS = ("#777777", "#f28e2b", "#17becf", "#d627a9")


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    return parser.parse_args()


def strict_json(path: Path) -> Any:
    return json.loads(
        path.read_text(encoding="utf-8"),
        parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)),
    )


def save(figure: plt.Figure, path: Path) -> None:
    if path.exists():
        raise FileExistsError(f"refusing to overwrite EXP-02D plot: {path}")
    figure.tight_layout()
    figure.savefig(path, dpi=170, bbox_inches="tight")
    plt.close(figure)


def method_value(row: dict[str, Any], method: str, section: str, key: str) -> float:
    return float(row["methods"][method][section][key])


def _outcome_counts(rows: Iterable[dict[str, Any]]) -> dict[str, int]:
    values = {
        "CHALLENGING_RESCUED": 0,
        "CHALLENGING_IMPROVED": 0,
        "CHALLENGING_MIXED": 0,
        "CHALLENGING_WORSE": 0,
    }
    for row in rows:
        label = str(row["success_failure_label"])
        if label in values:
            values[label] += 1
    return values


def generate(run: Path) -> tuple[Path, ...]:
    root = run.expanduser().resolve()
    records = strict_json(root / "summary/plot_records.json")["records"]
    pair_summary = strict_json(root / "summary/pair_balanced.json")
    config = yaml.safe_load((root / "config_snapshot.yaml").read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("EXP-02D config snapshot must be a mapping")
    if not records:
        raise ValueError("EXP-02D plot records are empty")
    plots = root / "plots"
    plots.mkdir(exist_ok=False)
    written: list[Path] = []

    # 01 — pair-balanced command score.
    values = [float(pair_summary["methods"][method]["mean_J_cmd"]) for method in METHODS]
    figure, axis = plt.subplots(figsize=(8.2, 5.0))
    axis.bar(METHOD_LABELS, values, color=METHOD_COLORS)
    axis.set_ylabel("Pair-balanced mean $J_{cmd}$ [dimensionless]")
    axis.set_ylim(bottom=0.0)
    axis.set_title("EXP-02D immediate desired-command discontinuity")
    axis.tick_params(axis="x", rotation=12)
    path = plots / "01_method_command_score_pair_balanced.png"
    save(figure, path); written.append(path)

    # 02 — benign preservation, using frozen RAW-benign membership.
    benign = [row for row in records if row["raw_difficulty"] == "BENIGN"]
    preserved = []
    broken = []
    for method in METHODS:
        keep = sum(bool(row["methods"][method]["command"]["difficulty"] == "BENIGN") for row in benign)
        preserved.append(keep / len(benign) if benign else 0.0)
        broken.append(1.0 - preserved[-1])
    figure, axis = plt.subplots(figsize=(8.2, 5.0))
    x = np.arange(len(METHODS))
    axis.bar(x, preserved, color="#59a14f", label="preserved")
    axis.bar(x, broken, bottom=preserved, color="#e15759", label="broken")
    axis.set_xticks(x, METHOD_LABELS, rotation=12)
    axis.set_ylabel("Fraction of RAW-benign transitions")
    axis.set_ylim(0.0, 1.0)
    axis.legend()
    axis.set_title("Benign preservation under frozen thresholds")
    path = plots / "02_benign_preservation.png"
    save(figure, path); written.append(path)

    # 03 — predeclared M3 outcomes on RAW-challenging cases.
    challenging = [row for row in records if row["raw_difficulty"] == "CHALLENGING"]
    outcomes = _outcome_counts(challenging)
    keys = tuple(outcomes)
    figure, axis = plt.subplots(figsize=(8.4, 5.0))
    axis.bar(
        ["rescued", "improved", "mixed", "worse"],
        [outcomes[key] for key in keys],
        color=("#59a14f", "#76b7b2", "#bab0ab", "#e15759"),
    )
    axis.set_ylabel("RAW-challenging transition count")
    axis.set_ylim(bottom=0.0)
    axis.set_title("M3 lookahead outcomes (predeclared labels)")
    path = plots / "03_challenging_outcomes.png"
    save(figure, path); written.append(path)

    # 04 — raw geometry mismatch comparison, colored by M3 relative benefit.
    x_entry = np.asarray([row["geometry"]["alpha_entry_rad"] for row in records], float)
    y_look = np.asarray([row["geometry"]["alpha_look_rad"] for row in records], float)
    relative = np.asarray(
        [
            (method_value(row, "M0_RAW", "command", "J_cmd")
             - method_value(row, "M3_LOOKAHEAD", "command", "J_cmd"))
            / max(method_value(row, "M0_RAW", "command", "J_cmd"), 1e-12)
            for row in records
        ],
        float,
    )
    limit = max(float(np.max(np.abs(relative))), 1e-9)
    figure, axis = plt.subplots(figsize=(7.2, 6.0))
    scatter = axis.scatter(np.degrees(x_entry), np.degrees(y_look), c=relative,
                           cmap="coolwarm_r", vmin=-limit, vmax=limit, s=15, alpha=0.65)
    axis.set_xlabel(r"$\alpha_{entry}$ [deg]")
    axis.set_ylabel(r"$\alpha_{look}$ [deg]")
    axis.set_xlim(left=0.0); axis.set_ylim(bottom=0.0)
    axis.set_title("Entry versus execution-lookahead direction")
    figure.colorbar(scatter, ax=axis, label="Relative M3 command-score benefit")
    path = plots / "04_alpha_entry_vs_alpha_look.png"
    save(figure, path); written.append(path)

    # 05 — direct semantics hypothesis.
    semantics_gain = x_entry - y_look
    m1_minus_m3 = np.asarray(
        [method_value(row, "M1_HISTORICAL_M4", "command", "J_cmd")
         - method_value(row, "M3_LOOKAHEAD", "command", "J_cmd") for row in records],
        float,
    )
    figure, axis = plt.subplots(figsize=(7.5, 5.4))
    axis.scatter(np.degrees(semantics_gain), m1_minus_m3, s=15, alpha=0.6, color="#4e79a7")
    axis.axhline(0.0, color="black", linewidth=0.8)
    axis.axvline(0.0, color="black", linewidth=0.8)
    axis.set_xlabel(r"$\alpha_{entry}-\alpha_{look}$ [deg]")
    axis.set_ylabel(r"$J_{M1}-J_{M3}$ [dimensionless]")
    axis.set_title("Geometry semantics gain")
    path = plots / "05_geometry_semantics_gain.png"
    save(figure, path); written.append(path)

    # 06 — amount changed versus immediate command benefit.
    deformation = np.asarray(
        [method_value(row, "M3_LOOKAHEAD", "deformation", "translation_deformation_rms_m") for row in records],
        float,
    )
    raw_minus_m3 = np.asarray(
        [method_value(row, "M0_RAW", "command", "J_cmd")
         - method_value(row, "M3_LOOKAHEAD", "command", "J_cmd") for row in records],
        float,
    )
    figure, axis = plt.subplots(figsize=(7.5, 5.4))
    axis.scatter(deformation, raw_minus_m3, s=15, alpha=0.6, color="#b07aa1")
    axis.axhline(0.0, color="black", linewidth=0.8)
    axis.set_xlabel("M3 translation deformation RMS [m]")
    axis.set_ylabel(r"$J_{RAW}-J_{M3}$ [dimensionless]")
    axis.set_xlim(left=0.0)
    axis.set_title("Candidate deformation versus command benefit")
    path = plots / "06_deformation_vs_benefit.png"
    save(figure, path); written.append(path)

    # 07 — residual from one best-fit left rigid transform.
    rigid = [
        [method_value(row, method, "deformation", "rigid_fit_translation_rms_m") for row in records]
        for method in METHODS[1:]
    ]
    figure, axis = plt.subplots(figsize=(8.0, 5.2))
    axis.boxplot(rigid, tick_labels=METHOD_LABELS[1:], showfliers=True)
    axis.set_ylabel("Best-fit rigid-transform translation RMS [m]")
    axis.set_ylim(bottom=0.0)
    axis.set_title("Rigid-like propagation diagnostic")
    axis.tick_params(axis="x", rotation=10)
    path = plots / "07_rigid_fit_distribution.png"
    save(figure, path); written.append(path)

    # 08 — success/failure composition by frozen FRESH geometry bin.
    geometries = ("STRAIGHT_LIKE", "POSITIVE_TURNING", "NEGATIVE_TURNING", "OTHER")
    labels = sorted({str(row["success_failure_label"]) for row in records})
    figure, axis = plt.subplots(figsize=(10.0, 5.8))
    bottom = np.zeros(len(geometries))
    for index, label in enumerate(labels):
        counts = np.asarray([
            sum(row["fresh_geometry"] == geometry and row["success_failure_label"] == label for row in records)
            for geometry in geometries
        ], float)
        axis.bar(geometries, counts, bottom=bottom, label=label,
                 color=plt.cm.tab20(index % 20))
        bottom += counts
    axis.set_ylabel("Transition count")
    axis.set_ylim(bottom=0.0)
    axis.set_title("M3 success/failure labels by raw FRESH geometry")
    axis.legend(fontsize=7, loc="upper right")
    axis.tick_params(axis="x", rotation=10)
    path = plots / "08_success_failure_by_fresh_geometry.png"
    save(figure, path); written.append(path)

    # 09 — outcome by predeclared latency bins.
    latency_edges = np.asarray(config["regimes"]["latency_bin_edges_s"], dtype=float)
    if (
        latency_edges.shape != (2,)
        or not np.all(np.isfinite(latency_edges))
        or not latency_edges[0] < latency_edges[1]
    ):
        raise ValueError("EXP-02D latency bins must contain two increasing finite edges")
    low, high = (float(value) for value in latency_edges)
    latency_labels = (f"<={low:.2f} s", f"({low:.2f},{high:.2f}] s", f">{high:.2f} s")
    latency_groups = (
        [row for row in records if float(row["effective_latency_s"]) <= low],
        [row for row in records if low < float(row["effective_latency_s"]) <= high],
        [row for row in records if float(row["effective_latency_s"]) > high],
    )
    figure, axis = plt.subplots(figsize=(9.0, 5.5))
    bottom = np.zeros(3)
    for index, label in enumerate(labels):
        counts = np.asarray([sum(row["success_failure_label"] == label for row in group)
                             for group in latency_groups], float)
        axis.bar(latency_labels, counts, bottom=bottom, label=label,
                 color=plt.cm.tab20(index % 20))
        bottom += counts
    axis.set_ylabel("Transition count")
    axis.set_ylim(bottom=0.0)
    axis.set_title("M3 success/failure labels by effective latency")
    axis.legend(fontsize=7, loc="upper right")
    path = plots / "09_success_failure_by_latency.png"
    save(figure, path); written.append(path)

    # 10 — duplicated-pair frequency versus lookahead/historical effect.
    frequency = np.asarray([row["ordered_raw_pair_frequency"] for row in records], float)
    figure, axis = plt.subplots(figsize=(7.6, 5.4))
    axis.scatter(frequency, m1_minus_m3, s=15, alpha=0.55, color="#f28e2b")
    axis.axhline(0.0, color="black", linewidth=0.8)
    axis.set_xscale("log")
    axis.set_xlabel("Exact ordered raw-pair frequency [count, log scale]")
    axis.set_ylabel(r"$J_{M1}-J_{M3}$ [dimensionless]")
    axis.set_title("Pair frequency versus method effect")
    path = plots / "10_pair_frequency_vs_method_effect.png"
    save(figure, path); written.append(path)

    if len(written) != 10 or any(not path.is_file() for path in written):
        raise RuntimeError("EXP-02D did not produce exactly ten aggregate plots")
    return tuple(written)


def main() -> None:
    paths = generate(arguments().run)
    print(f"EXP02D_PLOTS={len(paths)}")
    for path in paths:
        print(path)


if __name__ == "__main__":
    main()

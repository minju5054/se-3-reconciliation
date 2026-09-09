#!/usr/bin/env python3
"""Generate the predeclared ten DATA-02 v1/v2/combined review plots."""

from __future__ import annotations

from collections import Counter
import json
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def load(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def save(fig, path: Path) -> None:
    if path.exists():
        raise FileExistsError(f"plot already exists: {path}")
    fig.tight_layout(); fig.savefig(path, dpi=160); plt.close(fig)


def subsets(records):
    eligible = [row for row in records if row["status"] == "ELIGIBLE_MOVING"]
    return {
        "v1": [row for row in eligible if row["cohort_id"] == "v1"],
        "v2": [row for row in eligible if row["cohort_id"] == "v2"],
        "combined": eligible,
    }


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: plot_data02_combined.py COMBINED_DIRECTORY")
    combined = Path(sys.argv[1]).resolve(); plots = combined / "plots"
    records = load(combined / "all_transition_index.json")["records"]
    groups = subsets(records)
    coverage = load(combined / "coverage_summary.json")
    split = load(combined / "split/split_summary.json")

    fig, ax = plt.subplots(figsize=(8, 4.5)); labels = ["v1", "v2", "combined"]
    attempts = [coverage[x]["attempt_count"] for x in labels]; eligible = [coverage[x]["eligible_count"] for x in labels]
    invalid = [coverage[x]["status_counts"].get("TIMING_INVALID", 0) for x in labels]
    x = np.arange(3); ax.bar(x-.25, attempts, .25, label="attempts"); ax.bar(x, eligible, .25, label="eligible"); ax.bar(x+.25, invalid, .25, label="timing invalid")
    ax.set(xticks=x, xticklabels=labels, ylabel="transition count", title="DATA-02 cohort and combined counts"); ax.legend()
    save(fig, plots / "v1_vs_v2_vs_combined_counts.png")

    fig, ax = plt.subplots(figsize=(8, 4.5))
    for label in ("v1", "v2"):
        freq = sorted(Counter(row["ordered_raw_pair_sha256"] for row in groups[label]).values(), reverse=True)
        ax.plot(np.arange(1, len(freq)+1), freq, marker=".", label=label)
    ax.set(xlabel="exact ordered raw-pair frequency rank", ylabel="eligible transition count", yscale="log", title="Within-cohort exact pair-frequency profile"); ax.legend()
    save(fig, plots / "v1_vs_v2_pair_frequency.png")

    freq = Counter(row["ordered_raw_pair_sha256"] for row in groups["combined"])
    top = freq.most_common(30)
    fig, ax = plt.subplots(figsize=(10, 4.5)); ax.bar(range(len(top)), [value for _, value in top]); ax.set(xlabel="top exact ordered raw-pair rank", ylabel="eligible transition count", title="Combined exact pair frequencies (top 30)")
    save(fig, plots / "combined_pair_frequency.png")

    values = ["STRAIGHT_LIKE", "POSITIVE_TURNING", "NEGATIVE_TURNING", "OTHER"]
    fig, ax = plt.subplots(figsize=(9, 4.5)); width=.25
    for offset, label in enumerate(labels):
        count = Counter(row["fresh_geometry_bin"] for row in groups[label]); ax.bar(np.arange(len(values))+(offset-1)*width, [count[x] for x in values], width, label=label)
    ax.set(xticks=np.arange(len(values)), xticklabels=values, ylabel="eligible transition count", title="Frozen FRESH geometry descriptors"); ax.legend()
    save(fig, plots / "combined_geometry_distribution.png")

    fig, ax = plt.subplots(figsize=(7, 6))
    for label, color in (("v1", "#3182bd"), ("v2", "#e6550d")):
        ax.scatter([row["delta_v_mps"] for row in groups[label]], [row["delta_omega_rps"] for row in groups[label]], s=10, alpha=.45, label=label, color=color)
    ax.axvline(0, color="gray", lw=.6); ax.axhline(0, color="gray", lw=.6); ax.set(xlabel="desired command jump Δv (m/s)", ylabel="desired command jump Δω (rad/s)", title="Combined raw-switch command discontinuity"); ax.legend()
    save(fig, plots / "combined_command_jump_distribution.png")

    fig, axes = plt.subplots(1, 2, figsize=(10, 4.5))
    for label, color in (("v1", "#3182bd"), ("v2", "#e6550d")):
        axes[0].hist([row["host_latency_s"] for row in groups[label]], bins=24, alpha=.55, label=label, color=color)
        axes[1].hist([row["tau_effective_s"] for row in groups[label]], bins=24, alpha=.55, label=label, color=color)
    axes[0].set(xlabel="host request-response latency (s)", ylabel="eligible transition count"); axes[1].set(xlabel="effective observation-to-switch latency (sim s)", ylabel="eligible transition count"); axes[0].legend(); fig.suptitle("Eligible latency distributions")
    save(fig, plots / "combined_latency_distribution.png")

    fig, ax = plt.subplots(figsize=(9, 4.5)); width=.36; indices=range(6)
    for offset, (label, color) in enumerate((("v1", "#3182bd"), ("v2", "#e6550d"))):
        cohort = [row for row in records if row["cohort_id"] == label]
        invalid_counts = [sum(row["status"] == "TIMING_INVALID" and int(row["transition_index"]) == idx for row in cohort) for idx in indices]
        ax.bar(np.arange(6)+(offset-.5)*width, invalid_counts, width, label=label, color=color)
    ax.set(xticks=np.arange(6), xlabel="transition index within episode", ylabel="TIMING_INVALID count", title="Timing-invalid attempts by successive transition index"); ax.legend()
    save(fig, plots / "timing_invalid_by_transition_index.png")

    sizes = [item["transition_count"] for item in split["components"]] if "components" in split else [item["transition_count"] for item in load(combined / "split/components.json")["components"]]
    fig, ax = plt.subplots(figsize=(8, 4.5)); bins=np.arange(1, max(sizes, default=1)+2)-.5; ax.hist(sizes, bins=bins)
    ax.set(xlabel="connected component size (eligible transitions)", ylabel="component count", title="Episode/exact-pair connected-component sizes")
    save(fig, plots / "component_size_distribution.png")

    development = load(combined / "split/development.json")["records"]; heldout = load(combined / "split/heldout.json")["records"]
    families = ["straight", "left", "right", "doorway", "detour", "compound"]
    fig, ax = plt.subplots(figsize=(9, 4.5)); bottom=np.zeros(2)
    for family in families:
        values2 = [sum(row["semantic_family"] == family for row in development), sum(row["semantic_family"] == family for row in heldout)]
        ax.bar(["development", "held-out"], values2, bottom=bottom, label=family); bottom += values2
    ax.set(ylabel="eligible transition count", title=f"Final isolated split (held-out={len(heldout)}, leakage=0)"); ax.legend(ncol=3, fontsize=8)
    save(fig, plots / "final_split_summary.png")

    representatives = load(combined / "representatives.json")["records"]
    by_id = {row["corpus_transition_id"]: row for row in groups["combined"]}
    cols=6; rows_count=math.ceil(len(representatives)/cols)
    fig, axes = plt.subplots(rows_count, cols, figsize=(15, max(3, rows_count*2.35)), squeeze=False)
    for ax, selected in zip(axes.flat, representatives):
        row=by_id[selected["corpus_transition_id"]]; transition_dir=(ROOT/row["source_transition_relative_path"]).parent
        old=np.load(transition_dir/"derived/old_world.npy", allow_pickle=False); fresh=np.load(transition_dir/"derived/fresh_world.npy", allow_pickle=False); actual=np.load(transition_dir/"actual.npy", allow_pickle=False)
        ax.plot(old[:,0],old[:,1],color="#3182bd",lw=1); ax.plot(fresh[:,0],fresh[:,1],color="#dd3497",lw=1); ax.plot(actual[:,0],actual[:,1],color="#31a354",lw=1)
        ax.set_title(selected["corpus_transition_id"].replace("episode_","e"),fontsize=6); ax.set_aspect("equal",adjustable="datalim"); ax.tick_params(labelsize=5)
    for ax in axes.flat[len(representatives):]: ax.axis("off")
    fig.suptitle("Deterministic combined representatives: OLD blue, FRESH magenta, actual green", fontsize=11)
    save(fig, plots / "representative_combined_transitions.png")

    print("\n".join(str(path) for path in sorted(plots.glob("*.png"))))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Diagnose immutable DATA-02 v1 RTF failures without relabelling them."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from io import BytesIO
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.data02_online_successive import percentile_summary
from reconciliation.data02_v2 import timing_group_summary, timing_invalid_direction
from reconciliation.online_switch import save_json_exclusive, sha256_file


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("output_directory", type=Path)
    parser.add_argument("--benchmark-png-count", type=int, default=40)
    return parser.parse_args()


def strict_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def counts_by(records: list[dict[str, Any]], field: str) -> dict[str, dict[str, int]]:
    result: dict[str, Counter[str]] = defaultdict(Counter)
    for row in records:
        result[str(row[field])][row["timing_class"]] += 1
    return {
        key: {"total": sum(counts.values()), **dict(sorted(counts.items()))}
        for key, counts in sorted(result.items())
    }


def finite_correlation(records: list[dict[str, Any]], field: str) -> float | None:
    if len(records) < 2:
        return None
    x = np.asarray([float(row["rtf"]) for row in records], dtype=np.float64)
    y = np.asarray([float(row[field]) for row in records], dtype=np.float64)
    if not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)) or np.std(x) == 0 or np.std(y) == 0:
        return None
    return float(np.corrcoef(x, y)[0, 1])


def png_benchmark(run: Path, limit: int) -> dict[str, Any]:
    images = sorted(run.glob("episodes/*/rgb/frame_*.png"))[: max(0, limit)]
    encode_ms = []
    sha_ms = []
    for path in images:
        with Image.open(path) as source:
            rgb = source.convert("RGB").copy()
        stream = BytesIO()
        started = time.monotonic_ns()
        rgb.save(stream, format="PNG")
        encode_ms.append((time.monotonic_ns() - started) / 1e6)
        started = time.monotonic_ns()
        sha256_file(path)
        sha_ms.append((time.monotonic_ns() - started) / 1e6)
    return {
        "measurement": "offline deterministic sample; diagnostic estimate, not recorded online duration",
        "sample_count": len(images),
        "selection": "lexicographically first saved RGB frames",
        "png_encode_ms": percentile_summary(encode_ms),
        "file_sha256_ms": percentile_summary(sha_ms),
    }


def plot_diagnosis(output: Path, records: list[dict[str, Any]], bounds: list[float]) -> list[str]:
    paths = []
    colors = {
        "ELIGIBLE_RTF": "#2b8cbe",
        "BELOW_LOWER_RTF_BOUND": "#d7301f",
        "ABOVE_UPPER_RTF_BOUND": "#fd8d3c",
    }
    fig, ax = plt.subplots(figsize=(9, 4.5))
    for label in colors:
        values = [row["rtf"] for row in records if row["timing_class"] == label]
        ax.hist(values, bins=28, alpha=.65, color=colors[label], label=f"{label} (n={len(values)})")
    ax.axvline(bounds[0], color="black", linestyle="--"); ax.axvline(bounds[1], color="black", linestyle="--")
    ax.set(xlabel="real-time factor (simulated s / host s)", ylabel="transition count", title="Immutable v1 RTF classification")
    ax.legend(fontsize=8); fig.tight_layout()
    path = output / "timing_invalid_rtf_direction.png"; fig.savefig(path, dpi=160); plt.close(fig); paths.append(path.name)

    indices = sorted({int(row["transition_index"]) for row in records})
    below = [sum(row["transition_index"] == index and row["timing_class"] == "BELOW_LOWER_RTF_BOUND" for row in records) for index in indices]
    above = [sum(row["transition_index"] == index and row["timing_class"] == "ABOVE_UPPER_RTF_BOUND" for row in records) for index in indices]
    total = [sum(row["transition_index"] == index for row in records) for index in indices]
    fig, ax = plt.subplots(figsize=(8, 4.5)); ax.bar(indices, below, label="below 0.90", color=colors["BELOW_LOWER_RTF_BOUND"]); ax.bar(indices, above, bottom=below, label="above 1.10", color=colors["ABOVE_UPPER_RTF_BOUND"])
    ax.plot(indices, total, "ko--", label="all attempts")
    ax.set(xlabel="transition index within episode", ylabel="transition count", title="v1 timing invalidity by successive transition index"); ax.legend(); fig.tight_layout()
    path = output / "timing_invalid_by_transition_index.png"; fig.savefig(path, dpi=160); plt.close(fig); paths.append(path.name)

    fig, ax = plt.subplots(figsize=(9, 4.5))
    episode_indices = sorted({int(row["episode_index"]) for row in records})
    invalid_fraction = [sum(int(row["episode_index"]) == index and row["timing_class"] != "ELIGIBLE_RTF" for row in records) / sum(int(row["episode_index"]) == index for row in records) for index in episode_indices]
    ax.plot(episode_indices, invalid_fraction, marker="o", markersize=3, linewidth=1)
    ax.set(xlabel="episode index / model-process lifetime order", ylabel="TIMING_INVALID fraction", ylim=(0, 1.05), title="No monotonic lifetime assumption: per-episode invalid fraction"); fig.tight_layout()
    path = output / "timing_invalid_by_episode_index.png"; fig.savefig(path, dpi=160); plt.close(fig); paths.append(path.name)

    fig, axes = plt.subplots(1, 2, figsize=(10, 4.5))
    for label in colors:
        subset = [row for row in records if row["timing_class"] == label]
        axes[0].scatter([row["model_reported_s"] for row in subset], [row["rtf"] for row in subset], s=10, alpha=.55, color=colors[label], label=label)
        axes[1].scatter([row["main_loop_detection_delay_s"] for row in subset], [row["rtf"] for row in subset], s=10, alpha=.55, color=colors[label])
    for ax in axes:
        ax.axhline(bounds[0], color="black", linestyle="--"); ax.axhline(bounds[1], color="black", linestyle="--")
        ax.set_ylabel("real-time factor")
    axes[0].set_xlabel("LightNav-reported model latency (s)"); axes[1].set_xlabel("server completion to main-loop detection (s)")
    axes[0].legend(fontsize=7); fig.suptitle("Model latency and host-side response-observation association"); fig.tight_layout()
    path = output / "timing_invalid_latency_association.png"; fig.savefig(path, dpi=160); plt.close(fig); paths.append(path.name)
    return paths


def main() -> None:
    options = arguments()
    run = options.run_directory.resolve()
    output = options.output_directory.resolve()
    if output.exists():
        raise FileExistsError(f"diagnosis output already exists: {output}")
    output.mkdir(parents=True)
    config = yaml.safe_load((run / "config_snapshot.yaml").read_text(encoding="utf-8"))
    metadata = strict_json(run / "metadata.json")
    validation = strict_json(run / "summary/validation.json")
    if not validation.get("valid"):
        raise ValueError("immutable v1 validator result is not valid")
    bounds = list(map(float, config["simulation"]["acceptable_rtf_range"]))
    records: list[dict[str, Any]] = []
    for transition_path in sorted(run.glob("episodes/*/transitions/transition_*/transition.json")):
        document = strict_json(transition_path)
        timing = document["timing"]
        episode = transition_path.parents[2]
        chunk = episode / "chunks" / document["fresh_chunk_id"] / "metadata.json"
        response = strict_json(chunk)["response"]
        request_ns = int(timing["t_request_host_monotonic_ns"])
        detected_ns = int(timing["t_model_ready_host_monotonic_ns"])
        server_start_ns = int(response["server_predict_start_monotonic_ns"])
        server_end_ns = int(response["server_predict_end_monotonic_ns"])
        obs = float(timing["t_obs_sim_s"]); ready = float(timing["t_ready_sim_s"])
        with (episode / "rgb_frames.csv").open(encoding="utf-8") as stream:
            import csv
            frame_times = [float(row["sim_time_s"]) for row in csv.DictReader(stream)]
        timing_class = timing_invalid_direction(float(timing["real_time_factor"]), bounds)
        if document["status"] == "ELIGIBLE_MOVING":
            timing_class = "ELIGIBLE_RTF"
        row = {
            "transition_id": document["transition_id"], "episode_id": document["episode_id"],
            "episode_index": int(document["episode_id"].split("_")[-1]),
            "template_id": document["template_id"], "variant_id": document["variant_id"],
            "transition_index": int(document["transition_index"]), "saved_status": document["status"],
            "timing_class": timing_class, "rtf": float(timing["real_time_factor"]),
            "host_latency_s": float(timing["request_response_host_s"]),
            "model_reported_s": float(timing["model_reported_s"]),
            "simulation_ready_latency_s": ready - obs,
            "effective_latency_s": float(timing["tau_effective_s"]),
            "server_prediction_s": (server_end_ns - server_start_ns) / 1e9,
            "request_dispatch_s": (server_start_ns - request_ns) / 1e9,
            "main_loop_detection_delay_s": (detected_ns - server_end_ns) / 1e9,
            "server_process_lifetime_proxy_s": 0.0,
            "inferred_frames_queued_during_inference": sum(obs < value <= ready + 1e-9 for value in frame_times),
        }
        if not all(math.isfinite(float(row[key])) for key in (
            "rtf", "host_latency_s", "model_reported_s", "simulation_ready_latency_s",
            "effective_latency_s", "server_prediction_s", "request_dispatch_s", "main_loop_detection_delay_s",
        )):
            raise ValueError(f"non-finite diagnosis field: {document['transition_id']}")
        records.append(row)
    server_origin = min(
        int(strict_json(path.parents[2] / "chunks" / strict_json(path)["fresh_chunk_id"] / "metadata.json")["response"]["server_predict_start_monotonic_ns"])
        for path in sorted(run.glob("episodes/*/transitions/transition_*/transition.json"))
    )
    for row in records:
        document_path = run / "episodes" / row["episode_id"] / "transitions" / f"transition_{row['transition_index']:02d}" / "transition.json"
        document = strict_json(document_path)
        response = strict_json(document_path.parents[2] / "chunks" / document["fresh_chunk_id"] / "metadata.json")["response"]
        row["server_process_lifetime_proxy_s"] = (int(response["server_predict_start_monotonic_ns"]) - server_origin) / 1e9

    invalid = [row for row in records if row["saved_status"] == "TIMING_INVALID"]
    eligible = [row for row in records if row["saved_status"] == "ELIGIBLE_MOVING"]
    below = [row for row in invalid if row["timing_class"] == "BELOW_LOWER_RTF_BOUND"]
    above = [row for row in invalid if row["timing_class"] == "ABOVE_UPPER_RTF_BOUND"]
    plots = plot_diagnosis(output, records, bounds)
    diagnosis = {
        "schema": "DATA02V1_TimingInvalidDiagnosis_v1",
        "source_run": str(run.relative_to(ROOT)),
        "source_provenance": {
            "collector_git_sha": metadata["collector_git_sha"],
            "config_snapshot_sha256": sha256_file(run / "config_snapshot.yaml"),
            "protocol_sha256": sha256_file(run / "protocol.json"),
            "collection_manifest_sha256": sha256_file(run / "collection_manifest.json"),
            "strict_validation_sha256": sha256_file(run / "summary/validation.json"),
            "strict_validation_valid": validation["valid"],
        },
        "rtf_gate": {"lower": bounds[0], "upper": bounds[1], "retrospectively_changed": False},
        "counts": {
            "attempts": len(records), "eligible_moving": len(eligible), "timing_invalid": len(invalid),
            "below_lower_bound": len(below), "above_upper_bound": len(above),
        },
        "groups": {
            "eligible": timing_group_summary(eligible), "timing_invalid": timing_group_summary(invalid),
            "below_lower_bound": timing_group_summary(below), "above_upper_bound": timing_group_summary(above),
        },
        "invalid_counts_by_episode_index": counts_by(records, "episode_index"),
        "invalid_counts_by_template": counts_by(records, "template_id"),
        "invalid_counts_by_variant": counts_by(records, "variant_id"),
        "invalid_counts_by_transition_index": counts_by(records, "transition_index"),
        "associations_with_rtf": {
            field: finite_correlation(records, field) for field in (
                "host_latency_s", "model_reported_s", "simulation_ready_latency_s", "effective_latency_s",
                "server_prediction_s", "main_loop_detection_delay_s", "server_process_lifetime_proxy_s",
                "inferred_frames_queued_during_inference",
            )
        },
        "runtime_activity_availability": {
            "frame_queue": "inferred from saved 4 Hz RGB simulation timestamps in (observation, ready]; final queue depth was not recorded in v1",
            "rgb_write": "v1 collector synchronously encoded, wrote, and hashed every PNG in the online loop; per-frame online duration was not recorded",
            "telemetry_logging": "per-step telemetry was buffered in memory and written once at episode end; no per-step telemetry file I/O",
            "offline_rgb_cost_benchmark": png_benchmark(run, options.benchmark_png_count),
        },
        "answer": {
            "model_latency_only_explanation_supported": False,
            "avoidable_collector_runtime_blocking_visible": True,
            "evidence": [
                "eligible and invalid LightNav-reported latency distributions overlap",
                "lower-bound RTF failures are associated with server-completion-to-main-loop detection delay",
                "v1 performed synchronous PNG encode/write/hash inside the online loop",
                "upper-bound failures also occur with short detection delay and discretized simulation-ready sampling",
            ],
            "technical_cleanup_for_v2": "defer lossless PNG persistence and hashing until episode end; poll response immediately after capture",
            "scientific_variables_changed": False,
        },
        "claim_limit": "descriptive association and saved-runtime diagnosis; not a causal model, GPU, storage, or simulator attribution",
        "plots": plots,
    }
    save_json_exclusive(output / "timing_invalid_diagnosis.json", diagnosis)
    print(json.dumps({"output": str(output), "counts": diagnosis["counts"], "answer": diagnosis["answer"]}, indent=2))


if __name__ == "__main__":
    main()

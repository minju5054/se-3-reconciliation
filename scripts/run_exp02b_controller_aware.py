#!/usr/bin/env python3
"""Create immutable offline EXP-02B branches from the frozen EXP-01B cohort."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
import subprocess
from typing import Any

import numpy as np
import yaml

from reconciliation.exp02b import (
    CASE_IDS,
    METHODS,
    build_candidate_methods,
    geometric_preservation_metrics,
    inter_k_retention,
    load_source_case,
    validate_exp02b_config,
    validate_exp02b_output,
    verify_frozen_source_selection,
    transition_input,
)
from reconciliation.online_switch import save_json_exclusive, save_npy_exclusive
from reconciliation.oracle_correspondence import sha256_file


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPOSITORY_ROOT / path).resolve()


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a mapping")
    return value


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPOSITORY_ROOT / "configs/exp02b_controller_aware.yaml",
    )
    parser.add_argument("--run-id")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = load_yaml(args.config)
    validate_exp02b_config(config)
    source_root = resolve_path(config["paths"]["source_root"])
    graph_config_path = resolve_path(config["paths"]["exp02a_graph_config"])
    graph_config = load_yaml(graph_config_path)
    if graph_config.get("experiment") != "EXP-02A":
        raise ValueError("graph config is not the immutable EXP-02A formulation")
    selected = verify_frozen_source_selection(source_root, config)
    run_id = args.run_id or datetime.now(timezone.utc).strftime("exp02b-%Y%m%dT%H%M%SZ")
    output = resolve_path(config["paths"]["output_root"]) / run_id
    output.mkdir(parents=True, exist_ok=False)

    source_payload: dict[str, Any] = {
        "experiment": "EXP-02B",
        "source_root": str(source_root),
        "selection_frozen_before_branch_outputs": True,
        "selection_normalization": selected["_normalization"],
        "cases": {},
    }
    summary_cases: dict[str, Any] = {}
    for case_id in CASE_IDS:
        trial = Path(selected[case_id]["artifact_path"]).resolve()
        source = load_source_case(case_id, trial)
        hashes = {
            relative: sha256_file(trial / relative)
            for relative in config["frozen_source_cases"][case_id]["sha256"]
        }
        source_payload["cases"][case_id] = {
            "trial_directory": str(trial),
            "relative_trial_path": str(trial.relative_to(source_root)),
            "sha256": hashes,
            "condition_id": source.source_attempt["condition_id"],
            "cell_id": source.source_attempt["cell_id"],
            "scene_id": source.source_attempt["scene_id"],
            "classification": source.source_attempt["classification"],
            "boundary_pose_world": source.boundary_pose_world.tolist(),
            "previous_actual_pose_world": source.previous_pose_world.tolist(),
            "source_controller_metrics": source.source_attempt["controller_metrics"],
            "source_geometry": source.source_attempt["geometry"],
            "selection_score": selected[case_id].get("selection_score"),
        }
        case_root = output / case_id
        case_summary: dict[str, Any] = {"k": {}, "inter_k": {}}
        states_by_method: dict[str, dict[int, np.ndarray]] = {
            method: {} for method in METHODS
        }
        for k in config["entry_indices"]:
            inputs = transition_input(source, int(k))
            candidates, optimization = build_candidate_methods(
                inputs, graph_config=graph_config
            )
            original_fresh = inputs.fresh_poses_world.copy()
            original_old = inputs.old_poses_world.copy()
            original_boundary = inputs.committed_pose_world.copy()
            k_summary: dict[str, Any] = {}
            for method in METHODS:
                branch = case_root / f"k_{k}" / method
                branch.mkdir(parents=True, exist_ok=False)
                candidate = candidates[method]
                raw_reference = inputs.fresh_poses_world if method == "raw_f0" else inputs.selected_suffix
                metrics = geometric_preservation_metrics(
                    raw_reference=raw_reference,
                    candidate=candidate,
                    previous_pose_world=inputs.incoming_previous_pose,
                    boundary_pose_world=inputs.committed_pose_world,
                    minimum_translation_m=float(graph_config["minimum_direction_translation_m"]),
                )
                save_npy_exclusive(branch / "candidate.npy", candidate)
                save_json_exclusive(branch / "geometric_metrics.json", metrics)
                if method in optimization:
                    save_json_exclusive(branch / "construction.json", optimization[method])
                save_json_exclusive(
                    branch / "method.json",
                    {
                        "method": method,
                        "entry_index": int(k),
                        "baseline_role": (
                            "upstream naive FRESH[0:] reference"
                            if method == "raw_f0"
                            else "same-k backend baseline"
                            if method == "raw_k"
                            else "diagnostic pose-anchor"
                            if method == "pose_anchor"
                            else "analytic rigid baseline"
                            if method == "rigid"
                            else "unchanged EXP-02A incoming-motion-aware graph"
                        ),
                        "raw_reference": "FRESH[0:]" if method == "raw_f0" else f"FRESH[{k}:]",
                        "selector_implemented": False,
                        "controller_residual_in_graph": False,
                    },
                )
                states_by_method[method][int(k)] = candidate
                k_summary[method] = metrics
            if not np.array_equal(inputs.fresh_poses_world, original_fresh):
                raise RuntimeError("immutable full FRESH changed")
            if not np.array_equal(inputs.old_poses_world, original_old):
                raise RuntimeError("immutable OLD changed")
            if not np.array_equal(inputs.committed_pose_world, original_boundary):
                raise RuntimeError("committed boundary changed")
            case_summary["k"][str(k)] = k_summary

        for method in METHODS:
            case_summary["inter_k"][method] = inter_k_retention(
                fresh_world=source.fresh_world,
                entry_indices=config["entry_indices"],
                states_by_k=states_by_method[method],
            )
            save_json_exclusive(
                case_root / f"inter_k_{method}.json", case_summary["inter_k"][method]
            )
        summary_cases[case_id] = case_summary

    protocol = {
        "experiment": "EXP-02B",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "git_sha_at_run": subprocess.check_output(
            ["git", "-C", str(REPOSITORY_ROOT), "rev-parse", "HEAD"], text=True
        ).strip(),
        "entry_indices": list(config["entry_indices"]),
        "methods": list(METHODS),
        "graph_config": str(graph_config_path),
        "graph_objective_unchanged_from_exp02a": True,
        "controller_residual_in_graph": False,
        "selector_implemented": False,
        "z_evidence_used": False,
        "branch_semantics": "every k starts at the same saved committed pose B",
    }
    save_json_exclusive(output / "source_selection.json", source_payload)
    save_json_exclusive(output / "protocol.json", protocol)
    save_json_exclusive(
        output / "offline_summary.json",
        {
            "experiment": "EXP-02B",
            "run_id": run_id,
            "cases": summary_cases,
            "claim_boundary": (
                "Three deterministically selected development/stress cases and manual k only; "
                "offline geometry is not controller-level evidence."
            ),
        },
    )
    validation = validate_exp02b_output(output, config)
    save_json_exclusive(output / "offline_validation.json", validation)
    print("EXP02B_OUTPUT_DIR=" + str(output))


if __name__ == "__main__":
    main()

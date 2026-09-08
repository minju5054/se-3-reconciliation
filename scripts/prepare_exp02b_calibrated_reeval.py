#!/usr/bin/env python3
"""Prepare immutable EXP-02B-R provenance without regenerating candidates."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
from pathlib import Path
import subprocess
from typing import Any

import numpy as np
import yaml

from reconciliation.closed_loop_execution_validation import load_frozen_stage0d_candidate
from reconciliation.exp02b_calibrated_reeval import (
    historical_nominal_post_metrics,
    offline_first_command_invariant,
    validate_reeval_config,
    verify_file_hash,
    verify_historical_run,
    write_json_exclusive,
)
from reconciliation.online_switch import load_strict_json, sha256_file


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPOSITORY_ROOT / "configs/exp02b_calibrated_reeval.yaml",
    )
    parser.add_argument("--run-id")
    return parser.parse_args()


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        result = yaml.safe_load(stream)
    if not isinstance(result, dict):
        raise ValueError(f"{path} must contain a mapping")
    return result


def resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPOSITORY_ROOT / path).resolve()


def command_array(path: Path) -> np.ndarray:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    return np.asarray(
        [[float(row["v_command_mps"]), float(row["omega_command_rps"])] for row in rows],
        dtype=np.float64,
    )


def main() -> None:
    args = parse_args()
    config_path = args.config.resolve()
    config = load_yaml(config_path)
    validation = validate_reeval_config(config)
    paths = config["paths"]
    historical_run = resolve_path(paths["historical_exp02b_run"])
    historical_config_path = resolve_path(paths["historical_exp02b_config"])
    exp01b_config_path = resolve_path(paths["source_exp01b_config"])
    historical_config = load_yaml(historical_config_path)
    exp01b_config = load_yaml(exp01b_config_path)
    provenance = config["historical_provenance"]
    verify_file_hash(
        historical_config_path, provenance["exp02b_config_sha256"], "EXP-02B config"
    )
    verify_file_hash(
        exp01b_config_path, provenance["exp01b_config_sha256"], "EXP-01B config"
    )
    historical = verify_historical_run(historical_run, config, historical_config)

    stage0d_run = resolve_path(paths["stage0d_run"])
    stage0d_provenance = config["stage0d_candidate_provenance"]
    verify_file_hash(
        stage0d_run / "final_execution_layer_validation.json",
        stage0d_provenance["final_validation_sha256"],
        "Stage 0-D final validation",
    )
    candidate = load_frozen_stage0d_candidate(
        stage0d_run, stage0d_provenance, repository_root=REPOSITORY_ROOT
    )

    stage0f_run = resolve_path(paths["stage0f_run"])
    stage0f_provenance = config["stage0f_scope_provenance"]
    stage0f_hashes = {}
    for name, key in (
        ("config_snapshot.yaml", "config_snapshot_sha256"),
        ("execution_metadata.json", "execution_metadata_sha256"),
        ("summary/final_lightnav_execution_envelope_decision.json", "final_decision_sha256"),
    ):
        stage0f_hashes[name] = verify_file_hash(
            stage0f_run / name, stage0f_provenance[key], f"Stage 0-F {name}"
        )
    stage0f_decision = load_strict_json(
        stage0f_run / "summary/final_lightnav_execution_envelope_decision.json"
    )
    if stage0f_decision["status"] != stage0f_provenance["required_status"]:
        raise ValueError("Stage 0-F bounded workload decision changed")

    tolerance = float(config["protocol"]["first_desired_invariant_tolerance"])
    snapshot_branches = []
    manifest_branches = []
    for branch in historical["branches"]:
        invariant = offline_first_command_invariant(
            branch, exp01b_config["closed_loop"], tolerance=tolerance
        )
        if not invariant["passed"]:
            raise RuntimeError(
                "PROTOCOL_INVARIANT_FAILURE before execution: "
                f"{branch['case_id']}/k{branch['entry_index']}/{branch['method']}"
            )
        nominal_actual = np.load(branch["historical_actual_path"], allow_pickle=False)
        nominal_desired = command_array(Path(branch["historical_command_trace_path"]))
        nominal_metrics = historical_nominal_post_metrics(
            candidate=np.load(branch["candidate_path"], allow_pickle=False),
            actual=nominal_actual,
            desired=nominal_desired,
            control_dt_s=float(config["simulation"]["control_dt_s"]),
        )
        source_record = historical["source_selection"]["cases"][branch["case_id"]]
        snapshot_branches.append(
            {
                **branch,
                "saved_boundary_se2": source_record["boundary_pose_world"],
                "source_trial_directory": source_record["trial_directory"],
                "source_trial_sha256": source_record["sha256"],
                "offline_first_desired_invariant": invariant,
                "historical_nominal_post_metrics": nominal_metrics,
            }
        )
        manifest_branches.append(
            {
                "case_id": branch["case_id"],
                "entry_index": branch["entry_index"],
                "method": branch["method"],
                "candidate_path": branch["candidate_path"],
                "candidate_sha256": branch["candidate_sha256"],
                "candidate_shape": branch["candidate_shape"],
                "geometric_metrics_sha256": branch["geometric_metrics_sha256"],
            }
        )

    run_id = args.run_id or datetime.now(timezone.utc).strftime(
        "exp02b-r-%Y%m%dT%H%M%SZ"
    )
    output = resolve_path(paths["output_root"]) / run_id
    output.mkdir(parents=True, exist_ok=False)
    with (output / "config_snapshot.yaml").open("x", encoding="utf-8") as stream:
        stream.write(config_path.read_text(encoding="utf-8"))
    snapshot = {
        "experiment": "EXP-02B-R",
        "historical_run": str(historical_run),
        "historical_run_id": historical["run_id"],
        "branch_count": len(snapshot_branches),
        "methods": list(config["methods"]),
        "entry_indices": list(config["entry_indices"]),
        "branches": snapshot_branches,
    }
    write_json_exclusive(output / "historical_metric_snapshot.json", snapshot)
    write_json_exclusive(
        output / "source_snapshot/candidate_manifest.json",
        {
            "candidate_artifacts_regenerated": False,
            "candidate_artifacts_copied": False,
            "historical_candidates_loaded_in_place": True,
            "branch_count": len(manifest_branches),
            "branches": manifest_branches,
        },
    )
    source_provenance = {
        "historical_exp02b": {
            "run_directory": str(historical_run),
            "top_level_sha256": historical["top_level_sha256"],
            "strict_validation": historical["strict_validation"],
            "historical_config_path": str(historical_config_path),
            "historical_config_sha256": sha256_file(historical_config_path),
            "source_config_path": str(exp01b_config_path),
            "source_config_sha256": sha256_file(exp01b_config_path),
            "source_selection_sha256": sha256_file(historical_run / "source_selection.json"),
        },
        "stage0d_candidate": candidate,
        "stage0d_final_validation_sha256": sha256_file(
            stage0d_run / "final_execution_layer_validation.json"
        ),
        "stage0f_bounded_scope": {
            "run_directory": str(stage0f_run),
            "sha256": stage0f_hashes,
            "decision": stage0f_decision,
        },
        "code_sha256": {
            "trajectory_follower": sha256_file(
                REPOSITORY_ROOT / "src/reconciliation/controllers/trajectory_follower.py"
            ),
            "calibrated_execution_controller": sha256_file(
                REPOSITORY_ROOT / "src/reconciliation/controllers/jackal_execution_controller.py"
            ),
            "exp02b_reeval_helpers": sha256_file(
                REPOSITORY_ROOT / "src/reconciliation/exp02b_calibrated_reeval.py"
            ),
        },
    }
    write_json_exclusive(output / "source_provenance.json", source_provenance)
    git_sha = subprocess.check_output(
        ["git", "-C", str(REPOSITORY_ROOT), "rev-parse", "HEAD"], text=True
    ).strip()
    dirty = subprocess.run(
        ["git", "-C", str(REPOSITORY_ROOT), "diff", "--quiet"], check=False
    ).returncode != 0
    metadata = {
        "experiment": "EXP-02B-R",
        "run_id": run_id,
        "prepared_utc": datetime.now(timezone.utc).isoformat(),
        "git_sha_at_preparation": git_sha,
        "git_worktree_dirty": dirty,
        "config_snapshot_sha256": sha256_file(output / "config_snapshot.yaml"),
        "source_provenance_sha256": sha256_file(output / "source_provenance.json"),
        "historical_metric_snapshot_sha256": sha256_file(
            output / "historical_metric_snapshot.json"
        ),
        "candidate_manifest_sha256": sha256_file(
            output / "source_snapshot/candidate_manifest.json"
        ),
        "configuration_validation": validation,
        "branch_count": len(snapshot_branches),
        "all_offline_first_desired_invariants_passed": True,
        "candidate_regeneration_performed": False,
        "controller_or_reconciliation_tuning_performed": False,
        "historical_exp02b_modified": False,
    }
    write_json_exclusive(output / "metadata.json", metadata)
    print(f"EXP02B_R_OUTPUT_DIR={output}")
    print(f"EXP02B_R_OFFLINE_INVARIANTS={len(snapshot_branches)}/27 PASS")


if __name__ == "__main__":
    main()

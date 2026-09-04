"""Pure experiment construction and artifact validation for EXP-02."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from reconciliation.graph_metrics import assert_finite_json_tree
from reconciliation.online_switch import load_strict_json
from reconciliation.oracle_correspondence import sha256_file
from reconciliation.se2 import compose_poses
from reconciliation.se2_graph import OracleCorrespondence
from reconciliation.trajectory import validate_se2_trajectory


def synthetic_inputs(config: Mapping[str, Any]) -> dict[str, Any]:
    count = int(config["ground_truth_pose_count"])
    old_count = int(config["old_pose_count"])
    new_start = int(config["new_start_index"])
    if count < 4 or not 1 <= old_count <= count or not 0 <= new_start < count - 1:
        raise ValueError("synthetic trajectory slicing configuration is invalid")
    step = np.asarray(config["local_step"], dtype=np.float64)
    perturbation = np.asarray(config["global_left_perturbation"], dtype=np.float64)
    if step.shape != (3,) or perturbation.shape != (3,) or not np.all(
        np.isfinite(np.concatenate((step, perturbation)))
    ):
        raise ValueError("synthetic step and perturbation must be finite SE(2) vectors")
    poses = [np.zeros(3)]
    for _ in range(count - 1):
        poses.append(compose_poses(poses[-1], step))
    ground_truth = np.asarray(poses)
    old = ground_truth[:old_count].copy()
    expected_new = ground_truth[new_start:].copy()
    raw_new = compose_poses(perturbation, expected_new)
    correspondences = tuple(
        OracleCorrespondence(
            int(pair[0]), int(pair[1]), [0.0, 0.0, 0.0], "same synthetic physical pose"
        )
        for pair in config["identity_correspondences"]
    )
    return {
        "ground_truth": ground_truth,
        "fixed_old": old,
        "expected_new": expected_new,
        "raw_new": raw_new,
        "fixed_boundary": expected_new[0].copy(),
        "perturbation": perturbation,
        "correspondences": correspondences,
    }


def save_csv_exclusive(path: str | Path, rows: list[Mapping[str, Any]]) -> None:
    destination = Path(path)
    if not rows:
        raise ValueError("CSV output requires at least one row")
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def validate_exp02_output(path: str | Path) -> dict[str, Any]:
    root = Path(path)
    required = (
        "summary.json",
        "synthetic/raw_new.npy",
        "synthetic/ground_truth.npy",
        "synthetic/optimized_new.npy",
        "synthetic/oracle.json",
        "synthetic/metrics.json",
        "synthetic/optimization.json",
        "synthetic/trajectory.png",
        "synthetic/correction_profile.csv",
        "real_lightnav/source.json",
        "real_lightnav/raw_new_world.npy",
        "real_lightnav/optimized_new_world.npy",
        "real_lightnav/oracle.json",
        "real_lightnav/metrics_before.json",
        "real_lightnav/metrics_after.json",
        "real_lightnav/factor_residuals.json",
        "real_lightnav/optimization.json",
        "real_lightnav/correction_profile.csv",
        "real_lightnav/trajectory_before_after.png",
        "real_lightnav/correction_profile.png",
    )
    missing = [item for item in required if not (root / item).is_file()]
    if missing:
        raise ValueError(f"EXP-02 output is missing: {', '.join(missing)}")
    summary = load_strict_json(root / "summary.json")
    assert_finite_json_tree(summary)
    if summary.get("experiment") != "EXP-02" or summary["synthetic"]["accepted"] is not True:
        raise ValueError("EXP-02 summary/synthetic acceptance is invalid")
    synthetic_raw = validate_se2_trajectory(np.load(root / "synthetic/raw_new.npy", allow_pickle=False))
    synthetic_optimized = validate_se2_trajectory(
        np.load(root / "synthetic/optimized_new.npy", allow_pickle=False)
    )
    real_raw = validate_se2_trajectory(
        np.load(root / "real_lightnav/raw_new_world.npy", allow_pickle=False)
    )
    real_optimized = validate_se2_trajectory(
        np.load(root / "real_lightnav/optimized_new_world.npy", allow_pickle=False)
    )
    if synthetic_raw.shape != synthetic_optimized.shape or real_raw.shape != real_optimized.shape:
        raise ValueError("raw/optimized trajectory shapes differ")
    for name in ("full", "no_correspondence", "no_new_motion"):
        variant = root / "ablations" / name
        optimized = validate_se2_trajectory(
            np.load(variant / "optimized_new_world.npy", allow_pickle=False)
        )
        if optimized.shape != real_raw.shape:
            raise ValueError(f"ablation {name} shape differs from real raw NEW")
        for filename in ("metrics.json", "optimization.json"):
            assert_finite_json_tree(load_strict_json(variant / filename))
    source = load_strict_json(root / "real_lightnav/source.json")
    source_trial = Path(source["source_trial_directory"])
    hashes = source["raw_hashes"]
    if hashes["old_actions.npy"] != sha256_file(source_trial / "raw/old_actions.npy"):
        raise ValueError("source OLD hash changed")
    if hashes["new_actions.npy"] != sha256_file(source_trial / "raw/new_actions.npy"):
        raise ValueError("source NEW hash changed")
    sensitivity_dirs = sorted((root / "weight_sensitivity").glob("ratio_*"))
    if len(sensitivity_dirs) != 3:
        raise ValueError("weight sensitivity must contain exactly three predefined settings")
    return {
        "valid": True,
        "synthetic_shape": list(synthetic_raw.shape),
        "real_shape": list(real_raw.shape),
        "ablation_count": 3,
        "weight_sensitivity_count": 3,
        "source_hashes_match": True,
    }

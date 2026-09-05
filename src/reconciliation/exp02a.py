"""Pure construction and artifact validation helpers for EXP-02A."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import numpy as np

from reconciliation.graph_metrics import assert_finite_json_tree
from reconciliation.online_switch import load_strict_json
from reconciliation.oracle_correspondence import sha256_file
from reconciliation.se2 import compose_poses
from reconciliation.spatial_entry import SpatialEntryContext, TransitionReconciliationInput
from reconciliation.trajectory import validate_pose_se2, validate_se2_trajectory
from reconciliation.transition_graph import SUPPORTED_VARIANTS


CONTEXT_LABELS = ("early", "middle", "late")


def _repeated_motion(start: Any, step: Any, count: int, *, name: str) -> np.ndarray:
    pose = validate_pose_se2(start, name=f"{name}_start")
    increment = validate_pose_se2(step, name=f"{name}_step")
    if not isinstance(count, int) or count < 2:
        raise ValueError(f"{name}_pose_count must be an integer >= 2")
    poses = [pose]
    for _ in range(count - 1):
        poses.append(compose_poses(poses[-1], increment))
    return np.asarray(poses, dtype=np.float64)


def contexts_from_config(values: Mapping[str, Any]) -> dict[str, SpatialEntryContext]:
    if not isinstance(values, Mapping) or tuple(values) != CONTEXT_LABELS:
        raise ValueError("entry contexts must be ordered early, middle, late")
    contexts = {
        label: SpatialEntryContext.from_mapping(document) for label, document in values.items()
    }
    indices = [context.entry_index for context in contexts.values()]
    if not indices[0] < indices[1] < indices[2]:
        raise ValueError("early/middle/late entry indices must be strictly increasing")
    return contexts


def synthetic_inputs(config: Mapping[str, Any]) -> dict[str, TransitionReconciliationInput]:
    old = _repeated_motion(
        config["old_start_pose"],
        config["old_local_step"],
        int(config["old_pose_count"]),
        name="old",
    )
    fresh = _repeated_motion(
        config["fresh_initial_pose"],
        config["fresh_local_step"],
        int(config["fresh_pose_count"]),
        name="fresh",
    )
    boundary = validate_pose_se2(config["committed_pose"], name="committed_pose")
    contexts = contexts_from_config(config["contexts"])
    return {
        label: TransitionReconciliationInput(
            old_poses_world=old,
            fresh_poses_world=fresh,
            committed_pose_world=boundary,
            entry_context=context,
            actual_pose_before_committed=old[-1],
            metadata={
                "fixture": "deterministic synthetic mechanism test",
                "research_evidence": False,
            },
        )
        for label, context in contexts.items()
    }


def validate_exp02a_output(path: str | Path) -> dict[str, Any]:
    root = Path(path)
    summary = load_strict_json(root / "summary.json")
    assert_finite_json_tree(summary)
    if summary.get("experiment") != "EXP-02A":
        raise ValueError("summary is not EXP-02A")
    if summary.get("evidence_used_as_graph_measurement") is not False:
        raise ValueError("EXP-02A must state that evidence is not a graph measurement")

    variants = tuple(summary.get("variants", ()))
    if variants != SUPPORTED_VARIANTS:
        raise ValueError("EXP-02A summary variants differ from the fixed formulation set")
    for phase in ("synthetic", "real_lightnav"):
        phase_root = root / phase
        old = validate_se2_trajectory(np.load(phase_root / "old_world.npy", allow_pickle=False))
        fresh = validate_se2_trajectory(
            np.load(phase_root / "full_fresh_world.npy", allow_pickle=False)
        )
        boundary = validate_pose_se2(
            np.load(phase_root / "committed_pose_world.npy", allow_pickle=False),
            name="committed_pose_world",
        )
        if old.shape[0] < 1 or fresh.shape[0] < 2 or boundary.shape != (3,):
            raise ValueError(f"invalid fixed inputs in {phase}")
        for label in CONTEXT_LABELS:
            context_root = phase_root / f"k_{label}"
            context_document = load_strict_json(context_root / "entry_context.json")
            context = SpatialEntryContext.from_mapping(context_document)
            if context.to_dict() != context_document:
                raise ValueError(f"entry context did not round-trip exactly in {phase}/{label}")
            k = context.entry_index
            raw = validate_se2_trajectory(
                np.load(context_root / "raw_suffix.npy", allow_pickle=False), name="raw_suffix"
            )
            if not np.array_equal(raw, fresh[k:]):
                raise ValueError(f"raw suffix is not exactly FRESH[k:] in {phase}/{label}")
            assert_finite_json_tree(load_strict_json(context_root / "metrics_raw.json"))
            for variant in variants:
                optimized = validate_se2_trajectory(
                    np.load(context_root / f"optimized_{variant}.npy", allow_pickle=False)
                )
                if optimized.shape != raw.shape:
                    raise ValueError(f"optimized shape mismatch for {phase}/{label}/{variant}")
                for filename in (
                    f"metrics_{variant}.json",
                    f"optimization_{variant}.json",
                ):
                    assert_finite_json_tree(load_strict_json(context_root / filename))
                for filename in (
                    f"trajectory_{variant}.png",
                    f"correction_profile_{variant}.png",
                    f"correction_profile_{variant}.csv",
                ):
                    if not (context_root / filename).is_file():
                        raise ValueError(f"missing artifact {phase}/{label}/{filename}")
        for variant in variants:
            assert_finite_json_tree(
                load_strict_json(phase_root / f"inter_k_separation_{variant}.json")
            )
            if not (phase_root / f"inter_k_comparison_{variant}.png").is_file():
                raise ValueError(f"missing inter-k plot for {phase}/{variant}")

    source = load_strict_json(root / "real_lightnav/source.json")
    trial = Path(source["source_trial_directory"])
    if source["raw_hashes"]["old_actions.npy"] != sha256_file(trial / "raw/old_actions.npy"):
        raise ValueError("immutable EXP-01B OLD hash changed")
    if source["raw_hashes"]["new_actions.npy"] != sha256_file(trial / "raw/new_actions.npy"):
        raise ValueError("immutable EXP-01B FRESH hash changed")
    return {
        "valid": True,
        "phases": 2,
        "contexts_per_phase": len(CONTEXT_LABELS),
        "variants_per_context": len(SUPPORTED_VARIANTS),
        "source_hashes_match": True,
        "evidence_used_as_graph_measurement": False,
    }

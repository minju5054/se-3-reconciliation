"""Metrics that keep spatial-entry meaning separate from transition continuity."""

from __future__ import annotations

import math
from typing import Any, Mapping

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reconciliation.graph_metrics import correction_profile, residual_summary
from reconciliation.se2 import relative_pose, se2_log, wrap_angle
from reconciliation.se2_graph import new_motion_residual
from reconciliation.spatial_entry import TransitionReconciliationInput
from reconciliation.trajectory import validate_se2_trajectory
from reconciliation.transition_graph import world_translation_direction


FloatArray = NDArray[np.float64]


def _motion(start: ArrayLike, end: ArrayLike) -> tuple[float, float]:
    relative = relative_pose(start, end)
    return float(np.linalg.norm(relative[:2])), float(relative[2])


def _optional_direction(start: ArrayLike, end: ArrayLike, minimum: float, name: str) -> float | None:
    try:
        return world_translation_direction(
            start, end, minimum_translation_m=minimum, name=name
        )
    except ValueError as error:
        if "direction is undefined" not in str(error):
            raise
        return None


def evaluate_transition_state(
    inputs: TransitionReconciliationInput,
    state: ArrayLike,
    *,
    translation_scale_m: float,
    yaw_scale_rad: float,
    minimum_translation_m: float,
) -> dict[str, Any]:
    """Evaluate raw or optimized selected suffix without changing any input."""

    trajectory = validate_se2_trajectory(state, name="state")
    raw = inputs.selected_suffix
    if trajectory.shape != raw.shape:
        raise ValueError("state shape must equal selected FRESH suffix")
    previous = inputs.incoming_previous_pose
    boundary = inputs.committed_pose_world
    entry = trajectory[0]
    second = trajectory[1]

    incoming_translation, incoming_yaw = _motion(previous, boundary)
    transition_translation, transition_yaw = _motion(boundary, entry)
    first_fresh_translation, first_fresh_yaw = _motion(entry, second)
    incoming_direction = world_translation_direction(
        previous,
        boundary,
        minimum_translation_m=minimum_translation_m,
        name="incoming OLD motion",
    )
    transition_direction = _optional_direction(
        boundary,
        entry,
        minimum_translation_m,
        "boundary-to-entry transition",
    )
    fresh_direction = world_translation_direction(
        entry,
        second,
        minimum_translation_m=minimum_translation_m,
        name="first selected FRESH tangent",
    )
    distortions = np.asarray(
        [
            new_motion_residual(raw[index], raw[index + 1], trajectory[index], trajectory[index + 1])
            for index in range(raw.shape[0] - 1)
        ],
        dtype=np.float64,
    )
    entry_delta = se2_log(relative_pose(raw[0], entry))
    profile = correction_profile(raw, trajectory)
    k = inputs.entry_context.entry_index
    for suffix_index, row in enumerate(profile):
        row["suffix_index"] = suffix_index
        row["fresh_index"] = k + suffix_index
        row.pop("new_index")
    correction_translation = [float(row["translation_correction_m"]) for row in profile]
    correction_yaw = [float(row["yaw_correction_rad"]) for row in profile]

    return {
        "entry_index": k,
        "suffix_shape": list(trajectory.shape),
        "transition_pose": {
            "translation_from_boundary_m": transition_translation,
            "abs_yaw_from_boundary_rad": abs(transition_yaw),
            "signed_yaw_from_boundary_rad": transition_yaw,
        },
        "transition_motion": {
            "incoming_translation_m": incoming_translation,
            "incoming_direction_world_rad": incoming_direction,
            "incoming_yaw_increment_rad": incoming_yaw,
            "boundary_to_entry_translation_m": transition_translation,
            "boundary_to_entry_direction_world_rad": transition_direction,
            "boundary_to_entry_direction_defined": transition_direction is not None,
            "boundary_to_entry_yaw_increment_rad": transition_yaw,
            "first_fresh_translation_m": first_fresh_translation,
            "first_fresh_direction_world_rad": fresh_direction,
            "first_fresh_yaw_increment_rad": first_fresh_yaw,
            "incoming_to_transition_direction_jump_rad": (
                None
                if transition_direction is None
                else abs(float(wrap_angle(transition_direction - incoming_direction)))
            ),
            "transition_to_fresh_tangent_jump_rad": (
                None
                if transition_direction is None
                else abs(float(wrap_angle(fresh_direction - transition_direction)))
            ),
            "translational_motion_jump_m": abs(transition_translation - incoming_translation),
            "yaw_motion_jump_rad": abs(float(wrap_angle(transition_yaw - incoming_yaw))),
        },
        "entry_preservation": {
            "local_displacement": entry_delta.tolist(),
            "translation_displacement_m": float(np.linalg.norm(entry_delta[:2])),
            "abs_yaw_displacement_rad": abs(float(entry_delta[2])),
            "yaw_displacement_rad": float(entry_delta[2]),
        },
        "fresh_relative_motion_distortion": residual_summary(
            distortions,
            translation_scale=translation_scale_m,
            yaw_scale=yaw_scale_rad,
        ),
        "downstream_correction": {
            "per_pose": profile,
            "translation_range_m": (
                max(correction_translation) - min(correction_translation)
            ),
            "yaw_range_rad": max(correction_yaw) - min(correction_yaw),
            "max_adjacent_translation_change_m": max(
                (abs(right - left) for left, right in zip(correction_translation, correction_translation[1:])),
                default=0.0,
            ),
            "max_adjacent_yaw_change_rad": max(
                (
                    abs(float(wrap_angle(right - left)))
                    for left, right in zip(correction_yaw, correction_yaw[1:])
                ),
                default=0.0,
            ),
        },
        "fixed_inputs": {
            "planned_old_tail_to_boundary": inputs.planned_old_to_boundary.tolist(),
            "incoming_source": (
                "actual_pose_before_committed"
                if inputs.actual_pose_before_committed is not None
                else "planned_old_tail"
            ),
        },
    }


def inter_k_separation(
    inputs_by_label: Mapping[str, TransitionReconciliationInput],
    states_by_label: Mapping[str, ArrayLike],
    *,
    first_pose_count: int = 3,
    minimum_translation_m: float = 1e-9,
) -> dict[str, Any]:
    """Measure whether distinct k-conditioned branches remain spatially distinct."""

    if set(inputs_by_label) != set(states_by_label) or len(inputs_by_label) < 2:
        raise ValueError("matching input/state mappings with at least two k choices are required")
    count = int(first_pose_count)
    if count < 1:
        raise ValueError("first_pose_count must be positive")
    labels = list(inputs_by_label)
    pairs: list[dict[str, Any]] = []
    for left_index, left_label in enumerate(labels):
        left_input = inputs_by_label[left_label]
        left_state = validate_se2_trajectory(states_by_label[left_label], name=left_label)
        for right_label in labels[left_index + 1 :]:
            right_input = inputs_by_label[right_label]
            right_state = validate_se2_trajectory(states_by_label[right_label], name=right_label)
            raw_left = left_input.selected_suffix
            raw_right = right_input.selected_suffix
            if left_state.shape != raw_left.shape or right_state.shape != raw_right.shape:
                raise ValueError("each k state must match its selected suffix")
            raw_entry_distance = float(np.linalg.norm(raw_left[0, :2] - raw_right[0, :2]))
            optimized_entry_distance = float(np.linalg.norm(left_state[0, :2] - right_state[0, :2]))
            if raw_entry_distance <= minimum_translation_m:
                raise ValueError("distinct k choices have indistinguishable raw entry positions")
            raw_left_direction = world_translation_direction(
                left_input.committed_pose_world,
                raw_left[0],
                minimum_translation_m=minimum_translation_m,
                name=f"raw {left_label} transition",
            )
            raw_right_direction = world_translation_direction(
                right_input.committed_pose_world,
                raw_right[0],
                minimum_translation_m=minimum_translation_m,
                name=f"raw {right_label} transition",
            )
            optimized_left_direction = _optional_direction(
                left_input.committed_pose_world,
                left_state[0],
                minimum_translation_m,
                f"optimized {left_label} transition",
            )
            optimized_right_direction = _optional_direction(
                right_input.committed_pose_world,
                right_state[0],
                minimum_translation_m,
                f"optimized {right_label} transition",
            )
            sample_count = min(count, raw_left.shape[0], raw_right.shape[0])
            raw_separation = np.linalg.norm(
                raw_left[:sample_count, :2] - raw_right[:sample_count, :2], axis=1
            )
            optimized_separation = np.linalg.norm(
                left_state[:sample_count, :2] - right_state[:sample_count, :2], axis=1
            )
            pairs.append(
                {
                    "left": left_label,
                    "right": right_label,
                    "left_k": left_input.entry_context.entry_index,
                    "right_k": right_input.entry_context.entry_index,
                    "raw_entry_separation_m": raw_entry_distance,
                    "optimized_entry_separation_m": optimized_entry_distance,
                    "entry_separation_retention_ratio": optimized_entry_distance / raw_entry_distance,
                    "raw_transition_heading_difference_rad": abs(
                        float(wrap_angle(raw_left_direction - raw_right_direction))
                    ),
                    "optimized_transition_heading_difference_rad": (
                        None
                        if optimized_left_direction is None or optimized_right_direction is None
                        else abs(
                            float(wrap_angle(optimized_left_direction - optimized_right_direction))
                        )
                    ),
                    "first_pose_count": sample_count,
                    "raw_first_poses_mean_separation_m": float(np.mean(raw_separation)),
                    "optimized_first_poses_mean_separation_m": float(np.mean(optimized_separation)),
                    "first_poses_separation_retention_ratio": float(
                        np.mean(optimized_separation) / np.mean(raw_separation)
                    ),
                }
            )
    return {"pair_count": len(pairs), "pairs": pairs}

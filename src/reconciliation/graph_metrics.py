"""Physical residual and correction metrics for EXP-02 graph runs."""

from __future__ import annotations

import math
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reconciliation.metrics import compute_transition_metrics
from reconciliation.se2 import relative_pose, se2_log
from reconciliation.se2_graph import GraphProblem, physical_factor_residuals
from reconciliation.trajectory import validate_pose_se2, validate_se2_trajectory


FloatArray = NDArray[np.float64]


def residual_summary(residuals: ArrayLike, *, translation_scale: float, yaw_scale: float) -> dict[str, Any]:
    values = np.asarray(residuals, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3 or not np.all(np.isfinite(values)):
        raise ValueError("residuals must be a finite (K, 3) array")
    if translation_scale <= 0.0 or yaw_scale <= 0.0:
        raise ValueError("residual scales must be positive")
    if values.shape[0] == 0:
        return {
            "count": 0,
            "translation_rms_m": None,
            "yaw_rms_rad": None,
            "total_normalized_rms": None,
            "max_translation_m": None,
            "max_abs_yaw_rad": None,
            "per_factor": [],
        }
    translation = np.linalg.norm(values[:, :2], axis=1)
    yaw = np.abs(values[:, 2])
    normalized = values / np.array([translation_scale, translation_scale, yaw_scale])
    return {
        "count": int(values.shape[0]),
        "translation_rms_m": float(np.sqrt(np.mean(translation**2))),
        "yaw_rms_rad": float(np.sqrt(np.mean(yaw**2))),
        "total_normalized_rms": float(np.sqrt(np.mean(normalized**2))),
        "max_translation_m": float(np.max(translation)),
        "max_abs_yaw_rad": float(np.max(yaw)),
        "per_factor": [
            {
                "index": index,
                "residual": row.tolist(),
                "translation_m": float(translation[index]),
                "abs_yaw_rad": float(yaw[index]),
            }
            for index, row in enumerate(values)
        ],
    }


def correction_profile(raw_new: ArrayLike, optimized_new: ArrayLike) -> list[dict[str, float | int]]:
    raw = validate_se2_trajectory(raw_new, name="raw_new")
    optimized = validate_se2_trajectory(optimized_new, name="optimized_new")
    if raw.shape != optimized.shape:
        raise ValueError("raw and optimized trajectories must have equal shape")
    corrections = se2_log(relative_pose(raw, optimized))
    return [
        {
            "new_index": index,
            "local_dx_m": float(row[0]),
            "local_dy_m": float(row[1]),
            "translation_correction_m": float(np.linalg.norm(row[:2])),
            "yaw_correction_rad": float(row[2]),
        }
        for index, row in enumerate(corrections)
    ]


def trajectory_error(reference: ArrayLike, estimate: ArrayLike) -> dict[str, float]:
    reference_value = validate_se2_trajectory(reference, name="reference")
    estimate_value = validate_se2_trajectory(estimate, name="estimate")
    if reference_value.shape != estimate_value.shape:
        raise ValueError("reference and estimate must have equal shape")
    errors = se2_log(relative_pose(reference_value, estimate_value))
    translation = np.linalg.norm(errors[:, :2], axis=1)
    yaw = np.abs(errors[:, 2])
    return {
        "translation_rms_m": float(np.sqrt(np.mean(translation**2))),
        "translation_max_m": float(np.max(translation)),
        "yaw_rms_rad": float(np.sqrt(np.mean(yaw**2))),
        "yaw_max_rad": float(np.max(yaw)),
    }


def evaluate_graph_state(
    problem: GraphProblem,
    state: ArrayLike,
    *,
    actual_pose_before_boundary: ArrayLike | None = None,
) -> dict[str, Any]:
    trajectory = validate_se2_trajectory(state, name="state")
    factors = physical_factor_residuals(problem, trajectory)
    scales = problem.scales
    boundary = residual_summary(
        factors["boundary"],
        translation_scale=scales.translation_m,
        yaw_scale=scales.yaw_rad,
    )
    correspondence = residual_summary(
        factors["correspondence"],
        translation_scale=scales.translation_m,
        yaw_scale=scales.yaw_rad,
    )
    motion = residual_summary(
        factors["new_motion"],
        translation_scale=scales.translation_m,
        yaw_scale=scales.yaw_rad,
    )
    result: dict[str, Any] = {
        "boundary": boundary,
        "correspondence": correspondence,
        "new_motion_distortion": motion,
        "correction_profile": correction_profile(problem.raw_new, trajectory),
    }
    if actual_pose_before_boundary is not None:
        previous = validate_pose_se2(
            actual_pose_before_boundary, name="actual_pose_before_boundary"
        )
        result["switch_metrics"] = compute_transition_metrics(
            previous,
            problem.fixed_boundary,
            trajectory[0],
            trajectory[1],
        ).to_dict()
    return result


def assert_finite_json_tree(value: Any, path: str = "root") -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            assert_finite_json_tree(child, f"{path}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            assert_finite_json_tree(child, f"{path}[{index}]")
    elif isinstance(value, float) and not math.isfinite(value):
        raise ValueError(f"non-finite value at {path}")

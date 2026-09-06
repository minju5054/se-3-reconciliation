"""Controller-level and geometric metrics at an OLD-to-FRESH raw switch."""

from __future__ import annotations

import math
from typing import Any, Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike

from reconciliation.se2 import wrap_angle
from reconciliation.trajectory import validate_pose_se2, validate_se2_trajectory


def _positive(name: str, value: float) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and greater than zero")
    return result


def validate_command_array(commands: ArrayLike, *, name: str = "commands") -> np.ndarray:
    """Validate canonical ``[v_mps, omega_rps]`` controller commands."""

    value = np.asarray(commands, dtype=np.float64)
    if value.ndim != 2 or value.shape[0] < 1 or value.shape[1] != 2:
        raise ValueError(f"{name} must have non-empty shape (N, 2)")
    if not np.all(np.isfinite(value)):
        raise ValueError(f"{name} must contain only finite values")
    return value.copy()


def controller_switch_metrics(
    old_commands: ArrayLike,
    fresh_commands: ArrayLike,
    *,
    control_dt_s: float,
    window_size: int = 3,
) -> dict[str, Any]:
    """Measure command discontinuity without assuming a VLA waypoint time base.

    The immediate transition compares the final OLD command with the first FRESH
    command. Window changes are consecutive differences over ``[OLD[-1],
    FRESH[0:window_size]]``. Slew is explicitly a discrete controller-command
    quantity, not measured robot acceleration.
    """

    old = validate_command_array(old_commands, name="old_commands")
    fresh = validate_command_array(fresh_commands, name="fresh_commands")
    dt = _positive("control_dt_s", control_dt_s)
    if not isinstance(window_size, int) or window_size < 1:
        raise ValueError("window_size must be a positive integer")
    if old.shape[0] < window_size or fresh.shape[0] < window_size:
        raise ValueError(f"at least {window_size} OLD and FRESH commands are required")

    old_window = old[-window_size:]
    fresh_window = fresh[:window_size]
    immediate = fresh_window[0] - old_window[-1]
    sequence = np.vstack((old_window[-1], fresh_window))
    changes = np.diff(sequence, axis=0)
    absolute = np.abs(changes)
    return {
        "window_size": window_size,
        "control_dt_s": dt,
        "old_last_commands_v_omega": old_window.tolist(),
        "fresh_first_commands_v_omega": fresh_window.tolist(),
        "old_last_command_v_omega": old_window[-1].tolist(),
        "fresh_first_command_v_omega": fresh_window[0].tolist(),
        "delta_v_signed_mps": float(immediate[0]),
        "delta_omega_signed_rps": float(immediate[1]),
        "delta_v_abs_mps": float(abs(immediate[0])),
        "delta_omega_abs_rps": float(abs(immediate[1])),
        "post_switch_consecutive_delta_v_signed_mps": changes[:, 0].tolist(),
        "post_switch_consecutive_delta_omega_signed_rps": changes[:, 1].tolist(),
        "post_switch_max_abs_delta_v_mps": float(np.max(absolute[:, 0])),
        "post_switch_mean_abs_delta_v_mps": float(np.mean(absolute[:, 0])),
        "post_switch_max_abs_delta_omega_rps": float(np.max(absolute[:, 1])),
        "post_switch_mean_abs_delta_omega_rps": float(np.mean(absolute[:, 1])),
        "immediate_command_slew_v_mps2": float(immediate[0] / dt),
        "immediate_command_slew_omega_rps2": float(immediate[1] / dt),
        "slew_semantics": "discrete controller-command change divided by control_dt; not physical acceleration",
    }


def _first_meaningful_tangent(
    poses: np.ndarray, threshold_m: float
) -> tuple[float | None, float | None, int | None]:
    for index, delta in enumerate(np.diff(poses[:, :2], axis=0)):
        magnitude = float(np.linalg.norm(delta))
        if magnitude > threshold_m:
            return math.atan2(float(delta[1]), float(delta[0])), magnitude, index
    return None, None, None


def controlled_switch_geometry(
    *,
    actual_pose_before_switch: ArrayLike,
    actual_pose_at_switch: ArrayLike,
    robot_pose_at_fresh_observation: ArrayLike,
    fresh_world: ArrayLike,
    zero_motion_tolerance_m: float,
) -> dict[str, Any]:
    """Return secondary geometry descriptors with undefined tangents explicit."""

    before = validate_pose_se2(actual_pose_before_switch)
    boundary = validate_pose_se2(actual_pose_at_switch)
    observation = validate_pose_se2(robot_pose_at_fresh_observation)
    fresh = validate_se2_trajectory(fresh_world)
    tolerance = _positive("zero_motion_tolerance_m", zero_motion_tolerance_m)

    incoming_delta = boundary[:2] - before[:2]
    incoming_magnitude = float(np.linalg.norm(incoming_delta))
    incoming_heading = (
        math.atan2(float(incoming_delta[1]), float(incoming_delta[0]))
        if incoming_magnitude > tolerance
        else None
    )
    fresh_heading, fresh_step, fresh_edge_index = _first_meaningful_tangent(fresh, tolerance)
    entry_delta = fresh[0, :2] - boundary[:2]
    entry_distance = float(np.linalg.norm(entry_delta))
    entry_heading = (
        math.atan2(float(entry_delta[1]), float(entry_delta[0]))
        if entry_distance > tolerance
        else None
    )
    disagreement = (
        float(wrap_angle(fresh_heading - incoming_heading))
        if fresh_heading is not None and incoming_heading is not None
        else None
    )

    if incoming_heading is None:
        endpoint_lateral = None
        max_lateral = None
    else:
        normal = np.array([-math.sin(incoming_heading), math.cos(incoming_heading)])
        relative = fresh[:, :2] - boundary[:2]
        lateral = relative @ normal
        endpoint_lateral = float(lateral[-1])
        max_lateral = float(np.max(np.abs(lateral)))

    fresh_yaw_progression = np.asarray(wrap_angle(np.diff(fresh[:, 2])), dtype=float)
    fresh_total_yaw = float(wrap_angle(fresh[-1, 2] - fresh[0, 2]))
    return {
        "old_incoming_motion_m": incoming_magnitude,
        "old_incoming_heading_defined": incoming_heading is not None,
        "old_incoming_heading_rad": incoming_heading,
        "old_incoming_yaw_increment_rad": float(wrap_angle(boundary[2] - before[2])),
        "fresh_first_meaningful_edge_index": fresh_edge_index,
        "fresh_first_meaningful_motion_m": fresh_step,
        "fresh_first_meaningful_heading_defined": fresh_heading is not None,
        "fresh_first_meaningful_heading_rad": fresh_heading,
        "old_fresh_tangent_disagreement_rad": disagreement,
        "old_fresh_tangent_disagreement_abs_rad": (
            abs(disagreement) if disagreement is not None else None
        ),
        "boundary_to_fresh_entry_m": entry_distance,
        "boundary_to_fresh_entry_direction_defined": entry_heading is not None,
        "boundary_to_fresh_entry_direction_rad": entry_heading,
        "local_spatial_step_magnitude_mismatch_m": (
            abs(float(fresh_step) - incoming_magnitude) if fresh_step is not None else None
        ),
        "fresh_total_yaw_progression_rad": fresh_total_yaw,
        "fresh_sum_abs_yaw_progression_rad": float(np.sum(np.abs(fresh_yaw_progression))),
        "fresh_endpoint_lateral_from_incoming_tangent_m": endpoint_lateral,
        "fresh_max_abs_lateral_from_incoming_tangent_m": max_lateral,
        "observation_to_switch_translation_m": float(
            np.linalg.norm(boundary[:2] - observation[:2])
        ),
        "observation_to_switch_yaw_rad": float(wrap_angle(boundary[2] - observation[2])),
        "translation_pose_gap_m": entry_distance,
        "yaw_pose_gap_rad": float(abs(wrap_angle(fresh[0, 2] - boundary[2]))),
        "metric_semantics": {
            "local_spatial_step_magnitude_mismatch_m": (
                "actual control-interval displacement magnitude versus untimed FRESH spatial "
                "waypoint spacing; not velocity or acceleration"
            )
        },
    }


def command_rows_to_array(
    rows: Sequence[Mapping[str, Any]], *, source: str
) -> np.ndarray:
    selected = [
        [float(row["v_command_mps"]), float(row["omega_command_rps"])]
        for row in rows
        if row.get("reference_source") == source
    ]
    if not selected:
        raise ValueError(f"no controller commands found for {source}")
    return validate_command_array(selected, name=f"{source} commands")


def align_command_rows_to_raw_switch(
    rows: Sequence[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    """Return copied command rows with the first FRESH command at exactly ``t=0``."""

    if not rows:
        raise ValueError("controller command rows must be non-empty")
    try:
        switch_time = next(
            float(row["sim_time_s"])
            for row in rows
            if row.get("reference_source") == "FRESH"
        )
    except StopIteration as error:
        raise ValueError("controller command rows contain no FRESH command") from error
    if not math.isfinite(switch_time):
        raise ValueError("FRESH switch time must be finite")

    aligned: list[dict[str, Any]] = []
    for row in rows:
        copied = dict(row)
        sim_time = float(copied["sim_time_s"])
        if not math.isfinite(sim_time):
            raise ValueError("controller command sim_time_s must be finite")
        copied["time_from_raw_switch_s"] = sim_time - switch_time
        aligned.append(copied)
    return aligned

"""Controlled translation against one fixed observation-anchored FRESH path.

Tau is a simulation variable, not measured RTT or a waypoint schedule. No robot,
controller, OLD execution, alignment, correspondence or optimization is used.
"""

from __future__ import annotations

from collections.abc import Mapping
from numbers import Real

import numpy as np

from reconciliation.robotless_single_chunk import _agent_pose, validate_waypoints
from reconciliation.se2 import compose_poses, relative_pose, se2_log


def finite_scalar(value, name: str, *, nonnegative: bool = False) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite real number")
    result = float(value)
    if not np.isfinite(result) or (nonnegative and result < 0):
        raise ValueError(f"{name} must be finite" + (" and nonnegative" if nonnegative else ""))
    return result


def motion_parameters(tau_s, v_mps, omega_radps) -> tuple[np.ndarray, float]:
    if isinstance(tau_s, (list, tuple)) and any(isinstance(t, (bool, np.bool_)) for t in tau_s):
        raise ValueError("tau_s must use real times, not booleans")
    values = np.asarray(tau_s)
    if values.ndim != 1 or values.size == 0 or values.dtype.kind not in "iuf":
        raise ValueError("tau_s must be a nonempty real vector starting at zero")
    times = np.array(values, dtype=float, copy=True)
    if not np.all(np.isfinite(times)) or times[0] != 0 or np.any(np.diff(times) <= 0):
        raise ValueError("tau_s must start at zero and be finite, nonnegative and strictly increasing")
    velocity = finite_scalar(v_mps, "v_mps", nonnegative=True)
    if finite_scalar(omega_radps, "omega_radps") != 0.0:
        raise ValueError("only omega_radps == 0 is supported in this experiment")
    with np.errstate(over="ignore", invalid="ignore"):
        distance = velocity * times
    if not np.all(np.isfinite(distance)):
        raise ValueError("v_mps * tau_s must remain finite")
    return times, velocity


def validate_config(config: Mapping) -> None:
    if not isinstance(config, Mapping) or config.get("stage") != "robotless-controlled-staleness":
        raise ValueError("wrong controlled-staleness configuration stage")
    if config.get("schema_version") != 1 or not isinstance(config.get("source_run"), str):
        raise ValueError("configuration needs schema_version=1 and source_run")
    motion = config["motion"]
    if motion.get("model") != "constant_body_forward_translation_only":
        raise ValueError("unsupported controlled motion model")
    if motion.get("tau_semantics") != "controlled_simulation_variable_not_measured_LightNav_RTT":
        raise ValueError("tau must be explicitly a controlled simulation variable")
    times, _ = motion_parameters(motion["tau_s"], motion["v_mps"], motion["omega_radps"])
    visual = config["visualization"]
    colors = np.asarray(visual["boundary_colors_rgba"], dtype=float)
    if colors.shape != (len(times), 4) or not np.all(np.isfinite(colors)) or np.any((colors < 0) | (colors > 1)):
        raise ValueError("one finite RGBA boundary color is required per tau")
    if len(visual["boundary_color_names"]) != len(times):
        raise ValueError("boundary color names must match tau conditions")


def controlled_boundaries(R_obs, tau_s, v_mps, omega_radps=0.0) -> np.ndarray:
    """B(tau)=R_obs * [v*tau,0,0]; input pose/yaw and tau=0 stay exact."""
    pose = _agent_pose(R_obs)
    times, velocity = motion_parameters(tau_s, v_mps, omega_radps)
    displacements = np.column_stack((velocity * times, np.zeros((len(times), 2))))
    with np.errstate(over="ignore", invalid="ignore"):
        boundaries = compose_poses(pose, displacements)
    if not np.all(np.isfinite(boundaries)):
        raise ValueError("controlled world boundaries must remain finite")
    # Translation-only motion preserves the recorded yaw representation exactly.
    boundaries[:, 2] = pose[2]
    boundaries[0] = pose
    return boundaries


def point_to_polyline(point_xy, polyline_xy) -> dict:
    """Minimum distance to closed segments between rows, with clamped projection.

    A singleton is treated as a point; repeated points are zero-length segments.
    Ties use the first segment. No observation-to-first-row connector is added.
    This pure XY helper avoids importing the historical controller diagnostics.
    """
    point, path = np.asarray(point_xy), np.asarray(polyline_xy)
    if point.shape != (2,) or path.ndim != 2 or path.shape[1:] != (2,) or len(path) == 0:
        raise ValueError("expected point (2,) and nonempty polyline (N,2)")
    if point.dtype.kind not in "iuf" or path.dtype.kind not in "iuf":
        raise ValueError("point/polyline must be real numeric arrays")
    point, path = point.astype(float), path.astype(float)
    if not np.all(np.isfinite(point)) or not np.all(np.isfinite(path)):
        raise ValueError("point/polyline must be finite")
    if len(path) == 1:
        distance = float(np.linalg.norm(point - path[0]))
        if not np.isfinite(distance):
            raise ValueError("point/polyline distance overflow")
        return {"distance_m": distance,
                "nearest_xy_world_m": path[0].tolist(), "segment_index": None, "segment_fraction": 0.0}
    starts, ends = path[:-1], path[1:]
    delta = ends - starts
    squared_lengths = np.sum(delta * delta, axis=1)
    numerator = np.sum((point - starts) * delta, axis=1)
    fractions = np.divide(numerator, squared_lengths, out=np.zeros_like(numerator), where=squared_lengths > 0)
    fractions = np.clip(fractions, 0.0, 1.0)
    nearest = starts + fractions[:, None] * delta
    distances = np.linalg.norm(nearest - point, axis=1)
    if not np.all(np.isfinite(distances)):
        raise ValueError("point/polyline distance overflow")
    index = int(np.argmin(distances))
    return {"distance_m": float(distances[index]), "nearest_xy_world_m": nearest[index].tolist(),
            "segment_index": index, "segment_fraction": float(fractions[index])}


def characterize(R_obs, fresh_world, tau_s, v_mps, omega_radps=0.0) -> tuple[np.ndarray, list[dict]]:
    """Raw interpretable metrics with signed changes from the tau=0 baseline."""
    pose = _agent_pose(R_obs)
    fresh = validate_waypoints(fresh_world)
    fresh.setflags(write=False)  # The SAME frozen array is used in every condition.
    times, velocity = motion_parameters(tau_s, v_mps, omega_radps)
    boundaries = controlled_boundaries(pose, times, velocity, omega_radps)
    rows = []
    for index, (tau, boundary) in enumerate(zip(times, boundaries, strict=True)):
        delta = relative_pose(pose, boundary)
        entry_pose = relative_pose(boundary, fresh[0])
        entry_log = se2_log(entry_pose)
        nearest = point_to_polyline(boundary[:2], fresh[:, :2])
        rows.append({
            "tau_s": float(tau), "B_world": boundary.tolist(),
            "delta_B_relative_pose": delta.tolist(),
            "delta_B_translation_m": float(np.linalg.norm(delta[:2])),
            "delta_B_yaw_rad": float(delta[2]),
            "entry_relative_pose_B_frame": entry_pose.tolist(),
            "r_entry_log_B_frame": entry_log.tolist(),
            "r_entry_log_translation_norm_m": float(np.linalg.norm(entry_log[:2])),
            "d_entry_m": float(np.linalg.norm(boundary[:2] - fresh[0, :2])),
            "d_poly_m": nearest["distance_m"], "nearest_polyline": nearest,
        })
        rows[-1]["delta_d_entry_m"] = rows[-1]["d_entry_m"] - rows[0]["d_entry_m"]
        rows[-1]["delta_d_poly_m"] = rows[-1]["d_poly_m"] - rows[0]["d_poly_m"]
    return boundaries, rows


def csv_row(row: dict) -> dict:
    """Flatten metrics; log components and ordinary relative XY stay distinct."""
    flat = {key: row[key] for key in ("tau_s", "delta_B_translation_m", "delta_B_yaw_rad",
            "r_entry_log_translation_norm_m", "d_entry_m", "d_poly_m", "delta_d_entry_m", "delta_d_poly_m")}
    for prefix, key in (("B_world", "B_world"), ("delta_B", "delta_B_relative_pose"),
                        ("entry_relative", "entry_relative_pose_B_frame"), ("r_entry_log", "r_entry_log_B_frame")):
        for suffix, value in zip(("x_m", "y_m", "yaw_rad"), row[key], strict=True):
            flat[f"{prefix}_{suffix}"] = value
    nearest = row["nearest_polyline"]
    flat.update(nearest_segment_index=nearest["segment_index"], nearest_segment_fraction=nearest["segment_fraction"],
                nearest_x_world_m=nearest["nearest_xy_world_m"][0], nearest_y_world_m=nearest["nearest_xy_world_m"][1])
    return flat

"""Descriptive continuous projection geometry; no correspondence or correction."""

from __future__ import annotations

import numpy as np

from reconciliation.robotless_controlled_staleness import finite_scalar, point_to_polyline
from reconciliation.robotless_single_chunk import _agent_pose, validate_waypoints
from reconciliation.se2 import wrap_angle


DEFINITIONS = {
    "frames": "All XY positions in fixed Isaac world (+Z up); metres; yaw/direction CCW about +Z in radians",
    "projection": "closest clamped segment projection; exact ties choose lowest segment index; zero-length segments are points",
    "e_perp_m": "Euclidean ||B.xy-Q.xy||; unsigned minimum segment distance, including endpoint cases",
    "s_Q_m": "sum of preceding XY segment lengths + alpha * winning segment length",
    "normalized_progress": "s_Q / total_FRESH_XY_arc_length; descriptive, not a correspondence index",
    "phi_in_rad": "configured controlled body-forward motion direction = yaw(B), requiring source v>0 and omega=0",
    "phi_F_rad": "atan2 of winning nonzero XY segment; unavailable for a degenerate winner",
    "e_dir_rad": "wrap(phi_F-phi_in) in [-pi,pi); unavailable when tangent unavailable",
    "theta_Q_rad": "wrap(theta_j + alpha*wrap(theta_(j+1)-theta_j)); shortest-angle pose-yaw interpolation",
    "e_yaw_rad": "wrap(theta_Q-yaw(B)); independent of path tangent",
    "tau_s": "source controlled delay; not actual LightNav latency or execution time",
    "zero_total_arc_length": "rejected: normalized progress undefined",
    "degenerate_winner_yaw": "alpha=0 selects first endpoint pose yaw; no invented tangent",
}


def projection_geometry(boundary, fresh_world) -> dict:
    """Reuse the tested point-to-polyline projection and SE(2) angle utility."""
    b = _agent_pose(boundary)
    fresh = validate_waypoints(fresh_world)
    with np.errstate(over="ignore", invalid="ignore"):
        segments = np.diff(fresh[:, :2], axis=0)
        lengths = np.linalg.norm(segments, axis=1)
        total = float(lengths.sum())
    if not np.isfinite(total) or total <= 0:
        raise ValueError("FRESH total XY arc length must be finite and positive")
    projection = point_to_polyline(b[:2], fresh[:, :2])
    j, alpha = projection["segment_index"], projection["segment_fraction"]
    available = bool(lengths[j] > 0)
    tangent = float(np.arctan2(segments[j, 1], segments[j, 0])) if available else None
    direction_error = float(wrap_angle(tangent - b[2])) if available else None
    yaw_delta = float(wrap_angle(fresh[j + 1, 2] - fresh[j, 2]))
    yaw_q = float(wrap_angle(fresh[j, 2] + alpha * yaw_delta))
    yaw_error = float(wrap_angle(yaw_q - b[2]))
    progress = float(lengths[:j].sum() + alpha * lengths[j])
    interior = bool(available and 0 < alpha < 1)
    return {
        "B_world": b.tolist(), "segment_index": j, "alpha": alpha,
        "Q_xy_world_m": projection["nearest_xy_world_m"], "e_perp_m": projection["distance_m"],
        "projection_is_interior": interior, "projection_is_endpoint": not interior,
        "projection_location": "degenerate_point" if not available else "interior" if interior else "start_endpoint" if alpha == 0 else "end_endpoint",
        "segment_length_m": float(lengths[j]), "total_FRESH_arc_length_m": total,
        "s_Q_m": progress, "normalized_progress": progress / total,
        "phi_in_rad": float(b[2]), "phi_F_rad": tangent,
        "tangent_available": available, "tangent_unavailable_reason": None if available else "winning segment has zero XY length",
        "e_dir_rad": direction_error,
        "abs_e_dir_rad": abs(direction_error) if available else None,
        "abs_e_dir_deg": float(np.degrees(abs(direction_error))) if available else None,
        "theta_Q_rad": yaw_q, "e_yaw_rad": yaw_error,
        "abs_e_yaw_rad": abs(yaw_error), "abs_e_yaw_deg": float(np.degrees(abs(yaw_error))),
    }


def characterize_projection(boundaries, fresh_world, previous_conditions, motion, *, distance_atol_m) -> list[dict]:
    """Use saved B/tau, retaining previous d_entry/d_poly as references only."""
    v = finite_scalar(motion["v_mps"], "source v_mps")
    omega = finite_scalar(motion["omega_radps"], "source omega_radps")
    if v <= 0 or omega != 0:
        raise ValueError("incoming direction requires controlled v>0 and omega=0")
    tolerance = finite_scalar(distance_atol_m, "distance_atol_m", nonnegative=True)
    b = validate_waypoints(boundaries)
    fresh = validate_waypoints(fresh_world)
    fresh.setflags(write=False)
    if len(b) != len(previous_conditions) or len(b) != len(motion["tau_s"]):
        raise ValueError("saved boundaries/tau/conditions must have matching lengths")
    rows = []
    for pose, previous, tau in zip(b, previous_conditions, motion["tau_s"], strict=True):
        finite_scalar(tau, "controlled tau_s", nonnegative=True)
        if not np.array_equal(pose, previous["B_world"]) or tau != previous["tau_s"]:
            raise ValueError("boundary/tau differs from frozen controlled source")
        row = projection_geometry(pose, fresh)
        reference = finite_scalar(previous["d_poly_m"], "previous d_poly_m", nonnegative=True)
        difference = row["e_perp_m"] - reference
        if abs(difference) > tolerance:
            raise ValueError("e_perp differs from previous d_poly beyond numerical tolerance")
        row.update(tau_s=float(tau), d_entry_reference_m=finite_scalar(previous["d_entry_m"], "previous d_entry_m", nonnegative=True),
                   d_poly_reference_m=reference, e_perp_minus_previous_d_poly_m=difference)
        rows.append(row)
    return rows


def csv_row(row: dict) -> dict:
    flat = {key: value for key, value in row.items() if key not in ("B_world", "Q_xy_world_m")}
    flat.update(B_x_world_m=row["B_world"][0], B_y_world_m=row["B_world"][1], B_yaw_rad=row["B_world"][2],
                Q_x_world_m=row["Q_xy_world_m"][0], Q_y_world_m=row["Q_xy_world_m"][1])
    return flat

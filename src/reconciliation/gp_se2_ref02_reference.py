"""Fixed-geometry REF-02 source-progress selection and independent diagnostics.

Original progress is a source row-order label, not time or distance. Selection
never interpolates poses, changes nearest-pose weights, or modifies the path.
"""
from __future__ import annotations

import math

import numpy as np

from reconciliation.online_mpc_adapter import selection_audit

METHODS = ("A_NATIVE", "B_DENSE_ROW_STEP", "C_DENSE_SOURCE_PROGRESS")
SOURCE_PROGRESS_STRIDE = 1.0
AUDIT_ATOL = 1e-12
LINEAGE_GEOMETRY_ATOL = 1e-11
NEAR_TIE_COST_ATOL = 1e-10
NEAR_TIE_COST_RTOL = 1e-10


def _wrap(value):
    # Deliberately the same scalar floating-point operations as pinned mpc.py.
    return math.atan2(math.sin(float(value)), math.cos(float(value)))


def _path(reference):
    path = np.asarray(reference, dtype=np.float64)
    if path.ndim != 2 or path.shape[1:] != (3,) or not len(path) or not np.isfinite(path).all():
        raise ValueError("reference must be nonempty finite Nx3")
    return path


def _inputs(reference, pose, horizon, weights):
    path = _path(reference)
    state = np.asarray(pose, dtype=np.float64)
    w = np.asarray(weights, dtype=np.float64)
    if state.shape != (3,) or not np.isfinite(state).all():
        raise ValueError("pose must be a finite 3-vector")
    if isinstance(horizon, bool) or not isinstance(horizon, (int, np.integer)) or horizon <= 0:
        raise ValueError("horizon must be a positive integer")
    if w.shape != (3,) or not np.isfinite(w).all() or np.any(w <= 0.):
        raise ValueError("weights must be a finite positive 3-vector")
    return path, state, w


def _progress(reference, progress, constant_reference):
    path = _path(reference)
    s = np.asarray(progress, dtype=np.float64)
    if s.shape != (len(path),) or not np.isfinite(s).all() or np.any(s < 0.):
        raise ValueError("source progress must be finite nonnegative with one value per row")
    if constant_reference:
        if not np.all(s == s[0]) or s[0] != int(s[0]) or not np.array_equal(path, np.tile(path[0], (len(path), 1))):
            raise ValueError("constant reference must repeat one integer source row and identical pose")
    elif len(s) > 1 and not np.all(np.diff(s) > 0.):
        raise ValueError("source progress must be strictly increasing; constant metadata is required for repeats")
    return s


def validate_lineage(reference, lineage, *, constant_reference_metadata=None):
    """Authenticate saved REF-01 lineage without constructing correspondence.

The exceptional repeated-single-source case requires explicit metadata with
``constant_reference=True``, ``retained_source_row_count=1`` and the matching
``original_source_row_index``. Merely equal geometry is insufficient.
    """
    path = _path(reference)
    if len(lineage) != len(path):
        raise ValueError("lineage row count differs from reference")
    progress = []
    for i, row in enumerate(lineage):
        try:
            left, right = row["original_left_row_index"], row["original_right_row_index"]
            alpha = float(row["interpolation_alpha"])
            s = float(row["original_fractional_row_coordinate"])
            xy = np.asarray(row["world_xy"], dtype=np.float64)
            yaw = float(row["wrapped_yaw"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("lineage lacks valid source identity or geometry") from error
        if row.get("derived_row_index") != i:
            raise ValueError("lineage derived index/order mismatch")
        if (isinstance(left, bool) or isinstance(right, bool)
                or not isinstance(left, (int, np.integer)) or not isinstance(right, (int, np.integer))
                or left < 0 or right < left or right - left > 1):
            raise ValueError("lineage source indices must identify one row or adjacent original rows")
        if not math.isfinite(alpha) or not 0. <= alpha <= 1. or (left == right and alpha != 0.):
            raise ValueError("lineage interpolation alpha is invalid")
        if not math.isfinite(s) or s != left + alpha * (right - left):
            raise ValueError("lineage fractional coordinate does not equal its saved source identity")
        if (xy.shape != (2,) or not np.isfinite(xy).all() or not math.isfinite(yaw)
                or not np.allclose(xy, path[i, :2], atol=LINEAGE_GEOMETRY_ATOL, rtol=0.)
                or abs(_wrap(yaw - path[i, 2])) > LINEAGE_GEOMETRY_ATOL):
            raise ValueError("lineage geometry differs from installed reference")
        if "stored_yaw" in row and (not math.isfinite(float(row["stored_yaw"]))
                or abs(_wrap(float(row["stored_yaw"]) - path[i, 2])) > LINEAGE_GEOMETRY_ATOL):
            raise ValueError("lineage stored yaw differs from installed reference")
        if "unwrapped_yaw" in row and (not math.isfinite(float(row["unwrapped_yaw"]))
                or abs(_wrap(float(row["unwrapped_yaw"]) - path[i, 2])) > LINEAGE_GEOMETRY_ATOL):
            raise ValueError("lineage unwrapped yaw differs from installed reference")
        progress.append(s)
    constant = constant_reference_metadata is not None
    if constant:
        metadata = constant_reference_metadata
        if (metadata.get("constant_reference") is not True
                or metadata.get("retained_source_row_count") != 1
                or metadata.get("original_source_row_index") != progress[0]
                or any(row["original_left_row_index"] != progress[0]
                       or row["original_right_row_index"] != progress[0] for row in lineage)):
            raise ValueError("constant reference metadata does not prove one retained original source row")
    return _progress(path, progress, constant).copy()


def _costs(path, state, weights):
    errors = path - state
    errors[:, 2] = [_wrap(value) for value in errors[:, 2]]
    terms = errors * errors * weights
    # Keep exactly the official sum ordering for nearest-row/tie parity.
    total = np.sum(terms, axis=1)
    return total, np.sum(terms[:, :2], axis=1), terms[:, 2]


def _unwrap_selected(path, state, indices):
    reference = path[indices].copy()
    previous = float(state[2])
    for point in reference:
        point[2] = previous + _wrap(point[2] - previous)
        previous = float(point[2])
    return reference


def _stride_details(path, progress, nearest, indices, queries, reference, *, constant_reference):
    requested = progress[nearest] + np.arange(1, len(indices) + 1, dtype=np.float64)
    selected = progress[indices]
    repeated = [False] + [int(indices[i]) == int(indices[i-1]) for i in range(1, len(indices))]
    return {"nearest_index": int(nearest), "indices": np.asarray(indices, dtype=int).tolist(),
            "reference_world": reference.tolist(), "nearest_is_final_row": nearest == len(path)-1,
            "nearest_original_fractional_row_coordinate": float(progress[nearest]),
            "selected_original_fractional_row_coordinates": selected.tolist(),
            "requested_source_progress_before_clamp": requested.tolist(), "q_h": queries.tolist(),
            "progress_overshoot": (selected-queries).tolist(),
            "endpoint_clamped": (requested > progress[-1]).tolist(),
            "endpoint_selected": (np.asarray(indices) == len(path)-1).tolist(),
            "endpoint_repeated_per_target": repeated,
            "endpoint_repeated": len(set(indices)) < len(indices),
            "constant_reference": bool(constant_reference),
            "source_progress_stride": SOURCE_PROGRESS_STRIDE,
            "search_policy": "strict float64 searchsorted(side='left'); no search tolerance or rounding",
            "selection_semantics": "nearest unchanged weighted pose; s_j+h from same nearest row; ceil existing progress; endpoint clamp/repeat; sequential yaw unwrap"}


def select_source_progress(reference, pose, progress, *, horizon, weights, constant_reference=False):
    """Select existing rows with q_h=min(s_j+h,s_last), always from the same s_j.

``constant_reference=True`` is only for lineage previously authenticated by
``validate_lineage`` and repeats the installed final row. There is no state.
    """
    path, state, w = _inputs(reference, pose, horizon, weights)
    s = _progress(path, progress, constant_reference)
    total, _, _ = _costs(path, state, w)
    nearest = int(np.argmin(total))
    queries = np.minimum(s[nearest] + np.arange(1, horizon+1, dtype=np.float64), s[-1])
    indices = (np.full(horizon, len(path)-1, dtype=np.int64) if constant_reference else
               np.minimum(np.searchsorted(s, queries, side="left"), len(path)-1))
    selected = _unwrap_selected(path, state, indices)
    return selected, _stride_details(path, s, nearest, indices, queries, selected,
                                    constant_reference=constant_reference)


def audit_source_progress(reference, pose, progress, actual_reference, *, horizon, weights,
                          actual_indices=None, constant_reference=False):
    """Independent first-satisfying-row audit of the actual solver reference.

Uses explicit scans, not ``select_source_progress`` or ``searchsorted``. Both
periodic yaw identity and literal sequential unwrap are checked. The audit
never feeds a controller or disables the original baseline audit.
    """
    path, state, w = _inputs(reference, pose, horizon, weights)
    s = _progress(path, progress, constant_reference)
    # Independently reconstruct cost with pinned scalar angle wrapping.
    differences = np.array([[float(point[0]-state[0]), float(point[1]-state[1]),
                             math.atan2(math.sin(float(point[2]-state[2])),
                                        math.cos(float(point[2]-state[2])))] for point in path])
    costs = np.sum(differences*differences*w, axis=1)
    nearest = next(i for i, value in enumerate(costs) if value == min(costs))
    queries, ids = [], []
    for h in range(1, horizon+1):
        q = min(float(s[nearest])+h, float(s[-1]))
        queries.append(q)
        ids.append(len(path)-1 if constant_reference else next(i for i, value in enumerate(s) if value >= q))
    if actual_indices is not None and list(actual_indices) != ids:
        raise ValueError("actual source-progress indices disagree with independent lower-bound audit")
    actual = np.asarray(actual_reference, dtype=np.float64)
    if actual.shape != (horizon, 3) or not np.isfinite(actual).all():
        raise ValueError("actual selected reference has invalid shape or values")
    if not np.allclose(actual[:, :2], path[ids, :2], atol=AUDIT_ATOL, rtol=0.):
        raise ValueError("actual source-progress XY disagrees with selected existing rows")
    previous, expected_yaw = float(state[2]), []
    for i in ids:
        raw = float(path[i, 2]-previous)
        previous += math.atan2(math.sin(raw), math.cos(raw))
        expected_yaw.append(previous)
    if any(abs(_wrap(float(a-b))) > AUDIT_ATOL for a, b in zip(actual[:, 2], path[ids, 2])):
        raise ValueError("actual source-progress yaw violates periodic row identity")
    if not np.allclose(actual[:, 2], expected_yaw, atol=AUDIT_ATOL, rtol=0.):
        raise ValueError("actual source-progress yaw violates literal sequential unwrap")
    result = _stride_details(path, s, nearest, np.array(ids), np.array(queries), actual,
                             constant_reference=constant_reference)
    result["index_validation"] = "independent linear lower-bound scan checked against actual solver reference"
    return result


def enrich_selector_result(reference, pose, actual_reference, lineage, *, method, weights, horizon,
                           original_goal_row_index, selection_details=None, constant_reference_metadata=None):
    """Common recorded diagnostics with method-appropriate independent audits."""
    if method not in METHODS:
        raise ValueError("unknown REF-02 method")
    path, state, w = _inputs(reference, pose, horizon, weights)
    s = validate_lineage(path, lineage, constant_reference_metadata=constant_reference_metadata)
    actual = np.asarray(actual_reference, dtype=np.float64)
    if method == METHODS[2]:
        checked = audit_source_progress(path, state, s, actual, horizon=horizon, weights=w,
            actual_indices=None if selection_details is None else selection_details.get("indices"),
            constant_reference=constant_reference_metadata is not None)
    else:
        checked = selection_audit(path, state, actual, horizon=horizon, weights=w)
        expected = _unwrap_selected(path, state, checked["indices"])
        if not np.allclose(actual[:, 2], expected[:, 2], atol=AUDIT_ATOL, rtol=0.):
            raise ValueError("official selected yaw violates literal sequential unwrap")
    if selection_details is not None:
        for key in ("nearest_index", "indices", "reference_world"):
            if selection_details.get(key) != checked[key]:
                raise ValueError(f"selection details differ from actual reference audit: {key}")
        if method == METHODS[2]:
            for key in ("q_h", "progress_overshoot", "endpoint_clamped", "endpoint_repeated_per_target"):
                if selection_details.get(key) != checked[key]:
                    raise ValueError(f"source-progress details differ from independent audit: {key}")
    total, xy, yaw = _costs(path, state, w)
    nearest, ids = checked["nearest_index"], checked["indices"]
    if nearest != int(np.argmin(total)):
        raise ValueError("baseline audit nearest differs from pinned scalar-wrap official cost")
    ordered = np.sort(total)
    margin = float(ordered[1]-ordered[0]) if len(total)>1 else None
    near_tie = NEAR_TIE_COST_ATOL+NEAR_TIE_COST_RTOL*max(1., abs(float(total[nearest])))
    source_rows = [dict(lineage[i]) for i in ids]
    first_distance = float(np.linalg.norm(actual[0, :2]-state[:2]))
    last_distance = float(np.linalg.norm(actual[-1, :2]-state[:2]))
    arc = float(np.linalg.norm(np.diff(actual[:, :2], axis=0), axis=1).sum())
    yaw_span = float(actual[-1, 2]-actual[0, 2])
    result = {**checked, "method": method, "input_pose_world": state.tolist(), "weights": w.tolist(),
        "weighted_xy_contributions": xy.tolist(), "weighted_yaw_contributions": yaw.tolist(),
        "weighted_total_pose_distances": total.tolist(), "nearest_cost": float(total[nearest]),
        "nearest_original_fractional_row_coordinate": float(s[nearest]),
        "nearest_source_row": dict(lineage[nearest]), "nearest_cost_margin": margin, "nearest_tie_margin": margin,
        "exact_tied_nearest_indices": np.flatnonzero(total == total[nearest]).tolist(),
        "near_tied_nearest_indices": np.flatnonzero(total-total[nearest] <= near_tie).tolist(),
        "near_tie_cost_threshold": near_tie,
        "tie_policy": "unchanged official numpy.argmin: first lowest-cost row; near ties diagnostic only",
        "selected_original_fractional_row_coordinates": s[ids].tolist(), "selected_source_rows": source_rows,
        "selected_world_xy": actual[:, :2].tolist(), "selected_wrapped_yaw_rad": [_wrap(y) for y in actual[:, 2]],
        "official_sequentially_unwrapped_reference_yaw_rad": actual[:, 2].tolist(),
        "official_reference_unwrapped_yaw": actual[:, 2].tolist(),
        "current_to_first_target_distance_m": first_distance, "selected_first_target_distance_m": first_distance,
        "current_to_last_target_distance_m": last_distance, "selected_last_target_distance_m": last_distance,
        "selected_horizon_xy_arc_m": arc, "horizon_xy_arc_length_m": arc,
        "current_through_horizon_xy_arc_m": first_distance+arc,
        "selected_horizon_yaw_span_rad": yaw_span, "horizon_yaw_span_rad": yaw_span,
        "selected_horizon_yaw_range_rad": float(np.ptp(actual[:, 2])),
        "selected_horizon_yaw_variation_rad": float(np.abs(np.diff(actual[:, 2])).sum()),
        "current_to_last_target_yaw_rad": float(actual[-1, 2]-state[2]),
        "current_through_horizon_yaw_variation_rad": float(np.abs(np.diff(np.r_[state[2], actual[:, 2]])).sum()),
        "original_goal_row_index": int(original_goal_row_index),
        "final_goal_row_in_horizon": bool(any(s[i] == original_goal_row_index for i in ids)),
        "endpoint_repetition_count": len(ids)-len(set(ids)),
        "original_fractional_coordinate_units": "source row-order progress; not time or distance",
        "poses_interpolated_by_selector": False, "installed_path_modified": False}
    if method != METHODS[2]:
        result.update(q_h=None, progress_overshoot=None, source_progress_stride=None,
                      endpoint_clamped=[nearest+h > len(path)-1 for h in range(1, horizon+1)],
                      endpoint_selected=[i == len(path)-1 for i in ids],
                      endpoint_repeated_per_target=[False]+[ids[i] == ids[i-1] for i in range(1, len(ids))])
    return result

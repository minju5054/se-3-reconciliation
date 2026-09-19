"""REF-01 factorized reference inputs and selection diagnostics, never a controller.

Only original ``prepare_reference`` and ``interpolate_rows`` define the intervention.
Original fractional row coordinates are source identities, not time or distance.
"""
from __future__ import annotations

import hashlib

import numpy as np

from reconciliation.gp_se2_reference import interpolate_rows, poses, prepare_reference
from reconciliation.online_mpc_adapter import selection_audit
from reconciliation.se2 import wrap_angle

VARIANTS = ("R00_NATIVE", "R10_SUFFIX_ONLY", "R01_RESAMPLE_ONLY", "R11_CURRENT_ADAPTER")
FIRST_TIME_S = .1
HORIZON_S = 3.
OUTPUT_DT_S = .1
NEAR_ZERO_TRANSLATION_M = .001
YAW_DIAGNOSTIC_TOL_RAD = 1e-6
KNOT_IDENTITY_TOL = 1e-12
SAMPLED_DISTANCE_MAX_SPACING_M = .001
SAMPLED_DISTANCE_MIN_SUBDIVISIONS = 100
NEAR_TIE_COST_ATOL = 1e-10
NEAR_TIE_COST_RTOL = 1e-10


def _unwrap(yaw):
    a = np.asarray(yaw, dtype=np.float64)
    return np.r_[a[0], a[0] + np.cumsum(wrap_angle(np.diff(a)))]


def array_value_hash(value):
    """Value identity, explicitly distinct from the hash of an on-disk .npy file."""
    a = np.ascontiguousarray(value, dtype=np.float64)
    return hashlib.sha256(str(a.shape).encode() + b";float64;" + a.tobytes()).hexdigest()


def _lineage(native, start, output, resampled):
    retained = native[start:]
    source_yaw = _unwrap(native[:, 2])
    query = np.arange(1, 31, dtype=float) * OUTPUT_DT_S
    times = (np.linspace(FIRST_TIME_S, HORIZON_S, len(retained))
             if len(retained) > 1 else np.array([FIRST_TIME_S]))
    rows = []
    for i, point in enumerate(output):
        if not resampled:
            left = right = start + i
            alpha = 0.
        elif len(retained) == 1:
            left = right = start
            alpha = 0.
        else:
            q = query[i]
            local_right = min(int(np.searchsorted(times, q, side="right")), len(times) - 1)
            local_left = max(0, local_right - 1)
            alpha = float(np.clip((q - times[local_left]) / (times[local_right] - times[local_left]), 0, 1))
            left, right = local_left + start, local_right + start
            # Identity is based on interpolation position, never on equal XY/yaw.
            if alpha == 0.:
                right = left
            elif alpha == 1.:
                left = right
                alpha = 0.
        s = float(left + alpha * (right - left))
        interpolated = left != right
        unwrapped = float(source_yaw[left] + alpha * (source_yaw[right] - source_yaw[left]))
        predicted_xy = native[left, :2] + alpha * (native[right, :2] - native[left, :2])
        if not np.allclose(point[:2], predicted_xy, atol=1e-12, rtol=0) or abs(float(wrap_angle(point[2] - unwrapped))) > 1e-12:
            raise ValueError("row provenance does not reconstruct original helper output")
        rows.append({"derived_row_index": i, "original_left_row_index": left,
                     "original_right_row_index": right, "interpolation_alpha": alpha,
                     "original_fractional_row_coordinate": s, "world_xy": point[:2].tolist(),
                     "wrapped_yaw": float(wrap_angle(point[2])), "stored_yaw": float(point[2]),
                     "unwrapped_yaw": unwrapped, "original_row": not interpolated,
                     "interpolated": interpolated})
    return rows


def _sample_polyline(xy):
    a = np.asarray(xy, dtype=float)
    points = [a[:1]]
    counts = []
    for left, right in zip(a[:-1], a[1:]):
        count = max(SAMPLED_DISTANCE_MIN_SUBDIVISIONS,
                    int(np.ceil(np.linalg.norm(right - left) / SAMPLED_DISTANCE_MAX_SPACING_M)))
        counts.append(count)
        points.append(left + np.linspace(0, 1, count + 1)[1:, None] * (right - left))
    return np.concatenate(points), counts


def _point_polyline_distances(points, polyline):
    if len(polyline) == 1:
        return np.linalg.norm(points - polyline[0], axis=1)
    delta = np.diff(polyline, axis=0)
    norm2 = np.sum(delta * delta, axis=1)
    result = np.full(len(points), np.inf)
    for left, segment, square in zip(polyline[:-1], delta, norm2):
        alpha = np.zeros(len(points)) if square == 0 else np.clip((points - left) @ segment / square, 0, 1)
        result = np.minimum(result, np.linalg.norm(points - left - alpha[:, None] * segment, axis=1))
    return result


def sampled_polyline_distance(source_xy, derived_xy):
    """Samples each source segment and measures exact distance to target segments.

    The maximum is a sampled lower bound on continuous directed Hausdorff
    distance, not a proof that the continuous curves coincide.
    """
    original, derived = np.asarray(source_xy, float), np.asarray(derived_xy, float)
    a, ac = _sample_polyline(original)
    b, bc = _sample_polyline(derived)
    ab, ba = _point_polyline_distances(a, derived), _point_polyline_distances(b, original)
    return {"source_to_derived_max_sampled_m": float(np.max(ab)),
            "derived_to_source_max_sampled_m": float(np.max(ba)),
            "source_to_derived_mean_sampled_m": float(np.mean(ab)),
            "derived_to_source_mean_sampled_m": float(np.mean(ba)),
            "source_sample_count": len(a), "derived_sample_count": len(b),
            "source_segment_subdivisions": ac, "derived_segment_subdivisions": bc,
            "maximum_source_sampling_spacing_m": SAMPLED_DISTANCE_MAX_SPACING_M,
            "minimum_subdivisions_per_segment": SAMPLED_DISTANCE_MIN_SUBDIVISIONS,
            "target_distance": "exact distance from sampled point to all target XY line segments",
            "limitation": "sampled XY diagnostic; not continuous Hausdorff proof; ignores yaw"}


def _geometry(native, start, output, lineage):
    delta_xy = np.diff(output[:, :2], axis=0)
    lengths = np.linalg.norm(delta_xy, axis=1)
    dyaw = wrap_angle(np.diff(output[:, 2]))
    unwrapped = np.array([r["unwrapped_yaw"] for r in lineage])
    progress = np.array([r["original_fractional_row_coordinate"] for r in lineage])
    raw_lengths = np.linalg.norm(np.diff(native[:, :2], axis=0), axis=1)
    raw_dyaw = wrap_angle(np.diff(native[:, 2]))
    knots = []
    for i in range(len(native)):
        corner = None
        if 0 < i < len(native) - 1 and raw_lengths[i-1] > 1e-12 and raw_lengths[i] > 1e-12:
            before, after = native[i, :2] - native[i-1, :2], native[i+1, :2] - native[i, :2]
            corner = float(np.arctan2(before[0]*after[1] - before[1]*after[0], before @ after))
        adjacent = list(range(max(0, i-1), min(i+1, len(raw_lengths))))
        rotation_adjacent = any(raw_lengths[j] <= NEAR_ZERO_TRANSLATION_M and abs(raw_dyaw[j]) > YAW_DIAGNOSTIC_TOL_RAD for j in adjacent)
        knots.append({"original_row_index": i, "world_pose": native[i].tolist(),
                      "removed_by_suffix": i < start,
                      "included_derived_row_indices": np.flatnonzero(np.abs(progress - i) <= KNOT_IDENTITY_TOL).tolist(),
                      "corner_turn_rad": corner, "is_corner": corner is not None and abs(corner) > YAW_DIAGNOSTIC_TOL_RAD,
                      "adjacent_yaw_transition": any(abs(raw_dyaw[j]) > YAW_DIAGNOSTIC_TOL_RAD for j in adjacent),
                      "adjacent_near_zero_translation_rotation": rotation_adjacent})
    native_yaw = _unwrap(native[:, 2])
    return {"row_count": len(output), "start_pose_world": output[0].tolist(), "end_pose_world": output[-1].tolist(),
            "xy_segment_lengths_m": lengths.tolist(), "yaw_increments_wrapped_rad": dyaw.tolist(),
            "yaw_increments_unwrapped_rad": np.diff(unwrapped).tolist(),
            "total_xy_arc_length_m": float(lengths.sum()),
            "accumulated_yaw_variation_rad": float(np.abs(dyaw).sum()),
            "net_unwrapped_yaw_change_rad": float(unwrapped[-1] - unwrapped[0]),
            "near_zero_translation_threshold_m": NEAR_ZERO_TRANSLATION_M,
            "yaw_diagnostic_threshold_rad": YAW_DIAGNOSTIC_TOL_RAD,
            "near_zero_translation_segments": np.flatnonzero(lengths <= NEAR_ZERO_TRANSLATION_M).tolist(),
            "rotation_only_segments": np.flatnonzero((lengths <= 1e-12) & (np.abs(dyaw) > YAW_DIAGNOSTIC_TOL_RAD)).tolist(),
            "near_zero_translation_rotation_segments": np.flatnonzero((lengths <= NEAR_ZERO_TRANSLATION_M) & (np.abs(dyaw) > YAW_DIAGNOSTIC_TOL_RAD)).tolist(),
            "duplicate_xy_segments": np.flatnonzero(lengths == 0).tolist(),
            "removed_prefix_rows": [{"original_row_index": i, "world_pose": native[i].tolist(), "unwrapped_yaw": float(native_yaw[i])} for i in range(start)],
            "removed_prefix_including_edge_to_first_retained_xy_length_m": float(raw_lengths[:start].sum()),
            "removed_prefix_including_edge_to_first_retained_yaw_variation_rad": float(np.abs(raw_dyaw[:start]).sum()),
            "original_knots": knots,
            "comparison_to_native": sampled_polyline_distance(native[:, :2], output[:, :2]),
            "comparison_to_retained_source": sampled_polyline_distance(native[start:, :2], output[:, :2])}


def build_variants(native_world, boundary_pose, weights, *, expected_common, preparation_metadata=None):
    """Construct exactly the frozen 2x2 without mutating or reanchoring inputs.

    ``expected_common`` is mandatory to authenticate the original adapter.  If
    metadata is supplied, every available original preparation field is checked.
    """
    native = poses(native_world).copy()
    expected = poses(expected_common)
    prepared = prepare_reference(native, boundary_pose, weights, first_time_s=FIRST_TIME_S,
                                 horizon_s=HORIZON_S, output_dt_s=OUTPUT_DT_S)
    k = prepared["first_future_row_index"]
    if not np.array_equal(prepared["common_world"], expected):
        raise ValueError("R11 does not exactly equal saved F_common")
    if preparation_metadata is not None:
        for key in ("nearest_row_index", "first_future_row_index", "original_row_indices",
                    "intrinsic_model_waypoint_dt_s"):
            if key not in preparation_metadata or preparation_metadata[key] != prepared[key]:
                raise ValueError(f"original preparation metadata mismatch: {key}")
        for key in ("native_world", "suffix_world", "common_world", "goal_world", "row_times_s", "sample_times_s",
                    "weighted_squared_pose_distances"):
            if key not in preparation_metadata or not np.array_equal(preparation_metadata[key], prepared[key]):
                raise ValueError(f"original preparation array mismatch: {key}")
    variants = {}
    for name, start, resampled in ((VARIANTS[0], 0, False), (VARIANTS[1], k, False),
                                   (VARIANTS[2], 0, True), (VARIANTS[3], k, True)):
        retained = native[start:]
        row_times = (np.linspace(FIRST_TIME_S, HORIZON_S, len(retained))
                     if len(retained) > 1 else np.array([FIRST_TIME_S]))
        output_times = np.arange(1, 31, dtype=float) * OUTPUT_DT_S
        output = interpolate_rows(retained, row_times, output_times) if resampled else retained.copy()
        if not np.array_equal(output[-1, :2], native[-1, :2]) or abs(float(wrap_angle(output[-1, 2] - native[-1, 2]))) > 1e-12:
            raise ValueError("reference intervention changed original endpoint pose")
        lineage = _lineage(native, start, output, resampled)
        variants[name] = {"reference_world": output, "row_provenance": lineage,
                          "geometry_audit": _geometry(native, start, output, lineage),
                          "suffix_applied": start == k and name in (VARIANTS[1], VARIANTS[3]),
                          "resampling_applied": resampled, "original_first_row_index": start,
                          "row_times_s": row_times.tolist() if resampled else None,
                          "query_times_s": output_times.tolist() if resampled else None,
                          "reference_value_sha256": array_value_hash(output),
                          "endpoint_bitwise_equal": bool(np.array_equal(output[-1], native[-1])),
                          "endpoint_pose_equal_atol_1e_12": True}
    if not np.array_equal(variants[VARIANTS[0]]["reference_world"], native):
        raise ValueError("R00 native equality failed")
    if not np.array_equal(variants[VARIANTS[1]]["reference_world"], native[k:]):
        raise ValueError("R10 suffix equality failed")
    if not np.array_equal(variants[VARIANTS[3]]["reference_world"], expected):
        raise ValueError("R11 original adapter equality failed")
    return {"suffix_selection": {"nearest_row_index": prepared["nearest_row_index"],
                "first_future_row_index": k, "original_row_count": len(native),
                "removed_original_row_indices": list(range(k)), "original_goal_row_index": len(native)-1,
                "B_world": np.asarray(boundary_pose).tolist(), "weights": np.asarray(weights).tolist(),
                "weighted_squared_pose_distances": prepared["weighted_squared_pose_distances"],
                "computation": "original prepare_reference once on original F_native at original B; argmin then min(j+1,N-1)",
                "suffix_recomputed_during_rollout": False},
            "variants": variants,
            "conventions": {"frame": "unchanged fixed Isaac world; metres; yaw radians CCW about +Z",
                "interpolation_row_times_s": "linspace(0.1,3.0,L), or [0.1] for L=1",
                "interpolation_query_times_s": "arange(1,31)*0.1; original interpolate_rows",
                "intrinsic_lightnav_waypoint_dt_s": None,
                "timestamps_passed_to_mpc": False,
                "original_fractional_row_coordinate": "source row-order progress only; neither physical time nor distance",
                "unwrapped_yaw": "shortest-increment unwrap anchored to original native row 0; visualization/provenance only",
                "waypoint_timing_is_experimental_convention": True,
                "goal": "unchanged original native final pose", "raw_modified": False,
                "B_reanchoring": False}}


def enrich_selection(reference_world, input_pose, official_reference, row_provenance, *,
                     weights, horizon, original_goal_row_index, audit=None):
    """Check the actual official reference and describe its source-row identities.

    This is read-only: no state integration, memory modification or MPC solve.
    A tie diagnostic never changes NumPy/official first-argmin selection.
    """
    path, ref = poses(reference_world), poses(official_reference)
    state, w = np.asarray(input_pose, float), np.asarray(weights, float)
    checked = selection_audit(path, state, ref, horizon=horizon, weights=w)
    if audit is not None:
        for key in ("nearest_index", "indices", "reference_world", "endpoint_repeated", "nearest_is_final_row"):
            if audit.get(key) != checked[key]:
                raise ValueError(f"provided selection audit disagrees with independent check: {key}")
    if len(row_provenance) != len(path):
        raise ValueError("row provenance count differs from installed reference")
    for i, row in enumerate(row_provenance):
        if row["derived_row_index"] != i or not np.allclose(row["world_xy"], path[i, :2], atol=1e-11, rtol=0) or abs(float(wrap_angle(row["wrapped_yaw"] - path[i, 2]))) > 1e-11:
            raise ValueError("row provenance geometry does not match installed path")
    # selection_audit uses atan2(sin,cos), like the official wrapped-angle cost.
    residual = path - state
    residual[:, 2] = np.arctan2(np.sin(residual[:, 2]), np.cos(residual[:, 2]))
    xy_cost = np.sum(residual[:, :2] ** 2 * w[:2], axis=1)
    yaw_cost = residual[:, 2] ** 2 * w[2]
    total = xy_cost + yaw_cost
    nearest = checked["nearest_index"]
    sorted_cost = np.sort(total)
    margin = float(sorted_cost[1] - sorted_cost[0]) if len(total) > 1 else None
    near_tie_threshold = NEAR_TIE_COST_ATOL + NEAR_TIE_COST_RTOL * max(1., abs(float(total[nearest])))
    ids = checked["indices"]
    source_rows = [dict(row_provenance[i]) for i in ids]
    source_progress = [row["original_fractional_row_coordinate"] for row in source_rows]
    sequential_yaw = []
    previous = state[2]
    for i in ids:
        previous = previous + float(wrap_angle(path[i, 2] - previous))
        sequential_yaw.append(previous)
    if not np.allclose(ref[:, 2], sequential_yaw, atol=1e-12, rtol=0):
        raise ValueError("official reference does not have expected sequential yaw unwrap")
    goal_included = any(abs(s - original_goal_row_index) <= KNOT_IDENTITY_TOL for s in source_progress)
    result = {**checked, "input_pose_world": state.tolist(), "weights": w.tolist(),
              "weighted_xy_contributions": xy_cost.tolist(), "weighted_yaw_contributions": yaw_cost.tolist(),
              "weighted_total_pose_distances": total.tolist(),
              "nearest_original_fractional_row_coordinate": row_provenance[nearest]["original_fractional_row_coordinate"],
              "nearest_source_row": dict(row_provenance[nearest]),
              "selected_original_fractional_row_coordinates": source_progress,
              "selected_source_rows": source_rows,
              "selected_world_xy": ref[:, :2].tolist(),
              "selected_wrapped_yaw_rad": wrap_angle(ref[:, 2]).tolist(),
              "official_sequentially_unwrapped_reference_yaw_rad": ref[:, 2].tolist(),
              "nearest_cost_margin": margin,
              "exact_tied_nearest_indices": np.flatnonzero(total == total[nearest]).tolist(),
              "near_tied_nearest_indices": np.flatnonzero(total - total[nearest] <= near_tie_threshold).tolist(),
              "near_tie_cost_threshold": near_tie_threshold,
              "tie_policy": "unchanged official numpy.argmin: first lowest-cost row; diagnostic near ties do not alter selection",
              "current_to_first_target_distance_m": float(np.linalg.norm(ref[0, :2] - state[:2])),
              "current_to_last_target_distance_m": float(np.linalg.norm(ref[-1, :2] - state[:2])),
              "selected_horizon_xy_arc_m": float(np.linalg.norm(np.diff(ref[:, :2], axis=0), axis=1).sum()),
              "current_through_horizon_xy_arc_m": float(np.linalg.norm(np.diff(np.vstack((state[:2], ref[:, :2])), axis=0), axis=1).sum()),
              "selected_horizon_yaw_span_rad": float(ref[-1, 2] - ref[0, 2]),
              "selected_horizon_yaw_range_rad": float(np.ptp(ref[:, 2])),
              "selected_horizon_yaw_variation_rad": float(np.abs(np.diff(ref[:, 2])).sum()),
              "current_to_last_target_yaw_rad": float(ref[-1, 2] - state[2]),
              "current_through_horizon_yaw_variation_rad": float(np.abs(np.diff(np.r_[state[2], ref[:, 2]])).sum()),
              "original_goal_row_index": int(original_goal_row_index),
              "final_goal_row_in_horizon": goal_included,
              "endpoint_repetition_count": len(ids) - len(set(ids)),
              "original_fractional_coordinate_units": "row-order progress; not time or distance"}
    # Explicit aliases share the same measured quantity for worker/plot consumers.
    result.update(official_reference_unwrapped_yaw=result["official_sequentially_unwrapped_reference_yaw_rad"],
                  selected_first_target_distance_m=result["current_to_first_target_distance_m"],
                  selected_last_target_distance_m=result["current_to_last_target_distance_m"],
                  horizon_xy_arc_length_m=result["selected_horizon_xy_arc_m"],
                  horizon_yaw_span_rad=result["selected_horizon_yaw_span_rad"],
                  nearest_tie_margin=result["nearest_cost_margin"])
    return result

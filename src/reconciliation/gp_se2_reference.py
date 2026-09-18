"""Immutable source reference preparation and explicit local route checks.

The row-order clock is an evaluation convention, never a LightNav timestamp.
No artificial boundary connector is inserted into a raw or common reference.
"""
from __future__ import annotations

import numpy as np

from reconciliation.se2 import wrap_angle


def poses(value):
    a = np.asarray(value, dtype=float)
    if a.ndim != 2 or a.shape[1] != 3 or not len(a) or not np.isfinite(a).all():
        raise ValueError("nonempty finite N x 3 poses required")
    return a


def interpolate_rows(rows, row_times, query_times):
    a = poses(rows)
    t, q = np.asarray(row_times, float), np.asarray(query_times, float)
    if t.shape != (len(a),) or not np.isfinite(t).all() or not np.isfinite(q).all():
        raise ValueError("finite row/query times required")
    if len(t) > 1 and np.any(np.diff(t) <= 0):
        raise ValueError("row times must increase")
    if np.any(q < t[0] - 1e-12) or (len(t) > 1 and np.any(q > t[-1] + 1e-12)):
        raise ValueError("query outside future reference domain")
    yaw = np.r_[a[0, 2], a[0, 2] + np.cumsum(wrap_angle(np.diff(a[:, 2])))]
    return np.column_stack((np.interp(q, t, a[:, 0]), np.interp(q, t, a[:, 1]),
                            wrap_angle(np.interp(q, t, yaw))))


def prepare_reference(native_world, boundary_pose, weights, *, first_time_s=.1,
                      horizon_s=3., output_dt_s=.1):
    a = poses(native_world)
    b, w = np.asarray(boundary_pose, float), np.asarray(weights, float)
    if b.shape != (3,) or w.shape != (3,) or not np.isfinite([b, w]).all() or np.any(w <= 0):
        raise ValueError("finite boundary and positive official weights required")
    if not (0 < first_time_s <= horizon_s and output_dt_s > 0):
        raise ValueError("positive future timing required")
    count = round(horizon_s / output_dt_s)
    if not np.isclose(count * output_dt_s, horizon_s, rtol=0, atol=1e-12):
        raise ValueError("horizon must lie on output grid")
    residual = a - b
    residual[:, 2] = wrap_angle(residual[:, 2])
    distance = np.sum(residual ** 2 * w, axis=1)
    nearest = int(np.argmin(distance))
    start = min(nearest + 1, len(a) - 1)
    suffix = a[start:].copy()
    row_times = np.linspace(first_time_s, horizon_s, len(suffix)) if len(suffix) > 1 else np.array([first_time_s])
    output_times = np.arange(1, count + 1, dtype=float) * output_dt_s
    common = interpolate_rows(suffix, row_times, output_times)
    return dict(native_world=a.copy(), suffix_world=suffix, common_world=common,
                goal_world=a[-1].copy(), row_times_s=row_times, sample_times_s=output_times,
                nearest_row_index=nearest, first_future_row_index=start,
                original_row_indices=list(range(start, len(a))),
                weighted_squared_pose_distances=distance.tolist(),
                intrinsic_model_waypoint_dt_s=None,
                timing_convention="uniform original row order, future-only .1..3s; no boundary row",
                position_interpolation="linear XY", yaw_interpolation="shortest wrapped angle",
                rotation_only_rows_preserved=True)


def directed_gate_crossings(path_xy, times, gates, *, tolerance=1e-6):
    """Find strict negative-to-positive crossings, not touches or reverse motion.

    Endpoint interpolation only localizes the sampled polyline's crossing; it
    is not a claim that the counterfactual execution was a straight segment.
    The caller supplies <=10 ms execution sampling and its query uncertainty.
    """
    xy, ts = np.asarray(path_xy, float), np.asarray(times, float)
    if xy.ndim != 2 or xy.shape[1] != 2 or ts.shape != (len(xy),):
        raise ValueError("XY and time samples must align")
    if not np.isfinite(xy).all() or not np.isfinite(ts).all() or np.any(np.diff(ts) <= 0):
        raise ValueError("finite increasing execution samples required")
    results, previous = [], -np.inf
    for gate in gates:
        c, n = np.asarray(gate['center_xy'], float), np.asarray(gate['normal_xy'], float)
        if c.shape != (2,) or n.shape != (2,) or not np.isclose(np.linalg.norm(n), 1, atol=1e-8):
            raise ValueError("gate center and unit normal required")
        tangent = np.array([-n[1], n[0]])
        half_width = float(gate['half_width_m'])
        if half_width <= 0:
            raise ValueError("positive usable gate half-width required")
        signed = (xy-c) @ n
        negative_index = None
        found, reverse, touches = [], 0, 0
        for i, s in enumerate(signed):
            if s < -tolerance:
                if i and signed[i-1] > tolerance:
                    reverse += 1
                negative_index = i
            elif s > tolerance and negative_index is not None:
                a, z = negative_index, i
                middle=xy[a+1:z]
                grazing=False
                if len(middle):
                    # Never connect non-adjacent samples across a slide along
                    # the gate plane. A stationary pause on the plane is fine,
                    # but its actual crossing location must remain in the gate.
                    grazing=bool(np.max(np.linalg.norm(middle-middle[0],axis=1)) > tolerance)
                    point=middle[0]
                    at=float(ts[a+1])
                else:
                    f = -signed[a] / (signed[z] - signed[a])
                    point = xy[a] + f * (xy[z] - xy[a])
                    at = float(ts[a] + f * (ts[z]-ts[a]))
                if not grazing and abs(float((point-c) @ tangent)) < half_width - tolerance:
                    found.append(at)
                else:
                    touches += 1
                negative_index = None
        crossing = found[0] if found else None
        ordered = crossing is not None and crossing > previous + 1e-12
        results.append(dict(gate_id=gate['gate_id'], first_directed_crossing_s=crossing,
                            crossings_s=found, reverse_sample_crossings=reverse,
                            endpoint_or_outside_crossings=touches, ordered=ordered))
        if crossing is not None:
            previous = crossing
    return dict(valid=all(r['ordered'] for r in results), gates=results,
                no_required_gates=not bool(gates), status='PASS' if all(r['ordered'] for r in results) else 'FAIL')


def geometric_group_flags(metrics, shape, selection):
    """Only descriptive saved-source geometry; no optimized or new-rollout data."""
    window = metrics['fresh_projection']['window_0_10m']
    reliable = bool(metrics['incoming_execution']['available'] and window['available']
                    and metrics['incoming_execution']['chord_m'] >= selection['minimum_incoming_chord_m']
                    and window['chord_m'] >= selection['minimum_fresh_window_chord_m'])
    angle = metrics['abs_e_dir_window_deg']
    small = bool(reliable and angle is not None
                 and metrics['e_perp_m'] <= selection['small_position_gap_m']
                 and angle <= selection['small_window_direction_deg']
                 and metrics['abs_e_yaw_deg'] <= selection['small_pose_yaw_deg'])
    straight = bool(shape['arc_m'] >= selection['raw_straight_minimum_arc_m']
                    and shape['chord_over_arc'] >= selection['raw_straight_minimum_chord_over_arc']
                    and shape['yaw_travel_from_observation_deg'] <= selection['raw_straight_angle_deg']
                    and shape['max_forward_direction_deviation_deg'] is not None
                    and shape['max_forward_direction_deviation_deg'] <= selection['raw_straight_angle_deg'])
    return dict(reliable_direction=reliable, A_SMALL_STRAIGHT=small and straight,
                B_SMALL_TURN=small and shape['yaw_travel_from_observation_deg'] > selection['turn_raw_accumulated_yaw_deg'],
                C_LARGE_MISMATCH=bool(metrics['e_perp_m'] >= selection['large_position_gap_m']
                    or (reliable and angle is not None and angle >= selection['large_window_direction_deg'])))


def choose_cases(eligible, selection):
    """Frozen deterministic greedy selection, scarce groups first, no substitution."""
    used, episodes, pairs, selected = set(), set(), set(), []
    priority = selection['priority_events']
    for group in selection['group_order']:
        pool = [r for r in eligible if r['groups'].get(group, False) and r['case_id'] not in used]
        for _ in range(selection['target_per_group']):
            if not pool:
                break
            key = lambda r: (r['episode_id'] in episodes, r['ordered_raw_pair'] in pairs,
                priority.index(r['case_id']) if r['case_id'] in priority else len(priority), r['case_id'])
            row = min(pool, key=key)
            selected.append({**row, 'selected_group': group})
            used.add(row['case_id']); episodes.add(row['episode_id']); pairs.add(row['ordered_raw_pair'])
            pool.remove(row)
    return selected

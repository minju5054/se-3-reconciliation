"""Descriptive geometry of recorded online execution; never a path correction.

Input states are actual recorded pre-command states in one fixed world frame.
All durations subtract timestamps from the same explicitly named clock domain.
Synthetic callers are tests only; this module does not create execution history.
"""
from __future__ import annotations

from collections import Counter, defaultdict
import hashlib

import numpy as np

from reconciliation.robotless_controlled_staleness import point_to_polyline
from reconciliation.robotless_old_conditioned_handoff import window_tangent
from reconciliation.robotless_single_chunk import validate_waypoints
from reconciliation.se2 import wrap_angle

VALID_STATUSES = ("VALID_HANDOFF_MOVING", "VALID_HANDOFF_STATIONARY")
FRAME_CONVENTION = {
    "frame": "fixed Isaac world", "translation_units": "m", "yaw_units": "rad",
    "axes": "+Z up; yaw counterclockwise about +Z",
    "local_axes": "+x forward, +y left, +z up",
    "raw_anchor": "each cumulative local path is composed with its own RGB capture agent pose",
    "intrinsic_waypoint_dt_s": None,
    "correction": "none; no translation, rotation, row deletion or synthetic path connector",
}
METRIC_COLUMNS = (
    "client_rtt_s", "server_reported_latency_s", "observation_to_ready_sim_s", "observation_to_switch_sim_s",
    "ready_to_switch_sim_s", "inference_rtf", "inference_translation_m",
    "inference_net_translation_m", "inference_yaw_change_rad", "inference_yaw_travel_rad",
    "inference_state_updates", "post_switch_execution_duration_s", "actual_history_count",
    "model_input_unique_frame_count", "history_capture_sim_span_s", "history_capture_host_span_s",
    "d_first_row_m", "e_perp_m", "abs_e_dir_local_deg", "abs_e_dir_window_deg",
    "abs_e_yaw_deg", "delta_v_mps", "delta_omega_radps",
)


def _pose(pose):
    result = np.asarray(pose, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError("pose must have three finite world coordinates")
    return result


def projection_at_boundary(boundary, path):
    """Closest raw segment, including singleton/repeated/rotation-only outputs.

    Exact distance ties select the first segment. Yaw interpolation follows the
    shortest wrapped angle, separately from XY tangent. No connector is added.
    """
    boundary = _pose(boundary)
    path = validate_waypoints(path)
    p = point_to_polyline(boundary[:2], path[:, :2])
    lengths = np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)
    total = float(lengths.sum())
    j, alpha = p["segment_index"], p["segment_fraction"]
    segment_length = 0. if j is None else float(lengths[j])
    tangent = None if segment_length <= 0 else float(np.arctan2(*(path[j+1, :2]-path[j, :2])[::-1]))
    yaw_q = float(wrap_angle(path[0, 2])) if j is None else float(wrap_angle(path[j, 2] + alpha*wrap_angle(path[j+1, 2]-path[j, 2])))
    progress = 0. if j is None else float(lengths[:j].sum()+alpha*lengths[j])
    window = window_tangent(path, progress, .10) if total > 0 else {
        "available": False, "reason": "raw path has zero XY arc length", "phi_rad": None,
        "s_minus_m": None, "s_plus_m": None, "span_m": 0., "chord_m": 0.,
        "minus_xy": None, "plus_xy": None,
    }
    yaw_error = float(wrap_angle(yaw_q-boundary[2]))
    return {
        "B_world": boundary.tolist(), "Q_xy_world_m": p["nearest_xy_world_m"],
        "d_first_row_m": float(np.linalg.norm(boundary[:2]-path[0, :2])),
        "e_perp_m": p["distance_m"], "segment_index": j, "alpha": alpha,
        "segment_length_m": segment_length, "total_arc_length_m": total,
        "s_Q_m": progress, "normalized_progress": progress/total if total > 0 else None,
        "projection_location": "singleton" if j is None else "degenerate_point" if segment_length == 0 else "interior" if 0 < alpha < 1 else "start_endpoint" if alpha == 0 else "end_endpoint",
        "phi_local_rad": tangent, "local_tangent_available": tangent is not None,
        "local_tangent_unavailable_reason": None if tangent is not None else "winning raw segment has no XY direction",
        "window_0_10m": window, "theta_Q_rad": yaw_q,
        "e_yaw_rad": yaw_error, "abs_e_yaw_deg": float(np.degrees(abs(yaw_error))),
    }


def execution_arrays(poses, sim_times, host_times=None):
    poses = validate_waypoints(poses)
    times = np.asarray(sim_times, dtype=float)
    if times.shape != (len(poses),) or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError("execution simulation times must increase strictly and match states")
    host = None if host_times is None else np.asarray(host_times, dtype=float)
    if host is not None and (host.shape != times.shape or not np.isfinite(host).all() or np.any(np.diff(host) < 0)):
        raise ValueError("host monotonic times must be nondecreasing and match states")
    return poses, times, host


def _range(start, end, count):
    if not isinstance(start, (int, np.integer)) or not isinstance(end, (int, np.integer)) or not 0 <= start <= end < count:
        raise ValueError("execution range must be valid inclusive integer row indices")
    return int(start), int(end)


def motion_in_range(poses, sim_times, start, end):
    poses, times, _ = execution_arrays(poses, sim_times)
    start, end = _range(start, end, len(poses))
    path = poses[start:end+1]
    differences = np.diff(path, axis=0)
    yaw_steps = wrap_angle(differences[:, 2])
    return {
        "stream_rows_inclusive": [start, end], "start_sim_s": float(times[start]),
        "end_sim_s": float(times[end]), "duration_sim_s": float(times[end]-times[start]),
        "state_updates": end-start,
        "translation_m": float(np.linalg.norm(differences[:, :2], axis=1).sum()),
        "net_translation_m": float(np.linalg.norm(path[-1, :2]-path[0, :2])),
        "yaw_change_rad": float(yaw_steps.sum()), "yaw_travel_rad": float(np.abs(yaw_steps).sum()),
    }


def incoming_execution_direction(poses, sim_times, switch_index, *, lookback_s=.25, resolution_m=1e-9):
    """Chord of actual E(t) over the last available <= lookback seconds.

    B is the pre-first-FRESH-command state at switch_index; the last included
    displacement was produced before that command. Rotation never substitutes
    body yaw for unavailable translational direction.
    """
    poses, times, _ = execution_arrays(poses, sim_times)
    _, switch_index = _range(switch_index, switch_index, len(poses))
    if not np.isfinite(lookback_s) or lookback_s <= 0 or not np.isfinite(resolution_m) or resolution_m <= 0:
        raise ValueError("incoming window and resolution must be finite and positive")
    start = int(np.searchsorted(times, times[switch_index]-lookback_s, side="left"))
    start = min(start, switch_index)
    delta = poses[switch_index, :2]-poses[start, :2]
    chord = float(np.linalg.norm(delta))
    reason = "no prior execution sample in incoming window" if start == switch_index else "actual incoming XY chord at or below numerical resolution" if chord <= resolution_m else None
    return {
        "source": "actual pre-switch execution E(t)", "method": "endpoint XY chord",
        "requested_lookback_s": float(lookback_s), "resolution_m": float(resolution_m),
        "stream_rows_inclusive": [start, switch_index], "start_sim_s": float(times[start]),
        "end_sim_s": float(times[switch_index]), "actual_span_sim_s": float(times[switch_index]-times[start]),
        "start_world_pose": poses[start].tolist(), "end_world_pose": poses[switch_index].tolist(),
        "chord_m": chord, "available": reason is None, "unavailable_reason": reason,
        "phi_rad": None if reason else float(np.arctan2(delta[1], delta[0])),
    }


def compute_event_metrics(old_world, fresh_world, poses, sim_times, *,
                          request_index, ready_index, switch_index,
                          observation_sim_s, request_host_monotonic_s,
                          ready_host_monotonic_s, post_end_index=None,
                          previous_command=None, first_fresh_command=None,
                          incoming_lookback_s=.25, host_times=None,
                          actual_history_count=0):
    """Use recorded row indices/timestamps; do not infer B from a reference.

    request_index is the first state at/after actual request transmission;
    ready_index is the first simulation loop that observed the complete result.
    post_end_index is the pre-command state of the next activation, or final
    state; it can include the boundary of, but no displacement caused by, the
    next reference. Caller establishes that lineage from applied commands.
    """
    poses, times, host = execution_arrays(poses, sim_times, host_times)
    _range(request_index, ready_index, len(poses))
    _range(ready_index, switch_index, len(poses))
    post_end_index = switch_index if post_end_index is None else post_end_index
    _range(switch_index, post_end_index, len(poses))
    obs = float(observation_sim_s)
    request_host, ready_host = float(request_host_monotonic_s), float(ready_host_monotonic_s)
    if not all(np.isfinite(v) for v in (obs, request_host, ready_host)) or obs > times[request_index]+1e-12 or ready_host < request_host:
        raise ValueError("invalid observation/request/readiness clock ordering")
    boundary = poses[switch_index]
    fresh = projection_at_boundary(boundary, fresh_world)
    old = projection_at_boundary(boundary, old_world)
    incoming = incoming_execution_direction(poses, times, switch_index, lookback_s=incoming_lookback_s)
    seen_execution = motion_in_range(poses, times, request_index, ready_index)
    if host is not None:
        inflight_start = max(request_index, int(np.searchsorted(host, request_host, side="left")))
        inflight_end = min(ready_index, int(np.searchsorted(host, ready_host, side="right"))-1)
    else:
        inflight_start, inflight_end = request_index, ready_index
    if inflight_start <= inflight_end:
        inference = motion_in_range(poses, times, inflight_start, inflight_end)
        inference["available"] = True
        inference["unavailable_reason"] = None
    else:
        inference = {"stream_rows_inclusive": None, "start_sim_s": None, "end_sim_s": None,
            "duration_sim_s": 0., "state_updates": 0, "translation_m": 0., "net_translation_m": 0.,
            "yaw_change_rad": 0., "yaw_travel_rad": 0., "available": False,
            "unavailable_reason": "no execution state sample lies inside the exact client request-to-receipt host interval"}
    post = motion_in_range(poses, times, switch_index, post_end_index)
    local = None if incoming["phi_rad"] is None or fresh["phi_local_rad"] is None else float(wrap_angle(fresh["phi_local_rad"]-incoming["phi_rad"]))
    window = None if incoming["phi_rad"] is None or fresh["window_0_10m"]["phi_rad"] is None else float(wrap_angle(fresh["window_0_10m"]["phi_rad"]-incoming["phi_rad"]))
    # RTF below uses actual matched execution sample clock domains, not
    # simulation age divided by remote-server or UTC timestamps.
    elapsed_host = None if host is None or inflight_start > inflight_end else float(host[inflight_end]-host[inflight_start])
    seen_elapsed_host = None if host is None else float(host[ready_index]-host[request_index])
    command_delta = [None, None]
    if previous_command is not None and first_fresh_command is not None:
        before, after = np.asarray(previous_command, dtype=float), np.asarray(first_fresh_command, dtype=float)
        if before.shape != (2,) or after.shape != (2,) or not np.isfinite([before, after]).all():
            raise ValueError("commands must be finite [v_mps, omega_radps]")
        command_delta = (after-before).tolist()
    return {
        "frames": FRAME_CONVENTION, "boundary_source": "actual pre-first-FRESH-command execution state",
        "B_world": boundary.tolist(), "P_world": poses[switch_index-1].tolist() if switch_index else None,
        "P_stream_row": switch_index-1 if switch_index else None, "B_stream_row": switch_index,
        "fresh_projection": fresh, "old_reference_projection_diagnostic": old,
        "incoming_execution": incoming, "inference_execution": inference, "post_switch_execution": post,
        "request_to_ready_seen_execution": seen_execution,
        "inference_interval_convention": "only actual execution state samples whose client host timestamps are between wire request transmission and complete response receipt (inclusive); no interpolated endpoint states",
        "inference_host_range_exact": host is not None,
        "request_to_ready_seen_rtf": None if seen_elapsed_host is None or seen_elapsed_host <= 0 else seen_execution["duration_sim_s"]/seen_elapsed_host,
        "ready_receipt_to_ready_state_sample_host_s": None if host is None else float(host[ready_index]-ready_host),
        "client_rtt_s": ready_host-request_host,
        "observation_to_ready_sim_s": float(times[ready_index]-obs),
        "observation_to_switch_sim_s": float(times[switch_index]-obs),
        "ready_to_switch_sim_s": float(times[switch_index]-times[ready_index]),
        "inference_rtf": None if elapsed_host is None or elapsed_host <= 0 else inference["duration_sim_s"]/elapsed_host,
        "inference_rtf_convention": "matched execution samples inside client request-to-receipt interval: simulation elapsed / host-monotonic elapsed",
        "inference_translation_m": inference["translation_m"], "inference_net_translation_m": inference["net_translation_m"],
        "inference_yaw_change_rad": inference["yaw_change_rad"], "inference_yaw_travel_rad": inference["yaw_travel_rad"],
        "inference_state_updates": inference["state_updates"],
        "old_control_continuation_sim_s": float(times[switch_index]-times[request_index]),
        "post_switch_execution_duration_s": post["duration_sim_s"],
        "post_switch_execution_available": post["state_updates"] > 0,
        "post_switch_unavailable_reason": None if post["state_updates"] > 0 else "no recorded integration after first FRESH application in this reference lifetime",
        "actual_history_count": int(actual_history_count),
        "d_first_row_m": fresh["d_first_row_m"], "e_perp_m": fresh["e_perp_m"],
        "e_dir_local_rad": local, "abs_e_dir_local_deg": None if local is None else float(np.degrees(abs(local))),
        "e_dir_window_rad": window, "abs_e_dir_window_deg": None if window is None else float(np.degrees(abs(window))),
        "e_yaw_rad": fresh["e_yaw_rad"], "abs_e_yaw_deg": fresh["abs_e_yaw_deg"],
        "delta_v_mps": command_delta[0], "delta_omega_radps": command_delta[1],
        "command_jump_interpretation": "fixed official MPC velocity and acceleration limits may suppress jumps; no graph-needed inference",
        "no_robot_dynamics": True, "no_reconciliation": True,
    }


def ordered_pair_sha256(old_hash, fresh_hash):
    for digest in (old_hash, fresh_hash):
        if not isinstance(digest, str) or len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
            raise ValueError("raw source hashes must be lowercase SHA-256 hex")
    return hashlib.sha256((old_hash+":"+fresh_hash).encode("ascii")).hexdigest()


def distribution(values):
    values = list(values)
    finite = [float(v) for v in values if v is not None and np.isfinite(v)]
    result = {"available_count": len(finite), "unavailable_count": len(values)-len(finite)}
    result.update({key: None for key in ("min", "median", "p75", "p90", "max", "mean")})
    if finite:
        result.update(dict(zip(("min", "median", "p75", "p90", "max"), map(float, np.percentile(finite, [0, 50, 75, 90, 100])))))
        result["mean"] = float(np.mean(finite))
    return result


def summarize_events(rows, *, metric_columns=METRIC_COLUMNS):
    """Descriptive event distributions, with episodes and raw pairs separately.

    Episode summary gives equal weight to each within-episode median; duplicate
    summary gives equal weight to each within-pair median. Neither treats
    successive events as independent samples or supplies confidence intervals.
    """
    rows = list(rows)
    identities = [(r["episode_id"], r["event_id"]) for r in rows]
    if len(set(identities)) != len(identities):
        raise ValueError("duplicate event identity")
    valid = [r for r in rows if r["status"] in VALID_STATUSES]
    pairs = defaultdict(list)
    episodes = defaultdict(list)
    for row in valid:
        pair = ordered_pair_sha256(row["old_raw_sha256"], row["fresh_raw_sha256"])
        pairs[pair].append(row)
        episodes[row["episode_id"]].append(row)
    def group_medians(groups):
        return [{"group_id": key, "event_count": len(members),
                 **{metric: distribution(m.get(metric) for m in members)["median"] for metric in metric_columns}}
                for key, members in sorted(groups.items())]
    by_episode, by_pair = group_medians(episodes), group_medians(pairs)
    def stats(items):
        return {metric: distribution(r.get(metric) for r in items) for metric in metric_columns}
    multiplicities = [{"pair_sha256": key, "old_raw_sha256": members[0]["old_raw_sha256"],
                       "fresh_raw_sha256": members[0]["fresh_raw_sha256"], "event_count": len(members),
                       "episode_ids": sorted({m["episode_id"] for m in members})}
                      for key, members in sorted(pairs.items(), key=lambda item: (-len(item[1]), item[0]))]
    return {
        "attempted_count": len(rows), "valid_count": len(valid), "invalid_count": len(rows)-len(valid),
        "status_counts": dict(sorted(Counter(r["status"] for r in rows).items())),
        "history_full_valid_count": sum(bool(r.get("history_full", False)) for r in valid),
        "history_valid_count": sum(bool(r.get("history_valid", False)) for r in valid),
        "timing_valid_count": sum(bool(r.get("timing_valid", False)) for r in valid),
        "overlap_observed_count": sum(bool(r.get("overlap_observed", False)) for r in valid),
        "post_switch_execution_available_count": sum(bool(r.get("post_switch_execution_available", False)) for r in valid),
        "diversity": {"population": "activated valid handoffs; raw ordered array hashes",
            "unique_OLD_count": len({r["old_raw_sha256"] for r in valid}),
            "unique_FRESH_count": len({r["fresh_raw_sha256"] for r in valid}),
            "unique_ordered_pair_count": len(pairs),
            "dominant_pair_fraction": max(map(len, pairs.values()), default=0)/len(valid) if valid else None,
            "ordered_pair_multiplicities": multiplicities},
        "statistics": {"event_descriptive": stats(valid), "episode_weighted": stats(by_episode),
            "duplicate_aware": stats(by_pair), "episode_medians": by_episode, "pair_medians": by_pair,
            "method": "linear percentiles; episode-weighted = equal-weight within-episode medians; duplicate-aware = equal-weight within-raw-ordered-pair medians",
            "dependence": "successive events are dependent; no IID confidence intervals or occurrence-frequency claims",
            "unavailable": "null/nonfinite metric values excluded individually; counts retained"},
    }


def metrics_from_records(context, execution_rows, command_rows, old_world, fresh_world):
    """Canonical stream adapter; B/P and command lineage must match source rows."""
    rows = list(execution_rows)
    if not rows:
        raise ValueError("actual execution stream is required")
    ids = [int(r["state_id"]) for r in rows]
    if len(set(ids)) != len(ids):
        raise ValueError("execution state IDs must be unique")
    lookup = {state_id: i for i, state_id in enumerate(ids)}
    poses = np.array([[float(r[k]) for k in ("x", "y", "yaw")] for r in rows])
    times = np.array([float(r["sim_time_s"]) for r in rows])
    host = np.array([float(r["host_monotonic_s"]) for r in rows])
    execution_arrays(poses, times, host)
    switch = lookup[int(context["switch_state_id"])]
    ready = lookup[int(context["ready_state_id"])]
    obs = lookup[int(context["obs_state_id"])]
    request_host = float(context["t_request"]["host_monotonic_s"])
    if "request_state_id" in context:
        request = lookup[int(context["request_state_id"])]
    else:
        request = int(np.searchsorted(host, request_host, side="left"))
        if request >= len(rows):
            raise ValueError("request transmission has no following execution state")
    end_id = int(context.get("post_switch_end_state_id", context["switch_state_id"]))
    post_end = lookup[end_id]
    for key, index in (("R_obs", obs), ("R_ready", ready), ("B", switch)):
        if not np.allclose(context[key], poses[index], rtol=0, atol=1e-10):
            raise ValueError(f"{key} differs from actual execution stream")
    if switch:
        if int(context["previous_state_id"]) != ids[switch-1] or not np.allclose(context["P"], poses[switch-1], rtol=0, atol=1e-10):
            raise ValueError("P is not the actual preceding state sample")
    applied = {int(c["application_state_id"]): c for c in command_rows}
    if len(applied) != len(command_rows):
        raise ValueError("multiple applied commands at one execution state")
    for key, index in (("t_obs", obs), ("t_ready_seen_sim", ready), ("t_switch", switch)):
        timestamp = context.get(key) or {}
        if "sim_time_s" not in timestamp or not np.isclose(float(timestamp["sim_time_s"]), times[index], rtol=0, atol=1e-10):
            raise ValueError(f"{key} simulation timestamp differs from execution state")
    first = applied.get(ids[switch])
    if first is None or str(first["reference_version"]) != str(context["fresh_reference_version"]) or first["chunk_id"] != context["fresh_chunk_id"]:
        raise ValueError("B does not precede first actual FRESH command")
    for index in range(request, switch):
        command = applied.get(ids[index])
        if command is None or str(command["reference_version"]) != str(context["old_reference_version"]) or command["chunk_id"] != context["old_chunk_id"]:
            raise ValueError("inference/ready-to-switch execution does not retain OLD lineage")
    for index in range(switch, post_end):
        command = applied.get(ids[index])
        if command is None or str(command["reference_version"]) != str(context["fresh_reference_version"]) or command["chunk_id"] != context["fresh_chunk_id"]:
            raise ValueError("post-switch range includes another active reference")
    before = applied.get(ids[switch-1]) if switch else None
    def command_array(command):
        return None if command is None else [float(command["v_mps"]), float(command["omega_radps"])]
    result = compute_event_metrics(old_world, fresh_world, poses, times,
        request_index=request, ready_index=ready, switch_index=switch,
        observation_sim_s=float(context["t_obs"]["sim_time_s"]),
        request_host_monotonic_s=request_host,
        ready_host_monotonic_s=float(context["t_ready_host"]["host_monotonic_s"]),
        post_end_index=post_end, previous_command=command_array(before),
        first_fresh_command=command_array(first), host_times=host,
        actual_history_count=context["actual_history_count"])
    inference_rows = result["inference_execution"]["stream_rows_inclusive"]
    result.update(episode_id=context["episode_id"], event_id=context["event_id"],
        status=context["status"], old_chunk_id=context["old_chunk_id"], fresh_chunk_id=context["fresh_chunk_id"],
        old_raw_sha256=context["old_raw_local_ref"]["sha256"], fresh_raw_sha256=context["fresh_raw_local_ref"]["sha256"],
        history_full=bool(context["history_full"]),
        timing_valid=bool(context["timing_flags"].get("timing_valid", False)),
        history_valid=bool(context["timing_flags"].get("history_valid", False)),
        overlap_observed=result["inference_state_updates"] > 0,
        request_state_id=ids[request], ready_state_id=ids[ready], switch_state_id=ids[switch],
        post_switch_end_state_id=ids[post_end],
        server_timing=context.get("server_timing"),
        server_reported_latency_s=None if (context.get("server_timing") or {}).get("latency_ms") is None else float(context["server_timing"]["latency_ms"])/1000.,
        server_stage_timings_ms=(context.get("server_timing") or {}).get("timings_ms"),
        ready_receipt_to_seen_host_s=None if "host_monotonic_s" not in context["t_ready_seen_sim"] else float(context["t_ready_seen_sim"]["host_monotonic_s"])-float(context["t_ready_host"]["host_monotonic_s"]),
        server_timing_provenance="server-reported response fields preserved verbatim; not subtracted from client clocks",
        source_timing_flags=context.get("timing_flags", {}),
        execution_range_state_ids={"inference": None if inference_rows is None else [ids[inference_rows[0]], ids[inference_rows[1]]], "request_to_ready_seen": [ids[request], ids[ready]], "incoming": [ids[result["incoming_execution"]["stream_rows_inclusive"][0]], ids[switch]], "post_switch": [ids[switch], ids[post_end]]})
    return result


def interval_timing_summary(context, metrics, execution_rows, command_rows, loop_rows,
                            controller_rows, capture_rows, *, pacing_config=None):
    """Post-collection timing coverage; source event status is never filtered.

    Completion-based loop stalls may start before a window boundary, so their
    scope is stated separately from wholly enclosed execution sample pairs.
    Submit attempts are observable as replies; accepted submits carry an exact
    worker host timestamp. These two counts are deliberately not conflated.
    """
    pacing_config = pacing_config or {}
    def is_true(value):
        return value is True or (isinstance(value, str) and value.lower() == "true")
    def inside(value, start, end):
        return value is not None and start <= float(value) <= end
    def summarize_window(start, end, simulation_range):
        state_rows = [r for r in execution_rows if inside(r.get("host_monotonic_s"), start, end)]
        commands = [r for r in command_rows if inside(r.get("host_monotonic_s"), start, end)]
        loops = [r for r in loop_rows if inside(r.get("host_monotonic_s"), start, end)]
        captures = [r for r in capture_rows if inside(float(r["capture_monotonic_ns"])/1e9, start, end)]
        captures.sort(key=lambda r: int(r["capture_monotonic_ns"]))
        capture_host = [float(r["capture_monotonic_ns"])/1e9 for r in captures]
        capture_sim = [float(r["capture_sim_time_s"]) for r in captures]
        submissions = [r for r in controller_rows if r.get("type") == "reply" and r.get("op") == "submit"
                       and inside((r.get("seen_in_isaac") or {}).get("host_monotonic_s"), start, end)]
        accepted_submissions = [r for r in controller_rows if r.get("type") == "reply" and r.get("status") == "submitted"
                                and inside(r.get("submit_host_monotonic_s"), start, end)]
        completed = [r for r in controller_rows if r.get("type") == "solve_result"
                     and inside(r.get("solve_end_host_monotonic_s"), start, end)]
        durations = [float(r["loop_interval_host_s"]) for r in loops if r.get("loop_interval_host_s") is not None]
        simulation_count = 0 if simulation_range is None else simulation_range[1]-simulation_range[0]
        return {
            "host_monotonic_range_s": [start, end], "host_duration_s": end-start,
            "simulation_stream_rows_inclusive": simulation_range,
            "simulation_steps_wholly_enclosed": simulation_count,
            "execution_state_sample_count": len(state_rows),
            "actual_applied_command_count": len(commands),
            "new_solve_command_application_count": sum(not is_true(r.get("held", True)) and r.get("solve_id") not in (None, "") for r in commands),
            "distinct_solve_ids_applied_count": len({str(r["solve_id"]) for r in commands if r.get("solve_id") not in (None, "")}),
            "control_submit_attempt_replies_seen_count": len(submissions),
            "control_busy_submit_replies_seen_count": sum(r.get("status") == "busy" for r in submissions),
            "accepted_control_solve_submissions_count": len(accepted_submissions),
            "control_solve_completions_count": len(completed),
            "stale_control_result_count": sum(r.get("status") == "stale_rejected" for r in completed),
            "capture_count": len(captures), "capture_frame_ids": [r["frame_id"] for r in captures],
            "capture_intervals_host_s": distribution(np.diff(capture_host)),
            "capture_intervals_sim_s": distribution(np.diff(capture_sim)),
            "loop_completion_count": len(loops),
            "missed_deadline_count": sum(is_true(r.get("deadline_missed")) for r in loops),
            "maximum_loop_stall_s": max(durations) if durations else None,
            "maximum_deadline_lateness_s": max((float(r.get("lateness_s", 0)) for r in loops), default=None),
        }
    request = float(context["t_request"]["host_monotonic_s"])
    ready = float(context["t_ready_host"]["host_monotonic_s"])
    switch = float(context["t_switch"].get("host_monotonic_s", execution_rows[metrics["B_stream_row"]]["host_monotonic_s"]))
    inference = summarize_window(request, ready, metrics["inference_execution"]["stream_rows_inclusive"])
    seen = metrics["request_to_ready_seen_execution"]["stream_rows_inclusive"]
    request_to_switch = summarize_window(request, switch, [seen[0], metrics["B_stream_row"]])
    thresholds_available = all(key in pacing_config for key in ("rtf_valid_min", "rtf_valid_max", "max_loop_stall_valid_s"))
    rtf = metrics["inference_rtf"]
    stall = inference["maximum_loop_stall_s"]
    pacing_valid = None if not thresholds_available or stall is None else bool(rtf is not None and pacing_config["rtf_valid_min"] <= rtf <= pacing_config["rtf_valid_max"] and stall <= pacing_config["max_loop_stall_valid_s"])
    warnings = []
    if metrics["inference_state_updates"] == 0:
        warnings.append("No wholly enclosed request-to-receipt state transition; online overlap is not established by the ready-seen interval alone.")
    if pacing_valid is False:
        warnings.append("Exact client-inflight samples do not satisfy all frozen real-time pacing limits; recorded causal execution is preserved.")
    if inference["capture_count"] < 2:
        warnings.append("Fewer than two captures inside this inference interval; within-interval capture cadence is unavailable.")
    if inference["missed_deadline_count"]:
        warnings.append("One or more loop completions missed absolute pacing deadlines in this interval.")
    return {
        "inference_client_host_interval": inference, "request_to_actual_switch_interval": request_to_switch,
        "causal_execution_overlap_observed": metrics["inference_state_updates"] > 0,
        "actual_post_switch_execution_available": metrics["post_switch_execution_available"],
        "real_time_pacing_valid": pacing_valid,
        "frozen_pacing_limits": {key: pacing_config.get(key) for key in ("rtf_valid_min", "rtf_valid_max", "max_loop_stall_valid_s")},
        "warnings": warnings,
        "count_conventions": {
            "execution": "simulation_steps_wholly_enclosed counts only adjacent saved states inside host interval; command applications are independently counted by actual application timestamp",
            "loop": "loops whose completion timestamp is in interval; a stall may straddle its start boundary",
            "control": "accepted submissions/completions use worker host timestamps; submit-attempt reply counts use first Isaac-seen host timestamp because busy replies lack a worker submit timestamp",
            "capture": "actual capture timestamps, without filling missing frames or estimating capture times from configured fps",
            "simulation": "request-to-switch simulation range starts first recorded state at/after request; request host event is never converted into an invented simulator timestamp",
        },
    }

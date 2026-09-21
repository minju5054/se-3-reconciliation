"""Read-only JOIN-SOURCE-02 qualification; no model/controller/optimizer runtime.

Inputs are saved source records. A paired branch contains ``session_id``,
``instruction``, ``camera_config``, ``model_config``, ``history`` (preceding
capture records), and ``final_frame``. Capture records contain frame_id,
capture_sim_time_s, capture_monotonic_ns, pose_world, sha256, camera_id and
rendered_state_id. RGB equality alone never proves duplicate acquisition.
All geometry is Isaac Z-up world XY metres, yaw CCW radians. LightNav rows
have no intrinsic clock. Every returned path is the complete original path.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import math

import numpy as np

from .robotless_online import integrate_unicycle
from .robotless_projection_handoff import projection_geometry
from .se2 import local_trajectory_to_world, wrap_angle


BRANCHES = ("A_OFF", "B_ON", "A_SHAM")
HISTORY_COUNTS = (16, 32, 64)
REQUIRED_SCREENING_CHECKS = (
    "source_integrity", "official_contract", "paired_inputs", "wire_jpeg_parity",
    "visibility", "geometry_activation",
)


def _array(value, width, name, *, empty=False):
    a = np.asarray(value, dtype=np.float64)
    if empty and a.size == 0:
        return np.empty((0, width), dtype=np.float64)
    if a.ndim != 2 or a.shape[1] != width or (not empty and not len(a)) or not np.isfinite(a).all():
        raise ValueError(f"{name} must be finite N by {width}")
    return a


def _sha(value):
    return hashlib.sha256(value).hexdigest()


def declared_conditions(locations, placement_rules, history_counts=HISTORY_COUNTS):
    """Freeze location -> placement -> history order, at most twelve pairs."""
    locations, placement_rules = list(locations), list(placement_rules)
    if not 1 <= len(locations) <= 2 or not 1 <= len(placement_rules) <= 2:
        raise ValueError("at most two locations and two placement rules")
    if len(set(locations)) != len(locations) or len(set(placement_rules)) != len(placement_rules):
        raise ValueError("declaration IDs must be unique")
    if tuple(history_counts) != HISTORY_COUNTS:
        raise ValueError("delivered-history counts and order are fixed at 16,32,64")
    return [dict(condition_id=f"{loc}__{placement}__H{count:02d}", location_id=loc,
                 placement_rule_id=placement, delivered_frame_count=count)
            for loc in locations for placement in placement_rules for count in history_counts]


def validate_development_order(declaration, ledger):
    """No retry/replacement, no extra condition, stop at first complete qualifier.

    Ledger rows contain condition_id, label, and completed_branches. A declared
    HISTORY_UNAVAILABLE/technical failure remains a row, with no fabricated calls.
    """
    declared = [x["condition_id"] for x in declaration]
    actual = [x["condition_id"] for x in ledger]
    if len(declared) > 12 or len(set(declared)) != len(declared):
        raise ValueError("invalid frozen declaration")
    if actual != declared[:len(actual)] or len(actual) > len(declared):
        raise ValueError("undeclared, reordered, retried, or replaced condition")
    selected = None
    for index, row in enumerate(ledger):
        branches = tuple(row.get("completed_branches", ()))
        if branches != BRANCHES[:len(branches)] or len(branches) > 3:
            raise ValueError("paired branch order must be A_OFF/B_ON/A_SHAM")
        if row["label"] == "SAFE_BYPASS_CANDIDATE":
            if branches != BRANCHES:
                raise ValueError("qualifier requires complete A/B/A' condition")
            if index != len(ledger)-1:
                raise ValueError("development continued after first qualifier")
            selected = row["condition_id"]
    counts_known = all("terminal_predictions" in x.get("call_counts", {}) for x in ledger)
    return dict(valid=True, selected_condition_id=selected, condition_count=len(ledger),
                completed_branch_attempt_count=sum(len(x.get("completed_branches", ())) for x in ledger),
                terminal_prediction_count=sum(x["call_counts"]["terminal_predictions"] for x in ledger)
                if counts_known else None)


def _frame(record):
    for field in ("frame_id", "sha256", "camera_id", "rendered_state_id"):
        if record.get(field) is None:
            raise ValueError(f"missing frame {field}")
    if not isinstance(record["sha256"], str) or len(record["sha256"]) != 64:
        raise ValueError("invalid frame hash")
    try:
        int(record["sha256"], 16)
    except ValueError as exc:
        raise ValueError("invalid frame hash") from exc
    if type(record.get("capture_monotonic_ns")) is not int or record["capture_monotonic_ns"] < 0:
        raise ValueError("integer capture monotonic nanoseconds required")
    if not np.isfinite(record["capture_sim_time_s"]) or record["capture_sim_time_s"] < 0:
        raise ValueError("invalid frame simulation time")
    if np.asarray(record["pose_world"]).shape != (3,) or not np.isfinite(record["pose_world"]).all():
        raise ValueError("invalid frame observation pose")
    return deepcopy(record)


def validate_paired_inputs(branches):
    """Validate recorded paired contracts; hashes must first be file-verified.

    Distinct genuine captures may have identical JPEG bytes. Duplicate frame
    identity, backwards host/sim chronology and branch-specific history fail.
    Final off/sham bytes must match for a literal-input sham; ON may differ.
    Render steps can differ but simulated pose/time cannot advance between pairs.
    """
    if tuple(branches) != BRANCHES:
        raise ValueError("branches must be ordered A_OFF, B_ON, A_SHAM")
    sessions = [branches[x]["session_id"] for x in BRANCHES]
    if any(not x for x in sessions) or len(set(sessions)) != 3:
        raise ValueError("three independent sessions required")
    baseline = branches[BRANCHES[0]]
    base_final = _frame(baseline["final_frame"])
    checked = {}
    for name in BRANCHES:
        branch = branches[name]
        for key in ("instruction", "camera_config", "model_config"):
            if branch[key] != baseline[key]:
                raise ValueError(f"paired {key} mismatch")
        frames = [_frame(x) for x in branch["history"]]
        final = _frame(branch["final_frame"])
        total = frames+[final]
        if len(total) not in HISTORY_COUNTS:
            raise ValueError("terminal delivered count must be 16,32,64")
        if len({x["frame_id"] for x in total}) != len(total):
            raise ValueError("duplicate capture identity in history")
        if any(b["capture_monotonic_ns"] <= a["capture_monotonic_ns"] or
               b["capture_sim_time_s"] < a["capture_sim_time_s"] for a, b in zip(total[:-1], total[1:])):
            raise ValueError("future/reversed/nonchronological capture history")
        if frames != baseline["history"]:
            raise ValueError("paired preceding history metadata/bytes/order mismatch")
        for key in ("pose_world", "capture_sim_time_s", "camera_id"):
            if final[key] != base_final[key]:
                raise ValueError(f"paired final render {key} mismatch")
        if name == "A_SHAM" and final["sha256"] != base_final["sha256"]:
            raise ValueError("off sham is not the exact same final RGB input")
        checked[name] = dict(session_id=branch["session_id"], delivered_frame_count=len(total),
                             frame_ids=[x["frame_id"] for x in total],
                             jpeg_hashes=[x["sha256"] for x in total])
    return dict(valid=True, branches=checked, final_pose_world=base_final["pose_world"],
                final_sim_time_s=base_final["capture_sim_time_s"],
                classification="DEVELOPMENT_COUNTERFACTUAL_INPUT_SCREENING")


def jpeg_wire_parity(saved_jpeg, wire_jpeg):
    """Bytes sent as the image payload, after base64 decoding if applicable."""
    a, b = bytes(saved_jpeg), bytes(wire_jpeg)
    return dict(valid=a == b, saved_sha256=_sha(a), wire_sha256=_sha(b),
                saved_byte_count=len(a), wire_byte_count=len(b))


def visibility_check(rgb_record, mask_record, mask, obstacle_ids, *, present,
                     instance_map, obstacle_prim_path, minimum_pixels=20):
    """Check actual instance-ID ownership and RGB/mask render synchronization.

    ``instance_map`` maps IDs to USD prim paths. A mask without verified label
    ownership is a technical failure, not evidence of obstacle invisibility.
    """
    rgb, annotation = _frame(rgb_record), _frame(mask_record)
    pixels = np.asarray(mask)
    if pixels.ndim != 2 or any(s == 0 for s in pixels.shape):
        raise ValueError("instance mask must be H by W")
    ids = list(obstacle_ids)
    owns = bool(ids) and all(str(instance_map.get(str(i), instance_map.get(i, ""))) == obstacle_prim_path
                            or str(instance_map.get(str(i), instance_map.get(i, ""))).startswith(obstacle_prim_path+"/")
                            for i in ids)
    selected = np.isin(pixels, ids) if owns else np.zeros_like(pixels, dtype=bool)
    count = int(selected.sum())
    coords = np.argwhere(selected)
    bbox = None if not len(coords) else [int(coords[:, 1].min()), int(coords[:, 0].min()),
                                       int(coords[:, 1].max()+1), int(coords[:, 0].max()+1)]
    flags = {"instance_owner_verified": owns,
             "same_camera_pose_render": all(rgb[k] == annotation[k] for k in
                 ("camera_id", "pose_world", "rendered_state_id", "capture_sim_time_s")),
             "same_pixel_resolution": (rgb.get("resolution_width_height") ==
                                       [pixels.shape[1], pixels.shape[0]] ==
                                       annotation.get("resolution_width_height")),
             "visible_pixel_gate": count >= minimum_pixels if present else count == 0}
    return dict(valid=all(flags.values()), flags=flags, obstacle_present=bool(present),
                obstacle_pixels=count if owns else None, image_fraction=count/pixels.size if owns else None,
                bounding_box_xyxy=bbox, mask_shape=list(pixels.shape),
                minimum_visible_pixels=minimum_pixels, proves_model_recognition=False)


def obstacle_state_check(record, reveal_sim_time_s):
    """All three states change together at the frozen reveal simulation clock."""
    expected = record["sim_time_s"] >= reveal_sim_time_s
    flags = {key: type(record.get(key)) is bool and record[key] == expected
             for key in ("render_visible", "world_obstacle_present", "evaluator_obstacle_present")}
    return dict(valid=all(flags.values()), expected_present=expected, flags=flags)


def pointing_diagnostics(response):
    """Preserve optional official payload/``next`` packet fields verbatim."""
    response = response["data"] if response.get("action") == "next" and isinstance(response.get("data"), dict) else response
    point = response.get("pointing")
    point = point if isinstance(point, dict) else {}
    return {"raw_text": deepcopy(response.get("raw_text")),
            "pointing": {k: deepcopy(point.get(k)) for k in
                         ("mode", "frame_size", "apos_px", "apos_state", "apos_clamped",
                          "opos_px", "opos_state", "opos_clamped")},
            **{k: deepcopy(response.get(k)) for k in ("visible", "stop", "actions", "timings_ms", "timings", "latency_ms", "seq", "rc")},
            "visible_semantics": "target visibility, not obstacle visibility",
            "stop_semantics": "top-level stop is distinct from pointing.apos_state",
            "absent_fields": "null means unavailable, not false or zero"}


def observation_anchored_world(local, observation_pose_world):
    """No ready/B pose parameter: only original observation transforms the rows."""
    raw = _array(local, 3, "local LightNav rows")
    return local_trajectory_to_world(observation_pose_world, raw).copy()


def _influence(record):
    origin = np.asarray(record["origin_xy"], dtype=float)
    forward = np.asarray(record["forward_xy"], dtype=float)
    if origin.shape != (2,) or forward.shape != (2,) or not np.isfinite([origin, forward]).all():
        raise ValueError("finite influence origin/direction required")
    norm = float(np.linalg.norm(forward))
    lo, hi = float(record["progress_min_m"]), float(record["progress_max_m"])
    if norm <= 0 or not np.isfinite([lo, hi]).all() or lo >= hi:
        raise ValueError("invalid influence direction/range")
    return origin, forward/norm, lo, hi


def response_geometry(off_world, on_world, influence, *, sample_step_m=.01,
                      minimum_chord_m=.02, minimum_separation_m=.20,
                      minimum_angle_deg=20.):
    """Sample both directions; qualifying differences require interior projections.

    Closest-segment ties follow the existing helper. Report endpoint gaps but
    never promote them to lateral evidence. Both sample and projected point
    must be inside the predeclared obstacle-progress region. Direction/yaw
    evidence additionally requires both source/target chords >= 0.02 m.
    Finite sampling is diagnostic, not an exact extremum or causal proof.
    """
    off = _array(off_world, 3, "OFF path", empty=True)
    on = _array(on_world, 3, "ON path", empty=True)
    origin, forward, lo, hi = _influence(influence)
    if sample_step_m <= 0 or minimum_chord_m <= 0:
        raise ValueError("positive sampling and tangent reliability thresholds")
    rows = []
    for name, source, target in (("OFF_TO_ON", off, on), ("ON_TO_OFF", on, off)):
        if len(source) < 2 or len(target) < 2 or np.linalg.norm(np.diff(target[:, :2], axis=0), axis=1).sum() <= 0:
            continue
        for i, (a, b) in enumerate(zip(source[:-1], source[1:])):
            chord = float(np.linalg.norm(b[:2]-a[:2]))
            n = max(2, int(np.ceil(chord/sample_step_m))+1)
            tangent = float(np.arctan2(b[1]-a[1], b[0]-a[0])) if chord > 0 else None
            for alpha in np.linspace(0., 1., n):
                pose = np.r_[a[:2]+alpha*(b[:2]-a[:2]), wrap_angle(a[2]+alpha*wrap_angle(b[2]-a[2]))]
                progress = float((pose[:2]-origin)@forward)
                if not lo <= progress <= hi:
                    continue
                p = projection_geometry(pose, target)
                qprogress = float((np.asarray(p["Q_xy_world_m"])-origin)@forward)
                interior = bool(p["projection_is_interior"] and lo <= qprogress <= hi)
                reliable = bool(interior and chord >= minimum_chord_m and p["segment_length_m"] >= minimum_chord_m)
                angle = float(np.degrees(abs(wrap_angle(tangent-p["phi_F_rad"])))) if reliable else None
                rows.append(dict(direction=name, source_segment_index=i, source_alpha=float(alpha),
                                 source_pose_world=pose.tolist(), source_progress_m=progress,
                                 source_chord_m=chord, projection=p, projected_progress_m=qprogress,
                                 interior_influence_projection=interior, direction_reliable=reliable,
                                 tangent_difference_deg=angle,
                                 reliable_yaw_difference_deg=p["abs_e_yaw_deg"] if reliable else None))
    interior = [x for x in rows if x["interior_influence_projection"]]
    reliable = [x for x in rows if x["direction_reliable"]]
    lateral = max((x["projection"]["e_perp_m"] for x in interior), default=None)
    tangent = max((x["tangent_difference_deg"] for x in reliable), default=None)
    yaw = max((x["reliable_yaw_difference_deg"] for x in reliable), default=None)
    meaningful = ((lateral is not None and lateral >= minimum_separation_m) or
                  (tangent is not None and tangent >= minimum_angle_deg) or
                  (yaw is not None and yaw >= minimum_angle_deg))
    return dict(meaningful=bool(meaningful), maximum_interior_separation_m=lateral,
                maximum_reliable_tangent_difference_deg=tangent, maximum_reliable_yaw_difference_deg=yaw,
                endpoint_projection_count=sum(not x["projection"]["projection_is_interior"] for x in rows),
                sample_step_m=sample_step_m, minimum_chord_m=minimum_chord_m,
                influence=deepcopy(influence), rows=rows, exact_continuous_maximum=False)


def whole_raw_polyline_check(world, environment, *, radius=.20, required_clearance=.05):
    """No suffix selection, no observation connector, no silently removed rows."""
    poses = _array(world, 3, "original raw world poses", empty=True)
    if not len(poses):
        return dict(clearance_valid=False, minimum_clearance_m=None, reason="NO_RETURNED_POSES",
                    original_row_count=0, checked_row_count=0, connector_included=False)
    report = environment.check_polyline(poses, radius=radius, required_clearance=required_clearance)
    return dict(report, original_row_count=len(poses), checked_row_count=len(poses),
                original_first_row_index=0, original_last_row_index=len(poses)-1,
                connector_included=False, rows_deleted=False)


def evaluate_screening(off_world, on_world, sham_world, *, observation_pose_world,
                       target_xy, environment_off, environment_on, influence,
                       bypass_plane, technical_checks, stops=None,
                       radius=.20, required_clearance=.05):
    """Classify a complete A/B/A' saved source condition without source editing.

    ``bypass_plane`` has point_xy and forward_xy; the point already includes
    the frozen obstacle rear extent/footprint-clearance offset. No double radius
    subtraction. Full raw on-polyline must begin before and end beyond it.
    Target progress is endpoint distance reduction from the observation pose;
    it is an intent proxy, not an internal navigation-goal guarantee.
    """
    paths = [_array(x, 3, name, empty=True) for x, name in
             ((off_world, "OFF"), (on_world, "ON"), (sham_world, "SHAM"))]
    off, on, sham = paths
    stops = stops or {}
    flags = {key: technical_checks.get(key) is True for key in REQUIRED_SCREENING_CHECKS}
    flags.update({str(k): v is True for k, v in technical_checks.items()})
    technical_valid = all(flags.values())
    checks = {}
    for name, path in zip(BRANCHES, paths):
        checks[name] = {"off": whole_raw_polyline_check(path, environment_off, radius=radius, required_clearance=required_clearance),
                        "on": whole_raw_polyline_check(path, environment_on, radius=radius, required_clearance=required_clearance)}
    comparison = response_geometry(off, on, influence)
    sham_comparison = response_geometry(off, sham, influence)
    plane = np.asarray(bypass_plane["point_xy"], float)
    forward = np.array(bypass_plane["forward_xy"], dtype=float, copy=True)
    if plane.shape != (2,) or forward.shape != (2,) or not np.isfinite([plane, forward]).all() or np.linalg.norm(forward) <= 0:
        raise ValueError("invalid bypass plane")
    forward /= np.linalg.norm(forward)
    target = np.asarray(target_xy, float)
    obs = np.asarray(observation_pose_world, float)
    if target.shape != (2,) or obs.shape != (3,) or not np.isfinite(target).all() or not np.isfinite(obs).all():
        raise ValueError("invalid target or observation pose")
    distance_reduction = None if not len(on) else float(np.linalg.norm(obs[:2]-target)-np.linalg.norm(on[-1, :2]-target))
    side = (on[:, :2]-plane)@forward if len(on) else np.array([])
    crossed = bool(len(side) >= 2 and side[0] < 0 and side[-1] >= 0)
    target_progress = bool(distance_reduction is not None and distance_reduction > 0)
    off_good = bool(checks["A_OFF"]["off"]["clearance_valid"] and checks["A_SHAM"]["off"]["clearance_valid"])
    off_blocked = all(checks[k]["on"].get("workspace_known", False) and
                      not checks[k]["on"]["clearance_valid"] for k in ("A_OFF", "A_SHAM"))
    raw_hashes = {_name: _sha(path.tobytes()) for _name, path in zip(BRANCHES, paths)}
    stop = bool(stops.get("B_ON", False))
    safe = checks["B_ON"]["on"]["clearance_valid"]
    if not technical_valid:
        label = "TECHNICAL_INVALID"
    elif stop:
        label = "STOP_RESPONSE"
    elif not comparison["meaningful"]:
        label = "NO_MATERIAL_RESPONSE"
    elif not safe:
        label = "CHANGED_BUT_UNSAFE"
    elif crossed and target_progress and off_good and off_blocked and not any(stops.get(k, False) for k in ("A_OFF", "A_SHAM")):
        label = "SAFE_BYPASS_CANDIDATE"
    else:
        label = "SAFE_PARTIAL_DETOUR"
    return dict(label=label, qualified=label == "SAFE_BYPASS_CANDIDATE", technical_checks=flags,
                checks=checks, off_on=comparison, off_sham=sham_comparison,
                whole_raw_on_safe=bool(safe), off_sham_valid_without_obstacle=off_good,
                off_sham_obstructed_with_obstacle=bool(off_blocked), bypass_plane_crossed=crossed,
                target_directed_progress=target_progress, target_distance_reduction_m=distance_reduction,
                beyond_plane_endpoint_margin_m=float(side[-1]) if len(side) else None,
                raw_value_sha256=raw_hashes, off_on_hash_different=raw_hashes["A_OFF"] != raw_hashes["B_ON"],
                stops=deepcopy(stops), new_optimizer_or_controller_outcomes_used=False,
                actual_obstacle_traversal_tested=False)


def moving_boundary_qualification(states_world, state_times_s, applied_commands, *,
                                  physical_u_minus, previous_control, timing,
                                  source_flags, radius=.20, required_clearance=.05,
                                  environment_on=None):
    """Validate saved observation->first-FRESH-application prefix, without rollout.

    States include exact observation and B ticks, commands hold over each adjacent
    state pair. Timing fields: old/fresh_observation_sim_s, reveal_sim_s,
    ready_seen_sim_s, install_sim_s, application_sim_s, request_send_host_s,
    full_receipt_host_s, inflight_sim_span_s, inflight_host_span_s,
    maximum_loop_stall_s, overlap_observed, hold_timeout, stale_rejected,
    source_stop, aborted. Caller separately proves command/chunk identities and
    genuine source with ``source_flags``. No clock domains are subtracted.
    """
    states = _array(states_world, 3, "execution states")
    times = np.asarray(state_times_s, dtype=float)
    commands = _array(applied_commands, 2, "held commands", empty=True)
    u = np.asarray(physical_u_minus, dtype=float)
    memory = np.asarray(previous_control, dtype=float)
    if len(states) < 2 or times.shape != (len(states),) or len(commands) != len(states)-1 or np.any(np.diff(times) <= 0) or not np.isfinite(times).all():
        raise ValueError("matching chronological executed ticks and held commands required")
    if u.shape != (2,) or memory.shape != (2,) or not np.isfinite([u, memory]).all():
        raise ValueError("separate physical command and controller memory required")
    errors = np.array([np.abs(np.r_[integrate_unicycle(a, cmd, dt)[:2]-b[:2],
                     wrap_angle(integrate_unicycle(a, cmd, dt)[2]-b[2])])
                       for a, b, cmd, dt in zip(states[:-1], states[1:], commands, np.diff(times))])
    distance = float(np.linalg.norm(np.diff(states[:, :2], axis=0), axis=1).sum())
    host = float(timing["inflight_host_span_s"])
    rtf = float(timing["inflight_sim_span_s"])/host if host > 0 else None
    # Arc-to-chord allowance for each exact held-command arc, applied to safety
    # only, not to the experimental moving-distance measurement.
    angles = np.abs(commands[:, 1]*np.diff(times))
    radius_turn = np.divide(np.abs(commands[:, 0]), np.abs(commands[:, 1]),
                            out=np.zeros(len(commands)), where=np.abs(commands[:, 1]) > 0)
    curve_bound = float(np.max(radius_turn*np.where(angles <= np.pi,
                                                    1-np.cos(angles/2), 2.)))
    safe = None if environment_on is None else environment_on.check_trajectory(
        times, states, radius=radius, required_clearance=required_clearance,
        curved_path_error_bound_m=curve_bound)
    flags = {str(k): v is True for k, v in source_flags.items()}
    flags.update(genuine_source_flags_supplied=bool(source_flags),
                 moving_velocity=bool(u[0] > .20), moving_translation=distance >= .02,
                 physical_command_matches_last_applied=bool(np.array_equal(u, commands[-1])),
                 execution_reconstructed=bool(np.max(errors) <= 1e-9),
                 observation_and_B_ticks=bool(abs(times[0]-timing["fresh_observation_sim_s"]) <= 1e-9 and
                                              abs(times[-1]-timing["application_sim_s"]) <= 1e-9),
                 reveal_order=bool(timing["old_observation_sim_s"] < timing["reveal_sim_s"] <= timing["fresh_observation_sim_s"]),
                 ready_install_application_order=bool(timing["fresh_observation_sim_s"] <= timing["ready_seen_sim_s"] <= timing["install_sim_s"] <= timing["application_sim_s"]),
                 request_receipt_order=bool(timing["full_receipt_host_s"] >= timing["request_send_host_s"]),
                 request_local_RTF=bool(rtf is not None and .8 <= rtf <= 1.2),
                 loop_stall=bool(0 <= timing["maximum_loop_stall_s"] <= .25),
                 inference_OLD_overlap=bool(timing["overlap_observed"]),
                 no_hold_timeout=not timing["hold_timeout"], no_stale_rejection=not timing["stale_rejected"],
                 no_source_STOP=not timing["source_stop"], no_abort=not timing["aborted"],
                 actual_prefix_and_B_safe=bool(safe is not None and safe["clearance_valid"]))
    return dict(qualified=all(flags.values()), flags=flags,
                failure_reasons=[k for k, v in flags.items() if not v], B_world=states[-1].tolist(),
                physical_u_minus=u.tolist(), controller_previous_control=memory.tolist(),
                physical_memory_identical=bool(np.array_equal(u, memory)),
                observation_to_B_sampled_path_length_m=distance, request_local_RTF=rtf,
                request_RTT_s=float(timing["full_receipt_host_s"]-timing["request_send_host_s"]),
                observation_to_application_sim_s=float(timing["application_sim_s"]-timing["fresh_observation_sim_s"]),
                maximum_reconstruction_error=errors.max(axis=0).tolist(), prefix_clearance=safe,
                timing=deepcopy(timing), new_rollout_performed=False)

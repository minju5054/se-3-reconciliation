"""Frozen-state GP-SE2-01 counterfactuals using the unmodified official MPC.

Historical solve verification and the new synchronous experiment are deliberately
separate operations.  Simulation time never advances while a solve computes.
World candidates are represented in the ORIGINAL RGB capture frame before being
installed; this coordinate conversion never changes the agent's initial state.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import time
from typing import Any

import numpy as np

from reconciliation.online_mpc_adapter import finite_pose, selection_audit, sha256
from reconciliation.robotless_online import integrate_unicycle
from reconciliation.se2 import local_trajectory_to_world, relative_pose, wrap_angle

PREVIOUS_CONTROL_POLICY = (
    "restore previous_command supplied to the recorded first-FRESH solve, i.e. "
    "pre-FRESH controller memory; separately restore the last physically applied "
    "u_minus at B; never seed from the historical first-FRESH output"
)
FAILURE_POLICY = (
    "use official MpcTracker.poll behavior: a failed solve resets applied and "
    "internal previous command to zero; log the failure and continue the same "
    "fixed horizon; hold the resulting command until the next scheduled solve; "
    "collision never changes commands or terminates this counterfactual"
)


def _command(value, name="command"):
    a = np.asarray(value, dtype=float)
    if a.shape != (2,) or not np.isfinite(a).all():
        raise ValueError(f"{name} must be finite [v_mps, omega_radps]")
    return a.copy()


def _path(value):
    a = np.asarray(value, dtype=float)
    if a.ndim != 2 or a.shape[1:] != (3,) or not len(a) or not np.isfinite(a).all():
        raise ValueError("candidate must be nonempty finite Nx3 world poses")
    return a.copy()


def _same_pose(a, b, tolerance=1e-10):
    return bool(np.allclose(np.asarray(a)[:2], np.asarray(b)[:2], atol=tolerance, rtol=0)
                and abs(float(wrap_angle(np.asarray(a)[2]-np.asarray(b)[2]))) <= tolerance)


def _file(path, root):
    return {"path": str(path.relative_to(root)), "sha256": sha256(path), "bytes": path.stat().st_size}


def verify_source_records(source_root, records):
    root = Path(source_root).resolve()
    for record in records:
        path = (root / record["path"]).resolve()
        if not path.is_relative_to(root) or _file(path, root) != record:
            raise ValueError(f"source integrity mismatch: {record['path']}")


def load_frozen_context(source_root, episode_id, handoff_id):
    """Restore B, last applied command and PRE-FRESH solve memory separately.

    The completion manifest authenticates every read acquisition file.  Recorded
    CSV states, applied commands and controller JSONL independently corroborate
    the handoff context.  Full source-bank validation is a caller prerequisite.
    """
    root = Path(source_root).resolve()
    episode = root / "episodes" / episode_id
    if any(Path(value).name != value for value in (episode_id, handoff_id)):
        raise ValueError("episode and handoff must be single path components")
    manifest_path = episode / "completion.json"
    manifest = json.loads(manifest_path.read_text())["raw_manifest"]["files"]
    expected = {record["path"]: record for record in manifest}
    records = [_file(manifest_path, root)]

    def checked(relative):
        path = (episode / relative).resolve()
        if not path.is_relative_to(episode.resolve()):
            raise ValueError("source reference escaped episode")
        record = _file(path, episode)
        if expected.get(relative) != record:
            raise ValueError(f"acquisition hash mismatch: {relative}")
        records.append(_file(path, root))
        return path

    context = json.loads(checked(f"handoffs/{handoff_id}/context.json").read_text())
    if context["status"] not in ("VALID_HANDOFF_MOVING", "VALID_HANDOFF_STATIONARY"):
        raise ValueError("event is not a valid recorded handoff")
    world_arrays = {}
    for kind in ("old", "fresh"):
        for frame in ("raw_local", "world"):
            record = context[f"{kind}_{frame}_ref"]
            path = checked(record["path"])
            if sha256(path) != record["sha256"]:
                raise ValueError("event path hash differs from acquisition manifest")
            world_arrays[f"{kind}_{frame}"] = _path(np.load(path, allow_pickle=False))
    with checked("execution.csv").open(newline="") as stream:
        states = {int(row["state_id"]): row for row in csv.DictReader(stream)}
    with checked("commands.csv").open(newline="") as stream:
        commands = {int(row["command_id"]): row for row in csv.DictReader(stream)}
    logs = [json.loads(line) for line in checked("controller/events.jsonl").read_text().splitlines()]
    historical = context["first_fresh_solve"]
    logged = [row for row in logs if row.get("type") == "solve_result"
              and row.get("solve_id") == historical["solve_id"]]
    if len(logged) != 1:
        raise ValueError("historical solve must occur exactly once in controller log")
    for field in ("input_pose", "input_state_id", "previous_command", "command", "selection"):
        if logged[0][field] != historical[field]:
            raise ValueError(f"historical solve context/log disagree: {field}")
    b_state = states[context["switch_state_id"]]
    pose_from_row = lambda row: [float(row[k]) for k in ("x", "y", "yaw")]
    boundary = finite_pose(context["B"], "B")
    if not _same_pose(boundary, pose_from_row(b_state)):
        raise ValueError("B does not equal actual pre-first-FRESH-command state")
    if not _same_pose(historical["input_pose"], pose_from_row(states[historical["input_state_id"]])):
        raise ValueError("historical input pose differs from recorded execution")
    preceding = commands[int(b_state["incoming_command_id"])]
    declared = context["pre_switch_command"]
    if int(preceding["command_id"]) != declared["command_id"]:
        raise ValueError("u_minus is not the incoming physical command at B")
    u_minus = _command([float(preceding[k]) for k in ("v_mps", "omega_radps")])
    if not np.array_equal(u_minus, [declared["v_mps"], declared["omega_radps"]]):
        raise ValueError("physical u_minus context/CSV disagree")
    previous_state = states[context["previous_state_id"]]
    dt = float(b_state["sim_time_s"]) - float(previous_state["sim_time_s"])
    predicted_boundary = integrate_unicycle(pose_from_row(previous_state), u_minus, dt)
    if not _same_pose(predicted_boundary, boundary):
        raise ValueError("B is not the exact integration of the physical incoming command")
    first_applied = commands[context["first_fresh_command"]["command_id"]]
    if int(first_applied["application_state_id"]) != context["switch_state_id"]:
        raise ValueError("historical first FRESH command not applied at B")
    capture = finite_pose(context["R_obs"], "capture pose")
    recomposed = local_trajectory_to_world(capture, world_arrays["fresh_raw_local"])
    if not np.allclose(recomposed[:, :2], world_arrays["fresh_world"][:, :2], atol=1e-12, rtol=0):
        raise ValueError("FRESH original capture transform mismatch")
    if not np.allclose(wrap_angle(recomposed[:, 2]-world_arrays["fresh_world"][:, 2]), 0, atol=1e-12, rtol=0):
        raise ValueError("FRESH original capture yaw mismatch")
    previous_control = _command(historical["previous_command"])
    return {
        "case_id": f"{episode_id}/{handoff_id}", "episode_id": episode_id, "handoff_id": handoff_id,
        "source_root": str(root), "source_files": records,
        "B_world": boundary.tolist(), "u_minus": u_minus.tolist(),
        "previous_control": previous_control.tolist(),
        "previous_control_minus_u_minus": (previous_control-u_minus).tolist(),
        "previous_control_policy": PREVIOUS_CONTROL_POLICY,
        "original_capture_pose_world": capture.tolist(),
        "historical_first_fresh_solve": historical,
        "switch_state_id": context["switch_state_id"], "switch_sim_time_s": float(b_state["sim_time_s"]),
        "historical_solve_input_to_switch_sim_s": float(b_state["sim_time_s"])-historical["input_sim_time_s"],
        "source_timestamps": {k: context[k] for k in ("t_obs", "t_request", "t_ready_host", "t_install", "t_switch")},
        "frame": "fixed Isaac world; metres; radians CCW about +Z; capture local +x forward,+y left",
        "intrinsic_lightnav_waypoint_dt_s": None,
        **{k: v.tolist() for k, v in world_arrays.items()},
    }


def candidate_in_capture_frame(candidate_world, capture_pose):
    """Derived inverse projection, explicitly NOT re-anchoring at B."""
    world, anchor = _path(candidate_world), finite_pose(capture_pose, "capture pose")
    local = relative_pose(anchor, world)
    recovered = local_trajectory_to_world(anchor, local)
    errors = recovered-world
    errors[:, 2] = wrap_angle(errors[:, 2])
    maximum = float(np.max(np.abs(errors)))
    if maximum > 1e-11:
        raise ValueError("candidate capture-frame roundtrip failed")
    return local, {"source_frame": "fixed Isaac world", "target_frame": "original FRESH RGB capture agent frame",
        "transform": "T_capture_candidate = inverse(T_world_capture) * T_world_candidate",
        "capture_pose_world": anchor.tolist(), "units": "metres,radians",
        "local_axes": "+x forward,+y left,+z up;yaw CCW", "roundtrip_max_abs_error": maximum,
        "B_reanchoring": False, "raw_modified": False}


def _synchronous_solve(module, tracker, pose):
    """Wait for actual official submit/poll; elapsed wall time is separate."""
    pose = finite_pose(pose, "solve input")
    previous = _command(tracker.previous_command)
    predicted_selection = module.build_pose_aligned_reference(
        tracker._trajectory, pose, horizon=module.HORIZON, weights=module.Q_WEIGHTS)
    start = time.perf_counter()
    tracker.submit(pose.copy())
    future = tracker._future
    if future is None:
        raise RuntimeError("official tracker did not submit a solve")
    result, error = None, None
    try:
        result = future.result()
    except Exception as exc:  # Official poll handles precisely this same failure.
        error = f"{type(exc).__name__}: {exc}"
    command = tracker.poll()
    if command is None:
        raise RuntimeError("official synchronous solve produced no current-generation command")
    applied = _command(command)
    reference = predicted_selection if result is None else result.reference
    audit = selection_audit(tracker._trajectory, pose, reference,
                            horizon=module.HORIZON, weights=module.Q_WEIGHTS)
    return {
        "input_pose_world": pose.tolist(), "previous_control": previous.tolist(),
        "command": applied.tolist(), "previous_control_after": list(tracker.previous_command),
        "success": error is None and not tracker.error, "error": error or tracker.error or None,
        "selection": audit, "selection_source": "actual official solve result" if result is not None else "official submitted reference; solve failed",
        "prediction_world": None if result is None else np.asarray(result.prediction).tolist(),
        "official_solve_wall_s": None if result is None else float(result.solve_ms)/1000,
        "submit_wait_poll_wall_s": time.perf_counter()-start,
        "simulation_time_advanced_during_solve_s": 0.,
    }


def historical_solve_audit(module, frozen, *, command_atol=1e-6):
    """Re-solve at its HISTORICAL input state, never at the later B."""
    recorded = frozen["historical_first_fresh_solve"]
    tracker = module.MpcTracker()
    try:
        tracker.set_body_path(frozen["fresh_raw_local"], frozen["original_capture_pose_world"])
        tracker.command = tuple(frozen["u_minus"])
        tracker.previous_command = tuple(_command(recorded["previous_command"]))
        result = _synchronous_solve(module, tracker, recorded["input_pose"])
        command_error = np.asarray(result["command"])-recorded["command"]
        reference = np.asarray(result["selection"]["reference_world"])
        expected = np.asarray(recorded["selection"]["reference_world"])
        reference_error = reference-expected
        reference_error[:, 2] = wrap_angle(reference_error[:, 2])
        selection_match = result["selection"]["indices"] == recorded["selection"]["indices"]
        passed = bool(result["success"] and selection_match and np.max(np.abs(reference_error)) <= 1e-12
                      and np.max(np.abs(command_error)) <= command_atol)
        return {"audit_kind": "historical solve input-pose/previous-control replay", "passed": passed,
            "historical_solve_id": recorded["solve_id"], "historical_input_pose_world": recorded["input_pose"],
            "B_world": frozen["B_world"], "input_pose_equals_B": _same_pose(recorded["input_pose"], frozen["B_world"]),
            "recorded_command": recorded["command"], "command_error": command_error.tolist(),
            "command_comparison_atol": command_atol, "selection_indices_match": selection_match,
            "reference_max_abs_error": float(np.max(np.abs(reference_error))), "result": result,
            "B_counterfactual_equality_required": False}
    finally:
        tracker.close()


def counterfactual_rollout(module, frozen, candidate_world, *, horizon_s=3., control_hz=10., integration_hz=60.):
    """One NEW independent controller+state rollout; no observations or VLA.

    Solve at t=0, 0.1, ..., 2.9; each applied command drives six exact 1/60s
    integration steps.  The endpoint t=3 is recorded without an unused solve.
    """
    values = (horizon_s, control_hz, integration_hz)
    if any(not math.isfinite(v) or v <= 0 for v in values):
        raise ValueError("finite positive horizon and rates required")
    if not np.isclose(control_hz, module.CONTROL_RATE_HZ, atol=1e-12, rtol=0):
        raise ValueError("control rate differs from unchanged official controller")
    stride, steps = round(integration_hz/control_hz), round(horizon_s*integration_hz)
    if stride < 1 or not np.isclose(stride*control_hz, integration_hz, atol=1e-12, rtol=0) or not np.isclose(steps/integration_hz, horizon_s, atol=1e-12, rtol=0) or steps % stride:
        raise ValueError("horizon and control grid must align with integer integration ticks")
    world = _path(candidate_world)
    local, transform = candidate_in_capture_frame(world, frozen["original_capture_pose_world"])
    tracker = module.MpcTracker()
    pose = finite_pose(frozen["B_world"], "B")
    initial = _command(frozen["u_minus"])
    previous = _command(frozen["previous_control"])
    states = [{"tick": 0, "time_s": 0., "pose_world": pose.tolist()}]
    commands, solves = [], []
    started = time.perf_counter()
    try:
        tracker.set_body_path(local, frozen["original_capture_pose_world"])
        tracker.command, tracker.previous_command = tuple(initial), tuple(previous)
        installed = _path(tracker._trajectory)
        if not np.allclose(installed[:, :2], world[:, :2], atol=1e-11, rtol=0) or not np.allclose(wrap_angle(installed[:, 2]-world[:, 2]), 0, atol=1e-11, rtol=0):
            raise ValueError("official controller candidate projection differs from world candidate")
        command = initial.copy()
        for tick in range(steps):
            if tick % stride == 0:
                solve = _synchronous_solve(module, tracker, pose)
                solve.update(solve_index=len(solves), tick=tick, time_s=tick/integration_hz)
                solves.append(solve)
                command = _command(solve["command"])
            commands.append({"tick": tick, "time_s": tick/integration_hz,
                "end_time_s": (tick+1)/integration_hz, "command": command.tolist(),
                "solve_index": len(solves)-1, "held": bool(tick % stride)})
            pose = integrate_unicycle(pose, command, 1/integration_hz)
            states.append({"tick": tick+1, "time_s": (tick+1)/integration_hz, "pose_world": pose.tolist()})
    finally:
        tracker.close()
    return {
        "kind": "OFFLINE COUNTERFACTUAL HANDOFF COMPARISON", "case_id": frozen["case_id"],
        "horizon_s": float(horizon_s), "control_hz": float(control_hz), "integration_hz": float(integration_hz),
        "initial_pose_world": frozen["B_world"], "initial_physical_command": initial.tolist(),
        "initial_previous_control": previous.tolist(), "previous_control_policy": PREVIOUS_CONTROL_POLICY,
        "failure_policy": FAILURE_POLICY, "state_semantics": "state at tick is pre-command; following state is exact held-command SE(2) integration",
        "schedule": "idealized synchronous: new solve at t=0 then fixed control grid; wall time does not advance simulation",
        "new_lightnav_updates": 0, "pose_snaps": 0, "candidate_frame_transform": transform,
        "candidate_world": world.tolist(), "candidate_capture_local": local.tolist(),
        "states": states, "commands": commands, "controller_reference_selections": solves,
        "controller_failure_count": sum(not s["success"] for s in solves),
        "rollout_wall_s": time.perf_counter()-started,
        "official_mpc_solve_wall_s": [s["official_solve_wall_s"] for s in solves],
        "source_files": frozen.get("source_files", []),
        "resolved_limits": {"v_max": float(module.OBJNAV_V_MAX), "omega_max": float(module.W_MAX),
            "a_v_max": float(module.A_MAX_V), "a_omega_max": float(module.A_MAX_W)},
        "official_gains": {"Q_WEIGHTS": list(module.Q_WEIGHTS), "R_WEIGHTS": list(module.R_WEIGHTS)},
    }


def dense_rollout_samples(rollout, max_dt_s=.005):
    """Evaluate exact command-held execution at a denser independent time grid."""
    if not math.isfinite(max_dt_s) or max_dt_s <= 0:
        raise ValueError("positive finite dense query spacing required")
    horizon = rollout["horizon_s"]
    times = np.linspace(0, horizon, math.ceil(horizon/max_dt_s)+1)
    states, commands = rollout["states"], rollout["commands"]
    start_times = np.asarray([c["time_s"] for c in commands])
    poses = []
    for t in times:
        if t >= horizon:
            poses.append(states[-1]["pose_world"])
            continue
        index = int(np.searchsorted(start_times, t, side="right")-1)
        elapsed = float(t-start_times[index])
        pose = states[index]["pose_world"]
        poses.append(pose if elapsed <= 1e-14 else integrate_unicycle(pose, commands[index]["command"], elapsed))
    return times, np.asarray(poses, dtype=float)


def execution_metrics(rollout, goal, *, position_tolerance_m=.15, yaw_tolerance_rad=np.pi/12, dwell_s=.2,
                      limits=None):
    """Kinematic/task diagnostics only; environment and gates checked elsewhere."""
    goal = finite_pose(goal, "goal")
    times, poses = dense_rollout_samples(rollout)
    position_error = np.linalg.norm(poses[:, :2]-goal[:2], axis=1)
    yaw_error = np.abs(wrap_angle(poses[:, 2]-goal[2]))
    in_goal = (position_error <= position_tolerance_m) & (yaw_error <= yaw_tolerance_rad)
    terminal = times >= rollout["horizon_s"]-dwell_s-1e-12
    selected = rollout["controller_reference_selections"]
    commands = np.asarray([s["command"] for s in selected])
    command_steps = np.diff(np.vstack([rollout["initial_physical_command"], commands]), axis=0)
    accelerations = command_steps*rollout["control_hz"]
    settings = rollout["resolved_limits"] if limits is None else limits
    # Include the real pre-boundary applied command, not just controller memory.
    violations = {"speed": bool(np.any(commands[:, 0] < -1e-6) or np.any(commands[:, 0] > settings["v_max"]+1e-6)),
        "angular_speed": bool(np.any(np.abs(commands[:, 1]) > settings["omega_max"]+1e-6)),
        "linear_acceleration": bool(np.any(np.abs(accelerations[:, 0]) > settings["a_v_max"]+1e-5)),
        "angular_acceleration": bool(np.any(np.abs(accelerations[:, 1]) > settings["a_omega_max"]+1e-5))}
    official_times = [float(row["official_solve_wall_s"]) for row in selected if row.get("official_solve_wall_s") is not None]
    wait_times = [float(row["submit_wait_poll_wall_s"]) for row in selected if row.get("submit_wait_poll_wall_s") is not None]
    compute = {}
    for label, values in (("official_solve", official_times), ("submit_wait_poll", wait_times)):
        compute[f"mpc_{label}_count_available"] = len(values)
        compute[f"mpc_{label}_total_s"] = float(np.sum(values)) if values else None
        compute[f"mpc_{label}_median_s"] = float(np.median(values)) if values else None
        compute[f"mpc_{label}_max_s"] = float(np.max(values)) if values else None
    return {"initial_goal_distance_m": float(position_error[0]),
        "goal_distance_improvement_m": float(position_error[0]-position_error[-1]),
        "net_displacement_m": float(np.linalg.norm(poses[-1, :2]-poses[0, :2])),
        "progress_definition": "initial minus terminal Euclidean distance to the unchanged common goal; positive means closer, not navigation success",
        "mpc_solve_count_total": len(selected), **compute,
        "mpc_compute_semantics": "official solve compute and submit/wait/poll wall times separately; neither advances simulation; failed official solves may lack official duration but retain wait duration",
        "goal_reached": bool(in_goal.any()), "time_to_goal_s": float(times[np.flatnonzero(in_goal)[0]]) if in_goal.any() else None,
        "terminal_goal_dwell_pass": bool(in_goal[terminal].all()), "terminal_goal_dwell_s": dwell_s,
        "goal_position_tolerance_m": position_tolerance_m, "goal_yaw_tolerance_rad": yaw_tolerance_rad,
        "terminal_position_error_m": float(position_error[-1]), "terminal_yaw_error_rad": float(yaw_error[-1]),
        "first_command_delta_v_mps": float(command_steps[0, 0]), "first_command_delta_omega_radps": float(command_steps[0, 1]),
        "command_total_variation_v_mps": float(np.abs(command_steps[:, 0]).sum()),
        "command_total_variation_omega_radps": float(np.abs(command_steps[:, 1]).sum()),
        "control_grid_max_abs_acceleration_v_mps2": float(np.max(np.abs(accelerations[:, 0]))),
        "control_grid_max_abs_acceleration_omega_radps2": float(np.max(np.abs(accelerations[:, 1]))),
        "acceleration_definition": "10Hz finite differences including first candidate command minus physically applied u_minus; not a continuous acceleration bound",
        "path_length_m": float(sum(abs(c["command"][0])*(c["end_time_s"]-c["time_s"]) for c in rollout["commands"])),
        "goal_reentries_after_exit": int(np.count_nonzero(np.diff(in_goal.astype(int)) == 1))-int(not in_goal[0] and in_goal.any()),
        "motion_limit_violations": violations, "motion_limits_pass": not any(violations.values()),
        "controller_failure_count": rollout["controller_failure_count"],
        "controller_validity_pass": rollout["controller_failure_count"] == 0, "clearance_validity": "N/A: requires independent environment checker",
        "route_validity": "N/A: requires independent frozen-gate checker", "goal_query_dt_max_s": float(np.max(np.diff(times)))}


def save_rollout(output, rollout):
    """Write a new method rollout directory; refuse every overwrite."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    for name, payload in (("rollout_states.json", rollout["states"]),
                          ("rollout_commands.json", rollout["commands"]),
                          ("controller_reference_selections.json", rollout["controller_reference_selections"]),
                          ("rollout.json", rollout)):
        with (output/name).open("x") as stream:
            json.dump(payload, stream, indent=2, allow_nan=False)
            stream.write("\n")
    times, poses = dense_rollout_samples(rollout)
    for name, array in (("rollout_times.npy", times), ("rollout_poses_world.npy", poses),
                        ("sampled_mpc_reference.npy", np.asarray(rollout["candidate_world"])),
                        ("derived_capture_local.npy", np.asarray(rollout["candidate_capture_local"]))):
        with (output/name).open("xb") as stream:
            np.save(stream, array, allow_pickle=False)
    hashes = [_file(p, output) for p in sorted(output.iterdir()) if p.is_file()]
    with (output/"hashes.json").open("x") as stream:
        json.dump({"files": hashes}, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return output

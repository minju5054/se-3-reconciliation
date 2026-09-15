#!/usr/bin/env python3
"""Validate genuine online raw streams, activation lineage, history and plots.

No inference, rendering, path fitting, synthetic fallback or raw writes. Timing
limitations remain separate from causal stream validity. A valid stationary
handoff still needs actual simulation updates during pending inference.
"""

from __future__ import annotations

import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from reconciliation.online_history import SessionHistory, history_contract
from reconciliation.online_handoff_analysis import VALID_STATUSES
from reconciliation.robotless_online import STATUSES, integrate_unicycle, verify_resume, dump_new
from reconciliation.robotless_single_chunk import load_config, observation_to_world
from reconciliation.se2 import wrap_angle


def read_json(path):
    return json.loads(Path(path).read_text())


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def read_jsonl(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def equal_record(actual, expected, path="record", atol=1e-9):
    """Compare reconstructed nested numeric evidence, rejecting missing fields."""
    if isinstance(expected, dict):
        if not isinstance(actual, dict) or actual.keys() != expected.keys():
            raise ValueError(f"{path}: fields differ")
        for key in expected:
            equal_record(actual[key], expected[key], f"{path}.{key}", atol)
    elif isinstance(expected, (list, tuple)):
        if not isinstance(actual, (list, tuple)) or len(actual) != len(expected):
            raise ValueError(f"{path}: list dimensions differ")
        for i, (left, right) in enumerate(zip(actual, expected)):
            equal_record(left, right, f"{path}[{i}]", atol)
    elif isinstance(expected, (int, float)) and not isinstance(expected, bool):
        if isinstance(actual, bool) or not isinstance(actual, (int, float)) or not np.isfinite([actual, expected]).all() or not np.isclose(actual, expected, rtol=0, atol=atol):
            raise ValueError(f"{path}: numeric reconstruction mismatch")
    elif actual != expected:
        raise ValueError(f"{path}: value differs")


def verify_ref(root, record):
    import hashlib
    root = Path(root).resolve()
    path = (root / record["path"]).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError("source reference missing or escapes its root")
    if path.stat().st_size != record["bytes"] or hashlib.sha256(path.read_bytes()).hexdigest() != record["sha256"]:
        raise ValueError(f"source hash/size mismatch: {record['path']}")
    return path


def validate_execution(execution, commands, *, integration_dt_s=None, atol=1e-9):
    """Rebuild every finite-rate state transition from actually applied commands."""
    if len(execution) < 1:
        raise ValueError("execution stream is empty")
    ids = [int(r["state_id"]) for r in execution]
    if len(set(ids)) != len(ids) or any(b <= a for a, b in zip(ids, ids[1:])):
        raise ValueError("execution state IDs must be unique and increasing")
    poses = np.array([[float(row[k]) for k in ("x", "y", "yaw")] for row in execution])
    times = np.array([float(row["sim_time_s"]) for row in execution])
    host_ns = [int(row["host_monotonic_ns"]) for row in execution]
    if not np.isfinite(poses).all() or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError("state and simulation time must be finite and time increasing")
    if any(b < a for a, b in zip(host_ns, host_ns[1:])):
        raise ValueError("execution host monotonic time moved backwards")
    by_id = {int(c["application_state_id"]): c for c in commands}
    if len(by_id) != len(commands) or any(i not in set(ids) for i in by_id):
        raise ValueError("applied command state IDs are duplicate or unavailable")
    if any(i not in by_id for i in ids[:-1]):
        raise ValueError("state transition is missing its actual applied command")
    max_error = 0.0
    for i in range(len(execution) - 1):
        row, command = execution[i], by_id[ids[i]]
        dt = times[i+1] - times[i]
        if integration_dt_s is not None and not np.isclose(dt, integration_dt_s, rtol=0, atol=1e-8):
            raise ValueError("simulation step differs from frozen integration dt")
        if not np.isclose(float(command["sim_time_s"]), times[i], rtol=0, atol=1e-10):
            raise ValueError("command application time differs from its starting state")
        if "application_tick" in command and "tick" in row and int(command["application_tick"]) != int(row["tick"]):
            raise ValueError("applied command tick differs from state tick")
        expected = integrate_unicycle(poses[i], [float(command["v_mps"]), float(command["omega_radps"])], dt)
        error = poses[i+1] - expected
        error[2] = wrap_angle(error[2])
        max_error = max(max_error, float(np.abs(error).max()))
        if np.abs(error).max() > atol:
            raise ValueError(f"state {ids[i+1]} is not the recorded command integration; possible snap/teleport")
    return {"state_count": len(execution), "integration_count": len(execution)-1,
            "command_count": len(commands), "maximum_pose_reconstruction_error": max_error}


def validate_frame_anchor(frame, execution_by_id):
    """The model trigger belongs to the state actually rendered, not send time."""
    row = execution_by_id[int(frame["rendered_state_id"])]
    pose = [float(row[k]) for k in ("x", "y", "yaw")]
    if not np.allclose(frame["pose_world"], pose, rtol=0, atol=1e-10):
        raise ValueError("capture pose differs from its rendered state ID")
    if not np.isclose(frame["capture_sim_time_s"], float(row["sim_time_s"]), rtol=0, atol=1e-10):
        raise ValueError("capture simulation time differs from rendered state")


def validate_controller_records(execution, commands, events, worlds, settings, *,
                                submission_stride=None, minimum_submission_sim_s=None,
                                episode_id=None):
    """Associate applied commands with actual current-generation official solves."""
    from reconciliation.online_mpc_adapter import selection_audit
    by_state = {int(r["state_id"]): r for r in execution}
    foreign_results = [e for e in events if e.get("type") == "solve_result" and episode_id is not None and e.get("episode_id") != episode_id]
    if episode_id is not None:
        events = [e for e in events if e.get("episode_id") == episode_id]
    accepted_submissions = [e for e in events if e.get("type") == "reply" and e.get("status") == "submitted"]
    if submission_stride is not None:
        if type(submission_stride) is not int or submission_stride < 1 or minimum_submission_sim_s is None or minimum_submission_sim_s <= 0:
            raise ValueError("fixed controller schedule must have a positive integer tick stride and period")
        previous_time = None
        for record in accepted_submissions:
            state_id = int(record["input_state_id"])
            state = by_state[state_id]
            if state_id % submission_stride or int(state["tick"]) % submission_stride:
                raise ValueError("accepted controller solve was submitted off the frozen control tick grid")
            sim = float(record["input_sim_time_s"])
            if not np.isclose(sim, float(state["sim_time_s"]), rtol=0, atol=1e-10):
                raise ValueError("accepted submission input time differs from its simulation state")
            if previous_time is not None and sim - previous_time < minimum_submission_sim_s - 1e-8:
                raise ValueError("accepted controller submissions exceed the frozen control rate")
            previous_time = sim
    solved = [e for e in events if e.get("type") == "solve_result"]
    by_solve = {e["solve_id"]: e for e in solved}
    if len(by_solve) != len(solved):
        raise ValueError("controller solve IDs were reused")
    if submission_stride is not None:
        submitted_ids = [e["solve_id"] for e in accepted_submissions]
        if len(set(submitted_ids)) != len(submitted_ids) or any(e["solve_id"] not in submitted_ids for e in solved):
            raise ValueError("controller result has no unique accepted submission")
    for record in solved:
        if record.get("status") not in ("command", "stale_rejected"):
            continue  # failed solves remain preserved, and cannot be applied.
        state = by_state[int(record["input_state_id"])]
        if not np.allclose(record["input_pose"], [float(state[k]) for k in ("x", "y", "yaw")], rtol=0, atol=1e-10):
            raise ValueError("controller solve input does not match its actual state")
        if not np.isclose(record["input_sim_time_s"], float(state["sim_time_s"]), rtol=0, atol=1e-10):
            raise ValueError("controller solve input simulation time mismatch")
        start, end = record["solve_start_host_monotonic_s"], record["solve_end_host_monotonic_s"]
        if start < record["input_host_monotonic_s"] or end < start:
            raise ValueError("controller solve clock ordering is invalid")
        selected = selection_audit(worlds[record["chunk_id"]], record["input_pose"],
                                   record["selection"]["reference_world"],
                                   horizon=settings["HORIZON"], weights=settings["Q_WEIGHTS"])
        equal_record(record["selection"], selected, "official controller selected reference")
    for command in commands:
        solve_id = command.get("solve_id")
        if solve_id in (None, "", "None"):
            if float(command["v_mps"]) != 0 or float(command["omega_radps"]) != 0:
                raise ValueError("nonzero command was applied without a controller solve")
            continue
        record = by_solve.get(solve_id)
        if record is None or record.get("status") != "command" or not record.get("command_available_for_application"):
            raise ValueError("unavailable/stale controller solve was applied")
        if str(command["reference_version"]) != str(record["reference_version"]) or command["chunk_id"] != record["chunk_id"]:
            raise ValueError("applied command has the wrong controller reference generation")
        if record["result_generation"] != record["official_generation"]:
            raise ValueError("a stale official generation was accepted")
        if float(command["host_monotonic_s"]) < record["solve_end_host_monotonic_s"]:
            raise ValueError("command was applied before its solve completed")
        expected = [0, 0] if command.get("reason") == "controller_timeout" else record["command"]
        if not np.allclose([float(command["v_mps"]), float(command["omega_radps"])], expected, rtol=0, atol=1e-12):
            raise ValueError("applied command differs from the official solve/explicit timeout")
    report = {"solve_result_count": len(solved),
            "stale_result_rejected_count": sum(r.get("status") == "stale_rejected" for r in solved),
            "applied_command_lineage_valid": True}
    if submission_stride is not None:
        report.update(fixed_control_schedule_valid=True,
                      accepted_submission_count=len(accepted_submissions),
                      control_submission_stride_ticks=submission_stride,
                      minimum_submission_period_sim_s=minimum_submission_sim_s,
                      busy_replies_excluded_from_accepted_submissions=True)
    if foreign_results:
        report["foreign_episode_result_count_preserved_and_never_applied"] = len(foreign_results)
    return report


def validate_history_records(records, snapshots, contract, sampler, instruction):
    """Reconstruct transmission chronology and model sampling per prediction."""
    history = SessionHistory(instruction, contract, sampler)
    history.login()
    history.reset()
    prediction_count = 0
    for i, (record, saved) in enumerate(zip(records, snapshots, strict=True)):
        predict = record["kind"] == "prediction"
        pending = history.begin(record["observation"], predict=predict,
                                send_monotonic_ns=int(record["t_request_host"]["monotonic_ns"]))
        if pending["seq"] != record["seq"] or record["seq"] != i:
            raise ValueError("buffer-only/prediction wire seq is not contiguous")
        if record["status"] in ("BUFFERED", "PREDICTION_READY", "MODEL_STOP"):
            step = record["response"]["actions"]["step"] if predict else None
            expected = history.complete(record["seq"], server_actions_step=step)
            equal_record(saved, expected, "history snapshot", atol=0)
            prediction_count += int(predict)
        else:
            # Failed request metadata preserves the causal attempted input.
            expected = history.snapshot()
            expected["delivery_or_prediction_failed"] = True
            equal_record(saved, expected, "failed history snapshot", atol=0)
            if i != len(records)-1:
                raise ValueError("failed session was reused for another request")
    return {"next_count": len(records), "prediction_count": prediction_count,
            "history_valid": True, "selection_scope": "pinned-source reconstruction; internal indices not server telemetry"}


def validate_png(path):
    with Image.open(path) as image:
        if image.format != "PNG" or min(image.size) < 100:
            raise ValueError("trajectory image is missing or not a usable PNG")
        dpi = image.info.get("dpi")
        if dpi is None or min(dpi) < 159.9:
            raise ValueError("trajectory PNG is below 160 dpi")
        image.verify()


def validate_event(episode, context_path, run, *, require_plots=True):
    from plot_robotless_online_handoffs import load_event, prepare_plot_inputs
    context, execution, commands, arrays, metrics, old_anchor, sources = load_event(episode, context_path)
    switch = int(context["switch_state_id"])
    fresh_commands = [c for c in commands if c["chunk_id"] == context["fresh_chunk_id"] and str(c["reference_version"]) == str(context["fresh_reference_version"])]
    if not fresh_commands or int(fresh_commands[0]["application_state_id"]) != switch:
        raise ValueError("event is not the first actual application of this FRESH reference")
    if metrics["inference_state_updates"] <= 0:
        raise ValueError("valid handoff has no actual execution updates during pending inference")
    if not metrics["post_switch_execution_available"]:
        raise ValueError("valid handoff has no actual post-switch execution")
    captures = read_csv(Path(episode) / "rgb_index.csv")
    start_host = context["t_request"]["host_monotonic_ns"]
    ready_host = context["t_ready_host"]["host_monotonic_ns"]
    captured_during = [r for r in captures if int(r["capture_monotonic_ns"]) >= start_host and int(r["capture_monotonic_ns"]) <= ready_host]
    # Short valid inference may fit between camera deadlines, so zero captures
    # is reported as coverage, not a fabricated-cadence assertion or hard error.
    metrics_path = Path(context_path).parent / "metrics.json"
    if metrics_path.exists():
        equal_record(read_json(metrics_path), metrics, "saved event metrics")
    elif require_plots:
        raise ValueError("valid handoff is missing derived metrics")
    if require_plots:
        sources.append(metrics_path)
        expected = prepare_plot_inputs(context, execution, arrays, metrics, old_anchor, sources, run)
        plot_dir = Path(context_path).parent / "plots"
        equal_record(read_json(plot_dir / "plot_inputs.json"), expected, "plot inputs")
        for name in ("trajectory_world.png", "trajectory_boundary_zoom.png"):
            validate_png(plot_dir / name)
    return {"event_id": context["event_id"], "status": context["status"],
            "causal_valid": True, "timing_valid": metrics["timing_valid"],
            "history_full": context["history_full"],
            "inference_state_updates": metrics["inference_state_updates"],
            "inference_capture_count": len(captured_during),
            "post_switch_execution_available": metrics["post_switch_execution_available"],
            "valid_event_png_count": 2 if require_plots else 0}


def validate_episode(episode, run, contract, sampler, *, require_plots=True, integration_dt_s=None):
    episode, run = Path(episode), Path(run)
    if not verify_resume(episode):
        raise ValueError("episode is not complete with a verifiable immutable raw manifest")
    metadata = read_json(episode / "metadata.json")
    execution = read_csv(episode / "execution.csv")
    commands = read_csv(episode / "commands.csv") if (episode / "commands.csv").exists() else []
    if metadata["command_count"] != len(commands) or metadata["state_count"] != len(execution):
        raise ValueError("metadata state/command counts differ from preserved streams")
    integration = validate_execution(execution, commands, integration_dt_s=integration_dt_s)
    by_id = {int(r["state_id"]): r for r in execution}
    capture_records = read_jsonl(episode / "capture.jsonl") if (episode / "capture.jsonl").exists() else []
    if metadata["captured_frames"] != len(capture_records):
        raise ValueError("captured frames were omitted from stream")
    capture_ids = [r["frame_id"] for r in capture_records]
    if len(set(capture_ids)) != len(capture_ids):
        raise ValueError("captured frame IDs were reused")
    for frame in capture_records:
        validate_frame_anchor(frame, by_id)
        if not frame.get("render_state_stable"):
            raise ValueError("RGB readback does not attest a stable rendered state")
        if frame.get("render_sim_time_before_s") != frame.get("render_sim_time_after_s") or not np.isclose(frame["render_sim_time_before_s"], frame["capture_sim_time_s"], rtol=0, atol=1e-10):
            raise ValueError("RGB was paired with a different simulation/render time")
        verify_ref(episode, frame)
    opened, closed = read_json(episode / "session_open.json"), read_json(episode / "session_close.json")
    if opened["episode_id"] != episode.name or closed["episode_id"] != episode.name:
        raise ValueError("session episode identity mismatch")
    if any(closed[k] != 1 for k in ("connection_count", "wire_login_count", "wire_reset_count", "max_outstanding")):
        raise ValueError("episode must use one session/login/reset with maximum one outstanding request")
    if closed["reconnect_count"] or closed["retry_count"] or not closed["instruction_constant"]:
        raise ValueError("episode connection retried/reconnected or instruction changed")
    equal_record(opened["history_contract"], contract, "active history contract", atol=0)
    wire = read_jsonl(episode / "requests/wire.jsonl")
    sent = [r for r in wire if r["direction"] == "request"]
    if [r["action"] for r in sent[:2]] != ["login", "reset"] or any(r["action"] != "next" for r in sent[2:]):
        raise ValueError("wire protocol does not have exactly one initial login/reset")
    if [r["wire_id"] for r in sent] != list(range(len(sent))):
        raise ValueError("wire IDs must be unique including login/reset")
    if [r["seq"] for r in sent[2:]] != list(range(len(sent)-2)):
        raise ValueError("next seq must include every buffer-only append")
    for record in wire:
        verify_ref(episode, record["raw"])
    response_by_wire = {r["wire_id"]: r for r in wire if r["direction"] == "response"}
    if len(response_by_wire) != len([r for r in wire if r["direction"] == "response"]):
        raise ValueError("more than one response recorded for a wire request")
    records = [read_json(p) for p in sorted((episode / "requests").glob("seq_*_metadata.json"))]
    snapshots = []
    import base64
    for i, record in enumerate(records):
        observation = record["observation"]
        if observation["frame_id"] not in capture_ids:
            raise ValueError("transmitted RGB was never captured in this episode")
        validate_frame_anchor(observation, by_id)
        jpeg_path = episode / observation["path"]
        import hashlib
        if hashlib.sha256(jpeg_path.read_bytes()).hexdigest() != observation["sha256"]:
            raise ValueError("history capture hash changed")
        request = read_json(verify_ref(episode, sent[i+2]["raw"]))
        data = request["data"]
        if base64.b64decode(data["image"]) != jpeg_path.read_bytes():
            raise ValueError("wire input differs from this episode's captured JPEG")
        wanted = opened["instruction"] if record["kind"] == "prediction" else ""
        if data["instruction"] != wanted:
            raise ValueError("wire instruction changed or frame was duplicated for prediction")
        equal_record(record["t_request_host"], sent[i+2]["host"], "actual request send time", atol=0)
        response_wire = response_by_wire.get(sent[i+2]["wire_id"])
        if record.get("t_ready_host") is not None:
            if response_wire is None:
                raise ValueError("recorded readiness has no complete wire response")
            equal_record(record["t_ready_host"], response_wire["host"], "actual complete response time", atol=0)
            rtt = (record["t_ready_host"]["monotonic_ns"] - record["t_request_host"]["monotonic_ns"]) / 1e9
            if rtt < 0 or not np.isclose(record["client_rtt_s"], rtt, rtol=0, atol=1e-12):
                raise ValueError("client RTT does not subtract same-host monotonic timestamps")
        snapshots.append(read_json(verify_ref(episode, record["history_snapshot"])))
        if record["kind"] == "prediction" and record["status"] in ("PREDICTION_READY", "MODEL_STOP"):
            local = np.load(verify_ref(episode, record["raw_local_ref"]), allow_pickle=False)
            world = np.load(verify_ref(episode, record["world_ref"]), allow_pickle=False)
            response = read_json(verify_ref(episode, record["response_ref"]))["data"]
            if not np.array_equal(local, np.asarray(response["actions"]["actions"], dtype=np.float64)):
                raise ValueError("raw local array differs from original response numeric values")
            if not np.allclose(world, observation_to_world(observation["pose_world"], local), rtol=0, atol=1e-10):
                raise ValueError("world prediction is not in its own capture frame")
    if closed["next_count"] != len(records) or len(sent) != len(records)+2:
        raise ValueError("attempted wire requests were omitted from request records")
    history_report = validate_history_records(records, snapshots, contract, sampler, opened["instruction"])
    worlds = {r["chunk_id"]: np.load(verify_ref(episode, r["world_ref"]), allow_pickle=False)
              for r in records if r["kind"] == "prediction" and r.get("world_ref")}
    execution_config = metadata["execution"]
    fixed_schedule = execution_config.get("submit_on_install") is False
    controller = validate_controller_records(execution, commands,
                    read_jsonl(episode / "controller/events.jsonl"), worlds,
                    metadata["mpc_worker"]["provenance"]["official_settings"],
                    submission_stride=round(execution_config["integration_hz"] / execution_config["control_hz"]) if fixed_schedule else None,
                    minimum_submission_sim_s=1 / execution_config["control_hz"] if fixed_schedule else None,
                    episode_id=episode.name)
    contexts = [read_json(p) for p in sorted((episode / "handoffs").glob("*/context.json"))]
    reports = []
    previous_valid = None
    for context in contexts:
        if context["status"] not in STATUSES:
            raise ValueError("unknown event status")
        if context["status"] in VALID_STATUSES:
            if previous_valid is not None and context["old_chunk_id"] != previous_valid["fresh_chunk_id"]:
                raise ValueError("successive event OLD is not the previous activated FRESH")
            reports.append(validate_event(episode, episode / "handoffs" / context["event_id"] / "context.json", run, require_plots=require_plots))
            previous_valid = context
        else:
            reports.append({"event_id": context["event_id"], "status": context["status"], "invalid_attempt_preserved": True})
    completion = read_json(episode / "completion.json")
    return {"episode_id": episode.name, "valid": True, "integration": integration,
            "history": history_report, "controller": controller,
            "captured_frame_count": len(capture_records), "events": reports,
            "terminal_status": completion.get("status", completion.get("terminal_status")),
            "terminal_attempts_preserved": True, "raw_immutability_valid": True}


def validate_run(run, *, require_complete=True, require_plots=True):
    run = Path(run).resolve()
    errors, reports = [], []
    try:
        config = load_config(run / "config_snapshot.yaml")
        provenance = read_json(run / "provenance.json")
        for key in ("config", "protocol", "schedule"):
            verify_ref(run, provenance[key])
        schedule = read_json(run / "episode_schedule.json")
        expected = [e["episode_id"] for e in schedule["episodes"]]
        if len(set(expected)) != len(expected):
            raise ValueError("schedule episode IDs are not unique")
        primary = schedule["run_kind"] == "primary"
        if primary and (len(expected) != 60 or len({e["condition_id"] for e in schedule["episodes"]}) != 30):
            raise ValueError("primary must freeze the full 30 x 2 schedule")
        if primary and any(Counter(e["condition_id"] for e in schedule["episodes"])[e["condition_id"]] != 2 for e in schedule["episodes"]):
            raise ValueError("primary starting conditions must have two repetitions")
        actual = {p.name for p in (run / "episodes").iterdir() if p.is_dir()}
        if actual - set(expected):
            raise ValueError("unscheduled episodes were collected")
        if require_complete and actual != set(expected):
            raise ValueError("frozen schedule is not complete")
        def resolve(value):
            path = Path(value).expanduser()
            return path.resolve() if path.is_absolute() else (ROOT / path).resolve()
        contract, sampler = history_contract(resolve(config["paths"]["lightnav_checkout"]), resolve(config["paths"]["checkpoint_path"]))
        execution = config.get("execution", {})
        rate = execution.get("integration_hz", execution.get("integration_rate_hz", 60.0))
        for episode_id in expected:
            if episode_id not in actual:
                continue
            try:
                reports.append(validate_episode(run / "episodes" / episode_id, run, contract, sampler,
                                                require_plots=require_plots, integration_dt_s=1 / float(rate)))
            except Exception as exc:
                errors.append({"episode_id": episode_id, "error": f"{type(exc).__name__}: {exc}"})
        valid_events = [e for episode in reports for e in episode["events"] if e["status"] in VALID_STATUSES]
        limitations = []
        if len(valid_events) < 500:
            limitations.append("Fewer than 500 valid online handoff events; fixed episodes are not replaced.")
        if not valid_events or not all(e["timing_valid"] for e in valid_events):
            limitations.append("Not every valid causal event meets real-time pacing criteria.")
        if not valid_events or not all(e["history_full"] for e in valid_events):
            limitations.append("Nominal history-full coverage is incomplete; actual counts are reported.")
        result = {"schema_version": 1, "run_kind": schedule["run_kind"],
                  "valid": not errors, "causal_valid": not errors and bool(valid_events),
                  "schedule_complete": actual == set(expected), "episodes_planned": len(expected),
                  "episodes_attempted": len(actual), "episodes_validated": len(reports),
                  "valid_handoff_count": len(valid_events),
                  "valid_event_png_count": sum(e["valid_event_png_count"] for e in valid_events),
                  "history_full_valid_count": sum(e["history_full"] for e in valid_events),
                  "timing_valid_count": sum(e["timing_valid"] for e in valid_events),
                  "history_contract": contract, "episodes": reports, "errors": errors, "limitations": limitations}
        result["episode_terminal_status_counts"] = dict(Counter(r["terminal_status"] for r in reports))
        if errors or not valid_events:
            result["status"] = "ROBOTLESS_ONLINE_HANDOFF_RUNTIME_NOT_VALIDATED"
        elif primary and actual == set(expected):
            result["status"] = "ROBOTLESS_ONLINE_HANDOFF_DATASET_COLLECTED_WITH_LIMITATIONS" if limitations else "ROBOTLESS_ONLINE_HANDOFF_DATASET_COLLECTED"
        else:
            result["status"] = "ONLINE_TECHNICAL_SMOKE_VALIDATED" if len(valid_events) >= 3 else "ROBOTLESS_ONLINE_HANDOFF_RUNTIME_NOT_VALIDATED"
        return result
    except Exception as exc:
        return {"valid": False, "causal_valid": False,
                "status": "ROBOTLESS_ONLINE_HANDOFF_RUNTIME_NOT_VALIDATED",
                "errors": [{"error": f"{type(exc).__name__}: {exc}"}]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--allow-partial", action="store_true")
    parser.add_argument("--without-plots", action="store_true")
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    result = validate_run(args.run, require_complete=not args.allow_partial, require_plots=not args.without_plots)
    if args.write:
        for episode_result in result.get("episodes", []):
            episode_validation = args.run / "episodes" / episode_result["episode_id"] / "validation.json"
            if episode_validation.exists():
                equal_record(read_json(episode_validation), episode_result, "existing episode validation")
            else:
                dump_new(episode_validation, episode_result)
        path = args.run / "validation.json"
        if path.exists():
            equal_record(read_json(path), result, "existing validation")
        else:
            dump_new(path, result)
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

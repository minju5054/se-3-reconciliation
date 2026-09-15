"""Synthetic validator corruption tests, separate from live runtime evidence."""

from __future__ import annotations

from copy import deepcopy
import importlib.util
from pathlib import Path

import numpy as np
from PIL import Image
import pytest

from reconciliation.online_history import SessionHistory

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("test_online_validator", ROOT / "scripts/validate_robotless_online_handoffs.py")
validator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(validator)


def streams():
    # Analytic unit linear motion followed by rotation-only motion. Last row is
    # the actual terminal state, not another predicted waypoint.
    poses = [[0, 0, 0], [.1, 0, 0], [.1, 0, .1], [.1, 0, .2]]
    execution = [{"state_id": i, "tick": i, "sim_time_s": i / 10,
                  "host_monotonic_ns": i * 100_000_000, "host_monotonic_s": i / 10,
                  "episode_time_s": i / 10, "x": p[0], "y": p[1], "yaw": p[2]}
                 for i, p in enumerate(poses)]
    commands = [{"application_state_id": i, "application_tick": i, "sim_time_s": i / 10,
                 "v_mps": 1 if i == 0 else 0, "omega_radps": 0 if i == 0 else 1,
                 "chunk_id": "chunk_000" if i < 2 else "chunk_001",
                 "reference_version": 0 if i < 2 else 1}
                for i in range(3)]
    return execution, commands


def test_linear_and_rotation_only_execution_are_valid():
    result = validator.validate_execution(*streams(), integration_dt_s=.1)
    assert result["state_count"] == 4
    assert result["integration_count"] == 3
    assert result["maximum_pose_reconstruction_error"] < 1e-14


def test_command_switch_does_not_require_different_numeric_command():
    execution, commands = streams()
    assert commands[1]["omega_radps"] == commands[2]["omega_radps"]
    assert commands[1]["reference_version"] != commands[2]["reference_version"]
    assert validator.validate_execution(execution, commands)["integration_count"] == 3


@pytest.mark.parametrize("mutation", [
    lambda e, c: e[2].update(x=2),
    lambda e, c: e[2].update(yaw=1),
    lambda e, c: c[1].update(v_mps=2),
    lambda e, c: c.pop(1),
    lambda e, c: c.append(deepcopy(c[0])),
    lambda e, c: c[1].update(application_tick=99),
    lambda e, c: e[2].update(sim_time_s=.1),
    lambda e, c: e[2].update(host_monotonic_ns=1),
])
def test_stream_tampering_fails_reconstruction(mutation):
    execution, commands = streams()
    mutation(execution, commands)
    with pytest.raises(ValueError):
        validator.validate_execution(execution, commands, integration_dt_s=.1)


def test_capture_must_use_the_actual_rendered_state_not_latest_send_pose():
    execution, _ = streams()
    frame = {"rendered_state_id": 1, "pose_world": [.1, 0, 0], "capture_sim_time_s": .1}
    lookup = {r["state_id"]: r for r in execution}
    validator.validate_frame_anchor(frame, lookup)
    frame["pose_world"] = [.1, 0, .2]
    with pytest.raises(ValueError, match="rendered state"):
        validator.validate_frame_anchor(frame, lookup)


def test_capture_time_must_match_rendered_simulation_tick():
    execution, _ = streams()
    with pytest.raises(ValueError, match="simulation time"):
        validator.validate_frame_anchor({"rendered_state_id": 1, "pose_world": [.1, 0, 0],
                                         "capture_sim_time_s": .3}, {r["state_id"]: r for r in execution})


def control_fixture():
    from reconciliation.online_mpc_adapter import selection_audit
    execution, commands = streams()
    worlds = {"chunk_000": np.array([[0, 0, 0], [.2, 0, .1]]),
              "chunk_001": np.array([[.1, 0, .1], [.1, 0, .5]])}
    events = []
    for i, command in enumerate(commands):
        solve_id = f"solve_{i}"
        command.update(solve_id=solve_id, host_monotonic_s=i / 10 + .02, reason="new_solve")
        p = [execution[i][k] for k in ("x", "y", "yaw")]
        path = worlds[command["chunk_id"]]
        reference = path[-1:].copy()
        events.append({"type": "solve_result", "status": "command", "solve_id": solve_id,
                       "input_state_id": i, "input_pose": p, "input_sim_time_s": i / 10,
                       "input_host_monotonic_s": i / 10,
                       "solve_start_host_monotonic_s": i / 10 + .001,
                       "solve_end_host_monotonic_s": i / 10 + .01,
                       "chunk_id": command["chunk_id"], "reference_version": command["reference_version"],
                       "command_available_for_application": True, "result_generation": i,
                       "official_generation": i, "command": [command["v_mps"], command["omega_radps"]],
                       "selection": selection_audit(path, p, reference, horizon=1, weights=[1, 1, 1])})
    return execution, commands, events, worlds, {"HORIZON": 1, "Q_WEIGHTS": [1, 1, 1]}


def test_applied_commands_match_actual_official_controller_solves():
    result = validator.validate_controller_records(*control_fixture())
    assert result["solve_result_count"] == 3
    assert result["applied_command_lineage_valid"]


@pytest.mark.parametrize("mutation", [
    lambda e, c, s: c[1].update(reference_version=99),
    lambda e, c, s: c[1].update(v_mps=.5),
    lambda e, c, s: c[1].update(host_monotonic_s=0),
    lambda e, c, s: s[1].update(status="stale_rejected"),
    lambda e, c, s: s[1].update(result_generation=999),
    lambda e, c, s: s[1].update(input_pose=[9, 9, 9]),
    lambda e, c, s: s[1]["selection"].update(indices=[0]),
])
def test_controller_lineage_and_stale_solve_corruption_rejected(mutation):
    execution, commands, events, worlds, settings = control_fixture()
    mutation(execution, commands, events)
    with pytest.raises(ValueError):
        validator.validate_controller_records(execution, commands, events, worlds, settings)


def test_fixed_control_grid_ignores_busy_replies_and_accepts_nominal_10hz():
    execution, commands, events, worlds, settings = control_fixture()
    for row in execution:
        row["state_id"] *= 6
        row["tick"] *= 6
    for command in commands:
        command["application_state_id"] *= 6
    for event in events:
        event["input_state_id"] *= 6
    submissions = [{**e, "type": "reply", "status": "submitted"} for e in events]
    report = validator.validate_controller_records(execution, commands,
              submissions + [{"type": "reply", "status": "busy", "solve_id": "busy_without_state"}] + events,
              worlds, settings, submission_stride=6, minimum_submission_sim_s=.1)
    assert report["fixed_control_schedule_valid"]
    assert report["accepted_submission_count"] == 3


@pytest.mark.parametrize("state_id,sim", [(1, .1), (6, .05)])
def test_install_time_off_grid_or_extra_fast_submission_is_rejected(state_id, sim):
    rows = [{"state_id": 0, "tick": 0, "sim_time_s": 0},
            {"state_id": state_id, "tick": state_id, "sim_time_s": sim}]
    events = [{"type": "reply", "status": "submitted", "input_state_id": 0, "input_sim_time_s": 0},
              {"type": "reply", "status": "submitted", "input_state_id": state_id, "input_sim_time_s": sim}]
    with pytest.raises(ValueError, match="grid|control rate"):
        validator.validate_controller_records(rows, [], events, {}, {},
                                              submission_stride=6, minimum_submission_sim_s=.1)


def test_late_foreign_episode_result_is_preserved_and_never_applied():
    execution, commands, events, worlds, settings = control_fixture()
    for event in events:
        event["episode_id"] = "current"
    foreign = {**deepcopy(events[0]), "episode_id": "previous", "solve_id": "previous_solve",
               "input_state_id": 999, "status": "stale_rejected"}
    result = validator.validate_controller_records(execution, commands, events + [foreign], worlds, settings,
                                                   episode_id="current")
    assert result["foreign_episode_result_count_preserved_and_never_applied"] == 1
    commands[0]["solve_id"] = "previous_solve"
    with pytest.raises(ValueError, match="unavailable|stale"):
        validator.validate_controller_records(execution, commands, events + [foreign], worlds, settings,
                                              episode_id="current")


def histories():
    contract = {"num_history_frames": 4, "history_storage": "ring", "pool_enable": False,
                "pool_spatial": 1}
    history = SessionHistory("Go ahead", contract)
    history.login(); history.reset()
    records, snapshots = [], []
    for i in range(4):
        observation = {"frame_id": i, "path": f"rgb/{i}.jpg", "sha256": "0" * 64,
                       "rendered_state_id": i, "pose_world": [0, 0, 0],
                       "capture_monotonic_ns": i * 100, "capture_sim_time_s": i / 4,
                       "bootstrap": True, "stationary": True}
        history.begin(observation, predict=i == 3, send_monotonic_ns=i * 100 + 10)
        snapshots.append(history.complete(i, server_actions_step=4 if i == 3 else None))
        records.append({"seq": i, "kind": "prediction" if i == 3 else "buffer_only",
                        "observation": observation, "t_request_host": {"monotonic_ns": i * 100 + 10},
                        "status": "PREDICTION_READY" if i == 3 else "BUFFERED",
                        "response": {"actions": {"step": 4}} if i == 3 else None})
    return records, snapshots, contract


def test_history_reconstructed_from_chronological_wire_inputs():
    records, snapshots, contract = histories()
    result = validator.validate_history_records(records, snapshots, contract, None, "Go ahead")
    assert result["history_valid"]
    assert result["next_count"] == 4 and result["prediction_count"] == 1


@pytest.mark.parametrize("mutation", [
    lambda r, s: r[3].update(seq=0),
    lambda r, s: r[3]["observation"].update(frame_id=0),
    lambda r, s: r[3]["observation"].update(capture_monotonic_ns=999),
    lambda r, s: s[3]["chronological_history_frame_ids"].reverse(),
    lambda r, s: s[3].update(actual_history_count=64),
    lambda r, s: s[3].update(internal_selection_directly_observed=True),
    lambda r, s: s[3]["model_input_segments_reconstructed"][0]["frame_ids"].append(99),
])
def test_history_leakage_or_claim_tampering_is_rejected(mutation):
    records, snapshots, contract = histories()
    mutation(records, snapshots)
    with pytest.raises(ValueError):
        validator.validate_history_records(records, snapshots, contract, None, "Go ahead")


def test_comparison_rejects_geometry_scaling_and_missing_coordinates():
    expected = {"displayed": {"OLD": [[1, 2, .3]], "B": [2, 3, .4]}}
    validator.equal_record(deepcopy(expected), expected)
    corrupted = deepcopy(expected)
    corrupted["displayed"]["OLD"][0][0] *= 2
    with pytest.raises(ValueError, match="numeric"):
        validator.equal_record(corrupted, expected)
    del corrupted["displayed"]["B"]
    with pytest.raises(ValueError, match="fields"):
        validator.equal_record(corrupted, expected)


def test_float_tolerance_never_accepts_nonfinite_geometry():
    with pytest.raises(ValueError):
        validator.equal_record([float("nan")], [0.0])


def test_png_coverage_requires_real_png_and_160_dpi(tmp_path):
    image = tmp_path / "trajectory_world.png"
    Image.new("RGB", (160, 160)).save(image, dpi=(160, 160))
    validator.validate_png(image)
    image2 = tmp_path / "low_dpi.png"
    Image.new("RGB", (160, 160)).save(image2, dpi=(96, 96))
    with pytest.raises(ValueError, match="160 dpi"):
        validator.validate_png(image2)
    with pytest.raises(FileNotFoundError):
        validator.validate_png(tmp_path / "missing.png")


def test_missing_online_source_fails_closed_without_fixture_fallback(tmp_path):
    result = validator.validate_run(tmp_path)
    assert result["valid"] is False
    assert result["status"] == "ROBOTLESS_ONLINE_HANDOFF_RUNTIME_NOT_VALIDATED"
    assert not list(tmp_path.iterdir())

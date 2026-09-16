"""Synthetic numerical checks, not experimental trajectory evidence."""
import importlib.util
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location(
    "trajectory_mix", Path(__file__).parents[1] / "scripts/analyze_robotless_online_trajectory_mix.py")
mix = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mix)


def test_forward_line_and_raw_input_are_preserved():
    path = np.array([[.8, .1, 0], [1.8, .1, 0]])
    saved = path.copy()
    shape = mix.path_shape(path)
    assert shape["arc_m"] == pytest.approx(1.)  # No fabricated origin segment.
    assert shape["chord_over_arc"] == pytest.approx(1.)
    assert mix.raw_straight(shape, 5)
    np.testing.assert_array_equal(path, saved)


def test_lateral_line_requires_reorientation_from_capture_heading():
    shape = mix.path_shape([[0, .1, np.pi/2], [0, 1.1, np.pi/2]])
    assert shape["internal_tangent_turn_deg"] == pytest.approx(0.)
    assert shape["yaw_travel_from_observation_deg"] == pytest.approx(90.)
    assert not mix.raw_straight(shape, 5)


def test_yaw_wrap_uses_shortest_increment():
    shape = mix.path_shape([[0, 0, np.deg2rad(179)], [1, 0, np.deg2rad(-179)]])
    assert shape["yaw_travel_from_observation_deg"] == pytest.approx(181.)


def test_rotation_only_is_not_straight_translation():
    shape = mix.path_shape([[0, 0, 0], [0, 0, np.pi/2]])
    assert shape["arc_m"] == 0
    assert shape["max_forward_direction_deviation_deg"] is None
    assert not mix.raw_straight(shape, 180)


def test_right_angle_has_expected_arc_chord_and_turn():
    shape = mix.path_shape([[0, 0, 0], [1, 0, 0], [1, 1, np.pi/2]])
    assert shape["arc_m"] == 2
    assert shape["chord_over_arc"] == pytest.approx(1/np.sqrt(2))
    assert shape["internal_tangent_turn_deg"] == pytest.approx(90)
    assert not mix.raw_straight(shape, 180)


def test_executed_segment_thresholds_keep_rotation_and_curvature():
    post = {"translation_m": 1., "net_translation_m": 1., "yaw_travel_rad": np.deg2rad(6)}
    assert not mix.executed_straight(post, 5)
    assert mix.executed_straight(post, 10)
    assert not mix.executed_straight({**post, "net_translation_m": .9}, 10)
    assert not mix.executed_straight({**post, "translation_m": 0., "net_translation_m": 0.}, 10)


@pytest.mark.parametrize("v,w_deg,expected", [
    (.8, 0., "straight_translation"), (.8, 5., "straight_translation"),
    (.8, 6., "translating_turn"), (0., 30., "rotation_low_translation"),
    (0., 0., "low_motion"), (-.8, 0., "straight_translation"),
])
def test_command_class_has_explicit_velocity_units(v, w_deg, expected):
    assert mix.command_class(v, np.deg2rad(w_deg)) == expected


def test_recorded_simulation_duration_weighting_excludes_bootstrap(tmp_path):
    run = tmp_path / "synthetic_run"
    episode = run / "episodes/e0"
    event = episode / "handoffs/h0"
    event.mkdir(parents=True)
    raw = episode / "raw.npy"
    np.save(raw, np.array([[.1, 0., 0.], [1.1, 0., 0.]]))
    raw_hash = hashlib.sha256(raw.read_bytes()).hexdigest()
    for path, value in [
        (run / "validation.json", {"valid": True, "schedule_complete": True, "valid_handoff_count": 1}),
        (episode / "metadata.json", {"category": "synthetic_test"}),
        (episode / "completion.json", {}),
        (event / "context.json", {"fresh_raw_local_ref": {"path": "raw.npy", "sha256": raw_hash}}),
        (event / "metrics.json", {"status": "VALID_HANDOFF_MOVING", "event_id": "h0",
            "post_switch_execution": {"translation_m": 1., "net_translation_m": 1.,
                "yaw_travel_rad": 0., "duration_sim_s": .4}}),
    ]:
        path.write_text(json.dumps(value))
    (episode / "execution.csv").write_text(
        "state_id,sim_time_s,x,y\n0,0,0,0\n1,0.2,0,0\n2,0.3,0.1,0\n"
        f"3,0.6,{.1 + np.sin(.3)},{1-np.cos(.3)}\n")
    (episode / "commands.csv").write_text(
        "chunk_id,application_state_id,v_mps,omega_radps\n,0,0,0\nc0,1,1,0\nc0,2,1,1\n")
    result = mix.analyze(run, tmp_path / "derived")
    periods = result["active_execution_command_mix"]
    assert periods["straight_translation"]["time_fraction"] == pytest.approx(.25)
    assert periods["translating_turn"]["time_fraction"] == pytest.approx(.75)
    assert sum(p["sim_s"] for p in periods.values()) == pytest.approx(.4)
    assert sum(p["distance_m"] for p in periods.values()) == pytest.approx(.1 + 2*np.sin(.15))
    assert hashlib.sha256(raw.read_bytes()).hexdigest() == raw_hash
    with pytest.raises(FileExistsError):
        mix.analyze(run, tmp_path / "derived")
    with pytest.raises(ValueError, match="outside"):
        mix.analyze(run, run / "new_analysis")

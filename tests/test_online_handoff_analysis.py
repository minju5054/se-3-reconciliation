"""Deterministic synthetic pure/mock tests, never online experimental evidence."""
import copy
import csv
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image
import pytest

from reconciliation.online_handoff_analysis import (
    compute_event_metrics, distribution, incoming_execution_direction,
    metrics_from_records, motion_in_range, ordered_pair_sha256,
    projection_at_boundary, summarize_events,
)
from reconciliation.robotless_single_chunk import observation_to_world

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
import plot_robotless_online_handoffs as plotter


def arrays():
    # Actual E(t) goes north, while raw OLD runs east: incoming must use E(t).
    poses = np.column_stack((np.zeros(11), np.arange(11)*.02, np.ones(11)*np.pi/2))
    times = np.arange(11)*.1
    old = np.array([[-1., .12, 0.], [1., .12, 0.]])
    fresh = np.array([[-.5, .12, 3.], [.5, .12, -3.]])
    return old, fresh, poses, times


def metric_fixture(**changes):
    old, fresh, poses, times = arrays()
    kwargs = dict(request_index=2, ready_index=5, switch_index=6,
        observation_sim_s=.1, request_host_monotonic_s=100.2,
        ready_host_monotonic_s=100.5, post_end_index=9,
        previous_command=[.2, -.1], first_fresh_command=[.21, .1],
        host_times=100+times, actual_history_count=7)
    kwargs.update(changes)
    return compute_event_metrics(old, fresh, poses, times, **kwargs)


def canonical_fixture():
    old, fresh, poses, times = arrays()
    execution = [{"state_id": i*2, "tick": i, "sim_time_s": float(t),
        "host_monotonic_s": 100+float(t), "x": float(p[0]), "y": float(p[1]), "yaw": float(p[2])}
        for i, (p, t) in enumerate(zip(poses, times))]
    commands = [{"command_id": f"command_{i}", "application_state_id": i*2,
        "reference_version": 0 if i < 6 else 1 if i < 9 else 2,
        "chunk_id": "chunk_000" if i < 6 else "chunk_001" if i < 9 else "chunk_002",
        "v_mps": .2, "omega_radps": 0., "solve_id": i}
        for i in range(10)]
    context = {
        "episode_id": "synthetic_episode", "event_id": "handoff_000",
        "status": "VALID_HANDOFF_MOVING", "old_chunk_id": "chunk_000", "fresh_chunk_id": "chunk_001",
        "old_reference_version": 0, "fresh_reference_version": 1,
        "obs_state_id": 2, "request_state_id": 4, "ready_state_id": 10, "switch_state_id": 12,
        "previous_state_id": 10, "post_switch_end_state_id": 18,
        "R_obs": poses[1].tolist(), "R_ready": poses[5].tolist(), "P": poses[5].tolist(), "B": poses[6].tolist(),
        "t_obs": {"sim_time_s": .1}, "t_request": {"sim_time_s": .2, "host_monotonic_s": 100.2},
        "t_ready_host": {"host_monotonic_s": 100.5}, "t_ready_seen_sim": {"sim_time_s": .5}, "t_switch": {"sim_time_s": .6},
        "actual_history_count": 7, "history_full": False, "history_config": {"num_history_frames": 64, "sampling": "fixture only"},
        "timing_flags": {"timing_valid": True, "history_valid": True, "overlap_observed": True},
        "old_raw_local_ref": {"sha256": "a"*64}, "fresh_raw_local_ref": {"sha256": "b"*64},
    }
    return context, execution, commands, old, fresh


def test_actual_incoming_is_not_body_yaw_or_predicted_old_tangent():
    m = metric_fixture()
    assert m["incoming_execution"]["phi_rad"] == pytest.approx(np.pi/2)
    assert m["old_reference_projection_diagnostic"]["phi_local_rad"] == pytest.approx(0)
    assert m["abs_e_dir_local_deg"] == pytest.approx(90)
    assert m["abs_e_dir_window_deg"] == pytest.approx(90)
    assert m["e_perp_m"] == pytest.approx(0)
    assert m["d_first_row_m"] == pytest.approx(.5)
    assert m["abs_e_yaw_deg"] == pytest.approx(90)


def test_clock_domains_and_recorded_ranges_are_explicit():
    m = metric_fixture()
    assert m["client_rtt_s"] == pytest.approx(.3)
    assert m["observation_to_ready_sim_s"] == pytest.approx(.4)
    assert m["observation_to_switch_sim_s"] == pytest.approx(.5)
    assert m["ready_to_switch_sim_s"] == pytest.approx(.1)
    assert m["inference_rtf"] == pytest.approx(1)
    assert m["inference_state_updates"] == 3
    assert m["inference_translation_m"] == pytest.approx(.06)
    assert m["post_switch_execution_duration_s"] == pytest.approx(.3)
    assert m["delta_v_mps"] == pytest.approx(.01)
    assert m["delta_omega_radps"] == pytest.approx(.2)
    assert m["B_stream_row"] == 6 and m["P_stream_row"] == 5


def test_rtf_uses_matched_sample_clock_domains_not_observation_age():
    m = metric_fixture(host_times=300+np.arange(11)*.2, request_host_monotonic_s=300.4, ready_host_monotonic_s=301.)
    assert m["inference_rtf"] == pytest.approx(.5)
    assert m["client_rtt_s"] == pytest.approx(.6)


def test_rotation_only_has_motion_without_invented_translation_direction():
    poses = np.array([[0, 0, 3.1], [0, 0, -3.1], [0, 0, -3.]])
    times = [0, .1, .2]
    result = motion_in_range(poses, times, 0, 2)
    assert result["translation_m"] == 0
    assert result["yaw_change_rad"] == pytest.approx(2*np.pi-6.1)
    assert result["yaw_travel_rad"] == pytest.approx(2*np.pi-6.1)
    incoming = incoming_execution_direction(poses, times, 2)
    assert incoming["phi_rad"] is None
    assert not incoming["available"]
    assert "numerical resolution" in incoming["unavailable_reason"]


@pytest.mark.parametrize("path", [[[1, 2, 3]], [[1, 2, 3], [1, 2, -3]], [[1, 2, 0], [1, 2, .8], [1, 2, 2.]]])
def test_singleton_and_rotation_only_fresh_have_distance_and_yaw_but_no_tangent(path):
    before = np.array(path)
    result = projection_at_boundary([0, 0, 0], before)
    assert result["e_perp_m"] == pytest.approx(np.sqrt(5))
    assert result["phi_local_rad"] is None
    assert result["normalized_progress"] is None
    assert not result["window_0_10m"]["available"]
    np.testing.assert_array_equal(before, path)


def test_repeated_winning_segment_does_not_borrow_another_direction():
    result = projection_at_boundary([0, 0, .1], [[0, 0, .2], [0, 0, .3], [1, 0, .4]])
    assert result["segment_index"] == 0
    assert result["phi_local_rad"] is None
    assert result["projection_location"] == "degenerate_point"
    assert result["e_yaw_rad"] == pytest.approx(.1)


def test_projection_endpoint_and_shortest_yaw_interpolation():
    result = projection_at_boundary([0, 1, 0], [[-1, 0, 3.1], [1, 0, -3.1]])
    assert result["alpha"] == .5
    assert result["projection_location"] == "interior"
    assert result["theta_Q_rad"] == pytest.approx(-np.pi)
    assert result["window_0_10m"]["span_m"] == pytest.approx(.1)
    endpoint = projection_at_boundary([4, 1, 0], [[-1, 0, 0], [1, 0, 0]])
    assert endpoint["alpha"] == 1
    assert endpoint["projection_location"] == "end_endpoint"
    assert endpoint["window_0_10m"]["span_m"] == pytest.approx(.05)


@pytest.mark.parametrize("length", [1, 2, 8, 17, 101])
def test_arbitrary_path_length_and_own_observation_world_transform(length):
    raw = np.column_stack((np.arange(length)*.1, np.zeros(length), np.zeros(length)))
    anchor = [2, 3, np.pi/2]
    world = observation_to_world(anchor, raw)
    result = projection_at_boundary(anchor, world)
    assert result["e_perp_m"] == pytest.approx(0)
    np.testing.assert_array_equal(raw[:, 1:], 0)


@pytest.mark.parametrize("changes", [dict(request_index=7), dict(ready_index=8), dict(switch_index=50), dict(post_end_index=5),
    dict(observation_sim_s=.4), dict(ready_host_monotonic_s=99), dict(request_host_monotonic_s=float("nan"))])
def test_invalid_timing_or_execution_range_fails_closed(changes):
    with pytest.raises(ValueError): metric_fixture(**changes)


def test_absent_post_roll_never_uses_prediction_as_execution():
    m = metric_fixture(post_end_index=None)
    assert not m["post_switch_execution_available"]
    assert m["post_switch_execution_duration_s"] == 0
    assert "no recorded integration" in m["post_switch_unavailable_reason"]


def test_exact_canonical_state_ids_and_lineage_include_boundary_not_next_displacement():
    c, states, commands, old, fresh = canonical_fixture()
    m = metrics_from_records(c, states, commands, old, fresh)
    assert m["request_state_id"] == 4
    assert m["switch_state_id"] == 12
    assert m["post_switch_end_state_id"] == 18
    assert m["post_switch_execution"]["stream_rows_inclusive"] == [6, 9]
    assert m["execution_range_state_ids"]["post_switch"] == [12, 18]
    assert m["delta_v_mps"] == 0  # Same numbers still constitute a reference handoff.


@pytest.mark.parametrize("mutation", ["B", "P", "R_obs", "R_ready", "previous_state_id", "fresh_command", "old_during_inference", "post_next_reference"])
def test_canonical_stream_disagreement_or_wrong_lineage_rejected(mutation):
    c, states, commands, old, fresh = canonical_fixture()
    if mutation in ("B", "P", "R_obs", "R_ready"): c[mutation][0] += .001
    elif mutation == "previous_state_id": c[mutation] = 8
    elif mutation == "fresh_command": commands[6]["reference_version"] = 0
    elif mutation == "old_during_inference": commands[3]["chunk_id"] = "unrelated"
    elif mutation == "post_next_reference": c["post_switch_end_state_id"] = 20
    with pytest.raises(ValueError): metrics_from_records(c, states, commands, old, fresh)


def test_request_row_can_be_reconstructed_from_client_host_clock():
    c, states, commands, old, fresh = canonical_fixture()
    del c["request_state_id"]
    assert metrics_from_records(c, states, commands, old, fresh)["request_state_id"] == 4


def test_all_geometry_and_actual_execution_inputs_remain_immutable():
    c, states, commands, old, fresh = canonical_fixture()
    before = copy.deepcopy([c, states, commands]), old.copy(), fresh.copy()
    metrics_from_records(c, states, commands, old, fresh)
    assert [c, states, commands] == before[0]
    np.testing.assert_array_equal(old, before[1]); np.testing.assert_array_equal(fresh, before[2])


def test_duplicate_pair_grouping_and_episode_weighting_are_separate():
    rows = []
    for index, (episode, pair, value) in enumerate([("a", "a", 0), ("a", "a", 2), ("a", "b", 10), ("b", "b", 20)]):
        rows.append(dict(episode_id=episode, event_id=str(index), status="VALID_HANDOFF_MOVING",
                         old_raw_sha256=pair*64, fresh_raw_sha256="f"*64, e_perp_m=value))
    result = summarize_events(rows, metric_columns=["e_perp_m"])
    assert result["diversity"]["unique_OLD_count"] == 2
    assert result["diversity"]["unique_FRESH_count"] == 1
    assert result["diversity"]["unique_ordered_pair_count"] == 2
    assert result["diversity"]["dominant_pair_fraction"] == .5
    stats = result["statistics"]
    assert stats["event_descriptive"]["e_perp_m"]["median"] == 6
    assert stats["episode_weighted"]["e_perp_m"]["median"] == 11
    assert stats["duplicate_aware"]["e_perp_m"]["median"] == 8
    with pytest.raises(ValueError): summarize_events(rows+rows[:1])
    assert ordered_pair_sha256("a"*64, "b"*64) != ordered_pair_sha256("b"*64, "a"*64)


def test_invalid_and_stationary_events_are_preserved_without_inventing_geometry():
    rows = [dict(episode_id="a", event_id="0", status="MODEL_STOP"),
            dict(episode_id="a", event_id="1", status="VALID_HANDOFF_STATIONARY", old_raw_sha256="a"*64, fresh_raw_sha256="b"*64, history_full=False)]
    result = summarize_events(rows)
    assert result["attempted_count"] == 2 and result["valid_count"] == 1 and result["invalid_count"] == 1
    assert result["status_counts"] == {"MODEL_STOP": 1, "VALID_HANDOFF_STATIONARY": 1}
    assert result["statistics"]["event_descriptive"]["abs_e_dir_local_deg"]["unavailable_count"] == 1
    assert distribution([None, float("nan"), 1])["available_count"] == 1


def write_dataset_fixture(root):
    c, states, commands, old, fresh = canonical_fixture()
    episode = root/"episodes"/c["episode_id"]
    episode.mkdir(parents=True)
    plotter.write_csv(episode/"execution.csv", states)
    plotter.write_csv(episode/"commands.csv", commands)
    old_anchor = [0., 0., 0.]
    fresh_anchor = c["R_obs"]
    # Express each genuinely distinct fixture path in its own capture agent frame.
    from reconciliation.se2 import inverse_pose
    raw_old = observation_to_world(inverse_pose(old_anchor), old)
    raw_fresh = observation_to_world(inverse_pose(fresh_anchor), fresh)
    for chunk_id, raw, world, anchor in (("chunk_000", raw_old, old, old_anchor), ("chunk_001", raw_fresh, fresh, fresh_anchor)):
        directory = episode/"chunks"/chunk_id
        directory.mkdir(parents=True)
        np.save(directory/"raw_local.npy", raw)
        np.save(directory/"world.npy", world)
        plotter.write_json(directory/"metadata.json", {"observation": {"pose_world": anchor}, "synthetic_only": True})
        prefix = "old" if chunk_id == "chunk_000" else "fresh"
        c[prefix+"_raw_local_ref"] = plotter.source_record(directory/"raw_local.npy", episode)
        c[prefix+"_world_ref"] = plotter.source_record(directory/"world.npy", episode)
    plotter.write_json(episode/"handoffs"/"handoff_000"/"context.json", c)
    return episode, c


def test_every_valid_event_has_two_readable_160dpi_png_and_exact_plot_inputs(tmp_path):
    episode, context = write_dataset_fixture(tmp_path)
    before = {str(p): plotter.sha256(p) for p in episode.rglob("*") if p.is_file()}
    report = plotter.process_run(tmp_path)
    assert report["expected_valid_event_png_count"] == report["actual_valid_event_png_count"] == 2
    directory = episode/"handoffs"/"handoff_000"/"plots"
    inputs = plotter.read_json(directory/"plot_inputs.json")
    np.testing.assert_array_equal(inputs["displayed_coordinates"]["FRESH_raw_world"], np.load(episode/"chunks"/"chunk_001"/"world.npy"))
    assert inputs["stream_ranges_inclusive"]["post_switch"] == [6, 9]
    assert len(inputs["displayed_coordinates"]["post_switch_executed"]) == 4
    for name in ("trajectory_world.png", "trajectory_boundary_zoom.png"):
        with Image.open(directory/name) as image:
            assert image.info["dpi"][0] >= 159.99
            assert image.width >= 1600
            image.verify()
    for path, digest in before.items(): assert plotter.sha256(path) == digest
    assert (tmp_path/"index.html").exists()
    assert (episode/"plots"/"event_timeline.png").exists()
    assert (episode/"plots"/"handoff_contact_sheet_000.png").exists()
    # A no-change resume neither rewrites raw data nor rejects its own CSV.
    assert plotter.process_run(tmp_path) == report


def test_changed_raw_hash_cannot_be_plotted_as_valid(tmp_path):
    episode, _ = write_dataset_fixture(tmp_path)
    path = episode/"chunks"/"chunk_001"/"raw_local.npy"
    np.save(path, np.ones((2, 3)))
    with pytest.raises(ValueError, match="source reference hash"): plotter.process_run(tmp_path)


def test_nonexistent_dataset_has_no_synthetic_fallback(tmp_path):
    with pytest.raises(ValueError, match="missing online"): plotter.process_run(tmp_path)


def test_plot_inputs_resume_refuses_changed_derived_provenance(tmp_path):
    output = tmp_path/"inputs.json"
    plotter.write_json(output, {"raw_sha": "abc"})
    with pytest.raises(ValueError, match="refusing to replace"): plotter.write_json(output, {"raw_sha": "def"})
    assert plotter.read_json(output) == {"raw_sha": "abc"}


def test_unactivated_stop_is_preserved_with_available_execution_diagnostic(tmp_path):
    episode, context = write_dataset_fixture(tmp_path)
    path = episode/"handoffs"/"handoff_000"/"context.json"
    context.update(status="MODEL_STOP", switch_state_id=None, B=None, P=None, fresh_world_ref=None, fresh_raw_local_ref=None)
    path.write_text(json.dumps(context))  # Mutation of a synthetic test fixture only.
    report = plotter.process_run(tmp_path)
    assert report["valid_event_count"] == 0
    assert report["event_count"] == 1
    assert report["actual_all_event_png_count"] == 2
    inputs = plotter.read_json(episode/"handoffs"/"handoff_000"/"plots"/"plot_inputs.json")
    assert inputs["diagnostic_only"] and not inputs["actual_boundary_available"]
    assert inputs["displayed_coordinates"]["B"] is None
    assert plotter.read_json(tmp_path/"aggregate"/"status_counts.json")["status_counts"] == {"MODEL_STOP": 1}


def test_valid_png_replacement_is_detected_on_resume(tmp_path):
    episode, _ = write_dataset_fixture(tmp_path)
    plotter.process_run(tmp_path)
    path = episode/"handoffs"/"handoff_000"/"plots"/"trajectory_world.png"
    Image.new("RGB", (1680, 1328), "white").save(path, dpi=(160, 160))
    with pytest.raises(ValueError, match="changed derived JSON"): plotter.process_run(tmp_path)


def test_inference_motion_excludes_response_receipt_to_loop_seen_interval():
    m = metric_fixture(ready_host_monotonic_s=100.41)
    assert m["inference_execution"]["stream_rows_inclusive"] == [2, 4]
    assert m["inference_translation_m"] == pytest.approx(.04)
    assert m["inference_state_updates"] == 2
    assert m["request_to_ready_seen_execution"]["stream_rows_inclusive"] == [2, 5]
    assert m["request_to_ready_seen_execution"]["translation_m"] == pytest.approx(.06)
    assert m["inference_host_range_exact"]


def test_fast_response_without_enclosed_state_samples_has_no_invented_inflight_motion():
    m = metric_fixture(request_host_monotonic_s=100.201, ready_host_monotonic_s=100.21)
    assert m["inference_execution"]["stream_rows_inclusive"] is None
    assert m["inference_translation_m"] == 0
    assert m["inference_state_updates"] == 0
    assert m["inference_rtf"] is None


@pytest.mark.parametrize("key", ["t_obs", "t_ready_seen_sim", "t_switch"])
def test_canonical_simulation_timestamps_match_their_actual_state_ids(key):
    context, states, commands, old, fresh = canonical_fixture()
    context[key]["sim_time_s"] += .01
    with pytest.raises(ValueError, match="timestamp differs"):
        metrics_from_records(context, states, commands, old, fresh)


def test_interval_timing_counts_actual_commands_submits_captures_and_deadlines_separately():
    from reconciliation.online_handoff_analysis import interval_timing_summary
    context, states, commands, old, fresh = canonical_fixture()
    context["t_ready_host"]["host_monotonic_s"] = 100.41
    context["t_switch"]["host_monotonic_s"] = 100.6
    for i, command in enumerate(commands):
        command.update(host_monotonic_s=100+i*.1, held=i%2 == 1)
    metrics = metrics_from_records(context, states, commands, old, fresh)
    loops = [dict(host_monotonic_s=100+i*.1, loop_interval_host_s=.1 if i != 4 else .3,
                  deadline_missed=i == 4, lateness_s=.2 if i == 4 else 0) for i in range(1, 11)]
    controller = [dict(type="reply", op="submit", status="submitted", submit_host_monotonic_s=100.22,
                      seen_in_isaac={"host_monotonic_s": 100.23}),
                  dict(type="reply", op="submit", status="busy", seen_in_isaac={"host_monotonic_s": 100.3}),
                  dict(type="solve_result", status="command", solve_end_host_monotonic_s=100.24)]
    captures = [dict(frame_id="frame_0", capture_monotonic_ns=100210000000, capture_sim_time_s=.21),
                dict(frame_id="frame_1", capture_monotonic_ns=100400000000, capture_sim_time_s=.4)]
    result = interval_timing_summary(context, metrics, states, commands, loops, controller, captures,
        pacing_config={"rtf_valid_min": .8, "rtf_valid_max": 1.2, "max_loop_stall_valid_s": .25})
    inference = result["inference_client_host_interval"]
    assert inference["simulation_steps_wholly_enclosed"] == 2
    assert inference["actual_applied_command_count"] == 3
    assert inference["new_solve_command_application_count"] == 2
    assert inference["control_submit_attempt_replies_seen_count"] == 2
    assert inference["accepted_control_solve_submissions_count"] == 1
    assert inference["control_busy_submit_replies_seen_count"] == 1
    assert inference["control_solve_completions_count"] == 1
    assert inference["capture_count"] == 2
    assert inference["capture_intervals_sim_s"]["median"] == pytest.approx(.19)
    assert inference["missed_deadline_count"] == 1
    assert inference["maximum_loop_stall_s"] == .3
    assert not result["real_time_pacing_valid"]
    assert result["causal_execution_overlap_observed"]
    assert any("real-time" in warning for warning in result["warnings"])

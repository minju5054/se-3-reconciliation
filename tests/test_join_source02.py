"""Synthetic implementation fixtures only, never genuine source evidence."""
from copy import deepcopy
import ast
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
from shapely.geometry import box

from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.join_source02 import (
    BRANCHES, REQUIRED_SCREENING_CHECKS, declared_conditions, evaluate_screening, jpeg_wire_parity,
    moving_boundary_qualification, observation_anchored_world,
    obstacle_state_check, pointing_diagnostics, response_geometry,
    validate_development_order, validate_paired_inputs, visibility_check,
    whole_raw_polyline_check,
)
from reconciliation.robotless_online import integrate_unicycle


def _frame(i):
    return dict(frame_id=f"frame_{i}", sha256="a"*64, camera_id="camera",
                rendered_state_id=i, capture_sim_time_s=i*.1,
                capture_monotonic_ns=1_000_000_000+i*100_000_000,
                pose_world=[float(i)*.01, 0., 0.])


def _branches():
    frames = [_frame(i) for i in range(16)]
    branches = {}
    for i, key in enumerate(BRANCHES):
        branches[key] = dict(session_id=f"independent_{i}", instruction="Go to the target and stop.",
                             camera_config={"size": [640, 480]}, model_config={"task": "vln"},
                             history=deepcopy(frames[:-1]), final_frame=deepcopy(frames[-1]))
    branches["B_ON"]["final_frame"]["sha256"] = "b"*64
    branches["B_ON"]["final_frame"]["frame_id"] = "new_on_capture"
    return branches


@pytest.fixture
def geometry():
    off = HospitalEnvironment(box(50, 50, 51, 51), box(-10, -10, 10, 10))
    on = HospitalEnvironment(off.obstacles.union(box(1.7, -.2, 2.3, .2)), off.workspace)
    return off, on


def _paths():
    native = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0], [4, 0, 0.]], float)
    bypass = np.array([[0, 0, 0], [.8, 0, 0], [1.2, 1, .8], [2.8, 1, 0], [3.2, 0, -.8], [4, 0, 0]], float)
    return native, bypass


def _screen(geometry, on=None, **extra):
    off, default = _paths()
    kwargs = dict(observation_pose_world=[0, 0, 0], target_xy=[5, 0],
                  environment_off=geometry[0], environment_on=geometry[1],
                  influence=dict(origin_xy=[0, 0], forward_xy=[1, 0], progress_min_m=.5, progress_max_m=3.5),
                  bypass_plane=dict(point_xy=[2.55, 0], forward_xy=[1, 0]),
                  technical_checks={key: True for key in REQUIRED_SCREENING_CHECKS})
    kwargs.update(extra)
    return evaluate_screening(off, default if on is None else on, off, **kwargs)


def test_declaration_fixed_budget_order():
    d = declared_conditions(["location1", "location2"], ["placement1", "placement2"])
    assert len(d) == 12
    assert [x["delivered_frame_count"] for x in d[:3]] == [16, 32, 64]
    assert len(declared_conditions(["location1"], ["placement1"])) == 3
    with pytest.raises(ValueError):
        declared_conditions(["a", "b", "c"], ["x"])
    with pytest.raises(ValueError):
        declared_conditions(["a"], ["x"], [32, 16, 64])


def test_first_complete_qualifier_no_retry_extra_replacement():
    d = declared_conditions(["a"], ["x"])
    rows = [dict(condition_id=d[0]["condition_id"], label="HISTORY_UNAVAILABLE", completed_branches=[]),
            dict(condition_id=d[1]["condition_id"], label="SAFE_BYPASS_CANDIDATE", completed_branches=list(BRANCHES))]
    assert validate_development_order(d, rows)["selected_condition_id"] == d[1]["condition_id"]
    for invalid in (rows[::-1], rows+[rows[-1]], rows+[dict(condition_id=d[2]["condition_id"], label="NO_MATERIAL_RESPONSE")]):
        with pytest.raises(ValueError):
            validate_development_order(d, invalid)
    rows[-1]["completed_branches"] = list(BRANCHES[:2])
    with pytest.raises(ValueError, match="complete"):
        validate_development_order(d, rows)


def test_branch_attempts_are_not_inferred_prediction_calls():
    d = declared_conditions(["a"], ["x"])
    rows = [dict(condition_id=d[0]["condition_id"], label="TECHNICAL_INVALID",
                 completed_branches=list(BRANCHES), call_counts={"terminal_predictions": 2})]
    result = validate_development_order(d, rows)
    assert result["completed_branch_attempt_count"] == 3
    assert result["terminal_prediction_count"] == 2
    del rows[0]["call_counts"]
    assert validate_development_order(d, rows)["terminal_prediction_count"] is None


def test_same_history_independent_sessions_identical_jpeg_not_duplicate_capture():
    branches = _branches()
    assert validate_paired_inputs(branches)["valid"]
    assert len(set(x["sha256"] for x in branches["A_OFF"]["history"])) == 1
    branches["B_ON"]["session_id"] = branches["A_OFF"]["session_id"]
    with pytest.raises(ValueError, match="independent"):
        validate_paired_inputs(branches)


@pytest.mark.parametrize("mutation", ["duplicate", "future", "reverse", "different_history", "different_instruction", "different_final_pose", "different_simtime", "different_sham"])
def test_paired_input_corruption_rejected(mutation):
    b = _branches()
    branch = b["B_ON"]
    if mutation == "duplicate":
        branch["history"][2]["frame_id"] = branch["history"][1]["frame_id"]
    elif mutation == "future":
        branch["history"][-1]["capture_monotonic_ns"] += 100_000_000_000
    elif mutation == "reverse":
        branch["history"].reverse()
    elif mutation == "different_history":
        branch["history"][2]["sha256"] = "c"*64
    elif mutation == "different_instruction":
        branch["instruction"] += " Walk around it."
    elif mutation == "different_final_pose":
        branch["final_frame"]["pose_world"][0] += .01
    elif mutation == "different_simtime":
        branch["final_frame"]["capture_sim_time_s"] += .01
    elif mutation == "different_sham":
        b["A_SHAM"]["final_frame"]["sha256"] = "c"*64
    with pytest.raises(ValueError):
        validate_paired_inputs(b)


def test_exact_jpeg_payload_not_reencoded_parity():
    assert jpeg_wire_parity(b"original jpeg bytes", b"original jpeg bytes")["valid"]
    assert not jpeg_wire_parity(b"original jpeg bytes", b"different jpeg bytes")["valid"]


def test_visibility_sync_label_owner_and_hidden_gate():
    rgb, mask_record = _frame(1), _frame(1)
    rgb["resolution_width_height"] = mask_record["resolution_width_height"] = [10, 10]
    mask = np.zeros((10, 10), int)
    instance_map = {0: "/background", 7: "/World/RuntimeObstacle/Mesh"}
    kwargs = dict(instance_map=instance_map, obstacle_prim_path="/World/RuntimeObstacle")
    assert visibility_check(rgb, mask_record, mask, [7], present=False, **kwargs)["valid"]
    assert not visibility_check(rgb, mask_record, mask, [7], present=True, **kwargs)["valid"]
    mask[2:8, 3:7] = 7
    result = visibility_check(rgb, mask_record, mask, [7], present=True, **kwargs)
    assert result["valid"] and result["obstacle_pixels"] == 24
    assert result["bounding_box_xyxy"] == [3, 2, 7, 8]
    assert not result["proves_model_recognition"]
    mask_record["rendered_state_id"] = 2
    assert not visibility_check(rgb, mask_record, mask, [7], present=True, **kwargs)["valid"]
    kwargs["instance_map"][7] = "/World/UnrelatedProp"
    result = visibility_check(rgb, rgb, mask, [7], present=True, **kwargs)
    assert not result["valid"] and result["obstacle_pixels"] is None


def test_visibility_pixel_shape_metadata_is_required_and_empty_rejected():
    rgb = dict(_frame(1), resolution_width_height=[10, 10])
    kwargs = dict(instance_map={7: "/obstacle"}, obstacle_prim_path="/obstacle")
    result = visibility_check(rgb, rgb, np.full((8, 10), 7), [7], present=True, **kwargs)
    assert not result["valid"] and not result["flags"]["same_pixel_resolution"]
    with pytest.raises(ValueError):
        visibility_check(rgb, rgb, np.zeros((0, 0)), [7], present=False, **kwargs)


def test_visibility_occupancy_all_change_at_reveal():
    before = dict(sim_time_s=.9, render_visible=False, world_obstacle_present=False, evaluator_obstacle_present=False)
    assert obstacle_state_check(before, 1.)["valid"]
    after = dict(sim_time_s=1., render_visible=True, world_obstacle_present=True, evaluator_obstacle_present=True)
    assert obstacle_state_check(after, 1.)["valid"]
    after["evaluator_obstacle_present"] = False
    assert not obstacle_state_check(after, 1.)["valid"]


def test_pointing_optional_fields_target_visible_not_obstacle_or_stop():
    response = dict(visible=True, stop=False, pointing=dict(apos_state="stop", apos_px=[3, 4]),
                    raw_text="original", timings_ms=dict(infer=234.))
    output = pointing_diagnostics(response)
    assert output["visible"] and not output["stop"]
    assert output["pointing"]["apos_state"] == "stop"
    assert output["pointing"]["opos_px"] is None and output["seq"] is None
    assert "target visibility" in output["visible_semantics"]
    assert output["timings_ms"] == {"infer": 234.}
    assert pointing_diagnostics({})["stop"] is None
    assert response["pointing"] == {"apos_state": "stop", "apos_px": [3, 4]}
    assert pointing_diagnostics({"action": "next", "data": response}) == output


@pytest.mark.parametrize("count", [1, 2, 7, 16])
def test_observation_anchoring_variable_horizon_no_mutation(count):
    local = np.repeat([[1., 0., .1]], count, axis=0)
    copy = local.copy()
    world = observation_anchored_world(local, [4., 5., np.pi/2])
    np.testing.assert_allclose(world[:, :2], np.repeat([[4., 6.]], count, axis=0))
    np.testing.assert_array_equal(local, copy)
    assert not np.shares_memory(local, world)


def test_endpoint_gap_is_not_lateral_or_direction_evidence():
    a = [[0., 0, 0], [1., 0, 0]]
    b = [[2., 0, 0], [3., 0, 0]]
    influence = dict(origin_xy=[0, 0], forward_xy=[1, 0], progress_min_m=-1, progress_max_m=4)
    output = response_geometry(a, b, influence)
    assert not output["meaningful"]
    assert output["maximum_interior_separation_m"] is None
    assert output["endpoint_projection_count"] > 0


def test_interior_lateral_response_and_tangent_reliability():
    a = [[0., 0, 0], [3., 0, 0]]
    b = [[0., .21, 0], [3., .21, 0]]
    influence = dict(origin_xy=[0, 0], forward_xy=[1, 0], progress_min_m=.5, progress_max_m=2.5)
    output = response_geometry(a, b, influence)
    assert output["meaningful"]
    assert output["maximum_interior_separation_m"] == pytest.approx(.21)
    assert output["maximum_reliable_tangent_difference_deg"] == 0
    # Short-only/rotation rows cannot create a reliable tangent diagnosis.
    short = response_geometry([[0, 0, 0], [.001, 0, 0]], [[0, .001, 1], [.001, .001, 1]],
                              dict(influence, progress_min_m=-1, progress_max_m=2))
    assert not short["meaningful"] and short["maximum_reliable_yaw_difference_deg"] is None


def test_whole_raw_polyline_cannot_delete_unsafe_prefix(geometry):
    path = [[2, 0, 0], [2, 1, 0], [4, 1, 0]]
    report = whole_raw_polyline_check(path, geometry[1])
    assert not report["clearance_valid"]
    assert report["checked_row_count"] == 3 and not report["rows_deleted"]
    assert whole_raw_polyline_check(path[1:], geometry[1])["clearance_valid"]
    assert not report["connector_included"]


def test_safe_returned_points_do_not_make_intervening_segment_safe(geometry):
    path = np.array([[0., 0, 0], [4., 0, 0]])
    assert all(geometry[1].query(p[:2])["status"] == "CLEARANCE_VALID" for p in path)
    result = whole_raw_polyline_check(path, geometry[1])
    assert result["physical_overlap"] and not result["clearance_valid"]
    assert result["first_collision_segment_index"] == 0


def test_safe_bypass_full_raw_geometry_and_sham(geometry):
    result = _screen(geometry)
    assert result["label"] == "SAFE_BYPASS_CANDIDATE" and result["qualified"]
    assert result["off_sham_obstructed_with_obstacle"] and result["off_sham_valid_without_obstacle"]
    assert result["target_directed_progress"] and result["bypass_plane_crossed"]
    assert result["off_on"]["meaningful"] and not result["off_sham"]["meaningful"]
    assert result["checks"]["B_ON"]["on"]["original_row_count"] == 6
    assert not result["actual_obstacle_traversal_tested"]


def test_screening_never_modifies_original_arrays_or_plane(geometry):
    off, on = _paths()
    copied_off, copied_on = off.copy(), on.copy()
    plane = dict(point_xy=np.array([2.55, 0]), forward_xy=np.array([3., 0]))
    _screen(geometry, on=on, bypass_plane=plane)
    np.testing.assert_array_equal(off, copied_off)
    np.testing.assert_array_equal(on, copied_on)
    np.testing.assert_array_equal(plane["forward_xy"], [3., 0])


def test_stop_partial_unsafe_and_noresponse_are_distinct(geometry):
    assert _screen(geometry, on=[], stops={"B_ON": True})["label"] == "STOP_RESPONSE"
    assert _screen(geometry, on=_paths()[0])["label"] == "NO_MATERIAL_RESPONSE"
    partial = _paths()[1][:3]
    assert _screen(geometry, on=partial)["label"] == "SAFE_PARTIAL_DETOUR"
    unsafe = [[0, 0, 0], [1, 1, .5], [2, 0, 0], [4, 0, 0]]
    assert _screen(geometry, on=unsafe)["label"] == "CHANGED_BUT_UNSAFE"
    assert _screen(geometry, technical_checks={"mask_valid": False})["label"] == "TECHNICAL_INVALID"
    assert _screen(geometry, technical_checks={"some_check": True})["label"] == "TECHNICAL_INVALID"


def test_missing_named_or_nonboolean_check_is_technical_failure(geometry):
    for key in REQUIRED_SCREENING_CHECKS:
        incomplete = {k: True for k in REQUIRED_SCREENING_CHECKS if k != key}
        result = _screen(geometry, technical_checks=incomplete)
        assert result["label"] == "TECHNICAL_INVALID" and not result["technical_checks"][key]
    incomplete = {k: True for k in REQUIRED_SCREENING_CHECKS}
    incomplete["visibility"] = "false"
    assert _screen(geometry, technical_checks=incomplete)["label"] == "TECHNICAL_INVALID"


def _moving(geometry, **overrides):
    times = np.arange(7)/60
    cmd = np.repeat([[.3, .1]], 6, axis=0)
    states = [np.array([0., 0, 0])]
    for dt, u in zip(np.diff(times), cmd):
        states.append(integrate_unicycle(states[-1], u, dt))
    timing = dict(old_observation_sim_s=-1., reveal_sim_s=0., fresh_observation_sim_s=0.,
                  ready_seen_sim_s=.08, install_sim_s=.09, application_sim_s=.1,
                  request_send_host_s=100., full_receipt_host_s=100.08,
                  inflight_sim_span_s=.08, inflight_host_span_s=.08,
                  maximum_loop_stall_s=.017, overlap_observed=True,
                  hold_timeout=False, stale_rejected=False, source_stop=False, aborted=False)
    kwargs = dict(states_world=states, state_times_s=times, applied_commands=cmd,
                  physical_u_minus=cmd[-1], previous_control=[.25, .05], timing=timing,
                  source_flags={"source_integrity": True}, environment_on=geometry[1])
    kwargs.update(overrides)
    return moving_boundary_qualification(**kwargs)


def test_saved_motion_reconstruction_memory_distinct_and_clock_domains(geometry):
    result = _moving(geometry)
    assert result["qualified"] and not result["physical_memory_identical"]
    assert result["observation_to_B_sampled_path_length_m"] == pytest.approx(.03, abs=1e-8)
    assert result["request_RTT_s"] == pytest.approx(.08)
    assert result["observation_to_application_sim_s"] == pytest.approx(.1)
    assert result["maximum_reconstruction_error"] == pytest.approx([0, 0, 0])
    assert not result["new_rollout_performed"]


def test_boundary_moving_conditions_strict_and_failures_retained(geometry):
    result = _moving(geometry, physical_u_minus=[.20, .1])
    assert not result["qualified"] and "moving_velocity" in result["failure_reasons"]
    baseline = _moving(geometry)
    for key, value, failed in (("hold_timeout", True, "no_hold_timeout"),
                               ("stale_rejected", True, "no_stale_rejection"),
                               ("aborted", True, "no_abort"),
                               ("maximum_loop_stall_s", .251, "loop_stall"),
                               ("inflight_host_span_s", .2, "request_local_RTF")):
        timing = dict(baseline["timing"], **{key: value})
        out = _moving(geometry, timing=timing)
        assert not out["qualified"] and failed in out["failure_reasons"]


def test_module_has_no_solver_model_or_simulator_runtime():
    import reconciliation.join_source02 as module
    tree = ast.parse(Path(module.__file__).read_text())
    forbidden = {"minimize", "solve_gp", "run_instrumented", "MpcTracker", "MPCController", "SimulationApp", "DerivativeProvider"}
    names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    assert not names.intersection(forbidden)


def test_saved_report_includes_unavailable_conditions_and_no_confirmation_denominator(tmp_path):
    """Synthetic presentation fixture, not model evidence or an online episode."""
    p = Path(__file__).resolve().parents[1]/"scripts/report_join_source02.py"
    spec = importlib.util.spec_from_file_location("_join_source02_report_test", p)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    run = tmp_path/"synthetic_implementation_fixture"
    (run/"aggregate").mkdir(parents=True)
    records = [dict(condition_id="fixture_1", label="HISTORY_UNAVAILABLE", qualified=False,
                    availability="HISTORY_UNAVAILABLE", reason="fixture only", completed_branches=[]),
               dict(condition_id="fixture_2", label="NO_MATERIAL_RESPONSE", qualified=False,
                    completed_branches=list(BRANCHES))]
    (run/"aggregate/development_ledger.json").write_text(json.dumps(dict(records=records,
        selected_condition_id=None, status="NO_QUALIFYING_DEVELOPMENT_SCENARIO")))
    (run/"development/fixture_2").mkdir(parents=True)
    (run/"development/fixture_2/summary.json").write_text(json.dumps(dict(branches={name:
        dict(status="PREDICTION_READY", terminal_prediction=dict(world=[[0, 0, 0], [1, 0, 0]], response={}))
        for name in ("A", "B", "SHAM")})))
    output, bundle = run/"review", run/"review.zip"
    summary = module.report(run, output, bundle)
    assert summary["confirmation_attempted"] == 0 and summary["confirmation_qualified"] is None
    assert "N/A" in summary["confirmation_qualification_display"]
    assert summary["source_bundle_manifest"] is None
    assert "fixture_1" in (output/"index.html").read_text()
    assert not (output/"fixture_1/paired_world.png").exists()
    assert (output/"fixture_2/paired_world.png").is_file() and bundle.is_file()
    numbers = json.loads((output/"fixture_2/paired_world.json").read_text())["plotted_numbers"]
    assert numbers["world"]["B"] == [[0, 0, 0], [1, 0, 0]] and numbers["no_execution_trace"]
    assert json.loads((output/"validation.json").read_text())["valid"]
    with pytest.raises(FileExistsError):
        module.report(run, output, bundle)

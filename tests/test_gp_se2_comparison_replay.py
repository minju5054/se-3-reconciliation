"""Synthetic saved-GP replay fixtures; no simulation or performance evidence."""
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

SCRIPT = Path(__file__).resolve().parents[1]/"scripts/isaac/gp_se2_comparison_replay.py"
SPEC = importlib.util.spec_from_file_location("gp_comparison_replay_test", SCRIPT)
replay = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(replay)


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


@pytest.fixture
def artifact(tmp_path):
    source, run = tmp_path/"source", tmp_path/"experiment"
    source.mkdir()
    run.mkdir()
    (source/"config_snapshot.yaml").write_text(yaml.safe_dump({"scene": {"fixture": True}, "agent": {"z_m": 0.}}))
    (run/"config_snapshot.yaml").write_text(yaml.safe_dump({"source_run": str(source),
        "footprint": {"radius_m": .2, "required_clearance_m": .05},
        "formulation": {"goal_position_tolerance": .15}}))
    selected = []
    for case_id, group in (("ep2/handoff_002", "D_OBSTACLE_ROUTE"), ("ep1/handoff_001", "A_SMALL_STRAIGHT"),
                            ("ep3/handoff_003", "C_LARGE_MISMATCH")):
        directory = case_id.replace("/", "__")
        selected.append({"case_id": case_id, "case_directory": directory, "selected_group": group})
        folder = run/"cases"/directory
        context = {"B_world": [2., 3., .1], "old_world": [[1., 3., .1], [3., 3., .1]],
                   "fresh_world": [[2.1, 3., .1], [3., 3., .1]]}
        write_json(folder/"input_context.json", context)
        write_json(folder/"goal_route.json", {"goal_world": [3., 3., .1],
            "gates": [{"gate_id": "gate_0", "center_xy": [2.5, 3.], "normal_xy": [1., 0.], "half_width_m": .3, "time_s": 1.}]})
        for method in replay.METHODS:
            method_folder = folder/"methods"/method
            write_json(method_folder/"metrics.json", {"primary_success": not (case_id == "ep3/handoff_003" and method == "M3_GP_CONSTRAINED")})
            if case_id == "ep3/handoff_003" and method == "M3_GP_CONSTRAINED":
                write_json(method_folder/"status.json", {"status": "NO_FEASIBLE_CANDIDATE_FOUND"})
                continue
            candidate = np.array(context["fresh_world"])
            np.save(method_folder/"candidate_world.npy", candidate)
            poses = [[2., 3., .1], [2.01, 3.005, .12], [2.02, 3.006, .15]]
            rollout = {"states": [{"tick": i, "time_s": i/60, "pose_world": p} for i, p in enumerate(poses)],
                       "candidate_world": candidate.tolist()}
            write_json(method_folder/"rollout/rollout.json", rollout)
    write_json(run/"case_manifest.json", {"selected": selected})
    write_json(run/"environment/geometry/render_geometry.json", {"workspace_polygons": [[[0, 0], [5, 0], [5, 6], [0, 6], [0, 0]]]})
    return run


def test_representative_selection_includes_regression_obstacle_benign(artifact):
    saved = replay.SavedComparison(artifact)
    assert [saved.cases[i]["manifest"]["case_id"] for i in saved.representative_indices] == [
        "ep3/handoff_003", "ep2/handoff_002", "ep1/handoff_001"]
    assert all(r["available"] for r in saved.representative_report)
    assert len(saved.workspace_rings) == 1


def test_missing_representative_categories_are_explicit_not_fabricated(artifact):
    path = artifact/"case_manifest.json"
    manifest = json.loads(path.read_text())
    manifest["selected"] = [manifest["selected"][1]]
    manifest["selected"][0]["selected_group"] = "C_LARGE_MISMATCH"
    write_json(path, manifest)
    saved = replay.SavedComparison(artifact)
    assert all(not r["available"] for r in saved.representative_report[:3])
    assert saved.representative_report[-1]["reason"] == "no_requested_representative_category_available"
    assert saved.representative_indices == [0]


def test_no_candidate_has_no_zero_filled_or_reused_execution(artifact):
    saved = replay.SavedComparison(artifact)
    saved.select(case_index=2, method_index=4)
    assert saved.method["candidate"] is None
    assert saved.method["poses"] is None
    assert saved.method["times"] is None
    assert "NO CANDIDATE" in saved.method["display_status"]
    saved.playing = True
    saved.advance(1.)
    assert not saved.playing and saved.index == 0
    saved.seek_end()
    assert saved.replay_time_s == 0


def test_playback_selects_only_saved_rows_no_interpolation(artifact):
    saved = replay.SavedComparison(artifact)
    saved.select(case_index=0, method_index=0)
    saved.playing = True
    saved.advance(.02)
    assert saved.index == 1
    np.testing.assert_array_equal(saved.method["poses"][saved.index], [2.01, 3.005, .12])
    saved.advance(1.)
    assert saved.index == 2 and not saved.playing
    saved.reset()
    assert saved.index == 0 and saved.replay_time_s == 0
    assert not saved.method["poses"].flags.writeable
    assert not saved.method["times"].flags.writeable


def test_method_switch_resets_and_camera_geometry_is_method_independent(artifact):
    saved = replay.SavedComparison(artifact)
    saved.select(case_index=0)
    geometry = saved.overview_geometry()
    center, spans = replay.overview_aperture(geometry, 1.4)
    for index in range(5):
        saved.select(method_index=index)
        saved.seek_end()
        np.testing.assert_array_equal(saved.overview_geometry(), geometry)
        assert spans[0]/spans[1] == pytest.approx(1.4)
        saved.select(method_index=(index+1)%5)
        assert saved.index == 0


def test_source_mutation_detected_and_incomplete_methods_fail(artifact):
    saved = replay.SavedComparison(artifact)
    metrics = artifact/"cases/ep1__handoff_001/methods/M0_NATIVE/metrics.json"
    metrics.write_text('{"primary_success":false}')
    with pytest.raises(ValueError, match="source changed"):
        saved.verify_unchanged()
    metrics.unlink()
    with pytest.raises(FileNotFoundError):
        replay.SavedComparison(artifact)


def test_candidate_must_equal_controller_input_and_initial_state_exact_B(artifact):
    path = artifact/"cases/ep1__handoff_001/methods/M0_NATIVE/rollout/rollout.json"
    content = json.loads(path.read_text())
    content["candidate_world"][0][0] += .001
    write_json(path, content)
    with pytest.raises(ValueError, match="actual controller input"):
        replay.SavedComparison(artifact)
    content["candidate_world"][0][0] -= .001
    content["states"][0]["pose_world"][0] += 1e-12
    write_json(path, content)
    with pytest.raises(ValueError, match="exact frozen B"):
        replay.SavedComparison(artifact)


def test_common_gate_geometry_and_invalid_gate_rejected():
    gate = {"center_xy": [2., 3.], "normal_xy": [1., 0.], "half_width_m": .4}
    np.testing.assert_allclose(replay.gate_segment(gate), [[2., 2.6], [2., 3.4]])
    with pytest.raises(ValueError):
        replay.gate_segment({**gate, "normal_xy": [0., 0.]})
    with pytest.raises(ValueError):
        replay.gate_segment({"unknown": "no invented gate"})


@pytest.mark.parametrize("elapsed", [-.1, np.nan, np.inf])
def test_invalid_display_clock_rejected(artifact, elapsed):
    with pytest.raises(ValueError):
        replay.SavedComparison(artifact).advance(elapsed)


def test_case_directory_escape_and_duplicate_rejected(artifact):
    path = artifact/"case_manifest.json"
    manifest = json.loads(path.read_text())
    manifest["selected"][0]["case_directory"] = "../../escaped"
    write_json(path, manifest)
    with pytest.raises(ValueError, match="escaped"):
        replay.SavedComparison(artifact)


def test_launcher_static_flags_and_persistent_gui_default():
    launcher = SCRIPT.parents[1]/"launch_gp_se2_comparison_replay.sh"
    assert 'gp_se2_comparison_replay.py' in launcher.read_text()
    text = SCRIPT.read_text()
    assert 'parser.add_argument("--no-hold", action="store_true")' in text
    assert '"headless": False' in text
    assert 'new_mpc_solves' in text and 'OFFLINE COUNTERFACTUAL HANDOFF COMPARISON' in text

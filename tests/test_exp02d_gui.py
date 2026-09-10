from __future__ import annotations

import ast
import json
import math
from pathlib import Path
from types import MappingProxyType

import numpy as np
import pytest
import yaml

from reconciliation.data02_active_old import ActiveOldInterval
from reconciliation.exp02d_gui import (
    PHASE_A_FRACTION,
    RepresentativeUnavailableError,
    _validate_case_rule,
    available_representatives,
    gui_phase_and_saved_time,
    latest_completed_run,
    load_exp02d_gui_case,
    saved_only_gui_contract,
    saved_pose_at,
)
from reconciliation.online_switch import sha256_file


ROOT = Path(__file__).resolve().parents[1]
GUI_SCRIPT = ROOT / "scripts/isaac/exp02d_success_failure_gui.py"


def _readonly(value: list[list[float]] | list[float]) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    result.setflags(write=False)
    return result


def _active(*, geometry: str = "POSITIVE_TURNING") -> ActiveOldInterval:
    poses = _readonly(
        [
            [0.0, 0.0, 3.10],
            [1.0, 0.0, -3.10],
            [2.0, 0.0, -3.00],
        ]
    )
    times = _readonly([1.0, 2.0, 3.0])
    old = _readonly([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    fresh = _readonly(
        [[2.1, 0.0, 0.0], [2.3, 0.1, 0.2], [2.5, 0.3, 0.4]]
    )
    rows = tuple(
        MappingProxyType(
            {
                "active_chunk_id": "chunk_03",
                "sim_time_s": str(time),
            }
        )
        for time in times
    )
    return ActiveOldInterval(
        run=Path("/tmp/source"),
        episode_id="episode_000001",
        transition_index=3,
        transition_directory=Path("/tmp/source/transition_03"),
        transition_metadata=MappingProxyType(
            {"metrics": {"fresh_geometry_bin": geometry}}
        ),
        old_chunk_id="chunk_03",
        fresh_chunk_id="chunk_04",
        old_world=old,
        fresh_world=fresh,
        telemetry_rows=rows,
        telemetry_sim_times_s=times,
        telemetry_actual_poses=poses,
        display_sim_times_s=times,
        display_actual_poses=poses,
        activation_sim_time_s=1.0,
        activation_pose_world_se2=poses[0],
        activation_source="PREVIOUS_TRANSITION_SWITCH_BOUNDARY",
        activation_boundary_prepended=True,
        observation_sim_time_s=2.0,
        observation_pose_world_se2=poses[1],
        model_ready_sim_time_s=2.5,
        model_ready_pose_world_se2=_readonly([1.5, 0.0, -3.05]),
        p_sim_time_s=2.9,
        p_pose_world_se2=_readonly([1.9, 0.0, -3.01]),
        switch_sim_time_s=3.0,
        boundary_pose_world_se2=poses[-1],
        active_old_path_length_m=2.0,
        active_old_net_displacement_m=2.0,
        active_old_yaw_change_rad=0.18318530717958623,
        source_sha256=MappingProxyType({"transition.json": "digest"}),
    )


def _commands(
    *,
    raw=(0.01, 0.01, 0.1),
    m1=(0.06, 0.01, 1.0),
    m2=(0.08, 0.12, 0.8),
    m3=(0.02, 0.02, 0.2),
):
    def row(values):
        return {
            "command": {
                "delta_v_abs_mps": values[0],
                "delta_omega_abs_rps": values[1],
                "J_cmd": values[2],
            }
        }

    return {
        "M0_RAW": row(raw),
        "M1_HISTORICAL_M4": row(m1),
        "M2_NO_DIRECTION": row(m2),
        "M3_LOOKAHEAD": row(m3),
    }


def _representative(*, entry_deg=70.0, look_deg=10.0, gap=0.30):
    return {
        "metrics": {
            "alpha_entry_rad": math.radians(entry_deg),
            "alpha_look_rad": math.radians(look_deg),
            "b_to_fresh_k_translation_m": gap,
        }
    }


def test_saved_pose_uses_adjacent_saved_states_and_shortest_yaw() -> None:
    active = _active()
    display = saved_pose_at(active, 1.5)
    assert (display.lower_index, display.upper_index, display.alpha) == (0, 1, 0.5)
    np.testing.assert_allclose(display.pose[:2], [0.5, 0.0])
    assert abs(abs(float(display.pose[2])) - math.pi) < 1e-12
    assert not display.pose.flags.writeable
    np.testing.assert_array_equal(saved_pose_at(active, -10.0).pose, active.display_actual_poses[0])
    np.testing.assert_array_equal(saved_pose_at(active, 10.0).pose, active.display_actual_poses[-1])


def test_gui_phase_maps_phase_a_to_saved_interval_then_freezes_at_b() -> None:
    active = _active()
    phase, saved = gui_phase_and_saved_time(0.0, 10.0, active)
    assert phase == "PHASE_A_ACTUAL_OLD_ACTIVE"
    assert saved == active.activation_sim_time_s
    phase, saved = gui_phase_and_saved_time(PHASE_A_FRACTION * 10.0, 10.0, active)
    assert phase == "PHASE_B_OFFLINE_CANDIDATES_AT_B"
    assert saved == active.switch_sim_time_s
    phase, saved = gui_phase_and_saved_time(100.0, 10.0, active)
    assert phase == "PHASE_B_OFFLINE_CANDIDATES_AT_B"
    assert saved == active.switch_sim_time_s
    with pytest.raises(ValueError, match="positive"):
        gui_phase_and_saved_time(0.0, 0.0, active)
    with pytest.raises(ValueError, match="finite"):
        gui_phase_and_saved_time(float("nan"), 10.0, active)


def test_frozen_representative_explanations_require_their_actual_rules() -> None:
    outcome, explanation = _validate_case_rule(
        "S1", _representative(), _commands(), _active()
    )
    assert outcome == "SUCCESS — BENIGN RESCUE"
    assert "lookahead" in explanation

    outcome, _ = _validate_case_rule(
        "S2",
        _representative(),
        _commands(raw=(0.20, 0.01, 2.0), m2=(0.10, 0.20, 1.5), m3=(0.10, 0.20, 1.0)),
        _active(),
    )
    assert outcome == "SUCCESS — CHALLENGING IMPROVEMENT"

    outcome, explanation = _validate_case_rule(
        "F1",
        _representative(),
        _commands(raw=(0.10, 0.20, 1.0), m3=(0.20, 0.40, 1.21)),
        _active(geometry="NEGATIVE_TURNING"),
    )
    assert outcome == "CANDIDATE FAILURE — INTENT-CONFLICT CANDIDATE"
    assert "does not prove" in explanation

    outcome, _ = _validate_case_rule(
        "F2",
        _representative(look_deg=10.0, gap=0.30),
        _commands(raw=(0.10, 0.20, 1.0), m3=(0.10, 0.20, 1.05)),
        _active(),
    )
    assert outcome == "FAILURE — POSITIONAL / MISSING MECHANISM"

    with pytest.raises(ValueError, match="violates"):
        _validate_case_rule("S1", _representative(), _commands(m1=(0.01, 0.01, 0.1)), _active())


def test_saved_only_gui_contract_keeps_m2_opt_in_and_forbids_execution() -> None:
    contract = saved_only_gui_contract()
    assert contract["saved_primary_results_only"]
    assert not contract["m2_visible_by_default"]
    for key in (
        "lightnav_invoked",
        "optimizer_invoked",
        "controller_invoked",
        "physics_reexecution",
        "candidate_execution",
        "previous_chunk_actual_displayed",
        "post_switch_actual_displayed",
    ):
        assert not contract[key]


def test_latest_completed_run_skips_incomplete_and_is_deterministic(tmp_path: Path) -> None:
    incomplete = tmp_path / "999_incomplete"
    incomplete.mkdir()
    for name in ("001_primary", "002_primary"):
        run = tmp_path / name
        run.mkdir()
        (run / "result_manifest.json").write_text(
            json.dumps({"schema": "EXP02D_ResultManifest_v1"}), encoding="utf-8"
        )
        (run / "metadata.json").write_text(
            json.dumps(
                {
                    "experiment": "EXP-02D",
                    "technical_status": "EXP02D_RUN_COMPLETE",
                }
            ),
            encoding="utf-8",
        )
    assert latest_completed_run(tmp_path) == (tmp_path / "002_primary").resolve()
    with pytest.raises(FileNotFoundError):
        latest_completed_run(tmp_path / "absent")


def test_available_representatives_preserves_not_available(tmp_path: Path) -> None:
    summary = tmp_path / "summary"
    summary.mkdir()
    (summary / "representatives.json").write_text(
        json.dumps(
            {
                "representatives": {
                    "S1": {"status": "AVAILABLE", "corpus_transition_id": "v1:a"},
                    "S2": {"status": "S2_NOT_AVAILABLE", "corpus_transition_id": None},
                    "F1": {"status": "F1_NOT_AVAILABLE", "corpus_transition_id": None},
                    "F2": {"status": "AVAILABLE", "corpus_transition_id": "v2:b"},
                }
            }
        ),
        encoding="utf-8",
    )
    assert available_representatives(tmp_path) == {
        "S1": "v1:a",
        "S2": None,
        "F1": None,
        "F2": "v2:b",
    }


def test_gui_loader_refuses_a_frozen_unavailable_case(tmp_path: Path) -> None:
    (tmp_path / "summary").mkdir()
    metadata = {
        "experiment": "EXP-02D",
        "technical_status": "EXP02D_RUN_COMPLETE",
        "candidate_methods_physically_executed": False,
    }
    config = {"experiment": "EXP-02D"}
    representatives = {
        "representatives": {
            case: {
                "status": f"{case}_NOT_AVAILABLE",
                "corpus_transition_id": None,
                "criteria_relaxed": False,
            }
            for case in ("S1", "S2", "F1", "F2")
        }
    }
    (tmp_path / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
    (tmp_path / "config_snapshot.yaml").write_text(
        yaml.safe_dump(config), encoding="utf-8"
    )
    (tmp_path / "summary/representatives.json").write_text(
        json.dumps(representatives), encoding="utf-8"
    )
    files = (
        "metadata.json",
        "config_snapshot.yaml",
        "summary/representatives.json",
    )
    (tmp_path / "result_manifest.json").write_text(
        json.dumps(
            {
                "schema": "EXP02D_ResultManifest_v1",
                "artifact_sha256": {
                    name: sha256_file(tmp_path / name) for name in files
                },
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(RepresentativeUnavailableError, match="F1_NOT_AVAILABLE"):
        load_exp02d_gui_case(tmp_path, "F1")


def test_isaac_gui_is_saved_replay_only_and_has_required_captures_and_flags() -> None:
    source = GUI_SCRIPT.read_text(encoding="utf-8")
    tree = ast.parse(source)
    forbidden_calls = {
        "step",
        "apply_action",
        "set_joint_velocities",
        "forward",
        "solve",
        "solve_method",
    }
    called_attributes = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    called_names = {
        node.func.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }
    assert forbidden_calls.isdisjoint(called_attributes | called_names)
    assert "TrajectoryFollower" not in source
    assert "DifferentialController" not in source
    assert "OnlineLightNavClient" not in source
    assert "draw_heading_markers" not in source
    for capture in (
        "01_old_active.png",
        "02_at_observation.png",
        "03_at_B_raw.png",
        "04_method_comparison.png",
    ):
        assert capture in source
    for flag in ("--no-hold", "--show-m2", "--show-entry-vector", "--headless"):
        assert flag in source
    assert 'if ARGS.show_m2:' in source
    assert '"candidate_execution": False' in source
    assert '"physics_reexecution": False' in source

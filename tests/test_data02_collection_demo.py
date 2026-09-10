from __future__ import annotations

import ast
import csv
import json
from pathlib import Path

import numpy as np
import pytest

from reconciliation.data02_collection_demo import (
    DEFAULT_DURATION_S,
    DEFAULT_EPISODE,
    DEFAULT_RUN_RELATIVE,
    DEFAULT_TRANSITION,
    load_saved_demo,
    phase_at,
    phase_schedule,
    saved_only_contract,
    scientific_time_for_presentation,
)
from reconciliation.online_switch import sha256_file


ROOT = Path(__file__).resolve().parents[1]


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _telemetry_row(
    sample: int,
    sim_time: float,
    x: float,
    *,
    phase: str,
    chunk: str,
    inflight: int,
) -> dict[str, object]:
    return {
        "sample_index": sample,
        "sim_time_s": sim_time,
        "phase": phase,
        "actual_x": x,
        "actual_y": 0.1 * x,
        "actual_yaw": 0.01 * x,
        "active_chunk_id": chunk,
        "fresh_request_in_flight": inflight,
    }


def _save_npy(path: Path, value: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        np.save(stream, value, allow_pickle=False)


def demo_fixture(tmp_path: Path) -> Path:
    run = tmp_path / DEFAULT_RUN_RELATIVE
    episode = run / "episodes" / DEFAULT_EPISODE
    transition_dir = episode / "transitions/transition_04"
    old = np.array([[0.0, 0.0, 0.0], [1.0, 0.1, 0.01], [2.0, 0.2, 0.02]])
    fresh = np.array([[1.0, 0.1, 0.1], [1.8, 0.8, 0.4], [2.2, 1.2, 0.6]])
    actual = np.array([[0.0, 0.0, 0.0], [1.0, 0.1, 0.01], [2.0, 0.2, 0.02], [3.0, 0.3, 0.03]])
    _save_npy(transition_dir / "derived/old_world.npy", old)
    _save_npy(transition_dir / "derived/fresh_world.npy", fresh)
    _save_npy(transition_dir / "actual.npy", actual)
    transition_rows = [
        _telemetry_row(0, 1.0, 0.0, phase="ACTIVE_OLD", chunk="chunk_04", inflight=0),
        _telemetry_row(1, 2.0, 1.0, phase="FRESH_IN_FLIGHT", chunk="chunk_04", inflight=1),
        _telemetry_row(2, 3.0, 2.0, phase="FRESH_IN_FLIGHT", chunk="chunk_04", inflight=1),
        _telemetry_row(3, 4.0, 3.0, phase="FRESH_IN_FLIGHT", chunk="chunk_04", inflight=1),
    ]
    _write_csv(transition_dir / "telemetry.csv", transition_rows)
    artifacts = {
        relative: sha256_file(transition_dir / relative)
        for relative in (
            "derived/old_world.npy",
            "derived/fresh_world.npy",
            "actual.npy",
            "telemetry.csv",
        )
    }
    transition = {
        "episode_id": DEFAULT_EPISODE,
        "transition_id": f"{DEFAULT_EPISODE}_transition_04",
        "transition_index": DEFAULT_TRANSITION,
        "old_chunk_id": "chunk_04",
        "fresh_chunk_id": "chunk_05",
        "artifact_sha256": artifacts,
        "observation_pose_world_se2": [1.0, 0.1, 0.01],
        "pose_immediately_before_switch_P_world_se2": [2.9, 0.29, 0.029],
        "pose_immediately_before_switch_P_sim_time_s": 3.9,
        "switch_boundary_B_world_se2": [3.0, 0.3, 0.03],
        "timing": {
            "t_obs_sim_s": 2.0,
            "t_ready_sim_s": 4.0,
            "t_switch_sim_s": 4.0,
            "model_reported_s": 0.4,
            "tau_effective_s": 2.0,
        },
        "metrics": {"observation_to_model_ready_translation_m": 2.01},
    }
    (transition_dir / "transition.json").write_text(json.dumps(transition), encoding="utf-8")
    episode_rows = transition_rows + [
        _telemetry_row(4, 4.5, 3.3, phase="ACTIVE_OLD", chunk="chunk_05", inflight=0),
        _telemetry_row(5, 5.0, 3.6, phase="ACTIVE_OLD", chunk="chunk_05", inflight=0),
        _telemetry_row(6, 5.5, 3.9, phase="FRESH_IN_FLIGHT", chunk="chunk_05", inflight=1),
    ]
    _write_csv(episode / "telemetry.csv", episode_rows)
    chunk = episode / "chunks/chunk_05"
    chunk.mkdir(parents=True)
    (chunk / "metadata.json").write_text(
        json.dumps({
            "chunk_id": "chunk_05",
            "observation_rgb_frame_index": 81,
            "observation_sim_time_s": 2.0,
        }),
        encoding="utf-8",
    )
    rgb = episode / "rgb/frame_000081.png"
    rgb.parent.mkdir()
    rgb.write_bytes(b"exact-saved-rgb-fixture")
    _write_csv(episode / "rgb_frames.csv", [{
        "frame_index": 81,
        "sim_time_s": 2.0,
        "rgb_relative_path": "rgb/frame_000081.png",
        "rgb_file_sha256": sha256_file(rgb),
    }])
    return run


def _tree_hashes(path: Path) -> dict[str, str]:
    return {
        str(item.relative_to(path)): sha256_file(item)
        for item in sorted(path.rglob("*"))
        if item.is_file()
    }


def test_default_selection_loads_valid_saved_transition(tmp_path: Path) -> None:
    assert DEFAULT_RUN_RELATIVE == Path(
        "data/data02_online_successive_v2/data02-online-successive-extension-v2"
    )
    assert (DEFAULT_EPISODE, DEFAULT_TRANSITION, DEFAULT_DURATION_S) == (
        "episode_000061", 4, 12.0
    )
    evidence = load_saved_demo(demo_fixture(tmp_path))
    assert evidence.transition_id == "episode_000061_transition_04"


def test_five_phase_order_and_default_boundaries() -> None:
    phases = phase_schedule()
    assert [phase.key for phase in phases] == [
        "OLD_EXECUTING", "FRESH_IN_FLIGHT", "FRESH_READY", "RAW_SWITCH", "FRESH_ACTIVE"
    ]
    assert [(phase.start_s, phase.end_s) for phase in phases] == [
        (0.0, 2.0), (2.0, 5.0), (5.0, 7.0), (7.0, 10.0), (10.0, 12.0)
    ]
    assert phase_at(2.0).key == "FRESH_IN_FLIGHT"
    with pytest.raises(ValueError):
        phase_schedule(9.99)


def test_scientific_timestamps_preserved_and_mapping_monotonic(tmp_path: Path) -> None:
    evidence = load_saved_demo(demo_fixture(tmp_path))
    assert evidence.timing.t_obs_sim_s == 2.0
    assert evidence.timing.model_reported_s == 0.4
    assert evidence.timing.effective_latency_s == 2.0
    assert evidence.timing.t_switch_sim_s == 4.0
    mapped = np.array([
        scientific_time_for_presentation(value, DEFAULT_DURATION_S, evidence.timing)
        for value in np.linspace(0.0, DEFAULT_DURATION_S, 301)
    ])
    assert np.all(np.diff(mapped) >= 0.0)
    assert mapped[0] == 1.0 and mapped[-1] == 5.0


def test_old_is_active_and_moves_during_fresh_inflight(tmp_path: Path) -> None:
    evidence = load_saved_demo(demo_fixture(tmp_path))
    assert evidence.old_chunk_id == "chunk_04"
    assert evidence.motion_during_inference_m == pytest.approx(2.01)


def test_inflight_rejects_non_old_active_chunk(tmp_path: Path) -> None:
    run = demo_fixture(tmp_path)
    transition_dir = run / "episodes/episode_000061/transitions/transition_04"
    rows = list(csv.DictReader((transition_dir / "telemetry.csv").open(encoding="utf-8")))
    rows[1]["active_chunk_id"] = "chunk_05"
    (transition_dir / "telemetry.csv").unlink()
    _write_csv(transition_dir / "telemetry.csv", rows)
    document_path = transition_dir / "transition.json"
    document = json.loads(document_path.read_text(encoding="utf-8"))
    document["artifact_sha256"]["telemetry.csv"] = sha256_file(transition_dir / "telemetry.csv")
    document_path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(ValueError, match="OLD must remain active"):
        load_saved_demo(run)


def test_p_must_precede_b(tmp_path: Path) -> None:
    run = demo_fixture(tmp_path)
    document_path = run / "episodes/episode_000061/transitions/transition_04/transition.json"
    document = json.loads(document_path.read_text(encoding="utf-8"))
    document["pose_immediately_before_switch_P_sim_time_s"] = 4.0
    document_path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(ValueError, match="P must strictly precede"):
        load_saved_demo(run)


def test_fresh_is_loaded_from_hashed_saved_artifact_and_read_only(tmp_path: Path) -> None:
    run = demo_fixture(tmp_path)
    saved = np.load(
        run / "episodes/episode_000061/transitions/transition_04/derived/fresh_world.npy",
        allow_pickle=False,
    )
    evidence = load_saved_demo(run)
    assert np.array_equal(evidence.fresh_world, saved)
    assert not evidence.fresh_world.flags.writeable
    with pytest.raises(ValueError):
        evidence.fresh_world[0, 0] = 99.0


def test_loading_demo_does_not_modify_source_tree(tmp_path: Path) -> None:
    run = demo_fixture(tmp_path)
    before = _tree_hashes(run)
    load_saved_demo(run)
    assert _tree_hashes(run) == before


def test_demo_contract_forbids_inference_physics_and_controller_execution() -> None:
    assert saved_only_contract() == {
        "saved_only": True,
        "lightnav_inference": False,
        "physics_reexecution": False,
        "controller_execution": False,
        "scientific_timestamps_modified": False,
    }
    launcher = (ROOT / "scripts/isaac/run_data02_collection_demo.sh").read_text(encoding="utf-8")
    assert "--replay-run" in launcher and "--demo" in launcher
    assert "serve_online_lightnav.py" not in launcher
    base_launcher = (ROOT / "scripts/isaac/run_data02_online_successive.sh").read_text(
        encoding="utf-8"
    )
    replay_exit = base_launcher.index('if [[ "${replay}" == true')
    lightnav_invocation = base_launcher.index("serve_online_lightnav.py")
    assert replay_exit < lightnav_invocation
    assert "exit 0" in base_launcher[replay_exit:lightnav_invocation]
    source_path = ROOT / "scripts/isaac/data02_online_successive.py"
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    function = next(
        node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "replay_demo"
    )
    body = ast.unparse(function)
    for forbidden in (
        "world.step", "apply_action", "runtime_wheel_command", "TrajectoryFollower(",
        "DifferentialController(", "OnlineLightNavClient(", "set_joint_velocities",
    ):
        assert forbidden not in body

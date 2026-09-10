from __future__ import annotations

import ast
import csv
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import pytest

from reconciliation.data02_high_motion_demo import (
    DEFAULT_DURATION_S,
    MotionCandidate,
    ReplayEvidence,
    compute_camera_framing,
    interpolate_adjacent_saved_pose,
    interpolated_pose_at,
    load_replay_evidence,
    phase_schedule,
    saved_only_contract,
    saved_time_for_presentation,
    select_high_motion_candidate,
)
from reconciliation.data02_active_old import load_active_old_interval
from reconciliation.data02_online_successive import chunk_content_sha256
from reconciliation.online_switch import sha256_file


ROOT = Path(__file__).resolve().parents[1]


def candidate(
    name: str,
    inference: float,
    path: float,
    *,
    status: str = "ELIGIBLE_MOVING",
    geometry: str = "STRAIGHT_LIKE",
    net: float | None = None,
    omega: float = 0.0,
) -> MotionCandidate:
    episode = "episode_000062" if name == "old" else f"episode_{int(name):06d}"
    transition_index = 3 if name == "old" else 0
    run_id = "data02-online-successive-extension-v2" if name == "old" else "fixture-run"
    return MotionCandidate(
        run_id=run_id,
        source_run_path=f"/fixture/{run_id}",
        episode_id=episode,
        template_id="fixture",
        variant_id="V0",
        transition_id=f"{episode}_transition_{transition_index:02d}",
        transition_index=transition_index,
        old_chunk_id="chunk_00",
        fresh_chunk_id="chunk_01",
        status=status,
        fresh_geometry=geometry,
        t_obs_sim_s=1.0,
        t_ready_sim_s=2.0,
        t_switch_sim_s=2.0,
        p_sim_time_s=1.9,
        old_active_start_sim_s=0.0,
        old_active_end_sim_s=2.0,
        old_active_telemetry_sample_count=5,
        old_active_display_pose_count=5,
        old_activation_source="EARLIEST_BOOTSTRAP_OLD_TELEMETRY",
        activation_boundary_prepended=False,
        inference_translation_m=inference,
        inference_path_length_m=inference,
        active_old_path_length_m=path,
        active_old_net_displacement_m=path if net is None else net,
        active_old_yaw_change_rad=0.0,
        abs_delta_v_des_mps=0.0,
        abs_delta_omega_des_rps=omega,
        transition_json_sha256="a" * 64,
        old_world_sha256="b" * 64,
        fresh_world_sha256="c" * 64,
        actual_sha256="d" * 64,
        transition_telemetry_sha256="e" * 64,
        episode_telemetry_sha256="f" * 64,
    )


def test_selection_is_eligible_only_and_enforces_top_decile_inference() -> None:
    values = [candidate(str(index), float(index), 10.0 + index) for index in range(20)]
    old = candidate("old", 0.2, 0.2)
    invalid = candidate("99", 99.0, 999.0, status="TIMING_INVALID")
    result = select_high_motion_candidate([*values, old, invalid])
    # 21 eligible rows -> top three; index 17 wins by replay path among 17,18,19.
    assert result.inference_top_decile_count == 3
    assert result.selected.episode_id == "episode_000019"
    assert result.selected.status == "ELIGIBLE_MOVING"
    assert result.selected is not invalid


def test_selection_primary_path_and_tie_break_are_deterministic() -> None:
    old = candidate("old", 1.0, 1.0)
    rows = [candidate(str(index), 0.1, 0.1) for index in range(20)]
    first = candidate("21", 5.0, 8.0, geometry="STRAIGHT_LIKE", net=7.0, omega=2.0)
    second = candidate("20", 5.0, 8.0, geometry="POSITIVE_TURNING", net=7.0, omega=1.0)
    result = select_high_motion_candidate([old, *rows, first, second])
    assert result.selected is second  # non-straight precedes omega and lexicographic tie breaks
    reversed_result = select_high_motion_candidate(list(reversed([old, *rows, first, second])))
    assert reversed_result.selected == result.selected


def test_previous_default_is_not_selected_unless_it_wins() -> None:
    old = candidate("old", 10.0, 1.0)
    winner = candidate("2", 11.0, 5.0)
    rows = [candidate(str(index + 10), float(index) / 100.0, 0.1) for index in range(20)]
    assert select_high_motion_candidate([old, winner, *rows]).selected == winner
    winning_old = replace(old, active_old_path_length_m=20.0)
    assert select_high_motion_candidate([winning_old, winner, *rows]).selected == winning_old


def test_three_phase_replay_stops_at_selected_boundary() -> None:
    item = candidate("1", 1.0, 4.0)
    phases = phase_schedule()
    assert [phase.key for phase in phases] == [
        "OLD_ACTIVE",
        "FRESH_INFERENCE_OLD_ACTIVE",
        "AT_B",
    ]
    assert [(phase.start_s, phase.end_s) for phase in phases] == [
        (0.0, 4.0),
        (4.0, 11.0),
        (11.0, 15.0),
    ]
    mapped = [
        saved_time_for_presentation(value, DEFAULT_DURATION_S, item)
        for value in (0.0, 4.0, 11.0, 15.0)
    ]
    assert mapped == [0.0, 1.0, 2.0, 2.0]
    assert mapped[0] < mapped[1] < mapped[2]
    assert mapped[2] == mapped[3]


def test_adjacent_interpolation_exact_endpoints_linear_xy_and_shortest_yaw() -> None:
    first = np.array([0.0, 2.0, np.deg2rad(179.0)])
    second = np.array([4.0, 6.0, np.deg2rad(-179.0)])
    assert np.array_equal(interpolate_adjacent_saved_pose(first, second, 0.0), first)
    assert np.array_equal(interpolate_adjacent_saved_pose(first, second, 1.0), second)
    middle = interpolate_adjacent_saved_pose(first, second, 0.5)
    assert np.allclose(middle[:2], [2.0, 4.0])
    assert abs(abs(middle[2]) - np.pi) < 1e-12
    assert np.all(middle[:2] >= np.minimum(first[:2], second[:2]))
    assert np.all(middle[:2] <= np.maximum(first[:2], second[:2]))
    with pytest.raises(ValueError):
        interpolate_adjacent_saved_pose(first, second, 1.01)


def test_displayed_pose_changes_and_camera_bounds_are_finite() -> None:
    item = candidate("1", 1.0, 4.0)
    times = np.array([0.0, 1.0, 2.0, 4.0])
    poses = np.array([[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 2.0, 0.0], [0.0, 4.0, 0.0]])
    evidence = ReplayEvidence(
        item,
        Path("/fixture"),
        poses[:2],
        poses[2:],
        times,
        poses,
        poses[1],
        poses[2],
        poses[2],
        poses[2],
        None,
        None,
        {},
    )
    before = interpolated_pose_at(evidence, 0.5).pose
    after = interpolated_pose_at(evidence, 3.0).pose
    assert np.linalg.norm(after[:2] - before[:2]) > 0.0
    framing = compute_camera_framing(poses, poses[:2], poses[2:])
    assert np.all(np.isfinite((*framing.eye_xyz, *framing.target_xyz)))
    assert 0.60 <= framing.estimated_actual_viewport_fraction <= 0.75
    assert framing.estimated_all_geometry_viewport_fraction <= 0.82 + 1e-12


def _save_npy(path: Path, value: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        np.save(stream, value, allow_pickle=False)


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _replace_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.unlink()
    _write_csv(path, rows)


def replay_fixture(tmp_path: Path) -> Path:
    run = tmp_path / "fixture-run"
    episode = run / "episodes/episode_000001"
    transition_dir = episode / "transitions/transition_00"
    times = np.arange(0.0, 4.5, 0.5)
    poses = np.column_stack((times, 0.1 * times, 0.01 * times))
    rows = [
        {
            "sample_index": index,
            "sim_time_s": value,
            "phase": "ACTIVE_OLD" if value < 1.0 else "FRESH_IN_FLIGHT",
            "actual_x": pose[0],
            "actual_y": pose[1],
            "actual_yaw": pose[2],
            "active_chunk_id": "chunk_00" if value <= 2.0 else "chunk_01",
        }
        for index, (value, pose) in enumerate(zip(times, poses, strict=True))
    ]
    _write_csv(episode / "telemetry.csv", rows)
    old = poses[:5]
    fresh = poses[4:]
    transition_actual = poses[:5]
    raw_old = np.array(old, copy=True)
    raw_fresh = np.array(fresh, copy=True)
    for relative, value in (
        ("raw/old_actions.npy", raw_old),
        ("raw/fresh_actions.npy", raw_fresh),
        ("derived/old_world.npy", old),
        ("derived/fresh_world.npy", fresh),
        ("actual.npy", transition_actual),
    ):
        _save_npy(transition_dir / relative, value)
    _write_csv(transition_dir / "telemetry.csv", rows[:5])
    artifacts = {
        relative: sha256_file(transition_dir / relative)
        for relative in (
            "raw/old_actions.npy",
            "raw/fresh_actions.npy",
            "derived/old_world.npy",
            "derived/fresh_world.npy",
            "actual.npy",
            "telemetry.csv",
        )
    }
    metadata = {
        "episode_id": "episode_000001",
        "transition_id": "episode_000001_transition_00",
        "transition_index": 0,
        "template_id": "fixture",
        "variant_id": "V0",
        "status": "ELIGIBLE_MOVING",
        "old_chunk_id": "chunk_00",
        "fresh_chunk_id": "chunk_01",
        "waypoint_dt": None,
        "observation_pose_world_se2": poses[2].tolist(),
        "model_ready_pose_world_se2": poses[4].tolist(),
        "pose_immediately_before_switch_P_world_se2": poses[3].tolist(),
        "pose_immediately_before_switch_P_sim_time_s": 1.5,
        "switch_boundary_B_world_se2": poses[4].tolist(),
        "timing": {
            "valid": True,
            "t_obs_sim_s": 1.0,
            "t_ready_sim_s": 2.0,
            "t_switch_sim_s": 2.0,
        },
        "metrics": {
            "fresh_geometry_bin": "STRAIGHT_LIKE",
            "command_discontinuity": {"delta_v_mps": 0.1, "delta_omega_rps": -0.2},
        },
        "hashes": {
            "old_raw_sha256": chunk_content_sha256(raw_old),
            "fresh_raw_sha256": chunk_content_sha256(raw_fresh),
        },
        "artifact_sha256": artifacts,
    }
    transition_path = transition_dir / "transition.json"
    transition_path.write_text(json.dumps(metadata), encoding="utf-8")
    manifest = {
        "episodes": [
            {
                "episode_id": "episode_000001",
                "transition_count": 1,
                "telemetry_sha256": sha256_file(episode / "telemetry.csv"),
                "transition_metadata_sha256": [sha256_file(transition_path)],
            }
        ]
    }
    (run / "collection_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    return run


def two_transition_replay_fixture(tmp_path: Path) -> Path:
    run = replay_fixture(tmp_path)
    episode = run / "episodes/episode_000001"
    transition_dir = episode / "transitions/transition_01"
    with (episode / "telemetry.csv").open(encoding="utf-8", newline="") as stream:
        episode_rows = list(csv.DictReader(stream))
    poses = np.asarray(
        [
            [float(row["actual_x"]), float(row["actual_y"]), float(row["actual_yaw"])]
            for row in episode_rows
        ]
    )
    old = poses[4:]
    fresh = poses[6:]
    actual = poses[5:]
    for relative, value in (
        ("raw/old_actions.npy", old),
        ("raw/fresh_actions.npy", fresh),
        ("derived/old_world.npy", old),
        ("derived/fresh_world.npy", fresh),
        ("actual.npy", actual),
    ):
        _save_npy(transition_dir / relative, value)
    _write_csv(transition_dir / "telemetry.csv", episode_rows[5:])
    artifacts = {
        relative: sha256_file(transition_dir / relative)
        for relative in (
            "raw/old_actions.npy",
            "raw/fresh_actions.npy",
            "derived/old_world.npy",
            "derived/fresh_world.npy",
            "actual.npy",
            "telemetry.csv",
        )
    }
    metadata = {
        "episode_id": "episode_000001",
        "transition_id": "episode_000001_transition_01",
        "transition_index": 1,
        "template_id": "fixture",
        "variant_id": "V0",
        "status": "ELIGIBLE_MOVING",
        "old_chunk_id": "chunk_01",
        "fresh_chunk_id": "chunk_02",
        "waypoint_dt": None,
        "observation_pose_world_se2": poses[6].tolist(),
        "model_ready_pose_world_se2": poses[8].tolist(),
        "pose_immediately_before_switch_P_world_se2": poses[7].tolist(),
        "pose_immediately_before_switch_P_sim_time_s": 3.5,
        "switch_boundary_B_world_se2": poses[8].tolist(),
        "timing": {
            "valid": True,
            "t_obs_sim_s": 3.0,
            "t_ready_sim_s": 4.0,
            "t_switch_sim_s": 4.0,
        },
        "metrics": {
            "fresh_geometry_bin": "STRAIGHT_LIKE",
            "command_discontinuity": {"delta_v_mps": 0.1, "delta_omega_rps": -0.2},
        },
        "hashes": {
            "old_raw_sha256": chunk_content_sha256(old),
            "fresh_raw_sha256": chunk_content_sha256(fresh),
        },
        "artifact_sha256": artifacts,
    }
    transition_path = transition_dir / "transition.json"
    transition_path.write_text(json.dumps(metadata), encoding="utf-8")
    manifest_path = run / "collection_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["episodes"][0]["transition_count"] = 2
    manifest["episodes"][0]["transition_metadata_sha256"].append(
        sha256_file(transition_path)
    )
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    return run


def _tree_hashes(root: Path) -> dict[str, str]:
    return {
        str(path.relative_to(root)): sha256_file(path)
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


def test_loader_preserves_source_and_returns_read_only_arrays(tmp_path: Path) -> None:
    run = replay_fixture(tmp_path)
    before = _tree_hashes(run)
    evidence = load_replay_evidence(run, "episode_000001", 0)
    assert _tree_hashes(run) == before
    assert evidence.actual_sim_times_s[0] == 0.0
    assert evidence.candidate.t_obs_sim_s == 1.0
    assert evidence.candidate.t_switch_sim_s == 2.0
    assert evidence.actual_sim_times_s[-1] == 2.0
    assert evidence.candidate.old_chunk_id == "chunk_00"
    assert evidence.candidate.old_activation_source == "EARLIEST_BOOTSTRAP_OLD_TELEMETRY"
    assert not evidence.candidate.activation_boundary_prepended
    assert evidence.candidate.active_old_path_length_m > 0.0
    assert np.array_equal(evidence.actual_poses[-1], evidence.boundary_pose)
    assert not evidence.actual_poses.flags.writeable
    assert not evidence.old_world.flags.writeable
    assert not evidence.fresh_world.flags.writeable
    with pytest.raises(ValueError):
        evidence.actual_poses[0, 0] = 99.0


def test_later_active_old_interval_prepends_previous_but_no_previous_row(tmp_path: Path) -> None:
    run = two_transition_replay_fixture(tmp_path)
    interval = load_active_old_interval(run, "episode_000001", 1)
    assert interval.activation_source == "PREVIOUS_TRANSITION_SWITCH_BOUNDARY"
    assert interval.activation_boundary_prepended
    assert interval.activation_sim_time_s == 2.0
    assert interval.display_sim_times_s.tolist() == [2.0, 2.5, 3.0, 3.5, 4.0]
    assert interval.telemetry_sim_times_s.tolist() == [2.5, 3.0, 3.5, 4.0]
    assert all(row["active_chunk_id"] == "chunk_01" for row in interval.telemetry_rows)
    assert np.array_equal(interval.display_actual_poses[0], [2.0, 0.2, 0.02])
    assert np.array_equal(interval.display_actual_poses[-1], interval.boundary_pose_world_se2)
    assert interval.observation_sim_time_s == 3.0
    assert interval.p_sim_time_s < interval.switch_sim_time_s
    assert interval.display_sim_times_s[-1] == interval.switch_sim_time_s
    assert not interval.display_actual_poses.flags.writeable


def test_active_old_loader_rejects_a_different_active_chunk(tmp_path: Path) -> None:
    run = two_transition_replay_fixture(tmp_path)
    episode = run / "episodes/episode_000001"
    transition_dir = episode / "transitions/transition_01"
    with (episode / "telemetry.csv").open(encoding="utf-8", newline="") as stream:
        episode_rows = list(csv.DictReader(stream))
    with (transition_dir / "telemetry.csv").open(encoding="utf-8", newline="") as stream:
        transition_rows = list(csv.DictReader(stream))
    episode_rows[5]["active_chunk_id"] = "chunk_wrong"
    transition_rows[0]["active_chunk_id"] = "chunk_wrong"
    _replace_csv(episode / "telemetry.csv", episode_rows)
    _replace_csv(transition_dir / "telemetry.csv", transition_rows)

    transition_path = transition_dir / "transition.json"
    metadata = json.loads(transition_path.read_text(encoding="utf-8"))
    metadata["artifact_sha256"]["telemetry.csv"] = sha256_file(
        transition_dir / "telemetry.csv"
    )
    transition_path.write_text(json.dumps(metadata), encoding="utf-8")
    manifest_path = run / "collection_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["episodes"][0]["telemetry_sha256"] = sha256_file(episode / "telemetry.csv")
    manifest["episodes"][0]["transition_metadata_sha256"][1] = sha256_file(
        transition_path
    )
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(ValueError, match="different active chunk"):
        load_active_old_interval(run, "episode_000001", 1)


def test_saved_only_runner_has_no_inference_controller_or_physics_step() -> None:
    contract = saved_only_contract()
    assert contract == {
        "saved_only": True,
        "lightnav_inference": False,
        "controller_execution": False,
        "physics_reexecution": False,
        "spatial_scaling": False,
        "adjacent_sample_display_interpolation_only": True,
    }
    script_path = ROOT / "scripts/isaac/data02_high_motion_demo.py"
    source = script_path.read_text(encoding="utf-8")
    ast.parse(source)
    for forbidden in (
        "world.step(",
        "apply_action(",
        "DifferentialController",
        "TrajectoryFollower",
        "OnlineLightNavClient",
        "draw_heading_markers",
        "_footprint_segments",
        '"FRESH_ACTIVE"',
        '"03_after_switch.png"',
    ):
        assert forbidden not in source
    assert 'parser.set_defaults(hold=True)' in source
    assert '"display_contains_previous_chunk_actual": False' in source
    assert '"display_contains_post_switch_actual": False' in source
    launcher = (ROOT / "scripts/isaac/run_data02_high_motion_demo.sh").read_text(
        encoding="utf-8"
    )
    assert "serve_online_lightnav" not in launcher
    assert "--show-rgb" not in launcher

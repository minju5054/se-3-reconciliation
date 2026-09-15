"""Synthetic loader/playback tests only, not online collection evidence."""
import csv
import importlib.util
import json
from pathlib import Path

import numpy as np
from PIL import Image
import pytest
import yaml

from reconciliation.robotless_online import file_record, raw_manifest

SCRIPT = Path(__file__).parents[1] / "scripts/isaac/robotless_online_replay.py"
SPEC = importlib.util.spec_from_file_location("online_replay_test", SCRIPT)
replay = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(replay)


def dump(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


def csv_write(path, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)


@pytest.fixture
def dataset(tmp_path):
    run = tmp_path / "test_only_run"
    episode = run / "episodes/synthetic_episode"
    episode.mkdir(parents=True)
    (run / "config_snapshot.yaml").write_text(yaml.safe_dump({"agent": {"z_m": 0.}}))
    dump(episode / "metadata.json", {"episode_id": "synthetic_episode", "fixture_only": True})
    poses = [[i*.1, 0., 0.] for i in range(7)]
    rows = [dict(state_id=i, sim_time_s=10.+i*.1, host_monotonic_s=100.+i*.1,
                 x=p[0], y=p[1], yaw=p[2]) for i, p in enumerate(poses)]
    csv_write(episode / "execution.csv", rows)
    commands = [dict(application_state_id=i, reference_version=int(i >= 3),
                     chunk_id="fresh" if i >= 3 else "old", solve_id=f"s{i}",
                     v_mps=.1, omega_radps=0.) for i in range(6)]
    csv_write(episode / "commands.csv", commands)
    for chunk, anchor, raw in (("old", poses[0], [[.1, 0., 0.], [.7, 0., 0.]]),
                               ("fresh", poses[1], [[.1, 0., 0.], [.5, 0., 0.]])):
        folder = episode / "chunks" / chunk
        folder.mkdir(parents=True)
        np.save(folder / "raw_local.npy", raw)
        np.save(folder / "world.npy", replay.local_trajectory_to_world(anchor, raw))
        dump(folder / "metadata.json", {"observation_pose_world": anchor})
    context = dict(episode_id="synthetic_episode", event_id="handoff_000", status="VALID_HANDOFF_MOVING",
                   old_chunk_id="old", fresh_chunk_id="fresh", old_reference_version=0, fresh_reference_version=1,
                   obs_state_id=1, request_state_id=1, ready_state_id=2, switch_state_id=3,
                   previous_state_id=2, post_switch_end_state_id=6,
                   R_obs=poses[1], R_ready=poses[2], P=poses[2], B=poses[3],
                   t_obs={"sim_time_s": 10.1}, t_request={"host_monotonic_s": 100.1},
                   t_ready_host={"host_monotonic_s": 100.2}, t_ready_seen_sim={"sim_time_s": 10.2},
                   t_switch={"sim_time_s": 10.3}, actual_history_count=4, history_full=False,
                   timing_flags={"timing_valid": True, "history_valid": True, "overlap_observed": True})
    for prefix in ("old", "fresh"):
        for suffix, filename in (("raw_local", "raw_local.npy"), ("world", "world.npy")):
            context[f"{prefix}_{suffix}_ref"] = file_record(episode / "chunks" / prefix / filename, episode)
    dump(episode / "handoffs/handoff_000/context.json", context)
    rgb = episode / "rgb/frame_0.jpg"
    rgb.parent.mkdir()
    Image.new("RGB", (12, 8), "red").save(rgb)
    csv_write(episode / "rgb_index.csv", [dict(frame_id="frame_0", rendered_state_id=2,
                                              path="rgb/frame_0.jpg", sha256=replay.file_hash(rgb))])
    dump(episode / "completion.json", {"status": "TEST_ONLY", "raw_manifest": raw_manifest(episode)})
    return run, episode


def load(dataset):
    return replay.SavedEpisode(dataset[0], "synthetic_episode")


def test_saved_loader_uses_actual_samples_and_exact_boundary(dataset):
    saved = load(dataset)
    np.testing.assert_array_equal(saved.poses[3], saved.events[0]["context"]["B"])
    assert saved.events[0]["metrics"]["post_switch_execution"]["stream_rows_inclusive"] == [3, 6]
    assert not saved.poses.flags.writeable
    assert not saved.chunks["fresh"].flags.writeable
    assert saved.event_at_row()["context"]["event_id"] == "handoff_000"


def test_replay_clock_selects_recorded_row_without_interpolation(dataset):
    saved = load(dataset)
    saved.playing = True
    saved.advance(.15)
    assert saved.index == 1
    np.testing.assert_array_equal(saved.poses[saved.index], [.1, 0, 0])
    saved.advance(1.)
    assert saved.index == 6 and saved.playing is False
    saved.reset()
    assert saved.index == 0 and saved.replay_sim_s == 10. and saved.speed == 1.


def test_pause_and_display_speed_do_not_generate_states(dataset):
    saved = load(dataset)
    saved.advance(5.)
    assert saved.index == 0
    saved.speed = 2.; saved.playing = True
    saved.advance(.11)
    assert saved.index == 2
    assert len(saved.poses) == 7


def test_saved_rgb_cannot_appear_before_rendered_state(dataset):
    saved = load(dataset)
    assert saved.latest_rgb() is None
    saved.seek_row(2)
    row, path = saved.latest_rgb()
    assert row["rendered_state_id"] == "2"
    assert path == dataset[1] / "rgb/frame_0.jpg"


@pytest.mark.parametrize("value", [-1, 7, .5])
def test_seek_never_clamps_or_fabricates_outside_stream(dataset, value):
    with pytest.raises(ValueError):
        load(dataset).seek_row(value)


@pytest.mark.parametrize("dt", [-.1, np.inf, np.nan])
def test_display_clock_invalid_values_fail_closed(dataset, dt):
    with pytest.raises(ValueError):
        load(dataset).advance(dt)


def test_source_change_and_incomplete_episode_are_rejected(dataset):
    run, episode = dataset
    saved = load(dataset)
    (episode / "rgb/frame_0.jpg").write_bytes(b"changed")
    with pytest.raises(ValueError, match="changed"):
        saved.verify_unchanged()
    with pytest.raises(ValueError, match="changed"):
        load(dataset)
    (episode / "completion.json").unlink()
    with pytest.raises(FileExistsError):
        load(dataset)


def test_source_boundary_must_be_exact_actual_state_not_nearby_projection(dataset):
    path = dataset[1] / "handoffs/handoff_000/context.json"
    context = replay.read_json(path)
    context["B"][0] += 1e-12
    dump(path, context)
    dump(dataset[1] / "completion.json", {"raw_manifest": raw_manifest(dataset[1])})
    with pytest.raises(ValueError, match="exact actual"):
        load(dataset)


def test_invalid_attempt_without_activation_is_preserved_with_reason(dataset):
    dump(dataset[1] / "handoffs/handoff_001/context.json", {"status": "MODEL_STOP"})
    saved = load(dataset)
    assert len(saved.events) == 1
    assert saved.unrenderable_events[0]["status"] == "MODEL_STOP"
    assert "no complete activated" in saved.unrenderable_events[0]["reason"]


def test_actual_worker_nested_observation_anchor_is_supported(dataset):
    episode = dataset[1]
    for chunk in ("old", "fresh"):
        path = episode / "chunks" / chunk / "metadata.json"
        meta = replay.read_json(path)
        dump(path, {"observation": {"pose_world": meta["observation_pose_world"]}})
    dump(episode / "completion.json", {"raw_manifest": raw_manifest(episode)})
    assert len(load(dataset).chunks) == 2


def test_nonactivated_ready_context_with_null_boundary_is_preserved(dataset):
    source = dataset[1] / "handoffs/handoff_000/context.json"
    context = replay.read_json(source)
    context.update(event_id="handoff_001", status="TECHNICAL_INVALID", switch_state_id=None, B=None)
    dump(dataset[1] / "handoffs/handoff_001/context.json", context)
    saved = load(dataset)
    assert len(saved.events) == len(saved.unrenderable_events) == 1


def test_own_anchor_verification_independent_of_raw_manifest(dataset):
    run, episode = dataset
    path = episode / "chunks/fresh/world.npy"
    np.save(path, np.load(path) + [.01, 0, 0])
    dump(episode / "completion.json", {"raw_manifest": raw_manifest(episode)})
    with pytest.raises(ValueError, match="capture-anchor"):
        load(dataset)


def test_episode_path_may_not_escape_run(dataset):
    with pytest.raises(ValueError, match="direct episode"):
        replay.SavedEpisode(dataset[0], "../../elsewhere")


@pytest.mark.parametrize("aspect", [.5, 1., 1.6, 2.])
def test_overview_fits_source_extent_with_equal_xy_scale(aspect):
    poses = np.array([[19., 20., 0.], [19.2, 24., 1.]])
    before = poses.copy()
    center, spans = replay.overview_aperture(poses, aspect)
    assert spans[0] / spans[1] == pytest.approx(aspect)
    assert np.all(np.abs(poses[:, :2] - center) < np.asarray(spans) / 2)
    np.testing.assert_array_equal(poses, before)

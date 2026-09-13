"""Synthetic timing fixtures only; not experimental video evidence."""

from copy import deepcopy
import pytest
from reconciliation.video_timing import validate_video_frames


def fixture():
    times = [i / 60 for i in range(7)]
    poses = [[i / 100, 0, 0] for i in range(7)]
    frames = [{"physics_index": i, "sim_time_s": times[i], "pose_world_se2": poses[i]}
              for i in (0, 3, 6)]
    return frames, times, poses


def test_frame_times_and_poses_must_match_measurements():
    frames, times, poses = fixture()
    result = validate_video_frames(frames, 1 / 60, 3, times, poses)
    assert result["nominal_simulation_fps"] == 20
    assert result["last_sim_time_s"] == .1


@pytest.mark.parametrize("change", ["missing", "duplicate", "order", "time", "pose"])
def test_reject_fabricated_or_misaligned_frame_evidence(change):
    frames, times, poses = fixture()
    frames = deepcopy(frames)
    if change == "missing":
        frames.pop()
    elif change == "duplicate":
        frames.append(deepcopy(frames[-1]))
    elif change == "order":
        frames.reverse()
    elif change == "time":
        frames[1]["sim_time_s"] += .01
    else:
        frames[1]["pose_world_se2"][0] += .1
    with pytest.raises(ValueError):
        validate_video_frames(frames, 1 / 60, 3, times, poses)


@pytest.mark.parametrize("stride", [True, 0, -1, 1.5])
def test_reject_invalid_sampling_convention(stride):
    frames, times, poses = fixture()
    with pytest.raises(ValueError):
        validate_video_frames(frames, 1 / 60, stride, times, poses)

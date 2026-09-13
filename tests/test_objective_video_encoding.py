"""Synthetic movie-clock checks; not experimental evidence."""

import importlib.util
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location("objective_encoder", Path(__file__).resolve().parents[1] /
                                             "scripts/encode_exp02d_objective_videos.py")
encoder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(encoder)


def test_same_start_clock_and_end_card_at_measured_goal_time():
    # 20 simulation samples/s, played at 10 samples/s: 4.4 s becomes 8.8 s.
    long = encoder.synchronized_frame_indices(89, 88, 20)
    short = encoder.synchronized_frame_indices(85, 88, 20)
    assert long[:84] == short[:84] == list(range(84))
    assert long[84:88] == [84, 85, 86, 87]
    assert short[84:] == [None] * 24
    assert long[88:] == [None] * 20
    assert len(long) == len(short) == 108  # 10.8 s including 2 s final hold


def test_initially_finished_trial_is_only_its_measured_end_card():
    assert encoder.synchronized_frame_indices(1, 0, 20) == [None] * 20


@pytest.mark.parametrize("values", [(0, 1, 20), (89, 87, 20), (89, 88, 0)])
def test_reject_invalid_video_timing(values):
    with pytest.raises(ValueError):
        encoder.synchronized_frame_indices(*values)

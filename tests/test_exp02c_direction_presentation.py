"""Synthetic display-geometry checks only; not experimental evidence."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

from reconciliation.transition_graph import incoming_motion_residual

spec = importlib.util.spec_from_file_location(
    "exp02c_display", Path(__file__).resolve().parents[1] / "scripts/view_exp02c_direction_mechanism.py"
)
display = importlib.util.module_from_spec(spec)
spec.loader.exec_module(display)


@pytest.mark.parametrize("p,b,f,q", [
    ([-.1, 0, 0], [0, 0, 0], [-.2, 0, 0], [.3, 0, 0]),
    ([2, 3.1, -1.57], [2, 3, -1.57], [2, 3.2, -1.57], [2, 2.7, -1.57]),
])
def test_behind_entry_diagnostic_target_satisfies_actual_residual(p, b, f, q):
    result = display.direction_geometry(p, b, f, q)
    target = [*result["direction_zero_target_world_xy"], f[2]]
    np.testing.assert_allclose(incoming_motion_residual(p, b, target, f, minimum_translation_m=1e-6)[:2],
                               [0, 0], rtol=0, atol=1e-12)
    assert result["raw_entry_forward_projection_m"] < 0
    assert result["raw_follower_target_forward_projection_m"] > 0
    assert result["incoming_vs_raw_entry_angle_deg"] == pytest.approx(180)
    assert result["raw_to_direction_target_m"] == pytest.approx(.4)


def test_forward_entry_is_already_the_direction_target():
    result = display.direction_geometry([-.1, 0, 0], [0, 0, 0], [.2, 0, 0], [.5, 0, 0])
    assert result["incoming_vs_raw_entry_angle_deg"] == pytest.approx(0)
    assert result["raw_to_direction_target_m"] == pytest.approx(0)


@pytest.mark.parametrize("p,b,f,q", [
    ([0, 0, 0], [0, 0, 0], [-.2, 0, 0], [.3, 0, 0]),
    ([-.1, 0, 0], [0, 0, 0], [0, 0, 0], [.3, 0, 0]),
    ([-.1, 0, 0], [0, 0, 0], [float("nan"), 0, 0], [.3, 0, 0]),
])
def test_degenerate_or_nonfinite_geometry_is_rejected(p, b, f, q):
    with pytest.raises(ValueError):
        display.direction_geometry(p, b, f, q)

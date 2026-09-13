"""Synthetic display-clock fixtures, not simulation or experimental evidence."""

import pytest
from reconciliation.physics_display import render_paused


class SyntheticWorld:
    def __init__(self, broken=False):
        self.current_time = 12.5
        self.playing = True
        self.frames = 0
        self.broken = broken

    def pause(self):
        self.playing = False

    def render(self):
        if self.playing or self.broken:
            self.current_time += 1 / 60
        self.frames += 1


def test_render_pauses_a_playing_world_and_preserves_clock():
    world = SyntheticWorld()
    render_paused(world, 4)
    assert world.current_time == 12.5
    assert world.frames == 4
    assert not world.playing


def test_reject_a_render_route_that_steps_physics():
    with pytest.raises(RuntimeError, match="advanced simulation time"):
        render_paused(SyntheticWorld(broken=True))


@pytest.mark.parametrize("allow", [False, True])
def test_editor_stop_is_distinct_from_a_physics_step(allow):
    world = SyntheticWorld()
    world.render = lambda: setattr(world, "current_time", 0.0)
    if allow:
        assert render_paused(world, allow_editor_stop=True) is False
    else:
        with pytest.raises(RuntimeError):
            render_paused(world)


@pytest.mark.parametrize("frames", [0, -1, 1.5, True])
def test_reject_invalid_frame_count(frames):
    with pytest.raises(ValueError):
        render_paused(SyntheticWorld(), frames)

"""Render a completed physical state without advancing its simulation clock."""

from __future__ import annotations


def render_paused(world, frames: int = 1, *, allow_editor_stop: bool = False) -> bool:
    if not isinstance(frames, int) or isinstance(frames, bool) or frames < 1:
        raise ValueError("render frame count must be a positive integer")
    world.pause()
    before = float(world.current_time)
    for _ in range(frames):
        # World.render is the render-only route; raw app.update may also service
        # a timeline play request. Reassert pause before each display frame.
        world.pause()
        world.render()
        if allow_editor_stop and float(world.current_time) < before:
            # The editor's Stop action resets its clock; it is not a physics
            # step. Capture callers remain strict; interactive hold reports it.
            return False
        if float(world.current_time) != before:
            raise RuntimeError(f"paused rendering advanced simulation time: {before} -> {world.current_time}")
    return True

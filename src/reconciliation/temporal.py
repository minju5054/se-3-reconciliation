"""Discrete execution-time conventions for trajectory chunks."""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import NDArray

from reconciliation.types import TrajectoryChunk


def waypoint_execution_times(chunk: TrajectoryChunk) -> NDArray[np.float64]:
    """Return the scheduled timestamp of every waypoint.

    Convention: waypoint row ``i`` is the endpoint reached after ``i + 1`` control
    intervals, at ``observation_time + (i + 1) * waypoint_dt``. A timestamp equal to the
    availability/switch time is considered usable.
    """

    offsets = np.arange(1, chunk.horizon + 1, dtype=np.float64) * chunk.waypoint_dt
    return chunk.observation_time + offsets


def first_usable_waypoint_index(chunk: TrajectoryChunk) -> int:
    """Return the NEW suffix start, or ``chunk.horizon`` when all rows are stale."""

    times = waypoint_execution_times(chunk)
    return int(np.searchsorted(times, chunk.ready_time, side="left"))


def stale_prefix_length(chunk: TrajectoryChunk) -> int:
    """Return the number of waypoints scheduled strictly before ``ready_time``."""

    return first_usable_waypoint_index(chunk)


def last_reached_waypoint_index(chunk: TrajectoryChunk, time: float) -> int:
    """Return the last waypoint scheduled at or before ``time``.

    ``-1`` means no waypoint has yet been reached and the observation-time robot pose is
    the discrete boundary pose. Times after the horizon clamp naturally to the final row.
    """

    query_time = float(time)
    if not math.isfinite(query_time):
        raise ValueError("time must be finite")
    times = waypoint_execution_times(chunk)
    return int(np.searchsorted(times, query_time, side="right") - 1)

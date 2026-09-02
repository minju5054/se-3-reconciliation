"""Isaac-independent differential-drive command validation and wheel mapping."""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np
from numpy.typing import NDArray


FloatArray = NDArray[np.float64]


def _finite(name: str, value: float) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def differential_wheel_speeds(
    linear_velocity_mps: float,
    angular_velocity_rps: float,
    wheel_radius_m: float,
    wheel_separation_m: float,
) -> FloatArray:
    """Return ideal ``[left, right]`` wheel angular velocities in rad/s."""

    linear = _finite("linear_velocity_mps", linear_velocity_mps)
    angular = _finite("angular_velocity_rps", angular_velocity_rps)
    radius = _finite("wheel_radius_m", wheel_radius_m)
    separation = _finite("wheel_separation_m", wheel_separation_m)
    if radius <= 0.0:
        raise ValueError("wheel_radius_m must be greater than zero")
    if separation <= 0.0:
        raise ValueError("wheel_separation_m must be greater than zero")
    return np.array(
        [
            (linear - angular * separation / 2.0) / radius,
            (linear + angular * separation / 2.0) / radius,
        ],
        dtype=np.float64,
    )


def map_wheel_speeds_by_side(
    side_speeds_rad_s: Sequence[float],
    dof_sides: Sequence[str],
) -> FloatArray:
    """Expand official ``[left, right]`` output to runtime-discovered wheel DOFs."""

    speeds = np.asarray(side_speeds_rad_s, dtype=np.float64)
    if speeds.shape != (2,) or not np.all(np.isfinite(speeds)):
        raise ValueError("side_speeds_rad_s must be a finite [left, right] pair")
    if not dof_sides:
        raise ValueError("dof_sides must not be empty")
    invalid = [side for side in dof_sides if side not in {"left", "right"}]
    if invalid:
        raise ValueError(f"dof_sides contains invalid values: {invalid}")
    if dof_sides.count("left") != 2 or dof_sides.count("right") != 2:
        raise ValueError("Jackal mapping requires exactly two left and two right wheel DOFs")
    return np.asarray(
        [speeds[0] if side == "left" else speeds[1] for side in dof_sides],
        dtype=np.float64,
    )

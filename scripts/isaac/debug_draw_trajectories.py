"""Small reusable DebugDraw helpers shared by Isaac trajectory diagnostics."""

from __future__ import annotations

import math
from typing import Any, Sequence

import numpy as np


def rgba(values: Sequence[float]) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != 4 or not all(math.isfinite(value) for value in result):
        raise ValueError("DebugDraw colors must be finite RGBA")
    return result


def xyz_points(poses: np.ndarray, z: float) -> list[list[float]]:
    values = np.asarray(poses, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] < 2 or not np.all(np.isfinite(values)):
        raise ValueError("DebugDraw poses must be a finite array with at least two columns")
    return [[float(pose[0]), float(pose[1]), float(z)] for pose in values]


def draw_polyline(draw: Any, poses: np.ndarray, *, z: float, color, width: float) -> None:
    points = xyz_points(poses, z)
    if len(points) > 1:
        draw.draw_lines(
            points[:-1],
            points[1:],
            [rgba(color)] * (len(points) - 1),
            [float(width)] * (len(points) - 1),
        )


def draw_heading_markers(
    draw: Any,
    poses: np.ndarray,
    *,
    z: float,
    color,
    width: float,
    length_m: float,
    stride: int = 1,
) -> None:
    values = np.asarray(poses, dtype=np.float64)
    if stride < 1:
        raise ValueError("heading stride must be positive")
    selected = values[::stride]
    starts = xyz_points(selected, z)
    ends = [
        [
            point[0] + float(length_m) * math.cos(float(pose[2])),
            point[1] + float(length_m) * math.sin(float(pose[2])),
            float(z),
        ]
        for point, pose in zip(starts, selected, strict=True)
    ]
    if starts:
        draw.draw_lines(
            starts,
            ends,
            [rgba(color)] * len(starts),
            [float(width)] * len(starts),
        )


def draw_pose_points(
    draw: Any,
    poses: np.ndarray,
    *,
    z: float,
    color,
    size: float,
) -> None:
    points = xyz_points(poses, z)
    if points:
        draw.draw_points(points, [rgba(color)] * len(points), [float(size)] * len(points))

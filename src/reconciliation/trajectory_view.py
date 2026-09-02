"""Inspect, plot, and export canonical SE(2) trajectory NPY files."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np
from numpy.typing import NDArray

from reconciliation.se2 import wrap_angle
from reconciliation.trajectory import FloatArray, validate_se2_trajectory


@dataclass(frozen=True, slots=True)
class TrajectorySummary:
    """Human-readable statistics for one stored trajectory."""

    sample_count: int
    start_pose: NDArray[np.float64]
    end_pose: NDArray[np.float64]
    endpoint_displacement_m: float
    path_length_m: float
    wrapped_yaw_change_rad: float


def load_trajectory_npy(path: str | Path) -> FloatArray:
    """Load one non-pickled ``.npy`` file and validate canonical ``N x 3`` poses."""

    source = Path(path).expanduser()
    if source.suffix.lower() != ".npy":
        raise ValueError(f"trajectory file must use the .npy extension: {source}")
    loaded = np.load(source, allow_pickle=False)
    if not isinstance(loaded, np.ndarray):
        raise ValueError(f"trajectory file must contain one NumPy array: {source}")
    return validate_se2_trajectory(loaded, name=str(source))


def summarize_trajectory(trajectory: NDArray[np.float64]) -> TrajectorySummary:
    """Compute descriptive values without changing or resampling the trajectory."""

    poses = validate_se2_trajectory(trajectory)
    xy_steps = np.diff(poses[:, :2], axis=0)
    return TrajectorySummary(
        sample_count=int(poses.shape[0]),
        start_pose=poses[0].copy(),
        end_pose=poses[-1].copy(),
        endpoint_displacement_m=float(np.linalg.norm(poses[-1, :2] - poses[0, :2])),
        path_length_m=float(np.linalg.norm(xy_steps, axis=1).sum()),
        wrapped_yaw_change_rad=float(wrap_angle(poses[-1, 2] - poses[0, 2])),
    )


def format_trajectory_report(
    path: str | Path,
    trajectory: NDArray[np.float64],
    *,
    rows: int = 5,
) -> str:
    """Format summary plus leading/trailing stored rows for terminal inspection."""

    if rows < 0:
        raise ValueError("rows must be non-negative")
    poses = validate_se2_trajectory(trajectory)
    summary = summarize_trajectory(poses)
    lines = [
        f"file: {Path(path).expanduser()}",
        f"shape: {poses.shape}",
        "columns: [x_m, y_m, yaw_rad]",
        f"start: {np.array2string(summary.start_pose, precision=8)}",
        f"end: {np.array2string(summary.end_pose, precision=8)}",
        f"endpoint displacement: {summary.endpoint_displacement_m:.8f} m",
        f"path length: {summary.path_length_m:.8f} m",
        f"wrapped yaw change: {summary.wrapped_yaw_change_rad:.8f} rad",
    ]
    if rows == 0:
        return "\n".join(lines)

    count = poses.shape[0]
    if count <= 2 * rows:
        indices = list(range(count))
    else:
        indices = list(range(rows)) + list(range(count - rows, count))
    lines.append("rows:")
    for offset, index in enumerate(indices):
        if count > 2 * rows and offset == rows:
            lines.append("  ...")
        x, y, yaw = poses[index]
        lines.append(f"  {index:6d}: [{x:.8f}, {y:.8f}, {yaw:.8f}]")
    return "\n".join(lines)


def export_trajectory_csv(
    path: str | Path,
    trajectory: NDArray[np.float64],
    *,
    overwrite: bool = False,
) -> Path:
    """Export stored rows as ``x,y,yaw`` without interpolation or transformation."""

    destination = Path(path).expanduser()
    poses = validate_se2_trajectory(trajectory)
    destination.parent.mkdir(parents=True, exist_ok=True)
    mode = "w" if overwrite else "x"
    with destination.open(mode, newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("x", "y", "yaw"))
        writer.writerows(poses)
    return destination


def plot_trajectories(
    trajectories: Sequence[tuple[str, NDArray[np.float64]]],
):
    """Plot stored XY paths and yaw rows; return the Matplotlib figure."""

    if not trajectories:
        raise ValueError("at least one trajectory is required")

    from matplotlib import pyplot as plt

    figure, (path_axis, yaw_axis) = plt.subplots(1, 2, figsize=(12, 5.5))
    for label, trajectory in trajectories:
        poses = validate_se2_trajectory(trajectory, name=label)
        sample_indices = np.arange(poses.shape[0])
        (line,) = path_axis.plot(poses[:, 0], poses[:, 1], linewidth=2, label=label)
        path_axis.scatter(
            poses[0, 0], poses[0, 1], color=line.get_color(), marker="o", s=55
        )
        path_axis.scatter(
            poses[-1, 0], poses[-1, 1], color=line.get_color(), marker="X", s=70
        )
        yaw_axis.plot(sample_indices, poses[:, 2], linewidth=2, label=label)

    path_axis.set_title("SE(2) trajectory — circle: start, X: end")
    path_axis.set_xlabel("world/local x [m]")
    path_axis.set_ylabel("world/local y [m]")
    path_axis.axis("equal")
    path_axis.grid(True, alpha=0.3)
    path_axis.legend()
    yaw_axis.set_title("Stored yaw (no unwrap or interpolation)")
    yaw_axis.set_xlabel("sample index")
    yaw_axis.set_ylabel("yaw [rad]")
    yaw_axis.grid(True, alpha=0.3)
    yaw_axis.legend()
    figure.tight_layout()
    return figure

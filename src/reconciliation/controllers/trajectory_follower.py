"""Lightweight feedback follower for arbitrary canonical SE(2) trajectories."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import ArrayLike

from reconciliation.se2 import wrap_angle
from reconciliation.trajectory import FloatArray, validate_pose_se2, validate_se2_trajectory


def _positive(name: str, value: float) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and greater than zero")
    return result


@dataclass(frozen=True, slots=True)
class FollowerConfig:
    lookahead_distance_m: float
    position_gain: float
    heading_gain: float
    cross_track_gain: float
    max_linear_velocity_mps: float
    max_angular_velocity_rps: float
    goal_position_tolerance_m: float
    goal_yaw_tolerance_rad: float
    rotate_in_place_threshold_rad: float
    nearest_search_window: int

    def __post_init__(self) -> None:
        for name in (
            "lookahead_distance_m",
            "position_gain",
            "heading_gain",
            "cross_track_gain",
            "max_linear_velocity_mps",
            "max_angular_velocity_rps",
            "goal_position_tolerance_m",
            "goal_yaw_tolerance_rad",
            "rotate_in_place_threshold_rad",
        ):
            object.__setattr__(self, name, _positive(name, getattr(self, name)))
        if not isinstance(self.nearest_search_window, int) or self.nearest_search_window < 1:
            raise ValueError("nearest_search_window must be a positive integer")


@dataclass(frozen=True, slots=True)
class FollowerCommand:
    linear_velocity_mps: float
    angular_velocity_rps: float
    nearest_index: int
    target_index: int
    goal_reached: bool


def nearest_progress_index(
    trajectory: ArrayLike,
    current_pose: ArrayLike,
    previous_index: int,
    search_window: int,
) -> int:
    """Return a nearest future index without ever regressing progress."""

    poses = validate_se2_trajectory(trajectory)
    current = validate_pose_se2(current_pose)
    if previous_index < 0 or previous_index >= poses.shape[0]:
        raise ValueError("previous_index is outside the trajectory")
    if search_window < 1:
        raise ValueError("search_window must be positive")
    stop = min(poses.shape[0], previous_index + search_window + 1)
    distances = np.linalg.norm(poses[previous_index:stop, :2] - current[:2], axis=1)
    return previous_index + int(np.argmin(distances))


def lookahead_index(
    cumulative_distance_m: ArrayLike,
    progress_index: int,
    lookahead_distance_m: float,
) -> int:
    cumulative = np.asarray(cumulative_distance_m, dtype=np.float64)
    if cumulative.ndim != 1 or cumulative.shape[0] < 1 or not np.all(np.isfinite(cumulative)):
        raise ValueError("cumulative_distance_m must be a finite non-empty vector")
    if np.any(np.diff(cumulative) < 0.0):
        raise ValueError("cumulative_distance_m must be non-decreasing")
    if progress_index < 0 or progress_index >= cumulative.shape[0]:
        raise ValueError("progress_index is outside cumulative_distance_m")
    distance = _positive("lookahead_distance_m", lookahead_distance_m)
    target_distance = cumulative[progress_index] + distance
    return min(int(np.searchsorted(cumulative, target_distance, side="left")), cumulative.shape[0] - 1)


class TrajectoryFollower:
    """Monotonic nearest/lookahead SE(2) follower with terminal yaw alignment."""

    def __init__(self, trajectory: ArrayLike, config: FollowerConfig) -> None:
        self.trajectory: FloatArray = validate_se2_trajectory(trajectory)
        self.config = config
        self._progress_index = 0
        steps = np.linalg.norm(np.diff(self.trajectory[:, :2], axis=0), axis=1)
        self._cumulative_distance = np.concatenate(([0.0], np.cumsum(steps)))

    @property
    def progress_index(self) -> int:
        return self._progress_index

    def forward(self, current_pose: ArrayLike) -> FollowerCommand:
        current = validate_pose_se2(current_pose)
        self._progress_index = nearest_progress_index(
            self.trajectory,
            current,
            self._progress_index,
            self.config.nearest_search_window,
        )
        goal = self.trajectory[-1]
        goal_distance = float(np.linalg.norm(goal[:2] - current[:2]))
        goal_yaw_error = float(wrap_angle(goal[2] - current[2]))
        if (
            goal_distance <= self.config.goal_position_tolerance_m
            and abs(goal_yaw_error) <= self.config.goal_yaw_tolerance_rad
        ):
            self._progress_index = self.trajectory.shape[0] - 1
            return FollowerCommand(0.0, 0.0, self._progress_index, self._progress_index, True)
        if goal_distance <= self.config.goal_position_tolerance_m:
            angular = float(
                np.clip(
                    self.config.heading_gain * goal_yaw_error,
                    -self.config.max_angular_velocity_rps,
                    self.config.max_angular_velocity_rps,
                )
            )
            return FollowerCommand(0.0, angular, self._progress_index, len(self.trajectory) - 1, False)

        target_index = lookahead_index(
            self._cumulative_distance,
            self._progress_index,
            self.config.lookahead_distance_m,
        )
        target = self.trajectory[target_index]
        delta = target[:2] - current[:2]
        distance = float(np.linalg.norm(delta))
        heading_to_target = math.atan2(float(delta[1]), float(delta[0]))
        heading_error = float(wrap_angle(heading_to_target - current[2]))
        nearest = self.trajectory[self._progress_index]
        current_offset = current[:2] - nearest[:2]
        cross_track = float(
            -math.sin(float(nearest[2])) * current_offset[0]
            + math.cos(float(nearest[2])) * current_offset[1]
        )
        cross_track_correction = math.atan2(
            -cross_track,
            self.config.lookahead_distance_m,
        )
        angular = self.config.heading_gain * heading_error + self.config.cross_track_gain * cross_track_correction
        angular = float(
            np.clip(
                angular,
                -self.config.max_angular_velocity_rps,
                self.config.max_angular_velocity_rps,
            )
        )
        linear = min(self.config.max_linear_velocity_mps, self.config.position_gain * distance)
        if abs(heading_error) >= self.config.rotate_in_place_threshold_rad:
            linear = 0.0
        else:
            linear *= max(0.0, math.cos(heading_error))
        return FollowerCommand(float(linear), angular, self._progress_index, target_index, False)

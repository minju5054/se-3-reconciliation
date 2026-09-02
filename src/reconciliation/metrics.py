"""Raw OLD-to-NEW switching metrics for EXP-01."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reconciliation.se2 import local_trajectory_to_world, wrap_angle
from reconciliation.temporal import first_usable_waypoint_index, last_reached_waypoint_index
from reconciliation.types import TrajectoryChunk


def _pose(value: ArrayLike, name: str) -> NDArray[np.float64]:
    pose = np.asarray(value, dtype=np.float64)
    if pose.shape != (3,) or not np.all(np.isfinite(pose)):
        raise ValueError(f"{name} must be a finite pose with shape (3,)")
    return pose


@dataclass(frozen=True, slots=True)
class TransitionMetrics:
    """Pose mismatch and motion mismatch kept as distinct quantities."""

    translation_gap_m: float
    yaw_gap_rad: float
    previous_old_segment_translation_m: float
    first_new_segment_translation_m: float
    translational_motion_jump_m: float
    previous_old_yaw_increment_rad: float
    first_new_yaw_increment_rad: float
    yaw_motion_jump_rad: float

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class RawSwitchAnalysis:
    """Indices, boundary poses, and metrics for one discrete raw switch."""

    switch_time: float
    old_boundary_index: int
    new_first_usable_index: int
    new_stale_prefix_length: int
    old_horizon_exhausted: bool
    old_boundary_pose_world: NDArray[np.float64]
    new_first_pose_world: NDArray[np.float64]
    metrics: TransitionMetrics

    def to_dict(self) -> dict[str, Any]:
        return {
            "experiment": "EXP-01",
            "baseline": "naive_raw_switch",
            "temporal_convention": (
                "waypoint i executes at observation_time + (i + 1) * waypoint_dt; "
                "timestamps equal to ready_time are usable"
            ),
            "switch_time": self.switch_time,
            "old_boundary_index": self.old_boundary_index,
            "new_first_usable_index": self.new_first_usable_index,
            "new_stale_prefix_length": self.new_stale_prefix_length,
            "old_horizon_exhausted": self.old_horizon_exhausted,
            "old_boundary_pose_world": self.old_boundary_pose_world.tolist(),
            "new_first_pose_world": self.new_first_pose_world.tolist(),
            "metrics": self.metrics.to_dict(),
        }


def compute_transition_metrics(
    old_previous_pose: ArrayLike,
    old_boundary_pose: ArrayLike,
    new_first_pose: ArrayLike,
    new_second_pose: ArrayLike,
) -> TransitionMetrics:
    """Compute pose discontinuity and adjacent-segment motion discontinuity."""

    old_previous = _pose(old_previous_pose, "old_previous_pose")
    old_boundary = _pose(old_boundary_pose, "old_boundary_pose")
    new_first = _pose(new_first_pose, "new_first_pose")
    new_second = _pose(new_second_pose, "new_second_pose")

    translation_gap = float(np.linalg.norm(new_first[:2] - old_boundary[:2]))
    yaw_gap = abs(float(wrap_angle(new_first[2] - old_boundary[2])))
    old_motion = float(np.linalg.norm(old_boundary[:2] - old_previous[:2]))
    new_motion = float(np.linalg.norm(new_second[:2] - new_first[:2]))
    old_yaw_increment = float(wrap_angle(old_boundary[2] - old_previous[2]))
    new_yaw_increment = float(wrap_angle(new_second[2] - new_first[2]))

    return TransitionMetrics(
        translation_gap_m=translation_gap,
        yaw_gap_rad=yaw_gap,
        previous_old_segment_translation_m=old_motion,
        first_new_segment_translation_m=new_motion,
        translational_motion_jump_m=abs(new_motion - old_motion),
        previous_old_yaw_increment_rad=old_yaw_increment,
        first_new_yaw_increment_rad=new_yaw_increment,
        yaw_motion_jump_rad=abs(float(wrap_angle(new_yaw_increment - old_yaw_increment))),
    )


def analyze_raw_switch(old: TrajectoryChunk, new: TrajectoryChunk) -> RawSwitchAnalysis:
    """Analyze a naive switch at NEW readiness without smoothing or interpolation.

    OLD is sampled at the last discrete waypoint reached by ``new.ready_time``. NEW is
    transformed using its observation-time robot pose, then its strictly stale prefix is
    discarded. At least two usable NEW waypoints are required to measure its first suffix
    segment.
    """

    if old.frame != new.frame:
        raise ValueError(
            f"OLD and NEW frame labels differ ({old.frame!r} != {new.frame!r}); "
            "an explicit conversion is required before analysis"
        )

    old_world = local_trajectory_to_world(
        old.robot_pose_at_observation,
        old.poses_local,
    )
    new_world = local_trajectory_to_world(
        new.robot_pose_at_observation,
        new.poses_local,
    )

    new_first_index = first_usable_waypoint_index(new)
    if new_first_index >= new.horizon:
        raise ValueError("NEW chunk has no temporally usable waypoint at ready_time")
    if new_first_index + 1 >= new.horizon:
        raise ValueError("NEW usable suffix needs at least two waypoints for motion metrics")

    old_boundary_index = last_reached_waypoint_index(old, new.ready_time)
    if old_boundary_index < 0:
        old_boundary = old.robot_pose_at_observation
        old_previous = old.robot_pose_at_observation
    else:
        old_boundary = old_world[old_boundary_index]
        old_previous = (
            old.robot_pose_at_observation
            if old_boundary_index == 0
            else old_world[old_boundary_index - 1]
        )

    new_first = new_world[new_first_index]
    new_second = new_world[new_first_index + 1]
    old_last_time = old.observation_time + old.horizon * old.waypoint_dt

    return RawSwitchAnalysis(
        switch_time=new.ready_time,
        old_boundary_index=old_boundary_index,
        new_first_usable_index=new_first_index,
        new_stale_prefix_length=new_first_index,
        old_horizon_exhausted=new.ready_time > old_last_time,
        old_boundary_pose_world=np.asarray(old_boundary, dtype=np.float64).copy(),
        new_first_pose_world=np.asarray(new_first, dtype=np.float64).copy(),
        metrics=compute_transition_metrics(old_previous, old_boundary, new_first, new_second),
    )

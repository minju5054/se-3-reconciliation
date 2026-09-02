"""Canonical trajectory-chunk data types."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping

import numpy as np
from numpy.typing import NDArray


FloatArray = NDArray[np.float64]


def _finite_float(name: str, value: float) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True, slots=True)
class TrajectoryChunk:
    """One robot-local waypoint chunk and the timing/frame metadata that produced it.

    ``inference_latency`` is retained explicitly for auditability and must equal
    ``ready_time - observation_time`` within floating-point tolerance.
    """

    poses_local: FloatArray
    observation_time: float
    ready_time: float
    inference_latency: float
    waypoint_dt: float
    robot_pose_at_observation: FloatArray
    frame: str
    source: str

    def __post_init__(self) -> None:
        poses = np.asarray(self.poses_local, dtype=np.float64)
        if poses.ndim != 2 or poses.shape[1:] != (3,) or poses.shape[0] < 1:
            raise ValueError("poses_local must have shape (N, 3) with N >= 1")
        if not np.all(np.isfinite(poses)):
            raise ValueError("poses_local must contain only finite values")

        robot_pose = np.asarray(self.robot_pose_at_observation, dtype=np.float64)
        if robot_pose.shape != (3,):
            raise ValueError("robot_pose_at_observation must have shape (3,)")
        if not np.all(np.isfinite(robot_pose)):
            raise ValueError("robot_pose_at_observation must contain only finite values")

        observation_time = _finite_float("observation_time", self.observation_time)
        ready_time = _finite_float("ready_time", self.ready_time)
        inference_latency = _finite_float("inference_latency", self.inference_latency)
        waypoint_dt = _finite_float("waypoint_dt", self.waypoint_dt)

        if ready_time < observation_time:
            raise ValueError("ready_time must be greater than or equal to observation_time")
        if inference_latency < 0.0:
            raise ValueError("inference_latency must be non-negative")
        expected_latency = ready_time - observation_time
        if not math.isclose(inference_latency, expected_latency, rel_tol=1e-9, abs_tol=1e-9):
            raise ValueError("inference_latency must equal ready_time - observation_time")
        if waypoint_dt <= 0.0:
            raise ValueError("waypoint_dt must be greater than zero")
        if not isinstance(self.frame, str) or not self.frame.strip():
            raise ValueError("frame must be a non-empty string")
        if not isinstance(self.source, str) or not self.source.strip():
            raise ValueError("source must be a non-empty string")

        poses = poses.copy()
        robot_pose = robot_pose.copy()
        poses.setflags(write=False)
        robot_pose.setflags(write=False)
        object.__setattr__(self, "poses_local", poses)
        object.__setattr__(self, "robot_pose_at_observation", robot_pose)
        object.__setattr__(self, "observation_time", observation_time)
        object.__setattr__(self, "ready_time", ready_time)
        object.__setattr__(self, "inference_latency", inference_latency)
        object.__setattr__(self, "waypoint_dt", waypoint_dt)
        object.__setattr__(self, "frame", self.frame.strip())
        object.__setattr__(self, "source", self.source.strip())

    @property
    def horizon(self) -> int:
        """Number of waypoints, without assuming a model-specific fixed horizon."""

        return int(self.poses_local.shape[0])

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable representation."""

        return {
            "schema_version": 1,
            "poses_local": self.poses_local.tolist(),
            "observation_time": self.observation_time,
            "ready_time": self.ready_time,
            "inference_latency": self.inference_latency,
            "waypoint_dt": self.waypoint_dt,
            "robot_pose_at_observation": self.robot_pose_at_observation.tolist(),
            "frame": self.frame,
            "source": self.source,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "TrajectoryChunk":
        """Validate and construct a chunk from decoded JSON data."""

        schema_version = data.get("schema_version", 1)
        if schema_version != 1:
            raise ValueError(f"unsupported trajectory chunk schema_version: {schema_version}")
        required = {
            "poses_local",
            "observation_time",
            "ready_time",
            "inference_latency",
            "waypoint_dt",
            "robot_pose_at_observation",
            "frame",
            "source",
        }
        missing = sorted(required.difference(data))
        if missing:
            raise ValueError(f"trajectory chunk is missing fields: {', '.join(missing)}")
        return cls(
            poses_local=data["poses_local"],
            observation_time=data["observation_time"],
            ready_time=data["ready_time"],
            inference_latency=data["inference_latency"],
            waypoint_dt=data["waypoint_dt"],
            robot_pose_at_observation=data["robot_pose_at_observation"],
            frame=data["frame"],
            source=data["source"],
        )

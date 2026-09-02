"""Minimal SE(2) operations using ``[x, y, yaw]`` poses."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray


FloatArray = NDArray[np.float64]


def _poses(value: ArrayLike, name: str) -> FloatArray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape[-1:] != (3,):
        raise ValueError(f"{name} must have shape (..., 3)")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result


def wrap_angle(angle: ArrayLike) -> float | FloatArray:
    """Wrap angle(s) to the half-open interval ``[-pi, pi)``."""

    values = np.asarray(angle, dtype=np.float64)
    if not np.all(np.isfinite(values)):
        raise ValueError("angle must contain only finite values")
    wrapped = (values + np.pi) % (2.0 * np.pi) - np.pi
    if wrapped.ndim == 0:
        return float(wrapped)
    return wrapped


def compose_poses(left: ArrayLike, right: ArrayLike) -> FloatArray:
    """Compose SE(2) poses as ``T_result = T_left * T_right``.

    Inputs may broadcast as long as their final dimension is three.
    """

    left_pose = _poses(left, "left")
    right_pose = _poses(right, "right")
    yaw = left_pose[..., 2]
    cosine = np.cos(yaw)
    sine = np.sin(yaw)
    x = left_pose[..., 0] + cosine * right_pose[..., 0] - sine * right_pose[..., 1]
    y = left_pose[..., 1] + sine * right_pose[..., 0] + cosine * right_pose[..., 1]
    result_yaw = wrap_angle(yaw + right_pose[..., 2])
    return np.stack((x, y, result_yaw), axis=-1)


def inverse_pose(pose: ArrayLike) -> FloatArray:
    """Return the inverse of one or more SE(2) poses."""

    value = _poses(pose, "pose")
    yaw = value[..., 2]
    cosine = np.cos(yaw)
    sine = np.sin(yaw)
    x = -cosine * value[..., 0] - sine * value[..., 1]
    y = sine * value[..., 0] - cosine * value[..., 1]
    return np.stack((x, y, wrap_angle(-yaw)), axis=-1)


def relative_pose(reference: ArrayLike, target: ArrayLike) -> FloatArray:
    """Express ``target`` in ``reference`` coordinates."""

    return compose_poses(inverse_pose(reference), target)


def local_trajectory_to_world(
    robot_pose_at_observation: ArrayLike,
    poses_local: ArrayLike,
) -> FloatArray:
    """Transform robot-local waypoints with the robot pose at observation time.

    This intentionally accepts only the observation-time pose. A NEW-ready robot pose is
    not part of the transform defined for LightNav predictions.
    """

    robot_pose = _poses(robot_pose_at_observation, "robot_pose_at_observation")
    if robot_pose.shape != (3,):
        raise ValueError("robot_pose_at_observation must have shape (3,)")
    trajectory = _poses(poses_local, "poses_local")
    if trajectory.ndim != 2:
        raise ValueError("poses_local must have shape (N, 3)")
    return compose_poses(robot_pose, trajectory)

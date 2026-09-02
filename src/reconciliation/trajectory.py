"""Isaac-independent deterministic SE(2) trajectory generation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reconciliation.se2 import wrap_angle


FloatArray = NDArray[np.float64]


def validate_se2_trajectory(trajectory: ArrayLike, *, name: str = "trajectory") -> FloatArray:
    """Return a copied finite ``N x 3`` ``[x, y, yaw]`` trajectory."""

    poses = np.asarray(trajectory, dtype=np.float64)
    if poses.ndim != 2 or poses.shape[1:] != (3,) or poses.shape[0] < 1:
        raise ValueError(f"{name} must have shape (N, 3) with N >= 1")
    if not np.all(np.isfinite(poses)):
        raise ValueError(f"{name} must contain only finite values")
    return poses.copy()


def validate_pose_se2(pose: ArrayLike, *, name: str = "pose") -> FloatArray:
    value = np.asarray(pose, dtype=np.float64)
    if value.shape != (3,):
        raise ValueError(f"{name} must have shape (3,)")
    if not np.all(np.isfinite(value)):
        raise ValueError(f"{name} must contain only finite values")
    return value.copy()


def _finite(name: str, value: float) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True, slots=True)
class MotionSegment:
    """One constant-command segment of a deterministic smoke trajectory."""

    name: str
    duration_s: float
    linear_velocity_mps: float
    angular_velocity_rps: float

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("motion segment name must be a non-empty string")
        duration = _finite("duration_s", self.duration_s)
        linear = _finite("linear_velocity_mps", self.linear_velocity_mps)
        angular = _finite("angular_velocity_rps", self.angular_velocity_rps)
        if duration <= 0.0:
            raise ValueError("duration_s must be greater than zero")
        object.__setattr__(self, "name", self.name.strip())
        object.__setattr__(self, "duration_s", duration)
        object.__setattr__(self, "linear_velocity_mps", linear)
        object.__setattr__(self, "angular_velocity_rps", angular)

    def to_dict(self) -> dict[str, str | float]:
        return {
            "name": self.name,
            "duration_s": self.duration_s,
            "linear_velocity_mps": self.linear_velocity_mps,
            "angular_velocity_rps": self.angular_velocity_rps,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "MotionSegment":
        required = {
            "name",
            "duration_s",
            "linear_velocity_mps",
            "angular_velocity_rps",
        }
        missing = sorted(required.difference(data))
        if missing:
            raise ValueError(f"motion segment is missing fields: {', '.join(missing)}")
        return cls(**{key: data[key] for key in required})


@dataclass(frozen=True, slots=True)
class ReferenceTrajectory:
    """Reference poses and the command applied over each following interval."""

    poses: FloatArray
    times_s: FloatArray
    linear_velocity_mps: FloatArray
    angular_velocity_rps: FloatArray
    segment_names: tuple[str, ...]
    sample_dt: float

    def __post_init__(self) -> None:
        poses = validate_se2_trajectory(self.poses, name="reference poses")
        times = np.asarray(self.times_s, dtype=np.float64)
        linear = np.asarray(self.linear_velocity_mps, dtype=np.float64)
        angular = np.asarray(self.angular_velocity_rps, dtype=np.float64)
        count = poses.shape[0]
        for name, values in (
            ("times_s", times),
            ("linear_velocity_mps", linear),
            ("angular_velocity_rps", angular),
        ):
            if values.shape != (count,) or not np.all(np.isfinite(values)):
                raise ValueError(f"{name} must be a finite array with shape ({count},)")
        if len(self.segment_names) != count:
            raise ValueError(f"segment_names must contain {count} entries")
        sample_dt = _finite("sample_dt", self.sample_dt)
        if sample_dt <= 0.0:
            raise ValueError("sample_dt must be greater than zero")
        if not np.isclose(times[0], 0.0, atol=1e-12):
            raise ValueError("reference time must start at zero")
        if count > 1 and not np.allclose(np.diff(times), sample_dt, rtol=0.0, atol=1e-9):
            raise ValueError("reference times must use the configured sample_dt")
        object.__setattr__(self, "poses", poses)
        object.__setattr__(self, "times_s", times.copy())
        object.__setattr__(self, "linear_velocity_mps", linear.copy())
        object.__setattr__(self, "angular_velocity_rps", angular.copy())
        object.__setattr__(self, "segment_names", tuple(self.segment_names))
        object.__setattr__(self, "sample_dt", sample_dt)


def generate_reference_trajectory(
    initial_pose: ArrayLike,
    segments: Sequence[MotionSegment],
    sample_dt: float,
) -> ReferenceTrajectory:
    """Integrate configurable constant ``v``/``omega`` segments with Euler unicycle steps.

    Pose row ``k`` is the state at ``t = k * sample_dt``. The command stored at row ``k``
    advances pose ``k`` to pose ``k + 1``. The final row always carries a zero command.
    Segment durations must be integral multiples of ``sample_dt`` so timing is explicit and
    no hidden interpolation or partial step is introduced.
    """

    dt = _finite("sample_dt", sample_dt)
    if dt <= 0.0:
        raise ValueError("sample_dt must be greater than zero")
    pose = validate_pose_se2(initial_pose, name="initial_pose")
    if not segments:
        raise ValueError("at least one motion segment is required")

    poses = [pose]
    times = [0.0]
    linear_commands: list[float] = []
    angular_commands: list[float] = []
    names: list[str] = []
    time_s = 0.0

    for segment in segments:
        if not isinstance(segment, MotionSegment):
            raise TypeError("segments must contain MotionSegment instances")
        interval_count_float = segment.duration_s / dt
        interval_count = int(round(interval_count_float))
        if interval_count < 1 or not math.isclose(
            interval_count_float,
            interval_count,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise ValueError(
                f"segment {segment.name!r} duration must be an integer multiple of sample_dt"
            )
        for _ in range(interval_count):
            x, y, yaw = poses[-1]
            next_pose = np.array(
                [
                    x + segment.linear_velocity_mps * math.cos(yaw) * dt,
                    y + segment.linear_velocity_mps * math.sin(yaw) * dt,
                    wrap_angle(yaw + segment.angular_velocity_rps * dt),
                ],
                dtype=np.float64,
            )
            linear_commands.append(segment.linear_velocity_mps)
            angular_commands.append(segment.angular_velocity_rps)
            names.append(segment.name)
            time_s += dt
            poses.append(next_pose)
            times.append(time_s)

    linear_commands.append(0.0)
    angular_commands.append(0.0)
    names.append("complete")
    return ReferenceTrajectory(
        poses=np.asarray(poses, dtype=np.float64),
        times_s=np.asarray(times, dtype=np.float64),
        linear_velocity_mps=np.asarray(linear_commands, dtype=np.float64),
        angular_velocity_rps=np.asarray(angular_commands, dtype=np.float64),
        segment_names=tuple(names),
        sample_dt=dt,
    )


def segments_from_config(items: Sequence[Mapping[str, Any]]) -> tuple[MotionSegment, ...]:
    if not isinstance(items, Sequence) or isinstance(items, (str, bytes)):
        raise ValueError("motion_profile must be a sequence")
    return tuple(MotionSegment.from_dict(item) for item in items)

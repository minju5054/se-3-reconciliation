"""Minimal fixed-OLD, editable-NEW SE(2) factor graph for EXP-02."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reconciliation.se2 import relative_pose, se2_log
from reconciliation.trajectory import validate_pose_se2, validate_se2_trajectory


FloatArray = NDArray[np.float64]


def _positive(value: Any, name: str, *, allow_zero: bool = False) -> float:
    result = float(value)
    if not math.isfinite(result) or result < 0.0 or (result == 0.0 and not allow_zero):
        qualifier = "nonnegative" if allow_zero else "positive"
        raise ValueError(f"{name} must be finite and {qualifier}")
    return result


@dataclass(frozen=True, slots=True)
class OracleCorrespondence:
    """Desired measurement ``Z_ij = O_i^-1 X_j``."""

    old_index: int
    new_index: int
    relative_transform: FloatArray
    rationale: str = ""
    confidence: float | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.old_index, int) or self.old_index < 0:
            raise ValueError("old_index must be a nonnegative integer")
        if not isinstance(self.new_index, int) or self.new_index < 0:
            raise ValueError("new_index must be a nonnegative integer")
        transform = validate_pose_se2(self.relative_transform, name="relative_transform")
        object.__setattr__(self, "relative_transform", transform.copy())
        if not isinstance(self.rationale, str):
            raise ValueError("rationale must be a string")
        if self.confidence is not None:
            confidence = _positive(self.confidence, "confidence")
            object.__setattr__(self, "confidence", confidence)

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["relative_transform"] = self.relative_transform.tolist()
        return result


def validate_correspondences(
    correspondences: Sequence[OracleCorrespondence],
    *,
    old_count: int,
    new_count: int,
    require_nonempty: bool = True,
) -> tuple[OracleCorrespondence, ...]:
    values = tuple(correspondences)
    if require_nonempty and not values:
        raise ValueError("at least one oracle correspondence is required")
    previous: tuple[int, int] | None = None
    seen: set[tuple[int, int]] = set()
    for correspondence in values:
        if not isinstance(correspondence, OracleCorrespondence):
            raise TypeError("correspondences must contain OracleCorrespondence values")
        pair = (correspondence.old_index, correspondence.new_index)
        if pair in seen:
            raise ValueError("duplicate correspondence")
        if correspondence.old_index >= old_count or correspondence.new_index >= new_count:
            raise ValueError("correspondence index is out of bounds")
        if previous is not None and (pair[0] <= previous[0] or pair[1] <= previous[1]):
            raise ValueError("correspondences must be strictly monotonic and non-crossing")
        seen.add(pair)
        previous = pair
    return values


@dataclass(frozen=True, slots=True)
class ResidualScales:
    translation_m: float
    yaw_rad: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "translation_m", _positive(self.translation_m, "translation_m"))
        object.__setattr__(self, "yaw_rad", _positive(self.yaw_rad, "yaw_rad"))

    @property
    def vector(self) -> FloatArray:
        return np.array([self.translation_m, self.translation_m, self.yaw_rad])


@dataclass(frozen=True, slots=True)
class GraphWeights:
    boundary: float
    correspondence: float
    new_motion: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "boundary", _positive(self.boundary, "boundary", allow_zero=True))
        object.__setattr__(
            self,
            "correspondence",
            _positive(self.correspondence, "correspondence", allow_zero=True),
        )
        object.__setattr__(
            self, "new_motion", _positive(self.new_motion, "new_motion", allow_zero=True)
        )
        if self.boundary + self.correspondence + self.new_motion == 0.0:
            raise ValueError("at least one graph weight must be positive")


@dataclass(frozen=True, slots=True)
class GraphProblem:
    fixed_old: FloatArray
    raw_new: FloatArray
    fixed_boundary: FloatArray
    correspondences: tuple[OracleCorrespondence, ...]
    scales: ResidualScales
    weights: GraphWeights

    def __post_init__(self) -> None:
        old = validate_se2_trajectory(self.fixed_old, name="fixed_old")
        new = validate_se2_trajectory(self.raw_new, name="raw_new")
        if new.shape[0] < 2:
            raise ValueError("raw_new requires at least two poses")
        boundary = validate_pose_se2(self.fixed_boundary, name="fixed_boundary")
        correspondences = validate_correspondences(
            self.correspondences,
            old_count=old.shape[0],
            new_count=new.shape[0],
            require_nonempty=self.weights.correspondence > 0.0,
        )
        object.__setattr__(self, "fixed_old", old.copy())
        object.__setattr__(self, "raw_new", new.copy())
        object.__setattr__(self, "fixed_boundary", boundary.copy())
        object.__setattr__(self, "correspondences", correspondences)


def _error(measurement: ArrayLike, predicted: ArrayLike) -> FloatArray:
    """Return ``Log(measurement^-1 * predicted)``."""

    return se2_log(relative_pose(measurement, predicted))


def boundary_residual(boundary: ArrayLike, first_new: ArrayLike) -> FloatArray:
    return _error(np.zeros(3), relative_pose(boundary, first_new))


def correspondence_residual(
    old_pose: ArrayLike, new_pose: ArrayLike, measurement: ArrayLike
) -> FloatArray:
    return _error(measurement, relative_pose(old_pose, new_pose))


def new_motion_residual(
    raw_pose: ArrayLike,
    raw_next_pose: ArrayLike,
    optimized_pose: ArrayLike,
    optimized_next_pose: ArrayLike,
) -> FloatArray:
    desired = relative_pose(raw_pose, raw_next_pose)
    predicted = relative_pose(optimized_pose, optimized_next_pose)
    return _error(desired, predicted)


def physical_factor_residuals(problem: GraphProblem, state: ArrayLike) -> dict[str, FloatArray]:
    trajectory = validate_se2_trajectory(state, name="state")
    if trajectory.shape != problem.raw_new.shape:
        raise ValueError("state shape must equal raw_new shape")
    boundary = boundary_residual(problem.fixed_boundary, trajectory[0])[None, :]
    correspondence = np.asarray(
        [
            correspondence_residual(
                problem.fixed_old[item.old_index],
                trajectory[item.new_index],
                item.relative_transform,
            )
            for item in problem.correspondences
        ],
        dtype=np.float64,
    ).reshape((-1, 3))
    motion = np.asarray(
        [
            new_motion_residual(
                problem.raw_new[index],
                problem.raw_new[index + 1],
                trajectory[index],
                trajectory[index + 1],
            )
            for index in range(trajectory.shape[0] - 1)
        ],
        dtype=np.float64,
    ).reshape((-1, 3))
    return {"boundary": boundary, "correspondence": correspondence, "new_motion": motion}


def graph_residual_vector(problem: GraphProblem, state: ArrayLike) -> FloatArray:
    groups = physical_factor_residuals(problem, state)
    scale = problem.scales.vector
    blocks: list[FloatArray] = []
    for name, weight in (
        ("boundary", problem.weights.boundary),
        ("correspondence", problem.weights.correspondence),
        ("new_motion", problem.weights.new_motion),
    ):
        if weight > 0.0 and groups[name].size:
            blocks.append((math.sqrt(weight) * groups[name] / scale).reshape(-1))
    if not blocks:
        raise ValueError("graph contains no active factor residuals")
    result = np.concatenate(blocks)
    if not np.all(np.isfinite(result)):
        raise FloatingPointError("graph residual contains NaN or Inf")
    return result


def graph_cost(problem: GraphProblem, state: ArrayLike) -> float:
    residual = graph_residual_vector(problem, state)
    return float(residual @ residual)


def problem_from_config(
    *,
    fixed_old: ArrayLike,
    raw_new: ArrayLike,
    fixed_boundary: ArrayLike,
    correspondences: Sequence[OracleCorrespondence],
    residual_scales: Mapping[str, Any],
    weights: Mapping[str, Any],
) -> GraphProblem:
    return GraphProblem(
        fixed_old=np.asarray(fixed_old, dtype=np.float64),
        raw_new=np.asarray(raw_new, dtype=np.float64),
        fixed_boundary=np.asarray(fixed_boundary, dtype=np.float64),
        correspondences=tuple(correspondences),
        scales=ResidualScales(**residual_scales),
        weights=GraphWeights(**weights),
    )

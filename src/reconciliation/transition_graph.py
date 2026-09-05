"""Minimal k-conditioned transition graphs for EXP-02A.

The graph has no OLD/FRESH correspondence factor.  ``SpatialEntryContext``
selects the suffix, while its evidence is intentionally absent from every
residual and weight calculation.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Literal, Mapping

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reconciliation.se2 import relative_pose, se2_log, wrap_angle
from reconciliation.se2_graph import new_motion_residual
from reconciliation.spatial_entry import TransitionReconciliationInput
from reconciliation.trajectory import validate_pose_se2, validate_se2_trajectory


FloatArray = NDArray[np.float64]
TransitionVariant = Literal["pose_anchor", "entry_preservation", "incoming_motion_aware"]
SUPPORTED_VARIANTS: tuple[TransitionVariant, ...] = (
    "pose_anchor",
    "entry_preservation",
    "incoming_motion_aware",
)


def _nonnegative(value: Any, name: str, *, strictly_positive: bool = False) -> float:
    result = float(value)
    if not math.isfinite(result) or result < 0.0 or (strictly_positive and result == 0.0):
        qualifier = "positive" if strictly_positive else "nonnegative"
        raise ValueError(f"{name} must be finite and {qualifier}")
    return result


@dataclass(frozen=True, slots=True)
class TransitionResidualScales:
    translation_m: float
    yaw_rad: float
    direction_unitless: float

    def __post_init__(self) -> None:
        for name in ("translation_m", "yaw_rad", "direction_unitless"):
            object.__setattr__(
                self, name, _nonnegative(getattr(self, name), name, strictly_positive=True)
            )

    @property
    def pose_vector(self) -> FloatArray:
        return np.array([self.translation_m, self.translation_m, self.yaw_rad])


@dataclass(frozen=True, slots=True)
class TransitionWeights:
    pose_anchor: float = 0.0
    entry_preservation: float = 0.0
    incoming_direction: float = 0.0
    incoming_yaw: float = 0.0
    fresh_motion: float = 0.0

    def __post_init__(self) -> None:
        for name in (
            "pose_anchor",
            "entry_preservation",
            "incoming_direction",
            "incoming_yaw",
            "fresh_motion",
        ):
            object.__setattr__(self, name, _nonnegative(getattr(self, name), name))
        if sum(
            getattr(self, name)
            for name in (
                "pose_anchor",
                "entry_preservation",
                "incoming_direction",
                "incoming_yaw",
                "fresh_motion",
            )
        ) == 0.0:
            raise ValueError("at least one transition graph weight must be positive")


def world_translation_direction(
    start: ArrayLike,
    end: ArrayLike,
    *,
    minimum_translation_m: float,
    name: str,
) -> float:
    """Return the world-plane direction and explicitly reject zero translation."""

    first = validate_pose_se2(start, name=f"{name}_start")
    second = validate_pose_se2(end, name=f"{name}_end")
    threshold = _nonnegative(
        minimum_translation_m, "minimum_translation_m", strictly_positive=True
    )
    delta = second[:2] - first[:2]
    magnitude = float(np.linalg.norm(delta))
    if magnitude <= threshold:
        raise ValueError(
            f"{name} direction is undefined for translation {magnitude:.6g} m "
            f"at or below {threshold:.6g} m"
        )
    return math.atan2(float(delta[1]), float(delta[0]))


def pose_anchor_residual(boundary: ArrayLike, entry: ArrayLike) -> FloatArray:
    """Diagnostic EXP-02-style identity anchor ``Log(B^-1 X_k)``."""

    return se2_log(relative_pose(boundary, entry))


def entry_preservation_residual(raw_entry: ArrayLike, entry: ArrayLike) -> FloatArray:
    """Selected-entry displacement ``Log(F_k^-1 X_k)``."""

    return se2_log(relative_pose(raw_entry, entry))


def incoming_motion_residual(
    previous: ArrayLike,
    boundary: ArrayLike,
    entry: ArrayLike,
    raw_entry: ArrayLike,
    *,
    minimum_translation_m: float,
) -> FloatArray:
    """Compare an entry-radius-normalized transition vector and yaw increment.

    Components 0 and 1 compare ``(X_k.xy-B.xy)/||F_k.xy-B.xy||`` with the
    incoming OLD unit direction.  Thus the target transition retains the selected
    raw entry's radial distance from B while aligning its direction with OLD. It
    never equates that distance with the magnitude of an OLD execution sample.
    Unlike a normalized-current-vector or atan2 residual, it stays finite when a
    trial update crosses B and distinguishes parallel from antiparallel motion.
    Component 2 compares SE(2) yaw increments.
    """

    incoming_direction = world_translation_direction(
        previous,
        boundary,
        minimum_translation_m=minimum_translation_m,
        name="incoming OLD motion",
    )
    raw_direction = validate_pose_se2(raw_entry, name="raw_entry")
    boundary_pose = validate_pose_se2(boundary, name="boundary")
    raw_distance = float(np.linalg.norm(raw_direction[:2] - boundary_pose[:2]))
    if raw_distance <= minimum_translation_m:
        raise ValueError(
            "raw boundary-to-entry direction is undefined for translation "
            f"{raw_distance:.6g} m at or below {minimum_translation_m:.6g} m"
        )
    candidate = validate_pose_se2(entry, name="entry")
    delta = candidate[:2] - boundary_pose[:2]
    normalized_transition = delta / raw_distance
    desired_direction = np.array(
        [math.cos(incoming_direction), math.sin(incoming_direction)], dtype=np.float64
    )
    incoming_yaw = float(relative_pose(previous, boundary_pose)[2])
    transition_yaw = float(relative_pose(boundary_pose, candidate)[2])
    return np.array(
        [
            normalized_transition[0] - desired_direction[0],
            normalized_transition[1] - desired_direction[1],
            wrap_angle(transition_yaw - incoming_yaw),
        ],
        dtype=np.float64,
    )


@dataclass(frozen=True, slots=True)
class TransitionGraphProblem:
    inputs: TransitionReconciliationInput
    variant: TransitionVariant
    scales: TransitionResidualScales
    weights: TransitionWeights
    minimum_translation_m: float = 1e-6

    def __post_init__(self) -> None:
        if self.variant not in SUPPORTED_VARIANTS:
            raise ValueError(f"unsupported transition variant: {self.variant!r}")
        minimum = _nonnegative(
            self.minimum_translation_m, "minimum_translation_m", strictly_positive=True
        )
        object.__setattr__(self, "minimum_translation_m", minimum)
        active = {
            "pose_anchor": self.weights.pose_anchor > 0.0,
            "entry_preservation": self.weights.entry_preservation > 0.0,
            "incoming_motion_aware": (
                self.weights.entry_preservation > 0.0
                and self.weights.incoming_direction > 0.0
                and self.weights.incoming_yaw > 0.0
            ),
        }
        if not active[self.variant]:
            raise ValueError(f"weights do not activate required {self.variant} factors")
        if self.weights.fresh_motion <= 0.0:
            raise ValueError("EXP-02A variants require the complete FRESH-motion chain")
        if self.variant != "pose_anchor" and self.weights.pose_anchor != 0.0:
            raise ValueError("pose_anchor weight is allowed only in the diagnostic pose-anchor variant")
        if self.variant != "incoming_motion_aware" and (
            self.weights.incoming_direction != 0.0 or self.weights.incoming_yaw != 0.0
        ):
            raise ValueError("incoming-motion weights are allowed only in incoming_motion_aware")
        if self.variant == "pose_anchor" and self.weights.entry_preservation != 0.0:
            raise ValueError("pose-anchor baseline must not include entry preservation")
        if self.variant == "incoming_motion_aware":
            # Fail before optimization if the fixed incoming/raw directions are undefined.
            incoming_motion_residual(
                self.inputs.incoming_previous_pose,
                self.inputs.committed_pose_world,
                self.raw_new[0],
                self.raw_new[0],
                minimum_translation_m=self.minimum_translation_m,
            )

    @property
    def raw_new(self) -> FloatArray:
        return self.inputs.selected_suffix


def physical_transition_residuals(
    problem: TransitionGraphProblem, state: ArrayLike
) -> dict[str, FloatArray]:
    trajectory = validate_se2_trajectory(state, name="state")
    raw = problem.raw_new
    if trajectory.shape != raw.shape:
        raise ValueError("state shape must equal the selected FRESH suffix shape")
    empty_pose = np.empty((0, 3), dtype=np.float64)
    empty_scalar = np.empty((0,), dtype=np.float64)
    groups: dict[str, FloatArray] = {
        "pose_anchor": empty_pose,
        "entry_preservation": empty_pose,
        "incoming_direction": empty_scalar,
        "incoming_yaw": empty_scalar,
    }
    if problem.variant == "pose_anchor":
        groups["pose_anchor"] = pose_anchor_residual(
            problem.inputs.committed_pose_world, trajectory[0]
        )[None, :]
    if problem.variant in ("entry_preservation", "incoming_motion_aware"):
        groups["entry_preservation"] = entry_preservation_residual(raw[0], trajectory[0])[None, :]
    if problem.variant == "incoming_motion_aware":
        incoming = incoming_motion_residual(
            problem.inputs.incoming_previous_pose,
            problem.inputs.committed_pose_world,
            trajectory[0],
            raw[0],
            minimum_translation_m=problem.minimum_translation_m,
        )
        groups["incoming_direction"] = incoming[0:2]
        groups["incoming_yaw"] = incoming[2:3]
    groups["fresh_motion"] = np.asarray(
        [
            new_motion_residual(raw[index], raw[index + 1], trajectory[index], trajectory[index + 1])
            for index in range(raw.shape[0] - 1)
        ],
        dtype=np.float64,
    ).reshape((-1, 3))
    return groups


def transition_residual_vector(problem: TransitionGraphProblem, state: ArrayLike) -> FloatArray:
    groups = physical_transition_residuals(problem, state)
    blocks: list[FloatArray] = []
    pose_scale = problem.scales.pose_vector
    for name in ("pose_anchor", "entry_preservation", "fresh_motion"):
        weight = getattr(problem.weights, name)
        if weight > 0.0 and groups[name].size:
            blocks.append((math.sqrt(weight) * groups[name] / pose_scale).reshape(-1))
    for name, scale in (
        ("incoming_direction", problem.scales.direction_unitless),
        ("incoming_yaw", problem.scales.yaw_rad),
    ):
        weight = getattr(problem.weights, name)
        if weight > 0.0:
            blocks.append(math.sqrt(weight) * groups[name] / scale)
    if not blocks:
        raise ValueError("transition graph contains no active residuals")
    result = np.concatenate(blocks)
    if not np.all(np.isfinite(result)):
        raise FloatingPointError("transition residual contains NaN or Inf")
    return result


def transition_cost(problem: TransitionGraphProblem, state: ArrayLike) -> float:
    residual = transition_residual_vector(problem, state)
    return float(residual @ residual)


def problem_from_config(
    inputs: TransitionReconciliationInput,
    variant: TransitionVariant,
    *,
    residual_scales: Mapping[str, Any],
    weights: Mapping[str, Any],
    minimum_translation_m: float,
) -> TransitionGraphProblem:
    return TransitionGraphProblem(
        inputs=inputs,
        variant=variant,
        scales=TransitionResidualScales(**residual_scales),
        weights=TransitionWeights(**weights),
        minimum_translation_m=minimum_translation_m,
    )

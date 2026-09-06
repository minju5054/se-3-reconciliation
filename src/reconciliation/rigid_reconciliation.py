"""Analytic rigid-SE(2) baseline for k-conditioned transition reconciliation."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reconciliation.se2 import compose_poses, inverse_pose, relative_pose, wrap_angle
from reconciliation.trajectory import validate_pose_se2, validate_se2_trajectory


FloatArray = NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class RigidReconciliationResult:
    """One analytic target and the rigidly transformed complete selected suffix."""

    target_entry_world: FloatArray
    correction_world: FloatArray
    trajectory_world: FloatArray
    raw_entry_radius_m: float
    incoming_direction_world_rad: float

    def to_dict(self) -> dict[str, object]:
        return {
            "target_entry_world": self.target_entry_world.tolist(),
            "correction_world": self.correction_world.tolist(),
            "raw_entry_radius_m": self.raw_entry_radius_m,
            "incoming_direction_world_rad": self.incoming_direction_world_rad,
            "construction": (
                "Xk_xy=B_xy+||Fk_xy-B_xy||*u_old; "
                "yaw(B^-1 Xk)=yaw(P^-1 B); T_rigid=Xk*Fk^-1"
            ),
        }


def analytic_rigid_reconciliation(
    previous_pose_world: ArrayLike,
    committed_pose_world: ArrayLike,
    fresh_suffix_world: ArrayLike,
    *,
    minimum_translation_m: float,
) -> RigidReconciliationResult:
    """Align the selected entry analytically and move its suffix by one SE(2).

    This is deliberately independent of every graph output.  Both the measured
    incoming translation ``P -> B`` and raw entry radius ``||F_k.xy-B.xy||``
    must be defined.  The complete suffix is left-multiplied by one transform,
    so all internal relative poses are preserved exactly up to floating point.
    """

    previous = validate_pose_se2(previous_pose_world, name="previous_pose_world")
    boundary = validate_pose_se2(committed_pose_world, name="committed_pose_world")
    suffix = validate_se2_trajectory(fresh_suffix_world, name="fresh_suffix_world")
    threshold = float(minimum_translation_m)
    if not math.isfinite(threshold) or threshold <= 0.0:
        raise ValueError("minimum_translation_m must be finite and positive")

    incoming = boundary[:2] - previous[:2]
    incoming_norm = float(np.linalg.norm(incoming))
    if incoming_norm <= threshold:
        raise ValueError(
            "incoming translation is undefined at or below minimum_translation_m"
        )
    entry_radius = float(np.linalg.norm(suffix[0, :2] - boundary[:2]))
    if entry_radius <= threshold:
        raise ValueError(
            "raw boundary-to-entry translation is undefined at or below "
            "minimum_translation_m"
        )

    direction = incoming / incoming_norm
    incoming_yaw_increment = float(relative_pose(previous, boundary)[2])
    target = np.array(
        [
            boundary[0] + entry_radius * direction[0],
            boundary[1] + entry_radius * direction[1],
            wrap_angle(boundary[2] + incoming_yaw_increment),
        ],
        dtype=np.float64,
    )
    correction = compose_poses(target, inverse_pose(suffix[0]))
    trajectory = np.asarray(
        [compose_poses(correction, pose) for pose in suffix], dtype=np.float64
    )
    return RigidReconciliationResult(
        target_entry_world=target,
        correction_world=correction,
        trajectory_world=trajectory,
        raw_entry_radius_m=entry_radius,
        incoming_direction_world_rad=math.atan2(float(direction[1]), float(direction[0])),
    )

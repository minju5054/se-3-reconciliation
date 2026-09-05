"""Validated EXP-02A input conditioned on a spatial FRESH entry index.

Collaborator-level ``Z`` is represented by :class:`SpatialEntryContext`.  It is
deliberately not an SE(2) measurement and its evidence never enters graph cost.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Any, Mapping

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reconciliation.se2 import relative_pose
from reconciliation.trajectory import validate_pose_se2, validate_se2_trajectory


FloatArray = NDArray[np.float64]
JsonValue = None | bool | int | float | str | tuple["JsonValue", ...] | Mapping[str, "JsonValue"]


def _freeze_json(value: Any, *, path: str) -> JsonValue:
    if value is None or isinstance(value, (bool, str)):
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"{path} contains a non-finite numeric value")
        return value
    if isinstance(value, Mapping):
        frozen: dict[str, JsonValue] = {}
        for key, child in value.items():
            if not isinstance(key, str):
                raise ValueError(f"{path} keys must be strings")
            frozen[key] = _freeze_json(child, path=f"{path}.{key}")
        return MappingProxyType(frozen)
    if isinstance(value, (list, tuple)):
        return tuple(_freeze_json(child, path=f"{path}[{index}]") for index, child in enumerate(value))
    raise ValueError(f"{path} must contain only JSON-compatible values")


def _thaw_json(value: JsonValue) -> Any:
    if isinstance(value, Mapping):
        return {key: _thaw_json(child) for key, child in value.items()}
    if isinstance(value, tuple):
        return [_thaw_json(child) for child in value]
    return value


def _readonly(array: FloatArray) -> FloatArray:
    result = array.copy()
    result.setflags(write=False)
    return result


@dataclass(frozen=True, slots=True)
class SpatialEntryContext:
    """Pre-treatment selector context; never an SE(2) graph measurement."""

    entry_index: int
    evidence: Mapping[str, JsonValue] | None = None
    evidence_status: Mapping[str, JsonValue] | None = None
    source: str = ""
    metadata: Mapping[str, JsonValue] | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.entry_index, int) or isinstance(self.entry_index, bool):
            raise ValueError("entry_index must be an integer")
        if self.entry_index < 0:
            raise ValueError("entry_index must be nonnegative")
        if not isinstance(self.source, str) or not self.source.strip():
            raise ValueError("source must be a non-empty string")
        evidence = {} if self.evidence is None else self.evidence
        status = {} if self.evidence_status is None else self.evidence_status
        metadata = {} if self.metadata is None else self.metadata
        if not isinstance(evidence, Mapping) or not isinstance(status, Mapping) or not isinstance(metadata, Mapping):
            raise ValueError("evidence, evidence_status, and metadata must be mappings")
        object.__setattr__(self, "evidence", _freeze_json(evidence, path="evidence"))
        object.__setattr__(self, "evidence_status", _freeze_json(status, path="evidence_status"))
        object.__setattr__(self, "metadata", _freeze_json(metadata, path="metadata"))
        object.__setattr__(self, "source", self.source.strip())

    def to_dict(self) -> dict[str, Any]:
        return {
            "entry_index": self.entry_index,
            "evidence": _thaw_json(self.evidence),
            "evidence_status": _thaw_json(self.evidence_status),
            "source": self.source,
            "metadata": _thaw_json(self.metadata),
        }

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "SpatialEntryContext":
        if not isinstance(value, Mapping):
            raise ValueError("spatial entry context must be a mapping")
        allowed = {"entry_index", "evidence", "evidence_status", "source", "metadata"}
        unknown = sorted(set(value).difference(allowed))
        if unknown:
            raise ValueError(f"unknown spatial entry context fields: {', '.join(unknown)}")
        if "entry_index" not in value or "source" not in value:
            raise ValueError("spatial entry context requires entry_index and source")
        return cls(
            entry_index=value["entry_index"],
            evidence=value.get("evidence"),
            evidence_status=value.get("evidence_status"),
            source=value["source"],
            metadata=value.get("metadata"),
        )


@dataclass(frozen=True, slots=True)
class TransitionReconciliationInput:
    """Fixed OLD/FRESH data and the selected editable suffix for EXP-02A."""

    old_poses_world: FloatArray
    fresh_poses_world: FloatArray
    committed_pose_world: FloatArray
    entry_context: SpatialEntryContext
    actual_pose_before_committed: FloatArray | None = None
    metadata: Mapping[str, JsonValue] | None = None

    def __post_init__(self) -> None:
        old = validate_se2_trajectory(self.old_poses_world, name="old_poses_world")
        fresh = validate_se2_trajectory(self.fresh_poses_world, name="fresh_poses_world")
        committed = validate_pose_se2(self.committed_pose_world, name="committed_pose_world")
        if not isinstance(self.entry_context, SpatialEntryContext):
            raise TypeError("entry_context must be a SpatialEntryContext")
        k = self.entry_context.entry_index
        if k >= fresh.shape[0]:
            raise ValueError("entry_index is out of FRESH bounds")
        if k == fresh.shape[0] - 1:
            raise ValueError(
                "entry_index at the final FRESH pose is unsupported in EXP-02A because "
                "transition-tangent and suffix-motion evaluation require one following pose"
            )
        previous = None
        if self.actual_pose_before_committed is not None:
            previous = validate_pose_se2(
                self.actual_pose_before_committed, name="actual_pose_before_committed"
            )
        metadata = {} if self.metadata is None else self.metadata
        if not isinstance(metadata, Mapping):
            raise ValueError("metadata must be a mapping")
        object.__setattr__(self, "old_poses_world", _readonly(old))
        object.__setattr__(self, "fresh_poses_world", _readonly(fresh))
        object.__setattr__(self, "committed_pose_world", _readonly(committed))
        object.__setattr__(self, "actual_pose_before_committed", None if previous is None else _readonly(previous))
        object.__setattr__(self, "metadata", _freeze_json(metadata, path="metadata"))

    @property
    def selected_suffix(self) -> FloatArray:
        """Return an independent exact copy of ``FRESH[k:]``."""

        return self.fresh_poses_world[self.entry_context.entry_index :].copy()

    @property
    def incoming_previous_pose(self) -> FloatArray:
        """Prefer measured execution immediately before B; otherwise use planned OLD tail."""

        source = self.actual_pose_before_committed
        return (self.old_poses_world[-1] if source is None else source).copy()

    @property
    def planned_old_to_boundary(self) -> FloatArray:
        """Always expose the planned OLD-tail relation ``O_(m-1)^-1 B``."""

        return relative_pose(self.old_poses_world[-1], self.committed_pose_world)

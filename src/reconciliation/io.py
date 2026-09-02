"""I/O helpers for recorded arrays and validated chunk JSON."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

import numpy as np
from numpy.typing import NDArray

from reconciliation.types import TrajectoryChunk


def load_json(path: str | Path) -> Any:
    with Path(path).open("r", encoding="utf-8") as stream:
        return json.load(stream)


def save_json(data: Any, path: str | Path) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8") as stream:
        json.dump(data, stream, indent=2, sort_keys=True)
        stream.write("\n")


def load_poses(path: str | Path) -> NDArray[np.float64]:
    """Load an ``N x 3`` pose array from raw ``.npy`` or JSON data."""

    source = Path(path)
    if source.suffix.lower() == ".npy":
        values = np.load(source, allow_pickle=False)
    elif source.suffix.lower() == ".json":
        decoded = load_json(source)
        values = decoded.get("poses_local") if isinstance(decoded, Mapping) else decoded
    else:
        raise ValueError("pose input must use .npy or .json")
    poses = np.asarray(values, dtype=np.float64)
    if poses.ndim != 2 or poses.shape[1:] != (3,) or poses.shape[0] < 1:
        raise ValueError("pose input must have shape (N, 3) with N >= 1")
    if not np.all(np.isfinite(poses)):
        raise ValueError("pose input must contain only finite values")
    return poses


def build_chunk(poses_path: str | Path, metadata: Mapping[str, Any]) -> TrajectoryChunk:
    """Combine an untouched raw pose file with explicit recorded metadata."""

    data = dict(metadata)
    data["poses_local"] = load_poses(poses_path)
    data.setdefault("schema_version", 1)
    return TrajectoryChunk.from_dict(data)


def load_chunk(path: str | Path) -> TrajectoryChunk:
    decoded = load_json(path)
    if not isinstance(decoded, Mapping):
        raise ValueError("trajectory chunk JSON must contain an object")
    return TrajectoryChunk.from_dict(decoded)


def save_chunk(chunk: TrajectoryChunk, path: str | Path) -> None:
    save_json(chunk.to_dict(), path)

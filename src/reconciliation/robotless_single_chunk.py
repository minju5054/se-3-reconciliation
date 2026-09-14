"""Pure contracts for a static, robotless Isaac/LightNav file handoff.

This module imports neither Isaac nor LightNav. Waypoint rows are spatial poses,
not timed commands. The only pose accepted by the transform is the logical
agent's pose at the observation event.
"""

from __future__ import annotations

from collections.abc import Mapping
from datetime import datetime, timezone
import hashlib
import io
import json
from pathlib import Path
import re
import time
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray
import yaml

from reconciliation.se2 import local_trajectory_to_world


FRAME_CONVENTIONS = {
    "source_frame": "logical_agent_at_observation",
    "target_frame": "isaac_world",
    "agent_axes": {"x": "forward", "y": "left", "z": "up"},
    "world_up_axis": "+z",
    "positive_yaw": "counterclockwise about +z",
    "waypoint_columns": ["forward_m", "lateral_left_m", "yaw_ccw_rad"],
    "world_waypoint_columns": ["x_m", "y_m", "yaw_ccw_rad"],
    "translation_unit": "meter",
    "angle_unit": "radian",
    "yaw_interval": "[-pi, pi)",
    "transform": "T_world_waypoint = T_world_agent(t_obs) @ T_agent_waypoint",
    "timing": "untimed spatial waypoints; no execution or motion",
    "camera_frame": "USD camera: +x right, +y up, -z optical forward",
}


def load_config(path: str | Path) -> dict[str, Any]:
    """Read a YAML mapping without mutating the versioned configuration."""

    with Path(path).open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("configuration must be a mapping")
    return config


def validate_scene_frame(scene: Mapping[str, Any]) -> None:
    """Require metre stage coordinates and +Z up for the declared SE(2) interface.

    Translations, clipping distances and visualization geometry are expressed
    directly in metres. Other stage units require an explicit conversion that
    this static interface does not implement.
    """

    if not isinstance(scene, Mapping):
        raise ValueError("scene must declare stage units and up axis")
    units = scene.get("stage_units_in_meters")
    if isinstance(units, bool) or not isinstance(units, (int, float)) or units != 1.0:
        raise ValueError("robotless scene requires stage_units_in_meters == 1.0")
    if scene.get("up_axis") != "Z":
        raise ValueError("robotless scene requires up_axis == Z")


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def save_json_exclusive(path: str | Path, data: Any) -> None:
    """Create a JSON artifact once, rejecting nonfinite numbers and overwrites.

    Serialization happens before creating the destination, so an invalid payload
    cannot leave an empty artifact that looks like a completed handoff.
    """

    encoded = json.dumps(data, indent=2, sort_keys=True, allow_nan=False) + "\n"
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("x", encoding="utf-8") as stream:
        stream.write(encoded)


def save_npy_exclusive(path: str | Path, array: ArrayLike) -> None:
    """Save a numeric array without pickle or replacement of an existing file.

    This deliberately preserves input values and dtype. Validation and derived
    float64 conversion are separate from preservation of a raw model response.
    """

    buffer = io.BytesIO()
    np.save(buffer, np.asarray(array), allow_pickle=False)
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("xb") as stream:
        stream.write(buffer.getvalue())


def validate_waypoints(raw: ArrayLike) -> NDArray[np.float64]:
    """Return an independent finite float64 ``(N, 3)`` copy for any positive N."""

    source = np.asarray(raw)
    if source.ndim != 2 or source.shape[1] != 3 or source.shape[0] < 1:
        raise ValueError("waypoints must have shape (N, 3) with N >= 1")
    if source.dtype.kind not in "iuf":
        raise ValueError("waypoints must contain real numeric values")
    trajectory = np.array(source, dtype=np.float64, copy=True)
    if not np.all(np.isfinite(trajectory)):
        raise ValueError("waypoints must contain only finite values")
    return trajectory


def _agent_pose(value: ArrayLike) -> NDArray[np.float64]:
    pose = np.asarray(value)
    if pose.shape != (3,) or pose.dtype.kind not in "iuf":
        raise ValueError("agent_pose_world must be a real numeric [x, y, yaw] pose")
    pose = np.array(pose, dtype=np.float64, copy=True)
    if not np.all(np.isfinite(pose)):
        raise ValueError("agent_pose_world must contain only finite values")
    return pose


def observation_to_world(
    agent_pose_world_at_observation: ArrayLike,
    raw: ArrayLike,
) -> NDArray[np.float64]:
    """Compose ``T^W_Fj = T^W_R(t_obs) T^R_Fj`` using existing SE(2) code.

    Local +x is forward, local +y is left, and positive yaw is CCW about
    +z. Translation is in meters and yaw in radians, wrapped to [-pi, pi).
    Neither the observation pose nor the raw trajectory is changed.
    """

    return local_trajectory_to_world(
        _agent_pose(agent_pose_world_at_observation), validate_waypoints(raw)
    )


def frame_sanity_fixtures() -> list[dict[str, Any]]:
    """Deterministic forward/left markers; tests only, never research evidence."""

    local = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
    cases = [
        ("yaw_0", [0.0, 0.0, 0.0], [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
        (
            "yaw_90",
            [0.0, 0.0, np.pi / 2.0],
            [[0.0, 1.0, np.pi / 2.0], [-1.0, 0.0, np.pi / 2.0]],
        ),
    ]
    fixtures = []
    for name, pose, expected in cases:
        actual = observation_to_world(pose, local)
        fixtures.append(
            {
                "name": name,
                "synthetic_only": True,
                "purpose": "coordinate-frame test, not experimental evidence",
                "agent_pose_world": pose,
                "local_waypoints": [row.copy() for row in local],
                "point_labels": ["forward", "left"],
                "expected_world_waypoints": expected,
                "actual_world_waypoints": actual.tolist(),
                "passed": bool(np.allclose(actual, expected, atol=1e-12, rtol=0.0)),
            }
        )
    return fixtures


def _nonnegative_finite(value: Any, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a nonnegative finite number")
    if not np.isfinite(value) or value < 0:
        raise ValueError(f"{name} must be a nonnegative finite number")


def _validate_host_time(value: Any, name: str) -> None:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must record UTC and monotonic clocks separately")
    utc = value.get("utc")
    if not isinstance(utc, str) or "T" not in utc:
        raise ValueError(f"{name}.utc must be an ISO-8601 UTC timestamp")
    try:
        parsed = datetime.fromisoformat(utc.replace("Z", "+00:00"))
    except ValueError as error:
        raise ValueError(f"{name}.utc must be an ISO-8601 UTC timestamp") from error
    if parsed.tzinfo is None or parsed.utcoffset().total_seconds() != 0:
        raise ValueError(f"{name}.utc must use the UTC timezone")
    monotonic = value.get("monotonic_ns")
    if isinstance(monotonic, bool) or not isinstance(monotonic, int) or monotonic < 0:
        raise ValueError(f"{name}.monotonic_ns must be a nonnegative integer")


def make_observation_time(simulation_time_s: float) -> dict[str, Any]:
    """Stamp one static capture with three distinct, explicitly named clocks.

    UTC is human-readable provenance. ``time.monotonic_ns()`` is this host's
    monotonic clock; its epoch is not UTC. Simulation seconds are supplied by
    Isaac. No difference between these clocks represents model latency, and no
    timing is assigned to individual waypoints.
    """

    _nonnegative_finite(simulation_time_s, "simulation_time_s")
    return {
        "utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "monotonic_ns": time.monotonic_ns(),
        "simulation_time_s": float(simulation_time_s),
    }


def validate_observation_time(value: Any) -> None:
    _validate_host_time(value, "observation_time")
    _nonnegative_finite(value.get("simulation_time_s"), "observation_time.simulation_time_s")


def validate_observation_metadata(metadata: Mapping[str, Any]) -> None:
    """Validate the portable capture contract before invoking LightNav.

    Camera implementation details remain explicit in its own nested mapping.
    Optional readiness is a host event only; execution is absent in this static
    interface and must be recorded as null if the key is supplied.
    """

    if not isinstance(metadata, Mapping):
        raise ValueError("observation metadata must be a mapping")
    validate_observation_time(metadata.get("observation_time"))
    _agent_pose(metadata.get("agent_pose_world"))
    for field in ("camera", "scene", "coordinate_conventions"):
        if not isinstance(metadata.get(field), Mapping) or not metadata[field]:
            raise ValueError(f"{field} must be an explicit nonempty mapping")
    for field in ("instruction", "model_checkpoint_identifier"):
        if not isinstance(metadata.get(field), str) or not metadata[field].strip():
            raise ValueError(f"{field} must be a nonempty string")
    for field in ("research_git_sha", "lightnav_git_sha"):
        if not isinstance(metadata.get(field), str) or not re.fullmatch(
            r"[0-9a-fA-F]{40}", metadata[field]
        ):
            raise ValueError(f"{field} must be a full Git SHA")
    rgb = metadata.get("rgb")
    if not isinstance(rgb, Mapping):
        raise ValueError("rgb must record path, SHA-256, and resolution")
    if not isinstance(rgb.get("path"), str) or not rgb["path"].strip():
        raise ValueError("rgb.path must be a nonempty string")
    if not isinstance(rgb.get("sha256"), str) or not re.fullmatch(
        r"[0-9a-fA-F]{64}", rgb["sha256"]
    ):
        raise ValueError("rgb.sha256 must be a SHA-256 digest")
    resolution = rgb.get("resolution_width_height")
    if not isinstance(resolution, (list, tuple)) or len(resolution) != 2 or any(
        isinstance(n, bool) or not isinstance(n, int) or n <= 0 for n in resolution
    ):
        raise ValueError("rgb.resolution_width_height must contain two positive integers")
    if "inference_ready_time" in metadata:
        _validate_host_time(metadata["inference_ready_time"], "inference_ready_time")
    if metadata.get("execution_time") is not None:
        raise ValueError("execution_time must be null: this static task has no execution")

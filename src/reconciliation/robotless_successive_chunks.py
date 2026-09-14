"""Pure contracts for two robotless observations and their successive chunks.

The scripted displacement is an SE(2) pose, in metres and radians, expressed
in R0's local frame. Model rows are cumulative local poses, never increments.
No timestamp here schedules or measures trajectory execution.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reconciliation.robotless_single_chunk import (
    FRAME_CONVENTIONS, _agent_pose, _validate_host_time, observation_to_world,
    validate_observation_metadata,
)
from reconciliation.se2 import compose_poses


SUCCESSIVE_FRAME_CONVENTIONS = {
    **FRAME_CONVENTIONS,
    "timing": "untimed spatial waypoints; scripted pose assignment between observations only; no execution",
}


def successive_observation_poses(
    R0: ArrayLike, configured_local_displacement: ArrayLike,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return independent R0 and R1 = R0 * Delta; Delta is local, not world."""

    initial = _agent_pose(R0)
    delta = _agent_pose(configured_local_displacement)
    return initial, compose_poses(initial, delta)


def successive_to_world(
    R0: ArrayLike, R1: ArrayLike, old_raw: ArrayLike, fresh_raw: ArrayLike,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Use each chunk's observation pose, preserving raw arrays and any N > 0."""

    return observation_to_world(R0, old_raw), observation_to_world(R1, fresh_raw)


def round_trip_latency_ms(send_time: Mapping, receive_time: Mapping) -> float:
    """Convert this host's nondecreasing monotonic event difference ns -> ms.

    UTC is validated as provenance but never subtracted. This wall duration
    includes transport and inference; it is neither execution nor switch latency.
    Both events must be sampled by the same client on the same host.
    """

    _validate_host_time(send_time, "request_send_time")
    _validate_host_time(receive_time, "response_receive_time")
    elapsed = receive_time["monotonic_ns"] - send_time["monotonic_ns"]
    if elapsed < 0:
        raise ValueError("response_receive_time precedes request_send_time on host monotonic clock")
    return elapsed / 1_000_000.0


def validate_successive_observation_metadata(metadata: Mapping[str, Any]) -> None:
    """Validate two static captures separated by a configured local pose change."""

    if not isinstance(metadata, Mapping):
        raise ValueError("successive observation metadata must be a mapping")
    if metadata.get("coordinate_conventions") != SUCCESSIVE_FRAME_CONVENTIONS:
        raise ValueError("coordinate conventions must declare successive observation semantics")
    if "execution_time" not in metadata or metadata["execution_time"] is not None:
        raise ValueError("execution_time must be explicitly null: no trajectory is executed")
    initial, expected = successive_observation_poses(
        metadata.get("R0"), metadata.get("configured_local_displacement"),
    )
    fresh = _agent_pose(metadata.get("R1"))
    if not np.allclose(fresh, expected, rtol=0.0, atol=1e-7):
        raise ValueError("R1 must be R0 composed with configured local displacement")
    if np.allclose(initial, fresh, rtol=0.0, atol=1e-12):
        raise ValueError("successive observations require distinct R0 and R1")
    observations = metadata.get("observations")
    if not isinstance(observations, list) or len(observations) != 2:
        raise ValueError("exactly two observations are required")
    ids = []
    cameras = []
    previous_time = -1
    for seq, (label, pose, observation) in enumerate(zip(
        ("OLD", "FRESH"), (initial, fresh), observations, strict=True,
    )):
        if not isinstance(observation, Mapping):
            raise ValueError("observation must be a mapping")
        if type(observation.get("seq")) is not int or observation["seq"] != seq:
            raise ValueError("observation seq must be exactly [0, 1]")
        if observation.get("label") != label:
            raise ValueError("observation labels must be OLD then FRESH")
        identifier = observation.get("observation_id")
        if not ((type(identifier) is int and identifier >= 0)
                or (isinstance(identifier, str) and identifier.strip())):
            raise ValueError("observation_id must be a nonempty string or nonnegative integer")
        ids.append(identifier)
        # Reuse the existing capture schema for frames, RGB, provenance and clocks.
        complete = {**metadata, **observation}
        validate_observation_metadata(complete)
        if observation.get("instruction") != metadata.get("instruction"):
            raise ValueError("instruction must remain unchanged across observations")
        if not np.allclose(_agent_pose(observation["agent_pose_world"]), pose, rtol=0.0, atol=1e-7):
            raise ValueError(f"{label} observation pose differs from R{seq}")
        stamp = observation["observation_time"]["monotonic_ns"]
        if stamp <= previous_time:
            raise ValueError("observation host monotonic times must increase")
        previous_time = stamp
        camera = observation.get("camera")
        if not isinstance(camera, Mapping):
            raise ValueError("each observation must record its camera")
        cameras.append(camera)
    if ids[0] == ids[1]:
        raise ValueError("observation IDs must be distinct")
    # World extrinsics necessarily change with R0 -> R1; intrinsic and agent
    # extrinsic definitions must remain identical.
    for field in ("configured", "actual_intrinsics"):
        if field not in cameras[0] or cameras[0][field] != cameras[1].get(field):
            raise ValueError(f"camera {field} must remain unchanged")
    # Measured world-to-agent compositions incur roundoff when translating R1.
    extrinsics = [np.asarray(camera.get("T_agent_camera"), dtype=float) for camera in cameras]
    if any(value.shape != (4, 4) or not np.all(np.isfinite(value)) for value in extrinsics) or not np.allclose(
        extrinsics[0], extrinsics[1], rtol=0.0, atol=1e-10,
    ):
        raise ValueError("camera T_agent_camera must remain unchanged within 1e-10 numerical tolerance")

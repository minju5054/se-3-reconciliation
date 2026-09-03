"""Semantics-preserving adapter for one decoded LightNav action chunk."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reconciliation.se2 import compose_poses, local_trajectory_to_world, wrap_angle
from reconciliation.trajectory import validate_pose_se2, validate_se2_trajectory


FloatArray = NDArray[np.float64]
LIGHTNAV_COLUMNS = ("forward_m", "lateral_m_left_positive", "yaw_rad_ccw_positive")
DECODED_OUTPUT_SEMANTICS = "absolute_poses_in_observation_robot_frame"
INTRINSIC_WAYPOINT_TIME_BASE = False


def validate_raw_lightnav_actions(
    actions: ArrayLike,
    *,
    expected_horizon: int | None = None,
) -> np.ndarray:
    """Validate decoded LightNav API output without changing its dtype or values."""

    result = np.asarray(actions)
    if result.ndim != 2 or result.shape[0] < 1 or result.shape[1] != 3:
        raise ValueError("raw LightNav actions must have shape (N, 3) with N >= 1")
    if not np.issubdtype(result.dtype, np.floating):
        raise ValueError("raw LightNav actions must use a floating-point dtype")
    if not np.all(np.isfinite(result)):
        raise ValueError("raw LightNav actions must contain only finite values")
    if expected_horizon is not None:
        if not isinstance(expected_horizon, int) or expected_horizon < 1:
            raise ValueError("expected_horizon must be a positive integer")
        if result.shape[0] != expected_horizon:
            raise ValueError(
                f"raw LightNav horizon {result.shape[0]} != expected {expected_horizon}"
            )
    return result.copy()


def incremental_se2_to_absolute_local(deltas: ArrayLike) -> FloatArray:
    """Compose previous-step-frame SE(2) deltas; never element-wise cumsum."""

    increments = validate_se2_trajectory(deltas, name="incremental LightNav actions")
    output = np.empty_like(increments)
    pose = np.zeros(3, dtype=np.float64)
    for index, delta in enumerate(increments):
        pose = compose_poses(pose, delta)
        output[index] = pose
    return output


def raw_actions_to_local_path(
    actions: ArrayLike,
    *,
    decoded_output_semantics: str = DECODED_OUTPUT_SEMANTICS,
    expected_horizon: int | None = None,
) -> FloatArray:
    """Convert source-confirmed decoder output to canonical local `[x, y, yaw]`.

    LightNav's public API already returns absolute future poses in the observation robot
    frame, even when the checkpoint RVQ manifest stores ``se2_diff`` features internally:
    the official decoder performs that SE(2) composition before returning the array.
    Therefore the released-checkpoint conversion is an axis-preserving numeric copy.
    """

    raw = validate_raw_lightnav_actions(actions, expected_horizon=expected_horizon)
    if decoded_output_semantics == DECODED_OUTPUT_SEMANTICS:
        return raw.astype(np.float64, copy=True)
    if decoded_output_semantics == "incremental_previous_pose_frame_se2":
        return incremental_se2_to_absolute_local(raw)
    raise ValueError(f"unsupported decoded LightNav semantics: {decoded_output_semantics!r}")


def lightnav_local_to_world(actions_local: ArrayLike, observation_pose: ArrayLike) -> FloatArray:
    """Anchor a decoded chunk at the last inference-input frame's robot pose."""

    local = validate_se2_trajectory(actions_local, name="LightNav local path")
    anchor = validate_pose_se2(observation_pose, name="robot_pose_at_observation")
    return local_trajectory_to_world(anchor, local)


def save_npy_exclusive(path: str | Path, array: ArrayLike) -> Path:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("xb") as stream:
        np.save(stream, np.asarray(array), allow_pickle=False)
    return destination


def save_json_exclusive(path: str | Path, value: Mapping[str, Any]) -> Path:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("x", encoding="utf-8") as stream:
        json.dump(dict(value), stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    return destination


def trajectory_sanity(path: ArrayLike) -> dict[str, Any]:
    poses = validate_se2_trajectory(path)
    spacing = np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1)
    return {
        "shape": list(poses.shape),
        "start": poses[0].tolist(),
        "end": poses[-1].tolist(),
        "translation_magnitude_max_m": float(np.max(np.linalg.norm(poses[:, :2], axis=1))),
        "absolute_yaw_max_rad": float(np.max(np.abs(poses[:, 2]))),
        "consecutive_spacing_min_m": float(np.min(spacing)) if spacing.size else 0.0,
        "consecutive_spacing_mean_m": float(np.mean(spacing)) if spacing.size else 0.0,
        "consecutive_spacing_max_m": float(np.max(spacing)) if spacing.size else 0.0,
    }


def _load_json_object(path: Path, name: str) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(
            stream,
            parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)),
        )
    if not isinstance(value, dict):
        raise ValueError(f"{name} must contain a JSON object")
    return value


def derive_single_chunk(run_directory: str | Path) -> dict[str, Any]:
    """Create immutable local/world derived arrays from one real raw inference."""

    root = Path(run_directory)
    raw_dir = root / "raw"
    inference = _load_json_object(raw_dir / "lightnav_inference.json", "inference metadata")
    observation = _load_json_object(raw_dir / "observation_metadata.json", "observation metadata")
    forbidden_time_fields = {"waypoint_dt", "waypoint_dt_s", "ready_time", "ready_time_s"}
    present = sorted(forbidden_time_fields.intersection(inference))
    if present:
        raise ValueError(f"raw inference metadata fabricates forbidden timing fields: {present}")
    if inference.get("intrinsic_waypoint_time_base") is not False:
        raise ValueError("inference metadata must state that no intrinsic waypoint time base exists")
    semantics = str(inference.get("decoded_action_semantics", ""))
    horizon = int(inference["horizon"])
    raw = np.load(raw_dir / "lightnav_actions.npy", allow_pickle=False)
    raw_validated = validate_raw_lightnav_actions(raw, expected_horizon=horizon)
    local = raw_actions_to_local_path(
        raw_validated,
        decoded_output_semantics=semantics,
        expected_horizon=horizon,
    )
    anchor = validate_pose_se2(observation["robot_pose_at_observation"])
    world = lightnav_local_to_world(local, anchor)
    derived_dir = root / "derived"
    save_npy_exclusive(derived_dir / "lightnav_local_path.npy", local)
    save_npy_exclusive(derived_dir / "lightnav_world_path.npy", world)
    metadata = {
        "source_raw_actions": "raw/lightnav_actions.npy",
        "raw_dtype_preserved_in_raw_artifact": str(raw_validated.dtype),
        "local_path_dtype": str(local.dtype),
        "conversion": "identity axes/values; float64 derived copy of decoded absolute ego poses",
        "columns": list(LIGHTNAV_COLUMNS),
        "robot_pose_at_observation": anchor.tolist(),
        "world_transform": "T_world_waypoint = T_world_robot_at_observation * T_robot_waypoint",
        "intrinsic_waypoint_time_base": False,
        "local_sanity": trajectory_sanity(local),
        "world_sanity": trajectory_sanity(world),
    }
    save_json_exclusive(derived_dir / "derivation.json", metadata)
    return metadata


def validate_single_chunk_run(
    run_directory: str | Path,
    *,
    require_execution: bool = False,
) -> dict[str, Any]:
    """Validate raw/derived separation and optional playback artifacts without Isaac/LightNav."""

    root = Path(run_directory)
    required = (
        "raw/frame_samples.csv",
        "raw/observation_metadata.json",
        "raw/lightnav_actions.npy",
        "raw/lightnav_raw_text.txt",
        "raw/lightnav_inference.json",
        "derived/lightnav_local_path.npy",
        "derived/lightnav_world_path.npy",
        "derived/derivation.json",
    )
    missing = [name for name in required if not (root / name).is_file()]
    if missing:
        raise ValueError(f"single-chunk run is missing files: {', '.join(missing)}")
    observation = _load_json_object(root / "raw/observation_metadata.json", "observation metadata")
    inference = _load_json_object(root / "raw/lightnav_inference.json", "inference metadata")
    derivation = _load_json_object(root / "derived/derivation.json", "derivation metadata")
    if observation.get("stage") != "stage0-c-lightnav-single-chunk":
        raise ValueError("observation metadata has an invalid stage")
    frame_count = int(observation["frame_count"])
    if frame_count < 1:
        raise ValueError("observation frame_count must be positive")
    capture_fps = float(observation["capture_fps"])
    observation_time = float(observation["observation_time_s"])
    if not math.isfinite(capture_fps) or capture_fps <= 0.0:
        raise ValueError("observation capture_fps must be finite and positive")
    if not math.isfinite(observation_time):
        raise ValueError("observation time must be finite")
    anchor = validate_pose_se2(observation["robot_pose_at_observation"])
    camera = observation.get("camera")
    if not isinstance(camera, Mapping):
        raise ValueError("observation camera metadata must contain an object")
    if camera.get("rgb_format") != "H x W x 3 uint8 RGB sliced from Isaac rgb annotator RGBA":
        raise ValueError("camera metadata does not declare the captured uint8 RGB contract")
    if int(camera.get("resolution_width", 0)) < 1 or int(camera.get("resolution_height", 0)) < 1:
        raise ValueError("camera resolution must be positive")
    if inference.get("intrinsic_waypoint_time_base") is not False:
        raise ValueError("intrinsic waypoint time-base statement is missing or invalid")
    if any(key in inference for key in ("waypoint_dt", "waypoint_dt_s", "ready_time", "ready_time_s")):
        raise ValueError("inference metadata contains a fabricated waypoint/ready time")
    raw = validate_raw_lightnav_actions(
        np.load(root / "raw/lightnav_actions.npy", allow_pickle=False),
        expected_horizon=int(inference["horizon"]),
    )
    if inference.get("raw_shape") != list(raw.shape) or inference.get("raw_dtype") != str(raw.dtype):
        raise ValueError("raw action metadata differs from the immutable NPY artifact")
    if inference.get("decoded_action_semantics") != DECODED_OUTPUT_SEMANTICS:
        raise ValueError("decoded action semantics are unsupported or missing")
    axes = inference.get("axis_convention")
    if not isinstance(axes, Mapping) or axes.get("columns") != [
        "forward_m", "lateral_m", "yaw_rad"
    ]:
        raise ValueError("LightNav axis convention metadata is invalid")
    if float(inference["observation_time_s"]) != observation_time:
        raise ValueError("inference and observation simulation times differ")
    inference_anchor = validate_pose_se2(inference["robot_pose_at_observation"])
    if not np.array_equal(inference_anchor, anchor):
        raise ValueError("inference metadata uses a different observation anchor")
    local = validate_se2_trajectory(
        np.load(root / "derived/lightnav_local_path.npy", allow_pickle=False),
        name="derived local path",
    )
    world = validate_se2_trajectory(
        np.load(root / "derived/lightnav_world_path.npy", allow_pickle=False),
        name="derived world path",
    )
    expected_local = raw_actions_to_local_path(
        raw,
        decoded_output_semantics=str(inference["decoded_action_semantics"]),
        expected_horizon=int(inference["horizon"]),
    )
    expected_world = lightnav_local_to_world(
        expected_local, observation["robot_pose_at_observation"]
    )
    if not np.array_equal(local, expected_local):
        raise ValueError("derived local path differs from the declared raw conversion")
    if not np.array_equal(world, expected_world):
        raise ValueError("derived world path differs from the observation-pose transform")
    if derivation.get("intrinsic_waypoint_time_base") is not False:
        raise ValueError("derivation fabricated a waypoint time base")
    with (root / "raw/frame_samples.csv").open("r", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != frame_count:
        raise ValueError("frame sample count differs from observation metadata")
    try:
        frame_indices = np.asarray([int(row["frame_index"]) for row in rows])
        frame_times = np.asarray([float(row["sim_time_s"]) for row in rows])
        frame_poses = np.asarray(
            [
                [float(row["robot_x"]), float(row["robot_y"]), float(row["robot_yaw"])]
                for row in rows
            ]
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("frame_samples.csv has invalid columns or numeric values") from error
    if not np.array_equal(frame_indices, np.arange(frame_count)):
        raise ValueError("RGB frame indices are not contiguous from zero")
    if not np.all(np.isfinite(frame_times)) or np.any(np.diff(frame_times) <= 0.0):
        raise ValueError("RGB simulation timestamps must be finite and strictly increasing")
    validate_se2_trajectory(frame_poses, name="RGB frame robot poses")
    if frame_times[-1] != observation_time or not np.array_equal(frame_poses[-1], anchor):
        raise ValueError("observation anchor must match the final inference input frame")
    frame_paths = sorted((root / "raw/rgb").glob("frame_*.png"))
    if len(frame_paths) != len(rows):
        raise ValueError("RGB frame count differs from frame_samples.csv")

    execution = None
    actual_path = root / "derived/jackal_actual_trajectory.npy"
    metrics_path = root / "results/execution_metrics.json"
    if actual_path.exists() or metrics_path.exists() or require_execution:
        execution_required = (
            root / "derived/execution_reference.npy",
            root / "derived/execution_samples.csv",
            root / "results/execution_metrics.json",
            root / "results/execution_metadata.json",
        )
        if not actual_path.is_file() or any(not path.is_file() for path in execution_required):
            raise ValueError("execution artifacts are incomplete")
        actual = validate_se2_trajectory(np.load(actual_path, allow_pickle=False), name="actual")
        metrics = _load_json_object(metrics_path, "execution metrics")
        numeric = [value for value in metrics.values() if isinstance(value, (int, float))]
        if not numeric or not all(math.isfinite(float(value)) for value in numeric):
            raise ValueError("execution metrics must contain finite numeric values")
        reference = validate_se2_trajectory(
            np.load(root / "derived/execution_reference.npy", allow_pickle=False),
            name="execution reference",
        )
        expected_reference = np.vstack((anchor, world))
        if not np.array_equal(reference, expected_reference):
            raise ValueError("execution reference must be anchor + derived world future poses")
        with (root / "derived/execution_samples.csv").open(
            "r", newline="", encoding="utf-8"
        ) as stream:
            execution_rows = list(csv.DictReader(stream))
        if len(execution_rows) != actual.shape[0]:
            raise ValueError("execution sample count differs from actual trajectory")
        if int(metrics.get("sample_count", -1)) != actual.shape[0]:
            raise ValueError("execution metric sample_count differs from actual trajectory")
        execution = {"actual_shape": list(actual.shape), "metrics": metrics}
    return {
        "valid": True,
        "raw_shape": list(raw.shape),
        "raw_dtype": str(raw.dtype),
        "local_shape": list(local.shape),
        "world_shape": list(world.shape),
        "frame_count": len(rows),
        "execution": execution,
    }

import csv
import json
from pathlib import Path

import numpy as np
import pytest

from reconciliation.lightnav_adapter import (
    DECODED_OUTPUT_SEMANTICS,
    derive_single_chunk,
    incremental_se2_to_absolute_local,
    lightnav_local_to_world,
    raw_actions_to_local_path,
    save_json_exclusive,
    save_npy_exclusive,
    validate_raw_lightnav_actions,
    validate_single_chunk_run,
)


def raw_actions(count: int = 7) -> np.ndarray:
    return np.column_stack(
        (
            np.linspace(0.1, 0.7, count),
            np.linspace(0.0, 0.2, count),
            np.linspace(0.0, 0.3, count),
        )
    ).astype(np.float32)


def build_raw_run(root: Path, count: int = 7) -> np.ndarray:
    raw_dir = root / "raw"
    (raw_dir / "rgb").mkdir(parents=True)
    for index in range(2):
        (raw_dir / "rgb" / f"frame_{index:06d}.png").write_bytes(b"fixture")
    with (raw_dir / "frame_samples.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("frame_index", "sim_time_s", "robot_x", "robot_y", "robot_yaw"))
        writer.writerow((0, 0.0, 2.0, -1.0, 0.5))
        writer.writerow((1, 0.25, 2.0, -1.0, 0.5))
    save_json_exclusive(
        raw_dir / "observation_metadata.json",
        {
            "stage": "stage0-c-lightnav-single-chunk",
            "frame_count": 2,
            "capture_fps": 4.0,
            "observation_time_s": 0.25,
            "robot_pose_at_observation": [2.0, -1.0, 0.5],
            "camera": {
                "resolution_width": 448,
                "resolution_height": 256,
                "rgb_format": "H x W x 3 uint8 RGB sliced from Isaac rgb annotator RGBA",
            },
        },
    )
    actions = raw_actions(count)
    save_npy_exclusive(raw_dir / "lightnav_actions.npy", actions)
    (raw_dir / "lightnav_raw_text.txt").write_text("<act_l0_1>", encoding="utf-8")
    save_json_exclusive(
        raw_dir / "lightnav_inference.json",
        {
            "horizon": count,
            "raw_shape": [count, 3],
            "raw_dtype": "float32",
            "decoded_action_semantics": DECODED_OUTPUT_SEMANTICS,
            "intrinsic_waypoint_time_base": False,
            "axis_convention": {
                "columns": ["forward_m", "lateral_m", "yaw_rad"],
            },
            "observation_time_s": 0.25,
            "robot_pose_at_observation": [2.0, -1.0, 0.5],
        },
    )
    return actions


def test_raw_validation_preserves_dtype_values_and_arbitrary_horizon() -> None:
    raw = raw_actions(37)
    validated = validate_raw_lightnav_actions(raw, expected_horizon=37)
    assert validated.dtype == np.float32
    assert np.array_equal(validated, raw)
    assert validated is not raw


@pytest.mark.parametrize(
    "bad",
    (
        np.zeros((3, 2), dtype=np.float32),
        np.array([[0.0, np.nan, 0.0]], dtype=np.float32),
        np.array([[0.0, 0.0, np.inf]], dtype=np.float32),
    ),
)
def test_raw_validation_rejects_invalid_shape_nan_and_inf(bad: np.ndarray) -> None:
    with pytest.raises(ValueError):
        validate_raw_lightnav_actions(bad)


def test_released_decoder_output_is_axis_preserving_absolute_local_path() -> None:
    raw = raw_actions()
    local = raw_actions_to_local_path(raw)
    np.testing.assert_array_equal(local, raw.astype(np.float64))
    assert local.shape == (7, 3)


def test_incremental_conversion_uses_se2_composition_not_elementwise_sum() -> None:
    deltas = np.array([[1.0, 0.0, np.pi / 2], [1.0, 0.0, np.pi / 2]])
    absolute = incremental_se2_to_absolute_local(deltas)
    np.testing.assert_allclose(absolute[:, :2], [[1.0, 0.0], [1.0, 1.0]], atol=1e-12)
    np.testing.assert_allclose(absolute[:, 2], [np.pi / 2, -np.pi], atol=1e-12)


def test_observation_pose_world_transform_and_yaw_wrap() -> None:
    local = np.array([[1.0, 0.0, 0.2], [1.0, 1.0, 0.5]])
    world = lightnav_local_to_world(local, [2.0, 3.0, np.pi / 2])
    np.testing.assert_allclose(world[:, :2], [[2.0, 4.0], [1.0, 4.0]], atol=1e-12)
    np.testing.assert_allclose(world[:, 2], [np.pi / 2 + 0.2, np.pi / 2 + 0.5])
    wrapped = lightnav_local_to_world([[0.0, 0.0, 0.3]], [0.0, 0.0, np.pi - 0.1])
    assert wrapped[0, 2] == pytest.approx(-np.pi + 0.2)


def test_raw_derived_separation_validation_and_overwrite_protection(tmp_path: Path) -> None:
    raw = build_raw_run(tmp_path)
    derive_single_chunk(tmp_path)
    validation = validate_single_chunk_run(tmp_path)
    assert validation["valid"] is True
    assert validation["raw_dtype"] == "float32"
    assert np.array_equal(np.load(tmp_path / "raw/lightnav_actions.npy"), raw)
    assert np.load(tmp_path / "derived/lightnav_local_path.npy").dtype == np.float64
    with pytest.raises(FileExistsError):
        derive_single_chunk(tmp_path)
    with pytest.raises(FileExistsError):
        save_npy_exclusive(tmp_path / "raw/lightnav_actions.npy", raw)


def test_derive_rejects_fabricated_waypoint_time_base(tmp_path: Path) -> None:
    build_raw_run(tmp_path)
    metadata_path = tmp_path / "raw/lightnav_inference.json"
    metadata = json.loads(metadata_path.read_text())
    metadata["waypoint_dt_s"] = 0.1
    metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
    with pytest.raises(ValueError, match="forbidden timing"):
        derive_single_chunk(tmp_path)


def test_validator_rejects_inference_metadata_with_wrong_axes(tmp_path: Path) -> None:
    build_raw_run(tmp_path)
    derive_single_chunk(tmp_path)
    metadata_path = tmp_path / "raw/lightnav_inference.json"
    metadata = json.loads(metadata_path.read_text())
    metadata["axis_convention"]["columns"] = ["lateral_m", "forward_m", "yaw_rad"]
    metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
    with pytest.raises(ValueError, match="axis convention"):
        validate_single_chunk_run(tmp_path)


def test_validator_requires_final_frame_as_observation_anchor(tmp_path: Path) -> None:
    build_raw_run(tmp_path)
    derive_single_chunk(tmp_path)
    samples = tmp_path / "raw/frame_samples.csv"
    content = samples.read_text(encoding="utf-8").replace(
        "1,0.25,2.0,-1.0,0.5", "1,0.25,2.1,-1.0,0.5"
    )
    samples.write_text(content, encoding="utf-8")
    with pytest.raises(ValueError, match="final inference input frame"):
        validate_single_chunk_run(tmp_path)

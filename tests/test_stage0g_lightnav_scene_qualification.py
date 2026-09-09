import csv
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from reconciliation.lightnav_adapter import lightnav_local_to_world, save_json_exclusive, save_npy_exclusive
from reconciliation.stage0g_lightnav_scene_qualification import (
    SCENARIO_IDS,
    VARIANT_IDS,
    classify_descriptors,
    create_run_directory_exclusive,
    duplicate_groups,
    geometry_descriptors,
    geometry_signature,
    pairwise_trajectory_distance,
    sha256_file,
    validate_case,
    validate_config,
    variant_pose,
)


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture()
def config() -> dict:
    return yaml.safe_load((ROOT / "configs/stage0g_lightnav_scene_qualification.yaml").read_text())


def path(endpoint_y: float = 0.0, endpoint_yaw: float = 0.0) -> np.ndarray:
    t = np.linspace(0.1, 1.0, 10)
    return np.column_stack((t, endpoint_y * t, endpoint_yaw * t)).astype(np.float32)


def test_input_contract_config_and_checkpoint_provenance_are_explicit(config: dict) -> None:
    validate_config(config)
    assert config["baseline"]["expected_lightnav_git_sha"] == "a645828d81a8439651172197ca80a75dc1377977"
    assert len(config["baseline"]["expected_checkpoint_revision"]) == 40
    assert config["camera"]["rgb_format"] == "HWC_uint8_RGB"
    assert config["camera"]["model_preprocess_size_hw"] == [256, 448]
    assert config["lightnav"]["aspect_mode"] == "stretch"
    assert config["lightnav"]["intrinsic_waypoint_time_base"] is False


def test_six_scenarios_five_mirrored_nearby_variants_and_safe_poses(config: dict) -> None:
    assert tuple(item["id"] for item in config["scenarios"]) == SCENARIO_IDS
    assert tuple(item["id"] for item in config["variants"]) == VARIANT_IDS
    for scenario in SCENARIO_IDS:
        poses = [variant_pose(config, scenario, variant) for variant in VARIANT_IDS]
        assert all(np.all(np.isfinite(pose)) for pose in poses)
        assert poses[1][2] == pytest.approx(-poses[2][2])
        assert poses[3][1] - poses[0][1] == pytest.approx(-(poses[4][1] - poses[0][1]))


def test_straight_left_right_descriptors_and_classification(config: dict) -> None:
    criteria = config["qualification"]
    straight = geometry_descriptors(path())
    left = geometry_descriptors(path(0.35, 0.40))
    right = geometry_descriptors(path(-0.35, -0.40))
    assert classify_descriptors(straight, "straight", criteria)["matches_intended_geometry"]
    assert classify_descriptors(left, "left", criteria)["matches_intended_geometry"]
    assert classify_descriptors(right, "right", criteria)["matches_intended_geometry"]
    assert not classify_descriptors(right, "left", criteria)["matches_intended_geometry"]
    assert not classify_descriptors(left, "right", criteria)["matches_intended_geometry"]


def test_detour_excursion_and_contradictory_sign_rejection(config: dict) -> None:
    criteria = config["qualification"]
    left_detour = path()
    left_detour[:, 1] = np.sin(np.linspace(0.0, np.pi, 10)) * 0.35
    right_detour = left_detour.copy(); right_detour[:, 1] *= -1.0
    assert classify_descriptors(geometry_descriptors(left_detour), "detour_left", criteria)["matches_intended_geometry"]
    assert classify_descriptors(geometry_descriptors(right_detour), "detour_right", criteria)["matches_intended_geometry"]
    contradictory = path(0.25, -0.3)
    assert not classify_descriptors(geometry_descriptors(contradictory), "left", criteria)["matches_intended_geometry"]


def test_yaw_wrapping_and_zero_motion_stop_is_retained(config: dict) -> None:
    values = path(0.0, 2.0 * np.pi + 0.1)
    descriptor = geometry_descriptors(values)
    assert descriptor["signed_net_yaw_rad"] == pytest.approx(0.1, abs=1e-6)
    zero = np.zeros((10, 3), dtype=np.float32)
    stopped = geometry_descriptors(zero)
    assert stopped["exact_stop_output"] is True
    assert stopped["nonmoving"] is True
    assert geometry_signature(stopped).startswith("STOP")
    assert not classify_descriptors(stopped, "straight", config["qualification"])["matches_intended_geometry"]


@pytest.mark.parametrize(
    "bad",
    (np.full((10, 3), np.nan, dtype=np.float32), np.full((10, 3), np.inf, dtype=np.float32)),
)
def test_descriptor_rejects_nan_and_inf(bad: np.ndarray) -> None:
    with pytest.raises(ValueError):
        geometry_descriptors(bad)


def test_descriptors_do_not_mutate_source() -> None:
    source = path(0.3, 0.2)
    before = source.copy()
    geometry_descriptors(source)
    np.testing.assert_array_equal(source, before)


def test_observation_pose_is_the_only_world_anchor() -> None:
    local = path(0.2, 0.1)
    observation = np.array([2.0, -1.0, np.pi / 2])
    later = np.array([9.0, 9.0, 0.0])
    world = lightnav_local_to_world(local, observation)
    expected = lightnav_local_to_world(local, observation)
    wrong = lightnav_local_to_world(local, later)
    np.testing.assert_array_equal(world, expected)
    assert not np.array_equal(world, wrong)


def test_diversity_duplicate_groups_unique_count_and_pairwise_distance() -> None:
    arrays = {"a": path(), "b": path().copy(), "c": path(0.4, 0.3)}
    groups = duplicate_groups(arrays)
    assert len(groups) == 1
    assert next(iter(groups.values())) == ["a", "b"]
    comparison = pairwise_trajectory_distance(arrays)
    matrix = np.asarray(comparison["matrix"])
    assert matrix.shape == (3, 3)
    assert matrix[0, 1] == pytest.approx(0.0)
    assert matrix[0, 2] > 0.0


def build_case(root: Path) -> None:
    raw = path(0.2, 0.1)
    observation = [1.0, 2.0, 0.3]
    (root / "raw").mkdir(parents=True)
    (root / "derived").mkdir()
    (root / "input").mkdir()
    save_npy_exclusive(root / "raw/lightnav.npy", raw)
    (root / "raw/lightnav_raw_text.txt").write_text("tokens")
    save_npy_exclusive(root / "derived/trajectory_world.npy", lightnav_local_to_world(raw, observation))
    (root / "input/latest_rgb.png").write_bytes(b"fixture")
    with (root / "input/frame_samples.csv").open("x", newline="") as stream:
        csv.writer(stream).writerow(("frame_index", "sim_time_s", "robot_x", "robot_y", "robot_yaw"))
    save_json_exclusive(root / "input/capture_metadata.json", {"robot_pose_at_observation": observation})
    save_json_exclusive(root / "metadata.json", {"scenario_id": "Q0_STRAIGHT", "variant_id": "V0"})
    descriptor = geometry_descriptors(raw); descriptor["intrinsic_waypoint_time_base"] = False
    save_json_exclusive(root / "descriptors.json", descriptor)
    save_json_exclusive(root / "provenance.json", {"raw_lightnav_sha256": sha256_file(root / "raw/lightnav.npy")})


def test_case_validation_missing_artifact_raw_hash_and_overwrite_protection(tmp_path: Path) -> None:
    case = tmp_path / "case"
    build_case(case)
    assert validate_case(case)["valid"] is True
    with pytest.raises(FileExistsError):
        save_npy_exclusive(case / "raw/lightnav.npy", path())
    (case / "metadata.json").unlink()
    with pytest.raises(ValueError, match="missing"):
        validate_case(case)


def test_existing_run_directory_is_rejected_even_when_empty(tmp_path: Path) -> None:
    run = tmp_path / "existing"
    run.mkdir()
    with pytest.raises(FileExistsError):
        create_run_directory_exclusive(run)

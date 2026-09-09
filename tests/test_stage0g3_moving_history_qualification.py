import copy
from pathlib import Path

import numpy as np
import pytest
import yaml

from reconciliation.se2 import wrap_angle
from reconciliation.stage0g3_moving_history_qualification import (
    EXPECTED_G2_CONFIG_SHA256,
    FINAL_STATUSES,
    QUALIFICATION_CONDITION_KEYS,
    approach_history,
    class_transition,
    paired_trajectory_metrics,
    qualification_status,
    resolved_scientific_config,
    sha256_file,
    validate_config,
    validate_history_poses,
    validate_timestamps,
)
from reconciliation.lightnav_adapter import save_npy_exclusive


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture()
def config() -> dict:
    return yaml.safe_load((ROOT / "configs/stage0g3_moving_history_qualification.yaml").read_text())


def test_frozen_g2_control_path_hash_and_primary_run_are_required(config: dict) -> None:
    validate_config(config, ROOT)
    assert sha256_file(ROOT / config["frozen_control"]["config_path"]) == EXPECTED_G2_CONFIG_SHA256
    assert (ROOT / config["frozen_control"]["primary_run"]).is_dir()
    changed = copy.deepcopy(config); changed["frozen_control"]["expected_config_sha256"] = "0" * 64
    with pytest.raises(ValueError, match="SHA"): validate_config(changed, ROOT)


def test_g3_paired_metrics_and_qualification_thresholds_are_frozen(config: dict) -> None:
    changed = copy.deepcopy(config)
    changed["qualification"]["required_matches_per_scenario"] = 3
    with pytest.raises(ValueError, match="thresholds"):
        validate_config(changed, ROOT)
    changed = copy.deepcopy(config)
    changed["paired_comparison"]["translation_difference"] = "different"
    with pytest.raises(ValueError, match="paired comparison"):
        validate_config(changed, ROOT)


def test_resolved_config_changes_only_history_identity(config: dict) -> None:
    resolved = resolved_scientific_config(config, ROOT)
    g2 = yaml.safe_load((ROOT / config["frozen_control"]["config_path"]).read_text())
    assert resolved["robot"] == g2["robot"]
    assert resolved["environment"] == g2["environment"]
    assert resolved["camera"] == g2["camera"]
    assert resolved["baseline"] == g2["baseline"]
    assert resolved["scenarios"] == g2["scenarios"]
    assert resolved["variants"] == g2["variants"]
    assert resolved["capture"]["stationary_history"] is False
    assert resolved["robot"]["required_embodiment"] == "Jackal"
    assert resolved["robot"]["asset_relative_path"] == "Clearpath/Jackal/jackal.usd"
    assert resolved["environment"]["asset_relative_path"] == "/Isaac/Environments/Hospital/hospital.usd"
    assert resolved["baseline"]["expected_lightnav_git_sha"] == "a645828d81a8439651172197ca80a75dc1377977"


@pytest.mark.parametrize("pose", [
    np.array([19.0, 23.5, -np.pi / 2]),
    np.array([19.0, 26.7, np.pi / 2]),
    np.array([18.8, 26.9, np.pi]),
])
def test_approach_has_exact_64_poses_final_equality_constant_yaw_and_monotonic_motion(pose: np.ndarray) -> None:
    history = approach_history(pose, 1.5)
    result = validate_history_poses(history, pose, 1.5)
    assert history.shape == (64, 3)
    np.testing.assert_array_equal(history[-1], pose)
    assert np.linalg.norm(history[0, :2] - pose[:2]) == pytest.approx(1.5)
    assert np.all(history[:, 2] == pose[2])
    assert np.all(np.diff(np.linalg.norm(history[:, :2] - pose[:2], axis=1)) < 0)
    assert result["mean_interframe_translation_m"] == pytest.approx(1.5 / 63)


def test_symmetric_final_variants_produce_symmetric_history_offsets() -> None:
    base = np.array([10.0, 20.0, 0.7])
    left = base.copy(); right = base.copy()
    normal = np.array([-np.sin(base[2]), np.cos(base[2])])
    left[:2] += 0.09 * normal; right[:2] -= 0.09 * normal
    expected = np.tile(0.09 * normal, (64, 1))
    np.testing.assert_allclose(approach_history(left, 1.5)[:, :2] - approach_history(base, 1.5)[:, :2], expected)
    np.testing.assert_allclose(approach_history(right, 1.5)[:, :2] - approach_history(base, 1.5)[:, :2], -expected)


def test_timestamps_are_exactly_64_strictly_ordered_four_hz() -> None:
    values = 12.0 + np.arange(64) / 4.0
    assert validate_timestamps(values)["duration_s"] == pytest.approx(15.75)
    values[8] = values[7]
    with pytest.raises(ValueError, match="strictly ordered"): validate_timestamps(values)


def test_final_camera_pose_is_final_robot_pose_by_contract() -> None:
    final = np.array([1.0, 2.0, 3.0])
    history = approach_history(final, 1.0)
    np.testing.assert_array_equal(history[-1], final)


def test_pairwise_exact_equality_translation_wrapped_yaw_and_endpoint() -> None:
    stationary = np.zeros((10, 3), dtype=np.float32)
    moving = stationary.copy(); moving[:, 0] = 0.3; moving[:, 2] = np.float32(2 * np.pi - 0.1)
    before_a, before_b = stationary.copy(), moving.copy()
    result = paired_trajectory_metrics(stationary, moving)
    assert result["exact_raw_equal"] is False
    assert result["translation_rms_difference_m"] == pytest.approx(0.3)
    assert result["wrapped_yaw_rms_difference_rad"] == pytest.approx(0.1, abs=1e-6)
    assert result["delta_endpoint_forward_m"] == pytest.approx(0.3)
    assert result["delta_endpoint_yaw_rad"] == pytest.approx(-0.1, abs=1e-6)
    np.testing.assert_array_equal(stationary, before_a); np.testing.assert_array_equal(moving, before_b)
    assert paired_trajectory_metrics(stationary, stationary.copy())["exact_raw_equal"] is True


def test_frozen_geometry_class_transition_is_explicit() -> None:
    changed = class_transition("STRAIGHT", "LEFT")
    assert changed == {
        "stationary_class": "STRAIGHT",
        "moving_class": "LEFT",
        "class_transition": "STRAIGHT->LEFT",
        "class_changed": True,
    }
    assert class_transition("RIGHT", "RIGHT")["class_changed"] is False
    with pytest.raises(ValueError, match="non-empty"):
        class_transition("", "LEFT")


@pytest.mark.parametrize("bad", [np.full((10, 3), np.nan, np.float32), np.full((10, 3), np.inf, np.float32)])
def test_pairwise_nan_inf_rejected_and_source_immutable(bad: np.ndarray) -> None:
    before = bad.copy()
    with pytest.raises(ValueError, match="finite"): paired_trajectory_metrics(np.zeros((10, 3), np.float32), bad)
    np.testing.assert_array_equal(bad, before)


def test_history_nan_inf_and_wrong_final_rejected_without_mutation() -> None:
    final = np.array([1.0, 2.0, 0.4]); values = approach_history(final, 1.0); before = values.copy()
    with pytest.raises(ValueError, match="does not equal"): validate_history_poses(values, final + [0.01, 0.0, 0.0], 1.0)
    np.testing.assert_array_equal(values, before)
    values[2, 0] = np.nan
    with pytest.raises(ValueError, match="finite"): validate_history_poses(values, final, 1.0)


def test_isaac_float32_start_quantization_does_not_weaken_final_pose_gate() -> None:
    final = np.array([19.0, 31.50035285949707, -np.pi / 2])
    values = approach_history(final, 1.2).astype(np.float32).astype(np.float64)
    values[:, 2] = final[2]; values[-1] = final
    assert validate_history_poses(values, final, 1.2)["nonzero_motion"] is True
    values[-1, 0] += 2e-6
    with pytest.raises(ValueError, match="final pose"): validate_history_poses(values, final, 1.2)


def test_decision_vocab_pass_fail_infeasible_and_technical_invalid() -> None:
    assert qualification_status({"all": True}) == "STAGE0G3_READY_FOR_SUCCESSIVE_DATA_COLLECTION"
    assert qualification_status({"all": False}) == "STAGE0G3_MOVING_HISTORY_QUALIFICATION_FAILED"
    assert qualification_status({"all": True}, approach_feasible=False) == "STAGE0G3_APPROACH_PATH_NOT_FEASIBLE"
    assert qualification_status({"all": True}, technical_valid=False) == "STAGE0G3_TECHNICAL_INVALID"
    assert len(FINAL_STATUSES) == 4
    assert len(QUALIFICATION_CONDITION_KEYS) == 10


def test_no_pose_accumulation_world_anchor_uses_only_final_observation_pose() -> None:
    final = np.array([4.0, 5.0, np.pi / 2])
    raw = np.column_stack((np.arange(1, 11), np.zeros(10), np.zeros(10))).astype(np.float32)
    c, s = np.cos(final[2]), np.sin(final[2])
    world_xy = final[:2] + raw[:, :1] * np.array([c, s]) + raw[:, 1:2] * np.array([-s, c])
    np.testing.assert_allclose(world_xy[-1], [4.0, 15.0], atol=1e-6)
    assert wrap_angle(final[2] + raw[-1, 2]) == pytest.approx(np.pi / 2)


def test_diagnostic_outputs_refuse_overwrite(tmp_path: Path) -> None:
    target = tmp_path / "history_pose_se2.npy"
    source = approach_history(np.array([1.0, 2.0, 0.3]), 1.0)
    save_npy_exclusive(target, source)
    with pytest.raises(FileExistsError): save_npy_exclusive(target, source)

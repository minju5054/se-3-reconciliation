import copy
from pathlib import Path

import numpy as np
import pytest
import yaml

from reconciliation.stage0g2_jackal_domain_scene_qualification import (
    FINAL_STATUSES, SCENARIO_IDS, VARIANT_IDS, classify_descriptors,
    compare_stage0g, dominant_exact_raw_fraction, doorway_distinct_status,
    geometry_descriptors, qualification_status, sha256_file, validate_asset_manifest,
    validate_config, validate_preview_gate, variant_pose,
)
from reconciliation.se2 import wrap_angle


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture()
def config() -> dict:
    return yaml.safe_load((ROOT / "configs/stage0g2_jackal_domain_scene_qualification.yaml").read_text())


def trajectory(lateral: float = 0.0, yaw: float = 0.0) -> np.ndarray:
    t = np.linspace(0.1, 1.0, 10); return np.column_stack((1.5 * t, lateral * t, yaw * t)).astype(np.float32)


def test_frozen_contract_requires_jackal_hospital_camera_and_lightnav(config: dict) -> None:
    validate_config(config)
    assert config["robot"]["required_embodiment"] == "Jackal"
    assert config["environment"]["family"] == "isaac_hospital_6_0_1"
    assert config["camera"]["resolution_width"] == 480
    assert config["camera"]["resolution_height"] == 270
    assert config["camera"]["horizontal_fov_deg"] == 112.0
    assert config["capture"] == {"frame_count": 64, "fps": 4.0, "render_subframes": 2, "stationary_history": True}
    assert config["lightnav"]["intrinsic_waypoint_time_base"] is False


def test_alternate_robot_is_rejected(config: dict) -> None:
    changed = copy.deepcopy(config); changed["robot"]["required_embodiment"] = "Carter"
    with pytest.raises(ValueError, match="Jackal"): validate_config(changed)


def test_scenario_ids_and_mirrored_variants_in_each_robot_frame(config: dict) -> None:
    assert tuple(x["id"] for x in config["scenarios"]) == SCENARIO_IDS
    assert tuple(x["id"] for x in config["variants"]) == VARIANT_IDS
    for q in SCENARIO_IDS:
        p0, p1, p2, p3, p4 = (variant_pose(config, q, v) for v in VARIANT_IDS)
        assert wrap_angle(p1[2] - p0[2]) == pytest.approx(0.05)
        assert wrap_angle(p2[2] - p0[2]) == pytest.approx(-0.05)
        np.testing.assert_allclose(p3[:2] - p0[:2], -(p4[:2] - p0[:2]), atol=1e-12)


def test_route_clearance_and_environment_selection_are_explicit(config: dict) -> None:
    assert config["environment"]["selection_fixed_before_inference"] is True
    assert config["environment"]["scale"] == 1.0
    assert all(item["route_width_m"] > config["robot"]["approximate_footprint_width_m"] for item in config["scenarios"])
    assert all(item["minimum_intended_clearance_m"] > 0 for item in config["scenarios"])


def test_classification_and_wrapped_yaw_match_frozen_stage0g_criteria(config: dict) -> None:
    assert classify_descriptors(geometry_descriptors(trajectory()), "straight", config["qualification"])["matches_intended_geometry"]
    assert classify_descriptors(geometry_descriptors(trajectory(.35, .4)), "left", config["qualification"])["matches_intended_geometry"]
    values = trajectory(0, 2 * np.pi + .1); assert geometry_descriptors(values)["signed_net_yaw_rad"] == pytest.approx(.1, abs=1e-6)


@pytest.mark.parametrize("bad", [np.full((10, 3), np.nan, np.float32), np.full((10, 3), np.inf, np.float32)])
def test_nan_and_inf_rejected_without_source_mutation(bad: np.ndarray) -> None:
    before = bad.copy()
    with pytest.raises(ValueError): geometry_descriptors(bad)
    np.testing.assert_array_equal(bad, before)


def test_source_array_is_immutable_and_dominant_fraction_is_exact() -> None:
    a = trajectory(); b = a.copy(); c = trajectory(.4, .3); before = a.copy()
    assert dominant_exact_raw_fraction({"a": a, "b": b, "c": c}) == pytest.approx(2 / 3)
    np.testing.assert_array_equal(a, before)


def test_doorway_distinctness_is_separate_from_task_local_forward_pass() -> None:
    assert doorway_distinct_status({"same"}, {"same"}, set()) == "DISTINCT_INTENT_NOT_DEMONSTRATED"
    assert doorway_distinct_status({"door"}, {"straight"}, {"turn"}) == "DISTINCT_INTENT_DEMONSTRATED"


def test_preview_gate_requires_every_field_true() -> None:
    gate = {key: True for key in ("free_space_visible", "intended_route_visible", "referenced_feature_visible", "natural_scene", "camera_usable", "initial_pose_collision_free")}
    validate_preview_gate(gate); gate["camera_usable"] = False
    with pytest.raises(ValueError, match="must pass"): validate_preview_gate(gate)


def test_comparison_is_descriptive_and_status_vocabulary_is_exact() -> None:
    value = compare_stage0g({"unique_raw_output_count": 3}, {"unique_raw_output_count": 5})
    assert value["comparison_is_descriptive_only"] is True
    assert value["no_causal_statistical_claim"] is True
    assert FINAL_STATUSES == {"STAGE0G2_READY_FOR_DATA_COLLECTION", "STAGE0G2_SCENE_QUALIFICATION_FAILED", "STAGE0G2_NO_SUITABLE_DOMAIN_SCENE", "STAGE0G2_TECHNICAL_INVALID"}


def test_asset_manifest_schema_and_single_preinference_selection() -> None:
    candidates = []
    for family in ("Office", "Hospital", "Simple_Room", "Simple_Warehouse"):
        candidates.append({"family": family, "relative_path": "/asset.usd", "available": True, "loadable": True, "scale_m_per_stage_unit": 1.0, "lighting": "authored", "prim_count": 1, "collision_prim_count": 1,
                           "visual_quality": "recorded", "jackal_fit": "recorded", "decision": "SELECTED_BEFORE_INFERENCE" if family == "Hospital" else "REJECT"})
    manifest = {"audit_before_primary_inference": True, "candidates": candidates}; validate_asset_manifest(manifest)
    manifest["candidates"][0]["decision"] = "SELECTED_BEFORE_INFERENCE"
    with pytest.raises(ValueError, match="exactly one"): validate_asset_manifest(manifest)


def test_config_hash_and_decision_logic(tmp_path: Path, config: dict) -> None:
    snapshot = tmp_path / "config.yaml"; snapshot.write_bytes((ROOT / "configs/stage0g2_jackal_domain_scene_qualification.yaml").read_bytes())
    assert len(sha256_file(snapshot)) == 64
    assert qualification_status({"a": True, "b": True}) == "STAGE0G2_READY_FOR_DATA_COLLECTION"
    assert qualification_status({"a": True, "b": False}) == "STAGE0G2_SCENE_QUALIFICATION_FAILED"

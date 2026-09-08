from __future__ import annotations

import copy
from pathlib import Path

import numpy as np
import pytest

from reconciliation.lightnav_execution_envelope import (
    classify_fresh_output,
    compare_descriptor_to_fixtures,
    extract_suffixes,
    final_envelope_decision,
    fit_fixture_distance_model,
    geometry_descriptor,
    group_records_by_hash,
    rigid_transform_trajectory,
    select_representatives,
    severity_key,
    trajectory_content_sha256,
    verify_named_hashes,
    write_json_exclusive,
)
from reconciliation.online_switch import sha256_file


def arc(sign: float = 1.0) -> np.ndarray:
    angles = np.linspace(0.0, sign * 0.8, 9)
    radius = 2.0
    return np.column_stack(
        (
            radius * np.sin(np.abs(angles)),
            sign * radius * (1.0 - np.cos(angles)),
            angles,
        )
    )


def fixture_geometry() -> dict:
    straight = geometry_descriptor(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    )
    gentle = geometry_descriptor(arc())
    strong_path = arc()
    strong_path[:, :2] *= 0.25
    strong = geometry_descriptor(strong_path)
    return {
        "fixtures": [
            {
                "fixture_id": "straight",
                "calibrated_passed": True,
                "descriptor": straight,
            },
            {
                "fixture_id": "gentle",
                "calibrated_passed": True,
                "descriptor": gentle,
            },
            {
                "fixture_id": "strong_left",
                "calibrated_passed": False,
                "descriptor": strong,
            },
        ],
        "pass_fixture_ids": ["straight", "gentle"],
        "fail_fixture_ids": ["strong_left"],
    }


def distance_config() -> dict:
    return {
        "features": [
            "path_length_m",
            "abs_net_pose_yaw_change_rad",
            "tangent_curvature_p95_abs_per_m",
        ],
        "normalization": "stage0e_fixture_min_max_range",
        "scale_floor": 1e-9,
        "ambiguous_margin_fraction_of_outside_radius": 0.1,
    }


def test_straight_descriptor_has_no_invented_curvature_or_radius() -> None:
    path = np.array(
        [[0.0, 0.0, 0.0], [0.5, 0.0, 0.0], [1.25, 0.0, 0.0]]
    )
    descriptor = geometry_descriptor(path)
    assert descriptor["number_of_poses"] == 3
    assert descriptor["path_length_m"] == pytest.approx(1.25)
    assert descriptor["endpoint_displacement_m"] == pytest.approx(1.25)
    assert descriptor["tangent_curvature_max_abs_per_m"] == pytest.approx(0.0)
    assert descriptor["minimum_finite_local_turning_radius_m"] is None
    assert descriptor["curvature_sign_change_count"] == 0


def test_left_right_curvature_sign_and_s_curve_sign_change() -> None:
    left = geometry_descriptor(arc(1.0))
    right = geometry_descriptor(arc(-1.0))
    assert left["cumulative_signed_tangent_turn_rad"] > 0.0
    assert right["cumulative_signed_tangent_turn_rad"] < 0.0
    left_half = arc(1.0)[:5]
    delta = arc(-1.0)[:5]
    rotation = left_half[-1, 2]
    c, s = np.cos(rotation), np.sin(rotation)
    rotated = delta[:, :2] @ np.array([[c, s], [-s, c]])
    second = np.column_stack(
        (
            rotated + left_half[-1, :2],
            left_half[-1, 2] + delta[:, 2],
        )
    )
    s_curve = np.vstack((left_half, second[1:]))
    descriptor = geometry_descriptor(s_curve)
    assert descriptor["curvature_sign_change_count"] >= 1


def test_near_zero_segments_and_yaw_wrap_do_not_divide_by_zero() -> None:
    path = np.array(
        [
            [0.0, 0.0, np.pi - 0.05],
            [0.0, 0.0, -np.pi + 0.05],
            [1.0, 0.0, -np.pi + 0.10],
        ]
    )
    descriptor = geometry_descriptor(path)
    assert descriptor["near_zero_segment_count"] == 1
    assert descriptor["near_zero_segment_fraction"] == pytest.approx(0.5)
    assert descriptor["max_abs_pose_yaw_increment_rad"] == pytest.approx(0.10)
    assert descriptor["pose_yaw_per_meter_max_abs_rad_per_m"] == pytest.approx(0.05)


def test_pose_yaw_progression_is_separate_from_xy_tangent() -> None:
    path = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.4], [2.0, 0.0, 0.8]]
    )
    descriptor = geometry_descriptor(path)
    assert descriptor["pose_yaw_per_meter_max_abs_rad_per_m"] == pytest.approx(0.4)
    assert descriptor["tangent_curvature_max_abs_per_m"] == pytest.approx(0.0)
    assert descriptor["pose_yaw_and_xy_tangent_are_distinct"] is True


def test_rigid_se2_transform_preserves_descriptor_and_source() -> None:
    source = arc()
    before = source.copy()
    transformed = rigid_transform_trajectory(source, [3.0, -2.0, 1.1])
    original_descriptor = geometry_descriptor(source)
    transformed_descriptor = geometry_descriptor(transformed)
    for key in (
        "path_length_m",
        "endpoint_displacement_m",
        "net_pose_yaw_change_rad",
        "cumulative_abs_pose_yaw_change_rad",
        "tangent_curvature_p95_abs_per_m",
        "cumulative_abs_tangent_turn_rad",
        "curvature_sign_change_count",
    ):
        assert transformed_descriptor[key] == pytest.approx(original_descriptor[key])
    assert np.array_equal(source, before)


def test_unique_hash_grouping_stop_and_suffix_semantics() -> None:
    first = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    second = first.copy()
    third = np.array([[0.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    rows = [
        {"sha": trajectory_content_sha256(value), "index": index}
        for index, value in enumerate((first, second, third))
    ]
    grouped = group_records_by_hash(rows, "sha")
    assert sorted(len(values) for values in grouped.values()) == [1, 2]
    assert classify_fresh_output(np.zeros((10, 3))) == "STOP_OUTPUT"
    assert classify_fresh_output(first) == "MOVING_OUTPUT"
    suffixes = extract_suffixes(np.vstack((first, [[2.0, 0.0, 0.0]])))
    assert [k for k, _ in suffixes] == [0, 1]
    assert [value.shape[0] for _, value in suffixes] == [3, 2]
    with pytest.raises(ValueError, match=">= 2"):
        extract_suffixes(first, minimum_poses=1)


def test_fixture_only_normalization_labels_and_severity_order() -> None:
    fixtures = fixture_geometry()
    model = fit_fixture_distance_model(fixtures, distance_config())
    assert model["fitted_from_lightnav"] is False
    for fixture in fixtures["fixtures"]:
        result = compare_descriptor_to_fixtures(
            fixture["descriptor"], fixtures, model
        )
        assert result["nearest_fixture"] == fixture["fixture_id"]
        assert result["descriptive_not_classifier"] is True
    straight = fixtures["fixtures"][0]["descriptor"]
    strong = fixtures["fixtures"][-1]["descriptor"]
    order = [
        "tangent_curvature_p95_abs_per_m",
        "cumulative_abs_tangent_turn_rad",
        "max_abs_pose_yaw_increment_rad",
    ]
    assert severity_key(straight, order) < severity_key(strong, order)


def test_representative_selection_is_deterministic_and_merges_duplicates() -> None:
    fixtures = fixture_geometry()
    model = fit_fixture_distance_model(fixtures, distance_config())
    rows = []
    for kind in ("OLD", "FRESH"):
        for index, fixture in enumerate(fixtures["fixtures"]):
            digest = f"{index + (0 if kind == 'OLD' else 10):064x}"
            rows.append(
                {
                    "reference_id": f"{kind.lower()}_{index}",
                    "kind": kind,
                    "world_trajectory_sha256": digest,
                    "descriptor": fixture["descriptor"],
                    "fixture_comparison": compare_descriptor_to_fixtures(
                        fixture["descriptor"], fixtures, model
                    ),
                }
            )
    order = [
        "tangent_curvature_p95_abs_per_m",
        "cumulative_abs_tangent_turn_rad",
        "max_abs_pose_yaw_increment_rad",
    ]
    first = select_representatives(rows, order)
    second = select_representatives(copy.deepcopy(rows), order)
    assert first == second
    assert first["selection_frozen_before_execution"] is True
    assert first["reference_count"] <= 8
    assert any(
        "nearest_strong_turn_failing_fixture" in row["reasons"]
        for row in first["references"]
    )


def test_final_envelope_labels() -> None:
    passed_old = [{"passed": True, "fixture_label": "PASS_FIXTURE_LIKE"}]
    passed_fresh = [{"passed": True, "fixture_label": "PASS_FIXTURE_LIKE"}]
    covered = final_envelope_decision(
        old_results=passed_old, fresh_results=passed_fresh
    )
    assert (
        covered["status"]
        == "LIGHTNAV_ENVELOPE_COVERED_BY_CURRENT_CALIBRATED_PLATFORM"
    )
    strong_fail = [{"passed": False, "fixture_label": "STRONG_TURN_FAIL_FIXTURE_LIKE"}]
    result = final_envelope_decision(
        old_results=passed_old, fresh_results=strong_fail
    )
    assert result["status"] == "LIGHTNAV_ENVELOPE_REACHES_UNVALIDATED_STRONG_TURN_REGION"
    assert final_envelope_decision(old_results=[], fresh_results=[])["status"].endswith(
        "INSUFFICIENT_DATA"
    )
    assert result["execution_platform_validated"] is False


def test_exclusive_output_nan_rejection_and_source_hash_validation(
    tmp_path: Path,
) -> None:
    source = tmp_path / "source.bin"
    source.write_bytes(b"frozen")
    digest = sha256_file(source)
    assert verify_named_hashes(tmp_path, {"source.bin": digest}) == {
        "source.bin": digest
    }
    with pytest.raises(ValueError, match="provenance mismatch"):
        verify_named_hashes(tmp_path, {"source.bin": "0" * 64})
    output = tmp_path / "summary.json"
    write_json_exclusive(output, {"finite": 1.0})
    with pytest.raises(FileExistsError):
        write_json_exclusive(output, {"finite": 1.0})
    with pytest.raises(ValueError, match="finite"):
        write_json_exclusive(tmp_path / "bad.json", {"bad": float("nan")})
    with pytest.raises(ValueError, match="finite"):
        geometry_descriptor([[0.0, 0.0, 0.0], [np.inf, 0.0, 0.0]])

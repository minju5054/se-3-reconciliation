import copy
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from reconciliation.exp01b_controlled_latency import (
    ControlledLatencyTiming,
    aggregate_controlled,
    classify_controlled_attempt,
    enumerate_primary_attempts,
    select_representative_samples,
    validate_attempt_output,
    validate_controlled_config,
)


def config(*, frozen=True):
    geometries = []
    for identifier, geometry_class in (("G0", "straight"), ("G1", "turn"), ("G2", "route_change")):
        geometries.append(
            {
                "id": identifier,
                "geometry_class": geometry_class,
                "instruction": f"fixture {geometry_class}",
                "scene_id": f"scene_{geometry_class}",
                "initial_pose_se2": [0.0, 0.0, 0.0],
                "fresh_observation_delay_s": 0.5,
                "qualification": {"accepted": frozen, "artifact": f"qualification/{identifier}"},
            }
        )
    return {
        "controlled_latency_design": {
            "primary_config_frozen": frozen,
            "target_valid_moving_per_cell": 5,
            "max_attempts_per_cell": 10,
            "latency_conditions": [
                {"id": "L0_natural", "added_delay_s": 0.0},
                {"id": "L1_added_050", "added_delay_s": 0.5},
            ],
            "geometry_conditions": geometries,
        }
    }


def record(cell="G0__L0_natural", attempt=0, classification="VALID_MOVING", dv=0.1, dw=0.2):
    return {
        "cell_id": cell,
        "attempt_index": attempt,
        "artifact_path": f"/tmp/{cell}/attempt_{attempt:03d}",
        "classification": classification,
        "stop_output": classification == "MODEL_STOP_OUTPUT",
        "controller_metrics": {
            "delta_v_abs_mps": dv,
            "delta_omega_abs_rps": dw,
            "post_switch_max_abs_delta_v_mps": dv,
            "post_switch_mean_abs_delta_v_mps": dv,
            "post_switch_max_abs_delta_omega_rps": dw,
            "post_switch_mean_abs_delta_omega_rps": dw,
        },
        "geometry": {
            "translation_pose_gap_m": 0.03,
            "yaw_pose_gap_rad": 0.02,
            "local_spatial_step_magnitude_mismatch_m": 0.1,
            "old_fresh_tangent_disagreement_abs_rad": 0.15,
        },
        "timing": {
            "effective_latency_sim_s": 0.5,
            "model_latency_wall_s": 0.45,
            "robot_translation_observation_to_switch_m": 0.17,
            "robot_yaw_observation_to_switch_rad": 0.01,
            "real_time_factor": 1.0,
        },
    }


def test_exact_two_by_three_condition_enumeration_and_attempt_cap() -> None:
    cells = validate_controlled_config(config())
    assert len(cells) == 6
    assert [cell["cell_id"] for cell in cells] == [
        "G0__L0_natural", "G0__L1_added_050",
        "G1__L0_natural", "G1__L1_added_050",
        "G2__L0_natural", "G2__L1_added_050",
    ]
    assert len(enumerate_primary_attempts(config())) == 60


def test_invalid_latency_geometry_and_unqualified_frozen_config_rejected() -> None:
    source = config()
    source["controlled_latency_design"]["latency_conditions"][1]["added_delay_s"] = 0.25
    with pytest.raises(ValueError, match="exactly 0.0 and 0.5"):
        validate_controlled_config(source)
    source = config()
    source["controlled_latency_design"]["geometry_conditions"][0]["geometry_class"] = "fake"
    with pytest.raises(ValueError):
        validate_controlled_config(source)
    source = config()
    source["controlled_latency_design"]["geometry_conditions"][1]["qualification"]["accepted"] = False
    with pytest.raises(ValueError, match="accepted"):
        validate_controlled_config(source)


def test_model_ready_and_fresh_usable_timing_are_distinct() -> None:
    timing = ControlledLatencyTiming(
        observation_sim_time_s=10.0,
        request_host_monotonic_ns=1_000_000_000,
        model_ready_sim_time_s=10.45,
        model_ready_host_monotonic_ns=1_450_000_000,
        fresh_usable_sim_time_s=10.95,
        fresh_usable_host_monotonic_ns=1_950_000_000,
        configured_added_delay_s=0.5,
    ).to_dict()
    assert timing["model_latency_wall_s"] == pytest.approx(0.45)
    assert timing["measured_added_delay_sim_s"] == pytest.approx(0.5)
    assert timing["effective_latency_sim_s"] == pytest.approx(0.95)
    assert timing["real_time_factor"] == pytest.approx(1.0)


def test_invalid_timing_order_and_nan_rejected() -> None:
    with pytest.raises(ValueError):
        ControlledLatencyTiming(1.0, 100, 0.9, 200, 1.5, 300, 0.5)
    with pytest.raises(ValueError):
        ControlledLatencyTiming(1.0, 100, 1.5, 200, np.nan, 300, 0.5)


def test_attempt_classification_requires_old_active_and_controller_capture() -> None:
    checks = {
        "response_received": True,
        "rtf_in_range": True,
        "old_active_during_model": True,
        "old_active_during_added_delay": True,
        "robot_moved_observation_to_switch": True,
        "controller_switch_recorded": True,
        "request_after_observation": True,
        "fresh_anchored_at_observation": True,
        "fresh_raw_unchanged": True,
    }
    assert classify_controlled_attempt(checks=checks, stop_output=False) == "VALID_MOVING"
    assert classify_controlled_attempt(checks=checks, stop_output=True) == "MODEL_STOP_OUTPUT"
    changed = {**checks, "old_active_during_added_delay": False}
    assert classify_controlled_attempt(checks=changed, stop_output=False) == "OLD_EXHAUSTED"
    changed = {**checks, "controller_switch_recorded": False}
    assert classify_controlled_attempt(checks=changed, stop_output=False) == "OTHER_PROTOCOL_FAILURE"
    changed = {**checks, "request_after_observation": False}
    assert classify_controlled_attempt(checks=changed, stop_output=False) == "OTHER_PROTOCOL_FAILURE"


def test_aggregate_and_representative_are_deterministic() -> None:
    cell = "G0__L0_natural"
    rows = [record(cell, 0, dv=0.1), record(cell, 1, dv=0.2), record(cell, 2, dv=0.3)]
    result = aggregate_controlled(rows, cell_ids=[cell])
    assert result["per_cell"][cell]["valid_moving"] == 3
    assert result["per_cell"][cell]["statistics"]["delta_v_abs_mps"]["median"] == pytest.approx(0.2)
    selected = select_representative_samples(rows, [cell])
    assert selected[cell].endswith("attempt_001")


def test_config_validation_does_not_mutate_and_strict_json_round_trip() -> None:
    source = config()
    before = copy.deepcopy(source)
    cells = validate_controlled_config(source)
    encoded = json.dumps(cells, allow_nan=False, sort_keys=True)
    assert json.loads(encoded) == cells
    assert source == before


def test_no_graph_or_exp02a_fields_enter_protocol() -> None:
    encoded = json.dumps(validate_controlled_config(config())).lower()
    for forbidden in ("spatialentry", "correspondence", "graph", "gtsam", "entry_index"):
        assert forbidden not in encoded


def test_strict_attempt_reconstruction_and_raw_immutability(tmp_path: Path) -> None:
    root = tmp_path / "attempt_000"
    for directory in ("raw", "derived", "results"):
        (root / directory).mkdir(parents=True, exist_ok=True)
    old = np.array([[0.1, 0.0, 0.0], [0.2, 0.0, 0.0]], dtype=np.float32)
    fresh = np.array([[0.1, 0.0, 0.0], [0.25, 0.05, 0.1]], dtype=np.float32)
    np.save(root / "raw/old_actions.npy", old, allow_pickle=False)
    np.save(root / "raw/fresh_actions.npy", fresh, allow_pickle=False)
    digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    metadata = {
        "intrinsic_waypoint_time_base": False,
        "robot_pose_at_new_observation": [1.0, 2.0, 0.0],
        "raw_sha256": {
            "old_actions.npy": digest(root / "raw/old_actions.npy"),
            "fresh_actions.npy": digest(root / "raw/fresh_actions.npy"),
        },
    }
    (root / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
    from reconciliation.lightnav_adapter import lightnav_local_to_world
    np.save(
        root / "derived/fresh_world.npy",
        lightnav_local_to_world(fresh, metadata["robot_pose_at_new_observation"]),
        allow_pickle=False,
    )
    command_rows = [
        (0, 0.0, "OLD", 0.1, 0.0),
        (1, 0.1, "OLD", 0.2, 0.1),
        (2, 0.2, "OLD", 0.3, 0.2),
        (3, 0.3, "FRESH", 0.5, 0.4),
        (4, 0.4, "FRESH", 0.4, 0.3),
        (5, 0.5, "FRESH", 0.35, 0.2),
    ]
    with (root / "derived/controller_commands.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["command_index", "sim_time_s", "reference_source", "v_command_mps", "omega_command_rps"])
        writer.writeheader()
        for index, sim, source, v, omega in command_rows:
            writer.writerow({"command_index": index, "sim_time_s": sim, "reference_source": source, "v_command_mps": v, "omega_command_rps": omega})
    from reconciliation.controller_switch_metrics import controller_switch_metrics
    controller = controller_switch_metrics(
        [[0.1, 0.0], [0.2, 0.1], [0.3, 0.2]],
        [[0.5, 0.4], [0.4, 0.3], [0.35, 0.2]],
        control_dt_s=0.1,
    )
    timing = {"effective_latency_sim_s": 0.5}
    geometry = {"translation_pose_gap_m": 0.1}
    attempt = {"timing": timing, "controller_metrics": controller, "geometry": geometry, "stop_output": False, "classification": "VALID_MOVING"}
    for name, value in (("timing", timing), ("controller_switch_metrics", controller), ("geometry", geometry), ("attempt", attempt)):
        (root / f"results/{name}.json").write_text(json.dumps(value, allow_nan=False), encoding="utf-8")

    before = (root / "raw/fresh_actions.npy").read_bytes()
    assert validate_attempt_output(root)["valid_output"] is True
    assert (root / "raw/fresh_actions.npy").read_bytes() == before
    corrupted = np.load(root / "derived/fresh_world.npy", allow_pickle=False)
    corrupted[0, 0] += 0.01
    np.save(root / "derived/fresh_world.npy", corrupted, allow_pickle=False)
    with pytest.raises(ValueError, match="anchored at observation"):
        validate_attempt_output(root)

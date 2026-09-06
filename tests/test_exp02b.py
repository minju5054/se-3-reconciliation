from pathlib import Path

import numpy as np
import pytest
import yaml

from reconciliation.exp02b import (
    METHODS,
    SourceCase,
    build_candidate_methods,
    comparability_gate,
    controller_improvement,
    geometric_preservation_metrics,
    inter_k_retention,
    select_source_cases,
    transition_input,
    validate_exp02b_config,
    verify_frozen_source_selection,
)
from reconciliation.se2 import compose_poses


def path(count=10):
    poses = [np.array([0.3, 0.1, 0.1])]
    for _ in range(count - 1):
        poses.append(compose_poses(poses[-1], [0.15, 0.0, 0.04]))
    return np.asarray(poses)


def test_raw_and_rigid_preservation_metrics_and_turn_sign() -> None:
    fresh = path()
    metrics = geometric_preservation_metrics(
        raw_reference=fresh,
        candidate=fresh.copy(),
        previous_pose_world=[-0.1, 0.0, 0.0],
        boundary_pose_world=[0.0, 0.0, 0.0],
        minimum_translation_m=1e-6,
    )
    assert metrics["selected_entry_displacement"]["translation_m"] == pytest.approx(0.0)
    assert metrics["fresh_edge_deformation"]["translation_rms_m"] == pytest.approx(0.0)
    assert metrics["endpoint_deviation"]["translation_m"] == pytest.approx(0.0)
    assert metrics["yaw_progression"]["turn_direction_preserved"] is True


def test_inter_k_retention_distinguishes_outputs() -> None:
    fresh = path()
    states = {0: fresh[0:].copy(), 3: fresh[3:].copy(), 6: fresh[6:].copy()}
    retained = inter_k_retention(fresh_world=fresh, entry_indices=[0, 3, 6], states_by_k=states)
    assert retained["pair_count"] == 3
    assert all(row["entry_separation_retention_ratio"] == pytest.approx(1.0) for row in retained["pairs"])
    collapsed = {key: np.vstack(([0.0, 0.0, 0.0], value[1:])) for key, value in states.items()}
    result = inter_k_retention(fresh_world=fresh, entry_indices=[0, 3, 6], states_by_k=collapsed)
    assert all(row["entry_separation_retention_ratio"] == 0.0 for row in result["pairs"])


def test_comparability_gate_and_wrapped_yaw() -> None:
    result = comparability_gate(
        source_boundary=[0, 0, np.pi - 0.01],
        reproduced_boundary=[0.005, 0, -np.pi + 0.01],
        source_pre_switch_command=[0.2, 0.3],
        reproduced_pre_switch_command=[0.2, 0.3],
        translation_tolerance_m=0.01,
        yaw_tolerance_rad=0.03,
        command_tolerance=1e-9,
    )
    assert result["valid"] is True
    assert result["yaw_error_rad"] == pytest.approx(0.02)
    failed = comparability_gate(
        source_boundary=[0, 0, 0],
        reproduced_boundary=[0.02, 0, 0],
        source_pre_switch_command=[0.2, 0.3],
        reproduced_pre_switch_command=[0.2, 0.3],
        translation_tolerance_m=0.01,
        yaw_tolerance_rad=0.01,
        command_tolerance=1e-9,
    )
    assert failed["valid"] is False


def test_controller_improvement_near_zero_is_undefined() -> None:
    assert controller_improvement(0.1, 0.2, epsilon=1e-9) == pytest.approx(0.5)
    assert controller_improvement(0.0, 1e-12, epsilon=1e-9) is None


def test_exp02b_has_exact_method_set_and_no_selector_method() -> None:
    assert METHODS == ("raw_f0", "raw_k", "pose_anchor", "rigid", "graph")
    assert not any("selector" in method or "evidence" in method for method in METHODS)


def test_deterministic_source_selection_and_benign_rule(monkeypatch, tmp_path: Path) -> None:
    def row(path, dv, dw, latency="L0_natural"):
        return {
            "artifact_path": str(tmp_path / path),
            "classification": "VALID_MOVING",
            "latency_condition_id": latency,
            "controller_metrics": {"delta_v_abs_mps": dv, "delta_omega_abs_rps": dw},
        }

    records = [
        row("z", 0.5, 0.1),
        row("a", 0.5, 0.2),
        row("turn", 0.2, 1.2),
        row("delayed_bad", 0.4, 0.4, "L1_added_050"),
        row("delayed_good", 0.01, 0.01, "L1_added_050"),
        {**row("stop", 9, 9), "classification": "MODEL_STOP_OUTPUT"},
    ]
    monkeypatch.setattr("reconciliation.exp02b.load_attempt_records", lambda _: records)
    selected = select_source_cases(tmp_path, epsilon=1e-12)
    assert Path(selected["case_high_delta_v"]["artifact_path"]).name == "a"
    assert Path(selected["case_high_delta_omega"]["artifact_path"]).name == "turn"
    assert Path(selected["case_benign_delayed"]["artifact_path"]).name == "delayed_good"


def test_candidates_keep_raw_baselines_immutable_and_graph_contract() -> None:
    repository = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((repository / "configs/exp02b_controller_aware.yaml").read_text())
    graph_config = yaml.safe_load((repository / "configs/exp02a_spatial_entry.yaml").read_text())
    assert validate_exp02b_config(config)["entry_indices"] == [0, 3, 6]
    assert config["graph_contract"] == {
        "pose_anchor_variant": "pose_anchor",
        "graph_variant": "incoming_motion_aware",
        "objective_unchanged_from_exp02a": True,
        "controller_residuals": False,
        "selector_implemented": False,
        "z_evidence_used": False,
    }
    fresh = path()
    old = np.array([[-0.4, 0, 0], [-0.2, 0, 0], [0, 0, 0]], dtype=float)
    source = SourceCase(
        "case_high_delta_v",
        Path("/immutable/source"),
        old,
        fresh,
        np.array([-0.1, 0, 0], dtype=float),
        np.array([0, 0, 0], dtype=float),
        {},
        {},
        np.array([[0.1, 0], [0.2, 0], [0.3, 0]], dtype=float),
    )
    original = fresh.copy()
    inputs = transition_input(source, 3)
    candidates, _ = build_candidate_methods(inputs, graph_config=graph_config)
    assert np.array_equal(candidates["raw_f0"], fresh)
    assert np.array_equal(candidates["raw_k"], fresh[3:])
    assert candidates["graph"].shape == fresh[3:].shape
    assert np.array_equal(fresh, original)
    assert inputs.entry_context.evidence == {}


def test_frozen_source_hashes_and_selection_when_local_data_exist() -> None:
    repository = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((repository / "configs/exp02b_controller_aware.yaml").read_text())
    source = repository / config["paths"]["source_root"]
    if not source.is_dir():
        pytest.skip("ignored immutable EXP-01B source data are not present")
    selected = verify_frozen_source_selection(source, config)
    assert selected["_normalization"]["valid_moving_count"] == 30


def test_metrics_reject_undefined_incoming_and_shape_mismatch() -> None:
    fresh = path()
    with pytest.raises(ValueError, match="incoming tangent"):
        geometric_preservation_metrics(
            raw_reference=fresh,
            candidate=fresh,
            previous_pose_world=[0, 0, 0],
            boundary_pose_world=[0, 0, 0],
            minimum_translation_m=1e-6,
        )
    with pytest.raises(ValueError, match="shape"):
        geometric_preservation_metrics(
            raw_reference=fresh,
            candidate=fresh[:-1],
            previous_pose_world=[-0.1, 0, 0],
            boundary_pose_world=[0, 0, 0],
            minimum_translation_m=1e-6,
        )

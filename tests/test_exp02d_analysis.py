from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

from reconciliation.exp02d_analysis import (
    EXP02D_INPUT_RECONSTRUCTION_FAILED,
    INPUT_RECONSTRUCTION_FAILED,
    CommandDiscontinuity,
    ReconstructionFailure,
    ReconstructedCorpus,
    RegimeThresholds,
    classify_m3_outcome,
    cluster_bootstrap_difference,
    command_discontinuity,
    load_development_corpus,
    load_exp02d_corpus_index,
    m4_false_correction_rescued,
    ordered_pair_groups,
    pair_balanced_mean,
    reconstruct_transition_input,
    regime_memberships,
    select_representatives,
    transition_weighted_mean,
)
from reconciliation.online_switch import sha256_file


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "configs/exp02d_lookahead_direction.yaml"


def _row(pair: str, value: float, other: float = 0.0) -> dict[str, object]:
    return {
        "ordered_raw_pair_sha256": pair,
        "value": value,
        "other": other,
    }


def test_pair_balanced_macro_mean_does_not_multiply_duplicate_group_weight() -> None:
    rows = [_row("dominant", 0.0) for _ in range(100)] + [_row("rare", 10.0)]
    assert list(ordered_pair_groups(rows)) == ["dominant", "rare"]
    assert pair_balanced_mean(rows, "value") == pytest.approx(5.0)
    assert transition_weighted_mean(rows, "value") == pytest.approx(10.0 / 101.0)
    duplicated = rows + [_row("dominant", 0.0) for _ in range(500)]
    assert pair_balanced_mean(duplicated, "value") == pytest.approx(5.0)


def test_cluster_bootstrap_resamples_pairs_and_is_deterministic() -> None:
    rows = [
        _row("a", 1.0, 0.0),
        _row("a", 3.0, 0.0),
        _row("b", 10.0, 4.0),
    ]
    first = cluster_bootstrap_difference(
        rows, "value", "other", seed=20260910, repetitions=1000
    )
    second = cluster_bootstrap_difference(
        list(reversed(rows)), "value", "other", seed=20260910, repetitions=1000
    )
    assert first == second
    assert first["estimate"] == pytest.approx(4.0)  # group differences: 2 and 6
    assert first["ordered_pair_group_count"] == 2
    assert first["bootstrap_unit"] == "ordered_raw_pair_sha256"
    with pytest.raises(ValueError):
        cluster_bootstrap_difference(rows, "value", "other", seed=0, repetitions=0)


def _command(difficulty: str, score: float) -> CommandDiscontinuity:
    return CommandDiscontinuity(0.0, 0.0, score, difficulty)


@pytest.mark.parametrize(
    "raw,m3,expected",
    [
        (_command("BENIGN", 0.1), _command("BENIGN", 0.2), "BENIGN_PRESERVED"),
        (_command("BENIGN", 0.1), _command("INTERMEDIATE", 0.5), "BENIGN_BROKEN"),
        (
            _command("CHALLENGING", 2.0),
            _command("INTERMEDIATE", 1.0),
            "CHALLENGING_RESCUED",
        ),
        (
            _command("CHALLENGING", 2.0),
            _command("CHALLENGING", 1.6),
            "CHALLENGING_IMPROVED",
        ),
        (
            _command("CHALLENGING", 2.0),
            _command("CHALLENGING", 2.4),
            "CHALLENGING_WORSE",
        ),
        (
            _command("CHALLENGING", 2.0),
            _command("CHALLENGING", 2.1),
            "CHALLENGING_MIXED",
        ),
        (_command("INTERMEDIATE", 0.8), _command("BENIGN", 0.2), None),
    ],
)
def test_predeclared_success_failure_labels(raw, m3, expected) -> None:
    assert classify_m3_outcome(raw, m3) == expected


def test_geometry_and_solver_status_precedence_and_m4_rescue() -> None:
    benign = _command("BENIGN", 0.1)
    challenging = _command("CHALLENGING", 2.0)
    assert classify_m3_outcome(benign, None, geometry_undefined=True) == "GEOMETRY_UNDEFINED"
    assert classify_m3_outcome(benign, None) == "SOLVER_FAILURE"
    assert m4_false_correction_rescued(benign, challenging, benign)
    assert not m4_false_correction_rescued(benign, benign, benign)


def test_command_score_uses_frozen_scales_and_rejects_nonfinite() -> None:
    result = command_discontinuity(0.15, 0.30)
    assert result.j_cmd == pytest.approx(math.sqrt(2.0))
    assert result.difficulty == "CHALLENGING"
    with pytest.raises(ValueError):
        command_discontinuity(float("nan"), 0.0)


def _representative(
    identifier: str,
    *,
    raw_bin: str = "INTERMEDIATE",
    m1_bin: str = "INTERMEDIATE",
    m3_bin: str = "INTERMEDIATE",
    raw: float = 1.0,
    m1: float = 1.0,
    m2: float = 1.0,
    m3: float = 1.0,
    geometry: str = "STRAIGHT_LIKE",
    alpha: float = 1.0,
    gap: float = 0.0,
) -> dict[str, object]:
    return {
        "corpus_transition_id": identifier,
        "raw_difficulty_bin": raw_bin,
        "m1_difficulty_bin": m1_bin,
        "m3_difficulty_bin": m3_bin,
        "j_cmd_raw": raw,
        "j_cmd_m1": m1,
        "j_cmd_m2": m2,
        "j_cmd_m3": m3,
        "fresh_geometry_bin": geometry,
        "alpha_look_rad": alpha,
        "b_to_fresh_k_translation_m": gap,
    }


def test_representative_selection_uses_frozen_rules_and_lexicographic_ties() -> None:
    rows = [
        _representative(
            "z-s1", raw_bin="BENIGN", m1_bin="CHALLENGING", m3_bin="BENIGN", m1=3.0, m3=1.0
        ),
        _representative(
            "a-s1", raw_bin="BENIGN", m1_bin="CHALLENGING", m3_bin="BENIGN", m1=3.0, m3=1.0
        ),
        _representative(
            "s2", raw_bin="CHALLENGING", raw=3.0, m2=2.5, m3=1.0
        ),
        _representative(
            "f1", raw=1.0, m3=1.3, geometry="NEGATIVE_TURNING"
        ),
        _representative(
            "f2", raw=2.0, m3=2.1, alpha=math.radians(15.0), gap=0.25
        ),
    ]
    assert select_representatives(rows) == {
        "S1": "a-s1",
        "S2": "s2",
        "F1": "f1",
        "F2": "f2",
    }


def test_unavailable_representatives_remain_unavailable() -> None:
    result = select_representatives([_representative("ordinary")])
    assert result == {
        "S1": "S1_NOT_AVAILABLE",
        "S2": "S2_NOT_AVAILABLE",
        "F1": "F1_NOT_AVAILABLE",
        "F2": "F2_NOT_AVAILABLE",
    }


def _thresholds() -> RegimeThresholds:
    return RegimeThresholds(
        alpha_entry_large_rad=1.0,
        alpha_look_small_rad=0.25,
        alpha_look_large_rad=0.75,
        positional_gap_large_m=0.2,
        degenerate_lookahead_radius_m=0.05,
        insufficient_lookahead_arc_m=0.05,
        large_deformation_rms_m=0.2,
        small_command_benefit_fraction=0.1,
        rigid_correction_min_entry_m=0.05,
        rigid_fit_translation_rms_m=1e-6,
        rigid_fit_yaw_rms_rad=1e-6,
        rigid_endpoint_entry_difference_m=1e-6,
    )


def _regime_row() -> dict[str, object]:
    return {
        "alpha_entry_rad": 1.1,
        "alpha_look_rad": 0.2,
        "b_to_fresh_k_translation_m": 0.25,
        "d_q_m": 0.04,
        "fresh_arc_k_to_q_m": 0.04,
        "k_fresh": 2,
        "q_fresh": 2,
        "fresh_geometry_bin": "POSITIVE_TURNING",
        "j_cmd_raw": 1.0,
        "j_cmd_m3": 0.95,
        "m3_rms_translation_deformation_m": 0.25,
        "m3_entry_displacement_m": 0.1,
        "m3_endpoint_displacement_m": 0.1,
        "m3_best_fit_rigid_translation_rms_m": 0.0,
        "m3_best_fit_rigid_yaw_rms_rad": 0.0,
    }


def test_regimes_are_multiple_explicit_memberships() -> None:
    assert regime_memberships(_regime_row(), _thresholds()) == (
        "R1_FALSE_ENTRY_DIRECTION_MISMATCH",
        "R3_POSITION_GAP_DIRECTION_CONSISTENT",
        "R4_DEGENERATE_LOOKAHEAD",
        "R5_LARGE_CORRECTION_LITTLE_COMMAND_BENEFIT",
        "R6_RIGID_PROPAGATION",
    )
    turn = {**_regime_row(), "alpha_look_rad": 0.8, "q_fresh": 3, "d_q_m": 0.3, "fresh_arc_k_to_q_m": 0.3}
    assert "R2_INTENT_CONFLICT_CANDIDATE" in regime_memberships(turn, _thresholds())
    nonuniform = {**_regime_row(), "m3_endpoint_displacement_m": 0.2}
    assert "R6_RIGID_PROPAGATION" not in regime_memberships(nonuniform, _thresholds())
    with pytest.raises(ValueError):
        regime_memberships({**_regime_row(), "alpha_entry_rad": float("nan")}, _thresholds())


def test_real_development_corpus_reconstructs_read_only_when_present() -> None:
    if not CONFIG_PATH.is_file():
        pytest.skip("EXP-02D config is unavailable")
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    combined = ROOT / config["paths"]["combined_corpus"]
    if not combined.is_dir():
        pytest.skip("immutable DATA-02 corpus is unavailable")
    corpus = load_development_corpus(
        ROOT,
        config["paths"]["combined_corpus"],
        config["source_integrity"],
        config["oracle"]["first_command_absolute_tolerance"],
    )
    assert corpus.valid_count == corpus.index.eligible_count
    assert not corpus.failures
    assert all(item.difficulty_bin in ("BENIGN", "INTERMEDIATE", "CHALLENGING") for item in corpus.transitions)
    assert all(item.semantic_family and item.physical_region for item in corpus.transitions)
    assert all(0 <= item.k_fresh <= item.q_fresh < len(item.fresh_world) for item in corpus.transitions)
    first = corpus.transitions[0]
    first_record = corpus.index.eligible_records[0]
    assert first.semantic_family == first_record["semantic_family"]
    assert first.physical_region == first_record["physical_region"]
    assert first.semantic_family
    assert first.physical_region
    before = sha256_file(first.source_transition)
    for array in (
        first.old_raw,
        first.old_world,
        first.fresh_raw,
        first.fresh_world,
        first.observation_pose,
        first.pose_before_boundary,
        first.boundary_pose,
        first.old_desired_command,
        first.raw_fresh_first_desired_command,
    ):
        assert not array.flags.writeable
    with pytest.raises(ValueError):
        first.fresh_world[0, 0] = 999.0
    assert sha256_file(first.source_transition) == before


def test_pinned_integrity_hash_rejects_wrong_value_when_corpus_present() -> None:
    if not CONFIG_PATH.is_file():
        pytest.skip("EXP-02D config is unavailable")
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    combined = ROOT / config["paths"]["combined_corpus"]
    if not combined.is_dir():
        pytest.skip("immutable DATA-02 corpus is unavailable")
    bad = {**config["source_integrity"], "combined_manifest_sha256": "0" * 64}
    with pytest.raises(ValueError, match="pinned source hash mismatch"):
        load_exp02d_corpus_index(ROOT, config["paths"]["combined_corpus"], integrity_config=bad)


def test_require_complete_uses_exact_experiment_failure_status_when_present() -> None:
    if not CONFIG_PATH.is_file():
        pytest.skip("EXP-02D config is unavailable")
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    combined = ROOT / config["paths"]["combined_corpus"]
    if not combined.is_dir():
        pytest.skip("immutable DATA-02 corpus is unavailable")
    index = load_exp02d_corpus_index(
        ROOT, config["paths"]["combined_corpus"], integrity_config=config["source_integrity"]
    )
    corpus = ReconstructedCorpus(
        index,
        (),
        (ReconstructionFailure("fixture", INPUT_RECONSTRUCTION_FAILED, "fixture"),),
    )
    with pytest.raises(RuntimeError, match=EXP02D_INPUT_RECONSTRUCTION_FAILED):
        corpus.require_complete()


def test_reconstruct_transition_refuses_noneligible_record_when_present() -> None:
    if not CONFIG_PATH.is_file():
        pytest.skip("EXP-02D config is unavailable")
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    combined = ROOT / config["paths"]["combined_corpus"]
    if not combined.is_dir():
        pytest.skip("immutable DATA-02 corpus is unavailable")
    index = load_exp02d_corpus_index(
        ROOT, config["paths"]["combined_corpus"], integrity_config=config["source_integrity"]
    )
    noneligible = {**index.eligible_records[0], "status": "TIMING_INVALID"}
    with pytest.raises(ValueError, match="only ELIGIBLE_MOVING"):
        reconstruct_transition_input(index, noneligible)

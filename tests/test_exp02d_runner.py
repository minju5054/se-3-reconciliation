from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]


def _load(name: str, relative: str):
    path = ROOT / relative
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RUNNER = _load("exp02d_runner_test", "scripts/run_exp02d_lookahead_direction.py")
PLOTTER = _load("exp02d_plotter_test", "scripts/plot_exp02d_lookahead_direction.py")
VALIDATOR = _load(
    "exp02d_validator_schema_test",
    "scripts/validate_exp02d_lookahead_direction.py",
)
CONFIG_PATH = ROOT / "configs/exp02d_lookahead_direction.yaml"


def config():
    return yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))


def test_protocol_has_no_hard_coded_corpus_counts_and_threshold_mapping_is_valid():
    values = config()
    assert "expected_eligible_moving" not in values["source_integrity"]
    assert "expected_unique_ordered_raw_pairs" not in values["source_integrity"]
    RUNNER.validate_protocol(values)
    thresholds = RUNNER.regime_thresholds(values)
    assert thresholds.alpha_entry_large_rad == pytest.approx(
        values["regimes"]["alpha_entry_large_rad"]
    )
    assert thresholds.alpha_look_large_rad == pytest.approx(
        values["regimes"]["alpha_look_large_rad"]
    )
    assert thresholds.rigid_endpoint_entry_difference_m == pytest.approx(
        values["regimes"]["rigid_endpoint_entry_difference_m"]
    )


def test_all_real_m0_suffix_commands_and_absolute_indices_reconstruct_when_present():
    values = config()
    combined = ROOT / values["paths"]["combined_corpus"]
    if not combined.is_dir():
        pytest.skip("immutable DATA-02 corpus is unavailable")
    corpus = RUNNER.load_exp02d_development_corpus(
        ROOT,
        combined,
        command_atol=values["oracle"]["first_command_absolute_tolerance"],
    )
    summary = RUNNER.validate_m0_follower_reconstruction(corpus, values)
    assert summary["transition_count"] == corpus.index.eligible_count
    assert summary["maximum_full_raw_first_command_absolute_error"] <= 1e-12
    assert summary["maximum_selected_suffix_first_command_absolute_error"] <= 1e-12
    assert summary["all_selected_suffix_absolute_indices_match_raw_k_q"] is True
    assert sum(summary["k_fresh_distribution"].values()) == corpus.index.eligible_count
    assert sum(summary["q_fresh_distribution"].values()) == corpus.index.eligible_count
    assert sum(summary["q_minus_k_distribution"].values()) == corpus.index.eligible_count


def test_undefined_analysis_row_is_csv_safe_and_retains_status_identity():
    source = SimpleNamespace(
        corpus_transition_id="v1:episode_0:transition_0",
        cohort_id="v1",
        episode_id="episode_0",
        transition_index=0,
        template_id="template",
        variant_id="variant",
        semantic_family="family",
        physical_region="region",
        ordered_raw_pair_sha256="a" * 64,
        fresh_geometry_bin="STRAIGHT_LIKE",
        difficulty_bin="BENIGN",
        k_fresh=0,
        q_fresh=2,
        q_relative=2,
        host_latency_s=0.6,
        effective_latency_s=0.7,
        metrics={
            "observation_to_boundary_translation_m": 0.1,
            "observation_to_boundary_yaw_rad": 0.02,
            "observation_to_model_ready_translation_m": 0.08,
            "fresh_geometry": {
                "signed_net_yaw_rad": 0.1,
                "maximum_lateral_excursion_m": 0.2,
            },
        },
    )
    evaluation = {
        "source": source,
        "geometry": {
            "alpha_entry_rad": None,
            "alpha_look_rad": None,
            "alpha_entry_minus_alpha_look_rad": None,
            "boundary_to_entry_gap_m": 0.1,
            "boundary_to_lookahead_gap_m": 0.0,
            "entry_to_lookahead_arc_length_m": 0.2,
            "previous_to_boundary_translation_m": 0.1,
            "previous_to_boundary_yaw_rad": 0.01,
        },
        "undefined": ("LOOKAHEAD_DIRECTION_UNDEFINED",),
        "methods": {},
        "solver_failures": {},
    }
    row = RUNNER.analysis_row(evaluation, pair_frequency=3)
    csv = RUNNER.csv_row(row)
    assert row["success_failure_label"] == "GEOMETRY_UNDEFINED"
    assert row["semantic_family"] == "family"
    assert row["physical_region"] == "region"
    assert csv["host_latency_s"] == pytest.approx(0.6)
    assert csv["M3_LOOKAHEAD_J_cmd"] is None
    assert json.loads(csv["undefined_statuses"]) == [
        "LOOKAHEAD_DIRECTION_UNDEFINED"
    ]
    assert tuple(csv) == VALIDATOR.CSV_FIELDS


def test_empty_required_partition_summary_uses_null_not_nan():
    summary = RUNNER.method_table([], "m3")
    assert summary["count"] == 0
    assert summary["median_J_cmd"] is None
    assert summary["pair_balanced_mean_J_cmd"] is None
    assert summary["pair_balanced_mean_yaw_deformation_rms_rad"] is None
    assert summary["pair_balanced_mean_rigid_fit_yaw_rms_rad"] is None
    assert summary["candidate_target_changed_fraction"] is None


def test_method_summary_reports_pair_and_transition_weighted_yaw_diagnostics():
    rows = [
        {
            "ordered_raw_pair_sha256": pair,
            "j_cmd_m3": score,
            "m3_delta_v_abs_mps": 0.1,
            "m3_delta_omega_abs_rps": 0.2,
            "m3_translation_deformation_rms_m": score / 10.0,
            "m3_yaw_deformation_rms_rad": score / 20.0,
            "m3_rigid_fit_translation_rms_m": score / 100.0,
            "m3_rigid_fit_yaw_rms_rad": score / 200.0,
            "m3_candidate_target_changed_from_raw_q": changed,
            "m3_candidate_nearest_index": 1,
            "m3_candidate_target_index": 3 + int(changed),
        }
        for pair, score, changed in (("a", 1.0, False), ("b", 3.0, True))
    ]
    summary = RUNNER.method_table(rows, "m3")
    assert summary["pair_balanced_mean_yaw_deformation_rms_rad"] == pytest.approx(0.1)
    assert summary["transition_weighted_mean_rigid_fit_yaw_rms_rad"] == pytest.approx(0.01)
    assert summary["candidate_target_changed_pair_balanced_fraction"] == pytest.approx(0.5)


def test_geometry_hypothesis_bootstrap_uses_the_single_frozen_seed():
    rows = [
        {
            "ordered_raw_pair_sha256": pair,
            "alpha_entry_minus_alpha_look_rad": x,
            "j_cmd_m1": y,
            "j_cmd_m3": 0.0,
        }
        for pair, x, y in (
            ("pair-a", 0.0, 0.1),
            ("pair-b", 0.5, 0.4),
            ("pair-c", 1.0, 0.8),
        )
    ]
    values = config()
    result = RUNNER.hypothesis_analysis(rows, values)
    bootstrap = result["pair_cluster_bootstrap_pearson_ci"]
    assert bootstrap["seed"] == values["evaluation"]["bootstrap_seed"]
    assert bootstrap["repetitions"] == values["evaluation"]["bootstrap_repetitions"]


def _plot_record(index: int) -> dict:
    difficulties = ("BENIGN", "CHALLENGING", "INTERMEDIATE", "CHALLENGING")
    labels = (
        "BENIGN_PRESERVED",
        "CHALLENGING_IMPROVED",
        "RAW_INTERMEDIATE_UNLABELED",
        "CHALLENGING_WORSE",
    )
    geometries = (
        "STRAIGHT_LIKE",
        "POSITIVE_TURNING",
        "OTHER",
        "NEGATIVE_TURNING",
    )
    methods = {}
    for offset, method in enumerate(RUNNER.METHODS):
        methods[method] = {
            "command": {
                "J_cmd": 0.2 + 0.1 * offset + 0.05 * index,
                "difficulty": difficulties[index],
            },
            "deformation": {
                "translation_deformation_rms_m": 0.01 * offset,
                "rigid_fit_translation_rms_m": 1e-8 * (offset + 1),
            },
        }
    return {
        "corpus_transition_id": f"fixture-{index}",
        "raw_difficulty": difficulties[index],
        "fresh_geometry": geometries[index],
        "effective_latency_s": (0.5, 0.7, 0.9, 0.65)[index],
        "ordered_raw_pair_frequency": (1, 2, 10, 3)[index],
        "success_failure_label": labels[index],
        "geometry": {
            "alpha_entry_rad": 0.1 + index * 0.2,
            "alpha_look_rad": 0.05 + index * 0.1,
        },
        "methods": methods,
    }


def test_plotter_generates_exact_required_ten_plots(tmp_path):
    run = tmp_path / "run"
    (run / "summary").mkdir(parents=True)
    values = config()
    (run / "config_snapshot.yaml").write_text(
        yaml.safe_dump(values, sort_keys=False), encoding="utf-8"
    )
    (run / "summary/plot_records.json").write_text(
        json.dumps({"records": [_plot_record(index) for index in range(4)]}),
        encoding="utf-8",
    )
    (run / "summary/pair_balanced.json").write_text(
        json.dumps(
            {
                "methods": {
                    method: {"mean_J_cmd": 0.2 + 0.1 * index}
                    for index, method in enumerate(RUNNER.METHODS)
                }
            }
        ),
        encoding="utf-8",
    )
    paths = PLOTTER.generate(run)
    assert len(paths) == 10
    assert {path.name for path in paths} == {
        f"{index:02d}_{name}.png"
        for index, name in enumerate(
            (
                "method_command_score_pair_balanced",
                "benign_preservation",
                "challenging_outcomes",
                "alpha_entry_vs_alpha_look",
                "geometry_semantics_gain",
                "deformation_vs_benefit",
                "rigid_fit_distribution",
                "success_failure_by_fresh_geometry",
                "success_failure_by_latency",
                "pair_frequency_vs_method_effect",
            ),
            start=1,
        )
    }

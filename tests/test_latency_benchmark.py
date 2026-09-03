import json
from pathlib import Path

import numpy as np
import pytest

from reconciliation.latency_benchmark import (
    build_summary,
    latency_statistics,
    make_trial_record,
    save_json_exclusive,
    save_trials_csv_exclusive,
    stage0c_comparison,
    validate_action_array,
    validate_benchmark_output,
    validate_trials,
)


def action_chunk(offset: float = 0.0, horizon: int = 10) -> np.ndarray:
    values = np.column_stack(
        (
            np.linspace(0.1, 0.7, horizon) + offset,
            np.zeros(horizon),
            np.linspace(0.0, 0.1, horizon),
        )
    )
    return values.astype(np.float32)


def trial(index: int, latency: float, horizon: int = 10) -> dict:
    return make_trial_record(
        trial_index=index,
        trial_kind="first" if index == 0 else "warm",
        reset_ms=1.0,
        observe_history_ms=2.0,
        predict_host_latency_ms=latency,
        lightnav_reported_latency_ms=latency - 0.5,
        actions=action_chunk(index / 100.0, horizon),
        expected_horizon=horizon,
        raw_text=f"<act_{index}>",
        input_source_run="fixture-stage0c",
    )


def summary(trials: list[dict]) -> dict:
    return build_summary(
        model_load_ms=1000.0,
        total_benchmark_wall_ms=9000.0,
        trials=trials,
        expected_warm_trials=len(trials) - 1,
        expected_horizon=10,
        stage0c_execution_duration_s=2.0,
        environment={
            "backend": "vllm_local",
            "gpu": "fixture-gpu",
            "lightnav_sha": "abc",
            "lightnav_package_version": "0.1.0",
            "checkpoint_revision": "def",
        },
        caching_settings={"vllm_prefix_caching": True},
    )


def test_latency_statistics_and_linear_percentiles() -> None:
    result = latency_statistics([10.0, 20.0, 30.0, 40.0])
    assert result == {
        "count": 4,
        "mean_ms": 25.0,
        "median_ms": 25.0,
        "std_ms": pytest.approx(11.180339887498949),
        "min_ms": 10.0,
        "max_ms": 40.0,
        "p90_ms": pytest.approx(37.0),
        "p95_ms": pytest.approx(38.5),
    }


def test_first_is_excluded_from_warm_statistics_and_ratio() -> None:
    trials = [trial(0, 50.0), trial(1, 10.0), trial(2, 20.0)]
    result = summary(trials)
    assert result["first"]["host_latency_ms"] == 50.0
    assert result["warm"]["count"] == 2
    assert result["warm"]["median_ms"] == 15.0
    assert result["first_to_warm_median_ratio"] == pytest.approx(50.0 / 15.0)


@pytest.mark.parametrize("bad", [-1.0, np.nan, np.inf])
def test_invalid_latency_is_rejected(bad: float) -> None:
    with pytest.raises(ValueError, match="finite and nonnegative"):
        make_trial_record(
            trial_index=0,
            trial_kind="first",
            reset_ms=0.0,
            observe_history_ms=0.0,
            predict_host_latency_ms=bad,
            lightnav_reported_latency_ms=1.0,
            actions=action_chunk(),
            expected_horizon=10,
            raw_text="x",
            input_source_run="fixture",
        )


def test_empty_warm_trials_and_wrong_trial_count_are_rejected() -> None:
    with pytest.raises(ValueError, match="positive integer"):
        validate_trials([trial(0, 5.0)], expected_warm_trials=0, expected_horizon=10)
    with pytest.raises(ValueError, match="expected 3 trials"):
        validate_trials([trial(0, 5.0)], expected_warm_trials=2, expected_horizon=10)


def test_action_shape_dtype_and_finite_contract() -> None:
    validated = validate_action_array(action_chunk(horizon=27), expected_horizon=27)
    assert validated.shape == (27, 3)
    assert validated.dtype == np.float32
    with pytest.raises(ValueError, match="shape"):
        validate_action_array(np.zeros((10, 2), dtype=np.float32), expected_horizon=10)
    with pytest.raises(ValueError, match="floating"):
        validate_action_array(np.zeros((10, 3), dtype=np.int64), expected_horizon=10)
    invalid = action_chunk()
    invalid[3, 1] = np.nan
    with pytest.raises(ValueError, match="NaN or Inf"):
        validate_action_array(invalid, expected_horizon=10)


def test_stage0c_comparison_is_dimensionally_correct_and_descriptive() -> None:
    result = stage0c_comparison(4600.0, 2.3)
    assert result["warm_median_to_execution_ratio"] == pytest.approx(2.0)
    assert "not a LightNav horizon" in result["note"]
    with pytest.raises(ValueError, match="positive"):
        stage0c_comparison(10.0, 0.0)


def build_output(root: Path, trials: list[dict]) -> None:
    (root / "actions").mkdir(parents=True)
    (root / "raw_text").mkdir()
    for record in trials:
        index = record["trial_index"]
        with (root / "actions" / f"trial_{index:03d}.npy").open("xb") as stream:
            np.save(stream, action_chunk(index / 100.0), allow_pickle=False)
        (root / "raw_text" / f"trial_{index:03d}.txt").write_text(
            f"<act_{index}>", encoding="utf-8"
        )
    metadata = {
        "warm_trials": len(trials) - 1,
        "expected_horizon": 10,
        "intrinsic_waypoint_time_base": False,
        "model_load_ms": 1000.0,
        "total_benchmark_wall_ms": 9000.0,
        "stage0c_execution_duration_s": 2.0,
        "environment": {
            "backend": "vllm_local",
            "gpu": "fixture-gpu",
            "lightnav_sha": "abc",
            "lightnav_package_version": "0.1.0",
            "checkpoint_revision": "def",
        },
        "caching_settings": {"vllm_prefix_caching": True},
    }
    save_trials_csv_exclusive(root / "trials.csv", trials)
    save_json_exclusive(root / "trials.json", {"schema_version": 1, "trials": trials})
    save_json_exclusive(root / "metadata.json", metadata)
    save_json_exclusive(root / "summary.json", summary(trials))


def test_saved_output_is_strict_valid_and_immutable(tmp_path: Path) -> None:
    trials = [trial(0, 50.0), trial(1, 10.0), trial(2, 12.0)]
    build_output(tmp_path, trials)
    result = validate_benchmark_output(tmp_path)
    assert result["valid"] is True
    assert result["trial_count"] == 3
    assert result["all_actions_finite"] is True
    with pytest.raises(FileExistsError):
        save_json_exclusive(tmp_path / "summary.json", {})


def test_strict_json_rejects_nan(tmp_path: Path) -> None:
    trials = [trial(0, 50.0), trial(1, 10.0)]
    build_output(tmp_path, trials)
    (tmp_path / "summary.json").write_text('{"bad": NaN}', encoding="utf-8")
    with pytest.raises(ValueError, match="non-standard JSON"):
        validate_benchmark_output(tmp_path)


def test_validator_detects_action_metadata_mismatch(tmp_path: Path) -> None:
    trials = [trial(0, 50.0), trial(1, 10.0)]
    build_output(tmp_path, trials)
    actions_path = tmp_path / "actions/trial_001.npy"
    with actions_path.open("wb") as stream:
        np.save(stream, action_chunk(5.0), allow_pickle=False)
    with pytest.raises(ValueError, match="endpoints"):
        validate_benchmark_output(tmp_path)


def test_no_model_intrinsic_waypoint_time_base_is_allowed(tmp_path: Path) -> None:
    trials = [trial(0, 50.0), trial(1, 10.0)]
    build_output(tmp_path, trials)
    metadata_path = tmp_path / "metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata["waypoint_dt_s"] = 0.25
    metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
    with pytest.raises(ValueError, match="forbidden timing"):
        validate_benchmark_output(tmp_path)

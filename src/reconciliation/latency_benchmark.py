"""Pure validation and statistics for EXP-01A LightNav latency benchmarks."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike


PRIMARY_TRIAL_KINDS = ("first", "warm")


def _nonnegative_finite(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be numeric") from error
    if not math.isfinite(result) or result < 0.0:
        raise ValueError(f"{name} must be finite and nonnegative")
    return result


def validate_action_array(actions: ArrayLike, *, expected_horizon: int) -> np.ndarray:
    """Validate one decoded LightNav action result without changing its values/dtype."""

    if not isinstance(expected_horizon, int) or expected_horizon < 1:
        raise ValueError("expected_horizon must be a positive integer")
    result = np.asarray(actions)
    if result.shape != (expected_horizon, 3):
        raise ValueError(
            f"LightNav actions must have shape ({expected_horizon}, 3), got {result.shape}"
        )
    if not np.issubdtype(result.dtype, np.floating):
        raise ValueError("LightNav actions must use a floating-point dtype")
    if not np.all(np.isfinite(result)):
        raise ValueError("LightNav actions contain NaN or Inf")
    return result.copy()


def make_trial_record(
    *,
    trial_index: int,
    trial_kind: str,
    reset_ms: float,
    observe_history_ms: float,
    predict_host_latency_ms: float,
    lightnav_reported_latency_ms: float,
    actions: ArrayLike,
    expected_horizon: int,
    raw_text: str,
    input_source_run: str,
    internal_timings_ms: Mapping[str, Any] | None = None,
    vit_cache_entries_after_predict: int | None = None,
) -> dict[str, Any]:
    """Build a strict, self-validating trial record."""

    if not isinstance(trial_index, int) or trial_index < 0:
        raise ValueError("trial_index must be a nonnegative integer")
    if trial_kind not in PRIMARY_TRIAL_KINDS:
        raise ValueError(f"unsupported trial_kind: {trial_kind!r}")
    if not isinstance(raw_text, str):
        raise ValueError("raw_text must be a string")
    if not input_source_run:
        raise ValueError("input_source_run must not be empty")
    action_array = validate_action_array(actions, expected_horizon=expected_horizon)
    internal = {
        str(key): _nonnegative_finite(value, f"internal_timings_ms.{key}")
        for key, value in (internal_timings_ms or {}).items()
    }
    if vit_cache_entries_after_predict is not None:
        if not isinstance(vit_cache_entries_after_predict, int) or vit_cache_entries_after_predict < 0:
            raise ValueError("vit_cache_entries_after_predict must be a nonnegative integer")
    return {
        "trial_index": trial_index,
        "trial_kind": trial_kind,
        "reset_ms": _nonnegative_finite(reset_ms, "reset_ms"),
        "observe_history_ms": _nonnegative_finite(
            observe_history_ms, "observe_history_ms"
        ),
        "predict_host_latency_ms": _nonnegative_finite(
            predict_host_latency_ms, "predict_host_latency_ms"
        ),
        "lightnav_reported_latency_ms": _nonnegative_finite(
            lightnav_reported_latency_ms, "lightnav_reported_latency_ms"
        ),
        "action_horizon": int(action_array.shape[0]),
        "action_shape": list(action_array.shape),
        "action_dtype": str(action_array.dtype),
        "action_finite": True,
        "first_x": float(action_array[0, 0]),
        "first_y": float(action_array[0, 1]),
        "first_yaw": float(action_array[0, 2]),
        "last_x": float(action_array[-1, 0]),
        "last_y": float(action_array[-1, 1]),
        "last_yaw": float(action_array[-1, 2]),
        "raw_text_length": len(raw_text),
        "input_source_run": input_source_run,
        "internal_timings_ms": internal,
        "vit_cache_entries_after_predict": vit_cache_entries_after_predict,
    }


def validate_trials(
    trials: Sequence[Mapping[str, Any]],
    *,
    expected_warm_trials: int,
    expected_horizon: int,
) -> list[dict[str, Any]]:
    """Validate trial order, first/warm separation, latency, and action metadata."""

    if not isinstance(expected_warm_trials, int) or expected_warm_trials < 1:
        raise ValueError("expected_warm_trials must be a positive integer")
    if len(trials) != expected_warm_trials + 1:
        raise ValueError(
            f"expected {expected_warm_trials + 1} trials, found {len(trials)}"
        )
    output: list[dict[str, Any]] = []
    for index, source in enumerate(trials):
        record = dict(source)
        if record.get("trial_index") != index:
            raise ValueError("trial indices must be contiguous from zero")
        expected_kind = "first" if index == 0 else "warm"
        if record.get("trial_kind") != expected_kind:
            raise ValueError(f"trial {index} must have kind {expected_kind!r}")
        for key in (
            "reset_ms",
            "observe_history_ms",
            "predict_host_latency_ms",
            "lightnav_reported_latency_ms",
        ):
            record[key] = _nonnegative_finite(record.get(key), f"trial {index} {key}")
        if record.get("action_shape") != [expected_horizon, 3]:
            raise ValueError(f"trial {index} action_shape does not match checkpoint contract")
        if record.get("action_horizon") != expected_horizon:
            raise ValueError(f"trial {index} action_horizon does not match checkpoint contract")
        if record.get("action_finite") is not True:
            raise ValueError(f"trial {index} action finite validation is missing")
        for key in ("first_x", "first_y", "first_yaw", "last_x", "last_y", "last_yaw"):
            value = float(record.get(key))
            if not math.isfinite(value):
                raise ValueError(f"trial {index} {key} must be finite")
        output.append(record)
    return output


def latency_statistics(values_ms: Sequence[float]) -> dict[str, float | int]:
    """Return population statistics; percentiles use NumPy's linear estimator."""

    values = np.asarray(values_ms, dtype=np.float64)
    if values.ndim != 1 or values.size < 1:
        raise ValueError("latency statistics require at least one value")
    if not np.all(np.isfinite(values)) or np.any(values < 0.0):
        raise ValueError("latencies must be finite and nonnegative")
    return {
        "count": int(values.size),
        "mean_ms": float(np.mean(values)),
        "median_ms": float(np.median(values)),
        "std_ms": float(np.std(values, ddof=0)),
        "min_ms": float(np.min(values)),
        "max_ms": float(np.max(values)),
        "p90_ms": float(np.percentile(values, 90, method="linear")),
        "p95_ms": float(np.percentile(values, 95, method="linear")),
    }


def stage0c_comparison(
    warm_median_ms: float, execution_duration_s: float
) -> dict[str, Any]:
    """Contextual comparison only; this does not create a model waypoint time base."""

    median = _nonnegative_finite(warm_median_ms, "warm_median_ms")
    duration = _nonnegative_finite(execution_duration_s, "execution_duration_s")
    if duration == 0.0:
        raise ValueError("execution_duration_s must be positive")
    return {
        "execution_duration_s": duration,
        "note": (
            "One validated 0.72 m Stage 0-C controller run; not a LightNav horizon "
            "duration because decoded waypoints have no intrinsic time base."
        ),
        "warm_median_to_execution_ratio": median / (duration * 1000.0),
    }


def build_summary(
    *,
    model_load_ms: float,
    total_benchmark_wall_ms: float,
    trials: Sequence[Mapping[str, Any]],
    expected_warm_trials: int,
    expected_horizon: int,
    stage0c_execution_duration_s: float,
    environment: Mapping[str, Any],
    caching_settings: Mapping[str, Any],
) -> dict[str, Any]:
    """Build the strict EXP-01A summary with first and warm results separated."""

    validated = validate_trials(
        trials,
        expected_warm_trials=expected_warm_trials,
        expected_horizon=expected_horizon,
    )
    first = validated[0]
    warm = validated[1:]
    warm_host = latency_statistics([row["predict_host_latency_ms"] for row in warm])
    warm_reported = latency_statistics(
        [row["lightnav_reported_latency_ms"] for row in warm]
    )
    warm_median = float(warm_host["median_ms"])
    if warm_median == 0.0:
        raise ValueError("warm median latency must be positive")
    comparison = stage0c_comparison(warm_median, stage0c_execution_duration_s)
    return {
        "schema_version": 1,
        "model_load_ms": _nonnegative_finite(model_load_ms, "model_load_ms"),
        "total_benchmark_wall_ms": _nonnegative_finite(
            total_benchmark_wall_ms, "total_benchmark_wall_ms"
        ),
        "first": {
            "host_latency_ms": first["predict_host_latency_ms"],
            "reported_latency_ms": first["lightnav_reported_latency_ms"],
        },
        "warm": warm_host,
        "warm_reported": warm_reported,
        "warm_host_latency_raw_ms": [row["predict_host_latency_ms"] for row in warm],
        "warm_reported_latency_raw_ms": [
            row["lightnav_reported_latency_ms"] for row in warm
        ],
        "first_to_warm_median_ratio": first["predict_host_latency_ms"] / warm_median,
        "stage0c_reference": {
            "execution_duration_s": comparison["execution_duration_s"],
            "note": comparison["note"],
        },
        "warm_median_to_stage0c_execution_ratio": comparison[
            "warm_median_to_execution_ratio"
        ],
        "backend": environment.get("backend"),
        "gpu": environment.get("gpu"),
        "lightnav_sha": environment.get("lightnav_sha"),
        "lightnav_package_version": environment.get("lightnav_package_version"),
        "checkpoint_revision": environment.get("checkpoint_revision"),
        "caching_settings": dict(caching_settings),
    }


def load_strict_json(path: str | Path) -> Any:
    with Path(path).open("r", encoding="utf-8") as stream:
        return json.load(
            stream,
            parse_constant=lambda token: (_ for _ in ()).throw(
                ValueError(f"non-standard JSON constant: {token}")
            ),
        )


def save_json_exclusive(path: str | Path, value: Any) -> Path:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    return destination


def save_trials_csv_exclusive(path: str | Path, trials: Sequence[Mapping[str, Any]]) -> Path:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = (
        "trial_index",
        "trial_kind",
        "reset_ms",
        "observe_history_ms",
        "predict_host_latency_ms",
        "lightnav_reported_latency_ms",
        "action_horizon",
        "action_dtype",
        "first_x",
        "first_y",
        "first_yaw",
        "last_x",
        "last_y",
        "last_yaw",
    )
    with destination.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(trials)
    return destination


def validate_action_artifact(
    actions: ArrayLike,
    record: Mapping[str, Any],
    *,
    expected_horizon: int,
) -> None:
    array = validate_action_array(actions, expected_horizon=expected_horizon)
    if record.get("action_shape") != list(array.shape):
        raise ValueError("action artifact shape differs from trial metadata")
    if record.get("action_dtype") != str(array.dtype):
        raise ValueError("action artifact dtype differs from trial metadata")
    expected_values = np.asarray(
        [
            [record["first_x"], record["first_y"], record["first_yaw"]],
            [record["last_x"], record["last_y"], record["last_yaw"]],
        ]
    )
    if not np.array_equal(array[[0, -1]], expected_values):
        raise ValueError("action artifact endpoints differ from trial metadata")


def validate_benchmark_output(path: str | Path) -> dict[str, Any]:
    """Validate strict JSON, trial contract, per-trial action artifacts, and summary."""

    root = Path(path)
    metadata = load_strict_json(root / "metadata.json")
    trial_document = load_strict_json(root / "trials.json")
    summary = load_strict_json(root / "summary.json")
    if not isinstance(metadata, dict) or not isinstance(summary, dict):
        raise ValueError("metadata and summary must contain JSON objects")
    if not isinstance(trial_document, dict) or not isinstance(trial_document.get("trials"), list):
        raise ValueError("trials.json must contain a trials array")
    if metadata.get("intrinsic_waypoint_time_base") is not False:
        raise ValueError("benchmark metadata must not fabricate a waypoint time base")
    forbidden = {"waypoint_dt", "waypoint_dt_s", "ready_time", "ready_time_s"}
    present = sorted(forbidden.intersection(metadata))
    if present:
        raise ValueError(f"benchmark metadata contains forbidden timing fields: {present}")
    expected_warm = int(metadata["warm_trials"])
    expected_horizon = int(metadata["expected_horizon"])
    trials = validate_trials(
        trial_document["trials"],
        expected_warm_trials=expected_warm,
        expected_horizon=expected_horizon,
    )
    for record in trials:
        index = int(record["trial_index"])
        action_path = root / "actions" / f"trial_{index:03d}.npy"
        text_path = root / "raw_text" / f"trial_{index:03d}.txt"
        if not action_path.is_file() or not text_path.is_file():
            raise ValueError(f"trial {index} artifacts are missing")
        validate_action_artifact(
            np.load(action_path, allow_pickle=False),
            record,
            expected_horizon=expected_horizon,
        )
        if len(text_path.read_text(encoding="utf-8")) != int(record["raw_text_length"]):
            raise ValueError(f"trial {index} raw text length differs from metadata")
    rebuilt = build_summary(
        model_load_ms=float(metadata["model_load_ms"]),
        total_benchmark_wall_ms=float(metadata["total_benchmark_wall_ms"]),
        trials=trials,
        expected_warm_trials=expected_warm,
        expected_horizon=expected_horizon,
        stage0c_execution_duration_s=float(metadata["stage0c_execution_duration_s"]),
        environment=metadata["environment"],
        caching_settings=metadata["caching_settings"],
    )
    if rebuilt != summary:
        raise ValueError("summary.json does not match the validated trial data")
    with (root / "trials.csv").open("r", newline="", encoding="utf-8") as stream:
        csv_rows = list(csv.DictReader(stream))
    if len(csv_rows) != len(trials):
        raise ValueError("trials.csv row count differs from trials.json")
    return {
        "valid": True,
        "trial_count": len(trials),
        "warm_trial_count": expected_warm,
        "action_shapes": [record["action_shape"] for record in trials],
        "all_actions_finite": all(record["action_finite"] is True for record in trials),
        "summary": summary,
    }

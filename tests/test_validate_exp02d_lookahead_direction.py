from __future__ import annotations

import ast
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/validate_exp02d_lookahead_direction.py"
SPEC = importlib.util.spec_from_file_location("validate_exp02d", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
validator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(validator)


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _manifest(run: Path, paths: list[Path]) -> None:
    (run / "result_manifest.json").write_text(
        json.dumps(
            {
                "schema": "EXP02D_ResultManifest_v1",
                "file_count_excluding_manifest": len(paths),
                "artifact_sha256": {
                    str(path.relative_to(run)): _digest(path) for path in paths
                },
            }
        ),
        encoding="utf-8",
    )


def test_result_manifest_requires_exact_immutable_canonical_inventory(tmp_path: Path) -> None:
    run = tmp_path / "run"
    run.mkdir()
    artifact = run / "artifact.txt"
    artifact.write_text("frozen", encoding="utf-8")
    _manifest(run, [artifact])
    assert validator.validate_result_manifest(run) == {
        "artifact.txt": _digest(artifact)
    }

    artifact.write_text("changed", encoding="utf-8")
    with pytest.raises(ValueError, match="hash mismatch"):
        validator.validate_result_manifest(run)


def test_result_manifest_rejects_unmanifested_canonical_file_but_allows_gui(
    tmp_path: Path,
) -> None:
    run = tmp_path / "run"
    run.mkdir()
    artifact = run / "artifact.txt"
    artifact.write_text("frozen", encoding="utf-8")
    _manifest(run, [artifact])
    gui = run / "gui"
    gui.mkdir()
    (gui / "later_capture.png").write_bytes(b"capture")
    validator.validate_result_manifest(run)

    (run / "unmanifested.txt").write_text("unexpected", encoding="utf-8")
    with pytest.raises(ValueError, match="inventory mismatch"):
        validator.validate_result_manifest(run)


def _source(identifier: str, pair: str) -> SimpleNamespace:
    return SimpleNamespace(
        corpus_transition_id=identifier,
        cohort_id="v1",
        episode_id="episode_0",
        transition_index=0,
        template_id="H00",
        variant_id="V0",
        semantic_family="straight",
        physical_region="hall",
        ordered_raw_pair_sha256=pair,
        fresh_geometry_bin="STRAIGHT_LIKE",
        difficulty_bin="INTERMEDIATE",
        k_fresh=0,
        q_fresh=2,
        q_relative=2,
    )


def _invalid_row(source: SimpleNamespace, *, geometry: bool, solver: bool) -> dict[str, str]:
    return {
        "corpus_transition_id": source.corpus_transition_id,
        "cohort_id": source.cohort_id,
        "episode_id": source.episode_id,
        "transition_index": str(source.transition_index),
        "template_id": source.template_id,
        "variant_id": source.variant_id,
        "semantic_family": source.semantic_family,
        "physical_region": source.physical_region,
        "ordered_raw_pair_sha256": source.ordered_raw_pair_sha256,
        "ordered_raw_pair_frequency": "1",
        "fresh_geometry_bin": source.fresh_geometry_bin,
        "raw_difficulty_bin": source.difficulty_bin,
        "k_fresh": str(source.k_fresh),
        "q_fresh": str(source.q_fresh),
        "q_minus_k": str(source.q_relative),
        "geometry_undefined": str(geometry),
        "solver_failure": str(solver),
        "success_failure_label": (
            "GEOMETRY_UNDEFINED" if geometry else "SOLVER_FAILURE"
        ),
        "m4_false_correction_rescued": "False",
        "regimes": "",
    }


def test_status_validation_allows_reported_geometry_solver_overlap() -> None:
    first = _source("first", "pair-a")
    second = _source("second", "pair-b")
    rows = [
        _invalid_row(first, geometry=True, solver=True),
        _invalid_row(second, geometry=False, solver=True),
    ]
    corpus = SimpleNamespace(transitions=(first, second))
    manifest = {
        "undefined_geometry": {"count": 1, "transitions": {"first": ["status"]}},
        "solver_failures": {
            "count": 2,
            "transitions": {"first": {}, "second": {"M3": "failed"}},
        },
    }
    undefined, solver = validator._validate_csv_rows(rows, corpus, manifest, {})
    assert undefined == {"first"}
    assert solver == {"first", "second"}


def test_validator_is_read_only_and_does_not_hardcode_corpus_count() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    assert "959" not in source
    tree = ast.parse(source)
    forbidden_calls = {"write_text", "write_bytes", "mkdir", "unlink"}
    assert not [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr in forbidden_calls
    ]
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "open"
            and node.args
            and isinstance(node.args[0], ast.Constant)
        ):
            assert all(character not in str(node.args[0].value) for character in "wax+")


def test_plot_validator_requires_exact_frozen_ten_pngs(tmp_path: Path) -> None:
    plots = tmp_path / "plots"
    plots.mkdir()
    png = b"\x89PNG\r\n\x1a\n" + b"x" * 100
    for name in validator.PLOT_NAMES:
        (plots / name).write_bytes(png)
    validator._validate_plots(tmp_path)
    (plots / "11_unplanned.png").write_bytes(png)
    with pytest.raises(ValueError, match="exactly ten"):
        validator._validate_plots(tmp_path)


def test_recursive_json_comparison_rejects_nonfinite() -> None:
    with pytest.raises(ValueError, match="finite"):
        validator._same_json_value({"value": float("nan")}, {"value": 1.0}, "root")

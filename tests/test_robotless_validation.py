"""Offline audit fail-closed tests; these fixtures are not runtime evidence."""

import importlib.util
from pathlib import Path

import pytest

from reconciliation.robotless_single_chunk import save_json_exclusive, sha256_file


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/validate_robotless_single_chunk.py"
SPEC = importlib.util.spec_from_file_location("validate_robotless_single_chunk", SCRIPT)
validator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(validator)


def test_missing_runtime_artifacts_cannot_pass(tmp_path):
    result = validator.validate_run(tmp_path)
    assert result["status"] == validator.NOT_VALIDATED
    assert "raw/observation.jpg" in result["missing_artifacts"]
    assert "visual_review.json" in result["missing_artifacts"]


def test_explicit_failed_capture_dominates_missing_later_artifacts(tmp_path):
    save_json_exclusive(tmp_path / "capture_validation.json", {"scene_loaded": False})
    result = validator.validate_run(tmp_path)
    assert result["status"] == validator.FAILED
    assert any("scene_loaded" in item for item in result["failures"])


def test_corrupt_available_artifact_is_failed_even_when_incomplete(tmp_path):
    (tmp_path / "capture_validation.json").write_text("not JSON")
    assert validator.validate_run(tmp_path)["status"] == validator.FAILED


def test_failed_visual_review_dominates_other_missing_artifacts(tmp_path):
    save_json_exclusive(tmp_path / "visual_review.json", {
        "reviewed": True, "checks": {"waypoint_headings": False},
    })
    result = validator.validate_run(tmp_path)
    assert result["status"] == validator.FAILED
    assert any("waypoint_headings" in item for item in result["failures"])


def test_fabricated_all_present_files_cannot_trigger_pass(tmp_path):
    for name in validator.REQUIRED_ARTIFACTS:
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("{}")
    assert validator.validate_run(tmp_path)["status"] == validator.FAILED


def test_raw_mutation_detected_against_recorded_hash(tmp_path):
    raw = tmp_path / "raw/response.json"
    raw.parent.mkdir()
    raw.write_bytes(b'{"untouched":true}')
    record = {"path": "raw/response.json", "sha256": sha256_file(raw)}
    validator.check_hash(tmp_path, record)
    raw.write_bytes(b'{"untouched":false}')
    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        validator.check_hash(tmp_path, record)


def test_artifact_path_cannot_escape_run_directory(tmp_path):
    with pytest.raises(ValueError, match="escapes"):
        validator.artifact(tmp_path, "../another-run/raw.json")


def test_visual_review_requires_every_explicit_check():
    fields = validator.VISUAL_REVIEW_FLAGS
    checks = dict.fromkeys(fields, True)
    validator.true_flags(checks, fields, "visual_review")
    checks["waypoint_headings"] = False
    with pytest.raises(ValueError, match="waypoint_headings"):
        validator.true_flags(checks, fields, "visual_review")

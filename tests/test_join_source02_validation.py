"""Synthetic saved artifact validator fixtures; never runtime evidence."""
from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image
import pytest

ROOT = Path(__file__).resolve().parents[1]


def module(name, script):
    spec = importlib.util.spec_from_file_location(name, ROOT/"scripts"/script)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


validator = module("join_source02_validation_fixture", "validate_join_source02.py")
reporter = module("join_source02_validation_report_fixture", "report_join_source02.py")


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


@pytest.fixture
def unavailable_run(tmp_path):
    run = tmp_path/"synthetic_artifact_fixture"
    ids = [f"location__placement__H{n}" for n in (16, 32, 64)]
    protocol = dict(development=dict(condition_order=[dict(condition_id=i) for i in ids],
                    maximum_conditions=12, maximum_terminal_predictions=36))
    write(run/"protocol.json", protocol)
    (run/"config_snapshot.yaml").write_text("synthetic_fixture: true\n")
    write(run/"development_freeze.json", dict(source_sha256={}, environment_source_sha256={},
            protocol_sha256=validator.digest(run/"protocol.json"),
            config_snapshot_sha256=validator.digest(run/"config_snapshot.yaml")))
    records = []
    for cid in ids:
        manifest_path = run/"development/input_manifests"/(cid+".json")
        write(manifest_path, dict(condition_id=cid, availability="HISTORY_UNAVAILABLE"))
        row = dict(condition_id=cid, availability="HISTORY_UNAVAILABLE", label="HISTORY_UNAVAILABLE",
                   qualified=False, completed_branches=[], call_counts=dict(terminal_predictions=0, buffer_only=0),
                   input_manifest_sha256=validator.digest(manifest_path))
        write(run/"development"/cid/"evaluation.json", row)
        records.append(row)
    write(run/"aggregate/development_ledger.json", dict(records=records, selected_condition_id=None,
           actual_terminal_prediction_requests=0, actual_buffer_only_requests=0,
           status="NO_QUALIFYING_DEVELOPMENT_SCENARIO"))
    write(run/"aggregate/confirmation_ledger.json", dict(attempted=0, qualification_rate=None,
            records=[dict(episode_id=f"episode_{i}", attempted=False, qualified=None) for i in range(3)]))
    reporter.report(run, run/"review", run/"review_bundle.zip")
    return run


def forbid_runtime(monkeypatch):
    def no(*args, **kwargs):
        raise AssertionError("model/MPC/simulator process prohibited in saved validation")
    monkeypatch.setattr(subprocess, "run", no)
    monkeypatch.setattr(subprocess, "Popen", no)


def test_all_unavailable_is_valid_evidence_not_scientific_success(unavailable_run, monkeypatch):
    forbid_runtime(monkeypatch)
    result = validator.validate(unavailable_run)
    assert result["valid"] and result["complete_declared_coverage"]
    assert result["recomputed_terminal_predictions"] == 0
    assert all(not r["qualified"] for r in result["source_outcomes"])
    assert result["new_model_inference"] == result["new_MPC_solve"] == result["new_rollout"] == 0
    assert result["checks"]["confirmation_not_attempted_is_NA"]
    with pytest.raises(FileExistsError):
        validator.validate(unavailable_run)


def test_changed_frozen_source_is_not_relaxed(unavailable_run):
    p = unavailable_run/"protocol.json"
    p.write_text(p.read_text()+" ")
    result = validator.validate(unavailable_run)
    assert not result["valid"]
    assert any("frozen_config" in reason for reason in result["failures"])


def test_frozen_string_condition_ids_are_supported(unavailable_run):
    p = unavailable_run/"protocol.json"
    protocol = json.loads(p.read_text())
    protocol["development"]["condition_order"] = [r["condition_id"] for r in protocol["development"]["condition_order"]]
    write(p, protocol)
    freeze_path = unavailable_run/"development_freeze.json"
    freeze = json.loads(freeze_path.read_text())
    freeze["protocol_sha256"] = validator.digest(p)
    write(freeze_path, freeze)
    reporter.report(unavailable_run, unavailable_run/"review_strings", unavailable_run/"review_strings.zip")
    result = validator.validate(unavailable_run, review="review_strings")
    assert result["valid"]


def test_unavailable_cannot_acquire_fake_success(unavailable_run):
    p = unavailable_run/"aggregate/development_ledger.json"
    ledger = json.loads(p.read_text())
    ledger["records"][0]["qualified"] = True
    write(p, ledger)
    result = validator.validate(unavailable_run)
    assert not result["valid"]
    assert any("unavailable_not_success" in key for key in result["failures"])


def test_missing_planned_conditions_and_extra_calls_are_failure(unavailable_run):
    p = unavailable_run/"aggregate/development_ledger.json"
    ledger = json.loads(p.read_text())
    ledger["records"].pop()
    ledger["actual_terminal_prediction_requests"] = 37
    write(p, ledger)
    result = validator.validate(unavailable_run)
    assert not result["valid"]
    assert "complete_declared_coverage" in result["failures"]
    assert "terminal_prediction_count" in result["failures"]
    assert "no_hidden_or_undeclared_conditions" in result["failures"]


def test_plot_and_csv_numbers_are_verified(unavailable_run):
    p = unavailable_run/"review/development_outcomes.csv"
    p.write_text(p.read_text().replace("HISTORY_UNAVAILABLE", "FAKE_SUCCESS", 1))
    result = validator.validate(unavailable_run)
    assert not result["valid"]
    assert any("review_csv_label" in key for key in result["failures"])


def test_instance_mask_is_recomputed_not_trusted(tmp_path):
    p = tmp_path/"capture.json"
    image = tmp_path/"rgb.jpg"
    Image.new("RGB", (8, 6)).save(image)
    mask = np.zeros((6, 8), dtype=np.uint32)
    mask[1:5, 2:7] = 7
    maskpath = tmp_path/"mask.npz"
    np.savez_compressed(maskpath, mask=mask)
    record = dict(rgb_file=image.name, mask_file=maskpath.name,
                  rgb_jpeg_sha256=validator.digest(image), mask_sha256=validator.digest(maskpath),
                  camera=dict(actual_intrinsics=dict(resolution_width_height=[8, 6])),
                  instance=dict(idToLabels={"0": "BACKGROUND", "7": "/World/JOINSource02Obstacle/Mesh"},
                                matched_instance_ids=[7], visible_pixels=20, image_fraction=20/48,
                                bbox_xyxy=[2, 1, 6, 4]),
                  stable_camera_pose_and_simulation_time=True, same_render_product=True,
                  model_input_annotations=False, agent_pose_world=[0, 0, 0], simulation_time_s=1.)
    write(p, record)
    audit = validator.Audit()
    out = validator.validate_capture(p, audit)
    assert out["visible_pixels"] == 20 and all(audit.checks.values())
    record["instance"]["visible_pixels"] = 0
    write(p, record)
    audit = validator.Audit()
    validator.validate_capture(p, audit)
    assert not all(audit.checks.values())
    assert any(not value for key, value in audit.checks.items() if "pixel_count" in key)


def test_saved_geometry_reprojection_is_called_without_model(monkeypatch, tmp_path):
    calls = []
    class Runner:
        @staticmethod
        def load_environments(placement):
            calls.append(placement)
            raise ValueError("synthetic geometry mismatch detected")
    p = tmp_path/"placement.json"
    write(p, {"status": "AVAILABLE"})
    audit = validator.Audit()
    value = audit.attempt("geometry", lambda: validator.validate_placement(p, Runner, audit))
    assert value is None and len(calls) == 1 and not audit.checks["geometry"]

"""Runner protocol fixtures only; subprocess inference is mocked or forbidden."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("join_source02_runner_test", ROOT / "scripts/run_join_source02.py")
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


@pytest.fixture
def frozen(tmp_path, monkeypatch):
    def fake_git(*args):
        if args[0] == "status":
            return ""
        if args[0] == "branch":
            return "main"
        return "fixture_committed_sha"
    monkeypatch.setattr(runner, "git", fake_git)
    run = tmp_path / "run"
    runner.prepare(run, ROOT / "configs/join_source_02.yaml")
    ids = runner.declared_condition_ids(runner.read(run / "protocol.json"))
    (run / "development/input_manifests").mkdir(parents=True)
    return run, ids


def write_manifest(run, cid, availability):
    value = dict(condition_id=cid, availability=availability,
                 reason="implementation fixture, not actual source",
                 config_path=str(run / "config_snapshot.yaml"))
    path = run / "development/input_manifests" / (cid + ".json")
    path.write_text(json.dumps(value))


def test_prepare_needs_no_input_history_and_refuses_overwrite(frozen):
    run, ids = frozen
    assert len(ids) == 12
    assert not list((run / "development/input_manifests").iterdir())
    assert runner.read(run / "development_freeze.json")["new_inference_calls"] == 0
    with pytest.raises(FileExistsError):
        runner.prepare(run, ROOT / "configs/join_source_02.yaml")


def test_unavailable_all_conditions_preserved_without_calls(frozen, monkeypatch):
    run, ids = frozen
    for i, cid in enumerate(ids):
        write_manifest(run, cid, "HISTORY_UNAVAILABLE" if i % 2 else "PLACEMENT_UNAVAILABLE")
    def forbidden(*args, **kwargs):
        raise AssertionError("unexpected inference subprocess")
    monkeypatch.setattr(runner.subprocess, "run", forbidden)
    result = runner.screen(run)
    assert len(result["records"]) == 12
    assert result["actual_terminal_prediction_requests"] == 0
    assert result["status"] == "NO_QUALIFYING_DEVELOPMENT_SCENARIO"
    assert result["selected_condition_id"] is None
    with pytest.raises(FileExistsError):
        runner.screen(run)


def test_skip_or_partial_condition_prohibited_before_calls(frozen, monkeypatch):
    run, ids = frozen
    monkeypatch.setattr(runner.subprocess, "run", lambda *a, **k: pytest.fail("must not infer"))
    with pytest.raises(ValueError, match="next predeclared"):
        runner.screen(run, only=ids[1])
    (run / "development" / ids[0]).mkdir()
    with pytest.raises(ValueError, match="partial condition"):
        runner.screen(run)


def test_first_complete_qualifier_stops_without_later_manifest(frozen, monkeypatch):
    run, ids = frozen
    write_manifest(run, ids[0], "AVAILABLE")
    calls = []
    def fake_worker(argv, **kwargs):
        calls.append(argv)
        out = run / "development" / ids[0]
        out.mkdir()
        (out / "summary.json").write_text("{}")
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(runner.subprocess, "run", fake_worker)
    monkeypatch.setattr(runner, "evaluate_saved", lambda *a: {
        "condition_id": ids[0], "label": "SAFE_BYPASS_CANDIDATE", "qualified": True,
        "completed_branches": list(runner.BRANCHES),
        "call_counts": {"terminal_predictions": 3, "buffer_only": 45},
    })
    result = runner.screen(run)
    assert len(calls) == 1
    assert result["selected_condition_id"] == ids[0]
    assert result["actual_terminal_prediction_requests"] == 3
    assert result["status"] == "DEVELOPMENT_RESPONSE_ONLY"
    assert not result["confirmation_attempted"]


def test_frozen_protocol_change_blocks_before_calls(frozen):
    run, _ = frozen
    path = run / "protocol.json"
    path.write_text(path.read_text() + " ")
    with pytest.raises(ValueError, match="protocol.json changed"):
        runner.verify_freeze(run, require_pushed=True)


def test_saved_analysis_cannot_invoke_model_or_controller(frozen, monkeypatch):
    run, ids = frozen
    write_manifest(run, ids[0], "AVAILABLE")
    out = run / "development" / ids[0]
    out.mkdir()
    monkeypatch.setattr(runner.subprocess, "run", lambda *a, **k: pytest.fail("no runtime in analyze"))
    monkeypatch.setattr(runner, "evaluate_saved", lambda *a: {
        "label": "CHANGED_BUT_UNSAFE", "qualified": False,
    })
    result = runner.analyze(run, ids[0])
    assert result["new_inference_calls"] == result["new_MPC_calls"] == 0
    assert result["qualified"] is False
    with pytest.raises(FileExistsError):
        runner.analyze(run, ids[0])

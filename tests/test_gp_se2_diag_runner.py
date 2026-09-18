"""Phase gates, source integrity and exclusive writes; no optimizer invoked."""
import importlib.util
import json
from pathlib import Path

import pytest
import yaml

SPEC=importlib.util.spec_from_file_location('diag_runner_test',Path(__file__).resolve().parents[1]/'scripts/run_gp_se2_diag_01.py')
runner=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(runner)


def put(path,data):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(data))


def test_no_new_optimization_before_complete_preserved_audit(tmp_path):
    with pytest.raises(FileNotFoundError):runner.require_audit(tmp_path)
    out=tmp_path/'existing_failure_audit'
    put(out/'completion.json',dict(all_40_attempts_reclassified=True,source_files_unchanged=False))
    put(out/'summary.json',dict(attempt_count=40))
    with pytest.raises(ValueError):runner.require_audit(tmp_path)
    put(out/'completion.json',dict(all_40_attempts_reclassified=True,source_files_unchanged=True))
    runner.require_audit(tmp_path)


def test_settings_rejects_source_or_frozen_diagnostic_config_drift(tmp_path,monkeypatch):
    root=tmp_path/'repo';root.mkdir();monkeypatch.setattr(runner,'ROOT',root)
    run=root/'run';run.mkdir();primary=root/'original';primary.mkdir()
    (run/'config_snapshot.yaml').write_text('experiment: GP-SE2-DIAG-01\n')
    (primary/'config_snapshot.yaml').write_text('formulation: {}\n')
    put(primary/'case_manifest.json',dict(selected=[]))
    core=root/'core.py';core.write_text('# unchanged\n')
    put(run/'source.json',dict(primary_run=str(primary),config_sha256=runner.digest(run/'config_snapshot.yaml'),
        original_config_sha256=runner.digest(primary/'config_snapshot.yaml'),
        original_case_manifest_sha256=runner.digest(primary/'case_manifest.json'),
        core_numerical_source_sha256={'core.py':runner.digest(core)}))
    runner.settings(run)
    old=core.read_text();core.write_text('# changed\n')
    with pytest.raises(ValueError,match='numerical source changed'):runner.settings(run)
    core.write_text(old);(run/'config_snapshot.yaml').write_text('experiment: changed\n')
    with pytest.raises(ValueError,match='diagnostic configuration changed'):runner.settings(run)


def test_prepare_existing_run_refuses_before_reading_or_mutating_source(tmp_path,monkeypatch):
    monkeypatch.setattr(runner,'ROOT',tmp_path)
    config=tmp_path/'config.yaml';config.write_text(yaml.safe_dump({'primary_run':'absent'}))
    run=tmp_path/'data/robotless_gp_se2_diag_01/existing';run.mkdir(parents=True)
    guard=run/'guard';guard.write_text('immutable')
    with pytest.raises(ValueError,match='new directory'):runner.prepare(run,config)
    assert guard.read_text()=='immutable'


def test_json_and_array_writes_are_exclusive_and_nonfinite_is_null(tmp_path):
    import numpy as np
    path=tmp_path/'numeric.json';runner.write(path,dict(failed_metric=None,nonfinite=np.nan))
    assert json.loads(path.read_text())==dict(failed_metric=None,nonfinite=None)
    with pytest.raises(FileExistsError):runner.write(path,dict(failed_metric=0))
    array=tmp_path/'data.npy';runner.array(array,np.ones(3))
    with pytest.raises(FileExistsError):runner.array(array,np.zeros(3))

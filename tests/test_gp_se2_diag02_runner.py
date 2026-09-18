"""Immutable-source guards and numerical-improvement interpretation."""
import importlib.util
import json
from pathlib import Path
import pytest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('diag02_runner_tests',ROOT/'scripts/run_gp_se2_diag_02.py')
runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)


def test_settings_rejects_changed_config_core_and_seed_source(tmp_path,monkeypatch):
    monkeypatch.setattr(runner,'ROOT',tmp_path)
    run=tmp_path/'run';run.mkdir();config=run/'config_snapshot.yaml';config.write_text('experiment: fixture\n')
    core=tmp_path/'core.py';core.write_text('preserved')
    seed=tmp_path/'seed.npy';seed.write_bytes(b'preserved actual seed bytes')
    source={'diagnostic_config_sha256':runner.digest(config),'preserved_core_sha256':{'core.py':runner.digest(core)},
            'input_sha256':{str(seed):runner.digest(seed)}}
    (run/'source.json').write_text(json.dumps(source))
    assert runner.settings(run)[0]['experiment']=='fixture'
    seed.write_bytes(b'changed')
    with pytest.raises(ValueError,match='input drift'):runner.settings(run)
    seed.write_bytes(b'preserved actual seed bytes');core.write_text('changed')
    with pytest.raises(ValueError,match='source drift'):runner.settings(run)
    core.write_text('preserved');config.write_text('changed: true')
    with pytest.raises(AssertionError):runner.settings(run)


def test_new_run_output_refuses_historical_or_unrelated_directory(tmp_path):
    with pytest.raises(ValueError,match='new run below DIAG-02'):
        runner.prepare(tmp_path,ROOT/'configs/gp_se2_diag_02.yaml')

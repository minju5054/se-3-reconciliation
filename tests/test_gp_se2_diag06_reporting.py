"""Additive report repair never changes frozen numerical evidence."""
import importlib.util
import json
from pathlib import Path
import sys
import pytest
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
SPEC=importlib.util.spec_from_file_location('diag06_report_finish',ROOT/'scripts/finish_gp_se2_diag06_report.py')
m=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(m)


def test_repair_writes_only_missing_report_and_refuses_overwrite(tmp_path,monkeypatch):
    monkeypatch.setattr(m,'verify_frozen',lambda p:None)
    (tmp_path/'optimization_completed.json').write_text('{"actual_new_GP_solves":5}')
    (tmp_path/'experiment_manifest.json').write_text(json.dumps(dict(starts=[dict(solve_id='synthetic')])) )
    folder=tmp_path/'solves/synthetic';folder.mkdir(parents=True)
    (folder/'analysis.json').write_text('{"wall_s":1}')
    (tmp_path/'analysis_console.log').write_text('FileNotFoundError aggregate/all_methods.csv')
    before={p:p.read_bytes() for p in tmp_path.rglob('*') if p.is_file()};seen=[]
    monkeypatch.setattr(m,'assemble',lambda p:(dict(summary=dict(synthetic=True)),dict(all_methods=[dict(synthetic=True)])))
    monkeypatch.setattr(m,'generate',lambda p:seen.append(p))
    m.finish(tmp_path)
    assert seen==[tmp_path]
    assert all(p.read_bytes()==value for p,value in before.items())
    correction=json.loads((tmp_path/'reporting_correction.json').read_text())
    assert correction['new_GP_solves']==correction['new_MPC']==correction['new_rollout']==0
    assert not correction['numerical_analysis_rerun']
    with pytest.raises(FileExistsError):m.finish(tmp_path)


def test_repair_requires_complete_saved_analyses(tmp_path,monkeypatch):
    monkeypatch.setattr(m,'verify_frozen',lambda p:None)
    with pytest.raises(ValueError,match='saved'):m.finish(tmp_path)
    (tmp_path/'optimization_completed.json').write_text('{}')
    (tmp_path/'experiment_manifest.json').write_text('{"starts":[{"solve_id":"missing"}]}')
    with pytest.raises(ValueError,match='analyses'):m.finish(tmp_path)

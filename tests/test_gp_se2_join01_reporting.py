"""Reporting-only interval correction; no new source or solver execution."""
import importlib.util
from pathlib import Path


def helper():
    path=Path(__file__).resolve().parents[1]/'scripts/report_gp_se2_join01.py'
    spec=importlib.util.spec_from_file_location('join01_report',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.old_inflight_check


def test_inflight_old_interval_excludes_stationary_bootstrap():
    commands=[dict(application_state_id=i,chunk_id='' if i<2 else 'old' if i<5 else 'fresh') for i in range(7)]
    context=dict(client_inflight_state_range={'start_state_id':3},switch_state_id=5,old_chunk_id='old')
    r=helper()(commands,context);assert r['valid'] and r['command_count']==2


def test_wrong_inflight_chunk_or_missing_commands_fails():
    context=dict(client_inflight_state_range={'start_state_id':3},switch_state_id=5,old_chunk_id='old')
    assert not helper()([],context)['valid']
    assert not helper()([dict(application_state_id=4,chunk_id='fresh')],context)['valid']

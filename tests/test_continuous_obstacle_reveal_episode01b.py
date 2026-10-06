"""Synthetic launch tests and saved-only parity; no SimulationApp/model/solver."""
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import pytest
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import run_continuous_obstacle_reveal_episode01b as runner
import validate_continuous_obstacle_reveal_episode01b as validator
from reconciliation.join_source03 import read,save,sha


@pytest.fixture
def paired(tmp_path):
    old=ROOT/runner.declaration()['historical_run'];new=tmp_path/'new';new.mkdir()
    for name in (*runner.COPIED,'protocol.json'):
        (new/name).write_bytes((old/name).read_bytes())
    p=read(new/'protocol.json');p['experiment']=p['declaration']['experiment']=runner.declaration()['experiment']
    (new/'protocol.json').write_text(json.dumps(p))
    return old,new


def test_exact_historical_environment_and_no_global_mutation():
    cfg=runner.declaration(); historical=runner.historical_launch(cfg)
    before=dict(os.environ);sentinel={**before,**{k:'forbidden' for k in cfg['unset_environment']}}
    env=runner.sanitized_environment(sentinel,cfg)
    assert len(historical['unset'])==11 and len(historical['explicit'])==4
    assert all(k not in env for k in cfg['unset_environment'])
    assert all(sentinel[k]=='forbidden' for k in cfg['unset_environment'])
    assert dict(os.environ)==before
    output=subprocess.check_output([sys.executable,'-c','import os,json;print(json.dumps(dict(os.environ)))'],env=env,text=True)
    actual=json.loads(output)
    assert all(k not in actual for k in cfg['unset_environment'])
    assert all(actual[k]==v for k,v in cfg['set_environment'].items())
    assert dict(os.environ)==before


def test_new_namespace_cannot_target_failed_run():
    with pytest.raises(ValueError): runner.namespace(ROOT/runner.declaration()['historical_run'])
    assert runner.namespace(ROOT/'data/continuous_obstacle_reveal_episode_01b/fixture').name=='fixture'


def test_full_scientific_protocol_equal_and_inputs_not_modified(paired):
    old,new=paired;before=runner.tree_hashes(old)
    v=runner.equivalence(old,new)
    assert v['all_scientific_fields_equal'] and len(v['fields'])>=26
    assert all(r['equal'] for r in v['fields'].values())
    assert v['intentional_difference']=='process_launch_environment_only'
    assert runner.tree_hashes(old)==before
    p=read(new/'protocol.json')
    assert p['declaration']['maximum_terminal_predictions']==4
    assert p['declaration']['retry'] is False and p['reset_after_initialization']==0
    assert p['no_reconciliation'] and p['waypoint_dt'] is None


@pytest.mark.parametrize('field,value',[('postroll_sim_s',.2),('maximum_handoff_attempts',4),
    ('maximum_terminal_predictions',5),('retry',True),('request_rule','later frame')])
def test_protocol_drift_is_blocked(paired,field,value):
    old,new=paired;p=read(new/'protocol.json');p['declaration'][field]=value
    (new/'protocol.json').write_text(json.dumps(p))
    with pytest.raises(ValueError,match='protocol differs'):runner.equivalence(old,new)


def test_runtime_timing_or_camera_drift_is_blocked(paired):
    old,new=paired;p=yaml.safe_load((new/'config_snapshot.yaml').read_text())
    p['execution']['capture_hz']=5
    (new/'config_snapshot.yaml').write_text(yaml.safe_dump(p))
    with pytest.raises(ValueError,match='runtime config differs'):runner.equivalence(old,new)


def test_historical_failed_evidence_authenticates():
    preserved=runner.authenticate01(runner.declaration())
    assert len(preserved)>30
    assert all(sha(p)==h for p,h in preserved.items())


def test_single_launch_reservation_fail_closed(tmp_path,monkeypatch):
    monkeypatch.setattr(runner,'git',lambda *a:'synthetic_freeze')
    runner.reserve_launch(tmp_path,['test-only'])
    with pytest.raises(FileExistsError): runner.reserve_launch(tmp_path,['test-only'])
    assert read(tmp_path/'launch_attempt.json')['full_startup_budget']==1


@pytest.mark.parametrize('marker',['execution_start.json','episodes'])
def test_cannot_launch_after_existing_execution_marker(tmp_path,marker):
    (tmp_path/marker).touch()
    with pytest.raises(FileExistsError):runner.reserve_launch(tmp_path,[])


def test_failed_startup_never_retried_even_with_zero_exit(tmp_path,monkeypatch):
    cfg=runner.declaration();(tmp_path/'logs').mkdir()
    save(tmp_path/'generation_actual.json',{'valid':True})
    save(tmp_path/'launch_environment.json',{'argv':runner.launch_argv(tmp_path,cfg)})
    monkeypatch.setattr(runner,'verify',lambda *a:None)
    monkeypatch.setattr(runner,'git',lambda *a:'synthetic_freeze')
    calls=[]
    def popen(argv,**kw):
        calls.append(argv)
        return SimpleNamespace(poll=lambda:0,returncode=0)
    monkeypatch.setattr(runner.subprocess,'Popen',popen)
    runner.launch(tmp_path)
    v=read(tmp_path/'launch_result.json')
    assert v['returncode']==0 and not v['episode_directory_exists'] and not v['schedule_completed']
    with pytest.raises(FileExistsError):runner.launch(tmp_path)
    assert len(calls)==1 and calls[0]==runner.launch_argv(tmp_path,cfg)


def test_sanitized_loader_failure_cannot_pass(tmp_path,monkeypatch):
    (tmp_path/'technical_preflight').mkdir()
    cfg=runner.declaration()
    monkeypatch.setattr(runner.subprocess,'run',lambda *a,**kw:SimpleNamespace(returncode=0,
        stdout='undefined symbol: fake_12_8, version bad\n',stderr=''))
    v=runner.loader_checks(tmp_path,cfg)
    assert not v['valid'] and v['status']=='TECHNICAL_PREFLIGHT_BLOCKED'
    assert v['full_SimulationApp_starts']==0


def test_no_episode_counts_are_zero_not_prelaunch_intended_budget(tmp_path):
    save(tmp_path/'launch_attempt.json',{'full_startup_budget':1})
    save(tmp_path/'execution_start.json',{'scientific_episodes':1})
    c=validator.accounting(tmp_path)
    assert c['full_SimulationApp_launch_attempts']==1
    assert c['scientific_episodes_initialized']==c['scientific_episodes_completed']==0
    assert all(v==0 for k,v in c.items() if k!='full_SimulationApp_launch_attempts')


def test_saved_historical_request_kinds_are_counted_without_reexecution(tmp_path):
    cfg=runner.declaration()
    authority=read(ROOT/cfg['historical_run']/'source.json')['OSA03']
    source=Path(authority)/'episodes/REPEAT_00/requests'
    dest=tmp_path/'episodes/EPISODE_00/requests';dest.mkdir(parents=True)
    for p in source.glob('seq_*_metadata.json'):
        (dest/p.name).write_bytes(p.read_bytes())
    c=validator.accounting(tmp_path)
    assert c['terminal_LightNav_predictions']==2 and c['buffer_only_LightNav_requests']==5
    assert c['full_SimulationApp_launch_attempts']==c['MPC_submissions']==0


def test_frozen_01_collector_policy_validators_unchanged():
    cfg=runner.declaration();old=ROOT/cfg['historical_run']
    frozen=read(old/'freeze.json')['source_sha256']
    for name in ('scripts/isaac/continuous_obstacle_reveal_episode01.py',
                 'src/reconciliation/continuous_obstacle_reveal_episode01.py',
                 'scripts/validate_continuous_obstacle_reveal_episode01.py',
                 'scripts/report_continuous_obstacle_reveal_episode01.py',
                 'scripts/isaac/obstacle_source03_online.py','scripts/online_mpc_worker.py'):
        assert sha(ROOT/name)==frozen[name]


def test_saved_only_audit_report_have_no_scientific_entrypoints():
    import ast
    for kind in ('validate','report'):
        p=ROOT/f'scripts/{kind}_continuous_obstacle_reveal_episode01b.py'
        calls={n.func.id for n in ast.walk(ast.parse(p.read_text())) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)}
        assert not calls.intersection({'launch','SimulationApp','collect_episode','Worker','solve','optimize','start01'})


def test_blocked_report_is_one_explicit_status_figure(tmp_path,monkeypatch):
    import report_continuous_obstacle_reveal_episode01b as reporter
    out=tmp_path/'report';out.mkdir();monkeypatch.setattr(reporter,'OUT',out)
    save(tmp_path/'attempt_validation.json',dict(scientific=None,
        classification='TECHNICAL_EXECUTION_BLOCKED',scientific_validator_error='synthetic no episode',
        calls=dict(full_SimulationApp_launch_attempts=1,scientific_episodes_initialized=0,
                   terminal_LightNav_predictions=0)))
    reporter.report(tmp_path)
    assert reporter.check(tmp_path)['technical_only']
    assert len(list((out/'figures').glob('*.png')))==1
    assert all(not c['generated'] and not c['applied'] for c in read(out/'result_summary.json')['chunks'])

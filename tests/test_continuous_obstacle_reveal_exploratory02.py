"""Synthetic and saved-only tests; no model, MPC, optimizer or Isaac execution."""
from copy import deepcopy
import ast
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest
from shapely.geometry import box
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
from reconciliation.continuous_obstacle_reveal_exploratory02 import (
    POLICY, ExploratoryEnvironment, validate_policy, acquisition_classification,post_reveal_classification)
from reconciliation.join_online02 import guard_check
from reconciliation.join_source02 import whole_raw_polyline_check
from reconciliation.join_source03 import read,save,sha
from test_join_online02 import env
import run_continuous_obstacle_reveal_exploratory02 as runner
import validate_continuous_obstacle_reveal_exploratory02 as validator


def wrapped(clearance):
    return ExploratoryEnvironment(env(box(-1,.20+clearance,2,2)),POLICY)


@pytest.mark.parametrize('c,passed,overlap',[(.008,True,False),(.06,True,False),(-.001,False,True),(0.,False,False),(1e-8,False,False)])
def test_full_reference_uses_same_checker_and_retains_radius_tolerance(c,passed,overlap):
    e=wrapped(c);raw=np.array([[0,0,0],[.5,0,.3]]);before=raw.tobytes()
    r=whole_raw_polyline_check(raw,e)
    assert r['clearance_valid']==passed and r['physical_overlap']==overlap
    assert r['exploratory_overlap_free']==passed
    assert r['legacy_5cm_margin_pass']==(c>.05)
    assert r['footprint_radius_m']==.20 and r['required_clearance_m']==0
    expected=e.environment.check_polyline(raw,radius=.20,required_clearance=0.)
    assert all(r[k]==v for k,v in expected.items())
    assert raw.tobytes()==before and not r['connector_included']
    assert not e.environment.check_polyline(raw)['clearance_valid'] if c<.05 else True


@pytest.mark.parametrize('reason,duration',[('new_solve',.1),('hold',1/60)])
def test_low_positive_guard_accepted_without_modifying_command(reason,duration):
    cmd=dict(reason=reason,v_mps=.3,omega_radps=0.)
    r=guard_check(wrapped(.008),[0,0,0],cmd,1/60)
    assert r['safe'] and not r['check']['legacy_5cm_margin_pass']
    assert r['duration_s']==duration and r['command']==cmd and not r['command_modified']
    assert not guard_check(wrapped(-.002),[0,0,0],cmd,1/60)['safe']


def test_future_overlap_is_rejected_before_motion():
    e=ExploratoryEnvironment(env(box(.25,-1,1,1)),POLICY)
    cmd=dict(reason='new_solve',v_mps=1.,omega_radps=0.)
    pose=np.array([0.,0.,0.]);before=pose.copy()
    r=guard_check(e,pose,cmd,1/60)
    assert not r['safe'] and r['check']['physical_overlap']
    np.testing.assert_array_equal(pose,before)


@pytest.mark.parametrize('k,v',[('footprint_radius_m',0),('guard_enabled',False),('physical_overlap_rejection',False),('required_extra_clearance_m',-.01),('legacy_required_extra_clearance_m',0)])
def test_cannot_disable_guard_or_footprint(k,v):
    p=deepcopy(POLICY);p[k]=v
    with pytest.raises(ValueError):ExploratoryEnvironment(env(),p)


def test_unknown_workspace_is_not_overlap_free_candidate():
    r=whole_raw_polyline_check([[30,30,0],[31,30,0]],wrapped(.008))
    assert r['unknown'] and not r['exploratory_overlap_free']


def test_historical_config_bytes_all_implementation_and_official_workers_unchanged():
    old=ROOT/runner.declaration()['historical_run']
    frozen=read(old/'freeze.json')
    for p,h in frozen['source_sha256'].items(): assert sha(ROOT/p)==h,p
    assert runner.declaration()['exploratory_clearance']==POLICY
    raw=read(old/'protocol.json');assert 'exploratory_clearance' not in raw
    e=env();assert e.check_polyline([[0,0,0]])['required_clearance_m']==.05


def test_protocol_only_expected_delta_and_no_mutation(tmp_path):
    old=ROOT/runner.declaration()['historical_run'];before=runner.tree_hashes(old)
    for n in runner.COPIED:(tmp_path/n).write_bytes((old/n).read_bytes())
    p=read(old/'protocol.json');p['experiment']=p['declaration']['experiment']=runner.declaration()['experiment']
    p['exploratory_clearance']=POLICY
    p['declaration']['classification_priority']=runner.declaration()['classification_priority']
    save(tmp_path/'protocol.json',p)
    assert runner.equivalence(old,tmp_path)['unchanged_old_protocol_except_namespace_classification_and_policy']
    assert runner.tree_hashes(old)==before
    p['initial_pose_world'][0]+=1;(tmp_path/'protocol.json').write_text(json.dumps(p))
    with pytest.raises(ValueError):runner.equivalence(old,tmp_path)


def test_saved_request_history_payload_complete_and_unmodified():
    old=ROOT/runner.declaration()['historical_run'];ep=old/'episodes/EPISODE_00'
    for cid in ['chunk_000','chunk_001']:
        r=read(ep/'chunks'/cid/'metadata.json');before=sha(ep/r['response_ref']['path'])
        h=validator.history_provenance(ep,r,read(old/'protocol.json')['instruction'])
        assert len(h['history_frame_ids'])==len(h['history_frame_sha256'])>=4
        assert h['model_input_segments_reconstructed']
        assert not h['radius_or_map_in_model_payload']
        assert before==h['raw_response_sha256']


@pytest.mark.parametrize('pairs,expected',[([{'available':False},{'available':False}],None),
    ([{'available':True,'meaningful':False},{'available':False}],None),
    ([{'available':True,'meaningful':True},{'available':False}],'POST_REVEAL_INTENT_EVOLVING'),
    ([{'available':True,'meaningful':False}]*2,'POST_REVEAL_INTENT_STABLE')])
def test_missing_pairs_never_imply_stable(pairs,expected):
    assert post_reveal_classification(pairs)==expected


@pytest.mark.parametrize('applied,overlap,valid,expected',[
    (4,False,True,'CONTINUOUS_EXPLORATORY_C0_C3_ACQUIRED'),
    (2,False,False,'PARTIAL_CONTINUOUS_EXPLORATORY_SEQUENCE'),
    (1,True,False,'FIRST_POST_REVEAL_REFERENCE_OVERLAPS'),
    (0,False,False,'TECHNICAL_EXECUTION_BLOCKED')])
def test_predeclared_outcomes(applied,overlap,valid,expected):
    c=[dict(applied=i<applied) for i in range(4)];c[1]['geometry_on']=dict(physical_overlap=overlap)
    assert acquisition_classification(c,valid,technical=applied==0)==expected


def test_observer_permission_failure_does_not_abort_or_retry_child(tmp_path,monkeypatch):
    (tmp_path/'logs').mkdir();save(tmp_path/'generation_actual.json',dict(valid=True))
    save(tmp_path/'launch_environment.json',dict(argv=runner.launch_argv(tmp_path,runner.declaration())))
    monkeypatch.setattr(runner,'verify',lambda *a:None)
    monkeypatch.setattr(runner,'observe_process',lambda pid:(_ for _ in ()).throw(PermissionError('synthetic')))
    monkeypatch.setattr(runner.time,'sleep',lambda t:None)
    import run_continuous_obstacle_reveal_episode01b as oldrunner
    monkeypatch.setattr(oldrunner,'git',lambda *a:'synthetic-freeze')
    calls=[];polls=iter([None,0])
    def popen(*a,**k):
        calls.append(a);save(tmp_path/'execution_start.json',dict(pid=123))
        return SimpleNamespace(poll=lambda:next(polls),returncode=0)
    monkeypatch.setattr(runner.subprocess,'Popen',popen)
    runner.launch(tmp_path)
    r=read(tmp_path/'launch_result.json');assert r['returncode']==0 and len(r['observer_errors'])==1
    with pytest.raises(FileExistsError):runner.launch(tmp_path)
    assert len(calls)==1


def test_no_optimization_entrypoint_used_by_geometry_native_gate(monkeypatch,tmp_path):
    import reconciliation.graph_optimizer as graph
    import reconciliation.local_se2_reconciliation as local
    def forbidden(*a,**k):raise AssertionError('reconciliation forbidden')
    monkeypatch.setattr(graph,'solve_graph',forbidden)
    monkeypatch.setattr(graph,'solve_least_squares',forbidden)
    for name in ['reconcile','reconcile_local_se2','solve']:
        if hasattr(local,name):monkeypatch.setattr(local,name,forbidden)
    from continuous_obstacle_reveal_episode01 import NativePolicyWorker
    from test_continuous_obstacle_reveal_episode01 import frame
    from reconciliation.continuous_obstacle_reveal_episode01 import RequestPolicy
    policy=RequestPolicy();policy.send(frame(45),True,'chunk_000')
    raw=[[0,0,0],[.3,0,0]]
    msg=dict(type='result',kind='prediction',chunk_id='chunk_000',status='PREDICTION_READY',world=raw,raw_local_ref={},world_ref={})
    worker=SimpleNamespace(drain=lambda:[msg])
    geometry=SimpleNamespace(present=False,ask=lambda op,world:whole_raw_polyline_check(world,wrapped(.008)))
    gate=NativePolicyWorker(worker,SimpleNamespace(policy=policy),geometry,tmp_path)
    assert gate.drain()[0] is msg
    assert read(tmp_path/'reference_checks/chunk_000.json')['check']['exploratory_overlap_free']


def test_saved_only_scripts_do_not_call_scientific_entrypoints():
    for kind in ['validate','report']:
        source=ROOT/f'scripts/{kind}_continuous_obstacle_reveal_exploratory02.py'
        calls={n.func.id for n in ast.walk(ast.parse(source.read_text())) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)}
        assert not calls & {'launch','start','SimulationApp','Worker','solve_graph','solve_least_squares','collect_episode'}


def test_four_png_report_with_saved_fixture(tmp_path):
    import report_continuous_obstacle_reveal_exploratory02 as report
    old=ROOT/runner.declaration()['historical_run'];v=deepcopy(read(old/'validation.json'))
    v.update(exploratory_clearance=POLICY,exploratory_first_reaction_valid=False,interval_diagnostics=[])
    # Report tests use the saved historical episode as a fixture, never as new data.
    (tmp_path/'scenario.json').write_bytes((old/'scenario.json').read_bytes())
    save(tmp_path/'validation.json',v)
    out=tmp_path/'out';report.report(tmp_path,out)
    assert report.validate_report(tmp_path,out)['exactly_four_PNGs']


def test_behavior_only_uncovered_outcome_not_called_infrastructure_failure():
    c=[dict(applied=i==0) for i in range(4)]
    assert acquisition_classification(c,False) is None

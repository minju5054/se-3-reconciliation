"""Synthetic/saved-only long-source tests; no scientific MPC/model/Isaac calls."""
import ast
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
from reconciliation.long_continuous_obstacle_reveal_source01 import RequestPolicy,CHUNKS,classify,figure_names
from reconciliation.long_source_mpc01 import authenticate_speed,configure_before_tracker
from reconciliation.join_source03 import read,save,sha
from test_continuous_obstacle_reveal_episode01 import frame,FakeWorker,NativePolicyWorker
from test_continuous_obstacle_reveal_exploratory02 import wrapped
from reconciliation.join_source02 import whole_raw_polyline_check
from reconciliation.online_mpc_adapter import SETTING_NAMES
import run_long_continuous_obstacle_reveal_source01 as runner
import validate_long_continuous_obstacle_reveal_source01 as validator
OLD=ROOT/runner.declaration()['historical_run']


def test_ten_exact_first_frames_no_C10_no_stopping_at_six_or_eight():
    p=RequestPolicy();p.send(frame(45),True,CHUNKS[0]);p.received(CHUNKS[0])
    for i in range(1,10):
        tick=45+30*i;active=dict(chunk_id=CHUNKS[i-1]);applied=(tick-6)/60
        assert p.capture(frame(tick),active,applied)==CHUNKS[i]
        assert p.capture(frame(tick+15),active,applied) is None
        assert not p.allow(frame(tick+15))
        p.send(frame(tick),True,CHUNKS[i])
        with pytest.raises(ValueError):p.send(frame(tick),True,CHUNKS[i])
        assert p.capture(frame(tick+15),active,applied) is None
        p.received(CHUNKS[i])
    assert len(p.sent)==10 and p.capture(frame(360),dict(chunk_id=CHUNKS[-1]),5.9) is None
    with pytest.raises(ValueError):p.send(frame(360),True,'chunk_010')


@pytest.mark.parametrize('count,expected',[(2,'PARTIAL_CONTINUOUS_SOURCE'),(5,'PARTIAL_CONTINUOUS_SOURCE'),
    (6,'LONG_CONTINUOUS_SOURCE_QUALIFIED'),(7,'LONG_CONTINUOUS_SOURCE_QUALIFIED'),
    (8,'LONG_CONTINUOUS_SOURCE_TARGET_REACHED'),(9,'LONG_CONTINUOUS_SOURCE_TARGET_REACHED'),(10,'LONG_CONTINUOUS_SOURCE_MAX_REACHED')])
def test_classifications_require_all_gates(count,expected):
    assert classify(count,True,True)==expected
    assert classify(count,False,True)=='FIRST_REACTION_NOT_QUALIFIED'
    assert classify(count,True,False)=='TECHNICAL_EXECUTION_BLOCKED'
    assert classify(count,True,True,True)=='TECHNICAL_EXECUTION_BLOCKED'


def test_budget_and_cap_only_protocol_deltas(tmp_path):
    cfg=runner.declaration();assert cfg['maximum_terminal_predictions']==10
    assert cfg['maximum_handoff_attempts']==9 and cfg['maximum_active_sim_s']==8 and cfg['postroll_sim_s']==.1
    assert cfg['minimum_qualified_chunks']==6 and cfg['preferred_chunks']==8 and not cfg['retry']
    for n in runner.COPIED:(tmp_path/n).write_bytes((OLD/n).read_bytes())
    runtime=yaml.safe_load((tmp_path/'config_snapshot.yaml').read_text())
    for k in ('maximum_handoff_attempts','maximum_active_sim_s','postroll_sim_s'):runtime['online'][k]=cfg[k]
    (tmp_path/'config_snapshot.yaml').write_text(yaml.safe_dump(runtime))
    p=read(OLD/'protocol.json');p['experiment']=p['declaration']['experiment']=cfg['experiment']
    for k in ('maximum_terminal_predictions','maximum_handoff_attempts','maximum_active_sim_s','postroll_sim_s','classification_priority'):p['declaration'][k]=cfg[k]
    p['speed_intervention']=authenticate_speed(OLD);save(tmp_path/'speed_intervention.json',p['speed_intervention']);save(tmp_path/'protocol.json',p)
    assert runner.equivalence(OLD,tmp_path)['unchanged_scene_model_history_instruction_safety_and_scheduler']
    runtime['execution']['control_hz']=5;(tmp_path/'config_snapshot.yaml').write_text(yaml.safe_dump(runtime))
    with pytest.raises(AssertionError):runner.equivalence(OLD,tmp_path)


def test_speed_base_authentication_half_native_bound_and_external_bytes():
    frozen=authenticate_speed(OLD);before=sha(frozen['external_source'])
    assert frozen['base_effective_linear_limit_m_s']==read(OLD/'workers.json')['mpc']['provenance']['effective_linear_velocity_limit_m_s']
    assert frozen['new_effective_linear_limit_m_s']==.5*frozen['base_effective_linear_limit_m_s']
    module=SimpleNamespace(**frozen['prior_settings'])
    original=read(OLD/'workers.json')['mpc']['provenance'];preserved=deepcopy(original)
    _,p=configure_before_tracker(module,original,frozen)
    assert original==preserved and sha(frozen['external_source'])==before
    assert [k for k in SETTING_NAMES if p['official_settings'][k]!=frozen['prior_settings'][k]]==['OBJNAV_V_MAX']
    assert p['configured_before_tracker_construction'] and not p['speed_intervention']['added_output_postprocessing']
    wrong=deepcopy(frozen);wrong['base_effective_linear_limit_m_s']*=2
    with pytest.raises(AssertionError):configure_before_tracker(module,original,wrong)


def test_actual_official_tracker_reads_bound_without_real_MPC_solve():
    # Execute the pinned _solve function against a stub controller only. This
    # authenticates argument flow and avoids constructing CasADi/IPOPT entirely.
    frozen=authenticate_speed(OLD);tree=ast.parse(Path(frozen['external_source']).read_text())
    tracker=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='MpcTracker')
    method=next(n for n in tracker.body if isinstance(n,ast.FunctionDef) and n.name=='_solve')
    source=ast.Module(body=[method],type_ignores=[])
    import time
    calls=[]
    def solve(pose,reference,previous,limit):
        calls.append((previous,limit));return (.3,.2),np.zeros((6,3))
    scope=dict(np=np,time=time,OBJNAV_V_MAX=frozen['new_effective_linear_limit_m_s'],
        project_world_to_local=lambda ref,pose:ref,project_local_to_world=lambda ref,pose:ref,
        MpcSolveResult=lambda **kw:SimpleNamespace(**kw))
    # Defer annotations referring to upstream datatypes.
    exec(compile('from __future__ import annotations\n'+ast.unparse(source),'<pinned _solve stub>','exec'),scope)
    result=scope['_solve'](SimpleNamespace(controller=SimpleNamespace(solve=solve)),4,np.zeros(3),np.zeros((5,3)),(.2,.1))
    assert calls==[((.2,.1),.4)] and result.command==(.3,.2)


def test_worker_only_configures_before_tracker_no_command_path():
    for file in ['scripts/long_source_mpc_worker01.py','src/reconciliation/long_source_mpc01.py']:
        tree=ast.parse((ROOT/file).read_text())
        assert not any(isinstance(n,ast.Attribute) and n.attr=='clip' for n in ast.walk(tree))
    spec=importlib.util.spec_from_file_location('long_collector',ROOT/'scripts/isaac/long_continuous_obstacle_reveal_source01.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    argv=['python',str(ROOT/'scripts/online_lightnav_worker.py'),'--config','source']
    assert mod.worker_argv(argv,OLD)==argv
    mpc=['python',str(ROOT/'scripts/online_mpc_worker.py'),'--lightnav-checkout','external']
    changed=mod.worker_argv(mpc,OLD)
    assert changed[2:4]==mpc[2:4] and changed[4:]==['--run',str(OLD)]


def test_memory_matches_solved_commands_and_limits_no_scaling():
    events=[dict(status='reset'),dict(status='submitted',previous_command=[0,0]),
        dict(type='solve_result',status='command',solve_id='s0',command=[.4,.2]),
        dict(status='submitted',previous_command=[.4,.2])]
    commands=[dict(v_mps=.4,omega_radps=.2,solve_id='s0',reason='new_solve')]
    assert validator.audit_controller_speed(events,commands,dict(new_effective_linear_limit_m_s=.4))['memory_checks']==2
    events[-1]['previous_command']=[.2,.1]
    with pytest.raises(AssertionError):validator.audit_controller_speed(events,commands,dict(new_effective_linear_limit_m_s=.4))
    events.pop();commands[0]['v_mps']=.2
    with pytest.raises(AssertionError):validator.audit_controller_speed(events,commands,dict(new_effective_linear_limit_m_s=.4))


@pytest.mark.parametrize('clearance,accepted',[(.008,True),(-.001,False),(0.,False)])
def test_native_gate_raw_immutable_zero_margin_legacy_reject_no_next(tmp_path,clearance,accepted):
    p=RequestPolicy();p.send(frame(45),True,CHUNKS[0]);p.received(CHUNKS[0])
    p.capture(frame(60),dict(chunk_id=CHUNKS[0]),.9);p.send(frame(60),True,CHUNKS[1])
    raw=dict(type='result',kind='prediction',chunk_id=CHUNKS[1],status='PREDICTION_READY',world=[[0,0,0],[.3,0,0]],raw_local_ref={},world_ref={})
    before=deepcopy(raw);worker=FakeWorker([raw])
    proxy=NativePolicyWorker(worker,SimpleNamespace(policy=p),SimpleNamespace(present=True,ask=lambda op,world:whole_raw_polyline_check(world,wrapped(clearance))),tmp_path)
    msg=proxy.drain()[0];assert raw==before
    assert (msg['status']=='PREDICTION_READY')==accepted
    check=read(tmp_path/'reference_checks/chunk_001.json')['check']
    assert not check['legacy_5cm_margin_pass'] and check['required_clearance_m']==0
    if not accepted:
        assert not p.allow(frame(75))
        assert p.capture(frame(75),dict(chunk_id=CHUNKS[0]),.9) is None
        with pytest.raises(ValueError):p.send(frame(75),True,CHUNKS[2])


def test_saved_historical_implementation_and_pose_timing_unchanged():
    for name,h in read(OLD/'freeze.json')['source_sha256'].items():assert sha(ROOT/name)==h,name
    cfg=yaml.safe_load((OLD/'config_snapshot.yaml').read_text())
    assert cfg['execution']['intrinsic_waypoint_dt_s'] is None
    source=(ROOT/'scripts/isaac/long_continuous_obstacle_reveal_source01.py').read_text()
    assert 'collector.main()' in source and 'collect_episode(' not in source and 'world.reset' not in source
    assert 'collector.ContinuousGeometry = ExploratoryGeometry' in source


@pytest.mark.parametrize('count,n',[(2,4),(5,4),(6,5),(8,5),(10,5)])
def test_four_main_pngs_and_fifth_only_for_useful_source(count,n):
    assert len(figure_names(count))==n


def test_reporting_segments_follow_command_not_lagged_state_identity():
    from report_long_continuous_obstacle_reveal_source01 import segments
    assert list(segments([dict(chunk_id=x) for x in ['','c0','c0','c1','c1']]))==[('',0,1),('c0',1,3),('c1',3,5)]


def test_saved_only_validator_reporter_no_scientific_calls():
    for kind in ['validate','report']:
        tree=ast.parse((ROOT/f'scripts/{kind}_long_continuous_obstacle_reveal_source01.py').read_text())
        names={n.func.id for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)}
        assert not names & {'launch','start','SimulationApp','Worker','solve_graph','solve_least_squares','collect_episode'}


def test_saved_fixture_bundle_count_no_B_for_rejected_and_four_figures(tmp_path):
    import report_long_continuous_obstacle_reveal_source01 as report
    v=deepcopy(read(OLD/'validation.json'));applied=sum(r['applied'] for r in v['chunks'])
    v.update(consecutive_applied_count=applied,longest_applied_prefix=CHUNKS[:applied],usable_applied_handoffs=applied-1,
        usable_post_reveal_handoffs=max(0,applied-2),speed_intervention=authenticate_speed(OLD),speed_audit={},provenance_valid=True)
    (tmp_path/'scenario.json').write_bytes((OLD/'scenario.json').read_bytes())
    (tmp_path/'workers.json').write_bytes((OLD/'workers.json').read_bytes())
    save(tmp_path/'validation.json',v);validator.bundle(tmp_path,v)
    assert validator.validate_bundle(tmp_path,v)['valid']
    assert len(read(tmp_path/'source_bundle/handoff_index.json')['entries'])==applied-1
    assert not (tmp_path/'source_bundle/handoffs/C2_to_C3').exists()
    report.report(tmp_path,tmp_path/'out');assert report.validate_report(tmp_path,tmp_path/'out')['PNG_count']==4


def test_saved_official_memory_audit_matches_historical_path():
    from analyze_join_online02 import jsonlines,csvread
    ep=OLD/'episodes/EPISODE_00'
    events=jsonlines(ep/'controller/events.jsonl')+read(ep/'controller/post_episode_messages.json')['messages']
    audit=validator.audit_controller_speed(events,csvread(ep/'commands.csv'),dict(new_effective_linear_limit_m_s=authenticate_speed(OLD)['base_effective_linear_limit_m_s']))
    assert audit['maximum_applied_abs_v_m_s']==.8


def test_optional_handoff_figure_with_synthetic_six_chunk_plot_fixture(tmp_path,monkeypatch):
    import report_long_continuous_obstacle_reveal_source01 as report
    data=report.inputs(OLD);v=data['validation'];base=deepcopy(v['chunks'][1])
    v['chunks']=[deepcopy(base) for _ in range(6)]
    for i,r in enumerate(v['chunks']):
        r.update(chunk_id=CHUNKS[i],applied=True,switch_state_id=65+10*i)
    v.update(consecutive_applied_count=6,longest_applied_prefix=CHUNKS[:6],usable_applied_handoffs=5,
        usable_post_reveal_handoffs=4,speed_intervention={},speed_audit={},provenance_valid=True)
    save(tmp_path/'validation.json',v);save(tmp_path/'source_bundle/handoff_index.json',dict(entries=[]))
    monkeypatch.setattr(report,'inputs',lambda run:data)
    report.report(tmp_path,tmp_path/'out')
    assert report.validate_report(tmp_path,tmp_path/'out')['PNG_count']==5

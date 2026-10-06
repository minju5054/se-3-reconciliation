"""Synthetic fixtures and saved-only authority checks. No scientific calls."""
import ast
from copy import deepcopy
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
from reconciliation.continuous_obstacle_reveal_episode01 import RequestPolicy, CHUNKS, EPISODES, FIGURES, evolution, separation, classification
from reconciliation.data02_online_successive import observation_anchored_paths
from reconciliation.robotless_online import CommandActivation
from reconciliation.obstacle_source03 import reveal_due
from reconciliation.join_online02 import preview_activation, guard_check
from reconciliation.join_source02 import whole_raw_polyline_check
from reconciliation.join_source03 import read, sha
from continuous_obstacle_reveal_episode01 import NativePolicyWorker
from test_robotless_online import state, result
from test_join_online02 import env
CFG=yaml.safe_load((ROOT/'configs/continuous_obstacle_reveal_episode_01.yaml').read_text())


def frame(tick):
    return dict(frame_id=f'f{tick}',rendered_state_id=tick,capture_sim_time_s=tick/60)


def bootstrap_policy():
    p=RequestPolicy();p.send(frame(45),True,CHUNKS[0]);p.received(CHUNKS[0]);return p


def test_one_episode_four_predictions_no_retry_unchanged_cap():
    assert EPISODES==CFG['episodes']==['EPISODE_00']
    assert CFG['maximum_terminal_predictions']==4 and CFG['maximum_handoff_attempts']==3
    assert CFG['retry'] is False and CFG['maximum_active_sim_s']==4 and CFG['postroll_sim_s']==.1


def test_exact_first_eligible_frames_C1_C2_C3_and_no_C4():
    p=bootstrap_policy()
    for i,tick in enumerate((60,90,120),1):
        active=dict(chunk_id=CHUNKS[i-1]);applied=(tick-6)/60
        assert p.capture(frame(tick),active,applied)==CHUNKS[i]
        assert p.capture(frame(tick+15),active,applied) is None
        assert not p.allow(frame(tick-15)) and not p.allow(frame(tick+15))
        assert p.allow(frame(tick));p.send(frame(tick),True,CHUNKS[i])
        assert p.capture(frame(tick+15),active,applied) is None
        p.received(CHUNKS[i])
    assert p.capture(frame(150),dict(chunk_id=CHUNKS[-1]),2.2) is None
    with pytest.raises(ValueError):p.send(frame(150),True,'chunk_004')


@pytest.mark.parametrize('active,applied,tick',[(None,None,60),(dict(chunk_id=CHUNKS[0]),None,60),(dict(chunk_id=CHUNKS[0]),1.,60)])
def test_capture_requires_strictly_later_actual_application(active,applied,tick):
    p=bootstrap_policy();assert p.capture(frame(tick),active,applied) is None


def test_cannot_buffer_first_frame_or_replace_with_queued_capture():
    p=bootstrap_policy();p.capture(frame(60),dict(chunk_id=CHUNKS[0]),.9)
    with pytest.raises(ValueError):p.send(frame(60),False)
    with pytest.raises(ValueError):p.send(frame(75),True,CHUNKS[1])
    p.send(frame(45),False)


def test_single_inflight_and_strict_order_and_capture_cadence():
    p=RequestPolicy()
    with pytest.raises(ValueError):p.send(frame(45),True,CHUNKS[1])
    with pytest.raises(ValueError):p.send(frame(30),True,CHUNKS[0])
    p.send(frame(45),True,CHUNKS[0])
    with pytest.raises(ValueError):p.send(frame(60),True,CHUNKS[1])
    with pytest.raises(ValueError):p.received(CHUNKS[1])
    with pytest.raises(ValueError):p.capture(frame(61),None,None)


def test_reveal_only_once_after_C0_application():
    assert not reveal_due(60,1.,dict(chunk_id=CHUNKS[0]),1.,False)
    assert reveal_due(60,1.,dict(chunk_id=CHUNKS[0]),.9,False)
    assert not reveal_due(75,1.25,dict(chunk_id=CHUNKS[0]),.9,True)
    assert not reveal_due(75,1.25,dict(chunk_id=CHUNKS[1]),.9,False)


def test_installed_ready_not_B_old_continues_and_versions():
    m=CommandActivation();m.install('c0',0,{});m.accept(result(0),0)
    old,_=m.apply(state(0),None,None)
    m.install('c1',1,{})
    held,event=m.apply(state(1),state(0),old)
    assert event is None and held['chunk_id']=='c0'
    m.accept(result(1),.2);new,event=m.apply(state(2),state(1),held)
    assert event['B']==[3.02,4,.2] and event['B']!=[state(1)[k] for k in ('x','y','yaw')]
    assert event['pre_switch_command']==held and new['chunk_id']=='c1'
    assert not m.accept(result(0),.3)
    with pytest.raises(ValueError):m.install('c2',1,{})


@pytest.mark.parametrize('n',[1,2,7,10,17])
def test_own_observation_anchor_no_B_reanchor_immutable_generic_N(n):
    a=np.column_stack([np.linspace(.1,1,n),np.linspace(0,.4,n),np.linspace(0,.5,n)])
    before=a.tobytes();local,world=observation_anchored_paths(a,[2,3,np.pi/2])
    assert a.tobytes()==before and np.array_equal(local,a)
    np.testing.assert_allclose(world[:,:2],np.column_stack([2-a[:,1],3+a[:,0]]))
    _,different=observation_anchored_paths(a,[3,4,0])
    assert not np.allclose(world,different)


@pytest.mark.parametrize('first,successive,technical,want',[
    (True,True,False,'CONTINUOUS_OBSTACLE_REVEAL_EPISODE_QUALIFIED'),
    (True,False,False,'FIRST_REACTION_ONLY_INSUFFICIENT_SUCCESSIVE_CHUNKS'),
    (True,False,True,'FIRST_REACTION_ONLY_INSUFFICIENT_SUCCESSIVE_CHUNKS'),
    (False,False,True,'TECHNICAL_EXECUTION_BLOCKED'),
    (False,True,False,'CONTINUOUS_CHUNKS_NO_QUALIFIED_FIRST_REACTION')])
def test_classification_does_not_require_later_shape_change(first,successive,technical,want):
    assert classification(first,successive,technical)==want


def test_evolution_thresholds_not_floating_hash_inequality():
    a=np.array([[.1,0,0],[1,0,0.]])
    b=a.copy();b[1,1]=1e-10
    assert not evolution(a,b,CFG['evolution'])['meaningful']
    b[1,1]=.021;assert evolution(a,b,CFG['evolution'])['meaningful']
    b=a.copy();b[1,2]=np.radians(5.01)
    assert evolution(a,b,CFG['evolution'])['meaningful']
    b=a.copy();b[:,2]=2*np.pi
    assert not evolution(a,b,CFG['evolution'])['meaningful']


def test_arbitrary_row_counts_have_continuous_geometry_descriptor():
    a=np.array([[0,0,0],[1,0,0]])
    b=np.array([[0,0,0],[.2,0,0],[.8,0,0],[1,0,0]])
    assert separation(a,b)==0 and not evolution(a,b,CFG['evolution'])['meaningful']
    b[1,1]=.1;assert separation(a,b)==pytest.approx(.1)
    assert separation([[0,0,0]],[[1,0,0]])==1


class FakeWorker:
    def __init__(self,messages=()):self.messages=list(messages);self.sent=[]
    def send(self,op,**payload):self.sent.append((op,payload));return len(self.sent)
    def drain(self):r=self.messages;self.messages=[];return r


@pytest.mark.parametrize('safe',[True,False])
def test_raw_reference_policy_never_modifies_response_or_installs_unsafe(tmp_path,safe):
    p=bootstrap_policy();p.capture(frame(60),dict(chunk_id=CHUNKS[0]),.9);p.send(frame(60),True,CHUNKS[1])
    raw=dict(type='result',kind='prediction',chunk_id=CHUNKS[1],status='PREDICTION_READY',
        world=[[0,0,0],[1,0,0]],raw_local_ref={'sha256':'raw'},world_ref={'sha256':'world'})
    original=deepcopy(raw);worker=FakeWorker([raw])
    geo=SimpleNamespace(present=True,ask=lambda *a,**kw:{'clearance_valid':safe})
    proxy=NativePolicyWorker(worker,SimpleNamespace(policy=p),geo,tmp_path)
    r=proxy.drain()[0]
    assert raw==original and r['world']==raw['world'] and not worker.sent
    assert r['status']==('PREDICTION_READY' if safe else 'SCENE_INVALID')
    assert (tmp_path/'raw_unsafe_abort.json').exists() != safe


def test_wrapper_cannot_reopen_session(tmp_path):
    proxy=NativePolicyWorker(FakeWorker(),SimpleNamespace(policy=RequestPolicy()),None,tmp_path)
    proxy.send('open')
    with pytest.raises(ValueError):proxy.send('open')


def test_abort_guard_preserves_prefix_and_raw_check_adds_no_connector():
    from shapely.geometry import box
    environment=env(box(.32,-.2,.7,.2))
    m=CommandActivation();m.install('c0',0,{});m.accept(result(0,(.8,0)),0)
    s=state(0);s.update(x=0,y=0,yaw=0)
    before=deepcopy(m.__dict__)
    proposed,cmd,event=preview_activation(m,s,None,None)
    assert not guard_check(environment,[0,0,0],cmd,1/60)['safe']
    assert m.__dict__==before and m.active is None
    safe=whole_raw_polyline_check([[1,0,0],[2,0,0]],environment)
    assert safe['clearance_valid'] and not safe['connector_included']


def test_historical_source_authentication_no_runtime_edits():
    source=ROOT/CFG['authority']
    for name,h in read(source/'freeze.json')['source_sha256'].items():assert sha(ROOT/name)==h,name
    from run_join_source05 import verify
    verify(source)


def test_default_native_collector_mpc_pacing_and_null_waypoint_dt():
    source=ROOT/CFG['authority'];cfg=yaml.safe_load((source/'config_snapshot.yaml').read_text())
    assert cfg['execution']['intrinsic_waypoint_dt_s'] is None
    assert cfg['execution']['pacing_policy']=='minimum_wall_step_no_catchup_v1'
    assert [cfg['execution'][k] for k in ('integration_hz','control_hz','capture_hz')]==[60,10,4]
    src=(ROOT/'scripts/isaac/continuous_obstacle_reveal_episode01.py').read_text()
    assert 'from robotless_online_handoffs import Worker, collect_episode' in src
    assert 'command_guard=geo.guard' in src and 'world.reset()' not in src
    assert 'time.sleep' not in src and "set_present(prop, False)" in src


def test_saved_validation_and_report_have_no_scientific_entrypoints():
    for name in ('validate','report'):
        text=(ROOT/f'scripts/{name}_continuous_obstacle_reveal_episode01.py').read_text()
        calls={n.func.id for n in ast.walk(ast.parse(text)) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)}
        assert not calls.intersection({'collect_episode','Worker','OfficialMpcAdapter','solve','optimize','minimize','SimulationApp'})
    assert len(FIGURES)==len(set(FIGURES))==4


def test_four_PNG_report_with_censored_chunks(tmp_path,monkeypatch):
    import report_continuous_obstacle_reveal_episode01 as report
    from shapely.geometry import box
    v=dict(chunks=[dict(chunk_id=c,generated=False,applied=False) for c in CHUNKS],
        episode=str(tmp_path),reveal=None,evolution_classification=None)
    data=dict(validation=v,execution_world=[[0,0,0],[.1,0,0]],cart_wkb_hex=box(2,2,3,3).wkb_hex,
        execution_sim_s=[0,.1],active_chunk=['',''],request_seen=[])
    monkeypatch.setattr(report,'inputs',lambda run:data)
    monkeypatch.setattr(report,'compact',lambda v:{'synthetic':True})
    report.report(tmp_path,tmp_path/'report')
    assert sorted(p.name for p in (tmp_path/'report/figures').glob('*.png'))==sorted(FIGURES)
    (tmp_path/'validation.json').write_text(json.dumps(v))
    assert report.validate_report(tmp_path,tmp_path/'report')['valid']


def test_saved_historical_partial_episode_validator_and_report(tmp_path,monkeypatch):
    """Historical bytes used as a partial reporting fixture, never new evidence."""
    import shutil
    import validate_continuous_obstacle_reveal_episode01 as validator
    import report_continuous_obstacle_reveal_episode01 as reporter
    from reconciliation.join_source03 import save
    source=ROOT/CFG['authority'];run=tmp_path/'fixture';run.mkdir()
    ep=run/'episodes/REPEAT_00';shutil.copytree(source/'episodes/REPEAT_00',ep)
    for name in ('scenario.json','generation_actual.json','execution_start.json'):
        shutil.copyfile(source/name,run/name)
    (run/'logs').mkdir();shutil.copyfile(source/'logs/server.log',run/'logs/server.log')
    save(run/'source.json',dict(OSA03=str(source),starting_sha='historical_fixture'))
    cfg=yaml.safe_load((source/'config_snapshot.yaml').read_text());cfg['online']['maximum_handoff_attempts']=3
    (run/'config_snapshot.yaml').write_text(yaml.safe_dump(cfg))
    p=read(source/'protocol.json');p['declaration']=CFG;save(run/'protocol.json',p)
    old=read(source/'validation.json')['rows'][0]
    for i,label in enumerate(('OLD','FRESH')):
        r=old[label];save(ep/'reference_checks'/f'{CHUNKS[i]}.json',dict(
            check=r['geometry_on' if i else 'geometry_off'],before_install=True,cart_present=bool(i)))
    save(ep/'eligible_observations/chunk_001.json',dict(frame=old['FRESH']['observation'],terminal_inflight=None,cart_present=True))
    for line in (ep/'visibility.jsonl').read_text().splitlines():
        v=json.loads(line);save(ep/'cart_states'/f'{v["frame_id"]}.json',dict(
            renderer_present=v['cart_present'],oracle_present=v['cart_present'],actual_world_matrix_column=v['cart_transform']))
    monkeypatch.setattr(validator,'verify',lambda run:None)
    monkeypatch.setattr(validator,'EPISODES',['REPEAT_00'])
    v=validator.validate(run)
    assert v['classification']=='FIRST_REACTION_ONLY_INSUFFICIENT_SUCCESSIVE_CHUNKS'
    assert [r['applied'] for r in v['chunks']]==[True,True,False,False]
    save(run/'validation.json',v);validator.bundle(run,v)
    reporter.report(run,run/'report')
    assert reporter.validate_report(run,run/'report')['valid']

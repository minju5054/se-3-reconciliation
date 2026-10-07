"""Synthetic / saved-only checks. No LightNav, MPC solve or Isaac launches."""
import ast
from copy import deepcopy
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
from reconciliation.successive_native_source_acquisition02 import RequestPolicy,CHUNKS,classify,next_candidate,figure_names
from reconciliation.join_source03 import read,save,sha
from reconciliation.long_source_mpc01 import authenticate_speed
from test_continuous_obstacle_reveal_episode01 import frame,FakeWorker,NativePolicyWorker
from test_continuous_obstacle_reveal_exploratory02 import wrapped
from reconciliation.join_source02 import whole_raw_polyline_check
import run_successive_native_source_acquisition02 as runner
import validate_successive_native_source_acquisition02 as validator
OLD=ROOT/runner.declaration()['historical_run']
BATCH=ROOT/'data/successive_native_source_acquisition_02/primary_20261007'

def test_twelve_actual_first_frames_one_inflight_no_C12():
    p=RequestPolicy();p.send(frame(45),True,CHUNKS[0]);p.received(CHUNKS[0])
    for i in range(1,12):
        tick=45+i*30;active=dict(chunk_id=CHUNKS[i-1]);applied=(tick-6)/60
        assert p.capture(frame(tick),active,applied)==CHUNKS[i]
        assert p.capture(frame(tick+15),active,applied) is None
        assert not p.allow(frame(tick+15))
        p.send(frame(tick),True,CHUNKS[i])
        with pytest.raises(ValueError):p.send(frame(tick),True,CHUNKS[i])
        assert p.capture(frame(tick+15),active,applied) is None
        p.received(CHUNKS[i])
    assert p.capture(frame(420),dict(chunk_id=CHUNKS[-1]),6.9) is None
    with pytest.raises(ValueError):p.send(frame(420),True,'chunk_012')

@pytest.mark.parametrize('n,expect',[(0,'NO_USABLE_SUCCESSIVE_SOURCE'),(1,'NO_USABLE_SUCCESSIVE_SOURCE'),(2,'PARTIAL_SUCCESSIVE_SOURCE'),(5,'PARTIAL_SUCCESSIVE_SOURCE'),(6,'SUCCESSIVE_SOURCE_MINIMUM_REACHED'),(7,'SUCCESSIVE_SOURCE_MINIMUM_REACHED'),(8,'SUCCESSIVE_SOURCE_TARGET_REACHED'),(11,'SUCCESSIVE_SOURCE_TARGET_REACHED'),(12,'SUCCESSIVE_SOURCE_MAX_REACHED')])
def test_classification(n,expect):
    assert classify(n,True,True)==expect
    assert classify(n,True,False)=='TECHNICAL_EXECUTION_BLOCKED'
    assert classify(n,True,True,True)=='TECHNICAL_EXECUTION_BLOCKED'
    if n<6: assert classify(n,True,True,model_stop=True)=='MODEL_STOP_BEFORE_MINIMUM'
    if n>=6: assert classify(n,False,True)=='NO_USABLE_SUCCESSIVE_SOURCE'

@pytest.mark.parametrize('n',[2,6,8,12])
def test_lateral_sign_not_selection_gate(n):
    from reconciliation.continuous_obstacle_reveal_episode01 import evolution
    threshold=dict(separation_m=.02,endpoint_lateral_m=.02,yaw_deg=5.)
    # Both signed paths are physically identical under reflected geometry; the
    # qualification API deliberately accepts only provenance, visibility, count.
    positive=np.array([[0,0,0],[.2,.1,.2]]);negative=positive*np.array([1,-1,-1])
    assert positive[-1,1]>0 and negative[-1,1]<0
    results=[classify(n,True,True) for path in (positive,negative)]
    assert results[0]==results[1]
    tree=ast.parse((ROOT/'src/reconciliation/successive_native_source_acquisition02.py').read_text())
    f=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='classify')
    assert not {'lateral','side','endpoint','raw','direction'} & {n.id for n in ast.walk(f) if isinstance(n,ast.Name)}

def test_search_stops_first_qualified_maximum_three():
    manifest=[dict(candidate_id=f'C{i}',geometry_valid=True) for i in range(3)]
    complete={}
    for i in range(3):
        assert next_candidate(manifest,complete)==f'C{i}'
        complete[f'C{i}']=dict(classification='PARTIAL_SUCCESSIVE_SOURCE')
    assert next_candidate(manifest,complete) is None
    complete['C0']['classification']='SUCCESSIVE_SOURCE_MINIMUM_REACHED'
    assert next_candidate(manifest,complete) is None
    manifest[0]['geometry_valid']=False
    assert next_candidate(manifest,{})=='C1'

def test_failed_candidate_immutable(tmp_path,monkeypatch):
    first=tmp_path/'CANDIDATE_01';second=tmp_path/'CANDIDATE_02';first.mkdir();second.mkdir()
    save(first/'attempt_validation.json',dict(classification='PARTIAL_SUCCESSIVE_SOURCE'))
    f=first/'raw';f.write_text('immutable')
    save(first/'candidate_seal.json',dict(files={str(f):sha(f)}))
    save(second/'candidate.json',dict(geometry_valid=True))
    out=tmp_path/'out';save(out/'candidate_manifest.json',[dict(candidate_id=p.name,geometry_valid=True) for p in (first,second)])
    monkeypatch.setattr(runner,'OUT',out);runner.search_gate(second)
    f.write_text('changed')
    with pytest.raises(AssertionError):runner.search_gate(second)

@pytest.mark.parametrize('clearance,accepted',[(.008,True),(0,False),(-.001,False)])
def test_rejected_raw_no_install_B_or_next_prediction(tmp_path,clearance,accepted):
    p=RequestPolicy();p.send(frame(45),True,CHUNKS[0]);p.received(CHUNKS[0])
    p.capture(frame(60),dict(chunk_id=CHUNKS[0]),.9);p.send(frame(60),True,CHUNKS[1])
    raw=dict(type='result',kind='prediction',chunk_id=CHUNKS[1],status='PREDICTION_READY',world=[[0,0,0],[.3,0,0]],raw_local_ref={},world_ref={})
    before=deepcopy(raw)
    proxy=NativePolicyWorker(FakeWorker([raw]),SimpleNamespace(policy=p),SimpleNamespace(present=True,ask=lambda op,world:whole_raw_polyline_check(world,wrapped(clearance))),tmp_path)
    result=proxy.drain()[0];assert raw==before
    assert (result['status']=='PREDICTION_READY')==accepted
    if not accepted:
        assert 'B' not in result and not p.allow(frame(75))
        assert p.capture(frame(75),dict(chunk_id=CHUNKS[0]),.9) is None
        with pytest.raises(ValueError):p.send(frame(75),True,CHUNKS[2])

def test_STOP_no_fake_trajectory(tmp_path):
    p=RequestPolicy();p.send(frame(45),True,CHUNKS[0])
    stop=dict(type='result',kind='prediction',chunk_id=CHUNKS[0],status='MODEL_STOP',stop=True)
    proxy=NativePolicyWorker(FakeWorker([stop]),SimpleNamespace(policy=p),SimpleNamespace(present=False),tmp_path)
    assert proxy.drain()==[stop]
    assert not p.allow(frame(60)) and not list(tmp_path.glob('reference_checks/*'))

def test_pose_geometry_speed_cap_external_authentication():
    manifest=read(runner.OUT/'candidate_manifest.json')
    assert manifest==runner.candidates(OLD)
    for c in manifest:
        t=np.asarray(c['original_POSE11']);expected=t.copy();expected[:2]-=c['backward_offset_m']*np.array([np.cos(t[2]),np.sin(t[2])])
        np.testing.assert_array_equal(expected,c['pose_world']);assert c['geometry_valid']
        assert runner.equivalence(BATCH/c['candidate_id'])['valid']
        assert (BATCH/c['candidate_id']/'speed_intervention.json').read_bytes()==(OLD/'speed_intervention.json').read_bytes()
    cfg=runner.declaration();assert cfg['maximum_terminal_predictions']==12 and cfg['maximum_handoff_attempts']==11
    assert cfg['maximum_active_sim_s']==10 and cfg['maximum_launches']==3 and not cfg['retry']
    for name,h in read(OLD/'freeze.json')['source_sha256'].items():assert sha(ROOT/name)==h,name
    speed=read(OLD/'speed_intervention.json');assert speed==authenticate_speed(Path(speed['prior_workers']).parent)
    assert speed['new_effective_linear_limit_m_s']==.4 and sha(speed['external_source'])==speed['external_source_sha256']

def test_actual_camera_checked_before_workers(tmp_path,monkeypatch):
    spec=importlib.util.spec_from_file_location('successive_collector_test',ROOT/'scripts/isaac/successive_native_source_acquisition02.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    c=read(runner.OUT/'candidate_manifest.json')[0];save(tmp_path/'candidate.json',c)
    result=tuple(range(7))+(dict(camera={k:c[k] for k in ('T_world_agent','T_world_camera','T_agent_camera')}),)
    monkeypatch.setattr(module,'_original_setup',lambda *args:result)
    assert module.setup_scene(tmp_path,{},c['pose_world'],None)==result
    assert read(tmp_path/'candidate_scene_validation.json')['before_model_requests']

def test_speed_memory_and_applied_command_exact():
    e=[dict(status='reset'),dict(status='submitted',previous_command=[0,0]),dict(type='solve_result',status='command',solve_id='s',command=[.4,.2]),dict(status='submitted',previous_command=[.4,.2])]
    c=[dict(v_mps=.4,omega_radps=.2,solve_id='s',reason='new_solve')]
    assert validator.audit_controller_speed(e,c,dict(new_effective_linear_limit_m_s=.4))['memory_checks']==2
    c[0]['v_mps']=.2
    with pytest.raises(AssertionError):validator.audit_controller_speed(e,c,dict(new_effective_linear_limit_m_s=.4))

def test_saved_only_validator_reporter_no_new_science():
    for name in ('validate','report'):
        t=ast.parse((ROOT/f'scripts/{name}_successive_native_source_acquisition02.py').read_text())
        names={n.func.id for n in ast.walk(t) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)}
        assert not names & {'launch','start','SimulationApp','Worker','solve_graph','collect_episode'}

@pytest.mark.parametrize('count',[2,6,12])
def test_four_PNGs_arbitrary_length_fixture(tmp_path,monkeypatch,count):
    import report_successive_native_source_acquisition02 as report
    data=report.inputs(OLD);v=data['validation'];base=deepcopy(v['chunks'][1]);v['chunks']=[deepcopy(base) for _ in range(count)]
    for i,r in enumerate(v['chunks']):r.update(chunk_id=CHUNKS[i],applied=True,switch_state_id=30+5*i)
    v.update(consecutive_applied_count=count,longest_applied_prefix=CHUNKS[:count],usable_applied_handoffs=count-1,
        usable_post_reveal_handoffs=max(0,count-2),candidate_id='SYNTHETIC',reaction_onset=None,
        qualification_cart_visible=True,qualification_direction_gate=False,source_type='SYNTHETIC TEST ONLY')
    save(tmp_path/'validation.json',v);save(tmp_path/'source_bundle/handoff_index.json',dict(entries=[]))
    monkeypatch.setattr(report,'inputs',lambda run:data)
    report.report(tmp_path,tmp_path/'out');assert report.validate_report(tmp_path,tmp_path/'out')['PNG_count']==4
    assert len(figure_names(count))==4

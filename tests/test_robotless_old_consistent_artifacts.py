"""Synthetic/mock protocol, invalid-case retention and artifact integrity tests."""
import copy
import json
from pathlib import Path
import sys
import importlib.util
import types

import numpy as np
import pytest
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'));sys.path.insert(0,str(ROOT/'scripts/lightnav'))
import robotless_old_consistent_artifacts as a
import robotless_old_consistent_inference as batch
import validate_robotless_old_consistent as validator
from reconciliation.robotless_projection_handoff import projection_geometry
from reconciliation.robotless_single_chunk import observation_to_world
from robotless_successive_inference import exchange_successive_frames
from test_robotless_successive_client import FakeSocket, FakeConnector


@pytest.fixture
def frozen(tmp_path,monkeypatch):
    source=tmp_path/'source';source.mkdir();selection=tmp_path/'selection';selection.mkdir()
    base=yaml.safe_load((ROOT/'configs/robotless_handoff_screening.yaml').read_text())
    (source/'config_snapshot.yaml').write_text(yaml.safe_dump(base))
    (source/'manifest_frozen.json').write_text('{"synthetic_only":true}')
    old=np.array([[.1,0,0],[.5,0,0],[1.,0,0]])
    for episode_id in a.CASE_IDS:
        d=source/'episodes'/episode_id;(d/'raw').mkdir(parents=True);(d/'derived').mkdir()
        np.save(d/'raw/chunk_000.npy',old);np.save(d/'derived/chunk_000_world.npy',old)
        (d/'raw/observation_000.jpg').write_bytes(b'synthetic RGB0')
        (d/'metadata.json').write_text(json.dumps({'R0':[0,0,0],'R1':[.3,0,0],'category':'synthetic','instruction':'synthetic fixed instruction'}))
        p=projection_geometry([.3,0,0],old);p['tau_s']=0.
        (d/'derived/projection_metrics.json').write_text(json.dumps({'conditions':[p]}))
    pilot=yaml.safe_load((ROOT/'configs/robotless_old_consistent_observation.yaml').read_text())
    pilot.update(source_run=str(source),selection_run=str(selection));path=tmp_path/'pilot.yaml';path.write_text(yaml.safe_dump(pilot))
    monkeypatch.setattr(a,'validate_screening',lambda p:{'status':a.SCREENING_VALIDATED,'synthetic_only':True})
    monkeypatch.setattr(a,'validate_selection',lambda p:{'status':a.SELECTION_VALIDATED,'synthetic_only':True})
    monkeypatch.setattr(a,'file_record',lambda p,r:{'path':str(p.relative_to(r)) if p.is_relative_to(r) else str(p),'sha256':a.sha256_file(p),'bytes':p.stat().st_size})
    run=tmp_path/'run';a.freeze(path,run)
    return run,source,path


def prepare(run):
    (run/'server_process.json').write_text('{"synthetic_only":true}')
    (run/'capture_batch.json').write_text(json.dumps({'completed_time':{'monotonic_ns':1}}))
    for episode_id in a.CASE_IDS:
        d=run/'episodes'/episode_id;(d/'raw').mkdir(parents=True);(d/'derived').mkdir()
        (d/'capture_status.json').write_text(json.dumps({'capture_valid':True,'status':'CAPTURE_VALID','reason':None}))
        (d/'metadata.json').write_text(json.dumps({'R0':[0,0,0],'R1':[.3,0,0]}))


def inference(directory,timeout):
    old=np.array([[.1,0,0],[.5,0,0],[1.,0,0]]);fresh=np.array([[.1,0,.1],[.4,0,.1]])
    for seq,local in enumerate((old,fresh)):
        np.save(directory/f'raw/chunk_{seq:03d}.npy',local)
        np.save(directory/f'derived/chunk_{seq:03d}_world.npy',observation_to_world([.3*seq,0,0],local))
    result={'session':{'connection_id':directory.name},'chunks':[{'stop':False},{'stop':False}]}
    (directory/'inference_metadata.json').write_text(json.dumps(result))
    return result


def test_freeze_exact_cases_and_no_overwrite(frozen):
    run,source,path=frozen
    _,m,_,digest=a.load_frozen(run)
    assert [e['episode_id'] for e in m['episodes']]==list(a.CASE_IDS)
    assert digest==a.sha256_file(run/'manifest_frozen.json')
    with pytest.raises(FileExistsError):a.freeze(path,run)


@pytest.mark.parametrize('relative',['manifest_frozen.json','config_snapshot.yaml','source.json'])
def test_mutated_frozen_input_stops_before_request(frozen,relative):
    run,_,_=frozen
    with (run/relative).open('a') as f:f.write(' ')
    calls=[]
    with pytest.raises(ValueError):batch.collect(run,inference_fn=lambda *args:calls.append(args))
    assert not calls and not (run/'inference_started.json').exists()


def test_source_bytes_unchanged_guard(frozen):
    run,source,_=frozen
    with (source/'episodes/episode_006/raw/chunk_000.npy').open('ab') as f:f.write(b'corrupt')
    with pytest.raises(ValueError,match='source file'):a.load_frozen(run)


def test_all_cases_retained_mismatch_excluded_even_first_csv_row(frozen):
    run,source,_=frozen;prepare(run);calls=[]
    def fake(d,timeout):
        calls.append(d.name);result=inference(d,timeout)
        if d.name=='episode_006':
            old=np.load(d/'raw/chunk_000.npy');old[0,0]=np.nextafter(old[0,0],1.)
            np.save(d/'raw/chunk_000.npy',old)
        return result
    statuses=batch.collect(run,inference_fn=fake)
    assert calls==list(a.CASE_IDS) and statuses[0]['status']=='PLANNING_OLD_MISMATCH'
    summary=a.analyze(run)
    assert summary['valid_count']==4 and summary['invalid_count']==1
    assert summary['case_results'][0]['metrics'] is None
    assert summary['case_results'][0]['new_FRESH_hash'] is not None
    assert len((run/'aggregate/case_table.csv').read_text().splitlines())==6
    with pytest.raises(FileExistsError):batch.collect(run,inference_fn=fake)


def test_insufficient_capture_and_model_failures_preserved(frozen):
    run,_,_=frozen;prepare(run)
    (run/'episodes/episode_007/capture_status.json').write_text(json.dumps({'capture_valid':False,'status':'OLD_ARC_INSUFFICIENT','reason':'synthetic length不足'}))
    calls=[]
    def fake(d,timeout):
        calls.append(d.name)
        if d.name=='episode_013':raise ValueError('synthetic nonfinite output')
        result=inference(d,timeout)
        if d.name=='episode_016':result['chunks'][0]['stop']=True
        return result
    statuses=batch.collect(run,inference_fn=fake)
    assert len(statuses)==5 and len(calls)==4
    assert [s['status'] for s in statuses]==['VALID_PAIR','OLD_ARC_INSUFFICIENT','INFERENCE_FAILED','MODEL_STOP','VALID_PAIR']
    summary=a.analyze(run);assert summary['valid_count']==2 and summary['invalid_count']==3


def test_five_independent_exact_sessions(tmp_path):
    ids=[]
    for episode_id in a.CASE_IDS:
        d=tmp_path/episode_id;d.mkdir();socket=FakeSocket();connector=FakeConnector(socket)
        _,result=exchange_successive_frames('ws://synthetic.invalid',d,[b'original RGB0',b'new RGB1'],[[480,270]]*2,'unchanged',30,connect_fn=connector)
        req=[json.loads(x) for x in socket.sent]
        assert [r['action'] for r in req]==['login','reset','next','next']
        assert [r['data']['seq'] for r in req[2:]]==[0,1]
        assert [r['data']['instruction'] for r in req[2:]]==['unchanged']*2
        assert socket.enters==socket.exits==len(connector.calls)==1
        ids.append(result['session']['connection_id'])
    assert len(set(ids))==5


def test_missing_runtime_never_validates(tmp_path):
    result=validator.validate_run(tmp_path)
    assert result['status']==validator.NOT_VALIDATED and 'visual_review.json' in result['missing_artifacts']


@pytest.mark.parametrize('name',['manifest_frozen.json','source.json','config_snapshot.yaml'])
def test_corrupt_frozen_artifact_fails_even_when_runtime_missing(frozen,name):
    run,_,_=frozen
    with (run/name).open('a') as f:f.write(' ')
    assert validator.validate_run(run)['status']==validator.FAILED


def test_executed_source_snapshot_guard(tmp_path,monkeypatch):
    source=tmp_path/'code.py';source.write_text('original executed code')
    digest=a.sha256_file(source);monkeypatch.setattr(a,'ROOT',tmp_path)
    validator.check_sources(tmp_path,{'code.py':digest})
    source.write_text('changed code')
    with pytest.raises(ValueError,match='preserved executed snapshot'):validator.check_sources(tmp_path,{'code.py':digest})
    history=tmp_path/'processing_source_history';history.mkdir()
    (history/f'{digest}.py').write_text('original executed code')
    validator.check_sources(tmp_path,{'code.py':digest})
    (history/f'{digest}.py').write_text('wrong snapshot')
    with pytest.raises(ValueError):validator.check_sources(tmp_path,{'code.py':digest})


def test_double_usd_pose_matrix_and_repeated_assignment(monkeypatch):
    spec=importlib.util.spec_from_file_location('pilot_isaac',ROOT/'scripts/isaac/robotless_old_consistent_observation.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    class Op:
        def Set(self,value):self.value=value
    class Agent:
        def __init__(self):self.op=None;self.order=None
        def GetPrim(self):return self
        def GetAttribute(self,name):return self.op
        def AddTransformOp(self,*args):self.op=Op();return self.op
        def SetXformOpOrder(self,order):self.order=order
    class XformOp:
        PrecisionDouble='double'
        def __new__(cls,op):return op
    fake=types.SimpleNamespace(Gf=types.SimpleNamespace(Matrix4d=lambda *x:np.array(x).reshape(4,4)),
        UsdGeom=types.SimpleNamespace(Xformable=lambda x:x,XformOp=XformOp))
    monkeypatch.setitem(sys.modules,'pxr',fake)
    agent=Agent()
    for yaw in [2.6178131640565683,-2.888158697503187]:
        module.set_precise_agent_pose(agent,[9.3,7.05,yaw],z_m=0.)
        col=agent.op.value.T
        assert np.arctan2(col[1,0],col[0,0])==yaw
        np.testing.assert_array_equal(col[:3,3],[9.3,7.05,0.])
        assert agent.order==[agent.op]
    with pytest.raises(ValueError):module.set_precise_agent_pose(agent,[np.nan,0,0],z_m=0)

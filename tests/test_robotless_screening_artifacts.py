"""Synthetic bank/session plumbing. No actual Isaac or model evidence."""
import copy
import json
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
sys.path.insert(0,str(ROOT/'scripts/lightnav'))
import robotless_screening_artifacts as artifacts
import robotless_screening_inference as batch
from reconciliation.robotless_single_chunk import observation_to_world, sha256_file
from test_robotless_successive_client import FakeSocket, FakeConnector
from robotless_successive_inference import exchange_successive_frames


@pytest.fixture
def frozen(tmp_path):
    config=yaml.safe_load((ROOT/'configs/robotless_handoff_screening.yaml').read_text())
    path=tmp_path/'config.yaml'; path.write_text(yaml.safe_dump(config))
    run=tmp_path/'synthetic_run'; (run/'preparation').mkdir(parents=True)
    (run/'preparation/geometry_review.json').write_text(json.dumps({'reviewed_before_inference':True,'lightnav_inference_count':0,'synthetic_only':True,'bank_config_sha256':sha256_file(path)}))
    artifacts.freeze_bank(path,run)
    return run,path,config


def test_freeze_hash_and_overwrite_protection(frozen):
    run,path,config=frozen
    _,manifest,digest=artifacts.load_frozen(run)
    assert len(manifest['episodes'])==30
    assert digest==sha256_file(run/'manifest_frozen.json')
    with pytest.raises(FileExistsError): artifacts.freeze_bank(path,run)


@pytest.mark.parametrize('name',['manifest_frozen.json','config_snapshot.yaml','preparation/geometry_review.json'])
def test_mutation_after_freeze_stops_before_inference(frozen,name):
    run,_,_=frozen
    with (run/name).open('a') as stream: stream.write(' ')
    calls=[]
    with pytest.raises(ValueError): batch.collect(run,inference_fn=lambda *a:calls.append(a))
    assert calls==[] and not (run/'inference_batch_started.json').exists()


def prepare_synthetic_captures(run,config):
    (run/'server_process.json').write_text('{"synthetic_only":true}')
    for episode in config['episodes']:
        directory=run/'episodes'/episode['episode_id']
        (directory/'raw').mkdir(parents=True);(directory/'derived').mkdir()
        (directory/'capture_status.json').write_text('{"capture_valid":true}')
        (directory/'metadata.json').write_text(json.dumps({'R1':episode['R1']}))


def synthetic_inference(directory,timeout):
    metadata=json.loads((directory/'metadata.json').read_text())
    old=np.array([[0,0,0],[.1,0,0]])
    fresh=np.array([[.15,0,0],[.5,0,.1],[1,0,.2]])
    np.save(directory/'raw/chunk_000.npy',old)
    np.save(directory/'raw/chunk_001.npy',fresh)
    np.save(directory/'derived/chunk_001_world.npy',observation_to_world(metadata['R1'],fresh))
    result={'session':{'connection_id':directory.name},'chunks':[{'stop':False},{'stop':False}]}
    (directory/'inference_metadata.json').write_text(json.dumps(result))
    return result


def test_thirty_attempts_invalid_preserved_no_replacement_and_no_overwrite(frozen):
    run,_,config=frozen;prepare_synthetic_captures(run,config);calls=[]
    (run/'episodes/episode_008/capture_status.json').write_text('{"capture_valid":false,"reason":"synthetic blank capture"}')
    def inference(directory,timeout):
        calls.append(directory.name)
        _,manifest,_=artifacts.load_frozen(run)
        assert manifest['frozen_time']['monotonic_ns']<batch.host_event()['monotonic_ns']
        if directory.name=='episode_004': raise ValueError('output must be finite')
        result=synthetic_inference(directory,timeout)
        if directory.name=='episode_006': result['chunks'][0]['stop']=True
        return result
    statuses=batch.collect(run,inference_fn=inference)
    assert len(statuses)==30 and len(calls)==29
    assert statuses[4]['status']=='NONFINITE_OUTPUT'
    assert statuses[6]['status']=='OLD_STOP'
    assert statuses[8]['status']=='CAPTURE_INVALID'
    assert sum(s['status']=='VALID_PAIR' for s in statuses)==27
    assert [s['episode_id'] for s in statuses]==[e['episode_id'] for e in config['episodes']]
    raw=run/'episodes/episode_000/raw/chunk_001.npy';before=sha256_file(raw)
    with pytest.raises(FileExistsError): batch.collect(run,inference_fn=inference)
    assert sha256_file(raw)==before


def test_each_episode_has_new_connection_one_reset_constant_instruction(tmp_path):
    sessions=[]
    for i in range(30):
        directory=tmp_path/f'episode_{i:03d}';directory.mkdir()
        socket=FakeSocket();connector=FakeConnector(socket)
        _,result=exchange_successive_frames('ws://synthetic.invalid',directory,[b'RGB0',b'RGB1'],[[480,270]]*2,
            f'instruction {i}',30.,connect_fn=connector)
        requests=[json.loads(x) for x in socket.sent]
        assert [r['action'] for r in requests]==['login','reset','next','next']
        assert [r['data']['seq'] for r in requests[2:]]==[0,1]
        assert [r['data']['instruction'] for r in requests[2:]]==[f'instruction {i}']*2
        assert socket.enters==socket.exits==len(connector.calls)==1
        sessions.append(result['session']['connection_id'])
    assert len(set(sessions))==30

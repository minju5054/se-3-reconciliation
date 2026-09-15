"""Read-only source inventory and shared OLD-conditioned derivation."""
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from reconciliation.robotless_single_chunk import load_config, save_json_exclusive, sha256_file, make_observation_time
from reconciliation.robotless_old_conditioned_handoff import characterize_episode, validate_config
from robotless_screening_artifacts import read_json, file_record, verify_file, git, phase_sources
from validate_robotless_handoff_screening import validate_run as validate_screening, VALIDATED as SCREENING_VALIDATED


def processing_sources():
    return phase_sources(['src/reconciliation/robotless_old_conditioned_handoff.py',
        'scripts/robotless_old_conditioned_artifacts.py','scripts/characterize_robotless_old_conditioned.py'])


def inventory(source):
    return [file_record(p,source) for p in sorted(source.rglob('*')) if p.is_file()]


def initialize(config_path,run,*,validate_source_fn=validate_screening):
    config=load_config(config_path);validate_config(config)
    source=(ROOT/config['source_run']).resolve()
    if run.resolve().is_relative_to(source):
        raise ValueError('output cannot be inside the frozen source')
    result=validate_source_fn(source)
    if result['status']!=SCREENING_VALIDATED:
        raise ValueError('source screening validator did not pass')
    run.mkdir(parents=True,exist_ok=False)
    for name in ('derived','evidence','logs'): (run/name).mkdir()
    with (run/'config_snapshot.yaml').open('xb') as stream:stream.write(config_path.read_bytes())
    save_json_exclusive(run/'source.json',{'source_run':str(source),'source_validation':result,
        'files':inventory(source),'source_manifest_sha256':sha256_file(source/'manifest_frozen.json'),
        'configuration':file_record(run/'config_snapshot.yaml',run),'created_time':make_observation_time(0),
        'starting_git_sha':git('rev-parse','HEAD'),'source_validation_scope':'read-only full existing screening validator',
        'new_inference_readiness_time':None,'actual_execution_time':None,'new_lightnav_inference_count':0,
        'motion_convention':'counterfactual OLD-conditioned geometry-based execution surrogate; not actual OLD execution'})


def load_inputs(run):
    record=read_json(run/'source.json');source=Path(record['source_run'])
    if record['source_validation']['status']!=SCREENING_VALIDATED or not source.is_absolute():
        raise ValueError('source is not a validated absolute screening run')
    config=load_config(run/'config_snapshot.yaml');validate_config(config)
    verify_file(run,record['configuration'])
    if source!=(ROOT/config['source_run']).resolve():raise ValueError('source path changed')
    if record['files']!=inventory(source):raise ValueError('source file set or bytes changed')
    if sha256_file(source/'manifest_frozen.json')!=record['source_manifest_sha256']:raise ValueError('source manifest changed')
    manifest=read_json(source/'manifest_frozen.json')
    if len(manifest['episodes'])!=30 or len({e['episode_id'] for e in manifest['episodes']})!=30:
        raise ValueError('all 30 source episodes must be retained')
    return config,record,source,manifest


def derive(config,source,manifest):
    results=[]
    for episode in manifest['episodes']:
        location=source/'episodes'/episode['episode_id']
        metadata=read_json(location/'metadata.json')
        old=np.load(location/'derived/chunk_000_world.npy',allow_pickle=False)
        fresh=np.load(location/'derived/chunk_001_world.npy',allow_pickle=False)
        old.setflags(write=False);fresh.setflags(write=False)
        previous=read_json(location/'derived/projection_metrics.json')['conditions']
        result=characterize_episode(episode,metadata['R1'],old,fresh,previous,config)
        result.update(source_observation_time=metadata['observations'][1]['observation_time'],
            source_readiness_time=read_json(location/'inference_metadata.json')['chunks'][1]['response_receive_time'],
            actual_execution_time=None,new_inference_readiness_time=None,
            input_files=[file_record(location/p,source) for p in ('metadata.json','inference_metadata.json',
                'raw/chunk_000.npy','raw/chunk_001.npy','derived/chunk_000_world.npy','derived/chunk_001_world.npy','derived/projection_metrics.json')])
        results.append(result)
    return results

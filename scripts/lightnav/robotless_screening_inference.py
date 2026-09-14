#!/usr/bin/env python3
"""Thirty reset-separated sessions using the validated successive client."""
import argparse
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
import robotless_screening_artifacts as artifacts
from reconciliation.robotless_handoff_screening import transition_metrics
from reconciliation.robotless_single_chunk import save_json_exclusive, sha256_file
from robotless_successive_inference import run_inference
from robotless_single_frame_inference import host_event


def failure_status(run: Path, error: Exception) -> str:
    message = str(error).lower()
    if 'finite' in message:
        return 'NONFINITE_OUTPUT'
    if isinstance(error, TimeoutError) or 'lightnav next failed' in message:
        return 'MODEL_ERROR'
    if any(word in message for word in ('sequence', 'envelope', 'prediction', 'json', 'expecting', 'login', 'reset')):
        return 'PROTOCOL_ERROR'
    return 'PIPELINE_ERROR'


def collect(run: Path, timeout_s=120., *, inference_fn=None) -> list[dict]:
    if inference_fn is None:
        inference_fn = run_inference
    config, manifest, manifest_hash = artifacts.load_frozen(run)
    if (run/'inference_batch.json').exists() or any((run/'episodes'/e['episode_id']/'inference_attempt.json').exists() for e in manifest['episodes']):
        raise FileExistsError('the frozen bank has already been attempted; no repeated collection')
    started = host_event()
    if started['monotonic_ns'] <= manifest['frozen_time']['monotonic_ns']:
        raise ValueError('manifest must be frozen before inference')
    save_json_exclusive(run/'inference_batch_started.json', {'time':started, 'manifest_sha256':manifest_hash,
        'processing_source_sha256':artifacts.phase_sources(['scripts/lightnav/robotless_screening_inference.py','scripts/lightnav/robotless_successive_inference.py'])})
    statuses = []
    for episode in manifest['episodes']:
        artifacts.load_frozen(run)
        location = run/'episodes'/episode['episode_id']
        capture = artifacts.read_json(location/'capture_status.json')
        attempt = {'episode_id':episode['episode_id'],'category':episode['category'],
            'manifest_sha256':manifest_hash,'started_time':host_event(), 'capture_valid':capture['capture_valid']}
        save_json_exclusive(location/'inference_attempt.json',attempt)
        status = {**attempt,'attempted':True,'inference_attempted':False}
        if not capture['capture_valid']:
            status.update(status='CAPTURE_INVALID',reason=capture['reason'])
        else:
            try:
                with (location/'server_process.json').open('xb') as stream:
                    stream.write((run/'server_process.json').read_bytes())
                status['inference_attempted'] = True
                result = inference_fn(location, timeout_s)
                status['session'] = result['session']
                if result['chunks'][0]['stop']:
                    status.update(status='OLD_STOP',reason='seq=0 returned stop=true; both responses preserved')
                elif result['chunks'][1]['stop']:
                    status.update(status='FRESH_STOP',reason='seq=1 returned stop=true; both responses preserved')
                else:
                    metadata = artifacts.read_json(location/'metadata.json')
                    hashes = [sha256_file(location/f'raw/chunk_{i:03d}.npy') for i in range(2)]
                    fresh = np.load(location/'derived/chunk_001_world.npy',allow_pickle=False)
                    try:
                        rows = transition_metrics(episode,metadata['R1'],fresh,*hashes)
                    except ValueError as error:
                        if 'arc length' not in str(error):
                            raise
                        status.update(status='GEOMETRY_UNAVAILABLE',reason=str(error))
                    else:
                        save_json_exclusive(location/'derived/projection_metrics.json', {'conditions':rows,
                            'manifest_sha256':manifest_hash,'inference_metadata_sha256':sha256_file(location/'inference_metadata.json'),
                            'input_files':[artifacts.file_record(location/p,location) for p in ('metadata.json','raw/chunk_000.npy','raw/chunk_001.npy','derived/chunk_001_world.npy')]})
                        status.update(status='VALID_PAIR',reason=None)
            except Exception as error:
                status.update(status=failure_status(location,error),reason=f'{type(error).__name__}: {error}')
        artifacts.load_frozen(run)
        status.update(completed_time=host_event(), files=[artifacts.file_record(p,location) for p in sorted(location.rglob('*')) if p.is_file()])
        save_json_exclusive(location/'validation.json',status)
        statuses.append(status)
        print(episode['episode_id'],episode['category'],status['status'],status.get('reason'),flush=True)
    save_json_exclusive(run/'inference_batch.json', {'started_time':started,'completed_time':host_event(),
        'manifest_sha256':manifest_hash,'attempted':len(statuses),'statuses':statuses,
        'server_process':artifacts.file_record(run/'server_process.json',run),
        'session_semantics':'one new synchronous connection/login/reset per episode; same instruction and connection for seq 0/1',
        'capture_inference_overlap':False,'actual_OLD_execution':False})
    return statuses


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory',type=Path)
    parser.add_argument('--timeout-s',type=float,default=120.)
    args=parser.parse_args()
    collect(args.run_directory.resolve(),args.timeout_s)

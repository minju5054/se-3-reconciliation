#!/usr/bin/env python3
"""Five independent final sessions with an exact planning-OLD check; no retries."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
import numpy as np
import robotless_old_consistent_artifacts as a
from robotless_successive_inference import run_inference
from robotless_single_frame_inference import host_event
from reconciliation.robotless_old_consistent_observation import check_old_reproduction
from reconciliation.robotless_single_chunk import save_json_exclusive


def collect(run, timeout_s=120., *, inference_fn=None):
    inference_fn = run_inference if inference_fn is None else inference_fn
    config,manifest,source,mh=a.load_frozen(run)
    if (run/'inference_started.json').exists(): raise FileExistsError('pilot already attempted; no rerun')
    capture=a.read_json(run/'capture_batch.json'); started=host_event()
    if started['monotonic_ns']<=capture['completed_time']['monotonic_ns']: raise ValueError('inference must follow all captures')
    save_json_exclusive(run/'inference_started.json',{'time':started,'manifest_sha256':mh,
        'processing_source_sha256':a.processing_sources(['scripts/lightnav/robotless_old_consistent_inference.py','scripts/lightnav/robotless_successive_inference.py'])})
    statuses=[]
    for episode in manifest['episodes']:
        a.load_frozen(run); location=run/'episodes'/episode['episode_id']; captured=a.read_json(location/'capture_status.json')
        status={'episode_id':episode['episode_id'],'started_time':host_event(),'inference_attempted':False,
            'status':captured['status'],'reason':captured['reason'],'reproduction':None,'manifest_sha256':mh}
        if captured['capture_valid']:
            try:
                with (location/'server_process.json').open('xb') as stream: stream.write((run/'server_process.json').read_bytes())
                status['inference_attempted']=True
                result=inference_fn(location,timeout_s)
                reproduced=check_old_reproduction(np.load(Path(source['source_run'])/'episodes'/episode['episode_id']/'raw/chunk_000.npy',allow_pickle=False),
                    np.load(location/'raw/chunk_000.npy',allow_pickle=False))
                status.update(reproduction=reproduced,session=result['session'])
                if reproduced['status']=='PLANNING_OLD_MISMATCH': status.update(status='PLANNING_OLD_MISMATCH',reason='final seq0 differs from planning OLD')
                elif any(c['stop'] for c in result['chunks']): status.update(status='MODEL_STOP',reason='one or both final responses have stop=true')
                else: status.update(status='VALID_PAIR',reason=None)
            except Exception as error: status.update(status='INFERENCE_FAILED',reason=f'{type(error).__name__}: {error}')
        status['completed_time']=host_event(); save_json_exclusive(location/'inference_status.json',status); statuses.append(status)
        print(episode['episode_id'],status['status'],status['reason'],flush=True)
    a.load_frozen(run)
    save_json_exclusive(run/'inference_batch.json',{'statuses':statuses,'started_time':started,'completed_time':host_event(),
        'manifest_sha256':mh,'independent_session_count':sum(s['inference_attempted'] for s in statuses),
        'capture_inference_overlap':False,'actual_execution_time':None})
    return statuses


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('run_directory',type=Path); parser.add_argument('--timeout-s',type=float,default=120.)
    args=parser.parse_args(); collect(args.run_directory.resolve(),args.timeout_s)

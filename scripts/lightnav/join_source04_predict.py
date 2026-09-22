#!/usr/bin/env python3
"""One independent session and one terminal prediction for a frozen K condition."""
import argparse
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/lightnav')]
from reconciliation.join_source03 import read,save,sha
from online_lightnav_worker import Session,audited_startup
from join_source02_paired import copy_captured_frames,wire_parity
from robotless_single_frame_inference import git_state,resolve_path,verify_checkpoint_unchanged


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True);p.add_argument('--condition',required=True)
    a=p.parse_args();run=a.run.resolve();protocol=read(run/'protocol.json')
    if a.condition not in protocol['declaration']['order']:raise ValueError('undeclared condition')
    label=protocol['scope'];m=read(run/'conditions'/f'{a.condition}.json')
    if not m['premodel']['valid']:raise ValueError('PREMODEL_K_UNAVAILABLE')
    out=run/'predictions'/a.condition;out.mkdir(parents=True,exist_ok=False)
    save(out/'input_manifest.json',m);start=time.monotonic();events=[];session=None;frames=[]
    config,contract,sampler,ready=audited_startup(run/'config_snapshot.yaml');save(out/'official_startup.json',ready)
    result=dict(condition=a.condition,scope=label,status='TECHNICAL_INVALID',retry_count=0)
    try:
        frames=copy_captured_frames(out,m['frames'][:-1],m['frames'][-1])
        session=Session(config,dict(episode_id=a.condition,episode_dir=str(out),instruction=m['instruction']),contract,sampler,emit_fn=events.append)
        for frame in frames[:-1]:
            response=session.process_frame(dict(op='frame',frame=frame,predict=False))
            if response['status']!='BUFFERED':raise ValueError(response['status'])
        terminal=session.process_frame(dict(op='frame',frame=frames[-1],predict=True,chunk_id='terminal'))
        result.update(status=terminal['status'],terminal_prediction=terminal)
    except Exception as e:
        result['error']=f'{type(e).__name__}: {e}'
    finally:
        if session is not None:result['session_close']=session.close()
        result.update(wall_s=time.monotonic()-start,wire_events=events,wire_parity=wire_parity(out,frames))
        verify_checkpoint_unchanged(ready['checkpoint'])
        result['source_unchanged']=git_state(resolve_path(config['paths']['lightnav_checkout']))==ready['source']
        result['checkpoint_stat_unchanged']=True
        save(out/'result.json',result)
    return 0 if result['status'] in ('PREDICTION_READY','MODEL_STOP') else 2


if __name__=='__main__':raise SystemExit(main())

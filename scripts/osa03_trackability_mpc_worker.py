#!/usr/bin/env python3
"""Original worker loop plus exact, pre-execution OLD-memory restoration.

No solve is permitted during restoration. Official install then increments the
FRESH generation normally. No external source or solver implementation changes.
"""
import sys
from pathlib import Path
sys.path[:0]=[str(Path(__file__).resolve().parent),str(Path(__file__).resolve().parents[1]/'src')]
import numpy as np
import online_mpc_worker as original
from reconciliation.join_source03 import read,sha
from reconciliation.online_mpc_adapter import selection_audit
original_dispatch=original.dispatch


def restore(adapter,phase):
    assert not getattr(adapter,'observation_restored',False)
    assert not adapter.used_solve_ids and adapter.pending is None and adapter.tracker._future is None
    assert phase['pending_OLD']==[] and phase['resolved']
    old=phase['old_install'];r=phase['last_accepted_OLD'];t=adapter.tracker;m=adapter.module
    assert adapter.installed['chunk_id']==old['chunk_id']
    assert t._generation==old['official_generation']
    ref=m.build_pose_aligned_reference(adapter.world,r['input_pose'],horizon=m.HORIZON,weights=m.Q_WEIGHTS)
    assert selection_audit(adapter.world,r['input_pose'],ref,horizon=m.HORIZON,weights=m.Q_WEIGHTS)==r['selection']
    np.testing.assert_array_equal(r['command'],phase['memory_obs'])
    t.command=tuple(r['command']);t.previous_command=tuple(phase['memory_obs'])
    t.reference=np.asarray(r['selection']['reference_world']);t.prediction=np.asarray(r['prediction_world'])
    t.solve_ms=r['official_solve_ms'];t.error='';adapter.observation_restored=True
    return dict(status='observation_restored',previous_control=list(t.previous_command),command=list(t.command),
        generation=t._generation,old_world=adapter.world.tolist(),historical_solve_id=r['solve_id'],new_solve_calls=0)


def dispatch(adapter,request):
    if request['op']=='restore_observation':
        p=Path(request['phase_path']);assert sha(p)==request['phase_sha256']
        return restore(adapter,read(p))
    if request['op']=='state_audit':
        return dict(status='audited',memory=list(adapter.tracker.previous_command),command=list(adapter.tracker.command),
                    installed=adapter.installed,world=adapter.world.tolist(),pending=adapter.pending)
    return original_dispatch(adapter,request)

original.dispatch=dispatch
if __name__=='__main__':original.main()

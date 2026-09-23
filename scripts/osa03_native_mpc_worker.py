#!/usr/bin/env python3
"""Reuse original online MPC worker; add saved-result restoration only.

No changes to external tracker/solver/selection or to the worker polling loop.
Restoration is permitted only for the two frozen historical accepted results,
in order and before any new solve. It does not invoke MPCController.solve.
"""
import sys
from pathlib import Path
sys.path[:0]=[str(Path(__file__).resolve().parent),str(Path(__file__).resolve().parents[1]/'src')]
import numpy as np
import online_mpc_worker as original
from reconciliation.online_mpc_adapter import selection_audit
old_dispatch=original.dispatch
restored=[]


def dispatch(adapter,request):
    if request['op']!='restore_saved':return old_dispatch(adapter,request)
    from reconciliation.join_source03 import read,sha
    phase_path=Path(request['phase_path']);assert sha(phase_path)==request['phase_sha256']
    phase=read(phase_path);i=request['historical_index']
    assert i==len(restored) and i<len(phase['applications'])
    assert not adapter.used_solve_ids and adapter.pending is None and adapter.tracker._future is None
    r=phase['applications'][i]['result'];module=adapter.module;tracker=adapter.tracker
    ref=module.build_pose_aligned_reference(adapter.world,r['input_pose'],horizon=module.HORIZON,weights=module.Q_WEIGHTS)
    audit=selection_audit(adapter.world,r['input_pose'],ref,horizon=module.HORIZON,weights=module.Q_WEIGHTS)
    assert audit==r['selection']
    if i:np.testing.assert_array_equal(tracker.previous_command,r['previous_command'])
    tracker._generation=phase['original_generation'];adapter.installed['official_generation']=phase['original_generation']
    tracker.command=tuple(r['command']);tracker.previous_command=tuple(r['command'])
    tracker.reference=np.asarray(r['selection']['reference_world']);tracker.prediction=np.asarray(r['prediction_world'])
    tracker.solve_ms=r['official_solve_ms'];tracker.error=''
    restored.append(r['solve_id'])
    return dict(status='restored_saved',historical_solve_id=r['solve_id'],previous_control=list(tracker.previous_command),
                selection=audit,installed_world=adapter.world.tolist(),new_solve_calls=0)

original.dispatch=dispatch
if __name__=='__main__':original.main()

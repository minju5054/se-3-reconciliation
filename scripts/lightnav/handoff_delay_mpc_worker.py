#!/usr/bin/env python3
"""JSON-line bridge to frozen REF-02 official tracker; no LightNav client."""
import json,os,sys,traceback
from pathlib import Path
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):os.environ[k]='1'
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src'))
import numpy as np
from reconciliation.online_mpc_adapter import load_official
from reconciliation.gp_se2_rollout import candidate_in_capture_frame
from reconciliation.gp_se2_ref02_rollout import make_diagnostic_tracker,diagnostic_synchronous_solve,array_sha256

def main():
    tracker=None;module=None;instance_ordinal=0;solve_calls=0
    for line in sys.stdin:
        q=json.loads(line)
        try:
            if q['op']=='init':
                if tracker is not None:raise ValueError('close prior instance first')
                if module is None:module,provenance=load_official(q['checkout'])
                instance_ordinal+=1
                ref=np.asarray(q['reference'],dtype=float)
                tracker=make_diagnostic_tracker(module,q['method'],ref,q['lineage'],q['goal_index'],constant_reference_metadata=q['constant'])
                if q['method']=='A_NATIVE':
                    local=np.asarray(q['raw_local']);transform={'source':'unchanged original raw local','B_reanchoring':False}
                else:local,transform=candidate_in_capture_frame(ref,q['capture'])
                tracker.set_body_path(local,q['capture'])
                tracker.previous_command=tuple(q['memory']);tracker.command=tuple(q['physical'])
                reply=dict(instance_ordinal=instance_ordinal,provenance=provenance,identity=tracker._ref02_capture['identity'],
                           installed=tracker._trajectory.tolist(),installed_hash=array_sha256(tracker._trajectory),transform=transform)
            elif q['op']=='solve':
                solve_calls+=1
                reply=diagnostic_synchronous_solve(module,tracker,q['pose'])
                reply['worker_solve_ordinal']=solve_calls
            elif q['op']=='close':
                tracker.close();tracker=None;reply={'closed':True}
            else:raise ValueError('unknown operation')
            print(json.dumps({'ok':True,'result':reply},allow_nan=False),flush=True)
        except Exception:
            print(json.dumps({'ok':False,'error':traceback.format_exc()}),flush=True)
    if tracker is not None:tracker.close()
if __name__=='__main__':main()

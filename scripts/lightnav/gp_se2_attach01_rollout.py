#!/usr/bin/env python3
"""ATTACH-01 independent common-B rollouts, unmodified official selector/MPC."""
import argparse
import json
import os
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'src'))
sys.dont_write_bytecode=True
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from reconciliation.online_mpc_adapter import load_official,sha256
from reconciliation.gp_se2_rollout import counterfactual_rollout,save_rollout,verify_source_records
import numpy as np


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True)
    run=parser.parse_args().run.resolve();read=lambda p:json.loads(p.read_text())
    source=read(run/'source.json');official=source['official_mpc'];checkout=Path(official['lightnav_checkout'])
    if Path(sys.prefix).resolve()!=(checkout/'mujoco_demo/.venv').resolve():raise ValueError('official isolated MPC venv required')
    for record in [read(run/'execution_freeze.json'),read(run/'comparison_freeze.json')]:
        for path,expected in record['files'].items():
            if sha256(Path(path))!=expected:raise ValueError('frozen evidence changed: '+path)
    if sha256(Path(official['mpc_source']))!=official['mpc_source_sha256']:raise ValueError('official source changed')
    context=read(run/'inputs/context.json');verify_source_records(context['source_root'],context['source_files'])
    output=run/'rollouts';output.mkdir(exist_ok=False)
    module,provenance=load_official(checkout)
    if (module.HORIZON,module.MPC_DT_S,module.CONTROL_RATE_HZ,tuple(module.Q_WEIGHTS))!=(5,.1,10.,(10.,10.,1.)):
        raise ValueError('official settings mismatch')
    with (output/'provenance.json').open('x') as stream:json.dump(provenance,stream,indent=2)
    for method,record in read(run/'method_manifest.json').items():
        if not record['candidate_available']:continue
        if method.startswith(('M3','M4')) and not record['plan_valid']:raise ValueError('invalid GP plan cannot execute')
        if sha256(Path(record['reference_path']))!=record['reference_sha256']:raise ValueError('reference changed')
        roll=counterfactual_rollout(module,context,np.load(record['reference_path'],allow_pickle=False))
        roll.update(kind='OFFLINE SINGLE-HANDOFF COUNTERFACTUAL',method=method,duration_s=record['duration_s'],
            plan_valid=record['plan_valid'],diagnostic_plan_invalid=not record['plan_valid'],
            controller_statement='unchanged official MPC computation AND original nearest/+1 selector')
        save_rollout(output/method,roll)
        print(method,len(roll['controller_reference_selections']),flush=True)


if __name__=='__main__':main()

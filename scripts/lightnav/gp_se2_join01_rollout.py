#!/usr/bin/env python3
"""Unchanged official selector/MPC; independently recomputed from common B."""
import argparse,json,hashlib,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'src'))
import numpy as np
from reconciliation.online_mpc_adapter import load_official,sha256
from reconciliation.gp_se2_rollout import counterfactual_rollout,save_rollout,verify_source_records


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();run=a.run.resolve()
    read=lambda p:json.loads(p.read_text())
    source=read(run/'source.json');checkout=Path(source['official_checkout'])
    if Path(sys.prefix).resolve()!=(checkout/'mujoco_demo/.venv').resolve():raise ValueError('official MPC venv required')
    for record in [read(run/'event_freeze.json'),read(run/'comparison_freeze.json')]:
        for p,h in record['files'].items():
            if sha256(Path(p))!=h:raise ValueError('frozen source changed: '+p)
    context=read(run/'inputs/context.json');verify_source_records(context['source_root'],context['source_files'])
    methods=read(run/'method_manifest.json');output=run/'rollouts';output.mkdir(exist_ok=False)
    module,provenance=load_official(checkout)
    with (output/'provenance.json').open('x') as f:json.dump(provenance,f,indent=2)
    for name,record in methods.items():
        if not record['candidate_available']:continue
        if name.startswith(('M3','M4')) and not record['plan_valid']:raise ValueError('invalid GP execution forbidden')
        path=np.load(record['reference_path'],allow_pickle=False)
        roll=counterfactual_rollout(module,context,path)
        roll.update(kind='OFFLINE SINGLE-HANDOFF COUNTERFACTUAL',method=name,
                    plan_valid=record['plan_valid'],diagnostic_plan_invalid=not record['plan_valid'])
        save_rollout(output/name,roll)


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Import-isolated technical completion: same frozen references and official MPC."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import sys
import time
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):os.environ[key]='1'
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'src'))
import numpy as np
from reconciliation.gp_se2_diag08_execution import execute_reference,validate_execution_records,execution_admission
from reconciliation.gp_se2_diag07_lateral_execution import array_hash
from reconciliation.gp_se2_rollout import load_frozen_context,historical_solve_audit,verify_source_records
from reconciliation.online_mpc_adapter import load_official,sha256
EXTERNAL=Path('/home/gpuadmin/Workspace/external/LightNav-0-official-demo')

def read(p):return json.loads(p.read_text())
def write(p,v):
    p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',required=True,type=Path);p.add_argument('--stage',choices=['audit','execute'],required=True)
    a=p.parse_args();run=a.run.resolve()
    if Path(sys.prefix).resolve()!=(EXTERNAL/'mujoco_demo/.venv').resolve():raise ValueError('isolated official environment required')
    for name in ['execution_freeze.json','execution_request_freeze.json','runtime_freeze.json']:
        for path,h in read(run/name)['file_sha256'].items():
            if sha256(Path(path))!=h:raise ValueError('freeze mismatch: '+path)
    request=read(run/'mpc_request.json');source=Path(request['source_root']);module,provenance=load_official(EXTERNAL)
    if (module.HORIZON,module.MPC_DT_S,module.CONTROL_RATE_HZ)!=(5,.1,10.):raise ValueError('official settings changed')
    output=run/('historical_audits' if a.stage=='audit' else 'rollouts');output.mkdir(exist_ok=False)
    write(output/'started.json',dict(stage=a.stage,retry=False,request_sha256=sha256(run/'mpc_request.json')))
    write(output/'official_provenance.json',provenance);start=time.perf_counter();records=[]
    if a.stage=='audit':
        for caseid,path in request['context_paths'].items():
            frozen=load_frozen_context(source,*caseid.split('/'))
            if frozen!=read(Path(path))['context']:raise ValueError('original context changed')
            report=historical_solve_audit(module,frozen)
            write(output/(caseid.replace('/','__')+'.json'),report);records.append(dict(case_id=caseid,passed=report['passed']))
            if not report['passed']:raise ValueError('historical audit failed')
    else:
        if not read(run/'historical_audits/summary.json')['passed']:raise ValueError('audit gate failed')
        for r in request['unique_references']:
            uid=r['unique_reference_id'];folder=output/uid;write(folder/'started.json',dict(invocation=1,no_retry=True))
            frozen=load_frozen_context(source,*r['case_id'].split('/'))
            if frozen!=read(Path(request['context_paths'][r['case_id']]))['context']:raise ValueError('source state changed')
            plan=read(Path(r['plan_recheck_path']));admission=execution_admission(plan['original_full_check'],plan['reserve'],hard=r['case_role']=='HARD')
            if admission!=plan['admission'] or not admission['eligible_for_execution']:raise ValueError('plan admission mismatch')
            reference=np.load(r['world_reference_path'],allow_pickle=False)
            if sha256(Path(r['world_reference_path']))!=r['world_reference_file_sha256'] or array_hash(reference)!=r['world_reference_sha256']:raise ValueError('reference identity')
            rollout=execute_reference(module,frozen,reference,admission);rollout['reference_source']=r
            write(folder/'rollout.json',rollout)
            for name,key in [('states','states'),('commands','commands'),('selections','controller_reference_selections')]:write(folder/(name+'.json'),rollout[key])
            write(folder/'predictions.json',[dict(issue_time_s=s['time_s'],prediction_world=s['prediction_world']) for s in rollout['controller_reference_selections']])
            errors=validate_execution_records(rollout,frozen,reference,admission)
            write(folder/'record_validation.json',dict(valid=not errors,errors=errors,new_solve_during_validation=0))
            if errors:raise ValueError(str(errors))
            verify_source_records(source,frozen['source_files'])
            write(folder/'hashes.json',{p.name:sha256(p) for p in folder.iterdir() if p.is_file()})
            records.append(dict(unique_reference_id=uid,solves=len(rollout['controller_reference_selections']),controller_failures=rollout['controller_failure_count']))
            print(json.dumps(records[-1]),flush=True)
    _,after=load_official(EXTERNAL)
    if after['mpc_source_sha256']!=provenance['mpc_source_sha256'] or after['lightnav_sha']!=provenance['lightnav_sha']:raise ValueError('official source changed')
    write(output/'summary.json',dict(passed=True,records=records,wall_s=time.perf_counter()-start,
        mpc_solves=len(records) if a.stage=='audit' else sum(r['solves'] for r in records),
        primary_rollouts=0 if a.stage=='audit' else len(records),new_GP=0,new_VLA=0,new_RGB=0,new_GUI=0,
        independent_tracker_per_rollout=True,official_before_after_identical=True))
if __name__=='__main__':main()

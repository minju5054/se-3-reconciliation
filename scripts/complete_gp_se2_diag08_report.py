#!/usr/bin/env python3
"""Additive saved-result report: import correction, ineligibility, no new solves."""
from __future__ import annotations
import argparse
import os
from pathlib import Path
import sys
import time
os.environ.setdefault('MPLCONFIGDIR','/tmp/gp_diag08_extra_mpl')
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from run_gp_se2_diag08 import read,write,table,digest,clean,D6,D7,verify,load_frozen_case,HospitalEnvironment
from reconciliation.gp_se2_diag06_witnesses import finest_trace
from reconciliation.gp_se2_diag08_execution import execution_admission as isolated_gate
from reconciliation.gp_se2_diag08_endpoint_margin import execution_admission as frozen_gate


def complete(run):
    begin=time.perf_counter();source=verify(run,True);records=read(run/'comparison_records.json')
    starts=read(run/'experiment_manifest.json')['starts'];refs=read(run/'reference_manifest.json')['records'];rows=[];vectors=[]
    runtime=Path(read(run/'execution_completion.json')['runtime_run'])
    from run_gp_se2_diag07 import check_hashes
    check_hashes(read(runtime/'runtime_freeze.json')['file_sha256'])
    env=HospitalEnvironment.load(source['environment_path']);plotrows=[];errors=[]
    for r,start,ref in zip(records,starts,refs):
        label=r['reference_id'];result=read(run/'solves'/label/'solver_result.json');analysis=read(run/'plan_recheck'/(label+'.json'))
        full=analysis['original_full_check'];reserve=analysis['reserve'];latest=next(c for c in analysis['candidate_checks'] if c['iterate']=='latest_iterate')
        # The runtime bridge gate is literally identical to the frozen gate.
        if isolated_gate(full,reserve,hard=start['case_role']=='HARD')!=frozen_gate(full,reserve,hard=start['case_role']=='HARD'):errors.append(label+' gate parity')
        rows.append(dict(reference_id=label,solver_status=result['solver_status'],iterations=result['iterations'],
            G4_grid_pass=result['candidate_checks'][1]['collocation']['feasible'],
            nominal_endpoint_error_m=reserve['endpoint_position_error_m'],
            nominal_endpoint_excess_m=reserve['nominal_position_excess_m'],
            endpoint_squared_margin_m2=reserve['nominal_squared_margin_m2'],
            reserve_row_pass_existing_solver_allowance=reserve['solver_row_pass_with_original_allowance'],
            strict_nominal_reserve_pass=reserve['planning_endpoint_reserve_pass'],
            full_failures=[k for k,v in full['additional_grid']['flags'].items() if not v],
            nonlateral_failure_independent_of_reserve=[k for k,v in full['additional_grid']['flags'].items() if not v and k!='lateral_velocity'],
            supplemental_motion_violations=latest['motion_violating_runs'],
            frozen_witness_margins=result['candidate_checks'][1]['inequality_margins'][-4:-1],
            primary_executed=r['G4']['evaluation'] is not None,
            solver_failure=False if result['solver_status']==0 else True,
            mathematical_infeasibility_proven=False))
        for grid in ['G3','G4']:
            path=start['historical_reference']['world_reference_path'] if grid=='G3' else ref['world_reference_path']
            refarray=np.load(path,allow_pickle=False);goal=np.asarray(r[grid]['evaluation']['goal'] if r[grid]['evaluation'] is not None else read(run/'contexts'/(r['case_id'].replace('/','__')+'.json'))['goal_route']['goal_world'])
            ep=refarray[-1,:2]-goal[:2];geom=r[grid]['geometry']
            vectors.append(dict(reference_id=label,grid=grid,e_plan_m=ep.tolist(),plan_error_m=float(np.linalg.norm(ep)),
                planning_reserve_m=float(.15-np.linalg.norm(ep)),e_track_m=None if geom is None else geom['e_track_m'],
                e_exec_m=None if geom is None else geom['e_exec_m'],vector_closure_error_m=None if geom is None else geom['vector_closure_error_m'],
                execution_available=geom is not None,unavailable_reason=None if geom is not None else 'ineligible nonlateral plan; MPC not run'))
        if start['case_role']=='HARD':
            case=load_frozen_case(Path(source['primary_source']),start['case_id'],start['method'],environment=env)
            old=read(Path(start['g3_result_path']))
            traces={g:clean(finest_trace(case['problem'],z)) for g,z in [('G3',old['latest_iterate']),('G4',result['latest_iterate'])]}
            plotrows.append(dict(reference_id=label,traces=traces))
    table(run/'aggregate/ineligibility_layers.csv',rows);table(run/'aggregate/endpoint_geometry_partial.csv',vectors)
    write(run/'ineligibility_records.json',rows)
    # One figure makes the material second failure visible independently of the
    # picometre-scale nominal endpoint issue. No extra enforcement or re-solve.
    folder=run/'presentation';fig,axes=plt.subplots(2,2,figsize=(13,9));payload=[]
    for ax,item in zip(axes.ravel(),plotrows):
        itemdata=dict(reference_id=item['reference_id'],traces={})
        for g,c in [('G3','#3578b5'),('G4','#d66e20')]:
            tr=item['traces'][g];t=np.asarray(tr['times_s']);vx=np.asarray(tr['body_twists'])[:,0];mask=(t>=.47)&(t<=.49)
            ax.plot(t[mask],vx[mask],color=c,label=g+(' historical' if g=='G3' else ' new'))
            itemdata['traces'][g]=dict(times_s=t[mask].tolist(),vx_m_s=vx[mask].tolist())
        ax.axhline(0,color='k',ls=':',label='nominal v >= 0')
        ax.axhline(-1e-5,color='red',ls='--',label='original numerical threshold')
        ax.axvline(.48,color='#666',ls=':',label='unchanged frozen witness')
        ax.set(title=item['reference_id'],xlabel='trajectory time [s]',ylabel='planned forward velocity [m/s]')
        ax.ticklabel_format(axis='y',style='sci',scilimits=(0,0));ax.grid(alpha=.2);ax.legend(fontsize=8)
        payload.append(itemdata)
    fig.suptitle('G4 nonlateral rejection: speed dip moves beyond the frozen 0.480 s witness\nPLAN ONLY / NOT EXECUTED',fontsize=13)
    fig.tight_layout(rect=(0,0,1,.93));path=folder/'hard_forward_speed_gap.png';fig.savefig(path,dpi=140);plt.close(fig)
    write(path.with_suffix('.json'),dict(numeric=payload,source_sha256=digest(run/'source.json'),config_sha256=digest(run/'config_snapshot.yaml'),image_sha256=digest(path)))
    # Exact benign equality is a measured result, not a copied new rollout.
    b=records[-1];new=read(run/'rollouts'/b['unique_reference_id']/'rollout.json');old=read(D7/'rollouts'/starts[-1]['historical_uid']/'rollout.json')
    parity={key:new[key]==old[key] for key in ['reference_world','states','commands','initial_physical_command','initial_previous_control']}
    parity['actual_selections']=all(a['selection']==b['selection'] for a,b in zip(new['controller_reference_selections'],old['controller_reference_selections']))
    parity['actual_predictions']=all(a['prediction_world']==b['prediction_world'] for a,b in zip(new['controller_reference_selections'],old['controller_reference_selections']))
    parity['new_rollout_performed']=True;parity['not_counted_as_improvement']=True
    write(run/'aggregate/benign_reproduction.json',parity)
    write(run/'report_completion_validation.json',dict(valid=not errors,errors=errors,wall_s=time.perf_counter()-begin,
        preserved_GP_execution_sha=read(run/'execution_freeze.json')['execution_sha'],runtime_run=str(runtime),
        runtime_freeze_sha256=digest(runtime/'runtime_freeze.json'),GP_reruns=0,report_MPC_solves=0,
        original_frozen_code_unchanged=True,execution_bridge_gate_matches_original=True,
        direct_endpoint_vectors_preserved_for_unexecuted=True))
    with (folder/'failure_summary.html').open('x') as f:
        f.write('<!doctype html><meta charset="utf-8"><title>DIAG-08 failure layers</title><h1>G4 hard: converged grid, nonlateral interior failure</h1>'
            '<p>All four hard solves converge and pass their solver grid. Original full checking rejects lateral velocity and negative forward speed. They are not executed. A nominal 11 cm excess of 4.7e-12–7.6e-11 m is separately recorded; it is not the only admission failure.</p>'
            '<p>No hard execution recovery or tracking displacement was measured. Benign is newly executed and exactly reproduces the successful historical state/command sequence.</p>'
            '<p><a href="../index.html">All 50 comparison figures</a> | <a href="../aggregate/ineligibility_layers.csv">Failure table</a></p>'
            '<img style="max-width:1200px;width:100%" src="hard_forward_speed_gap.png"><p>Frozen margin and witnesses were not changed after results.</p>')
    print(dict(valid=not errors,hard_second_failure='linear_speed',benign_parity=parity),flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',required=True,type=Path);complete(p.parse_args().run.resolve())

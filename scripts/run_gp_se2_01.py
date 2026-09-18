#!/usr/bin/env python3
"""Freeze and run GP-SE2-01 without new observations or VLA inference.

Phases are exclusive writes: prepare -> optimize -> external official MPC ->
evaluate -> plot/replay. Existing partial outputs are evidence, never overwritten.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
import yaml

from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_evaluation import METHODS, comparisons, evaluate_plan, evaluate_rollout
from reconciliation.gp_se2_formulation import solve_gp, solve_rigid
from reconciliation.gp_se2_reference import prepare_reference, geometric_group_flags, choose_cases, directed_gate_crossings
from reconciliation.gp_se2_rollout import load_frozen_context


def digest(path):
    with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def clean(value):
    if isinstance(value,np.ndarray):return clean(value.tolist())
    if isinstance(value,np.generic):return clean(value.item())
    if isinstance(value,dict):return {k:clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean(v) for v in value]
    if isinstance(value,float) and not np.isfinite(value):return None
    return value


def read(path):return json.loads(Path(path).read_text())


def write(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x') as f:json.dump(clean(value),f,indent=2,allow_nan=False);f.write('\n')


def array(path,value):
    with Path(path).open('xb') as f:np.save(f,np.asarray(value),allow_pickle=False)


def table(path,rows):
    rows=[clean(r) for r in rows];fields=list(dict.fromkeys(k for r in rows for k in r)) or ['status']
    with Path(path).open('x',newline='') as f:
        w=csv.DictWriter(f,fields);w.writeheader()
        for r in rows:w.writerow({k:json.dumps(v) if isinstance(v,(list,dict)) else v for k,v in r.items()})


def file_records(root):
    return [{'path':str(p.relative_to(root)),'sha256':digest(p),'bytes':p.stat().st_size}
            for p in sorted(Path(root).rglob('*')) if p.is_file()]


def route_for_source(boundary,prepared,env):
    """Freeze actual scene gates crossed by the remaining original row sequence."""
    b=np.asarray(boundary);suffix=prepared['suffix_world'];goal=prepared['goal_world'];times=prepared['row_times_s']
    gates=[];examined=[]
    for original in env.gates:
        center=np.asarray(original.get('center_world_xy_m',original.get('center_xy')),float)
        normal=np.asarray(original.get('normal_world_xy',original.get('normal_xy')),float)
        if np.dot(goal[:2]-b[:2],normal)<0:normal=-normal
        if np.dot(b[:2]-center,normal)>=-1e-6 or np.dot(goal[:2]-center,normal)<=1e-6:
            continue
        gate=dict(gate_id=original['gate_id'],center_xy=center.tolist(),normal_xy=normal.tolist(),
                  half_width_m=original['half_width_m'],source=original,
                  crossing_offset_s=.05,minimum_progress_m=.0001)
        crossing=directed_gate_crossings(suffix[:,:2],times,[gate]) if len(suffix)>1 else {'valid':False}
        if crossing['valid']:
            at=crossing['gates'][0]['first_directed_crossing_s']
            if .1+1e-6<at<3.-1e-6:
                gate['time_s']=at;gates.append(gate)
        examined.append(dict(gate_id=gate['gate_id'],raw_suffix_crossing=crossing))
    gates.sort(key=lambda g:(g['time_s'],g['gate_id']))
    shortcut=env.check_polyline(np.vstack([b,goal]))
    if gates:
        status='REQUIRED_ORDERED_GATES';reason='original future suffix crosses independently verified scene passage between B and fixed original goal'
    elif shortcut['clearance_valid']:
        status='NOT_REQUIRED_CLEAR_SHORTCUT';reason='remaining raw suffix and direct B-to-goal segment lie in known clearance-valid space; no catalogued remaining passage crossing'
    else:
        status='UNKNOWN';reason='shortcut violates clearance/workspace but no reliable remaining scene gate could be assigned'
    return dict(goal_world=goal.tolist(),position_tolerance_m=.15,yaw_tolerance_rad=float(np.pi/12),
                goal_source='last original FRESH world pose, never optimized endpoint',
                gates=gates,route_status=status,reason=reason,gate_examinations=examined,
                shortcut_environment=shortcut,scope='local goal/route proxy, not complete natural-language instruction correctness')


def prepare(run,config_path,environment,source_audit):
    run=Path(run).resolve();cfg=yaml.safe_load(Path(config_path).read_text());source=(ROOT/cfg['source_run']).resolve()
    environment=Path(environment).resolve();source_audit=Path(source_audit).resolve()
    if run.exists() or run.is_relative_to(source):raise FileExistsError('output must be new and outside immutable source')
    validation=read(source_audit/'source_validation.json');hash_validation=read(source_audit/'source_hash_validation.json')
    envvalidation=read(environment/'validation.json')
    if not validation['valid'] or not hash_validation['valid'] or not envvalidation.get('valid',envvalidation.get('passed',False)):
        raise ValueError('source and actual environment validation must PASS before case freeze')
    # This first pilot intentionally supports one protocol, not a configurable sweep.
    expected=[cfg['footprint']['radius_m'],cfg['footprint']['required_clearance_m'],cfg['reference']['first_time_s'],cfg['reference']['final_time_s'],cfg['reference']['output_dt_s'],cfg['formulation']['horizon_s'],cfg['formulation']['goal_position_tolerance'],cfg['formulation']['goal_yaw_tolerance']]
    if not np.allclose(expected,[.2,.05,.1,3.,.1,3.,.15,np.pi/12],rtol=0,atol=1e-12):
        raise ValueError('this frozen pilot requires the declared common footprint, timing and goal protocol')
    env=HospitalEnvironment.load(environment)
    shapes={(r['episode_id'],r['event_id']):r for r in read(ROOT/cfg['shape_analysis']/'events.json')}
    rows=[];details={}
    for (episode_id,event_id),shape in sorted(shapes.items()):
        episode=source/'episodes'/episode_id;event=episode/'handoffs'/event_id
        context=read(event/'context.json');metrics=read(event/'metrics.json')
        ref=context['fresh_world_ref'];path=episode/ref['path']
        if digest(path)!=ref['sha256']:raise ValueError('source hash mismatch')
        native=np.load(path,allow_pickle=False);prepared=prepare_reference(native,context['B'],[10,10,1])
        bcheck=env.query(context['B'][:2]);goalcheck=env.query(native[-1,:2]);suffixcheck=env.check_polyline(prepared['suffix_world'])
        route=route_for_source(context['B'],prepared,env)
        timing=metrics['interval_timing'];inflight=timing['inference_client_host_interval']
        timing_ok=bool(timing['real_time_pacing_valid'] and timing['causal_execution_overlap_observed']
            and timing['actual_post_switch_execution_available'] and inflight['capture_count']>0)
        reasons=[]
        if not timing_ok:reasons.append('exact_timing_or_overlap_failed')
        if bcheck['status']!='CLEARANCE_VALID':reasons.append('boundary_'+bcheck['status'])
        if goalcheck['status']!='CLEARANCE_VALID':reasons.append('goal_'+goalcheck['status'])
        if not suffixcheck['clearance_valid']:reasons.append('raw_suffix_'+suffixcheck['status'])
        if route['route_status']=='UNKNOWN':reasons.append('route_unknown')
        flags=geometric_group_flags(metrics,shape['raw_shape'],cfg['selection'])
        flags['D_OBSTACLE_ROUTE']=bool(route['gates'] and not route['shortcut_environment']['clearance_valid']
                                       and route['shortcut_environment']['workspace_known'])
        case_id=episode_id+'/'+event_id
        row=dict(case_id=case_id,case_directory=episode_id+'__'+event_id,episode_id=episode_id,handoff_id=event_id,
            eligible=not reasons,rejection_reasons=reasons,groups=flags,
            ordered_raw_pair=metrics['old_raw_sha256']+':'+metrics['fresh_raw_sha256'],
            e_perp_m=metrics['e_perp_m'],direction_difference_deg=metrics['abs_e_dir_window_deg'],
            pose_yaw_difference_deg=metrics['abs_e_yaw_deg'],raw_shape=shape['raw_shape'],
            exact_timing_valid=timing_ok,B_environment=bcheck,goal_environment=goalcheck,
            raw_suffix_environment=suffixcheck,route_status=route['route_status'],
            shortcut_environment=route['shortcut_environment'])
        rows.append(row);details[case_id]=(prepared,route,metrics)
    selected=choose_cases([r for r in rows if r['eligible']],cfg['selection'])
    selected_ids={r['case_id'] for r in selected}
    for row in rows:
        row['selected']=row['case_id'] in selected_ids
        if row['eligible'] and not row['selected']:row['nonselection_reason']='deterministic group capacity/diversity order' if any(row['groups'].get(g) for g in cfg['selection']['group_order']) else 'outside four frozen descriptive groups'
    if not selected:raise RuntimeError('no eligible cases; preserve technical audit and report coverage blocker')
    run.mkdir(parents=True)
    shutil.copy2(config_path,run/'config_snapshot.yaml')
    shutil.copytree(environment,run/'environment')
    (run/'aggregate').mkdir()
    # Save evaluated workspace rings for Isaac, which does not import Shapely.
    polygons=list(env.workspace.geoms) if env.workspace.geom_type=='MultiPolygon' else [env.workspace]
    write(run/'environment/geometry/render_geometry.json',{'workspace_polygons':[list(g.exterior.coords) for g in polygons if g.geom_type=='Polygon'],
        'workspace_holes':[list(r.coords) for g in polygons if g.geom_type=='Polygon' for r in g.interiors]})
    table(run/'aggregate/eligibility.csv',rows)
    manifest=dict(planned_count=12,source_event_count=len(rows),eligible_count=sum(r['eligible'] for r in rows),
        group_candidate_counts={g:sum(r['eligible'] and r['groups'].get(g,False) for r in rows) for g in cfg['selection']['group_order']},
        selected_count=len(selected),selected=selected,selection=cfg['selection'],
        selection_used_new_optimization_or_rollouts=False,all_source_decisions=rows)
    write(run/'case_manifest.json',manifest)
    for row in selected:
        prepared,route,metrics=details[row['case_id']]
        case=run/'cases'/row['case_directory'];case.mkdir(parents=True)
        frozen=load_frozen_context(source,row['episode_id'],row['handoff_id'])
        write(case/'input_context.json',frozen);write(case/'reference_preparation.json',prepared)
        write(case/'goal_route.json',route);write(case/'source_metrics.json',metrics)
        array(case/'F_native.npy',prepared['native_world']);array(case/'F_common.npy',prepared['common_world'])
    sources=sorted(set([ROOT/rel for rel in ['pyproject.toml','configs/gp_se2_01.yaml','src/reconciliation/se2.py',
        'src/reconciliation/online_mpc_adapter.py','src/reconciliation/robotless_online.py',
        'scripts/isaac/robotless_runtime.py','scripts/isaac/robotless_online_replay.py',
        'scripts/isaac/robotless_old_consistent_observation.py']]+
        list((ROOT/'src/reconciliation').glob('gp_se2*.py'))+list((ROOT/'tests').glob('test_gp_se2*.py'))+
        list((ROOT/'scripts').glob('*gp_se2*'))+list((ROOT/'scripts/isaac').glob('*gp_se2*'))+
        list((ROOT/'scripts/lightnav').glob('*gp_se2*'))))
    code={str(p.relative_to(ROOT)):digest(p) for p in sources if p.is_file()}
    for rel in code:
        target=run/'implementation_sources'/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/rel,target)
    write(run/'source.json',dict(source_run=str(source),source_collector_git_sha=cfg['source_collector_git_sha'],
        starting_git_sha=cfg['starting_git_sha'],implementation_git_sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        experiment_source_sha256=code,source_audit=str(source_audit),source_audit_sha256={p.name:digest(p) for p in source_audit.glob('*.json')},
        environment_export_source=str(environment),environment_file_hashes=file_records(run/'environment'),
        case_input_file_hashes=file_records(run/'cases')))
    write(run/'protocol.json',dict(frozen_utc=datetime.now(timezone.utc).isoformat(),methods=list(METHODS),
        config_sha256=digest(run/'config_snapshot.yaml'),case_manifest_sha256=digest(run/'case_manifest.json'),
        horizon_s=3.,grid_support_s='0,.1,...,3',grid_mpc_reference_s='.1,.2,...,3',
        rollout_semantics=cfg['rollout'],no_new_inference=True,case_replacement_forbidden=True,
        objective='J_GP + lambda_F mean ||Log(F_common^-1 X)||^2_SigmaF; uniform preservation',
        m2_m3_difference='obstacle inequality only; same workspace, goal, gates, starts, scales, solver budget',
        solver='scipy.optimize.SLSQP; equality/inequality constraints, not large penalty constraints',
        initialization_selection='dense-feasible first, then smallest own objective; retain failures and every start',
        blas_threads=1,nonfinite_serialization='null, never zero; numeric failures retain status',
        claims='static authored oracle and idealized kinematic counterfactual; no physical safety/general superiority'))
    print(json.dumps({'run':str(run),'eligible':manifest['eligible_count'],'selected':len(selected),'groups':manifest['group_candidate_counts']}),flush=True)


def verify_frozen(run):
    source=read(run/'source.json');protocol=read(run/'protocol.json')
    assert digest(run/'config_snapshot.yaml')==protocol['config_sha256']
    assert digest(run/'case_manifest.json')==protocol['case_manifest_sha256']
    for rel,h in source['experiment_source_sha256'].items():
        if digest(ROOT/rel)!=h:raise ValueError(f'implementation changed after freeze: {rel}')
    for r in source['case_input_file_hashes']:
        if digest(run/'cases'/r['path'])!=r['sha256']:raise ValueError('frozen case input changed: '+r['path'])
    for r in source['environment_file_hashes']:
        if digest(run/'environment'/r['path'])!=r['sha256']:raise ValueError('frozen environment changed')


def optimize(run):
    run=Path(run).resolve();verify_frozen(run);cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    env=HospitalEnvironment.load(run/'environment');rows=read(run/'case_manifest.json')['selected'];attempts=[]
    for row in rows:
        case=run/'cases'/row['case_directory'];frozen=read(case/'input_context.json');route=read(case/'goal_route.json')
        common=np.load(case/'F_common.npy');b=np.asarray(frozen['B_world']);u=frozen['u_minus'];v=np.array([u[0],0.,u[1]])
        for method in METHODS:
            target=case/'methods'/method;target.mkdir(parents=True,exist_ok=False);started=time.monotonic()
            if method in METHODS[:2]:
                candidate=np.load(case/('F_native.npy' if method=='M0_NATIVE' else 'F_common.npy'))
                result=dict(method=method,status='REFERENCE_AVAILABLE',candidate_world=candidate,attempts=[],
                    support_poses=None,support_twists=None,constraint_report=None,factor_costs=None)
            else:
                args=(b,v,common,route['goal_world'],cfg['formulation'])
                kw=dict(obstacle_clearance=lambda xy:env.optimizer_clearance(xy,cfg['footprint']['radius_m']),
                        workspace_margin=lambda xy:env.workspace_margin(xy,cfg['footprint']['radius_m']),gates=route['gates'])
                result=solve_rigid(*args,**kw) if method=='M1_RIGID' else solve_gp(*args,include_obstacles=method=='M3_GP_CONSTRAINED',**kw)
                candidate=result['candidate_world']
            result['total_optimization_and_check_wall_s']=time.monotonic()-started
            write(target/'solver_result.json',result)
            if candidate is not None:array(target/'candidate_world.npy',candidate)
            if result.get('support_poses') is not None:
                array(target/'support_states.npy',result['support_poses']);array(target/'velocity_states.npy',result['support_twists'])
            write(target/'constraint_report.json',result.get('constraint_report'))
            plan=evaluate_plan(method,candidate,result,common,b,v,route,env,cfg);write(target/'plan_validation.json',plan)
            if not result['attempts']:
                attempts.append(dict(case_id=row['case_id'],method=method,initialization_index=None,
                    initialization='N/A: unoptimized reference',termination='N/A',solver_success=None,iterations=None,
                    wall_time_s=result['total_optimization_and_check_wall_s'],method_candidate_found=True,
                    method_plan_valid=plan['plan_valid'],infeasibility_proven=False))
            for i,attempt in enumerate(result['attempts']):
                attempts.append(dict(case_id=row['case_id'],method=method,initialization_index=i,
                    initialization=attempt['initialization'],termination=attempt['termination'],
                    solver_success=attempt['solver_success'],iterations=attempt['iterations'],wall_time_s=attempt['wall_time_s'],
                    method_candidate_found=candidate is not None,method_plan_valid=plan['plan_valid'],infeasibility_proven=False))
            print(json.dumps(dict(case=row['case_id'],method=method,status=result['status'],plan_valid=plan['plan_valid'],wall_s=result['total_optimization_and_check_wall_s'])),flush=True)
    table(run/'aggregate/all_attempts.csv',attempts)
    request=dict(source_root=read(run/'source.json')['source_run'],audit_only=False,horizon_s=3.,control_hz=10,integration_hz=60,
        goal_position_tolerance_m=.15,goal_yaw_tolerance_rad=float(np.pi/12),goal_dwell_s=.2,cases=[])
    for row in rows:
        case=run/'cases'/row['case_directory']
        request['cases'].append(dict(episode_id=row['episode_id'],handoff_id=row['handoff_id'],goal=read(case/'goal_route.json')['goal_world'],
            methods={m:str((case/'methods'/m/'candidate_world.npy').resolve()) if (case/'methods'/m/'candidate_world.npy').is_file() else None for m in METHODS}))
    write(run/'mpc_request.json',request)
    write(run/'optimization_completion.json',dict(all_selected_cases_attempted=True,cases=len(rows),methods=len(METHODS),attempt_records=len(attempts)))


def evaluate(run,mpc_output):
    run=Path(run).resolve();verify_frozen(run);mpc_output=Path(mpc_output).resolve();cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    for record in read(mpc_output/'output_hashes.json')['files']:
        path=(mpc_output/record['path']).resolve()
        if not path.is_relative_to(mpc_output) or digest(path)!=record['sha256']:
            raise ValueError('MPC output hash mismatch or path escape')
    if read(mpc_output/'provenance.json')['request_sha256']!=digest(run/'mpc_request.json'):
        raise ValueError('MPC batch was not produced from this frozen request')
    env=HospitalEnvironment.load(run/'environment');rows=[]
    for case_row in read(run/'case_manifest.json')['selected']:
        case=run/'cases'/case_row['case_directory'];route=read(case/'goal_route.json')
        if not read(mpc_output/case_row['case_directory']/'historical_solve_audit.json')['passed']:
            raise ValueError('historical solve audit failed')
        for method in METHODS:
            target=case/'methods'/method;candidate=(target/'candidate_world.npy').is_file();plan=read(target/'plan_validation.json');solver=read(target/'solver_result.json')
            source=mpc_output/case_row['case_directory']/method
            if candidate:
                if not (source/'rollout.json').is_file():raise ValueError('missing independent candidate rollout')
                rollout=read(source/'rollout.json')
                if rollout['candidate_source']['sha256']!=digest(target/'candidate_world.npy') or not np.array_equal(np.asarray(rollout['candidate_world']),np.load(target/'candidate_world.npy')):
                    raise ValueError('MPC rollout used another candidate')
                frozen=read(case/'input_context.json')
                for key,field in [('initial_pose_world','B_world'),('initial_physical_command','u_minus'),('initial_previous_control','previous_control')]:
                    if not np.array_equal(rollout[key],frozen[field]):raise ValueError('MPC rollout initial condition differs')
                shutil.copytree(source,target/'rollout')
                outcome=evaluate_rollout(rollout,route,env,cfg)
            else:
                outcome=dict(primary_success=False,failure_reasons=['no_candidate'],execution=None,environment=None,route=None,
                    rollout_performed=False,status=solver['status'])
            write(target/'metrics.json',outcome)
            write(target/'hashes.json',{'files':file_records(target)})
            rows.append(dict(case_id=case_row['case_id'],method=method,selected_group=case_row['selected_group'],candidate_found=candidate,
                plan_valid=plan['plan_valid'],primary_success=outcome['primary_success'],failure_reasons=outcome['failure_reasons'],
                optimizer_wall_s=solver['total_optimization_and_check_wall_s'],deformation=plan.get('deformation'),rollout_metrics=outcome))
        write(case/'validation.json',dict(method_count=len(METHODS),all_methods_accounted_for=True,
            historical_solve_audit=read(mpc_output/case_row['case_directory']/'historical_solve_audit.json')))
    summary=comparisons(rows);manifest=read(run/'case_manifest.json')
    summary.update(planned_cases=12,eligible_cases=manifest['eligible_count'],selected_cases=manifest['selected_count'],
        operational_status='GP_SE2_01_RUNTIME_NOT_VALIDATED',research_interpretation='INSUFFICIENT_EVIDENCE',
        interpretation_pending_full_failure_and_paired_review=True,
        limitations=['small dependent development pilot','static authored geometry oracle','idealized synchronous rollout; no optimizer latency in sim time',
                     'sampled continuous GP feasibility; no continuous-time safety proof','no physical robot or general navigation claim'])
    write(run/'aggregate/summary.json',summary);write(run/'aggregate/method_results.json',rows)
    table(run/'aggregate/primary_outcomes.csv',[{k:v for k,v in r.items() if k!='rollout_metrics'} for r in rows])
    table(run/'aggregate/regressions.csv',summary['regressions']);table(run/'aggregate/paired_metrics.csv',summary['paired_metrics'])
    write(run/'evaluation_completion.json',dict(all_cases_methods_evaluated=True,rows=len(rows),mpc_output=str(mpc_output),
        mpc_output_summary_sha256=digest(mpc_output/'summary.json'),new_inference=False))
    print(json.dumps({k:v for k,v in summary.items() if k not in ['paired_metrics','regressions','methods']},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('phase',choices=['prepare','optimize','evaluate']);p.add_argument('--run',type=Path,required=True)
    p.add_argument('--config',type=Path,default=ROOT/'configs/gp_se2_01.yaml');p.add_argument('--environment',type=Path)
    p.add_argument('--source-audit',type=Path,default=ROOT/'data/robotless_gp_se2_01/source_audit_20260918');p.add_argument('--mpc-output',type=Path)
    a=p.parse_args()
    if a.phase=='prepare':prepare(a.run,a.config,a.environment,a.source_audit)
    elif a.phase=='optimize':optimize(a.run)
    else:evaluate(a.run,a.mpc_output)

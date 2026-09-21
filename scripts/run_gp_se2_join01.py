#!/usr/bin/env python3
"""Exclusive JOIN-01 protocol, qualification and common-state comparison."""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
os.environ['JAX_PLATFORMS']='cpu';os.environ['JAX_ENABLE_X64']='true'
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
import yaml
from reconciliation.gp_se2_diag02_validation import write_new as write,file_sha256 as digest
from reconciliation.gp_se2_join01 import METHODS,join_candidates,sustained_join,select_candidate
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_join01_environment import RevealEnvironment,RevealDerivatives
from reconciliation.gp_se2_join01_qualification import qualification_from_episode
from reconciliation.gp_se2_reference import prepare_reference


def read(path):return json.loads(Path(path).read_text())
def sha():return subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
def array(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('xb') as f:np.save(f,np.asarray(value,dtype=np.float64),allow_pickle=False)


def prepare(run):
    run.mkdir(parents=True,exist_ok=False)
    protocol=yaml.safe_load((ROOT/'configs/gp_se2_join_01.yaml').read_text())
    config=yaml.safe_load((ROOT/protocol['collection_base']).read_text())
    config['online'].update(minimum_active_before_prediction_sim_s=.10,maximum_handoff_attempts=1,
        maximum_active_sim_s=8.,postroll_sim_s=.10,maximum_episode_host_s=150.)
    config['paths']['output_root']='data/robotless_gp_se2_join_01'
    config['run_kind']='controlled_obstacle_reveal_single_handoff'
    with (run/'config_snapshot.yaml').open('x') as f:yaml.safe_dump(config,f,sort_keys=False)
    numerical=yaml.safe_load((ROOT/protocol['formulation_base']).read_text())
    with (run/'planning_config.yaml').open('x') as f:yaml.safe_dump(numerical,f,sort_keys=False)
    write(run/'protocol.json',protocol)
    files=[p for base in [ROOT/'src/reconciliation',ROOT/'scripts',ROOT/'tests'] for p in base.rglob('*.py')]
    files += [ROOT/p for p in ['configs/gp_se2_join_01.yaml','configs/gp_se2_01.yaml','configs/robotless_online_handoffs.yaml',
        'scripts/launch_gp_se2_join01_collect.sh','configs/stage0_jackal_controller_validation.yaml','configs/stage0_lightnav_single_chunk.yaml']]
    env=ROOT/protocol['environment'];files += [p for p in env.rglob('*') if p.is_file()]
    authorities=[ROOT/'data/robotless_gp_se2_diag_07/primary_20260920T103000Z/verification/validation.json',
                 ROOT/'data/robotless_gp_se2_diag_08/primary_20260920T151000Z/verification/validation.json']
    for a in authorities:
        if not read(a)['valid']:raise ValueError('prior authoritative validation failed')
    files+=authorities
    write(run/'source.json',dict(execution_sha=sha(),files={str(p):digest(p) for p in sorted(set(files))},
        environment=str(env),official_checkout=str((ROOT/config['paths']['lightnav_checkout']).resolve()),
        protocol_hash=digest(run/'protocol.json'),config_hash=digest(run/'config_snapshot.yaml'),
        planning_hash=digest(run/'planning_config.yaml'),historical_authorities=[str(a) for a in authorities]))
    base=HospitalEnvironment.load(env);p=np.asarray(protocol['initial_pose_world'])
    samples=np.array([[x,y] for x in [18.2,19.2,20.2] for y in [23.,24.,25.,26.]])
    write(run/'placement_design.json',dict(initial=base.query(p[:2]),corridor_samples=[base.query(x) for x in samples],
        rationale='straight southbound corridor with lateral known-free space; 1.5m initial OLD arc design, followed only if upstream qualification fails by predeclared 1.3/1.1m narrower boxes; never extrapolate OLD',
        latency_design='recent request RTT ~0.67s, reveal >=0.10s after OLD activation; recorded B still must pass actual geometry independently',
        placement_absolute_world_pose='resolved once from actual OLD only before FRESH; never from an optimizer outcome'))
    print(run,flush=True)


def verify(run):
    source=read(run/'source.json')
    for p,h in source['files'].items():
        if digest(p)!=h:raise ValueError('frozen source changed: '+p)
    for name,key in [('protocol.json','protocol_hash'),('config_snapshot.yaml','config_hash'),('planning_config.yaml','planning_hash')]:
        if digest(run/name)!=source[key]:raise ValueError('frozen input changed: '+name)
    return source


def qualify(run,episode):
    source=verify(run);protocol=read(run/'protocol.json')
    if episode not in [p['id'] for p in protocol['placements']]:raise ValueError('not in frozen placement schedule')
    result=qualification_from_episode(run/'source_event',episode,HospitalEnvironment.load(source['environment']),protocol)
    write(run/'qualification'/(episode+'.json'),result)
    print(dict(episode=episode,qualified=result['qualified'],reasons=result['failure_reasons']),flush=True)


def freeze_event(run):
    source=verify(run);collection=read(run/'collection_result.json')
    chosen=collection['selected_episode']
    if chosen is None:raise ValueError('NO_QUALIFYING_OBSTACLE_REVEAL_HANDOFF: optimization prohibited')
    checks=[read(run/'qualification'/(p['id']+'.json')) for p in read(run/'protocol.json')['placements'] if (run/'qualification'/(p['id']+'.json')).exists()]
    first=next(q for q in checks if q['qualified'])
    q=read(run/'qualification'/(chosen+'.json'))
    if first['context']['case_id']!=q['context']['case_id']:raise ValueError('not first qualifying event')
    context=q['context'];prep=prepare_reference(context['fresh_world'],context['B_world'],[10.,10.,1.])
    array(run/'inputs/native.npy',context['fresh_world']);array(run/'inputs/common.npy',prep['common_world'])
    write(run/'inputs/context.json',context);write(run/'inputs/preparation.json',prep)
    write(run/'inputs/obstacle.json',q['obstacle']);write(run/'inputs/goal_route.json',q['goal_route'])
    candidates=join_candidates(context['fresh_world'],context['B_world'])
    files=[p for p in (run/'source_event').rglob('*') if p.is_file()]+[p for p in (run/'inputs').rglob('*') if p.is_file()]
    write(run/'event_freeze.json',dict(case_id=context['case_id'],candidates=candidates,
        files={str(p):digest(p) for p in files},execution_sha=sha(),max_m4_starts=2*len(candidates),
        method_order=list(METHODS),no_more_upstream_requests=True))


def load_case(run):
    source=verify(run);freeze=read(run/'event_freeze.json')
    for p,h in freeze['files'].items():
        if digest(p)!=h:raise ValueError('frozen event changed: '+p)
    ctx=read(run/'inputs/context.json');config=yaml.safe_load((run/'planning_config.yaml').read_text())
    env=RevealEnvironment(HospitalEnvironment.load(source['environment']),read(run/'inputs/obstacle.json'))
    case=dict(context=ctx,config=config,environment=env,goal_route=read(run/'inputs/goal_route.json'),
              common=np.load(run/'inputs/common.npy'),native=np.load(run/'inputs/native.npy'))
    return case


def optimize(run):
    from reconciliation.gp_se2_formulation import GPProblem,solve_rigid
    from reconciliation.gp_se2_diag_initialization import same_curvature_deceleration_seed
    from reconciliation.gp_se2_diag02_derivatives import DerivativeProvider,DerivativeError
    from reconciliation.gp_se2_diag04_constraints import ConstraintView
    from reconciliation.gp_se2_diag04_solver import run_refined
    from reconciliation.gp_se2_join01_formulation import JoinView,JoinDerivatives,verify_derivatives
    from reconciliation.gp_se2_evaluation import evaluate_plan
    case=load_case(run);ctx=case['context'];config=case['config'];env=case['environment'];c=config['formulation']
    (run/'methods').mkdir(exist_ok=False)
    b=ctx['B_world'];eta=np.array([ctx['u_minus'][0],0.,ctx['u_minus'][1]])
    kwargs=dict(boundary_pose=b,initial_twist=eta,common_reference=case['common'],goal_pose=case['goal_route']['goal_world'],
        config=c,obstacle_clearance=lambda xy:env.optimizer_clearance(xy,.2),workspace_margin=lambda xy:env.workspace_margin(xy,.2),gates=case['goal_route']['gates'])
    base=GPProblem(**kwargs);ed=RevealDerivatives(env,.2)
    seeds=[base.initializations()[0],same_curvature_deceleration_seed(base)]
    vectors=[base.vector(s['poses'],s['twists']) for s in seeds]
    for i,z in enumerate(vectors):array(run/'inputs'/f'seed_{i}.npy',z)
    outcomes={};starts=[]
    for method in METHODS:
        folder=run/'methods'/method;folder.mkdir()
        begin=time.perf_counter();candidate=None;plan=None;selected=None
        if method in ('M0_NATIVE','M0_ADAPTER'):
            candidate=case['native'].copy() if method=='M0_NATIVE' else case['common'].copy()
            plan=evaluate_plan(method,candidate,{},case['common'],b,eta,case['goal_route'],env,config)
        elif method=='M1_RIGID':
            result=solve_rigid(**kwargs);write(folder/'solver_result.json',result)
            candidate=result['candidate_world']
            plan=evaluate_plan(method,candidate,result,case['common'],b,eta,case['goal_route'],env,config)
        else:
            specifications=read(run/'event_freeze.json')['candidates'] if method=='M4_JOIN_GP' else [None]
            eligible=[]
            for spec in specifications:
                label='generic' if spec is None else spec['label']
                for i,z in enumerate(vectors):
                    target=folder/'starts'/f'{label}_I{i}';target.mkdir(parents=True,exist_ok=False)
                    construction=time.perf_counter();view=ConstraintView(base) if spec is None else JoinView(base,case['native'],spec)
                    record=dict(method=method,label=label,initialization=f'I{i}',candidate=spec,
                                seed_hash=hashlib.sha256(z.tobytes()).hexdigest(),solver_invoked=False)
                    try:
                        dp=DerivativeProvider(base,ed);provider=dp if spec is None else JoinDerivatives(view,dp)
                        record['construction_s']=time.perf_counter()-construction
                        warm=time.perf_counter();provider.warmup(z);record['warmup_s']=time.perf_counter()-warm
                        check=time.perf_counter();gate=verify_derivatives(view,provider,z);record['derivative_check_s']=time.perf_counter()-check
                        write(target/'derivative_check.json',gate)
                        if not gate['passed']:
                            record.update(status='DERIVATIVE_VALIDATION_FAILED');write(target/'status.json',record);starts.append(record);continue
                        result=run_refined(view,z,derivative_provider=provider,case=case,initialization_name=seeds[i]['name'])
                        record.update(solver_invoked=True,status=result['termination'],solver_status=result['solver_status'],
                            iterations=result['iterations'],solve_s=result['solve_wall_time_s'],check_s=result['post_solve_validation_time_s'])
                        write(target/'solver_result.json',result)
                        # Existing baseline retention is preserved. M4 explicitly selects
                        # among independently full-valid retained actual records.
                        if spec is None:
                            if result['selected_full_feasible']:
                                eligible.append(dict(full_valid=True,join_time_s=0.,join_index=0,objective=result['candidate_objective'],
                                    initialization=f'I{i}',source_label=result['selected_iterate'],vector=result['candidate_vector'],path=str(target)))
                        else:
                            tube_checks=[]
                            for checkrow in result['candidate_checks']:
                                if not checkrow['grid_and_full_feasible']:continue
                                tube=view.independent_tube_check(checkrow['vector'])
                                tube_checks.append(dict(source=checkrow['iterate'],**tube))
                                if tube['valid']:
                                    eligible.append(dict(full_valid=True,**spec,objective=checkrow['objective'],initialization=f'I{i}',
                                        source_label=checkrow['iterate'],vector=checkrow['vector'],path=str(target)))
                            write(target/'join_tube_checks.json',tube_checks)
                    except (DerivativeError,ValueError,FloatingPointError,np.linalg.LinAlgError) as exc:
                        record.update(status='IMPLEMENTATION_OR_DERIVATIVE_LIMITATION',error=f'{type(exc).__name__}: {exc}')
                    write(target/'status.json',record);starts.append(record)
            selected=select_candidate(eligible)
            write(folder/'eligible_candidates.json',eligible)
            if selected is not None:
                p,v=base.unpack(selected['vector']);candidate=p[1:]
                selected_result=dict(support_poses=p,support_twists=v,support_times=base.times)
                write(folder/'selected.json',selected);write(folder/'support.json',selected_result)
                plan=evaluate_plan('M3_GP_CONSTRAINED',candidate,selected_result,case['common'],b,eta,case['goal_route'],env,config)
                from reconciliation.gp_se2 import sample_gp
                from reconciliation.gp_se2_diag_acceptance import offset_grid
                qt=offset_grid(base.times);pp=sample_gp(base.times,p,v,qt)[0]
                write(folder/'planned_join.json',sustained_join(qt,pp,case['native']))
        if candidate is not None:array(folder/'reference.npy',candidate)
        write(folder/'plan.json',plan if plan is not None else dict(plan_valid=False,status='NO_FULL_VALID_CANDIDATE'))
        outcomes[method]=dict(candidate_available=candidate is not None,plan_valid=False if plan is None else plan['plan_valid'],
            reference_path=None if candidate is None else str(folder/'reference.npy'),optimizer_construction_solve_check_wall_s=time.perf_counter()-begin,
            selected=selected,rollout_policy='GP full-valid only; native/adapter/rigid retain historical diagnostic policy',
            failure_reason=None if candidate is not None else 'NO_FULL_VALID_CANDIDATE')
    write(run/'method_manifest.json',outcomes);write(run/'all_starts.json',starts)
    write(run/'comparison_freeze.json',dict(files={str(p):digest(p) for p in (run/'methods').rglob('*') if p.is_file()},
        actual_gp_solves=sum(r['solver_invoked'] for r in starts),rigid_solve_starts=2))


def evaluate(run):
    from reconciliation.gp_se2_evaluation import evaluate_rollout
    case=load_case(run);methods=read(run/'method_manifest.json');outcomes={}
    for method in METHODS:
        path=run/'rollouts'/method/'rollout.json'
        if not path.exists():
            outcomes[method]=dict(methods[method],rollout_performed=False,join_time_s=None,join_success=False,execution_success=None)
            continue
        roll=read(path);ev=evaluate_rollout(roll,case['goal_route'],case['environment'],case['config'])
        metric=sustained_join(ev['dense_times_s'],ev['dense_poses_world'],case['native'])
        write(run/'evaluation'/method/'execution.json',ev);write(run/'evaluation'/method/'join.json',metric)
        planned=run/'methods'/method/'planned_join.json';planned_time=read(planned)['sustained_join_time_s'] if planned.exists() else None
        jt=metric['sustained_join_time_s']
        outcomes[method]=dict(methods[method],rollout_performed=True,join_time_s=jt,join_success=metric['join_success'],
            planned_join_time_s=planned_time,plan_execution_join_difference_s=None if jt is None or planned_time is None else jt-planned_time,
            execution_success=ev['primary_success'],execution_failure_reasons=ev['failure_reasons'],
            minimum_clearance_m=ev['minimum_clearance_m'],metrics=ev['execution'],
            mpc_solve_wall_s=sum(t for t in roll['official_mpc_solve_wall_s'] if t is not None))
    write(run/'outcomes.json',outcomes)


def main():
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['prepare','verify','qualify','freeze_event','optimize','evaluate']);p.add_argument('--run',type=Path,required=True);p.add_argument('--episode')
    a=p.parse_args();run=a.run.resolve()
    if a.stage=='qualify':qualify(run,a.episode)
    else:globals()[a.stage](run)


if __name__=='__main__':main()

#!/usr/bin/env python3
"""ATTACH-01 source freeze; comparison may begin only after source qualification."""
from __future__ import annotations
import argparse
import os
from datetime import datetime, timezone
from pathlib import Path
import shutil
import subprocess
import sys
import time

for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'): os.environ[key]='1'
os.environ['JAX_PLATFORMS']='cpu'; os.environ['JAX_ENABLE_X64']='true'

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
import numpy as np
import yaml
from reconciliation.gp_se2_attach01_source import scan, read, digest
from reconciliation.gp_se2_diag02_validation import write_new as write
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_attach01 import (method_schedule, FixedAttachView, AttachDerivatives,
    verify_derivatives, select_full_valid, sustained_join, rejection_reasons)


def sha():
    return subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()


def array(path, value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('xb') as stream:np.save(stream,np.asarray(value,dtype=np.float64),allow_pickle=False)


def verify(run):
    source=read(run/'source.json')
    for path, expected in source['files'].items():
        if digest(path)!=expected:raise ValueError('preserved source changed: '+path)
    for name,key in [('protocol.json','protocol_sha256'),('config_snapshot.yaml','configuration_sha256')]:
        if digest(run/name)!=source[key]:raise ValueError('frozen configuration changed: '+name)
    official=source['official_mpc']
    if digest(official['mpc_source'])!=official['mpc_source_sha256']:raise ValueError('official MPC changed')
    return source


def load_case(run):
    source=verify(run)
    if read(run/'eligibility_ledger.json')['selected_case_id'] is None:
        raise ValueError('no eligible genuine source; comparison prohibited')
    config=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    env=HospitalEnvironment.load(source['environment'])
    return dict(context=read(run/'inputs/context.json'),common=np.load(run/'inputs/common.npy'),
        native=np.load(run/'inputs/native.npy'),goal_route=read(run/'inputs/goal_route.json'),config=config,environment=env)


def base_problem(case):
    from reconciliation.gp_se2_formulation import GPProblem
    ctx=case['context'];env=case['environment'];radius=case['config']['footprint']['radius_m']
    return GPProblem(boundary_pose=ctx['B_world'],initial_twist=[ctx['u_minus'][0],0.,ctx['u_minus'][1]],
        common_reference=case['common'],goal_pose=case['goal_route']['goal_world'],config=case['config']['formulation'],
        obstacle_clearance=lambda xy:env.optimizer_clearance(xy,radius),
        workspace_margin=lambda xy:env.workspace_margin(xy,radius),gates=case['goal_route']['gates'])


def derivatives(base, environment):
    from reconciliation.gp_se2_diag02_derivatives import DerivativeProvider
    from reconciliation.gp_se2_diag02_environment import EnvironmentDerivatives
    return DerivativeProvider(base,EnvironmentDerivatives(environment,.20))


def seed_vectors(base):
    from reconciliation.gp_se2_diag_initialization import same_curvature_deceleration_seed
    seeds=[base.initializations()[0],same_curvature_deceleration_seed(base)]
    return [(f'I{i}',s['name'],base.vector(s['poses'],s['twists'])) for i,s in enumerate(seeds)]


def preflight(run):
    """No optimization: freeze seeds and verify each fixed condition's derivatives."""
    from reconciliation.gp_se2_diag04_constraints import ConstraintView
    from reconciliation.gp_se2_diag02_derivatives import DerivativeError
    case=load_case(run);base=base_problem(case);directory=run/'preflight';directory.mkdir(exist_ok=False)
    seeds=seed_vectors(base)
    for label,name,z in seeds:array(run/'inputs'/(label+'.npy'),z)
    started=time.perf_counter();provider=derivatives(base,case['environment']);checks=[]
    for method,t in method_schedule()[2:]:
        view=ConstraintView(base) if t is None else FixedAttachView(base,t)
        dp=provider if t is None else AttachDerivatives(view,provider)
        for label,name,z in seeds:
            begin=time.perf_counter()
            try:
                dp.warmup(z);gate=verify_derivatives(view,dp,z)
                value=view.evaluate(z)
                parity=dict(original_equalities_literal=np.array_equal(value['equality'],base.evaluate(z)['equality']),
                    original_inequality_prefix_literal=np.array_equal(value['inequality'][:903],base.evaluate(z)['inequality']),
                    equality_Jacobian_literal=np.array_equal(dp.equality_jacobian(z),provider.equality_jacobian(z)),
                    inequality_Jacobian_prefix_literal=np.array_equal(dp.inequality_jacobian(z)[:903],provider.inequality_jacobian(z)))
                gate.update(parity=parity,passed=gate['passed'] and all(parity.values()),
                    variables=len(z),equality_rows=len(value['equality']),inequality_rows=len(value['inequality']))
            except DerivativeError as exc:gate=dict(passed=False,error=str(exc),reason_code=exc.reason_code)
            gate.update(method=method,duration_s=t,seed=label,seed_file_sha256=digest(run/'inputs'/(label+'.npy')),
                common_file_sha256=digest(run/'inputs/common.npy'),wall_s=time.perf_counter()-begin)
            write(directory/(method+'_'+label+'.json'),gate);checks.append(gate)
            print(method,label,'derivative passed',gate['passed'],flush=True)
    write(directory/'summary.json',dict(all_passed=all(r['passed'] for r in checks),checks=checks,
        wall_s=time.perf_counter()-started,optimization_calls=0))


def freeze(run):
    verify(run)
    if subprocess.check_output(['git','rev-parse','origin/main'],cwd=ROOT,text=True).strip()!=sha():
        raise ValueError('pre-primary implementation must first be committed and pushed')
    dirty=subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True)
    allowed={'configs/stage0_jackal_controller_validation.yaml','configs/stage0_lightnav_single_chunk.yaml'}
    if any(line[3:] not in allowed for line in dirty.splitlines()):raise ValueError('uncommitted experiment implementation')
    files={str(p):digest(p) for p in run.rglob('*') if p.is_file()}
    for directory in ['src/reconciliation','scripts','tests']:
        for p in (ROOT/directory).rglob('*.py'):files[str(p)]=digest(p)
    for name in ['gp_se2_attach_01.yaml','gp_se2_01.yaml']:
        p=ROOT/'configs'/name;files[str(p)]=digest(p)
    write(run/'execution_freeze.json',dict(execution_sha=sha(),files=files,no_primary_optimization_yet=True,
        schedule=[dict(method=m,duration_s=t,seed=label)for m,t in method_schedule()[2:]for label in ['I0','I1']],
        primary_GP_starts_planned=14,conditions=6,no_retry=True,correspondence_decision_variable=False))


def check_freeze(run):
    frozen=read(run/'execution_freeze.json')
    for path,expected in frozen['files'].items():
        if digest(path)!=expected:raise ValueError('execution freeze changed: '+path)
    return frozen


def optimize(run):
    from reconciliation.gp_se2_diag04_constraints import ConstraintView
    from reconciliation.gp_se2_diag04_solver import run_refined
    from reconciliation.gp_se2_evaluation import evaluate_plan
    from reconciliation.gp_se2_diag02_derivatives import DerivativeError
    from reconciliation.gp_se2_diag_acceptance import offset_grid
    from reconciliation.gp_se2 import sample_gp
    check_freeze(run);case=load_case(run);base=base_problem(case)
    root=run/'methods';root.mkdir(exist_ok=False)
    began=time.perf_counter();provider=derivatives(base,case['environment'])
    provider_construction_s=time.perf_counter()-began
    starts=[];methods={}
    for method,t in method_schedule():
        folder=root/method;folder.mkdir();method_begin=time.perf_counter()
        candidate=plan=selected=None;eligible=[];condition_starts=[]
        if method in ('M0_NATIVE','M0_ADAPTER'):
            candidate=case['native'].copy() if method=='M0_NATIVE' else case['common'].copy()
            plan=evaluate_plan(method,candidate,{},case['common'],base.boundary_pose,base.initial_twist,
                case['goal_route'],case['environment'],case['config'])
        else:
            construction=time.perf_counter()
            view=ConstraintView(base) if t is None else FixedAttachView(base,t)
            dp=provider if t is None else AttachDerivatives(view,provider)
            view_construction_s=time.perf_counter()-construction
            for label,name,unused in seed_vectors(base):
                target=folder/'starts'/label;target.mkdir(parents=True,exist_ok=False)
                z=np.load(run/'inputs'/(label+'.npy'))
                if not np.array_equal(z,unused):raise ValueError('saved original initialization changed')
                record=dict(method=method,duration_s=t,initialization=label,seed_file_sha256=digest(run/'inputs'/(label+'.npy')),
                    common_file_sha256=digest(run/'inputs/common.npy'),solver_invoked=False,
                    construction_s=view_construction_s if label=='I0' else 0.)
                gate=read(run/'preflight'/(method+'_'+label+'.json'))
                if not gate['passed']:
                    record.update(status='DERIVATIVE_VALIDATION_FAILED',failure_reasons=['pre-primary derivative gate failed'])
                else:
                    try:
                        warm=time.perf_counter();dp.warmup(z);record['warmup_s']=time.perf_counter()-warm
                        record['solver_invoked']=True
                        result=run_refined(view,z,derivative_provider=dp,case=case,initialization_name=name)
                        write(target/'solver_result.json',result)
                        inspected=[]
                        for c in result['candidate_checks']:
                            tube=None if t is None else view.independent_tube_check(c['vector'])
                            valid=bool(c['grid_and_full_feasible'] and (tube is None or tube['valid']))
                            inspected.append(dict(source=c['iterate'],full_valid=valid,tube=tube,
                                rejection_reasons=rejection_reasons(c,tube),objective=c['objective']))
                            # Generic M3 keeps the historical selected record. M4 uses
                            # full-valid retained records, independently within each T.
                            if valid and (t is not None or c['iterate']==result['selected_iterate']):
                                eligible.append(dict(full_valid=True,objective=c['objective'],initialization=label,
                                    source_label=c['iterate'],vector=c['vector'],path=str(target)))
                        write(target/'candidate_acceptance.json',inspected)
                        latest=next(c for c in inspected if c['source']=='latest_iterate')
                        diagnosis_started=time.perf_counter()
                        last_full=result['candidate_checks'][1]['full_acceptance']
                        if 'additional_grid' in last_full:
                            sampled=last_full['additional_grid']
                            latest_metric=sustained_join(sampled['query_times_s'],sampled['poses_world'],case['native'])
                            write(target/'latest_planned_attachment.json',dict(latest_metric,
                                full_valid=latest['full_valid'],role='latest iterate diagnostic; not an executable candidate unless full-valid'))
                        else:latest_metric=None
                        record.update(status=result['termination'],solver_status=result['solver_status'],
                            iterations=result['iterations'],solve_s=result['solve_wall_time_s'],
                            check_s=result['post_solve_validation_time_s'],latest_full_valid=latest['full_valid'],
                            latest_failure_reasons=latest['rejection_reasons'],latest_tube=latest['tube'],
                            initial_full_valid=inspected[0]['full_valid'],full_valid_retained=any(c['full_valid'] for c in inspected),
                            latest_diagnostic_planned_attachment_s=None if latest_metric is None else latest_metric['sustained_join_time_s'],
                            attachment_diagnostic_s=time.perf_counter()-diagnosis_started,
                            objective_evaluations=result['objective_evaluations'],derivative_calls=result['derivative_calls'])
                    except (DerivativeError,FloatingPointError,np.linalg.LinAlgError) as exc:
                        record.update(status='NUMERICAL_OR_DERIVATIVE_FAILURE',error=str(exc))
                write(target/'status.json',record);starts.append(record);condition_starts.append(record)
                print(method,label,record['status'],record.get('latest_failure_reasons'),flush=True)
            selected=select_full_valid(eligible)
            write(folder/'eligible_candidates.json',eligible)
            if selected is not None:
                p,v=base.unpack(selected['vector']);candidate=p[1:]
                support=dict(support_times=base.times,support_poses=p,support_twists=v)
                write(folder/'selected.json',selected);write(folder/'support.json',support)
                plan=evaluate_plan('M3_GP_CONSTRAINED',candidate,support,case['common'],base.boundary_pose,base.initial_twist,
                    case['goal_route'],case['environment'],case['config'])
                if not plan['plan_valid']:raise ValueError('full-valid selection disagrees with original independent plan checker')
                qt=offset_grid(base.times)
                write(folder/'planned_attachment.json',sustained_join(qt,sample_gp(base.times,p,v,qt)[0],case['native']))
        if candidate is not None:array(folder/'reference.npy',candidate)
        write(folder/'plan.json',plan if plan is not None else dict(plan_valid=False,status='NO_FULL_VALID_CANDIDATE'))
        methods[method]=dict(method=method,duration_s=t,candidate_available=candidate is not None,
            plan_valid=False if plan is None else plan['plan_valid'],selected=selected,
            reference_path=None if candidate is None else str(folder/'reference.npy'),
            reference_sha256=None if candidate is None else digest(folder/'reference.npy'),
            fixed_common_sha256=digest(run/'inputs/common.npy'),condition_starts=condition_starts,
            condition_wall_s=time.perf_counter()-method_begin,
            failure_reason=None if candidate is not None else 'NO_FULL_VALID_CANDIDATE',
            rollout_policy='full-valid GP only; unchanged native/adapter spatial-reference diagnostic policy')
    write(run/'method_manifest.json',methods);write(run/'all_starts.json',starts)
    write(run/'comparison_freeze.json',dict(files={str(p):digest(p)for p in root.rglob('*')if p.is_file()},
        actual_GP_starts=sum(r['solver_invoked']for r in starts),provider_construction_s=provider_construction_s,
        total_optimization_phase_s=time.perf_counter()-began,execution_sha=sha(),retry_count=0))


def evaluate(run):
    from reconciliation.gp_se2_evaluation import evaluate_rollout
    case=load_case(run);methods=read(run/'method_manifest.json');rows={}
    for method,definition in methods.items():
        path=run/'rollouts'/method/'rollout.json'
        planned=run/'methods'/method/'planned_attachment.json'
        pt=read(planned)['sustained_join_time_s'] if planned.exists() else None
        row=dict(method=method,duration_s=definition['duration_s'],candidate_available=definition['candidate_available'],
            plan_valid=definition['plan_valid'],failure_reason=definition['failure_reason'],planned_attachment_s=pt,
            rollout_performed=path.exists(),executed_attachment_s=None,execution_join_success=None,execution_success=None,
            minimum_clearance_m=None,motion_valid=None,workspace_valid=None,route_valid=None,goal_valid=None,
            attachment_relative_to_T=None,mpc_solve_wall_s=None)
        if path.exists():
            roll=read(path);ev=evaluate_rollout(roll,case['goal_route'],case['environment'],case['config'])
            metric=sustained_join(ev['dense_times_s'],ev['dense_poses_world'],case['native'])
            write(run/'evaluation'/method/'execution.json',ev);write(run/'evaluation'/method/'attachment.json',metric)
            jt=metric['sustained_join_time_s'];t=definition['duration_s']
            row.update(executed_attachment_s=jt,execution_join_success=metric['join_success'],execution_success=ev['primary_success'],
                failure_reason=ev['failure_reasons'],minimum_clearance_m=ev['minimum_clearance_m'],
                motion_valid=ev['execution']['motion_limits_pass'],workspace_valid=ev['environment']['workspace_known'],
                route_valid=ev['route']['valid'],goal_valid=ev['execution']['terminal_goal_dwell_pass'],
                attachment_relative_to_T=None if t is None or jt is None else ('AT' if abs(jt-t)<1e-12 else 'BEFORE' if jt<t else 'AFTER'),
                plan_execution_attachment_difference_s=None if pt is None or jt is None else jt-pt,
                post_attachment_mean_distance_m=metric['post_join_mean_distance_m'],post_attachment_max_distance_m=metric['post_join_max_distance_m'],
                pre_attachment_position_auc_m_s=metric['pre_join_position_auc_m_s'],
                full_window_position_auc_m_s=metric['full_window_position_auc_m_s'],
                execution_metrics=ev['execution'],mpc_solve_wall_s=sum(v for v in roll['official_mpc_solve_wall_s']if v is not None))
        rows[method]=row
    valid=[r['duration_s']for r in rows.values()if r['duration_s'] is not None and r['plan_valid']]
    write(run/'outcomes.json',rows)
    write(run/'summary.json',dict(shortest_tested_full_valid_duration_s=min(valid)if valid else None,
        tested_duration_set_s=[t for m,t in method_schedule()if t is not None],
        actual_GP_starts=read(run/'comparison_freeze.json')['actual_GP_starts'],
        actual_rollouts=sum(r['rollout_performed']for r in rows.values()),
        actual_primary_MPC_solves=sum(len(read(run/'rollouts'/m/'rollout.json')['controller_reference_selections'])for m,r in rows.items()if r['rollout_performed']),
        new_VLA_inferences=0,new_GUI=0,correspondence_optimized=False,duration_optimized=False,
        outcome_taxonomy='FORMULATION_LIMITATION' if not valid else 'EXECUTION_RESULTS_REQUIRE_PAIRED_INTERPRETATION'))


def prepare(run):
    run.mkdir(parents=True, exist_ok=False)
    begin = time.perf_counter()
    config_path = ROOT/'configs/gp_se2_attach_01.yaml'
    protocol = yaml.safe_load(config_path.read_text())
    write(run/'protocol.json', protocol)
    original = ROOT/protocol['original_run']/'config_snapshot.yaml'
    shutil.copyfile(original, run/'config_snapshot.yaml')
    provenance = read(ROOT/protocol['source_inventory_run']/'source.json')
    environment = HospitalEnvironment.load(provenance['environment_path'])
    result = scan(ROOT, protocol, environment)
    prepared, context = result.pop('prepared_selected'), result.pop('selected_context')
    write(run/'eligibility_ledger.json', result)
    if context is not None:
        inputs = run/'inputs'; inputs.mkdir()
        write(inputs/'context.json', context); write(inputs/'preparation.json', prepared)
        for name, value in [('native',context['fresh_world']), ('common',prepared['common_world'])]:
            with (inputs/(name+'.npy')).open('xb') as stream:
                np.save(stream, np.asarray(value,dtype=np.float64), allow_pickle=False)
        goal = dict(goal_world=prepared['goal_world'], gates=[],route_status='NOT_REQUIRED_CLEAR_SHORTCUT')
        write(inputs/'goal_route.json',goal)
    preserved = dict(result['source_hashes'])
    for name in ['gp_se2.py','se2.py','gp_se2_formulation.py','gp_se2_environment.py',
                 'gp_se2_diag_acceptance.py','gp_se2_diag02_ad.py','gp_se2_diag02_derivatives.py',
                 'gp_se2_reference.py','gp_se2_rollout.py','gp_se2_evaluation.py','gp_se2_join01.py']:
        p=ROOT/'src/reconciliation'/name;preserved[str(p)]=digest(p)
    prior = ROOT/'data/robotless_gp_se2_join_01/primary_20260921T083025Z'
    for p in prior.rglob('*'):
        if p.is_file():preserved[str(p)]=digest(p)
    write(run/'source.json',dict(starting_sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        created_utc=datetime.now(timezone.utc).isoformat(), environment=provenance['environment_path'],
        environment_export_source=provenance['environment_export_source'],
        official_mpc=provenance['official_mpc'], files=preserved,
        protocol_sha256=digest(run/'protocol.json'), configuration_sha256=digest(run/'config_snapshot.yaml'),
        scan_wall_s=time.perf_counter()-begin, source_only=True,
        new_LightNav_inferences=0,new_GP_solves=0,new_MPC_solves=0,new_rollouts=0,new_GUI=0))
    print({key:result[key] for key in ['status','selected_case_id','source_event_count','eligible_count','cumulative_pass_counts']},flush=True)


if __name__ == '__main__':
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=['prepare','preflight','freeze','optimize','evaluate'])
    parser.add_argument('--run',type=Path,required=True);args=parser.parse_args()
    globals()[args.action](args.run.resolve())

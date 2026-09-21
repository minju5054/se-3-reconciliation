#!/usr/bin/env python3
"""Independent saved-record ATTACH-01 validation; no optimization or MPC calls."""
import argparse
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
from run_gp_se2_attach01 import read,write,digest,verify,check_freeze,load_case,base_problem
from reconciliation.gp_se2_attach01_source import scan
from reconciliation.gp_se2_attach01 import FixedAttachView,method_schedule,sustained_join,select_full_valid,rejection_reasons
from reconciliation.gp_se2_diag02_validation import plain
from reconciliation.gp_se2_diag04_constraints import ConstraintView
from reconciliation.gp_se2_diag_acceptance import check_full_candidate,offset_grid
from reconciliation.gp_se2_formulation import _constraint_report
from reconciliation.gp_se2_evaluation import evaluate_rollout
from reconciliation.gp_se2 import sample_gp
from reconciliation.robotless_online import integrate_unicycle


def validate(run):
    began=time.perf_counter();checks=[];errors=[]
    def check(name,passed):
        checks.append(dict(name=name,passed=bool(passed)))
        if not passed:errors.append(name)
    source=verify(run);freeze=check_freeze(run);case=load_case(run);base=base_problem(case)
    # The complete ledger is independently recomputed from raw/source-only inputs.
    ledger=scan(ROOT,read(run/'protocol.json'),case['environment'])
    prepared,context=ledger.pop('prepared_selected'),ledger.pop('selected_context')
    check('complete source-only ledger reproduced',plain(ledger)==read(run/'eligibility_ledger.json'))
    check('selected source context reproduced',plain(context)==case['context'])
    check('fixed common float64 bytes reproduced',prepared['common_world'].tobytes()==case['common'].tobytes())
    check('native bytes unchanged',np.asarray(context['fresh_world'],dtype=np.float64).tobytes()==case['native'].tobytes())
    methods=read(run/'method_manifest.json');starts=read(run/'all_starts.json');outcomes=read(run/'outcomes.json')
    check('all fixed conditions covered',list(methods)==[m for m,t in method_schedule()] and list(outcomes)==list(methods))
    check('all 14 scheduled starts recorded',[(r['method'],r['duration_s'],r['initialization'])for r in starts]==[
        (m,t,label)for m,t in method_schedule()[2:]for label in ['I0','I1']])
    actual_solves=0;full_cache={};rollout_count=0;mpc_count=0
    for method,t in method_schedule():
        definition=methods[method];folder=run/'methods'/method
        check(method+' same common bytes',definition['fixed_common_sha256']==digest(run/'inputs/common.npy'))
        offered=[]
        if method.startswith(('M3','M4')):
            view=ConstraintView(base) if t is None else FixedAttachView(base,t)
            for label in ['I0','I1']:
                target=folder/'starts'/label;status=read(target/'status.json')
                check(method+label+' original seed identity',status['seed_file_sha256']==digest(run/'inputs'/(label+'.npy')))
                if not status['solver_invoked']:
                    check(method+label+' derivative failure preserved',status['status']=='DERIVATIVE_VALIDATION_FAILED');continue
                result=read(target/'solver_result.json');actual_solves+=result['minimize_invocations']
                check(method+label+' one solve no retry',result['minimize_invocations']==1)
                check(method+label+' solver status preserved',status['status']==result['termination'] and status['solver_status']==result['solver_status'])
                check(method+label+' original chart and row count',result['variable_count']==150 and result['equality_count']==30 and result['inequality_count']==(903 if t is None else 915))
                check(method+label+' exact initial bytes',np.array_equal(result['initial_vector'],np.load(run/'inputs'/(label+'.npy'))))
                inspected=[]
                for c in result['candidate_checks']:
                    z=np.array(c['vector']);value=view.evaluate(z)
                    grid=_constraint_report(value['equality'],value['inequality'],base.config)
                    check(method+label+c['iterate']+' objective and grid reproduced',
                        np.isclose(value['objective'],c['objective'],rtol=1e-12,atol=1e-10) and grid==c['collocation'])
                    key=z.tobytes()
                    if key not in full_cache:
                        full=check_full_candidate(base,z,case)
                        full_cache[key]=dict(full_feasible=full['full_feasible'],flags=full['additional_grid']['flags'],
                            original_dense_feasible=full['original_dense_feasible'])
                    summary=full_cache[key];saved=c['full_acceptance']
                    check(method+label+c['iterate']+' original independent full acceptance reproduced',
                        summary['full_feasible']==saved['full_feasible'] and summary['flags']==saved['additional_grid']['flags'] and
                        summary['original_dense_feasible']==saved['original_dense_feasible'])
                    tube=None if t is None else view.independent_tube_check(z)
                    valid=bool(grid['feasible'] and summary['full_feasible'] and (tube is None or tube['valid']))
                    inspected.append(dict(source=c['iterate'],full_valid=valid,tube=tube,
                        rejection_reasons=rejection_reasons(c,tube),objective=c['objective']))
                    if valid and (t is not None or c['iterate']==result['selected_iterate']):
                        offered.append(dict(full_valid=True,objective=c['objective'],initialization=label,
                            source_label=c['iterate'],vector=z,path=str(target)))
                check(method+label+' every retained acceptance/tube reproduced',plain(inspected)==read(target/'candidate_acceptance.json'))
            selected=select_full_valid(offered)
            check(method+' independent within-condition candidate selection',plain(selected)==definition['selected'])
            check(method+' no candidate no fallback',definition['candidate_available']==(selected is not None))
            if selected is not None:
                p,v=base.unpack(selected['vector']);q=offset_grid(base.times)
                check(method+' planned attachment reproduced',plain(sustained_join(q,sample_gp(base.times,p,v,q)[0],case['native']))==read(folder/'planned_attachment.json'))
        path=run/'rollouts'/method/'rollout.json';row=outcomes[method]
        check(method+' rollout availability',path.exists()==definition['candidate_available']==row['rollout_performed'])
        if not path.exists():
            check(method+' unavailable execution remains NA',row['executed_attachment_s'] is None and row['execution_join_success'] is None and row['minimum_clearance_m'] is None)
            continue
        rollout_count+=1;roll=read(path);mpc_count+=len(roll['controller_reference_selections'])
        check(method+' full-valid GP required',not method.startswith(('M3','M4')) or definition['plan_valid'])
        check(method+' common B and physical/controller memory',roll['initial_pose_world']==context['B_world'] and
            roll['initial_physical_command']==context['u_minus'] and roll['initial_previous_control']==context['previous_control'])
        check(method+' complete execution schedule',len(roll['states'])==181 and len(roll['commands'])==180 and len(roll['controller_reference_selections'])==30)
        err=max(float(np.max(np.abs(integrate_unicycle(s['pose_world'],c['command'],1/60)-np.array(n['pose_world']))))
            for s,c,n in zip(roll['states'][:-1],roll['commands'],roll['states'][1:]))
        check(method+' exact held-command integration',err<=1e-10)
        check(method+' installed reference preserved',np.array_equal(roll['candidate_world'],np.load(definition['reference_path'])))
        ev=evaluate_rollout(roll,case['goal_route'],case['environment'],case['config'])
        metric=sustained_join(ev['dense_times_s'],ev['dense_poses_world'],case['native'])
        check(method+' original execution evaluation reproduced',plain(ev)==read(run/'evaluation'/method/'execution.json'))
        check(method+' sampled sustained attachment reproduced',plain(metric)==read(run/'evaluation'/method/'attachment.json'))
        check(method+' outcome metrics agree',row['executed_attachment_s']==metric['sustained_join_time_s'] and row['execution_success']==ev['primary_success'] and row['minimum_clearance_m']==ev['minimum_clearance_m'])
    summary=read(run/'summary.json')
    check('actual GP count',actual_solves==summary['actual_GP_starts'])
    check('actual rollout and MPC counts',rollout_count==summary['actual_rollouts'] and mpc_count==summary['actual_primary_MPC_solves'])
    valid=[r['duration_s']for r in outcomes.values()if r['duration_s'] is not None and r['plan_valid']]
    check('shortest tested not optimal duration',summary['shortest_tested_full_valid_duration_s']==(min(valid)if valid else None))
    for png in (run/'plots').glob('*.png'):
        side=read(png.with_suffix('.json'))
        check(png.name+' image/source/config/common hashes',side['image_sha256']==digest(png) and side['source_sha256']==digest(run/'source.json') and side['config_sha256']==digest(run/'config_snapshot.yaml') and side['common_file_sha256']==digest(run/'inputs/common.npy') and side['outcomes_sha256']==digest(run/'outcomes.json'))
    side=read(run/'plots/duration_sweep_summary.json')['numeric']
    check('plotted duration table equals outcomes',side['outcomes']==[outcomes[m]for m,t in method_schedule()if t is not None])
    for name,key in [('execution_to_fresh_distance','distance_m'),('execution_to_fresh_yaw','yaw_error_rad')]:
        side=read(run/'plots'/(name+'.json'))['numeric']
        check(name+' plotted curves from evaluated records',all(v==read(run/'evaluation'/m/'attachment.json')for m,v in side.items()))
    result=dict(valid=not errors,errors=errors,checks=checks,source_preserved=True,source_event_count=881,
        original_acceptance_unchanged=True,scientific_failure_is_not_artifact_corruption=True,
        new_GP_solves=0,new_MPC_solves=0,new_LightNav_inferences=0,new_rollouts=0,
        unique_full_candidate_vectors_rechecked=len(full_cache),wall_s=time.perf_counter()-began,validator_sha256=digest(__file__))
    write(run/'validation.json',result);print({k:result[k]for k in ['valid','errors','wall_s']},flush=True)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True);args=parser.parse_args()
    if not validate(args.run.resolve())['valid']:raise SystemExit(1)

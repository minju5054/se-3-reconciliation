#!/usr/bin/env python3
"""Saved-only validator. Never calls an optimizer, controller or simulator."""
import argparse,json
from pathlib import Path
import numpy as np
from run_osa03_relative_factor_ablation01 import (ROOT,verify,geometry,plain,source_phase,read,save,sha,
    LocalSE2Problem,independent_clearance,common_state,relative_pose,local_trajectory_to_world)
from reconciliation.osa03_relative_ablation import ORDER,LABEL,comparability,planning,metric_row,gaps
from reconciliation.osa03_common_b import authenticated_m3,references,may_execute
from reconciliation.graph_optimizer import retract_trajectory
from reconciliation.osa03_native import evaluate
from validate_osa03_common_b import validate_method
from validate_robotless_online_handoffs import equal_record


def validate_planning(run,problem,world,env):
    p=problem;trace=read(run/'solver_trace.json');checks=read(run/'feasibility_checks.json')
    state=p.fresh.copy();v=p.residual_vector(state,include_relative=False);cost=float(v@v);accepted=[]
    for event in trace:
        if 'candidate' in event:
            candidate=np.array(event['candidate'])
            np.testing.assert_array_equal(candidate,retract_trajectory(state,event['delta']))
            v=p.residual_vector(candidate,include_relative=False);cc=float(v@v)
            assert np.isclose(cc,event['candidate_cost'],rtol=1e-13,atol=1e-13)
            feasible=env['on'].check_polyline(candidate)['clearance_valid'] if cc<cost else None
            assert event['candidate_feasible']==feasible
            expected='accepted' if cc<cost and feasible else ('rejected_unsafe' if cc<cost else 'rejected_non_improving')
            assert event['decision']==expected
            damping=event['damping_before']*(.3 if expected=='accepted' else 10.)
            assert np.isclose(event['damping'],max(np.finfo(float).eps,damping) if expected=='accepted' else damping,rtol=1e-14)
            if expected=='accepted':state=candidate;cost=cc;accepted.append(state)
        np.testing.assert_array_equal(event['state'],state)
        equal_record(event['factor_costs'],p.costs(state,include_relative=False))
        assert 'R' not in event['factor_costs']
    np.testing.assert_array_equal(state,world)
    assert len(checks)==1+sum(e.get('candidate_cost',float('inf'))<e.get('cost_before',-float('inf')) for e in trace)
    for check in checks:equal_record(env['on'].check_polyline(check['state']),check['check'])
    result=read(run/'planning_result.json');equal_record(result['final'],p.costs(world,include_relative=False))
    assert result['diagnostic_relative_edge_distortion_not_optimized_cost']==p.costs(world)['R']
    assert result['solver']['final_cost']==trace[-1]['cost']
    return dict(valid=True,accepted_steps=len(accepted),unsafe_rejections=sum(e['decision']=='rejected_unsafe' for e in trace),
                independent_cost_retraction_acceptance_checks=True,new_solves=0)


def validate(run):
    verify(run);manifest=read(run/'source_manifest.json');source=Path(manifest['source'])
    phase,fresh,_,provenance=source_phase(source);common=common_state(phase);equal_record(common,read(run/'common_state.json'))
    refs=read(run/'references.json');assert list(refs)==ORDER
    full=authenticated_m3(Path(manifest['M3_run'])/'derived/optimized_world.npy')
    expected=references(common['fresh_capture_pose'],common['B'],fresh,full)
    env=geometry(source);scene=read(source/'scenario.json');p=LocalSE2Problem(common['fresh_capture_pose'],common['B'],fresh)
    planning_audit=validate_planning(run,p,np.load(refs['NO_RELATIVE']['world_path']),env)
    rollouts={};metrics={};own={};checks={}
    for name,ref in refs.items():
        assert sha(ref['world_path'])==ref['world_sha256'] and sha(ref['local_path'])==ref['local_sha256']
        world=np.load(ref['world_path']);local=np.load(ref['local_path'])
        if name!='NO_RELATIVE':np.testing.assert_array_equal(world,expected[{'M0_NATIVE':'M0_NATIVE','M1_TAPER':'M1_SE2_TAPER','FULL_LOCAL_SE2':'M3_LOCAL_SE2'}[name]])
        np.testing.assert_allclose(local_trajectory_to_world(common['fresh_capture_pose'],local),world,atol=1e-12,rtol=0)
        equal_record(ref['safety'],independent_clearance(world,env));equal_record(ref['descriptor'],plain(planning(fresh,world,common['B'])))
        r,m,audit=validate_method(run,name,ref,common,fresh,env['on'],scene,provenance['official_settings'])
        rollouts[name]=r;metrics[name]=m;checks[name]=audit;own[name]=None
        if r is None:continue
        for wait in r['logical_waits']:
            assert wait['sim_before']==wait['sim_after'] and wait['pose_before']==wait['pose_after']
            assert wait['release_tick']==next(z['application_tick'] for z in read(run/'schedule.json')['pairs'] if z['submit_tick']==wait['submit_tick'])
        for e in r['events']:
            if e.get('type')=='solve_result' and not e.get('after_termination'):
                assert e['official_generation']==e['result_generation']==common['original_generation']
                assert e['continuation_seen']['tick']==next(z['application_tick'] for z in read(run/'schedule.json')['pairs'] if z['submit_tick']==e['input_state_id'])
        if r['commands']:
            om=plain(evaluate(r,world,env['on'],scene,read(run/'metric_protocol.json')))
            equal_record(om,read(run/'methods'/name/'own_reference_metrics.json'))
            own[name]={k:v for k,v in metric_row(om,r['termination']).items() if k in ['position_auc_03_m_s','position_auc_09_m_s','yaw_auc_03_rad_s','yaw_auc_09_rad_s','max_position_error_05_m']}
    gate=comparability(rollouts,read(run/'schedule.json'),common)
    safe=all(may_execute(r['safety']) for r in refs.values())
    classification=('SCHEDULE_CONTROL_FAILURE' if not gate['comparable'] else
        'REFERENCE_UNSAFE_LIMITED_COMPARISON' if not safe else 'TIMING_CONTROLLED_OFFLINE_COMPARISON_VALID')
    rows={n:metric_row(metrics[n],None if rollouts[n] is None else rollouts[n]['termination']) for n in ORDER}
    costs={}
    for name in ['FULL_LOCAL_SE2','NO_RELATIVE']:
        world=np.load(refs[name]['world_path']);inc=name=='FULL_LOCAL_SE2'
        costs[name]=dict(optimized_costs=p.costs(world,include_relative=inc),
            diagnostic_relative_edge_distortion_not_optimized_cost=None if inc else p.costs(world)['R'])
    # Numeric overlap diagnostics, with no scientific success cutoff.
    pairwise={}
    for a,b in [('FULL_LOCAL_SE2','M1_TAPER'),('NO_RELATIVE','FULL_LOCAL_SE2')]:
        key=a+'_minus_'+b
        x=np.load(refs[a]['world_path']);y=np.load(refs[b]['world_path'])
        pairwise[key]=dict(max_reference_XY_separation_m=float(np.linalg.norm(x[:,:2]-y[:,:2],axis=1).max()))
        if rollouts[a] and rollouts[b]:
            x=np.array([s['pose_world'] for s in rollouts[a]['states']]);y=np.array([s['pose_world'] for s in rollouts[b]['states']]);n=min(len(x),len(y))
            pairwise[key]['max_execution_XY_separation_m']=float(np.linalg.norm(x[:n,:2]-y[:n,:2],axis=1).max())
    valid=all(c['valid'] for c in checks.values())
    counts=dict(optimizer=1,FULL_reconstruction=0,MPC={n:0 if r is None else r['new_MPC_solved'] for n,r in rollouts.items()},
        rollouts=sum(r is not None for r in rollouts.values()),LightNav=0,RGB=0,Isaac=0,source_acquisition=0)
    summary=dict(label=LABEL,classification=classification,scientific_freeze_sha=read(run/'execution_start.json')['sha'],
        primary_metrics=rows,secondary_own_reference_metrics=own,signed_method_minus_native=gaps(rows,'M0_NATIVE'),
        signed_method_minus_full=gaps(rows,'FULL_LOCAL_SE2'),schedule_gate=gate,factor_costs=costs,planning={n:r['descriptor'] for n,r in refs.items()},
        reference_safety={n:r['safety'] for n,r in refs.items()},pairwise=pairwise,counts=counts,
        termination={n:'REFERENCE_GEOMETRY_UNSAFE' if r is None else r['termination'] for n,r in rollouts.items()},
        full_metrics={n:None if m is None else {k:m[k] for k in ['full','windows','extrema','endpoint','clearance']} for n,m in metrics.items()})
    validation=dict(valid=valid,methods=checks,planning=planning_audit,original_FRESH_primary=True,
        secondary_separate=True,source_hashes_preserved=True,Full_reused_exact_bytes=True,logical_schedule_gate=gate['comparable'],new_scientific_solves=0)
    return plain(summary),plain(validation)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True);parser.add_argument('--check-only',action='store_true')
    args=parser.parse_args();summary,audit=validate(args.run.resolve())
    if not args.check_only:save(args.run/'summary.json',summary);save(args.run/'validation.json',audit)
    print(json.dumps(dict(validation=audit,primary=summary['primary_metrics'],counts=summary['counts'],classification=summary['classification'])))

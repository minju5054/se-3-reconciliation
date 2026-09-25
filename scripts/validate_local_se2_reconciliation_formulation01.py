#!/usr/bin/env python3
"""Saved-record-only independent equations/acceptance/geometry audit. No solve."""
import argparse
from pathlib import Path
import numpy as np
from run_local_se2_reconciliation_formulation01 import (
    ROOT, read, save, sha, verify_hashes, load_source, independent_clearance, git)
from reconciliation.se2 import compose_poses, relative_pose, se2_log, se2_exp


def independent_costs(f, target, x):
    # Rebuild schedules and all three means from saved world arrays.
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(f[:,:2],axis=0),axis=1))]
    s=arc/arc[-1]; scale=np.array([.1,.1,np.pi/18])
    L=se2_log(relative_pose(target,x))/scale
    A=se2_log(relative_pose(f,x))/scale
    R=se2_log(relative_pose(relative_pose(f[:-1],f[1:]),relative_pose(x[:-1],x[1:])))/scale
    out=dict(L=float(np.sum((1-s)**2*np.sum(L**2,axis=1))/sum((1-s)**2)),
             A=float(np.sum(s**2*np.sum(A**2,axis=1))/sum(s**2)),R=float(np.mean(np.sum(R**2,axis=1))))
    out['total']=sum(out.values()); return out


def parity(actual, expected):
    if isinstance(expected,dict):
        assert actual.keys()==expected.keys()
        for k in expected:parity(actual[k],expected[k])
    elif isinstance(expected,(list,tuple,np.ndarray)):
        assert len(actual)==len(expected)
        for a,e in zip(actual,expected):parity(a,e)
    elif isinstance(expected,(int,float,np.number)) and not isinstance(expected,bool):
        np.testing.assert_allclose(actual,expected,rtol=2e-10,atol=2e-12)
    else: assert actual==expected


def validate(run, *, authoritative=True):
    verify_hashes({str(run/p):h for p,h in read(run/'result_hashes.json').items()})
    manifest=read(run/'source_manifest.json'); verify_hashes(manifest['files']); verify_hashes(manifest['historical_preservation'])
    cfg=read(run/'protocol.json'); loaded=load_source(cfg); p=loaded['problem']
    result=read(run/'result.json'); frozen=read(run/'freeze_inputs.json'); start=read(run/'execution_start.json')
    assert start['scientific_freeze_sha']==result['scientific_freeze_sha']
    for path,expected in frozen['code_sha256'].items():
        import hashlib,subprocess
        blob=subprocess.check_output(['git','-C',str(ROOT),'show',start['scientific_freeze_sha']+':'+path])
        assert hashlib.sha256(blob).hexdigest()==expected
    assert sha(run/'source_manifest.json')==frozen['source_manifest_sha256']
    assert sha(run/'protocol.json')==frozen['protocol_sha256']
    assert sha(run/'source_validation.json')==frozen['source_validation_sha256']
    f=np.load(run/'derived/original_world.npy'); target=np.load(run/'derived/transported_target.npy')
    x=np.load(run/'derived/optimized_world.npy'); local=np.load(run/'derived/optimized_original_A_local.npy')
    np.testing.assert_array_equal(f,p.fresh)
    np.testing.assert_allclose(target,p.target,rtol=0,atol=1e-12)
    np.testing.assert_allclose(relative_pose(p.B,target),relative_pose(p.A,f),rtol=0,atol=1e-12)
    np.testing.assert_allclose(compose_poses(p.A,local),x,rtol=0,atol=1e-12)
    parity(result['initial'],independent_costs(f,target,f)); parity(result['final'],independent_costs(f,target,x))
    parity(result['A'],p.A); parity(result['B'],p.B)
    parity(result['log_A_inverse_B'],se2_log(relative_pose(p.A,p.B)))
    parity(result['observation_B_translation_m'],np.linalg.norm(p.B[:2]-p.A[:2]))
    parity(result['raw_arc_m'],np.linalg.norm(np.diff(f[:,:2],axis=0),axis=1).sum()); assert result['N']==len(f)
    for key,arr in [('raw_safety',f),('optimized_safety',x)]:parity(result[key],independent_clearance(arr,loaded))
    assert result['raw_safety']['clearance_valid'] and result['optimized_safety']['clearance_valid']
    trace=read(run/'solver_trace.json'); state=f.copy(); costs=[result['initial']['total']]
    acceptance_counts={name:0 for name in ['accepted','rejected_unsafe','rejected_non_improving']}
    for event in trace:
        parity(event['factor_costs'],independent_costs(f,target,np.array(event['state'])))
        if 'candidate' in event:
            candidate=np.array(event['candidate']); delta=np.array(event['delta']).reshape(f.shape)
            np.testing.assert_allclose(candidate,compose_poses(state,se2_exp(delta)),rtol=0,atol=1e-12)
            current=independent_costs(f,target,state)['total']; c=independent_costs(f,target,candidate)['total']
            parity(event['cost_before'],current); parity(event['candidate_cost'],c)
            decision=event['decision']; acceptance_counts[decision]+=1
            if decision=='accepted':
                assert c < current and independent_clearance(candidate,loaded)['clearance_valid']
                state=candidate; costs.append(c)
                parity(event['damping'],max(np.finfo(float).eps,event['damping_before']*.3))
            else:
                parity(event['damping'],event['damping_before']*10)
                if decision=='rejected_unsafe':
                    assert c < current and not independent_clearance(candidate,loaded)['clearance_valid']
                else:assert c >= current
        np.testing.assert_allclose(event['state'],state,rtol=0,atol=1e-12)
    np.testing.assert_array_equal(state,x)
    if 'cost_history' in result['solver']:parity(result['solver']['cost_history'],costs)
    for decision,name in [('accepted','accepted_steps'),('rejected_unsafe','rejected_unsafe_steps'),('rejected_non_improving','rejected_non_improving_steps')]:assert result[name]==acceptance_counts[decision]
    checks=read(run/'feasibility_checks.json'); np.testing.assert_array_equal(checks[0]['state'],f)
    for check in checks:
        direct=independent_clearance(np.array(check['state']),loaded)
        parity(check['check']['minimum_clearance_m'],direct['minimum_clearance_m'])
        assert check['check']['clearance_valid']==direct['clearance_valid']
    from reconciliation.local_se2_reconciliation import planning_diagnostics
    parity(result['diagnostics'],planning_diagnostics(p,x))
    # Explicit node/edge identities in addition to aggregate diagnostics parity.
    for j,node in enumerate(result['diagnostics']['nodes']):
        parity(node['correction_log'],se2_log(relative_pose(f[j],x[j])))
        parity(node['target_log'],se2_log(relative_pose(target[j],x[j])))
    assert result['calls']==dict(planning=1,LightNav=0,MPC=0,GP=0,actual_execution=0,rigid_baseline=0,splice=0)
    assert result['GP_support_timing'] is None and result['waypoint_intrinsic_timing'] is None
    source_revalidated=False
    if authoritative:
        from validate_obstacle_source_acquisition03 import episode
        audit=episode(loaded['source'],'REPEAT_00')
        assert audit['qualified'] and audit['original_validation']['valid']; source_revalidated=True
    review=False
    if (run/'review/manifest.json').exists():
        m=read(run/'review/manifest.json'); verify_hashes({str(run/'review'/p):h for p,h in m['files'].items()})
        for sidecar in sorted((run/'review').glob('figure_*.json')):
            side=read(sidecar); verify_hashes(side['source_hashes'])
            assert side['scope']==result['scope']
            parity(side['result'],result); assert side['trace']==trace
        with (run/'review/nodes.csv').open() as stream:
            import csv
            rows=list(csv.DictReader(stream))
        assert len(rows)==len(x)
        for j,row in enumerate(rows):
            for key,value in row.items():parity(float(value),result['diagnostics']['nodes'][j][key])
        review=True
    return dict(valid=True,authoritative_R00_saved_only=source_revalidated,original_hashes_unchanged=True,
                no_R01_evaluation=True,independent_equations=True,right_local_steps=True,
                all_accepted_states_feasible=True,independent_full_union_safety=True,
                source_observation_anchor=True,plot_numeric_hash_parity=review,
                accepted_steps=acceptance_counts['accepted'],checked_feasibility_states=len(checks),
                validation_new_LightNav_MPC_GP_planning_execution_calls=0)


if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('--run',type=Path,required=True); parser.add_argument('--output',default='validation.json')
    args=parser.parse_args(); run=args.run.resolve(); result=validate(run); save(run/args.output,result); print(result)

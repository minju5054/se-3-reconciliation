#!/usr/bin/env python3
"""Saved-source/metric validation; no inference, optimizer, MPC or simulator."""
import argparse,csv,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
from run_gp_se2_join01 import read,write,digest,verify,load_case
from reconciliation.gp_se2_join01 import METHODS,sustained_join
from reconciliation.gp_se2_join01_qualification import qualification_from_episode
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.robotless_online import integrate_unicycle


def validate(run):
    errors=[];checks=[]
    def check(name,passed):
        checks.append(dict(name=name,passed=bool(passed)))
        if not passed:errors.append(name)
    source=verify(run);protocol=read(run/'protocol.json');collection=read(run/'collection_result.json')
    ids=[p['id'] for p in protocol['placements']];attempts=collection['attempts']
    check('predeclared ordered placement prefix',[r['placement'] for r in attempts]==ids[:len(attempts)])
    base=HospitalEnvironment.load(source['environment'])
    for row in attempts:
        ep=row['placement'];old=read(run/'qualification'/(ep+'.json'))
        new=qualification_from_episode(run/'source_event',ep,base,protocol)
        from reconciliation.gp_se2_diag02_validation import plain
        check(ep+' qualification reproduced',old==plain(new))
        folder=run/'source_event'/'episodes'/ep
        completion=read(folder/'completion.json')
        for f in completion['raw_manifest']['files']:
            check(ep+' '+f['path'],digest(folder/f['path'])==f['sha256'])
        states=list(csv.DictReader((folder/'execution.csv').open()));commands=list(csv.DictReader((folder/'commands.csv').open()))
        error=0.
        for i,cmd in enumerate(commands):
            p=np.array([float(states[i][k]) for k in ('x','y','yaw')]);q=np.array([float(states[i+1][k]) for k in ('x','y','yaw')])
            dt=float(states[i+1]['sim_time_s'])-float(states[i]['sim_time_s'])
            error=max(error,float(np.max(np.abs(integrate_unicycle(p,[float(cmd['v_mps']),float(cmd['omega_radps'])],dt)-q))))
        check(ep+' recorded commands reconstruct states',error<=1e-9)
        if 'context' in old:
            raw=read(folder/'handoffs/handoff_000/context.json');fid=raw['fresh_observation_pose_time']['frame_id'];vis=read(folder/'visibility'/(fid+'.json'))
            mask=np.load(folder/'visibility'/(fid+'.npz'))['mask'];labels=[int(k) for k,v in vis['labels'].items() if 'JOIN01Obstacle' in str(v)]
            check(ep+' actual visibility count',int(np.isin(mask,labels).sum())==vis['obstacle_pixels'])
            obstacle=read(folder/'obstacle_reveal.json');matrix=np.array(obstacle['usd_world_matrix']);p=np.array(obstacle['obstacle']['pose_world']);d=np.array(obstacle['obstacle']['dimensions_m'])
            check(ep+' box USD translation',np.allclose(matrix[:3,3],[p[0],p[1],d[2]/2],atol=1e-10,rtol=0))
            check(ep+' box USD dimensions',np.allclose(np.linalg.norm(matrix[:3,:3],axis=0),d,atol=1e-7,rtol=0))
            first=raw['old_observation_pose_time']['time'];switch=raw['switch_state_id']
            prior=[r for r in commands if float(r['sim_time_s'])>=first and int(r['application_state_id'])<switch]
            check(ep+' OLD commands during FRESH inference',bool(prior) and all(r['chunk_id']==raw['old_chunk_id'] for r in prior))
    qualifiers=[r['placement'] for r in attempts if r['qualified']]
    check('first qualifier stops placement schedule',not qualifiers or qualifiers==[attempts[-1]['placement']])
    check('selected event exact',collection['selected_episode']==(qualifiers[0] if qualifiers else None))
    if not qualifiers:
        check('no reconciliation on failed qualification',not (run/'methods').exists() and not (run/'rollouts').exists())
    else:
        case=load_case(run);outcomes=read(run/'outcomes.json');check('all method ledger coverage',list(outcomes)==list(METHODS))
        from reconciliation.gp_se2_evaluation import evaluate_rollout
        from reconciliation.gp_se2_formulation import GPProblem
        from reconciliation.gp_se2_diag_acceptance import check_full_candidate
        from reconciliation.gp_se2_join01_formulation import JoinView
        ctx=case['context'];env=case['environment']
        problem=GPProblem(boundary_pose=ctx['B_world'],initial_twist=[ctx['u_minus'][0],0.,ctx['u_minus'][1]],
            common_reference=case['common'],goal_pose=case['goal_route']['goal_world'],config=case['config']['formulation'],
            obstacle_clearance=lambda xy:env.optimizer_clearance(xy,.2),workspace_margin=lambda xy:env.workspace_margin(xy,.2),gates=case['goal_route']['gates'])
        for method in METHODS:
            r=outcomes[method];p=run/'rollouts'/method/'rollout.json'
            check(method+' rollout availability',p.exists()==r['rollout_performed'])
            if not p.exists():
                check(method+' missing outcome N/A',r['join_time_s'] is None and r['execution_success'] is None);continue
            roll=read(p);e=evaluate_rollout(roll,case['goal_route'],case['environment'],case['config'])
            j=sustained_join(e['dense_times_s'],e['dense_poses_world'],case['native'])
            check(method+' exact input B',roll['initial_pose_world']==case['context']['B_world'])
            check(method+' physical memory separate',roll['initial_physical_command']==case['context']['u_minus'] and roll['initial_previous_control']==case['context']['previous_control'])
            check(method+' clock coverage',len(roll['states'])==181 and len(roll['commands'])==180 and len(roll['controller_reference_selections'])==30)
            check(method+' original success recomputed',e['primary_success']==r['execution_success'])
            check(method+' join recomputed',j['sustained_join_time_s']==r['join_time_s'])
            err=max(float(np.max(np.abs(integrate_unicycle(s['pose_world'],c['command'],1/60)-np.asarray(n['pose_world'])))) for s,c,n in zip(roll['states'][:-1],roll['commands'],roll['states'][1:]))
            check(method+' independent exact command integration',err<=1e-10)
            if method.startswith(('M3','M4')):
                check(method+' full plan required',r['plan_valid'])
                selected=read(run/'methods'/method/'selected.json')
                full=check_full_candidate(problem,selected['vector'],case)
                check(method+' full checker independently repeated',full['full_feasible'])
                if method=='M4_JOIN_GP':
                    check(method+' original FRESH tube',JoinView(problem,case['native'],selected).independent_tube_check(selected['vector'])['valid'])
    for file in (run/'plots').glob('*.png'):
        side=read(file.with_suffix('.json'));check(file.name+' numeric/source sidecar',side['image_sha256']==digest(file) and side['source_sha256']==digest(run/'source.json') and side['protocol_sha256']==digest(run/'protocol.json'))
    result=dict(valid=not errors,errors=errors,checks=checks,new_gp_solves=0,new_mpc_solves=0,new_vla_inferences=0,
                original_acceptance_unchanged=True,scientific_failure_is_not_artifact_corruption=True)
    write(run/'validation.json',result);print(dict(valid=result['valid'],errors=errors,checks=len(checks)),flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args()
    if not validate(a.run.resolve())['valid']:raise SystemExit(1)

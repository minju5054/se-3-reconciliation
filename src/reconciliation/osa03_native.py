"""Single-source native continuation: saved asynchronous phase and diagnostics.

Historical accepted results are replayed, not recalculated. New controller calls
are made only by the separate official worker after the exact saved prefix.
"""
from copy import deepcopy
import numpy as np
from .robotless_online import integrate_unicycle, CommandActivation
from .online_mpc_adapter import selection_audit
from .gp_se2_join01 import forward_projection
from .handoff_execution_loss import attachment_loss, command_loss
from .join_online02 import preview_activation, guard_check
from .se2 import wrap_angle, local_trajectory_to_world


def pose(row):
    return np.array([float(row[k]) for k in ('x','y','yaw')])


def command(row):
    return [float(row[k]) for k in ('v_mps','omega_radps')]


def next_control_tick(tick, stride=6):
    return (tick//stride+1)*stride


def phase_audit(context, states, commands, events, boundary, fresh, raw, settings, dt):
    """Fail closed if phase/identity is missing or ambiguous; never choose it."""
    b=context['switch_state_id'];last=int(states[-1]['state_id'])
    assert len(states)==last+1 and len(commands)==last
    assert [int(s['state_id']) for s in states]==list(range(last+1))
    np.testing.assert_array_equal(pose(states[b]),context['B'])
    np.testing.assert_array_equal(boundary['pose_world'],context['B'])
    np.testing.assert_allclose(local_trajectory_to_world(context['R_obs'],raw),fresh,rtol=0,atol=1e-12)
    solved={}
    for e in events:
        if e.get('type')=='solve_result':
            assert e['solve_id'] not in solved
            solved[e['solve_id']]=e
    first=context['first_fresh_solve'];fid=first['solve_id']
    assert all(first[k]==v for k,v in solved[fid].items())
    assert int(commands[b]['application_state_id'])==b and commands[b]['solve_id']==fid
    assert commands[b]['reason']=='new_solve' and commands[b]['chunk_id']==context['fresh_chunk_id']
    assert first['input_state_id']<b and first['input_state_id']%6==0
    assert first['seen_sim_time_s']==float(states[b]['sim_time_s'])
    np.testing.assert_array_equal(command(commands[b]),first['command'])
    np.testing.assert_array_equal(command(commands[b-1]),boundary['u_minus'])
    assert boundary['memory']['available']
    np.testing.assert_array_equal(boundary['memory']['previous_control'],first['command'])
    prefix_commands=commands[b:];prefix_states=states[b:]
    application=[];last_id=None
    for row in prefix_commands:
        sid=int(row['application_state_id']);rid=row['solve_id']
        assert row['chunk_id']==context['fresh_chunk_id']
        if rid!=last_id:
            r=solved[rid];inputid=r['input_state_id']
            assert r['status']=='command' and r['input_state_id']%6==0
            assert r['seen_in_isaac']['sim_time_s']==float(states[sid]['sim_time_s'])
            np.testing.assert_array_equal(r['input_pose'],pose(states[inputid]))
            np.testing.assert_array_equal(r['command'],command(row))
            audit=selection_audit(fresh,r['input_pose'],r['selection']['reference_world'],horizon=settings['HORIZON'],weights=settings['Q_WEIGHTS'])
            assert audit==r['selection']
            application.append(dict(application_tick=sid,submit_tick=inputid,result=r))
            last_id=rid
    submissions=[e for e in events if e.get('status')=='submitted' and e['input_state_id']>=b]
    expected=list(range(next_control_tick(b),last,6))
    assert [e['input_state_id'] for e in submissions]==expected
    assert all(e['solve_id'] in solved for e in submissions)
    assert len(application)==len(submissions)+1
    assert all(e['solve_id'] in {a['result']['solve_id'] for a in application} for e in submissions)
    for prev,nxt in zip(application,application[1:]):
        np.testing.assert_array_equal(nxt['result']['previous_command'],prev['result']['command'])
    assert all(abs(float(z['sim_time_s'])-float(a['sim_time_s'])-dt)<1e-12 for a,z in zip(states,states[1:]))
    prior_changes=[x for x in commands[:b] if x['reason']=='new_solve']
    return dict(resolved=True,B_tick=b,B_sim_s=float(states[b]['sim_time_s']),B=context['B'],
        u_minus=command(commands[b-1]),u_B_plus=command(commands[b]),u_mem_B=boundary['memory']['previous_control'],
        delta_u_B=(np.array(command(commands[b]))-command(commands[b-1])).tolist(),
        previous_command_application_sim_s=float(prior_changes[-1]['sim_time_s']),
        first_FRESH_solve=first,applications=application,prefix_states=prefix_states,prefix_commands=prefix_commands,
        prefix_end_tick=last,prefix_steps=last-b,prefix_duration_s=(last-b)*dt,
        next_submit_after_B=next_control_tick(b),first_new_submit_tick=next_control_tick(last),
        fresh_capture_pose=context['R_obs'],fresh_chunk_id=context['fresh_chunk_id'],fresh_version=context['fresh_reference_version'],
        original_generation=first['official_generation'],integration_dt_s=dt,pending_at_prefix_end=False,
        replay_semantics='saved accepted controller results, independently integrated prefix; not a re-solve parity claim')


def restore_activation(audit):
    a=CommandActivation(.5)
    a.install(audit['fresh_chunk_id'],audit['fresh_version'],{})
    a.active=a.installed  # continuation of an already installed/activating chunk
    return a


def replay_prefix(audit, environment=None):
    """Six historical steps through unchanged activation/integrator; zero solve."""
    a=restore_activation(audit);x=np.array(audit['B']);dt=audit['integration_dt_s']
    states=[x.copy()];rows=[];guards=[]
    apps={r['application_tick']:r['result'] for r in audit['applications']}
    for saved in audit['prefix_commands']:
        tick=int(saved['application_state_id']);st=dict(audit['prefix_states'][tick-audit['B_tick']])
        st.update(x=float(x[0]),y=float(x[1]),yaw=float(x[2]),state_id=tick,tick=tick,sim_time_s=float(st['sim_time_s']),episode_time_s=float(st['episode_time_s']))
        if tick in apps:assert a.accept(deepcopy(apps[tick]),st['sim_time_s'])
        proposal,c,_=preview_activation(a,st,None,rows[-1] if rows else None)
        for k in ['v_mps','omega_radps','reference_version','chunk_id','solve_id','held','reason','application_state_id','application_tick']:
            expected=saved[k]
            if isinstance(c[k],bool):expected=expected=='True' if isinstance(expected,str) else expected
            elif isinstance(c[k],(float,int)):expected=type(c[k])(expected)
            assert c[k]==expected,(tick,k,c[k],expected)
        if environment is not None:
            g=guard_check(environment,x,c,dt);assert g['safe'];guards.append(g)
        a=proposal;rows.append(c);x=integrate_unicycle(x,command(c),dt);states.append(x.copy())
        np.testing.assert_allclose(x,pose(audit['prefix_states'][len(rows)]),rtol=0,atol=1e-10)
    return dict(passed=True,steps=len(rows),states=np.asarray(states),commands=rows,guards=guards,
        maximum_pose_error=float(np.max(abs(np.asarray(states)-np.array([pose(s) for s in audit['prefix_states']])))),
        activation=a,no_new_MPC_solves=True)


def classification(termination,joined,prefix_valid=True):
    if not prefix_valid:return 'NATIVE_PREFIX_REPLAY_INVALID'
    if termination=='SAFETY_ABORT_BEFORE_UNSAFE_COMMAND':return 'NATIVE_CONTINUATION_SAFETY_ABORT'
    if termination!='OBSERVATION_CAP':return 'NATIVE_CONTINUATION_CONTROLLER_FAILURE'
    return 'NATIVE_CONTINUATION_SAFE_SUSTAINED_ATTACHMENT' if joined else 'NATIVE_CONTINUATION_SAFE_NO_SUSTAINED_ATTACHMENT'


def evaluate(result,fresh,environment,scene,config):
    t=np.array([s['time_s'] for s in result['states']]);p=np.array([s['pose_world'] for s in result['states']]);u=np.array([command(c) for c in result['commands']])
    loss=attachment_loss(t,p,fresh);trace=loss.pop('trace');windows={}
    for n in config['windows_steps']:
        if len(u)<n:windows[str(n)]=dict(available=False,reason='termination_before_complete_window');continue
        w=attachment_loss(t[:n+1],p[:n+1],fresh);w.pop('trace');windows[str(n)]=dict(available=True,**w)
    comm=command_loss(t,p,u,result['phase']['u_minus'],config)
    delta=np.diff(u,axis=0)
    comm.update(post_B_linear_TV=float(abs(delta[:,0]).sum()),post_B_angular_TV=float(abs(delta[:,1]).sum()),
        post_B_nominal_max_dv=float(abs(delta[:,0]).max()*10) if len(delta) else None,
        post_B_nominal_max_domega=float(abs(delta[:,1]).max()*10) if len(delta) else None)
    new=[i for i,c in enumerate(result['commands']) if c['reason']=='new_solve']
    prev_u=np.array(result['phase']['u_minus']);prev_t=result['phase']['previous_command_application_sim_s']-result['phase']['B_sim_s'];rates=[]
    for i in new:
        span=float(t[i]-prev_t);change=u[i]-prev_u
        rates.append(dict(time_s=float(t[i]),interval_s=span,delta=change.tolist(),nominal_rate=(change/.1).tolist(),actual_interval_rate=(change/span).tolist(),historical_switch=i==0))
        prev_t=float(t[i]);prev_u=u[i]
    comm['application_intervals']=rates
    comm['actual_interval_rate_semantics']='descriptive command jumps divided by actual application spacing, not continuous physical acceleration'
    # Dense geometric diagnosis; no new integration policy/acceptance threshold.
    dense_t=[0.];dense_p=[p[0]]
    for i,dt in enumerate(np.diff(t)):
        for h in np.linspace(0,dt,int(np.ceil(dt/.005))+1)[1:]:
            dense_t.append(float(t[i]+h));dense_p.append(integrate_unicycle(p[i],u[i],float(h)))
    dense_p=np.asarray(dense_p);point=[environment.check_polyline([x])['minimum_clearance_m'] for x in dense_p]
    imin=int(np.argmin(point));curve=float(np.max(abs(u[:,0]*u[:,1])*np.diff(t)**2/8))
    safety=environment.check_trajectory(t,p,radius=.2,required_clearance=.05,curved_path_error_bound_m=curve)
    f=np.asarray(fresh);arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(f[:,:2],axis=0),axis=1))]
    progress=np.array([r['progress'] for r in trace['projection']]);sarc=np.interp(progress,np.arange(len(f)),arc)
    unrestricted=forward_projection(f,p[:1]) if len(p)==1 else [forward_projection(f,[x])[0] for x in p]
    unclipped=np.array([r['progress'] for r in unrestricted]);enddist=np.linalg.norm(p[:,:2]-f[-1,:2],axis=1);endyaw=abs(wrap_angle(p[:,2]-f[-1,2]))
    entry=np.flatnonzero((enddist<=.1)&(endyaw<=np.pi/12));early=np.flatnonzero(t<=.5+1e-7);idx=int(early[np.argmax(np.array(trace['distance_m'])[early])])
    center=np.asarray(scene['center_xy']);forward=np.asarray(scene['forward_xy']);left=np.array([-forward[1],forward[0]])
    q=dense_p[imin];lateral=(p[:,:2]-p[0,:2])@left
    tangent=f[-1,:2]-f[-2,:2] if len(f)>1 else np.zeros(2);norm=np.linalg.norm(tangent);beyond=None if norm==0 else (p[:,:2]-f[-1,:2])@(tangent/norm)
    joinidx=None if not loss['join_success'] else int(np.searchsorted(t,loss['join_time_s']))
    extrema=dict(max_yaw_error_rad=float(np.max(trace['yaw_error_rad'])),initial_growth_05_m=float(trace['distance_m'][idx]-trace['distance_m'][0]),
        max_distance_05_m=float(trace['distance_m'][idx]),max_distance_05_time_s=float(t[idx]),
        first_non_negligible_turn_time_s=next((float(t[i]) for i in range(len(u)) if abs(u[i,1])>config['turn_command_diagnostic_epsilon_radps']),None),
        max_omega_time_s=float(t[int(np.argmax(abs(u[:,1])))]),yaw_at_sustained_rad=None if joinidx is None else float(p[joinidx,2]))
    endpoint=dict(minimum_distance_m=float(enddist.min()),minimum_distance_time_s=float(t[int(np.argmin(enddist))]),
        first_pose_tube_entry_s=None if not len(entry) else float(t[entry[0]]),terminal_position_error_m=float(enddist[-1]),terminal_yaw_error_rad=float(endyaw[-1]),
        final_command=u[-1].tolist(),near_zero_final_command=bool(np.all(abs(u[-1])<=config['near_zero_command_diagnostic_epsilon'])),
        commands_near_zero_final_03=bool(np.all(abs(u[t[:-1]>=t[-1]-.3])<=config['near_zero_command_diagnostic_epsilon'])),
        final_03_max_endpoint_distance_m=float(enddist[t>=t[-1]-.3].max()),
        maximum_beyond_endpoint_tangent_m=None if beyond is None else float(beyond.max()),
        final_progress=float(progress[-1]),final_arc_progress_m=float(sarc[-1]),remaining_arc_m=float(arc[-1]-sarc[-1]),
        unrestricted_progress_backward_observed=bool(np.any(np.diff(unclipped)<-1e-10)),
        evaluator_monotonic=bool(np.all(np.diff(progress)>=0)),complete_bypass_claim=False)
    clearance=dict(raw_FRESH=environment.check_polyline(f),execution=safety,initial_m=float(point[0]),
        sampled_dense_minimum_m=float(point[imin]),minimum_time_s=float(dense_t[imin]),minimum_pose=q.tolist(),
        cart_longitudinal_m=float((q[:2]-center)@forward),cart_lateral_left_m=float((q[:2]-center)@left),
        minimum_is_finite_sample_not_exact_continuous_extremum=True)
    return dict(classification=classification(result['termination'],loss['join_success']),full=loss,windows=windows,command=comm,
        extrema=extrema,endpoint=endpoint,clearance=clearance,
        trace=dict(**trace,arc_progress_m=sarc,remaining_arc_m=arc[-1]-sarc,unrestricted_progress=unclipped,lateral_m=lateral,
                   dense_times_s=dense_t,dense_clearance_m=point),new_VLA_calls=0,new_optimization_calls=0)

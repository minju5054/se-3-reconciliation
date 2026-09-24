"""Saved observation phase, spatial descriptors and same-FRESH comparison.

No controller or optimizer is constructed here. Two time origins never share
an implicit zero. Synthetic inputs are implementation tests only.
"""
from copy import deepcopy
import numpy as np
from .handoff_delay_attribution import boundary_state
from .osa03_native import pose,command,evaluate
from .robotless_online import CommandActivation
from .join_online02 import preview_activation
from .se2 import local_trajectory_to_world,wrap_angle


def first_legal_tick(observation_tick,already_submitted=False,stride=6):
    if observation_tick%stride==0 and not already_submitted:return observation_tick
    return (observation_tick//stride+1)*stride


def audit_observation(context,states,commands,events,reveal,dt):
    """Conservative actual-phase audit; raise before any new solve if ambiguous."""
    initial=boundary_state(context,states,commands,events,'LATENCY_FREE')
    if not initial['available']:raise ValueError('OBSERVATION_CONTROLLER_STATE_UNRESOLVED: '+str(initial['reason']))
    sid=initial['state_id'];cut=context['t_obs']['host_monotonic_s']
    submissions=[e for e in events if e.get('status')=='submitted' and e['input_host_monotonic_s']<=cut]
    if any(e['submit_host_monotonic_s']>cut for e in submissions):raise ValueError('unresolved queued submit at observation')
    solved={e['solve_id']:e for e in events if e.get('type')=='solve_result'}
    pending=[]
    for e in submissions:
        r=solved.get(e['solve_id'])
        if r is None or r.get('seen_in_isaac',{}).get('host_monotonic_s',float('inf'))>cut:pending.append(e)
    # This exact source must have a uniquely empty pending set. No invented Future.
    if pending:raise ValueError('OBSERVATION_CONTROLLER_STATE_UNRESOLVED: pending/update interval not uniquely restored')
    installed=[e for e in events if e.get('status')=='installed' and e['installed_host_monotonic_s']<=cut]
    old=max(installed,key=lambda e:e['installed_host_monotonic_s'])
    assert old['chunk_id']==context['old_chunk_id'] and old['reference_version']==context['old_reference_version']
    assert old['seen_in_isaac']['host_monotonic_s']<=cut
    last=solved[initial['physical_command']['solve_id']]
    assert last['status']=='command' and last['chunk_id']==old['chunk_id']
    np.testing.assert_array_equal(last['command'],initial['u_minus'])
    np.testing.assert_array_equal(initial['memory']['previous_control'],last['command'])
    applied=[c for c in commands if c['solve_id']==last['solve_id'] and c['reason']=='new_solve']
    assert len(applied)==1
    assert float(applied[0]['host_monotonic_s'])<=cut and int(applied[0]['application_tick'])<sid
    # Validate recorded pose and reveal ordering independently of commands after cut.
    np.testing.assert_array_equal(initial['pose_world'],reveal['pose_world'])
    assert reveal['state_id']==sid and reveal['completed']['host_monotonic_s']<=cut
    assert reveal['rendering_present'] and reveal['oracle_present'] and not reveal['cart_pose_changed']
    assert context['t_request']['host_monotonic_s']>=cut
    submitted_here=any(e['input_state_id']==sid for e in submissions)
    first=first_legal_tick(sid,submitted_here)
    assert all(e['input_state_id']%6==0 for e in submissions)
    return dict(resolved=True,observation=initial,obs_tick=sid,obs_sim_s=float(initial['saved_state']['sim_time_s']),
        obs_pose=initial['pose_world'],u_obs=initial['u_minus'],memory_obs=initial['memory']['previous_control'],
        incoming_application=applied[0],last_accepted_OLD=last,old_install=old,pending_OLD=[],
        next_legal_submit_tick=first,OLD_submitted_before_observation_at_same_tick=submitted_here,
        integration_dt_s=dt,source_timestamps={k:context[k] for k in ['t_obs','t_request','t_ready_host','t_install','t_switch']},
        original_delayed_B=context['B'],fresh_capture_pose=context['R_obs'],fresh_chunk_id=context['fresh_chunk_id'],
        fresh_version=context['fresh_reference_version'],reveal=reveal,
        counterfactual='FRESH immediately available at original observation; regular absolute control clock retained',
        pending_semantics='none at actual cut; official install invalidates any obsolete generation, no custom discard/update')


def restored_activation(phase):
    a=CommandActivation(.5);old=phase['old_install'];a.install(old['chunk_id'],old['reference_version'],{})
    a.active=a.installed
    r=phase['last_accepted_OLD'];c=phase['incoming_application'];tick=int(c['application_tick'])
    st=dict(c,state_id=tick,tick=tick,sim_time_s=float(c['sim_time_s']),episode_time_s=float(c['episode_time_s']))
    a.accept(deepcopy(r),st['sim_time_s']);a,_,_=preview_activation(a,st,None,None)
    a.install(phase['fresh_chunk_id'],phase['fresh_version'],{})
    return a


def spatial_descriptors(raw,observation,fresh,min_segment=1e-6,reliable_chord=.1):
    raw=np.asarray(raw,float);f=np.asarray(fresh,float);obs=np.asarray(observation,float)
    np.testing.assert_allclose(local_trajectory_to_world(obs,raw),f,rtol=0,atol=1e-12)
    d=np.diff(raw[:,:2],axis=0);ds=np.linalg.norm(d,axis=1)
    r0=float(np.linalg.norm(raw[0,:2]));bear=None if r0<min_segment else float(np.arctan2(raw[0,1],raw[0,0]))
    reliable=next((i for i in range(1,len(raw)) if np.linalg.norm(raw[i,:2]-raw[0,:2])>=reliable_chord),None)
    tang=None if reliable is None else float(np.arctan2(*(raw[reliable,:2]-raw[0,:2])[::-1]))
    yawsteps=wrap_angle(np.diff(raw[:,2]));curv=[]
    for i in range(len(raw)-2):
        a=raw[i,:2];b=raw[i+1,:2];c=raw[i+2,:2];ab=b-a;bc=c-b;ac=c-a
        lengths=np.array([np.linalg.norm(ab),np.linalg.norm(bc),np.linalg.norm(ac)])
        k=None if np.min(lengths)<=min_segment else float(2*(ab[0]*ac[1]-ab[1]*ac[0])/np.prod(lengths))
        curv.append(dict(center_row=i+1,xy_curvature_per_m=k))
    return dict(N=len(raw),row0_distance_m=r0,row0_bearing_relative_robot_rad=bear,row0_yaw_relative_robot_rad=float(wrap_angle(raw[0,2])),
        reliable_tangent_rad=tang,reliable_tangent_end_row=reliable,row_spacings_m=ds,yaw_increments_rad=yawsteps,
        heading_change_per_m=[None if ds[i]<=min_segment else float(yawsteps[i]/ds[i]) for i in range(len(ds))],
        xy_curvature=curv,max_abs_local_lateral_m=float(abs(raw[:,1]).max()),total_XY_arc_m=float(ds.sum()),
        raw_local=raw,world=f,waypoint_intrinsic_dt_s=None,spatial_only_no_required_rates=True)


def slice_for_evaluation(result,start,stop=None):
    stop=len(result['commands']) if stop is None else stop
    assert 0<=start<stop<=len(result['commands'])
    r=deepcopy(result);dt=result['phase']['integration_dt_s'];phase=result['phase']
    offset=result['states'][start]['sim_time_s']
    prior=[c for c in result['commands'][:start] if c['reason']=='new_solve']
    previous_time=float(prior[-1]['sim_time_s']) if prior else float(phase['incoming_application']['sim_time_s'])
    incoming=command(result['commands'][start-1]) if start else phase['u_obs']
    r['phase']=dict(u_minus=incoming,B_sim_s=offset,previous_command_application_sim_s=previous_time)
    r['states']=[dict(s,time_s=i*dt) for i,s in enumerate(result['states'][start:stop+1])]
    r['commands']=result['commands'][start:stop]
    return r


def origin_metrics(result,fresh,environment,scene,config):
    if len(result['commands'])<1:return dict(availability=None,execution=None,reason='no executed interval')
    avail=evaluate(slice_for_evaluation(result,0),fresh,environment,scene,config)
    apps=[i for i,c in enumerate(result['commands']) if c['chunk_id']==result['phase']['fresh_chunk_id'] and c['reason']=='new_solve']
    if not apps:return dict(availability=avail,execution=None,reason='no FRESH command applied')
    i=apps[0];dt=result['phase']['integration_dt_s']
    ex=evaluate(slice_for_evaluation(result,i),fresh,environment,scene,config)
    progress=None;remaining=None
    if ex['full']['join_time_s'] is not None:
        j=int(np.searchsorted(ex['trace']['times_s'],ex['full']['join_time_s']))
        progress=ex['trace']['projection'][j]['progress'];remaining=ex['trace']['remaining_arc_m'][j]
    return dict(availability=avail,execution=ex,first_FRESH_index=i,first_FRESH_tick=result['commands'][i]['application_tick'],
        availability_to_application_s=i*dt,availability_to_application_travel_m=float(sum(abs(command(c)[0])*dt for c in result['commands'][:i])),
        execution_initial_pose=result['states'][i]['pose_world'],incoming_at_application=command(result['commands'][i-1]) if i else result['phase']['u_obs'],
        first_FRESH_command=command(result['commands'][i]),attachment_progress=progress,attachment_remaining_arc_m=remaining,
        full_execution_3s_available=ex['windows']['180']['available'])


def paired_metrics(obs,baseline):
    """Keep signed differences and each exposure; never invent missing times."""
    a=obs['execution'];b=baseline
    if a is None:return []
    def at(x,keys):
        for k in keys:
            if x is None or not isinstance(x,dict):return None
            x=x.get(k)
        return x
    specs=[('initial_position_m',['full','B_distance_m']),('initial_yaw_deg',['full','B_yaw_error_deg']),
        ('first05_max_position_m',['extrema','max_distance_05_m']),
        ('first03_position_AUC_m_s',['windows','18','position_auc_m_s']),
        ('first09_position_AUC_m_s',['windows','54','position_auc_m_s']),
        ('first09_yaw_AUC_rad_s',['windows','54','yaw_auc_rad_s']),
        ('attachment_time_s',['full','join_time_s']),
        ('minimum_execution_clearance_lower_bound_m',['clearance','execution','minimum_clearance_lower_bound_m']),
        ('angular_command_TV_radps',['command','angular_TV_radps']),('max_abs_omega_radps',['command','max_abs_omega_radps'])]
    rows=[]
    for label,path in specs:
        x=at(a,path);y=at(b,path);rows.append(dict(metric=label,observation_start=x,B_start=y,B_minus_observation=None if x is None or y is None else y-x,
            observation_execution_horizon_s=a['full']['observation_horizon_s'],B_execution_horizon_s=b['full']['observation_horizon_s']))
    bp=None
    if b['full']['join_time_s'] is not None:
        j=int(np.searchsorted(b['trace']['times_s'],b['full']['join_time_s']));bp=b['trace']['projection'][j]['progress']
    op=obs['attachment_progress'];rows.append(dict(metric='FRESH_progress_at_attachment',observation_start=op,B_start=bp,B_minus_observation=None if op is None or bp is None else bp-op,
        observation_execution_horizon_s=a['full']['observation_horizon_s'],B_execution_horizon_s=b['full']['observation_horizon_s']))
    return rows

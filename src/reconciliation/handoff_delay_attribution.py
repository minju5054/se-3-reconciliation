"""Frozen saved-state delay/interface diagnostic; no optimizer or VLA interface."""
from __future__ import annotations
import hashlib
import numpy as np
from .handoff_execution_loss import attachment_loss, command_loss
from .gp_se2_reference import prepare_reference
from .gp_se2_ref01_reference import _lineage
from .robotless_online import integrate_unicycle
from .se2 import wrap_angle

CONDITIONS = ('DELAYED_NATIVE', 'DELAYED_LOOKAHEAD', 'LATENCY_FREE_NATIVE', 'LATENCY_FREE_LOOKAHEAD')
IDS = ('episode_014_repeat_01/handoff_025','episode_001_repeat_01/handoff_013',
 'episode_013_repeat_00/handoff_033','episode_016_repeat_01/handoff_007','episode_016_repeat_01/handoff_001',
 'episode_008_repeat_01/handoff_023','episode_001_repeat_01/handoff_011','episode_013_repeat_00/handoff_020',
 'episode_013_repeat_01/handoff_024','episode_013_repeat_01/handoff_021','episode_020_repeat_00/handoff_017',
 'episode_013_repeat_00/handoff_026','episode_013_repeat_01/handoff_029')
DT = float(np.float32(1/60))
STEPS = 54
AUC = ('position_auc_m_s','position_excess_auc_m_s','yaw_auc_rad_s','yaw_excess_auc_rad_s')

def value_hash(a): return hashlib.sha256(np.asarray(a,dtype='<f8').tobytes()).hexdigest()

def memory_at(events, episode, cut):
    """Recover actual worker memory, with conservative poll/update time brackets.

    Worker result timestamp precedes official poll. Isaac receipt is an upper
    bound. A later submit's recorded previous_command is an exact snapshot and
    can tighten that upper bound. Never copy application memory into observation.
    """
    rows=[(i,e) for i,e in enumerate(events) if e.get('episode_id')==episode]
    snapshots=[]; mutations=[]
    for i,e in rows:
        status=e.get('status'); seen=e.get('seen_in_isaac',{}).get('host_monotonic_s')
        if status=='submitted':
            snapshots.append((e['submit_host_monotonic_s'],e['previous_command'],i,'submit_snapshot'))
        if e.get('op')=='reset' and status=='reset' and seen is not None:
            snapshots.append((seen,[0.,0.],i,'confirmed_episode_reset'))
        if e.get('op')=='install' and e.get('row_count')==0:
            lo=e['installed_host_monotonic_s']; hi=seen
            mutations.append((lo,hi,[0.,0.],i,'empty_path_reset'))
        if e.get('type')=='solve_result' and status in ('command','controller_error'):
            if status=='controller_error':
                # The adapter can detect an audit error even when official poll
                # accepts output: do not infer a zero memory from this label.
                mutations.append((e['result_seen_worker_host_monotonic_s'],seen,None,i,'ambiguous_error'))
                continue
            lo=e['result_seen_worker_host_monotonic_s']; hi=seen
            later=[q['submit_host_monotonic_s'] for j,q in rows if j>i and q.get('status')=='submitted'
                   and q['submit_host_monotonic_s']>=lo and q['previous_command']==e['command']]
            if later: hi=min([x for x in [hi,min(later)] if x is not None])
            mutations.append((lo,hi,e['command'],i,'accepted_result'))
    for lo,hi,u,i,kind in mutations:
        if hi is not None and u is not None: snapshots.append((hi,u,i,kind))
    past=[s for s in snapshots if s[0]<=cut]
    if not past:return dict(available=False,reason='NO_CONFIRMED_CONTROLLER_MEMORY',cut_host_s=cut)
    t,u,i,kind=max(past,key=lambda s:(s[0],s[2]))
    uncertain=[dict(lower_host_s=lo,upper_host_s=hi,row_index=j,kind=k) for lo,hi,v,j,k in mutations
               if lo<=cut and (hi is None or hi>cut) and (hi is None or hi>t) and v!=u]
    # Unknown errors after the last snapshot remain unavailable even after receipt.
    uncertain += [dict(row_index=j,kind=k) for lo,hi,v,j,k in mutations if v is None and t<lo<=cut]
    if uncertain:return dict(available=False,reason='CONTROLLER_UPDATE_STRADDLES_CUT',cut_host_s=cut,uncertain=uncertain)
    return dict(available=True,previous_control=list(u),cut_host_s=cut,confirmed_by_host_s=t,
                source_event_row=i,source_kind=kind,reason=None)

def boundary_state(context, states, commands, events, when):
    observation=when=='LATENCY_FREE'
    sid=context['obs_state_id' if observation else 'switch_state_id']
    state=next(r for r in states if int(r['state_id'])==sid)
    pose=[float(state[k]) for k in ('x','y','yaw')]
    expected=context['R_obs' if observation else 'B']
    if pose!=expected:raise ValueError('saved boundary pose differs')
    incoming=next(r for r in commands if int(r['command_id'])==int(state['incoming_command_id']))
    u=[float(incoming[k]) for k in ('v_mps','omega_radps')]
    previous=next(r for r in states if int(r['state_id'])==int(incoming['application_state_id']))
    rebuilt=integrate_unicycle([float(previous[k]) for k in ('x','y','yaw')],u,float(state['sim_time_s'])-float(previous['sim_time_s']))
    error=rebuilt-np.asarray(pose);error[2]=wrap_angle(error[2])
    if np.max(abs(error))>1e-10:raise ValueError('incoming physical command does not reconstruct boundary')
    cut=context['t_obs' if observation else 't_switch']['host_monotonic_s']
    if float(incoming['host_monotonic_s'])>cut:raise ValueError('physical command not yet applied at cut')
    memory=memory_at(events,context['episode_id'],cut)
    return dict(pose_world=pose,u_minus=u,state_id=sid,physical_command=incoming,memory=memory,
                saved_state=state,available=memory['available'],
                reason=None if memory['available'] else ('ORACLE_CONTROLLER_MEMORY_UNAVAILABLE' if observation else 'B_CONTROLLER_MEMORY_UNAVAILABLE'),
                memory_minus_physical=None if not memory['available'] else (np.asarray(memory['previous_control'])-u).tolist())

def fixed_references(fresh, B):
    raw=np.asarray(fresh,dtype=np.float64)
    prep=prepare_reference(raw,B,[10.,10.,1.]);common=prep['common_world'];k=prep['first_future_row_index']
    constant=dict(constant_reference=True,retained_source_row_count=1,original_source_row_index=k) if len(raw)-k==1 else None
    return dict(native=raw.tolist(),common=common.tolist(),native_hash=value_hash(raw),common_hash=value_hash(common),
                native_lineage=_lineage(raw,0,raw,False),common_lineage=_lineage(raw,k,common,True),
                suffix_start_index=k,constant_metadata=constant,
                semantics='PREPARED_REFERENCE_PLUS_SOURCE_PROGRESS_SELECTOR',prepared_at='B_ONCE',
                preparation_clock_s=[.1,3.],intrinsic_waypoint_dt_s=None)

def metrics(result,event,environment,config):
    t=np.asarray(result['times_s']);p=np.asarray(result['poses_world']);u=np.asarray(result['commands'])
    complete=len(u)==STEPS and result['status']=='COMPLETED'
    base=dict(case_id=event['case_id'],episode_id=event['episode_id'],cohort=event['cohort'],condition=result['condition'],
              available=result['status']!='UNAVAILABLE',status=result['status'],complete=complete,
              reason=result.get('reason'),integration_steps=len(u),solve_count=len(result.get('solves',[])),
              official_mpc_wall_s=sum(s['official_solve_wall_s'] or 0 for s in result.get('solves',[])),
              rollout_wall_s=result.get('wall_s'),controller_failures=sum(not s['success'] for s in result.get('solves',[])),
              primary=None,partial=None,command=None,geometry=None)
    if len(t)<2:return base
    att=attachment_loss(t,p,event['fresh_world'])
    cmd=command_loss(t,p,u,result['initial']['u_minus'],config)
    cmd['reconstruction_only_no_new_rollout']=False
    geom=environment.check_trajectory(t,p,radius=.2,required_clearance=.05,curved_path_error_bound_m=cmd['curve_bound_m'])
    base.update(primary=att if complete else None,partial=None if complete else att,command=cmd,geometry=geom)
    return base

def category_change(a,b):
    if a==b:return 'UNCHANGED'
    if b=='OBSERVED_SAMPLED_JOIN':return 'IMPROVED_TO_OBSERVED_JOIN'
    if a=='OBSERVED_SAMPLED_JOIN':return 'WORSENED_FROM_OBSERVED_JOIN'
    if a=='NO_TUBE_ENTRY_OBSERVED' and b in ('TRANSIENT_ENTRY_THEN_EXIT','TUBE_ENTERED_DWELL_RIGHT_CENSORED'):return 'ENTRY_GAIN_DWELL_UNRESOLVED'
    if b=='NO_TUBE_ENTRY_OBSERVED':return 'ENTRY_LOSS_DWELL_UNRESOLVED'
    return 'CHANGED_CENSORING_NOT_ORDERED'

def paired(rows):
    gaps=[];transitions=[]
    for cid in dict.fromkeys(r['case_id'] for r in rows):
        lookup={r['condition']:r for r in rows if r['case_id']==cid};example=next(iter(lookup.values()))
        for name,a,b in [('delay_native',CONDITIONS[0],CONDITIONS[2]),('delay_lookahead',CONDITIONS[1],CONDITIONS[3]),
                          ('interface_delayed',CONDITIONS[0],CONDITIONS[1]),('interface_latency_free',CONDITIONS[2],CONDITIONS[3])]:
            x,y=lookup[a],lookup[b];valid=x['primary'] is not None and y['primary'] is not None
            row=dict(case_id=cid,episode_id=example['episode_id'],cohort=example['cohort'],comparison=name,available=valid,
                     reason=None if valid else 'MISSING_EQUAL_EXPOSURE_PAIR')
            for key in AUC:row[key]=x['primary'][key]-y['primary'][key] if valid else None
            row['both_safe_motion_valid']=bool(valid and all(r['geometry']['clearance_valid'] and r['command']['nominal_command_grid_valid'] and not r['controller_failures'] for r in [x,y]))
            gaps.append(row)
            if name.startswith('delay_'):
                ca=x['primary']['observation_status'] if x['primary'] else None;cb=y['primary']['observation_status'] if y['primary'] else None
                transitions.append(dict(case_id=cid,cohort=example['cohort'],selector=name[6:],delayed=ca,latency_free=cb,
                                        change=category_change(ca,cb) if valid else 'UNAVAILABLE'))
        gn,gl=gaps[-4],gaps[-3]
        gaps.append(dict(case_id=cid,episode_id=example['episode_id'],cohort=example['cohort'],comparison='interaction',
                         available=gn['available'] and gl['available'],reason=None if gn['available'] and gl['available'] else 'MISSING_EQUAL_EXPOSURE_PAIR',
                         both_safe_motion_valid=gn['both_safe_motion_valid'] and gl['both_safe_motion_valid'],
                         **{k:gn[k]-gl[k] if gn['available'] and gl['available'] else None for k in AUC}))
    return gaps,transitions

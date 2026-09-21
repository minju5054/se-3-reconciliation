"""Source-only reveal qualification. No optimizer/MPC invocation or outcomes."""
import json
from pathlib import Path
import numpy as np
from shapely.geometry import LineString
from .gp_se2_join01 import nearest_index, box_polygon
from .gp_se2_join01_environment import RevealEnvironment
from .robotless_projection_handoff import projection_geometry
from .gp_se2_reference import directed_gate_crossings


def qualify(context, obstacle, base_environment, protocol, *, visibility, timing):
    q=protocol['qualification'];env=RevealEnvironment(base_environment,obstacle)
    old=np.asarray(context['old_world']);fresh=np.asarray(context['fresh_world']);b=np.asarray(context['B_world'])
    j0=nearest_index(fresh,b);oj=nearest_index(old,b)
    suffix=fresh[j0:];oldfuture=old[oj:]
    original_old=base_environment.check_polyline(oldfuture)
    revealed_old=env.check_polyline(oldfuture)
    fs=env.check_polyline(suffix);bv=env.query(b[:2])
    lengths=np.linalg.norm(np.diff(suffix[:,:2],axis=0),axis=1);arc=float(lengths.sum())
    chord=float(np.linalg.norm(suffix[-1,:2]-suffix[0,:2]))
    # Existing segment projection semantics, densely query original OLD future.
    dense=[]
    for a,z in zip(oldfuture[:-1],oldfuture[1:]):
        count=max(2,int(np.ceil(np.linalg.norm(z[:2]-a[:2])/.01))+1)
        from .gp_se2_reference import interpolate_rows
        dense.extend(interpolate_rows([a,z],[0.,1.],np.linspace(0,1,count)))
    if not dense:dense=[oldfuture[0]]
    projections=[projection_geometry(p,fresh) for p in dense]
    cross=max(r['e_perp_m'] for r in projections)
    yaw=max(r['abs_e_yaw_deg'] for r in projections)
    center=np.asarray(obstacle['pose_world']);n=np.array([np.cos(center[2]),np.sin(center[2])])
    front=float((fresh[-1,:2]-center[:2])@n-obstacle['dimensions_m'][0]/2-.25)
    failures=[]
    flags=dict(old_pre_reveal_valid=original_old['clearance_valid'],
        revealed_old_unsafe=not revealed_old['clearance_valid'] and revealed_old['workspace_known'],
        fresh_suffix_valid=fs['clearance_valid'],B_valid=bv['status']=='CLEARANCE_VALID',
        nontrivial_disagreement=cross>=q['minimum_cross_track_m'] or yaw>=q['minimum_yaw_difference_deg'],
        enough_suffix=len(suffix)>=q['minimum_remaining_rows'] and arc>=q['minimum_remaining_arc_m'],
        translation_dominant=arc>0 and chord/arc>=q['minimum_chord_over_arc'] and bool(np.all(lengths>=q['minimum_translation_segment_m'])),
        simple_polyline=len(fresh)>1 and LineString(fresh[:,:2]).is_simple,
        endpoint_beyond_box=front>=0,obstacle_visible=visibility['obstacle_pixels']>=protocol['reveal']['minimum_visible_pixels'],
        old_observation_before_reveal=timing['old_observation_before_reveal'],
        fresh_observation_after_reveal=timing['fresh_observation_after_reveal'],
        inflight_execution_valid=timing['inflight_execution_valid'])
    # Keep any pre-existing real gate crossing. No arbitrary side gate is invented.
    gates=[]
    for g in base_environment.gates:
        center2=np.asarray(g.get('center_world_xy_m',g.get('center_xy')))
        normal=np.asarray(g.get('normal_world_xy',g.get('normal_xy')))
        sides=(suffix[:,:2]-center2)@normal
        if np.min(sides)<-1e-6 and np.max(sides)>1e-6:
            ng=dict(gate_id=g['gate_id'],center_xy=center2.tolist(),normal_xy=normal.tolist(),half_width_m=g['half_width_m'])
            tt=np.linspace(.1,3.,len(suffix))
            report=directed_gate_crossings(suffix[:,:2],tt,[ng])
            if not report['valid']:
                ng['normal_xy']=(-normal).tolist();report=directed_gate_crossings(suffix[:,:2],tt,[ng])
            if report['valid']:
                ng['time_s']=report['gates'][0]['first_directed_crossing_s'];gates.append(ng)
            else:flags['interpretable_existing_gate']=False
    failures=[k for k,v in flags.items() if not v]
    gates.sort(key=lambda g:g['time_s'])
    return dict(qualified=not failures,flags=flags,failure_reasons=failures,
        nearest_original_index=j0,old_nearest_index=oj,original_old=original_old,revealed_old=revealed_old,
        fresh_suffix=fs,B_query=bv,maximum_cross_track_m=cross,maximum_projected_pose_yaw_difference_deg=yaw,
        remaining_arc_m=arc,remaining_rows=len(suffix),chord_over_arc=None if arc==0 else chord/arc,
        goal_beyond_box_margin_m=front,visibility=visibility,timing=timing,
        goal_route=dict(goal_world=fresh[-1].tolist(),position_tolerance_m=.15,yaw_tolerance_rad=np.pi/12,
            gates=gates,route_status='REQUIRED' if gates else 'NOT_REQUIRED_SIMPLE_CORRIDOR',
            reason='preserve any crossed original gate; no semantic side-of-box requirement invented'),
        optimizer_outcomes_used=False)


def qualification_from_episode(source, episode, base, protocol):
    from .gp_se2_rollout import load_frozen_context
    root=Path(source)/'episodes'/episode
    metadata=json.loads((root/'metadata.json').read_text())
    context_path=root/'handoffs/handoff_000/context.json'
    if not context_path.exists() or json.loads(context_path.read_text()).get('status') not in ('VALID_HANDOFF_MOVING','VALID_HANDOFF_STATIONARY'):
        reason=metadata.get('termination_reason','')
        placement_short='PLACEMENT_OUTSIDE_ACTUAL_OLD_FUTURE' in reason
        technical=metadata['status'] in ('TECHNICAL_INVALID','CONTROLLER_ERROR','MODEL_ERROR','PROTOCOL_ERROR','EXECUTOR_STALLED','SCENE_INVALID') and not placement_short
        return dict(qualified=False,failure_reasons=['PLACEMENT_OUTSIDE_ACTUAL_OLD_FUTURE' if placement_short else 'TECHNICAL_RUNTIME_BLOCKER' if technical else 'NO_ACTIVATED_FRESH'],
                    technical_blocker=technical,source_status=metadata['status'],error=reason,optimizer_outcomes_used=False)
    try:
        context=load_frozen_context(source,episode,'handoff_000')
        raw=json.loads((root/'handoffs/handoff_000/context.json').read_text())
        reveal=json.loads((root/'obstacle_reveal.json').read_text())
        visibility=json.loads((root/'visibility'/f"{raw['fresh_observation_pose_time']['frame_id']}.json").read_text())
        ids=raw['client_inflight_state_range']
        loops=[json.loads(s) for s in (root/'loop.jsonl').read_text().splitlines()]
        overlap=[s for s in loops if ids['start_state_id']<s['state_id']<=ids['end_state_id']]
        span_sim=(overlap[-1]['sim_time_s']-overlap[0]['sim_time_s']) if len(overlap)>1 else 0
        span_host=(overlap[-1]['host_monotonic_s']-overlap[0]['host_monotonic_s']) if len(overlap)>1 else 0
        rtf=span_sim/span_host if span_host>0 else None
        maximum=max((s['loop_interval_host_s'] for s in overlap),default=float('inf'))
        timing=dict(old_observation_before_reveal=raw['old_observation_pose_time']['time']<reveal['timestamp']['sim_time_s'],
            fresh_observation_after_reveal=raw['t_obs']['sim_time_s']>=reveal['timestamp']['sim_time_s'],
            inflight_execution_valid=bool(rtf is not None and protocol['qualification']['inflight_rtf_range'][0]<=rtf<=protocol['qualification']['inflight_rtf_range'][1]
                and maximum<=protocol['qualification']['maximum_overlap_loop_stall_s'] and raw['timing_flags']['overlap_observed']),
            inflight_rtf=rtf,maximum_inflight_loop_stall_s=maximum,source_timestamps=context['source_timestamps'],
            old_observation=raw['old_observation_pose_time'],reveal=reveal['timestamp'])
        result=qualify(context,reveal['obstacle'],base,protocol,visibility=visibility,timing=timing)
        return dict(result,context=context,obstacle=reveal['obstacle'])
    except (ValueError,KeyError,FileNotFoundError) as exc:
        return dict(qualified=False,failure_reasons=['SOURCE_INTEGRITY_OR_SCHEMA_BLOCKER'],error=f'{type(exc).__name__}: {exc}',
                    technical_blocker=True,optimizer_outcomes_used=False)

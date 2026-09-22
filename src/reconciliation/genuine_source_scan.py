"""Read-only genuine source audit. No new model, optimizer, controller or rollout.

Source predicates are separate from performance. Endpoint distance is decomposed
using the existing raw projection/window tangent; no new correspondence is chosen.
"""
from __future__ import annotations

import csv
import hashlib
from pathlib import Path

import numpy as np
import yaml

from .gp_se2_attach01_source import digest, read, scan as original_scan
from .gp_se2_environment import HospitalEnvironment
from .online_handoff_analysis import incoming_execution_direction, projection_at_boundary
from .robotless_online import integrate_unicycle
from .se2 import wrap_angle


def order_key(row):
    case = row['case_id']
    return hashlib.sha256(('MOVING-SOURCE-v1:' + case).encode()).hexdigest(), case


def mismatch(boundary, native, incoming, thresholds):
    p = projection_at_boundary(boundary, native)
    w = p['window_0_10m']
    tangent_valid = bool(w['available'] and w['chord_m'] >= thresholds['minimum_projection_window_chord_m'])
    normal = along = direction = None
    if tangent_valid:
        t = np.array([np.cos(w['phi_rad']), np.sin(w['phi_rad'])])
        delta = np.asarray(boundary)[:2] - p['Q_xy_world_m']
        along = float(delta @ t)
        normal = float(delta @ np.array([-t[1], t[0]]))
    reliable = bool(tangent_valid and incoming['available'] and incoming['chord_m'] >= thresholds['minimum_incoming_chord_m'])
    if reliable:
        direction = float(np.degrees(abs(wrap_angle(w['phi_rad'] - incoming['phi_rad']))))
    lateral_pass = normal is not None and abs(normal) >= thresholds['minimum_lateral_gap_m']
    direction_pass = direction is not None and direction >= thresholds['minimum_direction_or_pose_yaw_deg']
    yaw_pass = tangent_valid and p['abs_e_yaw_deg'] >= thresholds['minimum_direction_or_pose_yaw_deg']
    return dict(projection=p, signed_lateral_gap_m=normal, along_tangent_gap_m=along,
        reliable_window_direction_deg=direction, direction_reliable=reliable,
        pose_yaw_difference_deg=p['abs_e_yaw_deg'], lateral_mismatch=lateral_pass,
        direction_mismatch=direction_pass, pose_yaw_mismatch=yaw_pass,
        strict_mismatch=bool(lateral_pass or direction_pass or yaw_pass),
        position_AND_orientation_mismatch=bool(lateral_pass and (direction_pass or yaw_pass)),
        original_distance_mismatch=p['e_perp_m'] >= thresholds['minimum_lateral_gap_m'])


def motion_record(context, states, commands):
    """Authenticate recorded physical commands and reconstruct only existing ticks."""
    index = {int(r['state_id']): i for i, r in enumerate(states)}
    a, b = [index[context[k]] for k in ('obs_state_id', 'switch_state_id')]
    if b <= a:
        raise ValueError('observation must precede boundary')
    times = np.array([float(r['sim_time_s']) for r in states])
    poses = np.array([[float(r[k]) for k in ('x', 'y', 'yaw')] for r in states])
    if np.any(np.diff(times) <= 0):
        raise ValueError('nonchronological source execution')
    if not np.allclose(poses[a], context['R_obs'], atol=1e-12, rtol=0) or not np.allclose(poses[b], context['B'], atol=1e-12, rtol=0):
        raise ValueError('source observation/B pose mismatch')
    exact_arc = 0.; errors = []; curve_bounds = []
    for i in range(a+1, b+1):
        c = commands[int(states[i]['incoming_command_id'])]
        u = [float(c['v_mps']), float(c['omega_radps'])]
        dt = times[i] - times[i-1]
        expected = integrate_unicycle(poses[i-1], u, dt)
        error = expected - poses[i]; error[2] = wrap_angle(error[2])
        errors.append(float(np.abs(error).max()))
        exact_arc += abs(u[0])*dt
        # Uniform second derivative bound for a held unicycle arc vs its chord.
        curve_bounds.append(abs(u[0]*u[1])*dt*dt/8)
    if max(errors) > 1e-10:
        raise ValueError('saved applied commands do not reconstruct states')
    c = commands[int(states[b]['incoming_command_id'])]
    declared = context['pre_switch_command']
    if any(c[k] != str(declared[k]) for k in ('command_id',)) or any(float(c[k]) != declared[k] for k in ('v_mps','omega_radps')):
        raise ValueError('physical u_minus differs from command stream')
    incoming = incoming_execution_direction(poses, times, b)
    return dict(v_minus_mps=float(c['v_mps']), omega_minus_radps=float(c['omega_radps']),
        previous_control=context['first_fresh_solve']['previous_command'],
        observation_to_B_arc_m=float(exact_arc), observation_to_B_chord_sum_m=float(np.linalg.norm(np.diff(poses[a:b+1,:2],axis=0),axis=1).sum()),
        observation_to_B_net_m=float(np.linalg.norm(poses[b,:2]-poses[a,:2])),
        observation_to_B_sim_s=float(times[b]-times[a]), reconstruction_max_abs_error=max(errors),
        curve_bound_m=max(curve_bounds), incoming=incoming,
        states=poses[a:b+1], times=times[a:b+1])


def subsets(row, thresholds):
    p = row['source_predicates']
    base = all(p[k] for k in ('original_source_eligibility','boundary_clearance','enough_future','unambiguous_projection','exact_recorded_timing','physical_state_provenance'))
    moving = (row['motion']['v_minus_mps'] > thresholds['forward_speed_strictly_greater_mps'] and
              row['motion']['observation_to_B_arc_m'] >= thresholds['observation_to_boundary_minimum_arc_m'])
    future_safe = bool(p['raw_future_clearance'] and p['common_future_clearance'])
    full_safe = bool(future_safe and row['entire_raw_environment']['clearance_valid'])
    match = row['mismatch']['strict_mismatch']
    source_safe = bool(row['past_environment']['clearance_valid'])
    candidate = bool(base and moving and full_safe and source_safe and match)
    obstacle = row['raw_suffix_clearance_m'] <= thresholds['obstacle_sensitive_maximum_suffix_clearance_m']
    no_gate = p['no_required_gate_and_known_route']
    return dict(base_source_valid=base, actual_moving=moving, safe_future=future_safe, safe_entire_raw_and_common=full_safe,
        observed_prefix_safe=source_safe, sufficient_mismatch=match, candidate=candidate,
        candidate_no_gate=bool(candidate and no_gate), candidate_obstacle_sensitive=bool(candidate and obstacle),
        candidate_obstacle_sensitive_no_gate=bool(candidate and obstacle and no_gate),
        candidate_lateral=bool(candidate and row['mismatch']['lateral_mismatch']),
        candidate_position_AND_orientation=bool(candidate and row['mismatch']['position_AND_orientation_mismatch']),
        future_only_candidate=bool(base and moving and future_safe and source_safe and match),
        original_distance_only_candidate=bool(base and moving and full_safe and source_safe and row['mismatch']['original_distance_mismatch']))


def summarize(rows):
    fields = list(rows[0]['flags'])
    counts = {k: sum(r['flags'][k] for r in rows) for k in fields}
    cohorts = {}
    for key in ('candidate','candidate_no_gate','candidate_obstacle_sensitive','candidate_obstacle_sensitive_no_gate','future_only_candidate'):
        chosen = sorted([r for r in rows if r['flags'][key]], key=order_key)
        cohorts[key] = dict(count=len(chosen), unique_episodes=len({r['case_id'].split('/')[0] for r in chosen}),
            unique_ordered_raw_pairs=len({r['ordered_raw_pair'] for r in chosen}), case_ids=[r['case_id'] for r in chosen])
    survivors = rows
    cumulative = {}
    for key in ('base_source_valid','safe_entire_raw_and_common','observed_prefix_safe','actual_moving','sufficient_mismatch'):
        survivors = [r for r in survivors if r['flags'][key]]
        cumulative[key] = len(survivors)
    return dict(source_count=len(rows), independent_counts=counts, cumulative_counts=cumulative, subsets=cohorts,
        original_entire_raw_near_obstacle_candidates=[r['case_id'] for r in rows if r['flags']['candidate'] and r['entire_raw_environment']['minimum_clearance_m'] <= .20],
        new_inference=0,new_GP_or_rigid_solve=0,new_MPC_solve=0,new_rollout=0,
        historical_performance_used=False, selected_for_optimization=None)


def run_scan(root, config):
    root = Path(root).resolve()
    protocol = yaml.safe_load((root/config['reuse_source_policy']).read_text())
    for key in ('source_run','original_run','source_inventory_run'):
        if config[key] != protocol[key]:
            raise ValueError('source lineage differs from reused scan')
    provenance = read(root/config['source_inventory_run']/'source.json')
    environment = HospitalEnvironment.load(provenance['environment_path'])
    original = original_scan(root, protocol, environment)
    hashes = original['source_hashes']; cache = {}; rows = []
    for source in original['all_candidates']:
        cid=source['case_id']; episode=cid.split('/')[0]
        ep=root/config['source_run']/'episodes'/episode
        context=read(source['source_paths']['context'])
        if episode not in cache:
            completion=ep/'completion.json'; manifest={r['path']:r for r in read(completion)['raw_manifest']['files']}
            hashes[str(completion)]=digest(completion)
            streams=[]
            for name in ('execution.csv','commands.csv'):
                path=ep/name; h=digest(path)
                if h != manifest[name]['sha256'] or path.stat().st_size != manifest[name]['bytes']:
                    raise ValueError('source stream hash mismatch: '+str(path))
                hashes[str(path)]=h
                with path.open() as stream:
                    streams.append(list(csv.DictReader(stream)))
            cache[episode]=(streams[0],{int(r['command_id']):r for r in streams[1]})
        motion=motion_record(context,*cache[episode])
        native=np.load(source['source_paths']['fresh_world'])
        mismatch_result=mismatch(context['B'],native,motion['incoming'],config['thresholds'])
        metrics=read(source['source_paths']['metrics'])
        if mismatch_result['projection'] != metrics['fresh_projection'] or motion['incoming'] != metrics['incoming_execution']:
            raise ValueError('original projection/incoming metric does not reproduce: '+cid)
        past=environment.check_trajectory(motion['times'],motion['states'],curved_path_error_bound_m=motion['curve_bound_m'])
        row=dict(case_id=cid,ordered_raw_pair=source['ordered_raw_pair'],source_paths=source['source_paths'],
            raw_local_hash_changed=context['old_raw_local_ref']['sha256'] != context['fresh_raw_local_ref']['sha256'],
            source_predicates=source['predicates'],original_rejection_reasons=source['original_rejection_reasons'],
            route_status=source['route_status'],B_world=context['B'],observation_pose_world=context['R_obs'],
            motion={k:v for k,v in motion.items() if k not in ('states','times')},mismatch=mismatch_result,
            entire_raw_environment=environment.check_polyline(native),past_environment=past,
            boundary_environment=environment.query(context['B'][:2]),
            raw_suffix_clearance_m=source['raw_suffix_environment']['minimum_clearance_m'],
            common_clearance_m=source['prepared_environment']['minimum_clearance_m'],
            native_rows=source['original_row_count'],suffix_rows=source['suffix_row_count'],suffix_arc_m=source['suffix_arc_m'],
            suffix_start=source['suffix_start'],common_value_sha256=source['common_value_sha256'],
            timing={k:context[k] for k in ('t_obs','t_request','t_ready_host','t_install','t_switch')},
            inference_client_rtt_s=metrics['client_rtt_s'])
        row['flags']=subsets(row,config['thresholds'])
        row['rejection_reasons']=[k for k in ('base_source_valid','actual_moving','safe_entire_raw_and_common','observed_prefix_safe','sufficient_mismatch') if not row['flags'][k]]
        rows.append(row)
    for path,h in hashes.items():
        if digest(path)!=h:raise ValueError('source changed during scan: '+path)
    return dict(rows=rows,summary=summarize(rows),source_hashes=hashes,environment_path=provenance['environment_path'])

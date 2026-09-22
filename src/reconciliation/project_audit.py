"""PROJECT-AUDIT-01: deterministic SAVED-ONLY metrics; no controller/model calls.

World XY metres, yaw CCW radians. References are observation-anchored original
FRESH. Timing comes from saved execution; raw waypoint indices are not time.
"""
from __future__ import annotations

from collections import Counter
import numpy as np

from .handoff_execution_loss import attachment_loss, command_loss
from .online_handoff_analysis import projection_at_boundary
from .se2 import wrap_angle


def shape_features(fresh):
    """Intrinsic route shape, independent of B and execution outcome.

    Segment tangents shorter than 1e-6 m are undefined and excluded; pose yaw
    variation still retains rotations in place. No resampling or new transform.
    """
    f = np.asarray(fresh, dtype=float)
    if f.ndim != 2 or f.shape[1] != 3 or not len(f) or not np.isfinite(f).all():
        raise ValueError('finite nonempty Nx3 reference required')
    d = np.diff(f[:, :2], axis=0)
    lengths = np.linalg.norm(d, axis=1)
    valid = d[lengths > 1e-6]
    tangents = np.arctan2(valid[:, 1], valid[:, 0])
    return dict(raw_arc_m=float(lengths.sum()),
                raw_yaw_variation_deg=float(np.degrees(np.abs(wrap_angle(np.diff(f[:, 2]))).sum())),
                raw_tangent_variation_deg=float(np.degrees(np.abs(wrap_angle(np.diff(tangents))).sum())),
                tangent_segments=len(valid))


def saved_window(context, states, commands):
    """B is pre-integration state; exclude the next reference's first command.

    Episode termination is explicitly censored. Every applied command must have
    its following saved state; an unapplied trailing attempt is never integrated.
    """
    index = {int(s['state_id']): i for i, s in enumerate(states)}
    by_state = {int(c['application_state_id']): c for c in commands}
    start = index[context['switch_state_id']]
    fresh = context['fresh_chunk_id']
    end = start
    while end < len(states)-1:
        command = by_state.get(int(states[end]['state_id']))
        if command is None:
            raise ValueError('missing applied command inside saved execution')
        if command['chunk_id'] != fresh:
            break
        following = states[end+1]
        if int(following['incoming_command_id']) != int(command['command_id']):
            raise ValueError('command does not produce following state')
        end += 1
    if end <= start:
        raise ValueError('no recorded FRESH interval')
    selected = states[start:end+1]
    poses = np.asarray([[float(s[k]) for k in ('x', 'y', 'yaw')] for s in selected])
    absolute = np.asarray([float(s['sim_time_s']) for s in selected])
    if not np.array_equal(poses[0], context['B']) or np.any(np.diff(absolute) <= 0):
        raise ValueError('invalid B or nonchronological execution')
    applied = [by_state[int(s['state_id'])] for s in selected[:-1]]
    next_command = by_state.get(int(states[end]['state_id']))
    reason = ('NEXT_REFERENCE_APPLICATION' if next_command and next_command['chunk_id'] != fresh
              else 'EPISODE_END')
    return dict(poses=poses, times=absolute-absolute[0], absolute_times=absolute,
                commands=applied, start_row=start, end_row=end, end_reason=reason,
                next_chunk_id=None if reason == 'EPISODE_END' else next_command['chunk_id'])


def control_eligible(feature, protocol):
    return bool(feature['source_safe'] and feature['lateral_m'] is not None
                and abs(feature['lateral_m']) <= protocol['low_lateral_max_m']
                and feature['pose_yaw_deg'] <= protocol['low_pose_yaw_max_deg']
                and feature['direction_deg'] is not None
                and feature['direction_deg'] <= protocol['low_reliable_direction_max_deg'])


def match_controls(features, hard_ids, protocol):
    """Use ONLY input shape, B motion/mismatch, identity and saved exposure.

    Lifetime is realized exposure, hence a possible post-treatment confounder;
    matching it does not establish a randomized causal effect. No outcome fields
    are accessed. Matching with replacement and lexical ties is deterministic.
    """
    fields = [('raw_yaw_variation_deg', 'raw_yaw_variation_caliper_deg'),
              ('raw_tangent_variation_deg', 'raw_tangent_variation_caliper_deg'),
              ('v_minus_mps', 'speed_caliper_mps'), ('lifetime_s', 'lifetime_caliper_s'),
              ('raw_arc_m', 'raw_arc_caliper_m')]
    lookup = {r['case_id']: r for r in features}
    pool = [r for r in features if control_eligible(r, protocol) and r['case_id'] not in hard_ids]
    matches = []
    for cid in hard_ids:
        hard = lookup[cid]
        eligible = []
        for control in pool:
            if protocol['exclude_same_episode'] and control['episode_id'] == hard['episode_id']:
                continue
            if protocol['same_source_category'] and control['category'] != hard['category']:
                continue
            differences = [abs(hard[f]-control[f]) for f, _ in fields]
            if all(d <= protocol[p] for d, (_, p) in zip(differences, fields)):
                eligible.append((tuple(differences)+(control['case_id'],), control, differences))
        eligible.sort(key=lambda r: r[0])
        chosen = eligible[0] if eligible else None
        matches.append(dict(hard_id=cid, control_id=None if chosen is None else chosen[1]['case_id'],
                            eligible_control_count=len(eligible),
                            absolute_feature_differences=None if chosen is None else
                            dict(zip([f for f, _ in fields], chosen[2])),
                            reason='MATCHED' if chosen else 'NO_CONTROL_WITHIN_FROZEN_CALIPERS'))
    return matches


def compact_loss(loss):
    return {k: v for k, v in loss.items() if k != 'trace'}


def execution_metrics(window, fresh, old, context, command_config, attachment_config):
    t, poses = window['times'], window['poses']
    u = np.asarray([[float(c['v_mps']), float(c['omega_radps'])] for c in window['commands']])
    minus = [context['pre_switch_command'][k] for k in ('v_mps', 'omega_radps')]
    loss = attachment_loss(t, poses, fresh)
    commands = command_loss(t, poses, u, minus, command_config)
    n = attachment_config['common_steps']
    common = attachment_loss(t[:n+1], poses[:n+1], fresh) if len(u) >= n else None
    command_common = command_loss(t[:n+1], poses[:n+1], u[:n], minus, command_config) if common else None
    projection = loss['trace']['projection']
    d = np.asarray(loss['trace']['distance_m'])
    prefix = t <= attachment_config['first_growth_window_s']
    old_b = projection_at_boundary(poses[0], old)
    new_b = projection_at_boundary(poses[0], fresh)
    signs = np.sign(u[:, 1][np.abs(u[:, 1]) > 1e-6])
    return dict(full=compact_loss(loss), common=None if common is None else compact_loss(common),
                command={k: v for k, v in commands.items() if k not in ('commands', 'command_start_times_s')},
                common_command=None if command_common is None else {k: v for k, v in command_common.items() if k not in ('commands','command_start_times_s')},
                B_inside_joint_tube=bool(loss['trace']['good'][0]),
                initial_growth_0_30s_m=float(d[prefix].max()-d[0]),
                growth_observed=bool(d[prefix].max()-d[0] > attachment_config['growth_reporting_floor_m']),
                maximum_growth_m=float(d.max()-d[0]),
                original_row_progress_gain=float(projection[-1]['progress']-projection[0]['progress']),
                original_goal_distance_reduction_m=float(np.linalg.norm(poses[0,:2]-fresh[-1,:2])-np.linalg.norm(poses[-1,:2]-fresh[-1,:2])),
                stop_intervals=int(np.sum(np.abs(u[:,0])<=1e-6)),
                deceleration_intervals=int(np.sum(np.diff(np.r_[minus[0],u[:,0]]) < -1e-6)),
                angular_direction_reversals=int(np.sum(np.diff(signs)!=0)),
                boundary_same_pose_old_distance_m=old_b['e_perp_m'],
                boundary_same_pose_new_distance_m=new_b['e_perp_m'],
                boundary_reference_distance_jump_m=new_b['e_perp_m']-old_b['e_perp_m'],
                boundary_same_pose_old_yaw_deg=old_b['abs_e_yaw_deg'],
                boundary_same_pose_new_yaw_deg=new_b['abs_e_yaw_deg'],
                trace=loss['trace'], end_reason=window['end_reason'],
                next_chunk_id=window['next_chunk_id'])


def summarize_events(records):
    """Explicit event/episode denominators; no IID inference or pooled score."""
    if not records:
        return dict(events=0, episodes=0)
    episodes = sorted({r['episode_id'] for r in records})
    common = [r for r in records if r['common'] is not None]
    episode_auc = {ep:float(np.mean([r['common']['position_auc_m_s'] for r in common if r['episode_id']==ep]))
                   for ep in episodes if any(r['episode_id']==ep for r in common)}
    episode_rates = {ep:dict(events=sum(r['episode_id']==ep for r in records),
        B_inside_fraction=float(np.mean([r['B_inside_joint_tube'] for r in records if r['episode_id']==ep])),
        growth_fraction=float(np.mean([r['growth_observed'] for r in records if r['episode_id']==ep])),
        join_fraction=float(np.mean([r['full']['join_success'] for r in records if r['episode_id']==ep]))) for ep in episodes}
    return dict(events=len(records), episodes=len(episodes),
                B_inside_joint_tube=sum(r['B_inside_joint_tube'] for r in records),
                initial_growth=sum(r['growth_observed'] for r in records),
                attachment_status=dict(Counter(r['full']['observation_status'] for r in records)),
                common_window_available=len(common), common_window_missing=len(records)-len(common),
                event_mean_position_auc_m_s=None if not common else float(np.mean([r['common']['position_auc_m_s'] for r in common])),
                episode_mean_position_auc_m_s=None if not episode_auc else float(np.mean(list(episode_auc.values()))),
                episode_auc=episode_auc, episode_rates=episode_rates,
                episode_equal_B_inside_fraction=float(np.mean([r['B_inside_fraction'] for r in episode_rates.values()])),
                episode_equal_growth_fraction=float(np.mean([r['growth_fraction'] for r in episode_rates.values()])),
                episode_equal_join_fraction=float(np.mean([r['join_fraction'] for r in episode_rates.values()])))

"""Counterfactual OLD-relative continuation; no correction of either model path."""
from collections import Counter

import numpy as np

from reconciliation.robotless_handoff_screening import MOTION, ordered_pair_hash, percentiles
from reconciliation.robotless_projection_handoff import projection_geometry
from reconciliation.robotless_single_chunk import _agent_pose, validate_waypoints
from reconciliation.se2 import compose_poses, inverse_pose, wrap_angle

OLD_METRICS = ('e_perp_old_m', 'abs_e_dir_old_deg', 'abs_e_dir_window_deg', 'abs_e_yaw_old_deg')
PAIRED_METRICS = ('delta_e_perp_m', 'delta_abs_e_dir_deg', 'delta_abs_e_yaw_deg')
STRAIGHT_METRICS = ('e_perp_straight_m', 'abs_e_dir_straight_deg', 'abs_e_yaw_straight_deg')
SUMMARY_METRICS = OLD_METRICS + PAIRED_METRICS + STRAIGHT_METRICS


def validate_config(config):
    if config.get('schema_version') != 1 or config.get('stage') != 'robotless-old-conditioned-handoff':
        raise ValueError('wrong OLD-conditioned configuration')
    if config.get('motion') != {k: MOTION[k] for k in ('v_mps', 'tau_s')} or config.get('tangent_window_m') != .10:
        raise ValueError('fixed controlled speed, delays and window required')
    for key in ('minimum_window_span_m', 'minimum_window_chord_m', 'anchor_atol'):
        value = config['numerical_validation'][key]
        if not np.isfinite(value) or value <= 0:
            raise ValueError('numerical tolerances must be finite and positive')


def arc_data(path):
    path = validate_waypoints(path)
    with np.errstate(over='ignore', invalid='ignore'):
        lengths = np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)
        # Prefix sums use the same reduction as the validated projection helper.
        progress = np.array([lengths[:i].sum() for i in range(len(path))])
    if not len(lengths) or not np.all(np.isfinite(progress)) or progress[-1] <= 0:
        raise ValueError('trajectory needs finite positive total XY arc length')
    return path, lengths, progress


def interpolate_arc(path, s):
    """Interpolate within the path; reject out-of-range progress, never clamp it."""
    path, lengths, progress = arc_data(path)
    if not np.isfinite(s) or s < 0 or s > progress[-1]:
        raise ValueError('arc progress outside available trajectory')
    candidates = np.flatnonzero((lengths > 0) & (progress[1:] >= s))
    j = int(candidates[0])
    alpha = 1. if s == progress[j+1] else float((s-progress[j])/lengths[j])
    xy = path[j, :2] + alpha*(path[j+1, :2]-path[j, :2])
    yaw = wrap_angle(path[j, 2] + alpha*wrap_angle(path[j+1, 2]-path[j, 2]))
    return {'pose': [*xy.tolist(), yaw], 'segment_index': j, 'alpha': alpha,
        'segment_length_m': float(lengths[j]), 's_m': float(s),
        'phi_local_rad': float(np.arctan2(*(path[j+1, :2]-path[j, :2])[::-1]))}


def window_tangent(path, s, window_m=.10, minimum_span_m=1e-12, minimum_chord_m=1e-12):
    _, _, progress = arc_data(path)
    if not all(np.isfinite(x) for x in (s, window_m, minimum_span_m, minimum_chord_m)) or not 0 <= s <= progress[-1]:
        raise ValueError('invalid window/progress')
    if min(window_m, minimum_span_m, minimum_chord_m) <= 0:
        raise ValueError('window and numerical resolutions must be positive')
    low, high = max(0., s-window_m/2), min(float(progress[-1]), s+window_m/2)
    p0, p1 = interpolate_arc(path, low)['pose'][:2], interpolate_arc(path, high)['pose'][:2]
    delta = np.subtract(p1, p0); chord = float(np.linalg.norm(delta))
    reason = 'arc span at or below numerical resolution' if high-low <= minimum_span_m else (
        'window chord at or below numerical resolution' if chord <= minimum_chord_m else None)
    return {'s_minus_m': low, 's_plus_m': high, 'span_m': high-low, 'chord_m': chord,
        'minus_xy': p0, 'plus_xy': p1, 'available': reason is None, 'reason': reason,
        'phi_rad': None if reason else float(np.arctan2(delta[1], delta[0]))}


def old_reference(r_obs, old):
    r_obs = _agent_pose(r_obs)
    projection = projection_geometry(r_obs, old)
    a_obs = np.array([*projection['Q_xy_world_m'], projection['theta_Q_rad']])
    align = compose_poses(r_obs, inverse_pose(a_obs))
    return {'R_obs': r_obs.tolist(), 'A_obs': a_obs.tolist(), 'T_align_pose': align.tolist(),
        'alignment_reconstructed_R_obs': compose_poses(align, a_obs).tolist(),
        'segment_index': projection['segment_index'], 'alpha_obs': projection['alpha'],
        'Q_old_obs_xy_world_m': projection['Q_xy_world_m'], 's_old_obs_m': projection['s_Q_m'],
        'normalized_old_progress': projection['normalized_progress'], 'd_obs_to_old_m': projection['e_perp_m'],
        'old_projection_segment_length_m': projection['segment_length_m'],
        'total_old_arc_length_m': projection['total_FRESH_arc_length_m'],
        'available_remaining_m': projection['total_FRESH_arc_length_m']-projection['s_Q_m'],
        'phi_old_at_obs_unaligned_rad': projection['phi_F_rad']}


def characterize_episode(episode, r_obs, old, fresh, previous, config):
    validate_config(config)
    old = validate_waypoints(old); fresh = validate_waypoints(fresh); r_obs = _agent_pose(r_obs)
    reference = old_reference(r_obs, old)
    if not np.allclose(reference['alignment_reconstructed_R_obs'], r_obs, rtol=0, atol=config['numerical_validation']['anchor_atol']):
        raise ValueError('aligned reference must reconstruct R_obs')
    if [r['tau_s'] for r in previous] != MOTION['tau_s']:
        raise ValueError('all four unchanged straight source rows required')
    align = reference['T_align_pose']; s_obs = reference['s_old_obs_m']
    # Future path is derived display geometry only; source OLD/FRESH are untouched.
    _, _, old_progress = arc_data(old)
    continuation = compose_poses(align, np.vstack([reference['A_obs'], old[old_progress > s_obs]]))
    continuation[0] = r_obs
    window_args = {'window_m': config['tangent_window_m'],
        'minimum_span_m': config['numerical_validation']['minimum_window_span_m'],
        'minimum_chord_m': config['numerical_validation']['minimum_window_chord_m']}
    rows = []
    for straight in previous:
        if straight['episode_id'] != episode['episode_id'] or straight['category'] != episode['category']:
            raise ValueError('straight row belongs to another episode')
        if ordered_pair_hash(straight['OLD_raw_sha256'], straight['FRESH_raw_sha256']) != straight['pair_sha256']:
            raise ValueError('source pair hash mismatch')
        tau = straight['tau_s']; delta = config['motion']['v_mps']*tau
        row = {'episode_id': episode['episode_id'], 'category': episode['category'], 'tau_s': tau,
            **{k: straight[k] for k in ('OLD_raw_sha256', 'FRESH_raw_sha256', 'pair_sha256')},
            'straight': straight, 'requested_delta_s_m': delta, 'available_remaining_m': reference['available_remaining_m'],
            'requested_s_B_m': s_obs+delta, 'e_perp_straight_m': straight['e_perp_m'],
            'abs_e_dir_straight_deg': straight['abs_e_dir_deg'], 'abs_e_yaw_straight_deg': straight['abs_e_yaw_deg'],
            **dict.fromkeys(OLD_METRICS+PAIRED_METRICS), 'B_old': None, 'old_arc_sample': None,
            'fresh_projection': None, 'old_window': None, 'fresh_window': None,
            'phi_old_local_rad': None, 'phi_old_window_rad': None,
            'phi_fresh_local_rad': None, 'phi_fresh_window_rad': None,
            'e_dir_old_rad': None, 'e_dir_window_rad': None, 'e_yaw_old_rad': None}
        if delta > reference['available_remaining_m']:
            row.update(status='OLD_CONTINUATION_EXHAUSTED', reason='requested advance exceeds available remaining OLD arc length',
                local_direction_unavailable_reason='OLD continuation exhausted', window_direction_unavailable_reason='OLD continuation exhausted')
            rows.append(row); continue
        if tau == 0:
            sample = {'pose': reference['A_obs'], 'segment_index': reference['segment_index'], 'alpha': reference['alpha_obs'],
                'segment_length_m': reference['old_projection_segment_length_m'], 's_m': s_obs,
                'phi_local_rad': reference['phi_old_at_obs_unaligned_rad']}
            boundary = r_obs.copy()
        else:
            sample = interpolate_arc(old, s_obs+delta)
            boundary = compose_poses(align, sample['pose'])
        projection = projection_geometry(boundary, fresh)
        # The reused helper's incoming=yaw(B) residual is kept explicitly as a yaw-axis reference;
        # actual OLD-conditioned incoming direction comes from aligned OLD geometry below.
        projection['incoming_reference_convention'] = 'helper phi_in=yaw(B); helper e_dir is not OLD-conditioned e_dir'
        phi_old = None if sample['phi_local_rad'] is None else wrap_angle(align[2]+sample['phi_local_rad'])
        phi_fresh = projection['phi_F_rad']
        ow = window_tangent(old, s_obs+delta, **window_args)
        fw = window_tangent(fresh, projection['s_Q_m'], **window_args)
        phi_ow = None if ow['phi_rad'] is None else wrap_angle(align[2]+ow['phi_rad'])
        phi_fw = fw['phi_rad']
        local = None if phi_old is None or phi_fresh is None else wrap_angle(phi_fresh-phi_old)
        window = None if phi_ow is None or phi_fw is None else wrap_angle(phi_fw-phi_ow)
        row.update(status='AVAILABLE', reason=None, B_old=boundary.tolist(), old_arc_sample=sample,
            fresh_projection=projection, old_window=ow, fresh_window=fw,
            phi_old_local_rad=phi_old, phi_old_window_rad=phi_ow,
            phi_fresh_local_rad=phi_fresh, phi_fresh_window_rad=phi_fw,
            e_perp_old_m=projection['e_perp_m'], e_dir_old_rad=local, e_dir_window_rad=window,
            abs_e_dir_old_deg=None if local is None else float(np.degrees(abs(local))),
            abs_e_dir_window_deg=None if window is None else float(np.degrees(abs(window))),
            e_yaw_old_rad=projection['e_yaw_rad'], abs_e_yaw_old_deg=projection['abs_e_yaw_deg'],
            local_direction_unavailable_reason=None if local is not None else 'OLD or FRESH winning segment is zero length',
            window_direction_unavailable_reason=None if window is not None else {'OLD': ow['reason'], 'FRESH': fw['reason']})
        for delta_key, old_key, straight_key in zip(PAIRED_METRICS, ('e_perp_old_m', 'abs_e_dir_old_deg', 'abs_e_yaw_old_deg'), STRAIGHT_METRICS, strict=True):
            row[delta_key] = None if row[old_key] is None or row[straight_key] is None else row[old_key]-row[straight_key]
        rows.append(row)
    return {'episode_id': episode['episode_id'], 'category': episode['category'], 'reference': reference,
        'aligned_continuation_world': continuation.tolist(), 'conditions': rows}


def summarize(episodes):
    ids = [e['episode_id'] for e in episodes]
    if len(ids) != len(set(ids)):
        raise ValueError('duplicate episode')
    rows = [r for e in episodes for r in e['conditions']]
    for e in episodes:
        if [r['tau_s'] for r in e['conditions']] != MOTION['tau_s'] or any(r['episode_id'] != e['episode_id'] for r in e['conditions']):
            raise ValueError('missing or mismatched condition')
        if len({r['pair_sha256'] for r in e['conditions']}) != 1:
            raise ValueError('pair group changes with tau')
    groups = sorted({r['pair_sha256'] for r in rows}); group_rows = []
    for pair in groups:
        for tau in MOTION['tau_s']:
            members = [r for r in rows if r['pair_sha256'] == pair and r['tau_s'] == tau]
            group_rows.append({'pair_sha256': pair, 'tau_s': tau, 'count': len(members),
                'episode_ids': sorted(r['episode_id'] for r in members),
                **{key: percentiles([r[key] for r in members])['median'] for key in SUMMARY_METRICS},
                'available_counts': {key: sum(r[key] is not None for r in members) for key in SUMMARY_METRICS}})
    def distributions(items):
        return [{'tau_s': tau, 'metrics': {key: percentiles([r[key] for r in items if r['tau_s'] == tau])
            for key in SUMMARY_METRICS}} for tau in MOTION['tau_s']]
    anchor_values = [e['reference']['d_obs_to_old_m'] for e in episodes]
    anchor_pair_medians = [float(np.median([e['reference']['d_obs_to_old_m'] for e in episodes if e['conditions'][0]['pair_sha256'] == pair])) for pair in groups]
    return {'episode_count': len(ids), 'unique_pair_count': len(groups), 'unique_pairs': group_rows,
        'availability_by_tau': [{'tau_s': tau, 'counts': dict(Counter(r['status'] for r in rows if r['tau_s'] == tau))} for tau in MOTION['tau_s']],
        'statistics': {'episode_weighted': distributions(rows), 'unique_pair_weighted': distributions(group_rows),
            'anchor_distance_episode_weighted': percentiles(anchor_values), 'anchor_distance_unique_pair_weighted': percentiles(anchor_pair_medians),
            'percentile_method': 'linear; min/median/p75/p90/max',
            'unique_pair_rule': 'equal weight per within-pair median, separately for each metric and paired difference',
            'unavailable_rule': 'null values excluded individually, available/unavailable counts retained'}}


def select_representatives(episodes):
    rows = [r for e in episodes for r in e['conditions'] if r['tau_s'] == 1.]
    result = [{'rule': rule, 'episode_id': episode_id, 'tau_s': 1.} for rule, episode_id in
        [('previous_max_e_perp', 'episode_006'), ('previous_max_abs_e_dir', 'episode_016')]]
    for rule, key, absolute in [('max_old_e_perp', 'e_perp_old_m', False),
        ('max_old_abs_e_dir', 'abs_e_dir_old_deg', False), ('largest_absolute_direction_change', 'delta_abs_e_dir_deg', True)]:
        available = [r for r in rows if r[key] is not None]
        chosen = min(available, key=lambda r: (-(abs(r[key]) if absolute else r[key]), r['episode_id'])) if available else None
        result.append({'rule': rule, 'episode_id': chosen['episode_id'] if chosen else None, 'tau_s': 1.,
            'metric': key, 'value': chosen[key] if chosen else None, 'selection_value': (abs(chosen[key]) if absolute else chosen[key]) if chosen else None,
            'tie_rule': 'lowest episode_id', 'reason': None if chosen else 'no available metric'})
    return result

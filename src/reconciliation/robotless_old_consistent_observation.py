"""OLD-consistent spatial observation pilot; no waypoint timing or execution."""
import hashlib
import io

import numpy as np

from reconciliation.robotless_old_conditioned_handoff import interpolate_arc, window_tangent
from reconciliation.robotless_projection_handoff import projection_geometry
from reconciliation.robotless_single_chunk import _agent_pose, validate_waypoints
from reconciliation.se2 import relative_pose, wrap_angle

CASE_IDS = ('episode_006', 'episode_007', 'episode_013', 'episode_016', 'episode_027')
METRICS = ('e_perp_m', 'abs_e_dir_local_deg', 'abs_e_dir_window_deg', 'abs_e_yaw_deg')


def validate_config(config):
    if config.get('stage') != 'robotless-old-consistent-observation' or config.get('schema_version') != 1:
        raise ValueError('wrong pilot configuration')
    if tuple(config.get('episode_ids', ())) != CASE_IDS:
        raise ValueError('the five predeclared cases must be retained in order')
    if config.get('delta_s_obs_m') != .30 or config.get('primary_tau_s') != 0 or config.get('tangent_window_m') != .10:
        raise ValueError('fixed 0.30 m observation advance, tau=0 and 0.10 m window required')


def plan_observation(r0, fixed_r1, old_world, delta_s=.30):
    r0 = _agent_pose(r0); fixed_r1 = _agent_pose(fixed_r1)
    old = validate_waypoints(old_world)
    if not np.isfinite(delta_s) or delta_s != .30:
        raise ValueError('observation progress must be exactly 0.30 m')
    path = np.vstack([r0, old])
    lengths = np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)
    total = float(lengths.sum())
    if not np.isfinite(total):
        raise ValueError('augmented OLD arc must be finite')
    result = {'R0': r0.tolist(), 'fixed_R1': fixed_r1.tolist(), 'augmented_path_world': path.tolist(),
        'connector_length_m': float(lengths[0]), 'model_old_arc_length_m': float(lengths[1:].sum()),
        'total_augmented_old_arc_length_m': total, 'delta_s_obs_m': delta_s,
        'target_progress_m': delta_s, 'available_arc_length_m': total,
        'remaining_arc_length_m': None, 'R1_old': None, 'sample': None, 'fixed_R1_to_R1_old': None,
        'convention': 'OLD-consistent spatial observation surrogate; R0->O0 is a simulation-side connector, not a model waypoint',
        'actual_execution_time': None, 'waypoint_time_base': None}
    if total < delta_s:
        result.update(status='OLD_ARC_INSUFFICIENT', reason='requested 0.30 m exceeds augmented OLD XY arc; no endpoint clamp')
        return result
    sample = interpolate_arc(path, delta_s)
    result.update(status='OLD_OBSERVATION_PLANNED', reason=None, sample=sample, R1_old=sample['pose'],
        remaining_arc_length_m=total-delta_s,
        fixed_R1_to_R1_old=relative_pose(fixed_r1, sample['pose']).tolist())
    return result


def raw_array_hash(array):
    """SHA-256 of deterministic np.save bytes; shape/dtype/value bits retained."""
    array = np.asarray(array)
    validate_waypoints(array)
    stream = io.BytesIO()
    np.save(stream, array, allow_pickle=False)
    return hashlib.sha256(stream.getvalue()).hexdigest()


def check_old_reproduction(source, final):
    source = np.asarray(source); final = np.asarray(final)
    source_hash, final_hash = raw_array_hash(source), raw_array_hash(final)
    checks = {'shape_equal': source.shape == final.shape, 'dtype_equal': source.dtype == final.dtype,
        'exact_values_equal': bool(np.array_equal(source, final)), 'raw_array_hash_equal': source_hash == final_hash}
    return {**checks, 'source_shape': list(source.shape), 'final_shape': list(final.shape),
        'source_dtype': str(source.dtype), 'final_dtype': str(final.dtype),
        'source_OLD_hash': source_hash, 'final_OLD_hash': final_hash,
        'status': 'PLANNING_OLD_REPRODUCED' if all(checks.values()) else 'PLANNING_OLD_MISMATCH'}


def immediate_metrics(plan, fresh_world, previous_tau0, *, tau_s=0., window_m=.10):
    if tau_s != 0 or not np.isfinite(tau_s) or previous_tau0.get('tau_s') != 0 or window_m != .10:
        raise ValueError('primary and paired previous metrics must use tau=0; window is 0.10 m')
    if plan['status'] != 'OLD_OBSERVATION_PLANNED':
        raise ValueError('OLD observation is unavailable')
    b = _agent_pose(plan['R1_old'])
    p = projection_geometry(b, fresh_world)
    p['incoming_reference_convention'] = 'helper incoming is yaw(B); pilot incoming is augmented OLD geometric tangent'
    ow = window_tangent(plan['augmented_path_world'], plan['target_progress_m'], window_m)
    fw = window_tangent(fresh_world, p['s_Q_m'], window_m)
    old_phi, fresh_phi = plan['sample']['phi_local_rad'], p['phi_F_rad']
    local = None if old_phi is None or fresh_phi is None else wrap_angle(fresh_phi-old_phi)
    window = None if ow['phi_rad'] is None or fw['phi_rad'] is None else wrap_angle(fw['phi_rad']-ow['phi_rad'])
    row = {'primary_tau_s': 0., 'B_world': b.tolist(), 'fresh_projection': p, 'old_window': ow, 'fresh_window': fw,
        'phi_old_local_rad': old_phi, 'phi_old_window_rad': ow['phi_rad'],
        'phi_fresh_local_rad': fresh_phi, 'phi_fresh_window_rad': fw['phi_rad'],
        'e_perp_m': p['e_perp_m'], 'e_dir_local_rad': local, 'e_dir_window_rad': window, 'e_yaw_rad': p['e_yaw_rad'],
        'abs_e_dir_local_deg': None if local is None else float(np.degrees(abs(local))),
        'abs_e_dir_window_deg': None if window is None else float(np.degrees(abs(window))), 'abs_e_yaw_deg': p['abs_e_yaw_deg'],
        'local_unavailable_reason': None if local is not None else 'zero-length FRESH winning segment',
        'window_unavailable_reason': None if window is not None else {'OLD': ow['reason'], 'FRESH': fw['reason']},
        'previous_tau0': previous_tau0,
        'comparison_direction_convention': 'previous saved incoming=yaw(fixed R1), new incoming=augmented OLD tangent; protocol and incoming reference both change'}
    for key, oldkey in [('e_perp_m', 'e_perp_m'), ('abs_e_dir_local_deg', 'abs_e_dir_deg'), ('abs_e_yaw_deg', 'abs_e_yaw_deg')]:
        row['delta_'+key] = None if row[key] is None or previous_tau0[oldkey] is None else row[key]-previous_tau0[oldkey]
    return row

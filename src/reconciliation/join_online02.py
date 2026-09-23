"""Frozen native online approach diagnostics; no model or optimizer construction."""
from copy import copy
import math
import numpy as np

from .robotless_online import integrate_unicycle
from .se2 import wrap_angle
from .join_source02_acquisition import held_interval_check

ORDER = ['OFF_REPEAT_00', 'ON_REPEAT_00', 'OFF_REPEAT_01', 'ON_REPEAT_01']
INSTRUCTION = 'Avoid the supply cart and continue to the end of the hallway.'
DISTANCES = [5., 4.5, 4.]
LABEL = 'RECORDED ONLINE FAR-APPROACH REPLAY'
ABORT = 'SAFETY_ABORT_BEFORE_UNSAFE_COMMAND'


def preview_activation(activation, state, previous_state, previous_command):
    """An unapplied unsafe command must not create B or change active identity."""
    proposal = copy(activation)
    proposal.activations = list(activation.activations)
    command, event = proposal.apply(state, previous_state, previous_command)
    return proposal, command, event


def select_start(base, on, center, forward, cart):
    center, forward = np.asarray(center), np.asarray(forward)
    if not np.isclose(np.linalg.norm(forward), 1., atol=1e-12):
        raise ValueError('unit hallway direction required')
    from shapely.geometry import Point
    rows = []
    for distance in DISTANCES:
        pose = np.r_[center - distance * forward, math.atan2(forward[1], forward[0])]
        start = on.check_polyline([pose])
        approach = base.check_polyline([pose[:2], center])
        rows.append(dict(distance_m=distance, pose_world=pose.tolist(),
            start=start, off_approach=approach,
            cart_edge_clearance_m=float(cart.distance(Point(pose[:2]))-.20),
            valid=start['clearance_valid'] and approach['clearance_valid']))
    selected = next((x for x in rows if x['valid']), None)
    return dict(candidates=rows, selected=selected,
                status='GEOMETRY_START_BLOCKER' if selected is None else 'START_FROZEN')


def guard_check(environment, pose, command, integration_dt_s):
    """Inspect only the proposed held command, never the supplied FRESH path.

    New solves: one nominal .1s command interval. Already held commands: next
    60Hz step, so late/asynchronous results cannot extend unchecked execution.
    Exact unicycle endpoints and existing sagitta bound cover each subinterval.
    The guard returns a decision and never synthesizes or clips a command.
    """
    duration = .1 if command['reason'] == 'new_solve' else float(integration_dt_s)
    times = np.linspace(0., duration, max(1, math.ceil(duration/integration_dt_s))+1)
    u = [command['v_mps'], command['omega_radps']]
    poses = np.asarray([pose if t == 0 else integrate_unicycle(pose, u, float(t)) for t in times])
    allowances = [held_interval_check(environment,a,b)['curve_allowance_m'] for a,b in zip(poses[:-1],poses[1:])]
    result = environment.check_trajectory(times, poses, radius=.20, required_clearance=.05,
        curved_path_error_bound_m=max(allowances,default=0.))
    return dict(safe=bool(result['clearance_valid']), command=dict(command), duration_s=duration,
        start_pose=np.asarray(pose).tolist(), times_s=times.tolist(), poses=poses.tolist(), check=result,
        command_modified=False, raw_reference_consulted=False,
        interval_basis='nominal next control hold' if command['reason']=='new_solve' else 'next actual integration step')


def hallway_geometry(world, observation, center, forward, cart_extents, config):
    p = np.asarray(world, float)
    if p.ndim != 2 or p.shape[1] != 3 or not len(p) or not np.isfinite(p).all():
        raise ValueError('finite generic N x 3 path required')
    obs, center, f = np.asarray(observation), np.asarray(center), np.asarray(forward)
    left = np.array([-f[1], f[0]])
    longitudinal = (p[:,:2]-center)@f
    lateral = (p[:,:2]-center)@left
    obs_long = float((obs[:2]-center)@f)
    obs_lat = float((obs[:2]-center)@left)
    delta = np.diff(p[:,:2], axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    arc = float(lengths.sum())
    reliable = [i for i in range(1,len(p)) if np.linalg.norm(p[i,:2]-p[0,:2]) >= config['tangent_chord_m']]
    tangent = None
    if reliable:
        d = p[reliable[0],:2]-p[0,:2]
        tangent = float(math.atan2(d@left,d@f))
    dy = float(lateral[-1]-lateral[0])
    forward_progress = float(longitudinal[-1]-longitudinal[0])
    side = 1 if dy>0 else -1 if dy<0 else 0
    meaningful = abs(dy)>=config['lateral_m'] or (
        tangent is not None and abs(tangent)>=math.radians(config['tangent_deg']) and
        abs(dy)>=config['directional_lateral_minimum_m'])
    front,rear = cart_extents
    return dict(row_count=len(p), raw_arc_m=arc, max_lateral_m=float(np.max(np.abs(lateral))),
        endpoint_lateral_m=float(lateral[-1]), returned_lateral_change_m=dy, observation_lateral_m=obs_lat,
        reliable_initial_tangent_rad=tangent, final_yaw_world_rad=float(p[-1,2]),
        final_yaw_relative_hallway_rad=float(wrap_angle(p[-1,2]-math.atan2(f[1],f[0]))),
        accumulated_yaw_rad=float(np.abs(wrap_angle(np.diff(p[:,2]))).sum()),
        observation_longitudinal_m=obs_long, observation_cart_center_distance_m=float(np.linalg.norm(obs[:2]-center)),
        endpoint_longitudinal_m=float(longitudinal[-1]), endpoint_from_front_m=float(longitudinal[-1]-front),
        endpoint_from_rear_m=float(longitudinal[-1]-rear), returned_forward_progress_m=forward_progress,
        endpoint_forward_from_observation_m=float(longitudinal[-1]-obs_long),
        meaningful_direction=bool(meaningful), side=side,
        raw_reaches_influence=bool(longitudinal.max()>=front-config['obstacle_influence_before_front_m']))


def classify_chunk(geometry, safe, stop, free_sides, cart_extents, config):
    """Descriptive geometry only. OFF classifications are hypothetical cart tests."""
    g=geometry
    if stop:
        return 'SAFE_STOP' if safe else 'UNSAFE_INTERSECTING'
    if g['observation_longitudinal_m']>=cart_extents[1]+config['rear_margin_m']:
        return 'PAST_OBSTACLE' if safe else 'UNSAFE_INTERSECTING'
    if not safe:return 'UNSAFE_INTERSECTING'
    onset=(g['meaningful_direction'] and g['returned_forward_progress_m']>0 and
           g['side'] in free_sides)
    if onset and g['endpoint_from_rear_m']>=config['rear_margin_m']:return 'SAFE_BYPASS'
    if onset:return 'SAFE_BYPASS_ONSET'
    if g['endpoint_from_front_m']<0 and (g['raw_reaches_influence'] or g['raw_arc_m']<=config['short_arc_m']):
        return 'SAFE_SHORTEN'
    return 'STRAIGHT_OR_BASELINE'


def episode_outcome(rows, termination):
    # Only arrived-before-termination chunks count as prospective source evidence.
    rows=[r for r in rows if r.get('received_before_end',False)]
    labels=[r['classification'] for r in rows]
    if 'SAFE_BYPASS' in labels:return 'SAFE_BYPASS_BEFORE_ABORT'
    if 'SAFE_BYPASS_ONSET' in labels:return 'BYPASS_ONSET_BUT_NO_COMPLETE_BYPASS'
    if termination=='MODEL_STOP':return 'NATURAL_MODEL_STOP_BEFORE_BYPASS'
    if termination==ABORT:return 'UNSAFE_NO_BYPASS_BEFORE_GUARD_ABORT'
    if termination not in ('EPISODE_LIMIT','ATTEMPT_LIMIT'):return 'TECHNICAL_BLOCKER'
    if labels.count('SAFE_SHORTEN')>=2:return 'PROGRESSIVE_SHORTENING_OR_STOP'
    return 'EPISODE_LIMIT_BEFORE_CONCLUSION'


def candidate_source(rows):
    for ep in ['ON_REPEAT_00','ON_REPEAT_01']:
        for label in ['SAFE_BYPASS','SAFE_BYPASS_ONSET']:
            match=[r for r in rows if r['episode']==ep and r['classification']==label and
                r.get('received_before_end',False)]
            if match:return min(match,key=lambda r:r['t_obs'])
    return None

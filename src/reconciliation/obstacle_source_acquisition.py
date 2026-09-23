"""Frozen source qualification only; no controller/model/optimizer imports."""
from copy import deepcopy

import numpy as np

from .join_source02 import observation_anchored_world, response_geometry, whole_raw_polyline_check
from .se2 import wrap_angle


def select_side(passages):
    valid = [p for p in passages if p['check']['clearance_valid']]
    if not valid:
        return None
    p = min(valid, key=lambda p: (-p['check']['minimum_clearance_m'], p['side']))
    return 'RIGHT' if p['side'] == -1 else 'LEFT'


def paired_frames(bank, index, h=8):
    if not 1 <= h <= index + 1 or index >= len(bank):
        raise ValueError('insufficient chronological history')
    selected = bank[index-h+1:index+1]
    result = {}
    for branch in ('OFF', 'ON'):
        frames = [deepcopy(r['OFF']['frame']) for r in selected[:-1]]
        frames.append(deepcopy(selected[-1][branch]['frame']))
        if len({f['frame_id'] for f in frames}) != h or len({f['path'] for f in frames}) != h:
            raise ValueError('duplicated frame padding')
        if any(b['capture_monotonic_ns'] <= a['capture_monotonic_ns'] for a,b in zip(frames, frames[1:])):
            raise ValueError('nonchronological history')
        result[branch] = frames
    a,b = result['OFF'][-1], result['ON'][-1]
    if any(a[k] != b[k] for k in ('pose_world', 'camera', 'frame_id', 'capture_sim_time_s')):
        raise ValueError('terminal pair camera/pose/time mismatch')
    return result


def future_at_pose(world, pose):
    """Official weighted-pose nearest is an external diagnostic, not correspondence research."""
    a = np.asarray(world, float)
    error = a - np.asarray(pose, float)
    error[:,2] = wrap_angle(error[:,2])
    j = int(np.argmin(np.sum(error**2 * np.array([10.,10.,1.]), axis=1)))
    return dict(nearest_index=int(j), remaining_rows=int(len(a)-j),
                remaining_arc_m=float(np.linalg.norm(np.diff(a[j:,:2], axis=0), axis=1).sum()))


def geometry(local, pose, base, on, scenario):
    a = np.asarray(local, float)
    w = observation_anchored_world(a, pose)
    f = np.asarray(scenario['forward_xy']); c = np.asarray(scenario['center_xy'])
    delta = np.diff(a[:,:2], axis=0)
    reliable = np.linalg.norm(delta, axis=1) >= .02
    angles = np.abs(np.arctan2(delta[reliable,1], delta[reliable,0]))
    endpoint = float((w[-1,:2]-c)@f)
    return dict(raw_local=a.tolist(), world=w.tolist(), N=len(a),
        geometry_off=whole_raw_polyline_check(w, base), geometry_on=whole_raw_polyline_check(w, on),
        arc_m=float(np.linalg.norm(delta,axis=1).sum()), max_lateral_m=float(np.abs(a[:,1]).max()),
        max_yaw_deg=float(np.degrees(np.abs(wrap_angle(a[:,2])).max())),
        max_reliable_tangent_deg=None if not len(angles) else float(np.degrees(angles.max())),
        forward_progress_m=float((w[-1,:2]-w[0,:2])@f),
        endpoint_forward_from_observation_m=float((w[-1,:2]-np.asarray(pose)[:2])@f),
        endpoint_from_front_m=endpoint-scenario['cart_extents'][0],
        complete_bypass_descriptor=endpoint>=scenario['cart_extents'][1]+.25,
        future_proxy=future_at_pose(w, pose), future_proxy_is_actual_B=False,
        observation_anchor=list(pose), intrinsic_waypoint_dt=None)


def qualify(off, on, scenario, gates):
    influence = dict(origin_xy=scenario['center_xy'],forward_xy=scenario['forward_xy'],
        progress_min_m=scenario['cart_extents'][0]-gates['interaction_before_front_m'],
        progress_max_m=scenario['cart_extents'][1]+.5)
    diff = response_geometry(off['world'],on['world'],influence,
        minimum_separation_m=gates['lateral_m'],minimum_angle_deg=gates['angle_deg'],
        minimum_chord_m=gates['reliable_chord_m'])
    straight = (off['max_lateral_m']<gates['lateral_m'] and off['max_yaw_deg']<gates['angle_deg'] and
        (off['max_reliable_tangent_deg'] is not None and off['max_reliable_tangent_deg']<gates['angle_deg']))
    q=on['future_proxy']
    flags=dict(A_OFF=bool(not off['stop'] and off['geometry_off']['clearance_valid'] and straight and off['forward_progress_m']>0),
        B_OBSTACLE_RELEVANT=not off['geometry_on']['clearance_valid'],
        C_ON_WHOLE_SAFE=on['geometry_on']['clearance_valid'],
        D_MEANINGFUL_CHANGE=diff['meaningful'],
        E_TASK_PROGRESS=bool(not on['stop'] and on['forward_progress_m']>0 and on['arc_m']>=gates['remaining_arc_m'] and
            on['endpoint_from_front_m']>=-gates['interaction_before_front_m']),
        F_FUTURE_PROXY=bool(q['remaining_rows']>=gates['remaining_rows'] and q['remaining_arc_m']>=gates['remaining_arc_m']))
    return dict(qualified=all(flags.values()), gates=flags, failure_reasons=[k for k,v in flags.items() if not v],
        difference=diff, off_roughly_straight=bool(straight), complete_bypass_required=False,
        actual_B_qualification_pending=True)


def timing_gate(rtfs, max_stall, minimum_requests=3):
    return dict(qualified=bool(len(rtfs)>=minimum_requests and all(x is not None and .8<=x<=1.2 for x in rtfs)
        and max_stall is not None and max_stall<=.25), request_rtfs=rtfs, max_loop_stall_s=max_stall,
        minimum_requests=minimum_requests, thresholds=dict(rtf=[.8,1.2],stall_s=.25))


def moving_boundary(u_minus, memory, travel, clearance):
    return dict(valid=bool(u_minus[0]>.20 and travel>=.02 and clearance>=.05),
        u_minus=list(u_minus), controller_previous_control=list(memory),
        memory_equals_physical=bool(np.array_equal(u_minus,memory)), travel_m=travel, clearance_m=clearance)

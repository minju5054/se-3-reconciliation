"""Acquisition-only geometry and chronological history rules for JOIN-SOURCE-02.

No controller, model, optimizer or simulator is constructed here. An oracle
abort prevents a recorded held command's next interval; it never steers.
"""
from __future__ import annotations

import numpy as np
from .gp_se2_environment import HospitalEnvironment


class ObstacleState:
    """One activation version shared by renderer and independent geometry."""
    def __init__(self, base, primitive, render_setter):
        self.base = base
        self.on = HospitalEnvironment(base.obstacles.union(primitive), base.workspace,
            parts=[*base.parts, primitive], metadata=base.metadata.copy())
        self.on.gates = list(base.gates)
        self.render_setter = render_setter
        self.present = False
        self.version = 0
        self.events = []
        render_setter(False)

    @property
    def environment(self):
        return self.on if self.present else self.base

    def set(self, present, timestamp):
        if type(present) is not bool:
            raise ValueError('obstacle presence must be boolean')
        self.render_setter(present)  # Failed render mutation cannot activate oracle.
        self.present = present
        self.version += 1
        record = dict(present=present, render_visible=present, evaluator_occupied=present,
                      version=self.version, timestamp=dict(timestamp))
        self.events.append(record)
        return record


def held_interval_check(environment, start, end):
    """Swept chord plus exact circular-arc sagitta allowance, one 60Hz step.

    Poses are endpoints of the existing exact constant-command integrator.
    No endpoint-only safe-path assertion, no wheel-dynamics claim.
    """
    start, end = np.asarray(start, float), np.asarray(end, float)
    theta = abs(float(np.arctan2(np.sin(end[2]-start[2]), np.cos(end[2]-start[2]))))
    chord = float(np.linalg.norm(end[:2]-start[:2]))
    if theta >= np.pi/2:
        raise ValueError('held interval rotation exceeds declared local bound')
    allowance = chord * np.tan(theta/4)/2 if theta else 0.
    result = environment.check_trajectory([0., 1/60], [start, end], radius=.2,
        required_clearance=.05, curved_path_error_bound_m=allowance)
    result.update(curve_allowance_m=allowance, nominal_required_clearance_m=.05,
                  acquisition_abort_guard=True, avoidance_command_generated=False)
    return result


def history_suffix(frames, count):
    """Use true captured suffix, including terminal frame; no count padding."""
    if count not in (16, 32, 64):
        raise ValueError('undeclared delivered history count')
    if len(frames) < count:
        return dict(status='HISTORY_UNAVAILABLE', available=len(frames), required=count)
    chosen = frames[-count:]
    if len({f['frame_id'] for f in chosen}) != count:
        raise ValueError('duplicate frame identities')
    for left, right in zip(chosen, chosen[1:]):
        if right['capture_monotonic_ns'] <= left['capture_monotonic_ns']:
            raise ValueError('history is not chronological')
        if right['capture_sim_time_s'] <= left['capture_sim_time_s']:
            raise ValueError('history reuses a simulation frame')
    return dict(status='AVAILABLE', history=chosen[:-1], terminal=chosen[-1],
                count=count, model_temporal_index_restarts_at_zero=True,
                pure_history_length_causal_claim=False)


def placement_on_future(pose, old_world, *, fraction, half_forward_m,
                        design_latency_s=.6, design_speed_mps=.8):
    """Geometry-only placement inside available OLD future, never extrapolated.

    Closest original segment progress to current pose defines remaining arc.
    Two predeclared fractions choose the interior of the usable arc interval.
    The reserve is a design estimate, not a worst-case latency guarantee.
    """
    path = np.asarray(old_world, float)
    pose = np.asarray(pose, float)
    if fraction not in (.35, .65) or len(path) < 2:
        raise ValueError('undeclared placement or insufficient OLD')
    delta = np.diff(path[:, :2], axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    choices = []
    cumulative = np.r_[0., np.cumsum(lengths)]
    for i, (a, d, length) in enumerate(zip(path[:-1,:2], delta, lengths)):
        if length <= 1e-8:
            continue
        alpha = float(np.clip((pose[:2]-a)@d/(length*length), 0., 1.))
        point = a+alpha*d
        choices.append((float(np.linalg.norm(pose[:2]-point)), i, alpha,
                        float(cumulative[i]+alpha*length)))
    if not choices:
        return dict(status='PLACEMENT_UNAVAILABLE', reason='degenerate OLD')
    _, j, alpha, current_arc = min(choices)
    connector_added = j == 0 and alpha == 0 and float((pose[:2]-path[0,:2])@delta[0]) < 0
    if connector_added:
        # Placement-only observation-to-first-row connector, never counted as
        # model-returned geometry in source qualification.
        path = np.vstack([pose,path])
        delta = np.diff(path[:,:2],axis=0)
        lengths = np.linalg.norm(delta,axis=1)
        cumulative = np.r_[0.,np.cumsum(lengths)]
        current_arc = 0.
    lower = current_arc + design_speed_mps*design_latency_s + .25 + half_forward_m
    upper = cumulative[-1] - half_forward_m - .25
    if upper <= lower:
        return dict(status='PLACEMENT_UNAVAILABLE', reason='insufficient OLD horizon for latency reserve and beyond-plane future',
                    current_arc_m=current_arc, minimum_arc_m=lower, maximum_arc_m=float(upper))
    target_arc = lower+fraction*(upper-lower)
    i = min(int(np.searchsorted(cumulative, target_arc, side='right')-1), len(lengths)-1)
    u = (target_arc-cumulative[i])/lengths[i]
    point = path[i,:2]+u*delta[i]
    yaw = float(np.arctan2(delta[i,1],delta[i,0]))
    return dict(status='AVAILABLE', center_xy=point.tolist(), forward_yaw_rad=yaw,
                original_segment=i, alpha=float(u), current_projection_segment=j,
                current_projection_alpha=alpha, current_arc_m=current_arc,
                target_arc_m=float(target_arc), remaining_agent_to_center_arc_m=float(target_arc-current_arc),
                fraction=fraction, placement_only_connector_added=connector_added,
                connector_is_model_returned=False, design_latency_s=design_latency_s,
                design_speed_mps=design_speed_mps, guarantee=False)

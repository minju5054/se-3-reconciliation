"""Timing-controlled offline causal reference comparison. No controller changes."""
from copy import deepcopy
from pathlib import Path
import time
import numpy as np
from .join_source03 import read, sha
from .osa03_common_b import schedule, distortion
from .se2 import relative_pose, wrap_angle
from .gp_se2_join01 import forward_projection

ORDER = ['M0_NATIVE', 'M1_TAPER', 'FULL_LOCAL_SE2', 'NO_RELATIVE']
LABEL = 'timing-controlled offline causal reference comparison'


def load_schedule(rollout_path, expected_hash, tracked_path, tracked_hash):
    """Authenticate frozen M0, extracting actual events, never synthesizing ticks."""
    if sha(rollout_path) != expected_hash or sha(tracked_path) != tracked_hash:
        raise ValueError('frozen M0 schedule hash mismatch')
    r = read(rollout_path)
    assert r['method'] == 'M0_NATIVE' and r['termination'] == 'OBSERVATION_CAP'
    tracked = read(tracked_path)['summary']['timing']['schedules']['M0_NATIVE']
    assert schedule(r) == tracked
    full = schedule(r, len(r['commands']))
    assert full['attempted_submit_ticks'] == full['accepted_submit_ticks']
    pairs = full['applications']
    assert [p['submit_tick'] for p in pairs] == full['accepted_submit_ticks']
    # A release precedes the next submit, so official poll memory cannot race it.
    for i, pair in enumerate(pairs):
        assert pair['submit_tick'] < pair['application_tick']
        if i+1 < len(pairs):
            assert pair['application_tick'] < pairs[i+1]['submit_tick']
    return dict(primary=tracked, full=full, pairs=pairs,
                B_tick=r['phase']['B_tick'], integration_steps=len(r['commands']),
                integration_dt_s=r['phase']['integration_dt_s'],
                provenance=dict(rollout_path=str(rollout_path), rollout_sha256=expected_hash,
                                tracked_path=str(tracked_path), tracked_sha256=tracked_hash))


class LogicalRelease:
    """One outstanding official result; host wait has no simulation-clock access."""
    def __init__(self, pairs, generation):
        self.by_submit = {p['submit_tick']: p['application_tick'] for p in pairs}
        self.generation = generation
        self.pending = None

    def collect(self, worker, tick, solve_id, pose, sim, events, timeout=30.):
        if self.pending is not None or tick not in self.by_submit:
            raise RuntimeError('invalid logical submit or unreleased result')
        before = np.array(pose, copy=True)
        started = time.monotonic()
        worker.send('submit', solve_id=solve_id, pose=before.tolist(), input_state_id=tick,
                    input_sim_time_s=sim, input_host_monotonic_s=started)
        reply = result = None
        while reply is None or result is None:
            for event in worker.drain():
                if event.get('status') in ['error', 'controller_error', 'busy', 'stale_rejected'] or event.get('type') in ['fatal', 'worker_exit']:
                    events.append(event)
                    raise RuntimeError(f'official controller rejected solve: {event}')
                if event.get('status') == 'submitted':
                    assert reply is None and event['solve_id'] == solve_id
                    reply = event
                    events.append(event)
                elif event.get('type') == 'solve_result':
                    assert result is None and event['solve_id'] == solve_id
                    assert event['status'] == 'command'
                    if event['official_generation'] != self.generation or event['result_generation'] != self.generation:
                        events.append(event)
                        raise RuntimeError('generation mismatch; official result withheld')
                    assert event['input_state_id'] == tick
                    np.testing.assert_array_equal(event['input_pose'], before)
                    result = event
                else:
                    events.append(event)
            if time.monotonic()-started > timeout:
                raise TimeoutError('wall-time controller wait; logical clock remained paused')
            if reply is None or result is None:
                time.sleep(.001)
        np.testing.assert_array_equal(pose, before)
        self.pending = (self.by_submit[tick], deepcopy(result))
        return dict(submit_tick=tick, release_tick=self.by_submit[tick], sim_before=sim,
                    sim_after=sim, pose_before=before.tolist(), pose_after=np.asarray(pose).tolist(),
                    wall_wait_s=time.monotonic()-started, solve_id=solve_id)

    def release(self, tick):
        if self.pending is None:
            return None
        due, event = self.pending
        if tick < due:
            return None
        if tick != due:
            raise RuntimeError('missed predetermined logical application tick')
        self.pending = None
        return event


def comparability(rollouts, frozen, common):
    checks = {}
    for name in ORDER:
        r = rollouts.get(name)
        if r is None:
            checks[name] = dict(available=False, passed=False)
            continue
        actual = schedule(r)
        initial = r['phase'] == common and r['states'][0]['pose_world'] == common['B']
        steps = min(len(r['commands']), 54)
        checks[name] = dict(available=True, schedule=actual, primary_integration_steps=steps,
                            identical_initial_provenance=initial,
                            passed=actual == frozen['primary'] and steps == 54 and initial)
    eligible = [c for c in checks.values() if c['available']]
    return dict(comparable=bool(eligible) and all(c['passed'] for c in eligible),
                all_four_observed=len(eligible) == 4, methods=checks,
                rule='authenticated M0 attempted/accepted/application ticks, 54 steps and exact common provenance')


def planning(fresh, world, B):
    from shapely.geometry import LineString
    d = distortion(fresh, world)
    seg = np.linalg.norm(np.diff(world[:, :2], axis=0), axis=1)
    local = relative_pose(B, world)
    correction = relative_pose(fresh, world)
    reliable = np.flatnonzero(np.linalg.norm(world[1:, :2]-world[0, :2], axis=1) >= .02)
    chord = None if not len(reliable) else world[reliable[0]+1, :2]-world[0, :2]
    projection = forward_projection(world, [B])[0]
    d.update(total_XY_arc_m=float(seg.sum()), segment_length_min_m=float(seg.min()),
             segment_length_max_m=float(seg.max()),
             maximum_lateral_node_correction_m=float(abs(correction[:, 1]).max()),
             maximum_lateral_B_frame_m=float(abs(local[:, 1]).max()),
             maximum_yaw_correction_rad=float(abs(wrap_angle(world[:, 2]-fresh[:, 2])).max()),
             self_intersection=not LineString(world[:, :2]).is_simple,
             B_to_first_node_position_m=float(np.linalg.norm(local[0, :2])),
             B_to_first_node_yaw_rad=float(abs(local[0, 2])), B_projection=projection,
             first_reliable_chord_relative_B_rad=None if chord is None else float(wrap_angle(np.arctan2(chord[1], chord[0])-B[2])),
             reliable_chord_minimum_length_m=.02)
    return d


def metric_row(m, termination):
    if m is None:
        return None
    tr=m['trace'];full=m['full'];j=None if full['join_time_s'] is None else int(np.searchsorted(tr['times_s'],full['join_time_s']))
    return dict(initial_position_error_m=tr['distance_m'][0],initial_yaw_error_rad=tr['yaw_error_rad'][0],
        max_position_error_05_m=m['extrema']['max_distance_05_m'],initial_separation_growth_m=m['extrema']['initial_growth_05_m'],
        position_auc_03_m_s=m['windows']['18'].get('position_auc_m_s'),position_auc_09_m_s=m['windows']['54'].get('position_auc_m_s'),
        yaw_auc_03_rad_s=m['windows']['18'].get('yaw_auc_rad_s'),yaw_auc_09_rad_s=m['windows']['54'].get('yaw_auc_rad_s'),
        sustained_attachment_s=full['join_time_s'],
        attachment_fractional_row=None if j is None else tr['projection'][j]['progress'],
        attachment_fractional_arc=None if j is None else tr['arc_progress_m'][j]/(tr['arc_progress_m'][j]+tr['remaining_arc_m'][j]),
        remaining_arc_at_attachment_m=None if j is None else tr['remaining_arc_m'][j],
        execution_clearance_lower_bound_m=m['clearance']['execution']['minimum_clearance_lower_bound_m'],
        endpoint_error_m=m['endpoint']['terminal_position_error_m'],linear_command_TV=m['command']['linear_TV_mps'],
        angular_command_TV=m['command']['angular_TV_radps'],max_abs_omega_radps=m['command']['max_abs_omega_radps'],
        termination_reason=termination)


def gaps(rows, baseline):
    b=rows[baseline]
    return {n:None if r is None or b is None else {k:None if r[k] is None or b[k] is None else r[k]-b[k]
            for k in b if k != 'termination_reason'} for n,r in rows.items()}

"""Fixed C3 suffix selection and saved execution metrics; no optimization."""
from contextlib import contextmanager
import sys
import numpy as np

from .spatial_correspondence_selector import SpatialCurve, SOURCE_IDS
from .se2 import relative_pose, local_trajectory_to_world, wrap_angle
from .gp_se2_join01 import POSITION_M, YAW_RAD, DWELL_S
from .osa03_relative_ablation import metric_row
from .osa03_common_b import schedule

NATIVE = 'M0_NATIVE'
FULL = 'FULL_LOCAL_SE2'
ENTRY = 'C3_ENTRY_SUFFIX'
ORDER = [NATIVE, FULL, ENTRY]
PNGS = ['world_execution_overview.png', 'transition_and_endpoint_metrics.png',
        'progress_preservation.png', 'attachment_vs_completion.png']
DUPLICATE_ATOL = 1e-12
SIGN_ATOL = 1e-9
MORE_REMAINING_M = .10


def suffix_reference(fresh, raw, A, B, frozen_c3=None):
    """Keep downstream world AND original observation-local rows byte-identical."""
    fresh, raw = np.asarray(fresh), np.asarray(raw)
    if fresh.shape != raw.shape:
        raise ValueError('original local/world identity mismatch')
    curve = SpatialCurve(fresh)
    c3 = curve.correspondences(B)['C3_FORWARD_SE2']
    if frozen_c3 is not None:
        assert c3 == {k:v for k,v in frozen_c3.items() if k != 'safety'}, 'C3 does not exactly reproduce frozen diagnostic'
    k, beta = c3['segment'], c3['alpha']
    entry = np.asarray(c3['target_world'])
    if c3['remaining_arc_m'] <= 0:
        raise ValueError('no positive remaining original arc')
    same_next = (np.linalg.norm(entry[:2]-fresh[k+1, :2]) <= DUPLICATE_ATOL
                 and abs(wrap_angle(entry[2]-fresh[k+1, 2])) <= DUPLICATE_ATOL)
    if beta == 0.:
        start = k
        world, local = fresh[start:].copy(), raw[start:].copy()
        identities = list(range(start, len(fresh)))
        arcs = curve.arc[start:].copy()
        dedup = 'exact start vertex: preserve one original row'
    elif same_next:
        start = k+1
        world, local = fresh[start:].copy(), raw[start:].copy()
        identities = list(range(start, len(fresh)))
        arcs = curve.arc[start:].copy()
        dedup = 'entry equals next vertex within 1e-12: preserve one original row'
    else:
        start = k+1
        world = np.vstack([entry, fresh[start:]])
        local = np.vstack([relative_pose(A, entry), raw[start:]])
        identities = [None, *range(start, len(fresh))]
        arcs = np.r_[c3['arc_m'], curve.arc[start:]]
        dedup = 'continuous entry followed by unchanged original rows'
    if len(world) < 2:
        raise ValueError('entry plus at least one downstream pose required')
    restored = local_trajectory_to_world(A, local)
    np.testing.assert_allclose(restored[:, :2], world[:, :2], atol=1e-12, rtol=0)
    np.testing.assert_allclose(wrap_angle(restored[:, 2]-world[:, 2]), 0., atol=1e-12, rtol=0)
    fractional = [k+beta if identity is None else float(identity) for identity in identities]
    assert all(a >= c3['arc_m']-DUPLICATE_ATOL for a in arcs)
    for row, identity in enumerate(identities):
        if identity is not None:
            np.testing.assert_array_equal(world[row], fresh[identity])
            np.testing.assert_array_equal(local[row], raw[identity])
    return world, local, dict(correspondence=c3, row_count=len(world), original_count=len(fresh),
        raw_prefix_rows_removed=start, first_raw_identity_presented=start,
        row_original_identities=identities, row_fractional_original_progress=fractional,
        row_original_arc_m=arcs.tolist(), remaining_original_arc_at_start_m=float(curve.arc[-1]-arcs[0]),
        first_presented_original_arc_m=float(arcs[0]), first_presented_fractional_original_row=fractional[0],
        duplicate_handling=dedup, original_downstream_world_and_local_rows_exact=True,
        frame='world metres/yaw radians; entry local = A^-1 E; original raw suffix retained, no B re-anchor')


def first_dwell_index(times, good):
    """Same sampled inclusive .30 s rule and search tolerance as sustained_join."""
    t, good = np.asarray(times, float), np.asarray(good, bool)
    if t.ndim != 1 or not len(t) or good.shape != t.shape or not np.isfinite(t).all() or np.any(np.diff(t) <= 0):
        raise ValueError('finite increasing execution times with one flag per sample required')
    for i, start in enumerate(t):
        end = start+DWELL_S
        if end > t[-1]+1e-12:
            break
        stop = min(int(np.searchsorted(t, end-1e-12, side='left')), len(t)-1)
        if good[i:stop+1].all():
            return i
    return None


def endpoint_dwell(times, poses, endpoint, commands, attachment_s=None):
    t, p, end = np.asarray(times, float), np.asarray(poses, float), np.asarray(endpoint, float)
    u = np.asarray(commands, float).reshape((-1, 2))
    if p.shape != (len(t), 3) or u.shape != (len(t)-1, 2) or end.shape != (3,) or t[0] != 0:
        raise ValueError('B-start execution samples and one held command per interval required')
    distance = np.linalg.norm(p[:, :2]-end[:2], axis=1)
    yaw = abs(wrap_angle(p[:, 2]-end[2]))
    good = (distance <= POSITION_M) & (yaw <= YAW_RAD)
    index = first_dwell_index(t, good)
    entries = np.flatnonzero(good)
    duration = None if index is None else float(t[index])
    length = None if index is None else float(np.sum(abs(u[:index, 0])*np.diff(t)[:index]))
    if index is not None:
        status = 'OBSERVED_ORIGINAL_FRESH_ENDPOINT_DWELL'
    elif not len(entries):
        status = 'NO_ENDPOINT_TUBE_ENTRY_OBSERVED'
    elif good[-1]:
        status = 'ENDPOINT_TUBE_DWELL_RIGHT_CENSORED'
    else:
        status = 'TRANSIENT_ENDPOINT_TUBE_ENTRY_THEN_EXIT'
    return dict(original_FRESH_endpoint_dwell_s=duration,
        endpoint_dwell_index=index, endpoint_observation_status=status,
        first_endpoint_tube_entry_s=None if not len(entries) else float(t[entries[0]]),
        T_post_attach_s=None if duration is None or attachment_s is None else duration-attachment_s,
        executed_path_length_to_endpoint_dwell_m=length,
        mean_abs_commanded_v_before_endpoint_dwell_mps=None if duration is None or duration == 0 else length/duration,
        execution_observation_horizon_s=float(t[-1]),
        endpoint_dwell_trace=dict(distance_m=distance.tolist(), yaw_error_rad=yaw.tolist(), inside=good.tolist()),
        name='original-FRESH endpoint dwell time', sampled_only=True, navigation_task_completion_claim=False)


def execution_metrics(rollout, metric, fresh, cap_steps=180):
    if metric is None:
        return None
    result = metric_row(metric, rollout['termination'])
    trace = metric['trace']
    result.update(endpoint_dwell(trace['times_s'], trace['poses_world'], np.asarray(fresh)[-1],
        [[c['v_mps'], c['omega_radps']] for c in rollout['commands']], result['sustained_attachment_s']))
    j = None if result['sustained_attachment_s'] is None else int(np.searchsorted(trace['times_s'], result['sustained_attachment_s']))
    complete = len(rollout['commands']) == cap_steps
    endpoint_index = result['endpoint_dwell_index']
    result.update(attachment_original_arc_m=None if j is None else float(trace['arc_progress_m'][j]),
        endpoint_error_at_3s_cap_m=metric['endpoint']['terminal_position_error_m'] if complete else None,
        projected_original_row_at_3s_cap=metric['endpoint']['final_progress'] if complete else None,
        projected_original_arc_at_3s_cap_m=metric['endpoint']['final_arc_progress_m'] if complete else None,
        projected_original_arc_fraction_at_3s_cap=(metric['endpoint']['final_arc_progress_m'] /
            (metric['endpoint']['final_arc_progress_m']+metric['endpoint']['remaining_arc_m'])) if complete else None,
        remaining_original_arc_at_3s_cap_m=metric['endpoint']['remaining_arc_m'] if complete else None,
        nominal_cap_s=3., actual_fixed_cap_s=cap_steps*rollout['phase']['integration_dt_s'],
        fixed_cap_observed=complete,
        endpoint_dwell_projected_original_row=None if endpoint_index is None else float(trace['projection'][endpoint_index]['progress']),
        endpoint_dwell_projected_original_arc_m=None if endpoint_index is None else float(trace['arc_progress_m'][endpoint_index]),
        max_abs_v_mps=max(abs(c['v_mps']) for c in rollout['commands']),
        safety_abort_tick=None if rollout['safety_abort'] is None else rollout['safety_abort']['state']['tick'])
    return result


def selector_progress(rollout, mapping):
    rows = []
    for event in rollout['events']:
        if event.get('type') != 'solve_result' or event.get('status') != 'command':
            continue
        selected = event['selection']['indices']
        nearest = event['selection']['nearest_index']
        arcs = [mapping['row_original_arc_m'][i] for i in selected]
        rows.append(dict(submit_tick=event['input_state_id'], input_pose=event['input_pose'],
            application_tick=event.get('continuation_seen', {}).get('tick'),
            withheld=bool(event.get('withheld_by_logical_scheduler', False)),
            nearest_reference_row=nearest, nearest_original_arc_m=mapping['row_original_arc_m'][nearest],
            selected_reference_rows=selected,
            selected_original_identities=[mapping['row_original_identities'][i] for i in selected],
            selected_fractional_original_rows=[mapping['row_fractional_original_progress'][i] for i in selected],
            selected_original_arcs_m=arcs, first_selected_original_arc_m=arcs[0],
            endpoint_repeated=event['selection']['endpoint_repeated'],
            below_entry=any(a < mapping['first_presented_original_arc_m']-DUPLICATE_ATOL for a in arcs)))
    return dict(rows=rows, first=None if not rows else rows[0],
        any_identity_before_entry=any(r['below_entry'] for r in rows),
        first_progress_moves_backward_over_time=any(b['first_selected_original_arc_m'] < a['first_selected_original_arc_m']-DUPLICATE_ATOL
                                                   for a, b in zip(rows, rows[1:])),
        identity_scope='interpolated E has no raw identity; integer downstream identities are original FRESH rows')


def schedule_gate(rollouts, frozen, common):
    checks = {}
    for name in ORDER:
        r = rollouts.get(name)
        if r is None:
            checks[name] = dict(passed=False, available=False)
            continue
        exact = r['phase'] == common and r['states'][0]['pose_world'] == common['B']
        primary, full = schedule(r, 54), schedule(r, 180)
        checks[name] = dict(available=True, initial_provenance=exact, steps=len(r['commands']),
            primary=primary, full=full,
            primary_passed=exact and len(r['commands']) >= 54 and primary == frozen['primary'],
            passed=exact and len(r['commands']) == 180 and primary == frozen['primary'] and full == frozen['full'])
    return dict(passed=all(c['passed'] for c in checks.values()), methods=checks,
                rule='exact historical per-source B provenance and submit/application schedule at 54 and 180 intervals')


def comparison(sources):
    """Predeclared precedence; report every pair, including missing dwell outcomes."""
    pairs, early_sources, endpoint_worse, endpoint_observable, tradeoffs = {}, [], [], [], []
    technical = failure = False
    for sid, q in sources.items():
        e = q['primary_metrics'].get(ENTRY)
        technical |= not q.get('technical_valid', True)
        failure |= (not q['reference_safety'][ENTRY]['clearance_valid'] or q['termination'][ENTRY] == 'SAFETY_ABORT_BEFORE_UNSAFE_COMMAND'
                    or q.get('controller_reference_failure', False))
        if e is None:
            continue
        technical |= q['selector'][ENTRY]['any_identity_before_entry']
        if q['termination'][ENTRY] not in ['OBSERVATION_CAP', 'SAFETY_ABORT_BEFORE_UNSAFE_COMMAND'] and not q.get('controller_reference_failure', False):
            technical = True
        if q['termination'][ENTRY] == 'OBSERVATION_CAP' and not q['schedule_gate']['passed']:
            technical = True
        if e['execution_clearance_lower_bound_m'] < .05:
            failure = True
        full = q['primary_metrics'][FULL]
        if e['position_auc_09_m_s'] is not None and e['position_auc_09_m_s'] < full['position_auc_09_m_s']-SIGN_ATOL:
            early_sources.append(sid)
        pairs[sid] = {}
        observable = worse_here = False
        for name in [NATIVE, FULL]:
            b = q['primary_metrics'][name]
            ta, tb = e['sustained_attachment_s'], b['sustained_attachment_s']
            te, be = e['original_FRESH_endpoint_dwell_s'], b['original_FRESH_endpoint_dwell_s']
            earlier = ta is not None and tb is not None and ta < tb-SIGN_ATOL
            lost = be is not None and te is None
            later = te is not None and be is not None and te > be+SIGN_ATOL
            more_arc = (earlier and e['remaining_arc_at_attachment_m'] > b['remaining_arc_at_attachment_m']+MORE_REMAINING_M)
            observable |= be is not None
            worse_here |= lost or later
            row = dict(earlier_observed_attachment=earlier, attachment_recovered=ta is not None and tb is None,
                endpoint_later=bool(later), endpoint_lost=lost,
                endpoint_recovered=te is not None and be is None,
                substantially_more_arc_at_attachment=bool(more_arc),
                AUC_09_delta=None if e['position_auc_09_m_s'] is None else e['position_auc_09_m_s']-b['position_auc_09_m_s'],
                attachment_delta_s=None if ta is None or tb is None else ta-tb,
                endpoint_delta_s=None if te is None or be is None else te-be)
            pairs[sid][name] = row
            if earlier and (later or lost or more_arc):
                tradeoffs.append(dict(source_id=sid, baseline=name, **row))
        if observable:
            endpoint_observable.append(sid)
        if worse_here:
            endpoint_worse.append(sid)
    systematic = bool(endpoint_observable) and 2*len(endpoint_worse) > len(endpoint_observable)
    if technical:
        label = 'TECHNICAL_BLOCKED'
    elif failure:
        label = 'ENTRY_REFERENCE_FAILURE'
    elif tradeoffs:
        label = 'FAST_ATTACHMENT_SLOW_COMPLETION_TRADEOFF'
    elif len(early_sources) >= 3 and endpoint_observable and not systematic:
        label = 'ENTRY_TRANSITION_AND_COMPLETION_SUPPORTED'
    else:
        label = 'ENTRY_PROGRESS_INSUFFICIENT'
    return dict(classification=label, pairwise=pairs, lower_AUC_than_Full_sources=early_sources,
        endpoint_observable_baseline_sources=endpoint_observable, later_or_lost_endpoint_sources=endpoint_worse,
        systematically_later_endpoint=systematic, fast_attachment_tradeoffs=tradeoffs,
        rule='strict majority of sources with baseline endpoint observation; lost observed endpoint counts as worse, never cap-imputed')


@contextmanager
def no_reconciliation_optimizer():
    previous = sys.getprofile()
    def profile(frame, event, arg):
        if event == 'call' and frame.f_code.co_name in {'solve_least_squares', 'solve_graph', 'solve_half', 'solve_condition'}:
            raise RuntimeError('reconciliation optimizer forbidden in entry-suffix experiment')
    sys.setprofile(profile)
    try:
        yield
    finally:
        sys.setprofile(previous)

"""Saved V2 installation and descriptive execution comparison; no optimizer."""
import numpy as np
from shapely.geometry import LineString
from .se2 import relative_pose, wrap_angle, local_trajectory_to_world
from .spatial_entry_suffix import NATIVE, ENTRY, execution_metrics, endpoint_dwell, no_reconciliation_optimizer
from .b_to_entry_bridge import HERMITE
from .spatial_correspondence_selector import SOURCE_IDS
from .osa03_common_b import schedule

VECTOR = 'V2_GRAPH_BRIDGE'
ORDER = [NATIVE, ENTRY, HERMITE, VECTOR]
PNGS = ['world_execution_overview.png', 'transition_completion_metrics.png', 'bridge_tradeoff_diagnostics.png']
ATOL = 1e-9


def installed_reference(bridge, saved_world, suffix, A, raw, original_ids, B, E):
    """Use saved world bits and only transform the three derived bridge rows."""
    b, w, s, raw = map(np.asarray, (bridge, saved_world, suffix, raw))
    assert b.shape == (3, 3) and s.shape[1] == 3 and len(s) >= 2
    assert b[0].tobytes() == np.asarray(B, dtype=float).tobytes()
    assert b[2].tobytes() == np.asarray(E, dtype=float).tobytes() == s[0].tobytes()
    assert w.tobytes() == np.vstack([b, s[1:]]).tobytes()
    assert len(original_ids) == len(s)-1
    local = np.vstack([relative_pose(A, b), raw[original_ids]])
    restored = local_trajectory_to_world(A, local)
    np.testing.assert_allclose(restored, w, atol=1e-12, rtol=0)
    assert local[3:].tobytes() == raw[original_ids].tobytes()
    return w.copy(), local, ['B', 'bridge_1', 'E*', *[f'F_{j}' for j in original_ids]]


def geometry_pair(hermite, vector, safety):
    result = {}
    for name, reference in [(HERMITE, hermite), (VECTOR, vector)]:
        b = np.asarray(reference)[:3]; edges = np.diff(b[:, :2], axis=0)
        lengths = np.linalg.norm(edges, axis=1)
        turns = wrap_angle(np.diff(np.arctan2(edges[:, 1], edges[:, 0])))
        chord = float(np.linalg.norm(b[-1, :2]-b[0, :2]))
        result[name] = dict(nodes=b.tolist(), B_to_entry_chord_m=chord,
            bridge_XY_length_m=float(lengths.sum()), length_chord_ratio=float(lengths.sum()/chord),
            edge_lengths_m=lengths.tolist(), maximum_turning_angle_rad=float(max(abs(turns))),
            RMS_turning_angle_rad=float(np.sqrt(np.mean(turns**2))),
            reference_clearance_m=safety[name]['minimum_clearance_m'],
            bridge_self_intersection=not LineString(b[:, :2]).is_simple,
            full_reference_self_intersection=not LineString(np.asarray(reference)[:, :2]).is_simple)
    delta = np.asarray(vector)-np.asarray(hermite)
    result['difference'] = dict(max_bridge_node_XY_m=float(np.linalg.norm(delta[:3, :2], axis=1).max()),
        max_bridge_node_yaw_rad=float(abs(wrap_angle(delta[:3, 2])).max()),
        max_complete_reference_XY_m=float(np.linalg.norm(delta[:, :2], axis=1).max()),
        max_complete_reference_yaw_rad=float(abs(wrap_angle(delta[:, 2])).max()))
    return result


def exposure(rollout, labels):
    rows = []; common = rollout['phase']; bt = common['B_tick']; dt = common['integration_dt_s']
    for e in rollout['events']:
        if e.get('type') != 'solve_result' or e.get('status') != 'command': continue
        identities = [labels[i] for i in e['selection']['indices']]
        rows.append(dict(submit_tick=e['input_state_id'], elapsed_s=(e['input_state_id']-bt)*dt,
            application_tick=e.get('continuation_seen', {}).get('tick'),
            withheld=bool(e.get('withheld_by_logical_scheduler')), H5_identities=identities,
            nearest_identity=labels[e['selection']['nearest_index']], first_H5_identity=identities[0],
            bridge_1_in_H5='bridge_1' in identities, entry_in_H5='E*' in identities,
            any_bridge_in_H5=any(x in {'B', 'bridge_1', 'E*'} for x in identities)))
    bridge = [r for r in rows if r['any_bridge_in_H5']]
    original = next((r for r in rows if r['first_H5_identity'].startswith('F_')), None)
    return dict(rows=rows, first=None if not rows else rows[0],
        bridge_1_submit_count=sum(r['bridge_1_in_H5'] for r in rows),
        entry_submit_count=sum(r['entry_in_H5'] for r in rows),
        any_bridge_submit_count=len(bridge),
        first_original_H5_submit_tick=None if original is None else original['submit_tick'],
        last_bridge_H5_submit_tick=None if not bridge else bridge[-1]['submit_tick'],
        last_bridge_H5_elapsed_s=None if not bridge else bridge[-1]['elapsed_s'],
        bridge_exposure_sample_span_s=None if not bridge else bridge[-1]['elapsed_s']-bridge[0]['elapsed_s'],
        identity_scope='B, bridge_1 and E* remain derived identities; only F_k is an original row')


def command_comparison(h, v, hlabels, vlabels):
    """Align saved solve results and actual held commands by logical tick."""
    bt = h['phase']['B_tick']; dt = h['phase']['integration_dt_s']
    assert h['phase'] == v['phase']
    events = [{e['input_state_id']: e for e in r['events'] if e.get('type') == 'solve_result' and e.get('status') == 'command'} for r in (h, v)]
    states = [{s['absolute_tick']: s['pose_world'] for s in r['states']} for r in (h, v)]
    rows = []
    def row(kind, tick, hc, vc, hi, vi):
        delta = np.asarray(vc)-hc; pose_delta = np.asarray(states[1][tick])-states[0][tick]
        pose_delta[2] = wrap_angle(pose_delta[2])
        return dict(kind=kind, tick=tick, elapsed_s=(tick-bt)*dt, Hermite_command=hc, V2_command=vc,
            delta_v=float(delta[0]), delta_omega=float(delta[1]),
            differs=bool(np.any(abs(delta)>ATOL)), Hermite_H5=hi, V2_H5=vi,
            actual_pose_delta=pose_delta.tolist(), actual_XY_separation_m=float(np.linalg.norm(pose_delta[:2])))
    for tick in sorted(events[0].keys() & events[1].keys()):
        he, ve = events[0][tick], events[1][tick]
        rows.append(row('submit_result', tick, he['command'], ve['command'],
            [hlabels[i] for i in he['selection']['indices']], [vlabels[i] for i in ve['selection']['indices']]))
    command_maps = [{c['application_tick']: c for c in r['commands']} for r in (h, v)]
    by_id = [{e['solve_id']: e for e in m.values()} for m in events]
    for tick in sorted(command_maps[0].keys() & command_maps[1].keys()):
        hc, vc = command_maps[0][tick], command_maps[1][tick]
        ids = []
        for c, es, labels in zip((hc, vc), by_id, (hlabels, vlabels)):
            e = es.get(c.get('solve_id'))
            ids.append(['COMMON_B_COMMAND'] if e is None else [labels[i] for i in e['selection']['indices']])
        rows.append(row('applied_interval', tick, [hc['v_mps'], hc['omega_radps']],
            [vc['v_mps'], vc['omega_radps']], *ids))
    first = {kind: next((r for r in rows if r['kind'] == kind and r['differs']), None) for kind in ('submit_result', 'applied_interval')}
    return dict(rows=rows, first_different_submit=first['submit_result'], first_different_application=first['applied_interval'],
        max_matched_XY_separation_m=max((r['actual_XY_separation_m'] for r in rows), default=None),
        comparison_atol=ATOL, descriptive_only=True)


def schedule_check(r, frozen, common):
    if r is None: return dict(passed=False, available=False)
    initial = r['phase'] == common and r['states'][0]['pose_world'] == common['B']
    primary, full = schedule(r, 54), schedule(r, 180)
    return dict(available=True, initial_provenance=initial, steps=len(r['commands']), primary=primary, full=full,
        passed=initial and len(r['commands']) == 180 and primary == frozen['primary'] and full == frozen['full'])


def classify(sources):
    paired = {}; improved = 0
    for sid, q in sources.items():
        m = q['primary_metrics']; c, h, v = [m[n] for n in (ENTRY, HERMITE, VECTOR)]
        def diff(a, b, key): return None if a is None or b is None or a[key] is None or b[key] is None else a[key]-b[key]
        endpoint = 'original_FRESH_endpoint_dwell_s'
        row = dict(delay_Hermite_vs_C3_s=diff(h,c,endpoint), delay_V2_vs_C3_s=diff(v,c,endpoint),
            V2_minus_Hermite_endpoint_s=diff(v,h,endpoint), V2_minus_Hermite_attach_s=diff(v,h,'sustained_attachment_s'),
            V2_minus_Hermite_position_AUC_09=diff(v,h,'position_auc_09_m_s'),
            V2_minus_C3_position_AUC_09=diff(v,c,'position_auc_09_m_s'),
            V2_minus_Hermite_linear_TV=diff(v,h,'linear_command_TV'),
            V2_minus_Hermite_angular_TV=diff(v,h,'angular_command_TV'), one_tick_s=q['integration_dt_s'])
        threshold = q['integration_dt_s']-ATOL
        row['endpoint_improves_vs_Hermite_one_tick'] = row['V2_minus_Hermite_endpoint_s'] is not None and row['V2_minus_Hermite_endpoint_s'] <= -threshold
        row['endpoint_worsens_vs_Hermite_one_tick'] = row['V2_minus_Hermite_endpoint_s'] is not None and row['V2_minus_Hermite_endpoint_s'] >= threshold
        row['faster_attachment_slower_endpoint_vs_Hermite'] = (row['V2_minus_Hermite_attach_s'] is not None and
            row['V2_minus_Hermite_attach_s'] <= -threshold and row['endpoint_worsens_vs_Hermite_one_tick'])
        paired[sid] = row
        if sid != SOURCE_IDS[2]: improved += row['endpoint_improves_vs_Hermite_one_tick']
    hard = sources[SOURCE_IDS[2]]['primary_metrics'][VECTOR]
    hard_recovery = hard is not None and hard['sustained_attachment_s'] is not None and hard['original_FRESH_endpoint_dwell_s'] is not None
    easy = [sid for sid in SOURCE_IDS if sid != SOURCE_IDS[2]]
    retained = all(sources[s]['primary_metrics'][VECTOR] is not None and sources[s]['primary_metrics'][VECTOR]['original_FRESH_endpoint_dwell_s'] is not None for s in easy)
    if any(not q['technical_valid'] for q in sources.values()): category = 'TECHNICAL_BLOCKED'
    elif any(q['reference_or_execution_failure'] for q in sources.values()): category = 'VECTOR_REFERENCE_OR_EXECUTION_FAILURE'
    elif not hard_recovery: category = 'HARD_CASE_RECOVERY_LOST'
    elif not retained or any(paired[s]['endpoint_worsens_vs_Hermite_one_tick'] for s in easy): category = 'VECTOR_GRAPH_TRADEOFF_WORSE'
    elif improved >= 2 and all(paired[s]['V2_minus_Hermite_endpoint_s'] <= ATOL for s in easy): category = 'VECTOR_GRAPH_TRADEOFF_REDUCED'
    elif hard_recovery and retained: category = 'VECTOR_GRAPH_RECOVERY_NO_CLEAR_TRADEOFF_GAIN'
    else: category = 'MIXED_VECTOR_EXECUTION_EFFECT'
    decision = ('A' if category == 'VECTOR_GRAPH_TRADEOFF_REDUCED' else
                'B' if category == 'VECTOR_GRAPH_RECOVERY_NO_CLEAR_TRADEOFF_GAIN' else 'C')
    return dict(classification=category, next_step_decision=decision, paired=paired,
        S3_both_dwells=hard_recovery, easy_endpoint_dwells_retained=retained,
        easy_sources_improved_one_tick=improved)

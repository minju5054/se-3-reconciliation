"""R01 source compatibility and paired saved-result diagnostics; no new controller."""
from copy import deepcopy
import numpy as np
from .osa03_relative_ablation import ORDER, gaps

LABEL = 'timing-controlled offline replication'
EFFECT_KEYS = [
    'max_position_error_05_m', 'position_auc_03_m_s', 'position_auc_09_m_s',
    'yaw_auc_03_rad_s', 'yaw_auc_09_rad_s', 'sustained_attachment_s',
    'remaining_arc_at_attachment_m', 'execution_clearance_lower_bound_m',
    'endpoint_error_m', 'linear_command_TV', 'angular_command_TV',
    'relative_translation_RMS_m', 'relative_translation_max_m',
]


def compatibility(common, schedule, r00_common, settings, r00_settings):
    """Require the inherited wrapper's exact clock/identity assumptions, not poses."""
    keys = ['B_tick', 'B_sim_s', 'integration_dt_s', 'next_submit_after_B',
            'fresh_chunk_id', 'fresh_version']
    checks = {k: common[k] == r00_common[k] for k in keys}
    checks.update(B_origin=common['B_tick'] == schedule['B_tick'] == 92,
        dt=common['integration_dt_s'] == schedule['integration_dt_s'],
        first_submit=common['next_submit_after_B'] == schedule['pairs'][0]['submit_tick'] == 96,
        official_settings=settings == r00_settings,
        memory=common['u_mem_B'] == common['first_FRESH_solve']['command'] == common['u_B_plus'],
        generation=common['first_FRESH_solve']['official_generation'] == common['original_generation'] ==
                   common['first_FRESH_solve']['result_generation'],
        grid=all(p['submit_tick'] % 6 == 0 and p['submit_tick'] < p['application_tick']
                 for p in schedule['pairs']),
        release_before_next_submit=all(a['application_tick'] < b['submit_tick']
                                      for a, b in zip(schedule['pairs'], schedule['pairs'][1:])))
    return dict(compatible=all(checks.values()), checks=checks,
                initial_state_source='REPEAT_01 only; R00 used only for clock compatibility',
                generation_note='R01 generation is restored exactly; generation counters need not equal across independent acquisitions',
                restored_generation=common['original_generation'], R00_generation=r00_common['original_generation'])


def selector_diagnostics(rollouts):
    """Read copies of finished solve events; cannot influence selection or commands."""
    rows = {}
    for name in ['FULL_LOCAL_SE2', 'NO_RELATIVE']:
        r = rollouts.get(name)
        rows[name] = [] if r is None else [dict(submit_tick=e['input_state_id'],
            input_pose=deepcopy(e['input_pose']), nearest_row=e['selection']['nearest_index'],
            selected_H5_rows=deepcopy(e['selection']['indices']), command=deepcopy(e['command']),
            application_tick=e.get('continuation_seen', {}).get('tick'),
            applied=any(c['solve_id'] == e['solve_id'] for c in r['commands']))
            for e in r['events'] if e.get('type') == 'solve_result' and e.get('status') == 'command']
    full = {r['submit_tick']: r for r in rows['FULL_LOCAL_SE2']}
    no_r = {r['submit_tick']: r for r in rows['NO_RELATIVE']}
    differences = [t for t in full.keys() & no_r.keys()
                   if full[t]['selected_H5_rows'] != no_r[t]['selected_H5_rows']]
    return dict(methods=rows, first_differing_submit_tick=min(differences, default=None),
                differing_submit_ticks=sorted(differences), diagnostic_only=True)


def paired_effects(r00, r01):
    def effects(s):
        out = gaps(s['primary_metrics'], 'FULL_LOCAL_SE2')['NO_RELATIVE']
        if out is None:
            out = {k: None for k in EFFECT_KEYS}
        for k in EFFECT_KEYS[-2:]:
            a = s['planning'].get('NO_RELATIVE'); b = s['planning'].get('FULL_LOCAL_SE2')
            out[k] = None if a is None or b is None else a[k] - b[k]
        return out
    a, b = effects(r00), effects(r01)
    return [dict(metric=k, R00_NoR_minus_Full=a[k], R01_NoR_minus_Full=b[k],
                 sign_consistent=None if a[k] is None or b[k] is None else bool(np.sign(a[k]) == np.sign(b[k])))
            for k in EFFECT_KEYS]


def classify(summary, valid):
    """Predeclared directions only; no fitted thresholds or scalar score."""
    if not valid or not summary['schedule_gate']['comparable'] or any(
        t in ['CONTROLLER_ERROR', 'HOLD_TIMEOUT'] for t in summary['termination'].values()):
        return 'TECHNICAL_BLOCKED'
    rows = summary['primary_metrics']; full = rows.get('FULL_LOCAL_SE2'); no_r = rows.get('NO_RELATIVE')
    if full is None or no_r is None:
        return 'NOT_REPLICATED'
    d = gaps(rows, 'FULL_LOCAL_SE2')['NO_RELATIVE']
    earlier = d['sustained_attachment_s'] is not None and d['sustained_attachment_s'] < 0
    early = all(d[k] is not None and d[k] <= 0 for k in
                ['max_position_error_05_m', 'position_auc_03_m_s', 'position_auc_09_m_s'])
    safe = all(summary['reference_safety'][n]['clearance_valid'] and
               rows[n] is not None and rows[n]['execution_clearance_lower_bound_m'] >= .05 and
               summary['termination'][n] == 'OBSERVATION_CAP' for n in ORDER)
    distortion = all(summary['planning']['NO_RELATIVE'][k] > summary['planning']['FULL_LOCAL_SE2'][k]
                     for k in EFFECT_KEYS[-2:])
    if earlier and early and safe and distortion:
        return 'REPLICATED_WITHIN_PAIRED_OSA03'
    if earlier or early:
        return 'PARTIAL_REPLICATION'
    return 'NOT_REPLICATED'

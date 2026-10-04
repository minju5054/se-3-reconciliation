"""Fixed boundary-row staging and saved execution diagnostics; no planning solve."""
import numpy as np
from shapely.geometry import LineString
from .se2 import relative_pose, local_trajectory_to_world, wrap_angle
from .spatial_entry_suffix import NATIVE, ENTRY, DUPLICATE_ATOL, execution_metrics, no_reconciliation_optimizer
from .vector_bridge_execution03 import VECTOR, HERMITE, SOURCE_IDS, schedule_check, ATOL

STAGE = 'B_ENTRY_STAGE'
HISTORICAL = [NATIVE, ENTRY, HERMITE, VECTOR]
ORDER = [*HISTORICAL, STAGE]
PRIMARY = [ENTRY, STAGE, HERMITE, VECTOR]
PNGS = ['world_execution_overview.png', 'staging_transition_completion.png', 'selector_staging_diagnostic.png']


def staged_reference(native, raw, A, B, entry, saved_suffix):
    """Insert derived B before the frozen entry; copy downstream native/raw bits."""
    native, raw, suffix = map(np.asarray, (native, raw, saved_suffix))
    E = np.asarray(entry['correspondence']['target_world'], float)
    assert native.shape == raw.shape and native.shape[1:] == (3,)
    np.testing.assert_allclose(suffix[0, :2], E[:2], atol=DUPLICATE_ATOL, rtol=0)
    assert abs(wrap_angle(suffix[0, 2]-E[2])) <= DUPLICATE_ATOL
    ids = entry['row_original_identities'][1:]
    assert len(ids) == len(suffix)-1 and all(isinstance(i, int) for i in ids)
    assert suffix[1:].tobytes() == native[ids].tobytes()
    # The historical suffix already removed a duplicate entry vertex. Retain it once.
    world = np.vstack([np.asarray(B, float), E, native[ids]])
    local = np.vstack([relative_pose(A, world[:2]), raw[ids]])
    assert len(world) >= 2 and entry['correspondence']['remaining_arc_m'] > 0
    np.testing.assert_allclose(local_trajectory_to_world(A, local), world, atol=1e-12, rtol=0)
    assert world[0].tobytes() == np.asarray(B, float).tobytes()
    assert world[1].tobytes() == E.tobytes() and local[2:].tobytes() == raw[ids].tobytes()
    labels = ['B', 'E*', *[f'F_{i}' for i in ids]]
    return world, local, labels


def identity_mapping(labels, entry, native):
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(np.asarray(native)[:, :2], axis=0), axis=1))]
    c = entry['correspondence']; rows = []
    for label in labels:
        raw_id = int(label[2:]) if label.startswith('F_') else None
        fraction = float(raw_id) if raw_id is not None else c['segment']+c['alpha'] if label == 'E*' else None
        progress = float(arc[raw_id]) if raw_id is not None else c['arc_m'] if label == 'E*' else None
        rows.append(dict(label=label, original_raw_id=raw_id, fractional_original_row=fraction,
                         original_arc_m=progress, original_arc_fraction=None if progress is None else progress/arc[-1]))
    return rows


def selector_exposure(rollout, mapping):
    """Only actual successful official selection records; never query a new state."""
    rows = []; c = rollout['phase']; bt, dt = c['B_tick'], c['integration_dt_s']
    for event in rollout['events']:
        if event.get('type') != 'solve_result' or event.get('status') != 'command': continue
        selected = [mapping[i] for i in event['selection']['indices']]
        labels = [r['label'] for r in selected]; nearest = mapping[event['selection']['nearest_index']]['label']
        rows.append(dict(submit_tick=event['input_state_id'], elapsed_s=(event['input_state_id']-bt)*dt,
            input_pose=event['input_pose'], nearest_identity=nearest, first_H5_identity=labels[0],
            H5_identities=labels, H5_poses=event['selection']['reference_world'], H5_original_metadata=selected,
            first_H5_original_arc_m=selected[0]['original_arc_m'], B_in_H5='B' in labels,
            entry_in_H5='E*' in labels, bridge_1_in_H5='bridge_1' in labels,
            any_derived_in_H5=any(x in {'B','E*','bridge_1'} for x in labels),
            application_tick=event.get('continuation_seen', {}).get('tick'),
            withheld=bool(event.get('withheld_by_logical_scheduler', False))))
    derived = [r for r in rows if r['any_derived_in_H5']]
    entries = [r for r in rows if r['entry_in_H5']]
    boundary = [r for r in rows if r['entry_in_H5'] or r['B_in_H5']]
    original = next((r for r in rows if r['first_H5_identity'].startswith('F_')), None)
    def span(samples): return None if not samples else samples[-1]['elapsed_s']-samples[0]['elapsed_s']
    return dict(rows=rows, first=None if not rows else rows[0], nearest_B_submit_count=sum(r['nearest_identity']=='B' for r in rows),
        first_H5_entry_submit_count=sum(r['first_H5_identity']=='E*' for r in rows), entry_submit_count=len(entries),
        bridge_1_submit_count=sum(r['bridge_1_in_H5'] for r in rows), any_derived_submit_count=len(derived),
        last_B_or_entry_submit_tick=None if not boundary else boundary[-1]['submit_tick'],
        last_B_or_entry_elapsed_s=None if not boundary else boundary[-1]['elapsed_s'],
        first_original_H5_submit_tick=None if original is None else original['submit_tick'],
        entry_sample_span_s=span(entries), derived_sample_span_s=span(derived),
        last_entry_elapsed_s=None if not entries else entries[-1]['elapsed_s'],
        sampled_only=True, includes_successful_submits_withheld_at_cap=True)


def compare_commands(left, right, left_labels, right_labels):
    assert left['phase'] == right['phase']
    c=left['phase']; bt,dt=c['B_tick'],c['integration_dt_s']
    events=[{e['input_state_id']:e for e in r['events'] if e.get('type')=='solve_result' and e.get('status')=='command'} for r in [left,right]]
    states=[{s['absolute_tick']:s['pose_world'] for s in r['states']} for r in [left,right]]
    rows=[]
    def row(kind,tick,commands,identities,shared=False):
        delta=np.asarray(commands[1])-commands[0]; pd=np.asarray(states[1][tick])-states[0][tick]; pd[2]=wrap_angle(pd[2])
        return dict(kind=kind,tick=tick,elapsed_s=(tick-bt)*dt,left_command=commands[0],right_command=commands[1],
            delta_v=float(delta[0]),delta_omega=float(delta[1]),command_differs=bool(np.any(abs(delta)>ATOL)),
            left_H5=identities[0],right_H5=identities[1],H5_differs=identities[0]!=identities[1],
            shared_preexisting_B_command=shared, actual_pose_delta=pd.tolist(),actual_XY_separation_m=float(np.linalg.norm(pd[:2])))
    for tick in sorted(events[0].keys() & events[1].keys()):
        es=[e[tick] for e in events]
        rows.append(row('submit_result',tick,[e['command'] for e in es],[[lab[i] for i in e['selection']['indices']] for e,lab in zip(es,[left_labels,right_labels])]))
    maps=[{c['application_tick']:c for c in r['commands']} for r in [left,right]]
    by_id=[{e['solve_id']:e for e in ev.values()} for ev in events]
    for tick in sorted(maps[0].keys() & maps[1].keys()):
        cs=[m[tick] for m in maps];ids=[];shared=False
        for cmd,ev,labels in zip(cs,by_id,[left_labels,right_labels]):
            e=ev.get(cmd.get('solve_id'));shared |= e is None
            ids.append(['COMMON_B_COMMAND'] if e is None else [labels[i] for i in e['selection']['indices']])
        rows.append(row('applied_interval',tick,[[c['v_mps'],c['omega_radps']] for c in cs],ids,shared))
    # Include terminal sample in the all-state separation trace, even without a command.
    separation=[dict(tick=t,elapsed_s=(t-bt)*dt,XY_m=float(np.linalg.norm(np.asarray(states[1][t])[:2]-np.asarray(states[0][t])[:2]))) for t in sorted(states[0].keys() & states[1].keys())]
    def first(kind,field):return next((r for r in rows if r['kind']==kind and r[field] and not r['shared_preexisting_B_command']),None)
    return dict(rows=rows,first_different_H5=first('submit_result','H5_differs'),
        first_different_submit=first('submit_result','command_differs'),first_different_application=first('applied_interval','command_differs'),
        pose_separation_trace=separation,max_matched_XY_separation_m=max((s['XY_m'] for s in separation),default=None),comparison_atol=ATOL)


def reference_geometry(world, safety, labels):
    xy=np.asarray(world)[:,:2];lengths=np.linalg.norm(np.diff(xy,axis=0),axis=1)
    return dict(row_count=len(world),labels=labels,nodes=world.tolist(),total_XY_arc_m=float(lengths.sum()),
        min_segment_m=float(lengths.min()),max_segment_m=float(lengths.max()),reference_clearance_m=safety['minimum_clearance_m'],
        self_intersection=not LineString(xy).is_simple,complete_reference_includes_B_to_entry=labels[0]=='B')


def classify(sources):
    paired={};easy=[s for s in SOURCE_IDS if s!=SOURCE_IDS[2]]
    endpoint='original_FRESH_endpoint_dwell_s';attach='sustained_attachment_s'
    def diff(a,b,k):return None if a is None or b is None or a[k] is None or b[k] is None else a[k]-b[k]
    for sid,q in sources.items():
        m=q['primary_metrics'];b=m[STAGE];c=m[ENTRY];h=m[HERMITE];v=m[VECTOR];tick=q['integration_dt_s']-ATOL
        r=dict(delay_B_vs_C3_s=diff(b,c,endpoint),delay_Hermite_vs_C3_s=diff(h,c,endpoint),delay_V2_vs_C3_s=diff(v,c,endpoint),one_tick_s=q['integration_dt_s'])
        for name,base in [('C3',c),('Hermite',h),('V2',v)]:
            de,da=diff(b,base,endpoint),diff(b,base,attach)
            r.update({f'B_minus_{name}_endpoint_s':de,f'B_minus_{name}_attach_s':da,
                f'B_minus_{name}_AUC09':diff(b,base,'position_auc_09_m_s'),
                f'faster_attachment_slower_endpoint_vs_{name}':da is not None and de is not None and da<=-tick and de>=tick})
        r['improves_Hermite_one_tick']=r['B_minus_Hermite_endpoint_s'] is not None and r['B_minus_Hermite_endpoint_s']<=-tick
        r['worsens_Hermite_one_tick']=r['B_minus_Hermite_endpoint_s'] is not None and r['B_minus_Hermite_endpoint_s']>=tick
        paired[sid]=r
    def both(m):return m is not None and m[attach] is not None and m[endpoint] is not None
    hard=sources[SOURCE_IDS[2]]['primary_metrics'];recovered=both(hard[STAGE]);historical=both(hard[HERMITE]) and both(hard[VECTOR])
    retained=all(sources[s]['primary_metrics'][STAGE] is not None and sources[s]['primary_metrics'][STAGE][endpoint] is not None for s in easy)
    improved=sum(paired[s]['improves_Hermite_one_tick'] for s in easy)
    no_worse=retained and all(paired[s]['B_minus_Hermite_endpoint_s']<=ATOL for s in easy)
    if any(not q['technical_valid'] for q in sources.values()) or not historical:category='TECHNICAL_BLOCKED'
    elif any(q['reference_or_execution_failure'] for q in sources.values()):category='BOUNDARY_STAGE_REFERENCE_OR_EXECUTION_FAILURE'
    elif not recovered:category='INTERMEDIATE_BRIDGE_NEEDED_SUPPORTED'
    elif retained and no_worse and improved>=2:category='STAGING_SUFFICIENT_AND_PENALTY_REDUCED'
    elif retained and not any(paired[s]['worsens_Hermite_one_tick'] for s in easy):category='STAGING_SUFFICIENT_NO_PENALTY_GAIN'
    elif not retained or any(paired[s]['worsens_Hermite_one_tick'] for s in easy):category='STAGING_RECOVERY_WITH_NEW_TRADEOFF'
    else:category='MIXED_STAGING_EFFECT'
    direction={'INTERMEDIATE_BRIDGE_NEEDED_SUPPORTED':'B','STAGING_SUFFICIENT_AND_PENALTY_REDUCED':'A',
               'STAGING_SUFFICIENT_NO_PENALTY_GAIN':'C','STAGING_RECOVERY_WITH_NEW_TRADEOFF':'C'}.get(category)
    return dict(classification=category,direction=direction,paired=paired,S3_both_dwells=recovered,
        easy_endpoint_dwells_retained=retained,easy_sources_improved_one_tick=improved,graph_necessity_established=False)

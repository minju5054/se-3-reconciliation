#!/usr/bin/env python3
"""Saved-only validation of four C3 executions and eight historical comparisons."""
import argparse
import json
from pathlib import Path
import numpy as np

from run_spatial_entry_suffix_execution01 import ROOT, RESULTS, read, save, sha, plain, verify, authenticate
from reconciliation.spatial_entry_suffix import *
from reconciliation.relative_factor_multisource import geometry, reference_safety
from reconciliation.osa03_native import evaluate
from reconciliation.osa03_relative_ablation import metric_row
from validate_relative_factor_multisource01 import validate_method
from validate_robotless_online_handoffs import equal_record
from validate_spatial_correspondence_selector_diag01 import csv_write


def method_folder(folder, name):
    return folder/'methods'/name if name == ENTRY else Path(read(folder/'reuse.json')['methods'][name]['folder'])


def independent_endpoint_time(rollout, fresh):
    t = np.array([s['time_s'] for s in rollout['states']])
    p = np.array([s['pose_world'] for s in rollout['states']])
    d = np.linalg.norm(p[:, :2]-fresh[-1, :2], axis=1)
    a = p[:, 2]-fresh[-1, 2]
    good = (d <= .1) & (abs(wrap_angle(a)) <= np.pi/12)
    starts = np.flatnonzero(good & ~np.r_[False, good[:-1]])
    ends = np.flatnonzero(good & ~np.r_[good[1:], False])
    return next((float(t[i]) for i,j in zip(starts,ends) if t[j]-t[i] >= .3-1e-12), None)


def validate_source(folder, diagnostic_source, historical_source):
    common = read(folder/'common_state.json')
    cfg = read(folder/'protocol.json')
    reuse = read(folder/'reuse.json')
    refs = read(folder/'references.json')
    entry = read(folder/'entry.json')
    for file, original in reuse['copies'].items():
        assert sha(folder/file) == sha(original['path']) == original['sha256']
    fresh = np.load(refs[NATIVE]['world_path'])
    raw = np.load(refs[NATIVE]['local_path'])
    world, local, descriptor = suffix_reference(fresh, raw, common['fresh_capture_pose'], common['B'],
        diagnostic_source['correspondences']['C3_FORWARD_SE2'])
    assert descriptor == refs[ENTRY]['descriptor']
    for frame, expected in [('world',world), ('local',local)]:
        np.testing.assert_array_equal(np.load(refs[ENTRY][frame+'_path']), expected)
    for key, value in descriptor.items():
        assert entry[key] == value
    pre = read(folder/'restoration_preflight.json')
    assert pre['passed'] and pre['numerical_MPC_calls'] == 0
    assert len(pre['rows']) == 1 and pre['rows'][0]['identity'] == ENTRY
    native_installed = np.asarray(read(method_folder(folder,NATIVE)/'restoration.json')['installed_world'])
    for j, identity in enumerate(descriptor['row_original_identities']):
        if identity is not None:
            np.testing.assert_array_equal(pre['rows'][0]['installed_world'][j], native_installed[identity])
    env = geometry(folder)
    settings = read(folder/'source_manifest.json')['mpc']['official_settings']
    assert pre['provenance']['official_settings'] == settings
    scene = read(folder/'scenario.json')
    frozen = read(folder/'schedule.json')
    release = {p['submit_tick']:p['application_tick'] for p in frozen['pairs']}
    primary, secondary, metrics, rollouts, audits, selections, mappings = {}, {}, {}, {}, {}, {}, {}
    for name in ORDER:
        ref = refs[name]
        out = method_folder(folder,name)
        where = folder if name == ENTRY else out.parent.parent
        if name != ENTRY:
            old = reuse['methods'][name]
            assert ref == old['reference'] == read(Path(reuse['source_folder'])/'references.json')[name]
            assert sha(out/'hashes.json') == old['hashes_sha256']
        for frame in ['world', 'local']:
            assert sha(ref[frame+'_path']) == ref[frame+'_sha256']
        x = np.load(ref['world_path'])
        equal_record(ref['safety'], reference_safety(x, env))
        r, m, audit = validate_method(where,name,ref,common,fresh,env['on'],scene,settings)
        rollouts[name], metrics[name], audits[name] = r, m, audit
        primary[name] = None if r is None else plain(execution_metrics(r,m,fresh,cfg['integration_steps']))
        curve = SpatialCurve(fresh)
        mapping = descriptor if name == ENTRY else dict(row_original_arc_m=curve.arc.tolist(),
            row_original_identities=list(range(len(fresh))), row_fractional_original_progress=list(map(float,range(len(fresh)))),
            first_presented_original_arc_m=0.)
        mappings[name] = mapping
        selections[name] = dict(rows=[],first=None,any_identity_before_entry=False) if r is None else selector_progress(r,mapping)
        if r is None:
            continue
        assert r['raw_FRESH_sha256'] == read(folder/'source_manifest.json')['FRESH_sha256']
        restored = read(out/'restoration.json')
        for key in ['B','B_tick','B_sim_s','u_minus','integration_dt_s']:
            assert restored[key] == common[key]
        assert restored['capture_pose'] == common['fresh_capture_pose']
        assert restored['reference_version'] == common['fresh_version']
        assert restored['previous_control'] == common['u_mem_B'] and restored['held_command'] == common['u_B_plus']
        if name == ENTRY:
            assert read(out/'execution_start.json')['freeze_sha'] == read(folder.parents[1]/'execution_start.json')['sha']
            np.testing.assert_array_equal(restored['installed_world'], pre['rows'][0]['installed_world'])
        for w in r['logical_waits']:
            assert w['pose_before'] == w['pose_after'] and w['sim_before'] == w['sim_after']
            assert w['release_tick'] == release[w['submit_tick']]
        for event in r['events']:
            if event.get('type') == 'solve_result' and event.get('status') == 'command':
                assert event['official_generation'] == event['result_generation'] == common['original_generation']
                if event.get('after_termination'):
                    assert event.get('withheld_by_logical_scheduler') and release[event['input_state_id']] >= r['states'][-1]['absolute_tick']
                else:
                    assert event['continuation_seen']['tick'] == release[event['input_state_id']]
        if m is not None:
            assert primary[name]['original_FRESH_endpoint_dwell_s'] == independent_endpoint_time(r,fresh)
            assert primary[name]['first_endpoint_tube_entry_s'] == m['endpoint']['first_pose_tube_entry_s']
            if name != ENTRY:
                equal_record(metric_row(m,r['termination']), historical_source['primary_metrics'][name])
            else:
                own = plain(evaluate(r,x,env['on'],scene,read(folder/'metric_protocol.json')))
                equal_record(own,read(out/'own_reference_metrics.json'))
                secondary[name] = {k:v for k,v in metric_row(own,r['termination']).items() if k in
                    ['position_auc_03_m_s','position_auc_09_m_s','yaw_auc_03_rad_s','yaw_auc_09_rad_s','sustained_attachment_s']}
    assert not selections[ENTRY]['any_identity_before_entry'], 'implementation failure: stale identity before entry'
    gate = schedule_gate(rollouts,frozen,common)
    first_poses = [selections[n]['first']['input_pose'] for n in ORDER if selections[n]['first'] is not None]
    if len(first_poses) == 3:
        assert first_poses[0] == first_poses[1] == first_poses[2]
    r = rollouts[ENTRY]
    paired_gap = {}
    if r is not None:
        for baseline in [NATIVE,FULL]:
            n = min(len(r['states']),len(rollouts[baseline]['states']))
            a,b = [np.asarray([s['pose_world'][:2] for s in rr['states'][:n]]) for rr in [r,rollouts[baseline]]]
            paired_gap[baseline] = float(np.linalg.norm(a-b,axis=1).max())
    summary = dict(entry=entry, primary_metrics=primary, secondary_own_reference_metrics=secondary,
        selector=selections, schedule_gate=gate, reference_safety={n:refs[n]['safety'] for n in ORDER},
        termination={n:refs[n]['status'] if rr is None else rr['termination'] for n,rr in rollouts.items()},
        controller_reference_failure=bool(r is not None and r['termination']=='CONTROLLER_ERROR' and
            any(e.get('type')=='solve_result' and e.get('status')=='controller_error' for e in r['events'])),
        technical_valid=all(a['valid'] for a in audits.values()), method_folders={n:str(method_folder(folder,n)) for n in ORDER},
        first_submit_poses_identical=len(first_poses)==3, max_execution_XY_gap_m=paired_gap,
        historical_Half_context=historical_source['primary_metrics']['HALF_TRANSPORT_LOCAL_SE2'],
        counts=dict(new_rollouts=int(r is not None), MPC_solves=0 if r is None else r['new_MPC_solved'],
                    MPC_results_applied=0 if r is None else sum(e.get('type')=='solve_result' and 'continuation_seen' in e for e in r['events']),
                    optimizer=0,LightNav=0,RGB=0,Isaac=0,retries=0,reused_rollouts=2))
    return plain(summary), dict(valid=summary['technical_valid'],methods=audits)


def validate(run, deep=True):
    verify(run)
    diagnostic, transport = authenticate(read(run/'protocol.json'),deep=deep)
    selected = read(run/'selected_sources.json')
    assert [s['id'] for s in selected] == SOURCE_IDS
    sources, audits = {}, {}
    with no_reconciliation_optimizer():
        for spec in selected:
            sid = spec['id']
            sources[sid], audits[sid] = validate_source(Path(spec['folder']),diagnostic['summary']['sources'][sid],transport['summary']['sources'][sid])
    cross = comparison(sources)
    summary = dict(experiment='SPATIAL_ENTRY_SUFFIX_EXECUTION_01',label=read(run/'protocol.json')['label'],
        starting_sha=read(run/'protocol.json')['starting_sha'],scientific_freeze_sha=read(run/'execution_start.json')['sha'],
        selected_sources=selected,sources=sources,cross_source=cross,classification=cross['classification'],
        counts={k:sum(q['counts'][k] for q in sources.values()) for k in next(iter(sources.values()))['counts']})
    validation = dict(valid=all(a['valid'] for a in audits.values()),sources=audits,
        old_correspondence_transport_multisource_R00_R01_validators=deep,new_scientific_solves=0,
        independent_endpoint_dwell_check=True,original_attachment_evaluator_unchanged=True)
    if (run/'summary.json').exists():
        equal_record(summary,read(run/'summary.json'))
    if (run/'result_hashes.json').exists():
        for p,h in read(run/'result_hashes.json').items():
            assert sha(run/p) == h
        tracked = read(RESULTS/'result_summary.json')
        assert tracked['result_hashes_sha256'] == sha(run/'result_hashes.json') and tracked['summary'] == summary
        for p,h in tracked['table_hashes'].items():
            assert sha(RESULTS/p) == h
        assert sorted(p.name for p in (RESULTS/'figures').iterdir()) == sorted(PNGS)
        manifest = read(RESULTS/'figure_manifest.json')
        assert manifest['summary_sha256'] == sha(run/'summary.json')
        for record in manifest['figures']:
            assert sha(RESULTS/'figures'/record['file']) == record['sha256']
    return summary,validation


def write(run):
    s,v = validate(run)
    save(run/'summary.json',s)
    save(run/'validation.json',v)
    primary, entries, endpoint, selectors = [],[],[],[]
    for sid,q in s['sources'].items():
        entries.append(dict(source_id=sid,**q['entry']))
        for name,row in q['primary_metrics'].items():
            primary.append(dict(source_id=sid,method=name,**({} if row is None else {k:v for k,v in row.items() if k!='endpoint_dwell_trace'})))
            keys=['sustained_attachment_s','attachment_fractional_row','attachment_fractional_arc','remaining_arc_at_attachment_m',
                  'original_FRESH_endpoint_dwell_s','T_post_attach_s','endpoint_error_at_3s_cap_m','first_endpoint_tube_entry_s',
                  'executed_path_length_to_endpoint_dwell_m','mean_abs_commanded_v_before_endpoint_dwell_mps']
            endpoint.append(dict(source_id=sid,method=name,**{k:None if row is None else row[k] for k in keys}))
            selectors.extend(dict(source_id=sid,method=name,**r) for r in q['selector'][name]['rows'])
    for name,rows in [('primary',primary),('entry_reference',entries),('attachment_endpoint_comparison',endpoint),('selector_progress',selectors)]:
        csv_write(RESULTS/(name+'.csv'),rows)
    print(json.dumps(dict(valid=v['valid'],classification=s['classification'],counts=s['counts'])))


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--check-only',action='store_true')
    args=parser.parse_args()
    if args.check_only:
        print(json.dumps(validate(args.run.resolve())[1]))
    else:
        write(args.run.resolve())

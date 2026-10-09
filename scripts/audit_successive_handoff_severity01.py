#!/usr/bin/env python3
"""Authenticate, freeze, then rank saved geometry. No live scientific capabilities."""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.successive_isaac_replay01 import authenticate,authenticate_files,csvrows,jsonlines
from reconciliation.se2 import local_trajectory_to_world,wrap_angle
from reconciliation.continuous_obstacle_reveal_episode01 import evolution
from reconciliation.continuous_obstacle_reveal_exploratory02 import extra_evolution
from reconciliation.successive_handoff_severity01 import (
    HANDOFFS,CONTROLS,DIMENSIONS,EPS,WINDOW,geometry_descriptors,native_context,select_geometry,saved_only_guard)
from continuous_obstacle_reveal_exploratory_geometry02 import environments

NAME='successive_post_reveal_handoff_severity_audit_01'
CONFIG=ROOT/'configs'/f'{NAME}.yaml'
OUT=ROOT/'results'/NAME
DOC=ROOT/'docs'/f'{NAME.upper()}.md'
SCHEMA=['handoff','handoff_index','OLD_chunk','FRESH_chunk','evolution','raw_hash_equal',
'E_star_arc_m','E_star_progress','E_star_segment','E_star_alpha','E_star_world','E_star_nearest_raw_row',
'remaining_original_arc_m','removed_prefix_arc_m','removed_prefix_fraction','B_to_E_gap_m','B_to_E_yaw_gap_rad',
'incoming_old_tangent_rad','fresh_entry_tangent_rad','signed_transition_turn_demand_rad','abs_transition_turn_demand_rad',
'fresh_suffix_final_lateral_m','fresh_suffix_max_abs_lateral_m','fresh_suffix_net_yaw_rad','fresh_suffix_abs_net_yaw_rad',
'fresh_suffix_max_abs_tangent_change_rad','local_polyline_separation_m','existing_endpoint_lateral_delta_m',
'existing_wrapped_net_yaw_delta_rad','existing_max_absolute_local_yaw_difference_deg','max_abs_same_row_local_yaw_difference_rad',
'A_to_B_translation_m','A_to_B_yaw_change_rad','A_to_B_executed_XY_arc_m','original_FRESH_clearance_m','suffix_clearance_m',
'hypothetical_connector_clearance_m','connector_physical_overlap','original_legacy_05_pass','suffix_legacy_05_pass',
'hypothetical_connector_legacy_05_pass','native_position_auc_03_m_s','native_yaw_auc_03_rad_s','native_T_turn50_s',
'native_metric_coverage','native_coverage_s','native_T_turn50_status','native_position_auc_full_m_s','native_yaw_auc_full_rad_s',
'native_T_turn50_full_s','native_T_turn50_full_status','rank_progress','rank_gap','rank_turn','rank_fresh_turn','rank_revision',
'pareto_hard','candidate_role']


def git(*args):
    return subprocess.check_output(['git',*args],cwd=ROOT).decode().strip()


def namespace(run):
    run=Path(run).resolve()
    assert run.parent==ROOT/'data'/NAME
    return run


def load_inputs():
    cfg=yaml.safe_load(CONFIG.read_text());prior=yaml.safe_load((ROOT/cfg['source_config']).read_text())
    assert cfg['handoffs']==HANDOFFS and cfg['stable_controls']==CONTROLS
    assert cfg['turn_near_zero_epsilon_rad']==cfg['incoming_displacement_epsilon_m']==EPS
    assert cfg['native_window_s']==WINDOW and cfg['rank_dimensions']==list(DIMENSIONS)
    from reconciliation.spatial_correspondence_selector import POSITION_SCALE,YAW_SCALE,TIE_ATOL
    assert cfg['position_scale_m']==POSITION_SCALE and np.deg2rad(cfg['yaw_scale_deg'])==YAW_SCALE and cfg['selector_tie_atol']==TIE_ATOL
    previous=read(ROOT/'results/successive_isaac_four_method_comparison_01/freeze_manifest.json')['manifest']['files']
    for module in ['spatial_correspondence_selector.py','spatial_entry_suffix.py','spatial_entry.py']:
        path=ROOT/'src/reconciliation'/module
        assert sha(path)==previous[str(path)], 'frozen entry implementation changed'
    source,ep,auth=authenticate(ROOT,prior)
    pins={}
    for filename in ['handoff_index.json','evolution.json']:
        path=ROOT/'results/successive_native_source_acquisition_02'/filename
        assert subprocess.check_output(['git','show',f"{cfg['source_result_commit']}:{path.relative_to(ROOT)}"],cwd=ROOT)==path.read_bytes()
        pins[str(path)]=sha(path)
    index=read(source/'source_bundle/handoff_index.json')
    assert index==read(ROOT/'results/successive_native_source_acquisition_02/handoff_index.json')
    assert index['actual_handoff_count']==11 and index['usable_post_reveal_handoffs']==10
    assert [Path(x['path']).parent.name for x in index['entries']]==['C0_to_C1',*HANDOFFS]
    authenticate_files({source/'source_bundle'/r['path']:r['sha256'] for r in index['entries']})
    evo=read(ROOT/'results/successive_native_source_acquisition_02/evolution.json')
    states=csvrows(ep/'execution.csv');commands=csvrows(ep/'commands.csv');events=jsonlines(ep/'controller/events.jsonl')
    by_tick={int(s['tick']):s for s in states};cmds={int(c['application_tick']):c for c in commands}
    pose=lambda s:np.array([float(s[k]) for k in ['x','y','yaw']])
    by_time={float(s['sim_time_s']):int(s['tick']) for s in states}
    dt=read(ep/'metadata.json')['resolved_integration_dt_s'];bundles={}
    for i,record in enumerate(index['entries']):
        name=Path(record['path']).parent.name;r=read(source/'source_bundle'/record['path'])
        assert r['old_chunk_id']==f'chunk_{i:03d}' and r['fresh_chunk_id']==f'chunk_{i+1:03d}'
        assert r['intrinsic_waypoint_dt'] is None and not r['reconciliation_computed']
        arrays={}
        for key in ['old_raw_local','fresh_raw_local','old_world','fresh_world']:
            path=ep/r[key]['path'];assert sha(path)==r[key]['sha256'];arrays[key]=np.load(path,allow_pickle=False);arrays[key].setflags(write=False)
        for which in ['old','fresh']:
            np.testing.assert_array_equal(local_trajectory_to_world(r['A_'+which],arrays[which+'_raw_local']),arrays[which+'_world'])
        e=evo[i];assert e['pair']==f'C{i}->C{i+1}' and e['available']
        equal=r['old_raw_local']['sha256']==r['fresh_raw_local']['sha256']
        assert e['raw_hash_equal']==equal
        stable=e['classification'].endswith('_STABLE')
        assert stable==(name in CONTROLS) and stable==equal
        if equal:assert arrays['old_raw_local'].tobytes()==arrays['fresh_raw_local'].tobytes()
        bt=r['first_applied_command']['application_tick'];ot=r['t_obs_fresh']['rendered_state_id']
        np.testing.assert_array_equal(pose(by_tick[bt]),r['B']);np.testing.assert_array_equal(pose(by_tick[bt-1]),r['P'])
        np.testing.assert_array_equal(pose(by_tick[ot]),r['A_fresh'])
        assert float(by_tick[ot]['sim_time_s'])==r['t_obs_fresh']['capture_sim_time_s']
        assert float(by_tick[bt]['sim_time_s'])==r['t_switch_fresh']['sim_time_s']
        assert [float(cmds[bt-1][k]) for k in ['v_mps','omega_radps']]==r['u_minus']
        nxt=[by_time[x['install_context']['t_install']['sim_time_s']] for x in events
             if x.get('status')=='installed' and x['chunk_id']!=r['fresh_chunk_id']
             and x['install_context']['t_install']['sim_time_s']>float(by_tick[bt]['sim_time_s'])]
        end=min(nxt) if nxt else max(by_tick)
        assert all(cmds[t]['chunk_id']==r['fresh_chunk_id'] for t in range(bt,end))
        times=np.array([float(by_tick[t]['sim_time_s'])-float(by_tick[bt]['sim_time_s']) for t in range(bt,end+1)])
        np.testing.assert_allclose(np.diff(times),dt,atol=1e-10,rtol=0)
        bundles[name]=dict(index=i,ready=r,arrays=arrays,evolution=e,B_tick=bt,observation_tick=ot,end_tick=end,
            end_reason='BEFORE_NEXT_INSTALL' if nxt else 'END_OF_SAVED_EPISODE',
            poses=np.array([pose(by_tick[t]) for t in range(bt,end+1)]),times=times,
            observation_to_B_poses=np.array([pose(by_tick[t]) for t in range(ot,bt+1)]))
    assert sum(not b['evolution']['classification'].endswith('_STABLE') for n,b in bundles.items() if n in HANDOFFS)==8
    pins[str(source/'source_bundle/handoff_index.json')]=sha(source/'source_bundle/handoff_index.json')
    auth.update(handoff_index_sha256=pins[str(source/'source_bundle/handoff_index.json')],
        exact_post_reveal_handoffs=HANDOFFS,stable_controls=CONTROLS,tracked_inputs=pins,
        ready_record_hashes={r['path']:r['sha256'] for r in index['entries']},C0_ranking_excluded=True)
    return cfg,source,auth,bundles


def prepare(run):
    cfg,source,auth,_=load_inputs()
    run.mkdir(parents=True,exist_ok=False)
    save(run/'authentication.json',auth);save(run/'protocol.json',cfg);save(run/'output_schema.json',SCHEMA)
    print('Authenticated 10 post-reveal handoffs; 8 EVOLVING, 2 STABLE. No ranked computation.')


def freeze(run):
    cfg,source,auth,_=load_inputs();assert auth==read(run/'authentication.json') and cfg==read(run/'protocol.json')
    files=set(read(ROOT/'results/successive_isaac_four_method_comparison_01/freeze_manifest.json')['manifest']['files'])
    files.update(str(ROOT/p) for p in [str(CONFIG.relative_to(ROOT)),
        'configs/successive_isaac_reconciliation_replay_01.yaml',
        'src/reconciliation/successive_handoff_severity01.py','scripts/audit_successive_handoff_severity01.py',
        'scripts/report_successive_handoff_severity01.py','tests/test_successive_handoff_severity01.py',
        'src/reconciliation/gp_se2_environment.py','scripts/run_join_online02.py',
        'src/reconciliation/continuous_obstacle_reveal_episode01.py','src/reconciliation/continuous_obstacle_reveal_exploratory02.py'])
    (run/'protocol_document.md').write_bytes(DOC.read_bytes())
    scenario=read(source/'scenario.json');geometry_source=Path(scenario['source03_input'])
    export=Path(read(geometry_source)['environment_export'])
    geometry_files=[geometry_source,Path(scenario['triangles_path']),*sorted((export/'geometry/projected').glob('*'))]
    geometry_files += [p for p in [export/'gate_catalog.json',export/'doorway_gates.json'] if p.exists()]
    f=dict(code_hashes={p:sha(p) for p in sorted(files)},input_hashes={str(p):sha(p) for p in run.iterdir() if p.is_file()},
        geometry_hashes={str(p):sha(p) for p in geometry_files if p.is_file()},
        authentication=auth,output_schema=SCHEMA,protocol=cfg,ranked_result_computed=False)
    save(run/'freeze.json',f);save(OUT/'freeze_manifest.json',f)


def verify(run,pushed=False):
    f=read(run/'freeze.json')
    authenticate_files({**f['code_hashes'],**f['input_hashes'],**f['geometry_hashes']})
    assert load_inputs()[2]==f['authentication']
    if pushed:
        assert git('rev-parse','HEAD')==git('ls-remote','origin','refs/heads/main').split()[0]
        for path in f['code_hashes']:
            assert git('rev-parse','HEAD:'+str(Path(path).relative_to(ROOT)))==git('hash-object',path)
        assert git('rev-parse','HEAD:'+str(DOC.relative_to(ROOT)))==git('hash-object',str(run/'protocol_document.md'))
        assert not git('diff','HEAD','--',str(OUT/'freeze_manifest.json'))


def compute(run):
    # Required even on direct import: real ten-source ranking cannot precede freeze.
    assert (run/'audit_start.json').exists(),'pushed freeze required before full result computation'
    cfg,source,auth,bundles=load_inputs();env=environments(source)[1]
    rows=[];details={};native={}
    with saved_only_guard() as counts:
        for name in ['C0_to_C1',*HANDOFFS]:
            b=bundles[name];r=b['ready'];a=b['arrays'];e=b['evolution']
            row,detail=geometry_descriptors(a['fresh_world'],a['fresh_raw_local'],r['A_fresh'],r['B'],r['P'],env)
            assert row['original_FRESH_clearance_m']==e['reference_clearance_m'][1], 'historical checker parity'
            # Existing revision definitions are recomputed only with the historical function/scales.
            prior=evolution(a['old_raw_local'],a['fresh_raw_local'],read(source/'protocol.json')['declaration']['evolution'])
            assert all(prior[k]==e[k] for k in prior)
            pair=[]
            for kind in ['old','fresh']:
                pair.append(dict(A=r['A_'+kind],max_yaw_deg=float(np.degrees(abs(wrap_angle(a[kind+'_raw_local'][:,2])).max()))))
            extra=extra_evolution(*pair);assert all(extra[k]==e[k] for k in extra)
            same_shape=a['old_raw_local'].shape==a['fresh_raw_local'].shape
            row.update(handoff=name,handoff_index=b['index'],OLD_chunk=r['old_chunk_id'],FRESH_chunk=r['fresh_chunk_id'],
                evolution='STABLE' if name in CONTROLS else 'EVOLVING',raw_hash_equal=e['raw_hash_equal'],
                local_polyline_separation_m=e['local_polyline_separation_m'],existing_endpoint_lateral_delta_m=e['endpoint_local_delta'][1],
                existing_wrapped_net_yaw_delta_rad=e['wrapped_net_yaw_delta_rad'],
                existing_max_absolute_local_yaw_difference_deg=e['max_absolute_local_yaw_difference_deg'],
                max_abs_same_row_local_yaw_difference_rad=None if not same_shape else float(abs(wrap_angle(a['fresh_raw_local'][:,2]-a['old_raw_local'][:,2])).max()),
                A_to_B_executed_XY_arc_m=float(np.linalg.norm(np.diff(b['observation_to_B_poses'][:,:2],axis=0),axis=1).sum()))
            detail.update(A=r['A_fresh'],B=r['B'],P=r['P'],FRESH_world=a['fresh_world'].tolist(),
                OLD_world=a['old_world'].tolist(),B_tick=b['B_tick'],observation_tick=b['observation_tick'],end_tick=b['end_tick'],
                end_reason=b['end_reason'],raw_provenance={k:r[k] for k in ['old_raw_local','fresh_raw_local','old_world','fresh_world']},
                clocks={k:r[k] for k in ['t_request_fresh','t_receipt_fresh','t_ready_seen','t_install_fresh','t_switch_fresh']},
                observation_sim_time_s=r['t_obs_fresh']['capture_sim_time_s'],original_evolution=e)
            if name=='C0_to_C1':
                sanity=dict(row=row,entry=detail['entry'])
                prior_entry=read(ROOT/'results/successive_isaac_four_method_comparison_01/freeze_manifest.json')['entry']
                assert detail['entry']==prior_entry and row['E_star_arc_m']==0
                continue
            rows.append(row);details[name]=detail
        # Rank before computing any native performance context. Whitelist is enforced again in select_geometry.
        selection=select_geometry(rows)
        for row in rows:
            name=row['handoff'];b=bundles[name]
            ctx,trace=native_context(b['arrays']['fresh_world'],b['times'],b['poses'],b['ready']['B'],row['fresh_entry_tangent_rad'])
            row.update(ctx);native[name]=trace
            row.update({f'rank_{dim}':None if name not in order else order.index(name)+1 for dim,order in selection['rankings'].items()})
            row['pareto_hard']=name in selection['pareto_hard']
            row['candidate_role']=next((role for role,n in selection['recommended'].items() if n==name),'STABLE_CONTROL' if name in CONTROLS else None)
        assert select_geometry(rows)==selection
    result=dict(experiment=cfg['experiment'],classification='HANDOFF_SEVERITY_AUDIT_COMPLETE',starting_sha=cfg['starting_sha'],
        audit_freeze_sha=read(run/'audit_start.json')['freeze_sha'],source=str(source),authentication=auth,
        rows=rows,selection=selection,stable_controls=CONTROLS,C0_sanity_only=sanity,
        calls=cfg['budget'],guard_counts=counts,method_performance_selection=False,protocol_deviations=[])
    return result,details,native


def run_audit(run):
    verify(run,True)
    save(run/'audit_start.json',dict(freeze_sha=git('rev-parse','HEAD'),operation='saved-only geometry screening'))
    try:
        result,details,native=compute(run)
    except Exception:
        import traceback
        save(OUT/'technical_blocker.json',dict(classification='TECHNICAL_BLOCKED',
            error=traceback.format_exc(),calls=read(run/'protocol.json')['budget'],no_retry=True))
        raise
    save(run/'geometry_details.json',details);save(run/'native_diagnostics.json',native)
    save(OUT/'result_summary.json',result);save(OUT/'rankings.json',result['selection']);save(OUT/'call_accounting.json',result['calls'])
    with (OUT/'severity.csv').open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=SCHEMA,extrasaction='ignore',lineterminator='\n');w.writeheader()
        for row in result['rows']:
            w.writerow({k:json.dumps(row[k],allow_nan=False) if isinstance(row.get(k),(list,dict)) else row.get(k) for k in SCHEMA})
    print(result['classification'],result['selection'])


def validate(run,write=False):
    verify(run);result,details,native=compute(run)
    assert result==read(OUT/'result_summary.json') and details==read(run/'geometry_details.json') and native==read(run/'native_diagnostics.json')
    assert read(OUT/'rankings.json')==result['selection'] and read(OUT/'call_accounting.json')==result['calls']
    with (OUT/'severity.csv').open() as f:rows=list(csv.DictReader(f))
    assert len(rows)==10
    for a,b in zip(rows,result['rows']):
        for k in SCHEMA:
            v=b.get(k);expected='' if v is None else json.dumps(v,allow_nan=False) if isinstance(v,(list,dict)) else str(v)
            assert a[k]==expected,(b['handoff'],k)
    v=dict(valid=True,saved_only=True,source_authentication=True,exact_ten_handoffs=True,
        stable_controls=True,C3_unchanged=True,geometry_only_selection=True,no_new_scientific_calls=True)
    if write:save(OUT/'validation_summary.json',v)
    else:assert read(OUT/'validation_summary.json')==v
    print('Saved-only validation PASS')


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    p.add_argument('--mode',choices=['prepare','freeze','run','validate','verify'],required=True);p.add_argument('--write',action='store_true')
    a=p.parse_args();run=namespace(a.run)
    if a.mode=='run':run_audit(run)
    elif a.mode=='validate':validate(run,a.write)
    else:globals()[a.mode](run)


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Saved-only whole-raw source qualification, dynamic cart and actual B audit."""
import argparse
from copy import deepcopy
from pathlib import Path
import sys
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.obstacle_source_acquisition import geometry,qualify,future_at_pose,moving_boundary,timing_gate
from reconciliation.obstacle_source_online import REPETITIONS,representative,initial_pose
from reconciliation.handoff_delay_attribution import boundary_state
from reconciliation.online_history import history_contract
from reconciliation.join_online02 import guard_check
from run_join_source05 import verify
from run_join_online02 import environments
from analyze_join_online02 import csvread,jsonlines,pose_rows
from validate_obstacle_source_acquisition02 import scheduler_audit
from validate_robotless_online_handoffs import validate_episode,equal_record


def episode(run,eid):
    ep=run/'episodes'/eid;p=read(run/'protocol.json');cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    meta=read(ep/'metadata.json');dt=meta['resolved_integration_dt_s']
    contract,sampler=history_contract((ROOT/cfg['paths']['lightnav_checkout']).resolve(),(ROOT/cfg['paths']['checkpoint_path']).resolve())
    v=validate_episode(ep,run,contract,sampler,require_plots=False,integration_dt_s=dt)
    states=csvread(ep/'execution.csv');commands=csvread(ep/'commands.csv');poses=pose_rows(states)
    ev=jsonlines(ep/'controller/events.jsonl');captures=jsonlines(ep/'capture.jsonl');vis=jsonlines(ep/'visibility.jsonl')
    reveal=read(ep/'obstacle_reveal.json') if (ep/'obstacle_reveal.json').exists() else None
    first=read(ep/'first_post_reveal_observation.json') if (ep/'first_post_reveal_observation.json').exists() else None
    base,on,cart,_=environments(run);scene=read(run/'scenario.json')
    for row in vis:
        expected_present=reveal is not None and row['state_id']>=reveal['state_id']
        assert row['cart_present']==expected_present
        assert row['cart_transform']==scene['prop']['wrapper_matrix_column']
        if not expected_present:assert row['instance']['visible_pixels']==0
        assert row['same_render_product_state']
    guards=jsonlines(ep/'guard.jsonl');safe_min=[]
    for g in guards:
        expected_present=reveal is not None and g['command']['application_state_id']>=reveal['state_id']
        assert g['cart_present']==expected_present
        expected=guard_check(on if expected_present else base,g['start_pose'],g['command'],dt)
        equal_record(expected,{k:g[k] for k in expected},'guard')
        if g['safe']:safe_min.append(g['check']['minimum_clearance_lower_bound_m'])
    for name,h in read(ep/'acquisition_extension_manifest.json')['files'].items():assert sha(ep/name)==h
    requests=[read(q) for q in sorted((ep/'requests').glob('seq_*_metadata.json'))];pred=[r for r in requests if r['kind']=='prediction']
    assert len(pred)<=2,'no second post-reveal retry'
    scheduler=scheduler_audit(ep);stall=max(x['loop_interval_host_s'] for x in jsonlines(ep/'loop.jsonl'))
    records=[]
    for r in pred:
        row=dict(chunk_id=r['chunk_id'],status=r['status'],observation=r['observation'],t_request=r['t_request_host'],t_receipt=r.get('t_ready_host'),client_RTT_s=r.get('client_rtt_s'),stop=None)
        if r.get('raw_local_ref'):
            raw=np.load(ep/r['raw_local_ref']['path']);world=np.load(ep/r['world_ref']['path'])
            assert np.array_equal(raw,np.asarray(r['response']['actions']['actions'],float))
            if len(raw):
                row.update(geometry(raw,r['observation']['pose_world'],base,on,scene));np.testing.assert_allclose(world,row['world'],rtol=0,atol=1e-12)
            row.update(stop=bool(r['response'].get('stop')),raw_local_ref=r['raw_local_ref'],world_ref=r['world_ref'],raw_text=r['response'].get('raw_text'),pointing=r['response'].get('pointing'))
        records.append(row)
    contexts=[read(q) for q in sorted((ep/'handoffs').glob('*/context.json'))]
    ctx=next((c for c in contexts if c['fresh_chunk_id']=='chunk_001'),None)
    old=next((r for r in records if r['chunk_id']=='chunk_000'),None);fresh=next((r for r in records if r['chunk_id']=='chunk_001'),None)
    b=None;mem=None;travel=None;bcheck=None;future=None;q=None;timecheck=None
    if ctx and ctx.get('switch_state_id') is not None:
        b=boundary_state(ctx,states,commands,ev,'DELAYED');mem=b['memory']
        i,j=ctx['obs_state_id'],ctx['switch_state_id'];travel=float(np.linalg.norm(np.diff(poses[i:j+1,:2],axis=0),axis=1).sum())
        bcheck=on.check_polyline([b['pose_world']])
        if fresh and 'world' in fresh:future=future_at_pose(fresh['world'],b['pose_world'])
    if old and fresh and 'world' in old and 'world' in fresh:
        used=deepcopy(fresh)
        if future is not None:used['future_proxy']=future
        q=qualify(old,used,scene,p['gates']);q['actual_B_qualification_pending']=False
    timing=next((r for r in scheduler['requests'] if r['chunk_id']=='chunk_001'),None)
    timecheck=timing_gate([None if timing is None else timing['request_local_RTF']],stall,1)
    old_pre=bool(old and reveal and old['observation']['rendered_state_id']<reveal['state_id'] and reveal['old_chunk_id']=='chunk_000')
    fresh_post=bool(fresh and first and fresh['observation']['frame_id']==first['frame_id'] and fresh['observation']['rendered_state_id']>=reveal['state_id'])
    old_commands=[] if not fresh or not b else commands[int(fresh['observation']['rendered_state_id']):b['state_id']]
    old_continued=bool(old_commands and all(c['chunk_id']=='chunk_000' for c in old_commands))
    visible=next((x['instance']['visible_pixels'] for x in vis if fresh and x['frame_id']==fresh['observation']['frame_id']),None)
    flags=dict(raw_behavior=bool(q and q['qualified']),OLD_pre_reveal=old_pre,FRESH_first_post_reveal=fresh_post,
        OLD_continued=old_continued,cart_visible=visible is not None and visible>=20,
        moving_B=bool(b and b['u_minus'][0]>.20 and travel>=.02),B_clearance=bool(bcheck and bcheck['clearance_valid']),
        editable_future=bool(future and future['remaining_rows']>=4 and future['remaining_arc_m']>=.60),
        controller_memory=bool(mem and mem['available']),timing=timecheck['qualified'],scheduler=scheduler['valid'])
    if reveal:
        t=reveal['state_id'];assert t%15==0
        earlier=[r for r in captures if r['rendered_state_id']<t and r['capture_sim_time_s']>=read(ep/'bootstrap.json')['t_switch']['sim_time_s']]
        assert all((np.asarray(r['pose_world'])[:2]-np.asarray(p['reveal_plane_pose_world'])[:2])@np.asarray(p['hallway_forward'])<0 for r in earlier)
    abort=read(ep/'guard_abort.json') if (ep/'guard_abort.json').exists() else None
    submitted={r['solve_id'] for r in ev if r.get('status')=='submitted'};solved={r['solve_id']:r for r in ev if r.get('type')=='solve_result'}
    return dict(episode_id=eid,qualified=all(flags.values()),gates=flags,failure_reasons=[k for k,x in flags.items() if not x],
        raw_behavior=q,OLD=old,FRESH=fresh,reveal=reveal,first_post_reveal=first,cart_pixels=visible,
        B_state=b,observation_B_travel_m=travel,B_clearance=bcheck,remaining_future=future,
        timing=timecheck,request_diagnostic=timing,context=ctx,scheduler=scheduler,original_validation=v,
        applied_first_FRESH=None if ctx is None else ctx.get('first_fresh_command'),
        first_FRESH_solve=None if ctx is None else ctx.get('first_fresh_solve'),
        latest_completed_solve_at_B=None if not b else next((r.get('solve_id') for r in reversed(ev) if r.get('type')=='solve_result' and r.get('seen_in_isaac',{}).get('host_monotonic_s',float('inf'))<=ctx['t_switch']['host_monotonic_s']),None),
        actual_prefix_minimum_clearance_lower_bound_m=min(safe_min,default=None),safety_abort=abort,
        termination=meta['status'],complete_bypass=False if not fresh else fresh.get('complete_bypass_descriptor',False),
        calls=dict(terminal_predictions=len(pred),buffers=len(requests)-len(pred),MPC_submissions=len(submitted),MPC_saved_results=len(solved),missing_MPC_results=sorted(submitted-solved.keys()),saved_MPC_wall_s=sum(r.get('official_solve_ms',0.) for r in solved.values())/1000,
            terminal_RTT_sum_s=sum(r.get('client_rtt_s') or 0. for r in pred),GP_rigid_reconciliation=0),
        raw_source_hashes={str(q.relative_to(ep)):sha(q) for q in ep.rglob('*') if q.is_file()})


def validate(parent):
    verify(parent);run=parent/'phaseB';verify(run)
    assert read(parent/'phase0/qualification.json')['qualified']
    assert sorted(p.name for p in (run/'episodes').iterdir())==list(REPETITIONS)
    protocol=read(run/'protocol.json');assert protocol['candidate']=='POSE11'
    assert protocol['initial_pose_world']==initial_pose(read(parent/'selected_pose11.json')['pose_world'],read(parent/'scenario.json')['forward_xy'])
    assert protocol['instruction']==read(parent/'protocol.json')['instruction']
    for k in ['camera','lighting','capture','execution','lightnav']:
        assert yaml.safe_load((run/'config_snapshot.yaml').read_text())[k]==yaml.safe_load((parent/'config_snapshot.yaml').read_text())[k]
    rows=[episode(run,e) for e in REPETITIONS];rep=representative(rows);count=sum(r['qualified'] for r in rows)
    sessions=[read(run/'episodes'/e/'session_open.json')['connection_id'] for e in REPETITIONS];assert len(set(sessions))==2
    classification='QUALIFIED_GENUINE_OBSTACLE_HANDOFF_SOURCE' if count==2 else 'ONLINE_POSE11_SOURCE_PARTIALLY_REPLICATED' if count==1 else 'ONLINE_POSE11_SOURCE_NOT_QUALIFIED'
    return dict(valid=True,rows=rows,representative=rep,qualified_count=count,classification=classification,
        independent_sessions=True,same_instruction_geometry=True,validation_new_model_MPC_optimizer_calls=0)


def bundle(parent,result):
    root=parent/'source_bundle';root.mkdir(exist_ok=False);entries=[]
    for r in result['rows']:
        if not r['qualified']:continue
        ep=parent/'phaseB/episodes'/r['episode_id'];out=root/r['episode_id'];(out/'raw').mkdir(parents=True);(out/'derived').mkdir()
        for label,record in [('old',r['OLD']),('fresh',r['FRESH'])]:
            for key,dst in [('raw_local_ref',out/'raw'/(label+'_lightnav.npy')),('world_ref',out/'derived'/(label+'_world.npy'))]:
                with dst.open('xb') as f:f.write((ep/record[key]['path']).read_bytes())
            src=ep/'chunks'/record['chunk_id']/'response.json'
            with (out/'raw'/(label+'_response.json')).open('xb') as f:f.write(src.read_bytes())
            with (out/'raw'/(label+'_observation.jpg')).open('xb') as f:f.write((ep/record['observation']['path']).read_bytes())
        save(out/'raw/history_manifest.json',dict(captures=jsonlines(ep/'capture.jsonl'),wire=jsonlines(ep/'requests/wire.jsonl'),source_episode=str(ep)))
        save(out/'state_and_timing.json',{k:r[k] for k in ['B_state','observation_B_travel_m','context','first_FRESH_solve','applied_first_FRESH','latest_completed_solve_at_B','timing','remaining_future','safety_abort']})
        save(out/'environment_and_provenance.json',dict(scene=read(parent/'phaseB/acquisition_scene.json'),protocol=read(parent/'phaseB/protocol.json'),execution=read(parent/'phaseB/execution_start.json'),server=read(parent/'server_launch.json'),workers=read(parent/'phaseB/workers.json')))
        save(out/'qualification.json',dict(qualified=True,gates=r['gates'],complete_bypass=r['complete_bypass'],scope='genuine obstacle-induced local avoidance moving-B handoff; no reconciliation'))
        save(out/'hashes.json',{str(q.relative_to(out)):sha(q) for q in out.rglob('*') if q.is_file()});entries.append(str(out))
    save(root/'manifest.json',dict(representative=result['representative'],qualified_count=result['qualified_count'],entries=entries,classification=result['classification'],no_next_stage=True))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();parent=a.run.resolve()
    r=validate(parent);save(parent/'phaseB/validation.json',r);bundle(parent,r)
    print(__import__('json').dumps({k:r[k] for k in ('valid','representative','qualified_count','classification')},indent=2))

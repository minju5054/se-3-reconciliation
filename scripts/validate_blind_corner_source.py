#!/usr/bin/env python3
"""Saved-only, direct-geometry source validator; no inference/controller calls."""
import argparse
from pathlib import Path
import sys
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.blind_corner_source import FirstCrossing,old_turning,motion_gate,mismatch,turn_descriptor,select_representative,INSTRUCTION
from reconciliation.obstacle_source_acquisition import geometry,future_at_pose,timing_gate
from reconciliation.handoff_delay_attribution import boundary_state
from reconciliation.online_history import history_contract
from reconciliation.join_online02 import guard_check
from run_join_source05 import verify
from run_join_online02 import environments
from run_blind_corner_source import audit_geometry
from analyze_join_online02 import csvread,jsonlines,pose_rows
from validate_obstacle_source_acquisition02 import scheduler_audit
from validate_robotless_online_handoffs import validate_episode,equal_record


def episode(parent,cid):
    run=parent/'candidates'/cid;ep=run/'episodes'/cid;cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    meta=read(ep/'metadata.json');dt=meta['resolved_integration_dt_s'];assert cfg['join_online02']['instruction']==INSTRUCTION
    contract,sampler=history_contract((ROOT/cfg['paths']['lightnav_checkout']).resolve(),(ROOT/cfg['paths']['checkpoint_path']).resolve())
    original=validate_episode(ep,run,contract,sampler,require_plots=False,integration_dt_s=dt)
    assert original['valid']
    base,on,cart,_=environments(run);scenario=read(run/'scenario.json')
    states=csvread(ep/'execution.csv');poses=pose_rows(states);commands=csvread(ep/'commands.csv');events=jsonlines(ep/'controller/events.jsonl')
    captures=jsonlines(ep/'capture.jsonl');vis=jsonlines(ep/'visibility.jsonl');guards=jsonlines(ep/'guard.jsonl')
    initial=read(run/'static_initial.json')['state'];final=read(run/'static_final.json');assert initial==final['state'] and final['exact_mesh_unchanged']
    assert initial['oracle_present'] and initial['logical_present'] and initial['authored_visibility']=='inherited'
    latch=FirstCrossing();assert len(vis)==len(captures)
    for row,cap in zip(vis,captures):
        assert row['frame_id']==cap['frame_id'] and row['state_id']==cap['rendered_state_id'] and row['pose_world']==cap['pose_world'] and row['camera']==cap['camera']
        assert row['cart_present'] and row['cart_transform']==scenario['prop']['wrapper_matrix_column'] and row['same_render_product_state']
        assert sha(ep/row['mask_path'])==row['mask_sha256']
        mask=np.load(ep/row['mask_path'])['mask'];ids=row['instance']['matched_instance_ids'];pixels=int(np.isin(mask,ids).sum())
        assert pixels==row['instance']['visible_pixels']
        latch.observe(row['frame_id'],pixels,row['state_id'])
    cross=read(ep/'first_visibility_crossing.json') if (ep/'first_visibility_crossing.json').exists() else None
    assert (cross is None)==(latch.crossing is None)
    if cross:assert {k:cross[k] for k in ['previous','current']}==latch.crossing
    safe=[]
    for g in guards:
        assert g['cart_present'] and g['static_obstacle_state']==initial
        expected=guard_check(on,g['start_pose'],g['command'],dt);equal_record(expected,{k:g[k] for k in expected},'static guard')
        if g['safe']:safe.append(g['check']['minimum_clearance_lower_bound_m'])
    for f,h in read(ep/'acquisition_extension_manifest.json')['files'].items():assert sha(ep/f)==h
    req=[read(q) for q in sorted((ep/'requests').glob('seq_*_metadata.json'))];pred=[r for r in req if r['kind']=='prediction']
    assert len(pred)<=2 and len(req)<=64
    records=[]
    for r in pred:
        row=dict(chunk_id=r['chunk_id'],status=r['status'],observation=r['observation'],request=r['t_request_host'],receipt=r.get('t_ready_host'),client_RTT_s=r.get('client_rtt_s'),stop=None)
        if r.get('raw_local_ref'):
            raw=np.load(ep/r['raw_local_ref']['path']);world=np.load(ep/r['world_ref']['path'])
            np.testing.assert_array_equal(raw,np.asarray(r['response']['actions']['actions'],float))
            if len(raw):
                row.update(geometry(raw,r['observation']['pose_world'],base,on,scenario));np.testing.assert_allclose(world,row['world'],rtol=0,atol=1e-12)
            row.update(stop=bool(r['response'].get('stop')),raw_local_ref=r['raw_local_ref'],world_ref=r['world_ref'],raw_text=r['response'].get('raw_text'),pointing=r['response'].get('pointing'))
        records.append(row)
    old=next((r for r in records if r['chunk_id']=='chunk_000'),None);fresh=next((r for r in records if r['chunk_id']=='chunk_001'),None)
    if fresh:assert cross and fresh['observation']['frame_id']==cross['current']['frame_id'],'later substitution forbidden'
    oldpixels=next((r['instance']['visible_pixels'] for r in vis if old and r['frame_id']==old['observation']['frame_id']),None)
    contexts=[read(q) for q in sorted((ep/'handoffs').glob('*/context.json'))];ctx=next((c for c in contexts if c['fresh_chunk_id']=='chunk_001'),None)
    b=travel=bc=future=physical_obs=recent_yaw=None
    if ctx and ctx.get('switch_state_id') is not None:
        b=boundary_state(ctx,states,commands,events,'DELAYED');i,j=ctx['obs_state_id'],ctx['switch_state_id']
        travel=float(np.linalg.norm(np.diff(poses[i:j+1,:2],axis=0),axis=1).sum());bc=on.check_polyline([b['pose_world']]);future=future_at_pose(fresh['world'],b['pose_world']) if fresh and 'world' in fresh else None
        before=commands[max(0,i-1)];physical_obs=[float(before['v_mps']),float(before['omega_radps'])]
        recent_yaw=float(np.unwrap(poses[max(0,i-30):i+1,2])[-1]-np.unwrap(poses[max(0,i-30):i+1,2])[0])
    turn=old_turning(old['raw_local']) if old and 'raw_local' in old else None
    diff=mismatch(old['world'],fresh['world'],scenario) if old and fresh and 'world' in old and 'world' in fresh else None
    descriptor=turn_descriptor(old['raw_local'],fresh['raw_local']) if diff else None
    schedule=scheduler_audit(ep);stall=max(r['loop_interval_host_s'] for r in jsonlines(ep/'loop.jsonl'))
    reqtime=next((r for r in schedule['requests'] if r['chunk_id']=='chunk_001'),None)
    timing=timing_gate([None if reqtime is None else reqtime['request_local_RTF']],stall,1)
    overlap=[] if not b or not fresh else commands[int(fresh['observation']['rendered_state_id']):b['state_id']]
    bootstrap=read(ep/'bootstrap.json') if (ep/'bootstrap.json').exists() else None
    abort=read(ep/'guard_abort.json') if (ep/'guard_abort.json').exists() else None
    flags=dict(artifact_integrity=original['valid'],static_cart=True,OLD_hidden=oldpixels is not None and oldpixels<20,
        OLD_nonSTOP=bool(old and old.get('stop') is False),OLD_turning=bool(turn and turn['qualified']),
        OLD_base_safe=bool(old and old.get('geometry_off',{}).get('clearance_valid')),
        finite_OLD_cart_obstruction=bool(old and old.get('geometry_on',{}).get('minimum_clearance_m',np.inf)<.05 and old.get('geometry_off',{}).get('clearance_valid')),
        first_crossing_FRESH=bool(fresh and cross and fresh['observation']['frame_id']==cross['current']['frame_id']),
        OLD_executes_during_FRESH=bool(overlap and all(r['chunk_id']=='chunk_000' for r in overlap)),
        FRESH_nonSTOP=bool(fresh and fresh.get('stop') is False),FRESH_whole_safe=bool(fresh and fresh.get('geometry_on',{}).get('clearance_valid')),
        FRESH_progress_arc=bool(fresh and fresh.get('forward_progress_m',0)>0 and fresh.get('arc_m',0)>=.60),meaningful_change=bool(diff and diff['meaningful']),
        moving_rotating_B=bool(b and bc and motion_gate(b['u_minus'],travel,bc['minimum_clearance_m'])),
        memory_available=bool(b and b['memory']['available']),
        future_remaining=bool(future and future['remaining_rows']>=4 and future['remaining_arc_m']>=.60),
        safe_prefix_no_abort=bool(abort is None and safe and min(safe)>=.05),timing=timing['qualified'],scheduler=schedule['valid'])
    solved={r['solve_id']:r for r in events if r.get('type')=='solve_result'};submitted={r['solve_id'] for r in events if r.get('status')=='submitted'}
    return dict(candidate_id=cid,qualified=all(flags.values()),gates=flags,failure_reasons=[k for k,v in flags.items() if not v],
        OLD=old,FRESH=fresh,OLD_turn=turn,OLD_pixels=oldpixels,crossing=cross,mismatch=diff,FRESH_turn_descriptor=descriptor,
        B=b,observation_B_travel_m=travel,B_clearance=bc,remaining_future=future,physical_at_FRESH_observation=physical_obs,recent_actual_yaw_change_rad=recent_yaw,
        context=ctx,bootstrap=bootstrap,timing=timing,request_timing=reqtime,scheduler=schedule,termination=meta['status'],safety_abort=abort,
        minimum_actual_prefix_clearance_m=min(safe,default=None),original_validation=original,
        calls=dict(terminal_predictions=len(pred),buffer_requests=len(req)-len(pred),MPC_submissions=len(submitted),MPC_results=len(solved),
            MPC_wall_s=sum(r.get('official_solve_ms',0) for r in solved.values())/1000,terminal_RTT_s=sum(r.get('client_rtt_s') or 0 for r in pred),optimization=0),
        raw_source_hashes={str(p.relative_to(ep)):sha(p) for p in ep.rglob('*') if p.is_file()})


def validate(run):
    verify(run);g=audit_geometry();equal_record(g,read(run/'geometry_qualification.json'),'geometry')
    order=read(run/'protocol.json')['eligible_order'];assert order==g['eligible'];rows=[episode(run,c) for c in order]
    sessions=[read(run/'candidates'/c/'episodes'/c/'session_open.json')['connection_id'] for c in order];assert len(set(sessions))==len(order)
    rep=select_representative(rows,order)
    classification='QUALIFIED_BLIND_CORNER_OBSTACLE_HANDOFF_SOURCE' if rep else ('BLIND_CORNER_LIGHTNAV_SOURCE_NOT_QUALIFIED' if order else 'BLIND_CORNER_GEOMETRY_UNAVAILABLE')
    return dict(valid=True,geometry=g,rows=rows,representative=rep,classification=classification,validation_model_MPC_optimizer_calls=0,
        initial_sha=read(run/'source.json')['starting_sha'],scientific_sha=read(run/'candidates'/order[0]/'execution_start.json')['sha'] if order else None)


def bundle(run,result):
    entries=[]
    for row in result['rows']:
        if not row['qualified']:continue
        cid=row['candidate_id'];ep=run/'candidates'/cid/'episodes'/cid;out=run/'source_bundle'/cid
        for label in ['OLD','FRESH']:
            r=row[label]
            for key,rel in [('raw_local_ref',f'raw/{label}_local.npy'),('world_ref',f'derived/{label}_world.npy')]:
                dst=out/rel;dst.parent.mkdir(parents=True,exist_ok=True)
                with dst.open('xb') as f:f.write((ep/r[key]['path']).read_bytes())
        save(out/'source_references.json',dict(episode=str(ep),qualification=row,protocol=read(run/'protocol.json'),scene=read(run/'candidates'/cid/'acquisition_scene.json'),hashes=row['raw_source_hashes']))
        entries.append(str(out))
    save(run/'source_bundle/manifest.json',dict(entries=entries,representative=result['representative'],classification=result['classification'],no_next_stage=True))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();r=validate(a.run.resolve());save(a.run/'validation.json',r);bundle(a.run.resolve(),r)
    print({k:r[k] for k in ['valid','representative','classification']})

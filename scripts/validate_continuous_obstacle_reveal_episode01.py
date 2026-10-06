#!/usr/bin/env python3
"""Independent saved-only source, timing, safety and continuous lineage audit."""
import argparse
from copy import deepcopy
from pathlib import Path
import sys
import numpy as np
import yaml
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
from reconciliation.join_source03 import read, save, sha
from reconciliation.continuous_obstacle_reveal_episode01 import CHUNKS, EPISODES, FIGURES, evolution, classification, separation
from reconciliation.obstacle_source_acquisition import geometry, qualify, future_at_pose, timing_gate
from reconciliation.join_source02 import whole_raw_polyline_check
from reconciliation.handoff_delay_attribution import boundary_state
from reconciliation.online_history import history_contract
from reconciliation.join_online02 import guard_check
from reconciliation.se2 import wrap_angle
from run_continuous_obstacle_reveal_episode01 import verify
from run_join_online02 import environments
from analyze_join_online02 import csvread, jsonlines, pose_rows
from validate_obstacle_source_acquisition02 import scheduler_audit
from validate_robotless_online_handoffs import validate_episode, equal_record


def validate(run):
    verify(run)
    source = Path(read(run/'source.json')['OSA03'])
    cfg = yaml.safe_load((run/'config_snapshot.yaml').read_text())
    oldcfg = yaml.safe_load((source/'config_snapshot.yaml').read_text())
    cmp = deepcopy(cfg); cmp['online']['maximum_handoff_attempts'] = 1
    assert cmp == oldcfg, 'only terminal budget differs from historical runtime config'
    p = read(run/'protocol.json'); declaration = p['declaration']
    assert p['initial_pose_world'] == read(source/'selected_pose11.json')['pose_world']
    assert p['instruction'] == read(source/'protocol.json')['instruction']
    assert sorted(q.name for q in (run/'episodes').iterdir()) == EPISODES
    ep = run/'episodes'/EPISODES[0]
    meta = read(ep/'metadata.json'); dt = meta['resolved_integration_dt_s']
    contract, sampler = history_contract((ROOT/cfg['paths']['lightnav_checkout']).resolve(), (ROOT/cfg['paths']['checkpoint_path']).resolve())
    original = validate_episode(ep, run, contract, sampler, require_plots=False, integration_dt_s=dt)
    for name, h in read(ep/'acquisition_extension_manifest.json')['files'].items():
        assert sha(ep/name) == h, name
    assert read(run/'generation_actual.json')['valid']
    states = csvread(ep/'execution.csv'); commands = csvread(ep/'commands.csv'); poses = pose_rows(states)
    events = jsonlines(ep/'controller/events.jsonl'); captures = jsonlines(ep/'capture.jsonl')
    visibility = jsonlines(ep/'visibility.jsonl'); guards = jsonlines(ep/'guard.jsonl')
    requests = [read(q) for q in sorted((ep/'requests').glob('seq_*_metadata.json'))]
    pred = [r for r in requests if r['kind'] == 'prediction']
    assert [r['chunk_id'] for r in pred] == CHUNKS[:len(pred)] and len(pred) <= 4
    assert meta['predictions_requested'] == len(pred)
    contexts = [read(q) for q in sorted((ep/'handoffs').glob('*/context.json'))]
    bootstrap = read(ep/'bootstrap.json') if (ep/'bootstrap.json').exists() else None
    by_context = {c['fresh_chunk_id']:c for c in ([bootstrap] if bootstrap else [])+contexts}
    applied = [by_context[c] for c in CHUNKS if c in by_context and by_context[c].get('switch_state_id') is not None]
    assert [c['fresh_chunk_id'] for c in applied] == CHUNKS[:len(applied)]
    assert [c['fresh_reference_version'] for c in applied] == list(range(len(applied)))
    np.testing.assert_array_equal(poses[0], p['initial_pose_world'])
    if applied:
        sid = applied[0]['switch_state_id']
        np.testing.assert_array_equal(poses[:sid+1], np.tile(p['initial_pose_world'], (sid+1,1)))
        assert all(float(c['v_mps']) == float(c['omega_radps']) == 0 for c in commands[:sid])
    reveal = read(ep/'obstacle_reveal.json') if (ep/'obstacle_reveal.json').exists() else None
    base, on, cart, _ = environments(run); scene = read(run/'scenario.json')
    if reveal:
        expected = next(f for f in captures if f['capture_sim_time_s'] > bootstrap['t_switch']['sim_time_s'])
        assert expected['rendered_state_id'] == reveal['state_id']
        assert reveal['rendering_present'] and reveal['oracle_present'] and not reveal['cart_pose_changed']
        assert reveal['cart_transform'] == scene['prop']['wrapper_matrix_column']
    for v in visibility:
        present = bool(reveal and v['state_id'] >= reveal['state_id'])
        assert v['cart_present'] == present
        assert v['cart_transform'] == scene['prop']['wrapper_matrix_column']
        assert v['same_render_product_state']
        assert sha(ep/v['mask_path']) == v['mask_sha256']
        mask = np.load(ep/v['mask_path'])['mask']
        assert int(np.isin(mask,v['instance']['matched_instance_ids']).sum()) == v['instance']['visible_pixels']
        if not present: assert v['instance']['visible_pixels'] == 0
        actual_cart = read(ep/'cart_states'/f'{v["frame_id"]}.json')
        assert actual_cart['renderer_present'] == actual_cart['oracle_present'] == present
        np.testing.assert_allclose(actual_cart['actual_world_matrix_column'],scene['prop']['wrapper_matrix_column'],atol=1e-12,rtol=0)
    safe_min = []
    for i, g in enumerate(guards):
        present = bool(reveal and g['command']['application_state_id'] >= reveal['state_id'])
        assert g['cart_present'] == present
        expected = guard_check(on if present else base, g['start_pose'], g['command'], dt)
        equal_record(expected, {k:g[k] for k in expected}, 'guard')
        if g['safe']:
            safe_min.append(g['check']['minimum_clearance_lower_bound_m'])
        else:
            assert i == len(commands) and i == len(guards)-1
            assert not read(ep/'guard_abort.json')['command_applied']
    assert len(safe_min) == len(commands)
    scheduler = scheduler_audit(ep)
    stall = max(r['loop_interval_host_s'] for r in jsonlines(ep/'loop.jsonl'))
    records = []
    from shapely.geometry import LineString, Point
    for cid in CHUNKS:
        r = next((r for r in pred if r['chunk_id'] == cid), None)
        if r is None:
            records.append(dict(chunk_id=cid, generated=False, applied=False)); continue
        obs = r['observation']; ctx = by_context.get(cid)
        vis = next(v for v in visibility if v['frame_id'] == obs['frame_id'])
        i = CHUNKS.index(cid)
        if i:
            prev = by_context[CHUNKS[i-1]]
            first = next(f for f in captures if f['capture_sim_time_s'] > prev['t_switch']['sim_time_s'])
            assert obs['frame_id'] == first['frame_id'], 'exact first eligible frame, never queued substitute'
            eligibility = read(ep/'eligible_observations'/f'{cid}.json')
            assert eligibility['frame']['frame_id'] == obs['frame_id'] and eligibility['terminal_inflight'] is None
            assert eligibility['cart_present']
            if i == 1: assert obs['rendered_state_id'] == reveal['state_id']
        else:
            assert obs['rendered_state_id'] == 45 and not vis['cart_present']
            np.testing.assert_array_equal(obs['pose_world'], p['initial_pose_world'])
        timing = next((q for q in scheduler['requests'] if q['chunk_id'] == cid), None)
        row = dict(chunk_id=cid, generated=True, applied=bool(ctx and ctx.get('switch_state_id') is not None),
            model_status=r['status'], stop=r.get('stop'), observation=obs,
            t_request_host=r['t_request_host'], t_receipt_host=r.get('t_ready_host'),
            client_RTT_s=r.get('client_rtt_s'), request_timing=timing,
            A=obs['pose_world'], ready_pose=None if not ctx else ctx['R_ready'],
            install_pose=None if not ctx or not ctx.get('t_install') else ctx['R_ready'],
            t_ready_seen_sim=None if not ctx else ctx['t_ready_seen_sim'],
            t_install=None if not ctx else ctx['t_install'],
            cart_visible_pixels=vis['instance']['visible_pixels'], cart_present=vis['cart_present'],
            cart_transform=vis['cart_transform'], cart_state_sha256=sha(run/'scenario.json'),
            physical_command_before_observation=commands[obs['rendered_state_id']-1] if obs['rendered_state_id'] else None,
            raw_local_ref=r.get('raw_local_ref'), world_ref=r.get('world_ref'),
            request_metadata_path=str(next(q for q in sorted((ep/'requests').glob('seq_*_metadata.json')) if read(q)['seq'] == r['seq'])),
            history_ref=r.get('history_snapshot'), response_ref=r.get('response_ref'), intrinsic_waypoint_dt=None)
        if r.get('world_ref'):
            raw, world = np.load(ep/r['raw_local_ref']['path']), np.load(ep/r['world_ref']['path'])
            if len(raw):
                row.update(geometry(raw, obs['pose_world'], base, on, scene))
                np.testing.assert_allclose(row['world'], world, atol=1e-12, rtol=0)
                line = LineString(world[:,:2]) if len(world)>1 else Point(world[0,:2])
                row.update(endpoint_local=raw[-1].tolist(), net_yaw_rad=float(wrap_angle(raw[-1,2]-raw[0,2])),
                    cart_only_edge_clearance_m=float(cart.distance(line)-.20-on.numerical_tolerance_m),
                    raw_reference_safe=(row['geometry_on'] if i else row['geometry_off'])['clearance_valid'])
                checkpath = ep/'reference_checks'/f'{cid}.json'
                if checkpath.exists():
                    check = read(checkpath)
                    equal_record(check['check'], whole_raw_polyline_check(world, on if i else base))
                    assert check['before_install'] and check['cart_present'] == bool(i)
                    if not check['check']['clearance_valid']: assert not row['applied']
                if row['applied']: assert checkpath.exists() and row['raw_reference_safe']
        if row['applied']:
            b = boundary_state(ctx, states, commands, events, 'DELAYED')
            sid = ctx['switch_state_id']; oid = ctx['obs_state_id']
            assert commands[sid]['chunk_id'] == cid
            assert not any(c['chunk_id'] == cid for c in commands[:sid])
            if i: assert all(c['chunk_id'] == CHUNKS[i-1] for c in commands[oid:sid])
            assert ctx['t_obs']['sim_time_s'] <= ctx['t_ready_seen_sim']['sim_time_s'] <= ctx['t_install']['sim_time_s'] <= ctx['t_switch']['sim_time_s']
            row.update(B=ctx['B'], P=ctx['P'], switch_state_id=sid, t_application=ctx['t_switch'],
                B_state=b, B_clearance=(on if i else base).check_polyline([ctx['B']]),
                observation_B_travel_m=float(np.linalg.norm(np.diff(poses[oid:sid+1,:2], axis=0), axis=1).sum()),
                first_applied_command=ctx['first_fresh_command'], first_solve=ctx['first_fresh_solve'],
                remaining_future=future_at_pose(row['world'], ctx['B']), context=ctx)
        records.append(row)
    first_flags = {}; q = None
    a, b = records[:2]
    if a.get('world') and b.get('world'):
        used = deepcopy(b)
        if b['applied']: used['future_proxy'] = b['remaining_future']
        q = qualify(a, used, scene, p['gates'])
        q['actual_B_qualification_pending'] = not b['applied']
    first_flags = dict(raw_behavior=bool(q and q['qualified']), revealed=bool(reveal),
        C0_applied=a['applied'], C1_applied=b['applied'],
        cart_visible=b.get('cart_visible_pixels',0)>=20,
        moving_B=bool(b['applied'] and b['B_state']['u_minus'][0]>.20 and b['observation_B_travel_m']>=.02),
        B_safe=bool(b['applied'] and b['B_clearance']['clearance_valid']),
        memory_available=bool(b['applied'] and b['B_state']['memory']['available']),
        timing=timing_gate([None if not b.get('request_timing') else b['request_timing']['request_local_RTF']], stall, 1)['qualified'],
        scheduler=scheduler['valid'])
    all_timing = timing_gate([r['request_timing']['request_local_RTF'] for r in records if r.get('request_timing')], stall, 4)
    successive = (len(applied)==4 and all(r.get('raw_reference_safe') and r['B_clearance']['clearance_valid'] for r in records[1:])
        and all_timing['qualified'] and scheduler['valid'] and not (ep/'guard_abort.json').exists())
    if len(applied)==4 and meta['status']=='ATTEMPT_LIMIT':
        assert abs(float(states[-1]['sim_time_s'])-applied[-1]['t_switch']['sim_time_s']-6*dt)<1e-9
    pairs = []
    for i in (1,2):
        a, b = records[i:i+2]
        if not a.get('raw_local') or not b.get('raw_local'):
            pairs.append(dict(pair=f'C{i}->C{i+1}', available=False)); continue
        e = evolution(a['raw_local'], b['raw_local'], declaration['evolution'])
        d = np.asarray(b['A'])-a['A']; d[2] = wrap_angle(d[2])
        pairs.append(dict(pair=f'C{i}->C{i+1}', available=True, **e,
            raw_hash_equal=a['raw_local_ref']['sha256']==b['raw_local_ref']['sha256'],
            world_hash_equal=a['world_ref']['sha256']==b['world_ref']['sha256'], observation_pose_delta=d.tolist(),
            world_polyline_separation_m=separation(a['world'], b['world']),
            cart_clearance_delta_m=b['cart_only_edge_clearance_m']-a['cart_only_edge_clearance_m'],
            cart_pixels_delta=b['cart_visible_pixels']-a['cart_visible_pixels']))
    evolution_class = (None if not any(r['available'] for r in pairs) else
        'POST_REVEAL_LOCAL_INTENT_EVOLVING' if any(r.get('meaningful') for r in pairs) else 'POST_REVEAL_LOCAL_INTENT_STABLE')
    technical = meta['status'] in ('TECHNICAL_INVALID','PROTOCOL_ERROR','MODEL_ERROR','EXECUTOR_STALLED','CONTROLLER_ERROR')
    reason = 'RAW_UNSAFE' if (ep/'raw_unsafe_abort.json').exists() else meta['status']
    submissions = [e for e in events if e.get('status')=='submitted']
    tail_path = ep/'controller/post_episode_messages.json'
    tail = read(tail_path)['messages'] if tail_path.exists() else []
    results = [e for e in events+tail if e.get('type')=='solve_result']
    log = (run/'logs/server.log').read_text()
    return dict(valid=True, experiment=declaration['experiment'], run=str(run), episode=str(ep),
        scientific_freeze_sha=read(run/'execution_start.json')['sha'], starting_sha=read(run/'source.json')['starting_sha'],
        classification=classification(all(first_flags.values()), successive, technical),
        first_response_gates=first_flags, first_response_geometry=q, evolution=pairs, evolution_classification=evolution_class,
        chunks=records, reveal=reveal, original_validator=original, scheduler=scheduler, all_request_timing=all_timing,
        initial_pose=p['initial_pose_world'], end_sim_time_s=meta['end_sim_time_s'], status=meta['status'], termination_reason=reason,
        raw_unsafe_abort=read(ep/'raw_unsafe_abort.json') if (ep/'raw_unsafe_abort.json').exists() else None,
        safety_abort=read(ep/'guard_abort.json') if (ep/'guard_abort.json').exists() else None,
        actual_prefix_minimum_clearance_lower_bound_m=min(safe_min, default=None),
        calls=dict(scientific_episodes=1, terminal_predictions=len(pred), buffer_only=len(requests)-len(pred),
            startup_model_warmups=log.count('[lightnav-ws] warmup done'), MPC_submit_attempts=len(scheduler['control_ticks']),MPC_submissions=len(submissions),
            MPC_saved_results=len(results), MPC_successful_results=sum(e.get('status')=='command' for e in results),
            MPC_unobserved_result_ids=sorted({e['solve_id'] for e in submissions}-{e['solve_id'] for e in results}),
            MPC_physical_new_applications=sum(c['reason']=='new_solve' for c in commands),
            integration_intervals=len(commands), cart_reveals=int(reveal is not None), initialization_resets=1,
            resets_after_initialization=0, graph_optimizer=0, canonical_optimizer=0, B_ENTRY=0,
            retries=0, new_source_searches=0, validation_model_MPC_optimizer=0),
        source_hashes={str(q.relative_to(ep)):sha(q) for q in ep.rglob('*') if q.is_file()})


def bundle(run, v):
    """Derived indices and actual execution segments; immutable raw files stay in episode."""
    ep = Path(v['episode']); root = run/'source_bundle'
    states, commands = csvread(ep/'execution.csv'), csvread(ep/'commands.csv')
    poses = pose_rows(states)
    applied = [r for r in v['chunks'] if r['applied']]
    for i, r in enumerate(applied):
        sid = r['switch_state_id']; end = applied[i+1]['switch_state_id'] if i+1<len(applied) else len(states)-1
        out = root/'execution_segments'/r['chunk_id']; out.mkdir(parents=True, exist_ok=False)
        with (out/'actual.npy').open('xb') as f: np.save(f, poses[sid:end+1])
        gs = jsonlines(ep/'guard.jsonl')[sid:end]
        save(out/'segment.json', dict(start=states[sid], end=states[end], states=states[sid:end+1], commands=commands[sid:end],
            executed_arc_m=float(np.linalg.norm(np.diff(poses[sid:end+1,:2],axis=0),axis=1).sum()),
            minimum_swept_clearance_m=min((g['check']['minimum_clearance_lower_bound_m'] for g in gs),default=None),
            controller_events_path=str(ep/'controller/events.jsonl')))
        if i:
            old = applied[i-1]; start = old['switch_state_id']; out = root/'handoffs'/f'C{i-1}_to_C{i}'
            out.mkdir(parents=True, exist_ok=False)
            with (out/'actual_old_to_B.npy').open('xb') as f: np.save(f, poses[start:sid+1])
            save(out/'context.json', dict(old_chunk_id=old['chunk_id'], fresh_chunk_id=r['chunk_id'],
                old_raw_path=str(ep/old['raw_local_ref']['path']), fresh_raw_path=str(ep/r['raw_local_ref']['path']),
                old_observation_pose=old['A'], fresh_observation_pose=r['A'], B=r['B'], P=r['P'],
                u_minus=r['B_state']['u_minus'], previous_control=r['B_state']['memory'],
                reference_version=i, episode_id=EPISODES[0], request_metadata_path=r['request_metadata_path'],
                exact_runtime_context=r['context'], actual_old_to_B=str(out/'actual_old_to_B.npy')))
    save(root/'chunks.json', v['chunks'])
    save(root/'manifest.json', dict(source_episode=str(ep), source_hashes=v['source_hashes'],
        derived_hashes={str(q.relative_to(root)):sha(q) for q in root.rglob('*') if q.is_file()},
        classification=v['classification'], no_next_stage=True))


if __name__ == '__main__':
    p=argparse.ArgumentParser(); p.add_argument('--run',type=Path,required=True); p.add_argument('--seal',action='store_true')
    a=p.parse_args(); run=a.run.resolve(); v=validate(run)
    if a.seal:
        save(run/'validation.json',v); bundle(run,v)
    else:
        equal_record(v,read(run/'validation.json'))
    print(v['classification'],v['calls'])

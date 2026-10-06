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
from run_continuous_obstacle_reveal_exploratory02 import verify, declaration, namespace, OUT
from continuous_obstacle_reveal_exploratory_geometry02 import environments
from reconciliation.continuous_obstacle_reveal_exploratory02 import (
    acquisition_classification, post_reveal_classification, extra_evolution)
from validate_continuous_obstacle_reveal_episode01b import accounting
from validate_continuous_obstacle_reveal_episode01 import bundle
from analyze_join_online02 import csvread, jsonlines, pose_rows
from validate_obstacle_source_acquisition02 import scheduler_audit
from validate_robotless_online_handoffs import validate_episode, equal_record


def validate_episode_data(run):
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
    safe_min = []; interval_diagnostics = []; actual_min = []
    for i, g in enumerate(guards):
        present = bool(reveal and g['command']['application_state_id'] >= reveal['state_id'])
        assert g['cart_present'] == present
        expected = guard_check(on if present else base, g['start_pose'], g['command'], dt)
        equal_record(expected, {k:g[k] for k in expected}, 'guard')
        # Limiting obstacle group on the identical guarded sweep; descriptive only.
        from shapely.geometry import LineString
        line = LineString(g['poses'])
        bound = g['check'].get('curved_path_error_bound_m', 0.)
        hospital = base.environment.check_polyline(g['poses'])['minimum_clearance_m']-bound
        cart_clearance = float(cart.distance(line)-.20-bound) if present else None
        interval_diagnostics.append(dict(interval=i,cart_present=present,guard_pass=g['safe'],
            applied=i<len(commands),physical_footprint_clearance_lower_bound_m=g['check']['minimum_clearance_lower_bound_m'],
            legacy_5cm_margin_pass=g['check']['legacy_5cm_margin_pass'],
            exploratory_overlap_free=g['check']['exploratory_overlap_free'],
            Hospital_clearance_lower_bound_m=hospital, cart_clearance_lower_bound_m=cart_clearance,
            limiting_geometry='cart' if present and cart_clearance<hospital else 'Hospital'))
        if g['safe']:
            # Recheck exactly the executed integration interval, separately from
            # the pre-application guard's nominal .1 s lookahead.
            actual = guard_check(on if present else base, g['start_pose'],
                {**g['command'], 'reason':'held_interval_saved_only'}, dt)
            assert actual['safe'] and not actual['check']['physical_overlap']
            actual_min.append(actual['check']['minimum_clearance_lower_bound_m'])
            interval_diagnostics[-1].update(
                actual_interval_clearance_lower_bound_m=actual['check']['minimum_clearance_lower_bound_m'],
                actual_interval_legacy_5cm_pass=actual['check']['legacy_5cm_margin_pass'])
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
                chosen = row['geometry_on'] if i else row['geometry_off']
                row.update(physical_footprint_clearance_m=chosen['minimum_clearance_m'],
                    exploratory_overlap_free=chosen['exploratory_overlap_free'],
                    legacy_5cm_margin_pass=chosen['legacy_5cm_margin_pass'],
                    history_provenance=history_provenance(ep, r, p['instruction']))
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
        pairs.append(dict(pair=f'C{i}->C{i+1}', available=True, **e, **extra_evolution(a,b),
            classification=f'C{i}_TO_C{i+1}_'+('EVOLVING' if e['meaningful'] else 'STABLE'),
            raw_hash_equal=a['raw_local_ref']['sha256']==b['raw_local_ref']['sha256'],
            world_hash_equal=a['world_ref']['sha256']==b['world_ref']['sha256'], observation_pose_delta=d.tolist(),
            world_polyline_separation_m=separation(a['world'], b['world']),
            cart_clearance_delta_m=b['cart_only_edge_clearance_m']-a['cart_only_edge_clearance_m'],
            cart_pixels_delta=b['cart_visible_pixels']-a['cart_visible_pixels']))
    evolution_class = post_reveal_classification(pairs)
    technical = meta['status'] in ('TECHNICAL_INVALID','PROTOCOL_ERROR','MODEL_ERROR','EXECUTOR_STALLED','CONTROLLER_ERROR')
    if (ep/'raw_unsafe_abort.json').exists():
        rejected=read(ep/'raw_unsafe_abort.json')
        checked=read(ep/'reference_checks'/f"{rejected['chunk_id']}.json")
        equal_record(rejected['check'],checked['check'])
        assert not rejected['reference_installed'] and not rejected['check']['clearance_valid']
    reason = 'RAW_UNSAFE' if (ep/'raw_unsafe_abort.json').exists() else meta['status']
    submissions = [e for e in events if e.get('status')=='submitted']
    tail_path = ep/'controller/post_episode_messages.json'
    tail = read(tail_path)['messages'] if tail_path.exists() else []
    results = [e for e in events+tail if e.get('type')=='solve_result']
    log = (run/'logs/server.log').read_text()
    a0,b1=records[:2]
    return dict(valid=True, experiment=declaration['experiment'], run=str(run), episode=str(ep),
        scientific_freeze_sha=read(run/'execution_start.json')['sha'], starting_sha=read(run/'source.json')['starting_sha'],
        classification=acquisition_classification(records,successive,technical or (len(applied)==4 and not successive)),
        historical_first_response_gates=first_flags,
        exploratory_first_reaction_valid=bool(a0['applied'] and b1['applied'] and reveal and not b1.get('stop') and
            b1.get('exploratory_overlap_free') and q and q['difference']['meaningful']),
        exploratory_clearance=p['exploratory_clearance'],interval_diagnostics=interval_diagnostics,
        first_response_gates=first_flags, first_response_geometry=q, evolution=pairs, evolution_classification=evolution_class,
        chunks=records, reveal=reveal, original_validator=original, scheduler=scheduler, all_request_timing=all_timing,
        initial_pose=p['initial_pose_world'], end_sim_time_s=meta['end_sim_time_s'], status=meta['status'], termination_reason=reason,
        raw_unsafe_abort=read(ep/'raw_unsafe_abort.json') if (ep/'raw_unsafe_abort.json').exists() else None,
        safety_abort=read(ep/'guard_abort.json') if (ep/'guard_abort.json').exists() else None,
        actual_prefix_minimum_clearance_lower_bound_m=min(actual_min, default=None),
        minimum_guard_lookahead_clearance_lower_bound_m=min(safe_min, default=None),
        calls=dict(scientific_episodes=1, terminal_predictions=len(pred), buffer_only=len(requests)-len(pred),
            startup_model_warmups=log.count('[lightnav-ws] warmup done'), MPC_submit_attempts=len(scheduler['control_ticks']),MPC_submissions=len(submissions),
            MPC_saved_results=len(results), MPC_successful_results=sum(e.get('status')=='command' for e in results),
            MPC_unobserved_result_ids=sorted({e['solve_id'] for e in submissions}-{e['solve_id'] for e in results}),
            MPC_physical_new_applications=sum(c['reason']=='new_solve' for c in commands),
            integration_intervals=len(commands), cart_reveals=int(reveal is not None), initialization_resets=1,
            resets_after_initialization=0, graph_optimizer=0, canonical_optimizer=0, B_ENTRY=0,
            retries=0, new_source_searches=0, validation_model_MPC_optimizer=0),
        source_hashes={str(q.relative_to(ep)):sha(q) for q in ep.rglob('*') if q.is_file()})


def history_provenance(ep, r, instruction):
    import base64, hashlib, json
    history=read(ep/r['history_snapshot']['path'])
    wire=next(w for w in jsonlines(ep/'requests/wire.jsonl')
        if w['direction']=='request' and w.get('seq')==r['seq'] and w['action']=='next')
    path=ep/wire['raw']['path']; assert sha(path)==wire['raw']['sha256']
    payload=json.loads(path.read_bytes())
    assert set(payload)=={'action','data'}
    assert set(payload['data'])=={'seq','image','instruction'}
    assert payload['data']['instruction']==instruction
    assert hashlib.sha256(base64.b64decode(payload['data']['image'])).hexdigest()==r['observation']['sha256']
    assert sha(ep/r['raw_local_ref']['path'])==r['raw_local_ref']['sha256']
    assert sha(ep/r['response_ref']['path'])==r['response_ref']['sha256']
    frames=history['frames']
    for f in frames: assert sha(ep/f['path'])==f['sha256']
    assert history['chronological_history_frame_ids']==[f['frame_id'] for f in frames]
    return dict(request_RGB_sha256=r['observation']['sha256'],
        history_frame_ids=[f['frame_id'] for f in frames],history_frame_sha256=[f['sha256'] for f in frames],
        model_input_segments_reconstructed=history['model_input_segments_reconstructed'],
        internal_selection_directly_observed=False,instruction=instruction,
        instruction_sha256=hashlib.sha256(instruction.encode()).hexdigest(),
        serialized_request=wire['raw'],raw_response_sha256=r['response_ref']['sha256'],
        raw_action_sha256=r['raw_local_ref']['sha256'],radius_or_map_in_model_payload=False)

def validate(run):
    verify(run)
    cfg=declaration(); launch=read(run/'launch_result.json')
    attempt=read(run/'launch_attempt.json'); plan=read(run/'launch_environment.json')
    assert attempt['argv']==plan['argv'] and attempt['full_startup_budget']==1
    assert launch['full_launch_attempts']==1 and launch['no_retry']
    for p,h in launch['actual_library_sha256'].items(): assert sha(p)==h,p
    samples=[read(p) for p in sorted((run/'process_observations').glob('*.json'))]
    expected=read(run/'loader_verification.json')['cases'][1]
    observed=sorted({p for s in samples for p in s['libraries'] if 'libnvJitLink.so' in p})
    for p in observed: assert str(Path(p).resolve())==expected['resolved_path'],p
    if launch['scene_initialized']:
        assert samples and observed, 'actual launch environment/library provenance missing'
    for s in samples:
        # Isaac's own python.sh sets LD_LIBRARY_PATH/PYTHONPATH after sanitation.
        for k in cfg['unset_environment'][2:]: assert s['relevant_environment'][k] is None,k
        for k,v in cfg['set_environment'].items(): assert s['relevant_environment'][k]==v,k
    calls=accounting(run)
    assert calls['full_SimulationApp_launch_attempts']==1
    assert calls['terminal_LightNav_predictions']<=4 and calls['cart_reveals']<=1
    ep=run/'episodes/EPISODE_00'; scientific=None; error=None
    if (ep/'metadata.json').exists():
        try:
            scientific=validate_episode_data(run)
            old=scientific['calls']
            for a,b in [('terminal_LightNav_predictions','terminal_predictions'),
                        ('buffer_only_LightNav_requests','buffer_only'),
                        ('MPC_submissions','MPC_submissions'),('MPC_solved_results','MPC_saved_results'),
                        ('MPC_physical_applications','MPC_physical_new_applications'),
                        ('integration_intervals','integration_intervals')]:
                assert calls[a]==old[b],(a,calls[a],old[b])
        except Exception as exc:
            error=f'{type(exc).__name__}: {exc}'
            scientific=None
    else:
        error='No completed episode metadata; scientific validator has no complete record to validate'
    qualified=bool(scientific and scientific['classification']=='CONTINUOUS_EXPLORATORY_C0_C3_ACQUIRED')
    return dict(experiment=cfg['experiment'],starting_sha=read(run/'source.json')['starting_sha'],
        scientific_freeze_sha=attempt['scientific_freeze_sha'],run=str(run),
        historical01b_preserved=True,historical01b_calls=read(ROOT/cfg['historical_results']/'result_summary.json')['calls'],
        protocol_delta_valid=True,intentional_difference='required_extra_clearance_0.05_to_0.00_only',
        policy=cfg['exploratory_clearance'],
        loader_verification=read(run/'loader_verification.json'),actual_nvJitLink_paths=observed,
        actual_library_mapping_observed=bool(observed),launch=launch,
        scientific_episode_validator_passed=scientific is not None,scientific_validator_error=error,
        source_bundle_valid=qualified,partial_data_valid=scientific is not None and not qualified,
        source_bundle=str(run/'source_bundle') if scientific else None,
        classification=scientific['classification'] if scientific else 'TECHNICAL_EXECUTION_BLOCKED',
        calls=calls,scientific=scientific,
        technical_evidence_sha256={str(p.relative_to(run)):sha(p) for p in run.rglob('*')
            if p.is_file() and (p.parent.name in ('logs','process_observations') or
                p.name in ('launch_attempt.json','launch_result.json','execution_start.json','collection_failure.json',
                           'generation_actual.json','server_shutdown.json','schedule_completion.json'))})


def validate_bundle(run, v):
    root=run/'source_bundle'; manifest=read(root/'manifest.json')
    assert manifest['source_hashes']==v['source_hashes']
    assert manifest['classification']==v['classification'] and manifest['no_next_stage']
    for p,h in manifest['source_hashes'].items(): assert sha(Path(v['episode'])/p)==h,p
    for p,h in manifest['derived_hashes'].items(): assert sha(root/p)==h,p
    equal_record(read(root/'chunks.json'),v['chunks'])
    return dict(valid=True,source_files=len(manifest['source_hashes']),
        derived_files=len(manifest['derived_hashes']),new_scientific_calls=0)


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--seal',action='store_true')
    a=p.parse_args();run=namespace(a.run);v=validate(run)
    if a.seal:
        if v['scientific']:
            save(run/'validation.json',v['scientific']);bundle(run,v['scientific'])
            save(run/'source_bundle/qualification.json',dict(valid_continuous_C0_C3_source=v['source_bundle_valid'],
                validated_partial_data=v['partial_data_valid'],classification=v['classification'],
                validation_sha256=sha(run/'validation.json')))
        if v['scientific']:
            save(run/'source_bundle_validation.json',validate_bundle(run,v['scientific']))
        save(run/'attempt_validation.json',v)
        save(OUT/'execution_audit.json',{k:value for k,value in v.items() if k!='scientific'})
    else:
        equal_record(v,read(run/'attempt_validation.json'))
        if v['scientific']: equal_record(validate_bundle(run,v['scientific']),read(run/'source_bundle_validation.json'))
    print(v['classification'],v['calls'],v['scientific_validator_error'])


if __name__=='__main__':main()

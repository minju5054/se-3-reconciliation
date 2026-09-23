#!/usr/bin/env python3
"""Read-only source audit; exclusively creates a NEW derived directory."""
import argparse
import base64
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
from audit_join_online02_stop import point_relation, write_new
from reconciliation.join_online03_turnback import (
    mask_statistics, project_mesh, point_bbox_relation, sampled_slots,
    history_aggregate, trajectory_metrics, transition,
)
from reconciliation.join_source04 import pixel_ray, ground_intersection
from reconciliation.online_history import history_contract
from reconciliation.se2 import local_trajectory_to_world
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.join_source03 import path_geometry
from reconciliation.join_online02 import hallway_geometry, classify_chunk
from run_join_online02 import environments
from validate_robotless_online_handoffs import equal_record

EPISODES = ['ON_REPEAT_00', 'ON_REPEAT_01']
PROTOCOL = dict(
    experiment='JOIN-ONLINE-03 TURNBACK VISUAL-HISTORY AUDIT',
    scope='SAVED OBSERVATIONAL AUDIT; no causal intervention', episodes=EPISODES,
    chunks=list(range(7)), primary_transition=[4, 5], context_transitions=[[3, 4], [5, 6]],
    minimum_visible_pixels=20, threshold_source='unchanged SOURCE03/04 renderer visibility gate',
    image_region='bbox centre thirds; boundary contact overrides as partial-edge; descriptive only',
    visible_fraction='visible instance pixels / original RGB image area',
    projection='full saved world mesh, saved USD camera and near/far/image clipping; pixel-centre triangle union; no scene occlusion',
    history='wire-confirmed delivered frames + read-only pinned official sampler reconstruction; NOT internal server telemetry',
    history_counting='unique sampled images and input slots separately; preceding-only aggregate excludes current image',
    turnback='signed hallway lateral endpoint AND full-polyline extrema, inward segments/interior arc and wrapped final yaw/tangent deltas',
    safety='unchanged .20m footprint / .05m edge margin and original whole raw-polyline checker; no connector or trimming',
    interpretation='report predeclared user categories without fitted thresholds; available pixels are not neural recognition/attention',
    scientific_model_calls=0, MPC_calls=0, GP_calls=0, rigid_calls=0,
    new_rollouts=0, scientific_renders=0, offline_geometric_projection_only=True)


def read(p):
    return json.loads(Path(p).read_text())


def sha(p):
    h = hashlib.sha256()
    with Path(p).open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def inventory(root):
    return {str(p.resolve()): sha(p) for p in sorted(Path(root).rglob('*')) if p.is_file()}


def lines(p):
    return [json.loads(x) for x in Path(p).read_text().splitlines() if x]


def collect(run):
    """Recompute from primary records, not from the audit's saved result."""
    hashes = {}
    def track(p, expected=None):
        p = Path(p).resolve()
        actual = hashes.get(str(p)) or sha(p)
        if expected is not None and actual != expected:
            raise ValueError(f'source hash mismatch: {p}')
        hashes[str(p)] = actual
        return p
    source = read(track(run/'source.json'))
    for p, h in source['preserved'].items():
        track(p, h)
    for name in ['protocol.json', 'config_snapshot.yaml', 'freeze.json', 'validation.json',
                 'generation_actual.json', 'server_launch.json', 'mpc_audit.json', 'side_passages.json']:
        track(run/name)
    assert read(run/'validation.json')['valid']
    launch = read(run/'server_launch.json')
    contract, sampler = history_contract(Path(launch['lightnav_checkout']), Path(launch['checkpoint_path']))
    for n, h in contract['source_sha256'].items():
        track(Path(launch['lightnav_checkout'])/n, h)
    track(Path(launch['checkpoint_path'])/'eval_config.json', contract['eval_config_sha256'])
    scenario = read(track(run/'scenario.json'))
    triangles = np.load(track(scenario['triangles_path']))['triangles']
    original = read(track(run/'aggregate/analysis.json'))
    old = {(r['episode'], r['chunk_id']): r for e in original['episodes'] for r in e['rows']}
    base, on, cart, _ = environments(run)
    cart_env = HospitalEnvironment(cart, on.workspace, metadata=on.metadata)
    cfg = yaml.safe_load((run/'config_snapshot.yaml').read_text())['join_online02']['classification']
    free_sides = [r['side'] for r in read(run/'side_passages.json') if r['check']['clearance_valid']]
    rows, history_rows, frames = [], [], []
    for eid in EPISODES:
        ep = run/'episodes'/eid
        session = read(track(ep/'session_open.json'))
        track(ep/'metadata.json'); track(ep/'completion.json'); track(ep/'guard_abort.json')
        vis = {r['frame_id']: r for r in lines(track(ep/'visibility.jsonl'))}
        captures = {r['frame_id']: r for r in lines(track(ep/'capture.jsonl'))}
        wire = lines(track(ep/'requests/wire.jsonl'))
        sent = [r for r in wire if r['direction'] == 'request' and r['action'] == 'next']
        frame_cache = {}
        metadata = sorted((ep/'chunks').glob('*/metadata.json'))
        assert [p.parent.name for p in metadata] == [f'chunk_{i:03}' for i in range(7)]
        for mp in metadata:
            m = read(track(mp)); ob = m['observation']; chunk = m['chunk_id']
            response = read(track(ep/m['response_ref']['path'], m['response_ref']['sha256']))['data']
            assert response == m['response']
            h = read(track(ep/m['history_snapshot']['path'], m['history_snapshot']['sha256']))
            assert h['history_contract'] == contract and h['triggering_observation_frame_id'] == ob['frame_id']
            assert h['server_actions_step_observed'] == response['actions']['step'] == len(h['frames'])
            slots = sampled_slots(h, sampler)
            delivered = {f['frame_id']: f for f in h['frames']}
            for f in h['frames']:
                cap = captures[f['frame_id']]
                for key in ['pose_world', 'camera', 'capture_sim_time_s', 'capture_monotonic_ns', 'sha256', 'path']:
                    assert f[key] == cap[key]
                req = sent[f['wire_seq']]
                body = read(track(ep/req['raw']['path'], req['raw']['sha256']))['data']
                jpeg = track(ep/f['path'], f['sha256'])
                assert base64.b64decode(body['image']) == jpeg.read_bytes()
            group = []
            for slot in slots:
                f = delivered[slot['frame_id']]
                if f['frame_id'] not in frame_cache:
                    v = vis[f['frame_id']]
                    assert v['same_render_product_state'] and not v['model_input']
                    assert v['pose_world'] == f['pose_world'] and v['camera'] == f['camera']
                    assert v['state_id'] == f['rendered_state_id'] and v['sim_time_s'] == f['capture_sim_time_s']
                    assert v['cart_transform'] == scenario['prop']['wrapper_matrix_column']
                    mask_path = track(ep/v['mask_path'], v['mask_sha256'])
                    mask = np.load(mask_path)['mask']
                    prefix = scenario['prop']['runtime_prim']
                    ids = [int(k) for k, label in v['instance']['idToLabels'].items() if str(label).startswith(prefix)]
                    assert sorted(ids) == sorted(v['instance']['matched_instance_ids'])
                    selected = np.isin(mask, ids)
                    assert int(selected.sum()) == v['instance']['visible_pixels']
                    proj = project_mesh(triangles, f['camera'])
                    stats = mask_statistics(selected, proj)
                    from PIL import Image
                    with Image.open(ep/f['path']) as image:
                        assert image.size == (stats['width'], stats['height'])
                    entry = dict(episode=eid, frame_id=f['frame_id'], RGB_path=str((ep/f['path']).resolve()),
                        RGB_sha256=f['sha256'], mask_path=str(mask_path), mask_sha256=v['mask_sha256'],
                        cart_ids=ids, capture_sim_time_s=f['capture_sim_time_s'],
                        capture_monotonic_ns=f['capture_monotonic_ns'], pose_world=f['pose_world'],
                        camera=f['camera'], cart_transform=v['cart_transform'], **stats)
                    frames.append(entry); frame_cache[f['frame_id']] = entry
                entry = frame_cache[f['frame_id']]
                hr = dict(episode=eid, chunk=chunk, session=session['connection_id'], **slot,
                          is_current=slot['frame_id'] == ob['frame_id'],
                          frame_source_key=f'{eid}/{slot["frame_id"]}', **{k: entry[k] for k in [
                              'RGB_path', 'RGB_sha256', 'pose_world', 'capture_sim_time_s', 'visible_pixels',
                              'visibility_gate_pass', 'visible_image_fraction', 'bbox_xyxy_inclusive', 'region']})
                history_rows.append(hr); group.append(hr)
            current = frame_cache[ob['frame_id']]
            assert ob['pose_world'] == current['pose_world'] and ob['capture_sim_time_s'] == current['capture_sim_time_s']
            v = vis[ob['frame_id']]; mask = np.load(current['mask_path'])['mask']
            selected = np.isin(mask, current['cart_ids'])
            pointing = {}
            for channel in ['apos', 'opos']:
                point = point_relation(response['pointing'], channel, mask, v['instance'])
                point.update(point_bbox_relation(point['pixel'], selected))
                prim = point['raster_prim'] or ''
                # Keep exact label primary; a floor proxy is not a free-path proof.
                point['raster_class'] = ('cart' if point['raster_on_cart'] else
                    'floor' if ('floor' in prim.lower() or 'ground' in prim.lower()) and 'sign' not in prim.lower()
                    else 'other' if prim else None)
                point['ground_proxy'] = None
                if point['pixel'] is not None:
                    origin, direction = pixel_ray(point['pixel'], ob['camera'])
                    ground = ground_intersection(origin, direction)
                    if ground is not None:
                        point['ground_proxy'] = dict(world_xyz=ground.tolist(),
                            kind='BOUNDARY_RAY_PROXY' if point['clamped'] else 'NOMINAL_GROUND_RAY_PROXY',
                            metric_model_waypoint=False)
                pointing[channel] = point
            raw = np.load(track(ep/m['raw_local_ref']['path'], m['raw_local_ref']['sha256']))
            world = np.load(track(ep/m['world_ref']['path'], m['world_ref']['sha256']))
            np.testing.assert_array_equal(raw, response['actions']['actions'])
            np.testing.assert_array_equal(raw, m['raw_local'])
            np.testing.assert_array_equal(world, m['world'])
            np.testing.assert_allclose(world, local_trajectory_to_world(ob['pose_world'], raw), rtol=0, atol=1e-10)
            prior = old[(eid, chunk)]
            g = path_geometry(world, on)
            equal_record(g, prior['raw_geometry'], 'original full raw geometry')
            motion = hallway_geometry(world, ob['pose_world'], scenario['center_xy'], scenario['forward_xy'], scenario['cart_extents'], cfg)
            classification = classify_chunk(motion, g['whole']['clearance_valid'], m['stop'], free_sides, scenario['cart_extents'], cfg)
            assert classification == prior['classification']
            row = dict(episode=eid, chunk=chunk, session=session['connection_id'],
                observation_time_s=ob['capture_sim_time_s'], observation_pose=ob['pose_world'],
                current=current, history=history_aggregate(group, include_current=True),
                prior_history=history_aggregate(group, include_current=False),
                sampled_history=group, raw_pointing=response['pointing'], pointing=pointing,
                raw_text=response['raw_text'], target_visible=response.get('visible'), stop=response['stop'],
                raw_local_ref=dict(path=str((ep/m['raw_local_ref']['path']).resolve()), sha256=m['raw_local_ref']['sha256']),
                world_ref=dict(path=str((ep/m['world_ref']['path']).resolve()), sha256=m['world_ref']['sha256']),
                raw_local=raw.tolist(), world=world.tolist(), trajectory=trajectory_metrics(world, scenario, cfg['tangent_chord_m']),
                original_geometry=g['whole'], cart_geometry=cart_env.check_polyline(world),
                first_unsafe_row=g['first_unsafe_waypoint_zero_based'], first_unsafe_segment=g['first_unsafe_segment_zero_based'],
                original_classification=classification, safe_bypass_onset=prior['onset'], full_bypass=prior['full_bypass'],
                applied=prior['t_apply'] is not None, t_apply=prior['t_apply'], execution=prior['execution'],
                raw_response_path=str((ep/m['response_ref']['path']).resolve()),
                history_snapshot_path=str((ep/m['history_snapshot']['path']).resolve()))
            rows.append(row)
    transitions = [transition(next(r for r in rows if r['episode'] == e and r['chunk'] == f'chunk_{a:03}'),
                              next(r for r in rows if r['episode'] == e and r['chunk'] == f'chunk_{b:03}'))
                   for e in EPISODES for a, b in [[3, 4], [4, 5], [5, 6]]]
    return dict(rows=rows, history_rows=history_rows, frames=frames, transitions=transitions,
                source_hashes=hashes, scenario=scenario, history_contract=contract)


def table_rows(result):
    chunks, points = [], []
    for r in result['rows']:
        g, h, prior = r['trajectory'], r['history']['unique'], r['prior_history']['unique']
        chunks.append(dict(episode=r['episode'], chunk=r['chunk'], time_s=r['observation_time_s'],
            current_pixels=r['current']['visible_pixels'], projected_pixels=r['current']['projected_pixels'],
            current_image_fraction=r['current']['visible_image_fraction'], current_bbox=r['current']['bbox_xyxy_inclusive'],
            region=r['current']['region'], sampled_frames=h['count'], visible_sampled_frames=h['cart_visible_count'],
            history_pixels=h['total_cart_pixels'], history_visible_fraction=h['visible_fraction'],
            latest_visible_age_s=h['most_recent_visible_age_s'], oldest_visible_age_s=h['oldest_visible_age_s'],
            preceding_frames=prior['count'], preceding_cart_pixels=prior['total_cart_pixels'],
            preceding_latest_visible_age_s=prior['most_recent_visible_age_s'],
            max_signed_lateral_m=g['max_signed_lateral_m'], major_signed_lateral_m=g['major_signed_lateral_m'],
            max_abs_lateral_m=g['max_abs_lateral_m'], endpoint_lateral_m=g['endpoint_lateral_m'],
            final_tangent_rad=g['final_tangent_rad'], final_yaw_rad=g['final_yaw_hallway_rad'], raw_arc_m=g['raw_arc_m'],
            min_edge_m=r['original_geometry']['minimum_clearance_m'], cart_min_edge_m=r['cart_geometry']['minimum_clearance_m'],
            safe=r['original_geometry']['clearance_valid'], overlap=r['original_geometry']['physical_overlap'],
            first_unsafe_row=r['first_unsafe_row'], first_unsafe_segment=r['first_unsafe_segment'],
            applied=r['applied'], classification=r['original_classification']))
        for channel in ['apos', 'opos']:
            points.append(dict(episode=r['episode'], chunk=r['chunk'], channel=channel, **r['pointing'][channel]))
    return dict(chunk_visibility=chunks, history_visibility=result['history_rows'], apos_opos_audit=points)


def csv_value(v):
    return 'N/A' if v is None else json.dumps(v, ensure_ascii=False, allow_nan=False) if not isinstance(v, str) else v


def write_csv(path, rows):
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with Path(path).open('x', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows({k: csv_value(r.get(k)) for k in fields} for r in rows)


def summarize(result):
    primary = [t for t in result['transitions'] if t['source'] == 'chunk_004']
    late = [r for r in result['rows'] if r['chunk'] in ['chunk_005', 'chunk_006']]
    retained = all(r['current']['visibility_gate_pass'] and
                   r['prior_history']['unique']['cart_visible_count'] == r['prior_history']['unique']['count']
                   and r['prior_history']['unique']['count'] > 0 for r in late)
    increasing = all(t['delta_current_pixels'] > 0 and
        t['after_prior_history']['unique']['total_cart_pixels'] > t['before_prior_history']['unique']['total_cart_pixels'] for t in primary)
    return dict(repetitions=2, chunks=14, distinct_capture_frames=len(result['frames']),
        sampled_slots=sum(r['history']['slots']['count'] for r in result['rows']),
        primary_transitions=primary, late_cart_evidence_retained=retained,
        current_and_prior_pixel_totals_increase=increasing,
        interpretation='CART_EVIDENCE_REMAINS_STRONG_DURING_TURNBACK' if retained and increasing else 'MIXED_OR_INSUFFICIENT_EVIDENCE',
        direct_internal_selection_telemetry=False,
        conclusion='Available cart pixels do not disappear at the turn-back. This does not identify neural recognition, memory, attention or causal mechanism.',
        scientific_model_calls=0, MPC_calls=0, GP_calls=0, rigid_calls=0, rollouts=0, scientific_renders=0,
        limitation='Counts describe source RGB before resize/post-ViT spatial pooling; pixel totals are not token salience or attention.',
        geometry_limitation='Lateral inward direction is relative to hallway/cart centre; no time correspondence between chunk rows.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(); run, out = a.run.resolve(), a.output.resolve()
    if out == run or run in out.parents:
        raise ValueError('derived output must be outside original run')
    out.mkdir(parents=True, exist_ok=False)
    write_new(out/'protocol.json', PROTOCOL)
    before = inventory(run)
    start = time.monotonic()
    result = collect(run)
    write_new(out/'records.json', result)
    write_new(out/'turnback_metrics.json', result['transitions'])
    write_new(out/'summary.json', summarize(result))
    for name, rows in table_rows(result).items():
        write_csv(out/(name+'.csv'), rows)
    code = [ROOT/'src/reconciliation/join_online03_turnback.py', Path(__file__),
            ROOT/'scripts/validate_join_online03_turnback.py', ROOT/'scripts/plot_join_online03_turnback.py',
            ROOT/'tests/test_join_online03_turnback.py']
    write_new(out/'source_manifest.json', dict(primary_run=str(run), starting_sha=subprocess.check_output(
        ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        input_sha256=result['source_hashes'], entire_primary_before_sha256=before,
        code_sha256={str(f.resolve()): sha(f) for f in code},
        unrelated_edits_sha256={str(ROOT/n): sha(ROOT/n) for n in [
            'configs/stage0_jackal_controller_validation.yaml', 'configs/stage0_lightnav_single_chunk.yaml']}))
    if before != inventory(run):
        raise ValueError('original run changed during saved audit')
    write_new(out/'audit_compute.json', dict(analysis_wall_s=time.monotonic()-start, scientific_calls=0))
    print(json.dumps(summarize(result), indent=2))


if __name__ == '__main__':
    main()

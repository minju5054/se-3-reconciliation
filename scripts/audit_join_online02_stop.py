#!/usr/bin/env python3
"""Saved-only STOP/pointing audit. No model, renderer, controller, or new rollout.

Pixel hits are raster observations, not a claim about hidden target identity.
The 48x27 grid-cell footprint exposes quantization uncertainty in OPOS/APOS.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from reconciliation.join_source04 import pixel_ray, ground_intersection
from reconciliation.se2 import local_trajectory_to_world


def read(path):
    return json.loads(Path(path).read_text())


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_new(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def point_relation(pointing, channel, mask, instance):
    """Integer raster sample plus entire quantized cell, no semantic inference."""
    if pointing['mode'] != 'grid':
        raise ValueError('this saved audit requires the official grid vocabulary')
    w, h = pointing['frame_size']
    if mask.shape != (h, w):
        raise ValueError('mask and client RGB resolution differ')
    pixel = pointing.get(channel + '_px')
    out = dict(state=pointing[channel + '_state'], pixel=pixel,
               clamped=pointing[channel + '_clamped'], raster_prim=None,
               raster_on_cart=None, grid_cell_cart_fraction=None,
               pixel_to_cart_distance=None, grid_cell_xyxy=None)
    if pixel is None:
        return out
    u, v = map(float, pixel)
    if not (0 <= u < w and 0 <= v < h):
        raise ValueError('point outside client image')
    x, y = int(np.floor(u)), int(np.floor(v))
    cart = np.isin(mask, instance['matched_instance_ids'])
    ys, xs = np.nonzero(cart)
    cell_w, cell_h = w / 48, h / 27
    bx, by = int(u / cell_w), int(v / cell_h)
    x0, x1 = int(np.ceil(bx * cell_w)), int(np.ceil((bx + 1) * cell_w))
    y0, y1 = int(np.ceil(by * cell_h)), int(np.ceil((by + 1) * cell_h))
    out.update(raster_prim=instance['idToLabels'].get(str(int(mask[y, x]))),
               raster_on_cart=bool(cart[y, x]),
               grid_cell_cart_fraction=float(cart[y0:y1, x0:x1].mean()),
               pixel_to_cart_distance=float(np.min(np.hypot(xs-u, ys-v))) if len(xs) else None,
               grid_cell_xyxy=[x0, y0, x1, y1],
               interpretation='CENSORED_BY_CLAMPING' if out['clamped'] else 'COARSE_GRID_POINT')
    return out


def stop_evidence(response, tokenizer_manifest):
    """The official RVQ stop predicate checks level zero, not APOS."""
    tokens = re.findall(r'<act_l(\d+)_(\d+)>', response['raw_text'])
    if [int(x[0]) for x in tokens] != list(range(len(tokenizer_manifest['levels']))):
        raise ValueError('missing/reordered action levels')
    codes = [int(x[1]) for x in tokens]
    path = np.asarray(response['actions']['actions'], dtype=float)
    if path.ndim != 2 or path.shape[1] != 3 or len(path) == 0:
        raise ValueError('invalid raw Nx3')
    return dict(codes=codes, explicit_stop_l0=codes[0] == tokenizer_manifest['stop']['l0'],
                full_manifest_stop_tuple=codes == tokenizer_manifest['stop']['codes'],
                decoded_all_zero=bool(np.all(path == 0)),
                wire_stop=response['stop'], apos_state=response['pointing']['apos_state'],
                rows=len(path))


def analyze(run):
    run = Path(run).resolve()
    hashes = {}

    def tracked(path, expected=None):
        path = Path(path).resolve()
        actual = digest(path)
        if expected is not None and actual != expected:
            raise ValueError(f'source hash mismatch: {path}')
        hashes[str(path)] = actual
        return path

    source = read(tracked(run / 'source.json'))
    for path, expected in source['preserved'].items():
        tracked(path, expected)
    assert read(tracked(run / 'validation.json'))['valid']
    launch = read(tracked(run / 'server_launch.json'))
    checkpoint = Path(launch['checkpoint_path'])
    manifest_path = checkpoint / 'action_tokenizer/manifest.json'
    expected = next(x['sha256'] for x in launch['checkpoint_files'] if x['path'] == 'action_tokenizer/manifest.json')
    manifest = read(tracked(manifest_path, expected))
    external = Path(launch['lightnav_checkout'])
    for name in ['src/lightnav/tracking.py', 'src/lightnav/traj_vocab.py',
                 'src/lightnav/serving/protocol.py', 'src/lightnav/prompts.py',
                 'mujoco_demo/vln_mujoco/server.py']:
        tracked(external / name)
    for name in ['scripts/isaac/robotless_online_handoffs.py', 'scripts/online_lightnav_worker.py',
                 'src/reconciliation/join_source04.py', 'src/reconciliation/se2.py']:
        tracked(ROOT / name)
    scenario = read(tracked(run / 'scenario.json'))
    original = read(tracked(run / 'aggregate/analysis.json'))
    original_rows = {(r['episode'], r['chunk_id']): r for e in original['episodes'] for r in e['rows']}
    rows = []
    for ep in sorted((run / 'episodes').iterdir()):
        metadata = read(tracked(ep / 'metadata.json'))
        visibility = {x['frame_id']: x for x in map(json.loads, tracked(ep / 'visibility.jsonl').read_text().splitlines())}
        for path in sorted((ep / 'chunks').glob('*/metadata.json')):
            meta = read(tracked(path))
            response_path = tracked(ep / meta['response_ref']['path'], meta['response_ref']['sha256'])
            response = read(response_path)['data']
            observation = meta['observation']
            rgb = tracked(ep / observation['path'], observation['sha256'])
            raw = np.load(tracked(ep / meta['raw_local_ref']['path'], meta['raw_local_ref']['sha256']))
            world = np.load(tracked(ep / meta['world_ref']['path'], meta['world_ref']['sha256']))
            np.testing.assert_array_equal(raw, response['actions']['actions'])
            np.testing.assert_allclose(world, local_trajectory_to_world(observation['pose_world'], raw), rtol=0, atol=1e-10)
            vis = visibility[observation['frame_id']]
            assert vis['same_render_product_state'] and vis['pose_world'] == observation['pose_world']
            mask_path = tracked(ep / vis['mask_path'], vis['mask_sha256'])
            mask = np.load(mask_path)['mask']
            cart = np.isin(mask, vis['instance']['matched_instance_ids'])
            assert int(cart.sum()) == vis['instance']['visible_pixels']
            p = response['pointing']
            apos = point_relation(p, 'apos', mask, vis['instance'])
            opos = point_relation(p, 'opos', mask, vis['instance'])
            ground = None
            if apos['pixel'] is not None:
                origin, direction = pixel_ray(apos['pixel'], vis['camera'])
                q = ground_intersection(origin, direction)
                if q is not None:
                    local = np.linalg.inv(np.asarray(vis['camera']['T_world_agent'])) @ np.r_[q, 1.]
                    ground = dict(world_xyz=q.tolist(), agent_forward_left_m=local[:2].tolist(),
                                  kind='BOUNDARY_RAY_PROXY' if apos['clamped'] else 'NOMINAL_GROUND_RAY_PROXY',
                                  exact_model_metric_waypoint=False)
            old = original_rows[(ep.name, meta['chunk_id'])]
            arc = float(np.linalg.norm(np.diff(raw[:, :2], axis=0), axis=1).sum())
            assert abs(arc-old['raw_arc_m']) < 1e-10
            evidence = stop_evidence(response, manifest)
            assert evidence['wire_stop'] == bool(np.allclose(raw, 0.))
            if evidence['explicit_stop_l0']:
                assert evidence['decoded_all_zero'] and evidence['wire_stop']
            rows.append(dict(episode=ep.name, chunk=meta['chunk_id'], frame=observation['frame_id'],
                             time_s=observation['capture_sim_time_s'], observation_pose=observation['pose_world'],
                             cart_present=metadata['cart_present'], cart_pixels=int(cart.sum()),
                             cart_center_distance_m=float(np.linalg.norm(np.asarray(observation['pose_world'])[:2]-scenario['center_xy'])),
                             raw_arc_m=arc, raw_max_abs_lateral_m=float(np.max(np.abs(raw[:, 1]))),
                             raw_text=response['raw_text'], visible=response.get('visible'),
                             apos=apos, opos=opos, apos_ground=ground, stop=evidence,
                             RGB_path=str(rgb), mask_path=str(mask_path), cart_ids=vis['instance']['matched_instance_ids'],
                             response_path=str(response_path), source_key=f'{ep.name}/{meta["chunk_id"]}',
                             original_geometry=old['actual_input_scene_geometry']['whole']))
    on = [r for r in rows if r['cart_present']]
    stops = [r for r in rows if r['stop']['wire_stop']]
    on_stops = [r for r in on if r['stop']['wire_stop']]
    summary = dict(response_count=len(rows), on_response_count=len(on), stop_count=len(stops),
                   on_stop_count=len(on_stops),
                   all_stops_match_explicit_tuple=all(r['stop']['full_manifest_stop_tuple'] for r in stops),
                   on_stop_OPOS_cell_entirely_cart=sum(r['opos']['grid_cell_cart_fraction'] == 1. for r in on_stops),
                   prior_on_OPOS_center_cart=sum(r['opos']['raster_on_cart'] is True for r in on if not r['stop']['wire_stop']),
                   prior_on_APOS_center_cart=sum(r['apos']['raster_on_cart'] is True for r in on if not r['stop']['wire_stop']),
                   original_episode_summaries=[e['summary'] for e in original['episodes']],
                   geometric_side_checks=read(tracked(run / 'side_passages.json')),
                   new_model_calls=0, new_MPC_solves=0, new_GP_solves=0, new_rollouts=0, new_renders=0,
                   conclusion='EXPLICIT_UPSTREAM_STOP; TARGET_CART_OVERLAP_AT_STOP; INTERNAL_CAUSE_NOT_IDENTIFIED',
                   ambiguity='Target-object confusion and occlusion/blocked-route stopping are not causally separated.')
    return dict(rows=rows, summary=summary, input_sha256=hashes)


def plots(out, result):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from PIL import Image
    for name, chosen, shape in [
        ('on_pointing_sequence', [r for r in result['rows'] if r['cart_present'] and int(r['chunk'][-3:]) in (0, 4, 5, 6)], (2, 4)),
        ('stop_comparison', [r for r in result['rows'] if r['stop']['wire_stop']], (2, 2)),
    ]:
        fig, axes = plt.subplots(*shape, figsize=(4.8*shape[1], 4.2*shape[0]), layout='constrained')
        for ax, r in zip(axes.flat, chosen):
            ax.imshow(Image.open(r['RGB_path']))
            mask = np.load(r['mask_path'])['mask']
            selected = np.isin(mask, r['cart_ids'])
            if selected.any():
                ax.contour(selected.astype(float), levels=[.5], colors=['yellow'], linewidths=.7)
            for channel, color, marker in [('apos', 'cyan', 'o'), ('opos', '#ff3355', 'x')]:
                p = r[channel]
                if p['pixel'] is not None:
                    ax.scatter(*p['pixel'], color=color, marker=marker, s=70, linewidths=2)
                    x0, y0, x1, y1 = p['grid_cell_xyxy']
                    ax.add_patch(Rectangle((x0, y0), x1-x0, y1-y0, fill=False, edgecolor=color, linewidth=1))
            ax.set_title(f'{r["episode"]} / {r["chunk"]}\narc={r["raw_arc_m"]:.3f} m; stop={r["stop"]["wire_stop"]}', fontsize=10)
            ax.set_xlabel(f'APOS: {r["apos"]["state"]}; clamped={r["apos"]["clamped"]}\nOPOS cell cart fraction: {r["opos"]["grid_cell_cart_fraction"]:.2f}', fontsize=9)
            ax.set_xticks([]); ax.set_yticks([])
        fig.suptitle('SAVED RGB DIAGNOSTIC ONLY | cyan=APOS; red=OPOS; yellow=cart mask\nRaster overlap does not prove hidden target identity. No new model call.', fontsize=12)
        fig.savefig(out / f'{name}.png', dpi=150)
        plt.close(fig)
        write_new(out / f'{name}.json', dict(records=chosen, input_sha256=result['input_sha256']))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--validate', action='store_true')
    args = parser.parse_args()
    result = analyze(args.run)
    if args.validate:
        assert read(args.output / 'analysis.json') == result
        for name in ['on_pointing_sequence', 'stop_comparison']:
            side = read(args.output / f'{name}.json')
            assert side['input_sha256'] == result['input_sha256']
            rows = {r['source_key']: r for r in result['rows']}
            assert all(rows[r['source_key']] == r for r in side['records'])
        saved = read(args.output / 'validation.json')
        assert all(digest(path) == h for path, h in saved['output_sha256'].items())
        print('PASS: saved records, hashes, transforms, STOP codes, masks and plot sidecars recomputed; zero calls')
        return
    args.output.mkdir(parents=True, exist_ok=False)
    write_new(args.output / 'analysis.json', result)
    plots(args.output, result)
    write_new(args.output / 'provenance.json', dict(
        audit_sha=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        audit_code_sha256=digest(__file__), source_run=str(args.run.resolve()),
        mode='POST_HOC_SAVED_ONLY_AUDIT', nominal_ground_z_m=0, units='metres/radians/pixels',
        no_source_edits=True, observations_not_internal_reasoning=True))
    (args.output / 'index.html').write_text('<meta charset="utf-8"><title>Saved STOP audit</title>'
        '<h1>Saved STOP / pointing audit</h1><p>No new inference or execution. '
        'Cart overlap supports a target-grounding hypothesis but does not identify its cause.</p>'
        '<img width="100%" src="on_pointing_sequence.png"><img width="100%" src="stop_comparison.png">'
        '<a href="analysis.json">All numeric records and input hashes</a>')
    assert analyze(args.run) == result
    write_new(args.output / 'validation.json', dict(valid=True, saved_only_recomputed=True,
        response_count=len(result['rows']), new_calls=0,
        output_sha256={str(p.resolve()): digest(p) for p in args.output.iterdir() if p.is_file()}))
    print(json.dumps({k: v for k, v in result['summary'].items() if k not in ('original_episode_summaries', 'geometric_side_checks')}, indent=2))


if __name__ == '__main__':
    main()

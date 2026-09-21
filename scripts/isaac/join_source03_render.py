#!/usr/bin/env python3
"""Actual Hospital lighting/instance preflight. No model, MPC or physics steps.

Only an in-memory USD fill light is added; authored assets remain read-only.
Paired camera assignments are diagnostic renders, never executed trajectories.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts/isaac')]
from reconciliation.join_source03 import read, save, sha, luminance_statistics, lighting_gate


def lights(stage):
    from pxr import Usd, UsdLux
    rows = []
    for p in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if not p.HasAPI(UsdLux.LightAPI):
            continue
        api = UsdLux.LightAPI(p)
        rows.append(dict(path=str(p.GetPath()), type=p.GetTypeName(),
                         instance_proxy=p.IsInstanceProxy(),
                         intensity=api.GetIntensityAttr().Get(), exposure=api.GetExposureAttr().Get(),
                         color=list(api.GetColorAttr().Get()),
                         attributes={a.GetName(): str(a.Get()) for a in p.GetAttributes()}))
    return rows


def protected_scene_signature(stage):
    """Hash all non-light authored/composed attributes, topology and materials.

Exclude only new diagnostic render infrastructure; light intensity is the sole
allowed difference on existing prims. No camera/material/shadow exceptions.
"""
    from pxr import Usd
    rows = []
    for p in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if not str(p.GetPath()).startswith(('/World/Hospital', '/World/JOINSource02Obstacle', '/World/LogicalAgent')):
            continue
        attrs = [(a.GetName(), str(a.Get())) for a in p.GetAttributes() if a.GetName() != 'inputs:intensity']
        rels = [(r.GetName(), [str(t) for t in r.GetTargets()]) for r in p.GetRelationships()]
        rows.append((str(p.GetPath()), p.GetTypeName(), attrs, rels))
    return hashlib.sha256(json.dumps(rows, sort_keys=True).encode()).hexdigest()


def render_diagnostic_inputs(run, out, cfg, config, historical, placement, manifest,
                             stage, agent, camera, rgb, inst, sem, app, fill):
    """All conditional images/positions prepared before any new model output."""
    from copy import deepcopy
    import math
    import numpy as np
    import omni.timeline
    import omni.replicator.core as rep
    from pxr import UsdGeom
    from robotless_old_consistent_observation import set_precise_agent_pose
    from join_source02_preflight import create_prop, set_present, snapshot
    from join_source02_collect import final_frame
    ep = historical/'development/history_acquisition/episodes/location_01'
    frames = [json.loads(x) for x in (ep/'capture.jsonl').read_text().splitlines()]
    for f in frames:
        f['path'] = str((ep/f['path']).resolve())
        f['camera_id'] = config['camera']['prim_path']
    target = next(f for f in frames if f['frame_id'] == 'frame_000032')
    off_path = np.load(historical/'development/location_01__placement_01__H16/branches/A/chunks/terminal/world.npy')
    pose = np.asarray(placement['observation_pose_world'])
    connected = np.vstack([pose, off_path])
    lengths = np.linalg.norm(np.diff(connected[:, :2], axis=0), axis=1)
    arcs = np.r_[0, np.cumsum(lengths)]
    items = [('core_dark_H16', pose, placement['center_xy'], False, 16, manifest['history'], manifest['final_frames']['A']['capture_sim_time_s']),
             ('core_bright_H16', pose, placement['center_xy'], True, 16, manifest['history'], manifest['final_frames']['A']['capture_sim_time_s'])]
    tp = np.asarray(target['pose_world'])
    tc = tp[:2]+.9830290731378802*np.array([np.cos(tp[2]), np.sin(tp[2])])
    for count in (16, 32):
        history = frames[33-count:32]
        items.append((f'target_bright_H{count}', tp, tc, True, count, history, target['capture_sim_time_s']))
    for label, arc in zip(('near', 'medium', 'far'), cfg['distance']['center_arc_m']):
        i = min(int(np.searchsorted(arcs, arc, side='right')-1), len(lengths)-1)
        if not 0 <= arc <= arcs[-1]:
            raise ValueError('distance diagnostic cannot extrapolate source path')
        a = (arc-arcs[i])/lengths[i]
        center = connected[i, :2]*(1-a)+connected[i+1, :2]*a
        items.append((f'distance_{label}_H16', pose, center, True, 16, manifest['history'], manifest['final_frames']['A']['capture_sim_time_s']))
    inputs = run/'paired_inputs'
    inputs.mkdir(exist_ok=False)
    for cid, p, center, bright, count, history, sim in items:
        folder = inputs/cid
        folder.mkdir()
        set_precise_agent_pose(agent, p, z_m=config['agent']['z_m'])
        omni.timeline.get_timeline_interface().set_current_time(float(sim))
        UsdGeom.Imageable(fill.GetPrim()).MakeVisible() if bright else UsdGeom.Imageable(fill.GetPrim()).MakeInvisible()
        stage.RemovePrim(placement['prop']['runtime_prim'])
        prop = create_prop(stage, placement['prop']['source_prim'], center,
                           placement['prop']['desired_root_yaw_rad'])
        with (folder/'triangles.npz').open('xb') as f:
            np.savez_compressed(f, triangles=prop['triangles'])
        recs, finals, protected = {}, {}, {}
        for branch, present in [('A', False), ('B', True)]:
            set_present(prop, present)
            rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=True)
            before = protected_scene_signature(stage)
            rec = snapshot(folder, branch, config, app, agent, camera, rgb, inst, sem,
                           prefix=placement['prop']['runtime_prim'])
            if before != protected_scene_signature(stage):
                raise ValueError('capture mutated geometry/camera/material')
            recs[branch], protected[branch] = rec, before
            finals[branch] = final_frame(rec, folder, cid+'_'+branch, rec['simulation_time_s'], config['camera']['prim_path'])
            # Original frame capture/observation anchoring remains explicit.
            finals[branch]['diagnostic_re_render_of_recorded_pose'] = True
        finals['SHAM'] = deepcopy(finals['A'])
        target_counts = {}
        for b, rec in recs.items():
            target_counts[b] = sum(x['pixels'] for x in rec['visible_scene_objects']
                                    if x['prim_path'].startswith(cfg['source']['target_prim']))
        save(folder/'render_manifest.json', dict(condition_id=cid, history_count=count,
            history=history, final_frames=finals, instruction=cfg['source']['instruction'],
            bright=bright, prop=prop['record'], triangles_path=str((folder/'triangles.npz').resolve()),
            environment_export=str((ROOT/cfg['environment']).resolve()),
            target_xy=cfg['source']['target_xy'], target_pixels=target_counts,
            cart_pixels={b:r['instance']['visible_pixels'] for b,r in recs.items()},
            protected_signatures=protected, center_xy=np.asarray(center).tolist(),
            observation_pose_world=p.tolist(), source_no_obstacle_path=off_path.tolist(),
            declared_before_model_output=True, model_calls=0, MPC_calls=0))
    save(out/'prepared_inputs.json', dict(condition_ids=[x[0] for x in items],
        source_capture_jsonl=str(ep/'capture.jsonl'), source_sha256=sha(ep/'capture.jsonl'),
        authentic_frame_count=len(frames), new_model_calls=0, new_MPC_calls=0,
        history_not_brightened=True, no_history_cloning=True))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--mode', choices=['inspect', 'render', 'inputs'], required=True)
    p.add_argument('--attempt', default=None, help='new technical output only; refuses overwrite')
    args = p.parse_args()
    run = args.run.resolve()
    if read(run/'mpc_audit/result.json')['status'] != 'MPC_PROVENANCE_MATCHES_PINNED_OFFICIAL':
        raise RuntimeError('MPC provenance gate blocks lighting')
    out = run/'technical_preflight'/(args.attempt or args.mode)
    out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    import numpy as np
    import yaml
    config = yaml.safe_load((ROOT/'configs/robotless_online_handoffs.yaml').read_text())
    historical = ROOT/'data/robotless_join_source_02/source_20260921T112111Z'
    placement = read(historical/'development/paired_inputs/location_01/placement_01/placement.json')
    manifest = read(historical/'development/input_manifests/location_01__placement_01__H16.json')
    from isaacsim import SimulationApp
    app = SimulationApp(dict(headless=True, renderer='RayTracedLighting', anti_aliasing=0, display_options=0))
    try:
        from pxr import UsdLux, UsdGeom, Gf
        import omni.timeline
        import omni.replicator.core as rep
        from robotless_runtime import runtime_scene
        from robotless_old_consistent_observation import set_precise_agent_pose
        from join_source02_preflight import create_prop, set_present, same_product_annotators, snapshot, instance_data
        pose = np.asarray(placement['observation_pose_world'])
        stage, agent, camera, rgb, scene = runtime_scene(config, pose, app=app)
        set_precise_agent_pose(agent, pose, z_m=config['agent']['z_m'])
        omni.timeline.get_timeline_interface().set_current_time(manifest['final_frames']['A']['capture_sim_time_s'])
        inst, sem, product = same_product_annotators(rgb)
        prop = create_prop(stage, placement['prop']['source_prim'], placement['center_xy'], placement['prop']['desired_root_yaw_rad'])
        old_triangles = np.load(placement['triangles_path'])['triangles']
        if not np.array_equal(old_triangles, prop['triangles']):
            raise ValueError('runtime cart differs from historical exact mesh')
        original_lights = lights(stage)
        save(out/'light_inventory.json', original_lights)
        save(out/'scene.json', dict(scene=scene, product=product, prop=prop['record'],
                                   exact_historical_cart_mesh=True, input_postprocessing=False,
                                   renderer='RayTracedLighting', anti_aliasing=0))
        set_present(prop, True)
        # Resolve Isaac's float32 timeline representation before the stable-state
        # measurement. These are setup renders, not extra history/model inputs.
        rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=True)
        dark = snapshot(out, 'DARK_ON', config, app, agent, camera, rgb, inst, sem, prefix=placement['prop']['runtime_prim'])
        if args.mode == 'inspect':
            save(out/'result.json', dict(model_calls=0, MPC_calls=0, lights=len(original_lights),
                                        wall_s=time.monotonic()-started))
            return
        cfg = yaml.safe_load((ROOT/'configs/join_source_03_bright_cause.yaml').read_text())
        # Frozen before any diagnostic prediction. Scalar applied once, not a sweep.
        light_config = cfg['lighting']
        before = protected_scene_signature(stage)
        fill = UsdLux.RectLight.Define(stage, light_config['prim'])
        fill.CreateIntensityAttr(light_config['intensity'])
        fill.CreateExposureAttr(light_config['exposure'])
        fill.CreateColorAttr(Gf.Vec3f(*light_config['color']))
        fill.CreateNormalizeAttr(light_config['normalize'])
        fill.CreateWidthAttr(light_config['width_m'])
        fill.CreateHeightAttr(light_config['height_m'])
        UsdGeom.Xformable(fill.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*light_config['translation_world_m']))
        after = protected_scene_signature(stage)
        if before != after:
            raise ValueError('non-intensity scene/camera/material change')
        save(out/'lighting_mutation.json', dict(original=original_lights, bright=lights(stage),
            protected_before=before, protected_after=after, only_added_fill_light=True,
            profile=light_config, authored_layers_saved=False, shader_or_shadow_settings_changed=False))
        bright = snapshot(out, 'BRIGHT_ON', config, app, agent, camera, rgb, inst, sem, prefix=placement['prop']['runtime_prim'])
        from PIL import Image
        stats = {}
        for name, rec in [('DARK', dark), ('BRIGHT', bright)]:
            a = np.asarray(Image.open(out/rec['rgb_file']).convert('RGB'))
            mask = np.load(out/rec['mask_file'])['mask']
            labels = rec['instance']['idToLabels']
            target_ids = [int(i) for i, path in labels.items() if path.startswith(cfg['source']['target_prim'])]
            cart_mask = np.isin(mask, rec['instance']['matched_instance_ids'])
            target_mask = np.isin(mask, target_ids)
            stats[name] = dict(whole=luminance_statistics(a), cart=luminance_statistics(a, cart_mask),
                target=luminance_statistics(a, target_mask), cart_pixels=int(cart_mask.sum()),
                target_pixels=int(target_mask.sum()), rgb_sha256=sha(out/rec['rgb_file']))
        gate = lighting_gate(stats['DARK'], stats['BRIGHT'], cfg['brightness_acceptance'])
        save(out/'luminance.json', stats)
        save(out/'validation.json', dict(**gate, model_calls=0, MPC_calls=0, wall_s=time.monotonic()-started))
        if not gate['valid']:
            raise RuntimeError('TECHNICAL_LIGHTING_BLOCKER: frozen image-quality gate failed')
        if args.mode == 'inputs':
            render_diagnostic_inputs(run, out, cfg, config, historical, placement, manifest,
                                     stage, agent, camera, rgb, inst, sem, app, fill)
    except BaseException as exc:
        save(out/'technical_failure.json', dict(error=f'{type(exc).__name__}: {exc}', new_model_calls=0, new_MPC_calls=0))
        raise
    finally:
        app.close()


if __name__ == '__main__':
    main()

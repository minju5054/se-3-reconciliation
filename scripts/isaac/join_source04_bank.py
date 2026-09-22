#!/usr/bin/env python3
"""Re-render 16 recorded moving poses, paired cart OFF/ON, no motion/model call."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts/isaac')]
from reconciliation.join_source03 import read,save,sha


def pair_signature(stage):
    from pxr import Usd
    rows=[]
    for p in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies()):
        path=str(p.GetPath())
        if not path.startswith(('/World/Hospital','/World/JOINSource02Obstacle','/World/LogicalAgent','/World/JOINSource03Fill')):continue
        attrs=[(a.GetName(),str(a.Get())) for a in p.GetAttributes()
               if not (path.startswith('/World/JOINSource02Obstacle') and a.GetName()=='visibility')]
        rows.append((path,p.GetTypeName(),attrs,[(r.GetName(),list(map(str,r.GetTargets()))) for r in p.GetRelationships()]))
    return hashlib.sha256(json.dumps(rows,sort_keys=True).encode()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    args=p.parse_args();run=args.run.resolve();out=run/'frame_bank';out.mkdir(exist_ok=False)
    import numpy as np
    import yaml
    cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    src=read(run/'source03_input.json');poses=read(run/'pose_lineage.json')
    from isaacsim import SimulationApp
    app=SimulationApp(dict(headless=True,renderer='RayTracedLighting',anti_aliasing=0,display_options=0))
    started=time.monotonic()
    try:
        import omni.replicator.core as rep
        import omni.timeline
        from pxr import UsdLux,UsdGeom,Gf
        from robotless_runtime import runtime_scene
        from robotless_old_consistent_observation import set_precise_agent_pose
        from join_source02_preflight import create_prop,set_present,same_product_annotators,snapshot
        from join_source03_render import lights
        stage,agent,camera,rgb,scene=runtime_scene(cfg,np.asarray(poses[0]['pose_world']),app=app)
        inst,sem,product=same_product_annotators(rgb)
        depth=rep.AnnotatorRegistry.get_annotator('distance_to_camera');depth.attach(product)
        prop=create_prop(stage,src['prop']['source_prim'],src['center_xy'],src['prop']['desired_root_yaw_rad'])
        assert np.array_equal(prop['triangles'],np.load(src['triangles_path'])['triangles']), 'cart mesh differs'
        assert prop['record']['wrapper_matrix_column']==src['prop']['wrapper_matrix_column'], 'cart transform differs'
        with (out/'triangles.npz').open('xb') as f:np.savez_compressed(f,triangles=prop['triangles'])
        original=lights(stage);lc=cfg['lighting']
        fill=UsdLux.RectLight.Define(stage,lc['prim'])
        fill.CreateIntensityAttr(lc['intensity']);fill.CreateExposureAttr(lc['exposure'])
        fill.CreateColorAttr(Gf.Vec3f(*lc['color']));fill.CreateNormalizeAttr(lc['normalize'])
        fill.CreateWidthAttr(lc['width_m']);fill.CreateHeightAttr(lc['height_m'])
        UsdGeom.Xformable(fill.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*lc['translation_world_m']))
        save(out/'scene.json',dict(scene=scene,product=product,original_lights=original,bright_lights=lights(stage),
            profile=lc,prop=prop['record'],new_model_MPC_GP_calls=0,authored_scene_saved=False,
            depth_annotator='distance_to_camera',depth_semantics='Euclidean camera-to-first-rendered-surface distance in metres'))
        bank=[]
        for i,source in enumerate(poses):
            folder=out/f'{i:02}';folder.mkdir()
            pose=np.asarray(source['pose_world']);set_precise_agent_pose(agent,pose,z_m=cfg['agent']['z_m'])
            omni.timeline.get_timeline_interface().set_current_time(source['capture_sim_time_s'])
            row=dict(index=i,source_frame_id=source['frame_id'],source_pose_world=pose.tolist(),source=source)
            for name,present in [('OFF',False),('ON',True)]:
                set_present(prop,present)
                rep.orchestrator.step(rt_subframes=1,delta_time=0.,pause_timeline=True)
                sig=pair_signature(stage)
                rec=snapshot(folder,name,cfg,app,agent,camera,rgb,inst,sem,prefix=src['prop']['runtime_prim'])
                assert sig==pair_signature(stage),'unexpected scene mutation during capture'
                d=np.asarray(depth.get_data()).copy()
                if d.shape!=(270,480) or not np.issubdtype(d.dtype,np.floating):
                    raise ValueError('depth not synchronized HxW float raster')
                with (folder/(name+'_depth.npz')).open('xb') as f:np.savez_compressed(f,distance_m=d)
                # Exact source pose semantics, separate from newly captured host time.
                assert np.allclose(rec['camera']['T_world_camera'],source['camera']['T_world_camera'],atol=1e-12,rtol=0),'source camera drift'
                frame=dict(frame_id=source['frame_id'],path=str(folder/(name+'.jpg')),sha256=rec['rgb_jpeg_sha256'],
                    rendered_state_id=f'pose_{i:02}',capture_sim_time_s=rec['simulation_time_s'],
                    capture_monotonic_ns=rec['capture_completed_monotonic_ns'],capture_host_utc=rec['capture_completed_utc'],
                    episode_time_s=source.get('episode_time_s',source['capture_sim_time_s']),pose_world=rec['agent_pose_world'],
                    camera=rec['camera'],camera_id=cfg['camera']['prim_path'],bootstrap=False,stationary=False,
                    resolution_width_height=[480,270],all_bright=True,counterfactual_moving_pose_history=True,
                    original_source_capture_sim_time_s=source['capture_sim_time_s'],original_source_rgb_sha256=source['sha256'],
                    presence=present,render_state_stable=True)
                target_pixels=sum(o['pixels'] for o in rec['visible_scene_objects'] if o['prim_path'].startswith(cfg['source']['target_prim']))
                row[name]=dict(frame=frame,camera=rec['camera'],instance=rec['instance'],target_pixels=target_pixels,
                    cart_pixels=rec['instance']['visible_pixels'],cart_prefix=src['prop']['runtime_prim'],
                    cart_transform=deepcopy(prop['record']['wrapper_matrix_column']),present=present,
                    scene_signature=sig,lighting_sha256=hashlib.sha256(json.dumps(lc,sort_keys=True).encode()).hexdigest(),
                    stable=rec['stable_camera_pose_and_simulation_time'],same_product=rec['same_render_product'],
                    mask_path=str(folder/(name+'_instance.npz')),depth_path=str(folder/(name+'_depth.npz')),
                    capture_record=str(folder/(name+'.json')),depth_same_render_state=True)
            assert row['OFF']['scene_signature']==row['ON']['scene_signature'],'pair differs beyond cart visibility'
            assert row['OFF']['camera']==row['ON']['camera'],'pair camera drift'
            bank.append(row);print(dict(frame=i,cart_pixels=row['ON']['cart_pixels']),flush=True)
        save(out/'bank.json',bank)
        save(out/'completed.json',dict(frame_pairs=len(bank),rgb_frames=2*len(bank),wall_s=time.monotonic()-started,
            model_MPC_GP_calls=0,scope='COUNTERFACTUAL MOVING-POSE HISTORY'))
    except BaseException as e:
        save(out/'technical_failure.json',dict(error=f'{type(e).__name__}: {e}',model_MPC_GP_calls=0));raise
    finally:app.close()


if __name__=='__main__':main()

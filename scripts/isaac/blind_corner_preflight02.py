#!/usr/bin/env python3
"""Actual Hospital/static-cart diagnostic renders; zero model/controller calls."""
import argparse
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts/isaac')]
from join_source02_preflight import save, sha


def main():
    import json, numpy as np, yaml
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args()
    run=a.run.resolve(); declaration=json.loads((run/'declaration.json').read_text());cfg=declaration['config']
    assert sha(ROOT/'configs/blind_corner_source_acquisition_02.yaml')==declaration['config_sha256']
    config=yaml.safe_load((ROOT/cfg['source_config']).read_text());out=run/'technical_preflight'
    out.mkdir(exist_ok=False); start=time.monotonic()
    save(out/'invocation.json',dict(script_sha256=sha(__file__),declaration_sha256=sha(run/'declaration.json'),model_calls=0,MPC_calls=0))
    from isaacsim import SimulationApp
    app=SimulationApp(dict(headless=True,renderer='RayTracedLighting',anti_aliasing=0,display_options=0))
    try:
        from pxr import UsdLux,UsdGeom,Usd,Gf
        from robotless_runtime import runtime_scene
        from robotless_old_consistent_observation import set_precise_agent_pose
        from join_source02_preflight import create_prop,set_present,same_product_annotators,snapshot,collect_meshes
        stage,agent,camera,rgb,scene=runtime_scene(config,np.array(declaration['candidates'][0]['approach_pose']),app=app)
        save(out/'scene.json',dict(scene=scene,static_layers={x.identifier:__import__('hashlib').sha256(x.ExportToString().encode()).hexdigest() for x in stage.GetUsedLayers() if not x.anonymous}))
        inst,sem,product=same_product_annotators(rgb)
        lc=config['lighting'];fill=UsdLux.RectLight.Define(stage,lc['prim'])
        fill.CreateIntensityAttr(lc['intensity']);fill.CreateExposureAttr(lc['exposure']);fill.CreateColorAttr(Gf.Vec3f(*lc['color']));fill.CreateNormalizeAttr(lc['normalize'])
        fill.CreateWidthAttr(lc['width_m']);fill.CreateHeightAttr(lc['height_m']);trans=UsdGeom.Xformable(fill.GetPrim()).AddTranslateOp()
        for c in declaration['candidates']:
            folder=out/c['id'];folder.mkdir();root='/World/JOINSource02Obstacle'
            if stage.GetPrimAtPath(root).IsValid():stage.RemovePrim(root) # candidate reset, never episode motion
            prop=create_prop(stage,cfg['cart_source'],c['cart_center_xy'],c['cart_yaw']);set_present(prop,True)
            def state():
                prim=prop['prim'];im=UsdGeom.Imageable(prim)
                return dict(world_matrix_column=np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T.tolist(),
                    authored_visibility=str(im.GetVisibilityAttr().Get()),computed_visibility=str(im.ComputeVisibility()),present=prop['present'])
            initial=state();trans.Set(Gf.Vec3d(*c['fill_translation']))
            with (folder/'triangles.npz').open('xb') as f:np.savez_compressed(f,triangles=prop['triangles'])
            save(folder/'cart.json',prop['record']);records=[]
            for i,pose in enumerate(c['probe_poses']):
                assert state()==initial
                set_precise_agent_pose(agent,np.array(pose),z_m=config['agent']['z_m'])
                r=snapshot(folder,f'probe_{i:02}',config,app,agent,camera,rgb,inst,sem,prefix=root)
                # Preserve snapshot raw record, put static state in an exclusive paired record.
                save(folder/f'probe_{i:02}_static.json',dict(static_cart_state=state(),snapshot_sha256=sha(folder/f'probe_{i:02}.json')))
                records.append(dict(index=i,pixels=r['instance']['visible_pixels'],pose=r['agent_pose_world']))
                print(c['id'],i,records[-1],flush=True)
            finaltri,_=collect_meshes(stage,root)
            save(folder/'manifest.json',dict(candidate=c,records=records,lighting={**lc,'translation_world_m':c['fill_translation']},
                initial_cart_state=initial,final_cart_state=state(),mesh_unchanged=bool(np.array_equal(prop['triangles'],finaltri)),model_calls=0,MPC_calls=0))
        save(out/'complete.json',dict(model_calls=0,MPC_calls=0,completed=True,wall_s=time.monotonic()-start))
    finally:app.close()

if __name__=='__main__':main()

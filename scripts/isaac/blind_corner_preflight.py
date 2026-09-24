#!/usr/bin/env python3
"""Model-free static-cart blind-corner renders. Diagnostic poses are not execution."""
import argparse
import json
from pathlib import Path
import sys
import time
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts/isaac')]
from join_source02_preflight import save, sha

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args()
    run=a.run.resolve();out=run/'technical_preflight';out.mkdir(parents=True,exist_ok=False)
    import yaml,numpy as np
    cfg=yaml.safe_load((ROOT/'configs/blind_corner_source_acquisition_01.yaml').read_text())
    config=yaml.safe_load((ROOT/cfg['source_config']).read_text())
    save(out/'declaration.json',dict(protocol=cfg,config_sha256=sha(ROOT/'configs/blind_corner_source_acquisition_01.yaml'),model_calls=0,MPC_calls=0))
    from isaacsim import SimulationApp
    app=SimulationApp(dict(headless=True,renderer='RayTracedLighting',anti_aliasing=0,display_options=0))
    try:
        from pxr import UsdLux,UsdGeom,Gf
        from robotless_runtime import runtime_scene
        from robotless_old_consistent_observation import set_precise_agent_pose
        from join_source02_preflight import create_prop,set_present,same_product_annotators,snapshot
        stage,agent,camera,rgb,scene=runtime_scene(config,np.array(cfg['candidates'][0]['approach_pose']),app=app)
        save(out/'scene.json',dict(scene=scene,static_layers={x.identifier:__import__('hashlib').sha256(x.ExportToString().encode()).hexdigest() for x in stage.GetUsedLayers() if not x.anonymous}))
        inst,sem,product=same_product_annotators(rgb)
        lc=config['lighting'];fill=UsdLux.RectLight.Define(stage,lc['prim'])
        fill.CreateIntensityAttr(lc['intensity']);fill.CreateExposureAttr(lc['exposure']);fill.CreateColorAttr(Gf.Vec3f(*lc['color']));fill.CreateNormalizeAttr(lc['normalize'])
        fill.CreateWidthAttr(lc['width_m']);fill.CreateHeightAttr(lc['height_m']);trans=UsdGeom.Xformable(fill.GetPrim()).AddTranslateOp()
        for c in cfg['candidates']:
            folder=out/c['id'];folder.mkdir();root='/World/JOINSource02Obstacle'
            if stage.GetPrimAtPath(root).IsValid():stage.RemovePrim(root)
            prop=create_prop(stage,cfg['cart_source'],c['cart_center_xy'],c['cart_yaw'])
            set_present(prop,True) # initialization only; held constant over all probes
            trans.Set(Gf.Vec3d(*c['fill_translation']))
            with (folder/'triangles.npz').open('xb') as f:np.savez_compressed(f,triangles=prop['triangles'])
            save(folder/'cart.json',prop['record']);records=[]
            for i,pose in enumerate(c['probe_poses']):
                set_precise_agent_pose(agent,np.array(pose),z_m=config['agent']['z_m'])
                r=snapshot(folder,f'probe_{i:02}',config,app,agent,camera,rgb,inst,sem,prefix=root)
                records.append(dict(index=i,pixels=r['instance']['visible_pixels'],pose=r['agent_pose_world']))
                print(c['id'],i,records[-1],flush=True)
            save(folder/'manifest.json',dict(candidate=c,records=records,lighting={**lc,'translation_world_m':c['fill_translation']},constant_cart=True,model_calls=0,MPC_calls=0))
        save(out/'complete.json',dict(model_calls=0,MPC_calls=0,completed=True))
    finally:app.close()
if __name__=='__main__':main()

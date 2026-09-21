#!/usr/bin/env python3
"""At most three declared genuine reveal attempts; stop at first qualifier."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import numpy as np
import yaml
from robotless_online_handoffs import Worker,collect_episode
from reconciliation.robotless_online import dump_new,stamp
from reconciliation.gp_se2_join01 import obstacle_on_old


class Reveal:
    def __init__(self,stage,config,placement,segmentation):
        self.stage,self.config,self.placement,self.seg=stage,config,placement,segmentation
        self.record=None

    def before_capture(self,ep,pose,state,active,last_activation):
        if self.record is not None or active is None or state['sim_time_s']-last_activation<self.config['reveal']['minimum_old_active_s']:
            return
        from pxr import Gf,UsdGeom,UsdSemantics
        ctx=active['context'];old=np.load(ep/ctx['world_ref']['path'],allow_pickle=False)
        obstacle=obstacle_on_old(ctx['observation']['pose_world'],old,self.placement['distance_along_old_m'],self.placement['dimensions_m'])
        cube=UsdGeom.Cube.Define(self.stage,obstacle['prim_path']);cube.CreateSizeAttr(1.)
        xf=UsdGeom.Xformable(cube.GetPrim());p=obstacle['pose_world'];d=obstacle['dimensions_m']
        xf.AddTranslateOp().Set(Gf.Vec3d(p[0],p[1],obstacle['center_z_m']))
        xf.AddRotateZOp().Set(float(np.degrees(p[2])));xf.AddScaleOp().Set(Gf.Vec3f(*d))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*self.config['reveal']['color_rgb'])])
        UsdSemantics.LabelsAPI.Apply(cube.GetPrim(),'class').CreateLabelsAttr().Set(['JOIN01Obstacle'])
        # USD readback authenticates the exact runtime primitive for oracle use.
        matrix=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(cube.GetPrim())).T
        self.record=dict(obstacle=obstacle,timestamp=stamp(state['sim_time_s'],state['episode_time_s']),
            state_id=state['state_id'],actual_agent_pose=pose.tolist(),placement=self.placement,
            usd_world_matrix=matrix.tolist(),usd_cube_size=float(cube.GetSizeAttr().Get()),
            hospital_asset_modified=False,rigid_body_dynamics=False)
        dump_new(ep/'obstacle_reveal.json',self.record)

    def after_capture(self,ep,frame):
        data=self.seg.get_data();pixels=0;labels={}
        if isinstance(data,dict):
            labels=data.get('info',{}).get('idToLabels',{})
            ids=[int(k) for k,v in labels.items() if 'JOIN01Obstacle' in str(v)]
            mask=np.asarray(data['data'])
            pixels=int(np.count_nonzero(np.isin(mask,ids)))
            folder=ep/'visibility';folder.mkdir(exist_ok=True)
            with (folder/(frame['frame_id']+'.npz')).open('xb') as stream:
                np.savez_compressed(stream,mask=mask)
        dump_new(ep/'visibility'/(frame['frame_id']+'.json'),dict(frame_id=frame['frame_id'],
            obstacle_pixels=pixels,labels=labels,revealed=self.record is not None,
            source='actual same-camera semantic segmentation; not painted RGB',
            capture_sim_time_s=frame['capture_sim_time_s']))

    def allow_fresh(self,frame):
        return self.record is not None and frame['capture_sim_time_s']>=self.record['timestamp']['sim_time_s']


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();run=a.run.resolve()
    subprocess.run([str(ROOT/'.venv/bin/python'),str(ROOT/'scripts/run_gp_se2_join01.py'),'verify','--run',str(run)],check=True)
    config=yaml.safe_load((run/'config_snapshot.yaml').read_text());protocol=json.loads((run/'protocol.json').read_text())
    source=run/'source_event';source.mkdir();(source/'episodes').mkdir();(source/'logs').mkdir()
    checkout=(ROOT/config['paths']['lightnav_checkout']).resolve()
    model=Worker([str(checkout/'.venv/bin/python'),str(ROOT/'scripts/online_lightnav_worker.py'),'--config',str(run/'config_snapshot.yaml')],source/'logs/model.log')
    mpc=Worker([str(checkout/'mujoco_demo/.venv/bin/python'),str(ROOT/'scripts/online_mpc_worker.py'),'--lightnav-checkout',str(checkout)],source/'logs/mpc.log')
    app=None
    try:
        mr=model.await_type('ready');cr=mpc.await_type('ready')
        dump_new(source/'workers.json',dict(model=mr,mpc=cr))
        from isaacsim import SimulationApp
        app=SimulationApp(dict(headless=True,renderer='RayTracedLighting',anti_aliasing=0,display_options=0))
        import omni.replicator.core as rep
        from robotless_runtime import runtime_scene
        from isaacsim.core.api import World
        stage,agent,camera,rgb,scene=runtime_scene(config,np.array(protocol['initial_pose_world']),app=app)
        product=rep.create.render_product(config['camera']['prim_path'],(config['camera']['resolution_width'],config['camera']['resolution_height']))
        seg=rep.AnnotatorRegistry.get_annotator('semantic_segmentation',init_params={'colorize':False});seg.attach(product)
        world=World(physics_dt=1/60,rendering_dt=1/60,stage_units_in_meters=1.);world.reset()
        dump_new(source/'scene.json',scene)
        records=[]
        for placement in protocol['placements']:
            if stage.GetPrimAtPath('/World/JOIN01Obstacle').IsValid():stage.RemovePrim('/World/JOIN01Obstacle')
            spec=dict(episode_id=placement['id'],R0=protocol['initial_pose_world'],instruction=protocol['instruction'],placement=placement)
            hook=Reveal(stage,protocol,placement,seg)
            metadata=collect_episode(spec,source,config,app,world,agent,camera,rgb,scene,model,mpc,None,mr,cr,intervention=hook)
            subprocess.run([str(ROOT/'.venv/bin/python'),str(ROOT/'scripts/run_gp_se2_join01.py'),'qualify','--run',str(run),'--episode',placement['id']],check=True)
            q=json.loads((run/'qualification'/(placement['id']+'.json')).read_text());records.append(dict(placement=placement['id'],qualified=q['qualified'],failure_reasons=q['failure_reasons']))
            if q['qualified']:break
            if q.get('technical_blocker',False):
                break
        dump_new(run/'collection_result.json',dict(attempts=records,selected_episode=next((r['placement'] for r in records if r['qualified']),None),
            status='QUALIFIED' if any(r['qualified'] for r in records) else 'TECHNICAL_RUNTIME_BLOCKER' if q.get('technical_blocker',False) else 'NO_QUALIFYING_OBSTACLE_REVEAL_HANDOFF'))
    finally:
        model.shutdown('shutdown');mpc.shutdown()
        if app:app.close()


if __name__=='__main__':main()

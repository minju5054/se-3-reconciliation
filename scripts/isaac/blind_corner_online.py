#!/usr/bin/env python3
"""One frozen genuine episode; cart initialized ON and never mutated thereafter."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.robotless_online import stamp
from reconciliation.blind_corner_source import FirstCrossing
from robotless_online_handoffs import Worker,collect_episode
from join_online02_collect import setup_scene,GeometryWorker,Visibility

class StaticOcclusion(Visibility):
    def __init__(self,inst,prop,geometry):
        super().__init__(inst,prop,True);self.geometry=geometry;self.latch=FirstCrossing();self.saved=False
        self.initial=self.static_state();self.check_count=0
    def static_state(self):
        from pxr import UsdGeom,Usd
        p=self.prop['prim'];matrix=np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T.tolist()
        return dict(world_matrix_column=matrix,computed_visibility=str(UsdGeom.Imageable(p).ComputeVisibility()),
            authored_visibility=str(UsdGeom.Imageable(p).GetVisibilityAttr().Get()),oracle_present=bool(self.geometry.present),logical_present=bool(self.prop['present']))
    def assert_static(self):
        state=self.static_state();assert state==self.initial,'static obstacle mutated'
        assert state['computed_visibility']=='inherited' and state['oracle_present'] and state['logical_present']
        self.check_count+=1;return state
    def before_capture(self,*args):
        self.assert_static()
        if len(self.latch.ids)>=64:raise RuntimeError('frozen history budget exhausted')
    def after_capture(self,ep,frame):
        super().after_capture(ep,frame)
        from join_source02_preflight import instance_data
        _,rec=instance_data(self.inst,self.prop['record']['runtime_prim'])
        cross=self.latch.observe(frame['frame_id'],rec['visible_pixels'],frame['rendered_state_id'])
        if cross and not self.saved:
            save(ep/'first_visibility_crossing.json',dict(**cross,observation=frame,
                rule='FIRST scheduled <20 to >=20; no later substitution',static_state=self.assert_static()))
            self.saved=True
    def allow_fresh(self,frame):return self.latch.allows(frame['frame_id'])
    def guard(self,pose,command,dt):
        state=self.assert_static();r=self.geometry.guard(pose,command,dt);r['static_obstacle_state']=state;return r


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--candidate',required=True);a=p.parse_args();parent=a.run.resolve()
    env={k:v for k,v in os.environ.items() if k not in ('LD_LIBRARY_PATH','PYTHONPATH')}
    subprocess.run([str(ROOT/'.venv/bin/python'),str(ROOT/'scripts/run_blind_corner_source.py'),'--run',str(parent),'--mode','verify'],check=True,env=env)
    protocol=read(parent/'protocol.json');assert a.candidate in protocol['eligible_order']
    for cid in protocol['eligible_order'][:protocol['eligible_order'].index(a.candidate)]:assert (parent/'candidates'/cid/'completion.json').exists()
    assert read(parent/'generation_actual.json')['valid']
    run=parent/'candidates'/a.candidate;config=yaml.safe_load((run/'config_snapshot.yaml').read_text());spec=read(run/'episode_schedule.json')['episodes'][0]
    save(run/'execution_start.json',dict(at=stamp(),sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),candidate=a.candidate))
    from isaacsim import SimulationApp
    app=SimulationApp(dict(headless=True,renderer='RayTracedLighting',anti_aliasing=0,display_options=0))
    model=mpc=geo=hook=None;start=time.monotonic()
    try:
        stage,agent,camera,rgb,inst,sem,prop,scene=setup_scene(run,config,spec['R0'],app)
        from join_source02_preflight import set_present,collect_meshes
        set_present(prop,True) # sole activation, before World reset/session/episode
        scene['exact_preflight_cart']=scene.pop('source04_exact_cart')
        save(run/'acquisition_scene.json',scene)
        from isaacsim.core.api import World
        world=World(physics_dt=1/60,rendering_dt=1/60,stage_units_in_meters=1.);world.reset()
        checkout=(ROOT/config['paths']['lightnav_checkout']).resolve()
        model=Worker([str(checkout/'.venv/bin/python'),str(ROOT/'scripts/online_lightnav_worker.py'),'--config',str(run/'config_snapshot.yaml')],run/'logs/lightnav_worker.log')
        mpc=Worker([str(checkout/'mujoco_demo/.venv/bin/python'),str(ROOT/'scripts/online_mpc_worker.py'),'--lightnav-checkout',str(checkout)],run/'logs/mpc_worker.log')
        geo=GeometryWorker(run);geo.set(True);hook=StaticOcclusion(inst,prop,geo)
        mr=model.await_type('ready');cr=mpc.await_type('ready');save(run/'workers.json',dict(model=mr,mpc=cr))
        save(run/'static_initial.json',dict(state=hook.assert_static(),at=stamp(),initialization_only_activation=True,triangles_sha256=sha(read(run/'scenario.json')['triangles_path'])))
        try:
            result=collect_episode(spec,run,config,app,world,agent,camera,rgb,scene,model,mpc,None,mr,cr,intervention=hook,command_guard=hook.guard)
            row=dict(candidate_id=a.candidate,status=result['status'],predictions=result['predictions_requested'])
        except BaseException as e:row=dict(candidate_id=a.candidate,status='TECHNICAL_RUNTIME_FAILURE',error=repr(e))
        ep=run/'episodes'/a.candidate;hook.close()
        finaltri,_=collect_meshes(stage,prop['record']['runtime_prim']);same=np.array_equal(finaltri,prop['triangles']);assert same
        save(run/'static_final.json',dict(state=hook.assert_static(),check_count=hook.check_count,exact_mesh_unchanged=same,at=stamp()))
        files=[ep/'guard.jsonl',ep/'visibility.jsonl',*sorted((ep/'visibility').glob('*'))]
        if (ep/'first_visibility_crossing.json').exists():files.append(ep/'first_visibility_crossing.json')
        save(ep/'acquisition_extension_manifest.json',dict(files={str(q.relative_to(ep)):sha(q) for q in files if q.exists()}))
        save(run/'completion.json',dict(**row,wall_s=time.monotonic()-start,at=stamp(),no_retry=True));print(row,flush=True)
    finally:
        if hook:hook.close()
        if model:model.shutdown('shutdown')
        if mpc:mpc.shutdown()
        if geo:geo.close()
        app.close()
if __name__=='__main__':main()

#!/usr/bin/env python3
"""Exactly two genuine fixed-POSE11 sudden reveals after pushed PhaseB freeze."""
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
from reconciliation.obstacle_source_online import reveal_due,first_post_reveal_allowed,REPETITIONS
from robotless_online_handoffs import Worker,collect_episode
from join_online02_collect import setup_scene,GeometryWorker,Visibility


class SuddenReveal(Visibility):
    def __init__(self,inst,prop,geometry,protocol):
        super().__init__(inst,prop,False)
        self.geometry,self.protocol=geometry,protocol
        self.reveal=None;self.eligible=None

    def before_capture(self,ep,pose,state,active,last_activation):
        if not reveal_due(pose,self.protocol['reveal_plane_pose_world'],self.protocol['hallway_forward'],active is not None,self.reveal is not None):return
        from join_source02_preflight import set_present
        before=stamp(state['sim_time_s'],state['episode_time_s'])
        set_present(self.prop,True);self.geometry.set(True);self.present=True
        self.reveal=dict(start=before,completed=stamp(state['sim_time_s'],state['episode_time_s']),
            state_id=state['state_id'],pose_world=pose.tolist(),cart_transform=self.prop['record']['wrapper_matrix_column'],
            old_chunk_id=active['chunk_id'],old_active_since_sim_s=last_activation,
            rendering_present=True,oracle_present=True,cart_pose_changed=False)
        save(ep/'obstacle_reveal.json',self.reveal)

    def after_capture(self,ep,frame):
        super().after_capture(ep,frame)
        if self.reveal is not None and self.eligible is None:
            self.eligible=frame['frame_id']
            save(ep/'first_post_reveal_observation.json',dict(frame_id=self.eligible,state_id=frame['rendered_state_id'],
                capture_sim_time_s=frame['capture_sim_time_s'],pose_world=frame['pose_world'],cart_present=True))

    def allow_fresh(self,frame):
        return first_post_reveal_allowed(frame,None if self.reveal is None else self.reveal['state_id'],self.eligible)


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();parent=a.run.resolve();run=parent/'phaseB'
    env={k:v for k,v in os.environ.items() if k not in ('LD_LIBRARY_PATH','PYTHONPATH')}
    subprocess.run([str(ROOT/'.venv/bin/python'),str(ROOT/'scripts/run_obstacle_source02_online.py'),'--run',str(parent),'--mode','verify'],check=True,env=env)
    assert read(parent/'phase0/qualification.json')['qualified'] and read(parent/'generation_actual.json')['valid']
    assert not (run/'execution_start.json').exists(),'no retry'
    save(run/'execution_start.json',dict(at=stamp(),sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()))
    config=yaml.safe_load((run/'config_snapshot.yaml').read_text());specs=read(run/'episode_schedule.json')['episodes'];protocol=read(run/'protocol.json')
    assert [r['episode_id'] for r in specs]==list(REPETITIONS)
    from isaacsim import SimulationApp
    app=SimulationApp(dict(headless=True,renderer='RayTracedLighting',anti_aliasing=0,display_options=0))
    model=mpc=geo=None;start=time.monotonic();ledger=[]
    try:
        stage,agent,camera,rgb,inst,sem,prop,scene=setup_scene(run,config,specs[0]['R0'],app)
        save(run/'acquisition_scene.json',scene)
        from join_source02_preflight import set_present
        set_present(prop,False)
        from isaacsim.core.api import World
        world=World(physics_dt=1/60,rendering_dt=1/60,stage_units_in_meters=1.);world.reset()
        checkout=(ROOT/config['paths']['lightnav_checkout']).resolve()
        model=Worker([str(checkout/'.venv/bin/python'),str(ROOT/'scripts/online_lightnav_worker.py'),'--config',str(run/'config_snapshot.yaml')],run/'logs/lightnav_worker.log')
        mpc=Worker([str(checkout/'mujoco_demo/.venv/bin/python'),str(ROOT/'scripts/online_mpc_worker.py'),'--lightnav-checkout',str(checkout)],run/'logs/mpc_worker.log')
        geo=GeometryWorker(run)
        mr=model.await_type('ready');cr=mpc.await_type('ready');save(run/'workers.json',dict(model=mr,mpc=cr))
        for spec in specs:
            set_present(prop,False);geo.set(False);hook=SuddenReveal(inst,prop,geo,protocol)
            try:
                result=collect_episode(spec,run,config,app,world,agent,camera,rgb,scene,model,mpc,None,mr,cr,intervention=hook,command_guard=geo.guard)
                row=dict(episode_id=spec['episode_id'],status=result['status'],predictions=result['predictions_requested'])
            except BaseException as e:
                row=dict(episode_id=spec['episode_id'],status='TECHNICAL_RUNTIME_FAILURE',error=repr(e))
            finally:hook.close()
            ep=run/'episodes'/spec['episode_id'];files=[ep/'guard.jsonl',ep/'visibility.jsonl',*sorted((ep/'visibility').glob('*'))]
            files += [q for q in [ep/'guard_abort.json',ep/'obstacle_reveal.json',ep/'first_post_reveal_observation.json'] if q.exists()]
            save(ep/'acquisition_extension_manifest.json',dict(files={str(q.relative_to(ep)):sha(q) for q in files if q.exists()}))
            ledger.append(row);save(run/'ledger'/(spec['episode_id']+'.json'),row)
        save(run/'completion.json',dict(ledger=ledger,wall_s=time.monotonic()-start,at=stamp(),no_retry=True))
        print(ledger,flush=True)
    finally:
        if model:model.shutdown('shutdown')
        if mpc:mpc.shutdown()
        if geo:geo.close()
        app.close()

if __name__=='__main__':main()

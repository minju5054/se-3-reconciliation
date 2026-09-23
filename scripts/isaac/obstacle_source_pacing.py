#!/usr/bin/env python3
"""One separately counted technical OFF pacing qualification, unchanged collector."""
import argparse
from pathlib import Path
import subprocess
import sys
import time
import yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.robotless_online import stamp
from robotless_online_handoffs import Worker,collect_episode
from join_online02_collect import setup_scene,GeometryWorker,Visibility


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();parent=a.run.resolve();run=parent/'phase0'
    subprocess.run([str(ROOT/'.venv/bin/python'),str(ROOT/'scripts/run_obstacle_source_acquisition.py'),
        '--run',str(parent),'--mode','verify'],check=True,
        env={k:v for k,v in __import__('os').environ.items() if k not in ('LD_LIBRARY_PATH','PYTHONPATH')})
    assert read(parent/'generation_actual.json')['valid']
    if (run/'episodes').exists():raise FileExistsError('no technical retry in same run')
    config=yaml.safe_load((run/'config_snapshot.yaml').read_text());spec=read(run/'episode_schedule.json')['episodes'][0]
    save(run/'execution_start.json',dict(at=stamp(),sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()))
    from isaacsim import SimulationApp
    app=SimulationApp(dict(headless=True,renderer='RayTracedLighting',anti_aliasing=0,display_options=0))
    model=mpc=geo=None;start=time.monotonic()
    try:
        stage,agent,camera,rgb,inst,sem,prop,scene=setup_scene(run,config,spec['R0'],app)
        save(run/'acquisition_scene.json',scene)
        from join_source02_preflight import set_present
        set_present(prop,False)
        from isaacsim.core.api import World
        world=World(physics_dt=1/60,rendering_dt=1/60,stage_units_in_meters=1.);world.reset()
        checkout=(ROOT/config['paths']['lightnav_checkout']).resolve()
        model=Worker([str(checkout/'.venv/bin/python'),str(ROOT/'scripts/online_lightnav_worker.py'),'--config',str(run/'config_snapshot.yaml')],run/'logs/lightnav_worker.log')
        mpc=Worker([str(checkout/'mujoco_demo/.venv/bin/python'),str(ROOT/'scripts/online_mpc_worker.py'),'--lightnav-checkout',str(checkout)],run/'logs/mpc_worker.log')
        geo=GeometryWorker(run);geo.set(False)
        mr=model.await_type('ready');cr=mpc.await_type('ready');save(run/'workers.json',dict(model=mr,mpc=cr))
        hook=Visibility(inst,prop,False)
        try:r=collect_episode(spec,run,config,app,world,agent,camera,rgb,scene,model,mpc,None,mr,cr,intervention=hook,command_guard=geo.guard)
        finally:hook.close()
        ep=run/'episodes'/spec['episode_id']
        files=[ep/'guard.jsonl',ep/'visibility.jsonl',*sorted((ep/'visibility').glob('*'))]
        if (ep/'guard_abort.json').exists():files.append(ep/'guard_abort.json')
        save(ep/'acquisition_extension_manifest.json',dict(files={str(p.relative_to(ep)):sha(p) for p in files}))
        save(run/'completion.json',dict(status=r['status'],predictions=r['predictions_requested'],technical_only=True,wall_s=time.monotonic()-start))
        print(read(run/'completion.json'),flush=True)
    except BaseException as e:
        save(run/'technical_failure.json',dict(error=repr(e),at=stamp()));raise
    finally:
        if model:model.shutdown('shutdown')
        if mpc:mpc.shutdown()
        if geo:geo.close()
        app.close()


if __name__=='__main__':main()

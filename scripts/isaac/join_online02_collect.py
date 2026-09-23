#!/usr/bin/env python3
"""Existing native collector + immutable cart/BRIGHT setup + abort-only oracle."""
import argparse,hashlib,json,os,subprocess,sys,time
from pathlib import Path
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.robotless_online import stamp
from robotless_online_handoffs import Worker,collect_episode

def setup_scene(run,config,pose,app):
    from robotless_runtime import runtime_scene,camera_metadata
    from robotless_old_consistent_observation import set_precise_agent_pose
    from join_source02_preflight import create_prop,same_product_annotators
    from join_source03_render import lights
    from pxr import UsdLux,UsdGeom,Gf
    scenario=read(run/'scenario.json')
    stage,agent,camera,rgb,scene=runtime_scene(config,np.asarray(pose),app=app)
    set_precise_agent_pose(agent,np.asarray(pose),z_m=config['agent']['z_m'])
    source=read(scenario['source03_input'])
    original=read(Path(source['environment_export'])/'scene_provenance.json')
    expected={x['identifier']:x['composed_layer_text_sha256'] for x in original['used_layers'] if not x['anonymous']}
    actual={x.identifier:hashlib.sha256(x.ExportToString().encode()).hexdigest() for x in stage.GetUsedLayers() if not x.anonymous}
    if expected!=actual:raise ValueError('static Hospital layer mismatch')
    prop=create_prop(stage,scenario['prop']['source_prim'],scenario['center_xy'],scenario['prop']['desired_root_yaw_rad'])
    if not np.array_equal(prop['triangles'],np.load(scenario['triangles_path'])['triangles']):raise ValueError('cart mesh mismatch')
    if prop['record']['wrapper_matrix_column']!=scenario['prop']['wrapper_matrix_column']:raise ValueError('cart transform mismatch')
    before=lights(stage);lc=config['lighting']
    fill=UsdLux.RectLight.Define(stage,lc['prim'])
    fill.CreateIntensityAttr(lc['intensity']);fill.CreateExposureAttr(lc['exposure'])
    fill.CreateColorAttr(Gf.Vec3f(*lc['color']));fill.CreateNormalizeAttr(lc['normalize'])
    fill.CreateWidthAttr(lc['width_m']);fill.CreateHeightAttr(lc['height_m'])
    UsdGeom.Xformable(fill.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*lc['translation_world_m']))
    inst,sem,product=same_product_annotators(rgb)
    record=dict(scene=scene,static_layers_match=True,static_layers=actual,prop=prop['record'],
        original_lights=before,bright_lights=lights(stage),lighting_profile=lc,
        camera=camera_metadata(camera,agent,config),product=str(product),source04_exact_cart=True)
    return stage,agent,camera,rgb,inst,sem,prop,record

class GeometryWorker:
    def __init__(self,run):
        self.log=(run/'logs/geometry.log').open('x')
        env=os.environ.copy()
        for k in ['LD_LIBRARY_PATH','PYTHONPATH']:env.pop(k,None)
        env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
        self.p=subprocess.Popen([str(ROOT/'.venv/bin/python'),str(ROOT/'scripts/join_online02_geometry_worker.py'),'--run',str(run)],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,text=True,bufsize=1,env=env,cwd=ROOT)
        assert json.loads(self.p.stdout.readline())=={'ready':True}
    def ask(self,op,**data):
        self.p.stdin.write(json.dumps(dict(op=op,**data))+'\n');self.p.stdin.flush()
        r=json.loads(self.p.stdout.readline())
        if not r['ok']:raise RuntimeError(r['error'])
        return r['result']
    def guard(self,pose,command,dt):
        start=time.monotonic_ns();r=self.ask('guard',pose=pose.tolist(),command=command,dt=dt)
        r.update(query_start_ns=start,query_end_ns=time.monotonic_ns(),cart_present=self.present)
        return r
    def set(self,present):
        self.ask('set',present=present);self.present=present
    def close(self):
        self.p.stdin.write('{"op":"close"}\n');self.p.stdin.flush();self.p.wait(timeout=10);self.log.close()

class Visibility:
    def __init__(self,inst,prop,present):
        self.inst,self.prop,self.present=inst,prop,present;self.stream=None
    def before_capture(self,*args):pass
    def allow_fresh(self,frame):return True
    def after_capture(self,ep,frame):
        from join_source02_preflight import instance_data
        mask,record=instance_data(self.inst,self.prop['record']['runtime_prim'])
        if not self.present and record['visible_pixels']!=0:raise ValueError('hidden runtime cart visible')
        # No second render: this raster is from the exact completed RGB product.
        folder=ep/'visibility';folder.mkdir(exist_ok=True)
        path=folder/(frame['frame_id']+'.npz')
        with path.open('xb') as f:np.savez_compressed(f,mask=mask)
        if self.stream is None:self.stream=(ep/'visibility.jsonl').open('x')
        row=dict(frame_id=frame['frame_id'],state_id=frame['rendered_state_id'],sim_time_s=frame['capture_sim_time_s'],
            pose_world=frame['pose_world'],camera=frame['camera'],cart_present=self.present,
            cart_transform=self.prop['record']['wrapper_matrix_column'],mask_path=str(path.relative_to(ep)),
            mask_sha256=sha(path),instance=record,same_render_product_state=True,model_input=False)
        self.stream.write(json.dumps(row)+'\n');self.stream.flush()
    def close(self):
        if self.stream:self.stream.close()

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    p.add_argument('--mode',choices=['preflight','collect'],required=True);a=p.parse_args();run=a.run.resolve()
    config=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    if a.mode=='collect':
        subprocess.run([str(ROOT/'.venv/bin/python'),str(ROOT/'scripts/run_join_online02.py'),'--run',str(run),'--mode','verify'],check=True,
            env={k:v for k,v in os.environ.items() if k not in ['LD_LIBRARY_PATH','PYTHONPATH']})
        assert read(run/'generation_actual.json')['valid']
        if (run/'episodes').exists():raise FileExistsError('primary episode root already exists; no retry')
        save(run/'execution_start.json',dict(sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),at=stamp(),pid=os.getpid()))
    start=read(run/'start_selection.json')['selected']
    if start is None:raise ValueError('GEOMETRY_START_BLOCKER')
    from isaacsim import SimulationApp
    app=SimulationApp(dict(headless=True,renderer='RayTracedLighting',anti_aliasing=0,display_options=0))
    model=mpc=geometry=None
    try:
        stage,agent,camera,rgb,inst,sem,prop,scene=setup_scene(run,config,start['pose_world'],app)
        from join_source02_preflight import set_present,snapshot
        if a.mode=='preflight':
            out=run/'technical_preflight';out.mkdir(exist_ok=False)
            save(out/'scene.json',scene)
            records={}
            for label,present in [('OFF',False),('ON',True)]:
                set_present(prop,present)
                records[label]=snapshot(out,label,config,app,agent,camera,rgb,inst,sem,prefix=prop['record']['runtime_prim'])
            # Real native controller import and constants: no submit or solve.
            checkout=(ROOT/config['paths']['lightnav_checkout']).resolve()
            mpc=Worker([str(checkout/'mujoco_demo/.venv/bin/python'),str(ROOT/'scripts/online_mpc_worker.py'),'--lightnav-checkout',str(checkout)],out/'mpc_import.log')
            controller=mpc.await_type('ready');save(out/'mpc_import.json',controller)
            from isaacsim.core.api import World
            from robotless_runtime import actual_pose
            world=World(physics_dt=1/60,rendering_dt=1/60,stage_units_in_meters=1.)
            world.reset();before=float(world.current_time);pose=actual_pose(agent).copy()
            for _ in range(6):world.step(render=False)
            checks=dict(source_geometry=scene['static_layers_match'] and scene['source04_exact_cart'],
                off_hidden=records['OFF']['instance']['visible_pixels']==0,on_visible=records['ON']['instance']['visible_pixels']>=20,
                camera=records['OFF']['camera']==records['ON']['camera'],
                state_unchanged=np.array_equal(pose,actual_pose(agent)),
                step_clock=bool(np.isclose(float(world.current_time)-before,float(np.float32(1/60))*6,atol=1e-9)),
                controller_pinned=controller['provenance']['mpc_source_sha256']=='2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1')
            save(out/'result.json',dict(valid=all(checks.values()),checks=checks,cart_pixels=records['ON']['instance']['visible_pixels'],
                scientific_model_predictions=0,MPC_solves=0,technical_zero_motion_steps=6))
            if not all(checks.values()):raise RuntimeError('technical preflight failed')
            print(checks,flush=True);return
        save(run/'acquisition_scene.json',scene)
        from isaacsim.core.api import World
        world=World(physics_dt=1/60,rendering_dt=1/60,stage_units_in_meters=1.)
        world.reset()
        checkout=(ROOT/config['paths']['lightnav_checkout']).resolve()
        model=Worker([str(checkout/'.venv/bin/python'),str(ROOT/'scripts/online_lightnav_worker.py'),'--config',str(run/'config_snapshot.yaml')],run/'logs/lightnav_worker.log')
        mpc=Worker([str(checkout/'mujoco_demo/.venv/bin/python'),str(ROOT/'scripts/online_mpc_worker.py'),'--lightnav-checkout',str(checkout)],run/'logs/mpc_worker.log')
        geometry=GeometryWorker(run)
        mr=model.await_type('ready');cr=mpc.await_type('ready');save(run/'workers.json',dict(model=mr,mpc=cr))
        ledger=[]
        for spec in read(run/'episode_schedule.json')['episodes']:
            present=spec['cart_present'];set_present(prop,present);geometry.set(present)
            # Visibility change warmup occurs before session/acquisition, never history padding.
            import omni.replicator.core as rep
            for _ in range(4):rep.orchestrator.step(rt_subframes=1,delta_time=0.,pause_timeline=True)
            hook=Visibility(inst,prop,present)
            try:
                result=collect_episode(spec,run,config,app,world,agent,camera,rgb,scene,model,mpc,None,mr,cr,
                    intervention=hook,command_guard=geometry.guard)
            finally:hook.close()
            ep=run/'episodes'/spec['episode_id']
            extra=[ep/'guard.jsonl',ep/'visibility.jsonl',*sorted((ep/'visibility').glob('*'))]
            if (ep/'guard_abort.json').exists():extra.append(ep/'guard_abort.json')
            save(ep/'acquisition_extension_manifest.json',dict(files={str(p.relative_to(ep)):sha(p) for p in extra}))
            row=dict(episode=spec['episode_id'],status=result['status'],predictions=result['predictions_requested'],
                activated_after_C0=result['activated_handoffs'],states=result['state_count'])
            ledger.append(row);save(run/'ledger_steps'/f'{len(ledger):02}.json',row);print(row,flush=True)
        save(run/'schedule_completion.json',dict(episodes=ledger,end=stamp(),no_retry=True))
    except BaseException as e:
        path=run/('technical_preflight_failure.json' if a.mode=='preflight' else 'collection_failure.json')
        if not path.exists():save(path,dict(error=repr(e),at=stamp()))
        raise
    finally:
        if model:model.shutdown('shutdown')
        if mpc:mpc.shutdown()
        if geometry:geometry.close()
        app.close()
if __name__=='__main__':main()

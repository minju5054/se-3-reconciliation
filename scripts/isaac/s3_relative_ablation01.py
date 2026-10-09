#!/usr/bin/env python3
"""S3 actual-Isaac reset/step RPC; original Hospital, no added cart, no model calls."""
import argparse
import json
import os
from pathlib import Path
import sys
import traceback
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);args=p.parse_args();run=args.run.resolve()
    # Keep a dedicated JSON protocol fd; Kit/native stdout belongs in the raw log.
    protocol=os.fdopen(os.dup(sys.stdout.fileno()),'w',buffering=1)
    os.dup2(sys.stderr.fileno(),sys.stdout.fileno())
    def emit(value): protocol.write(json.dumps(value,allow_nan=False)+'\n');protocol.flush()
    app=None
    try:
        import numpy as np
        import yaml
        from reconciliation.join_source03 import read,save,sha
        from reconciliation.robotless_online import integrate_unicycle,without_automatic_physics,stamp
        from reconciliation.s3_isaac_ablation01 import prime_s3_clock
        frozen=read(run/'freeze.json')
        for path,digest in {**frozen['files'],**frozen['inputs'],**frozen['external_files']}.items(): assert sha(path)==digest,path
        common=read(run/'common_state.json');cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text())
        order=read(run/'execution_order.json');schedule=read(run/'schedule.json')
        save(run/'isaac_start.json',dict(pid=os.getpid(),at=stamp(),freeze_sha=read(run/'scientific_start.json')['freeze_sha']))
        from isaacsim import SimulationApp
        app=SimulationApp(dict(headless=True,renderer='RayTracedLighting',anti_aliasing=0,display_options=0))
        from robotless_runtime import runtime_scene
        from robotless_runtime import actual_pose
        from robotless_old_consistent_observation import set_precise_agent_pose
        from isaacsim.core.api import World
        from pxr import Usd,UsdGeom
        import omni.timeline
        import carb
        stage,agent,camera,rgb,scene=runtime_scene(cfg,np.asarray(common['B']),app=app)
        import hashlib
        historical=read(read(run/'source_authentication.json')['scene_provenance_path'])
        expected={r['identifier']:r['composed_layer_text_sha256'] for r in historical['used_layers'] if not r['anonymous']}
        def scene_check():
            actual={layer.identifier:hashlib.sha256(layer.ExportToString().encode()).hexdigest() for layer in stage.GetUsedLayers() if not layer.anonymous}
            assert actual==expected,'static Hospital layer mismatch'
            root=stage.GetPrimAtPath(cfg['scene']['reference_prim_path'])
            assert str(UsdGeom.Imageable(root).ComputeVisibility())!='invisible'
            matrix=np.asarray(UsdGeom.Xformable(root).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
            np.testing.assert_array_equal(matrix,np.eye(4))
            assert not stage.GetPrimAtPath('/World/ReconciliationCart').IsValid()
            return dict(static_layers=actual,Hospital_root_matrix=matrix.tolist(),Hospital_visible=True,cart_present=False)
        scene['restoration']=scene_check()
        assert all(scene[k]==v for k,v in cfg['scene'].items())
        save(run/'actual_scene.json',scene)
        world=World(physics_dt=cfg['execution']['simulation_dt_s'],rendering_dt=cfg['execution']['simulation_dt_s'],stage_units_in_meters=1.)
        timeline=omni.timeline.get_timeline_interface();settings=carb.settings.get_settings()
        dt=common['integration_dt_s'];bt=common['B_tick'];bs=common['B_sim_s']
        def sync():
            before=float(world.current_time)
            def update(): timeline.set_current_time(before);timeline.commit()
            without_automatic_physics(settings,update)
            assert float(world.current_time)==before
        current=None;tick=None;completed=[];broken=False
        emit(dict(type='ready',pid=os.getpid(),scene=scene,scientific_starts=1))
        for line in sys.stdin:
            q=json.loads(line);rid=q['request_id'];op=q['op']
            if op=='close':
                emit(dict(type='reply',request_id=rid,status='closed'));break
            try:
                if broken: raise RuntimeError('failed method is not retried')
                if op=='reset_method':
                    # A guard-censored prefix may end early. The next method still
                    # gets a full reset; order forbids retrying the failed method.
                    if current is not None: completed.append(current)
                    assert q['method']==order[len(completed)]
                    current=q['method']
                    world.reset();timeline.set_auto_update(False)
                    scene_check()
                    set_precise_agent_pose(agent,np.asarray(common['B']),z_m=cfg['agent']['z_m'])
                    clock=prime_s3_clock(world,bs,dt,read(run/'clock_budget.json')['maximum_steps']);sync()
                    pose=actual_pose(agent)
                    np.testing.assert_array_equal(pose,np.asarray(common['B']))
                    tick=bt
                    out=dict(method=current,tick=tick,pose_world=pose.tolist(),sim_time_s=float(world.current_time),
                        clock_initialization=clock,scene=scene_check(),world_step_index=world.current_time_step_index,
                        no_RGB_reads=True,no_live_model=True)
                elif op=='snapshot':
                    assert current==q['method'] and tick==q['tick']
                    out=dict(method=current,tick=tick,pose_world=actual_pose(agent).tolist(),sim_time_s=float(world.current_time),scene=scene_check())
                elif op=='step':
                    assert current==q['method'] and q['tick']==tick and bt<=tick<schedule['stop_tick']
                    pose=actual_pose(agent);np.testing.assert_array_equal(pose,np.asarray(q['pose_world']))
                    g=q['guard'];c=q['command']
                    assert g['safe'] and not g['command_modified'] and g['command']==c
                    np.testing.assert_array_equal(g['start_pose'],pose)
                    before=float(world.current_time);assert abs(before-(bs+(tick-bt)*dt))<=1e-10
                    nextpose=integrate_unicycle(pose,[c['v_mps'],c['omega_radps']],dt)
                    set_precise_agent_pose(agent,nextpose,z_m=cfg['agent']['z_m'])
                    world.step(render=False);sync();after=float(world.current_time)
                    assert abs(after-before-dt)<=1e-10
                    observed=actual_pose(agent);np.testing.assert_allclose(observed,nextpose,rtol=0,atol=1e-10)
                    tick+=1
                    out=dict(method=current,tick=tick,pose_world=observed.tolist(),sim_time_s=after,
                        world_step_index=world.current_time_step_index,USD_readback_error=float(abs(observed-nextpose).max()))
                else: raise ValueError('no other operation allowed')
                emit(dict(type='reply',request_id=rid,status='ok',**out))
            except Exception:
                broken=True
                emit(dict(type='reply',request_id=rid,status='error',error=traceback.format_exc()))
        save(run/'isaac_shutdown.json',dict(at=stamp(),last_method=current,last_tick=tick,broken=broken,RGB_reads=0))
    except Exception:
        emit(dict(type='fatal',error=traceback.format_exc()))
        return 1
    finally:
        if app is not None: app.close()
        protocol.close()
    return 0


if __name__=='__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Live history acquisition and paused paired RGB inputs, never reconciliation.

The historical collector is reused unchanged. The outer World proxy validates
each exact held-command interval before World.step. On abort the pending command
is explicitly *unapplied* (historical journal has one attempted trailing command).
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
from reconciliation.robotless_online import dump_new,stamp
from reconciliation.join_source02_acquisition import history_suffix,placement_on_future
from robotless_online_handoffs import Worker,collect_episode


class Geometry:
    def __init__(self,environment,log):
        self.log=Path(log).open('x');self.p=subprocess.Popen([str(ROOT/'.venv/bin/python'),
            str(ROOT/'scripts/join_source02_geometry_worker.py'),'--environment',str(environment)],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,text=True,bufsize=1)
        if json.loads(self.p.stdout.readline())!={'ready':True}:raise RuntimeError('geometry worker unavailable')
    def ask(self,op,**data):
        self.p.stdin.write(json.dumps(dict(op=op,**data))+'\n');self.p.stdin.flush()
        reply=json.loads(self.p.stdout.readline())
        if not reply['ok']:raise RuntimeError(reply['error'])
        return reply['result']
    def close(self):
        self.p.stdin.write('{"op":"close"}\n');self.p.stdin.flush();self.p.wait(timeout=10);self.log.close()


class GuardedWorld:
    def __init__(self,world,agent,geometry,output):
        self.world,self.agent,self.geometry,self.output=world,agent,geometry,output
        self.previous=None;self.step_count=0;self.guard_time_s=0.;self.aborted=False
    def __getattr__(self,name):return getattr(self.world,name)
    def reset(self):self.previous=None;return self.world.reset()
    def step(self,**kwargs):
        from robotless_runtime import actual_pose
        from robotless_old_consistent_observation import set_precise_agent_pose
        end=actual_pose(self.agent)
        if self.previous is None:raise RuntimeError('missing pre-integration boundary')
        started=time.monotonic();result=self.geometry.ask('held',start=self.previous.tolist(),end=end.tolist())
        self.guard_time_s+=time.monotonic()-started
        if not result['clearance_valid']:
            self.aborted=True
            dump_new(self.output/'oracle_abort.json',dict(pre_state=self.previous.tolist(),
                rejected_endpoint=end.tolist(),check=result,world_time_s=float(self.world.current_time),
                attempted_command_is_unapplied=True,completed_steps=self.step_count,
                policy='terminate before unsafe integration; no steering/stop/teleport appended'))
            # Restore the not-yet-integrated render assignment; this is not a
            # trajectory teleport. No next state or World.step is recorded.
            set_precise_agent_pose(self.agent,self.previous,z_m=0.)
            raise RuntimeError('ORACLE_ACQUISITION_ABORT_BEFORE_INTEGRATION')
        self.world.step(**kwargs);self.previous=end.copy();self.step_count+=1


class HistoryHook:
    def __init__(self,world,location,protocol):
        self.world,self.location,self.protocol=world,location,protocol
        self.frames=[];self.done=False;self.reason=None;self.last_active=None
    def before_capture(self,ep,pose,state,active,last_activation):
        if self.world.previous is None:self.world.previous=pose.copy()
        self.last_active=copy.deepcopy(active)
        if active and np.linalg.norm(pose[:2]-np.asarray(self.location['target_xy']))<=self.protocol['history']['end_if_target_distance_m']:
            self.done=True;self.reason='TARGET_APPROACH_BOUNDARY'
        if active and len(self.frames)>=8 and max(np.linalg.norm(np.asarray(f['pose_world'])[:2]-pose[:2]) for f in self.frames[-4:])<.005:
            self.done=True;self.reason='NO_APPROACH_MOTION_OVER_FOUR_CAPTURES'
    def after_capture(self,ep,frame):
        self.frames.append(frame)
        if len(self.frames)>=self.protocol['history']['maximum_live_frames']:
            self.done=True;self.reason='DECLARED_HISTORY_FRAME_LIMIT'
    def allow_fresh(self,frame):return True


class AcquisitionApp:
    def __init__(self,app,hook):self.app,self.hook=app,hook
    def is_running(self):return self.app.is_running() and not self.hook.done
    def __getattr__(self,name):return getattr(self.app,name)


def final_frame(record,folder,name,sim_time,camera_id):
    s=stamp(sim_time,sim_time)
    return dict(frame_id=name,path=str((folder/record['rgb_file']).resolve()),sha256=record['rgb_jpeg_sha256'],
        rendered_state_id=name,capture_sim_time_s=sim_time,capture_monotonic_ns=record['capture_completed_monotonic_ns'],
        capture_host_utc=record['capture_completed_utc'],episode_time_s=sim_time,pose_world=record['agent_pose_world'],
        camera=record['camera'],camera_id=camera_id,bootstrap=False,stationary=True,
        resolution_width_height=record['camera']['actual_intrinsics']['resolution_width_height'],
        render_state_stable=True,paired_render_not_online_execution=True)


def render_inputs(run,config,protocol,location,ep,hook,stage,agent,camera,rgb,app,world,geometry):
    from join_source02_preflight import create_prop,set_present,same_product_annotators,capture_sameproduct,PROP_ROOT
    from robotless_old_consistent_observation import set_precise_agent_pose
    frames=[json.loads(x) for x in (ep/'capture.jsonl').read_text().splitlines()]
    acquired_count=len(frames)
    for f in frames:
        f['path']=str((ep/f['path']).resolve());f['camera_id']=config['camera']['prim_path']
    if hook.last_active:
        fid=hook.last_active['context']['observation']['frame_id']
        cut=next(i for i,f in enumerate(frames) if f['frame_id']==fid)
        frames=frames[:cut+1]
    bank=dict(location_id=location['id'],acquired_frames=acquired_count,available_frames=len(frames),termination=hook.reason,
              frames=frames,actual_old_context=hook.last_active,source='new live official approach; not screening replay')
    dump_new(ep/'history_bank.json',bank)
    if not frames:return
    pose=np.asarray(frames[-1]['pose_world']);set_precise_agent_pose(agent,pose,z_m=config['agent']['z_m'])
    instance,semantic,product=same_product_annotators(rgb)
    old=None if not hook.last_active else np.load(ep/hook.last_active['context']['world_ref']['path'],allow_pickle=False)
    for placement in protocol['placements']:
        folder=run/'development/paired_inputs'/location['id']/placement['id'];folder.mkdir(parents=True,exist_ok=False)
        # Exact original mesh bounds determine forward extent after yaw rotation.
        from join_source02_preflight import collect_meshes
        source_triangles,_=collect_meshes(stage,protocol['prop']['source_prim'])
        from pxr import UsdGeom,Usd
        matrix=np.asarray(UsdGeom.Xformable(stage.GetPrimAtPath(protocol['prop']['source_prim'])).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
        source_yaw=math.atan2(matrix[1,0],matrix[0,0])
        c,s=math.cos(source_yaw),math.sin(source_yaw)
        aligned=source_triangles.reshape(-1,3)@np.array([[c,-s,0],[s,c,0],[0,0,1]])
        bounds=np.ptp(aligned,axis=0)
        result=dict(status='PLACEMENT_UNAVAILABLE',reason='no recorded active OLD') if old is None else placement_on_future(
            pose,old,fraction=placement['usable_remaining_arc_fraction'],half_forward_m=float(bounds[1]/2),
            design_latency_s=protocol['placement_design']['latency_estimate_s'],
            design_speed_mps=protocol['placement_design']['speed_estimate_mps'])
        if result['status']=='AVAILABLE':
            if stage.GetPrimAtPath(PROP_ROOT).IsValid():stage.RemovePrim(PROP_ROOT)
            prop=create_prop(stage,protocol['prop']['source_prim'],result['center_xy'],result['forward_yaw_rad']-math.pi/2)
            triangles_path=folder/'actual_prop_triangles.npz'
            with triangles_path.open('xb') as f:np.savez_compressed(f,triangles=prop['triangles'])
            projection=geometry.ask('prop',triangles=str(triangles_path))
            recs={};activation=[]
            for label,present in [('A',False),('B',True)]:
                set_present(prop,present);geometry.ask('set',present=present)
                activation.append(dict(present=present,render=prop['record']['computed_visibility'],
                                       oracle_present=present,timestamp=stamp(float(world.current_time))))
                recs[label]=capture_sameproduct(folder,label,config,app,agent,camera,rgb,instance,semantic,prefix=PROP_ROOT)
            set_present(prop,False);geometry.ask('set',present=False)
            on_pixels=recs['B']['instance']['visible_pixels'];off_pixels=recs['A']['instance']['visible_pixels']
            forward=np.array([math.cos(result['forward_yaw_rad']),math.sin(result['forward_yaw_rad'])]);left=np.array([-forward[1],forward[0]])
            coords=prop['triangles'].reshape(-1,3)[:,:2];along=(coords-np.array(result['center_xy']))@forward
            lateral=(coords-np.array(result['center_xy']))@left
            half=float(np.max(np.abs(along)));width=float(np.max(np.abs(lateral)))
            passages=[]
            for sign in (-1,1):
                center=np.asarray(result['center_xy'])+sign*(width+.26)*left
                points=[center-(half+.3)*forward,center+(half+.3)*forward]
                passages.append(geometry.ask('polyline',poses=np.asarray(points).tolist(),present=True))
            placement_record=dict(**result,environment_export=str((ROOT/protocol['environment']).resolve()),
                prop=prop['record'],projection=projection,triangles_path=str(triangles_path.resolve()),
                target_xy=location['target_xy'],activation=activation,lateral_passages=passages,
                influence=dict(origin_xy=result['center_xy'],forward_xy=forward.tolist(),progress_min_m=-half-.5,progress_max_m=half+.5),
                bypass_plane=dict(point_xy=(np.asarray(result['center_xy'])+(half+.25)*forward).tolist(),forward_xy=forward.tolist()),
                technical_checks=dict(source_integrity=True,official_contract=True,
                    visibility=off_pixels==0 and on_pixels>=20,geometry_activation=bool(any(x['clearance_valid'] for x in passages))),
                source_old_path=hook.last_active['context']['world_ref'],observation_pose_world=pose.tolist(),
                shared_render_product=product,off_visible_pixels=off_pixels,on_visible_pixels=on_pixels)
            dump_new(folder/'placement.json',placement_record)
            sim=float(world.current_time)
            final={label:final_frame(rec,folder,location['id']+'_'+placement['id']+'_'+label,sim,config['camera']['prim_path']) for label,rec in recs.items()}
            final['SHAM']=copy.deepcopy(final['A'])
        else:
            dump_new(folder/'placement.json',result);final=None
        for h in protocol['history']['counts']:
            cid=f"{location['id']}__{placement['id']}__H{h}"
            suffix=history_suffix(frames,h)
            available=suffix['status']=='AVAILABLE' and result['status']=='AVAILABLE'
            availability='AVAILABLE' if available else suffix['status'] if suffix['status']!='AVAILABLE' else result['status']
            if available and not all(placement_record['technical_checks'].values()):availability='TECHNICAL_INVALID'
            row=dict(condition_id=cid,availability=availability,
                location_id=location['id'],placement_id=placement['id'],history_count=h,
                instruction=location['instruction'],config_path=str((run/'config_snapshot.yaml').resolve()),
                placement_path=str((folder/'placement.json').resolve()),bank_path=str((ep/'history_bank.json').resolve()))
            if available:row.update(history=suffix['history'],final_frames=final)
            dump_new(run/'development/input_manifests'/(cid+'.json'),row)


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();run=a.run.resolve()
    protocol=json.loads((run/'protocol.json').read_text());config=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    if not (run/'development_freeze.json').exists():raise RuntimeError('commit/push freeze required before acquisition')
    subprocess.run([str(ROOT/'.venv/bin/python'),str(ROOT/'scripts/run_join_source02.py'),'--run',str(run),'--mode','verify'],check=True)
    source=run/'development/history_acquisition';source.mkdir();(source/'episodes').mkdir();(source/'logs').mkdir()
    (run/'development/input_manifests').mkdir()
    checkout=(ROOT/config['paths']['lightnav_checkout']).resolve()
    model=Worker([str(checkout/'.venv/bin/python'),str(ROOT/'scripts/online_lightnav_worker.py'),'--config',str(run/'config_snapshot.yaml')],source/'logs/model.log')
    mpc=Worker([str(checkout/'mujoco_demo/.venv/bin/python'),str(ROOT/'scripts/online_mpc_worker.py'),'--lightnav-checkout',str(checkout)],source/'logs/mpc.log')
    geometry=Geometry((ROOT/protocol['environment']).resolve(),source/'logs/geometry.log');app=None
    try:
        mr=model.await_type('ready');cr=mpc.await_type('ready');dump_new(source/'workers.json',dict(model=mr,mpc=cr))
        from isaacsim import SimulationApp
        app=SimulationApp(dict(headless=True,renderer='RayTracedLighting',anti_aliasing=0,display_options=0))
        from robotless_runtime import runtime_scene
        from isaacsim.core.api import World
        stage,agent,camera,rgb,scene=runtime_scene(config,np.asarray(protocol['locations'][0]['initial_pose_world']),app=app)
        original=json.loads((ROOT/protocol['environment']/'scene_provenance.json').read_text())
        expected={x['identifier']:x['composed_layer_text_sha256'] for x in original['used_layers'] if not x['anonymous']}
        actual={x.identifier:hashlib.sha256(x.ExportToString().encode()).hexdigest() for x in stage.GetUsedLayers() if not x.anonymous}
        dump_new(source/'static_layers.json',dict(expected=expected,actual=actual,valid=expected==actual))
        if expected!=actual:raise RuntimeError('static Hospital layer hash mismatch')
        world=World(physics_dt=1/60,rendering_dt=1/60,stage_units_in_meters=1.);world.reset()
        dump_new(source/'scene.json',scene)
        for location in protocol['locations']:
            if stage.GetPrimAtPath('/World/JOINSource02Obstacle').IsValid():stage.RemovePrim('/World/JOINSource02Obstacle')
            geometry.ask('set',present=False)
            ep=source/'episodes'/location['id']
            guard=GuardedWorld(world,agent,geometry,ep);hook=HistoryHook(guard,location,protocol)
            spec=dict(episode_id=location['id'],R0=location['initial_pose_world'],instruction=location['instruction'])
            metadata=collect_episode(spec,source,config,AcquisitionApp(app,hook),guard,agent,camera,rgb,scene,model,mpc,None,mr,cr,intervention=hook)
            dump_new(ep/'acquisition_end.json',dict(reason=hook.reason,oracle_abort=guard.aborted,
                guarded_steps=guard.step_count,guard_wall_s=guard.guard_time_s,original_collector_status=metadata['status'],
                trailing_unapplied_command_count=1 if guard.aborted else 0))
            render_inputs(run,config,protocol,location,ep,hook,stage,agent,camera,rgb,app,world,geometry)
    finally:
        model.shutdown('shutdown');mpc.shutdown();geometry.close()
        if app:app.close()


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Genuine live Isaac camera -> official LightNav -> official MPC -> SE(2).

The main loop never waits on a model response or CPU solve. Offline analysis and
saved replay are separate entry points. Acquisition files are exclusive-create.
"""
from __future__ import annotations
import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import csv
import io
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import traceback

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import numpy as np
import yaml
from reconciliation.robotless_online import (
    AbsolutePacer, CommandActivation, dump_new, file_record, integrate_unicycle,
    raw_manifest, stamp, verify_resume, without_automatic_physics,
)


def host_normalized(value):
    ns=value.get('host_monotonic_ns',value.get('monotonic_ns'))
    return {'host_monotonic_ns':ns,'host_monotonic_s':ns/1e9,
            'host_utc':value.get('host_utc',value.get('utc'))}


def read_json(path): return json.loads(Path(path).read_text())


class Worker:
    def __init__(self, argv, log):
        self.messages=queue.Queue(); self.inputs=queue.Queue(); self.serial=0
        env=os.environ.copy()
        for key in ('PYTHONPATH','LD_LIBRARY_PATH'): env.pop(key,None)
        env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONUNBUFFERED='1')
        self.log=log.open('x')
        self.process=subprocess.Popen(argv,cwd=ROOT,env=env,stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,stderr=self.log,text=True,bufsize=1)
        self.reader=threading.Thread(target=self._read,daemon=True); self.reader.start()
        self.writer=threading.Thread(target=self._write,daemon=True); self.writer.start()
    def _read(self):
        try:
            for line in self.process.stdout:
                try: self.messages.put(json.loads(line))
                except ValueError: self.messages.put({'type':'fatal','error':'non-JSON worker stdout','line':line[:1000]})
        finally: self.messages.put({'type':'worker_exit','returncode':self.process.poll()})
    def _write(self):
        try:
            while True:
                item=self.inputs.get()
                if item is None: return
                self.process.stdin.write(json.dumps(item,allow_nan=False)+'\n'); self.process.stdin.flush()
        except Exception as exc: self.messages.put({'type':'fatal','error':str(exc)})
    def send(self,op,**payload):
        self.serial+=1
        self.inputs.put({'op':op,'request_id':self.serial,**payload})
        return self.serial
    def drain(self):
        out=[]
        while True:
            try: out.append(self.messages.get_nowait())
            except queue.Empty: return out
    def await_type(self,kind,timeout=120):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            msg=self.messages.get(timeout=max(.01,deadline-time.monotonic()))
            if msg.get('type')==kind: return msg
            if msg.get('type') in ('fatal','worker_exit','error'): raise RuntimeError(msg)
        raise TimeoutError(kind)
    def shutdown(self, operation='close'):
        if self.process.poll() is None:
            self.send(operation)
            try: self.process.wait(timeout=15)
            except subprocess.TimeoutExpired: self.process.terminate(); self.process.wait(timeout=10)
        self.inputs.put(None); self.log.close()


class Journal:
    """Small records persisted by one bounded writer, never image-encode on loop."""
    def __init__(self, root):
        self.root=root; self.queue=queue.Queue(maxsize=20000); self.error=None
        self.thread=threading.Thread(target=self.run,daemon=True); self.thread.start()
    def write(self,name,row):
        if self.error: raise RuntimeError(self.error)
        self.queue.put_nowait((name,row.copy()))
    def run(self):
        streams={}; writers={}
        try:
            while True:
                item=self.queue.get()
                if item is None: break
                name,row=item
                if name not in streams:
                    path=self.root/name; path.parent.mkdir(parents=True,exist_ok=True)
                    streams[name]=path.open('x',newline='')
                    if name.endswith('.csv'):
                        writers[name]=csv.DictWriter(streams[name],fieldnames=list(row)); writers[name].writeheader()
                if name.endswith('.csv'): writers[name].writerow(row)
                else: streams[name].write(json.dumps(row,allow_nan=False)+'\n')
        except Exception as exc: self.error=repr(exc)
        finally:
            for stream in streams.values(): stream.close()
    def close(self):
        self.queue.put(None); self.thread.join()
        if self.error: raise RuntimeError(self.error)


def encode_frame(root,rgb,frame,quality):
    from PIL import Image
    path=root/frame['path']; path.parent.mkdir(exist_ok=True)
    with path.open('xb') as stream: Image.fromarray(rgb).save(stream,format='JPEG',quality=quality)
    return {**frame,**file_record(path,root)}


class LivePanel:
    def __init__(self,app,config,pose):
        import omni.ui as ui
        from isaacsim.util.debug_draw import _debug_draw
        from isaacsim.core.utils.viewports import set_camera_view
        self.app=app; self.last_rgb_url=None; self.draw=_debug_draw.acquire_debug_draw_interface()
        self.window=ui.Window('LIVE ONLINE COLLECTION',width=440,height=570,position_x=10,position_y=40)
        with self.window.frame:
            with ui.VStack(spacing=6):
                ui.Label('LIVE ONLINE COLLECTION',height=35,style={'font_size':22})
                ui.Label('Logical SE(2), official MPC, genuine live LightNav',height=25)
                self.info=ui.Label('',height=190,word_wrap=True)
                self.rgb=ui.Image('',height=235,fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT)
                ui.Label('OLD blue | FRESH magenta | actual execution green\nNo robot dynamics; collision validity unknown',height=45)
        set_camera_view(eye=[pose[0],pose[1]-.001,2.4],target=[pose[0],pose[1],0],camera_prim_path='/OmniverseKit_Persp')
    def update(self,ep,state,active,pending,inflight,history,events,rtf,executed,root,last_frame):
        self.info.text=(f'{ep}\nSimulation {state["sim_time_s"]:.3f} s | RTF {rtf:.3f}\n'
            f'Executed [{state["x"]:.3f}, {state["y"]:.3f}, {state["yaw"]:.3f}]\n'
            f'Active OLD: {active["chunk_id"] if active else "bootstrap"}\n'
            f'Pending FRESH: {pending or "none"}\nInference in flight: {inflight}\n'
            f'Delivered history: {history} (full-episode SlowFast)\nActual handoff activations: {events}')
        if last_frame:
            url='file:'+str(root/last_frame['path'])
            if url!=self.last_rgb_url:
                self.rgb.source_url=url;self.last_rgb_url=url
            self.info.text+=f'\nRGB: state {last_frame["rendered_state_id"]}, sim {last_frame["capture_sim_time_s"]:.3f}s'
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=[state['x'],state['y']-.001,2.4],target=[state['x'],state['y'],0],camera_prim_path='/OmniverseKit_Persp')
        from omni.kit.viewport.utility import get_active_viewport
        import omni.usd
        from pxr import UsdGeom
        viewport=get_active_viewport(); resolution=viewport.resolution
        display_camera=UsdGeom.Camera(omni.usd.get_context().get_stage().GetPrimAtPath('/OmniverseKit_Persp'))
        display_camera.CreateProjectionAttr(UsdGeom.Tokens.orthographic)
        display_camera.CreateHorizontalApertureAttr(6.0*float(resolution[0])/float(resolution[1])*10.)
        display_camera.CreateVerticalApertureAttr(60.0)
        self.draw.clear_lines(); self.draw.clear_points()
        starts=[]; ends=[]; colors=[]; widths=[]
        def pathline(poses,color,width):
            if poses is None or len(poses)<2: return
            for a,b in zip(poses[:-1],poses[1:]):
                starts.append((float(a[0]),float(a[1]),.18)); ends.append((float(b[0]),float(b[1]),.18));colors.append(color);widths.append(width)
        if active:
            c=active['context']; pathline(np.load(root/c['world_ref']['path']),(0.1,.35,1.,1.),5)
        if pending and (root/'chunks'/pending/'world.npy').exists():
            pathline(np.load(root/'chunks'/pending/'world.npy'),(1.,.1,.8,1.),5)
        pathline([[r['x'],r['y']] for r in executed[-1200:]],(.1,.8,.2,1.),3)
        if starts:self.draw.draw_lines(starts,ends,colors,widths)


def make_event_context(ep,result,old,ready_state,installed_stamp,history_contract):
    obs=result['observation']; snap=read_json(ep/result['history_snapshot']['path']) if result.get('history_snapshot') else {}
    old_result=old['context'] if old else {}
    return {'schema_version':1,'episode_id':ep.name,
      'old_chunk_id':old_result.get('chunk_id'),'fresh_chunk_id':result.get('chunk_id'),
      'old_reference_version':old.get('reference_version') if old else None,
      'fresh_reference_version':int(result['chunk_id'].split('_')[-1]),
      'old_raw_local_ref':old_result.get('raw_local_ref'),'old_world_ref':old_result.get('world_ref'),
      'fresh_raw_local_ref':result.get('raw_local_ref'),'fresh_world_ref':result.get('world_ref'),
      'old_observation_pose_time':{'pose_world':old_result.get('observation',{}).get('pose_world'),
          'time':old_result.get('observation',{}).get('capture_sim_time_s')},
      'fresh_observation_pose_time':{'pose_world':obs['pose_world'],'time':obs['capture_sim_time_s'],'frame_id':obs['frame_id']},
      'R_old_obs':old_result.get('observation',{}).get('pose_world'),'R_obs':obs['pose_world'],
      'R_ready':[ready_state[k] for k in ('x','y','yaw')],
      'obs_state_id':obs['rendered_state_id'],'ready_state_id':ready_state['state_id'],
      't_obs':{'sim_time_s':obs['capture_sim_time_s'],'episode_time_s':obs['episode_time_s'],
          'host_monotonic_ns':obs['capture_monotonic_ns'],'host_monotonic_s':obs['capture_monotonic_ns']/1e9,'host_utc':obs['capture_host_utc']},
      't_request':host_normalized(result['t_request_host']),
      't_ready_host':host_normalized(result['t_ready_host']) if result.get('t_ready_host') else None,
      't_ready_seen_sim':{k:ready_state[k] for k in ('sim_time_s','episode_time_s','host_monotonic_s','host_monotonic_ns','host_utc')},
      't_install':installed_stamp,'t_switch':None,'P':None,'B':None,'switch_state_id':None,'previous_state_id':None,
      'request_history_frame_ids':snap.get('chronological_history_frame_ids',snap.get('history_frame_ids',[])),
      'actual_history_count':result.get('history_count',snap.get('actual_history_count',0)),
      'history_full':result.get('history_full',False),'history_config':history_contract,
      'history_snapshot_ref':result.get('history_snapshot'),'client_rtt_s':result.get('client_rtt_s'),
      'server_response_ref':result.get('response_ref'),'server_timing':result.get('response',{}),
      'status':result['status'],'error':result.get('error'), 'timing_flags':{},
      'provenance':{'source':'genuine online live camera and command-integrated Isaac logical agent','intrinsic_waypoint_dt_s':None,
         'no_added_inference_or_activation_delay':True,'collision_validity':'unknown'}}


def collect_episode(spec,run,config,app,world,agent,camera,annotator,scene,model,mpc,panel,model_ready,mpc_ready):
    from robotless_runtime import actual_pose,camera_metadata
    from robotless_old_consistent_observation import set_precise_agent_pose
    import omni.replicator.core as rep
    import omni.timeline
    ep=run/'episodes'/spec['episode_id']
    if verify_resume(ep): print('SKIP_VERIFIED',spec['episode_id'],flush=True);return
    ep.mkdir(parents=True); (ep/'controller').mkdir(); (ep/'requests').mkdir(); (ep/'rgb').mkdir()
    dump_new(ep/'started.json',{'spec':spec,'start':stamp()})
    model.send('open',episode_id=ep.name,episode_dir=str(ep),instruction=spec['instruction'])
    opened=model.await_type('opened'); mpc.send('reset',episode_id=ep.name)
    # Episode session setup is before acquisition. The model stays warmed.
    world.reset()
    timeline=omni.timeline.get_timeline_interface()
    timeline.set_auto_update(False)
    import carb
    settings=carb.settings.get_settings()
    def sync_timeline(value):
        def update():
            timeline.set_current_time(value)
            timeline.commit()
        before=float(world.current_time)
        without_automatic_physics(settings,update)
        if float(world.current_time)!=before:raise RuntimeError('timeline synchronization advanced physics')
    sync_timeline(float(world.current_time))
    pose=np.asarray(spec['R0'],float); set_precise_agent_pose(agent,pose,z_m=config['agent']['z_m'])
    origin=float(world.current_time); host_origin=time.monotonic()
    dt=float(np.float32(config['execution']['simulation_dt_s'])); capture_stride=round(config['execution']['integration_hz']/config['execution']['capture_hz'])
    control_stride=round(config['execution']['integration_hz']/config['execution']['control_hz'])
    pacer=AbsolutePacer(host_origin,origin,config['execution']['target_rtf'],config['online']['rtf_deadline_tolerance_s'])
    journal=Journal(ep); encoder=ThreadPoolExecutor(max_workers=1,thread_name_prefix='jpeg-writer')
    encoder_pending=deque(); frames=deque(); allframes=[]; states=[]; commands=[]; contexts=[]; chunks={}
    activation=CommandActivation(config['execution']['command_hold_timeout_s'])
    wire_busy=False; pending_prediction=None; outstanding_frame=None; predictions=0; history_count=0
    first_activation=None; last_activation=None; normal_activations=0; stopping=None; last_frame=None
    last_result_s=None; terminal='EPISODE_LIMIT'; terminal_reason=''; timeout_response=None; initial_context=None
    tick=0; solve_counter=0; loop_previous_host=host_origin; current_context=None
    st={'state_id':0,'tick':0,**stamp(origin,0),'x':float(pose[0]),'y':float(pose[1]),'yaw':float(pose[2]),
        'incoming_command_id':None,'active_reference_version':None,'active_chunk_id':None,
        'isaac_timeline_time_s':float(omni.timeline.get_timeline_interface().get_current_time())}
    states.append(st); journal.write('execution.csv',st)
    try:
        while app.is_running():
            now=time.monotonic(); sim=float(world.current_time); elapsed=sim-origin
            if sim!=st['sim_time_s']:raise RuntimeError(f'uncommanded between-loop clock step {st["sim_time_s"]}->{sim}')
            if now-host_origin>config['online']['maximum_episode_host_s']:
                terminal='EXECUTOR_STALLED';terminal_reason='maximum episode host duration';break
            for message in mpc.drain():
                message['seen_in_isaac']=stamp(sim,elapsed);journal.write('controller/events.jsonl',message)
                if message.get('type') in ('fatal','worker_exit','error') or message.get('status') in ('controller_error','error'):
                    terminal='CONTROLLER_ERROR';terminal_reason=str(message);stopping='error';break
                if message.get('type')=='solve_result':
                    if message.get('status')=='stale_rejected' and message.get('error'):
                        terminal='CONTROLLER_ERROR';terminal_reason='official stale solver failure may reset internal previous_command';stopping='error';break
                    if message.get('episode_id')!=ep.name: continue
                    if activation.accept(message,sim):
                        last_result_s=sim
                    else: journal.write('controller/root_rejections.jsonl',message)
            if stopping=='error':break
            for message in model.drain():
                if message.get('type') in ('fatal','worker_exit','error'):
                    terminal='PROTOCOL_ERROR';terminal_reason=str(message);stopping='error';break
                if message.get('type')=='request_sent':
                    journal.write('requests/client_seen.jsonl',{**message,'seen_in_isaac':stamp(sim,elapsed),'state_id':st['state_id']})
                    if outstanding_frame: outstanding_frame['wire_seq']=message['seq'];outstanding_frame['transmission_status']='SENT'
                if message.get('type')!='result':continue
                wire_busy=False
                if outstanding_frame: outstanding_frame['transmission_status']=message['status']
                history_count=message.get('history_count',history_count)
                if message.get('kind')=='buffer_only':
                    if message['status']!='BUFFERED':terminal=message['status'];terminal_reason=message.get('error','');stopping='error'
                    continue
                pending_prediction=None
                result=message;chunks[result['chunk_id']]=result
                seen={**st,**stamp(sim,elapsed)}
                context=make_event_context(ep,result,activation.active,seen,None,model_ready['history_contract'])
                if result['status']!='PREDICTION_READY':
                    if activation.active:context['event_id']=f'handoff_{len(contexts):03d}';contexts.append(context)
                    else:initial_context=context
                    terminal=result['status'];terminal_reason=result.get('error','official model STOP');stopping='error';continue
                version=int(result['chunk_id'].split('_')[-1]); installed=stamp(sim,elapsed)
                context['t_install']=installed
                activation.install(result['chunk_id'],version,result)
                mpc.send('install',episode_id=ep.name,chunk_id=result['chunk_id'],reference_version=version,
                    raw_local_path=str(ep/result['raw_local_ref']['path']),capture_pose=result['observation']['pose_world'],
                    raw_sha256=result['raw_local_ref']['sha256'],t_ready_seen_sim=context['t_ready_seen_sim'],t_install=installed)
                current_context=context
                # The next fixed 10Hz control tick submits this reference.
                # This natural scheduling delay is preserved in t_install/t_switch.
            if stopping=='error':break
            # Immutable encoded frames become available chronologically.
            while encoder_pending and encoder_pending[0].done():
                frame=encoder_pending.popleft().result();frames.append(frame);allframes.append(frame);last_frame=frame
                journal.write('capture.jsonl',frame)
            if len(frames)+len(encoder_pending)>=config['online']['capture_queue_capacity']:
                terminal='TECHNICAL_INVALID';terminal_reason='bounded capture queue overflow; no silent drop';break
            if tick % capture_stride==0:
                # DebugDraw is renderer-wide: remove display lines before model RGB.
                if panel:panel.draw.clear_lines();panel.draw.clear_points()
                before_clock=float(world.current_time); before_pose=actual_pose(agent)
                before=stamp(sim,elapsed)
                # Native SimulationContext.render uses this same switch to avoid
                # an uncommanded PhysX step during app-driven RGB rendering.
                without_automatic_physics(settings,lambda: rep.orchestrator.step(rt_subframes=config['capture']['render_subframes'],delta_time=0.0,pause_timeline=False))
                rgb=np.asarray(annotator.get_data())
                after_clock=float(world.current_time);after_pose=actual_pose(agent)
                if before_clock!=after_clock or not np.allclose(before_pose,after_pose,atol=1e-10,rtol=0) or not np.allclose(pose,after_pose,atol=1e-10,rtol=0):
                    raise RuntimeError(f'render readback changed state/clock: {before_clock}->{after_clock}, {pose}->{after_pose}')
                if rgb.shape!=(config['camera']['resolution_height'],config['camera']['resolution_width'],4) or rgb.dtype!=np.uint8:
                    raise RuntimeError(f'unexpected RGB array {rgb.shape} {rgb.dtype}')
                fid=f'frame_{len(allframes)+len(encoder_pending):06d}'
                prior=allframes[-1] if allframes else None
                frame={'frame_id':fid,'path':f'rgb/{fid}.jpg','capture_sim_time_s':sim,'episode_time_s':elapsed,
                    'capture_monotonic_ns':before['host_monotonic_ns'],'capture_host_utc':before['host_utc'],
                    'readback_finished_host':stamp(sim,elapsed),'rendered_state_id':st['state_id'],
                    'pose_world':pose.tolist(),'camera':camera_metadata(camera,agent,config),
                    'render_state_stable':True,'display_annotations_cleared_before_capture':True,'render_sim_time_before_s':before_clock,'render_sim_time_after_s':after_clock,
                    'bootstrap':first_activation is None,'stationary':prior is None or bool(np.linalg.norm(np.asarray(prior['pose_world'])-pose)<1e-10),
                    'transmission_status':'CAPTURED_NOT_SENT','wire_seq':None}
                encoder_pending.append(encoder.submit(encode_frame,ep,rgb[:,:,:3].copy(),frame,config['capture']['jpeg_quality']))
            # Flush older queued frames as buffer-only before predicting the latest available one.
            active_age=None if last_activation is None else sim-last_activation
            can_predict=(activation.active is not None and activation.installed==activation.active and
                active_age+1e-10>=config['online']['minimum_active_before_prediction_sim_s'] and
                predictions-1<config['online']['maximum_handoff_attempts'] and
                sim-first_activation<config['online']['maximum_active_sim_s']-config['online']['postroll_sim_s'])
            if not wire_busy and frames:
                predict=(predictions==0 and history_count==3) or (can_predict and len(frames)==1)
                frame=frames.popleft();outstanding_frame=frame
                if predict:
                    chunk_id=f'chunk_{predictions:03d}';predictions+=1;pending_prediction=chunk_id
                    model.send('frame',frame=frame,predict=True,chunk_id=chunk_id)
                else:model.send('frame',frame=frame,predict=False)
                wire_busy=True
            if activation.installed is not None and tick%control_stride==0:
                mpc.send('submit',solve_id=f'{ep.name}_solve_{solve_counter:06d}',pose=pose.tolist(),input_state_id=st['state_id'],
                    input_sim_time_s=sim,input_host_monotonic_s=time.monotonic());solve_counter+=1
            command,event=activation.apply(st,states[-2] if len(states)>1 else None,commands[-1] if commands else None)
            applied_stamp=stamp(sim,elapsed);command.update(applied_stamp);command['command_id']=len(commands)
            if event:
                event['t_switch']=applied_stamp;event['first_fresh_command']=command
                context=current_context
                for key in ('B','P','switch_state_id','previous_state_id','t_switch','pre_switch_command','first_fresh_command'):
                    context[key]=event[key]
                context['controller_reference_selection']=activation.latest.get('selection')
                context['first_fresh_solve']=activation.latest
                context['old_reference_progress']=commands[-1].get('solve_id') if commands else None
                context['status']='ACTIVATED_PENDING_FINALIZATION'
                if event['bootstrap']:
                    initial_context=context;first_activation=sim
                else:
                    context['event_id']=f'handoff_{len(contexts):03d}';contexts.append(context);normal_activations+=1
                last_activation=sim;last_result_s=sim
                print('ACTIVATION',ep.name,context['fresh_chunk_id'],f'sim={sim:.3f}',f'history={history_count}',flush=True)
            if activation.active and last_result_s is not None and sim-last_result_s>config['online']['controller_stall_sim_s']:
                terminal='CONTROLLER_ERROR';terminal_reason='no current controller result within fixed stall timeout';break
            commands.append(command);journal.write('commands.csv',command)
            nextpose=integrate_unicycle(pose,[command['v_mps'],command['omega_radps']],dt)
            set_precise_agent_pose(agent,nextpose,z_m=config['agent']['z_m'])
            world.step(render=False)
            new_sim=float(world.current_time)
            # Explicit physics stepping is authoritative; USD timeline follows the
            # completed physics clock once per tick, without a second dynamics step.
            sync_timeline(new_sim)
            if not np.isclose(new_sim-sim,dt,atol=1e-9,rtol=0):raise RuntimeError(f'Isaac simulation step {new_sim-sim} != {dt}')
            wait=pacer.remaining(new_sim,time.monotonic())
            if wait>0:time.sleep(wait)
            pose=actual_pose(agent)
            if not np.allclose(nextpose,pose,atol=1e-10,rtol=0):raise RuntimeError('applied unicycle state disagrees with USD readback')
            tick+=1
            st={'state_id':tick,'tick':tick,**stamp(new_sim,new_sim-origin),'x':float(pose[0]),'y':float(pose[1]),'yaw':float(pose[2]),
                'incoming_command_id':command['command_id'],'active_reference_version':command['reference_version'],'active_chunk_id':command['chunk_id'],
                'isaac_timeline_time_s':float(omni.timeline.get_timeline_interface().get_current_time())}
            states.append(st);journal.write('execution.csv',st)
            timing=pacer.observe(new_sim,st['host_monotonic_s']);stall=st['host_monotonic_s']-loop_previous_host;loop_previous_host=st['host_monotonic_s']
            journal.write('loop.jsonl',{'state_id':tick,'sim_time_s':new_sim,'host_monotonic_s':st['host_monotonic_s'],
                'loop_interval_host_s':stall,'inference_in_flight':pending_prediction is not None,
                'capture_queue_depth':len(frames)+len(encoder_pending),'pending_chunk_id':pending_prediction,**timing})
            if panel and (tick-1)%capture_stride==0:
                panel.update(ep.name,st,activation.active,pending_prediction or (activation.installed['chunk_id'] if activation.installed!=activation.active else None),
                    pending_prediction is not None,history_count,normal_activations,timing['rtf'] or 0,states,ep,last_frame)
                # Display the overlay only after the model RGB was copied. The
                # next capture clears it before rendering again.
                world.render()
                if float(world.current_time)!=new_sim:raise RuntimeError('GUI render advanced physics')
            if first_activation is not None:
                if new_sim-first_activation>=config['online']['maximum_active_sim_s']-1e-9:
                    terminal='EPISODE_LIMIT';terminal_reason='maximum active simulation duration';break
                if predictions-1>=config['online']['maximum_handoff_attempts'] and activation.installed==activation.active and not pending_prediction and new_sim-last_activation>=config['online']['postroll_sim_s']-1e-9:
                    terminal='ATTEMPT_LIMIT';terminal_reason='fixed attempt count plus available 1s postroll';break
    except Exception as exc:
        terminal='TECHNICAL_INVALID';terminal_reason=f'{type(exc).__name__}: {exc}'
        dump_new(ep/'runtime_error.json',{'error':terminal_reason,'traceback':traceback.format_exc(),'state':st})
    finally:
        encoder.shutdown(wait=True)
        while encoder_pending:
            frame=encoder_pending.popleft().result();allframes.append(frame);journal.write('capture.jsonl',frame)
        # Any already-sent response is preserved separately if it arrives after
        # execution ends; it cannot become an activated or valid handoff.
        if wire_busy:
            deadline=time.monotonic()+config['online']['request_timeout_s']+5
            while wire_busy and time.monotonic()<deadline:
                for msg in model.drain():
                    if msg.get('type')=='result':
                        wire_busy=False;timeout_response=msg
                        history_count=msg.get('history_count',history_count)
                        if outstanding_frame:outstanding_frame['transmission_status']=msg['status']
                        if msg.get('kind')=='prediction':
                            dump_new(ep/'nonactivated_response.json',{'reason':'response observed after execution termination','result':msg,'end_state':st})
                            if activation.active:
                                ctx=make_event_context(ep,msg,activation.active,{**st,**stamp(st['sim_time_s'],st['episode_time_s'])},None,model_ready['history_contract'])
                                ctx.update(event_id=f'handoff_{len(contexts):03d}',status=msg['status'] if msg['status']!='PREDICTION_READY' else 'TECHNICAL_INVALID',error='not activated before episode termination')
                                contexts.append(ctx)
                    if msg.get('type') in ('fatal','worker_exit','error'):wire_busy=False
                if wire_busy:time.sleep(.01)
        model.send('close');closed=model.await_type('closed',timeout=20)
        journal.close()
    # Preserve an installed response even if no corresponding command activated.
    if current_context and current_context.get('switch_state_id') is None and not any(c is current_context for c in contexts):
        current_context.update(status=terminal if terminal in ('CONTROLLER_ERROR','MODEL_ERROR','PROTOCOL_ERROR','TECHNICAL_INVALID','EXECUTOR_STALLED','SCENE_INVALID') else 'TECHNICAL_INVALID',error='installed response never activated: '+terminal_reason)
        if current_context.get('old_chunk_id'):
            current_context['event_id']=f'handoff_{len(contexts):03d}';contexts.append(current_context)
        else:initial_context=current_context
    # Finalize only from actually logged states/commands, never from model RTT.
    for i,context in enumerate(contexts):
        switch=context.get('switch_state_id')
        context['execution_stream_range']={'start_state_id':0,'end_state_id':states[-1]['state_id']}
        context['request_state_id']=next((r['state_id'] for r in states if r['host_monotonic_s']>=context['t_request']['host_monotonic_s']),states[-1]['state_id'])
        if switch is not None:
            next_switch=next((c['switch_state_id'] for c in contexts[i+1:] if c.get('switch_state_id') is not None),states[-1]['state_id'])
            context['post_switch_end_state_id']=next_switch
            a=context['request_state_id'];b=context['ready_state_id'];
            exact_ids=[r['state_id'] for r in states if context['t_request']['host_monotonic_s']<=r['host_monotonic_s']<=context['t_ready_host']['host_monotonic_s']]
            exact_a=exact_ids[0] if exact_ids else a; exact_b=exact_ids[-1] if exact_ids else a
            segment=states[exact_a:exact_b+1]
            context['client_inflight_state_range']={'start_state_id':exact_a,'end_state_id':exact_b,'sample_count':len(exact_ids)}
            xy=sum(np.hypot(s2['x']-s1['x'],s2['y']-s1['y']) for s1,s2 in zip(segment,segment[1:]))
            yaw=sum(abs(float(np.arctan2(np.sin(s2['yaw']-s1['yaw']),np.cos(s2['yaw']-s1['yaw'])))) for s1,s2 in zip(segment,segment[1:]))
            context['status']=('VALID_HANDOFF_MOVING' if xy>1e-6 or yaw>1e-6 else 'VALID_HANDOFF_STATIONARY') if exact_b>exact_a and next_switch>switch else 'TECHNICAL_INVALID'
            if context['status']=='TECHNICAL_INVALID':context['error']='no measured client-inflight state interval or no FRESH post-switch integration'
            hostspan=states[b]['host_monotonic_s']-states[a]['host_monotonic_s'];simspan=states[b]['sim_time_s']-states[a]['sim_time_s']
            rtf=simspan/hostspan if hostspan>0 else None
            context['timing_flags']={'timing_valid':rtf is not None and config['online']['rtf_valid_min']<=rtf<=config['online']['rtf_valid_max'],
                'timing_basis':'request-seen to ready-seen state interval; exact client-inflight pacing independently derived in metrics',
                'history_valid':True,'history_full':context['history_full'],'overlap_observed':exact_b>exact_a,
                'post_switch_execution_available':next_switch>switch,'rtf':rtf,
                'inference_state_update_count':max(0,exact_b-exact_a),'inference_translation_path_m':xy,'inference_abs_yaw_change_rad':yaw}
        else:
            context['post_switch_end_state_id']=None
            context['timing_flags']={'timing_valid':False,'history_valid':bool(context.get('history_snapshot_ref')),
                'history_full':context['history_full'],'overlap_observed':context['ready_state_id']>context['request_state_id'],'post_switch_execution_available':False}
        dump_new(ep/'handoffs'/context['event_id']/'context.json',context)
    if initial_context:dump_new(ep/'bootstrap.json',initial_context)
    with (ep/'rgb_index.csv').open('x',newline='') as stream:
        fields=['frame_id','path','sha256','bytes','capture_sim_time_s','episode_time_s','capture_monotonic_ns','capture_host_utc','rendered_state_id','pose_world','camera','bootstrap','stationary','render_state_stable','transmission_status','wire_seq']
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for frame in allframes:writer.writerow({key:json.dumps(frame[key]) if isinstance(frame.get(key),(list,dict)) else frame.get(key) for key in fields})
    metadata={'schema_version':1,**spec,'status':terminal,'termination_reason':terminal_reason,
        'initial_activation_sim_time_s':first_activation,'end_sim_time_s':states[-1]['sim_time_s'],
        'state_count':len(states),'command_count':len(commands),'captured_frames':len(allframes),
        'predictions_requested':predictions,'handoff_attempts':max(0,predictions-1),'activated_handoffs':normal_activations,
        'history_count_final':history_count,'scene':scene,'camera':camera_metadata(camera,agent,config),
        'execution':config['execution'],'history_contract':model_ready['history_contract'],
        'model_worker':model_ready,'mpc_worker':mpc_ready,'session_open':opened,'session_close':closed,
        'collision_validity':'unknown','kinematic_execution':True,'physical_robot_execution':False,
        'missed_deadlines':pacer.missed_deadlines,'maximum_lateness_s':pacer.maximum_lateness_s,
        'episode_rtf':(states[-1]['sim_time_s']-states[0]['sim_time_s'])/(states[-1]['host_monotonic_s']-states[0]['host_monotonic_s']) if len(states)>1 else None,
        'finalization_host_s':time.monotonic()-states[-1]['host_monotonic_s'],
        'resolved_integration_dt_s':dt,'isaac_clock_precision':'PhysX emits float32 dt; exact integrator uses that resolved value',
        'timeline_policy':'auto_update disabled; after each World physics-only step, set and commit USD timeline to actual World.current_time',
        'untransmitted_frames':sum(f['transmission_status']=='CAPTURED_NOT_SENT' for f in allframes)}
    dump_new(ep/'metadata.json',metadata)
    dump_new(ep/'completion.json',{'completed_at':stamp(),'status':terminal,'raw_manifest':raw_manifest(ep)})
    if panel:
        panel.update(ep.name,states[-1],activation.active,None,False,history_count,normal_activations,metadata['episode_rtf'] or 0,states,ep,allframes[-1] if allframes else None)
    print('EPISODE_COMPLETE',ep.name,terminal,'handoffs',normal_activations,'frames',len(allframes),flush=True)
    return metadata


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path);parser.add_argument('--gui',action='store_true')
    parser.add_argument('--episode',help='run only an unexecuted frozen episode; otherwise all remaining')
    args=parser.parse_args();run=args.run.resolve();config=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    frozen=read_json(run/'provenance.json')
    for entry in frozen['source_files']:
        if file_record(ROOT/entry['path'],ROOT)!=entry:raise ValueError(f'collector source differs from freeze: {entry["path"]}')
    for field in ('config','schedule','protocol'):
        if file_record(run/frozen[field]['path'],run)!=frozen[field]:raise ValueError(f'frozen {field} changed')
    (run/'logs').mkdir(exist_ok=True);session=str(time.time_ns())
    checkout=(ROOT/config['paths']['lightnav_checkout']).resolve()
    model=Worker([str(checkout/'.venv/bin/python'),str(ROOT/'scripts/online_lightnav_worker.py'),'--config',str(run/'config_snapshot.yaml')],run/'logs'/f'lightnav_worker_{session}.log')
    mpc=Worker([str(checkout/'mujoco_demo/.venv/bin/python'),str(ROOT/'scripts/online_mpc_worker.py'),'--lightnav-checkout',str(checkout)],run/'logs'/f'mpc_worker_{session}.log')
    app=None
    try:
        model_ready=model.await_type('ready');mpc_ready=mpc.await_type('ready')
        dump_new(run/'logs'/f'workers_{session}.json',{'lightnav':model_ready,'mpc':mpc_ready})
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':not args.gui,'width':1280,'height':800,'window_width':1440,'window_height':960,
            'renderer':'RayTracedLighting','anti_aliasing':0,'display_options':0})
        from robotless_runtime import runtime_scene
        from isaacsim.core.api import World
        schedule=read_json(run/'episode_schedule.json')['episodes']
        specs=[ep for ep in schedule if args.episode is None or ep['episode_id']==args.episode]
        if not specs:raise ValueError('episode not in frozen schedule')
        stage,agent,camera,annotator,scene=runtime_scene(config,np.array(specs[0]['R0']),app=app)
        world=World(physics_dt=config['execution']['simulation_dt_s'],rendering_dt=config['execution']['simulation_dt_s'],stage_units_in_meters=1.0)
        world.reset()
        panel=LivePanel(app,config,specs[0]['R0']) if args.gui else None
        dump_new(run/'logs'/f'isaac_runtime_{session}.json',{'started':stamp(),'pid':os.getpid(),'scene':scene,'gui':args.gui,
            'simulation':'World physics stepping with only logical Xform, no robot or rigid body dynamics',
            'config':file_record(run/'config_snapshot.yaml',run),'nvidia_smi':subprocess.check_output(['nvidia-smi','--query-gpu=name,memory.total,memory.used','--format=csv'],text=True),
            'gpu_processes':subprocess.check_output(['nvidia-smi','--query-compute-apps=pid,process_name,used_memory','--format=csv'],text=True)})
        for spec in specs:
            collect_episode(spec,run,config,app,world,agent,camera,annotator,scene,model,mpc,panel,model_ready,mpc_ready)
            if args.gui:
                import omni.kit.renderer.capture
                screen=run/'logs'/f'live_gui_{spec["episode_id"]}_{session}.png'
                # Allow the new UI image to resolve before the requested screenshot.
                for _ in range(12):world.render()
                omni.kit.renderer.capture.acquire_renderer_capture_interface().capture_next_frame_swapchain(str(screen))
                for _ in range(8):world.render()
                dump_new(run/'logs'/f'live_gui_{spec["episode_id"]}_{session}.json',{'label':'LIVE ONLINE COLLECTION',
                    'capture_after_episode_execution':True,'screenshot':file_record(screen,run) if screen.exists() else None})
        print('COLLECTION_SCHEDULE_FINISHED',flush=True)
    finally:
        model.shutdown('shutdown');mpc.shutdown()
        if app:app.close()

if __name__=='__main__':main()

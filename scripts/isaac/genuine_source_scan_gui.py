#!/usr/bin/env python3
"""Isaac display of the saved moving/mismatch source inventory. No new execution."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts/isaac')]
import numpy as np
import yaml
from reconciliation.gp_se2_reference import prepare_reference
from reconciliation.se2 import local_trajectory_to_world,wrap_angle

LABEL='SAVED SOURCE INSPECTION - NO RECONCILIATION / NEW EXECUTION'


def read(path):return json.loads(Path(path).read_text())
def sha(path):
    with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(path,value):
    with Path(path).open('x') as f:json.dump(value,f,indent=2,allow_nan=False)


class SavedSources:
    def __init__(self,run):
        self.run=Path(run).resolve();self.hashes={}
        for name in ('validation.json','source.json','summary.json','ledger.json','protocol.yaml'):
            self.record(self.run/name)
        if not read(self.run/'validation.json')['valid']:raise ValueError('source scan not valid')
        self.source=read(self.run/'source.json')
        for path,h in {**self.source['source_hashes'],**self.source['code_hashes']}.items():
            if sha(path)!=h:raise ValueError('source mismatch: '+path)
        protocol=yaml.safe_load((self.run/'protocol.yaml').read_text())
        self.corpus=(ROOT/protocol['source_run']).resolve()
        cfg=self.corpus/'config_snapshot.yaml';self.record(cfg);self.config=yaml.safe_load(cfg.read_text())
        self.summary=read(self.run/'summary.json');lookup={r['case_id']:r for r in read(self.run/'ledger.json')}
        self.ids=self.summary['subsets']['candidate']['case_ids'];self.cases=[]
        for cid in self.ids:
            row=lookup[cid]
            if not row['flags']['candidate']:raise ValueError('candidate summary/ledger mismatch')
            self.cases.append(self.load_case(row))
        if not self.cases:raise ValueError('no saved qualifying sources')
        self.index=0;self.sample=len(self.case['past'])-1;self.playing=False

    def record(self,path):
        path=Path(path).resolve();self.hashes[str(path)]=sha(path)
        return path

    def load_case(self,row):
        c=read(self.record(row['source_paths']['context']));arrays={}
        for kind,anchor in [('old',c['R_old_obs']),('fresh',c['R_obs'])]:
            world=np.load(self.record(row['source_paths'][kind+'_world']))
            raw=np.load(self.record(row['source_paths'][kind+'_raw_local']))
            expected=local_trajectory_to_world(anchor,raw)
            if not np.allclose(world[:,:2],expected[:,:2],atol=1e-12,rtol=0) or not np.allclose(wrap_angle(world[:,2]-expected[:,2]),0,atol=1e-12,rtol=0):raise ValueError('observation anchor differs')
            world.flags.writeable=False;arrays[kind]=world
        common=prepare_reference(arrays['fresh'],c['B'],[10.,10.,1.])['common_world']
        if hashlib.sha256(common.tobytes()).hexdigest()!=row['common_value_sha256']:raise ValueError('common reference hash differs')
        common.flags.writeable=False;arrays['common']=common
        ep=Path(row['source_paths']['context']).parents[2]
        with self.record(ep/'execution.csv').open() as f:
            states=[r for r in csv.DictReader(f) if c['obs_state_id']<=int(r['state_id'])<=c['switch_state_id']]
        past=np.array([[float(r[k]) for k in ('x','y','yaw')] for r in states]);times=np.array([float(r['sim_time_s']) for r in states])
        if len(past)<2 or np.any(np.diff(times)<=0) or not np.array_equal(past[0],c['R_obs']) or not np.array_equal(past[-1],c['B']):raise ValueError('saved observation/B execution differs')
        meta=read(self.record(ep/'chunks'/c['fresh_chunk_id']/'metadata.json'));obs=meta['observation'];rgb=self.record(ep/obs['path'])
        if sha(rgb)!=obs['sha256'] or obs['pose_world']!=c['R_obs']:raise ValueError('saved observation image differs')
        past.flags.writeable=False;times.flags.writeable=False
        return dict(row=row,context=c,**arrays,past=past,times=times-times[0],rgb=str(rgb))

    @property
    def case(self):return self.cases[self.index]

    def select(self,index):
        self.index=int(index)%len(self.cases);self.sample=len(self.case['past'])-1;self.playing=False

    def seek(self,elapsed):
        self.sample=max(0,min(len(self.case['past'])-1,int(np.searchsorted(self.case['times'],elapsed,side='right')-1)))
        if elapsed>=self.case['times'][-1]:self.playing=False


class Gui:
    def __init__(self,saved,app,out):
        import omni.ui as ui
        from pxr import UsdLux,UsdGeom,Gf
        from isaacsim.util.debug_draw import _debug_draw
        from robotless_runtime import runtime_scene
        self.saved,self.app,self.out=saved,app,out;self.show_common=False;self.actions=[];self.clock=time.monotonic()
        self.stage,self.agent,_,_,scene=runtime_scene(saved.config,np.asarray(saved.case['context']['B']),app=app)
        # Viewer-only fill: no modified RGB is sent to a model or replaces source data.
        light=UsdLux.RectLight.Define(self.stage,'/World/SourceInventoryDisplayFill')
        light.CreateIntensityAttr(1000.);light.CreateExposureAttr(0.);light.CreateColorAttr(Gf.Vec3f(1.,1.,1.));light.CreateNormalizeAttr(False)
        light.CreateWidthAttr(8.);light.CreateHeightAttr(8.)
        self.light_translation=UsdGeom.Xformable(light.GetPrim()).AddTranslateOp()
        self.draw=_debug_draw.acquire_debug_draw_interface();self.panel=ui.Window('Genuine source inventory',width=650,height=1000)
        with self.panel.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=5):
                    ui.Label('GENUINE MOVING HANDOFF SOURCES',height=30,style={'font_size':22})
                    ui.Label('13 / 881 qualify; 11 interior, 2 endpoint cases',height=26,style={'font_size':19})
                    ui.Label(LABEL,height=35,word_wrap=True,style={'font_size':16})
                    self.case_label=ui.Label('',height=34,style={'font_size':18})
                    self.combo=ui.ComboBox(saved.index,*saved.ids,height=30)
                    self.combo.model.add_item_changed_fn(lambda model,item:self.change(model.get_item_value_model().as_int))
                    with ui.HStack(height=30):
                        ui.Button('Previous case',clicked_fn=lambda:self.set_combo(saved.index-1))
                        ui.Button('Next case',clicked_fn=lambda:self.set_combo(saved.index+1))
                    self.metrics=ui.Label('',height=155,word_wrap=True,style={'font_size':17})
                    ui.Label('BLUE: original OLD prediction\nPINK: original FRESH prediction\nORANGE: recorded OLD execution, observation to B\nGREEN: FRESH observation; YELLOW: actual B\nWHITE: 20cm footprint; outer ring: +5cm clearance',height=112,style={'font_size':17})
                    with ui.HStack(height=28):
                        ui.Label('Show unchanged prepared FRESH (cyan)',width=430)
                        check=ui.CheckBox();check.model.add_value_changed_fn(lambda m:self.toggle_common(m.as_bool))
                    ui.Label('Original saved FRESH observation RGB:',height=24)
                    self.image=ui.Image(saved.case['rgb'],height=210,fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT)
                    self.time_label=ui.Label('',height=30,style={'font_size':17})
                    with ui.HStack(height=30):
                        ui.Button('Replay saved OLD to B',clicked_fn=self.play)
                        ui.Button('Show B',clicked_fn=self.end)
                    with ui.HStack(height=30):
                        ui.Button('Overhead',clicked_fn=self.overhead)
                        ui.Button('Capture',clicked_fn=lambda:self.capture('manual_'+str(time.monotonic_ns())))
                    ui.Label('No optimized transition exists in this source scan.\nGeometric source validity does not prove B-to-FRESH feasibility.\nWorld XY metres, yaw radians; FRESH is not re-anchored.\nOverlay z=1.35m and neutral fill are display only.\nPast replay selects saved samples; no integration/physics.',height=112,word_wrap=True,style={'font_size':15})
        for _ in range(4):app.update()
        w=ui.Workspace.get_window('Stage')
        if w:self.panel.dock_in(w,ui.DockPosition.SAME)
        for name in ('Property','Content','Console','Stage','Render Settings'):
            w=ui.Workspace.get_window(name)
            if w:w.visible=False
        for _ in range(5):app.update()
        ui.Workspace.set_dock_id_width(self.panel.dock_id,700);self.panel.focus();self.overhead();self.refresh()
        save(out/'scene.json',dict(scene=scene,display_fill=dict(prim='/World/SourceInventoryDisplayFill',intensity=1000,exposure=0,color=[1,1,1],width=8,height=8,z_m=2.6),source_scene_unchanged_on_disk=True))

    def set_combo(self,index):self.combo.model.get_item_value_model().set_value(index%len(self.saved.cases))
    def change(self,index):
        self.saved.select(index);self.image.source_url=self.saved.case['rgb'];self.overhead();self.refresh()
        self.actions.append(dict(action='select',case_id=self.saved.ids[self.saved.index]))
    def toggle_common(self,enabled):self.show_common=enabled;self.refresh()
    def play(self):
        self.saved.sample=0;self.saved.playing=True;self.clock=time.monotonic();self.refresh()
    def end(self):
        self.saved.playing=False;self.saved.sample=len(self.saved.case['past'])-1;self.refresh()
    def tick(self):
        if self.saved.playing:self.saved.seek(time.monotonic()-self.clock);self.refresh()

    def overhead(self):
        from pxr import UsdGeom,Gf
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport
        from robotless_online_replay import overview_aperture
        c=self.saved.case;points=np.vstack([c['old'],c['fresh'],c['past']]);res=get_active_viewport().resolution
        center,span=overview_aperture(points,float(res[0])/res[1])
        set_camera_view(eye=[*center,2.4],target=[*center,.1],camera_prim_path='/OmniverseKit_Persp')
        cam=UsdGeom.Camera(self.stage.GetPrimAtPath('/OmniverseKit_Persp'));cam.CreateProjectionAttr(UsdGeom.Tokens.orthographic)
        cam.CreateHorizontalApertureAttr(float(span[0])*10);cam.CreateVerticalApertureAttr(float(span[1])*10)
        self.light_translation.Set(Gf.Vec3d(*center,2.6));self.camera=dict(center_world_xy=center.tolist(),span_world_xy_m=span,equal_xy_scale=True)

    def refresh(self):
        from robotless_old_consistent_observation import set_precise_agent_pose
        s=self.saved;c=s.case;r=c['row'];m=r['mismatch'];v=r['motion'];b=np.array(c['context']['B']);p=c['past'][s.sample]
        set_precise_agent_pose(self.agent,p,z_m=s.config['agent']['z_m'])
        self.draw.clear_lines();self.draw.clear_points()
        def line(points,color,width=4):
            q=[[float(x[0]),float(x[1]),1.35] for x in points]
            if len(q)>1:self.draw.draw_lines(q[:-1],q[1:],[color]*(len(q)-1),[float(width)]*(len(q)-1))
        def point(p,color,size):self.draw.draw_points([[float(p[0]),float(p[1]),1.36]],[color],[float(size)])
        blue=(.08,.4,1.,1.);pink=(1.,.05,.6,1.);orange=(1.,.45,.02,1.);yellow=(1.,1.,0.,1.);green=(.05,1.,.3,1.)
        line(c['old'],blue,4);line(c['fresh'],pink,5)
        for p0 in c['fresh']:
            point(p0,pink,6);line([p0[:2],p0[:2]+.08*np.array([np.cos(p0[2]),np.sin(p0[2])])],pink,2)
        if self.show_common:line(c['common'],(0.,1.,1.,1.),2)
        line(c['past'][:s.sample+1],orange,6);point(c['context']['R_obs'],green,12);point(b,yellow,14)
        line([b[:2],b[:2]+.22*np.array([np.cos(b[2]),np.sin(b[2])])],yellow,4)
        angle=np.linspace(0,2*np.pi,65)
        for radius,color in [(.20,(1.,1.,1.,1.)),(.25,(.6,.6,.6,1.))]:line(p[:2]+radius*np.column_stack([np.cos(angle),np.sin(angle)]),color,2)
        direction=m['reliable_window_direction_deg'];normal=m['signed_lateral_gap_m'];proj=m['projection'];remaining=proj['total_arc_length_m']-proj['s_Q_m']
        self.case_label.text=f'{s.index+1}/13  {r["case_id"]}'
        self.metrics.text=(f'Physical v before B: {v["v_minus_mps"]:.3f} m/s; travel: {v["observation_to_B_arc_m"]:.3f} m\n'
            f'Lateral gap: {abs(normal):.3f} m; direction: {direction:.2f} deg; yaw: {m["pose_yaw_difference_deg"]:.2f} deg\n'
            f'Clearance raw / future suffix: {r["entire_raw_environment"]["minimum_clearance_m"]:.3f} / {r["raw_suffix_clearance_m"]:.3f} m\n'
            f'B clearance: {r["boundary_environment"]["clearance_m"]:.3f} m; required: 0.050 m\n'
            f'Projection: {proj["projection_location"]}; raw arc after Q: {remaining:.3f} m\n'
            f'Future clearance <=0.20m: {r["flags"]["candidate_obstacle_sensitive"]}; GP/MPC outcome: NOT TESTED')
        self.time_label.text=f'Saved OLD playback: {c["times"][s.sample]:.3f} / {c["times"][-1]:.3f}s (ends at B)'

    def capture(self,name):
        import omni.kit.renderer.capture as capture
        path=self.out/(name+'.png')
        if path.exists():raise FileExistsError(path)
        for _ in range(20):self.app.update()
        capture.acquire_renderer_capture_interface().capture_next_frame_swapchain(str(path))
        deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            self.app.update()
            if path.exists() and path.stat().st_size:break
        else:raise RuntimeError('GUI capture failed')
        for p,h in self.saved.hashes.items():
            if sha(p)!=h:raise ValueError('source changed during GUI display')
        save(path.with_suffix('.json'),dict(label=LABEL,capture_utc=datetime.now(timezone.utc).isoformat(),png_sha256=sha(path),
            case_id=self.saved.ids[self.saved.index],record=self.saved.case['row'],camera=self.camera,
            saved_sample=self.saved.sample,display_overlay_z_m=1.35,source_hashes=self.saved.hashes,new_model_GP_MPC_rollout_calls=0))
        print('GENUINE_SOURCE_GUI_READY '+str(path),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--case',default='episode_008_repeat_01/handoff_023');parser.add_argument('--no-hold',action='store_true');a=parser.parse_args()
    saved=SavedSources(a.run);saved.select(saved.ids.index(a.case));out=a.output.resolve()
    if out.is_relative_to(a.run.resolve()):raise ValueError('display output must be separate from source scan')
    out.mkdir(parents=True,exist_ok=False)
    from isaacsim import SimulationApp
    app=SimulationApp(dict(headless=False,window_width=1900,window_height=1100,width=1200,height=1000,renderer='RayTracedLighting',anti_aliasing=0))
    try:
        gui=Gui(saved,app,out);gui.capture('source_inventory_gui')
        save(out/'runtime.json',dict(pid=__import__('os').getpid(),label=LABEL,candidate_ids=saved.ids,viewer_source_sha256=sha(__file__),initial_case=a.case,source_run=str(a.run.resolve()),new_model_GP_MPC_rollout_calls=0))
        while app.is_running() and not a.no_hold:gui.tick();app.update();time.sleep(.02)
    finally:app.close()


if __name__=='__main__':main()

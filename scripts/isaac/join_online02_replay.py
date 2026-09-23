#!/usr/bin/env python3
"""Recorded online Isaac GUI. Reads saved samples; no new model or controller."""
import argparse,json,sys,time
from pathlib import Path
import numpy as np,yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
from reconciliation.join_online02 import ORDER,LABEL
from reconciliation.join_source03 import read,save,sha
from robotless_online_replay import SavedEpisode,overview_aperture

class Viewer:
    def __init__(self,run,output,app):
        import omni.ui as ui
        from isaacsim.util.debug_draw import _debug_draw
        from join_online02_collect import setup_scene
        self.run,self.output,self.app=run,output,app
        self.saved={e:SavedEpisode(run,e) for e in ORDER}
        self.analysis={e['summary']['episode']:e for e in read(run/'aggregate/analysis.json')['episodes']}
        self.config=yaml.safe_load((run/'config_snapshot.yaml').read_text())
        self.stage,self.agent,_,_,_,_,self.prop,self.scene=setup_scene(run,self.config,self.saved[ORDER[0]].poses[0],app)
        self.draw=_debug_draw.acquire_debug_draw_interface();self.eid=ORDER[0];self.chunk_index=0;self.busy=False
        self.clock=time.monotonic();self.samples=[];self.actions=[]
        self.panel=ui.Window(LABEL,width=660,height=1120)
        with self.panel.frame:
            with ui.VStack(spacing=5):
                ui.Label(LABEL,height=44,word_wrap=True,style={'font_size':19})
                ui.Label('SAVED SAMPLES ONLY — no new inference/execution',height=24)
                self.episodes=ui.ComboBox(0,*ORDER,height=30)
                self.episodes.model.add_item_changed_fn(lambda m,i:self.select_episode(m.get_item_value_model().as_int))
                with ui.HStack(height=32):
                    ui.Button('Previous chunk',clicked_fn=lambda:self.choose(self.chunk_index-1))
                    ui.Button('Next chunk',clicked_fn=lambda:self.choose(self.chunk_index+1))
                    ui.Button('Play / Pause',clicked_fn=self.play)
                self.slider=ui.IntSlider(min=0,max=1,height=28);self.slider.model.add_value_changed_fn(lambda m:self.choose(m.as_int))
                with ui.HStack(height=25):
                    self.show=[]
                    for label in ['Ck-2','OLD','FRESH','Next recorded']:
                        ui.Label(label,width=90);cb=ui.CheckBox();cb.model.set_value(label!='Next recorded');cb.model.add_value_changed_fn(lambda m:self.refresh());self.show.append(cb)
                self.info=ui.Label('',height=370,word_wrap=True,style={'font_size':16})
                ui.Label('GRAY executed / BLUE OLD / MAGENTA FRESH / YELLOW B\nWHITE observation circle. Prediction is not actual execution.',height=45,word_wrap=True)
                self.rgb=ui.Image('',height=305,fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT)
                self.rgb_label=ui.Label('',height=55,word_wrap=True)
                ui.Label('Unmodified saved triggering RGB. Cart OFF world has no runtime cart.\nWorld view: equal XY metres, orthographic.',height=42,word_wrap=True)
        for _ in range(6):app.update()
        stagewindow=ui.Workspace.get_window('Stage')
        if stagewindow:self.panel.dock_in(stagewindow,ui.DockPosition.SAME)
        for name in ['Property','Content','Console','Stage','Render Settings']:
            w=ui.Workspace.get_window(name)
            if w:w.visible=False
        for _ in range(5):app.update()
        ui.Workspace.set_dock_id_width(self.panel.dock_id,670);self.panel.focus()
        self.select_episode(1)
    @property
    def s(self):return self.saved[self.eid]
    @property
    def rows(self):return self.analysis[self.eid]['rows']
    def select_episode(self,index):
        if self.busy:return
        self.busy=True;self.eid=ORDER[int(index)];self.s.reset()
        self.episodes.model.get_item_value_model().set_value(int(index));self.slider.max=max(0,len(self.rows)-1)
        from join_source02_preflight import set_present
        set_present(self.prop,self.eid.startswith('ON_'));self.busy=False
        self.actions.append(dict(action='episode',episode=self.eid));self.choose(0);self.camera()
    def choose(self,index):
        if self.busy or not self.rows:return
        self.busy=True;self.chunk_index=int(np.clip(index,0,len(self.rows)-1));self.slider.model.set_value(self.chunk_index)
        r=self.rows[self.chunk_index];self.s.seek_row(r['B_state_id'] if r['B_state_id'] is not None else r['observation_state_id'])
        self.s.playing=False;self.busy=False;self.actions.append(dict(action='chunk',episode=self.eid,chunk=r['chunk_id']));self.refresh()
    def play(self):
        self.s.playing=not self.s.playing;self.clock=time.monotonic();self.actions.append(dict(action='play_pause',playing=self.s.playing));self.refresh()
    def camera(self):
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport
        from pxr import UsdGeom
        scenario=read(self.run/'scenario.json');geometry=np.vstack([self.s.poses,*self.s.chunks.values(),np.r_[scenario['center_xy'],0.]])
        resolution=get_active_viewport().resolution;aspect=resolution[0]/resolution[1];xy,spans=overview_aperture(geometry,aspect)
        set_camera_view(eye=[float(xy[0]),float(xy[1]),2.7],target=[float(xy[0]),float(xy[1]),.15],camera_prim_path='/OmniverseKit_Persp')
        c=UsdGeom.Camera(self.stage.GetPrimAtPath('/OmniverseKit_Persp'));c.CreateProjectionAttr(UsdGeom.Tokens.orthographic)
        c.CreateHorizontalApertureAttr(float(spans[0])*10);c.CreateVerticalApertureAttr(float(spans[1])*10)
        self.view_record=dict(equal_xy_scale=True,world_xy_center=xy.tolist(),spans_m=list(spans))
    def refresh(self):
        if self.busy or not self.rows:return
        from robotless_old_consistent_observation import set_precise_agent_pose
        from robotless_runtime import actual_pose
        s=self.s;r=self.rows[self.chunk_index];set_precise_agent_pose(self.agent,s.poses[s.index],z_m=self.config['agent']['z_m'])
        np.testing.assert_allclose(actual_pose(self.agent),s.poses[s.index],atol=1e-10,rtol=0)
        self.draw.clear_lines();self.draw.clear_points()
        def path(p,color,width=3.):
            p=np.asarray(p)
            if len(p)<2:return
            points=[[float(x[0]),float(x[1]),.16] for x in p]
            self.draw.draw_lines(points[:-1],points[1:],[color]*(len(p)-1),[width]*(len(p)-1))
        path(s.poses[:s.index+1],(.2,.2,.2,1.),5.)
        for offset,cb,color,width in zip([-2,-1,0,1],self.show,[(.7,.7,.7,1),(.1,.3,1,1),(1,0,.7,1),(.6,.6,.6,1)],[2.,4.,4.,1.]):
            i=self.chunk_index+offset
            if cb.model.as_bool and 0<=i<len(self.rows) and 'world' in self.rows[i]:path(self.rows[i]['world'],color,width)
        for pose,color in [(r['observation'],(1,1,1,1)),(r['B'],(1,1,0,1))]:
            if pose is None:continue
            a=np.linspace(0,2*np.pi,65);circle=np.column_stack([pose[0]+.2*np.cos(a),pose[1]+.2*np.sin(a)])
            path(circle,color,2.);self.draw.draw_points([[pose[0],pose[1],.18]],[color],[12.])
        f=lambda x:'N/A' if x is None else f'{x:.3f}' if isinstance(x,(float,int)) and not isinstance(x,bool) else str(x)
        vals=[('Chunk',r['display_id']),('Class',r['classification']),('RAW SAFE / ONSET / FULL',f"{r.get('raw_safe')} / {r.get('onset')} / {r.get('full_bypass')}"),
            ('t obs / ready / apply (s)',' / '.join(f(r.get(k)) for k in ['t_obs','t_ready','t_apply'])),
            ('Obs centre / B edge clearance (m)',f(r.get('observation_cart_center_distance_m'))+' / '+f(r.get('B_cart_edge_clearance_m'))),
            ('Obs → B actual travel (m)',r.get('observation_to_B_travel_m')),
            ('Raw minimum edge (m)',(r.get('raw_geometry') or {}).get('whole',{}).get('minimum_clearance_m')),
            ('Raw arc / max lateral (m)',f(r.get('raw_arc_m'))+' / '+f(r.get('max_lateral_m'))),
            ('Final yaw vs hallway (rad)',r.get('final_yaw_relative_hallway_rad')),
            ('Endpoint vs front / rear (m)',f(r.get('endpoint_from_front_m'))+' / '+f(r.get('endpoint_from_rear_m'))),
            ('OLD / lifetime (s)',str(r.get('old_chunk_id'))+' / '+f(r.get('reference_lifetime_s'))),
            ('Episode termination',self.analysis[self.eid]['summary']['termination']),
            ('Saved replay time / state',f(s.times[s.index])+' / '+str(s.index))]
        self.info.text='\n'.join(k+': '+f(v) for k,v in vals)
        image=Path(r['RGB_path']);assert sha(image)==r['RGB_sha256'];self.rgb.source_url='file:'+str(image)
        self.rgb_label.text=r['frame_id']+' — exact saved model trigger\n'+r['classification_basis']
        self.samples.append(dict(episode=self.eid,chunk=r['chunk_id'],saved_state_id=s.index,pose=s.poses[s.index].tolist(),RGB_sha256=r['RGB_sha256']))
    def tick(self):
        now=time.monotonic();prior=self.s.index;self.s.advance(now-self.clock);self.clock=now
        if prior!=self.s.index:self.refresh()
        self.app.update()
    def capture(self,name):
        import omni.kit.renderer.capture as capture
        from PIL import Image
        p=self.output/(name+'.png')
        for _ in range(12):self.app.update()
        capture.acquire_renderer_capture_interface().capture_next_frame_swapchain(str(p))
        deadline=time.monotonic()+25
        while time.monotonic()<deadline:
            self.app.update()
            if p.exists() and p.stat().st_size:
                try:
                    im=np.asarray(Image.open(p));assert im.std()>1;break
                except OSError:pass
        else:raise RuntimeError('GUI screenshot unavailable')
        save(p.with_suffix('.json'),dict(label=LABEL,saved=self.samples[-1],image_sha256=sha(p),camera=self.view_record,
            source_analysis_sha256=sha(self.run/'aggregate/analysis.json'),new_model_calls=0,new_MPC_calls=0))
    def verify(self):
        for i,e in enumerate(ORDER):
            self.select_episode(i);self.choose(len(self.rows)-1);self.camera();self.capture(e)
            self.s.seek_row(0);self.play();start=time.monotonic()
            while time.monotonic()-start<.35:self.tick()
            if len(self.s.times)>1:assert self.s.index>0
            if self.s.playing:self.play()
            for s in self.saved.values():s.verify_unchanged()
        save(self.output/'runtime_validation.json',dict(valid=True,label=LABEL,actions=self.actions,displayed_saved_samples=self.samples,
            episodes=ORDER,actual_Isaac_renderer=True,OS_mouse_interaction_tested=False,source_JPEG_unchanged=True,
            new_model_calls=0,new_MPC_calls=0,new_execution_states=0))
        self.select_episode(1);self.choose(len(self.rows)-1)

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--verify',action='store_true');p.add_argument('--no-hold',action='store_true');a=p.parse_args()
    run=a.run.resolve();output=a.output.resolve();output.mkdir(parents=True,exist_ok=False)
    from isaacsim import SimulationApp
    app=SimulationApp(dict(headless=False,window_width=2400,window_height=1320,width=1600,height=1100,renderer='RayTracedLighting'))
    try:
        gui=Viewer(run,output,app)
        if a.verify:gui.verify()
        if not a.no_hold:
            while app.is_running():gui.tick()
    finally:app.close()
if __name__=='__main__':main()

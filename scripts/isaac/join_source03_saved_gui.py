#!/usr/bin/env python3
"""Display saved SOURCE03 curved FRESH in Hospital. No prediction or execution."""
import argparse
from datetime import datetime,timezone
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts/isaac')]
import numpy as np
import yaml
from reconciliation.join_source03 import read,save,sha
from reconciliation.join_source02 import observation_anchored_world

LABEL='SAVED LIGHTNAV PREDICTION - NOT EXECUTED'


def load_saved(run,condition='distance_near_H16'):
    if not read(run/'validation_v2.json')['valid']:raise ValueError('historical validation invalid')
    m=read(run/'input_manifests'/f'{condition}.json');e=read(run/'paired_diagnostics'/condition/'evaluation.json')
    paths=[run/'config_snapshot.yaml',run/'input_manifests'/f'{condition}.json',run/'paired_diagnostics'/condition/'evaluation.json',Path(m['triangles_path'])]
    arrays={}
    for branch in ('A','B','SHAM'):
        root=run/'paired_diagnostics'/condition/'branches'/branch/'chunks/terminal'
        raw,world=[np.load(root/name) for name in ('raw_local.npy','world.npy')]
        if raw.ndim!=2 or raw.shape[1]!=3 or len(raw)==0:raise ValueError('nonempty Nx3 required')
        if not np.array_equal(raw,np.asarray(e['details'][branch]['raw_local'])):raise ValueError('saved raw differs')
        if not np.allclose(world,observation_anchored_world(raw,m['observation_pose_world']),atol=1e-12,rtol=0):raise ValueError('observation anchor differs')
        paths.extend([root/'raw_local.npy',root/'world.npy',Path(m['final_frames'][branch]['path'])]);world.flags.writeable=False
        arrays[branch]=world
    return dict(manifest=m,evaluation=e,arrays=arrays,config=yaml.safe_load((run/'config_snapshot.yaml').read_text()),sources={str(p):sha(p) for p in paths})


class SavedGui:
    def __init__(self,saved,app,out):
        import omni.ui as ui
        from pxr import UsdLux,UsdGeom,Gf
        from isaacsim.util.debug_draw import _debug_draw
        from robotless_runtime import runtime_scene
        from robotless_old_consistent_observation import set_precise_agent_pose
        from join_source02_preflight import create_prop,set_present
        self.saved,self.app,self.out=saved,app,out;self.row=0;self.actions=[]
        cfg=saved['config'];m=saved['manifest'];self.pose=np.asarray(m['observation_pose_world'])
        self.stage,agent,_,_,scene=runtime_scene(cfg,self.pose,app=app)
        set_precise_agent_pose(agent,self.pose,z_m=cfg['agent']['z_m'])
        prop=create_prop(self.stage,m['prop']['source_prim'],m['center_xy'],m['prop']['desired_root_yaw_rad']);set_present(prop,True)
        if not np.array_equal(prop['triangles'],np.load(m['triangles_path'])['triangles']):raise ValueError('cart mesh mismatch')
        light=cfg['lighting'];fill=UsdLux.RectLight.Define(self.stage,light['prim'])
        fill.CreateIntensityAttr(light['intensity']);fill.CreateExposureAttr(light['exposure']);fill.CreateColorAttr(Gf.Vec3f(*light['color']))
        fill.CreateNormalizeAttr(light['normalize']);fill.CreateWidthAttr(light['width_m']);fill.CreateHeightAttr(light['height_m'])
        UsdGeom.Xformable(fill.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*light['translation_world_m']))
        self.draw=_debug_draw.acquire_debug_draw_interface();self.panel=ui.Window('SOURCE03 - curved FRESH',width=550,height=960)
        with self.panel.frame:
            with ui.VStack(spacing=6):
                ui.Label('SOURCE03 / distance_near_H16',height=34,style={'font_size':23})
                ui.Label(LABEL,height=28,style={'font_size':19})
                ui.Label('RED: original obstacle-ON FRESH\nBLUE: same-pose obstacle-OFF prediction\nYELLOW: observation pose (not moving B)',height=70,style={'font_size':18})
                ui.Label('Saved terminal RGB actually sent to LightNav:',height=25)
                ui.Image(m['final_frames']['B']['path'],height=280,fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT)
                ui.Label(m['instruction'],height=28,style={'font_size':17})
                ui.Label('Right curve relative to robot = decreasing world X.\nThis prediction turns, but its footprint hits the cart.',height=50,word_wrap=True,style={'font_size':18})
                self.info=ui.Label('',height=88,word_wrap=True,style={'font_size':18})
                with ui.HStack(height=32):
                    ui.Button('Previous row',clicked_fn=lambda:self.set_row(self.row-1))
                    ui.Button('Next row',clicked_fn=lambda:self.set_row(self.row+1))
                with ui.HStack(height=32):
                    ui.Button('Overhead',clicked_fn=self.overhead)
                    ui.Button('Capture GUI',clicked_fn=lambda:self.capture('manual_'+str(time.monotonic_ns())))
                ui.Label('White circle: 0.20m footprint at selected prediction row\nGray circle: footprint + 0.05m required clearance',height=50,word_wrap=True)
                ui.Label('World XY metres; rows are NOT timestamps.\nPath overlays raised to z=1.35m for visibility only.\nNo MPC / GP / new inference / motion / physics.',height=68,word_wrap=True)
        for _ in range(4):app.update()
        stage_panel=ui.Workspace.get_window('Stage')
        if stage_panel:self.panel.dock_in(stage_panel,ui.DockPosition.SAME)
        for name in ('Property','Content','Console','Stage','Render Settings'):
            w=ui.Workspace.get_window(name)
            if w:w.visible=False
        for _ in range(5):app.update()
        ui.Workspace.set_dock_id_width(self.panel.dock_id,570);self.panel.focus()
        self.overhead();self.set_row(saved['evaluation']['details']['B']['geometry_on']['first_unsafe_waypoint_zero_based'] or 0)
        save(out/'scene.json',dict(scene=scene,cart_exact_source=True,lighting=light,label=LABEL))

    def overhead(self):
        from pxr import UsdGeom
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport
        from robotless_online_replay import overview_aperture
        points=np.vstack([self.saved['arrays']['A'],self.saved['arrays']['B'],self.pose])
        resolution=get_active_viewport().resolution;center,span=overview_aperture(points,float(resolution[0])/resolution[1])
        set_camera_view(eye=[*center,2.8],target=[*center,.1],camera_prim_path='/OmniverseKit_Persp')
        camera=UsdGeom.Camera(self.stage.GetPrimAtPath('/OmniverseKit_Persp'));camera.CreateProjectionAttr(UsdGeom.Tokens.orthographic)
        camera.CreateHorizontalApertureAttr(float(span[0])*10);camera.CreateVerticalApertureAttr(float(span[1])*10)
        self.camera=dict(center_world_xy=center.tolist(),span_world_xy_m=span,projection='orthographic',equal_xy=True)

    def set_row(self,row):
        self.row=max(0,min(len(self.saved['arrays']['B'])-1,int(row)));self.draw.clear_lines();self.draw.clear_points()
        def line(points,color,width=4.):
            points=[[float(p[0]),float(p[1]),1.35] for p in points]
            self.draw.draw_lines(points[:-1],points[1:],[color]*(len(points)-1),[width]*(len(points)-1))
        for b,col in [('A',(.1,.45,1.,1.)),('B',(1.,.08,.12,1.))]:
            w=self.saved['arrays'][b];line(w,col,5.)
            self.draw.draw_points([[float(p[0]),float(p[1]),1.36] for p in w],[col]*len(w),[7.]*len(w))
            for p in w:line([p[:2],p[:2]+.10*np.array([np.cos(p[2]),np.sin(p[2])])],col,2.)
        line([self.pose[:2],self.pose[:2]+.2*np.array([np.cos(self.pose[2]),np.sin(self.pose[2])])],(1.,.9,0.,1.),5.)
        self.draw.draw_points([[*self.pose[:2],1.36]],[(1.,.9,0.,1.)],[11.])
        p=self.saved['arrays']['B'][self.row];angles=np.linspace(0,2*np.pi,65)
        for radius,color in [(.20,(1.,1.,1.,1.)),(.25,(.6,.6,.6,1.))]:line(p[:2]+radius*np.column_stack([np.cos(angles),np.sin(angles)]),color,2.)
        g=self.saved['evaluation']['details']['B']['geometry_on'];raw=self.saved['evaluation']['details']['B']['raw_local'][-1]
        self.info.text=(f'Row {self.row} / {len(self.saved["arrays"]["B"])-1}: edge clearance {g["node_clearance_m"][self.row]:.4f} m\n'
            f'Whole path minimum: {g["whole"]["minimum_clearance_m"]:.4f} m (required +0.05m)\n'
            f'Final lateral: {raw[1]:.3f} m; final yaw: {np.degrees(raw[2]):.1f} deg')
        self.actions.append(dict(prediction_row=self.row,is_execution=False,wall_monotonic_s=time.monotonic()))

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
        for p,h in self.saved['sources'].items():
            if sha(p)!=h:raise ValueError('historical source changed')
        save(path.with_suffix('.json'),dict(label=LABEL,png_sha256=sha(path),source_sha256=self.saved['sources'],camera=self.camera,
            selected_prediction_row=self.row,display_overlay_z_m=1.35,agent_pose_world=self.pose.tolist(),new_inference_MPC_GP_execution=0))
        print('SOURCE03_GUI_READY '+str(path),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--no-hold',action='store_true')
    a=p.parse_args();saved=load_saved(a.run.resolve());out=a.output.resolve()
    if out.is_relative_to(a.run.resolve()):raise ValueError('GUI artifacts must be outside historical run')
    out.mkdir(parents=True,exist_ok=False)
    from isaacsim import SimulationApp
    app=SimulationApp(dict(headless=False,window_width=1800,window_height=1050,width=1200,height=950,renderer='RayTracedLighting',anti_aliasing=0))
    try:
        gui=SavedGui(saved,app,out);gui.capture('curved_fresh_gui')
        save(out/'runtime.json',dict(pid=__import__('os').getpid(),label=LABEL,condition='distance_near_H16',source_run=str(a.run.resolve()),new_model_MPC_GP_calls=0,physics_advanced=False))
        while app.is_running() and not a.no_hold:app.update();time.sleep(.02)
    finally:app.close()


if __name__=='__main__':main()

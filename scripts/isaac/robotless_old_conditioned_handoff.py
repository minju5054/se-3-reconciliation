#!/usr/bin/env python3
"""Actual Hospital viewport evidence from saved OLD/FRESH and derived geometry."""
import argparse
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT/'scripts'))
import numpy as np
import robotless_old_conditioned_artifacts as a
from reconciliation.robotless_single_chunk import load_config, save_json_exclusive, make_observation_time, observation_to_world


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory',type=Path);parser.add_argument('--view-only',action='store_true')
    parser.add_argument('--no-hold',action='store_true')
    args=parser.parse_args();run=args.run_directory.resolve()
    config,record,source,manifest=a.load_inputs(run)
    metrics=a.read_json(run/'derived/episode_metrics.json');expected=a.derive(config,source,manifest)
    if metrics['episodes']!=expected:raise ValueError('saved geometry differs from source recomputation')
    selected=a.read_json(run/'derived/representatives.json')
    ids=sorted({r['episode_id'] for r in selected['rules'] if r['episode_id'] is not None})
    if not args.view_only and (run/'visualization.json').exists():raise FileExistsError('visualization already exists')
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':False})
    try:
        import omni.ui as ui
        from isaacsim.core.utils.viewports import set_camera_view
        from isaacsim.util.debug_draw import _debug_draw
        from robotless_runtime import runtime_scene, set_agent_pose, actual_pose, viewport_capture, stopped_time
        from debug_draw_trajectories import draw_polyline, draw_pose_points
        by_id={e['episode_id']:e for e in expected}
        _,agent,_,_,scene=runtime_scene(load_config(source/'config_snapshot.yaml'),np.array(by_id[ids[0]]['reference']['R_obs']),app=app)
        draw=_debug_draw.acquire_debug_draw_interface();v=config['visualization'];z=v['path_z_m']
        def line(points,color,height=z,width=None):
            if len(points)>1:draw_polyline(draw,np.array(points),z=height,color=v[color],width=width or v['line_width'])
        def point(xy,color,height):draw_pose_points(draw,np.array([xy]),z=height,color=v[color],size=v['point_size'])
        def arrow(xy,phi,color,height):
            if phi is None:return
            start=[*xy,height];end=[xy[0]+v['arrow_length_m']*math.cos(phi),xy[1]+v['arrow_length_m']*math.sin(phi),height]
            tips=[[end[0]+v['arrowhead_length_m']*math.cos(phi+s*5*math.pi/6),end[1]+v['arrowhead_length_m']*math.sin(phi+s*5*math.pi/6),height] for s in (-1,1)]
            draw.draw_lines([start,end,end],[end,*tips],[v[color]]*3,[v['line_width']]*3)
            draw.draw_lines([[*xy,z]],[[*xy,height]],[v[color]],[1.5])
        def show(episode_id,detail=False):
            e=by_id[episode_id];ref=e['reference'];row=e['conditions'][-1];obs=ref['R_obs']
            old=np.load(source/'episodes'/episode_id/'derived/chunk_000_world.npy',allow_pickle=False)
            fresh=np.load(source/'episodes'/episode_id/'derived/chunk_001_world.npy',allow_pickle=False)
            set_agent_pose(agent,np.array(obs),z_m=0.)
            draw.clear_lines();draw.clear_points()
            line(old,'old_color');line(fresh,'fresh_color')
            line(e['aligned_continuation_world'],'aligned_color',v['aligned_z_m'])
            point(obs,'observation_color',z+.02);point(ref['Q_old_obs_xy_world_m'],'old_projection_color',z+.01)
            # Distinct drawing heights expose physically coincident markers without XY offsets.
            straight_xy=row['straight']['B_world'][:2]
            straight_height=v['fresh_tangent_z_m']+.05
            point(straight_xy,'straight_color',straight_height)
            draw.draw_lines([[*straight_xy,z]],[[*straight_xy,straight_height]],[v['straight_color']],[1.5])
            line([obs[:2],ref['Q_old_obs_xy_world_m']],'old_projection_color',z+.01,2.)
            if row['status']=='AVAILABLE':
                b=row['B_old'];q=row['fresh_projection']['Q_xy_world_m']
                point(b,'old_boundary_color',z+.025);point(q,'fresh_projection_color',z+.08)
                draw.draw_lines([[*q,z]],[[*q,z+.08]],[v['fresh_projection_color']],[1.5])
                line([b[:2],q],'fresh_projection_color',z+.01,2.)
                arrow(b[:2],row['phi_old_local_rad'],'old_boundary_color',v['incoming_z_m'])
                arrow(q,row['phi_fresh_local_rad'],'fresh_tangent_color',v['fresh_tangent_z_m'])
            offsets=np.array([[-.7,-.6,1.35],[.12,0,.12]] if detail else [[-1.8,-1.25,1.95],[.35,0,.12]])
            xy=observation_to_world(obs,np.column_stack([offsets[:,:2],np.zeros(2)]))
            set_camera_view(eye=[*xy[0,:2],offsets[0,2]],target=[*xy[1,:2],offsets[1,2]],camera_prim_path='/OmniverseKit_Persp')
            return old,fresh
        panel=ui.Window('OLD-conditioned handoff',width=590,height=360)
        with panel.frame:
            with ui.VStack(spacing=4):
                ui.Label('BLUE raw OLD | MAGENTA raw FRESH | GREEN aligned OLD future',height=25)
                ui.Label('RED R_obs | LAVENDER Q_old_obs | YELLOW B_straight',height=25)
                ui.Label('ORANGE B_old / OLD tangent | WHITE Q | CYAN FRESH tangent',height=25)
                ui.Label('Controlled delay 1 s; no execution; only drawing heights differ',height=25)
                for episode_id in ids:
                    ui.Button(episode_id+' overview',clicked_fn=lambda e=episode_id:show(e),height=25)
        records=[]
        for episode_id in ids:
            old,fresh=show(episode_id);before=actual_pose(agent);images=[]
            for detail in (False,True):
                show(episode_id,detail)
                relative=f'evidence/representative_{episode_id}_{"detail" if detail else "overview"}.png'
                if not args.view_only:
                    if (run/relative).exists():raise FileExistsError(relative)
                    viewport_capture(run/relative,app=app);images.append(a.file_record(run/relative,run))
            after=actual_pose(agent)
            if not np.array_equal(before,after):raise ValueError('logical observation agent moved')
            records.append({'episode_id':episode_id,'episode_geometry':by_id[episode_id],
                'OLD_world_drawn':old.tolist(),'FRESH_world_drawn':fresh.tolist(),
                'agent_pose_before':before.tolist(),'agent_pose_after':after.tolist(),'images':images,'time':stopped_time()})
        a.load_inputs(run)
        if not args.view_only:
            save_json_exclusive(run/'visualization.json',{'created_time':stopped_time(),'scene':scene,'scene_load_count':1,
                'representatives':records,'source':a.file_record(run/'source.json',run),
                'metrics':a.file_record(run/'derived/episode_metrics.json',run),'selection':a.file_record(run/'derived/representatives.json',run),
                'configuration':a.file_record(run/'config_snapshot.yaml',run),'timeline_time_s':0,
                'new_lightnav_inference_count':0,'actual_execution_time':None,'geometry_scaling':1,'angular_magnification':1,
                'scene_visibility_modifications':[],'fresh_reanchored':False,
                'display_convention':'unchanged world XY/yaw, display Z layers only; original OLD and FRESH remain unchanged',
                'point_display_heights_m':{'R_obs':z+.02,'Q_old_obs':z+.01,'B_straight':v['fresh_tangent_z_m']+.05,'B_old':z+.025,'Q_fresh':z+.08},
                'processing_source_sha256':{str(p.relative_to(ROOT)):a.sha256_file(p) for p in (Path(__file__),ROOT/'scripts/isaac/robotless_runtime.py',ROOT/'scripts/isaac/debug_draw_trajectories.py')}})
        print('OLD_CONDITIONED_ISAAC_RENDERED',flush=True)
        if not args.no_hold:
            while app.is_running() and panel.visible:app.update()
    finally:app.close()


if __name__=='__main__':main()

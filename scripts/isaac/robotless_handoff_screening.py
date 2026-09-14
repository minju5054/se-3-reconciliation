#!/usr/bin/env python3
"""One-scene frozen-bank capture and deterministic representative views."""
import argparse
import copy
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src'))
sys.path.insert(0, str(ROOT/'scripts'))
import numpy as np
import yaml
import robotless_screening_artifacts as artifacts
from reconciliation.robotless_single_chunk import save_json_exclusive, sha256_file, observation_to_world
from reconciliation.robotless_successive_chunks import SUCCESSIVE_FRAME_CONVENTIONS, validate_successive_observation_metadata
from reconciliation.se2 import relative_pose


def capture(run, app):
    from robotless_runtime import runtime_scene, set_agent_pose, actual_pose, capture_observation, stopped_time, git_output
    config, manifest, manifest_hash = artifacts.load_frozen(run)
    if (run/'episodes').exists():
        raise FileExistsError('capture requires a new, frozen bank; no overwrites')
    checkout = (ROOT/config['paths']['lightnav_checkout']).resolve()
    source_sha = git_output(checkout,'rev-parse','HEAD')
    if source_sha != config['lightnav']['expected_git_sha'] or git_output(checkout,'status','--porcelain','--untracked-files=no'):
        raise ValueError('LightNav source is not the pinned clean checkout')
    _, agent, camera, annotator, scene = runtime_scene(config,np.array(manifest['episodes'][0]['R0']),app=app)
    inventory=scene['runtime_inventory']
    if inventory['rigid_body_count'] or inventory['physics_scene_count']:
        raise ValueError('robotless capture cannot contain rigid bodies or physics scenes')
    results=[]
    for episode in manifest['episodes']:
        artifacts.load_frozen(run)
        location=run/'episodes'/episode['episode_id']
        (location/'raw').mkdir(parents=True)
        (location/'derived').mkdir()
        epconfig=copy.deepcopy({k:v for k,v in config.items() if k != 'episodes'})
        epconfig['agent']['pose_world']=episode['R0']
        epconfig['instruction']=episode['instruction']
        epconfig['screening_episode']=episode
        epconfig['screening_manifest_sha256']=manifest_hash
        with (location/'config_snapshot.yaml').open('x') as stream:
            yaml.safe_dump(epconfig,stream,sort_keys=False)
        status={'episode_id':episode['episode_id'],'category':episode['category'],'manifest_sha256':manifest_hash}
        try:
            set_agent_pose(agent,np.array(episode['R0']),z_m=config['agent']['z_m'])
            old,old_check=capture_observation(epconfig,location,agent,camera,annotator,np.array(episode['R0']),0)
            before=actual_pose(agent)
            start=stopped_time()
            set_agent_pose(agent,np.array(episode['R1']),z_m=config['agent']['z_m'])
            end=stopped_time()
            after=actual_pose(agent)
            fresh,fresh_check=capture_observation(epconfig,location,agent,camera,annotator,np.array(episode['R1']),1)
            metadata={'schema_version':1,'stage':config['stage'],'instruction':episode['instruction'],
                'episode_id':episode['episode_id'],'category':episode['category'],'manifest_sha256':manifest_hash,
                'R0':old['agent_pose_world'],'R1':fresh['agent_pose_world'],
                'configured_local_displacement':episode['local_displacement'],'observations':[old,fresh],
                'agent_prim_path':str(agent.GetPath()),'agent_z_m':config['agent']['z_m'],'scene':scene,
                'coordinate_conventions':SUCCESSIVE_FRAME_CONVENTIONS,
                'research_git_sha':git_output(ROOT,'rev-parse','HEAD'),'research_git_status':git_output(ROOT,'status','--short'),
                'research_source_sha256':artifacts.phase_sources(['scripts/isaac/robotless_handoff_screening.py','scripts/isaac/robotless_runtime.py']),
                'lightnav_git_sha':source_sha,'lightnav_checkout':str(checkout),'lightnav_tracked_source_clean':True,
                'model_checkpoint_identifier':config['lightnav']['checkpoint_identifier'],
                'model_checkpoint_revision':config['lightnav']['checkpoint_revision'],
                'model_checkpoint_path':str((ROOT/config['paths']['checkpoint_path']).resolve()),
                'config':{'path':'config_snapshot.yaml','sha256':sha256_file(location/'config_snapshot.yaml')},
                'execution_time':None,'motion_during_capture':False,'inference_concurrent_with_capture':False,'isaac_scene_load_count':1,
                'scripted_displacement':{'configured_local_se2':episode['local_displacement'],
                    'actual_local_se2':relative_pose(before,after).tolist(),'actual_world_delta_xy_m':(after[:2]-before[:2]).tolist(),
                    'agent_pose_before':before.tolist(),'agent_pose_after':after.tolist(),
                    'started_time':start,'completed_time':end,'source_frame':'logical_agent_at_observation_000','target_frame':'Isaac_world_z_up',
                    'operation':'T_world_R1 = T_world_R0 @ Delta_local','assignment':'direct logical USD pose assignment','dynamics_advanced':False},
                'observation_time_semantics':'host UTC/monotonic render-readback completion; each static capture bracketed; stopped timeline zero'}
            validate_successive_observation_metadata(metadata)
            save_json_exclusive(location/'metadata.json',metadata)
            save_json_exclusive(location/'capture_validation.json',{'scene_loaded':True,'scene_load_count':1,'no_robot_model':True,
                'logical_agent_poses_set':True,'rgb_captured':True,'camera_unchanged':True,'scripted_displacement_matches_config':True,
                'no_simulation_dynamics':True,'timeline_time_s':0.,'observations':[old_check,fresh_check],
                'source_metadata_sha256':sha256_file(location/'metadata.json')})
            status.update(capture_valid=True,reason=None)
        except Exception as error:
            status.update(capture_valid=False,reason=f'{type(error).__name__}: {error}')
        status['completed_time']=stopped_time()
        save_json_exclusive(location/'capture_status.json',status)
        results.append(status)
        print(episode['episode_id'],'CAPTURE_VALID' if status['capture_valid'] else status['reason'],flush=True)
    artifacts.load_frozen(run)
    save_json_exclusive(run/'capture_batch.json',{'manifest_sha256':manifest_hash,'scene':scene,'scene_load_count':1,
        'episode_count':len(results),'statuses':results,'completed_time':stopped_time(),'lightnav_inference_count':0})
    print('SCREENING_CAPTURE_COMPLETE',flush=True)


def visualize(run, app, view_only=False):
    from robotless_runtime import runtime_scene, set_agent_pose, actual_pose, viewport_capture, stopped_time
    from debug_draw_trajectories import draw_polyline, draw_pose_points
    from isaacsim.core.utils.viewports import set_camera_view
    from isaacsim.util.debug_draw import _debug_draw
    import omni.ui as ui
    config,manifest,manifest_hash=artifacts.load_frozen(run)
    representatives=artifacts.read_json(run/'aggregate/representatives.json')
    ids=sorted({r['episode_id'] for r in representatives['rules'] if r['episode_id'] is not None})
    if not ids:
        raise ValueError('no defined representative to visualize')
    if not view_only and (run/'visualization.json').exists():
        raise FileExistsError('existing visualization is immutable')
    by_id={e['episode_id']:e for e in manifest['episodes']}
    _,agent,_,_,scene=runtime_scene(config,np.array(by_id[ids[0]]['R1']),app=app)
    draw=_debug_draw.acquire_debug_draw_interface()
    v=config['projection_visualization']; z=v['path_z_m']; width=v['line_width']; size=v['point_size']

    def arrow(xy,angle,prefix):
        height,length=v[f'{prefix}_z_m'],v[f'{prefix}_length_m']; color=v[f'{prefix}_color']
        start=[*xy,height]; end=[xy[0]+length*math.cos(angle),xy[1]+length*math.sin(angle),height]
        ends=[[end[0]+v['arrowhead_length_m']*math.cos(angle+s*5*math.pi/6),end[1]+v['arrowhead_length_m']*math.sin(angle+s*5*math.pi/6),height] for s in (-1,1)]
        draw.draw_lines([start,end,end],[end,*ends],[color]*3,[width]*3)

    def show(episode_id,detail):
        location=run/'episodes'/episode_id
        metadata=artifacts.read_json(location/'metadata.json')
        metrics=artifacts.read_json(location/'derived/projection_metrics.json')
        row=next(r for r in metrics['conditions'] if r['tau_s']==1.)
        b,q,r_obs=row['B_world'],row['Q_xy_world_m'],metadata['R1']
        set_agent_pose(agent,np.array(r_obs),z_m=config['agent']['z_m'])
        draw.clear_lines();draw.clear_points()
        arrays=[np.load(location/f'derived/chunk_{i:03d}_world.npy',allow_pickle=False) for i in range(2)]
        for array,color in zip(arrays,[v['old_color'],v['fresh_color']],strict=True):
            draw_polyline(draw,array,z=z,color=color,width=width)
        for xy,color in [(r_obs,v['observation_color']),(b,v['boundary_color']),(q,v['projection_color'])]:
            draw_pose_points(draw,np.array([xy]),z=z+.01,color=color,size=size)
        draw_polyline(draw,np.array([b[:2],q]),z=z+.01,color=v['connector_color'],width=width)
        draw.draw_lines([[*b[:2],z],[*q,z]],[[*b[:2],v['incoming_z_m']],[*q,v['pose_yaw_z_m']]],
            [v['boundary_color'],v['projection_color']],[1.5,1.5])
        arrow(b[:2],row['phi_in_rad'],'incoming')
        if row['tangent_available']:
            arrow(q,row['phi_F_rad'],'tangent')
        arrow(q,row['theta_Q_rad'],'pose_yaw')
        prefix='detail' if detail else 'overview'
        offsets=np.array([v[f'{prefix}_eye_local_m'],v[f'{prefix}_target_local_m']])
        anchor=np.array(b if detail else r_obs)
        xy=observation_to_world(anchor,np.column_stack((offsets[:,:2],np.zeros(2))))
        set_camera_view(eye=[*xy[0,:2],offsets[0,2]],target=[*xy[1,:2],offsets[1,2]],camera_prim_path='/OmniverseKit_Persp')
        return row,arrays,r_obs

    panel=ui.Window('Screening representatives',width=540,height=310)
    with panel.frame:
        with ui.VStack(spacing=5):
            ui.Label('BLUE OLD | MAGENTA FRESH | RED R_obs | WHITE Q',height=24)
            ui.Label('YELLOW B/incoming | GREEN B-Q | CYAN tangent | ORANGE pose yaw',height=24)
            ui.Label('Controlled delay=1 s; only arrow drawing heights differ',height=24)
            for episode_id in ids:
                ui.Button(episode_id+' overview',clicked_fn=lambda e=episode_id:show(e,False),height=27)
    records=[]
    for episode_id in ids:
        row,arrays,r_obs=show(episode_id,False)
        before=actual_pose(agent)
        images=[]
        for detail in (False,True):
            show(episode_id,detail)
            name=f'evidence/representative_{episode_id}_{"detail" if detail else "overview"}.png'
            if not view_only:
                viewport_capture(run/name,app=app)
                images.append(artifacts.file_record(run/name,run))
        after=actual_pose(agent)
        if not np.array_equal(before,after):
            raise ValueError('logical agent moved while rendering representative')
        records.append({'episode_id':episode_id,'row':row,'R_obs':r_obs,'OLD_world_drawn':arrays[0].tolist(),
            'FRESH_world_drawn':arrays[1].tolist(),'agent_pose_before':before.tolist(),'agent_pose_after':after.tolist(),
            'created_time':stopped_time(),'images':images})
    artifacts.load_frozen(run)
    if not view_only:
        save_json_exclusive(run/'visualization.json',{'manifest_sha256':manifest_hash,'scene':scene,'scene_load_count':1,
            'selection_input':artifacts.file_record(run/'aggregate/representatives.json',run),'representatives':records,
            'geometry_scaling':1.,'angular_magnification':1.,'scene_visibility_modifications':[],
            'timeline_time_s':0.,'new_lightnav_inference_count':0,'created_time':stopped_time(),
            'display_convention':'Prior projection height layers; unchanged world XY and actual direction angles; separate overview/detail cameras',
            'processing_source_sha256':artifacts.phase_sources(['scripts/isaac/robotless_handoff_screening.py','scripts/isaac/robotless_runtime.py'])})
    print('SCREENING_REPRESENTATIVES_RENDERED',flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['capture','visualize'])
    parser.add_argument('run_directory',type=Path)
    parser.add_argument('--view-only',action='store_true')
    args=parser.parse_args()
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':False})
    try:
        if args.mode=='capture':
            if args.view_only: raise ValueError('view-only requires visualize')
            capture(args.run_directory.resolve(),app)
        else:
            visualize(args.run_directory.resolve(),app,args.view_only)
    except Exception:
        import traceback
        traceback.print_exc()
        raise
    finally:
        app.close()


if __name__=='__main__':
    main()

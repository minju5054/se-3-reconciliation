#!/usr/bin/env python3
"""Capture only new RGB1, or display saved OLD-consistent pilot geometry."""
import argparse
import copy
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src')); sys.path.insert(0, str(ROOT/'scripts'))
import numpy as np
import yaml
import robotless_old_consistent_artifacts as a
from reconciliation.robotless_single_chunk import save_json_exclusive, sha256_file, observation_to_world
from reconciliation.robotless_successive_chunks import SUCCESSIVE_FRAME_CONVENTIONS, validate_successive_observation_metadata
from reconciliation.se2 import relative_pose


def set_precise_agent_pose(agent, pose, *, z_m):
    """Double USD matrix avoids float32 Euler storage exceeding capture tolerance."""
    from pxr import Gf, UsdGeom
    pose=np.asarray(pose,dtype=float)
    if pose.shape!=(3,) or not np.all(np.isfinite(pose)) or not np.isfinite(z_m):
        raise ValueError('finite logical pose required')
    c,s=math.cos(pose[2]),math.sin(pose[2])
    matrix=np.array([[c,-s,0,pose[0]],[s,c,0,pose[1]],[0,0,1,z_m],[0,0,0,1]])
    xform=UsdGeom.Xformable(agent)
    attribute=agent.GetPrim().GetAttribute('xformOp:transform:oldConsistentPose')
    op=UsdGeom.XformOp(attribute) if attribute else xform.AddTransformOp(UsdGeom.XformOp.PrecisionDouble,'oldConsistentPose')
    op.Set(Gf.Matrix4d(*matrix.T.reshape(-1).tolist()))
    xform.SetXformOpOrder([op])


def capture(run, app):
    from robotless_runtime import runtime_scene, set_agent_pose, actual_pose, capture_observation, stopped_time, git_output
    config, manifest, record, mh = a.load_frozen(run)
    if (run/'capture_started.json').exists(): raise FileExistsError('capture already attempted; no overwrite/retry')
    save_json_exclusive(run/'capture_started.json', {'time': stopped_time(), 'manifest_sha256': mh,
        'processing_source_sha256': a.processing_sources(['scripts/isaac/robotless_old_consistent_observation.py','scripts/isaac/robotless_runtime.py'])})
    checkout = (ROOT/config['paths']['lightnav_checkout']).resolve()
    source_sha = git_output(checkout, 'rev-parse', 'HEAD')
    if source_sha != config['lightnav']['expected_git_sha'] or git_output(checkout,'status','--porcelain'):
        raise ValueError('upstream must be pinned and clean')
    _, agent, camera, annotator, scene = runtime_scene(config, np.array(manifest['episodes'][0]['plan']['R0']), app=app)
    results = []
    for episode in manifest['episodes']:
        a.load_frozen(run)
        location = run/'episodes'/episode['episode_id']; (location/'raw').mkdir(parents=True); (location/'derived').mkdir()
        plan = episode['plan']; source = Path(record['source_run'])/'episodes'/episode['episode_id']
        status = {'episode_id': episode['episode_id'], 'manifest_sha256': mh, 'capture_valid': False, 'reason': plan['reason']}
        if plan['status'] == 'OLD_ARC_INSUFFICIENT':
            status['status'] = 'OLD_ARC_INSUFFICIENT'
        else:
            epconfig = copy.deepcopy(config)
            epconfig['agent']['pose_world'] = plan['R0']
            epconfig['agent']['local_displacement'] = relative_pose(plan['R0'], plan['R1_old']).tolist()
            epconfig['instruction'] = episode['instruction']
            with (location/'config_snapshot.yaml').open('x') as stream: yaml.safe_dump(epconfig, stream, sort_keys=False)
            # Exact frozen JPEG copy for the unchanged successive client; never recapture RGB0.
            with (location/'raw/observation_000.jpg').open('xb') as stream: stream.write((source/'raw/observation_000.jpg').read_bytes())
            try:
                old = copy.deepcopy(a.read_json(source/'metadata.json')['observations'][0])
                start = stopped_time()
                set_precise_agent_pose(agent, np.array(plan['R1_old']), z_m=config['agent']['z_m'])
                assigned = stopped_time()
                fresh, check = capture_observation(epconfig, location, agent, camera, annotator, np.array(plan['R1_old']), 1)
                metadata = {'schema_version': 1, 'stage': config['stage'], 'instruction': episode['instruction'],
                    'episode_id': episode['episode_id'], 'category': episode['category'], 'manifest_sha256': mh,
                    'R0': old['agent_pose_world'], 'R1': fresh['agent_pose_world'], 'planned_R1_old': plan['R1_old'],
                    'configured_local_displacement': epconfig['agent']['local_displacement'], 'observations': [old, fresh],
                    'agent_prim_path': str(agent.GetPath()), 'agent_z_m': config['agent']['z_m'], 'scene': scene,
                    'coordinate_conventions': SUCCESSIVE_FRAME_CONVENTIONS,
                    'research_git_sha': git_output(ROOT,'rev-parse','HEAD'), 'lightnav_git_sha': source_sha,
                    'lightnav_checkout': str(checkout), 'lightnav_tracked_source_clean': True,
                    'model_checkpoint_identifier': config['lightnav']['checkpoint_identifier'],
                    'model_checkpoint_revision': config['lightnav']['checkpoint_revision'],
                    'model_checkpoint_path': str((ROOT/config['paths']['checkpoint_path']).resolve()),
                    'config': a.file_record(location/'config_snapshot.yaml', location),
                    'execution_time': None, 'motion_during_capture': False, 'inference_concurrent_with_capture': False,
                    'isaac_scene_load_count': 1, 'new_rgb_capture_count': 1,
                    'source_RGB0_reused': a.file_record(source/'raw/observation_000.jpg', Path(record['source_run'])),
                    'source_RGB0_observation_metadata_reused_unchanged': True,
                    'pose_assignment': {'requested_R1_old': plan['R1_old'], 'actual_readback': actual_pose(agent).tolist(),
                        'started_time': start, 'assigned_time': assigned, 'operation': 'direct double-precision USD matrix assignment; no dynamics',
                        'meaning': 'OLD-consistent spatial observation surrogate'},
                    'timing': 'RGB0 retains original source timestamp; RGB1 is new actual render readback; no intrinsic waypoint time'}
                validate_successive_observation_metadata(metadata)
                save_json_exclusive(location/'metadata.json', metadata)
                save_json_exclusive(location/'capture_validation.json', {'new_RGB1': check, 'source_RGB0_recaptured': False,
                    'requested_R1_old': plan['R1_old'], 'readback': fresh['agent_pose_world'],
                    'pose_error': (np.array(fresh['agent_pose_world'])-plan['R1_old']).tolist(),
                    'pose_readback_atol': config['pose_readback_atol'], 'metadata': a.file_record(location/'metadata.json',location)})
                status.update(status='CAPTURE_VALID', capture_valid=True, reason=None)
            except Exception as error:
                status.update(status='CAPTURE_INVALID', reason=f'{type(error).__name__}: {error}')
        status['completed_time'] = stopped_time()
        save_json_exclusive(location/'capture_status.json', status); results.append(status)
        print(episode['episode_id'], status['status'], status['reason'], flush=True)
    save_json_exclusive(run/'capture_batch.json', {'scene': scene, 'scene_load_count': 1, 'statuses': results,
        'manifest_sha256': mh, 'completed_time': stopped_time(), 'new_RGB0_capture_count': 0,
        'new_RGB1_capture_count': sum(r['capture_valid'] for r in results), 'lightnav_inference_count': 0})
    print('OLD_CONSISTENT_CAPTURE_COMPLETE', flush=True)


def visualize(run, app):
    from robotless_runtime import runtime_scene, set_agent_pose, actual_pose, viewport_capture, stopped_time
    from debug_draw_trajectories import draw_polyline, draw_pose_points
    from isaacsim.core.utils.viewports import set_camera_view
    from isaacsim.util.debug_draw import _debug_draw
    import omni.ui as ui
    config, manifest, _, mh = a.load_frozen(run)
    summary = a.read_json(run/'aggregate/summary.json'); valid = [r for r in summary['case_results'] if r['status']=='VALID_PAIR']
    if not valid: raise ValueError('no valid cases to visualize')
    if (run/'visualization.json').exists(): raise FileExistsError('visualization already exists')
    _, agent, _, _, scene = runtime_scene(config, np.array(valid[0]['plan']['R1_old']), app=app)
    draw = _debug_draw.acquire_debug_draw_interface()
    colors = {'old': [0.05,.25,1.,1.], 'fresh': [1.,0.,.85,1.], 'r0': [1.,.9,0.,1.],
        'fixed': [.7,.7,1.,1.], 'new': [1.,.35,.05,1.], 'q': [1.,1.,1.,1.], 'tangent': [0.,1.,1.,1.], 'advance': [.2,1.,.35,1.]}
    def line(points, color, z=.12, width=4.):
        if len(points)>1: draw_polyline(draw,np.asarray(points),z=z,color=colors[color],width=width)
    def point(p, color, z):
        draw_pose_points(draw,np.array([p]),z=z,color=colors[color],size=15.)
        draw.draw_lines([[*p[:2],.12]],[[*p[:2],z]],[colors[color]],[1.5])
    def arrow(p, phi, color, z):
        if phi is None: return
        end=[p[0]+.18*math.cos(phi),p[1]+.18*math.sin(phi),z]
        ends=[[end[0]+.025*math.cos(phi+s*5*math.pi/6),end[1]+.025*math.sin(phi+s*5*math.pi/6),z] for s in (-1,1)]
        draw.draw_lines([[*p[:2],z],end,end],[end,*ends],[colors[color]]*3,[4.]*3)
    panel=ui.Window('OLD-consistent observation pilot',width=650,height=175)
    with panel.frame:
        with ui.VStack():
            ui.Label('BLUE OLD | MAGENTA NEW FRESH | YELLOW R0 | LAVENDER fixed R1')
            ui.Label('ORANGE R1_old = B / OLD tangent | WHITE Q | CYAN FRESH tangent')
            ui.Label('GREEN augmented OLD advance (connector explicit) | tau = 0; no execution')
            ui.Label('Actual XY and angles; marker/tangent Z layers only')
    records=[]
    for r in valid:
        location=run/'episodes'/r['episode_id']; plan=r['plan']; m=r['metrics']; b=m['B_world']; q=m['fresh_projection']['Q_xy_world_m']
        old,fresh=[np.load(location/f'derived/chunk_{i:03d}_world.npy',allow_pickle=False) for i in (0,1)]
        set_precise_agent_pose(agent,np.array(b),z_m=config['agent']['z_m']); before=actual_pose(agent)
        draw.clear_lines(); draw.clear_points(); line(old,'old'); line(fresh,'fresh')
        j=plan['sample']['segment_index']; line(plan['augmented_path_world'][:j+1]+[b],'advance',.17)
        line([plan['R0'][:2],plan['fixed_R1'][:2]],'fixed',.15,2.)
        for p,color,z in [(plan['R0'],'r0',.20),(plan['fixed_R1'],'fixed',.34),(b,'new',.24),(q,'q',.30)]: point(p,color,z)
        line([b[:2],q],'q',.13,2.); arrow(b,m['phi_old_local_rad'],'new',.24); arrow(q,m['phi_fresh_local_rad'],'tangent',.30)
        # A wider camera intersects Hospital geometry at 007; retain its clear closer view.
        offsets=np.array([[-1.8,-1.25,2.15],[.35,0,.12]] if r['episode_id']=='episode_007' else [[-2.3,-1.6,2.7],[.3,0,.12]])
        xy=observation_to_world(plan['R0'],np.column_stack([offsets[:,:2],np.zeros(2)]))
        set_camera_view(eye=[*xy[0,:2],offsets[0,2]],target=[*xy[1,:2],offsets[1,2]],camera_prim_path='/OmniverseKit_Persp')
        path=run/f'evidence/{r["episode_id"]}_overview.png'; viewport_capture(path,app=app)
        after=actual_pose(agent)
        if not np.array_equal(before,after): raise ValueError('logical agent moved during view')
        records.append({'episode_id':r['episode_id'],'result':r,'OLD_world_drawn':old.tolist(),'FRESH_world_drawn':fresh.tolist(),
            'agent_pose_before':before.tolist(),'agent_pose_after':after.tolist(),'image':a.file_record(path,run),'time':stopped_time(),
            'overview_eye_R0_frame_m':offsets[0].tolist(),'overview_target_R0_frame_m':offsets[1].tolist()})
    save_json_exclusive(run/'visualization.json',{'scene':scene,'scene_load_count':1,'manifest_sha256':mh,
        'records':records,'summary':a.file_record(run/'aggregate/summary.json',run),'created_time':stopped_time(),
        'geometry_scaling':1.,'angular_magnification':1.,'scene_visibility_modifications':[],'timeline_time_s':0.,
        'actual_execution_time':None,'new_lightnav_inference_count':0,'display_colors':colors,
        'display_convention':'original XY and angles; raw paths z=.12, advance=.17, marker/tangent height layers; arrows .18m',
        'processing_source_sha256':a.processing_sources(['scripts/isaac/robotless_old_consistent_observation.py','scripts/isaac/robotless_runtime.py'])})
    print('OLD_CONSISTENT_VISUALIZATION_COMPLETE',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('mode',choices=['capture','visualize']); parser.add_argument('run_directory',type=Path)
    args=parser.parse_args()
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':False})
    try: (capture if args.mode=='capture' else visualize)(args.run_directory.resolve(),app)
    finally: app.close()

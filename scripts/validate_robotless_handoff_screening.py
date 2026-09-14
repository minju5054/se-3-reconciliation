#!/usr/bin/env python3
"""Audit frozen-bank collection, all metrics, duplication and reviewed images."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from PIL import Image

import robotless_screening_artifacts as artifacts
from reconciliation.robotless_handoff_screening import METRICS, MOTION, select_representatives, summarize, transition_metrics
from reconciliation.robotless_projection_handoff import csv_row
from reconciliation.robotless_single_chunk import load_config, observation_to_world, save_json_exclusive, sha256_file, validate_observation_time, validate_waypoints
from reconciliation.robotless_successive_chunks import round_trip_latency_ms, validate_successive_observation_metadata
from validate_robotless_single_chunk import check_camera, check_external_provenance, check_hash, protocol_wire, require, same_array
from validate_robotless_successive_chunks import check_protocol
from validate_robotless_projection_handoff import compare_value
from screen_robotless_handoffs import PLOT_NAMES, PLOT_LABELS

VALIDATED='ROBOTLESS_LIGHTNAV_HANDOFF_SCREENING_VALIDATED'
FAILED='ROBOTLESS_LIGHTNAV_HANDOFF_SCREENING_FAILED'
NOT_VALIDATED='ROBOTLESS_LIGHTNAV_HANDOFF_SCREENING_RUNTIME_NOT_VALIDATED'
REVIEW_FLAGS=('actual_capture_pairs','representative_OLD_FRESH','observation_boundary_projection',
    'connector_and_three_directions','all_representatives','all_required_plots','no_geometry_or_angle_exaggeration','no_robot')


def host_ns(event):
    validate_observation_time({**event,'simulation_time_s':event.get('simulation_time_s',0.)})
    return event['monotonic_ns']


def check_scene(scene,config):
    inventory=scene['runtime_inventory']
    require(inventory['no_robot_model'] is True and not inventory['articulation_roots'] and not inventory['robot_named_paths']
        and inventory['rigid_body_count']==0 and inventory['physics_scene_count']==0 and inventory['dynamics_advanced'] is False,'robot/dynamics present')
    require(inventory['authored_asset_references']==[config['scene']['asset_relative_path']],'wrong scene')
    require(scene['stage_units_in_meters']==1. and scene['up_axis']=='Z','wrong world convention')


def check_csv(path,expected):
    with path.open(newline='') as stream: actual=list(csv.DictReader(stream))
    require(len(actual)==len(expected),'CSV row count differs')
    for row,reference in zip(actual,expected,strict=True):
        require(set(row)==set(reference),'CSV columns differ')
        for key,value in reference.items():
            if isinstance(value,(list,dict)):
                require(json.loads(row[key])==value,f'CSV {key}')
            elif value is None or isinstance(value,(str,bool)):
                require(row[key]==('' if value is None else str(value)),f'CSV {key}')
            else:
                same_array(float(row[key]),value,f'CSV {key}',1e-12)


def check_evidence_png(path):
    with Image.open(path) as image:
        require(image.format=='PNG' and min(image.size)>=100,'invalid evidence PNG')
        image.verify()
    # PNG structural verification must precede decode; use a new handle for pixels.
    with Image.open(path) as image:
        require(float(np.asarray(image).std())>1,'blank evidence PNG')


def check_episode(run,episode,manifest_hash,freeze_ns):
    location=run/'episodes'/episode['episode_id']
    status=artifacts.read_json(location/'validation.json')
    require(status['episode_id']==episode['episode_id'] and status['category']==episode['category'],'episode replaced')
    require(status['manifest_sha256']==manifest_hash and status['attempted'] is True,'episode not tied to frozen manifest')
    for record in status['files']: artifacts.verify_file(location,record)
    require({x['path'] for x in status['files']}=={str(p.relative_to(location)) for p in location.rglob('*') if p.is_file()}-{'validation.json'},'episode file inventory changed')
    attempt=artifacts.read_json(location/'inference_attempt.json')
    require(attempt['manifest_sha256']==manifest_hash and host_ns(attempt['started_time'])>freeze_ns,'inference preceded freeze')
    epconfig=load_config(location/'config_snapshot.yaml')
    require(epconfig['screening_episode']==episode and epconfig['screening_manifest_sha256']==manifest_hash,'episode config changed')
    capture_status=artifacts.read_json(location/'capture_status.json')
    if not capture_status['capture_valid']:
        require(status['status']=='CAPTURE_INVALID' and bool(status['reason']) and status['inference_attempted'] is False,'invalid capture was not preserved')
        return status,[],None
    metadata=artifacts.read_json(location/'metadata.json')
    capture=artifacts.read_json(location/'capture_validation.json')
    validate_successive_observation_metadata(metadata)
    require(metadata['instruction']==episode['instruction']==epconfig['instruction'],'instruction changed')
    require(metadata['manifest_sha256']==manifest_hash,'capture manifest mismatch')
    for key in ('R0','R1'): same_array(metadata[key],episode[key],key,1e-7)
    check_hash(location,metadata['config'],'config_snapshot.yaml')
    require(capture['source_metadata_sha256']==sha256_file(location/'metadata.json'),'capture metadata hash')
    check_scene(metadata['scene'],epconfig)
    require(metadata['motion_during_capture'] is False and metadata['inference_concurrent_with_capture'] is False,'capture overlapped motion/inference')
    previous=freeze_ns
    displacement=metadata['scripted_displacement']
    order=[metadata['observations'][0]['capture_started_time'],metadata['observations'][0]['observation_time'],
        displacement['started_time'],displacement['completed_time'],metadata['observations'][1]['capture_started_time'],metadata['observations'][1]['observation_time']]
    for event in order:
        now=host_ns(event); require(now>=previous and event['simulation_time_s']==0,'capture/displacement clock order');previous=now
    require(previous<=host_ns(attempt['started_time']),'inference attempt precedes capture')
    for seq,observation in enumerate(metadata['observations']):
        check_hash(location,observation['rgb'],f'raw/observation_{seq:03d}.jpg')
        with Image.open(location/observation['rgb']['path']) as image:
            require(image.format=='JPEG' and image.mode=='RGB' and list(image.size)==observation['rgb']['resolution_width_height'],'invalid actual RGB')
            require(float(np.asarray(image).std())>1,'blank RGB');image.verify()
        check_camera({**metadata,**observation},epconfig,capture['observations'][seq],{'camera_basis_checks':observation['camera']['basis_checks']})
        pose=observation['agent_pose_world']; c,s=np.cos(pose[2]),np.sin(pose[2]); transform=np.eye(4)
        transform[:2,:2]=[[c,-s],[s,c]];transform[:3,3]=[pose[0],pose[1],epconfig['agent']['z_m']]
        same_array(observation['camera']['T_world_agent'],transform,'camera own pose',1e-7)
    connection=None
    protocol_path=location/'raw/lightnav_protocol.jsonl'
    if protocol_path.exists():
        records=[json.loads(line) for line in protocol_path.read_text().splitlines()]
        connections={r['connection_id'] for r in records}
        require(len(connections)<=1,'reconnect inside episode')
        connection=next(iter(connections),None)
        requests=[json.loads(protocol_wire(r)) for r in records if r['direction']=='request']
        expected=['login','reset','next','next']
        require([r['action'] for r in requests]==expected[:len(requests)],'unexpected protocol request sequence')
        for seq,request in enumerate(requests[2:]):
            require(request['data']['seq']==seq and request['data']['instruction']==episode['instruction'],'request instruction/sequence mismatch')
    if not (location/'inference_metadata.json').exists():
        require(status['status'] in ('MODEL_ERROR','PROTOCOL_ERROR','NONFINITE_OUTPUT') and bool(status['reason']),'missing inference result without a recorded model/protocol failure')
        return status,[],connection
    inference=artifacts.read_json(location/'inference_metadata.json')
    require(inference['execution_time'] is None and inference['agent_motion_during_inference'] is False,'unexpected actual execution')
    require(inference['transform']['correction_applied'] is False and inference['transform']['raw_arrays_unchanged'] is True,'unexpected trajectory correction')
    for key,relative in [('capture_metadata','metadata.json'),('protocol','raw/lightnav_protocol.jsonl')]:
        check_hash(location,inference['raw_inputs'][key],relative)
    check_hash(location,inference['config'],'config_snapshot.yaml')
    check_hash(location,inference['server_process'],'server_process.json')
    require((location/'server_process.json').read_bytes()==(run/'server_process.json').read_bytes(),'episode used a different server')
    locals_=[];worlds=[]
    for seq,chunk in enumerate(inference['chunks']):
        local=validate_waypoints(np.load(location/f'raw/chunk_{seq:03d}.npy',allow_pickle=False))
        world=validate_waypoints(np.load(location/f'derived/chunk_{seq:03d}_world.npy',allow_pickle=False))
        same_array(world,observation_to_world(metadata[f'R{seq}'],local),'own observation world transform',1e-12)
        require(chunk['response_actions_step']==seq+1,'server history did not reset to steps 1/2')
        require(chunk['agent_pose_world_at_observation']==metadata[f'R{seq}'],'chunk anchor changed')
        same_array(chunk['round_trip_latency_ms'],round_trip_latency_ms(chunk['request_send_time'],chunk['response_receive_time']),'RTT convention',1e-9)
        locals_.append(local);worlds.append(world)
    check_protocol(location,metadata,inference,locals_)
    check_external_provenance(metadata,epconfig,inference,artifacts.read_json(run/'server_process.json'))
    if inference['chunks'][0]['stop']:
        require(status['status']=='OLD_STOP','OLD stop omitted');return status,[],connection
    if inference['chunks'][1]['stop']:
        require(status['status']=='FRESH_STOP','FRESH stop omitted');return status,[],connection
    try:
        expected=transition_metrics(episode,metadata['R1'],worlds[1],*[sha256_file(location/f'raw/chunk_{i:03d}.npy') for i in range(2)])
    except ValueError as error:
        require('arc length' in str(error) and status['status']=='GEOMETRY_UNAVAILABLE','unexplained unavailable metrics')
        return status,[],connection
    require(status['status']=='VALID_PAIR','finite defined pair was silently excluded')
    metrics=artifacts.read_json(location/'derived/projection_metrics.json')
    require(metrics['manifest_sha256']==manifest_hash and metrics['inference_metadata_sha256']==sha256_file(location/'inference_metadata.json'),'metric provenance mismatch')
    for item in metrics['input_files']: artifacts.verify_file(location,item)
    require(len(metrics['conditions'])==4,'all controlled delays required')
    for row,reference in zip(metrics['conditions'],expected,strict=True):
        require(set(row)==set(reference),'projection fields differ')
        for key,value in reference.items(): compare_value(row[key],value,key)
    return status,metrics['conditions'],connection


def validate_analysis(run):
    config,manifest,manifest_hash=artifacts.load_frozen(run)
    started=artifacts.read_json(run/'inference_batch_started.json')
    freeze_ns=host_ns(manifest['frozen_time'])
    require(started['manifest_sha256']==manifest_hash and host_ns(started['time'])>freeze_ns,'bank freeze/inference order')
    capture=artifacts.read_json(run/'capture_batch.json');batch=artifacts.read_json(run/'inference_batch.json')
    require(capture['episode_count']==batch['attempted']==30 and capture['scene_load_count']==1,'wrong attempt or scene-load count')
    require(capture['manifest_sha256']==batch['manifest_sha256']==manifest_hash,'batch manifest mismatch')
    require(host_ns(capture['completed_time'])<host_ns(started['time']),'inference overlapped actual capture')
    statuses=[];rows=[];connections=[]
    require({p.name for p in (run/'episodes').iterdir()}=={e['episode_id'] for e in manifest['episodes']},'episode directory inventory changed')
    for episode in manifest['episodes']:
        status,conditions,connection=check_episode(run,episode,manifest_hash,freeze_ns)
        statuses.append(status);rows.extend(conditions)
        if connection is not None: connections.append(connection)
    require(len(connections)==len(set(connections)),'session identity reused across episodes')
    require(statuses==batch['statuses'],'batch status list differs from all episode records')
    require(all(s['status']!='PIPELINE_ERROR' for s in statuses),'pipeline error needs resolution')
    expected=summarize(rows,statuses)
    require(expected['diversity']['valid']>0,'no valid metric pairs collected')
    aggregate=run/'aggregate'
    transitions=artifacts.read_json(aggregate/'transitions.json')
    require(transitions['conditions']==rows and transitions['manifest_sha256']==manifest_hash,'aggregate dropped/changed episode rows')
    for record in transitions['input_episode_validations']: artifacts.verify_file(run,record)
    require(artifacts.read_json(aggregate/'diversity.json')==expected['diversity'],'diversity differs')
    require(artifacts.read_json(aggregate/'statistics.json')==expected['statistics'],'distribution statistics differ')
    require(artifacts.read_json(aggregate/'unique_pairs.json')['groups']==expected['unique_pairs'],'duplicate-aware medians differ')
    check_csv(aggregate/'transitions.csv',[csv_row(row) for row in rows])
    check_csv(aggregate/'unique_pairs.csv',expected['unique_pairs'])
    selection=artifacts.read_json(aggregate/'representatives.json')
    require(selection['rules']==select_representatives(rows) and selection['selection_is_visualization_only'] is True,'representative selection changed')
    artifacts.verify_file(run,selection['metrics_input'])
    analysis=artifacts.read_json(run/'analysis_manifest.json')
    for record in analysis['files']: artifacts.verify_file(run,record)
    artifacts.verify_file(run,analysis['plot_manifest'])
    plots=artifacts.read_json(run/'plot_manifest.json')
    require([Path(p['path']).name for p in plots['plots']]==PLOT_NAMES,'required plot set differs')
    artifacts.verify_file(run,plots['metrics_input']);artifacts.verify_file(run,plots['configuration'])
    for index,plot in enumerate(plots['plots']):
        artifacts.verify_file(run,plot)
        points=plot['plotted_points']
        if index<4:
            metric=METRICS[index]
            expected_points={(r['episode_id'],r['tau_s']):r for r in rows if r[metric] is not None}
            require(len(points)==len(expected_points) and {(p['episode_id'],p['tau_s']) for p in points}==set(expected_points),'plot dropped raw episode points')
            for p in points:
                r=expected_points[(p['episode_id'],p['tau_s'])]
                require(p['value']==r[metric] and p['pair_sha256']==r['pair_sha256'],'plotted value changed')
                same_array(p['display_x'],r['tau_s']+.05*(int(r['episode_id'].split('_')[-1])/29-.5),'display-only offset',1e-12)
        elif index<6:
            metric=('abs_e_dir_deg','abs_e_yaw_deg')[index-4]
            expected_points={r['episode_id']:r for r in rows if r['tau_s']==1. and r[metric] is not None}
            require(len(points)==len(expected_points) and {p['episode_id'] for p in points}==set(expected_points),'scatter dropped points')
            for p in points:
                r=expected_points[p['episode_id']];require(p['x']==r['e_perp_m'] and p['y']==r[metric],'scatter coordinates changed')
        else:
            require({p['pair_sha256']:p['episode_ids'] for p in points}==expected['diversity']['pair_groups'],'pair plot grouping differs')
    return {'manifest_sha256':manifest_hash,'diversity':expected['diversity'],'statistics':expected['statistics'],
        'representative_rules':selection['rules'],'validated_session_count':len(connections),'conditions':rows}


def validate_episode_points(run,details):
    panels=artifacts.read_json(run/'episode_points_manifest.json')
    require(panels['manifest_sha256']==details['manifest_sha256'],'episode panels manifest mismatch')
    for key in ('metrics_input','configuration','image'): artifacts.verify_file(run,panels[key])
    expected={(r['episode_id'],r['tau_s'],metric):r[metric] for r in details['conditions'] for metric in METRICS if r[metric] is not None}
    actual=panels['plotted_points']
    require(len(actual)==len(expected) and {(p['episode_id'],p['tau_s'],p['metric']):p['value'] for p in actual}==expected,
        'episode panels dropped or changed raw metric points')


def validate_visual(run,details):
    config,_,manifest_hash=artifacts.load_frozen(run)
    visual=artifacts.read_json(run/'visualization.json')
    require(visual['manifest_sha256']==manifest_hash and visual['scene_load_count']==1,'visual manifest/scene mismatch')
    require(visual['geometry_scaling']==visual['angular_magnification']==1 and visual['scene_visibility_modifications']==[],'visual geometry altered')
    require(visual['timeline_time_s']==0 and visual['new_lightnav_inference_count']==0,'visual execution or inference')
    check_scene(visual['scene'],config);artifacts.verify_file(run,visual['selection_input'])
    expected_ids=sorted({r['episode_id'] for r in details['representative_rules'] if r['episode_id'] is not None})
    require([r['episode_id'] for r in visual['representatives']]==expected_ids,'wrong representative views')
    images=[]
    for item in visual['representatives']:
        episode_id=item['episode_id'];location=run/'episodes'/episode_id
        row=next(r for r in details['conditions'] if r['episode_id']==episode_id and r['tau_s']==1.)
        require(item['row']==row,'rendered metric row differs')
        metadata=artifacts.read_json(location/'metadata.json')
        for key in ('R_obs','agent_pose_before','agent_pose_after'): same_array(item[key],metadata['R1'],key,1e-7)
        for seq,key in enumerate(('OLD_world_drawn','FRESH_world_drawn')):
            same_array(item[key],np.load(location/f'derived/chunk_{seq:03d}_world.npy',allow_pickle=False),key,1e-12)
        expected_names=[f'evidence/representative_{episode_id}_{name}.png' for name in ('overview','detail')]
        require([r['path'] for r in item['images']]==expected_names,'representative overview/detail missing')
        for image in item['images']: artifacts.verify_file(run,image);images.append(image['path'])
    validate_episode_points(run,details)
    images += ['evidence/'+name for name in PLOT_NAMES]+['evidence/episode_metrics_by_id.png']
    review=artifacts.read_json(run/'visual_review.json')
    require(review['reviewed'] is True,'actual images not reviewed')
    for flag in REVIEW_FLAGS: require(review['checks'][flag] is True,'visual review failed: '+flag)
    for relative in images:
        require(review['image_sha256'][relative]==sha256_file(run/relative),'reviewed image changed')
        check_evidence_png(run/relative)


def validate_run(run):
    missing=[];failures=[];details={}
    required=['manifest_freeze.json','capture_batch.json','inference_batch.json','analysis_manifest.json',
        'visualization.json','visual_review.json','external_final_audit.json','server_shutdown.json','episode_points_manifest.json']
    missing=[p for p in required if not (run/p).is_file()]
    try:
        if (run/'analysis_manifest.json').is_file(): details=validate_analysis(run)
        if not missing:
            validate_visual(run,details)
            audit=artifacts.read_json(run/'external_final_audit.json')
            require(audit['source_unchanged'] is True and audit['checkpoint_unchanged'] is True,'upstream changed')
            require(audit['manifest_sha256']==details['manifest_sha256'],'upstream audit belongs to another bank')
            artifacts.verify_file(run,audit['server_process'])
            config,_,_=artifacts.load_frozen(run)
            server=artifacts.read_json(run/'server_process.json')
            source=audit['source_after']
            require(source['clean'] is True and source['status_porcelain']==''
                and source['git_sha']==config['lightnav']['expected_git_sha']==server['lightnav_git_sha'],'final source audit mismatch')
            expected=[{k:v for k,v in item.items() if k!='size'} for item in server['checkpoint_files']]
            require(audit['checkpoint_after']['files']==expected,'final checkpoint audit differs from server startup')
            check_hash(run,server['config'],'config_snapshot.yaml')
            shutdown=artifacts.read_json(run/'server_shutdown.json')
            require(shutdown['process_id']==server['process_id'],'shutdown process identity mismatch')
            require(host_ns(artifacts.read_json(run/'inference_batch.json')['completed_time'])<=host_ns(shutdown['request_event'])
                <=host_ns(shutdown['exit_observed_event'])<=host_ns(audit['time']),'shutdown/final audit order mismatch')
    except FileNotFoundError as error: missing.append(str(error.filename))
    except (ValueError,KeyError,TypeError,IndexError,OSError,AttributeError,RuntimeError,SyntaxError) as error: failures.append(f'{type(error).__name__}: {error}')
    return {'schema_version':1,'run_directory':str(run),'status':FAILED if failures else NOT_VALIDATED if missing else VALIDATED,
        'missing_artifacts':missing,'failures':failures,**details}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory',type=Path);parser.add_argument('--write',action='store_true')
    args=parser.parse_args();run=args.run_directory.resolve();result=validate_run(run)
    if args.write: save_json_exclusive(run/'validation.json',result)
    print(json.dumps(result,indent=2));raise SystemExit(0 if result['status']==VALIDATED else 1)

#!/usr/bin/env python3
"""Fail-closed audit of fixed cases, actual captures, final sessions and geometry."""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image
import robotless_old_consistent_artifacts as a
from reconciliation.robotless_old_consistent_observation import CASE_IDS, check_old_reproduction
from reconciliation.robotless_single_chunk import observation_to_world, save_json_exclusive
from reconciliation.robotless_successive_chunks import validate_successive_observation_metadata, round_trip_latency_ms
from validate_robotless_handoff_screening import require, check_scene, check_csv, check_evidence_png, host_ns
from validate_robotless_single_chunk import check_camera, check_external_provenance, check_hash, same_array
from validate_robotless_successive_chunks import check_protocol

VALIDATED='ROBOTLESS_OLD_CONSISTENT_OBSERVATION_PILOT_VALIDATED'
FAILED='ROBOTLESS_OLD_CONSISTENT_OBSERVATION_PILOT_FAILED'
NOT_VALIDATED='ROBOTLESS_OLD_CONSISTENT_OBSERVATION_PILOT_RUNTIME_NOT_VALIDATED'
REVIEW_FLAGS=('all_five_new_RGB1','all_valid_overviews','R0_fixed_R1_R1_old','raw_OLD_new_FRESH',
    'boundary_projection_tangents','no_scaling_magnification_scene_hiding','no_robot')


def check_sources(run, hashes):
    for relative, digest in hashes.items():
        current=a.ROOT/relative
        if current.is_file() and a.sha256_file(current)==digest: continue
        historical=run/'processing_source_history'/f'{digest}.py'
        require(historical.is_file() and a.sha256_file(historical)==digest, 'processing source changed without preserved executed snapshot: '+relative)


def check_capture(run, episode, config, source, mh, freeze_ns):
    location=run/'episodes'/episode['episode_id']; source_ep=source/'episodes'/episode['episode_id']
    status=a.read_json(location/'capture_status.json')
    require(status['episode_id']==episode['episode_id'] and status['manifest_sha256']==mh,'capture case/manifest changed')
    if episode['plan']['status']=='OLD_ARC_INSUFFICIENT':
        require(status['status']=='OLD_ARC_INSUFFICIENT' and not status['capture_valid'] and bool(status['reason']),'insufficient arc omitted')
        require(not (location/'raw/observation_001.jpg').exists(),'insufficient path used for capture')
        return None
    if not status['capture_valid']:
        require(status['status']=='CAPTURE_INVALID' and bool(status['reason']),'unexplained capture failure')
        return None
    meta=a.read_json(location/'metadata.json'); epconfig=a.load_config(location/'config_snapshot.yaml')
    validate_successive_observation_metadata(meta); check_scene(meta['scene'],config)
    require(meta['episode_id']==episode['episode_id'] and meta['manifest_sha256']==mh,'wrong capture metadata')
    require(meta['instruction']==epconfig['instruction']==episode['instruction'],'instruction changed')
    require(epconfig['camera']==config['camera'] and epconfig['scene']==config['scene'],'scene/camera configuration changed')
    require(meta['R0']==episode['plan']['R0'] and meta['planned_R1_old']==episode['plan']['R1_old'],'planned poses changed')
    same_array(meta['R1'],episode['plan']['R1_old'],'actual R1_old pose',config['pose_readback_atol'])
    require(meta['new_rgb_capture_count']==1 and meta['source_RGB0_observation_metadata_reused_unchanged'] is True,'RGB0 recaptured')
    require(meta['observations'][0]==a.read_json(source_ep/'metadata.json')['observations'][0],'source observation metadata altered')
    require((location/'raw/observation_000.jpg').read_bytes()==(source_ep/'raw/observation_000.jpg').read_bytes(),'source RGB0 bytes altered')
    require(meta['execution_time'] is None and not meta['motion_during_capture'] and not meta['inference_concurrent_with_capture'],'unexpected execution/overlap')
    check_hash(location,meta['config'],'config_snapshot.yaml')
    cv=a.read_json(location/'capture_validation.json');a.verify_file(location,cv['metadata'])
    require(cv['source_RGB0_recaptured'] is False and cv['pose_readback_atol']==config['pose_readback_atol'],'capture convention changed')
    same_array(cv['readback'],meta['R1'],'readback record',0)
    same_array(cv['pose_error'],np.array(meta['R1'])-episode['plan']['R1_old'],'pose error',0)
    times=[freeze_ns,host_ns(meta['pose_assignment']['started_time']),host_ns(meta['pose_assignment']['assigned_time']),
        host_ns(meta['observations'][1]['capture_started_time']),host_ns(meta['observations'][1]['observation_time']),host_ns(status['completed_time'])]
    require(times==sorted(times),'new capture precedes manifest/assignment or reverses time')
    for seq,obs in enumerate(meta['observations']):
        check_hash(location,obs['rgb'],f'raw/observation_{seq:03d}.jpg')
        with Image.open(location/obs['rgb']['path']) as im:
            require(im.format=='JPEG' and im.mode=='RGB' and list(im.size)==obs['rgb']['resolution_width_height'],'invalid RGB')
            im.verify()
        with Image.open(location/obs['rgb']['path']) as im: require(float(np.asarray(im).std())>1,'blank RGB')
        checks={'camera_basis_checks':obs['camera']['basis_checks']}
        check_camera({**meta,**obs},epconfig,cv['new_RGB1'] if seq else checks,checks)
        pose=obs['agent_pose_world'];c,s=np.cos(pose[2]),np.sin(pose[2]);t=np.eye(4)
        t[:2,:2]=[[c,-s],[s,c]];t[:3,3]=[*pose[:2],config['agent']['z_m']]
        same_array(obs['camera']['T_world_agent'],t,'own observation camera pose',1e-12)
    return meta


def check_inference(run, episode, metadata, config, source, mh, started_ns):
    location=run/'episodes'/episode['episode_id'];status=a.read_json(location/'inference_status.json')
    require(status['episode_id']==episode['episode_id'] and status['manifest_sha256']==mh,'inference case changed')
    require(host_ns(status['started_time'])>=started_ns,'episode inference precedes batch')
    if metadata is None:
        require(status['inference_attempted'] is False and status['status'] in ('CAPTURE_INVALID','OLD_ARC_INSUFFICIENT'),'invalid capture used')
        return None
    require(status['inference_attempted'] is True,'valid capture skipped')
    if not (location/'inference_metadata.json').exists():
        require(status['status']=='INFERENCE_FAILED' and bool(status['reason']),'inference result missing')
        return None
    inference=a.read_json(location/'inference_metadata.json');epconfig=a.load_config(location/'config_snapshot.yaml')
    require(inference['instruction']==episode['instruction'] and inference['execution_time'] is None and not inference['agent_motion_during_inference'],'inference instruction/execution')
    require(inference['transform']['raw_arrays_unchanged'] and not inference['transform']['correction_applied'] and inference['transform']['waypoint_time_base'] is None,'raw/time convention')
    for key,relative in [('capture_metadata','metadata.json'),('protocol','raw/lightnav_protocol.jsonl')]:check_hash(location,inference['raw_inputs'][key],relative)
    check_hash(location,inference['config'],'config_snapshot.yaml');check_hash(location,inference['server_process'],'server_process.json')
    require((location/'server_process.json').read_bytes()==(run/'server_process.json').read_bytes(),'server provenance changed')
    locals_=[]
    for seq,chunk in enumerate(inference['chunks']):
        local=np.load(location/f'raw/chunk_{seq:03d}.npy',allow_pickle=False)
        world=np.load(location/f'derived/chunk_{seq:03d}_world.npy',allow_pickle=False)
        require(np.array_equal(world,observation_to_world(metadata[f'R{seq}'],local)),'FRESH/OLD own observation anchor changed')
        require(chunk['raw_shape']==list(local.shape) and chunk['raw_dtype']==str(local.dtype),'raw shape/dtype provenance')
        require(chunk['agent_pose_world_at_observation']==metadata[f'R{seq}'] and chunk['observation_time']==metadata['observations'][seq]['observation_time'],'chunk observation changed')
        require(chunk['response_actions_step']==seq+1,'history must advance 1 then 2')
        require(chunk['round_trip_latency_ms']==round_trip_latency_ms(chunk['request_send_time'],chunk['response_receive_time']),'RTT convention')
        for entry in chunk['raw_inputs'].values(): check_hash(location,entry)
        check_hash(location,chunk['world_waypoints']);locals_.append(local)
    check_protocol(location,metadata,inference,locals_)
    check_external_provenance(metadata,epconfig,inference,a.read_json(run/'server_process.json'))
    session=inference['session']
    require(session['reconnect_count']==session['retry_count']==0 and session['instruction_constant'],'session retried/reconnected')
    reproduced=check_old_reproduction(np.load(source/'episodes'/episode['episode_id']/'raw/chunk_000.npy',allow_pickle=False),locals_[0])
    require(status['reproduction']==reproduced,'OLD exact check differs')
    expected='PLANNING_OLD_MISMATCH' if reproduced['status']=='PLANNING_OLD_MISMATCH' else 'MODEL_STOP' if any(c['stop'] for c in inference['chunks']) else 'VALID_PAIR'
    require(status['status']==expected,'invalid OLD/stop included or valid pair excluded')
    check_sources(run,inference['processing_source_sha256'])
    return session['connection_id']


def validate_complete(run):
    config,manifest,record,mh=a.load_frozen(run);source=Path(record['source_run'])
    require(a.validate_screening(source)==record['source_validation'],'source validator result changed')
    require(a.validate_selection(Path(record['selection_run']))==record['selection_validation'],'selection validator result changed')
    freeze_ns=host_ns(manifest['frozen_time']); capture=a.read_json(run/'capture_batch.json');batch=a.read_json(run/'inference_batch.json')
    check_sources(run,manifest['processing_source_sha256'])
    for name in ('capture_started.json','inference_started.json'):
        start=a.read_json(run/name);require(start['manifest_sha256']==mh and host_ns(start['time'])>freeze_ns,'phase before freeze')
        check_sources(run,start['processing_source_sha256'])
    require(capture['manifest_sha256']==batch['manifest_sha256']==mh and capture['scene_load_count']==1,'batch manifest/scene count')
    require(capture['new_RGB0_capture_count']==0 and capture['lightnav_inference_count']==0,'RGB0 recaptured/inference overlapped')
    require(host_ns(capture['completed_time'])<host_ns(batch['started_time']) and batch['capture_inference_overlap'] is False,'capture/inference ordering')
    require({p.name for p in (run/'episodes').iterdir()}==set(CASE_IDS),'case added/deleted/replaced')
    results=[];connections=[]; statuses=[];captures=[]
    for episode in manifest['episodes']:
        location=run/'episodes'/episode['episode_id']
        metadata=check_capture(run,episode,config,source,mh,freeze_ns)
        connection=check_inference(run,episode,metadata,config,source,mh,host_ns(batch['started_time']))
        if connection: connections.append(connection)
        status=a.read_json(location/'inference_status.json');statuses.append(status);captures.append(a.read_json(location/'capture_status.json'))
        expected=a.derive_case(run,episode);require(a.read_json(location/'derived/metrics.json')==expected,'primary metric/provenance changed')
        if expected['status']!='VALID_PAIR': require(expected['metrics'] is None and bool(expected['reason']),'invalid case included')
        results.append(expected)
    require(len(connections)==len(set(connections)),'episode sessions reused')
    require(batch['statuses']==statuses and capture['statuses']==captures,'batch statuses differ')
    require(capture['new_RGB1_capture_count']==sum(c['capture_valid'] for c in captures),'RGB1 count differs')
    require(batch['independent_session_count']==sum(s['inference_attempted'] for s in statuses),'session count differs')
    summary=a.read_json(run/'aggregate/summary.json');valid=[r for r in results if r['status']=='VALID_PAIR']
    require(summary['case_results']==results and summary['case_count']==5 and summary['valid_count']==len(valid) and summary['invalid_count']==5-len(valid),'aggregate count/results')
    require(summary['primary_tau_s']==0 and summary['manifest_sha256']==mh,'aggregate primary condition')
    check_sources(run,summary['processing_source_sha256'])
    rows=[]
    for result in results:
        row={k:v for k,v in result.items() if k not in ('metrics','input_files')};row.update(result['metrics'] or dict.fromkeys(a.METRICS));rows.append(row)
    keys=list(dict.fromkeys(key for row in rows for key in row));check_csv(run/'aggregate/case_table.csv',[{key:row.get(key) for key in keys} for row in rows])
    visual=a.read_json(run/'visualization.json');review=a.read_json(run/'visual_review.json');check_scene(visual['scene'],config)
    require(visual['manifest_sha256']==mh and visual['scene_load_count']==1 and visual['timeline_time_s']==0,'visual scene/time/manifest')
    require(visual['geometry_scaling']==visual['angular_magnification']==1 and visual['scene_visibility_modifications']==[],'visual distortion')
    require(visual['actual_execution_time'] is None and visual['new_lightnav_inference_count']==0,'visual execution/inference')
    a.verify_file(run,visual['summary']);check_sources(run,visual['processing_source_sha256'])
    require([r['episode_id'] for r in visual['records']]==[r['episode_id'] for r in valid],'valid overview set differs')
    require(review['reviewed'] is True and all(review['checks'][key] is True for key in REVIEW_FLAGS),'visual review incomplete')
    for v,r in zip(visual['records'],valid,strict=True):
        require(v['result']==r,'displayed metric changed')
        for seq,key in enumerate(('OLD_world_drawn','FRESH_world_drawn')):
            require(np.array_equal(v[key],np.load(run/'episodes'/r['episode_id']/f'derived/chunk_{seq:03d}_world.npy',allow_pickle=False)),'displayed raw world path changed')
        same_array(v['agent_pose_before'],r['plan']['R1_old'],'visual agent',1e-7)
        require(v['agent_pose_before']==v['agent_pose_after'],'visual agent moved')
        a.verify_file(run,v['image']);check_evidence_png(run/v['image']['path'])
        require(review['image_sha256'][v['image']['path']]==v['image']['sha256'],'review image changed')
    for episode in manifest['episodes']:
        rgb=f'episodes/{episode["episode_id"]}/raw/observation_001.jpg'
        if (run/rgb).exists(): require(review['image_sha256'][rgb]==a.sha256_file(run/rgb),'review RGB changed')
    return {'manifest_sha256':mh,'source_verified_file_count':len(record['source_files']),
        'selection_verified_file_count':len(record['selection_files']),'case_count':5,'valid_count':len(valid),
        'invalid_count':5-len(valid),'unique_final_session_count':len(connections),'statuses':summary['statuses'],
        'primary_tau_s':0.,'all_cases':results}


def validate_run(run):
    required=['source.json','manifest_frozen.json','manifest_freeze.json','capture_batch.json','inference_batch.json',
        'aggregate/summary.json','aggregate/case_table.csv','visualization.json','visual_review.json']
    missing=[p for p in required if not (run/p).is_file()];failures=[];details={}
    try:
        if (run/'manifest_frozen.json').exists(): a.load_frozen(run)
        if not missing: details=validate_complete(run)
    except FileNotFoundError as error: missing.append(str(error.filename))
    except (ValueError,KeyError,TypeError,IndexError,OSError,RuntimeError,SyntaxError) as error: failures.append(f'{type(error).__name__}: {error}')
    status=FAILED if failures or details.get('invalid_count',0) else NOT_VALIDATED if missing else VALIDATED
    return {'schema_version':1,'run_directory':str(run),'status':status,'missing_artifacts':missing,'failures':failures,**details}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run_directory',type=Path);parser.add_argument('--write',action='store_true')
    args=parser.parse_args();run=args.run_directory.resolve();result=validate_run(run)
    if args.write:save_json_exclusive(run/'validation.json',result)
    print(json.dumps({k:v for k,v in result.items() if k!='all_cases'},indent=2))
    raise SystemExit(0 if result['status']==VALIDATED else 1)

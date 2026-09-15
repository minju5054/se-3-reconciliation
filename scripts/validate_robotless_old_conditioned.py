#!/usr/bin/env python3
"""Fail-closed source, paired metric, availability and actual viewport audit."""
import argparse
import json
from pathlib import Path

import numpy as np
import robotless_old_conditioned_artifacts as a
from characterize_robotless_old_conditioned import PLOTS
from reconciliation.robotless_old_conditioned_handoff import summarize, select_representatives
from reconciliation.robotless_single_chunk import save_json_exclusive, validate_observation_time
from validate_robotless_handoff_screening import check_csv, check_scene, check_evidence_png, require

VALIDATED='ROBOTLESS_OLD_CONDITIONED_HANDOFF_VALIDATED'
FAILED='ROBOTLESS_OLD_CONDITIONED_HANDOFF_FAILED'
NOT_VALIDATED='ROBOTLESS_OLD_CONDITIONED_HANDOFF_RUNTIME_NOT_VALIDATED'
REVIEW_FLAGS=('raw_OLD_FRESH','aligned_continuation','observation_and_old_projection',
    'straight_and_old_boundaries','fresh_projection_and_tangents','all_representatives','all_plots','no_exaggeration_or_scene_hiding','no_robot')


def validate_analysis(run):
    config,record,source,manifest=a.load_inputs(run)
    require(a.validate_screening(source)==record['source_validation'],'source validator result changed')
    require(record['new_lightnav_inference_count']==0 and record['actual_execution_time'] is None and record['new_inference_readiness_time'] is None,'unexpected inference/execution')
    validate_observation_time(record['created_time'])
    expected=a.derive(config,source,manifest)
    metrics=a.read_json(run/'derived/episode_metrics.json')
    require(metrics['episodes']==expected,'derived geometry/paired rows changed')
    require(metrics['source_json_sha256']==a.sha256_file(run/'source.json') and metrics['config_sha256']==a.sha256_file(run/'config_snapshot.yaml'),'metric provenance mismatch')
    require(metrics['processing_source_sha256']==a.processing_sources(),'processing code differs from analysis snapshot')
    validate_observation_time(metrics['created_time'])
    require(metrics['created_time']['monotonic_ns']>=record['created_time']['monotonic_ns'],'analysis clock precedes source snapshot')
    rows=[r for e in expected for r in e['conditions']]
    require(len(expected)==30 and len(rows)==120,'all 30 episodes and 120 conditions required')
    for episode in expected:
        require(episode['conditions'][0]['B_old']==episode['reference']['R_obs'],'zero-delay anchor snapped')
        require(episode['actual_execution_time'] is None and episode['new_inference_readiness_time'] is None,'episode timing convention')
        for item in episode['input_files']:a.verify_file(source,item)
        for row in episode['conditions']:
            exhausted=row['requested_delta_s_m']>row['available_remaining_m']
            require((row['status']=='OLD_CONTINUATION_EXHAUSTED')==exhausted,'exhaustion classification/clamp error')
            require((row['B_old'] is None)==exhausted,'unavailable boundary fabricated')
    summary=summarize(expected)
    require(a.read_json(run/'derived/statistics.json')==summary['statistics'],'statistics changed')
    require(a.read_json(run/'derived/summary.json')=={k:v for k,v in summary.items() if k not in ('statistics','unique_pairs')},'counts changed')
    require(a.read_json(run/'derived/unique_pairs.json')['groups']==summary['unique_pairs'],'pair medians changed')
    selection=a.read_json(run/'derived/representatives.json')
    require(selection=={'rules':select_representatives(expected),'selection_scope':'visualization only'},'representatives changed')
    check_csv(run/'derived/episode_metrics.csv',rows)
    check_csv(run/'derived/unique_pairs.csv',summary['unique_pairs'])
    check_csv(run/'derived/anchor_diagnostics.csv',[{'episode_id':e['episode_id'],'category':e['category'],**e['reference']} for e in expected])
    check_csv(run/'derived/critical_cases.csv',[{**r,'d_obs_to_old_m':e['reference']['d_obs_to_old_m']} for e in expected for r in e['conditions'] if r['tau_s']==1 and r['episode_id'] in ('episode_006','episode_016','episode_027')])
    analysis=a.read_json(run/'analysis_manifest.json')
    require(analysis['new_lightnav_inference_count']==0 and analysis['fresh_reanchored'] is False,'analysis inference/reanchor')
    for file in analysis['derived_files']+[analysis['plot_manifest'],analysis['source']]:a.verify_file(run,file)
    require({f['path'] for f in analysis['derived_files']}=={str(p.relative_to(run)) for p in (run/'derived').iterdir() if p.is_file()},'derived inventory changed')
    plots=a.read_json(run/'plot_manifest.json')
    for key in ('metrics_input','configuration'):a.verify_file(run,plots[key])
    require([p['path'] for p in plots['plots']]==['evidence/'+n for n in PLOTS],'required plot set')
    metric_pairs=[('e_perp_straight_m','e_perp_old_m'),('abs_e_dir_straight_deg','abs_e_dir_old_deg'),
        ('abs_e_yaw_straight_deg','abs_e_yaw_old_deg'),('abs_e_dir_old_deg','abs_e_dir_window_deg')]
    for index,plot in enumerate(plots['plots']):
        a.verify_file(run,plot)
        if index<4:
            points=[{'episode_id':r['episode_id'],'metric':key,'value':r[key],'display_x':i+shift} for i,r in enumerate([r for r in rows if r['tau_s']==1]) for key,shift in zip(metric_pairs[index],[-.16,.16])]
        else:points=[{'episode_id':e['episode_id'],'d_obs_to_old_m':e['reference']['d_obs_to_old_m']} for e in expected]
        require(plot['points']==points,'plot dropped or changed raw episode values')
    return {'source_run':str(source),'source_manifest_sha256':record['source_manifest_sha256'],
        'source_verified_file_count':len(record['files']),'episode_count':30,'condition_count':120,
        'unique_pair_count':summary['unique_pair_count'],'availability_by_tau':summary['availability_by_tau'],
        'statistics':summary['statistics'],'representatives':selection['rules'],'episodes':expected}


def validate_visual(run,details):
    config,_,source,_=a.load_inputs(run)
    visual=a.read_json(run/'visualization.json')
    check_scene(visual['scene'],a.load_config(source/'config_snapshot.yaml'))
    require(visual['scene_load_count']==1 and visual['timeline_time_s']==0,'scene/timeline')
    require(visual['geometry_scaling']==visual['angular_magnification']==1 and visual['scene_visibility_modifications']==[],'visual distortion')
    require(visual['new_lightnav_inference_count']==0 and visual['actual_execution_time'] is None and visual['fresh_reanchored'] is False,'visual execution/reanchor')
    z=config['visualization']['path_z_m']
    require(visual['point_display_heights_m']=={'R_obs':z+.02,'Q_old_obs':z+.01,
        'B_straight':config['visualization']['fresh_tangent_z_m']+.05,'B_old':z+.025,'Q_fresh':z+.08},'marker drawing layers changed')
    for name,sha in visual['processing_source_sha256'].items():
        require(a.sha256_file(a.ROOT/name)==sha,'visual processing source changed')
    for key in ('source','metrics','selection','configuration'):a.verify_file(run,visual[key])
    ids=sorted({r['episode_id'] for r in details['representatives'] if r['episode_id'] is not None})
    require([e['episode_id'] for e in visual['representatives']]==ids,'wrong representative IDs')
    images=[]
    for item in visual['representatives']:
        episode_id=item['episode_id'];expected=next(e for e in details['episodes'] if e['episode_id']==episode_id)
        require(item['episode_geometry']==expected,'rendered derived geometry changed')
        for key in ('agent_pose_before','agent_pose_after'):
            require(np.allclose(item[key],expected['reference']['R_obs'],rtol=0,atol=1e-7),'agent moved')
        for seq,key in enumerate(('OLD_world_drawn','FRESH_world_drawn')):
            require(np.array_equal(item[key],np.load(source/'episodes'/episode_id/f'derived/chunk_{seq:03d}_world.npy',allow_pickle=False)),'drawn source path changed')
        names=[f'evidence/representative_{episode_id}_{kind}.png' for kind in ('overview','detail')]
        require([p['path'] for p in item['images']]==names,'missing representative view')
        for p in item['images']:a.verify_file(run,p)
        images.extend(names)
    review=a.read_json(run/'visual_review.json');require(review['reviewed'] is True,'images not reviewed')
    for flag in REVIEW_FLAGS:require(review['checks'][flag] is True,'visual review check: '+flag)
    images+=['evidence/'+p for p in PLOTS]
    for relative in images:
        require(review['image_sha256'][relative]==a.sha256_file(run/relative),'reviewed image changed')
        check_evidence_png(run/relative)


def validate_run(run):
    required=['source.json','config_snapshot.yaml','analysis_manifest.json','visualization.json','visual_review.json']
    missing=[name for name in required if not (run/name).is_file()];failures=[];details={}
    try:
        if (run/'analysis_manifest.json').is_file():details=validate_analysis(run)
        if not missing:validate_visual(run,details)
    except FileNotFoundError as error:missing.append(str(error.filename))
    except (ValueError,KeyError,TypeError,IndexError,OSError,RuntimeError,SyntaxError) as error:failures.append(f'{type(error).__name__}: {error}')
    return {'schema_version':1,'run_directory':str(run),'status':FAILED if failures else NOT_VALIDATED if missing else VALIDATED,
        'missing_artifacts':missing,'failures':failures,**details}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run_directory',type=Path);parser.add_argument('--write',action='store_true')
    args=parser.parse_args();run=args.run_directory.resolve();result=validate_run(run)
    if args.write:save_json_exclusive(run/'validation.json',result)
    print(json.dumps(result,indent=2));raise SystemExit(0 if result['status']==VALIDATED else 1)

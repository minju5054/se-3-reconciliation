"""Synthetic source/aggregate audits, explicitly not actual runtime evidence."""
import json
from pathlib import Path
import shutil
import sys

import numpy as np
import pytest
import yaml
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import robotless_old_conditioned_artifacts as a
import characterize_robotless_old_conditioned as pipeline
import validate_robotless_old_conditioned as validator
from reconciliation.robotless_handoff_screening import transition_metrics
from reconciliation.robotless_single_chunk import make_observation_time, sha256_file


@pytest.fixture(scope='module')
def synthetic_run(tmp_path_factory):
    base=tmp_path_factory.mktemp('synthetic_old_conditioned');source=base/'source';source.mkdir()
    config=yaml.safe_load((ROOT/'configs/robotless_old_conditioned_handoff.yaml').read_text());config['source_run']=str(source)
    path=base/'config.yaml';path.write_text(yaml.safe_dump(config))
    episodes=[{'episode_id':f'episode_{i:03d}','category':'straight'} for i in range(30)]
    (source/'manifest_frozen.json').write_text(json.dumps({'episodes':episodes,'synthetic_only':True}))
    (source/'config_snapshot.yaml').write_text(yaml.safe_dump({'scene':{'asset_relative_path':'synthetic_fixture.usd'}}))
    for i,e in enumerate(episodes):
        location=source/'episodes'/e['episode_id'];(location/'raw').mkdir(parents=True);(location/'derived').mkdir()
        obs=[.2,.1,0.]
        old=np.array([[0.,0.,0.],[.4 if i==0 else 2.,0.,0.]])
        fresh=np.array([[.1,0.,0.],[1.,0.,.1],[2.,0.,.2]])
        for seq,array in enumerate((old,fresh)):
            np.save(location/f'raw/chunk_{seq:03d}.npy',array)
            np.save(location/f'derived/chunk_{seq:03d}_world.npy',array)
        hashes=[sha256_file(location/f'raw/chunk_{seq:03d}.npy') for seq in (0,1)]
        rows=transition_metrics(e,obs,fresh,*hashes)
        (location/'derived/projection_metrics.json').write_text(json.dumps({'conditions':rows}))
        event=make_observation_time(0)
        (location/'metadata.json').write_text(json.dumps({'R1':obs,'observations':[{'observation_time':event}]*2}))
        (location/'inference_metadata.json').write_text(json.dumps({'chunks':[{'response_receive_time':event}]*2}))
    validate=lambda _: {'status':a.SCREENING_VALIDATED,'synthetic_only':True}
    run=base/'analysis'
    original=a.initialize
    try:
        a.initialize=lambda config,run:original(config,run,validate_source_fn=validate)
        pipeline.analyze(run,path)
    finally:a.initialize=original
    return run,source,path


@pytest.fixture
def run(synthetic_run,tmp_path,monkeypatch):
    original,source,path=synthetic_run
    destination=tmp_path/'analysis';shutil.copytree(original,destination)
    monkeypatch.setattr(a,'validate_screening',lambda _: {'status':a.SCREENING_VALIDATED,'synthetic_only':True})
    return destination


def test_synthetic_analysis_preserves_exhausted_row_and_cannot_claim_runtime(run):
    result=validator.validate_run(run)
    assert result['status']==validator.NOT_VALIDATED and not result['failures']
    assert result['episode_count']==30 and result['condition_count']==120
    assert result['availability_by_tau'][-1]['counts']=={'AVAILABLE':29,'OLD_CONTINUATION_EXHAUSTED':1}
    assert result['statistics']['episode_weighted'][-1]['metrics']['e_perp_old_m']['n_unavailable']==1
    assert result['episodes'][0]['conditions'][-1]['B_old'] is None


def test_analysis_never_overwrites_or_writes_inside_source(synthetic_run):
    run,source,path=synthetic_run
    before=a.inventory(source)
    check=lambda _: {'status':a.SCREENING_VALIDATED,'synthetic_only':True}
    with pytest.raises(FileExistsError):a.initialize(path,run,validate_source_fn=check)
    with pytest.raises(ValueError,match='inside'):a.initialize(path,source/'new_output',validate_source_fn=check)
    assert a.inventory(source)==before


def mutate(run,relative,fn):
    path=run/relative;value=a.read_json(path);fn(value);path.write_text(json.dumps(value))


@pytest.mark.parametrize(('relative','fn','reason'),[
    ('derived/episode_metrics.json',lambda v:v['episodes'].pop(),'derived geometry'),
    ('derived/episode_metrics.json',lambda v:v['episodes'][0]['conditions'][0].update(B_old=[0,0,0]),'derived geometry'),
    ('derived/statistics.json',lambda v:v['episode_weighted'][0]['metrics']['e_perp_old_m'].update(median=99),'statistics changed'),
    ('derived/unique_pairs.json',lambda v:v['groups'][0].update(count=999),'pair medians changed'),
    ('derived/representatives.json',lambda v:v['rules'][0].update(episode_id='episode_000'),'representatives changed'),
    ('source.json',lambda v:v.update(new_lightnav_inference_count=1),'unexpected inference'),
])
def test_corrupted_analysis_fails_closed(run,relative,fn,reason):
    mutate(run,relative,fn)
    result=validator.validate_run(run)
    assert result['status']==validator.FAILED and reason in str(result['failures'])


def test_source_hash_change_rejected_without_modifying_source(run):
    mutate(run,'source.json',lambda v:v['files'][0].update(sha256='0'*64))
    with pytest.raises(ValueError,match='source file set or bytes'):a.load_inputs(run)


def test_configuration_change_rejected(run):
    with (run/'config_snapshot.yaml').open('a') as stream:stream.write('\n# synthetic corruption\n')
    with pytest.raises(ValueError,match='input changed'):a.load_inputs(run)


def test_required_csv_preserves_all_rows(run):
    target=run/'derived/episode_metrics.csv';lines=target.read_text().splitlines();target.write_text('\n'.join(lines[:-1])+'\n')
    result=validator.validate_run(run)
    assert result['status']==validator.FAILED and 'CSV row count' in str(result['failures'])


def test_plot_point_change_rejected_even_with_updated_file_hash(run):
    mutate(run,'plot_manifest.json',lambda v:v['plots'][0]['points'].pop())
    mutate(run,'analysis_manifest.json',lambda v:v.update(plot_manifest=a.file_record(run/'plot_manifest.json',run)))
    result=validator.validate_run(run)
    assert result['status']==validator.FAILED and 'raw episode values' in str(result['failures'])


def synthetic_visual(run,details):
    source=Path(details['source_run']);config=a.load_config(run/'config_snapshot.yaml');z=config['visualization']['path_z_m']
    ids=sorted({r['episode_id'] for r in details['representatives'] if r['episode_id']})
    images=[];records=[]
    for episode_id in ids:
        episode=next(e for e in details['episodes'] if e['episode_id']==episode_id);files=[]
        for kind in ('overview','detail'):
            path=run/f'evidence/representative_{episode_id}_{kind}.png'
            Image.fromarray(np.tile(np.arange(128,dtype=np.uint8),(128,1))).save(path)
            files.append(a.file_record(path,run));images.append(path)
        arrays=[np.load(source/'episodes'/episode_id/f'derived/chunk_{seq:03d}_world.npy').tolist() for seq in (0,1)]
        records.append({'episode_id':episode_id,'episode_geometry':episode,'OLD_world_drawn':arrays[0],'FRESH_world_drawn':arrays[1],
            'agent_pose_before':episode['reference']['R_obs'],'agent_pose_after':episode['reference']['R_obs'],'images':files})
    inventory={'no_robot_model':True,'articulation_roots':[],'robot_named_paths':[],'rigid_body_count':0,'physics_scene_count':0,
        'dynamics_advanced':False,'authored_asset_references':['synthetic_fixture.usd']}
    visual={'scene':{'runtime_inventory':inventory,'stage_units_in_meters':1.,'up_axis':'Z'},'scene_load_count':1,
        'timeline_time_s':0,'geometry_scaling':1,'angular_magnification':1,'scene_visibility_modifications':[],
        'new_lightnav_inference_count':0,'actual_execution_time':None,'fresh_reanchored':False,'representatives':records,
        'point_display_heights_m':{'R_obs':z+.02,'Q_old_obs':z+.01,'B_straight':config['visualization']['fresh_tangent_z_m']+.05,'B_old':z+.025,'Q_fresh':z+.08},
        'processing_source_sha256':{}}
    visual.update({key:a.file_record(run/relative,run) for key,relative in [('source','source.json'),('metrics','derived/episode_metrics.json'),('selection','derived/representatives.json'),('configuration','config_snapshot.yaml')]})
    (run/'visualization.json').write_text(json.dumps(visual))
    images += [run/'evidence'/name for name in pipeline.PLOTS]
    (run/'visual_review.json').write_text(json.dumps({'reviewed':True,'synthetic_only':True,
        'checks':dict.fromkeys(validator.REVIEW_FLAGS,True),'image_sha256':{str(p.relative_to(run)):sha256_file(p) for p in images}}))


@pytest.mark.parametrize('corruption',['none','scale','fresh','review','image'])
def test_visual_schema_and_review_guard_with_explicit_synthetic_images(run,corruption):
    details=validator.validate_analysis(run);synthetic_visual(run,details)
    if corruption=='none':validator.validate_visual(run,details);return
    if corruption=='scale':mutate(run,'visualization.json',lambda v:v.update(geometry_scaling=2))
    elif corruption=='fresh':mutate(run,'visualization.json',lambda v:v['representatives'][0]['FRESH_world_drawn'][0].__setitem__(0,99))
    elif corruption=='review':mutate(run,'visual_review.json',lambda v:v['checks'].update(no_robot=False))
    else:
        image=next((run/'evidence').glob('representative_*.png'));image.write_bytes(b'corrupted synthetic image')
    with pytest.raises(ValueError):validator.validate_visual(run,details)

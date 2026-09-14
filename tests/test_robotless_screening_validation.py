"""Synthetic aggregate corruption checks; never runtime experiment evidence."""
import json
from pathlib import Path
import shutil
import sys

import pytest
import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import robotless_screening_artifacts as artifacts
import screen_robotless_handoffs as pipeline
import validate_robotless_handoff_screening as validator
from plot_robotless_screening_episode_points import make_episode_panels
from test_robotless_screening_artifacts import frozen, prepare_synthetic_captures, synthetic_inference, batch


@pytest.fixture(scope='module')
def synthetic_aggregate(tmp_path_factory):
    run,_,config=frozen.__wrapped__(tmp_path_factory.mktemp('synthetic_screening'))
    prepare_synthetic_captures(run,config)
    _,_,digest=artifacts.load_frozen(run)
    (run/'capture_batch.json').write_text(json.dumps({'episode_count':30,'scene_load_count':1,
        'manifest_sha256':digest,'completed_time':batch.host_event()}))
    batch.collect(run,inference_fn=synthetic_inference)
    pipeline.analyze(run)
    make_episode_panels(run)
    return run


@pytest.fixture
def aggregate(synthetic_aggregate,tmp_path,monkeypatch):
    run=tmp_path/'synthetic';shutil.copytree(synthetic_aggregate,run)
    # Isolate aggregation from the already separately tested capture/protocol layer.
    def checked_episode(root,episode,digest,freeze_ns):
        location=root/'episodes'/episode['episode_id']
        return (artifacts.read_json(location/'validation.json'),
            artifacts.read_json(location/'derived/projection_metrics.json')['conditions'],episode['episode_id'])
    monkeypatch.setattr(validator,'check_episode',checked_episode)
    return run


def test_complete_aggregate_and_all_points_are_still_not_runtime_evidence(aggregate):
    details=validator.validate_analysis(aggregate)
    assert len(details['conditions'])==120 and details['validated_session_count']==30
    assert details['diversity']['unique_ordered_pair_count']==1
    validator.validate_episode_points(aggregate,details)
    result=validator.validate_run(aggregate)
    assert result['status']==validator.NOT_VALIDATED and not result['failures']
    assert 'visualization.json' in result['missing_artifacts']
    with pytest.raises(FileExistsError): pipeline.analyze(aggregate)


def mutate_json(run,path,mutate):
    target=run/path;value=artifacts.read_json(target);mutate(value)
    target.write_text(json.dumps(value))


@pytest.mark.parametrize(('path','mutate','reason'),[
    ('aggregate/transitions.json',lambda x:x['conditions'].pop(),'aggregate dropped'),
    ('aggregate/diversity.json',lambda x:x.update(unique_ordered_pair_count=30),'diversity differs'),
    ('aggregate/statistics.json',lambda x:x['episode_weighted'][0]['metrics']['e_perp_m'].update(median=99),'statistics differ'),
    ('aggregate/unique_pairs.json',lambda x:x['groups'][0].update(count=1),'duplicate-aware medians differ'),
    ('aggregate/representatives.json',lambda x:x['rules'][0].update(episode_id='episode_029'),'representative selection changed'),
    ('inference_batch_started.json',lambda x:x['time'].update(monotonic_ns=1),'bank freeze/inference order'),
])
def test_altered_results_fail_closed(aggregate,path,mutate,reason):
    mutate_json(aggregate,path,mutate)
    result=validator.validate_run(aggregate)
    assert result['status']==validator.FAILED
    assert any(reason in failure for failure in result['failures'])


def test_changed_csv_cannot_disagree_with_json(aggregate):
    target=aggregate/'aggregate/transitions.csv'
    lines=target.read_text().splitlines();target.write_text('\n'.join(lines[:-1])+'\n')
    result=validator.validate_run(aggregate)
    assert result['status']==validator.FAILED and 'CSV row count' in str(result['failures'])


def test_session_reuse_across_episodes_rejected(aggregate,monkeypatch):
    original=validator.check_episode
    def reused(*args):
        status,conditions,_=original(*args)
        return status,conditions,'reused_connection'
    monkeypatch.setattr(validator,'check_episode',reused)
    assert 'session identity reused' in str(validator.validate_run(aggregate)['failures'])


@pytest.mark.parametrize('change',['drop','value'])
def test_supplementary_panels_preserve_every_raw_value(aggregate,change):
    details=validator.validate_analysis(aggregate)
    def modify(value):
        if change=='drop': value['plotted_points'].pop()
        else: value['plotted_points'][0]['value']+=1
    mutate_json(aggregate,'episode_points_manifest.json',modify)
    with pytest.raises(ValueError,match='dropped or changed'):
        validator.validate_episode_points(aggregate,details)


def test_png_is_verified_before_pixels_are_decoded(tmp_path):
    path=tmp_path/'synthetic.png'
    Image.fromarray(np.tile(np.arange(128,dtype=np.uint8),(128,1))).save(path)
    validator.check_evidence_png(path)


@pytest.mark.parametrize('kind',['blank','truncated','wrong_format','too_small'])
def test_unusable_evidence_images_rejected(tmp_path,kind):
    path=tmp_path/'synthetic.png'
    Image.fromarray(np.tile(np.arange(128,dtype=np.uint8),(128,1))).save(path)
    if kind=='blank': Image.new('RGB',(128,128),'black').save(path)
    elif kind=='truncated': path.write_bytes(path.read_bytes()[:60])
    elif kind=='wrong_format': Image.new('RGB',(128,128),'black').save(path,format='JPEG')
    else: Image.new('RGB',(16,16),'black').save(path)
    with pytest.raises((ValueError,OSError,SyntaxError)):
        validator.check_evidence_png(path)

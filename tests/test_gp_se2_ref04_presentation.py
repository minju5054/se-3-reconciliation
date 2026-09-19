"""Synthetic-only presentation checks, not additional experimental evidence."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import zipfile

import pytest

ROOT=Path(__file__).resolve().parents[1]
def load(name, path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
p=load('present_gp_se2_ref04',ROOT/'scripts/present_gp_se2_ref04.py')
fixtures=load('ref04_frozen_visual_fixtures',ROOT/'tests/test_gp_se2_ref04_visuals.py')


def fixture(tmp_path, monkeypatch):
    primary=tmp_path/'primary';primary.mkdir()
    audit,context,route,ref,config,env=fixtures.fixture(primary)
    (primary/'source.json').write_text(json.dumps(dict(environment_path=str(primary/'environment'))))
    past=dict(poses_world=[[-.1,0.,0.],context['B_world']],times_relative_to_B_s=[-.1,0.])
    (primary/'actual_past_execution.json').write_text(json.dumps(past))
    manifest=p.frozen.render(primary,audit,context,route,ref,config,env,past=past)
    (primary/'validation.json').write_text(json.dumps(dict(valid=True,authoritative=True,synthetic_only=True)))
    monkeypatch.setattr(p.frozen,'HospitalEnvironment',SimpleNamespace(load=lambda path:env))
    return primary,audit,context,route,ref,config,env,past


def test_shared_external_legends_never_cover_data_axes(tmp_path):
    audit,context,route,ref,config,env=fixtures.fixture(tmp_path)
    data=p.frozen.expected_numeric(audit,context,route,ref,config)
    for name in ('selected_targets_and_predictions','prediction_vs_applied_prefix'):
        fig,rect=p.corrected_figure(name,data[name],env)
        assert len(fig.legends)==1
        assert all(ax.get_legend() is None for ax in fig.axes)
        assert rect[-1] < .962
        p.frozen.plt.close(fig)


def test_heatmap_has_explicit_negative_zero_ticks_and_same_values(tmp_path):
    import numpy as np
    audit,context,route,ref,config,env=fixtures.fixture(tmp_path)
    data=p.frozen.expected_numeric(audit,context,route,ref,config)['predicted_clearance_heatmap']
    before=deepcopy(data)
    fig,_=p.corrected_figure('predicted_clearance_heatmap',data,env)
    assert data==before
    image_axes=[ax for ax in fig.axes if ax.images]
    assert len(image_axes)==4
    for ax in image_axes:
        ticks=ax.images[0].colorbar.get_ticks()
        assert min(ticks)<0 and 0 in ticks
    np.testing.assert_allclose(image_axes[0].images[0].get_array()[0],.06-.0500001)
    assert any('Negative margin' in t.get_text() for t in fig.texts)
    p.frozen.plt.close(fig)


def test_presentation_all_seven_exact_numbers_copies_provenance_and_zip(tmp_path,monkeypatch):
    primary,*_=fixture(tmp_path,monkeypatch)
    output=tmp_path/'presentation'
    originals={str(path.relative_to(primary)):p.digest(path) for path in primary.rglob('*') if path.is_file()}
    result=p.main(primary,output)
    assert result['presentation_validation']['valid']
    assert result['presentation_validation']['ZIP_member_bytes_verified']
    assert result['presentation_entrypoint_wall_s']>0
    assert result['image_count']==7
    for name in p.frozen.PLOT_NAMES:
        left=primary/'plots'/(name+'.png');right=output/'plots'/(name+'.png')
        a,b=p.read(left.with_suffix('.json')),p.read(right.with_suffix('.json'))
        assert a['numeric_data']==b['numeric_data'] and a['source_hashes']==b['source_hashes']
        if name in p.UNCHANGED:
            assert left.read_bytes()==right.read_bytes()
            assert left.with_suffix('.json').read_bytes()==right.with_suffix('.json').read_bytes()
        else:
            assert p.digest(left)!=p.digest(right)
            assert b['primary_validation_sha256']==p.digest(primary/'validation.json')
    with zipfile.ZipFile(output/'review_bundle.zip') as z:
        assert len(z.namelist())==result['file_count']
        assert 'FINDINGS.md' in z.namelist() and 'primary_validation.json' in z.namelist()
        assert 'aggregate/example.csv' in z.namelist()
        assert not any(n.endswith('.npy') for n in z.namelist())
        for name in z.namelist(): assert z.read(name)==(output/name).read_bytes()
    assert originals=={str(path.relative_to(primary)):p.digest(path) for path in primary.rglob('*') if path.is_file()}
    with pytest.raises(FileExistsError):p.main(primary,output)
    side=output/'plots/selected_targets_and_predictions.json';bad=p.read(side)
    bad['numeric_data']['methods'][p.frozen.METHODS[0]][0]['input_pose_world'][0]+=.1
    side.write_text(json.dumps(bad))
    assert not p.validate(primary,output)['valid']


def test_rejects_non_authoritative_primary_before_output(tmp_path,monkeypatch):
    primary,*_=fixture(tmp_path,monkeypatch)
    (primary/'validation.json').write_text(json.dumps(dict(valid=True,authoritative=False)))
    output=tmp_path/'presentation'
    with pytest.raises(ValueError,match='authoritative'):p.main(primary,output)
    assert not output.exists()

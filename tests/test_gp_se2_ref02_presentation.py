"""Rendering-only REF-02 correction guards; no new numerical experiment."""
import importlib.util
from pathlib import Path
import json
import pytest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('ref02_presentation_test',ROOT/'scripts/present_gp_se2_ref02.py')
presentation=importlib.util.module_from_spec(spec);spec.loader.exec_module(presentation)


@pytest.mark.parametrize('count',[1,10,30])
def test_all_rows_fit_below_title_without_dropping_identities(count):
    rows=[dict(derived_row_index=i,original_fractional_row_coordinate=float(i)/3,
               world_xy=[2.,3.],wrapped_yaw=i/100) for i in range(count)]
    spec=presentation.table_spec(rows)
    x,y,w,h=spec['bbox']
    assert x==0 and w==1 and y>=0 and y+h==pytest.approx(.86)
    assert y+h < spec['title_y']-.08
    assert len(spec['cells'])==count
    assert [r[0] for r in spec['cells']]==list(map(str,range(count)))
    # Equal geometry never merges source identities.
    assert len({r[1] for r in spec['cells']})==count


def test_presentation_refuses_existing_output_before_reading_primary(tmp_path):
    output=tmp_path/'existing';output.mkdir();(output/'unchanged').write_text('original')
    with pytest.raises(FileExistsError):presentation.main(tmp_path/'missing-primary',output)
    assert (output/'unchanged').read_text()=='original'


def test_primary_source_manifest_rejects_modified_review_artifact(tmp_path):
    (tmp_path/'plot_manifest.json').write_text(json.dumps(dict(experiment='GP-SE2-REF-02')))
    bundle=tmp_path/'review_bundle';bundle.mkdir();(bundle/'metrics.json').write_text('{}')
    (bundle/'manifest.json').write_text(json.dumps(dict(allowlisted_files=[dict(path='metrics.json',sha256='wrong')])))
    with pytest.raises(ValueError,match='review artifact changed'):
        presentation.source_manifest(tmp_path)


def validator_module():
    spec=importlib.util.spec_from_file_location('ref02_presentation_validator_test',ROOT/'scripts/validate_gp_se2_ref02_presentation.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_independent_presentation_validator_refuses_overwrite(tmp_path):
    original=tmp_path/'independent_validation.json';original.write_text('{"original":true}')
    with pytest.raises(FileExistsError):validator_module().validate(tmp_path)
    assert original.read_text()=='{"original":true}'


def test_missing_presentation_artifacts_are_failure_not_substitution(tmp_path):
    report=validator_module().validate(tmp_path,write_output=False)
    assert not report['valid'] and report['errors']
    assert report['new_controller_solves']==report['new_optimizer_solves']==0
    assert list(tmp_path.iterdir())==[]

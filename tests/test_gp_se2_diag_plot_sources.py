"""Small artifact provenance fixtures, not trajectories or research evidence."""
import hashlib
import json

import numpy as np
import pytest

from reconciliation.gp_se2_diag_plot_sources import (
    build_math_plot_sources, verify_math_plot_sources, write_math_plot_sources,
)


def dump(path, value):
    path.write_text(json.dumps(value))


@pytest.fixture
def bundle(tmp_path):
    source = tmp_path/'implementation_sources'
    source.mkdir()
    code = source/'gp_se2_diag_fixtures.py'
    code.write_text('# unit-test plotting source placeholder\n')
    dump(tmp_path/'source.json', {'source_sha256': {code.name: hashlib.sha256(code.read_bytes()).hexdigest()}})
    q = np.linspace(0, 3, 3001)
    pose = np.column_stack([q, q*.2, q*.1])
    zeros = np.zeros_like(pose)
    independent = q[10:280:10]
    result = dict(query_times_s=q.tolist(), interior_derivative_times_s=independent.tolist(),
                  truth_poses_world=pose.tolist(), gp_poses_world=pose.tolist(),
                  fixture_input=dict(support_poses=pose[::100].tolist(),
                                     config=dict(a_v_max=2., a_w_max=5., equality_tolerance=1e-5)),
                  position_error_m=np.zeros(len(q)).tolist(), yaw_error_rad=np.zeros(len(q)).tolist(),
                  gp_body_twists=zeros.tolist(), gp_body_accelerations=zeros.tolist(),
                  truth_body_accelerations=zeros.tolist(),
                  independent_body_twists=np.zeros((len(independent), 3)).tolist(),
                  independent_body_accelerations=np.zeros((len(independent), 3)).tolist(),
                  summary=dict(lateral_velocity_max_m_s=0.))
    for name in ('S0', 'S1', 'S2', 'S3'):
        directory = tmp_path/name
        directory.mkdir()
        dump(directory/'numeric_results.json', result)
        for filename in ('truth_vs_gp_reconstruction.png', 'lateral_and_acceleration_vs_time.png',
                         'acceleration_reconstruction_error.png'):
            (directory/filename).write_bytes(b'unit-test image-byte placeholder, not rendered evidence')
    directory = tmp_path/'collocation_blind_spot'
    directory.mkdir()
    times = np.linspace(0, .1, 1001)
    body = np.zeros((len(times), 3))
    body[:, 1] = .0002*np.sin(times/.1*2*np.pi)
    index = np.argmax(np.abs(body[:, 1]))
    fractions = np.array([0., .25, .5, .75, 1.])
    result = dict(times_s=times.tolist(), body_twists=body.tolist(),
                  independent_times_s=times[5:-5].tolist(), independent_body_twists=body[5:-5].tolist(),
                  summary=dict(fractions=fractions.tolist(), lateral_at_prescribed_points_m_s=body[::250, 1].tolist(),
                               equality_tolerance_m_s=1e-5,
                               maximum_absolute_lateral_velocity_m_s=float(abs(body[index, 1])),
                               maximum_violation_time_s=float(times[index])))
    dump(directory/'numeric_results.json', result)
    (directory/'lateral_velocity_vs_time.png').write_bytes(b'unit-test image-byte placeholder')
    return tmp_path


def test_build_write_verify_is_exact_and_does_not_invent_visual_review(bundle):
    before = {str(p): p.read_bytes() for p in bundle.rglob('*') if p.is_file()}
    built = build_math_plot_sources(bundle)
    assert built['record_count'] == 13
    assert built['visual_review'] == []
    assert built['visual_findings'].startswith('NOT_EVALUATED')
    result = write_math_plot_sources(bundle)
    assert result['plot_record_count'] == 13
    assert json.loads((bundle/'plot_sources.json').read_text()) == built
    checked = verify_math_plot_sources(bundle)
    assert checked['valid'] and checked['errors'] == []
    assert checked['plot_count'] == 13 and checked['trace_count'] == 64
    assert not checked['wrote_files']
    assert all(__import__('pathlib').Path(path).read_bytes() == content for path, content in before.items())


def test_existing_sidecars_never_overwritten(bundle):
    write_math_plot_sources(bundle)
    before = (bundle/'plot_sources.json').read_bytes()
    with pytest.raises(FileExistsError):
        write_math_plot_sources(bundle)
    assert (bundle/'plot_sources.json').read_bytes() == before


@pytest.mark.parametrize('corruption', ['unit', 'limit', 'trace_hash', 'png', 'source'])
def test_verifier_detects_hash_units_limits_and_source_corruption(bundle, corruption):
    write_math_plot_sources(bundle)
    if corruption in ('unit', 'limit', 'trace_hash'):
        sidecar = bundle/'plot_sources.json'
        data = json.loads(sidecar.read_text())
        if corruption == 'unit': data['records'][0]['traces'][0]['x_units'] = 'cm'
        if corruption == 'limit': data['records'][1]['limits'][0]['upper'] = 1e-3
        if corruption == 'trace_hash': data['records'][0]['traces'][0]['x_float64_le_sha256'] = '0'*64
        dump(sidecar, data)
    elif corruption == 'png':
        (bundle/'S0/truth_vs_gp_reconstruction.png').write_bytes(b'changed bytes')
    else:
        (bundle/'implementation_sources/gp_se2_diag_fixtures.py').write_text('# altered source\n')
    checked = verify_math_plot_sources(bundle)
    assert not checked['valid'] and checked['errors']


def test_independent_derived_error_check_survives_fresh_metadata(bundle):
    numeric = bundle/'S0/numeric_results.json'
    data = json.loads(numeric.read_text())
    data['position_error_m'][5] = .1
    dump(numeric, data)
    write_math_plot_sources(bundle)
    result = verify_math_plot_sources(bundle)
    assert not result['valid']
    assert 'S0 position error trace mismatch' in result['errors']


def test_visual_review_metadata_requires_explicit_evidence(bundle):
    metadata = dict(visual_review=['S0/truth_vs_gp_reconstruction.png'], visual_findings='test supplied review')
    built = build_math_plot_sources(bundle, visual_review=metadata)
    assert built['visual_review'] == metadata['visual_review']
    with pytest.raises(ValueError, match='visual review metadata'):
        build_math_plot_sources(bundle, visual_review={'visual_review': []})


def test_nonfinite_trace_rejected_without_zero_fill(bundle):
    numeric = bundle/'S0/numeric_results.json'
    data = json.loads(numeric.read_text())
    data['gp_body_twists'][5][1] = float('nan')
    dump(numeric, data)
    with pytest.raises(ValueError, match='finite'):
        write_math_plot_sources(bundle)
    assert not (bundle/'plot_sources.json').exists()

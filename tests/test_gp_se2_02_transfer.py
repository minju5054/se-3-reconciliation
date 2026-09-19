"""Transfer protocol and fail-closed bookkeeping; fixtures are not evidence."""
import json
from pathlib import Path

import numpy as np
import pytest

from reconciliation import gp_se2_02_transfer as transfer
from reconciliation.gp_se2_diag02_validation import PROTOCOL, ConstantEnvironmentDerivatives
from reconciliation.gp_se2_diag_fixtures import make_fixture_problem


def test_fixed_case_method_start_scope_and_original_validation_thresholds():
    assert transfer.FIXED_CASE_IDS == (
        'episode_001_repeat_01/handoff_002', 'episode_013_repeat_01/handoff_024',
        'episode_014_repeat_01/handoff_026', 'episode_000_repeat_01/handoff_019')
    assert transfer.METHODS == ('M2_GP_NO_OBSTACLE', 'M3_GP_CONSTRAINED')
    assert transfer.INITIALIZATIONS == ('I0_FRESH', 'I1_DECEL')
    assert transfer.TRANSFER_PROTOCOL['point_count'] == 32
    assert transfer.TRANSFER_PROTOCOL['start_count'] == 16
    for key in ('directional_fd_steps', 'primal_tolerance', 'objective_derivative_tolerance',
                'constraint_derivative_tolerance', 'smooth_direction_acceptance', 'nonsmooth_policy'):
        assert transfer.TRANSFER_PROTOCOL[key] == PROTOCOL[key]
    assert not transfer.TRANSFER_PROTOCOL['full_coordinate_sweep_repeated']


def test_seeds_are_prescribed_original_vectors_and_bitwise_fixed_boundary():
    from reconciliation.gp_se2_diag_initialization import same_curvature_deceleration_seed
    problem, _ = make_fixture_problem('S1')
    seeds = transfer.seed_vectors(problem)
    original = problem.initializations()[0]
    expected = same_curvature_deceleration_seed(problem)
    assert seeds['I0_FRESH'].tobytes() == problem.vector(original['poses'], original['twists']).tobytes()
    assert seeds['I1_DECEL'].tobytes() == problem.vector(expected['poses'], expected['twists']).tobytes()
    for vector in seeds.values():
        poses, twists = problem.unpack(vector)
        assert poses[0].tobytes() == problem.boundary_pose.tobytes()
        assert twists[0].tobytes() == problem.initial_twist.tobytes()
    assert expected['metadata']['original_goal_used_to_fit'] is False


def test_exact_vector_hash_rejects_nonfinite_wrong_shape_and_detects_signed_zero():
    x = np.zeros(150, dtype=np.float64)
    a = transfer.vector_sha256(x)
    x[0] = -0.
    assert transfer.vector_sha256(x) != a
    with pytest.raises(ValueError): transfer.vector_sha256(np.zeros(149))
    x[2] = np.nan
    with pytest.raises(ValueError): transfer.vector_sha256(x)


@pytest.mark.parametrize('statuses,expected', [
    (['VERIFIED', 'VERIFIED'], 'VERIFIED'),
    (['VERIFIED', 'DERIVATIVE_UNSUPPORTED'], 'DERIVATIVE_UNSUPPORTED'),
    (['DERIVATIVE_UNSUPPORTED', 'VERIFIED'], 'DERIVATIVE_UNSUPPORTED'),
    (['DERIVATIVE_UNSUPPORTED', 'DERIVATIVE_VALIDATION_FAILED'], 'DERIVATIVE_VALIDATION_FAILED'),
    (['DERIVATIVE_VALIDATION_FAILED', 'VERIFIED'], 'DERIVATIVE_VALIDATION_FAILED'),
])
def test_each_start_requires_supported_verified_base_and_perturbation(statuses, expected):
    assert transfer.classify_start([dict(status=s) for s in statuses]) == expected


def test_gate_scope_preserved_and_missing_probe_cannot_authorize():
    assert transfer.classify_start([], gates=[{'existing': 'must remain'}]) == 'UNSUPPORTED_GATE_CASE'
    with pytest.raises(ValueError, match='both frozen'): transfer.classify_start([dict(status='VERIFIED')])
    with pytest.raises(ValueError, match='unknown'): transfer.classify_start([dict(status='VERIFIED'), dict(status='NA')])


def test_unsupported_domain_is_distinct_from_derivative_mismatch_or_generic_error():
    from reconciliation.gp_se2_diag02_derivatives import DerivativeError
    from reconciliation.gp_se2_diag02_environment import UnsupportedEnvironmentDerivative
    cut = DerivativeError('known cut', reason_code='UNSUPPORTED_WRAP_CUT')
    assert transfer._unsupported_reason(cut) == 'UNSUPPORTED_WRAP_CUT'
    assert transfer._unsupported_reason(DerivativeError('nonfinite evaluation')) is None
    assert transfer._unsupported_reason(ValueError('ordinary mismatch')) is None
    nested = DerivativeError('analytic workspace derivative failed')
    nested.__cause__ = UnsupportedEnvironmentDerivative('feature unavailable')
    assert transfer._unsupported_reason(nested) == 'UNSUPPORTED_ENVIRONMENT_DERIVATIVE'


@pytest.fixture
def unit_sources(monkeypatch, tmp_path):
    """Unit-only fixture resolver, never saved as actual scientific evidence."""
    import reconciliation.gp_se2_diagnostics as diagnostic
    import reconciliation.gp_se2_diag02_environment as geometry
    import reconciliation.gp_se2_diag02_derivatives as derivative
    primary, diag02 = tmp_path/'primary', tmp_path/'prior'
    primary.mkdir(); diag02.mkdir()
    def loader(primary, case_id, method, environment=None):
        problem, _ = make_fixture_problem('S0')
        problem.include_obstacles = method == 'M3_GP_CONSTRAINED'
        return dict(problem=problem, environment=environment or object(),
                    config={'footprint': {'radius_m': .2}}, hashes={'unit_fixture': 'not actual evidence'})
    monkeypatch.setattr(diagnostic, 'load_frozen_case', loader)
    monkeypatch.setattr(geometry, 'EnvironmentDerivatives', lambda *a, **kw: ConstantEnvironmentDerivatives())
    monkeypatch.setattr(transfer, 'verify_diag02_authority', lambda path: dict(
        final_artifact_validation_sha256='unit-final', authoritative_derivative_validation_sha256='unit-derivatives'))
    class UnitProvider:
        def __init__(self, *args): pass
        def stats(self): return {'numerical_fallback_used': False}
    monkeypatch.setattr(derivative, 'DerivativeProvider', UnitProvider)
    return primary, diag02


def _unit_report(point, provider, directions):
    row = dict(compared_elements=1, excluded_elements=0, maximum_scaled_error=.25, maximum_absolute_error=1e-10)
    return (dict(valid=True, required_primal_pass=True, primal_reports=[row],
                 directional=[dict(step=h, smooth_reports=[row]) for h in PROTOCOL['directional_fd_steps']],
                 scope='synthetic harness test only', optimizer_executed=False),
            {'vector': point['vector'], 'directions': directions})


def test_freeze_inventory_same_seed_policy_and_no_runtime_evaluation(unit_sources, tmp_path):
    primary, diag02 = unit_sources
    directory = tmp_path/'smoke'
    manifest = transfer.freeze_transfer(directory, primary, diag02)
    assert manifest['point_count'] == 32 and manifest['start_count'] == 16
    assert not (directory/'results').exists()
    starts = manifest['starts']
    for case_id in transfer.FIXED_CASE_IDS:
        for initialization in transfer.INITIALIZATIONS:
            paired = [row for row in starts if row['case_id'] == case_id and row['initialization'] == initialization]
            assert len(paired) == 2 and paired[0]['seed_vector_sha256'] == paired[1]['seed_vector_sha256']
    records = [json.loads((directory/row['path']).read_text()) for row in manifest['points']]
    shifts = [np.asarray(records[i+1]['perturbation']) for i in range(0, 32, 2)]
    np.testing.assert_array_equal(shifts[0], shifts[-1])
    for i in range(0, 32, 2):
        np.testing.assert_array_equal(np.asarray(records[i]['vector'])+shifts[i//2], records[i+1]['vector'])
    with pytest.raises(FileExistsError): transfer.freeze_transfer(directory, primary, diag02)


def test_complete_smoke_reports_all_starts_and_preserves_numeric_sidecars(unit_sources, monkeypatch, tmp_path):
    primary, diag02 = unit_sources
    directory = tmp_path/'smoke'
    transfer.freeze_transfer(directory, primary, diag02)
    monkeypatch.setattr(transfer, 'validate_point', _unit_report)
    summary = transfer.run_transfer(directory, primary)
    assert summary['complete'] and summary['all_starts_verified']
    assert len(summary['start_lookup']) == 16
    assert set(summary['start_lookup'].values()) == {'VERIFIED'}
    assert len(list((directory/'results').glob('*.npz'))) == 32
    assert summary['maximum_required_directional_scaled_error'] == .25
    assert not summary['optimizer_executed']
    with pytest.raises(FileExistsError): transfer.run_transfer(directory, primary)


def test_unsupported_perturbation_blocks_only_its_start_and_preserves_later_points(unit_sources, monkeypatch, tmp_path):
    from reconciliation.gp_se2_diag02_derivatives import DerivativeError
    primary, diag02 = unit_sources
    directory = tmp_path/'smoke'
    transfer.freeze_transfer(directory, primary, diag02)
    def probe(point, provider, directions):
        if point['name'] == 'case_00_M2_I0_FRESH_fixed_perturbation':
            raise DerivativeError('unit exact cut', reason_code='UNSUPPORTED_WRAP_CUT', diagnostics={'unit': True})
        return _unit_report(point, provider, directions)
    monkeypatch.setattr(transfer, 'validate_point', probe)
    summary = transfer.run_transfer(directory, primary)
    assert summary['complete'] and not summary['all_starts_verified']
    assert len(summary['start_lookup']) == 16
    assert list(summary['start_lookup'].values()).count('VERIFIED') == 15
    assert list(summary['start_lookup'].values()).count('DERIVATIVE_UNSUPPORTED') == 1
    report = json.loads((directory/'results/case_00_M2_I0_FRESH_fixed_perturbation.json').read_text())
    assert report['guard_diagnostics'] == {'unit': True}
    assert not report['valid'] and report['no_finite_difference_fallback']


@pytest.mark.parametrize('corruption', ['protocol', 'point', 'source'])
def test_mutation_after_freeze_fails_before_any_derivative_evaluation(unit_sources, monkeypatch, tmp_path, corruption):
    primary, diag02 = unit_sources
    directory = tmp_path/'smoke'
    manifest = transfer.freeze_transfer(directory, primary, diag02)
    if corruption == 'protocol':
        path = directory/'protocol.json'
        data = json.loads(path.read_text()); data['constraint_derivative_tolerance']['atol'] *= 10
        path.write_text(json.dumps(data))
    elif corruption == 'point':
        path = directory/manifest['points'][0]['path']
        data = json.loads(path.read_text()); data['vector'][0] += .1
        path.write_text(json.dumps(data))
    else:
        monkeypatch.setattr(transfer, '_source_hashes', lambda: {'changed': 'provider'})
    monkeypatch.setattr(transfer, 'validate_point', lambda *a: pytest.fail('must reject before evaluate'))
    with pytest.raises(ValueError, match='changed'): transfer.run_transfer(directory, primary)
    assert not (directory/'results').exists()


def test_current_numerical_sources_match_final_diag02_authority_when_available():
    prior = Path('data/robotless_gp_se2_diag_02/diagnostic_20260918T163000Z')
    if not prior.exists(): pytest.skip('original local DIAG-02 artifact not distributed with source')
    authority = transfer.verify_diag02_authority(prior)
    assert authority['original_validation_not_rebuilt']
    assert authority['current_source_sha256']['gp_se2_diag02_derivatives.py'] == authority['checked_source_sha256']['gp_se2_diag02_derivatives.py']

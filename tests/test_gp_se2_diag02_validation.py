"""Derivative validation harness tests; no scientific optimizer invocation."""
import json

import numpy as np
import pytest

from reconciliation.gp_se2_diag02_validation import (
    PROTOCOL, ConstantEnvironmentDerivatives, _branch_signature, _smooth_rows,
    constraint_families, error_report, freeze_points, freeze_protocol, validate_point,
    validate_unsupported_cut, file_sha256,
)
from reconciliation.gp_se2_diag_fixtures import make_fixture_problem


def test_frozen_error_rule_uses_absolute_plus_relative_scale_and_records_worst_coordinate():
    actual = np.array([[1., 2.], [3., 4.001]])
    reference = np.array([[1., 2.], [3., 4.]])
    report = error_report(actual, reference, {'atol': 1e-5, 'rtol': 1e-5}, row_offset=7)
    assert not report['passed']
    assert report['worst_row'] == 8 and report['worst_column'] == 1
    assert report['actual_at_worst'] == 4.001
    assert report['reference_at_worst'] == 4.
    assert report['allowed_error_at_worst'] == pytest.approx(1e-5+1e-5*4.001)
    assert report['maximum_scaled_error'] > 1


def test_nonsmooth_exclusion_is_unknown_not_zero_error_or_success():
    report = error_report(np.ones((2, 2)), np.zeros((2, 2)), {'atol': 1e-5, 'rtol': 1e-5},
                          mask=np.zeros((2, 2), bool))
    assert report['passed'] is None
    assert report['maximum_absolute_error'] is report['maximum_scaled_error'] is None
    assert report['compared_elements'] == 0 and report['excluded_elements'] == 4


@pytest.mark.parametrize('value', [float('nan'), float('inf')])
def test_nonfinite_derivative_never_passes(value):
    report = error_report([[value]], [[0]], {'atol': 1e-5, 'rtol': 1e-5})
    assert not report['passed'] and not report['finite']


def test_exact_original_constraint_row_units_and_workspace_obstacle_coverage():
    problem, _ = make_fixture_problem('S0')
    groups = constraint_families(problem)
    ineq = [g for g in groups if g['field'] == 'inequality']
    assert [(g['start'], g['stop']) for g in ineq] == [
        (0,90),(90,180),(180,270),(270,360),(360,450),(450,540),(540,630),(630,720),
        (720,721),(721,723),(723,813),(813,903)]
    assert next(g for g in groups if g['name'] == 'goal_squared_distance')['unit'] == 'm^2'
    assert next(g for g in groups if g['name'] == 'linear_acceleration_upper')['unit'] == 'm/s^2'
    problem.include_obstacles = False
    assert constraint_families(problem)[-1]['name'] == 'workspace'
    problem.gates = [{'unit_test': True}]
    with pytest.raises(ValueError, match='nonempty gates'):
        constraint_families(problem)


def test_geometry_crossing_is_excluded_without_masking_motion_or_goal():
    problem, _ = make_fixture_problem('S0')
    constant = dict(smooth=True, kind='constant', selected_segment_id=0)
    center = {f: {'points': [dict(constant) for _ in range(90)]} for f in ('workspace', 'obstacle')}
    plus = {f: {'points': [dict(constant) for _ in range(90)]} for f in ('workspace', 'obstacle')}
    plus['obstacle']['points'][4]['cell_xy'] = [1, 2]
    mask = _smooth_rows(problem, center, plus, center)
    assert mask[:813].all()
    assert not mask[817]
    assert sum(~mask) == 1
    assert _branch_signature({'cell_xy': None})[3] == ()


def test_protocol_and_points_freeze_refuse_overwrite_or_threshold_drift(tmp_path):
    freeze_protocol(tmp_path)
    with pytest.raises(FileExistsError): freeze_protocol(tmp_path)
    problem, fixture = make_fixture_problem('S0')
    point = dict(name='unit_synthetic', problem=problem, geometry=ConstantEnvironmentDerivatives(),
                 vector=fixture['known_vector'], full_coordinate=True, mode='smooth')
    manifest = freeze_points(tmp_path, [point])
    assert len(manifest['points']) == 1
    assert len(json.loads((tmp_path/'directions.json').read_text())['directions']) == 3
    with pytest.raises(FileExistsError): freeze_points(tmp_path, [point])
    data = json.loads((tmp_path/'protocol.json').read_text())
    data['constraint_derivative_tolerance']['atol'] *= 10
    (tmp_path/'protocol.json').write_text(json.dumps(data))
    with pytest.raises(ValueError, match='never silently update'):
        freeze_points(tmp_path, [point])


class LinearUnitProvider:
    """Exact linear black-box stand-in tests harness decisions, not GP math."""
    def __init__(self, problem):
        self.problem = problem
        self.gradient = np.linspace(.1, 1., 150)
        self.eq = np.eye(30, 150)
        self.iq = np.eye(903, 150)
        self.bad_primal = False
        self.bad_derivative = False

    def original(self, x):
        return dict(objective=1.+self.gradient@x, equality=self.eq@x, inequality=self.iq@x)

    def warmup(self, z): return None
    def values(self, z):
        values = self.original(z)
        if self.bad_primal: values['objective'] += .1
        return values
    def objective_gradient(self, z): return self.gradient+(0.1 if self.bad_derivative else 0.)
    def equality_jacobian(self, z): return self.eq
    def inequality_jacobian(self, z): return self.iq
    def geometry_metadata(self, z): return {'scope': 'unit stand-in; never scientific evidence'}
    def stats(self): return dict(scope='unit stand-in; no optimizer', float64_enabled=True,
                                device_backend='cpu', numerical_finite_difference_calls=0,
                                numerical_fallback_used=False)


@pytest.mark.parametrize('failure', [None, 'primal', 'derivative'])
def test_validation_distinguishes_primal_and_full_coordinate_derivative_failures(failure):
    problem, fixture = make_fixture_problem('S0')
    provider = LinearUnitProvider(problem)
    problem.evaluate = provider.original
    provider.bad_primal = failure == 'primal'
    provider.bad_derivative = failure == 'derivative'
    point = dict(name='linear_unit_fixture', problem=problem, geometry=ConstantEnvironmentDerivatives(),
                 vector=fixture['known_vector'], full_coordinate=True, mode='smooth')
    directions = np.random.default_rng(PROTOCOL['random_seed']).normal(size=(3, 150))
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    result, arrays = validate_point(point, provider, directions)
    assert result['valid'] == (failure is None)
    assert result['required_primal_pass'] == (failure != 'primal')
    assert result['coordinate_pass'] == (failure != 'derivative')
    assert arrays['coordinate_fd_inequality'].shape == (903, 150)
    assert result['optimizer_executed'] is False


@pytest.mark.parametrize('failure_kind', ['correct_cut', 'wrong_error', 'no_rejection'])
def test_exact_cut_requires_specific_unsupported_failure_without_fallback(tmp_path, failure_kind):
    from reconciliation.gp_se2_diag02_derivatives import DerivativeError
    problem, fixture = make_fixture_problem('S0')
    point = dict(name='relative_yaw_wrap_cut', problem=problem, vector=fixture['known_vector'],
                 mode='nonsmooth_relative_log_wrap_cut')
    evidence, matrices = tmp_path/'old.json', tmp_path/'old.npz'
    evidence.write_text('preserved characterization test evidence')
    matrices.write_bytes(b'preserved test arrays')
    addendum = dict(expected_unsupported_point=point['name'], previous_cut_report=str(evidence),
                    previous_cut_report_sha256=file_sha256(evidence), previous_cut_matrices=str(matrices),
                    previous_cut_matrices_sha256=file_sha256(matrices))
    provider = LinearUnitProvider(problem)
    def warmup(z):
        if failure_kind != 'no_rejection':
            error = DerivativeError('unit-test failure')
            error.reason_code = 'UNSUPPORTED_WRAP_CUT' if failure_kind == 'correct_cut' else 'UNRELATED_ERROR'
            error.diagnostics = {'scope': 'unit test only'}
            raise error
    provider.warmup = warmup
    report = validate_unsupported_cut(point, provider, addendum)
    assert report['valid'] == (failure_kind == 'correct_cut')
    assert report['correctly_rejected'] == (failure_kind == 'correct_cut')
    assert not report['supported_derivative_domain']
    assert report['primal_reports'] == []

"""Saved derivative smoke arithmetic corruption tests, never research evidence."""
from copy import deepcopy

import numpy as np
import pytest

from reconciliation.gp_se2_02_transfer_validation import validate_supported_point
from reconciliation.gp_se2_diag02_validation import (
    PROTOCOL, ConstantEnvironmentDerivatives, plain, validate_point,
)
from reconciliation.gp_se2_diag_fixtures import make_fixture_problem


class Checks:
    def check(self, condition, label):
        assert condition, label

    def equal(self, actual, expected, label, *, exact=False):
        actual, expected = plain(actual), plain(expected)
        def match(a, b):
            if isinstance(a, dict) and isinstance(b, dict):
                return a.keys() == b.keys() and all(match(a[key], b[key]) for key in a)
            if isinstance(a, list) and isinstance(b, list):
                return len(a) == len(b) and all(match(x, y) for x, y in zip(a, b))
            if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not exact:
                return bool(np.isclose(a, b, atol=1e-11, rtol=1e-9))
            return type(a) == type(b) and a == b
        assert match(actual, expected), label


class LinearUnitProvider:
    """Exact linear harness stand-in; it does not claim GP derivative evidence."""
    def __init__(self):
        self.gradient = np.linspace(.1, 1., 150)
        self.equality = np.eye(30, 150)
        self.inequality = np.eye(903, 150)

    def original(self, vector):
        return dict(objective=1+self.gradient@vector, equality=self.equality@vector,
                    inequality=self.inequality@vector)

    def warmup(self, vector): pass
    def values(self, vector): return self.original(vector)
    def objective_gradient(self, vector): return self.gradient
    def equality_jacobian(self, vector): return self.equality
    def inequality_jacobian(self, vector): return self.inequality
    def geometry_metadata(self, vector): return {'unit_only': True}
    def stats(self):
        return dict(float64_enabled=True, device_backend='cpu',
                    numerical_finite_difference_calls=0, numerical_fallback_used=False)


@pytest.fixture
def saved_point():
    problem, fixture = make_fixture_problem('S0')
    provider = LinearUnitProvider()
    problem.evaluate = provider.original
    point = dict(name='unit_synthetic', problem=problem, geometry=ConstantEnvironmentDerivatives(),
                 vector=fixture['known_vector'], full_coordinate=False, mode='smooth_with_geometry_classification')
    directions = np.random.default_rng(PROTOCOL['random_seed']).normal(size=(3, 150))
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    record, arrays = validate_point(point, provider, directions)
    record.update(status='VERIFIED', central_nonsmooth_exclusions=0, coordinate_sweep_not_repeated=True)
    return point, record, arrays, directions, provider


def test_saved_point_replays_original_primal_full_matrices_and_every_required_error(saved_point):
    report = validate_supported_point(*saved_point, Checks())
    assert report['status'] == 'VERIFIED'
    assert len(report['required_reports']) == 28  # 14 original families × two finest h.
    assert all(row['passed'] for row in report['required_reports'])


@pytest.mark.parametrize('corruption,match', [
    ('vector', 'saved chart'),
    ('direction', 'saved directions'),
    ('jacobian', 'independent supplied equality'),
    ('prediction', 'Jd projection'),
    ('central', 'central and one-sided arithmetic'),
    ('one_side', 'central and one-sided arithmetic'),
    ('mask', 'no objective/lateral/motion/goal row excluded'),
    ('geometry_mask', 'branch-consistent mask'),
    ('reported_error', 'original family error arithmetic'),
    ('step', 'unchanged multistep rule'),
    ('status', 'status follows original'),
    ('primal', 'independent original primal reports'),
])
def test_corrupted_saved_math_or_pass_flag_cannot_validate(saved_point, corruption, match):
    point, record, arrays, directions, provider = saved_point
    record, arrays = deepcopy(record), {key: value.copy() for key, value in arrays.items()}
    if corruption == 'vector': arrays['vector'][0] += 1
    elif corruption == 'direction': arrays['directions'][0, 0] += .01
    elif corruption == 'jacobian': arrays['equality_jacobian'][0, 0] += .1
    elif corruption == 'prediction': arrays['directional_1_objective_prediction'][0, 0] += .1
    elif corruption == 'central': arrays['directional_1_equality_central_fd'][0, 0] += .1
    elif corruption == 'one_side': arrays['directional_1_equality_plus_fd'][0, 0] += .1
    elif corruption == 'mask': arrays['directional_1_equality_smooth_mask'][0, 0] = False
    elif corruption == 'geometry_mask': arrays['directional_1_inequality_smooth_mask'][813, 0] = False
    elif corruption == 'reported_error': record['directional'][1]['smooth_reports'][0]['maximum_scaled_error'] = 1234.
    elif corruption == 'step': record['directional'][1]['step'] = 1e-3
    elif corruption == 'status': record['valid'] = False
    elif corruption == 'primal': record['primal_reports'][0]['passed'] = False
    with pytest.raises(AssertionError, match=match):
        validate_supported_point(point, record, arrays, directions, provider, Checks())


def test_replay_uses_no_original_primal_finite_difference_sweep(saved_point):
    point, record, arrays, directions, provider = saved_point
    calls = []
    def original_at_center_only(vector):
        calls.append(np.asarray(vector).copy())
        np.testing.assert_array_equal(vector, point['vector'])
        return provider.original(vector)
    point['problem'].evaluate = original_at_center_only
    validate_supported_point(point, record, arrays, directions, provider, Checks())
    assert len(calls) == 1

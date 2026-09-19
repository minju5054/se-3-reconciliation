"""Fixed GP-SE2-02 identities and candidate policy; no trajectory mathematics."""
from __future__ import annotations

import numpy as np

CASES = (
    ('episode_001_repeat_01/handoff_002', 'BENIGN_CONTROL'),
    ('episode_013_repeat_01/handoff_024', 'HARD_POSITION_AND_DIRECTION'),
    ('episode_014_repeat_01/handoff_026', 'HARD_LARGE_TURN'),
    ('episode_000_repeat_01/handoff_019', 'HARD_POSITION_STRAIGHT'),
)
METHODS = ('M0_NATIVE', 'M0_ADAPTER', 'M1_RIGID', 'M2_GP_NO_OBSTACLE',
           'M3_GP_CONSTRAINED', 'SEED_ONLY')
GP_METHODS = METHODS[3:5]
INITIALIZATIONS = ('I0_FRESH', 'I1_DECEL')


def validate_config(config):
    if config['experiment'] != 'GP-SE2-02':
        raise ValueError('wrong experiment')
    if [(c['case_id'], c['role']) for c in config['cases']] != list(CASES):
        raise ValueError('four fixed cases and order are immutable')
    if tuple(config['methods']) != METHODS or tuple(config['gp_initializations']) != INITIALIZATIONS:
        raise ValueError('fixed method/initialization coverage changed')
    if config['prepared_solve_seconds_per_gp_start'] != 30 or config['max_iterations'] != 200:
        raise ValueError('original solver budgets required')
    if (config['blas_threads'] != 1 or config['solver_retry'] or config['new_inference']
            or config['new_environment_export'] or config['new_initialization']):
        raise ValueError('sequential frozen scope required')


def execution_order():
    """Rigid call contains its unchanged two starts; GP starts are explicit."""
    rows = []
    for case_id, role in CASES:
        rows.append(dict(case_id=case_id, case_role=role, method='M1_RIGID',
                         initializations=['identity', 'finite_lookahead_alignment']))
        for method in GP_METHODS:
            for initialization in INITIALIZATIONS:
                rows.append(dict(case_id=case_id, case_role=role, method=method,
                                 initialization=initialization))
    return rows


def select_retained_start(results):
    """Keep original dense-first minimum-cost rule; never swap after full check.

    `results` is in the frozen I0/I1 order and may contain explicit unsupported
    start records with no candidate. Full acceptance deliberately does not alter
    this selection. The caller decides whether the selected plan may be executed.
    """
    if len(results) != 2:
        raise ValueError('both planned initialization records are required')
    choices = []
    for index, result in enumerate(results):
        if result.get('candidate_found'):
            if result.get('candidate_vector') is None or result.get('candidate_objective') is None:
                raise ValueError('candidate flag requires actual vector and objective')
            if not result['constraint_report']['feasible']:
                raise ValueError('retained candidate must pass unchanged dense acceptance')
            cost = float(result['candidate_objective'])
            if not np.isfinite(cost):
                raise ValueError('nonfinite objective cannot be selected')
            choices.append((cost, index, result.get('selected_iterate') or ''))
    return min(choices)[1] if choices else None


def classify_derivative_error(error):
    """Domain unsupported is distinct from generic numerical/backend failure."""
    from .gp_se2_diag02_environment import UnsupportedEnvironmentDerivative
    if getattr(error, 'reason_code', None) == 'UNSUPPORTED_WRAP_CUT':
        return 'DERIVATIVE_UNSUPPORTED'
    cause = error
    while cause is not None:
        if isinstance(cause, UnsupportedEnvironmentDerivative):
            return 'DERIVATIVE_UNSUPPORTED'
        cause = cause.__cause__
    return 'NUMERICAL_FAILURE'


class RecordingDerivatives:
    """Capture exact provider failures while forwarding every original callback."""
    def __init__(self, provider):
        self.provider = provider
        self.errors = []

    def _call(self, name, vector):
        from .gp_se2_diag02_derivatives import DerivativeError
        try:
            return getattr(self.provider, name)(vector)
        except DerivativeError as error:
            self.errors.append(dict(callback=name, reason_code=error.reason_code,
                                    classification=classify_derivative_error(error),
                                    message=str(error), diagnostics=error.diagnostics,
                                    vector=np.array(vector, dtype=np.float64, copy=True)))
            raise

    def objective_gradient(self, vector):
        return self._call('objective_gradient', vector)

    def equality_jacobian(self, vector):
        return self._call('equality_jacobian', vector)

    def inequality_jacobian(self, vector):
        return self._call('inequality_jacobian', vector)

    def stats(self):
        return self.provider.stats()

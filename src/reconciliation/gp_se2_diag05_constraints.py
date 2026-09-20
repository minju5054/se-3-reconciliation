"""G2 ablation: unchanged G0 equalities, only DIAG-04 quarter inequalities.

Historical modules are imported, never patched. Quarter lateral values may be
computed inside the frozen AD graph, but cannot enter an optimizer callback.
"""
from __future__ import annotations
import time
import numpy as np
from .gp_se2_diag04_constraints import (
    ConstraintView, RefinedDerivativeProvider, quarter_motion_values,
    G1_QUARTER, MOTION_ROWS, _vector, constraint_row_metadata,
)

G2 = 'G2_QUARTER_INEQUALITY_ONLY'


class InequalityOnlyView(ConstraintView):
    def __init__(self, base_problem):
        super().__init__(base_problem, G1_QUARTER)
        self.grid = self.grid_name = G2

    def evaluate(self, vector):
        z = _vector(self.base, vector)
        if self._view_last_x is not None and np.array_equal(z, self._view_last_x):
            return self._view_last_eval
        original = self.base.evaluate(z)
        started = time.perf_counter()
        quarter = quarter_motion_values(self.base, z)
        self._quarter_seconds += time.perf_counter()-started
        self._quarter_calls += 1
        result = dict(original, quarter_inequality=quarter['quarter_inequality'],
            inequality=np.concatenate((original['inequality'], quarter['quarter_inequality'])))
        # original['equality'] itself is retained, without even a concatenation.
        self._view_last_x, self._view_last_eval = z.copy(), result
        return result

    def appended_row_metadata(self, vector):
        return [r for r in super().appended_row_metadata(vector) if r['constraint_kind']=='inequality']


class InequalityOnlyDerivatives:
    def __init__(self, view, base_provider):
        if not isinstance(view, InequalityOnlyView) or base_provider.problem is not view.base:
            raise TypeError('G2 view and identical original provider required')
        self.view, self.base_provider = view, base_provider
        self.problem = view.base
        # Reuse the entire frozen AD implementation; only inequality output is exposed.
        self._quarter = RefinedDerivativeProvider(ConstraintView(view.base, G1_QUARTER), base_provider)
        self.variable_count = base_provider.variable_count
        self.equality_count = base_provider.equality_count
        self.inequality_count = base_provider.inequality_count+480

    def values(self, vector):
        original = self.base_provider.values(vector)
        quarter = self._quarter.quarter_values(vector)['quarter_inequality']
        return dict(original, quarter_inequality=quarter,
            inequality=np.concatenate((original['inequality'], quarter)))

    def objective_gradient(self, vector):
        return self.base_provider.objective_gradient(vector)

    def equality_jacobian(self, vector):
        return self.base_provider.equality_jacobian(vector)

    def inequality_jacobian(self, vector):
        return self._quarter.inequality_jacobian(vector)

    def warmup(self, vector):
        self._quarter.warmup(vector)
        return self.stats()

    def reset_stats(self, *, clear_cache=True):
        self._quarter.reset_stats(clear_cache=clear_cache)

    def stats(self):
        return dict(self._quarter.stats(), grid=G2, equality_count=self.equality_count,
            inequality_count=self.inequality_count, appended_equalities=0,
            internal_quarter_lateral_outputs='computed by frozen AD helper; never exposed to optimizer')

    def geometry_metadata(self, vector):
        return self.base_provider.geometry_metadata(vector)


def margin_diagnostics(view, provider, vector):
    """Saved-margin diagnostic, not the SLSQP internal active set."""
    e=view.evaluate(vector); margins=e['quarter_inequality'].reshape(8,60)
    jac=provider.inequality_jacobian(vector)[-480:].reshape(8,60,150)
    tolerance=view.config['inequality_tolerance']; near=max(1e-6,10*tolerance)
    return [dict(family=family,unit=unit,margin=values.tolist(),
        minimum_margin=float(np.min(values)),maximum_violation=float(max(0.,-np.min(values))),
        violating_rows=int(np.sum(values < -tolerance)),
        active_within_tolerance=int(np.sum(np.abs(values)<=tolerance)),
        near_active_including_violating=int(np.sum(values<=near)),
        feasible_near_active=int(np.sum((values>=-tolerance)&(values<=near))),
        near_active_threshold=near,feasibility_tolerance=tolerance,
        row_norms=np.linalg.norm(jac[k],axis=1).tolist(),
        source='descriptive saved constraint margins; not solver internal active set')
        for k,((family,_,unit,*_),values) in enumerate(zip(MOTION_ROWS,margins))]

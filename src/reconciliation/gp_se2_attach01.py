"""Fixed correspondence and externally imposed duration; no correspondence search.

F_common[k-1] belongs to support k (time k*.1s); support zero is B.
The trajectory variables and all original physical constraints are unchanged.
"""
from __future__ import annotations

import hashlib
import numpy as np
from .se2 import relative_pose, se2_log, wrap_angle
# Reuse only the existing numeric masked-cost/tube AD helper and derivative gate.
# Its constructor consumes view.reference, never builds or selects correspondence.
from .gp_se2_join01_formulation import JoinDerivatives, verify_derivatives
from .gp_se2_join01 import sustained_join, forward_projection

T_SET = (.4, .6, .8, 1., 1.2, 1.4)
POSITION_M = .10
YAW_RAD = np.pi/12
DWELL_S = .30


def method_schedule():
    return [('M0_NATIVE', None), ('M0_ADAPTER', None), ('M3_CURRENT_GP', None)] + [
        (f'M4_FIXED_ATTACH_T{round(t*10):02d}', t) for t in T_SET]


def support_index(duration, times):
    t = np.asarray(times, dtype=np.float64)
    if duration not in T_SET:
        raise ValueError('external duration is not in the frozen tested set')
    k = int(round(duration/(t[1]-t[0])))
    if not np.isclose(t[k], duration, rtol=0, atol=1e-12) or duration+DWELL_S > t[-1]+1e-12:
        raise ValueError('duration and persistence must lie on the original support grid')
    return k


def select_full_valid(records):
    """Select within ONE condition only; no ordering/selection across durations."""
    valid = [r for r in records if r['full_valid']]
    return min(valid, key=lambda r:(r['objective'], r['initialization'], r['source_label'])) if valid else None


class FixedAttachView:
    def __init__(self, base, duration_s):
        self.base = self.base_problem = base
        self.config, self.times = base.config, base.times
        self.duration_s = float(duration_s)
        self.k = support_index(self.duration_s, self.times)
        self.grid_name = f'M4_FIXED_ATTACH_T{round(duration_s*10):02d}'
        self.reference = base.common_reference[self.k-1:].copy()
        self.reference.setflags(write=False)
        self.tube_count = 4
        self.common_value_sha256 = hashlib.sha256(base.common_reference.tobytes()).hexdigest()
        self.clear_cache()

    def clear_cache(self):
        self.base._last_x = self.base._last_eval = None
        self._last_x = self._last_eval = None

    def unpack(self, vector):
        return self.base.unpack(vector)

    def evaluate(self, vector):
        if self._last_x is not None and np.array_equal(vector, self._last_x):
            return self._last_eval
        original = self.base.evaluate(vector)
        p = original['poses'][self.k:]
        residual = se2_log(relative_pose(self.reference, p))/self.config['fresh_std']
        fresh_cost = float(np.mean(np.sum(residual**2, axis=1)))
        a, r = p[:self.tube_count], self.reference[:self.tube_count]
        yaw = wrap_angle(a[:,2]-r[:,2])
        margins = np.r_[POSITION_M**2-np.sum((a[:,:2]-r[:,:2])**2,axis=1), YAW_RAD-yaw, YAW_RAD+yaw]
        result = dict(original, fresh_cost=fresh_cost,
            objective=float(np.sum(original['gp_factor_costs'])+self.config['lambda_fresh']*fresh_cost),
            inequality=np.r_[original['inequality'],margins], attachment_margins=margins)
        self._last_x, self._last_eval = np.array(vector,copy=True), result
        return result

    def independent_tube_check(self, vector):
        p,_ = self.base.unpack(vector)
        a, r = p[self.k:self.k+4], self.reference[:4]
        d = np.linalg.norm(a[:,:2]-r[:,:2],axis=1)
        yaw = np.abs(wrap_angle(a[:,2]-r[:,2]))
        return dict(valid=bool(np.all(d<=POSITION_M) and np.all(yaw<=YAW_RAD)),
            distance_m=d, yaw_error_rad=yaw, times_s=self.times[self.k:self.k+4],
            support_indices=list(range(self.k,self.k+4)), reference_row_indices=list(range(self.k-1,self.k+3)),
            position_threshold_m=POSITION_M, yaw_threshold_rad=YAW_RAD,
            tolerance_policy='direct nominal check, no solver numerical allowance', continuous_time_proof=False)


class AttachDerivatives(JoinDerivatives):
    """Unchanged CPU AD algebra, using fixed common rows supplied by this view.

No JoinView, post_join_reference, join_candidates or candidate search is used.
The existing helper differentiates only masked cost and four tube constraints.
Original objective Jacobian is adjusted; original equality and inequality
prefix Jacobians remain literal. No new backend or finite-difference fallback.
"""
    def __init__(self, view, base_provider):
        if not isinstance(view, FixedAttachView):
            raise TypeError('fixed externally supplied duration view required')
        super().__init__(view, base_provider)

    def stats(self):
        out = super().stats()
        out.update(experiment='ATTACH01', correspondence_fixed=True, duration_decision_variable=False,
                   reused_AD_helper='JOIN01 masked-cost/tube algebra only; no outer search or reference construction')
        return out


def rejection_reasons(check, tube=None):
    reasons = []
    if not check['collocation']['feasible']: reasons.append('SOLVER_GRID_FAILURE')
    if not check['constraint_report']['feasible']: reasons.append('ORIGINAL_DENSE_FAILURE')
    full = check['full_acceptance']
    if full.get('additional_grid'):
        reasons += [k for k,v in full['additional_grid']['flags'].items() if not v]
    if not full.get('full_feasible'): reasons.append('ORIGINAL_FULL_FAILURE')
    if tube is not None and not tube['valid']: reasons.append('NOMINAL_ATTACHMENT_TUBE_FAILURE')
    return reasons

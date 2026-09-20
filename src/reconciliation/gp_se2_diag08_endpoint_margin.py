"""G4: one constant-shifted terminal-position row; all G3 prefixes delegate.

The derivative is the existing CPU float64 AD goal-position row, including the
right-chart retraction chain rule. A different constant radius has zero
additional derivative. No finite difference, graph, or numerical core change.
"""
from __future__ import annotations
import time
import numpy as np
from .gp_se2_diag04_constraints import ConstraintView, _vector
from .gp_se2_diag06_constraints import WitnessView
from .gp_se2_diag02_derivatives import DerivativeError
from .gp_se2_diag07_lateral_execution import REQUIRED_FLAGS

G4 = 'G4_FIXED_ENDPOINT_RESERVE'
RESERVE_M = .04
PLAN_RADIUS_M = .11
EXEC_RADIUS_M = .15

DEFINITION = dict(reserve_m=RESERVE_M, planning_radius_m=PLAN_RADIUS_M,
    execution_radius_m=EXEC_RADIUS_M, family='planning_endpoint_reserve_position',
    unit='m^2', convention='g >= 0', expression='0.11**2 - ||p_T-original_goal_xy||**2',
    added_equalities=0, added_inequalities=1, added_yaw_rows=0, added_environment_rows=0,
    derivative='unchanged G3 original goal-position AD Jacobian row; constant radius shift',
    planning_endpoint_reserve_pass='norm(endpoint-original_goal_xy) <= 0.11, no added tolerance',
    solver_grid_allowance='unchanged original inequality_tolerance in m^2; reported separately',
    execution_reference_policy='saved latest_iterate poses[1:] for every start; no reselection or fallback',
    historical_baseline='DIAG06 G3 / DIAG07 executions, never rerun',
    development_event_margin=True, margin_search=False)


class G4EndpointMarginView(ConstraintView):
    def __init__(self, g3):
        if not isinstance(g3, WitnessView):
            raise TypeError('frozen G3 WitnessView required')
        if g3.config['goal_position_tolerance'] != EXEC_RADIUS_M:
            raise ValueError('original execution goal tolerance changed')
        self.g3 = g3
        super().__init__(g3.base)
        self.grid = self.grid_name = G4
        # Original evaluator: 8 signed motion families x 3 sites x intervals,
        # immediately followed by goal-position, yaw upper/lower, gates, env.
        self.goal_row = 8*3*(len(self.times)-1)

    def evaluate(self, vector):
        z = _vector(self.base, vector)
        if self._view_last_x is not None and np.array_equal(z, self._view_last_x):
            return self._view_last_eval
        original = self.g3.evaluate(z)
        t = time.perf_counter()
        delta = original['poses'][-1, :2]-self.base.goal_pose[:2]
        row = np.array([PLAN_RADIUS_M**2-np.sum(delta**2)], dtype=np.float64)
        if not np.isfinite(row).all():
            raise FloatingPointError('nonfinite endpoint reserve')
        self._margin_s += time.perf_counter()-t
        self._margin_calls += 1
        result = dict(original, endpoint_margin=row,
                      inequality=np.concatenate((original['inequality'], row)))
        self._view_last_x, self._view_last_eval = z.copy(), result
        return result

    def clear_cache(self):
        super().clear_cache()
        self.g3.clear_cache()

    def reset_stats(self):
        self._margin_s = 0.
        self._margin_calls = 0
        self.g3.reset_stats()

    def stats(self):
        return dict(self.g3.stats(), endpoint_primal_calls=self._margin_calls,
                    endpoint_primal_wall_s=self._margin_s, endpoint_extra_environment_queries=0)

    def dimensions(self, vector):
        original = self.g3.dimensions(vector)
        return dict(original, grid=G4, inequality_count=original['inequality_count']+1,
                    appended_inequality_count=original['appended_inequality_count']+1,
                    added_goal_rows=1, G3_prefix_inequality_count=original['inequality_count'],
                    G4_added_equality_count=0, G4_added_inequality_count=1)

    def appended_row_metadata(self, vector):
        rows = self.g3.appended_row_metadata(vector)
        rows.append(dict(constraint_kind='inequality',row_index=len(self.g3.evaluate(vector)['inequality']),
            family=DEFINITION['family'], physical_quantity='squared_goal_position_distance',
            physical_unit='m^2', expression=DEFINITION['expression'], source='G4_fixed_endpoint_reserve',
            nominal_limit=PLAN_RADIUS_M**2, trajectory_time_s=float(self.times[-1]),
            actual_numerical_tolerance=self.config['inequality_tolerance'],
            signed_residual_or_margin=float(self.evaluate(vector)['endpoint_margin'][0]),
            derivative_source_row_index=self.goal_row))
        return rows


class G4EndpointMarginDerivatives:
    def __init__(self, view, g3_provider):
        if view.g3 is not g3_provider.view:
            raise ValueError('identical G3 view/provider required')
        self.view, self.g3 = view, g3_provider
        self.base_provider = g3_provider.base_provider
        self.problem = view.base
        self.variable_count = g3_provider.variable_count
        self.equality_count = g3_provider.equality_count
        self.inequality_count = g3_provider.inequality_count+1

    def values(self, vector):
        original = self.g3.values(vector)
        # Same AD endpoint function, shifted only by the radius constant.
        row = np.array([original['inequality'][self.view.goal_row]
                        + PLAN_RADIUS_M**2-EXEC_RADIUS_M**2])
        return dict(original, endpoint_margin=row,
                    inequality=np.concatenate((original['inequality'], row)))

    def objective_gradient(self, vector):return self.g3.objective_gradient(vector)
    def equality_jacobian(self, vector):return self.g3.equality_jacobian(vector)
    def inequality_jacobian(self, vector):
        original = self.g3.inequality_jacobian(vector)
        row = original[self.view.goal_row:self.view.goal_row+1]
        if row.shape != (1,self.variable_count) or not np.isfinite(row).all():
            raise DerivativeError('invalid endpoint Jacobian; no FD fallback')
        return np.concatenate((original, row))
    def warmup(self, vector):
        self.g3.warmup(vector)
        self.inequality_jacobian(vector)
        return self.stats()
    def reset_stats(self, *, clear_cache=True):self.g3.reset_stats(clear_cache=clear_cache)
    def geometry_metadata(self, vector):return self.g3.geometry_metadata(vector)
    def stats(self):
        return dict(self.g3.stats(), grid=G4, inequality_count=self.inequality_count,
                    endpoint_derivative_source='existing original AD goal-position row',
                    endpoint_additional_compilations=0, endpoint_FD_calls=0)


def reserve_report(problem, vector):
    poses, _ = problem.unpack(_vector(problem,vector))
    delta = poses[-1,:2]-problem.goal_pose[:2]
    distance = float(np.linalg.norm(delta))
    margin = float(PLAN_RADIUS_M**2-np.sum(delta**2))
    return dict(endpoint_position_error_m=distance, planning_reserve_m=EXEC_RADIUS_M-distance,
        required_reserve_m=RESERVE_M, planning_radius_m=PLAN_RADIUS_M,
        planning_endpoint_reserve_pass=bool(distance <= PLAN_RADIUS_M),
        nominal_squared_margin_m2=margin,
        solver_row_pass_with_original_allowance=bool(margin >= -problem.config['inequality_tolerance']),
        original_squared_margin_allowance_m2=problem.config['inequality_tolerance'],
        nominal_position_excess_m=max(0., distance-PLAN_RADIUS_M),
        original_goal_unchanged=True)


def execution_admission(full, reserve, *, hard):
    flags = full.get('additional_grid',{}).get('flags',{})
    if set(flags) != REQUIRED_FLAGS:raise ValueError('missing original full checker coverage')
    failed = sorted(k for k,v in flags.items() if not v)
    motion_failed = sorted(k for k,v in full['original_plan_report']['motion']['violations'].items() if v)
    full_valid = bool(full['full_feasible'])
    lateral_only = (failed == ['lateral_velocity'] and motion_failed == ['lateral_velocity']
        and not full_valid and full['original_dense_report']['maximum_inequality_violation'] <=
        full['original_config']['formulation']['inequality_tolerance'])
    reserve_ok = reserve['planning_endpoint_reserve_pass']
    eligible = bool(reserve_ok and (full_valid or (hard and lateral_only)))
    reasons = ([] if reserve_ok else ['planning_endpoint_reserve']) + failed
    if not full_valid and not failed:reasons += ['original_dense_or_plan_failure']
    return dict(plan_valid=full_valid, planning_endpoint_reserve_pass=reserve_ok,
        G4_plan_valid=bool(full_valid and reserve_ok),
        deployment_candidate=bool(full_valid and reserve_ok),
        eligible_for_execution=eligible,
        plan_failure_reason=None if full_valid else 'lateral_velocity' if lateral_only else reasons,
        reference_kind='PLAN_VALID_CANDIDATE' if full_valid else
            'LATERAL_ONLY_PLAN_INVALID_REFERENCE' if lateral_only else 'OTHER_PLAN_INVALID_REFERENCE',
        diagnostic_execution_authorized=eligible, diagnostic_execution_only=bool(eligible and not full_valid),
        ineligible_reasons=[] if eligible else reasons,
        label='PLAN INVALID: LATERAL ONLY / DIAGNOSTIC EXECUTION ONLY' if lateral_only else
            'PLAN VALID CANDIDATE / OFFLINE COUNTERFACTUAL' if full_valid else 'PLAN INVALID / NOT EXECUTED')


def endpoint_geometry(goal, plan, executed):
    g,p,x = (np.asarray(v,dtype=np.float64)[:2] for v in (goal,plan,executed))
    ep,et,ee = p-g,x-p,x-g
    pn,tn,en = (float(np.linalg.norm(v)) for v in (ep,et,ee))
    dot = float(ep@et)
    angle = None if pn*tn == 0. else float(np.arccos(np.clip(dot/(pn*tn),-1.,1.)))
    return dict(e_plan_m=ep.tolist(),e_track_m=et.tolist(),e_exec_m=ee.tolist(),
        plan_error_m=pn,tracking_displacement_m=tn,execution_error_m=en,
        vector_closure_error_m=float(np.linalg.norm(ee-ep-et)),dot_plan_track_m2=dot,
        angle_plan_track_rad=angle,planning_reserve_m=EXEC_RADIUS_M-pn,execution_reserve_m=EXEC_RADIUS_M-en,
        triangle_conditions_observed=bool(pn<=PLAN_RADIUS_M and tn<=RESERVE_M),
        scalar_norm_sum_is_not_actual_error=True)


def classify(*, hard, admission, execution, historical_error):
    if not hard:
        if not admission['G4_plan_valid']:return 'BENIGN_MARGIN_REGRESSION'
        if execution is None:return 'BENIGN_MARGIN_PLAN_ONLY'
        return 'BENIGN_MARGIN_EXECUTION_PRESERVED' if execution['primary_success'] else 'BENIGN_MARGIN_REGRESSION'
    if not admission['eligible_for_execution']:
        return ('MARGIN_INTRODUCES_OTHER_PLAN_FAILURE' if admission['planning_endpoint_reserve_pass']
                else 'MARGIN_PLAN_INFEASIBLE_OR_SOLVER_FAILURE')
    if execution is None:return 'MARGIN_PLAN_INFEASIBLE_OR_SOLVER_FAILURE'
    if execution['primary_success']:return 'MARGIN_EXECUTION_RECOVERY'
    # Descriptive 1e-6 m numerical threshold, frozen before results.
    if historical_error-execution['execution']['terminal_position_error_m'] > 1e-6:
        return 'MARGIN_IMPROVES_GOAL_BUT_NOT_SUCCESS'
    return 'NO_MARGIN_EXECUTION_BENEFIT'


def execute_reference(module, frozen, reference, admission):
    """Same DIAG07 transport, with explicit G4 valid/invalid plan labels.

    Its storage wrapper restricts all references to diagnostic-only terminology.
    Keep the execution calculation identical and restore G4's independently
    checked candidate terminology afterwards. There is no controller override.
    """
    from .gp_se2_diag07_lateral_execution import diagnostic_rollout
    if not admission['eligible_for_execution']:raise ValueError('ineligible G4 reference')
    transport = dict(admission, deployment_candidate=False,diagnostic_execution_only=True)
    result = diagnostic_rollout(module,frozen,reference,transport)
    result.update(admission)
    result['actual_deployment_performed'] = False
    return result


def validate_execution_records(rollout, context, reference, admission):
    from .gp_se2_diag07_lateral_execution import validate_rollout_records
    errors = [f'admission field {k}' for k,v in admission.items() if rollout.get(k)!=v]
    if not admission['eligible_for_execution']:errors.append('ineligible execution')
    if not admission['plan_valid'] and (admission['deployment_candidate'] or
        not admission['diagnostic_execution_only']):errors.append('invalid plan promoted')
    # Only DIAG07's experiment-specific labeling assertion is normalized in a
    # temporary copy. Every recorded state/command/reference/timing is unchanged.
    common = dict(rollout,deployment_candidate=False,diagnostic_execution_only=True)
    return errors + validate_rollout_records(common,context,reference)

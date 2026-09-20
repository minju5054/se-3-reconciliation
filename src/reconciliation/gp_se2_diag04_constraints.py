"""DIAG-04 appends quarter-point motion rows to an unchanged GP problem.

The primal source remains GPProblem.evaluate and the original NumPy interval
interpolator.  The original derivative provider supplies every original row;
a separate CPU float64 JAX graph differentiates only the appended motion rows.
This view intentionally has no dense_report: independent acceptance must use
``view.base`` so its historical meaning cannot change through inheritance.
"""
from __future__ import annotations

import copy
import time

import numpy as np

from .gp_se2 import interpolate_interval
from .gp_se2_diag02_derivatives import DerivativeError, DerivativeProvider
from .gp_se2_formulation import GPProblem, _motion_inequalities


G0_ORIGINAL = "G0_ORIGINAL"
G1_QUARTER = "G1_QUARTER"
QUARTER_FRACTIONS = (.25, .75)
MOTION_ROWS = (
    ("linear_speed_lower", "v_x", "m/s", 1., "v_x >= 0", None),
    ("linear_speed_upper", "v_x", "m/s", -1., "v_max - v_x >= 0", "v_max"),
    ("angular_speed_upper", "omega", "rad/s", -1., "w_max - omega >= 0", "w_max"),
    ("angular_speed_lower", "omega", "rad/s", 1., "w_max + omega >= 0", "w_max"),
    ("linear_acceleration_upper", "a_x", "m/s^2", -1., "a_v_max - a_x >= 0", "a_v_max"),
    ("linear_acceleration_lower", "a_x", "m/s^2", 1., "a_v_max + a_x >= 0", "a_v_max"),
    ("angular_acceleration_upper", "alpha", "rad/s^2", -1., "a_w_max - alpha >= 0", "a_w_max"),
    ("angular_acceleration_lower", "alpha", "rad/s^2", 1., "a_w_max + alpha >= 0", "a_w_max"),
)


def _vector(problem, vector):
    z = np.asarray(vector, dtype=np.float64)
    if z.shape != (5 * (len(problem.times)-1),) or not np.isfinite(z).all():
        raise ValueError("finite original-chart vector required")
    return z


def quarter_motion_values(problem, vector):
    """NumPy primal quarter rows; no goal, workspace or obstacle queries."""
    p, v = problem.unpack(_vector(problem, vector))
    pp, vv, aa = interpolate_interval(
        p[:-1, None], v[:-1, None], p[1:, None], v[1:, None],
        problem.config["support_dt_s"], np.asarray(QUARTER_FRACTIONS))
    velocity, acceleration = vv.reshape((-1, 3)), aa.reshape((-1, 3))
    return dict(quarter_equality=velocity[:, 1].copy(),
                quarter_inequality=_motion_inequalities(velocity, acceleration, problem.config),
                quarter_poses=pp.reshape((-1, 3)), quarter_body_twists=velocity,
                quarter_body_accelerations=acceleration)


class ConstraintView:
    """One-vector primal cache, identical original rows, explicit grid name."""

    def __init__(self, base_problem, grid=G0_ORIGINAL):
        if not isinstance(base_problem, GPProblem):
            raise TypeError("base_problem must be the original GPProblem")
        if grid not in (G0_ORIGINAL, G1_QUARTER):
            raise ValueError("unsupported fixed motion grid")
        self.base = self.base_problem = self.problem = base_problem
        self.grid = self.grid_name = grid
        self.config, self.times = base_problem.config, base_problem.times
        self._view_last_x = self._view_last_eval = None
        self.reset_stats()

    @property
    def _last_x(self):
        return self.base._last_x if self.grid == G0_ORIGINAL else self._view_last_x

    @_last_x.setter
    def _last_x(self, value):
        if self.grid == G0_ORIGINAL:
            self.base._last_x = value
        else:
            self._view_last_x = value

    @property
    def _last_eval(self):
        return self.base._last_eval if self.grid == G0_ORIGINAL else self._view_last_eval

    @_last_eval.setter
    def _last_eval(self, value):
        if self.grid == G0_ORIGINAL:
            self.base._last_eval = value
        else:
            self._view_last_eval = value

    def unpack(self, vector):
        return self.base.unpack(vector)

    def clear_cache(self):
        self.base._last_x = self.base._last_eval = None
        self._view_last_x = self._view_last_eval = None

    def reset_stats(self):
        self._quarter_calls = 0
        self._quarter_seconds = 0.

    def stats(self):
        return dict(quarter_primal_interpolation_calls=self._quarter_calls,
                    quarter_primal_interpolation_wall_s=self._quarter_seconds,
                    cache_scope="one exact float64 vector; base and appended rows cached together",
                    extra_environment_queries=0)

    def evaluate(self, vector):
        if self.grid == G0_ORIGINAL:
            return self.base.evaluate(vector)
        z = _vector(self.base, vector)
        if self._view_last_x is not None and np.array_equal(z, self._view_last_x):
            return self._view_last_eval
        original = self.base.evaluate(z)
        started = time.perf_counter()
        quarter = quarter_motion_values(self.base, z)
        self._quarter_seconds += time.perf_counter()-started
        self._quarter_calls += 1
        result = dict(original, **quarter)
        for field in ("equality", "inequality"):
            result[field] = np.concatenate((original[field], quarter["quarter_"+field]))
        self._view_last_x, self._view_last_eval = z.copy(), result
        return result

    def dimensions(self, vector):
        original, actual = self.base.evaluate(vector), self.evaluate(vector)
        return dict(grid=self.grid, variable_count=5*(len(self.times)-1),
                    support_count=len(self.times),
                    base_equality_count=len(original["equality"]),
                    base_inequality_count=len(original["inequality"]),
                    equality_count=len(actual["equality"]), inequality_count=len(actual["inequality"]),
                    appended_equality_count=len(actual["equality"])-len(original["equality"]),
                    appended_inequality_count=len(actual["inequality"])-len(original["inequality"]),
                    added_environment_rows=0, added_goal_rows=0)

    def appended_row_metadata(self, vector):
        if self.grid == G0_ORIGINAL:
            return []
        original, quarter = self.base.evaluate(vector), quarter_motion_values(self.base, vector)
        n, h, rows = len(self.times)-1, self.config["support_dt_s"], []
        specifications = [("equality", "lateral", "v_y", "m/s", 1., "v_y = 0", None)]
        specifications += [("inequality", *row) for row in MOTION_ROWS]
        for family_index, (kind, family, quantity, unit, sign, expression, limit_name) in enumerate(specifications):
            offset = 0 if kind == "equality" else (family_index-1)*n*2
            for interval in range(n):
                for fraction_index, fraction in enumerate(QUARTER_FRACTIONS):
                    local = offset+interval*2+fraction_index
                    rows.append(dict(constraint_kind=kind, row_index=len(original[kind])+local,
                                     appended_row_index=local, family=family, physical_quantity=quantity,
                                     physical_unit=unit, sign=sign, expression=expression,
                                     nominal_limit=(0. if limit_name is None else
                                                    (-1. if family.endswith("_lower") else 1.)*self.config[limit_name]),
                                     actual_numerical_tolerance=self.config[kind+"_tolerance"],
                                     interval_index=interval, local_fraction=fraction,
                                     trajectory_time_s=float(self.times[interval]+h*fraction),
                                     knot_side=None, location_type="quarter", source="appended_quarter_motion",
                                     signed_residual_or_margin=float(quarter["quarter_"+kind][local])))
        return rows


def constraint_row_metadata(view, vector):
    """Historical complete row map followed by additional motion metadata."""
    from .gp_se2_diag03_intervals import constraint_time_map
    result = constraint_time_map(view.base, vector)
    result["rows"] = [dict(row, source="original_base") for row in result["rows"]]
    result["rows"].extend(view.appended_row_metadata(vector))
    result.update(view.dimensions(vector))
    return result


def make_quarter_core(problem):
    """Reuse frozen AD primitives, including both body acceleration terms."""
    from . import gp_se2_diag02_ad as ad
    import jax.numpy as jnp
    c = dict(problem.config)
    boundary = jnp.asarray(problem.boundary_pose, dtype=jnp.float64)
    initial = jnp.asarray(problem.initial_twist, dtype=jnp.float64)
    anchors = jnp.asarray(problem._anchors[1:], dtype=jnp.float64)

    def core(vector):
        z = vector.reshape((-1, 5))
        poses = jnp.concatenate((boundary[None], ad.retract_pose(anchors, z[:, :3])))
        future = jnp.stack((z[:, 3], jnp.zeros_like(z[:, 3]), z[:, 4]), axis=-1)
        twists = jnp.concatenate((initial[None], future))
        _, vv, aa = ad.interpolate_interval(
            poses[:-1, None], twists[:-1, None], poses[1:, None], twists[1:, None],
            c["support_dt_s"], jnp.asarray(QUARTER_FRACTIONS, dtype=jnp.float64))
        v, a = vv.reshape((-1, 3)), aa.reshape((-1, 3))
        motion = jnp.concatenate((v[:, 0], c["v_max"]-v[:, 0],
                                  c["w_max"]-v[:, 2], c["w_max"]+v[:, 2],
                                  c["a_v_max"]-a[:, 0], c["a_v_max"]+a[:, 0],
                                  c["a_w_max"]-a[:, 2], c["a_w_max"]+a[:, 2]))
        packed = jnp.concatenate((v[:, 1], motion))
        return packed, packed
    return core


class RefinedDerivativeProvider:
    """Composition, not alteration, of the original provider and quarter AD."""

    def __init__(self, view, base_provider):
        started = time.perf_counter()
        if not isinstance(view, ConstraintView) or not isinstance(base_provider, DerivativeProvider):
            raise TypeError("ConstraintView and original DerivativeProvider required")
        if base_provider.problem is not view.base:
            raise ValueError("base provider must refer to the identical original GPProblem")
        self.view, self.base_provider = view, base_provider
        self.problem = view.base
        self.variable_count = base_provider.variable_count
        count = 2*(len(view.times)-1) if view.grid == G1_QUARTER else 0
        self.quarter_equality_count = count
        self.equality_count = base_provider.equality_count+count
        self.inequality_count = base_provider.inequality_count+8*count
        self._compiled = self._jitted = None
        self._compile_seconds = self._warmup_seconds = 0.
        self._compile_count = self._warmup_count = 0
        self._cache_x = self._cache = None
        self._phases = {name: self._empty_phase() for name in ("runtime", "warmup")}
        if count:
            self._jitted = base_provider._jax.jit(base_provider._jax.jacfwd(make_quarter_core(view.base), has_aux=True))
        self._construction_seconds = time.perf_counter()-started

    @staticmethod
    def _empty_phase():
        return dict(requests=0, cache_hits=0, cache_misses=0, ad_evaluations=0, ad_wall_s=0.)

    def clear_cache(self):
        self.base_provider.clear_cache()
        self._cache_x = self._cache = None

    def reset_stats(self, *, clear_cache=True):
        self.base_provider.reset_stats(clear_cache=clear_cache)
        self._phases["runtime"] = self._empty_phase()
        if clear_cache:
            self.clear_cache()

    def _ensure_quarter(self, vector, phase="runtime"):
        z = self.base_provider._vector(vector)
        stats = self._phases[phase]
        stats["requests"] += 1
        if self._cache_x is not None and np.array_equal(z, self._cache_x):
            stats["cache_hits"] += 1
            return self._cache
        stats["cache_misses"] += 1
        # The original relative-Log/goal branch exclusions remain in force.
        self.base_provider._guard_wrap_cuts(z, phase)
        if self._compiled is None:
            started = time.perf_counter()
            try:
                self._compiled = self._jitted.lower(z).compile()
            except Exception as exc:
                raise DerivativeError(f"quarter JAX compilation failed: {type(exc).__name__}: {exc}") from exc
            self._compile_seconds += time.perf_counter()-started
            self._compile_count += 1
        started = time.perf_counter()
        try:
            jacobian, values = self._compiled(z)
            jacobian, values = np.asarray(jacobian, dtype=np.float64), np.asarray(values, dtype=np.float64)
        except Exception as exc:
            raise DerivativeError(f"quarter JAX evaluation failed: {type(exc).__name__}: {exc}") from exc
        stats["ad_wall_s"] += time.perf_counter()-started
        stats["ad_evaluations"] += 1
        n = self.quarter_equality_count
        if values.shape != (9*n,) or jacobian.shape != (9*n, self.variable_count):
            raise DerivativeError("quarter value/Jacobian shape mismatch")
        if not np.isfinite(values).all() or not np.isfinite(jacobian).all():
            raise DerivativeError("nonfinite quarter AD output; no numerical fallback")
        self._cache_x = z.copy()
        self._cache = dict(quarter_equality=values[:n].copy(), quarter_inequality=values[n:].copy(),
                           quarter_equality_jacobian=jacobian[:n].copy(),
                           quarter_inequality_jacobian=jacobian[n:].copy())
        return self._cache

    def warmup(self, vector):
        self.base_provider.warmup(vector)
        if self.quarter_equality_count:
            started = time.perf_counter()
            self._ensure_quarter(vector, "warmup")
            self._warmup_seconds += time.perf_counter()-started
            self._warmup_count += 1
            self.clear_cache()
        return self.stats()

    def quarter_values(self, vector):
        if not self.quarter_equality_count:
            return dict(quarter_equality=np.empty(0), quarter_inequality=np.empty(0))
        return {key: value.copy() for key, value in self._ensure_quarter(vector).items()
                if not key.endswith("_jacobian")}

    def values(self, vector):
        original = self.base_provider.values(vector)
        if not self.quarter_equality_count:
            return original
        quarter = self.quarter_values(vector)
        result = dict(original, **quarter)
        for field in ("equality", "inequality"):
            result[field] = np.concatenate((original[field], quarter["quarter_"+field]))
        return result

    def objective_gradient(self, vector):
        return self.base_provider.objective_gradient(vector)

    def equality_jacobian(self, vector):
        original = self.base_provider.equality_jacobian(vector)
        if not self.quarter_equality_count:
            return original
        return np.concatenate((original, self._ensure_quarter(vector)["quarter_equality_jacobian"]))

    def inequality_jacobian(self, vector):
        original = self.base_provider.inequality_jacobian(vector)
        if not self.quarter_equality_count:
            return original
        return np.concatenate((original, self._ensure_quarter(vector)["quarter_inequality_jacobian"]))

    def geometry_metadata(self, vector):
        return self.base_provider.geometry_metadata(vector)

    def stats(self):
        base = self.base_provider.stats()
        return dict(base, grid=self.view.grid, equality_count=self.equality_count,
                    inequality_count=self.inequality_count,
                    setup_wall_s=base["setup_wall_s"]+self._construction_seconds,
                    compile_wall_s=base["compile_wall_s"]+self._compile_seconds,
                    warmup_wall_s=base["warmup_wall_s"]+self._warmup_seconds,
                    compile_count=base["compile_count"]+self._compile_count,
                    base_provider=base,
                    quarter_provider=dict(construction_wall_s=self._construction_seconds,
                                          compile_wall_s=self._compile_seconds, compile_count=self._compile_count,
                                          warmup_wall_s=self._warmup_seconds, warmup_count=self._warmup_count,
                                          phases=copy.deepcopy(self._phases),
                                          cache_scope="one exact original float64 vector shared by quarter callbacks",
                                          extra_environment_queries=0, numerical_finite_difference_calls=0),
                    timing_scope="top-level setup/compile/warmup sum base+quarter; warmup includes compilation; nested times overlap")

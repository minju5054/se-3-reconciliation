"""Read-only interpolation consistency checks for GP-SE2-DIAG-03.

These are diagnostics of fixed support arrays, never trajectory optimization.
The independent algebraic path uses cubic coefficients instead of the production
Hermite basis. It deliberately shares the original SE(2) Exp/Log/J_r convention;
finite differences then check the pose/body-twist time derivative independently
of the analytic right-Jacobian derivative. Physical acceptance is not changed.
"""
from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from .gp_se2 import (interpolate_interval, right_jacobian,
                     right_jacobian_directional)
from .se2 import compose_poses, relative_pose, se2_exp, se2_log, wrap_angle


FIXED_INTERIOR_FRACTIONS = (0.125, 0.25, 0.5, 0.75, 0.875)
FD_EPSILONS_S = (1e-4, 3e-5, 1e-5)
DIAGNOSTIC_TOLERANCES = {
    "coefficient_pose_absolute": [1e-9, 1e-9, 1e-9],
    "coefficient_twist_absolute": [1e-9, 1e-9, 1e-9],
    "coefficient_acceleration_absolute": [1e-8, 1e-8, 1e-8],
    "finite_difference_twist_absolute": [2e-6, 2e-6, 2e-6],
    "finite_difference_acceleration_absolute": [2e-5, 2e-5, 2e-5],
}
UNITS = {"pose": ["m", "m", "rad"],
         "body_twist": ["m/s", "m/s", "rad/s"],
         "body_acceleration": ["m/s^2", "m/s^2", "rad/s^2"]}


def derivative_protocol() -> dict:
    """Fresh JSON-compatible protocol; callers cannot mutate module defaults."""
    return {
        "fixed_interior_fractions": list(FIXED_INTERIOR_FRACTIONS),
        "finite_difference_epsilons_s": list(FD_EPSILONS_S),
        "tolerances": {k: list(v) for k, v in DIAGNOSTIC_TOLERANCES.items()},
        "units": {k: list(v) for k, v in UNITS.items()},
        "acceptance_rule": "Both finest valid epsilon checks must pass componentwise; fixed probes require both.",
        "algebraic_independence": "Cubic coefficients, not production Hermite basis; original Exp/Log/J_r primitives shared.",
        "pose_derivative": "Central world XY difference rotated by center yaw; central wrapped yaw difference.",
        "acceleration_derivative": "Central difference of coefficient-reconstructed body twist; no analytic D J_r in FD estimate.",
        "boundary_policy": "Strictly interior probes only; no finite difference across support knots or local yaw cut.",
        "knot_policy": "Separate production left/right interval limits; acceleration jumps are not averaged.",
        "physical_acceptance_modified": False,
        "new_optimization": False,
    }


DERIVATIVE_PROTOCOL = derivative_protocol()


def _state(value, name: str) -> np.ndarray:
    a = np.asarray(value, dtype=np.float64)
    if a.shape != (3,) or not np.all(np.isfinite(a)):
        raise ValueError(f"{name} must be finite shape (3,)")
    return a


def hermite_coefficients(pose0, twist0, pose1, twist1, h: float) -> np.ndarray:
    """Return c1,c2,c3 in xi(u)=c1*u+c2*u²+c3*u³, without GP sampling."""
    if not np.isfinite(h) or h <= 0:
        raise ValueError("h must be finite and positive")
    p0, v0, p1, v1 = (_state(x, n) for x, n in
                       zip((pose0, twist0, pose1, twist1),
                           ("pose0", "twist0", "pose1", "twist1")))
    xi1 = se2_log(relative_pose(p0, p1))
    d1 = np.linalg.solve(right_jacobian(xi1), v1)
    return np.stack((h*v0, 3*xi1-2*h*v0-h*d1,
                     -2*xi1+h*v0+h*d1))


def _coefficient_terms(pose0, coefficients, h: float, fraction):
    p0 = _state(pose0, "pose0")
    c = np.asarray(coefficients, dtype=np.float64)
    u = np.asarray(fraction, dtype=np.float64)
    if c.shape != (3, 3) or not np.all(np.isfinite(c)):
        raise ValueError("coefficients must be finite shape (3,3)")
    if not np.isfinite(h) or h <= 0:
        raise ValueError("h must be finite and positive")
    if not np.all(np.isfinite(u)) or np.any((u < 0) | (u > 1)):
        raise ValueError("fraction must lie in [0,1]")
    u = u[..., None]
    xi = c[0]*u+c[1]*u**2+c[2]*u**3
    dxi = (c[0]+2*c[1]*u+3*c[2]*u**2)/h
    ddxi = (2*c[1]+6*c[2]*u)/h**2
    j = right_jacobian(xi)
    velocity = np.einsum("...ij,...j->...i", j, dxi)
    return compose_poses(p0, se2_exp(xi)), velocity, xi, dxi, ddxi, j


def coefficient_interpolation(pose0, coefficients, h: float, fraction):
    """Independent polynomial evaluation with original Lie-group primitives."""
    pose, velocity, xi, dxi, ddxi, j = _coefficient_terms(pose0, coefficients, h, fraction)
    acceleration = (np.einsum("...ij,...j->...i", right_jacobian_directional(xi, dxi), dxi)
                    + np.einsum("...ij,...j->...i", j, ddxi))
    return pose, velocity, acceleration


def finite_difference_at(pose0, coefficients, h: float, fraction: float,
                         epsilon_s: float) -> dict:
    """Time derivatives from poses/twists; refused when a probe touches a knot.

    World coordinate differencing avoids reusing the analytic body-Jacobian
    formula for the pose derivative. Yaw differences are taken relative to the
    center pose so a display wrap at +/-pi does not create a spurious rotation.
    """
    if not np.isfinite(epsilon_s) or epsilon_s <= 0:
        raise ValueError("epsilon_s must be finite and positive")
    if not np.isfinite(h) or h <= 0 or not np.isfinite(fraction) or not 0 <= fraction <= 1:
        raise ValueError("positive h and fraction in [0,1] required")
    du = epsilon_s/h
    if fraction-du <= 0 or fraction+du >= 1:
        return {"available": False, "reason": "PROBE_WOULD_TOUCH_OR_CROSS_KNOT",
                "epsilon_s": float(epsilon_s)}
    center = _coefficient_terms(pose0, coefficients, h, fraction)
    before = _coefficient_terms(pose0, coefficients, h, fraction-du)
    after = _coefficient_terms(pose0, coefficients, h, fraction+du)
    yaw_before = float(wrap_angle(before[0][2]-center[0][2]))
    yaw_after = float(wrap_angle(after[0][2]-center[0][2]))
    if max(abs(yaw_before), abs(yaw_after)) >= np.pi-1e-8:
        return {"available": False, "reason": "PROBE_LOCAL_YAW_BRANCH_CUT",
                "epsilon_s": float(epsilon_s)}
    world_velocity = (after[0][:2]-before[0][:2])/(2*epsilon_s)
    yaw = center[0][2]
    rotation_inverse = np.array([[np.cos(yaw), np.sin(yaw)],
                                 [-np.sin(yaw), np.cos(yaw)]])
    velocity = np.r_[rotation_inverse@world_velocity,
                     (yaw_after-yaw_before)/(2*epsilon_s)]
    acceleration = (after[1]-before[1])/(2*epsilon_s)
    return {"available": True, "epsilon_s": float(epsilon_s),
            "twist_fd": velocity.tolist(), "acceleration_fd": acceleration.tolist(),
            "probe_fraction_before": float(fraction-du),
            "probe_fraction_after": float(fraction+du)}


def _pose_error(left, right) -> np.ndarray:
    error = np.asarray(left)-np.asarray(right)
    error[..., 2] = wrap_angle(error[..., 2])
    return np.abs(error)


def _support_arrays(times, poses, twists):
    t = np.asarray(times, dtype=np.float64)
    p = np.asarray(poses, dtype=np.float64)
    v = np.asarray(twists, dtype=np.float64)
    if (t.ndim != 1 or len(t) < 2 or p.shape != (len(t), 3) or v.shape != p.shape
            or not all(np.all(np.isfinite(x)) for x in (t, p, v))
            or np.any(np.diff(t) <= 0)
            or not np.allclose(np.diff(t), t[1]-t[0], atol=1e-12)):
        raise ValueError("finite, uniform ordered support times and matching (N,3) arrays required")
    return t, p, v


def check_interpolation_consistency(times, poses, twists, *,
                                    witness_points: Sequence[dict] | None = None) -> dict:
    """Check every interval and preserved knot side, plus observed witnesses.

    Witnesses carry ``interval_index`` and ``local_fraction`` and an optional
    label. Their selection is post hoc; predetermined interior probes remain
    distinguishable in all records. No support/vector value is changed.
    """
    t, p, v = _support_arrays(times, poses, twists)
    points: list[dict[float, set[str]]] = []
    for _ in range(len(t)-1):
        points.append({u: {"FIXED_INTERIOR"} for u in FIXED_INTERIOR_FRACTIONS})
    for point in witness_points or []:
        i = int(point["interval_index"])
        u = float(point["local_fraction"])
        if i != point["interval_index"] or not 0 <= i < len(t)-1 or not np.isfinite(u) or not 0 <= u <= 1:
            raise ValueError("witness must identify an existing interval and fraction in [0,1]")
        points[i].setdefault(u, set()).add("POST_HOC_WITNESS:"+str(point.get("label", "observed_violation")))

    result = {"protocol": derivative_protocol(), "interval_coefficients": [],
              "coefficient_checks": [], "derivative_checks": [], "knot_limits": []}
    coefficient_max = {"pose": np.zeros(3), "twist": np.zeros(3), "acceleration": np.zeros(3)}
    fd_max = {"twist": np.zeros(3), "acceleration": np.zeros(3)}
    fd_by_epsilon = {str(eps): {"twist": np.zeros(3), "acceleration": np.zeros(3)}
                     for eps in FD_EPSILONS_S}
    coefficient_pass = True
    derivative_pass = True
    incomplete = 0
    fixed_incomplete = 0
    # Match sample_gp's original uniform interval duration; do not turn the
    # last-bit differences in decimal support timestamps into new h values.
    h = float(t[1]-t[0])
    for i in range(len(t)-1):
        c = hermite_coefficients(p[i], v[i], p[i+1], v[i+1], h)
        result["interval_coefficients"].append({"interval_index": i,
            "start_time_s": float(t[i]), "duration_s": h,
            "c1": c[0].tolist(), "c2": c[1].tolist(), "c3": c[2].tolist()})
        check_fractions = sorted(set(points[i]) | {0., 1.})
        independent = coefficient_interpolation(p[i], c, h, check_fractions)
        production = interpolate_interval(p[i], v[i], p[i+1], v[i+1], h, check_fractions)
        for j, u in enumerate(check_fractions):
            errors = {"pose": _pose_error(independent[0][j], production[0][j]),
                      "twist": np.abs(independent[1][j]-production[1][j]),
                      "acceleration": np.abs(independent[2][j]-production[2][j])}
            passes = {name: bool(np.all(error <= DIAGNOSTIC_TOLERANCES[f"coefficient_{name}_absolute"]))
                      for name, error in errors.items()}
            coefficient_pass &= all(passes.values())
            for name, error in errors.items():
                coefficient_max[name] = np.maximum(coefficient_max[name], error)
            result["coefficient_checks"].append({"interval_index": i, "local_fraction": u,
                "time_s": float(t[i]+u*h), "knot_side": "right" if u == 0 else "left" if u == 1 else None,
                "pose_absolute_error": errors["pose"].tolist(),
                "twist_absolute_error": errors["twist"].tolist(),
                "acceleration_absolute_error": errors["acceleration"].tolist(),
                "component_group_pass": passes, "consistent": all(passes.values())})
            if u not in points[i]:
                continue
            eps_records = []
            for eps in FD_EPSILONS_S:
                fd = finite_difference_at(p[i], c, h, u, eps)
                if fd["available"]:
                    errv = np.abs(np.asarray(fd["twist_fd"])-production[1][j])
                    erra = np.abs(np.asarray(fd["acceleration_fd"])-production[2][j])
                    fd.update({"twist_absolute_error": errv.tolist(),
                               "acceleration_absolute_error": erra.tolist(),
                               "twist_component_pass": (errv <= DIAGNOSTIC_TOLERANCES["finite_difference_twist_absolute"]).tolist(),
                               "acceleration_component_pass": (erra <= DIAGNOSTIC_TOLERANCES["finite_difference_acceleration_absolute"]).tolist()})
                    for name, error in (("twist", errv), ("acceleration", erra)):
                        fd_max[name] = np.maximum(fd_max[name], error)
                        fd_by_epsilon[str(eps)][name] = np.maximum(fd_by_epsilon[str(eps)][name], error)
                eps_records.append(fd)
            # A coarser check need not satisfy the finest-step tolerance; its
            # actual error is retained to expose truncation/roundoff trends.
            required = eps_records[-2:]
            available = all(row["available"] for row in required)
            consistent = (all(all(row["twist_component_pass"]) and all(row["acceleration_component_pass"])
                              for row in required) if available else None)
            if available:
                derivative_pass &= bool(consistent)
            else:
                incomplete += 1
                fixed_incomplete += int("FIXED_INTERIOR" in points[i][u])
            result["derivative_checks"].append({"interval_index": i, "local_fraction": u,
                "time_s": float(t[i]+u*h), "probe_sources": sorted(points[i][u]),
                "analytic_twist": production[1][j].tolist(),
                "analytic_acceleration": production[2][j].tolist(),
                "epsilon_checks": eps_records,
                "consistent": consistent,
                "status": ("CONSISTENT" if consistent else "INCONSISTENT") if available else "NOT_VERIFIED_BOUNDARY_OR_BRANCH_GUARD"})

    for k in range(1, len(t)-1):
        left = interpolate_interval(p[k-1], v[k-1], p[k], v[k], h, 1.)
        right = interpolate_interval(p[k], v[k], p[k+1], v[k+1], h, 0.)
        result["knot_limits"].append({"knot_index": k, "time_s": float(t[k]),
            "left_interval_index": k-1, "right_interval_index": k,
            "left_pose": left[0].tolist(), "right_pose": right[0].tolist(),
            "left_twist": left[1].tolist(), "right_twist": right[1].tolist(),
            "left_acceleration": left[2].tolist(), "right_acceleration": right[2].tolist(),
            "pose_periodic_absolute_difference": _pose_error(left[0], right[0]).tolist(),
            "twist_absolute_difference": np.abs(left[1]-right[1]).tolist(),
            "right_minus_left_acceleration": (right[2]-left[2]).tolist(),
            "finite_difference_across_knot_performed": False})
    result["summary"] = {
        "coefficient_cross_check_consistent": coefficient_pass,
        "interpolation_derivative_consistent": bool(derivative_pass and fixed_incomplete == 0),
        "interval_count": len(t)-1,
        "fixed_interior_probe_count": (len(t)-1)*len(FIXED_INTERIOR_FRACTIONS),
        "total_unique_derivative_probes": len(result["derivative_checks"]),
        "unverified_boundary_or_branch_probe_count": incomplete,
        "unverified_fixed_probe_count": fixed_incomplete,
        "maximum_coefficient_errors": {k: value.tolist() for k, value in coefficient_max.items()},
        "maximum_fd_errors_all_epsilons": {k: value.tolist() for k, value in fd_max.items()},
        "maximum_fd_errors_by_epsilon": {e: {k: value.tolist() for k, value in groups.items()}
                                          for e, groups in fd_by_epsilon.items()},
        "support_arrays_modified": False,
        "new_optimization": False,
        "limitation": "Finite-difference and coefficient agreement at recorded probes does not prove continuous-time feasibility; shared Exp/Log/J_r primitives are not a second complete SE(2) library.",
    }
    return result

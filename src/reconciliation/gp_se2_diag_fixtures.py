"""Optimizer-free, synthetic SE(2) GP representation diagnostics.

These fixtures are mathematical tests, not navigation performance evidence.
The original GP implementation and acceptance thresholds are used unchanged.
The reference ODE and world-coordinate pose differentiation below do not use
the GP Jacobian, exponential, logarithm, or reported twist/acceleration.
"""
from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
from typing import Callable

import numpy as np
from scipy.integrate import solve_ivp

from .gp_se2 import gp_residual, interpolate_interval, sample_gp
from .gp_se2_formulation import GPProblem, resolve_config

FIXTURE_NAMES = ("S0", "S1", "S2", "S3")
BOUNDARY = np.array([1.7, -0.8, 0.63])
PERTURBATION_SEED = 20260918
PERTURBATION_SCALES = np.array([0.01, 0.01, np.deg2rad(0.5), 0.02, 0.02])


def _wrapped(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def truth_twist(name: str, t) -> np.ndarray:
    """Body velocities in [m/s, m/s, rad/s]; t is measured in seconds."""
    t = np.asarray(t, float)
    if name not in FIXTURE_NAMES:
        raise ValueError(f"unknown fixture {name}")
    v = np.full_like(t, 0.3)
    w = np.zeros_like(t) if name == "S0" else np.full_like(t, 0.4)
    if name == "S2":
        v = 0.3 * (1 - t / 3.0)
        w = 0.4 * (1 - t / 3.0)
    elif name == "S3":
        w = 0.2 + 0.1 * t
    return np.stack([v, np.zeros_like(t), w], axis=-1)


def truth_acceleration(name: str, t) -> np.ndarray:
    """Time derivative of body twist in [m/s^2, m/s^2, rad/s^2]."""
    t = np.asarray(t, float)
    result = np.zeros(t.shape + (3,))
    if name == "S2":
        result[...] = [-0.3 / 3.0, 0, -0.4 / 3.0]
    elif name == "S3":
        result[..., 2] = 0.1
    elif name not in FIXTURE_NAMES:
        raise ValueError(f"unknown fixture {name}")
    return result


def closed_form_pose(name: str, t) -> np.ndarray:
    """Independent scalar trigonometric solution for S0/S1/S2.

    q has units s; q_dot is dimensionless; q_ddot has units 1/s. The fixed
    direction eta=[0.3 m/s, 0 m/s, 0.4 rad/s] gives q*eta=[m,m,rad].
    No repository SE(2) operation is used to construct the truth.
    """
    if name not in ("S0", "S1", "S2"):
        raise ValueError("closed form is defined for S0/S1/S2 only")
    t = np.asarray(t, float)
    q = t - t * t / 6.0 if name == "S2" else t
    theta0 = BOUNDARY[2]
    if name == "S0":
        x = BOUNDARY[0] + 0.3 * q * np.cos(theta0)
        y = BOUNDARY[1] + 0.3 * q * np.sin(theta0)
        yaw = np.full_like(t, theta0)
    else:
        yaw = theta0 + 0.4 * q
        x = BOUNDARY[0] + (0.3 / 0.4) * (np.sin(yaw) - np.sin(theta0))
        y = BOUNDARY[1] - (0.3 / 0.4) * (np.cos(yaw) - np.cos(theta0))
    return np.stack([x, y, _wrapped(yaw)], axis=-1)


def independent_ode_reference(name: str, *, tighter: bool = False):
    """DOP853 integrates world unicycle equations directly, with dense output."""
    def rhs(t, pose):
        twist = truth_twist(name, t)
        return [twist[0] * np.cos(pose[2]), twist[0] * np.sin(pose[2]), twist[2]]

    tolerance = 2.3e-14 if tighter else 5e-13
    solution = solve_ivp(rhs, (0.0, 3.0), BOUNDARY.copy(), method="DOP853",
                         rtol=tolerance, atol=tolerance, dense_output=True,
                         max_step=0.025 if tighter else 0.1)
    if not solution.success:
        raise RuntimeError(solution.message)
    metadata = {"method": "independent world-coordinate DOP853",
                "rtol": tolerance, "atol": tolerance,
                "max_step_s": 0.025 if tighter else 0.1,
                "nfev": solution.nfev, "success": bool(solution.success)}
    return lambda t: solution.sol(np.asarray(t, float)).T, metadata


def independent_pose_derivatives(pose_at: Callable, t, step_s: float = 1e-4,
                                offsets=(-2, -1, 0, 1, 2)):
    """Differentiate poses in world coordinates and rotate to the body frame.

    The five-point stencil can be central or one-sided. The second derivative
    of body position is R^T p_ddot - omega*J*(R^T p_dot). This path never reads
    the GP-returned velocities or derivatives and never calls an SE(2) log.
    Subtracting the centre pose before weighted sums reduces cancellation.
    """
    t = np.asarray(t, float)
    offsets = np.asarray(offsets, float)
    if len(offsets) != 5 or len(np.unique(offsets)) != 5 or step_s <= 0:
        raise ValueError("five distinct stencil offsets and positive step required")
    matrix = np.stack([offsets ** k for k in range(5)])
    first = np.linalg.solve(matrix, [0, 1, 0, 0, 0]) / step_s
    second = np.linalg.solve(matrix, [0, 0, 2, 0, 0]) / step_s**2
    centre = pose_at(t)
    values = np.stack([pose_at(t + offset * step_s) for offset in offsets], axis=-2)
    delta = values - centre[..., None, :]
    delta[..., 2] = _wrapped(delta[..., 2])
    d = np.einsum("...ij,i->...j", delta, first)
    dd = np.einsum("...ij,i->...j", delta, second)
    c, s = np.cos(centre[..., 2]), np.sin(centre[..., 2])
    body = np.stack([c*d[..., 0]+s*d[..., 1], -s*d[..., 0]+c*d[..., 1], d[..., 2]], axis=-1)
    acceleration = np.stack([c*dd[..., 0]+s*dd[..., 1]+d[..., 2]*body[..., 1],
                             -s*dd[..., 0]+c*dd[..., 1]-d[..., 2]*body[..., 0],
                             dd[..., 2]], axis=-1)
    return body, acceleration


def make_fixture(name: str, config: dict | None = None) -> dict:
    c = resolve_config(config)
    if c["horizon_s"] != 3.0 or c["support_dt_s"] != 0.1:
        raise ValueError("diagnostic fixtures require the actual 3 s / 31 support size")
    times = np.linspace(0, 3, 31)
    ode, metadata = independent_ode_reference(name)
    poses = ode(times) if name == "S3" else closed_form_pose(name, times)
    twists = truth_twist(name, times)
    rng = np.random.default_rng(PERTURBATION_SEED)
    delta = rng.normal(size=(30, 5)) * PERTURBATION_SCALES
    return {"fixture": name, "evidence_kind": "SYNTHETIC_MATHEMATICAL_DIAGNOSTIC",
            "support_times": times, "support_poses": poses, "support_twists": twists,
            "boundary_pose": BOUNDARY.copy(), "initial_twist": twists[0].copy(),
            "common_reference": poses[1:].copy(), "goal_pose": poses[-1].copy(),
            "config": c, "ode_reference": metadata,
            "perturbation_seed": PERTURBATION_SEED,
            "perturbation_scales": PERTURBATION_SCALES.copy(), "perturbation_vector": delta.ravel(),
            "perturbation_convention": "add to original right-local optimization vector; first pose/twist fixed; normal draws, scales are standard deviations; same fixed vector for S0/S1/S2",
            "frame": "world, X_world_agent; right local perturbations; body x forward/y left, yaw CCW",
            "pose_units": ["m", "m", "rad"], "twist_units": ["m/s", "m/s", "rad/s"],
            "acceleration_units": ["m/s^2", "m/s^2", "rad/s^2"],
            "time_units": "s", "q_units": "s", "q_dot_units": "dimensionless", "q_ddot_units": "1/s",
            "synthetic_environment": "constant 10 m clearance and known-workspace margin; not Hospital evidence"}


def make_fixture_problem(name: str, config: dict | None = None) -> tuple[GPProblem, dict]:
    f = make_fixture(name, config)
    def free_space(xy):
        return np.full(np.asarray(xy).shape[:-1], 10.0)
    problem = GPProblem(f["boundary_pose"], f["initial_twist"], f["common_reference"],
                        f["goal_pose"], f["config"], obstacle_clearance=free_space,
                        workspace_margin=free_space, include_obstacles=True)
    vector = problem.vector(f["support_poses"], f["support_twists"])
    f["known_vector"] = vector
    f["perturbed_vector"] = vector + f["perturbation_vector"]
    f["variable_count"] = len(vector)
    return problem, f


def _max_abs(x):
    return float(np.max(np.abs(x), initial=0))


def evaluate_fixture(name: str, config: dict | None = None) -> dict:
    problem, f = make_fixture_problem(name, config)
    times, p, v = f["support_times"], f["support_poses"], f["support_twists"]
    query = np.linspace(0, 3, 3001)
    ode, ode_metadata = independent_ode_reference(name)
    tighter, tighter_metadata = independent_ode_reference(name, tighter=True)
    truth = ode(query) if name == "S3" else closed_form_pose(name, query)
    gp, gv, ga = sample_gp(times, p, v, query)
    truev, truea = truth_twist(name, query), truth_acceleration(name, query)
    # Every interval gets nine independent interior derivative checks and both
    # one-sided endpoint checks. No stencil crosses a support knot.
    fractions = np.arange(1, 10) / 10
    interior_t = (times[:-1, None] + 0.1 * fractions).ravel()
    pose_at = lambda t: sample_gp(times, p, v, t)[0]
    fdv, fda = independent_pose_derivatives(pose_at, interior_t, step_s=.001)
    _, iv, ia = sample_gp(times, p, v, interior_t)
    endpoints, side_fdv, side_fda, side_v, side_a = [], [], [], [], []
    for interval in range(30):
        interval_pose = lambda q, i=interval: interpolate_interval(p[i], v[i], p[i+1], v[i+1], 0.1, np.asarray(q)/0.1)[0]
        for fraction, stencil in ((0., (0, 1, 2, 3, 4)), (1., (-4, -3, -2, -1, 0))):
            sv, sa = independent_pose_derivatives(interval_pose, fraction*0.1, step_s=.001, offsets=stencil)
            _, actual_v, actual_a = interpolate_interval(p[interval], v[interval], p[interval+1], v[interval+1], .1, fraction)
            endpoints.append(times[interval]+fraction*.1)
            side_fdv.append(sv); side_fda.append(sa); side_v.append(actual_v); side_a.append(actual_a)
    side_fdv, side_fda, side_v, side_a = map(np.asarray, (side_fdv, side_fda, side_v, side_a))
    residuals = gp_residual(p[:-1], v[:-1], p[1:], v[1:], .1)
    expected_residual = np.zeros_like(residuals)
    if name == "S2":
        acceleration = np.array([-.3/3, 0, -.4/3])
        expected_residual[:] = np.r_[.5*.1**2*acceleration, .1*acceleration]
    dense = problem.dense_report(f["known_vector"])
    pp, vv, _ = sample_gp(times, p, v, times)
    position_error = np.linalg.norm(gp[:, :2]-truth[:, :2], axis=1)
    yaw_error = np.abs(_wrapped(gp[:, 2]-truth[:, 2]))
    truth_acceleration_error = np.max(np.abs(ga-truea), axis=0)
    summary = {
        "fixture": name, "evidence_kind": f["evidence_kind"], "support_count": 31, "variable_count": 150,
        "horizon_s": 3.0, "support_dt_s": .1, "additional_dense_dt_s": .001,
        "known_seed_original_dense_feasible": bool(dense["feasible"]),
        "support_pose_max_absolute_error": _max_abs(pp-p), "support_twist_max_absolute_error": _max_abs(vv-v),
        "position_error_max_m": float(np.max(position_error)), "yaw_error_max_rad": float(np.max(yaw_error)),
        "lateral_velocity_max_m_s": _max_abs(gv[:, 1]),
        "linear_speed_min_m_s": float(np.min(gv[:, 0])), "linear_speed_max_m_s": float(np.max(gv[:, 0])),
        "angular_speed_max_rad_s": _max_abs(gv[:, 2]),
        "linear_acceleration_max_m_s2": _max_abs(ga[:, 0]), "angular_acceleration_max_rad_s2": _max_abs(ga[:, 2]),
        "body_acceleration_error_max_by_component": truth_acceleration_error,
        "independent_interior_twist_error_max_by_component": np.max(np.abs(fdv-iv), axis=0),
        "independent_interior_acceleration_error_max_by_component": np.max(np.abs(fda-ia), axis=0),
        "independent_one_sided_twist_error_max_by_component": np.max(np.abs(side_fdv-side_v), axis=0),
        "independent_one_sided_acceleration_error_max_by_component": np.max(np.abs(side_fda-side_a), axis=0),
        "pose_fd_step_s": .001, "pose_fd_method": "five-point world-coordinate pose first/second differences, rotated analytically; 270 interior + 60 one-sided checks",
        "ode_vs_tighter_position_error_max_m": float(np.max(np.linalg.norm(ode(query)[:, :2]-tighter(query)[:, :2], axis=1))),
        "ode_vs_tighter_yaw_error_max_rad": _max_abs(_wrapped(ode(query)[:, 2]-tighter(query)[:, 2])),
        "closed_form_vs_ode_max_by_component": np.max(np.abs(truth-ode(query)), axis=0) if name != "S3" else None,
        "gp_residual_max_by_component": np.max(np.abs(residuals), axis=0),
        "known_analytic_gp_residual_error_max": _max_abs(residuals-expected_residual) if name != "S3" else None,
        "initial_twist": v[0], "terminal_twist": v[-1],
        "equality_tolerance_m_s": problem.config["equality_tolerance"],
        "inequality_tolerance": problem.config["inequality_tolerance"],
        "interpretation": ("Known same-algebra-direction feasible witness; no claim of full-objective global optimality" if name != "S3" else
                           "Variable-curvature sampled-unicycle GP reconstruction diagnostic; mismatch is not impossibility of all feasible GP trajectories")}
    return {"fixture_input": f, "summary": summary, "ode_reference": ode_metadata,
            "tighter_ode_reference": tighter_metadata, "original_dense_report": dense,
            "query_times_s": query, "truth_poses_world": truth, "truth_body_twists": truev,
            "truth_body_accelerations": truea, "gp_poses_world": gp, "gp_body_twists": gv,
            "gp_body_accelerations": ga, "position_error_m": position_error, "yaw_error_rad": yaw_error,
            "interior_derivative_times_s": interior_t, "independent_body_twists": fdv,
            "independent_body_accelerations": fda, "gp_interior_twists": iv, "gp_interior_accelerations": ia,
            "one_sided_times_s": np.asarray(endpoints), "one_sided_independent_twists": side_fdv,
            "one_sided_independent_accelerations": side_fda, "one_sided_gp_twists": side_v,
            "one_sided_gp_accelerations": side_a, "gp_residuals": residuals}


def evaluate_blind_spot(config: dict | None = None) -> dict:
    c = resolve_config(config)
    p0, p1 = np.array([0., 0., 0.]), np.array([.03, 0., 0.])
    v0 = v1 = np.array([.3, 0., .08])
    h = .1
    fractions = np.linspace(0, 1, 1001)
    pose, velocity, acceleration = interpolate_interval(p0, v0, p1, v1, h, fractions)
    pose_at = lambda t: interpolate_interval(p0, v0, p1, v1, h, np.asarray(t)/h)[0]
    qt = fractions[5:-5] * h
    fdv, fda = independent_pose_derivatives(pose_at, qt, step_s=1e-4)
    points = np.array([0., .25, .5, .75, 1.])
    _, pv, _ = interpolate_interval(p0, v0, p1, v1, h, points)
    tolerance = c["equality_tolerance"]
    max_index = int(np.argmax(np.abs(velocity[:, 1])))
    summary = {"fixture": "COLLOCATION_BLIND_SPOT", "evidence_kind": "SYNTHETIC_NONFEASIBLE_GP_INTERPOLATION_EXAMPLE",
               "h_s": h, "fractions": points, "lateral_at_prescribed_points_m_s": pv[:, 1],
               "prescribed_point_pass": np.abs(pv[:, 1]) <= tolerance,
               "endpoint_midpoint_lateral_pass": bool(np.all(np.abs(pv[[0, 2, 4], 1]) <= tolerance)),
               "quarter_point_lateral_fail": bool(np.all(np.abs(pv[[1, 3], 1]) > tolerance)),
               "dense_lateral_pass": bool(np.max(np.abs(velocity[:, 1])) <= tolerance),
               "maximum_absolute_lateral_velocity_m_s": float(np.abs(velocity[max_index, 1])),
               "maximum_violation_m_s": float(max(0, abs(velocity[max_index, 1])-tolerance)),
               "maximum_violation_time_s": float(fractions[max_index]*h),
               "independent_pose_derivative_velocity_error_max": _max_abs(fdv-velocity[5:-5]),
               "independent_pose_derivative_acceleration_error_max": _max_abs(fda-acceleration[5:-5]),
               "independent_pose_derivative_step_s": 1e-4,
               "equality_tolerance_m_s": tolerance,
               "interpretation": "Lateral pass at support and midpoint does not imply interval-wide pass; not evidence of historical rejection counts or feasible motion"}
    return {"summary": summary, "input": {"pose0": p0, "pose1": p1, "twist0": v0, "twist1": v1, "h_s": h},
            "fractions": fractions, "times_s": fractions*h, "poses_world": pose, "body_twists": velocity,
            "body_accelerations": acceleration, "independent_times_s": qt,
            "independent_body_twists": fdv, "independent_body_accelerations": fda}


def _jsonable(value):
    if isinstance(value, np.ndarray): return _jsonable(value.tolist())
    if isinstance(value, np.generic): return _jsonable(value.item())
    if isinstance(value, float) and not np.isfinite(value): return None
    if isinstance(value, dict): return {key: _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)): return [_jsonable(item) for item in value]
    return value


def _write_json(path: Path, data):
    with path.open("x") as stream:
        json.dump(_jsonable(data), stream, indent=2, allow_nan=False)
        stream.write("\n")


def write_optimizer_free_fixtures(output: str | Path, config: dict | None = None) -> dict:
    """Write a new exclusive synthetic directory; never invokes an optimizer."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    source_dir = output / "implementation_sources"
    source_dir.mkdir()
    source_hashes = {}
    for name in ("gp_se2_diag_fixtures.py", "gp_se2.py", "gp_se2_formulation.py", "se2.py"):
        source = Path(__file__).parent / name
        payload = source.read_bytes()
        (source_dir / name).write_bytes(payload)
        source_hashes[name] = hashlib.sha256(payload).hexdigest()
    _write_json(output / "source.json", {"source_sha256": source_hashes,
                "resolved_config": resolve_config(config), "optimizer_executed": False,
                "raw_inputs": None, "evidence_kind": "SYNTHETIC_MATHEMATICAL_DIAGNOSTIC",
                "nonfinite_serialization": "null, never zero"})
    summaries = []
    for name in FIXTURE_NAMES:
        result = evaluate_fixture(name, config)
        destination = output / name
        destination.mkdir()
        _write_json(destination / "input.json", result["fixture_input"])
        _write_json(destination / "numeric_results.json", result)
        summary = result["summary"]
        summaries.append(summary)
        t = result["query_times_s"]
        truth, gp = result["truth_poses_world"], result["gp_poses_world"]
        f, ax = plt.subplots(1, 3, figsize=(15, 4.4))
        ax[0].plot(truth[:, 0], truth[:, 1], label="independent unicycle truth", linewidth=3)
        ax[0].plot(gp[:, 0], gp[:, 1], "--", label="GP reconstruction")
        support = result["fixture_input"]["support_poses"]
        ax[0].scatter(support[:, 0], support[:, 1], s=9, color="black", label="31 supports")
        ax[0].set(xlabel="world x [m]", ylabel="world y [m]", title="World XY; equal aspect")
        ax[0].set_aspect("equal", adjustable="datalim")
        ax[1].plot(t, result["position_error_m"])
        ax[1].set(xlabel="time [s]", ylabel="position error [m]", title="GP minus truth")
        ax[2].plot(t, result["yaw_error_rad"])
        ax[2].set(xlabel="time [s]", ylabel="absolute yaw error [rad]", title="GP minus truth")
        ax[0].legend(fontsize=8)
        for a in ax: a.grid(alpha=.25)
        f.suptitle(f"{name} — SYNTHETIC mathematical diagnostic; not research performance evidence")
        f.tight_layout(); f.savefig(destination / "truth_vs_gp_reconstruction.png", dpi=160); plt.close(f)
        f, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
        vy = result["gp_body_twists"][:, 1]
        tolerance = result["fixture_input"]["config"]["equality_tolerance"]
        ax[0].plot(t, vy, label="GP lateral velocity")
        ax[0].plot(result["interior_derivative_times_s"], result["independent_body_twists"][:, 1], ".", ms=2, label="independent pose differentiation")
        for sign in (-1, 1): ax[0].axhline(sign*tolerance, color="red", linestyle=":")
        ax[0].set(ylabel="body v_y [m/s]", title=f"Lateral tolerance ±{tolerance:g} m/s")
        for index, component, limit, unit in ((1, 0, 2., "m/s²"), (2, 2, 5., "rad/s²")):
            ax[index].plot(t, result["gp_body_accelerations"][:, component], label="GP analytic acceleration")
            ax[index].plot(t, result["truth_body_accelerations"][:, component], "--", label="true acceleration")
            ax[index].plot(result["interior_derivative_times_s"], result["independent_body_accelerations"][:, component], ".", ms=2, label="independent pose differentiation")
            for sign in (-1, 1): ax[index].axhline(sign*limit, color="red", linestyle=":")
            ax[index].set(ylabel=f"{'linear' if component == 0 else 'angular'} acceleration [{unit}]")
        for a in ax: a.legend(fontsize=8); a.grid(alpha=.25)
        ax[-1].set_xlabel("time [s]")
        f.suptitle(f"{name} — SYNTHETIC motion / derivative checks; limits unchanged")
        f.tight_layout(); f.savefig(destination / "lateral_and_acceleration_vs_time.png", dpi=160); plt.close(f)
        f, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        for index, component, unit in ((0, 0, "m/s²"), (1, 2, "rad/s²")):
            ax[index].plot(t, result["gp_body_accelerations"][:, component]-result["truth_body_accelerations"][:, component], label="GP minus independent truth")
            ax[index].set_ylabel(f"acceleration error [{unit}]")
            ax[index].grid(alpha=.25); ax[index].legend(fontsize=8)
        ax[-1].set_xlabel("time [s]")
        f.suptitle(f"{name} — SYNTHETIC acceleration reconstruction error (magnified)")
        f.tight_layout(); f.savefig(destination / "acceleration_reconstruction_error.png", dpi=160); plt.close(f)
    blind = evaluate_blind_spot(config)
    destination = output / "collocation_blind_spot"
    destination.mkdir()
    _write_json(destination / "numeric_results.json", blind)
    f, ax = plt.subplots(figsize=(10, 5))
    ax.plot(blind["times_s"], blind["body_twists"][:, 1], label="GP body lateral velocity")
    ax.plot(blind["independent_times_s"][::20], blind["independent_body_twists"][::20, 1], ".", label="independent pose differentiation")
    b = blind["summary"]
    for sign in (-1, 1): ax.axhline(sign*b["equality_tolerance_m_s"], color="red", linestyle=":", label="original tolerance" if sign == 1 else None)
    fractions = b["fractions"]
    ax.scatter(fractions[[0, 2, 4]]*.1, b["lateral_at_prescribed_points_m_s"][[0, 2, 4]], c="green", marker="o", s=70, zorder=5, label="support / midpoint: pass")
    ax.scatter(fractions[[1, 3]]*.1, b["lateral_at_prescribed_points_m_s"][[1, 3]], c="red", marker="x", s=80, zorder=5, label="quarter points: fail")
    ax.set(xlabel="interval time [s]", ylabel="body lateral velocity [m/s]",
           title=f"SYNTHETIC GP blind spot — max |v_y|={b['maximum_absolute_lateral_velocity_m_s']:.6g} m/s\nThis example is not a feasible unicycle motion")
    ax.legend(fontsize=9); ax.grid(alpha=.25)
    f.tight_layout(); f.savefig(destination / "lateral_velocity_vs_time.png", dpi=160); plt.close(f)
    _write_json(output / "summary.json", {"fixtures": summaries, "blind_spot": blind["summary"], "optimizer_executed": False})
    csv_fields = ["fixture", "known_seed_original_dense_feasible", "position_error_max_m", "yaw_error_max_rad",
                  "lateral_velocity_max_m_s", "linear_acceleration_max_m_s2", "angular_acceleration_max_rad_s2",
                  "ode_vs_tighter_position_error_max_m", "ode_vs_tighter_yaw_error_max_rad"]
    with (output / "fixture_results.csv").open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=csv_fields, extrasaction="ignore")
        writer.writeheader(); writer.writerows(summaries)
    return {"fixtures": summaries, "blind_spot": blind["summary"], "optimizer_executed": False}

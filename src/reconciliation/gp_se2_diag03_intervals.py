"""Saved GP curve queries for DIAG-03; no solve and no changed acceptance.

Every sample retains its support interval.  A knot therefore has two records
when both limits are queried; acceleration is not averaged across that knot.
Supplemental maxima and transition brackets describe observed samples, not a
continuous-time certificate.  ``a_y`` is diagnostic, never a new constraint.
"""
from __future__ import annotations

import numpy as np

from .gp_se2 import interpolate_interval, sample_gp
from .gp_se2_diag_acceptance import offset_grid
from .se2 import wrap_angle


QUANTITY_NAMES = ("vx", "vy", "omega", "ax", "ay", "alpha")
GRID_NAMES = ("ORIGINAL_COLLOCATION", "ORIGINAL_DENSE", "ORIGINAL_FULL_OFFSET",
              "SUPPLEMENTAL_0.010", "SUPPLEMENTAL_0.005", "SUPPLEMENTAL_0.001_OFFSET")


def motion_definitions(config):
    """Original units, bounds and numerical tolerances, with no mixed-unit norm."""
    c = config
    return {
        "vx": dict(family="linear_speed", unit="m/s", lower=0., upper=c["v_max"],
                   tolerance=c["inequality_tolerance"], enforced=True),
        "vy": dict(family="lateral_velocity", unit="m/s", lower=0., upper=0.,
                   tolerance=c["equality_tolerance"], enforced=True),
        "omega": dict(family="angular_speed", unit="rad/s", lower=-c["w_max"], upper=c["w_max"],
                      tolerance=c["inequality_tolerance"], enforced=True),
        "ax": dict(family="linear_acceleration", unit="m/s^2", lower=-c["a_v_max"], upper=c["a_v_max"],
                   tolerance=c["inequality_tolerance"], enforced=True),
        "ay": dict(family="lateral_acceleration_diagnostic", unit="m/s^2", lower=None, upper=None,
                   tolerance=None, enforced=False),
        "alpha": dict(family="angular_acceleration", unit="rad/s^2", lower=-c["a_w_max"], upper=c["a_w_max"],
                      tolerance=c["inequality_tolerance"], enforced=True),
    }


def _validate_vector(problem, vector):
    x = np.asarray(vector, dtype=np.float64)
    if x.shape != (5*(len(problem.times)-1),) or not np.all(np.isfinite(x)):
        raise ValueError("finite saved vector in the original five-variable-per-support chart required")
    return x


def _site(u):
    if abs(u) <= 1e-10:
        return "support", "right"
    if abs(u-1.) <= 1e-10:
        return "support", "left"
    return ("midpoint", None) if abs(u-.5) <= 1e-10 else ("off-collocation", None)


def _location(problem, t):
    i = int(np.clip(np.searchsorted(problem.times, t, side="right")-1, 0, len(problem.times)-2))
    u = float((t-problem.times[i])/problem.config["support_dt_s"])
    site, side = _site(u)
    return dict(interval_index=i, trajectory_time_s=float(t), local_fraction=u,
                location_type=site, knot_side=side)


def _sample_fraction_lists(problem, vector, fractions):
    p, v = problem.unpack(_validate_vector(problem, vector))
    indices = np.concatenate([np.full(len(u), i, dtype=int) for i, u in enumerate(fractions)])
    u = np.concatenate(fractions)
    h = problem.config["support_dt_s"]
    pp, vv, aa = interpolate_interval(p[indices], v[indices], p[indices+1], v[indices+1], h, u)
    sites, sides = zip(*[_site(float(x)) for x in u])
    return dict(times_s=problem.times[indices]+h*u, interval_index=indices, local_fraction=u,
                location_type=list(sites), knot_side=list(sides), poses_world=pp,
                body_twists=vv, body_accelerations=aa,
                quantities=dict(zip(QUANTITY_NAMES, np.column_stack([vv, aa]).T)))


def sample_interval_grid(problem, vector, query_times):
    """Query specified global times plus both interval ends, without knot merging."""
    q = np.asarray(query_times, float)
    if q.ndim != 1 or not np.all(np.isfinite(q)) or np.any(q < problem.times[0]-1e-12) or np.any(q > problem.times[-1]+1e-12):
        raise ValueError("query times must be finite and within the stored support domain")
    h = problem.config["support_dt_s"]
    fractions = []
    for left, right in zip(problem.times[:-1], problem.times[1:]):
        interior = q[(q > left+1e-12) & (q < right-1e-12)]
        fractions.append(np.unique(np.r_[0., (interior-left)/h, 1.]))
    return _sample_fraction_lists(problem, vector, fractions)


def constraint_time_map(problem, vector):
    """All actual evaluator rows, in its family-major flattened ordering.

    Values are independently assembled in that ordering and compared to the
    original evaluator.  Workspace/obstacle invalid-domain sentinels preserve
    the original semantics and are explicitly labeled, not physical distances.
    """
    x = _validate_vector(problem, vector)
    e = problem.evaluate(x)
    c, n, h = problem.config, len(problem.times)-1, problem.config["support_dt_s"]
    trace = _sample_fraction_lists(problem, x, [np.array([0., .5, 1.])]*n)
    pp, vv, aa = trace["poses_world"], trace["body_twists"], trace["body_accelerations"]
    rows, expected = [], {"equality": [], "inequality": []}

    def row(kind, family, quantity, unit, t, actual, residual, limit, expression, *, index=None, u=None,
            gate_index=None, nonfinite_query=False):
        loc = _location(problem, t)
        if index is not None:
            site, side = _site(float(u))
            loc.update(interval_index=int(index), local_fraction=float(u), location_type=site, knot_side=side)
        number = len(expected[kind])
        expected[kind].append(float(residual))
        rows.append(dict(constraint_kind=kind, row_index=number, family=family, physical_quantity=quantity,
                         physical_unit=unit, actual_value=float(actual), signed_residual_or_margin=float(residual),
                         nominal_limit=limit, actual_numerical_tolerance=c["equality_tolerance" if kind == "equality" else "inequality_tolerance"],
                         expression=expression, gate_index=gate_index, nonfinite_query_mapped=bool(nonfinite_query), **loc))

    for i in range(n):
        value = vv[3*i+1, 1]
        row("equality", "lateral", "v_y", "m/s", problem.times[i]+h*.5, value, value, 0., "v_y = 0", index=i, u=.5)
    gate_data = []
    poses, twists = e["poses"], e["twists"]
    for j, gate in enumerate(problem.gates):
        gt = float(gate["time_s"])
        ts = np.array([max(0., gt-gate.get("crossing_offset_s", .05)), gt,
                       min(problem.times[-1], gt+gate.get("crossing_offset_s", .05))])
        point = sample_gp(problem.times, poses, twists, ts)[0][:, :2]-np.asarray(gate["center_xy"])
        normal = np.asarray(gate["normal_xy"])
        signed, lateral = point@normal, point[1]@np.array([-normal[1], normal[0]])
        row("equality", "gate_plane", "signed_gate_plane_distance", "m", gt, signed[1], signed[1], 0., "signed_distance = 0", gate_index=j)
        gate_data.append((ts, signed, float(lateral), gate))
    specs = (
        ("linear_speed_lower", "v_x", "m/s", vv[:, 0], vv[:, 0], 0., "v_x >= 0"),
        ("linear_speed_upper", "v_x", "m/s", vv[:, 0], c["v_max"]-vv[:, 0], c["v_max"], "v_max - v_x >= 0"),
        ("angular_speed_upper", "omega", "rad/s", vv[:, 2], c["w_max"]-vv[:, 2], c["w_max"], "w_max - omega >= 0"),
        ("angular_speed_lower", "omega", "rad/s", vv[:, 2], c["w_max"]+vv[:, 2], -c["w_max"], "w_max + omega >= 0"),
        ("linear_acceleration_upper", "a_x", "m/s^2", aa[:, 0], c["a_v_max"]-aa[:, 0], c["a_v_max"], "a_v_max - a_x >= 0"),
        ("linear_acceleration_lower", "a_x", "m/s^2", aa[:, 0], c["a_v_max"]+aa[:, 0], -c["a_v_max"], "a_v_max + a_x >= 0"),
        ("angular_acceleration_upper", "alpha", "rad/s^2", aa[:, 2], c["a_w_max"]-aa[:, 2], c["a_w_max"], "a_w_max - alpha >= 0"),
        ("angular_acceleration_lower", "alpha", "rad/s^2", aa[:, 2], c["a_w_max"]+aa[:, 2], -c["a_w_max"], "a_w_max + alpha >= 0"),
    )
    for family, quantity, unit, values, margins, limit, expression in specs:
        for k, (value, margin) in enumerate(zip(values, margins)):
            row("inequality", family, quantity, unit, trace["times_s"][k], value, margin, limit, expression,
                index=k//3, u=[0., .5, 1.][k % 3])
    distance2 = np.sum((poses[-1, :2]-problem.goal_pose[:2])**2)
    yaw = float(wrap_angle(poses[-1, 2]-problem.goal_pose[2]))
    row("inequality", "goal_position", "squared_goal_position_distance", "m^2", problem.times[-1], distance2,
        c["goal_position_tolerance"]**2-distance2, c["goal_position_tolerance"]**2, "position_tolerance^2 - squared_distance >= 0")
    for sign, suffix in ((-1, "upper"), (1, "lower")):
        row("inequality", "goal_yaw_"+suffix, "wrapped_goal_yaw_error", "rad", problem.times[-1], yaw,
            c["goal_yaw_tolerance"]+sign*yaw, -sign*c["goal_yaw_tolerance"], "yaw_tolerance + sign*yaw_error >= 0")
    for j, (ts, signed, lateral, gate) in enumerate(gate_data):
        half, progress = gate["half_width_m"], gate.get("minimum_progress_m", 1e-4)
        for fam, t, value, margin, limit in (
                ("gate_width_upper", ts[1], lateral, half-lateral, half),
                ("gate_width_lower", ts[1], lateral, half+lateral, -half),
                ("gate_before_direction", ts[0], signed[0], -signed[0]-progress, -progress),
                ("gate_after_direction", ts[2], signed[2], signed[2]-progress, progress)):
            row("inequality", fam, "gate_distance", "m", t, value, margin, limit, "original gate signed margin >= 0", gate_index=j)
    for family, callback, limit in (("workspace", problem.workspace_margin, 0.),
                                    ("obstacle", problem.obstacle_clearance if problem.include_obstacles else None, c["required_clearance"])):
        if callback is None:
            continue
        values = np.asarray(callback(pp[:, :2]), float)
        for k, raw in enumerate(values):
            value = float(raw) if np.isfinite(raw) else -1e6
            row("inequality", family, family+"_margin", "m", trace["times_s"][k], value, value-limit,
                limit, "original query - nominal_limit >= 0", index=k//3, u=[0., .5, 1.][k % 3], nonfinite_query=not np.isfinite(raw))
    errors = {}
    for kind in ("equality", "inequality"):
        actual, mapped = np.asarray(e[kind]), np.asarray(expected[kind])
        if actual.shape != mapped.shape:
            raise AssertionError(f"original {kind} row count disagrees with time map")
        errors[kind] = float(np.max(np.abs(actual-mapped), initial=0.))
        if not np.allclose(actual, mapped, rtol=1e-13, atol=1e-12):
            raise AssertionError(f"original {kind} values disagree with time map: {errors[kind]}")
    return dict(rows=rows, comparison_maximum_absolute_error=errors,
                equality_count=len(expected["equality"]), inequality_count=len(expected["inequality"]),
                support_lateral_zero=dict(source="GPProblem.unpack variable parameterization", explicit_constraint_rows=0,
                                          support_count=n+1, fixed_boundary=True, support_values=twists[:, 1].tolist()),
                goal_unit_note="Optimizer goal-position inequality and its numerical tolerance are m^2; the independent goal checker uses distance in m.")


def _excess(values, definition):
    values = np.asarray(values, float)
    if not definition["enforced"]:
        return None, None
    raw = np.maximum(definition["lower"]-values, values-definition["upper"])
    return np.maximum(0., raw), raw-definition["tolerance"]


def _runs(mask):
    mask = np.asarray(mask, bool)
    edges = np.diff(np.r_[False, mask, False].astype(int))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)-1))


def refine_transition(residual, left, right, *, maximum_width_s=1e-4):
    """Bisect an already observed sign transition only; do not search safe pairs."""
    left, right = float(left), float(right)
    if not 0 < maximum_width_s or right <= left:
        raise ValueError("positive width and increasing observed bracket required")
    fl, fr = float(residual(left)), float(residual(right))
    if not np.isfinite([fl, fr]).all() or (fl > 0) == (fr > 0):
        raise ValueError("a finite safe/violating observed transition is required")
    original = [left, right]
    calls = 2
    while right-left > maximum_width_s:
        middle = (left+right)/2
        fm = float(residual(middle))
        calls += 1
        if not np.isfinite(fm):
            raise FloatingPointError("nonfinite stored-curve residual during refinement")
        if (fm > 0) == (fl > 0):
            left, fl = middle, fm
        else:
            right, fr = middle, fm
    return dict(original_bracket_s=original, bracket_s=[left, right], bracket_width_s=right-left,
                endpoint_residuals=[fl, fr], direction="entry" if fr > 0 else "exit",
                query_count=calls, status="OBSERVED_TRANSITION_REFINED", continuous_root_uniqueness_proven=False)


def _interval_extrema(problem, trace, grid_name, definitions, collocation):
    result = []
    n = len(problem.times)-1
    for i in range(n):
        indices = np.flatnonzero(trace["interval_index"] == i)
        colloc = np.flatnonzero(collocation["interval_index"] == i)
        for quantity, definition in definitions.items():
            values = trace["quantities"][quantity][indices]
            nominal, excess = _excess(values, definition)
            k_min, k_max, k_abs = int(np.argmin(values)), int(np.argmax(values)), int(np.argmax(np.abs(values)))
            worst = int(np.argmax(excess)) if excess is not None else k_abs
            absolute = indices[worst]
            u = float(trace["local_fraction"][absolute])
            enforcement = np.array([0., .5, 1.])
            row = dict(grid=grid_name, interval_index=i, quantity=quantity, **definition,
                       sample_count=len(indices), minimum=float(values[k_min]), maximum=float(values[k_max]),
                       maximum_absolute_value=float(abs(values[k_abs])),
                       minimum_time_s=float(trace["times_s"][indices[k_min]]), minimum_u=float(trace["local_fraction"][indices[k_min]]),
                       maximum_time_s=float(trace["times_s"][indices[k_max]]), maximum_u=float(trace["local_fraction"][indices[k_max]]),
                       maximum_absolute_time_s=float(trace["times_s"][indices[k_abs]]), maximum_absolute_u=float(trace["local_fraction"][indices[k_abs]]),
                       worst_observed_value=float(values[worst]), worst_time_s=float(trace["times_s"][absolute]), worst_u=u,
                       worst_location_type=trace["location_type"][absolute], worst_knot_side=trace["knot_side"][absolute],
                       nominal_bound_excess=None if nominal is None else float(np.max(nominal)),
                       maximum_tolerance_excess=None if excess is None else float(max(0., np.max(excess))),
                       maximum_signed_tolerance_residual=None if excess is None else float(np.max(excess)),
                       nearest_enforced_or_structural_time_distance_s=None if not definition["enforced"] else float(np.min(np.abs(enforcement-u))*problem.config["support_dt_s"]),
                       nearest_explicit_enforced_time_distance_s=None if not definition["enforced"] else float(np.min(np.abs((np.array([.5]) if quantity == "vy" else enforcement)-u))*problem.config["support_dt_s"]),
                       original_enforcement_note=("midpoint equality; endpoint lateral zeros are structural" if quantity == "vy" else
                                                  "diagnostic only, no acceptance bound" if quantity == "ay" else "u=0,0.5,1 inequalities including both knot limits"),
                       original_collocation_u=[0., .5, 1.], original_collocation_values=collocation["quantities"][quantity][colloc].tolist(),
                       violating_sample_count=0 if excess is None else int(np.sum(excess > 0)),
                       observed_violation_run_count=0 if excess is None else len(_runs(excess > 0)),
                       first_observed_violation_time_s=None, last_observed_violation_time_s=None,
                       first_observed_violation_u=None, last_observed_violation_u=None,
                       finite_sampling_only=True)
            if excess is not None and np.any(excess > 0):
                viol = indices[excess > 0]
                row.update(first_observed_violation_time_s=float(trace["times_s"][viol[0]]),
                           last_observed_violation_time_s=float(trace["times_s"][viol[-1]]),
                           first_observed_violation_u=float(trace["local_fraction"][viol[0]]),
                           last_observed_violation_u=float(trace["local_fraction"][viol[-1]]))
            result.append(row)
    return result


def _violation_brackets(problem, vector, trace, definitions):
    p, v = problem.unpack(vector)
    h, rows = problem.config["support_dt_s"], []
    for i in range(len(problem.times)-1):
        index = np.flatnonzero(trace["interval_index"] == i)
        times = trace["times_s"][index]
        for qindex, (quantity, definition) in enumerate(definitions.items()):
            if not definition["enforced"]:
                continue
            _, residuals = _excess(trace["quantities"][quantity][index], definition)
            def query(t):
                _, velocity, acceleration = interpolate_interval(p[i], v[i], p[i+1], v[i+1], h, (t-problem.times[i])/h)
                value = np.r_[velocity, acceleration][qindex]
                return float(_excess(np.array([value]), definition)[1][0])
            for run, (first, last) in enumerate(_runs(residuals > 0)):
                base = dict(quantity=quantity, family=definition["family"], unit=definition["unit"],
                            lower=definition["lower"], upper=definition["upper"], tolerance=definition["tolerance"],
                            interval_index=i, observed_run_index=run, first_observed_time_s=float(times[first]),
                            last_observed_time_s=float(times[last]), finite_sampling_only=True)
                for boundary, a, b in (("entry", first-1, first), ("exit", last, last+1)):
                    if a < 0 or b >= len(times):
                        k = first if boundary == "entry" else last
                        rows.append(dict(**base, boundary=boundary, status="VIOLATING_AT_INTERVAL_LIMIT",
                                         bracket_s=[float(times[k]), float(times[k])], bracket_width_s=0.,
                                         knot_side="right" if boundary == "entry" else "left",
                                         query_count=0, root_bracket_available=False))
                    else:
                        rows.append(dict(**base, boundary=boundary, knot_side=None, root_bracket_available=True,
                                         **refine_transition(query, times[a], times[b])))
    return rows


def _grid_detection(problem, vector, finest, definitions):
    n = len(problem.times)-1
    fractions = {"G0_ORIGINAL": [np.array([0., .5, 1.]) for _ in range(n)],
                 "G1_QUARTERS": [np.array([0., .25, .5, .75, 1.]) for _ in range(n)]}
    witness = []
    fractions["G2_WITNESS"] = [np.array([0., .5, 1.]) for _ in range(n)]
    for i in range(n):
        indices = np.flatnonzero(finest["interval_index"] == i)
        for quantity, definition in definitions.items():
            if not definition["enforced"]:
                continue
            _, excess = _excess(finest["quantities"][quantity][indices], definition)
            k = int(np.argmax(excess))
            if excess[k] > 0:
                j = indices[k]
                u = float(finest["local_fraction"][j])
                # A witness numerically coincident with an existing enforced
                # location must not be counted as an additional constraint.
                # This 1e-10 fraction identity rule affects only diagnostic
                # accounting; it never changes the curve or physical limit.
                existing = fractions["G2_WITNESS"][i]
                if np.min(np.abs(existing-u)) > 1e-10:
                    fractions["G2_WITNESS"][i] = np.sort(np.r_[existing, u])
                witness.append(dict(quantity=quantity, interval_index=i, time_s=float(finest["times_s"][j]),
                                    u=u, value=float(finest["quantities"][quantity][j]), tolerance_excess=float(excess[k]),
                                    selection="post-hoc maximum observed tolerance excess per interval/quantity; earliest tie"))
    rows = []
    base_motion, base_lateral = 3*n, n
    for name, grid in fractions.items():
        trace = _sample_fraction_lists(problem, vector, grid)
        # Endpoints are structural lateral zeros; midpoint plus new interior
        # points would add equality rows, while motion evaluates all points.
        motion_count = sum(len(x) for x in grid)
        lateral_count = sum(np.count_nonzero((x > 1e-10) & (x < 1.-1e-10)) for x in grid)
        for quantity, definition in definitions.items():
            if not definition["enforced"]:
                continue
            _, residual = _excess(trace["quantities"][quantity], definition)
            _, truth = _excess(finest["quantities"][quantity], definition)
            detected = sorted(set(trace["interval_index"][residual > 0].tolist()))
            observed = sorted(set(finest["interval_index"][truth > 0].tolist()))
            run_count, missed_runs = 0, []
            for i in range(n):
                fi = np.flatnonzero(finest["interval_index"] == i)
                detected_times = trace["times_s"][(trace["interval_index"] == i) & (residual > 0)]
                for first, last in _runs(truth[fi] > 0):
                    run_count += 1
                    left, right = float(finest["times_s"][fi[first]]), float(finest["times_s"][fi[last]])
                    if not np.any((detected_times >= left-1e-12) & (detected_times <= right+1e-12)):
                        missed_runs.append(dict(interval_index=i, first_observed_time_s=left, last_observed_time_s=right))
            rows.append(dict(grid=name, quantity=quantity, family=definition["family"], unit=definition["unit"],
                             maximum_tolerance_excess=float(max(0., np.max(residual))),
                             violating_sample_count=int(np.sum(residual > 0)), detected_interval_indices=detected,
                             finest_observed_violating_interval_indices=observed,
                             finest_observed_intervals_missed=sorted(set(observed)-set(detected)),
                             finest_observed_violation_run_count=run_count,
                             finest_observed_violation_runs_detected=run_count-len(missed_runs),
                             finest_observed_violation_runs_missed=missed_runs,
                             detection_note="A detected family/interval does not imply all disconnected sampled violating runs were detected.",
                             passed=bool(not np.any(residual > 0)), lateral_equality_count=int(lateral_count),
                             additional_lateral_equalities=int(lateral_count-base_lateral),
                             motion_evaluation_count=motion_count, additional_motion_evaluations=motion_count-base_motion,
                             motion_inequality_count=8*motion_count, additional_motion_inequalities=8*(motion_count-base_motion),
                             optimization_variable_count=5*n, hypothetical_independent_equality_rank=None,
                             detection_only=True, constraints_actually_added=False, optimizer_calls=0))
    return rows, witness


def audit_intervals(problem, vector):
    """Audit every stored interval, retaining original and supplemental grids."""
    x = _validate_vector(problem, vector)
    before = x.copy()
    definitions, n, end = motion_definitions(problem.config), len(problem.times)-1, problem.times[-1]
    collocation = _sample_fraction_lists(problem, x, [np.array([0., .5, 1.])]*n)
    dense = np.linspace(problem.times[0], end, int(np.ceil(end/problem.config["dense_dt_s"]))+1)
    full = offset_grid(problem.times)
    grids = {"ORIGINAL_COLLOCATION": collocation,
             "ORIGINAL_DENSE": sample_interval_grid(problem, x, dense),
             "ORIGINAL_FULL_OFFSET": sample_interval_grid(problem, x, full)}
    for dt in (.010, .005, .001):
        q = (offset_grid(problem.times, dt) if dt == .001 else
             np.unique(np.r_[np.linspace(0., end, int(np.ceil(end/dt))+1), problem.times,
                             (problem.times[:-1]+problem.times[1:])/2]))
        name = f"SUPPLEMENTAL_{dt:.3f}"+("_OFFSET" if dt == .001 else "")
        grids[name] = sample_interval_grid(problem, x, q)
    extrema = [r for name, trace in grids.items() for r in _interval_extrema(problem, trace, name, definitions, collocation)]
    finest = grids["SUPPLEMENTAL_0.001_OFFSET"]
    detection, witnesses = _grid_detection(problem, x, finest, definitions)
    reps = []
    for quantity, definition in definitions.items():
        if definition["enforced"]:
            choices = [r for r in extrema if r["grid"] == "SUPPLEMENTAL_0.001_OFFSET" and r["quantity"] == quantity]
            best = min(choices, key=lambda r: (-r["maximum_tolerance_excess"], r["interval_index"]))
            reps.append(dict(quantity=quantity, interval_index=best["interval_index"],
                             maximum_tolerance_excess=best["maximum_tolerance_excess"],
                             worst_time_s=best["worst_time_s"], worst_u=best["worst_u"],
                             selection_rule="maximum observed tolerance excess; earliest interval on ties"))
    p, v = problem.unpack(x)
    _, ends_v, ends_a = interpolate_interval(p[:-1, None], v[:-1, None], p[1:, None], v[1:, None], problem.config["support_dt_s"], np.array([0., 1.]))
    knots = []
    for k in range(1, n):
        knots.append(dict(support_index=k, time_s=float(problem.times[k]), left_interval=k-1, right_interval=k,
                          body_twist_left=ends_v[k-1, 1], body_twist_right=ends_v[k, 0],
                          body_acceleration_left=ends_a[k-1, 1], body_acceleration_right=ends_a[k, 0],
                          acceleration_jump=ends_a[k, 0]-ends_a[k-1, 1]))
    assert np.array_equal(x, before), "audit must not change saved optimization variables"
    return dict(constraint_time_map=constraint_time_map(problem, x), quantity_definitions=definitions,
                interval_extrema=extrema, violation_brackets=_violation_brackets(problem, x, finest, definitions),
                grid_detection_comparison=detection, diagnostic_witnesses=witnesses,
                representative_intervals=reps, finest_trace=finest, original_collocation_trace=collocation,
                knot_limits=knots,
                grid_summary=[dict(grid=name, interval_side_sample_count=len(trace["times_s"]),
                                   unique_time_count=len(np.unique(trace["times_s"])), both_knot_sides=True,
                                   unique_time_count_semantics="literal float64 timestamps; last-bit coincident knot labels may remain distinct",
                                   unique_time_count_rounded_12dp=len(np.unique(np.round(trace["times_s"], 12))))
                              for name, trace in grids.items()],
                interpolation_queries_only=True, optimizer_calls=0, primary_acceptance_modified=False,
                continuous_time_feasibility_proven=False,
                limitations=["Finite sampled extrema are not exact continuous maxima.",
                             "Only observed safe/violating transitions are refined; safe endpoints do not exclude interior violations.",
                             "G2 witnesses are post-hoc; detecting a violation does not establish feasibility of a refined optimization problem.",
                             "Lateral acceleration has no acceptance bound in the original formulation."])

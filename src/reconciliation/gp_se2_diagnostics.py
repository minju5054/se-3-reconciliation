"""Read-only residual diagnostics for the frozen GP-SE2-01 formulation.

These functions do not optimize or change acceptance. Times are seconds after
the recorded boundary; poses are fixed-world metres / CCW radians; velocities
and accelerations are body-frame quantities. A saved iterate is evidence only
about that iterate, never about an unrecorded iteration history.
"""
from __future__ import annotations

from collections import Counter
from datetime import datetime, timezone
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from .gp_se2 import interpolate_interval, sample_gp
from .gp_se2_formulation import GPProblem, _constraint_report
from .se2 import relative_pose, se2_log, wrap_angle


CLASSIFICATIONS = (
    "INITIAL_ALREADY_FEASIBLE", "COLLOCATION_INFEASIBLE",
    "COLLOCATION_PASS_DENSE_FAIL", "DENSE_FEASIBLE",
    "NUMERICAL_EVALUATION_ERROR", "INSUFFICIENT_SAVED_EVIDENCE",
)
CLASSIFICATION_CODES = dict(zip(CLASSIFICATIONS, "ABCDEF"))


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    return value


def _read(path):
    return json.loads(Path(path).read_text())


def load_frozen_case(primary, case_id, method="M3_GP_CONSTRAINED", *, environment=None):
    """Resolve the case through its manifest and use original saved inputs.

    The physical ``u_minus`` supplies the initial GP twist. Controller memory is
    returned in context, separately, and is not substituted for physical motion.
    """
    from .gp_se2_environment import HospitalEnvironment

    primary = Path(primary).resolve()
    rows = [row for row in _read(primary / "case_manifest.json")["selected"]
            if row["case_id"] == case_id]
    if len(rows) != 1:
        raise ValueError("case_id must resolve uniquely through the frozen manifest")
    if method not in ("M2_GP_NO_OBSTACLE", "M3_GP_CONSTRAINED"):
        raise ValueError("diagnostics require an original GP method")
    case = primary / "cases" / rows[0]["case_directory"]
    config = yaml.safe_load((primary / "config_snapshot.yaml").read_text())
    context, route = _read(case / "input_context.json"), _read(case / "goal_route.json")
    common = np.load(case / "F_common.npy", allow_pickle=False)
    env = environment if environment is not None else HospitalEnvironment.load(primary / "environment")
    radius = config["footprint"]["radius_m"]
    physical = context["u_minus"]
    problem = GPProblem(
        context["B_world"], [physical[0], 0., physical[1]], common,
        route["goal_world"], config["formulation"],
        obstacle_clearance=lambda xy: env.optimizer_clearance(xy, radius),
        workspace_margin=lambda xy: env.workspace_margin(xy, radius),
        gates=route["gates"], include_obstacles=method == "M3_GP_CONSTRAINED")
    inputs = [primary / "config_snapshot.yaml", primary / "case_manifest.json",
              case / "input_context.json", case / "goal_route.json",
              case / "F_native.npy", case / "F_common.npy"]
    return dict(problem=problem, config=config, environment=env, case_directory=case,
                context=context, goal_route=route, common_reference=common,
                manifest_row=rows[0], hashes={str(p): file_sha256(p) for p in inputs})


def _sample_locations(times, support_times):
    """Classify time locations without assigning a knot's derivative side."""
    times, knots = np.asarray(times), np.asarray(support_times)
    interval = np.clip(np.searchsorted(knots, times, side="right") - 1, 0, len(knots)-2)
    u = (times - knots[interval]) / (knots[interval+1] - knots[interval])
    sites = np.where(np.isclose(u, 0., atol=1e-9) | np.isclose(u, 1., atol=1e-9),
                     "support", np.where(np.isclose(u, .5, atol=1e-9), "midpoint", "off-collocation"))
    return interval, sites


def _residual_summary(family, unit, values, lower, upper, tolerance, times,
                      intervals, sites, sides, *, phase, enforced=True):
    """Retain physical values as well as excess above the nominal bound."""
    values = np.asarray(values, float).reshape(-1)
    if not np.all(np.isfinite(values)):
        raise FloatingPointError(f"nonfinite {family} values")
    times = np.asarray(times, float).reshape(-1)
    intervals, sites, sides = map(np.asarray, (intervals, sites, sides))
    if not all(len(a) == len(values) for a in (times, intervals, sites, sides)):
        raise ValueError("residual value/location shape mismatch")
    lo = -np.inf if lower is None else lower
    hi = np.inf if upper is None else upper
    violation = np.maximum.reduce([np.zeros_like(values), lo-values, values-hi])
    rows = []
    for site, side in sorted(set(zip(sites.tolist(), sides.tolist()))):
        index = np.flatnonzero((sites == site) & (sides == side))
        worst = int(index[np.argmax(violation[index])])
        # In a passing group, retain an informative most extreme actual value.
        if np.max(violation[index]) == 0:
            worst = int(index[np.argmax(np.abs(values[index]))])
        excess = float(violation[worst])
        rows.append(dict(
            family=family, physical_unit=unit, phase=phase, site=site,
            derivative_side=side or None, sample_count=int(len(index)),
            actual_minimum=float(np.min(values[index])), actual_maximum=float(np.max(values[index])),
            actual_value_at_maximum_violation=float(values[worst]),
            allowed_minimum=lower, allowed_maximum=upper,
            applicable_tolerance=float(tolerance), maximum_violation=excess,
            maximum_excess_after_tolerance=max(0., excess-tolerance),
            violation_time_s=float(times[worst]), interval_index=int(intervals[worst]),
            violating_sample_count=int(np.sum(violation[index] > tolerance)),
            passed=bool(excess <= tolerance), enforced_in_original_checker=bool(enforced)))
    return rows


def detailed_residuals(problem, x, *, dense_dt_s=None):
    """Evaluate an iterate with unchanged checkers plus located residuals.

    ``dense_dt_s`` only controls the extra diagnostic trace; the authoritative
    ``dense`` acceptance always comes from the original ``dense_report``. Both
    sides of support acceleration are retained explicitly. Goal position uses
    the original squared-distance inequality (m²), including its m² tolerance.
    """
    c = problem.config
    x = np.asarray(x, float)
    if x.shape != (5*(len(problem.times)-1),) or not np.all(np.isfinite(x)):
        raise ValueError("saved optimization vector has invalid shape or nonfinite values")
    evaluation = problem.evaluate(x)
    collocation = _constraint_report(evaluation["equality"], evaluation["inequality"], c)
    dense = problem.dense_report(x)
    p, v = evaluation["poses"], evaluation["twists"]
    n, h = len(problem.times)-1, c["support_dt_s"]
    dt = c["dense_dt_s"] if dense_dt_s is None else float(dense_dt_s)
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("diagnostic dense_dt_s must be positive")
    query = np.linspace(0., problem.times[-1], int(np.ceil(problem.times[-1]/dt))+1)
    trace_p, trace_v, trace_a = sample_gp(problem.times, p, v, query)
    rows = []

    def motion(pp, vv, aa, tt, ii, sites, sides, phase):
        for family, unit, val, lo, hi, tol in (
            ("lateral_velocity", "m/s", vv[:, 1], 0., 0., c["equality_tolerance"]),
            ("linear_speed", "m/s", vv[:, 0], 0., c["v_max"], c["inequality_tolerance"]),
            ("angular_speed", "rad/s", vv[:, 2], -c["w_max"], c["w_max"], c["inequality_tolerance"]),
            ("linear_acceleration", "m/s^2", aa[:, 0], -c["a_v_max"], c["a_v_max"], c["inequality_tolerance"]),
            ("angular_acceleration", "rad/s^2", aa[:, 2], -c["a_w_max"], c["a_w_max"], c["inequality_tolerance"]),
        ):
            rows.extend(_residual_summary(family, unit, val, lo, hi, tol, tt, ii, sites, sides, phase=phase))
        for family, callback, lo in (
            ("workspace_margin", problem.workspace_margin, 0.),
            ("obstacle_clearance", problem.obstacle_clearance if problem.include_obstacles else None,
             c["required_clearance"]),
        ):
            if callback is not None:
                val = np.asarray(callback(pp[:, :2]), float)
                # Original _query maps unknown to a negative sentinel. Expose
                # that mapping rather than recording a fictitious measurement.
                unknown = ~np.isfinite(val)
                mapped = np.where(unknown, -1e6, val)
                part = _residual_summary(family, "m", mapped, lo, None, c["inequality_tolerance"],
                                         tt, ii, sites, sides, phase=phase)
                for row in part:
                    row["unknown_query_count"] = int(np.sum(unknown))
                    row["nonfinite_query_mapping"] = "original checker sentinel -1e6 m; not physical distance"
                rows.extend(part)

    fractions = np.array([0., .5, 1.])
    cp, cv, ca = interpolate_interval(p[:-1, None], v[:-1, None], p[1:, None], v[1:, None], h, fractions)
    ct = (problem.times[:-1, None] + h*fractions).reshape(-1)
    ci = np.repeat(np.arange(n), 3)
    cs = np.tile(["support", "midpoint", "support"], n)
    cd = np.tile(["right", "", "left"], n)
    motion(cp.reshape(-1, 3), cv.reshape(-1, 3), ca.reshape(-1, 3), ct, ci, cs, cd, "collocation")
    qi, qs = _sample_locations(query, problem.times)
    qside = np.where(qs == "support", np.where(query == problem.times[-1], "left", "right"), "")
    motion(trace_p, trace_v, trace_a, query, qi, qs, qside, "dense")
    # Include both knot derivative limits in dense diagnostics, as the original
    # dense checker does; C1 GP curves need not have continuous acceleration.
    ends = np.array([0., 1.])
    ep, ev, ea = interpolate_interval(p[:-1, None], v[:-1, None], p[1:, None], v[1:, None], h, ends)
    et = (problem.times[:-1, None] + h*ends).reshape(-1)
    motion(ep.reshape(-1, 3), ev.reshape(-1, 3), ea.reshape(-1, 3), et, np.repeat(np.arange(n), 2),
           np.full(2*n, "support"), np.tile(["right", "left"], n), "dense_one_sided_knots")

    def scalar(family, unit, value, lower, upper, tol, t, site="support", index=None):
        ii = int(_sample_locations(np.array([t]), problem.times)[0][0]) if index is None else index
        rows.extend(_residual_summary(family, unit, [value], lower, upper, tol, [t], [ii],
                                      [site], [""], phase="shared_collocation_and_dense"))
    squared = float(np.sum((p[-1, :2]-problem.goal_pose[:2])**2))
    scalar("goal_position_squared_distance", "m^2", squared, 0., c["goal_position_tolerance"]**2,
           c["inequality_tolerance"], problem.times[-1])
    scalar("goal_yaw", "rad", float(wrap_angle(p[-1, 2]-problem.goal_pose[2])),
           -c["goal_yaw_tolerance"], c["goal_yaw_tolerance"], c["inequality_tolerance"], problem.times[-1])
    for gate_index, gate in enumerate(problem.gates):
        gt = float(gate["time_s"])
        offset, progress = float(gate.get("crossing_offset_s", .05)), float(gate.get("minimum_progress_m", 1e-4))
        tt = np.array([max(0., gt-offset), gt, min(problem.times[-1], gt+offset)])
        gp = sample_gp(problem.times, p, v, tt)[0][:, :2] - np.asarray(gate["center_xy"])
        normal = np.asarray(gate["normal_xy"])
        signed = gp @ normal
        tangent = float(gp[1] @ np.array([-normal[1], normal[0]]))
        before = len(rows)
        scalar("gate_plane", "m", float(signed[1]), 0., 0., c["equality_tolerance"], gt, "gate")
        scalar("gate_width", "m", tangent, -gate["half_width_m"], gate["half_width_m"], c["inequality_tolerance"], gt, "gate")
        scalar("gate_before_direction", "m", float(signed[0]), None, -progress, c["inequality_tolerance"], tt[0], "gate")
        scalar("gate_after_direction", "m", float(signed[2]), progress, None, c["inequality_tolerance"], tt[2], "gate")
        for row in rows[before:]:
            row["gate_index"] = gate_index
    mids = (problem.times[:-1]+problem.times[1:])/2
    eps = 1e-5
    center, body, _ = sample_gp(problem.times, p, v, mids)
    before, after = sample_gp(problem.times, p, v, mids-eps)[0], sample_gp(problem.times, p, v, mids+eps)[0]
    measured = (se2_log(relative_pose(center, after))-se2_log(relative_pose(center, before)))/(2*eps)
    for j, (component, unit) in enumerate((("vx", "m/s"), ("vy", "m/s"), ("omega", "rad/s"))):
        rows.extend(_residual_summary("pose_derivative_identity_"+component, unit, measured[:, j]-body[:, j],
                     0., 0., c["equality_tolerance"], mids, np.arange(n), np.full(n, "midpoint"),
                     np.full(n, ""), phase="dense_derivative_identity"))
    dense_brief = {key: value for key, value in dense.items()
                   if key not in ("times", "poses", "body_twists", "body_accelerations", "equality_residuals", "inequality_margins")}
    return _plain(dict(
        objective=evaluation["objective"], collocation=collocation, dense=dense_brief,
        residuals=rows, failed_families=sorted({r["family"] for r in rows if not r["passed"]}),
        diagnostic_trace=dict(times_s=query, poses_world=trace_p, body_twists=trace_v,
                              body_accelerations=trace_a, interval_index=qi, site=qs,
                              one_sided_knot_times_s=et, one_sided_knot_accelerations=ea.reshape(-1, 3),
                              one_sided_knot_interval_index=np.repeat(np.arange(n), 2),
                              one_sided_knot_side=np.tile(["right", "left"], n)),
        diagnostic_grid_max_dt_s=float(np.max(np.diff(query))),
        authoritative_acceptance="original GPProblem.dense_report; unchanged",
        goal_position_distance_m=float(np.sqrt(squared)),
        independent_environment_check_required=True,
        continuous_time_feasibility_proven=False))


def classify_iterate(problem, x, *, initial=False):
    """Classify observed evidence only; exceptions are not zero residuals."""
    if x is None:
        return dict(classification="INSUFFICIENT_SAVED_EVIDENCE", classification_code="F",
                    evaluated=False, reason="no saved optimization vector")
    try:
        result = detailed_residuals(problem, x)
        if result["dense"]["feasible"]:
            classification = "INITIAL_ALREADY_FEASIBLE" if initial else "DENSE_FEASIBLE"
        elif not result["collocation"]["feasible"]:
            classification = "COLLOCATION_INFEASIBLE"
        else:
            classification = "COLLOCATION_PASS_DENSE_FAIL"
        result.update(classification=classification, classification_code=CLASSIFICATION_CODES[classification], evaluated=True)
        return result
    except (ValueError, FloatingPointError, np.linalg.LinAlgError) as exc:
        return dict(classification="NUMERICAL_EVALUATION_ERROR", classification_code="E",
                    evaluated=False, error=f"{type(exc).__name__}: {exc}")


def classify_saved_attempt(problem, attempt):
    initial = classify_iterate(problem, attempt.get("initial_vector"), initial=True)
    latest = classify_iterate(problem, attempt.get("latest_iterate"))
    best = classify_iterate(problem, attempt.get("best_feasible_iterate"))
    classification = latest["classification"]
    if initial["classification"] == "INITIAL_ALREADY_FEASIBLE":
        classification = "INITIAL_ALREADY_FEASIBLE"
    elif best["classification"] == "DENSE_FEASIBLE":
        classification = "DENSE_FEASIBLE"
    comparisons = []
    reports = {"initial": initial, "latest_iterate": latest, "best_feasible_intermediate": best}
    for saved in attempt.get("candidate_checks", []):
        current = reports.get(saved.get("iterate"), {}).get("dense")
        old = saved.get("constraint_report", {})
        if current is None:
            comparisons.append(dict(iterate=saved.get("iterate"), status="UNKNOWN_NO_REEVALUATED_REPORT"))
            continue
        fields = ("feasible", "maximum_equality_residual", "maximum_inequality_violation",
                  "maximum_absolute_lateral_velocity_m_s", "pose_derivative_body_twist_error_max")
        check = {}
        for key in fields:
            if key not in old or key not in current:
                check[key] = dict(status="UNKNOWN_MISSING_FIELD")
            elif isinstance(old[key], bool):
                check[key] = dict(saved=old[key], reevaluated=current[key], matches=old[key] == current[key])
            else:
                check[key] = dict(saved=old[key], reevaluated=current[key],
                                  absolute_difference=abs(float(old[key])-float(current[key])),
                                  matches=bool(np.isclose(old[key], current[key], rtol=1e-10, atol=1e-12)))
        comparisons.append(dict(iterate=saved["iterate"], fields=check,
                                matches=all(item.get("matches", False) for item in check.values())))
    return dict(classification=classification, classification_code=CLASSIFICATION_CODES[classification],
                original_termination=attempt.get("termination"),
                original_solver_success=attempt.get("solver_success"),
                original_iterations=attempt.get("iterations"),
                initialization=attempt.get("initialization"),
                initial=initial, latest=latest, best_feasible_intermediate=best,
                observed_iterate_labels=[label for label, result in (("initial", initial), ("latest", latest),
                                         ("best_feasible_intermediate", best)) if result["evaluated"]],
                iteration_history_status="UNKNOWN_NOT_SAVED",
                missing_best_feasible_meaning="no saved feasible vector; does not reconstruct any unrecorded state",
                saved_callback_candidate_checks=attempt.get("callback_candidate_checks", []),
                saved_report_reproduction=comparisons,
                original_acceptance_rewritten=False)


def audit_saved_attempts(primary, output):
    """Write a new exclusive audit directory; never write into primary inputs."""
    from .gp_se2_environment import HospitalEnvironment

    primary, output = Path(primary).resolve(), Path(output).resolve()
    if output == primary or primary in output.parents:
        raise ValueError("audit output must not be inside the historical run")
    output.mkdir(parents=True, exist_ok=False)
    source_files = [Path(__file__), Path(__file__).with_name("gp_se2.py"),
                    Path(__file__).with_name("gp_se2_formulation.py"), Path(__file__).with_name("se2.py"),
                    Path(__file__).with_name("gp_se2_environment.py")]
    source_hashes = {str(p.resolve()): file_sha256(p) for p in source_files}
    freeze_matches = {}
    for path in source_files[1:]:
        frozen = primary / "implementation_sources" / "src" / "reconciliation" / path.name
        freeze_matches[path.name] = dict(frozen_path=str(frozen), frozen_exists=frozen.is_file(),
            unchanged_from_numerical_freeze=frozen.is_file() and file_sha256(frozen) == file_sha256(path))
    (output / "source.json").write_text(json.dumps(dict(
        started_utc=datetime.now(timezone.utc).isoformat(), source_hashes=source_hashes,
        numerical_freeze_comparison=freeze_matches, operation="saved-iterate reevaluation; no optimization"), indent=2)+"\n")
    env = HospitalEnvironment.load(primary / "environment")
    hashes, records, summary_rows, residual_rows = {}, [], [], []
    for row in _read(primary / "case_manifest.json")["selected"]:
        for method in ("M2_GP_NO_OBSTACLE", "M3_GP_CONSTRAINED"):
            loaded = load_frozen_case(primary, row["case_id"], method, environment=env)
            hashes.update(loaded["hashes"])
            source = loaded["case_directory"] / "methods" / method / "solver_result.json"
            hashes[str(source)] = file_sha256(source)
            saved = _read(source)
            for index, attempt in enumerate(saved["attempts"]):
                result = classify_saved_attempt(loaded["problem"], attempt)
                result.update(case_id=row["case_id"], method=method, initialization_index=index,
                              source_solver_result=str(source), source_solver_result_sha256=hashes[str(source)],
                              original_method_status=saved["status"],
                              original_method_candidate_available=saved.get("candidate_world") is not None)
                records.append(result)
                latest = result["latest"]
                flat = dict(case_id=row["case_id"], method=method, initialization_index=index,
                            initialization=result["initialization"], classification=result["classification"],
                            classification_code=result["classification_code"],
                            original_termination=result["original_termination"],
                            initial_classification=result["initial"]["classification"],
                            latest_classification=latest["classification"],
                            best_feasible_classification=result["best_feasible_intermediate"]["classification"],
                            latest_collocation_feasible=latest.get("collocation", {}).get("feasible"),
                            latest_dense_feasible=latest.get("dense", {}).get("feasible"),
                            latest_failed_families=";".join(latest.get("failed_families", [])),
                            iteration_history_status=result["iteration_history_status"])
                summary_rows.append(flat)
                for label in ("initial", "latest", "best_feasible_intermediate"):
                    for residual in result[label].get("residuals", []):
                        residual_rows.append(dict(case_id=row["case_id"], method=method,
                            initialization_index=index, iterate=label, **residual))
    changed = [path for path, digest in hashes.items() if file_sha256(path) != digest]
    if changed:
        raise RuntimeError(f"audit inputs changed: {changed}")
    counts = {label: sum(r["classification"] == label for r in records) for label in CLASSIFICATIONS}
    family_counts, site_counts = Counter(), Counter()
    for record in records:
        residuals = record["latest"].get("residuals", [])
        family_counts.update({r["family"] for r in residuals if not r["passed"]})
        site_counts.update({(r["family"], r["site"], r["derivative_side"] or "interior")
                            for r in residuals if not r["passed"]})
    summary = dict(attempt_count=len(records), classification_counts=counts,
        termination_counts=dict(Counter(r["original_termination"] for r in records)),
        initial_classification_counts=dict(Counter(r["initial"]["classification"] for r in records)),
        latest_classification_counts=dict(Counter(r["latest"]["classification"] for r in records)),
        initial_and_latest_classification_counts=dict(Counter(r[label]["classification"] for r in records for label in ("initial", "latest"))),
        best_feasible_saved_evidence_counts=dict(Counter(r["best_feasible_intermediate"]["classification"] for r in records)),
        saved_report_reproduction_count=sum(len(r["saved_report_reproduction"]) for r in records),
        saved_report_reproduction_matches=sum(item.get("matches", False) for r in records for item in r["saved_report_reproduction"]),
        latest_failed_family_counts=dict(family_counts),
        latest_failure_site_counts=[dict(family=k[0], site=k[1], derivative_side=k[2], attempt_count=v)
                                    for k, v in sorted(site_counts.items())],
        history_status="UNKNOWN_NOT_SAVED", saved_iterates_evaluated=sum(len(r["observed_iterate_labels"]) for r in records),
        unrecorded_iterates_evaluated=0, original_acceptance_rewritten=False,
        original_inputs_unchanged=True, source_run=str(primary),
        independent_environment_note="classification uses original optimizer callbacks and original dense checker; full direct geometry remains a separate candidate acceptance step",
        classification_rule="A if initial dense-feasible, else D if saved best dense-feasible, else classify latest saved iterate; termination is separate")
    payload = dict(summary=summary, attempts=records)
    (output / "audit.json").write_text(json.dumps(_plain(payload), indent=2, allow_nan=False)+"\n")
    (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n")
    (output / "input_hashes.json").write_text(json.dumps(dict(before=hashes, after=hashes, unchanged=True), indent=2)+"\n")
    for name, rows in (("existing_rejections.csv", summary_rows), ("residuals.csv", residual_rows)):
        keys = list(dict.fromkeys(k for row in rows for k in row))
        with (output / name).open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=keys)
            writer.writeheader()
            writer.writerows(rows)
    (output / "completion.json").write_text(json.dumps(dict(
        completed_utc=datetime.now(timezone.utc).isoformat(), optimization_performed=False,
        all_40_attempts_reclassified=len(records) == 40,
        source_files_unchanged=all(file_sha256(p) == digest for p, digest in source_hashes.items()),
        classification_counts=counts), indent=2)+"\n")
    return summary

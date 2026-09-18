#!/usr/bin/env python3
"""Create exclusive diagnostic plots and a small review bundle from saved data.

No optimization, inference, or rollout occurs here. Failed/missing metrics stay
null; rejected trajectories are labelled and never presented as execution.
Every generated figure has its source hashes and plotted numbers in a sidecar.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "scripts")]
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
from shapely.geometry import box

from reconciliation.gp_se2 import interpolate_interval, sample_gp
from reconciliation.gp_se2_diagnostics import CLASSIFICATIONS, file_sha256, load_frozen_case
from plot_gp_se2_01 import geometry


METHODS = ("M2_GP_NO_OBSTACLE", "M3_GP_CONSTRAINED")


def read(path):
    return json.loads(Path(path).read_text())


def plain(value):
    if isinstance(value, np.ndarray):
        return plain(value.tolist())
    if isinstance(value, np.generic):
        return plain(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(k): plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    return value


def write(path, data):
    with Path(path).open("x") as stream:
        json.dump(plain(data), stream, indent=2, allow_nan=False)
        stream.write("\n")


def table(path, rows):
    keys = list(dict.fromkeys(k for row in rows for k in row))
    with Path(path).open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def save_plot(fig, path, title, numeric_data, sources, *, evidence_kind, notes=None):
    """Save without overwrite; numeric_data is exactly what callers plotted."""
    path = Path(path)
    if path.exists() or path.with_suffix(".json").exists():
        raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.suptitle(title, fontsize=11)
    fig.text(.015, .01, "GP-SE2-DIAG-01 | saved diagnostics | no new execution | adjacent JSON: values and source hashes",
             fontsize=7, color="#555555")
    fig.tight_layout(rect=(0, .035, 1, .94))
    fig.savefig(path, dpi=160)
    plt.close(fig)
    source_hashes = {str(Path(p).resolve()): file_sha256(p) for p in sources}
    source_hashes[str(Path(__file__).resolve())] = file_sha256(__file__)
    payload = dict(image=path.name, image_sha256=file_sha256(path), evidence_kind=evidence_kind,
                   source_hashes=source_hashes, numeric_data=numeric_data,
                   missing_values="null; never filled with zero", notes=notes or [],
                   executed_trajectory=False)
    write(path.with_suffix(".json"), payload)
    return dict(path=str(path), sidecar=str(path.with_suffix(".json")),
                sha256=file_sha256(path), sidecar_sha256=file_sha256(path.with_suffix(".json")))


def verify_plot_sidecar(path):
    sidecar = Path(path)
    record = read(sidecar)
    missing = [p for p, digest in record["source_hashes"].items()
               if not Path(p).is_file() or file_sha256(p) != digest]
    image = sidecar.with_name(record["image"])
    return dict(valid=not missing and image.is_file() and file_sha256(image) == record["image_sha256"],
                changed_source_paths=missing, image=str(image))


def history_data(result):
    """Read actual callbacks and the actual initial check, never interpolate."""
    n = int(result["variable_count"]) // 5
    samples = []
    initial = next((r for r in result.get("candidate_checks", []) if r["iterate"] == "initial"), None)
    if initial is not None:
        report = initial.get("constraint_report", {})
        samples.append(dict(iteration=0, elapsed_s=0., objective=initial.get("objective"),
            equality_residuals=report.get("equality_residuals"), inequality_margins=report.get("inequality_margins"),
            source="saved initial checker"))
    samples.extend(result.get("callback_snapshots", []))
    data = dict(elapsed_s=[], iteration=[], objective=[], lateral_max_abs_m_s=[],
                linear_acceleration_excess_m_s2=[], angular_acceleration_excess_rad_s2=[], source=[])
    for sample in samples:
        eq, iq = sample.get("equality_residuals"), sample.get("inequality_margins")
        lateral = None if eq is None or len(eq) < n else float(np.max(np.abs(eq[:n])))
        linear = angular = None
        if iq is not None and len(iq) >= 8*3*n:
            # Original _motion_inequalities has eight blocks, each with support,
            # midpoint, support samples for every interval (including both sides).
            count = 3*n
            linear = max(0., -float(np.min(iq[4*count:6*count])))
            angular = max(0., -float(np.min(iq[6*count:8*count])))
        for key, value in dict(elapsed_s=sample.get("elapsed_s"), iteration=sample.get("iteration"),
                              objective=sample.get("objective"), lateral_max_abs_m_s=lateral,
                              linear_acceleration_excess_m_s2=linear,
                              angular_acceleration_excess_rad_s2=angular,
                              source=sample.get("source", "actual SLSQP callback")).items():
            data[key].append(value)
    return data


def trajectory_data(result, *, full_accepted=False):
    """A sampled candidate is not labelled accepted until the full check passes."""
    if result.get("candidate_found") and result.get("support_poses") is not None:
        poses, twists = result["support_poses"], result["support_twists"]
        label = "FULL accepted GP candidate (not executed)" if full_accepted else "REJECTED / not full-accepted sampled candidate"
        kind = "accepted_candidate" if full_accepted else "rejected_sampled_candidate"
    else:
        poses, twists = result.get("latest_support_poses"), result.get("latest_support_twists")
        label, kind = "REJECTED latest iterate (no candidate)", "rejected_latest_iterate"
    if poses is None or twists is None:
        return dict(available=False, label="NO SAVED TRAJECTORY", kind="missing", poses_world=None)
    poses, twists = np.asarray(poses), np.asarray(twists)
    c = result["config"]
    knots = np.linspace(0, c["horizon_s"], len(poses))
    query = np.unique(np.r_[np.linspace(0, c["horizon_s"], round(c["horizon_s"]/.005)+1), knots])
    pp, vv, aa = sample_gp(knots, poses, twists, query)
    _, _, sa = interpolate_interval(poses[:-1, None], twists[:-1, None], poses[1:, None], twists[1:, None],
                                    c["support_dt_s"], np.array([0., 1.]))
    return plain(dict(available=True, label=label, kind=kind, times_s=query, poses_world=pp,
                body_twists=vv, body_accelerations=aa, support_times_s=knots,
                one_sided_times_s=np.column_stack([knots[:-1], knots[1:]]).ravel(),
                one_sided_accelerations=sa.reshape(-1, 3),
                one_sided_side=np.tile(["right", "left"], len(knots)-1),
                evaluation_grid_max_step_s=float(np.max(np.diff(query))), execution_available=False))


def profiling_parts(result):
    """Disjoint plotting levels; inclusive caller times are never stacked."""
    profile = result["profiling"]
    numeric = profile["numerical_component_seconds"]
    parts = dict(
        gp_prior=numeric.get("gp_prior_residual", 0.)+numeric.get("gp_prior_whitening", 0.),
        gp_interpolation=numeric.get("collocation_gp_interpolation", 0.)+numeric.get("gate_gp_interpolation", 0.),
        environment_query=numeric.get("environment_query", 0.),
        other_evaluate=profile["other_evaluate_inclusive_seconds"],
        solver_outside_evaluate=max(0., result["solve_wall_time_s"]-profile["evaluate_inclusive_seconds"]),
        seed_or_restoration=result.get("pre_solve_budget_cost_s", 0.),
        setup=result["setup_wall_time_s"], post_validation=result["post_solve_validation_time_s"])
    counts = dict(objective=result["objective_evaluations"], equality=result["equality_evaluations"],
                  inequality=result["inequality_evaluations"], unique_vectors=profile["unique_vector_evaluations"])
    return dict(disjoint_wall_seconds=parts, function_counts=counts,
                disjoint_total_seconds=sum(parts.values()),
                measured_total_seconds=result.get("total_pre_setup_solve_post_wall_time_s", result["total_setup_solve_post_wall_time_s"]),
                measured_bookkeeping_seconds=profile["measured_instrumentation_bookkeeping_seconds"],
                estimated_timer_seconds=profile["calibrated_timer_call_estimate_seconds"],
                overhead_note=profile["instrumentation_overhead_scope"])


def _history_figure(records, destination, title, evidence_kind):
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    data, sources = {}, []
    fields = ("lateral_max_abs_m_s", "linear_acceleration_excess_m_s2", "angular_acceleration_excess_rad_s2")
    labels = ("max midpoint |body vy| [m/s]", "linear acceleration excess [m/s²]", "angular acceleration excess [rad/s²]")
    for label, path, result, _ in records:
        history = history_data(result)
        data[label] = history
        sources.append(path)
        for axis, field in zip(axes, fields):
            axis.plot(history["elapsed_s"], history[field], ".-", ms=3, label=label)
    tolerance = records[0][2]["config"]["equality_tolerance"] if records else 1e-5
    for axis, label in zip(axes, labels):
        axis.axhline(tolerance, color="red", ls="--", label=f"original numeric tolerance {tolerance:g}")
        axis.set(ylabel=label, yscale="symlog", ylim=(0, None))
        axis.set_yscale("symlog", linthresh=1e-6)
        axis.grid(alpha=.25)
    axes[0].legend(fontsize=7, ncol=2)
    axes[-1].set_xlabel("elapsed solver wall time [s]; initial + actual callbacks only")
    return save_plot(fig, destination, title, dict(histories=data, tolerance=tolerance), sources,
                     evidence_kind=evidence_kind, notes=["Missing callback evaluations remain null; no unrecorded history inferred."])


def plot_existing(run, output):
    source = run / "existing_failure_audit/audit.json"
    audit = read(source)
    counts = audit["summary"]["classification_counts"]
    fig, ax = plt.subplots(figsize=(12, 5))
    names = ["A: initial feasible", "B: collocation fail", "C: collocation pass / dense fail", "D: dense feasible", "E: numeric error", "F: missing evidence"]
    values = [counts[name] for name in CLASSIFICATIONS]
    bars = ax.bar(np.arange(6), values, color=["#348c64", "#c55745", "#bb7f28", "#348c64", "#9b5783", "#888888"])
    ax.bar_label(bars)
    ax.set(xticks=np.arange(6), xticklabels=names, ylabel="saved attempts", ylim=(0, max(values)+5))
    ax.tick_params(axis="x", labelrotation=15)
    files = [save_plot(fig, output / "existing_rejection_classification.png", "Historical 40 GP starts: diagnostic class; termination kept separate",
                        dict(classifications=list(CLASSIFICATIONS), counts=values,
                             termination_counts=audit["summary"]["termination_counts"]), [source], evidence_kind="ACTUAL_SAVED_OPTIMIZATION_DIAGNOSTIC")]
    matrix, ids = [], []
    for record in audit["attempts"]:
        ids.append(f'{record["case_id"]} / {record["method"]} / start {record["initialization_index"]}')
        values = []
        for site in ("support", "midpoint", "off-collocation"):
            rows = [r for r in record["latest"].get("residuals", []) if r["family"] == "lateral_velocity" and r["site"] == site]
            values.append(None if not rows else max(max(abs(r["actual_minimum"]), abs(r["actual_maximum"])) for r in rows))
        matrix.append(values)
    fig, ax = plt.subplots(figsize=(12, 5))
    for index, site in enumerate(("support", "midpoint", "off-collocation")):
        ax.plot(np.arange(1, len(matrix)+1), [r[index] for r in matrix], ".-", label=site)
    ax.axhline(1e-5, color="red", ls="--", label="original tolerance 1e-5 m/s")
    ax.set_yscale("symlog", linthresh=1e-10)
    ax.set(xlabel="saved attempt number; exact IDs in adjacent JSON", ylabel="maximum |body vy| [m/s]")
    ax.grid(alpha=.25); ax.legend()
    files.append(save_plot(fig, output / "existing_lateral_by_location.png", "Historical latest iterates: support / midpoint / off-collocation",
                           dict(attempt_ids=ids, site_order=["support", "midpoint", "off-collocation"],
                                maximum_lateral_m_s=matrix, tolerance_m_s=1e-5), [source], evidence_kind="ACTUAL_SAVED_OPTIMIZATION_DIAGNOSTIC"))
    return files


def _is_selected(summary, path):
    selected = summary.get("selected_start")
    if isinstance(selected, int):
        return path.stem == f"start_{selected:02d}"
    return selected is not None and (str(selected) in (path.name, path.stem, str(path)) or Path(str(selected)).name == path.name)


def actual_records(run, method):
    records = []
    for phase in ("baseline", "single_change"):
        directory = run / "actual_event" / phase / method
        summary = read(directory / "summary.json")
        for path in sorted(directory.glob("start_*.json")):
            result = read(path)
            full = bool(summary.get("full_candidate_found") and _is_selected(summary, path))
            records.append((f"{phase} {path.stem}", path, result, full))
    if not records:
        raise ValueError(f"no completed actual-event starts for {method}")
    return records


def plot_actual(run, output, primary, case_id):
    files, comparison = [], []
    loaded = load_frozen_case(primary, case_id)
    c, context, environment = loaded["config"]["formulation"], loaded["context"], loaded["environment"]
    for method in METHODS:
        records = actual_records(run, method)
        status_paths = [run / "actual_event" / phase / method / "summary.json" for phase in ("baseline", "single_change")]
        status_paths += [run / "actual_event" / phase / method / "independent_acceptance.json"
                         for phase in ("baseline", "single_change")
                         if (run / "actual_event" / phase / method / "independent_acceptance.json").is_file()]
        phase_summaries = {phase: read(run / "actual_event" / phase / method / "summary.json")
                           for phase in ("baseline", "single_change")}
        directory = output / method
        files.append(_history_figure(records, directory / "constraint_history.png", f"{case_id} | {method}\nOriginal baseline vs one selected change; actual callback history", "ACTUAL_EVENT_OPTIMIZATION_DIAGNOSTIC"))
        traces = {label: trajectory_data(result, full_accepted=full) for label, _, result, full in records}
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        for label, trace in traces.items():
            if not trace["available"]:
                continue
            velocity, acceleration = np.asarray(trace["body_twists"]), np.asarray(trace["body_accelerations"])
            side = np.asarray(trace["one_sided_accelerations"])
            for index, values in enumerate((velocity[:, 1], acceleration[:, 0], acceleration[:, 2])):
                line = axes[index].plot(trace["times_s"], values, label=f'{label}: {trace["label"]}', alpha=.9)[0]
                if index:
                    axes[index].scatter(trace["one_sided_times_s"], side[:, 0 if index == 1 else 2],
                                        color=line.get_color(), marker=".", s=8, alpha=.55)
        for axis, limit, unit in zip(axes, (c["equality_tolerance"], c["a_v_max"], c["a_w_max"]),
                                    ("body vy [m/s]", "linear acceleration [m/s²]", "angular acceleration [rad/s²]")):
            for sign in (-1, 1): axis.axhline(sign*limit, color="red", ls="--")
            axis.set_ylabel(unit); axis.grid(alpha=.25)
        axes[0].legend(fontsize=6.5, ncol=1, loc="upper right")
        axes[-1].set_xlabel("time after fixed boundary B [s]; dots include both knot derivative sides")
        files.append(save_plot(fig, directory / "lateral_and_acceleration.png", f"{case_id} | {method}\nGP trajectories only: accepted status requires independent full check; no execution",
                               dict(traces=traces, lateral_tolerance_m_s=c["equality_tolerance"], linear_acceleration_limit_m_s2=c["a_v_max"],
                                    angular_acceleration_limit_rad_s2=c["a_w_max"], inequality_tolerance=c["inequality_tolerance"]),
                               [path for _, path, _, _ in records]+status_paths, evidence_kind="ACTUAL_EVENT_OPTIMIZATION_DIAGNOSTIC"))
        fig, (context_ax, ax) = plt.subplots(1, 2, figsize=(17, 8))
        original = dict(OLD=np.asarray(context["old_world"]), F_native=np.load(loaded["case_directory"] / "F_native.npy"),
                        F_common=loaded["common_reference"])
        all_xy = [poses[:, :2] for poses in original.values()]
        all_xy += [np.asarray(t["poses_world"])[:, :2] for t in traces.values() if t["available"]]
        all_xy.append(np.asarray(context["B_world"])[None, :2])
        xy = np.vstack(all_xy); lo, hi = xy.min(axis=0)-.55, xy.max(axis=0)+.55
        clip = box(lo[0], lo[1], hi[0], hi[1])
        ax.set_facecolor("#e7e3df")
        geometry(ax, environment.workspace.intersection(clip), facecolor="#f2faf0", edgecolor="#8eac83")
        geometry(ax, environment.obstacles.intersection(clip), facecolor="#8c8c8c", edgecolor="#555555")
        for label, poses, style, color in (("original OLD", original["OLD"], "--", "#2976b4"),
                                          ("original FRESH native", original["F_native"], "--", "#b83582"),
                                          ("original F_common", original["F_common"], ":", "#4d4d4d")):
            ax.plot(poses[:, 0], poses[:, 1], style, c=color, label=label, lw=2)
        for label, trace in traces.items():
            if trace["available"]:
                poses = np.asarray(trace["poses_world"])
                ax.plot(poses[:, 0], poses[:, 1], "-" if trace["kind"] == "accepted_candidate" else ":",
                        lw=1.5, label=f'{label}: {trace["label"]}')
        boundary = np.asarray(context["B_world"]); goal = np.asarray(loaded["goal_route"]["goal_world"])
        ax.scatter(*boundary[:2], c="black", s=45, label="fixed B", zorder=7)
        ax.arrow(*boundary[:2], .25*np.cos(boundary[2]), .25*np.sin(boundary[2]), width=.004, head_width=.05, color="black", zorder=7)
        radius = loaded["config"]["footprint"]["radius_m"]
        ax.add_patch(Circle(boundary[:2], radius, fill=False, ec="black", label=f"footprint radius {radius:g} m"))
        ax.add_patch(Circle(boundary[:2], radius+c["required_clearance"], fill=False, ls=":", ec="black"))
        ax.scatter(*goal[:2], marker="*", c="#348c64", s=90, label="unchanged original goal", zorder=7)
        ax.add_patch(Circle(goal[:2], c["goal_position_tolerance"], color="#348c64", alpha=.15))
        ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), xlabel="Hospital world x [m]", ylabel="Hospital world y [m]")
        ax.set_title("Local view: overlapping curves remain at true scale", fontsize=10)
        ax.set_aspect("equal"); ax.grid(alpha=.2); ax.legend(fontsize=6.5, loc="upper left", bbox_to_anchor=(1.02, 1))
        context_lo, context_hi = xy.min(axis=0)-2., xy.max(axis=0)+2.
        context_clip = box(context_lo[0], context_lo[1], context_hi[0], context_hi[1])
        context_ax.set_facecolor("#e7e3df")
        geometry(context_ax, environment.workspace.intersection(context_clip), facecolor="#f2faf0", edgecolor="#8eac83")
        geometry(context_ax, environment.obstacles.intersection(context_clip), facecolor="#8c8c8c", edgecolor="#555555")
        for label, poses, style, color in (("original OLD", original["OLD"], "--", "#2976b4"),
                                          ("original FRESH native", original["F_native"], "--", "#b83582"),
                                          ("original F_common", original["F_common"], ":", "#4d4d4d")):
            context_ax.plot(poses[:, 0], poses[:, 1], style, c=color, lw=1.8)
        # Match local-panel curve colours without altering coordinates.
        for label, trace in traces.items():
            if trace["available"]:
                poses = np.asarray(trace["poses_world"])
                context_ax.plot(poses[:, 0], poses[:, 1], "-" if trace["kind"] == "accepted_candidate" else ":", lw=1.5)
        context_ax.scatter(*boundary[:2], c="black", s=35, zorder=7)
        context_ax.add_patch(Circle(boundary[:2], radius, fill=False, ec="black"))
        context_ax.scatter(*goal[:2], marker="*", c="#348c64", s=70, zorder=7)
        context_ax.set(xlim=(context_lo[0], context_hi[0]), ylim=(context_lo[1], context_hi[1]),
                       xlabel="Hospital world x [m]", ylabel="Hospital world y [m]",
                       title="Context: actual obstacles (grey), known workspace (green)")
        context_ax.set_aspect("equal"); context_ax.grid(alpha=.2)
        files.append(save_plot(fig, directory / "world_xy.png", f"{case_id} | {method}\nFrozen references, boundary, static obstacle/workspace oracle; no new execution",
            dict(original=original, boundary_world=boundary, goal_world=goal, traces=traces, axes_bounds_m=[*lo, *hi],
                 context_axes_bounds_m=[*context_lo, *context_hi],
                 footprint_radius_m=radius, required_clearance_m=c["required_clearance"],
                 workspace_wkt=environment.workspace.intersection(clip).wkt, obstacles_wkt=environment.obstacles.intersection(clip).wkt,
                 context_workspace_wkt=environment.workspace.intersection(context_clip).wkt,
                 context_obstacles_wkt=environment.obstacles.intersection(context_clip).wkt),
            [path for _, path, _, _ in records]+status_paths+list(loaded["hashes"])+
            [Path(primary)/"environment/geometry/projected"/name for name in ("workspace.wkb", "obstacles.wkb")],
            evidence_kind="ACTUAL_EVENT_OPTIMIZATION_DIAGNOSTIC"))
        profiles = {label: profiling_parts(result) for label, _, result, _ in records}
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        names = list(profiles); bottom = np.zeros(len(names))
        for component in next(iter(profiles.values()))["disjoint_wall_seconds"]:
            values = [profiles[name]["disjoint_wall_seconds"][component] for name in names]
            axes[0].bar(np.arange(len(names)), values, bottom=bottom, label=component)
            bottom += values
        for i, count in enumerate(("objective", "equality", "inequality", "unique_vectors")):
            axes[1].bar(np.arange(len(names))+(i-1.5)*.2, [profiles[name]["function_counts"][count] for name in names], .2, label=count)
        for axis in axes:
            axis.set_xticks(np.arange(len(names)), names, rotation=22); axis.legend(fontsize=7); axis.grid(axis="y", alpha=.2)
        axes[0].set_ylabel("disjoint wall time components [s]"); axes[1].set_ylabel("calls / distinct vectors (includes FD probes)")
        files.append(save_plot(fig, directory / "profiling.png", f"{case_id} | {method}\nMeasured component timings; caller-inclusive levels are not added",
                               profiles, [path for _, path, _, _ in records]+status_paths,
                               evidence_kind="ACTUAL_EVENT_OPTIMIZATION_DIAGNOSTIC",
                               notes=["Per-start timing plot; additional method-level independent full-check timing appears separately in method summaries and actual_event_comparison.csv.",
                                      {phase: {key: summary.get(key) for key in ("full_independent_validation_wall_s", "seed_construction_wall_s", "seed_independent_validation_wall_s")}
                                       for phase, summary in phase_summaries.items()}]))
        for label, path, result, full in records:
            trace = traces[label]
            comparison.append(dict(method=method, phase=label.split()[0], start=path.stem,
                termination=result["termination"], solver_success=result["solver_success"],
                initial_feasible=result["initial_feasible"], sampled_candidate_found=result["candidate_found"],
                selected_full_candidate=full, returned_initial_unchanged=result["returned_initial_unchanged"],
                recovered_from_infeasible=result["recovered_from_infeasible"], objective_delta=result["objective_delta"],
                selected_objective_improved_beyond_ftol=result["selected_objective_improved_beyond_ftol"],
                solve_wall_time_s=result["solve_wall_time_s"], setup_wall_time_s=result["setup_wall_time_s"],
                seed_or_restoration_wall_time_s=result.get("pre_solve_budget_cost_s", 0.),
                total_pre_setup_solve_post_wall_time_s=result.get("total_pre_setup_solve_post_wall_time_s", result["total_setup_solve_post_wall_time_s"]),
                post_solve_validation_time_s=result["post_solve_validation_time_s"],
                method_full_independent_validation_wall_s=phase_summaries[label.split()[0]].get("full_independent_validation_wall_s"),
                method_seed_construction_wall_s=phase_summaries[label.split()[0]].get("seed_construction_wall_s"),
                method_seed_independent_validation_wall_s=phase_summaries[label.split()[0]].get("seed_independent_validation_wall_s"),
                method_timing_scope="method full-check cost repeated on start rows; count once per phase/method",
                objective_evaluations=result["objective_evaluations"], equality_evaluations=result["equality_evaluations"],
                inequality_evaluations=result["inequality_evaluations"], unique_vectors=result["profiling"]["unique_vector_evaluations"],
                plotted_trajectory_kind=trace["kind"],
                plotted_lateral_max_m_s=None if not trace["available"] else float(np.max(np.abs(np.asarray(trace["body_twists"])[:, 1]))),
                source_sha256=file_sha256(path)))
    return files, comparison


def plot_synthetic_recovery(run, output):
    files, rows = [], []
    for name in ("S0", "S1", "S2"):
        records = []
        for seed in ("known", "perturbed"):
            path = run / "synthetic_fixtures/solver" / name / f"{seed}.json"
            result = read(path)
            records.append((seed, path, result, False))
            rows.append(dict(fixture=name, seed=seed, termination=result["termination"],
                initial_feasible=result["initial_feasible"], sampled_candidate_found=result["candidate_found"],
                returned_initial_unchanged=result["returned_initial_unchanged"], recovered_from_infeasible=result["recovered_from_infeasible"],
                objective_delta=result["objective_delta"], objective_improved_beyond_ftol=result["selected_objective_improved_beyond_ftol"],
                solve_wall_time_s=result["solve_wall_time_s"], source_sha256=file_sha256(path)))
        files.append(_history_figure(records, output / name / "perturbation_recovery_history.png",
            f"{name} | SYNTHETIC 3 s fixture\nKnown-feasible initial seed vs deterministic perturbation; not research performance evidence",
            "SYNTHETIC_MATHEMATICAL_DIAGNOSTIC"))
    return files, rows


def build_review_bundle(run, plots, comparison, recovery):
    """Allowlist only small review content; raw datasets/checkpoints never enter."""
    directory = run / "review_bundle"
    archive = run / "review_bundle.zip"
    if directory.exists() or archive.exists():
        raise FileExistsError("review bundle exists")
    directory.mkdir()
    audit = read(run / "existing_failure_audit/summary.json")
    math = read(run / "synthetic_fixtures/math/summary.json")
    decision = read(run / "decision.json")
    summary = dict(created_utc=datetime.now(timezone.utc).isoformat(), source_run=str(run),
        existing_failure_audit=audit, mathematical_fixtures=math, synthetic_solver_results=recovery,
        actual_event_comparison=comparison,
        full_candidate_found=any(row["selected_full_candidate"] for row in comparison),
        no_new_execution=True, synthetic_is_not_performance_evidence=True,
        scope="one fixed actual event, one selected change, unchanged physical acceptance")
    write(directory / "summary.json", summary)
    write(directory / "decision.json", decision)
    shutil.copy2(run / "existing_failure_audit/existing_rejections.csv", directory / "existing_rejections.csv")
    shutil.copy2(run / "synthetic_fixtures/math/fixture_results.csv", directory / "fixture_results.csv")
    table(directory / "synthetic_solver_results.csv", recovery)
    table(directory / "actual_event_comparison.csv", comparison)
    representatives = [p for p in plots if Path(p["path"]).name == "existing_lateral_by_location.png"
        or ("M3_GP_CONSTRAINED" in p["path"] and Path(p["path"]).name in ("world_xy.png", "constraint_history.png", "lateral_and_acceleration.png", "profiling.png"))
        or ("S2" in Path(p["path"]).parts and Path(p["path"]).name == "perturbation_recovery_history.png")]
    for record in representatives:
        path = Path(record["path"])
        name = "__".join(path.relative_to(run / "plots").parts)
        shutil.copy2(path, directory / name)
        sidecar = read(record["sidecar"])
        sidecar["image"] = name
        write((directory / name).with_suffix(".json"), sidecar)
    for relative in ("S3/truth_vs_gp_reconstruction.png", "collocation_blind_spot/lateral_velocity_vs_time.png"):
        source = run / "synthetic_fixtures/math" / relative
        name = "synthetic__"+relative.replace("/", "__")
        shutil.copy2(source, directory / name)
        numeric = source.parent / "numeric_results.json"
        write((directory / name).with_suffix(".json"), dict(image=name, image_sha256=file_sha256(source),
            source_hashes={str(source): file_sha256(source), str(numeric): file_sha256(numeric)},
            numeric_data=read(numeric), evidence_kind="SYNTHETIC_MATHEMATICAL_DIAGNOSTIC", executed_trajectory=False,
            notes=["Copied original fixture plot; full numeric source included to make this review bundle self-contained."]))
    (directory / "README.md").write_text(
        "# GP-SE2-DIAG-01 review bundle\n\n"
        "One fixed actual event: episode_001_repeat_01/handoff_002. Numerical acceptance is unchanged.\n\n"
        "- `summary.json`: historical rejection classes, synthetic evidence, and actual-event outcomes.\n"
        "- `existing_rejections.csv`: all 40 historical starts; termination remains separate.\n"
        "- `fixture_results.csv` and `synthetic_solver_results.csv`: mathematical reconstruction and known/perturbed solver checks.\n"
        "- `actual_event_comparison.csv`: baseline and the single selected change; missing metrics remain empty/null.\n"
        "- `decision.json`: pre-execution choice and interpretation limits.\n"
        "- PNGs have adjacent JSON source hashes and plotted numeric values.\n\n"
        "SYNTHETIC figures are mathematical diagnostics, not research performance evidence. REJECTED traces are not candidates. "
        "Accepted candidates were not executed: no MPC performance or navigation benefit is claimed. "
        "No raw RGB, dataset, checkpoint, or external source is included. Absolute source paths identify local provenance; "
        "the numeric data and tables required for this review are included here.\n")
    allow = {".json", ".csv", ".md", ".png"}
    files = sorted(directory.iterdir())
    if any(not p.is_file() or p.suffix not in allow for p in files):
        raise ValueError("unexpected review bundle content")
    with zipfile.ZipFile(archive, "x", compression=zipfile.ZIP_DEFLATED) as bundle:
        for path in files:
            bundle.write(path, arcname="review_bundle/"+path.name)
    return dict(directory=str(directory), archive=str(archive), archive_sha256=file_sha256(archive),
                archive_bytes=archive.stat().st_size, files=[dict(name=p.name, sha256=file_sha256(p)) for p in files])


def plot_run(run):
    run = Path(run).resolve()
    # Fail before creating output if a phase is incomplete.
    for path in (run / "decision.json", run / "existing_failure_audit/completion.json",
                 run / "synthetic_fixtures/solver/S2/perturbed.json"):
        if not path.is_file(): raise FileNotFoundError(path)
    for method in METHODS:
        actual_records(run, method)
    source = read(run / "source.json")
    protocol = read(run / "protocol.json")
    output = run / "plots"
    output.mkdir(exist_ok=False)
    files = plot_existing(run, output / "existing_failure_audit")
    synthetic, recovery = plot_synthetic_recovery(run, output / "synthetic_fixtures")
    files += synthetic
    actual, comparison = plot_actual(run, output / "actual_event", source["primary_run"], protocol["actual_case_id"])
    files += actual
    table(output / "actual_event_comparison.csv", comparison)
    table(output / "synthetic_solver_results.csv", recovery)
    bundle = build_review_bundle(run, files, comparison, recovery)
    validation = [verify_plot_sidecar(record["sidecar"]) for record in files]
    result = dict(created_utc=datetime.now(timezone.utc).isoformat(), plots=files, review_bundle=bundle,
                  valid=all(item["valid"] for item in validation), sidecar_validation=validation,
                  no_new_optimization=True, no_new_execution=True)
    write(output / "manifest.json", result)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    args = parser.parse_args()
    result = plot_run(args.run)
    print(json.dumps(dict(valid=result["valid"], plots=len(result["plots"]), review_zip=result["review_bundle"]["archive"])))

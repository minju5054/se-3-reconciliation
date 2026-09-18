#!/usr/bin/env python3
"""Exclusive saved-result DIAG-02 plots and a small allowlisted review bundle.

This command never solves, infers, executes, or changes acceptance. All eight
completed comparison cells are required, including failures. Every PNG has the
plotted values and input/source hashes in its JSON sidecar.
"""
from __future__ import annotations

import argparse
import csv
import html
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
import numpy as np
from shapely.geometry import box

from reconciliation.gp_se2 import interpolate_interval, sample_gp
from reconciliation.gp_se2_diagnostics import file_sha256, load_frozen_case
from plot_gp_se2_01 import geometry
from plot_gp_se2_diag_01 import history_data, plain

METHODS = ("M2_GP_NO_OBSTACLE", "M3_GP_CONSTRAINED")
MODES = ("FD_BASELINE", "SUPPLIED_JAC")
SEEDS = ("I0_FRESH", "I1_DECEL")
COLORS = {"FD_BASELINE": "#2775b6", "SUPPLIED_JAC": "#d97720"}
PLOT_NAMES = ("objective_iteration", "objective_wall_time", "collocation_time",
              "best_full_feasible_objective", "lateral", "accelerations",
              "world_xy", "evaluation_counts", "profiling")


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    with Path(path).open("x") as stream:
        json.dump(plain(value), stream, indent=2, allow_nan=False)
        stream.write("\n")


def records(run):
    """Preflight all eight cells before creating any plotting output."""
    run = Path(run)
    for name in ("execution_completed.json", "source.json", "protocol.json", "config_snapshot.yaml",
                 "actual_event/input_snapshot.json"):
        if not (run / name).is_file():
            raise FileNotFoundError(run / name)
    completed = read(run / "execution_completed.json")
    if completed.get("solve_count") != 8:
        raise ValueError("all eight solves must be complete before plotting")
    output = {}
    for method in METHODS:
        for seed in SEEDS:
            for mode in MODES:
                directory = run / "actual_event" / method / mode / seed
                files = {name: directory / f"{name}.json" for name in
                         ("solver_result", "result_summary", "full_acceptance", "post_full_checks", "setup")}
                for path in files.values():
                    if not path.is_file():
                        raise FileNotFoundError(path)
                record = {name: read(path) for name, path in files.items()}
                summary = record["result_summary"]
                if (summary["method"], summary["derivative_mode"], summary["initialization"]) != (method, mode, seed):
                    raise ValueError(f"comparison cell identity mismatch: {directory}")
                if bool(summary["selected_full_feasible"]) != bool(record["full_acceptance"]["full_feasible"]):
                    raise ValueError(f"selected acceptance mismatch: {directory}")
                record.update(paths=files, directory=directory, method=method, mode=mode, seed=seed)
                output[method, seed, mode] = record
    return output


def callback_history(result):
    result_data = history_data(result)
    initial = next((x for x in result["candidate_checks"] if x["iterate"] == "initial"), None)
    reports = ([] if initial is None else [initial.get("collocation")]) + [
        x.get("collocation") for x in result.get("callback_snapshots", [])]
    for target, key in (("equality_max", "maximum_equality_residual"),
                        ("inequality_max", "maximum_inequality_violation")):
        result_data[target] = [None if r is None else r.get(key) for r in reports]
    return result_data


def best_full_history(record):
    """Post-certified saved iterates, never acceptance asserted at callback time."""
    rows = sorted(record["post_full_checks"]["rows"], key=lambda r: (r["discovery_time_s"], r["iterate"]))
    points, best = [], None
    for row in rows:
        if row.get("full_feasible") and row.get("objective") is not None:
            best = row["objective"] if best is None else min(best, row["objective"])
        points.append(dict(discovery_time_s=row["discovery_time_s"], best_full_objective=best,
                           iterate=row["iterate"], full_feasible=bool(row.get("full_feasible")),
                           vector_sha256=row.get("vector_sha256")))
    initial = next((r for r in rows if r["iterate"] == "initial"), None)
    return dict(points=points, seed_objective=None if initial is None else initial.get("objective"),
                seed_full_feasible=False if initial is None else bool(initial.get("full_feasible")),
                certification="post-solve; discovery time does not imply certification at that time",
                missing_before_first_full_feasible="null, not zero")


def sample_support(poses, twists, config):
    if poses is None or twists is None:
        return None
    poses, twists = np.asarray(poses, float), np.asarray(twists, float)
    knots = np.linspace(0., config["horizon_s"], len(poses))
    times = np.unique(np.r_[np.linspace(0., config["horizon_s"], int(np.ceil(config["horizon_s"] / .005)) + 1), knots])
    pp, vv, aa = sample_gp(knots, poses, twists, times)
    _, _, sided = interpolate_interval(poses[:-1, None], twists[:-1, None], poses[1:, None], twists[1:, None],
                                       config["support_dt_s"], np.array([0., 1.]))
    return plain(dict(times_s=times, poses_world=pp, body_twists=vv, body_accelerations=aa,
                      one_sided_times_s=np.column_stack((knots[:-1], knots[1:])).ravel(),
                      one_sided_accelerations=sided.reshape(-1, 3), execution_available=False))


def trajectory_records(record, problem):
    result = record["solver_result"]
    seed_pose, seed_twist = problem.unpack(np.asarray(result["initial_vector"]))
    seed = sample_support(seed_pose, seed_twist, result["config"])
    final = sample_support(result.get("latest_support_poses"), result.get("latest_support_twists"), result["config"])
    selected = sample_support(result.get("support_poses"), result.get("support_twists"), result["config"])
    summary = record["result_summary"]
    return dict(seed=seed, final=final, selected=selected,
                final_label="FULL-feasible final (not executed)" if summary["final_full_feasible"] else "REJECTED final (not executed)",
                selected_label="FULL-accepted selected (not executed)" if summary["selected_full_feasible"] else "NO FULL-accepted selected candidate",
                selected_full_feasible=bool(summary["selected_full_feasible"]),
                seed_full_feasible=bool(summary["initial_full_feasible"]))


def disjoint_profile(record):
    result, setup, summary = record["solver_result"], record["setup"], record["result_summary"]
    primal = float(result["profiling"]["evaluate_inclusive_seconds"])
    derivatives = float(sum(result["derivative_callback_seconds"].values()))
    remainder = float(result["solve_wall_time_s"] - primal - derivatives)
    if remainder < -1e-6:
        raise ValueError("profiling clocks overlap; cannot produce disjoint chart")
    parts = dict(primal_evaluator=primal, supplied_derivative_callbacks=derivatives,
                 solver_and_bookkeeping_outside_callbacks=max(0., remainder),
                 solver_setup=result["setup_wall_time_s"],
                 case_loading_seed=setup["case_loading_seed_preparation_s"],
                 derivative_graph=setup["derivative_graph_construction_s"],
                 compile_and_warmup=setup["compilation_first_call_warmup_s"],
                 original_post_validation=result["post_solve_validation_time_s"],
                 independent_full_validation=summary["independent_full_validation_time_s"])
    return dict(disjoint_wall_seconds=parts, total_seconds=sum(parts.values()),
                prepared_solve_seconds=result["solve_wall_time_s"],
                shared_environment_loading_s=setup["shared_environment_loading_s"],
                optional_optimality_diagnostic_seconds=summary.get("optimality_diagnostic_time_s"),
                optional_optimality_provider_setup_seconds=summary.get("optimality_provider_setup_s"),
                provider_statistics=result.get("derivative_provider_stats"),
                primal_nested_breakdown=result["profiling"],
                note="Only disjoint_wall_seconds is stacked. Shared environment load and optional optimality diagnostics are separate; nested provider/primal timings are never added again.")


def _draw(ax, x, y, label, **kwargs):
    values = np.array([np.nan if value is None else value for value in y], dtype=float)
    ax.plot(x, values, label=label, **kwargs)
    return bool(np.any(np.isfinite(values)))


def _axes(rows=1, *, height=None):
    fig, axes = plt.subplots(rows, 2, figsize=(13.5, height or (4.4 * rows + 1.2)), squeeze=False)
    for column, seed in enumerate(SEEDS):
        axes[0, column].set_title(seed)
    for ax in axes.flat:
        ax.grid(alpha=.23)
    return fig, axes


def _finish_axis(ax, xlabel, ylabel, *, log=False):
    ax.set(xlabel=xlabel, ylabel=ylabel)
    if log:
        arrays = [np.asarray(line.get_ydata(orig=False), dtype=float).ravel() for line in ax.get_lines()]
        values = np.concatenate(arrays) if arrays else np.array([])
        values = values[np.isfinite(values)]
        # Autoscale's linear padding can cross zero even when all recorded
        # objectives are positive. Decide the scale from the actual data,
        # never from that padding, to avoid invented negative log decades.
        if len(values) and values.min() > 0:
            if values.max() / values.min() >= 10:
                ax.set_yscale("log")
                margin = (values.max() / values.min()) ** .05
                ax.set_ylim(values.min() / margin, values.max() * margin)
        else:
            ax.set_yscale("symlog", linthresh=1e-7)
            if len(values) and values.min() >= 0:
                ax.set_ylim(bottom=0.)
    handles, _ = ax.get_legend_handles_labels()
    if handles:
        ax.legend(fontsize=7, loc="best")


def save_plot(fig, path, title, numeric, sources, *, dpi=160):
    path = Path(path)
    if dpi < 160:
        raise ValueError("at least 160 dpi required")
    if path.exists() or path.with_suffix(".json").exists():
        raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.suptitle(title, fontsize=12)
    fig.text(.012, .012, "GP-SE2-DIAG-02 | saved optimization diagnostics | no execution | missing values stay unavailable", fontsize=8)
    fig.tight_layout(rect=(0, getattr(fig, "_diag02_bottom_margin", .04), 1, .93))
    fig.savefig(path, dpi=dpi)
    plt.close(fig)
    write(path.with_suffix(".json"), dict(image=path.name, image_sha256=file_sha256(path), dpi=dpi,
          source_sha256=sources, numeric_data=numeric, no_new_execution=True,
          missing_value_policy="null or explicitly unavailable; never fabricated zero"))
    return dict(path=str(path), sidecar=str(path.with_suffix(".json")),
                sha256=file_sha256(path), sidecar_sha256=file_sha256(path.with_suffix(".json")))


def plot_method(run, method, cells, loaded, snapshot, *, dpi=160):
    selected = {(seed, mode): cells[method, seed, mode] for seed in SEEDS for mode in MODES}
    source_paths = [run / name for name in ("source.json", "protocol.json", "config_snapshot.yaml", "actual_event/input_snapshot.json")]
    source_paths += [p for record in selected.values() for p in record["paths"].values()]
    source_paths += [Path(__file__), ROOT / "scripts/plot_gp_se2_diag_01.py", ROOT / "scripts/plot_gp_se2_01.py",
                     ROOT / "src/reconciliation/gp_se2.py", ROOT / "src/reconciliation/se2.py"]
    sources = {str(p.resolve()): file_sha256(p) for p in source_paths}
    # Loaded environment provenance is kept in source.json; additionally hash
    # the actual WKB used to render the ground and obstacles.
    from shapely import to_wkb
    import hashlib
    environment = loaded["environment"]
    geometry_hash = dict(workspace_wkb_sha256=hashlib.sha256(to_wkb(environment.workspace)).hexdigest(),
                         obstacles_wkb_sha256=hashlib.sha256(to_wkb(environment.obstacles)).hexdigest())
    history = {key: callback_history(record["solver_result"]) for key, record in selected.items()}
    traces = {key: trajectory_records(record, loaded["problem"]) for key, record in selected.items()}
    directory, files = run / "plots" / method, []
    def save(fig, name, data, note):
        files.append(save_plot(fig, directory / f"{name}.png", f"{snapshot['case_id']} | {method}\n{note}", data, sources, dpi=dpi))
    for name, xfield, xlabel in (("objective_iteration", "iteration", "actual iteration; seed at 0"),
                                ("objective_wall_time", "elapsed_s", "prepared solver wall time [s]")):
        fig, axes = _axes()
        numeric = {}
        for col, seed in enumerate(SEEDS):
            for mode in MODES:
                h = history[seed, mode]; record = selected[seed, mode]
                _draw(axes[0, col], h[xfield], h["objective"], f"{mode}: {record['solver_result']['termination']}", color=COLORS[mode], marker=".", ms=3)
                numeric[f"{seed}/{mode}"] = dict(x=h[xfield], objective=h["objective"], history_source=h["source"])
            _finish_axis(axes[0, col], xlabel, "original objective", log=True)
        save(fig, name, numeric, "Seed + saved actual callbacks; no unrecorded iteration history")
    fig, axes = _axes(2)
    numeric = {}
    for col, seed in enumerate(SEEDS):
        for mode in MODES:
            h = history[seed, mode]; numeric[f"{seed}/{mode}"] = h
            for row, field in enumerate(("equality_max", "inequality_max")):
                _draw(axes[row, col], h["elapsed_s"], h[field], mode, color=COLORS[mode], marker=".", ms=3)
        for row, key in enumerate(("equality_tolerance", "inequality_tolerance")):
            limit = selected[seed, MODES[0]]["solver_result"]["config"][key]
            axes[row, col].axhline(limit, color="#b22222", ls="--", label=f"frozen tolerance {limit:g}")
            _finish_axis(axes[row, col], "prepared solver wall time [s]", "maximum equality residual" if row == 0 else "maximum inequality violation", log=True)
    save(fig, "collocation_time", numeric, "Original collocation checks; inequality rows retain their original mixed physical units")
    fig, axes = _axes()
    numeric = {}
    for col, seed in enumerate(SEEDS):
        for index, mode in enumerate(MODES):
            h = best_full_history(selected[seed, mode]); numeric[f"{seed}/{mode}"] = h
            points = h["points"]
            available = _draw(axes[0, col], [r["discovery_time_s"] for r in points], [r["best_full_objective"] for r in points], mode,
                              color=COLORS[mode], drawstyle="steps-post", marker=".")
            if h["seed_objective"] is not None:
                axes[0, col].axhline(h["seed_objective"], color=COLORS[mode], ls=":", alpha=.65,
                                    label=f"{mode} seed baseline ({'full feasible' if h['seed_full_feasible'] else 'infeasible'})")
            if not available:
                axes[0, col].text(.03, .93 - .08 * index, f"{mode}: N/A — no full-feasible saved iterate", transform=axes[0, col].transAxes, color=COLORS[mode], fontsize=8)
        _finish_axis(axes[0, col], "recorded discovery wall time [s]", "best post-certified full-feasible objective", log=True)
        axes[0, col].set_xlim(0., max(selected[seed, mode]["solver_result"]["solve_wall_time_s"] for mode in MODES) * 1.02)
    save(fig, "best_full_feasible_objective", numeric, "Post-solve certification: discovery times are retrospective; no feasible value before first discovery")
    for name, fields, labels, limits in (("lateral", [("body_twists", 1)], ["body lateral velocity [m/s]"], ["equality_tolerance"]),
                                       ("accelerations", [("body_accelerations", 0), ("body_accelerations", 2)],
                                        ["linear acceleration [m/s²]", "angular acceleration [rad/s²]"], ["a_v_max", "a_w_max"])):
        fig, axes = _axes(len(fields)); numeric = {}
        for col, seed in enumerate(SEEDS):
            for mode in MODES:
                trace = traces[seed, mode]; numeric[f"{seed}/{mode}"] = trace
                for kind, style, alpha in (("final", "--", .75), ("selected", "-", 1.)):
                    if kind == "selected" and not trace["selected_full_feasible"]:
                        continue
                    series = trace[kind]
                    if series is None:
                        continue
                    for row, (field, component) in enumerate(fields):
                        _draw(axes[row, col], series["times_s"], np.asarray(series[field])[:, component],
                              f"{mode} {trace[kind + '_label']}", color=COLORS[mode], ls=style, alpha=alpha)
                        if field == "body_accelerations":
                            axes[row, col].scatter(series["one_sided_times_s"], np.asarray(series["one_sided_accelerations"])[:, component], s=7, color=COLORS[mode], alpha=.4)
                if trace["final"] is None:
                    axes[0, col].text(.02, .95 - .08 * MODES.index(mode), f"{mode}: NO SAVED FINAL TRAJECTORY", transform=axes[0, col].transAxes, fontsize=8)
            for row, key in enumerate(limits):
                limit = selected[seed, MODES[0]]["solver_result"]["config"][key]
                for sign in (-1, 1): axes[row, col].axhline(sign * limit, color="#b22222", ls=":")
                _finish_axis(axes[row, col], "time after fixed B [s]", labels[row], log=name == "lateral")
        save(fig, name, numeric, "Saved GP interpolation only; dashed final status labelled, solid selected only if full accepted; knot-side acceleration dots")
    fig, axes = _axes(height=9.2)
    fig._diag02_bottom_margin = .27
    raw = np.asarray(snapshot["F_native"])
    all_xy = [raw[:, :2], np.asarray(snapshot["context"]["B_world"])[None, :2]]
    for trace in traces.values():
        all_xy += [np.asarray(trace[kind]["poses_world"])[:, :2] for kind in ("seed", "final", "selected") if trace[kind] is not None]
    xy = np.vstack(all_xy); lo, hi = xy.min(axis=0) - .55, xy.max(axis=0) + .55
    clip = box(*lo, *hi)
    numeric = dict(traces={f"{seed}/{mode}": trace for (seed, mode), trace in traces.items()},
                   raw_fresh_world=raw, axis_limits_world_m=[lo, hi], aspect="equal", geometry=geometry_hash,
                   boundary_world=snapshot["context"]["B_world"], goal_world=snapshot["goal_route"]["goal_world"])
    for col, seed in enumerate(SEEDS):
        ax = axes[0, col]; ax.set_facecolor("#e7e3df")
        geometry(ax, environment.workspace.intersection(clip), facecolor="#eff6ec", edgecolor="#9da995")
        geometry(ax, environment.obstacles.intersection(clip), facecolor="#8c8c8c", edgecolor="#555555")
        ax.plot(raw[:, 0], raw[:, 1], color="#933b83", ls=":", label="original raw FRESH world poses")
        for mode in MODES:
            trace = traces[seed, mode]
            for kind, style, alpha in (("seed", ":", .7), ("final", "--", .7), ("selected", "-", 1.)):
                if trace[kind] is None or (kind == "selected" and not trace["selected_full_feasible"]):
                    continue
                poses = np.asarray(trace[kind]["poses_world"])
                label = "seed (not executed)" if kind == "seed" else trace[kind + "_label"]
                ax.plot(poses[:, 0], poses[:, 1], ls=style, alpha=alpha, color=COLORS[mode], label=f"{mode} {label}")
        ax.scatter(*np.asarray(snapshot["context"]["B_world"])[:2], c="black", marker="o", label="fixed B", zorder=9)
        ax.scatter(*np.asarray(snapshot["goal_route"]["goal_world"])[:2], c="black", marker="*", label="original goal", zorder=9)
        ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), aspect="equal")
        _finish_axis(ax, "world X [m]", "world Y [m]")
        handles, labels = ax.get_legend_handles_labels()
        ax.get_legend().remove()
        fig.legend(handles, labels, loc="lower center", bbox_to_anchor=(.27 if col == 0 else .75, .065), fontsize=7)
    save(fig, "world_xy", numeric, "Same environment and metre scale; seed/final/accepted references, no execution or physical safety claim")
    fig, axes = _axes(); numeric = {}
    count_keys = ("objective_evaluations", "equality_evaluations", "inequality_evaluations")
    for col, seed in enumerate(SEEDS):
        labels = ["objective", "equality", "inequality", "unique primal", "obj gradient", "eq Jacobian", "ineq Jacobian"]
        for index, mode in enumerate(MODES):
            result = selected[seed, mode]["solver_result"]
            counts = [result[key] for key in count_keys] + [result["profiling"]["unique_vector_evaluations"]] + [
                result["derivative_calls"][key] for key in ("objective_gradient", "equality_jacobian", "inequality_jacobian")]
            numeric[f"{seed}/{mode}"] = dict(labels=labels, counts=counts)
            axes[0, col].bar(np.arange(len(labels)) + (index - .5) * .36, counts, width=.36, label=mode, color=COLORS[mode])
        axes[0, col].set_xticks(np.arange(len(labels)), labels, rotation=35, ha="right")
        _finish_axis(axes[0, col], "recorded function / derivative calls", "count", log=True)
        # Counts are nonnegative integers; sub-count decades carry no data.
        axes[0, col].set_yscale("symlog", linthresh=1.)
        axes[0, col].set_ylim(bottom=0.)
    save(fig, "evaluation_counts", numeric, "Measured counts include finite-difference probes; zero derivative callbacks in baseline is a recorded zero")
    fig, axes = _axes(height=7); numeric = {}
    for col, seed in enumerate(SEEDS):
        profiles = [disjoint_profile(selected[seed, mode]) for mode in MODES]
        for mode, profile in zip(MODES, profiles): numeric[f"{seed}/{mode}"] = profile
        bottom = np.zeros(2)
        for component in profiles[0]["disjoint_wall_seconds"]:
            values = [profile["disjoint_wall_seconds"][component] for profile in profiles]
            axes[0, col].bar(np.arange(2), values, bottom=bottom, label=component.replace("_", " "))
            bottom += values
        axes[0, col].set_xticks(np.arange(2), MODES)
        _finish_axis(axes[0, col], "mode; cold components shown separately", "disjoint elapsed time [s]")
    save(fig, "profiling", numeric, "Prepared solve + separate cold setup and validation; nested timers never double counted")
    return files


def _index(cells, files, run, *, bundled=False):
    lines = ['<!doctype html><meta charset="utf-8"><title>GP-SE2-DIAG-02 review</title>',
             '<style>body{font:16px system-ui;max-width:1250px;margin:30px auto}table{border-collapse:collapse}td,th{padding:8px;border:1px solid #ccc}img{max-width:100%}</style>',
             '<h1>GP-SE2-DIAG-02 saved optimization review</h1>',
             '<p>Eight fixed comparison cells, including failures. No new inference, controller rollout or physical safety claim. Full feasibility is post-certified; discovery times are retrospective. Missing values remain unavailable.</p>',
             '<table><tr><th>Method / seed / mode</th><th>Termination</th><th>Initial / final / selected full feasible</th><th>Evidence</th></tr>']
    for (method, seed, mode), record in cells.items():
        s = record["result_summary"]
        rel = Path("results") / method / mode / seed / "result_summary.json" if bundled else record["paths"]["result_summary"].relative_to(run)
        links = f'<a href="{rel.as_posix()}">summary</a>'
        if not bundled:
            for label in ("solver_result", "post_full_checks", "full_acceptance", "setup"):
                links += f' · <a href="{record["paths"][label].relative_to(run).as_posix()}">{label}</a>'
        lines.append(f'<tr><td>{method}<br>{seed}<br>{mode}</td><td>{html.escape(s["termination"])}</td><td>{s["initial_full_feasible"]} / {s["final_full_feasible"]} / {s["selected_full_feasible"]}</td><td>{links}</td></tr>')
    lines.append('</table>')
    for record in files:
        path = Path(record["path"]).relative_to(run).as_posix()
        sidecar = Path(record["sidecar"]).relative_to(run).as_posix()
        lines.append(f'<h2>{html.escape(path)}</h2><a href="{sidecar}">Plotted numbers and provenance</a><br><a href="{path}"><img loading="lazy" src="{path}" alt="{html.escape(path)}"></a>')
    return "\n".join(lines) + "\n"


def generate(run, *, loaded=None, dpi=160):
    run = Path(run).resolve()
    if dpi < 160:
        raise ValueError("at least 160 dpi required")
    for name in ("plots", "plot_manifest.json", "index.html", "review_bundle", "review_bundle.zip"):
        if (run / name).exists():
            raise FileExistsError(run / name)
    cells = records(run)
    snapshot, source = read(run / "actual_event/input_snapshot.json"), read(run / "source.json")
    if loaded is None:
        loaded = load_frozen_case(source["primary_run"], snapshot["case_id"])
    (run / "plots").mkdir()
    files = []
    for method in METHODS:
        files += plot_method(run, method, cells, loaded, snapshot, dpi=dpi)
    if len(files) != 18:
        raise ValueError("expected exactly nine plots per method")
    manifest = dict(plot_count=len(files), comparison_cell_count=len(cells), dpi=dpi, no_new_execution=True,
                    records=[{**record, "path":str(Path(record["path"]).relative_to(run)),
                              "sidecar":str(Path(record["sidecar"]).relative_to(run))} for record in files])
    write(run / "plot_manifest.json", manifest)
    (run / "index.html").write_text(_index(cells, files, run))
    bundle = run / "review_bundle"; bundle.mkdir()
    allowed = []
    for record in files:
        allowed.extend([Path(record["path"]), Path(record["sidecar"])])
    allowed.append(run / "plot_manifest.json")
    for source_path in allowed:
        target = bundle / source_path.relative_to(run)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_path, target)
    summaries = []
    for (method, seed, mode), record in cells.items():
        target = bundle / "results" / method / mode / seed
        target.mkdir(parents=True)
        for label in ("result_summary", "setup", "post_full_checks"):
            shutil.copy2(record["paths"][label], target / f"{label}.json")
        summaries.append(record["result_summary"])
    write(bundle / "summary.json", dict(cells=summaries, comparison_cell_count=8, no_new_execution=True))
    columns = ["method", "derivative_mode", "initialization", "termination", "iterations", "initial_full_feasible",
               "final_full_feasible", "selected_full_feasible", "initial_objective", "final_objective", "selected_objective", "solve_wall_time_s"]
    with (bundle / "comparison.csv").open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns); writer.writeheader()
        for row in summaries: writer.writerow({key:row.get(key) for key in columns})
    (bundle / "index.html").write_text(_index(cells, files, run, bundled=True))
    (bundle / "README.md").write_text("# GP-SE2-DIAG-02 review bundle\n\nOpen index.html. All eight fixed solver cells, including failures, are present.\n\n18 PNGs have adjacent plotted numeric data and input/source hashes. Original traces are saved optimization iterates or GP interpolation, never executions. Full-feasibility histories are certified after solving and located at recorded discovery times. Cold setup, prepared solve and validation costs are disjoint; shared environment load and optional optimality diagnostics remain separate.\n\nThis allowlist contains only figures, numeric sidecars, compact summaries/setup/check tables and this index. It excludes raw source data, environment geometry/grids, checkpoints, solver vectors and external source. Source hashes refer to the original local files, which are not included. Synthetic plot tests are not research evidence.\n")
    inventory = [dict(path=str(p.relative_to(bundle)), bytes=p.stat().st_size, sha256=file_sha256(p))
                 for p in sorted(bundle.rglob("*")) if p.is_file()]
    write(bundle / "manifest.json", dict(allowlisted_files=inventory, file_count=len(inventory),
          self_excluded="manifest.json", excluded=["raw data", "environment", "checkpoints", "solver vectors", "external source"]))
    with zipfile.ZipFile(run / "review_bundle.zip", "x", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(bundle.rglob("*")):
            if path.is_file(): archive.write(path, arcname=str(path.relative_to(bundle)))
    return dict(plot_count=18, comparison_cell_count=8, index=str(run / "index.html"),
                bundle=str(run / "review_bundle.zip"), bundle_bytes=(run / "review_bundle.zip").stat().st_size)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(); parser.add_argument("run", type=Path); parser.add_argument("--dpi", type=int, default=160)
    arguments = parser.parse_args()
    print(json.dumps(generate(arguments.run, dpi=arguments.dpi), indent=2))

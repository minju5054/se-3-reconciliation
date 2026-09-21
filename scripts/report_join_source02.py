#!/usr/bin/env python3
"""Static JOIN-SOURCE-02 review from saved records; no runtime or new queries.

Reads aggregate/development_ledger.json and per-condition frozen manifests,
worker summaries and evaluations. Optional confirmation/call-count ledgers are
reported as unavailable if absent. It never substitutes replay for online data.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import html
import json
from pathlib import Path
import sys
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from reconciliation.join_source02 import pointing_diagnostics


MAP = {"A": "A_OFF", "B": "B_ON", "SHAM": "A_SHAM"}
COLORS = {"A": "#2875b9", "B": "#cf423d", "SHAM": "#814eab"}


def read(path, default=None):
    return json.loads(path.read_text()) if path.is_file() else default


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.parent.mkdir(exist_ok=True, parents=True)
    with path.open("x") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def csv_write(path, rows, fields):
    with path.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) if row.get(key) is not None else "N/A" for key in fields})


def path_from(value, run):
    p = Path(value)
    return p if p.is_absolute() else run/p


def sources(paths):
    return {str(p): sha(p) for p in paths if p is not None and p.is_file()}


def geometry_draw(ax, geom, color, alpha=.3, linewidth=1.):
    if geom.is_empty:
        return
    if geom.geom_type == "Polygon":
        x, y = geom.exterior.xy
        ax.fill(x, y, facecolor=color, edgecolor=color, alpha=alpha, linewidth=linewidth)
        for ring in geom.interiors:
            ax.fill(*ring.xy, facecolor="white", edgecolor=color, linewidth=.5)
    elif geom.geom_type in ("LineString", "LinearRing"):
        ax.plot(*geom.xy, color=color, alpha=alpha, linewidth=linewidth)
    elif hasattr(geom, "geoms"):
        for child in geom.geoms:
            geometry_draw(ax, child, color, alpha, linewidth)


def saved_geometry(manifest, run):
    import shapely
    placement_path = None
    for key in ("placement_path", "placement_file", "geometry_path"):
        if manifest.get(key):
            placement_path = path_from(manifest[key], run)
            break
    placement = read(placement_path, {}) if placement_path else {}
    if isinstance(manifest.get("placement"), dict):
        placement = manifest["placement"]
    elif isinstance(manifest.get("placement"), str):
        placement_path = path_from(manifest["placement"], run)
        placement = read(placement_path, {})
    geom_paths = [placement_path]
    obstacle, workspace, runtime = None, None, None
    environment = placement.get("environment_export", manifest.get("environment_export"))
    if environment:
        envpath = Path(environment)
        if not envpath.is_absolute():
            envpath = ROOT/envpath
        projected = envpath/"geometry/projected"
        for name in ("obstacles", "workspace"):
            p = projected/f"{name}.wkb"
            if p.is_file():
                geom_paths.append(p)
                if name == "obstacles":
                    obstacle = shapely.from_wkb(p.read_bytes())
                else:
                    workspace = shapely.from_wkb(p.read_bytes())
    if "projection" in placement:
        metadata = placement["projection"].get("metadata", placement["projection"])
        runtime = shapely.from_wkb(bytes.fromhex(metadata["obstacle_wkb_hex"]))
        if workspace is not None and metadata.get("unknown_wkb_hex"):
            workspace = workspace.difference(shapely.from_wkb(bytes.fromhex(metadata["unknown_wkb_hex"])))
        if placement.get("triangles_path"):
            geom_paths.append(path_from(placement["triangles_path"], run))
    elif placement.get("obstacle_wkb_hex"):
        runtime = shapely.from_wkb(bytes.fromhex(placement["obstacle_wkb_hex"]))
    elif placement.get("obstacle_wkb_path") or placement.get("obstacle_geometry_path"):
        p = path_from(placement.get("obstacle_wkb_path", placement.get("obstacle_geometry_path")), run)
        if p.is_file():
            geom_paths.append(p)
            runtime = (shapely.from_wkb(bytes.fromhex(read(p)["obstacle_wkb_hex"])) if p.suffix == ".json"
                       else shapely.from_wkb(p.read_bytes()))
    return placement, obstacle, workspace, runtime, geom_paths


def save_figure(fig, path, numbers, source_hashes):
    import matplotlib.pyplot as plt
    fig.savefig(path, dpi=160, facecolor="white", metadata={
        "Description": "JOIN-SOURCE-02 saved evidence; numeric/source hashes in adjacent JSON"})
    plt.close(fig)
    write(path.with_suffix(".json"), dict(plotted_numbers=numbers, source_sha256=source_hashes,
                                        png_sha256=sha(path), plotting_code_sha256=sha(Path(__file__))))


def condition_figures(run, output, record):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
    from PIL import Image
    import shapely
    condition_id = record["condition_id"]
    folder = run/"development"/condition_id
    manifest_path = folder/"frozen_input_manifest.json"
    if not manifest_path.is_file() and record.get("worker_output_path"):
        folder = path_from(record["worker_output_path"], run)
        manifest_path = folder/"frozen_input_manifest.json"
    if not manifest_path.is_file():
        manifest_path = run/"development/input_manifests"/f"{condition_id}.json"
    manifest = read(manifest_path, {})
    summary_path = folder/"summary.json"
    summary = read(summary_path, {})
    base = output/condition_id
    base.mkdir()
    source_files = [manifest_path, summary_path, folder/"evaluation.json", run/"protocol.json", run/"config_snapshot.yaml"]
    paths, responses, branch_status = {}, {}, {}
    for name in MAP:
        branch_path = folder/"branches"/name/"paired_result.json"
        branch = summary.get("branches", {}).get(name, read(branch_path, {}))
        if branch_path.is_file():
            source_files.append(branch_path)
        terminal = branch.get("terminal_prediction") or {}
        world = terminal.get("world")
        world_path = folder/"branches"/name/"chunks/terminal/world.npy"
        for raw in (world_path, world_path.with_name("raw_local.npy"), world_path.with_name("response.json")):
            if raw.is_file():
                source_files.append(raw)
        if world_path.is_file():
            saved_world = np.load(world_path, allow_pickle=False)
            if world is not None and not np.array_equal(saved_world, np.asarray(world)):
                raise ValueError(f"worker summary and saved world differ: {world_path}")
            world = saved_world
        if world is not None:
            array = np.asarray(world, dtype=float)
            if array.ndim == 2 and array.shape[1] == 3 and len(array) and np.isfinite(array).all():
                paths[name] = array
        responses[name] = pointing_diagnostics(terminal.get("response") or {})
        branch_status[name] = branch.get("status", "NOT_EXECUTED")
    placement, obstacle, workspace, runtime, geom_files = saved_geometry(manifest, run)
    source_files.extend(geom_files)
    hashes = sources(source_files)
    figures, links = [], []
    if paths:
        fig, ax = plt.subplots(figsize=(8.6, 6.3))
        for name, path in paths.items():
            ax.plot(path[:, 0], path[:, 1], "--o", color=COLORS[name], markersize=3,
                    linewidth=1.7, label=f"{name}: original predicted future ({len(path)} rows)")
        bounds = np.concatenate([p[:, :2] for p in paths.values()])
        obs = manifest.get("final_frames", {}).get("A", {}).get("pose_world")
        if obs:
            ax.plot(obs[0], obs[1], "k*", markersize=10, label="shared observation pose")
            bounds = np.vstack([bounds, np.asarray(obs[:2])])
            ax.arrow(obs[0], obs[1], .12*np.cos(obs[2]), .12*np.sin(obs[2]), head_width=.035,
                     color="black", length_includes_head=True)
        if runtime is not None:
            x0, y0, x1, y1 = runtime.bounds
            bounds = np.vstack([bounds, [x0, y0], [x1, y1]])
        lo, hi = bounds.min(axis=0)-.45, bounds.max(axis=0)+.45
        clip = shapely.box(*lo, *hi)
        if workspace is not None:
            geometry_draw(ax, workspace.intersection(clip), "#dbe7d8", .35)
        if obstacle is not None:
            geometry_draw(ax, obstacle.intersection(clip), "#515151", .5)
        if runtime is not None:
            geometry_draw(ax, runtime, "#cc8a27", .65)
            # Actual footprint edge requirement becomes center-path radius+.05.
            geometry_draw(ax, runtime.buffer(.25).boundary, "#af6610", .9, 1.1)
        handles, labels = ax.get_legend_handles_labels()
        if runtime is not None:
            handles += [Patch(facecolor="#cc8a27", alpha=.65), Patch(facecolor="none", edgecolor="#af6610")]
            labels += ["actual runtime prop mesh projection", "center-path outline: radius .20 + clearance .05 m"]
        ax.legend(handles, labels, loc="best", fontsize=7)
        ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), xlabel="world X [m]", ylabel="world Y [m]",
               title=f"{condition_id}\n{record.get('label', 'N/A')}")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=.15)
        fig.text(.5, .012, "DEVELOPMENT / COUNTERFACTUAL INPUT SCREENING — predicted paths, not online execution",
                 ha="center", fontsize=8)
        fig.tight_layout(rect=(0, .035, 1, 1))
        p = base/"paired_world.png"
        save_figure(fig, p, dict(world={k: v.tolist() for k, v in paths.items()}, observation_pose=obs,
                               axis_bounds_xy=[lo.tolist(), hi.tolist()], evaluation=record,
                               runtime_obstacle_wkb_hex=None if runtime is None else runtime.wkb_hex,
                               no_execution_trace=True), hashes)
        figures.append(p)
    finals = manifest.get("final_frames", {})
    image_data, originals = {}, {}
    for name in ("A", "B"):
        if finals.get(name, {}).get("path"):
            original = path_from(finals[name]["path"], run)
            if original.is_file():
                data = original.read_bytes()
                if hashlib.sha256(data).hexdigest() != finals[name].get("sha256"):
                    raise ValueError(f"original JPEG hash mismatch: {original}")
                target = base/f"original_{name}.jpg"
                with target.open("xb") as stream:
                    stream.write(data)
                image_data[name] = np.asarray(Image.open(original).convert("RGB"))
                originals[name] = original
                hashes[str(original)] = sha(original)
                links.append(target)
    if image_data:
        fig, axes = plt.subplots(1, 2, figsize=(12, 4.4))
        for ax, name in zip(axes, ("A", "B")):
            if name in image_data:
                ax.imshow(image_data[name])
            else:
                ax.text(.5, .5, "N/A: no saved RGB", ha="center")
            ax.set_title(f"{name}: obstacle {'off' if name == 'A' else 'on'}", fontsize=11)
            ax.axis("off")
        fig.suptitle(f"{condition_id}: actual captured paired inputs", fontsize=12)
        fig.text(.5, .025, "No annotation was sent to LightNav. These plots only display the saved JPEGs.", ha="center", fontsize=9)
        fig.tight_layout(rect=(0, .04, 1, .94))
        p = base/"paired_rgb.png"
        save_figure(fig, p, dict(image_paths={k: str(v) for k, v in originals.items()},
                               capture_frames={k: finals[k] for k in originals}), hashes)
        figures.append(p)
        fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))
        point_rows = {}
        for ax, name in zip(axes, MAP):
            rgb = image_data.get("A" if name == "SHAM" else name)
            diag = responses[name]
            pointing = diag["pointing"]
            available = rgb is not None and pointing.get("frame_size") == [rgb.shape[1], rgb.shape[0]]
            if rgb is not None:
                ax.imshow(rgb)
            if available:
                for key, color, marker in (("apos_px", "#00ffff", "x"), ("opos_px", "#ff00ff", "+")):
                    if pointing.get(key) is not None:
                        xy = pointing[key]
                        if len(xy) == 2 and np.isfinite(xy).all():
                            ax.plot(xy[0], xy[1], marker=marker, color=color, markersize=12, markeredgewidth=2, label=key)
                if ax.get_legend_handles_labels()[0]:
                    ax.legend(fontsize=8, loc="upper right")
            ax.set_title(f"{name}: target visible={diag['visible']}, stop={diag['stop']}\n"
                         f"APOS={pointing['apos_state']}; pixel frame {'matches' if available else 'N/A/unverified'}", fontsize=9)
            ax.axis("off")
            point_rows[name] = dict(raw_diagnostic=diag, overlay_coordinate_valid=available)
        fig.suptitle(f"{condition_id}: separate pointing diagnostic", fontsize=12)
        fig.text(.5, .02, "Diagnostic overlays were never model inputs. Target visibility is not obstacle visibility; pointing is not proof of avoidance.",
                 ha="center", fontsize=9)
        fig.tight_layout(rect=(0, .045, 1, .93))
        p = base/"pointing_diagnostic.png"
        save_figure(fig, p, point_rows, hashes)
        figures.append(p)
    details = dict(condition_id=condition_id, classification=record.get("label"), branch_status=branch_status,
                   figures=[str(p.relative_to(output)) for p in figures],
                   raw_rgb=[str(p.relative_to(output)) for p in links], sources_sha256=hashes,
                   pointing=responses, unavailable_curves_not_fabricated=True)
    write(base/"presentation.json", details)
    return details


def scalar_rows(records):
    output = []
    for r in records:
        row = dict(condition_id=r["condition_id"], label=r.get("label"), qualified=r.get("qualified", False),
                   availability=r.get("availability"), reason=r.get("reason"),
                   maximum_interior_separation_m=r.get("off_on", {}).get("maximum_interior_separation_m"),
                   maximum_reliable_tangent_difference_deg=r.get("off_on", {}).get("maximum_reliable_tangent_difference_deg"),
                   maximum_reliable_yaw_difference_deg=r.get("off_on", {}).get("maximum_reliable_yaw_difference_deg"),
                   sham_maximum_interior_separation_m=r.get("off_sham", {}).get("maximum_interior_separation_m"),
                   on_raw_hash_different=r.get("off_on_hash_different"),
                   target_progress=r.get("target_directed_progress"), bypass=r.get("bypass_plane_crossed"))
        for branch in MAP.values():
            for state in ("off", "on"):
                check = r.get("checks", {}).get(branch, {}).get(state, {})
                row[f"{branch}_{state}_clearance_m"] = check.get("minimum_clearance_m")
                row[f"{branch}_{state}_valid"] = check.get("clearance_valid")
        output.append(row)
    return output


def _cell(value):
    if value is None:
        return "N/A"
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


def table_html(rows, fields):
    head = "".join(f"<th>{html.escape(x)}</th>" for x in fields)
    body = "".join("<tr>"+"".join(f"<td>{html.escape(_cell(r.get(k)))}</td>" for k in fields)+"</tr>" for r in rows)
    return f"<div class='scroll'><table><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table></div>"


def report(run, output, bundle):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ledger_path = run/"aggregate/development_ledger.json"
    ledger = read(ledger_path)
    if ledger is None:
        raise FileNotFoundError(f"Saved development ledger required: {ledger_path}")
    records = ledger["records"]
    if output.exists() or bundle.exists():
        raise FileExistsError("review output/ZIP already exists; no overwrite")
    output.mkdir(parents=True)
    presentations = [condition_figures(run, output, r) for r in records]
    rows = scalar_rows(records)
    fields = list(rows[0]) if rows else ["condition_id", "label"]
    csv_write(output/"development_outcomes.csv", rows, fields)
    confirmation_path = run/"aggregate/confirmation_ledger.json"
    confirmation = read(confirmation_path)
    confirmation_records = [] if confirmation is None else confirmation.get("records", confirmation.get("episodes", []))
    attempted = sum(bool(r.get("attempted", r.get("status") not in (None, "NOT_RUN", "NOT_ATTEMPTED"))) for r in confirmation_records)
    qualified = sum(r.get("qualified") is True for r in confirmation_records)
    calls_path = run/"aggregate/actual_call_counts.json"
    counts = read(calls_path)
    if counts is None:
        dev_counts_path = run/"aggregate/actual_call_counts_development.json"
        if dev_counts_path.is_file():
            calls_path = dev_counts_path
            counts = dict(development=read(dev_counts_path), live_history_acquisition=None,
                          confirmation=None, startup_warmup=None, tests=None)
    selected = ledger.get("selected_condition_id")
    status = ledger.get("status", "NO_QUALIFYING_DEVELOPMENT_SCENARIO" if selected is None else "DEVELOPMENT_RESPONSE_ONLY")
    if attempted:
        status = ("ONLINE_SOURCE_REPRODUCED_IN_ALL_THREE_ATTEMPTS" if attempted == 3 and qualified == 3 else
                  "ONLINE_SOURCE_OBTAINED_WITH_LIMITED_REPRODUCIBILITY" if qualified else "ONLINE_SOURCE_NOT_REPRODUCED")
    source_bundle = run/"source_bundle/manifest.json"
    summary = dict(status=status, selected_development_condition_id=selected,
                   development_conditions_recorded=len(records), confirmation_attempted=attempted,
                   confirmation_qualified=qualified if attempted else None,
                   confirmation_qualification_display=f"{qualified}/{attempted} attempted" if attempted else "N/A — confirmation not run",
                   source_bundle_manifest=str(source_bundle) if source_bundle.is_file() and qualified else None,
                   source_bundle_reason=None if source_bundle.is_file() and qualified else "No qualifying online confirmation bundle recorded",
                   actual_calls=counts, missing_call_counts_are_not_zero=counts is None,
                   new_runtime_calls_by_reporter=0, development_is_online_evidence=False,
                   source_sha256=sources([ledger_path, calls_path, confirmation_path, run/"protocol.json", run/"config_snapshot.yaml"]))
    write(output/"reporting_summary.json", summary)
    write(output/"confirmation_status.json", dict(ledger=confirmation, **{k: summary[k] for k in (
        "confirmation_attempted", "confirmation_qualified", "confirmation_qualification_display")}))
    fig, ax = plt.subplots(figsize=(12, max(3.5, .50*max(1, len(rows)))))
    ax.axis("off")
    content = [[r["condition_id"], r["label"],
                _cell(r.get("A_OFF_on_clearance_m")), _cell(r.get("B_ON_on_clearance_m")),
                _cell(r.get("A_SHAM_on_clearance_m")), _cell(r.get("maximum_interior_separation_m"))] for r in rows]
    if content:
        tab = ax.table(cellText=content, colLabels=["Condition", "Development outcome", "OFF/on [m]", "ON/on [m]", "SHAM/on [m]", "Interior separation [m]"],
                       loc="center", cellLoc="left", colWidths=[.24, .27, .12, .12, .12, .13])
        tab.auto_set_font_size(False)
        tab.set_fontsize(8)
        tab.scale(1, 1.55)
    else:
        ax.text(.5, .5, "No development condition was executed", ha="center")
    ax.set_title("JOIN-SOURCE-02: complete development ledger\n"+summary["confirmation_qualification_display"], fontsize=12, pad=18)
    fig.tight_layout()
    save_figure(fig, output/"outcome_summary.png", rows, summary["source_sha256"])
    parts = ["<!doctype html><html><head><meta charset='utf-8'><title>JOIN-SOURCE-02 saved evidence</title>",
             "<style>body{font:16px system-ui;max-width:1280px;margin:32px auto;padding:0 20px;color:#202934}h1{font-size:27px}h2{margin-top:40px}img{max-width:100%;height:auto}table{border-collapse:collapse;width:100%;font-size:12px}td,th{border:1px solid #ccd3db;padding:7px;text-align:left}th{background:#eef2f6}.scroll{overflow:auto}pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#f5f7f9;padding:14px}.warning{background:#fff2d9;padding:16px}a{color:#155c99}</style></head><body>",
             "<h1>JOIN-SOURCE-02 — saved source evidence</h1>",
             f"<p><strong>{html.escape(status)}</strong></p>",
             "<p class='warning'>Development figures are counterfactual input screening through independent sessions, not online handoff or obstacle traversal. No GP, rigid or reconciliation comparison is presented.</p>",
             f"<p>Online confirmation: <strong>{html.escape(summary['confirmation_qualification_display'])}</strong>. Source bundle: {html.escape(summary['source_bundle_manifest'] or 'none')}.</p>",
             "<p><a href='development_outcomes.csv'>All development numbers (CSV)</a> · <a href='reporting_summary.json'>Source/call-count summary</a></p>",
             "<img src='outcome_summary.png' alt='Every recorded development condition'>",
             table_html(rows, ["condition_id", "label", "reason", "A_OFF_on_clearance_m", "B_ON_on_clearance_m", "A_SHAM_on_clearance_m", "maximum_interior_separation_m", "sham_maximum_interior_separation_m"])]
    parts += ["<h2>Actual calls and runtime</h2><p>Scopes are copied from the authoritative saved ledger. Missing scopes are N/A, never inferred as zero. Buffer requests are not model predictions; startup/warmup/history/confirmation remain separate.</p>",
              "<pre>"+html.escape(json.dumps(counts, indent=2) if counts is not None else "N/A — call-count ledger unavailable")+"</pre>"]
    if confirmation_records:
        parts += ["<h2>All confirmation records</h2><pre>"+html.escape(json.dumps(confirmation, indent=2))+"</pre>"]
    for p in presentations:
        parts += [f"<h2>{html.escape(p['condition_id'])}</h2><p>{html.escape(str(p['classification']))}</p>"]
        if not p["figures"]:
            parts += ["<p>N/A: no saved trajectory/image for this condition; no line was fabricated.</p>"]
        for figure in p["figures"]:
            sidecar = str(Path(figure).with_suffix(".json"))
            parts += [f"<p><a href='{html.escape(sidecar)}'>Numeric/source sidecar</a></p><img src='{html.escape(figure)}'>"]
        for raw in p["raw_rgb"]:
            parts += [f"<p><a href='{html.escape(raw)}'>Byte-identical original captured RGB: {html.escape(Path(raw).name)}</a></p>"]
    parts += ["<h2>Scope and interpretation</h2><p>Interior-only projections exclude finite endpoint gaps. Full original returned segments are checked without suffix deletion. Safe local prediction does not establish B→FRESH feasibility or executed obstacle traversal. Pointing fields are diagnostics, not proof of obstacle perception or decoder failure.</p></body></html>"]
    (output/"index.html").write_text("\n".join(parts))
    # This validation checks presentation/source parity, not scientific success.
    sidecars = list(output.rglob("*.json"))
    checks = {}
    for p in sidecars:
        record = read(p)
        if "png_sha256" in record:
            checks[str(p.relative_to(output))] = record["png_sha256"] == sha(p.with_suffix(".png"))
    validation = dict(valid=all(checks.values()), presentation_checks=checks,
                      condition_coverage=[r["condition_id"] for r in records],
                      new_inference_GP_MPC_rollout_calls=0,
                      source_sha256_unchanged=all(sha(Path(p)) == h for p, h in summary["source_sha256"].items()))
    validation["valid"] &= validation["source_sha256_unchanged"]
    write(output/"validation.json", validation)
    with zipfile.ZipFile(bundle, "x", compression=zipfile.ZIP_DEFLATED) as archive:
        for p in sorted(output.rglob("*")):
            if p.is_file():
                archive.write(p, str(p.relative_to(output)))
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", default="review")
    parser.add_argument("--bundle", default="review_bundle.zip")
    args = parser.parse_args()
    run = args.run.resolve()
    summary = report(run, run/args.output, run/args.bundle)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()

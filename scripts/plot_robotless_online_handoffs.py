#!/usr/bin/env python3
"""Plot saved genuine online streams after collection, without running inference."""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import html
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageOps, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from reconciliation.online_handoff_analysis import (
    FRAME_CONVENTION, METRIC_COLUMNS, VALID_STATUSES, interval_timing_summary, metrics_from_records, summarize_events,
)
from reconciliation.robotless_single_chunk import observation_to_world, validate_waypoints

PLOT_CONFIG = {
    "version": 1, "dpi": 160, "world_figsize_inches": [10.5, 8.3],
    "aspect": "equal", "zoom_min_halfwidth_m": .35, "zoom_projection_margin_m": .15,
    "raw_waypoint_yaw_glyph_length_m": .08, "state_yaw_glyph_length_m": .10,
    "geometry_scaling": False, "angle_exaggeration": False,
    "zoom_convention": "axis limits only; same fixed world coordinates",
    "lineage_convention": "post-switch displacements end at next activation pre-command state, or final saved state",
    "colors": {"old": "#1976D2", "fresh": "#C000AA", "past": "#444444", "inference": "#238B45", "post": "#E67E22"},
}


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(value, indent=2, allow_nan=False)+"\n"
    if path.exists():
        if read_json(path) != value:
            raise ValueError(f"refusing to replace changed derived JSON: {path}")
        return
    with path.open("x") as stream:
        stream.write(text)


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    path = Path(path)
    rows = list(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(dict.fromkeys(k for row in rows for k in row)) or ["episode_id", "event_id", "status", "reason"]
    import io
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    text = stream.getvalue()
    if path.exists():
        if path.read_text() != text:
            raise ValueError(f"refusing to replace changed derived CSV: {path}")
        return
    with path.open("x", newline="") as output:
        output.write(text)


def source_record(path, relative_root):
    path = Path(path).resolve()
    return {"path": str(path.relative_to(Path(relative_root).resolve())), "sha256": sha256(path), "bytes": path.stat().st_size}


def verified_ref(episode, record):
    path = (Path(episode)/record["path"]).resolve()
    if not path.is_relative_to(Path(episode).resolve()):
        raise ValueError("source reference escapes episode")
    if not path.is_file() or sha256(path) != record["sha256"] or path.stat().st_size != record["bytes"]:
        raise ValueError(f"source reference hash/size mismatch: {path}")
    return path


def load_event(episode, context_path):
    episode = Path(episode)
    context = read_json(context_path)
    execution = read_csv(episode/"execution.csv")
    commands = read_csv(episode/"commands.csv")
    arrays = {key: np.load(verified_ref(episode, context[key]), allow_pickle=False)
              for key in ("old_raw_local_ref", "old_world_ref", "fresh_raw_local_ref", "fresh_world_ref")}
    for array in arrays.values():
        validate_waypoints(array)
    fresh_anchor = context["R_obs"]
    old_meta_path = episode/"chunks"/context["old_chunk_id"]/"metadata.json"
    old_meta = read_json(old_meta_path)
    # Both fields are explicit agent capture poses; neither is a camera optical pose.
    old_anchor = old_meta.get("observation_pose_world", old_meta.get("R_obs", old_meta.get("observation", {}).get("pose_world")))
    if old_anchor is None:
        old_record = context.get("old_observation_pose_time", {})
        old_anchor = old_record.get("pose_world", old_record.get("pose"))
    if old_anchor is None:
        raise ValueError("OLD capture agent pose missing from saved metadata")
    for prefix, anchor in (("old", old_anchor), ("fresh", fresh_anchor)):
        expected = observation_to_world(anchor, arrays[prefix+"_raw_local_ref"])
        if not np.allclose(expected, arrays[prefix+"_world_ref"], rtol=0, atol=1e-10):
            raise ValueError(f"{prefix} world path is not anchored to its actual observation")
    metrics = metrics_from_records(context, execution, commands, arrays["old_world_ref"], arrays["fresh_world_ref"])
    history_source = []
    if context.get("history_snapshot_ref"):
        history_path = verified_ref(episode, context["history_snapshot_ref"])
        history = read_json(history_path)
        if context["actual_history_count"] != history["actual_history_count"] or context["history_full"] != history["history_full"] or context["request_history_frame_ids"] != history["chronological_history_frame_ids"]:
            raise ValueError("event history fields differ from saved actual request snapshot")
        metrics.update(model_input_unique_frame_count=history["model_input_unique_frame_count_reconstructed"],
            model_input_slots_including_padding=history["model_input_slots_including_processor_padding_reconstructed"],
            model_input_selection_is_source_reconstruction=True,
            history_capture_sim_span_s=history["capture_sim_span_s"], history_capture_host_span_s=history["capture_host_span_s"],
            bootstrap_frames_included=history["bootstrap_frames_included"], stationary_frames_included=history["stationary_frames_included"])
        history_source.append(history_path)
    timing_paths = {"loop": episode/"loop.jsonl", "controller": episode/"controller"/"events.jsonl", "captures": episode/"rgb_index.csv"}
    if all(path.exists() for path in timing_paths.values()):
        config_path = episode.parent.parent/"config_snapshot.yaml"
        config = {}
        if config_path.exists():
            import yaml
            config = yaml.safe_load(config_path.read_text())
            history_source.append(config_path)
        jsonl = lambda path: [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
        metrics["interval_timing"] = interval_timing_summary(context, metrics, execution, commands,
            jsonl(timing_paths["loop"]), jsonl(timing_paths["controller"]), read_csv(timing_paths["captures"]), pacing_config=config.get("online", {}))
        if metrics["interval_timing"]["real_time_pacing_valid"] is not None:
            metrics["timing_valid"] = metrics["interval_timing"]["real_time_pacing_valid"]
        metrics["timing_warnings"] = metrics["interval_timing"]["warnings"]
        history_source.extend(timing_paths.values())
    source_paths = history_source+[Path(context_path), episode/"execution.csv", episode/"commands.csv", old_meta_path]
    source_paths += [verified_ref(episode, context[key]) for key in arrays]
    return context, execution, commands, arrays, metrics, old_anchor, source_paths


def pose_rows(execution):
    return np.array([[float(row[key]) for key in ("x", "y", "yaw")] for row in execution])


def prepare_plot_inputs(context, execution, arrays, metrics, old_anchor, sources, run_root):
    poses = pose_rows(execution)
    b_index = metrics["B_stream_row"]
    inference_range = metrics["inference_execution"]["stream_rows_inclusive"]
    inference_poses = [] if inference_range is None else poses[inference_range[0]:inference_range[1]+1].tolist()
    post_start, post_end = metrics["post_switch_execution"]["stream_rows_inclusive"]
    return {
        "schema_version": 1, "episode_id": context["episode_id"], "event_id": context["event_id"],
        "source_files": [source_record(p, run_root) for p in sources],
        "frames": FRAME_CONVENTION, "plot_configuration": PLOT_CONFIG,
        "stream_ranges_inclusive": {"past": [0, b_index], "inference": inference_range, "post_switch": [post_start, post_end]},
        "stream_state_id_ranges": metrics["execution_range_state_ids"],
        "processing_sources": [source_record(ROOT / "scripts" / "plot_robotless_online_handoffs.py", ROOT), source_record(ROOT / "src" / "reconciliation" / "online_handoff_analysis.py", ROOT)],
        "displayed_coordinates": {
            "OLD_raw_world": arrays["old_world_ref"].tolist(), "FRESH_raw_world": arrays["fresh_world_ref"].tolist(),
            "executed_past": poses[:b_index+1].tolist(), "inference_executed": inference_poses,
            "post_switch_executed": poses[post_start:post_end+1].tolist(),
            "OLD_observation": list(old_anchor), "R_obs": context["R_obs"], "R_ready": context["R_ready"],
            "P": context.get("P"), "B": context["B"], "Q_xy": metrics["fresh_projection"]["Q_xy_world_m"],
        },
        "post_switch_execution_available": metrics["post_switch_execution_available"],
        "post_switch_unavailable_reason": metrics["post_switch_unavailable_reason"],
    }


def record_plot_outputs(directory):
    """Hash rendered derivatives so resume cannot silently accept another PNG."""
    directory = Path(directory)
    outputs = []
    for name in ("trajectory_world.png", "trajectory_boundary_zoom.png"):
        path = directory/name
        with Image.open(path) as image:
            dpi = list(image.info.get("dpi", (0., 0.)))
            if min(dpi) < 159.99:
                raise ValueError("per-event PNG resolution is below 160 dpi")
            size = list(image.size)
            image.verify()
        outputs.append({"path": name, "sha256": sha256(path), "bytes": path.stat().st_size, "pixels": size, "dpi": dpi})
    write_json(directory/"plot_outputs.json", {"plot_inputs_sha256": sha256(directory/"plot_inputs.json"), "images": outputs})


def _number(value, precision=3):
    return "unavailable" if value is None else f"{value:.{precision}f}"


def _yaw_arrows(ax, path, color, length, label=None, alpha=.7):
    path = np.asarray(path)
    if not len(path):
        return
    ax.quiver(path[:, 0], path[:, 1], length*np.cos(path[:, 2]), length*np.sin(path[:, 2]),
              angles="xy", scale_units="xy", scale=1, color=color, alpha=alpha,
              width=.003, headwidth=3.5, zorder=4, label=label)


def plot_event(context, metrics, inputs, output, *, zoom=False):
    """Draw two views of identical coordinates; only bounds differ for zoom."""
    output = Path(output)
    if output.exists():
        with Image.open(output) as image:
            image.verify()
        return
    coords, colors = inputs["displayed_coordinates"], PLOT_CONFIG["colors"]
    fig, ax = plt.subplots(figsize=PLOT_CONFIG["world_figsize_inches"])
    for key, label, color, style, width in (
        ("OLD_raw_world", "OLD raw predicted reference", colors["old"], "--", 1.5),
        ("FRESH_raw_world", "FRESH raw predicted reference", colors["fresh"], "-.", 1.5),
        ("executed_past", "Actual executed E(t), past", colors["past"], "-", 1.5),
        ("inference_executed", "Actual execution while inference pending", colors["inference"], "-", 3.),
        ("post_switch_executed", "Actual execution under this FRESH", colors["post"], ":", 3.),
    ):
        path = np.asarray(coords[key]).reshape(-1, 3)
        if not len(path):
            continue
        ax.plot(path[:, 0], path[:, 1], linestyle=style, color=color, linewidth=width, label=label, zorder=2 if "raw" in key else 3)
        if "raw" in key:
            ax.scatter(path[:, 0], path[:, 1], s=12, facecolors="none", edgecolors=color, zorder=3)
            _yaw_arrows(ax, path, color, PLOT_CONFIG["raw_waypoint_yaw_glyph_length_m"])
    offsets = {"OLD_observation": (6, 12), "R_obs": (6, -17), "R_ready": (6, 4), "P": (-42, -15), "B": (-14, 12)}
    for key, marker, color, label in (
        ("OLD_observation", "s", "#174A8B", "OLD observation agent pose"),
        ("R_obs", "s", "#8E128E", "FRESH observation R_obs"),
        ("R_ready", "^", "#007E86", "First ready-seen state R_ready"),
        ("P", "x", "#111111", "Previous actual state P"),
        ("B", "*", "#BC9700", "Switch pre-command state B"),
    ):
        if coords[key] is None:
            continue
        pose = np.asarray(coords[key])
        ax.scatter([pose[0]], [pose[1]], marker=marker, c=color, s=80 if key == "B" else 35, label=label, zorder=6)
        ax.annotate(key.replace("_observation", " obs"), pose[:2], xytext=offsets[key], textcoords="offset points", fontsize=8,
                    bbox={"facecolor": "white", "alpha": .65, "edgecolor": "none", "pad": 1}, zorder=7)
        _yaw_arrows(ax, [pose], color, PLOT_CONFIG["state_yaw_glyph_length_m"])
    q = np.asarray(coords["Q_xy"])
    ax.scatter([q[0]], [q[1]], marker="D", facecolors="white", edgecolors="black", s=34, label="FRESH raw-polyline projection Q", zorder=6)
    ax.annotate("Q", q, xytext=(6, 8), textcoords="offset points", fontsize=8, bbox={"facecolor": "white", "alpha": .65, "edgecolor": "none", "pad": 1}, zorder=7)
    if zoom:
        b = np.asarray(coords["B"][:2])
        half = max(PLOT_CONFIG["zoom_min_halfwidth_m"], float(np.max(np.abs(q-b)))+PLOT_CONFIG["zoom_projection_margin_m"])
        ax.set_xlim(b[0]-half, b[0]+half)
        ax.set_ylim(b[1]-half, b[1]+half)
    else:
        all_xy = np.vstack([np.asarray(coords[key])[:, :2] for key in ("OLD_raw_world", "FRESH_raw_world", "executed_past", "post_switch_executed")])
        low, high = all_xy.min(axis=0), all_xy.max(axis=0)
        center = (low+high)/2
        half = max(.35, float(np.max(high-low))*.60+.10)
        ax.set_xlim(center[0]-half, center[0]+half)
        ax.set_ylim(center[1]-half, center[1]+half)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("World X (m)")
    ax.set_ylabel("World Y (m)")
    ax.grid(alpha=.2)
    history = context.get("history_config", {})
    capacity = history.get("num_history_frames", history.get("capacity", "unknown"))
    sampling = history.get("sampling", history.get("history_storage", history.get("history_mode", "see saved history config")))
    view = "boundary zoom (axis limits only)" if zoom else "fixed world view"
    fig.suptitle(f"{context['episode_id']} / {context['event_id']} — {view}\n"
                 f"OLD {context['old_chunk_id']} → FRESH {context['fresh_chunk_id']} | {context['status']}", fontsize=11, y=.98)
    ax.set_title(f"RTT {_number(metrics['client_rtt_s'])} s | obs→switch sim age {_number(metrics['observation_to_switch_sim_s'])} s | "
                 f"RTF {_number(metrics['inference_rtf'])}\n"
                 f"history actual {context['actual_history_count']} | nominal {capacity} ({sampling}) | timing valid {metrics['timing_valid']} | "
                 f"projection {metrics['fresh_projection']['projection_location']}\n"
                 f"B→raw polyline {_number(metrics['e_perp_m'], 5)} m | actual incoming / FRESH local angle {_number(metrics['abs_e_dir_local_deg'], 2)}°",
                 fontsize=8, pad=12)
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", bbox_to_anchor=(.5, .055), ncol=2, fontsize=7, frameon=False)
    note = "Pose-yaw arrows: direction glyphs only (raw 0.08 m / state 0.10 m). Actual XY is unchanged."
    if not metrics["post_switch_execution_available"]:
        note += "\nPost-switch unavailable: "+str(metrics["post_switch_unavailable_reason"])
    fig.text(.5, .015, note, ha="center", fontsize=7)
    fig.subplots_adjust(left=.1, right=.96, top=.79, bottom=.27)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=PLOT_CONFIG["dpi"], facecolor="white")
    plt.close(fig)


def plot_invalid_diagnostic(episode, context_path, run_root, reason):
    """Available recorded paths/states remain useful when no activation exists.

    Without an actual B this is explicitly an observation-centered diagnostic,
    never an inferred handoff boundary or a fabricated execution segment.
    """
    episode, context_path = Path(episode), Path(context_path)
    context = read_json(context_path)
    execution_path = episode/"execution.csv"
    if not execution_path.exists():
        return None
    execution = read_csv(execution_path)
    if not execution:
        return None
    poses = pose_rows(execution)
    sources = [context_path, execution_path]
    paths = {}
    for prefix in ("old", "fresh"):
        record = context.get(prefix+"_world_ref")
        if record:
            path = verified_ref(episode, record)
            paths[prefix] = validate_waypoints(np.load(path, allow_pickle=False))
            sources.append(path)
    directory = context_path.parent/"plots"
    directory.mkdir(exist_ok=True)
    center_pose = context.get("B") or context.get("R_ready") or context.get("R_obs") or poses[-1].tolist()
    center = np.asarray(center_pose)[:2]
    inputs = {
        "schema_version": 1, "episode_id": context["episode_id"], "event_id": context["event_id"],
        "diagnostic_only": True, "status": context["status"], "reason": reason,
        "actual_boundary_available": context.get("B") is not None,
        "frames": FRAME_CONVENTION, "plot_configuration": PLOT_CONFIG,
        "source_files": [source_record(p, run_root) for p in sources],
        "stream_ranges_inclusive": {"episode_execution": [0, len(poses)-1]},
        "displayed_coordinates": {"actual_episode_execution": poses.tolist(),
            **{prefix+"_raw_world": path.tolist() for prefix, path in paths.items()},
            "R_obs": context.get("R_obs"), "R_ready": context.get("R_ready"), "B": context.get("B")},
        "post_switch_execution_available": False,
        "post_switch_unavailable_reason": "event has no validated first FRESH command activation",
    }
    inputs_path = directory/"plot_inputs.json"
    write_json(inputs_path, inputs)
    outputs = []
    for zoom in (False, True):
        output = directory/("trajectory_boundary_zoom.png" if zoom else "trajectory_world.png")
        outputs.append(output)
        if output.exists():
            continue
        fig, ax = plt.subplots(figsize=(10, 8))
        ax.plot(poses[:, 0], poses[:, 1], color="#444444", label="Actual recorded episode E(t), no FRESH attribution")
        for prefix, path in paths.items():
            color = PLOT_CONFIG["colors"][prefix]
            ax.plot(path[:, 0], path[:, 1], color=color, linestyle="--" if prefix == "old" else "-.", label=prefix.upper()+" raw predicted reference")
            _yaw_arrows(ax, path, color, .08)
        for key, color in (("R_obs", "#8E128E"), ("R_ready", "#007E86"), ("B", "#BC9700")):
            pose = context.get(key)
            if pose is not None:
                ax.scatter(pose[0], pose[1], color=color, label=key)
                ax.annotate(key, pose[:2], xytext=(5, 5), textcoords="offset points")
        if zoom:
            half = .5
            title = "Diagnostic boundary zoom" if context.get("B") is not None else "Diagnostic observation/readiness zoom — no activation boundary exists"
        else:
            points = np.vstack([poses[:, :2], *(path[:, :2] for path in paths.values())])
            low, high = points.min(axis=0), points.max(axis=0)
            center = (low+high)/2
            half = max(.5, float(np.max(high-low))*.60+.10)
            title = "Diagnostic fixed world view"
        ax.set_xlim(center[0]-half, center[0]+half); ax.set_ylim(center[1]-half, center[1]+half)
        ax.set_aspect("equal", adjustable="box"); ax.grid(alpha=.2)
        ax.set_xlabel("World X (m)"); ax.set_ylabel("World Y (m)")
        ax.set_title(f"{context['episode_id']} / {context['event_id']}\n{context['status']} | {title}", fontsize=10)
        ax.legend(fontsize=8, loc="best")
        fig.text(.5, .035, "No validated FRESH activation is represented. "+reason[:180], ha="center", fontsize=7, wrap=True)
        fig.tight_layout(rect=(0, .06, 1, .97)); fig.savefig(output, dpi=160); plt.close(fig)
        center = np.asarray(center_pose)[:2]
    record_plot_outputs(directory)
    return {"world_png": str(outputs[0].relative_to(run_root)), "zoom_png": str(outputs[1].relative_to(run_root)), "plot_inputs": str(inputs_path.relative_to(run_root))}


def plot_episode(episode, contexts, run_root):
    episode = Path(episode)
    execution_path = episode/"execution.csv"
    if not execution_path.exists():
        return {"episode_id": episode.name, "reason": "no execution stream"}
    execution = read_csv(execution_path)
    if not execution:
        return {"episode_id": episode.name, "reason": "empty execution stream"}
    poses = pose_rows(execution)
    output = episode/"plots"
    output.mkdir(exist_ok=True)
    overview = output/"executed_trajectory_world.png"
    if not overview.exists():
        fig, ax = plt.subplots(figsize=(9, 7))
        ax.plot(poses[:, 0], poses[:, 1], color="#444444", label="Actual kinematic execution E(t)")
        ax.scatter(poses[0, 0], poses[0, 1], marker="s", color="green", label="Episode start")
        for i, context in enumerate(contexts):
            b = context.get("B")
            if b is not None:
                ax.scatter(b[0], b[1], marker="*", color="#BC9700", s=35)
                ax.annotate(context["event_id"].replace("handoff_", ""), b[:2], xytext=(4, 4), textcoords="offset points", fontsize=7)
        low, high = poses[:, :2].min(axis=0), poses[:, :2].max(axis=0)
        center = (low+high)/2
        half = max(.35, float(np.max(high-low))*.60+.10)
        ax.set_xlim(center[0]-half, center[0]+half); ax.set_ylim(center[1]-half, center[1]+half)
        ax.ticklabel_format(axis="both", useOffset=False)
        ax.set_title(f"{episode.name}\nActual executed path and numbered handoff boundaries")
        ax.set_xlabel("World X (m)"); ax.set_ylabel("World Y (m)")
        ax.set_aspect("equal", adjustable="box"); ax.grid(alpha=.2); ax.legend(fontsize=8)
        fig.tight_layout(); fig.savefig(overview, dpi=160); plt.close(fig)
    timeline = output/"event_timeline.png"
    if not timeline.exists():
        height = max(6, min(16, 4+len(contexts)*.40))
        fig, axes = plt.subplots(1, 2, figsize=(15, height), sharey=True)
        host_origin = float(execution[0]["host_monotonic_s"])
        specifications = [
            (axes[0], "host_monotonic_s", [("t_obs", "RGB capture", "s", "#8E128E"), ("t_request", "Wire request", "o", "#1976D2"),
                ("t_ready_host", "Complete response receipt", "^", "#238B45"), ("t_switch", "First FRESH command", "*", "#E67E22")], host_origin),
            (axes[1], "sim_time_s", [("t_obs", "RGB capture state", "s", "#8E128E"),
                ("t_ready_seen_sim", "First ready-seen state", "^", "#238B45"), ("t_switch", "First FRESH command tick", "*", "#E67E22")], 0.),
        ]
        for ax, clock_key, entries, origin in specifications:
            used_labels = set()
            for i, context in enumerate(contexts):
                values = []
                for key, label, marker, color in entries:
                    value = (context.get(key) or {}).get(clock_key)
                    if value is None:
                        continue
                    value = float(value)-origin
                    ax.scatter(value, i, marker=marker, color=color, s=30, label=label if label not in used_labels else None, zorder=3)
                    used_labels.add(label); values.append(value)
                if values: ax.plot([min(values), max(values)], [i, i], color="#BBBBBB", zorder=0)
            ax.grid(axis="x", alpha=.2)
            if used_labels: ax.legend(fontsize=8, loc="best")
        axes[0].set_yticks(range(len(contexts)), [c["event_id"] for c in contexts], fontsize=7)
        axes[0].set_xlabel("Client host monotonic elapsed from first saved state (s)")
        axes[1].set_xlabel("Actual Isaac simulation time (s)")
        axes[0].set_title("Actual capture / wire request / receipt / application")
        axes[1].set_title("Recorded capture / ready-seen / switch states")
        fig.suptitle(f"{episode.name} — separate host and simulation event timelines\nHost-only request/receipt events are never assigned invented simulation timestamps", fontsize=11)
        fig.tight_layout(rect=(0, 0, 1, .93)); fig.savefig(timeline, dpi=160); plt.close(fig)
    write_json(output/"episode_plot_inputs.json", {"frames": FRAME_CONVENTION, "source_execution": source_record(execution_path, run_root),
        "source_contexts": [source_record(episode/"handoffs"/c["event_id"]/"context.json", run_root) for c in contexts],
        "execution_stream_rows_inclusive": [0, len(execution)-1], "dpi": 160, "geometry_scaling": False})
    return {"episode_id": episode.name, "overview": str(overview.relative_to(run_root)), "timeline": str(timeline.relative_to(run_root))}


def contact_sheets(episode, event_rows, run_root):
    episode = Path(episode)
    sheets = []
    available = [r for r in event_rows if r.get("world_png")]
    for offset in range(0, len(available), 12):
        page = available[offset:offset+12]
        output = episode/"plots"/f"handoff_contact_sheet_{offset//12:03d}.png"
        sheets.append(str(output.relative_to(run_root)))
        if output.exists(): continue
        tile_w, tile_h = 530, 450
        canvas = Image.new("RGB", (tile_w*3, tile_h*((len(page)+2)//3)), "white")
        draw = ImageDraw.Draw(canvas)
        for i, row in enumerate(page):
            with Image.open(Path(run_root)/row["world_png"]) as source:
                thumb = ImageOps.contain(source.convert("RGB"), (tile_w, tile_h-28))
            x, y = (i%3)*tile_w, (i//3)*tile_h
            canvas.paste(thumb, (x+(tile_w-thumb.width)//2, y+25))
            draw.text((x+8, y+6), f"{row['episode_id']} / {row['event_id']}", fill="black")
        canvas.save(output, dpi=(160, 160))
    return sheets


def plot_summary(rows, run_root):
    output = Path(run_root)/"plots"/"summary"/"dataset_distributions.png"
    if output.exists(): return str(output.relative_to(run_root))
    output.parent.mkdir(parents=True, exist_ok=True)
    keys = [("client_rtt_s", "Client RTT (host monotonic s)"), ("observation_to_switch_sim_s", "Observation → switch age (simulation s)"),
            ("inference_translation_m", "Actual translation during pending inference (m)"), ("actual_history_count", "Actual request history frame count"),
            ("e_perp_m", "B to raw FRESH polyline (m)"), ("abs_e_dir_local_deg", "Actual incoming / FRESH local direction (deg)")]
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    for ax, (key, label) in zip(axes.flat, keys):
        values = [r[key] for r in rows if r.get(key) is not None and r["status"] in VALID_STATUSES]
        if values:
            ax.hist(values, bins=min(30, max(1, int(np.sqrt(len(values))))), color="#546E7A", edgecolor="white")
            low, high = min(values), max(values)
            minimum_span = max(1e-6, abs((high+low)/2)*.002)
            if high-low < minimum_span:
                center = (high+low)/2
                ax.set_xlim(center-minimum_span/2, center+minimum_span/2)
            ax.ticklabel_format(axis="x", useOffset=False)
        else: ax.text(.5, .5, "No available valid-event values", transform=ax.transAxes, ha="center")
        ax.set_xlabel(label, fontsize=8); ax.set_ylabel("Dependent successive events"); ax.grid(alpha=.15)
    fig.suptitle("Descriptive online handoff corpus distributions\nEvents within an episode are dependent; episode and raw-pair summaries are separate")
    fig.tight_layout(rect=(0, 0, 1, .93)); fig.savefig(output, dpi=160); plt.close(fig)
    return str(output.relative_to(run_root))


def build_index(run_root, event_rows, episodes, summary_image):
    links = ["<!doctype html><html><head><meta charset='utf-8'><title>Recorded online handoff trajectories</title>",
             "<style>body{font:15px sans-serif;max-width:1400px;margin:32px auto}table{border-collapse:collapse;width:100%}td,th{padding:7px;border:1px solid #ccc}img{max-width:100%}</style></head><body>",
             "<h1>Recorded online robotless handoff trajectory corpus</h1><p>Saved actual kinematic execution and raw predictions. No new inference, path correction, robot dynamics, or navigation outcome claim.</p>",
             f"<p><a href='{html.escape(summary_image)}'>Dataset distributions</a> · <a href='aggregate/statistics.json'>Episode-weighted and duplicate-aware statistics</a></p>"]
    for episode in episodes:
        links.append(f"<h2>{html.escape(episode['episode_id'])}</h2>")
        for key in ("overview", "timeline"):
            if episode.get(key): links.append(f"<a href='{html.escape(episode[key])}'>{key}</a> · ")
        for path in episode.get("contact_sheets", []): links.append(f"<a href='{html.escape(path)}'>contact sheet</a> · ")
        if episode.get("reason"): links.append(f"<p>{html.escape(episode['reason'])}</p>")
        links.append("<table><tr><th>Event</th><th>Status</th><th>World</th><th>Boundary zoom</th><th>Inputs / reason</th></tr>")
        for row in [r for r in event_rows if r["episode_id"] == episode["episode_id"]]:
            def link(key, text):
                return f"<a href='{html.escape(row[key])}'>{text}</a>" if row.get(key) else "—"
            links.append(f"<tr><td>{html.escape(row['event_id'])}</td><td>{html.escape(row['status'])}</td><td>{link('world_png','PNG')}</td><td>{link('zoom_png','PNG')}</td><td>{link('plot_inputs','JSON')} {html.escape(row.get('reason') or '')}</td></tr>")
        links.append("</table>")
    links.append("</body></html>")
    output = Path(run_root)/"index.html"
    text = "\n".join(links)+"\n"
    if output.exists():
        if output.read_text() != text: raise ValueError("refusing to replace changed image index")
    else: output.write_text(text)


def process_run(run_root):
    """Postprocess complete saved episodes. Raw and collected streams stay read-only."""
    run_root = Path(run_root).resolve()
    if not (run_root/"episodes").is_dir():
        raise ValueError("missing online episode source; no fixture fallback")
    metric_rows, plot_rows, episode_rows, episode_metadata = [], [], [], []
    for episode in sorted((run_root/"episodes").iterdir()):
        if not episode.is_dir(): continue
        if (episode/"metadata.json").exists():
            episode_metadata.append(read_json(episode/"metadata.json"))
        contexts, local_plot_rows = [], []
        for context_path in sorted((episode/"handoffs").glob("*/context.json")):
            context = read_json(context_path)
            contexts.append(context)
            row = {"episode_id": context.get("episode_id", episode.name), "event_id": context.get("event_id", context_path.parent.name),
                   "status": context.get("status", "TECHNICAL_INVALID"), "world_png": None, "zoom_png": None, "plot_inputs": None, "reason": None}
            try:
                c, execution, commands, arrays, metrics, old_anchor, sources = load_event(episode, context_path)
                metrics_path = context_path.parent/"metrics.json"
                write_json(metrics_path, metrics)
                sources.append(metrics_path)
                inputs = prepare_plot_inputs(c, execution, arrays, metrics, old_anchor, sources, run_root)
                directory = context_path.parent/"plots"
                inputs_path = directory/"plot_inputs.json"
                write_json(inputs_path, inputs)
                world_path, zoom_path = directory/"trajectory_world.png", directory/"trajectory_boundary_zoom.png"
                plot_event(c, metrics, inputs, world_path)
                plot_event(c, metrics, inputs, zoom_path, zoom=True)
                record_plot_outputs(directory)
                row.update(world_png=str(world_path.relative_to(run_root)), zoom_png=str(zoom_path.relative_to(run_root)), plot_inputs=str(inputs_path.relative_to(run_root)))
                metric_rows.append(metrics)
            except (ValueError, KeyError, FileNotFoundError, IndexError, TypeError) as error:
                row["reason"] = f"{type(error).__name__}: {error}"
                if row["status"] in VALID_STATUSES:
                    raise ValueError(f"valid event has no faithful plot: {row['episode_id']}/{row['event_id']}: {error}") from error
                metric_rows.append({"episode_id": row["episode_id"], "event_id": row["event_id"], "status": row["status"], "diagnostic_unavailable_reason": row["reason"]})
                try:
                    diagnostic = plot_invalid_diagnostic(episode, context_path, run_root, row["reason"])
                    if diagnostic:
                        row.update(diagnostic)
                        row["reason"] = "Diagnostic only: "+row["reason"]
                except (ValueError, KeyError, FileNotFoundError, IndexError, TypeError) as diagnostic_error:
                    row["reason"] += "; diagnostic unavailable: "+str(diagnostic_error)
            local_plot_rows.append(row); plot_rows.append(row)
        episode_row = plot_episode(episode, contexts, run_root)
        episode_row["contact_sheets"] = contact_sheets(episode, local_plot_rows, run_root)
        episode_rows.append(episode_row)
    summary = summarize_events(metric_rows)
    summary.update(episodes_present=len(episode_rows),
        episodes_with_valid_handoffs=len({r["episode_id"] for r in metric_rows if r["status"] in VALID_STATUSES}),
        episode_terminal_status_counts=dict(sorted(Counter(m.get("status", "UNKNOWN") for m in episode_metadata).items())),
        handoff_attempts_reported_in_episode_metadata=sum(int(m.get("handoff_attempts", 0)) for m in episode_metadata),
        initial_model_stop_episode_count=sum(m.get("status") == "MODEL_STOP" and m.get("initial_activation_sim_time_s") is None for m in episode_metadata),
        episode_metadata_available_count=len(episode_metadata))
    if (run_root/"episode_schedule.json").exists():
        summary["episodes_planned"] = len(read_json(run_root/"episode_schedule.json")["episodes"])
    aggregate = run_root/"aggregate"
    write_json(aggregate/"status_counts.json", {k: summary[k] for k in summary if k not in ("statistics", "diversity")})
    write_json(aggregate/"diversity.json", summary["diversity"])
    write_json(aggregate/"statistics.json", summary["statistics"])
    write_csv(aggregate/"handoffs.csv", [{k: row.get(k) for k in ("episode_id", "event_id", "status", "old_chunk_id", "fresh_chunk_id", "old_raw_sha256", "fresh_raw_sha256", "history_full", "history_valid", "timing_valid", "overlap_observed", "post_switch_execution_available", *METRIC_COLUMNS)} for row in metric_rows])
    write_csv(aggregate/"plot_index.csv", plot_rows)
    summary_image = plot_summary(metric_rows, run_root)
    build_index(run_root, plot_rows, episode_rows, summary_image)
    report = {"episode_count": len(episode_rows), "event_count": len(plot_rows), "valid_event_count": summary["valid_count"],
              "expected_valid_event_png_count": 2*summary["valid_count"],
              "actual_valid_event_png_count": sum(2 for r in plot_rows if r["status"] in VALID_STATUSES and r["world_png"] and r["zoom_png"]),
              "actual_all_event_png_count": sum(2 for r in plot_rows if r["world_png"] and r["zoom_png"]),
              "unplottable_event_count": sum(not r["world_png"] for r in plot_rows),
              "index": "index.html", "plot_configuration": PLOT_CONFIG}
    write_json(aggregate/"plot_validation.json", report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_root", type=Path)
    args = parser.parse_args()
    print(json.dumps(process_run(args.run_root), indent=2))


if __name__ == "__main__":
    main()

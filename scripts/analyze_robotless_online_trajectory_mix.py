#!/usr/bin/env python3
"""Read-only descriptive trajectory mix; thresholds do not define task difficulty."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from reconciliation.se2 import wrap_angle

CONFIG = {
    "minimum_path_length_m": 0.05,
    "minimum_segment_length_for_direction_m": 0.0001,
    "minimum_chord_over_arc": 0.995,
    "angle_thresholds_deg": [1, 5, 10, 15],
    "primary_angle_threshold_deg": 5,
    "command_translation_threshold_mps": 0.05,
    "command_turn_threshold_degps": 5,
    "raw_frame": "capture-agent local: x forward, y left, yaw rad CCW",
    "execution_frame": "original fixed world: metres, Z up, yaw rad CCW",
    "raw_convention": "XY uses only saved rows; no prepended XY origin. Yaw travel includes observation yaw=0 to first waypoint.",
    "time_convention": "adjacent saved simulation-state timestamps; only commands with an active chunk; bootstrap excluded",
    "scope": "descriptive thresholds; no collision, navigation failure, or graph-needed classification",
}


def path_shape(path):
    a = np.asarray(path, dtype=float)
    if a.ndim != 2 or a.shape[1] != 3 or len(a) < 1 or not np.isfinite(a).all():
        raise ValueError("finite nonempty N x 3 raw path required")
    delta = np.diff(a[:, :2], axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    length = float(lengths.sum())
    chord = float(np.linalg.norm(a[-1, :2] - a[0, :2]))
    usable = delta[lengths > CONFIG["minimum_segment_length_for_direction_m"]]
    directions = np.arctan2(usable[:, 1], usable[:, 0])
    return {
        "arc_m": length,
        "chord_over_arc": chord / length if length > 0 else None,
        "yaw_travel_from_observation_deg": float(np.degrees(np.abs(wrap_angle(np.diff(np.r_[0., a[:, 2]]))).sum())),
        "max_forward_direction_deviation_deg": float(np.degrees(np.abs(directions)).max()) if len(directions) else None,
        "internal_tangent_turn_deg": float(np.degrees(np.abs(wrap_angle(np.diff(directions)))).sum()) if len(directions) else None,
    }


def raw_straight(shape, angle_deg):
    direction = shape["max_forward_direction_deviation_deg"]
    return bool(shape["arc_m"] >= CONFIG["minimum_path_length_m"]
                and shape["chord_over_arc"] >= CONFIG["minimum_chord_over_arc"]
                and shape["yaw_travel_from_observation_deg"] <= angle_deg
                and direction is not None and direction <= angle_deg)


def executed_straight(post, angle_deg):
    length = post["translation_m"]
    return bool(length >= CONFIG["minimum_path_length_m"]
                and post["net_translation_m"] / length >= CONFIG["minimum_chord_over_arc"]
                and np.degrees(post["yaw_travel_rad"]) <= angle_deg)


def command_class(v_mps, omega_radps):
    translating = abs(v_mps) >= CONFIG["command_translation_threshold_mps"]
    turning = abs(np.degrees(omega_radps)) > CONFIG["command_turn_threshold_degps"]
    if translating:
        return "translating_turn" if turning else "straight_translation"
    return "rotation_low_translation" if turning else "low_motion"


def analyze(run, output):
    run, output = Path(run).resolve(), Path(output).resolve()
    if output == run or run in output.parents:
        raise ValueError("new derived analysis root must be outside original acquisition run")
    if output.exists():
        raise FileExistsError("refusing to replace existing derived analysis")
    sources = {}

    def source(path):
        path = Path(path)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        sources[str(path.relative_to(run))] = digest
        return digest

    def read_json(path):
        source(path)
        return json.loads(path.read_text())

    validation = read_json(run / "validation.json")
    if not validation["valid"] or not validation["schedule_complete"]:
        raise ValueError("validated complete source run required")
    rows = []
    periods = {key: {"sim_s": 0., "distance_m": 0., "ticks": 0} for key in
               ("straight_translation", "translating_turn", "rotation_low_translation", "low_motion")}
    for episode in sorted((run / "episodes").iterdir()):
        if not episode.is_dir():
            continue
        meta = read_json(episode / "metadata.json")
        read_json(episode / "completion.json")
        for mp in sorted((episode / "handoffs").glob("*/metrics.json")):
            m = read_json(mp)
            if not m["status"].startswith("VALID_HANDOFF_"):
                continue
            c = read_json(mp.parent / "context.json")
            ref = c["fresh_raw_local_ref"]
            raw_path = episode / ref["path"]
            if source(raw_path) != ref["sha256"]:
                raise ValueError(f"raw hash changed: {raw_path}")
            rows.append({"episode_id": episode.name, "event_id": m["event_id"],
                         "category": meta["category"], "raw_sha256": ref["sha256"],
                         "raw_shape": path_shape(np.load(raw_path)),
                         "post_switch_execution": m["post_switch_execution"]})
        for name in ("execution.csv", "commands.csv"):
            source(episode / name)
        with (episode / "execution.csv").open() as f:
            states = {int(s["state_id"]): s for s in csv.DictReader(f)}
        with (episode / "commands.csv").open() as f:
            for c in csv.DictReader(f):
                if not c["chunk_id"]:
                    continue
                a = states[int(c["application_state_id"])]
                b = states[int(c["application_state_id"]) + 1]
                dt = float(b["sim_time_s"]) - float(a["sim_time_s"])
                if dt <= 0:
                    raise ValueError("simulation timestamps must increase")
                key = command_class(float(c["v_mps"]), float(c["omega_radps"]))
                periods[key]["sim_s"] += dt
                periods[key]["distance_m"] += float(np.hypot(float(b["x"])-float(a["x"]), float(b["y"])-float(a["y"])))
                periods[key]["ticks"] += 1
    if len(rows) != validation["valid_handoff_count"]:
        raise ValueError("valid handoff coverage mismatch")
    unique = {r["raw_sha256"]: r for r in rows}
    sensitivity = []
    for angle in CONFIG["angle_thresholds_deg"]:
        raw = [r for r in rows if raw_straight(r["raw_shape"], angle)]
        exe = [r for r in rows if executed_straight(r["post_switch_execution"], angle)]
        episode_rates = [np.mean([raw_straight(r["raw_shape"], angle) for r in rows if r["episode_id"] == eid])
                         for eid in sorted({r["episode_id"] for r in rows})]
        sensitivity.append({"angle_deg": angle, "raw_straight_count": len(raw),
            "raw_event_fraction": len(raw)/len(rows), "unique_raw_straight_count": len({r["raw_sha256"] for r in raw}),
            "unique_raw_fraction": len({r["raw_sha256"] for r in raw})/len(unique),
            "equal_episode_raw_fraction": float(np.mean(episode_rates)),
            "executed_straight_count": len(exe), "executed_event_fraction": len(exe)/len(rows),
            "executed_segment_time_fraction": sum(r["post_switch_execution"]["duration_sim_s"] for r in exe)/sum(r["post_switch_execution"]["duration_sim_s"] for r in rows),
            "executed_segment_distance_fraction": sum(r["post_switch_execution"]["translation_m"] for r in exe)/sum(r["post_switch_execution"]["translation_m"] for r in rows)})
    for p in periods.values():
        p["time_fraction"] = p["sim_s"]/sum(q["sim_s"] for q in periods.values())
        p["distance_fraction"] = p["distance_m"]/sum(q["distance_m"] for q in periods.values())
    summary = {"valid_fresh_events": len(rows), "unique_raw_fresh_paths": len(unique),
        "threshold_sensitivity": sensitivity, "active_execution_command_mix": periods,
        "raw_yaw_travel_tails": {str(t): {"events": sum(r["raw_shape"]["yaw_travel_from_observation_deg"] > t for r in rows),
            "unique_raw": sum(r["raw_shape"]["yaw_travel_from_observation_deg"] > t for r in unique.values())} for t in (15,30,60,90)},
        "raw_path_below_minimum_translation_count": sum(r["raw_shape"]["arc_m"] < CONFIG["minimum_path_length_m"] for r in rows)}
    output.mkdir(parents=True)
    provenance = {"run": str(run), "research_git_sha": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "processing_source": str(Path(__file__).relative_to(ROOT)), "processing_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "source_sha256": sources, "config": CONFIG, "no_source_files_modified": True}
    for name, value in (("summary.json", summary), ("events.json", rows), ("provenance.json", provenance)):
        with (output / name).open("x") as f:
            json.dump(value, f, indent=2, allow_nan=False)
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(args.run, args.output), indent=2))

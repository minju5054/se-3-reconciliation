#!/usr/bin/env python3
"""Bounded OFFLINE paired RGB replay through independent official sessions.

This is development/counterfactual input screening, never an online handoff.
No capture, rendering, server launch, optimizer or MPC is performed here.
Run in the pinned external LightNav virtual environment after its server is ready.
"""
from __future__ import annotations

import argparse
import base64
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "scripts/lightnav"))
sys.path.insert(0, str(ROOT / "src"))
from online_lightnav_worker import Session, audited_startup
from reconciliation.online_history import validate_frame
from reconciliation.robotless_single_chunk import save_json_exclusive, sha256_file
from robotless_single_frame_inference import git_state, resolve_path, verify_checkpoint_unchanged

BRANCHES = ("A", "B", "SHAM")
LABEL = "DEVELOPMENT / COUNTERFACTUAL INPUT SCREENING; saved RGB replay, not online motion"


def read_json(path):
    return json.loads(Path(path).read_text())


def declared_condition_ids(protocol):
    rows = protocol.get("development", {}).get("condition_order")
    if not isinstance(rows, list) or not 1 <= len(rows) <= 12:
        raise ValueError("protocol must freeze 1..12 development.condition_order entries")
    ids = [r["condition_id"] if isinstance(r, dict) else r for r in rows]
    if any(not isinstance(x, str) or not x or Path(x).name != x for x in ids):
        raise ValueError("condition IDs must be simple path components")
    if len(set(ids)) != len(ids):
        raise ValueError("duplicate declared condition")
    return ids


def validate_manifest(manifest, protocol, *, now_ns=None):
    """Validate source metadata before any session/model call, without altering clocks."""
    m = deepcopy(manifest)
    if m.get("condition_id") not in declared_condition_ids(protocol):
        raise ValueError("undeclared development condition")
    if not isinstance(m.get("instruction"), str) or not m["instruction"].strip():
        raise ValueError("fixed nonempty instruction required")
    history = m.get("history")
    if not isinstance(history, list) or len(history) + 1 not in (16, 32, 64):
        raise ValueError("H includes final frame and must be 16, 32 or 64")
    finals = m.get("final_frames", {})
    if set(finals) != set(BRANCHES):
        raise ValueError("exact A/B/SHAM final frames required")
    if now_ns is None:
        now_ns = time.monotonic_ns()
    for branch in BRANCHES:
        frames = [validate_frame(f) for f in history + [finals[branch]]]
        if len({f["frame_id"] for f in frames}) != len(frames):
            raise ValueError("duplicated capture frame ID")
        if len({str(f["rendered_state_id"]) for f in frames}) != len(frames):
            raise ValueError("duplicated rendered capture identity")
        if any(f["capture_monotonic_ns"] > now_ns for f in frames):
            raise ValueError("future capture cannot enter replay")
        for left, right in zip(frames, frames[1:]):
            if right["capture_monotonic_ns"] <= left["capture_monotonic_ns"]:
                raise ValueError("history must preserve strict chronological capture order")
            if right["capture_sim_time_s"] < left["capture_sim_time_s"]:
                raise ValueError("history simulation time went backwards")
        paths = []
        for f in frames:
            p = Path(f["path"])
            if not p.is_absolute() or not p.is_file():
                raise ValueError("captured input paths must be existing absolute files")
            if sha256_file(p) != f["sha256"]:
                raise ValueError("captured JPEG source hash mismatch")
            paths.append(p.resolve())
        if len(set(paths)) != len(paths):
            raise ValueError("same source JPEG file cannot fill multiple history slots")
    anchor = finals["A"]
    for branch in BRANCHES[1:]:
        frame = finals[branch]
        if frame["pose_world"] != anchor["pose_world"]:
            raise ValueError("paired terminal agent/camera pose differs")
        if frame["capture_sim_time_s"] != anchor["capture_sim_time_s"]:
            raise ValueError("paired rendering advanced simulation time")
        for key in ("camera_prim_path", "camera_pose_world", "camera_parameters", "resolution_width_height"):
            if frame.get(key) != anchor.get(key):
                raise ValueError(f"paired camera metadata differs: {key}")
    if finals["SHAM"]["sha256"] != anchor["sha256"]:
        raise ValueError("same-input sham must use the exact actual off JPEG bytes")
    return m


def copy_captured_frames(branch_dir, history, final):
    """Copy original JPEG bytes for unchanged Session's episode/rgb confinement."""
    rgb = branch_dir / "rgb"
    rgb.mkdir()
    copies = []
    for idx, source in enumerate(history + [final]):
        frame = deepcopy(source)
        original = Path(frame["path"])
        encoded = original.read_bytes()
        if hashlib.sha256(encoded).hexdigest() != frame["sha256"]:
            raise ValueError("source JPEG changed after preflight")
        target = rgb / f"frame_{idx:06d}.jpg"
        with target.open("xb") as stream:
            stream.write(encoded)
        frame["replay_source_path"] = str(original)
        frame["replay_source_sha256"] = frame["sha256"]
        frame["path"] = str(target.resolve())
        copies.append(frame)
    save_json_exclusive(branch_dir / "replayed_inputs.json", {
        "label": LABEL, "copied_bytes_are_not_new_captures": True,
        "original_capture_timestamps_preserved": True, "frames": copies,
        "repeated_content_hashes": len(copies) - len({f["sha256"] for f in copies}),
    })
    return copies


def wire_parity(branch_dir, frames):
    """Saved-record verification only; never communicates with the model."""
    by_id = {f["frame_id"]: f for f in frames}
    rows = []
    log = branch_dir / "requests/wire.jsonl"
    if not log.exists():
        return {"valid": False, "reason": "no wire log", "rows": []}
    for line in log.read_text().splitlines():
        row = json.loads(line)
        if row["direction"] != "request" or row["action"] != "next":
            continue
        request = read_json(branch_dir / row["raw"]["path"])
        raw = base64.b64decode(request["data"]["image"], validate=True)
        frame = by_id[row["frame_id"]]
        digest = hashlib.sha256(raw).hexdigest()
        rows.append({"frame_id": row["frame_id"], "seq": request["data"]["seq"],
                     "wire_sha256": digest, "source_sha256": frame["sha256"],
                     "byte_identical": raw == Path(frame["replay_source_path"]).read_bytes(),
                     "prediction_request": bool(request["data"]["instruction"])})
    return {"valid": len(rows) == len(frames) and all(r["byte_identical"] for r in rows),
            "rows": rows, "actual_next_requests_sent": len(rows),
            "actual_terminal_predictions_sent": sum(r["prediction_request"] for r in rows)}


def run_branches(output, manifest, config, contract, sampler, *, session_factory=Session):
    """Run precisely one A/B/A' attempt each; errors are preserved without retry."""
    results = {}
    for branch in BRANCHES:
        branch_dir = output / "branches" / branch
        branch_dir.mkdir(parents=True, exist_ok=False)
        session = None
        events = []
        started = time.monotonic()
        result = {"branch": branch, "label": LABEL, "terminal_prediction": None,
                  "status": "TECHNICAL_INVALID", "retry_count": 0}
        frames = []
        try:
            frames = copy_captured_frames(branch_dir, manifest["history"], manifest["final_frames"][branch])
            command = {"episode_id": f"{manifest['condition_id']}_{branch}",
                       "episode_dir": str(branch_dir), "instruction": manifest["instruction"]}
            session = session_factory(config, command, contract, sampler, emit_fn=events.append)
            for frame in frames[:-1]:
                buffered = session.process_frame({"op": "frame", "frame": frame, "predict": False})
                if buffered["status"] != "BUFFERED":
                    raise ValueError(f"history delivery failed: {buffered['status']}")
            terminal = session.process_frame({"op": "frame", "frame": frames[-1],
                                              "predict": True, "chunk_id": "terminal"})
            result.update(status=terminal["status"], terminal_prediction=terminal)
        except Exception as exc:
            result["error"] = f"{type(exc).__name__}: {exc}"
        finally:
            if session is not None:
                try:
                    result["session_close"] = session.close()
                except Exception as exc:
                    result["close_error"] = f"{type(exc).__name__}: {exc}"
            result["wall_time_s"] = time.monotonic() - started
            result["wire_events"] = events
            result["wire_parity"] = wire_parity(branch_dir, frames)
            save_json_exclusive(branch_dir / "paired_result.json", result)
        results[branch] = result
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--condition", type=Path, required=True, help="frozen condition JSON manifest")
    args = parser.parse_args()
    run = args.run.resolve()
    manifest_path = args.condition.resolve()
    protocol_path = run / "protocol.json"
    manifest = validate_manifest(read_json(manifest_path), read_json(protocol_path))
    output = run / "development" / manifest["condition_id"]
    output.mkdir(parents=True, exist_ok=False)
    save_json_exclusive(output / "frozen_input_manifest.json", manifest)
    source = {"condition_path": str(manifest_path), "condition_sha256": sha256_file(manifest_path),
              "protocol_sha256": sha256_file(protocol_path), "wrapper_sha256": sha256_file(Path(__file__))}
    started = time.monotonic()
    try:
        config_path = Path(manifest["config_path"]).resolve()
        config, contract, sampler, ready = audited_startup(config_path)
        save_json_exclusive(output / "official_startup.json", ready)
        results = run_branches(output, manifest, config, contract, sampler)
        verify_checkpoint_unchanged(ready["checkpoint"])
        if git_state(resolve_path(config["paths"]["lightnav_checkout"])) != ready["source"]:
            raise ValueError("official checkout changed during paired screening")
        summary = {"condition_id": manifest["condition_id"], "label": LABEL, "source": source,
                   "H_delivered_frames": len(manifest["history"]) + 1,
                   "branches": results, "checkpoint_stat_unchanged": True, "source_unchanged": True,
                   "wall_time_s": time.monotonic() - started,
                   "actual_terminal_prediction_requests": sum(r["wire_parity"].get("actual_terminal_predictions_sent", 0) for r in results.values()),
                   "actual_buffer_only_requests": sum(r["wire_parity"].get("actual_next_requests_sent", 0) - r["wire_parity"].get("actual_terminal_predictions_sent", 0) for r in results.values()),
                   "new_MPC_solves": 0, "new_captures": 0, "new_GP_solves": 0, "retry_count": 0}
        save_json_exclusive(output / "summary.json", summary)
        print(json.dumps({"output": str(output), "branch_status": {b: r["status"] for b, r in results.items()},
                          "terminal_requests": summary["actual_terminal_prediction_requests"]}))
        return 0 if all(r["status"] in ("PREDICTION_READY", "MODEL_STOP") and r["wire_parity"]["valid"] for r in results.values()) else 2
    except Exception as exc:
        save_json_exclusive(output / "technical_failure.json", {
            "status": "TECHNICAL_INVALID", "error": f"{type(exc).__name__}: {exc}",
            "source": source, "wall_time_s": time.monotonic() - started, "retry_count": 0})
        raise


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Persistent JSONL client to the unmodified official LightNav WebSocket server.

Run in the already verified external checkout's .venv. The Isaac loop owns its
bounded capture queue and sends at most one frame operation at a time; this
isolated synchronous process blocks on wire I/O without blocking Isaac ticks.
One open/close pair is one episode/connection, with one login and one wire reset.
No reconnect, retry, model reload, inference-time sleep, or upstream edits.

CLI: EXTERNAL/.venv/bin/python scripts/online_lightnav_worker.py --config PATH
Commands on stdin (one JSON object per line):
  {"op":"open", "episode_id":"...", "episode_dir":"...", "instruction":"..."}
  {"op":"frame", "frame":CAPTURE_RECORD, "predict":false}
  {"op":"frame", "frame":CAPTURE_RECORD, "predict":true, "chunk_id":"chunk_000"}
  {"op":"close"}
  {"op":"shutdown"}
Stdout is JSONL only: ready/opened/request_sent/result/closed/error/shutdown.
See Session.process_frame for exact results and online_history.validate_frame
for the required capture record. All file references are episode-relative.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import importlib.metadata
import io
import json
from pathlib import Path
import sys
import uuid

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts/lightnav"))

from reconciliation.online_history import SessionHistory, history_contract, validate_frame
from reconciliation.robotless_single_chunk import (
    load_config, observation_to_world, save_json_exclusive, save_npy_exclusive, sha256_file,
    validate_waypoints,
)
from robotless_single_frame_inference import (
    checkpoint_manifest, git_state, host_event, resolve_path,
    verify_checkpoint_unchanged, verify_expected_checkpoint_hashes, write_bytes_exclusive,
)


def emit(value: dict) -> None:
    print(json.dumps(value, ensure_ascii=False, allow_nan=False), flush=True)


def artifact(path: Path, root: Path) -> dict:
    return {"path": str(path.relative_to(root)), "sha256": sha256_file(path), "bytes": path.stat().st_size}


def strict_component(value: str, label: str) -> str:
    if not isinstance(value, str) or not value or Path(value).name != value or value in (".", ".."):
        raise ValueError(f"{label} must be one nonempty path component")
    return value


class ProtocolFailure(ValueError):
    def __init__(self, status: str, message: str):
        self.status = status
        super().__init__(message)


class Session:
    """Serial official protocol, with raw persistence preceding interpretation."""

    def __init__(self, config: dict, command: dict, contract: dict, sampler,
                 *, connect_fn=None, emit_fn=emit, event_fn=host_event):
        self.config = config
        self.episode_id = strict_component(command["episode_id"], "episode_id")
        self.root = Path(command["episode_dir"]).expanduser().resolve()
        if not self.root.is_dir():
            raise FileNotFoundError("parent must create the new episode directory")
        self.history = SessionHistory(command["instruction"], contract, sampler)
        self.emit = emit_fn
        self.event = event_fn
        self.connection_id = str(uuid.uuid4())
        self.timeout_s = float(config.get("online", {}).get("request_timeout_s", 120.0))
        if not np.isfinite(self.timeout_s) or self.timeout_s <= 0:
            raise ValueError("request timeout must be finite and positive")
        self.protocol_dir = self.root / "requests"
        self.protocol_dir.mkdir(exist_ok=True)
        self.protocol = (self.protocol_dir / "wire.jsonl").open("x", encoding="utf-8")
        self.ws = None
        self.closed = False
        self.wire_id = 0
        self.results = []
        if connect_fn is None:
            from websockets.sync.client import connect
            connect_fn = connect
        try:
            self.ws = connect_fn(config["lightnav"]["server_url"], max_size=64 * 1024 * 1024,
                                 open_timeout=self.timeout_s, close_timeout=10)
            for action, data in (("login", {"clientId": self.episode_id}), ("reset", {})):
                raw, _, _ = self.exchange(action, data)
                self.parse_envelope(raw, action)
                getattr(self.history, action)()
            save_json_exclusive(self.root / "session_open.json", {
                "episode_id": self.episode_id, "connection_id": self.connection_id,
                "instruction": self.history.instruction, "open_host": self.event(),
                "wire_login_count": 1, "wire_reset_count": 1,
                "server_initialization": "official ensure_session also initializes its newly created session with reset; this is distinct from the one explicit wire reset",
                "history_contract": contract,
            })
        except BaseException:
            self.history.failed = True
            self.close()
            raise

    def record(self, event: dict) -> None:
        self.protocol.write(json.dumps({"episode_id": self.episode_id,
                                       "connection_id": self.connection_id, **event},
                                      ensure_ascii=False, allow_nan=False) + "\n")
        self.protocol.flush()

    @staticmethod
    def parse_envelope(raw: bytes | str, expected_action: str) -> dict:
        try:
            response = json.loads(raw)
        except Exception as exc:
            raise ProtocolFailure("PROTOCOL_ERROR", "response is not JSON") from exc
        if not isinstance(response, dict) or response.get("action") != expected_action:
            raise ProtocolFailure("PROTOCOL_ERROR", "response action mismatch")
        data = response.get("data")
        if not isinstance(data, dict) or type(data.get("rc")) is not int:
            raise ProtocolFailure("PROTOCOL_ERROR", "response is missing integer rc")
        if data["rc"] != 0:
            raise ProtocolFailure("MODEL_ERROR" if data["rc"] == 500 else "PROTOCOL_ERROR",
                                  f"official response rc={data['rc']}: {data.get('msg')}")
        return data

    def exchange(self, action: str, data: dict, *, sent=None, chunk_id=None,
                 frame_id=None) -> tuple[str | bytes, dict, dict]:
        wire_id = self.wire_id
        self.wire_id += 1
        seq = data.get("seq")
        prefix = f"wire_{wire_id:06d}_{action}"
        request_path = self.protocol_dir / f"{prefix}_request.json"
        response_path = self.protocol_dir / f"{prefix}_response.json"
        wire = json.dumps({"action": action, "data": data}, ensure_ascii=False, allow_nan=False)
        write_bytes_exclusive(request_path, wire.encode("utf-8"))
        # Stamp after serialization and request-file persistence, immediately
        # before socket send. The earlier begin() stamp only rejects future RGB.
        send = self.event()
        self.ws.send(wire)
        self.record({"direction": "request", "action": action, "wire_id": wire_id,
                     "seq": seq, "host": send, "frame_id": frame_id,
                     "raw": artifact(request_path, self.root)})
        if action == "next":
            self.emit({"type": "request_sent", "episode_id": self.episode_id,
                       "seq": seq, "wire_id": wire_id, "chunk_id": chunk_id,
                       "frame_id": frame_id, "kind": "prediction" if chunk_id else "buffer_only",
                       "t_request_host": send, "request": artifact(request_path, self.root)})
        raw = self.ws.recv(timeout=self.timeout_s)
        ready = self.event()  # Full response receipt, before parsing/NPY/metadata I/O.
        write_bytes_exclusive(response_path, raw.encode("utf-8") if isinstance(raw, str) else raw)
        self.record({"direction": "response", "action": action, "wire_id": wire_id,
                     "seq": seq, "host": ready, "frame_id": frame_id,
                     "raw": artifact(response_path, self.root),
                     "wire_representation": "UTF-8 text bytes" if isinstance(raw, str) else "binary bytes"})
        return raw, send, ready

    def process_frame(self, command: dict) -> dict:
        if self.closed or self.history.failed:
            raise ValueError("session is closed or failed")
        frame = validate_frame(command["frame"])
        predict = command["predict"]
        if type(predict) is not bool:
            raise ValueError("predict must be boolean")
        chunk_id = strict_component(command["chunk_id"], "chunk_id") if predict else None
        frame_path = Path(frame["path"])
        frame_path = frame_path.resolve() if frame_path.is_absolute() else (self.root / frame_path).resolve()
        if not frame_path.is_relative_to(self.root / "rgb"):
            raise ValueError("online primary input must be this episode's captured rgb/ file")
        encoded = frame_path.read_bytes()
        if hashlib.sha256(encoded).hexdigest() != frame["sha256"]:
            raise ValueError("captured JPEG hash changed before transmission")
        with Image.open(io.BytesIO(encoded)) as image:
            if image.mode != "RGB" or image.format != "JPEG":
                raise ValueError("send native captured RGB JPEG without client conversion")
            resolution = list(image.size)
            image.verify()
        frame["path"] = str(frame_path.relative_to(self.root))
        frame["resolution_width_height"] = resolution
        chunk_dir = self.root / "chunks" / chunk_id if predict else None
        if predict:
            chunk_dir.mkdir(parents=True, exist_ok=False)
        # Mark time immediately before the next request is constructed/sent; same
        # host clock as all captures. It is never subtracted from server clocks.
        send = self.event()
        pending = self.history.begin(frame, predict=predict, send_monotonic_ns=send["monotonic_ns"],
                                     instruction=command.get("instruction"))
        seq = pending["seq"]
        snapshot_path = (chunk_dir / "history_snapshot.json") if predict else self.protocol_dir / f"seq_{seq:06d}_history.json"
        request_record = self.protocol_dir / f"seq_{seq:06d}_metadata.json"
        result = {"type": "result", "episode_id": self.episode_id,
                  "kind": "prediction" if predict else "buffer_only", "seq": seq,
                  "chunk_id": chunk_id, "frame_id": frame["frame_id"],
                  "observation": frame, "t_request_host": send,
                  "prediction_index": pending["prediction_index"]}
        try:
            raw, send, ready = self.exchange("next", {
                "seq": seq, "image": base64.b64encode(encoded).decode("ascii"),
                "instruction": self.history.instruction if predict else "",
            }, sent=send, chunk_id=chunk_id, frame_id=frame["frame_id"])
            result.update(t_request_host=send, t_ready_host=ready,
                          client_rtt_s=(ready["monotonic_ns"] - send["monotonic_ns"]) / 1e9)
            if predict:
                write_bytes_exclusive(chunk_dir / "response.json", raw.encode("utf-8") if isinstance(raw, str) else raw)
                result["response_ref"] = artifact(chunk_dir / "response.json", self.root)
            data = self.parse_envelope(raw, "next")
            if type(data.get("seq")) is not int or data["seq"] != seq:
                raise ProtocolFailure("PROTOCOL_ERROR", "response seq differs from request")
            if not predict:
                if "actions" in data:
                    raise ProtocolFailure("PROTOCOL_ERROR", "buffer-only unexpectedly predicted")
                snapshot = self.history.complete(seq)
                save_json_exclusive(snapshot_path, snapshot)
                result.update(status="BUFFERED", history_snapshot=artifact(snapshot_path, self.root),
                              history_count=snapshot["actual_history_count"])
            else:
                if not isinstance(data.get("actions"), dict) or "actions" not in data["actions"]:
                    raise ProtocolFailure("PROTOCOL_ERROR", "missing nested raw actions")
                # JSON numeric values are retained as float64 exactly, without
                # recasting to float32, interpolation, row removal, or correction.
                # Save even a nonfinite/wrong-shape numeric array before filtering.
                try:
                    local = np.asarray(data["actions"]["actions"], dtype=np.float64)
                except (TypeError, ValueError) as exc:
                    raise ProtocolFailure("PROTOCOL_ERROR", "raw action array is not rectangular numeric") from exc
                save_npy_exclusive(chunk_dir / "raw_local.npy", local)
                result["raw_local_ref"] = artifact(chunk_dir / "raw_local.npy", self.root)
                if not np.all(np.isfinite(local)):
                    raise ProtocolFailure("NONFINITE_OUTPUT", "nonfinite raw output preserved")
                try:
                    validate_waypoints(local)
                except ValueError as exc:
                    raise ProtocolFailure("PROTOCOL_ERROR", str(exc)) from exc
                if type(data.get("stop")) is not bool:
                    raise ProtocolFailure("PROTOCOL_ERROR", "stop must be boolean")
                pointing = data.get("pointing")
                if pointing is not None and (not isinstance(pointing, dict) or pointing.get("frame_size") != resolution):
                    raise ProtocolFailure("PROTOCOL_ERROR", "pointing image resolution mismatch")
                snapshot = self.history.complete(seq, server_actions_step=data["actions"].get("step"))
                save_json_exclusive(snapshot_path, snapshot)
                world = observation_to_world(frame["pose_world"], local)
                save_npy_exclusive(chunk_dir / "world.npy", world)
                result.update(status="MODEL_STOP" if data["stop"] else "PREDICTION_READY",
                              raw_local=local.tolist(), world=world.tolist(),
                              world_ref=artifact(chunk_dir / "world.npy", self.root),
                              history_snapshot=artifact(snapshot_path, self.root),
                              history_count=snapshot["actual_history_count"],
                              history_full=snapshot["history_full"], stop=data["stop"],
                              response=data,
                              transform={"source_frame": "logical_agent_at_triggering_capture",
                                         "target_frame": "Isaac_world_z_up", "anchor_pose_world": frame["pose_world"],
                                         "equation": "T_world_waypoint = T_world_capture @ T_capture_waypoint",
                                         "columns_local": ["forward_m", "left_m", "yaw_ccw_rad"],
                                         "columns_world": ["x_m", "y_m", "yaw_ccw_rad"],
                                         "yaw_wrap": "[-pi, pi)", "rows": "cumulative spatial poses",
                                         "waypoint_time_base": None, "raw_correction_applied": False})
        except Exception as exc:
            self.history.failed = True
            result.update(status=getattr(exc, "status", "PROTOCOL_ERROR"),
                          error=f"{type(exc).__name__}: {exc}")
            if not snapshot_path.exists() and self.history.pending is not None:
                snapshot = self.history.snapshot()
                snapshot["delivery_or_prediction_failed"] = True
                save_json_exclusive(snapshot_path, snapshot)
                result["history_snapshot"] = artifact(snapshot_path, self.root)
        save_json_exclusive(request_record, result)
        result["request_metadata"] = artifact(request_record, self.root)
        if predict:
            save_json_exclusive(chunk_dir / "metadata.json", result)
        self.results.append({k: result[k] for k in ("seq", "kind", "status", "chunk_id", "frame_id")})
        return result

    def close(self) -> dict:
        if self.closed:
            return {"type": "closed", "episode_id": self.episode_id, "already_closed": True}
        self.closed = True
        if self.ws is not None:
            self.ws.close()
        self.protocol.close()
        result = {"type": "closed", "episode_id": self.episode_id,
                  "connection_id": self.connection_id, "connection_count": 1,
                  "wire_login_count": self.history.login_count, "wire_reset_count": self.history.reset_count,
                  "next_count": self.history.next_seq, "prediction_count": self.history.prediction_count,
                  "reconnect_count": 0, "retry_count": 0, "max_outstanding": 1,
                  "instruction_constant": True, "session_failed": self.history.failed,
                  "close_host": self.event(), "requests": self.results}
        save_json_exclusive(self.root / "session_close.json", result)
        return result


def audited_startup(config_path: Path) -> tuple[dict, dict, object, dict]:
    config = load_config(config_path)
    checkout = resolve_path(config["paths"]["lightnav_checkout"])
    checkpoint = resolve_path(config["paths"]["checkpoint_path"])
    if Path(sys.prefix).resolve() != (checkout / ".venv").resolve():
        raise ValueError("worker must run in the existing verified external LightNav .venv")
    source = git_state(checkout)
    if not source["clean"] or source["git_sha"] != config["lightnav"]["expected_git_sha"]:
        raise ValueError("external LightNav source must be clean and at the configured pinned SHA")
    manifest = checkpoint_manifest(checkpoint)
    verify_expected_checkpoint_hashes(manifest, config["lightnav"]["checkpoint_sha256"])
    contract, sampler = history_contract(checkout, checkpoint)
    ready = {"type": "ready", "source": source, "checkpoint": manifest,
             "history_contract": contract, "worker_python": sys.executable,
             "config_sha256": sha256_file(config_path),
             "package_versions": {name: importlib.metadata.version(name) for name in ("numpy", "Pillow", "PyYAML", "websockets")}}
    return config, contract, sampler, ready


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    session = None
    try:
        config, contract, sampler, ready = audited_startup(args.config.resolve())
        emit(ready)
        seen_episodes = set()
        for line in sys.stdin:
            command = None
            try:
                if len(line) > 1_048_576:
                    raise ValueError("JSONL command exceeds bounded input size")
                command = json.loads(line)
                op = command["op"]
                if op == "open":
                    if session is not None:
                        raise ValueError("previous episode must close before another opens")
                    if command["episode_id"] in seen_episodes:
                        raise ValueError("same episode may not open a second session")
                    seen_episodes.add(command["episode_id"])
                    session = Session(config, command, contract, sampler)
                    emit({"type": "opened", "episode_id": session.episode_id,
                          "connection_id": session.connection_id, "history_contract": contract})
                elif op == "frame":
                    if session is None:
                        raise ValueError("open an episode before sending a captured frame")
                    emit(session.process_frame(command))
                elif op == "close":
                    if session is None:
                        raise ValueError("no episode is open")
                    emit(session.close())
                    session = None
                elif op == "shutdown":
                    if session is not None:
                        emit(session.close())
                        session = None
                    break
                else:
                    raise ValueError(f"unsupported JSONL operation {op!r}")
            except Exception as exc:
                if session is not None and isinstance(command, dict) and command.get("op") == "frame":
                    session.history.failed = True
                emit({"type": "error", "op": command.get("op") if isinstance(command, dict) else None,
                      "error": f"{type(exc).__name__}: {exc}"})
        if session is not None:
            emit(session.close())
        verify_checkpoint_unchanged(ready["checkpoint"])
        if git_state(resolve_path(config["paths"]["lightnav_checkout"])) != ready["source"]:
            raise ValueError("external LightNav source changed during worker session")
        emit({"type": "shutdown", "checkpoint_stat_unchanged": True, "source_unchanged": True})
        return 0
    except Exception as exc:
        emit({"type": "error", "status": "TECHNICAL_INVALID", "error": f"{type(exc).__name__}: {exc}"})
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""CPU-only JSONL subprocess for the pinned external official LightNav MPC.

Use the existing external ``mujoco_demo/.venv/bin/python``. This imports mpc.py
only; it does not launch MuJoCo. Solver output is sent to stderr so stdout stays
strict JSONL. Solve results are emitted asynchronously, without a poll request.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import select
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from reconciliation.online_mpc_adapter import (  # noqa: E402
    PROTOCOL, OfficialMpcAdapter, audited_tracker, load_official,
)


def dispatch(adapter, request):
    if not isinstance(request, dict):
        raise ValueError("request must be an object")
    payload = dict(request)
    operation = payload.pop("op")
    payload.pop("request_id")
    if operation == "reset":
        return adapter.reset(**payload)
    if operation == "install":
        return adapter.install(**payload)
    if operation == "submit":
        return adapter.submit(**payload)
    if operation == "poll":
        if payload:
            raise ValueError("poll has no additional arguments")
        return {"status": "polled", "result": adapter.poll()}
    if operation == "close":
        if payload:
            raise ValueError("close has no additional arguments")
        adapter.close()
        return {"status": "closed"}
    raise ValueError(f"unknown operation: {operation}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lightnav-checkout", required=True)
    parser.add_argument("--max-request-bytes", type=int, default=1048576)
    args = parser.parse_args()
    if args.max_request_bytes <= 0:
        parser.error("--max-request-bytes must be positive")
    # Preserve the protocol pipe before moving all native stdout (including
    # CasADi/IPOPT's one-time banner) to stderr. Never redirect external code.
    protocol_out = os.fdopen(os.dup(sys.stdout.fileno()), "w", buffering=1)
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())

    def emit(record):
        protocol_out.write(json.dumps(record, allow_nan=False, separators=(",", ":")) + "\n")
        protocol_out.flush()

    adapter = None
    try:
        module, provenance = load_official(args.lightnav_checkout)
        adapter = OfficialMpcAdapter(module, audited_tracker(module))
        emit({"type": "ready", "protocol": PROTOCOL, "provenance": provenance,
              "worker_ready_host_monotonic_s": time.monotonic()})
        buffer = b""
        request_ids = set()
        while not adapter.closed:
            result = adapter.poll()
            if result is not None:
                emit(result)
            readable, _, _ = select.select([sys.stdin.fileno()], [], [], 0.002)
            if not readable:
                continue
            piece = os.read(sys.stdin.fileno(), 65536)
            if not piece:
                if buffer:
                    raise ValueError("incomplete final JSONL request")
                break
            buffer += piece
            while b"\n" in buffer and not adapter.closed:
                line, buffer = buffer.split(b"\n", 1)
                if len(line) > args.max_request_bytes:
                    raise ValueError("request exceeds bounded line capacity")
                request = None
                try:
                    request = json.loads(line)
                    if not isinstance(request, dict):
                        raise ValueError("request must be an object")
                    request_id = request.get("request_id")
                    if not isinstance(request_id, (str, int)) or isinstance(request_id, bool):
                        raise ValueError("request_id must be a unique string or integer")
                    if request_id in request_ids:
                        raise ValueError("duplicate request_id")
                    request_ids.add(request_id)
                    reply = dispatch(adapter, request)
                    emit({"type": "reply", "request_id": request_id,
                          "op": request.get("op"), **reply})
                except Exception as exc:
                    emit({"type": "reply", "request_id": request.get("request_id") if isinstance(request, dict) else None,
                          "op": request.get("op") if isinstance(request, dict) else None,
                          "status": "error", "error": f"{type(exc).__name__}: {exc}"})
            if len(buffer) > args.max_request_bytes:
                raise ValueError("request exceeds bounded line capacity")
    except Exception as exc:
        emit({"type": "fatal", "status": "controller_error",
              "error": f"{type(exc).__name__}: {exc}"})
        return 1
    finally:
        if adapter is not None:
            adapter.close()
        protocol_out.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

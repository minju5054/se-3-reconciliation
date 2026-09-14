#!/usr/bin/env python3
"""Launch the pinned external server and record provenance for one successive run.

This research-side helper imports no LightNav, Torch or Isaac package. It makes
no prediction requests; the separate client owns the single two-frame session.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from urllib.parse import urlparse

from robotless_single_frame_inference import (
    checkpoint_manifest, git_state, host_event, load_config, read_json,
    resolve_path, save_json_exclusive, sha256_file, verify_expected_checkpoint_hashes,
)


def server_arguments(config: dict, run: Path) -> list[str]:
    """Bind only the configured local endpoint, in the external environment."""
    address = urlparse(config["lightnav"]["server_url"])
    if (address.scheme != "ws" or address.hostname != "127.0.0.1"
            or address.username or address.password or address.path not in ("", "/")
            or address.query or address.fragment or address.port is None):
        raise ValueError("local server launcher requires ws://127.0.0.1:PORT")
    checkout = resolve_path(config["paths"]["lightnav_checkout"])
    return [str(checkout / ".venv/bin/lightnav-serve"), "--task", "vln",
            "--model_path", str(resolve_path(config["paths"]["checkpoint_path"])),
            "--backend", "vllm_local", "--host", "127.0.0.1", "--port", str(address.port),
            "--ready_file", str(run / "server.ready")]


def process_identity(pid: int) -> dict | None:
    """Linux process start tick prevents sending a signal to a reused PID."""
    try:
        proc = Path("/proc") / str(pid)
        fields = (proc / "stat").read_text().rsplit(")", 1)[1].split()
        return {"start_ticks": int(fields[19]), "state": fields[0],
                "cmdline": (proc / "cmdline").read_bytes().split(b"\0")}
    except FileNotFoundError:
        return None


def start(run: Path, timeout_s: float) -> None:
    config = load_config(run / "config_snapshot.yaml")
    argv = server_arguments(config, run)
    for name in ("server_launch.json", "server_process.json", "server.ready", "logs/server.log"):
        if (run / name).exists():
            raise FileExistsError(run / name)
    checkout = resolve_path(config["paths"]["lightnav_checkout"])
    checkpoint = resolve_path(config["paths"]["checkpoint_path"])
    source = git_state(checkout)
    if not source["clean"] or source["git_sha"] != config["lightnav"]["expected_git_sha"]:
        raise ValueError("external source must be clean and at the pinned Git SHA")
    manifest = checkpoint_manifest(checkpoint)
    verify_expected_checkpoint_hashes(manifest, config["lightnav"]["checkpoint_sha256"])
    env = os.environ.copy()
    for key in ("PYTHONPATH", "LD_LIBRARY_PATH"):
        env.pop(key, None)
    env.update(CUDA_VISIBLE_DEVICES="0", PYTHONUNBUFFERED="1")
    (run / "logs").mkdir(exist_ok=True)
    started = host_event()
    with (run / "logs/server.log").open("xb") as log:
        process = subprocess.Popen(argv, cwd=checkout, env=env, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
    try:
        identity = process_identity(process.pid)
        if identity is None:
            raise RuntimeError("server exited before its process identity was recorded")
        state = {
            "process_id": process.pid, "process_start_ticks": identity["start_ticks"],
            "argv": argv, "command_cwd": str(checkout), "start_utc": started["utc"],
            "start_event": started, "stdout_log": "logs/server.log",
            "lightnav_git_sha": source["git_sha"], "lightnav_checkout": str(checkout),
            "checkpoint_identifier": config["lightnav"]["checkpoint_identifier"],
            "checkpoint_path": str(checkpoint), "source_status_clean": True,
            "checkpoint_files": [{**entry, "size": entry["bytes"]} for entry in manifest["files"]],
            "config": {"path": "config_snapshot.yaml", "sha256": sha256_file(run / "config_snapshot.yaml")},
            "launcher_source_sha256": sha256_file(Path(__file__)),
            "environment_overrides": {"unset": ["PYTHONPATH", "LD_LIBRARY_PATH"], "CUDA_VISIBLE_DEVICES": "0"},
        }
        save_json_exclusive(run / "server_launch.json", state)
        deadline = time.monotonic() + timeout_s
        while True:
            if process.poll() is not None:
                raise RuntimeError("server exited before readiness; inspect logs/server.log")
            if ((run / "server.ready").is_file()
                    and "[lightnav-ws] READY" in (run / "logs/server.log").read_text()):
                break
            if time.monotonic() > deadline:
                raise TimeoutError("server startup timeout; inspect logs/server.log")
            time.sleep(0.25)
        ready = host_event()
        state.update(ready_confirmed=True, ready_observed_utc=ready["utc"], ready_event=ready)
        save_json_exclusive(run / "server_process.json", state)
        print(f"OFFICIAL_LIGHTNAV_READY pid={process.pid}", flush=True)
    except BaseException:
        if process.poll() is None:
            process.terminate()
        raise


def stop(run: Path, timeout_s: float) -> None:
    record = read_json(run / "server_process.json")
    if (run / "server_shutdown.json").exists():
        raise FileExistsError(run / "server_shutdown.json")
    pid = record["process_id"]
    identity = process_identity(pid)
    requested = host_event()
    sent = False
    if identity is not None and identity["state"] != "Z":
        if (identity["start_ticks"] != record["process_start_ticks"]
                or os.fsencode(record["argv"][0]) not in identity["cmdline"]):
            raise ValueError("process identity changed; refusing to signal a different process")
        os.kill(pid, signal.SIGTERM)
        sent = True
        deadline = time.monotonic() + timeout_s
        while True:
            current = process_identity(pid)
            if current is None or current["state"] == "Z":
                break
            if current["start_ticks"] != identity["start_ticks"]:
                break
            if time.monotonic() > deadline:
                raise TimeoutError("server has not exited after SIGTERM")
            time.sleep(0.25)
    save_json_exclusive(run / "server_shutdown.json", {
        "process_id": pid, "signal_sent": "SIGTERM" if sent else None,
        "request_event": requested, "exit_observed_event": host_event(),
        "reason": "release GPU after the two-request session before Isaac visualization",
    })
    print(f"OFFICIAL_LIGHTNAV_STOPPED pid={pid}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("start", "stop"))
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--timeout-s", type=float, default=180)
    args = parser.parse_args()
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0:
        parser.error("timeout must be finite and positive")
    (start if args.mode == "start" else stop)(args.run_directory.resolve(), args.timeout_s)


if __name__ == "__main__":
    main()

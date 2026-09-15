"""Read-only official MPC reuse with auditable asynchronous solve provenance.

The external tracker alone selects references and computes commands. The index
calculation here is an independent diagnostic checked against that selection.
No robot backend or MuJoCo module is imported. Tests inject a mock tracker.
"""
from __future__ import annotations

import hashlib
import importlib.util
import math
from pathlib import Path
import subprocess
import sys
import time
from types import ModuleType
from typing import Any, Callable

import numpy as np

from reconciliation.se2 import local_trajectory_to_world, wrap_angle


PINNED_LIGHTNAV_SHA = "c6f40e3220edbf7011e4f17eaf2c865416737d4d"
PINNED_MPC_SHA256 = "2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1"
PROTOCOL = "official_mpc_jsonl_v1"
SETTING_NAMES = (
    "CONTROL_RATE_HZ", "HORIZON", "MPC_DT_S", "WAYPOINT_DT_S", "TRACK_V_MAX",
    "OBJNAV_V_MAX", "W_MAX", "A_MAX_V", "A_MAX_W", "Q_WEIGHTS", "R_WEIGHTS",
    "ODOM_MATCH_MAX_GAP_S", "ODOM_TIMEOUT_S",
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def finite_pose(value: Any, name: str) -> np.ndarray:
    pose = np.asarray(value, dtype=np.float64)
    if pose.shape != (3,) or not np.isfinite(pose).all():
        raise ValueError(f"{name} must be a finite [x,y,yaw] pose")
    return pose.copy()


def finite_number(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def selection_audit(world: Any, pose: Any, official_reference: Any, *,
                    horizon: int, weights: Any) -> dict[str, Any]:
    """Report original row indices, including endpoint repetition and yaw unwrap.

    This never feeds the controller. It checks the official selected poses modulo
    yaw periodicity, and preserves the exact unwrapped reference for the record.
    """
    path = np.asarray(world, dtype=np.float64)
    state = finite_pose(pose, "pose")
    ref = np.asarray(official_reference, dtype=np.float64)
    w = np.asarray(weights, dtype=np.float64)
    if path.ndim != 2 or path.shape[1:] != (3,) or not len(path) or not np.isfinite(path).all():
        raise ValueError("world trajectory must be nonempty finite Nx3")
    if horizon <= 0 or w.shape != (3,) or not np.isfinite(w).all() or np.any(w <= 0):
        raise ValueError("invalid official selection configuration")
    residual = path - state
    residual[:, 2] = np.arctan2(np.sin(residual[:, 2]), np.cos(residual[:, 2]))
    nearest = int(np.argmin(np.sum(residual * residual * w, axis=1)))
    indices = [min(nearest + offset, len(path) - 1) for offset in range(1, horizon + 1)]
    if ref.shape != (horizon, 3) or not np.isfinite(ref).all():
        raise ValueError("official selected reference has invalid shape or values")
    if not np.allclose(path[indices, :2], ref[:, :2], atol=1e-12, rtol=0):
        raise ValueError("official selected XY disagrees with reference index audit")
    if not np.allclose(wrap_angle(path[indices, 2] - ref[:, 2]), 0, atol=1e-12, rtol=0):
        raise ValueError("official selected yaw disagrees with reference index audit")
    return {
        "nearest_index": nearest,
        "indices": indices,
        "reference_world": ref.tolist(),
        "endpoint_repeated": len(set(indices)) < len(indices),
        "nearest_is_final_row": nearest == len(path) - 1,
        "selection_semantics": "nearest weighted XY/wrapped-yaw pose; next rows; endpoint repetition; sequential yaw unwrap",
        "index_validation": "independent diagnostic checked against actual official reference",
    }


def load_official(checkout: str | Path) -> tuple[ModuleType, dict[str, Any]]:
    """Pin and import only mpc.py; refuse dirty or different external source."""
    root = Path(checkout).resolve()
    source = root / "mujoco_demo/vln_mujoco/mpc.py"
    current_sha = subprocess.check_output(
        ["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
    status = subprocess.check_output(
        ["git", "-C", str(root), "status", "--porcelain"], text=True).strip()
    if current_sha != PINNED_LIGHTNAV_SHA or status:
        raise ValueError("official LightNav checkout must be clean and at the pinned revision")
    if sha256(source) != PINNED_MPC_SHA256:
        raise ValueError("official MPC source hash differs from pinned source")
    name = "_online_official_lightnav_mpc"
    spec = importlib.util.spec_from_file_location(name, source)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load official MPC from {source}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    import casadi
    provenance = {
        "protocol": PROTOCOL,
        "lightnav_checkout": str(root), "lightnav_sha": current_sha,
        "external_git_status": status, "mpc_source": str(source),
        "mpc_source_sha256": sha256(source), "python_executable": sys.executable,
        "python_version": sys.version, "casadi_version": casadi.__version__,
        "numpy_version": np.__version__,
        "official_settings": {key: getattr(module, key) for key in SETTING_NAMES},
        "effective_linear_velocity_limit_m_s": module.OBJNAV_V_MAX,
        "intrinsic_model_waypoint_dt_s": None,
        "controller_reference_timing_convention": "pose-aligned next-row horizon at MPC_DT_S; WAYPOINT_DT_S is a declared upstream constant, not model waypoint timing",
        "world_frame": "metres, +Z up; yaw radians CCW",
        "local_frame": "+x forward, +y left, +z up; yaw radians CCW",
        "projection": "T_W_waypoint = T_W_agent_at_capture * T_agent_raw_cumulative_waypoint",
        "external_source_modified": False,
        "tracker_instrumentation": "subclass timestamps super()._solve; no objective, gain, limit, selection, or solve changes",
        "robot_backend_imported": False,
    }
    return module, provenance


def audited_tracker(module: ModuleType, *, monotonic: Callable[[], float] = time.monotonic):
    """Instrument the actual official worker invocation, without reimplementing it."""
    class AuditedTracker(module.MpcTracker):
        def __init__(self):
            self.solve_timing: dict[int, dict[str, float]] = {}
            super().__init__()

        def _solve(self, generation, pose, reference, previous):
            timing = {"solve_start_host_monotonic_s": monotonic()}
            self.solve_timing[generation] = timing
            try:
                return super()._solve(generation, pose, reference, previous)
            finally:
                timing["solve_end_host_monotonic_s"] = monotonic()

    return AuditedTracker()


class OfficialMpcAdapter:
    """Single-owner command adapter; call from one control worker event loop.

    Installs never activate commands. ``poll`` returns only the completed solve's
    identity. The Isaac executor must log first application separately and reject
    results that became stale while queued outside this worker.
    """
    def __init__(self, module: ModuleType, tracker: Any, *,
                 monotonic: Callable[[], float] = time.monotonic):
        self.module, self.tracker, self.monotonic = module, tracker, monotonic
        self.episode_id: str | None = None
        self.installed: dict[str, Any] | None = None
        self.pending: dict[str, Any] | None = None
        self.world: np.ndarray | None = None
        self.used_solve_ids: set[str] = set()
        self.used_chunk_ids: set[str] = set()
        self.closed = False

    def reset(self, episode_id: str) -> dict[str, Any]:
        self._open()
        if not isinstance(episode_id, str) or not episode_id:
            raise ValueError("episode_id must be nonempty")
        self.tracker.reset()
        self.episode_id = episode_id
        self.installed, self.world = None, None
        self.used_solve_ids.clear()
        self.used_chunk_ids.clear()
        return {"status": "reset", "episode_id": episode_id,
                "official_generation": self.tracker._generation}

    def install(self, *, episode_id: str, chunk_id: str, reference_version: int,
                raw_local_path: str | Path, capture_pose: Any,
                raw_sha256: str | None = None, **context: Any) -> dict[str, Any]:
        self._open()
        if episode_id != self.episode_id or not episode_id:
            raise ValueError("reset the matching episode before installation")
        if not isinstance(reference_version, int) or isinstance(reference_version, bool) or reference_version < 0:
            raise ValueError("reference_version must be a nonnegative integer")
        if self.installed and reference_version <= self.installed["reference_version"]:
            raise ValueError("reference_version must increase")
        if not isinstance(chunk_id, str) or not chunk_id or chunk_id in self.used_chunk_ids:
            raise ValueError("chunk_id must be unique within episode")
        path = Path(raw_local_path).resolve()
        digest = sha256(path)
        if raw_sha256 is not None and digest != raw_sha256:
            raise ValueError("raw local source hash mismatch")
        raw = np.load(path, allow_pickle=False)
        if raw.ndim != 2 or raw.shape[1:] != (3,) or not len(raw) or not np.isfinite(raw).all():
            raise ValueError("raw local trajectory must be nonempty finite Nx3")
        anchor = finite_pose(capture_pose, "capture_pose")
        expected = local_trajectory_to_world(anchor, raw)
        self.tracker.set_body_path(raw, anchor)
        world = np.asarray(self.tracker._trajectory, dtype=np.float64).copy()
        if world.shape != raw.shape or not np.allclose(world[:, :2], expected[:, :2], atol=1e-12, rtol=0):
            raise ValueError("official capture-time world projection mismatch")
        if not np.allclose(wrap_angle(world[:, 2] - expected[:, 2]), 0, atol=1e-12, rtol=0):
            raise ValueError("official capture-time yaw projection mismatch")
        if sha256(path) != digest:
            raise ValueError("raw local source changed during installation")
        self.world = world
        self.world.flags.writeable = False
        self.installed = {
            "episode_id": episode_id, "chunk_id": chunk_id,
            "reference_version": reference_version,
            "official_generation": self.tracker._generation,
            "raw_local_path": str(path), "raw_sha256": digest,
            "capture_pose": anchor.tolist(), "row_count": len(raw),
            "install_context": context,
            "installed_host_monotonic_s": self.monotonic(),
        }
        self.used_chunk_ids.add(chunk_id)
        return {"status": "installed", **self.installed, "command_activated": False}

    def submit(self, *, solve_id: str, pose: Any, input_state_id: Any,
               input_sim_time_s: float, input_host_monotonic_s: float) -> dict[str, Any]:
        self._open()
        if self.installed is None:
            raise ValueError("no reference installed")
        if not isinstance(solve_id, str) or not solve_id or solve_id in self.used_solve_ids:
            raise ValueError("solve_id must be unique and nonempty within episode")
        if self.pending is not None or self.tracker._future is not None:
            return {"status": "busy", "solve_id": solve_id}
        state = finite_pose(pose, "input pose")
        sim = finite_number(input_sim_time_s, "input_sim_time_s")
        host = finite_number(input_host_monotonic_s, "input_host_monotonic_s")
        if sim < 0:
            raise ValueError("input_sim_time_s must be nonnegative")
        record = {
            "episode_id": self.installed["episode_id"],
            "chunk_id": self.installed["chunk_id"],
            "reference_version": self.installed["reference_version"],
            "official_generation": self.installed["official_generation"],
            "solve_id": solve_id, "input_state_id": input_state_id,
            "input_sim_time_s": sim, "input_host_monotonic_s": host,
            "input_pose": state.tolist(),
            "previous_command": list(self.tracker.previous_command),
            "submit_host_monotonic_s": self.monotonic(),
        }
        self.tracker.submit(state)
        if self.tracker._future is None:
            raise RuntimeError(self.tracker.error or "official tracker did not submit solve")
        record["_world"] = self.world.copy()
        self.pending = record
        self.used_solve_ids.add(solve_id)
        return {"status": "submitted", **{k: v for k, v in record.items() if not k.startswith("_")}}

    def poll(self) -> dict[str, Any] | None:
        self._open()
        future = self.tracker._future
        if future is None or not future.done():
            return None
        if self.pending is None:
            raise RuntimeError("official future has no solve provenance")
        pending, self.pending = self.pending, None
        world = pending.pop("_world")
        generation = pending["official_generation"]
        record = {"type": "solve_result", **pending,
                  "result_seen_worker_host_monotonic_s": self.monotonic()}
        failure = None
        try:
            solved = future.result()
            command = np.asarray(solved.command, dtype=np.float64)
            if command.shape != (2,) or not np.isfinite(command).all():
                raise ValueError("official command is not finite [v,omega]")
            if not np.isfinite(solved.prediction).all():
                raise ValueError("official predicted states are nonfinite")
            record.update({"command": list(solved.command),
                           "official_solve_ms": float(solved.solve_ms),
                           "prediction_world": solved.prediction.tolist(),
                           "result_generation": solved.generation})
            record["selection"] = selection_audit(
                world, pending["input_pose"], solved.reference,
                horizon=self.module.HORIZON, weights=self.module.Q_WEIGHTS)
            if solved.generation != generation:
                raise ValueError("official result generation disagrees with submitted solve")
        except Exception as exc:
            failure = f"{type(exc).__name__}: {exc}"
        official_command = self.tracker.poll()
        record.update(getattr(self.tracker, "solve_timing", {}).pop(generation, {}))
        stale = generation != self.tracker._generation
        if stale:
            record.update(status="stale_rejected", command_available_for_application=False)
        elif failure or self.tracker.error:
            record.update(status="controller_error", command_available_for_application=False)
        elif official_command is None:
            record.update(status="controller_error", command_available_for_application=False)
            failure = "official tracker did not accept current-generation completed solve"
        else:
            if not np.array_equal(np.asarray(official_command), np.asarray(record["command"])):
                raise RuntimeError("official polled command differs from audited result")
            record.update(status="command", command_available_for_application=True)
        if failure or self.tracker.error:
            record["error"] = failure or self.tracker.error
        return record

    def close(self) -> None:
        if not self.closed:
            self.tracker.close()
            self.closed = True

    def _open(self) -> None:
        if self.closed:
            raise ValueError("MPC adapter is closed")

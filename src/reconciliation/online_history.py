"""Causal camera/delivery bookkeeping and pinned-source sampling reconstruction.

No camera pixels are fabricated here. Model segment indices are reconstructed
through the external official sampler, never represented as server telemetry.
"""

from __future__ import annotations

from collections import deque
from copy import deepcopy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from typing import Any, Callable


HISTORY_SOURCE_FILES = (
    "docs/PROTOCOL.md", "src/lightnav/inference/policies.py",
    "src/lightnav/serving/tracking_service.py", "src/lightnav/serving/ws_server.py",
    "src/lightnav/inference/config.py", "src/lightnav/inference/model.py",
    "src/lightnav/inference/samples.py", "src/lightnav/inference/frame_preprocessing.py",
    "src/lightnav/slowfast.py", "src/lightnav/data_processor.py",
)


def history_contract(checkout: Path, checkpoint: Path) -> tuple[dict, Callable]:
    """Resolve the unoverridden official VLN checkpoint's history contract.

    Caller audits checkout SHA and server argv separately. Loading only the
    standalone official sampler does not import Torch, a model, or a simulator.
    """
    checkout, checkpoint = Path(checkout), Path(checkpoint)
    config_path = checkpoint / "eval_config.json"
    config = json.loads(config_path.read_text())
    common, task = config["common"], config["tasks"]["vlnce"]
    hashes = {name: hashlib.sha256((checkout / name).read_bytes()).hexdigest()
              for name in HISTORY_SOURCE_FILES}
    sampler_path = checkout / "src/lightnav/slowfast.py"
    spec = importlib.util.spec_from_file_location("_online_readonly_official_slowfast", sampler_path)
    if spec is None or spec.loader is None:
        raise ValueError("official SlowFast sampler cannot be loaded read-only")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    tiers = module.validate_slowfast_tiers(task["slowfast_tiers"]) if task.get("slowfast_tiers") else []
    contract = {
        "num_history_frames": int(task["num_history_frames"]),
        "history_storage": "full_episode_slowfast" if tiers else "ring",
        "storage_capacity_frames": None if tiers else int(task["num_history_frames"]),
        "video_fps": float(task["video_fps"]),
        "video_size_height_width": list(common["video_size"]),
        "aspect_mode": "stretch", "slowfast_tiers": tiers,
        "pool_enable": bool(common["pool_enable"]),
        "pool_spatial": int(common["pool_spatial"]), "pool_mode": common["pool_mode"],
        "pool_stage": common["pool_stage"], "temporal_patch_size": 2,
        "processor_do_sample_frames": False,
        "timestamp_relative_from_checkpoint": bool(task.get("timestamp_relative", False)),
        "model_timestamp_basis": "session absolute delivered-frame index / configured video_fps; not measured capture time",
        "spatial_preprocessing": "RGB uint8 -> CHW float /255 -> bilinear resize align_corners=False -> 2*x-1",
        "history_full_definition": "actual delivered history count >= nominal num_history_frames; not a full-episode storage cap or all-tier coverage claim",
        "sampling_verification": "reconstructed by read-only pinned official slowfast_segments; server does not return internal selected frame IDs",
        "source_sha256": hashes,
        "eval_config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
        "processing_overrides": None,
    }
    if contract["num_history_frames"] <= 0 or contract["video_fps"] <= 0:
        raise ValueError("invalid checkpoint history configuration")
    return contract, module.slowfast_segments


def validate_frame(frame: dict) -> dict:
    """Copy a capture record while enforcing one render-state/time association."""
    if not isinstance(frame, dict):
        raise ValueError("frame metadata must be a dictionary")
    out = deepcopy(frame)
    for key in ("frame_id", "path", "sha256", "rendered_state_id"):
        if key not in out or out[key] is None or str(out[key]) == "":
            raise ValueError(f"frame is missing {key}")
    if not isinstance(out["frame_id"], (str, int)) or isinstance(out["frame_id"], bool):
        raise ValueError("frame_id must be a string or integer")
    if not isinstance(out["sha256"], str) or len(out["sha256"]) != 64:
        raise ValueError("frame SHA-256 must be hexadecimal")
    try:
        int(out["sha256"], 16)
    except ValueError as exc:
        raise ValueError("frame SHA-256 must be hexadecimal") from exc
    for key in ("capture_sim_time_s", "capture_monotonic_ns"):
        if isinstance(out.get(key), bool) or not isinstance(out.get(key), (int, float)) or not math.isfinite(out[key]) or out[key] < 0:
            raise ValueError(f"invalid {key}")
    if type(out["capture_monotonic_ns"]) is not int:
        raise ValueError("capture_monotonic_ns must be integer nanoseconds")
    pose = out.get("pose_world")
    if not isinstance(pose, (list, tuple)) or len(pose) != 3 or not all(math.isfinite(float(x)) for x in pose):
        raise ValueError("capture pose must be a finite world SE(2) pose")
    for key in ("bootstrap", "stationary"):
        if type(out.get(key)) is not bool:
            raise ValueError(f"{key} must be recorded explicitly")
    return out


def _after(previous: dict, current: dict) -> None:
    if current["capture_monotonic_ns"] <= previous["capture_monotonic_ns"]:
        raise ValueError("capture host times must be strictly chronological")
    if current["capture_sim_time_s"] < previous["capture_sim_time_s"]:
        raise ValueError("capture simulation time moved backwards")


class CaptureQueue:
    """Bounded capture queue; overflow is explicit and never silently drops data."""

    def __init__(self, capacity: int = 256):
        if type(capacity) is not int or capacity < 1:
            raise ValueError("queue capacity must be a positive integer")
        self.capacity = capacity
        self._queue: deque[dict] = deque()
        self._ids: set = set()
        self._last: dict | None = None
        self.overflow_count = 0

    def append(self, frame: dict) -> None:
        frame = validate_frame(frame)
        if frame["frame_id"] in self._ids:
            raise ValueError("camera frame ID was already captured")
        if self._last is not None:
            _after(self._last, frame)
        if len(self._queue) >= self.capacity:
            self.overflow_count += 1
            raise OverflowError("capture queue overflow; frame was not enqueued")
        self._queue.append(frame)
        self._ids.add(frame["frame_id"])
        self._last = frame

    def pop(self) -> dict:
        return deepcopy(self._queue.popleft())

    def __len__(self) -> int:
        return len(self._queue)


class SessionHistory:
    """One episode, one wire session, serial `next` calls including buffer-only.

    `begin` reserves a frame/sequence once, freezing this prediction's causal
    input. Captures arriving during inference stay in the separate queue.
    """

    def __init__(self, instruction: str, contract: dict, sampler: Callable | None = None):
        if not isinstance(instruction, str) or not instruction.strip():
            raise ValueError("navigation instruction must be nonempty")
        if int(contract.get("num_history_frames", 0)) < 1:
            raise ValueError("history capacity must be positive")
        self.instruction = instruction
        self.contract = deepcopy(contract)
        self.sampler = sampler
        self.frames: list[dict] = []
        self._seen: set = set()
        self.pending: dict | None = None
        self.next_seq = 0
        self.prediction_count = 0
        self.failed = False
        self.login_count = 0
        self.reset_count = 0

    def login(self) -> None:
        if self.login_count or self.reset_count or self.frames:
            raise ValueError("exactly one login per episode")
        self.login_count = 1

    def reset(self) -> None:
        if self.login_count != 1 or self.reset_count or self.frames:
            raise ValueError("one initial reset only; successive resets are prohibited")
        self.reset_count = 1

    def begin(self, frame: dict, *, predict: bool, send_monotonic_ns: int,
              instruction: str | None = None) -> dict:
        if self.failed:
            raise ValueError("failed session cannot be reused")
        if self.login_count != 1 or self.reset_count != 1:
            raise ValueError("login and one initial reset are required")
        if self.pending is not None:
            raise ValueError("maximum one outstanding wire request")
        if type(predict) is not bool:
            raise ValueError("predict must be boolean")
        if instruction is not None and instruction != self.instruction:
            raise ValueError("navigation instruction cannot change within an episode")
        frame = validate_frame(frame)
        if frame["frame_id"] in self._seen:
            raise ValueError("duplicate frame append is prohibited")
        if type(send_monotonic_ns) is not int or frame["capture_monotonic_ns"] > send_monotonic_ns:
            raise ValueError("future frame cannot enter request history")
        if self.frames:
            _after(self.frames[-1], frame)
        seq = self.next_seq
        self.next_seq += 1
        frame["server_frame_index_reconstructed"] = len(self.frames)
        frame["wire_seq"] = seq
        self.frames.append(frame)
        self._seen.add(frame["frame_id"])
        pending = {"seq": seq, "predict": predict, "frame": frame,
                   "send_monotonic_ns": send_monotonic_ns,
                   "prediction_index": self.prediction_count if predict else None}
        if predict:
            self.prediction_count += 1
        self.pending = pending
        return deepcopy(pending)

    def snapshot(self, server_actions_step: int | None = None) -> dict:
        if self.pending is None:
            raise ValueError("a snapshot belongs to an outstanding request")
        full_episode = self.contract["history_storage"] == "full_episode_slowfast"
        capacity = int(self.contract["num_history_frames"])
        frames = self.frames if full_episode else self.frames[-capacity:]
        if server_actions_step is not None and (type(server_actions_step) is not int or server_actions_step != len(frames)):
            raise ValueError("server actions.step differs from reconstructed delivery history")
        indices = [f["server_frame_index_reconstructed"] for f in frames]
        if full_episode:
            if self.sampler is None:
                raise ValueError("official sampler is required for SlowFast reconstruction")
            segments = self.sampler(len(self.frames) - 1, len(self.frames), self.contract["slowfast_tiers"])
        elif self.contract.get("pool_enable") and self.contract.get("pool_spatial", 1) > 1 and len(frames) > 2:
            segments = [{"frame_ids": indices[:-2], "pool_spatial": self.contract["pool_spatial"]},
                        {"frame_ids": indices[-2:], "pool_spatial": 1}]
        else:
            segments = [{"frame_ids": indices, "pool_spatial": 1}]
        mapped = []
        for segment in segments:
            ids = list(segment["frame_ids"])
            # Ring model's video processor also pads odd segments to tubelet size2.
            if not full_episode and len(ids) % 2:
                ids.append(ids[-1])
            if any(type(i) is not int or i < 0 or i >= len(self.frames) for i in ids):
                raise ValueError("sampler selected a future or unavailable frame")
            mapped.append({**segment, "server_frame_indices_reconstructed": ids,
                           "frame_ids": [self.frames[i]["frame_id"] for i in ids]})
        selected = sorted({i for segment in mapped for i in segment["server_frame_indices_reconstructed"]})
        return {
            "triggering_observation_frame_id": self.pending["frame"]["frame_id"],
            "wire_seq": self.pending["seq"], "prediction_index": self.pending["prediction_index"],
            "chronological_history_frame_ids": [f["frame_id"] for f in frames],
            "frames": deepcopy(frames), "actual_history_count": len(frames),
            "total_delivered_frames_this_session": len(self.frames),
            "server_actions_step_observed": server_actions_step,
            "server_count_agrees": server_actions_step == len(frames) if server_actions_step is not None else None,
            "history_full": len(frames) >= capacity,
            "bootstrap_frames_included": any(f["bootstrap"] for f in frames),
            "stationary_frames_included": any(f["stationary"] for f in frames),
            "capture_sim_range_s": [frames[0]["capture_sim_time_s"], frames[-1]["capture_sim_time_s"]],
            "capture_monotonic_range_ns": [frames[0]["capture_monotonic_ns"], frames[-1]["capture_monotonic_ns"]],
            "capture_sim_span_s": frames[-1]["capture_sim_time_s"] - frames[0]["capture_sim_time_s"],
            "capture_host_span_s": (frames[-1]["capture_monotonic_ns"] - frames[0]["capture_monotonic_ns"]) / 1e9,
            "model_input_segments_reconstructed": mapped,
            "model_input_unique_frame_ids_reconstructed": [self.frames[i]["frame_id"] for i in selected],
            "model_input_unique_frame_count_reconstructed": len(selected),
            "model_input_slots_including_processor_padding_reconstructed": sum(len(x["frame_ids"]) for x in mapped),
            "internal_selection_directly_observed": False,
            "internal_selection_verification": self.contract.get("sampling_verification", "source reconstruction"),
            "processor_padding_is_not_a_new_camera_frame": True,
            "history_contract": deepcopy(self.contract),
        }

    def complete(self, response_seq: int, *, server_actions_step: int | None = None) -> dict:
        if self.pending is None or type(response_seq) is not int or response_seq != self.pending["seq"]:
            self.failed = True
            raise ValueError("response sequence differs from outstanding request")
        if self.pending["predict"] and server_actions_step is None:
            self.failed = True
            raise ValueError("prediction is missing server actions.step")
        try:
            result = self.snapshot(server_actions_step)
        except Exception:
            self.failed = True
            raise
        self.pending = None
        return result

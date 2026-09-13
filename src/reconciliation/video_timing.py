"""Validate sampled live-physics video timing; no pose interpolation or replay."""

import math


def validate_video_frames(frames, physics_dt, stride, telemetry_times, telemetry_poses):
    if isinstance(stride, bool) or not isinstance(stride, int) or stride < 1:
        raise ValueError("capture stride must be a positive integer")
    if not math.isfinite(physics_dt) or physics_dt <= 0:
        raise ValueError("physics dt must be finite and positive")
    expected = list(range(0, len(telemetry_times), stride))
    if [row["physics_index"] for row in frames] != expected:
        raise ValueError("video frames are missing, duplicated or out of order")
    for row in frames:
        index = row["physics_index"]
        if abs(row["sim_time_s"] - float(telemetry_times[index])) > 1e-8:
            raise ValueError("video time differs from measured telemetry")
        if list(row["pose_world_se2"]) != list(telemetry_poses[index]):
            raise ValueError("video pose differs from measured telemetry")
    return {"validated_frames": len(frames), "nominal_simulation_fps": 1 / (physics_dt * stride),
            "last_sim_time_s": frames[-1]["sim_time_s"],
            "timing": "one real rendered state per sampled physics step; no interpolated poses"}

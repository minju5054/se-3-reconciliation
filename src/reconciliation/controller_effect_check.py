"""Untuned rotation checks using the existing execution telemetry conventions.

The runtime is supplied by Isaac or an explicit synthetic test double. No
reference waypoint is assigned a timestamp. Command rows describe the physical
interval ending at that row, as in the existing Stage 0-D/E telemetry.
"""

from __future__ import annotations

import math
from typing import Any, Mapping

import numpy as np

from reconciliation.controller_validation import estimate_body_velocities
from reconciliation.execution_calibration import ExecutionTelemetry


def rotation_schedule(values: Mapping[str, Any], dt: float) -> list[str]:
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("control interval must be positive and finite")
    result = []
    for phase, key in (
        ("INITIAL_STOP", "initial_stop_s"),
        ("ACTIVE", "active_s"),
        ("FINAL_STOP", "final_stop_s"),
    ):
        duration = float(values[key])
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError("phase duration must be positive and finite")
        count = round(duration / dt)
        if not math.isclose(count * dt, duration, rel_tol=0, abs_tol=1e-9):
            raise ValueError("phase duration must be divisible by control interval")
        result.extend([phase] * count)
    return result


def record_rotation(runtime, values: Mapping[str, Any], mode: str, omega: float):
    """Observe an actual runtime; synthetic runtimes are used only in tests.

    Runtime supplies reset/apply/step/stop, read_pose/read_wheels, ideal_sides,
    canonical_targets, correction, and explicit physics/control clock values.
    """
    if mode not in ("nominal", "calibrated"):
        raise ValueError("unknown execution mode")
    if not math.isfinite(omega) or omega == 0:
        raise ValueError("rotation omega must be finite and nonzero")
    schedule = rotation_schedule(values, runtime.control_dt)
    if not math.isclose(runtime.physics_steps * runtime.physics_dt,
                        runtime.control_dt, rel_tol=0, abs_tol=1e-9):
        raise ValueError("physics steps must span one control interval")
    runtime.reset(np.asarray(values["initial_pose_world_se2"], dtype=float), 1.0)
    correction = runtime.correction() if mode == "calibrated" else None
    poses = [runtime.read_pose().copy()]
    times, indices, phases = [0.0], [-1], ["INITIAL_STOP"]
    desired_rows, executed_rows, ideal_rows = [np.zeros(2)], [np.zeros(2)], [np.zeros(2)]
    targets, wheels, saturated = [np.zeros(4)], [runtime.read_wheels().copy()], [False]
    states = []
    origin = float(runtime.world.current_time)
    measured_omega = 0.0
    for index, phase in enumerate(schedule):
        desired = np.array([0.0, omega if phase == "ACTIVE" else 0.0])
        start_time = float(runtime.world.current_time) - origin
        previous = poses[-1]
        executed, target, state = runtime.apply(desired, mode, correction, measured_omega)
        ideal = runtime.ideal_sides(desired)
        canonical = runtime.canonical_targets(target)
        for _ in range(runtime.physics_steps):
            runtime.step()
            poses.append(runtime.read_pose().copy())
            times.append(float(runtime.world.current_time) - origin)
            indices.append(index)
            phases.append(phase)
            desired_rows.append(desired.copy())
            executed_rows.append(executed.copy())
            ideal_rows.append(ideal.copy())
            targets.append(canonical.copy())
            wheels.append(runtime.read_wheels().copy())
            saturated.append(bool(state["saturated"]))
        elapsed = times[-1] - start_time
        if not math.isclose(elapsed, runtime.control_dt, rel_tol=0, abs_tol=1e-6):
            raise ValueError("runtime clock differs from the declared control interval")
        _, observed_omega = estimate_body_velocities(
            np.asarray([previous, poses[-1]]), np.array([0.0, elapsed])
        )
        measured_omega = float(observed_omega[-1])
        states.append({"control_index": index, "phase": phase,
                       "execution_start_s": start_time, "observation_end_s": times[-1],
                       "measured_omega_for_next_control_rps": measured_omega, **state})
    runtime.stop()
    telemetry = ExecutionTelemetry(
        np.asarray(poses), np.asarray(times), np.asarray(indices), tuple(phases),
        np.asarray(desired_rows), np.asarray(executed_rows), np.asarray(ideal_rows),
        np.asarray(targets), np.asarray(wheels), np.asarray(saturated),
    )
    return telemetry, states


def rotation_drift_metrics(telemetry: ExecutionTelemetry) -> dict[str, Any]:
    """XY movement relative to the pose just before ACTIVE, in metres."""
    active = np.flatnonzero(np.asarray(telemetry.phases) == "ACTIVE")
    if not len(active) or active[0] == 0:
        raise ValueError("ACTIVE requires a preceding observed pose")
    if np.any(telemetry.desired_body[active, 0] != 0):
        raise ValueError("rotation drift requires zero desired translation")
    start = telemetry.actual_trajectory[active[0] - 1, :2]
    distances = np.linalg.norm(telemetry.actual_trajectory[active, :2] - start, axis=1)
    return {
        "active_max_xy_drift_m": float(distances.max()),
        "active_end_xy_drift_m": float(distances[-1]),
        "after_final_stop_xy_drift_m": float(np.linalg.norm(telemetry.actual_trajectory[-1, :2] - start)),
        "drift_anchor_sim_time_s": float(telemetry.sim_times_s[active[0] - 1]),
        "drift_semantics": "world XY displacement from observed pose immediately before ACTIVE",
    }

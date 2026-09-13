"""Synthetic tests for the diagnostic recorder, not experimental evidence."""

from types import SimpleNamespace

import numpy as np
import pytest

from reconciliation.controller_effect_check import (
    record_rotation, rotation_drift_metrics, rotation_schedule,
)
from reconciliation.se2 import wrap_angle


def settings():
    return {"initial_pose_world_se2": [2.0, -3.0, np.pi - 0.05],
            "initial_stop_s": 0.2, "active_s": 0.2, "final_stop_s": 0.2}


class SyntheticRuntime:
    physics_dt = 0.05
    control_dt = 0.1
    physics_steps = 2

    def __init__(self, clock_scale=1.0):
        self.world = SimpleNamespace(current_time=0.0)
        self.clock_scale = clock_scale
        self.feedback = []
        self.stopped = False

    def reset(self, pose, settling_s):
        self.pose = pose.copy()
        self.world.current_time = settling_s

    def correction(self):
        return object()

    def apply(self, desired, mode, correction, measured_omega):
        assert (correction is not None) == (mode == "calibrated")
        self.command = desired.copy()
        self.feedback.append(measured_omega)
        return desired.copy(), np.zeros(4), {"saturated": False}

    def step(self):
        self.world.current_time += self.physics_dt * self.clock_scale
        # Explicit synthetic lateral drift tests the metric's active-phase anchor.
        if self.command[1] != 0:
            self.pose[0] += 0.01 * self.physics_dt
        self.pose[2] = wrap_angle(self.pose[2] + self.command[1] * self.physics_dt)

    def read_pose(self):
        return self.pose

    def read_wheels(self):
        return np.zeros(4)

    def ideal_sides(self, desired):
        return np.array([-desired[1], desired[1]])

    def canonical_targets(self, target):
        return target

    def stop(self):
        self.stopped = True


@pytest.mark.parametrize("mode", ["nominal", "calibrated"])
@pytest.mark.parametrize("omega", [-1.5, 1.5])
def test_recorder_uses_ending_intervals_and_previous_control_feedback(mode, omega):
    runtime = SyntheticRuntime()
    telemetry, states = record_rotation(runtime, settings(), mode, omega)
    assert runtime.stopped
    assert telemetry.sample_count == 13
    assert telemetry.sim_times_s == pytest.approx(np.arange(13) * 0.05)
    assert telemetry.control_indices.tolist() == [-1, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5]
    assert telemetry.phases[4:7] == ("INITIAL_STOP", "ACTIVE", "ACTIVE")
    assert telemetry.desired_body[:5, 1] == pytest.approx(np.zeros(5))
    assert telemetry.desired_body[5:9, 1] == pytest.approx([omega] * 4)
    assert telemetry.desired_body[9:, 1] == pytest.approx(np.zeros(4))
    assert runtime.feedback == pytest.approx([0, 0, 0, omega, omega, 0])
    assert states[2]["execution_start_s"] == pytest.approx(0.2)
    assert states[2]["observation_end_s"] == pytest.approx(0.3)
    # The positive case crosses +pi; feedback must still be +1.5, not -61.3.
    assert states[2]["measured_omega_for_next_control_rps"] == pytest.approx(omega)
    drift = rotation_drift_metrics(telemetry)
    assert drift["drift_anchor_sim_time_s"] == pytest.approx(0.2)
    assert drift["active_end_xy_drift_m"] == pytest.approx(0.002)
    assert drift["active_max_xy_drift_m"] == pytest.approx(0.002)
    assert drift["after_final_stop_xy_drift_m"] == pytest.approx(0.002)


@pytest.mark.parametrize("value", [0, -1, float("nan"), 0.15])
def test_reject_invalid_or_fractional_control_phase(value):
    values = settings()
    values["active_s"] = value
    with pytest.raises(ValueError):
        rotation_schedule(values, 0.1)


def test_reject_wrong_runtime_clock():
    with pytest.raises(ValueError, match="runtime clock"):
        record_rotation(SyntheticRuntime(clock_scale=0.9), settings(), "nominal", 1.5)


def test_drift_cannot_label_forward_command_as_rotation():
    telemetry, _ = record_rotation(SyntheticRuntime(), settings(), "nominal", 1.5)
    telemetry.desired_body[5, 0] = 0.1
    with pytest.raises(ValueError, match="zero desired translation"):
        rotation_drift_metrics(telemetry)

"""Minimal deterministic Jackal body yaw-rate execution correction."""

from __future__ import annotations

from dataclasses import dataclass
import math


def _finite(name: str, value: float) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _positive(name: str, value: float) -> float:
    result = _finite(name, value)
    if result <= 0.0:
        raise ValueError(f"{name} must be greater than zero")
    return result


@dataclass(frozen=True, slots=True)
class JackalExecutionControllerConfig:
    physical_wheel_separation_m: float
    calibrated_effective_wheel_separation_m: float
    yaw_rate_kp: float
    yaw_rate_ki: float
    maximum_abs_executed_omega_rps: float
    sign_protection: bool = True

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "physical_wheel_separation_m",
            _positive("physical_wheel_separation_m", self.physical_wheel_separation_m),
        )
        object.__setattr__(
            self,
            "calibrated_effective_wheel_separation_m",
            _positive(
                "calibrated_effective_wheel_separation_m",
                self.calibrated_effective_wheel_separation_m,
            ),
        )
        for name in ("yaw_rate_kp", "yaw_rate_ki"):
            value = _finite(name, getattr(self, name))
            if value < 0.0:
                raise ValueError(f"{name} must be non-negative")
            object.__setattr__(self, name, value)
        object.__setattr__(
            self,
            "maximum_abs_executed_omega_rps",
            _positive(
                "maximum_abs_executed_omega_rps",
                self.maximum_abs_executed_omega_rps,
            ),
        )
        if not isinstance(self.sign_protection, bool):
            raise ValueError("sign_protection must be boolean")

    @property
    def feedforward_scale(self) -> float:
        return (
            self.calibrated_effective_wheel_separation_m
            / self.physical_wheel_separation_m
        )


@dataclass(frozen=True, slots=True)
class JackalExecutionCommand:
    desired_v_mps: float
    desired_omega_rps: float
    executed_v_mps: float
    executed_omega_rps: float
    omega_error_rps: float
    integral_error_rad: float
    unsaturated_omega_rps: float
    saturated: bool


class JackalExecutionController:
    """Effective-width feedforward plus optional PI body-yaw-rate feedback.

    The controller changes only the body command sent to the existing official
    DifferentialController. It never changes physical wheel geometry or USD
    properties. Conditional integration provides deterministic anti-windup.
    """

    def __init__(self, config: JackalExecutionControllerConfig) -> None:
        self.config = config
        self._integral_error_rad = 0.0
        self._previous_desired_omega_rps = 0.0

    @property
    def integral_error_rad(self) -> float:
        return self._integral_error_rad

    def reset(self) -> None:
        self._integral_error_rad = 0.0
        self._previous_desired_omega_rps = 0.0

    def forward(
        self,
        *,
        desired_v_mps: float,
        desired_omega_rps: float,
        measured_omega_rps: float,
        control_dt_s: float,
    ) -> JackalExecutionCommand:
        desired_v = _finite("desired_v_mps", desired_v_mps)
        desired_omega = _finite("desired_omega_rps", desired_omega_rps)
        measured_omega = _finite("measured_omega_rps", measured_omega_rps)
        dt = _positive("control_dt_s", control_dt_s)
        if (
            desired_omega == 0.0
            or desired_omega * self._previous_desired_omega_rps < 0.0
        ):
            self._integral_error_rad = 0.0
        self._previous_desired_omega_rps = desired_omega
        error = desired_omega - measured_omega
        candidate_integral = self._integral_error_rad + error * dt

        def control(integral: float) -> float:
            return (
                self.config.feedforward_scale * desired_omega
                + self.config.yaw_rate_kp * error
                + self.config.yaw_rate_ki * integral
            )

        unsaturated = control(candidate_integral)
        limit = self.config.maximum_abs_executed_omega_rps
        executed = max(-limit, min(limit, unsaturated))
        saturated = not math.isclose(executed, unsaturated, rel_tol=0.0, abs_tol=1e-12)

        # Reject integration when it would drive an already saturated output
        # farther into saturation. Recompute using the retained state.
        if saturated and error * unsaturated > 0.0:
            unsaturated = control(self._integral_error_rad)
            executed = max(-limit, min(limit, unsaturated))
        else:
            self._integral_error_rad = candidate_integral

        if (
            self.config.sign_protection
            and abs(desired_omega) > 0.0
            and executed * desired_omega < 0.0
        ):
            executed = 0.0
            saturated = True

        for name, value in (
            ("executed_omega_rps", executed),
            ("integral_error_rad", self._integral_error_rad),
            ("unsaturated_omega_rps", unsaturated),
        ):
            _finite(name, value)
        return JackalExecutionCommand(
            desired_v_mps=desired_v,
            desired_omega_rps=desired_omega,
            executed_v_mps=desired_v,
            executed_omega_rps=executed,
            omega_error_rps=error,
            integral_error_rad=self._integral_error_rad,
            unsaturated_omega_rps=unsaturated,
            saturated=saturated,
        )

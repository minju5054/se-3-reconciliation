import math

import pytest

from reconciliation.controllers.jackal_execution_controller import (
    JackalExecutionController,
    JackalExecutionControllerConfig,
)


def make_controller(**overrides: float | bool) -> JackalExecutionController:
    values = {
        "physical_wheel_separation_m": 0.4,
        "calibrated_effective_wheel_separation_m": 0.8,
        "yaw_rate_kp": 1.0,
        "yaw_rate_ki": 0.5,
        "maximum_abs_executed_omega_rps": 2.0,
        "sign_protection": True,
    }
    values.update(overrides)
    return JackalExecutionController(JackalExecutionControllerConfig(**values))


def test_feedforward_feedback_reset_and_finite_output() -> None:
    controller = make_controller()
    output = controller.forward(
        desired_v_mps=0.3,
        desired_omega_rps=0.4,
        measured_omega_rps=0.1,
        control_dt_s=0.1,
    )
    assert output.executed_v_mps == pytest.approx(0.3)
    assert output.executed_omega_rps == pytest.approx(1.115)
    assert math.isfinite(output.executed_omega_rps)
    assert controller.integral_error_rad == pytest.approx(0.03)
    controller.reset()
    assert controller.integral_error_rad == 0.0


def test_saturation_uses_conditional_anti_windup() -> None:
    controller = make_controller(
        calibrated_effective_wheel_separation_m=4.0,
        maximum_abs_executed_omega_rps=1.0,
    )
    first = controller.forward(
        desired_v_mps=0.0,
        desired_omega_rps=0.5,
        measured_omega_rps=0.0,
        control_dt_s=0.1,
    )
    assert first.saturated is True
    assert first.executed_omega_rps == 1.0
    assert controller.integral_error_rad == 0.0


def test_sign_reversal_resets_integrator_and_sign_protection() -> None:
    controller = make_controller(calibrated_effective_wheel_separation_m=0.4)
    controller.forward(
        desired_v_mps=0.0,
        desired_omega_rps=0.4,
        measured_omega_rps=0.0,
        control_dt_s=0.1,
    )
    output = controller.forward(
        desired_v_mps=0.0,
        desired_omega_rps=-0.1,
        measured_omega_rps=1.0,
        control_dt_s=0.1,
    )
    assert output.integral_error_rad < 0.0
    assert output.executed_omega_rps <= 0.0


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_non_finite_inputs_are_rejected(value: float) -> None:
    controller = make_controller()
    with pytest.raises(ValueError, match="finite"):
        controller.forward(
            desired_v_mps=0.0,
            desired_omega_rps=value,
            measured_omega_rps=0.0,
            control_dt_s=0.1,
        )


def test_invalid_controller_configuration_is_rejected() -> None:
    with pytest.raises(ValueError, match="greater than zero"):
        make_controller(physical_wheel_separation_m=0.0)

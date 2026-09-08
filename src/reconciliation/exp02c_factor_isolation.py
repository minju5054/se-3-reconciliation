"""Pure EXP-02C factor-isolation and failure-attribution helpers.

This module deliberately does not generalize :mod:`transition_graph`.  It mirrors
the frozen EXP-02A/02B physical residuals in a diagnostic-only problem so factor
subsets, numerical gradients, and the no-propagation counterfactual can be
measured without changing the production objective.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Callable, Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reconciliation.controllers.trajectory_follower import FollowerConfig, TrajectoryFollower
from reconciliation.graph_optimizer import SolverConfig, numerical_jacobian, retract_trajectory
from reconciliation.se2 import compose_poses, inverse_pose, relative_pose, se2_log, wrap_angle
from reconciliation.se2_graph import new_motion_residual
from reconciliation.spatial_entry import TransitionReconciliationInput
from reconciliation.trajectory import validate_pose_se2, validate_se2_trajectory
from reconciliation.transition_graph import (
    TransitionResidualScales,
    entry_preservation_residual,
    incoming_motion_residual,
)


FloatArray = NDArray[np.float64]
FACTOR_ORDER = ("entry_preservation", "fresh_motion", "incoming_direction", "incoming_yaw")
VARIANT_ACTIVE_FACTORS: dict[str, tuple[str, ...]] = {
    "V0_RAW": (),
    "V1_ENTRY_FRESH": ("entry_preservation", "fresh_motion"),
    "V2_ENTRY_DIRECTION_FRESH": (
        "entry_preservation",
        "fresh_motion",
        "incoming_direction",
    ),
    "V3_ENTRY_YAW_FRESH": ("entry_preservation", "fresh_motion", "incoming_yaw"),
    "V4_FULL_CURRENT_M4": FACTOR_ORDER,
    "V5_NO_ENTRY": ("fresh_motion", "incoming_direction", "incoming_yaw"),
    "V6_NO_DIRECTION": ("entry_preservation", "fresh_motion", "incoming_yaw"),
    "V7_NO_YAW": ("entry_preservation", "fresh_motion", "incoming_direction"),
    "V8_DIAGNOSTIC_NO_PROPAGATION": (
        "entry_preservation",
        "incoming_direction",
        "incoming_yaw",
    ),
}


def _finite_positive(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


@dataclass(frozen=True, slots=True)
class FactorIsolationProblem:
    inputs: TransitionReconciliationInput
    active_factors: tuple[str, ...]
    scales: TransitionResidualScales
    weights: Mapping[str, float]
    minimum_translation_m: float
    diagnostic_no_propagation: bool = False

    def __post_init__(self) -> None:
        active = tuple(self.active_factors)
        if len(active) != len(set(active)) or any(name not in FACTOR_ORDER for name in active):
            raise ValueError("active_factors contains duplicates or unknown factors")
        if not active:
            raise ValueError("an optimization problem needs at least one active factor")
        clean_weights: dict[str, float] = {}
        for name in FACTOR_ORDER:
            clean_weights[name] = _finite_positive(self.weights[name], f"weight {name}")
        object.__setattr__(self, "active_factors", active)
        object.__setattr__(self, "weights", clean_weights)
        object.__setattr__(
            self,
            "minimum_translation_m",
            _finite_positive(self.minimum_translation_m, "minimum_translation_m"),
        )
        if self.diagnostic_no_propagation and "fresh_motion" in active:
            raise ValueError("no-propagation diagnostic cannot activate fresh_motion")
        # Fail before solving if either fixed direction is undefined.
        if "incoming_direction" in active:
            incoming_motion_residual(
                self.inputs.incoming_previous_pose,
                self.inputs.committed_pose_world,
                self.raw[0],
                self.raw[0],
                minimum_translation_m=self.minimum_translation_m,
            )

    @property
    def raw(self) -> FloatArray:
        return self.inputs.selected_suffix


def problem_for_variant(
    inputs: TransitionReconciliationInput,
    variant: str,
    *,
    residual_scales: Mapping[str, Any],
    weights: Mapping[str, Any],
    minimum_translation_m: float,
) -> FactorIsolationProblem | None:
    if variant not in VARIANT_ACTIVE_FACTORS:
        raise ValueError(f"unknown EXP-02C variant: {variant}")
    active = VARIANT_ACTIVE_FACTORS[variant]
    if not active:
        return None
    return FactorIsolationProblem(
        inputs=inputs,
        active_factors=active,
        scales=TransitionResidualScales(**residual_scales),
        weights={name: float(weights[name]) for name in FACTOR_ORDER},
        minimum_translation_m=minimum_translation_m,
        diagnostic_no_propagation=variant == "V8_DIAGNOSTIC_NO_PROPAGATION",
    )


def physical_factor_residuals(
    inputs: TransitionReconciliationInput,
    state: ArrayLike,
    *,
    minimum_translation_m: float,
) -> dict[str, FloatArray]:
    """Return every current M4 factor in physical units, active or not."""

    trajectory = validate_se2_trajectory(state, name="state")
    raw = inputs.selected_suffix
    if trajectory.shape != raw.shape:
        raise ValueError("state shape must equal selected FRESH suffix shape")
    incoming = incoming_motion_residual(
        inputs.incoming_previous_pose,
        inputs.committed_pose_world,
        trajectory[0],
        raw[0],
        minimum_translation_m=_finite_positive(
            minimum_translation_m, "minimum_translation_m"
        ),
    )
    fresh = np.asarray(
        [
            new_motion_residual(raw[index], raw[index + 1], trajectory[index], trajectory[index + 1])
            for index in range(raw.shape[0] - 1)
        ],
        dtype=np.float64,
    ).reshape((-1, 3))
    return {
        "entry_preservation": entry_preservation_residual(raw[0], trajectory[0])[None, :],
        "fresh_motion": fresh,
        "incoming_direction": incoming[:2],
        "incoming_yaw": incoming[2:3],
    }


def factor_scale(problem: FactorIsolationProblem, factor: str) -> FloatArray | float:
    if factor in ("entry_preservation", "fresh_motion"):
        return problem.scales.pose_vector
    if factor == "incoming_direction":
        return problem.scales.direction_unitless
    if factor == "incoming_yaw":
        return problem.scales.yaw_rad
    raise ValueError(f"unknown factor: {factor}")


def single_factor_vector(
    problem: FactorIsolationProblem,
    state: ArrayLike,
    factor: str,
    *,
    normalization: str = "physical",
) -> FloatArray:
    groups = physical_factor_residuals(
        problem.inputs, state, minimum_translation_m=problem.minimum_translation_m
    )
    residual = groups[factor] / factor_scale(problem, factor)
    if normalization == "physical":
        residual = groups[factor]
    elif normalization == "normalized":
        pass
    elif normalization == "weighted":
        residual = math.sqrt(problem.weights[factor]) * residual
    else:
        raise ValueError("normalization must be physical, normalized, or weighted")
    result = np.asarray(residual, dtype=np.float64).reshape(-1)
    if not np.all(np.isfinite(result)):
        raise FloatingPointError("factor residual contains NaN or Inf")
    return result


def factor_residual_vector(problem: FactorIsolationProblem, state: ArrayLike) -> FloatArray:
    """Concatenate active factors in the exact historical M4 block order."""

    blocks = [
        single_factor_vector(problem, state, name, normalization="weighted")
        for name in FACTOR_ORDER
        if name in problem.active_factors
    ]
    if not blocks:
        raise ValueError("factor-isolation objective has no residuals")
    return np.concatenate(blocks)


@dataclass(frozen=True, slots=True)
class DiagnosticOptimizationResult:
    optimized: FloatArray
    initial_cost: float
    final_cost: float
    iterations: int
    accepted_steps: int
    rejected_steps: int
    converged: bool
    termination_reason: str
    cost_history: tuple[float, ...]
    damping_history: tuple[float, ...]

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value.pop("optimized")
        value["cost_history"] = list(self.cost_history)
        value["damping_history"] = list(self.damping_history)
        return value


def solve_diagnostic_least_squares(
    initial_state: ArrayLike,
    residual_function: Callable[[FloatArray], FloatArray],
    config: SolverConfig,
) -> DiagnosticOptimizationResult:
    """Frozen LM algorithm with explicit accepted/rejected step telemetry."""

    state = validate_se2_trajectory(initial_state, name="initial_state")

    def cost(candidate: FloatArray) -> float:
        residual = np.asarray(residual_function(candidate), dtype=np.float64)
        if residual.ndim != 1 or not np.all(np.isfinite(residual)):
            raise FloatingPointError("residual function must return a finite vector")
        return float(residual @ residual)

    initial_cost = cost(state)
    current_cost = initial_cost
    costs = [current_cost]
    damping = config.initial_damping
    dampings: list[float] = []
    accepted = 0
    rejected = 0
    converged = False
    reason = "maximum_iterations"

    for iteration in range(1, config.max_iterations + 1):
        residual = np.asarray(residual_function(state), dtype=np.float64)
        jacobian = numerical_jacobian(
            residual_function, state, config.finite_difference_epsilon
        )
        gradient = jacobian.T @ residual
        if not np.all(np.isfinite(gradient)):
            raise RuntimeError("normal-equation gradient is not finite")
        if float(np.linalg.norm(gradient, ord=np.inf)) <= config.gradient_tolerance:
            converged = True
            reason = "gradient_tolerance"
            break
        normal = jacobian.T @ jacobian
        dampings.append(damping)
        try:
            delta = np.linalg.solve(
                normal + damping * np.eye(normal.shape[0]), -gradient
            )
        except np.linalg.LinAlgError as error:
            raise RuntimeError("damped normal system is singular") from error
        if not np.all(np.isfinite(delta)):
            raise RuntimeError("solver produced a non-finite update")
        if float(np.linalg.norm(delta)) <= config.step_tolerance:
            converged = True
            reason = "step_tolerance"
            break
        candidate = retract_trajectory(state, delta)
        candidate_cost = cost(candidate)
        if candidate_cost < current_cost:
            decrease = current_cost - candidate_cost
            state = candidate
            current_cost = candidate_cost
            costs.append(current_cost)
            accepted += 1
            damping = max(np.finfo(np.float64).eps, damping * config.damping_decrease)
            if decrease <= config.cost_tolerance:
                converged = True
                reason = "cost_tolerance"
                break
        else:
            rejected += 1
            damping *= config.damping_increase
            if damping > config.maximum_damping:
                raise RuntimeError("damping exceeded maximum without a cost decrease")
    else:
        iteration = config.max_iterations

    return DiagnosticOptimizationResult(
        optimized=validate_se2_trajectory(state, name="optimized"),
        initial_cost=initial_cost,
        final_cost=current_cost,
        iterations=iteration,
        accepted_steps=accepted,
        rejected_steps=rejected,
        converged=converged,
        termination_reason=reason,
        cost_history=tuple(costs),
        damping_history=tuple(dampings),
    )


def solve_variant(
    problem: FactorIsolationProblem | None,
    raw: ArrayLike,
    solver: SolverConfig,
) -> DiagnosticOptimizationResult:
    reference = validate_se2_trajectory(raw, name="raw")
    if problem is None:
        return DiagnosticOptimizationResult(
            optimized=reference,
            initial_cost=0.0,
            final_cost=0.0,
            iterations=0,
            accepted_steps=0,
            rejected_steps=0,
            converged=True,
            termination_reason="RAW_NO_OPTIMIZATION",
            cost_history=(0.0,),
            damping_history=(),
        )
    if problem.diagnostic_no_propagation:
        fixed = reference.copy()

        def residual(candidate: FloatArray) -> FloatArray:
            reconstructed = fixed.copy()
            reconstructed[0] = candidate[0]
            return factor_residual_vector(problem, reconstructed)

        unconstrained = solve_diagnostic_least_squares(reference, residual, solver)
        output = fixed.copy()
        output[0] = unconstrained.optimized[0]
        return DiagnosticOptimizationResult(
            optimized=output,
            initial_cost=unconstrained.initial_cost,
            final_cost=unconstrained.final_cost,
            iterations=unconstrained.iterations,
            accepted_steps=unconstrained.accepted_steps,
            rejected_steps=unconstrained.rejected_steps,
            converged=unconstrained.converged,
            termination_reason=unconstrained.termination_reason,
            cost_history=unconstrained.cost_history,
            damping_history=unconstrained.damping_history,
        )
    return solve_diagnostic_least_squares(
        reference, lambda candidate: factor_residual_vector(problem, candidate), solver
    )


def residual_diagnostics(
    problem: FactorIsolationProblem,
    state: ArrayLike,
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for name in FACTOR_ORDER:
        physical = single_factor_vector(problem, state, name, normalization="physical")
        normalized = single_factor_vector(problem, state, name, normalization="normalized")
        weighted = single_factor_vector(problem, state, name, normalization="weighted")
        result[name] = {
            "active": name in problem.active_factors,
            "historical_weight": problem.weights[name],
            "physical_residual": physical.tolist(),
            "physical_norm": float(np.linalg.norm(physical)),
            "normalized_norm": float(np.linalg.norm(normalized)),
            "diagnostic_weighted_cost": float(weighted @ weighted),
            "objective_weighted_cost": (
                float(weighted @ weighted) if name in problem.active_factors else 0.0
            ),
        }
    return result


def gradient_diagnostics(
    problem: FactorIsolationProblem,
    state: ArrayLike,
    *,
    epsilon: float,
) -> tuple[FloatArray, dict[str, Any]]:
    trajectory = validate_se2_trajectory(state, name="state")
    arrays: list[FloatArray] = []
    payload: dict[str, Any] = {
        "finite_difference_epsilon": _finite_positive(epsilon, "epsilon"),
        "retract_convention": "right-local T <- T * Exp(delta)",
        "state_shape": list(trajectory.shape),
        "factor_order": list(FACTOR_ORDER),
        "factors": {},
    }
    for name in FACTOR_ORDER:
        factor_payload: dict[str, Any] = {"active": name in problem.active_factors}
        gradients = []
        for normalization in ("physical", "weighted"):
            function = lambda value, n=name, mode=normalization: single_factor_vector(
                problem, value, n, normalization=mode
            )
            residual = function(trajectory)
            jacobian = numerical_jacobian(function, trajectory, epsilon)
            gradient = jacobian.T @ residual
            node_norms = np.linalg.norm(gradient.reshape(trajectory.shape), axis=1)
            gradients.append(gradient)
            factor_payload[normalization] = {
                "residual_size": int(residual.size),
                "jacobian_shape": list(jacobian.shape),
                "gradient_norm": float(np.linalg.norm(gradient)),
                "first_node_gradient_norm": float(node_norms[0]),
                "downstream_node_gradient_norms": node_norms[1:].tolist(),
                "per_node_gradient_norms": node_norms.tolist(),
            }
        arrays.append(np.stack(gradients))
        payload["factors"][name] = factor_payload
    stacked = np.stack(arrays)
    if not np.all(np.isfinite(stacked)):
        raise FloatingPointError("factor gradients contain NaN or Inf")
    payload["array_layout"] = "[factor, physical_or_weighted, flattened_right_local_state]"
    payload["normalization_order"] = ["physical", "weighted"]
    return stacked, payload


def gradient_cosines(
    gradients: ArrayLike,
    *,
    active_factors: Sequence[str],
    zero_norm_tolerance: float,
) -> dict[str, Any]:
    values = np.asarray(gradients, dtype=np.float64)
    if values.ndim != 3 or values.shape[0] != len(FACTOR_ORDER) or values.shape[1] != 2:
        raise ValueError("gradients must have layout [factor, physical/weighted, variables]")
    if not np.all(np.isfinite(values)):
        raise ValueError("gradients must be finite")
    threshold = _finite_positive(zero_norm_tolerance, "zero_norm_tolerance")
    indices = {name: index for index, name in enumerate(FACTOR_ORDER)}
    active = tuple(active_factors)
    pairs: dict[str, float | None] = {}
    matrix: list[list[float | None]] = []
    for left in active:
        row: list[float | None] = []
        a = values[indices[left], 1]
        a_norm = float(np.linalg.norm(a))
        for right in active:
            b = values[indices[right], 1]
            b_norm = float(np.linalg.norm(b))
            cosine = (
                None
                if a_norm <= threshold or b_norm <= threshold
                else float(np.clip((a @ b) / (a_norm * b_norm), -1.0, 1.0))
            )
            row.append(cosine)
            if indices[left] < indices[right]:
                pairs[f"{left}__vs__{right}"] = cosine
        matrix.append(row)
    return {
        "gradient": "normalized/weighted J_f^T r_f",
        "zero_norm_tolerance": threshold,
        "factor_order": list(active),
        "matrix": matrix,
        "pairs": pairs,
    }


def geometry_metrics(
    inputs: TransitionReconciliationInput,
    candidate: ArrayLike,
    *,
    minimum_translation_m: float,
) -> dict[str, Any]:
    raw = inputs.selected_suffix
    output = validate_se2_trajectory(candidate, name="candidate")
    if output.shape != raw.shape:
        raise ValueError("candidate shape must equal selected FRESH suffix")
    entry = se2_log(relative_pose(raw[0], output[0]))
    endpoint = se2_log(relative_pose(raw[-1], output[-1]))
    correction = se2_log(relative_pose(raw, output))
    edges = np.asarray(
        [
            new_motion_residual(raw[index], raw[index + 1], output[index], output[index + 1])
            for index in range(len(raw) - 1)
        ]
    )
    edge_translation = np.linalg.norm(edges[:, :2], axis=1)
    edge_yaw = np.abs(edges[:, 2])
    boundary = inputs.committed_pose_world
    transition = output[0, :2] - boundary[:2]
    distance = float(np.linalg.norm(transition))
    direction = (
        None
        if distance <= minimum_translation_m
        else math.atan2(float(transition[1]), float(transition[0]))
    )
    incoming = boundary[:2] - inputs.incoming_previous_pose[:2]
    tangent = incoming / np.linalg.norm(incoming)
    normal = np.asarray([-tangent[1], tangent[0]])
    lateral = float((output[-1, :2] - raw[-1, :2]) @ normal)
    return {
        "entry": {
            "se2_log_Fk_inverse_Xk": entry.tolist(),
            "translation_displacement_m": float(np.linalg.norm(entry[:2])),
            "yaw_displacement_rad": float(entry[2]),
            "abs_yaw_displacement_rad": abs(float(entry[2])),
        },
        "boundary_to_entry": {
            "distance_m": distance,
            "direction_world_rad": direction,
            "direction_defined": direction is not None,
            "yaw_increment_rad": float(relative_pose(boundary, output[0])[2]),
        },
        "fresh_relative_motion": {
            "per_edge_se2_log": edges.tolist(),
            "translation_rms_m": float(np.sqrt(np.mean(edge_translation**2))),
            "translation_max_m": float(np.max(edge_translation)),
            "yaw_rms_rad": float(np.sqrt(np.mean(edge_yaw**2))),
            "yaw_max_rad": float(np.max(edge_yaw)),
        },
        "absolute_fresh_correction": {
            "per_node_se2_log": correction.tolist(),
            "translation_m": np.linalg.norm(correction[:, :2], axis=1).tolist(),
            "abs_yaw_rad": np.abs(correction[:, 2]).tolist(),
        },
        "endpoint": {
            "se2_log": endpoint.tolist(),
            "translation_displacement_m": float(np.linalg.norm(endpoint[:2])),
            "yaw_displacement_rad": float(endpoint[2]),
            "abs_yaw_displacement_rad": abs(float(endpoint[2])),
            "lateral_displacement_in_incoming_frame_m": lateral,
        },
    }


def best_fit_left_se2(raw: ArrayLike, candidate: ArrayLike) -> dict[str, Any]:
    """Fit one left transform ``T`` such that ``candidate ~= T * raw``."""

    source = validate_se2_trajectory(raw, name="raw")
    target = validate_se2_trajectory(candidate, name="candidate")
    if source.shape != target.shape:
        raise ValueError("raw and candidate must have equal shape")
    source_center = np.mean(source[:, :2], axis=0)
    target_center = np.mean(target[:, :2], axis=0)
    covariance = (source[:, :2] - source_center).T @ (target[:, :2] - target_center)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1] *= -1.0
        rotation = vt.T @ u.T
    yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    translation = target_center - rotation @ source_center
    transform = np.asarray([translation[0], translation[1], yaw])
    fitted = compose_poses(transform, source)
    residual = se2_log(relative_pose(fitted, target))
    translation_error = np.linalg.norm(residual[:, :2], axis=1)
    yaw_error = np.abs(residual[:, 2])

    corrections = compose_poses(target, inverse_pose(source))
    relative_corrections = se2_log(relative_pose(corrections[0], corrections))
    variation_translation = np.linalg.norm(relative_corrections[:, :2], axis=1)
    variation_yaw = np.abs(relative_corrections[:, 2])
    return {
        "fit_convention": "best XY least-squares single left SE(2) transform T*: X_j ~= T* F_j",
        "best_fit_transform_se2": transform.tolist(),
        "translation_rms_m": float(np.sqrt(np.mean(translation_error**2))),
        "translation_max_m": float(np.max(translation_error)),
        "yaw_rms_rad": float(np.sqrt(np.mean(yaw_error**2))),
        "yaw_max_rad": float(np.max(yaw_error)),
        "per_node_fit_residual_se2_log": residual.tolist(),
        "node_correction_convention": "C_j = X_j * inverse(F_j)",
        "per_node_left_correction_se2": corrections.tolist(),
        "correction_variation_from_first_se2_log": relative_corrections.tolist(),
        "correction_variation_translation_rms_m": float(
            np.sqrt(np.mean(variation_translation**2))
        ),
        "correction_variation_translation_max_m": float(np.max(variation_translation)),
        "correction_variation_yaw_rms_rad": float(np.sqrt(np.mean(variation_yaw**2))),
        "correction_variation_yaw_max_rad": float(np.max(variation_yaw)),
    }


def controller_desired_metrics(
    candidate: ArrayLike,
    *,
    boundary: ArrayLike,
    old_final_command: ArrayLike,
    follower_values: Mapping[str, Any],
    window_size: int = 3,
) -> dict[str, Any]:
    """Probe desired commands at B, X0, X1 without inventing waypoint timestamps."""

    trajectory = validate_se2_trajectory(candidate, name="candidate")
    boundary_pose = validate_pose_se2(boundary, name="boundary")
    old = np.asarray(old_final_command, dtype=np.float64)
    if old.shape != (2,) or not np.all(np.isfinite(old)):
        raise ValueError("old_final_command must be finite shape (2,)")
    if not isinstance(window_size, int) or window_size < 1:
        raise ValueError("window_size must be positive")
    follower = TrajectoryFollower(
        trajectory,
        FollowerConfig(
            **{name: follower_values[name] for name in FollowerConfig.__dataclass_fields__}
        ),
    )
    probe_poses = [boundary_pose]
    probe_poses.extend(trajectory[min(index, len(trajectory) - 1)] for index in range(window_size - 1))
    commands = []
    follower_state = []
    for pose in probe_poses:
        command = follower.forward(pose)
        commands.append([command.linear_velocity_mps, command.angular_velocity_rps])
        follower_state.append(
            {
                "nearest_index": command.nearest_index,
                "target_index": command.target_index,
                "goal_reached": command.goal_reached,
            }
        )
    desired = np.asarray(commands)
    changes = np.diff(np.vstack((old, desired)), axis=0)
    immediate = desired[0] - old
    return {
        "old_final_desired_v_omega": old.tolist(),
        "candidate_first_desired_v_omega": desired[0].tolist(),
        "delta_v_des_signed_mps": float(immediate[0]),
        "delta_omega_des_signed_rps": float(immediate[1]),
        "delta_v_des_abs_mps": abs(float(immediate[0])),
        "delta_omega_des_abs_rps": abs(float(immediate[1])),
        "first3_desired_v_omega": desired.tolist(),
        "max3_abs_delta_v_mps": float(np.max(np.abs(changes[:, 0]))),
        "mean3_abs_delta_v_mps": float(np.mean(np.abs(changes[:, 0]))),
        "max3_abs_delta_omega_rps": float(np.max(np.abs(changes[:, 1]))),
        "mean3_abs_delta_omega_rps": float(np.mean(np.abs(changes[:, 1]))),
        "probe_poses_se2": np.asarray(probe_poses).tolist(),
        "follower_state": follower_state,
        "semantics": (
            "TrajectoryFollower desired-command spatial probe at B then candidate nodes; "
            "LightNav waypoint rows have no intrinsic timestamps and this is not a simulated response"
        ),
    }


def synthetic_conditions() -> dict[str, dict[str, FloatArray | str]]:
    """Deterministic mechanism fixtures; never experimental evidence."""

    old = np.asarray([[-0.4, 0.0, 0.0], [-0.2, 0.0, 0.0]])
    boundary = np.asarray([0.0, 0.0, 0.0])

    def line(entry: Sequence[float], step: Sequence[float]) -> FloatArray:
        poses = [np.asarray(entry, dtype=np.float64)]
        delta = np.asarray(step, dtype=np.float64)
        for _ in range(5):
            poses.append(compose_poses(poses[-1], delta))
        return np.asarray(poses)

    s0 = line([0.2, 0.0, 0.0], [0.2, 0.0, 0.0])
    s1 = line([0.0, 0.2, 0.0], [0.2, 0.0, 0.0])
    s2 = line([0.2, 0.0, 0.35], [0.2, 0.0, 0.0])
    s3 = line([0.8, 0.0, 0.0], [0.2, 0.0, 0.0])
    s4 = line([0.0, 0.2, 0.0], [0.2, 0.0, 0.0])
    desired_s4 = s4.copy()
    desired_s4[0] = [0.2, 0.0, 0.0]
    return {
        "S0_PERFECTLY_BENIGN": {
            "old": old.copy(), "boundary": boundary.copy(), "fresh": s0,
            "purpose": "all current transition factors are zero at raw",
        },
        "S1_DIRECTION_ONLY": {
            "old": old.copy(), "boundary": boundary.copy(), "fresh": s1,
            "purpose": "only incoming direction is nonzero at raw",
        },
        "S2_YAW_ONLY": {
            "old": old.copy(), "boundary": boundary.copy(), "fresh": s2,
            "purpose": "only incoming yaw is nonzero at raw",
        },
        "S3_MAGNITUDE_STRESS": {
            "old": old.copy(), "boundary": boundary.copy(), "fresh": s3,
            "purpose": "current factors have no OLD-step magnitude target",
        },
        "S4_DOWNSTREAM_CONFLICT": {
            "old": old.copy(), "boundary": boundary.copy(), "fresh": s4,
            "desired_diagnostic_target": desired_s4,
            "purpose": "local entry correction with downstream raw endpoint held as a diagnostic target",
        },
    }

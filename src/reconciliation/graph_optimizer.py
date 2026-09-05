"""Small damped Gauss-Newton/LM solver using right-local SE(2) updates."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Callable

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reconciliation.se2 import retract_pose
from reconciliation.se2_graph import GraphProblem, graph_residual_vector
from reconciliation.trajectory import validate_se2_trajectory


FloatArray = NDArray[np.float64]


class OptimizationError(RuntimeError):
    """Raised when a configured numerical solve cannot produce a finite step."""


@dataclass(frozen=True, slots=True)
class SolverConfig:
    max_iterations: int = 80
    finite_difference_epsilon: float = 1e-6
    initial_damping: float = 1e-3
    damping_increase: float = 10.0
    damping_decrease: float = 0.3
    maximum_damping: float = 1e12
    gradient_tolerance: float = 1e-9
    step_tolerance: float = 1e-9
    cost_tolerance: float = 1e-12

    def __post_init__(self) -> None:
        if not isinstance(self.max_iterations, int) or self.max_iterations < 1:
            raise ValueError("max_iterations must be a positive integer")
        for name in (
            "finite_difference_epsilon",
            "initial_damping",
            "damping_increase",
            "damping_decrease",
            "maximum_damping",
            "gradient_tolerance",
            "step_tolerance",
            "cost_tolerance",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
            object.__setattr__(self, name, value)
        if self.damping_increase <= 1.0:
            raise ValueError("damping_increase must exceed one")
        if not 0.0 < self.damping_decrease < 1.0:
            raise ValueError("damping_decrease must lie between zero and one")
        if self.maximum_damping < self.initial_damping:
            raise ValueError("maximum_damping must not be smaller than initial_damping")


@dataclass(frozen=True, slots=True)
class OptimizationResult:
    optimized: FloatArray
    initial_cost: float
    final_cost: float
    iterations: int
    converged: bool
    termination_reason: str
    cost_history: tuple[float, ...]
    damping_history: tuple[float, ...]

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result.pop("optimized")
        result["cost_history"] = list(self.cost_history)
        result["damping_history"] = list(self.damping_history)
        return result


def retract_trajectory(state: ArrayLike, delta: ArrayLike) -> FloatArray:
    trajectory = validate_se2_trajectory(state, name="state")
    update = np.asarray(delta, dtype=np.float64)
    if update.shape == (trajectory.size,):
        update = update.reshape(trajectory.shape)
    if update.shape != trajectory.shape or not np.all(np.isfinite(update)):
        raise ValueError("delta must be finite with state shape or flattened state size")
    return retract_pose(trajectory, update)


def numerical_jacobian(
    residual_function: Callable[[FloatArray], FloatArray],
    state: ArrayLike,
    epsilon: float,
) -> FloatArray:
    trajectory = validate_se2_trajectory(state, name="state")
    step = float(epsilon)
    if not math.isfinite(step) or step <= 0.0:
        raise ValueError("finite-difference epsilon must be finite and positive")
    base = np.asarray(residual_function(trajectory), dtype=np.float64)
    if base.ndim != 1 or not np.all(np.isfinite(base)):
        raise FloatingPointError("residual function must return a finite vector")
    jacobian = np.empty((base.size, trajectory.size), dtype=np.float64)
    for column in range(trajectory.size):
        delta = np.zeros(trajectory.size, dtype=np.float64)
        delta[column] = step
        plus = np.asarray(residual_function(retract_trajectory(trajectory, delta)), dtype=np.float64)
        minus = np.asarray(
            residual_function(retract_trajectory(trajectory, -delta)), dtype=np.float64
        )
        if plus.shape != base.shape or minus.shape != base.shape:
            raise ValueError("residual vector size changed during finite differencing")
        jacobian[:, column] = (plus - minus) / (2.0 * step)
    if not np.all(np.isfinite(jacobian)):
        raise FloatingPointError("numerical Jacobian contains NaN or Inf")
    return jacobian


def solve_least_squares(
    initial_state: ArrayLike,
    residual_function: Callable[[FloatArray], FloatArray],
    config: SolverConfig,
) -> OptimizationResult:
    """Solve a finite SE(2)-trajectory least-squares problem.

    This is the graph-independent numerical machinery.  Legacy EXP-02 keeps the
    ``solve_graph`` wrapper below, while EXP-02A supplies its own residual function.
    """

    state = validate_se2_trajectory(initial_state, name="initial_state")

    def cost_function(candidate: FloatArray) -> float:
        residual = np.asarray(residual_function(candidate), dtype=np.float64)
        if residual.ndim != 1 or not np.all(np.isfinite(residual)):
            raise FloatingPointError("residual function must return a finite vector")
        return float(residual @ residual)

    initial_cost = cost_function(state)
    cost = initial_cost
    costs = [cost]
    damping = config.initial_damping
    dampings: list[float] = []
    converged = False
    reason = "maximum_iterations"

    for iteration in range(1, config.max_iterations + 1):
        residual = np.asarray(residual_function(state), dtype=np.float64)
        jacobian = numerical_jacobian(
            residual_function,
            state,
            config.finite_difference_epsilon,
        )
        gradient = jacobian.T @ residual
        if not np.all(np.isfinite(gradient)):
            raise OptimizationError("normal-equation gradient is not finite")
        if float(np.linalg.norm(gradient, ord=np.inf)) <= config.gradient_tolerance:
            converged = True
            reason = "gradient_tolerance"
            break
        normal = jacobian.T @ jacobian
        dampings.append(damping)
        try:
            delta = np.linalg.solve(normal + damping * np.eye(normal.shape[0]), -gradient)
        except np.linalg.LinAlgError as error:
            raise OptimizationError("damped normal system is singular") from error
        if not np.all(np.isfinite(delta)):
            raise OptimizationError("solver produced a non-finite update")
        step_norm = float(np.linalg.norm(delta))
        if step_norm <= config.step_tolerance:
            converged = True
            reason = "step_tolerance"
            break
        candidate = retract_trajectory(state, delta)
        candidate_cost = cost_function(candidate)
        if candidate_cost < cost:
            decrease = cost - candidate_cost
            state = candidate
            cost = candidate_cost
            costs.append(cost)
            damping = max(np.finfo(np.float64).eps, damping * config.damping_decrease)
            if decrease <= config.cost_tolerance:
                converged = True
                reason = "cost_tolerance"
                break
        else:
            damping *= config.damping_increase
            if damping > config.maximum_damping:
                raise OptimizationError("damping exceeded maximum without a cost decrease")
    else:
        iteration = config.max_iterations

    optimized = validate_se2_trajectory(state, name="optimized").copy()
    if not np.all(np.isfinite(optimized)):
        raise OptimizationError("optimized state contains NaN or Inf")
    return OptimizationResult(
        optimized=optimized,
        initial_cost=initial_cost,
        final_cost=cost,
        iterations=iteration,
        converged=converged,
        termination_reason=reason,
        cost_history=tuple(costs),
        damping_history=tuple(dampings),
    )


def solve_graph(problem: GraphProblem, config: SolverConfig) -> OptimizationResult:
    """Backward-compatible solver entry point for the EXP-02 pilot graph."""

    return solve_least_squares(
        problem.raw_new,
        lambda candidate: graph_residual_vector(problem, candidate),
        config,
    )

"""Pure lookahead-direction formulation and diagnostics for EXP-02D.

EXP-02D is deliberately separate from :mod:`reconciliation.transition_graph` so
the historical EXP-02A/B/C M4 objective remains immutable.  The selected state
starts at raw FRESH index ``k``.  The new direction factor acts on state index
``q - k``, where ``q`` is the target returned by one frozen
``TrajectoryFollower.forward(B)`` call on the complete raw FRESH trajectory.

LightNav waypoint rows have no intrinsic timestamps.  The controller helpers in
this module are spatial probes at the exact saved boundary, not simulated robot
responses or time-aligned trajectory measurements.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Mapping

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reconciliation.controllers.trajectory_follower import FollowerConfig, TrajectoryFollower
from reconciliation.exp02c_factor_isolation import best_fit_left_se2
from reconciliation.graph_optimizer import OptimizationResult, SolverConfig, solve_least_squares
from reconciliation.se2 import relative_pose, se2_log, wrap_angle
from reconciliation.se2_graph import new_motion_residual
from reconciliation.spatial_entry import TransitionReconciliationInput
from reconciliation.trajectory import validate_pose_se2, validate_se2_trajectory
from reconciliation.transition_graph import (
    TransitionGraphProblem,
    TransitionResidualScales,
    entry_preservation_residual,
    physical_transition_residuals,
    problem_from_config as historical_problem_from_config,
    transition_residual_vector,
)


FloatArray = NDArray[np.float64]

METHODS = (
    "M0_RAW",
    "M1_HISTORICAL_M4",
    "M2_NO_DIRECTION",
    "M3_LOOKAHEAD",
)

# Residual ordering intentionally matches the historical M4 vector ordering.
METHOD_FACTORS: dict[str, tuple[str, ...]] = {
    "M0_RAW": (),
    "M1_HISTORICAL_M4": (
        "entry_preservation",
        "fresh_motion",
        "incoming_direction",
        "incoming_yaw",
    ),
    "M2_NO_DIRECTION": (
        "entry_preservation",
        "fresh_motion",
        "incoming_yaw",
    ),
    "M3_LOOKAHEAD": (
        "entry_preservation",
        "fresh_motion",
        "lookahead_direction",
        "incoming_yaw",
    ),
}

INCOMING_DIRECTION_UNDEFINED = "INCOMING_DIRECTION_UNDEFINED"
LOOKAHEAD_DIRECTION_UNDEFINED = "LOOKAHEAD_DIRECTION_UNDEFINED"
HISTORICAL_ENTRY_DIRECTION_UNDEFINED = "HISTORICAL_ENTRY_DIRECTION_UNDEFINED"
INPUT_RECONSTRUCTION_FAILED = "INPUT_RECONSTRUCTION_FAILED"


class GeometryUndefinedError(ValueError):
    """A named EXP-02D geometry exclusion without an invented epsilon direction."""

    def __init__(self, status: str, message: str) -> None:
        super().__init__(f"{status}: {message}")
        self.status = status


class InputReconstructionError(ValueError):
    """The reconstructed frozen follower command disagrees with saved evidence."""

    status = INPUT_RECONSTRUCTION_FAILED


def _finite_positive(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


@dataclass(frozen=True, slots=True)
class Exp02DWeights:
    """The four historical M4 weights, before per-method factor removal."""

    entry_preservation: float
    incoming_direction: float
    incoming_yaw: float
    fresh_motion: float

    def __post_init__(self) -> None:
        for name in (
            "entry_preservation",
            "incoming_direction",
            "incoming_yaw",
            "fresh_motion",
        ):
            object.__setattr__(self, name, _finite_positive(getattr(self, name), name))

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "Exp02DWeights":
        if not isinstance(values, Mapping):
            raise TypeError("weights must be a mapping")
        required = {
            "entry_preservation",
            "incoming_direction",
            "incoming_yaw",
            "fresh_motion",
        }
        missing = sorted(required.difference(values))
        if missing:
            raise ValueError(f"weights are missing: {', '.join(missing)}")
        if "pose_anchor" in values and float(values["pose_anchor"]) != 0.0:
            raise ValueError("historical M4 must not contain a pose-anchor weight")
        return cls(**{name: values[name] for name in required})


@dataclass(frozen=True, slots=True)
class LookaheadDirectionProblem:
    """Fixed EXP-02D inputs shared by the four predeclared methods.

    ``raw_target_index`` is the absolute raw FRESH index ``q``.  The entry index
    ``k`` comes only from ``inputs.entry_context``; no discrete index is changed
    during optimization.
    """

    inputs: TransitionReconciliationInput
    raw_target_index: int
    scales: TransitionResidualScales
    weights: Exp02DWeights
    minimum_translation_m: float

    def __post_init__(self) -> None:
        if not isinstance(self.inputs, TransitionReconciliationInput):
            raise TypeError("inputs must be a TransitionReconciliationInput")
        if not isinstance(self.raw_target_index, int) or isinstance(
            self.raw_target_index, bool
        ):
            raise ValueError("raw_target_index must be an integer")
        if not isinstance(self.scales, TransitionResidualScales):
            raise TypeError("scales must be TransitionResidualScales")
        if not isinstance(self.weights, Exp02DWeights):
            raise TypeError("weights must be Exp02DWeights")
        k = self.entry_index
        q = self.raw_target_index
        if q < k or q >= self.inputs.fresh_poses_world.shape[0]:
            raise ValueError("raw_target_index q must satisfy k <= q < FRESH length")
        object.__setattr__(
            self,
            "minimum_translation_m",
            _finite_positive(self.minimum_translation_m, "minimum_translation_m"),
        )

    @property
    def entry_index(self) -> int:
        return self.inputs.entry_context.entry_index

    @property
    def q_relative(self) -> int:
        return self.raw_target_index - self.entry_index

    @property
    def raw(self) -> FloatArray:
        return self.inputs.selected_suffix

    @property
    def raw_lookahead_pose(self) -> FloatArray:
        return self.inputs.fresh_poses_world[self.raw_target_index].copy()


def problem_from_config(
    inputs: TransitionReconciliationInput,
    raw_target_index: int,
    *,
    residual_scales: Mapping[str, Any],
    weights: Mapping[str, Any],
    minimum_translation_m: float,
) -> LookaheadDirectionProblem:
    """Construct an EXP-02D problem from the frozen historical M4 values."""

    return LookaheadDirectionProblem(
        inputs=inputs,
        raw_target_index=raw_target_index,
        scales=TransitionResidualScales(**dict(residual_scales)),
        weights=Exp02DWeights.from_mapping(weights),
        minimum_translation_m=minimum_translation_m,
    )


def _incoming_unit_direction(
    previous: ArrayLike,
    boundary: ArrayLike,
    *,
    minimum_translation_m: float,
) -> tuple[FloatArray, float]:
    before = validate_pose_se2(previous, name="previous")
    committed = validate_pose_se2(boundary, name="boundary")
    threshold = _finite_positive(minimum_translation_m, "minimum_translation_m")
    delta = committed[:2] - before[:2]
    distance = float(np.linalg.norm(delta))
    if distance <= threshold:
        raise GeometryUndefinedError(
            INCOMING_DIRECTION_UNDEFINED,
            f"||B-P||={distance:.17g} m is at or below {threshold:.17g} m",
        )
    return delta / distance, distance


def lookahead_direction_residual(
    previous: ArrayLike,
    boundary: ArrayLike,
    lookahead_candidate: ArrayLike,
    raw_lookahead: ArrayLike,
    *,
    minimum_translation_m: float,
) -> FloatArray:
    """Return ``(X_q.xy-B.xy)/d_q - u_in`` using fixed raw ``d_q``.

    The candidate vector is intentionally *not* normalized by its own length.
    Consequently the residual is finite even if a trial update places ``X_q``
    at, or arbitrarily close to, ``B``.
    """

    desired, _ = _incoming_unit_direction(
        previous, boundary, minimum_translation_m=minimum_translation_m
    )
    committed = validate_pose_se2(boundary, name="boundary")
    candidate = validate_pose_se2(lookahead_candidate, name="lookahead_candidate")
    raw = validate_pose_se2(raw_lookahead, name="raw_lookahead")
    threshold = _finite_positive(minimum_translation_m, "minimum_translation_m")
    raw_radius = float(np.linalg.norm(raw[:2] - committed[:2]))
    if raw_radius <= threshold:
        raise GeometryUndefinedError(
            LOOKAHEAD_DIRECTION_UNDEFINED,
            f"d_q={raw_radius:.17g} m is at or below {threshold:.17g} m",
        )
    residual = (candidate[:2] - committed[:2]) / raw_radius - desired
    if not np.all(np.isfinite(residual)):
        raise FloatingPointError("lookahead direction residual contains NaN or Inf")
    return np.asarray(residual, dtype=np.float64)


def incoming_yaw_residual(
    previous: ArrayLike, boundary: ArrayLike, entry_candidate: ArrayLike
) -> FloatArray:
    """Historical M4 yaw increment residual, independent of translation direction."""

    before = validate_pose_se2(previous, name="previous")
    committed = validate_pose_se2(boundary, name="boundary")
    entry = validate_pose_se2(entry_candidate, name="entry_candidate")
    incoming_yaw = float(relative_pose(before, committed)[2])
    transition_yaw = float(relative_pose(committed, entry)[2])
    return np.asarray([wrap_angle(transition_yaw - incoming_yaw)], dtype=np.float64)


def _fresh_motion_residuals(raw: FloatArray, state: FloatArray) -> FloatArray:
    return np.asarray(
        [
            new_motion_residual(raw[index], raw[index + 1], state[index], state[index + 1])
            for index in range(raw.shape[0] - 1)
        ],
        dtype=np.float64,
    ).reshape((-1, 3))


def _historical_problem(problem: LookaheadDirectionProblem) -> TransitionGraphProblem:
    # Give the experiment runner named geometry failures before delegating every
    # valid M1 residual and solve to the frozen production implementation.
    _incoming_unit_direction(
        problem.inputs.incoming_previous_pose,
        problem.inputs.committed_pose_world,
        minimum_translation_m=problem.minimum_translation_m,
    )
    entry_radius = float(
        np.linalg.norm(problem.raw[0, :2] - problem.inputs.committed_pose_world[:2])
    )
    if entry_radius <= problem.minimum_translation_m:
        raise GeometryUndefinedError(
            HISTORICAL_ENTRY_DIRECTION_UNDEFINED,
            f"d_k={entry_radius:.17g} m is at or below "
            f"{problem.minimum_translation_m:.17g} m",
        )
    return historical_problem_from_config(
        problem.inputs,
        "incoming_motion_aware",
        residual_scales=asdict(problem.scales),
        weights={
            "pose_anchor": 0.0,
            "entry_preservation": problem.weights.entry_preservation,
            "incoming_direction": problem.weights.incoming_direction,
            "incoming_yaw": problem.weights.incoming_yaw,
            "fresh_motion": problem.weights.fresh_motion,
        },
        minimum_translation_m=problem.minimum_translation_m,
    )


def physical_method_residuals(
    problem: LookaheadDirectionProblem,
    state: ArrayLike,
    method: str,
) -> dict[str, FloatArray]:
    """Return only the physical residual groups active in ``method``."""

    if method not in METHODS:
        raise ValueError(f"unknown EXP-02D method: {method!r}")
    trajectory = validate_se2_trajectory(state, name="state")
    raw = problem.raw
    if trajectory.shape != raw.shape:
        raise ValueError("state shape must equal selected raw FRESH suffix shape")
    if method == "M0_RAW":
        return {}
    if method == "M1_HISTORICAL_M4":
        historical = _historical_problem(problem)
        groups = physical_transition_residuals(historical, trajectory)
        return {name: groups[name] for name in METHOD_FACTORS[method]}

    groups: dict[str, FloatArray] = {
        "entry_preservation": entry_preservation_residual(raw[0], trajectory[0])[None, :],
        "fresh_motion": _fresh_motion_residuals(raw, trajectory),
        "incoming_yaw": incoming_yaw_residual(
            problem.inputs.incoming_previous_pose,
            problem.inputs.committed_pose_world,
            trajectory[0],
        ),
    }
    if method == "M3_LOOKAHEAD":
        groups["lookahead_direction"] = lookahead_direction_residual(
            problem.inputs.incoming_previous_pose,
            problem.inputs.committed_pose_world,
            trajectory[problem.q_relative],
            problem.raw_lookahead_pose,
            minimum_translation_m=problem.minimum_translation_m,
        )
    return {name: groups[name] for name in METHOD_FACTORS[method]}


def method_residual_vector(
    problem: LookaheadDirectionProblem, state: ArrayLike, method: str
) -> FloatArray:
    """Return the frozen-scale least-squares vector for M1, M2, or M3."""

    if method == "M0_RAW":
        raise ValueError("M0_RAW has no optimization residual")
    if method not in METHODS:
        raise ValueError(f"unknown EXP-02D method: {method!r}")
    if method == "M1_HISTORICAL_M4":
        return transition_residual_vector(_historical_problem(problem), state)

    groups = physical_method_residuals(problem, state, method)
    pose_scale = problem.scales.pose_vector
    blocks: list[FloatArray] = []
    for name in METHOD_FACTORS[method]:
        if name in ("entry_preservation", "fresh_motion"):
            scale: FloatArray | float = pose_scale
        elif name == "lookahead_direction":
            scale = problem.scales.direction_unitless
        else:
            scale = problem.scales.yaw_rad
        weight_name = (
            "incoming_direction" if name == "lookahead_direction" else name
        )
        weight = float(getattr(problem.weights, weight_name))
        blocks.append((math.sqrt(weight) * groups[name] / scale).reshape(-1))
    result = np.concatenate(blocks)
    if not np.all(np.isfinite(result)):
        raise FloatingPointError("EXP-02D residual contains NaN or Inf")
    return result


@dataclass(frozen=True, slots=True)
class MethodResult:
    method: str
    candidate: FloatArray
    optimization: OptimizationResult | None

    def __post_init__(self) -> None:
        if self.method not in METHODS:
            raise ValueError(f"unknown EXP-02D method: {self.method!r}")
        value = validate_se2_trajectory(self.candidate, name="candidate")
        value.setflags(write=False)
        object.__setattr__(self, "candidate", value)
        if self.method == "M0_RAW" and self.optimization is not None:
            raise ValueError("M0_RAW must not have an optimization result")
        if self.method != "M0_RAW" and not isinstance(
            self.optimization, OptimizationResult
        ):
            raise TypeError("optimized methods require OptimizationResult")

    def optimization_dict(self) -> dict[str, Any] | None:
        return None if self.optimization is None else self.optimization.to_dict()


def solve_method(
    problem: LookaheadDirectionProblem,
    method: str,
    solver: SolverConfig,
) -> MethodResult:
    """Build one candidate with exactly the frozen solver and method definition."""

    if method not in METHODS:
        raise ValueError(f"unknown EXP-02D method: {method!r}")
    raw = problem.raw
    if method == "M0_RAW":
        return MethodResult(method, raw, None)
    if not isinstance(solver, SolverConfig):
        raise TypeError("solver must be SolverConfig")

    if method == "M1_HISTORICAL_M4":
        historical = _historical_problem(problem)
        optimization = solve_least_squares(
            historical.raw_new,
            lambda candidate: transition_residual_vector(historical, candidate),
            solver,
        )
    else:
        optimization = solve_least_squares(
            raw,
            lambda candidate: method_residual_vector(problem, candidate, method),
            solver,
        )
    return MethodResult(method, optimization.optimized, optimization)


def undefined_geometry_statuses(problem: LookaheadDirectionProblem) -> tuple[str, ...]:
    """Return predeclared EXP-02D direction exclusions without mutating inputs."""

    statuses: list[str] = []
    previous = problem.inputs.incoming_previous_pose
    boundary = problem.inputs.committed_pose_world
    if float(np.linalg.norm(boundary[:2] - previous[:2])) <= problem.minimum_translation_m:
        statuses.append(INCOMING_DIRECTION_UNDEFINED)
    if (
        float(np.linalg.norm(problem.raw_lookahead_pose[:2] - boundary[:2]))
        <= problem.minimum_translation_m
    ):
        statuses.append(LOOKAHEAD_DIRECTION_UNDEFINED)
    return tuple(statuses)


def transition_geometry_metrics(
    problem: LookaheadDirectionProblem,
    *,
    observation_pose: ArrayLike | None = None,
) -> dict[str, Any]:
    """Compute EXP-02D geometry features with undefined angles represented by ``None``."""

    previous = problem.inputs.incoming_previous_pose
    boundary = problem.inputs.committed_pose_world
    raw = problem.raw
    raw_entry = raw[0]
    raw_lookahead = problem.raw_lookahead_pose
    threshold = problem.minimum_translation_m

    incoming_delta = boundary[:2] - previous[:2]
    entry_delta = raw_entry[:2] - boundary[:2]
    lookahead_delta = raw_lookahead[:2] - boundary[:2]
    incoming_distance = float(np.linalg.norm(incoming_delta))
    entry_distance = float(np.linalg.norm(entry_delta))
    lookahead_distance = float(np.linalg.norm(lookahead_delta))

    theta_in = (
        math.atan2(float(incoming_delta[1]), float(incoming_delta[0]))
        if incoming_distance > threshold
        else None
    )
    theta_entry = (
        math.atan2(float(entry_delta[1]), float(entry_delta[0]))
        if entry_distance > threshold
        else None
    )
    theta_look = (
        math.atan2(float(lookahead_delta[1]), float(lookahead_delta[0]))
        if lookahead_distance > threshold
        else None
    )

    def disagreement(left: float | None, right: float | None) -> float | None:
        return None if left is None or right is None else abs(float(wrap_angle(left - right)))

    selected_edges = np.linalg.norm(np.diff(raw[:, :2], axis=0), axis=1)
    q_edges = np.linalg.norm(
        np.diff(
            problem.inputs.fresh_poses_world[
                problem.entry_index : problem.raw_target_index + 1, :2
            ],
            axis=0,
        ),
        axis=1,
    )
    fresh_yaw_steps = np.asarray(wrap_angle(np.diff(raw[:, 2])), dtype=np.float64)

    lateral_excursion: float | None
    if theta_in is None:
        lateral_excursion = None
    else:
        normal = np.asarray([-math.sin(theta_in), math.cos(theta_in)])
        lateral = (raw[:, :2] - boundary[:2]) @ normal
        lateral_excursion = float(np.max(np.abs(lateral)))

    observation: dict[str, float] | None = None
    if observation_pose is not None:
        obs = validate_pose_se2(observation_pose, name="observation_pose")
        observation = {
            "translation_m": float(np.linalg.norm(boundary[:2] - obs[:2])),
            "yaw_rad": float(relative_pose(obs, boundary)[2]),
        }

    return {
        "entry_index_k": problem.entry_index,
        "target_index_q": problem.raw_target_index,
        "q_minus_k": problem.q_relative,
        "incoming_direction_world_rad": theta_in,
        "entry_direction_world_rad": theta_entry,
        "lookahead_direction_world_rad": theta_look,
        "alpha_entry_rad": disagreement(theta_entry, theta_in),
        "alpha_look_rad": disagreement(theta_look, theta_in),
        "alpha_entry_minus_alpha_look_rad": (
            None
            if disagreement(theta_entry, theta_in) is None
            or disagreement(theta_look, theta_in) is None
            else disagreement(theta_entry, theta_in) - disagreement(theta_look, theta_in)
        ),
        "d_k_m": entry_distance,
        "d_q_m": lookahead_distance,
        "boundary_to_entry_gap_m": entry_distance,
        "boundary_to_lookahead_gap_m": lookahead_distance,
        "entry_to_lookahead_arc_length_m": float(np.sum(q_edges)),
        "selected_fresh_arc_length_m": float(np.sum(selected_edges)),
        "previous_to_boundary_translation_m": incoming_distance,
        "previous_to_boundary_yaw_rad": float(relative_pose(previous, boundary)[2]),
        "observation_to_boundary": observation,
        "fresh_net_yaw_rad": float(np.sum(fresh_yaw_steps)),
        "fresh_lateral_excursion_m": lateral_excursion,
        "undefined_geometry_statuses": list(undefined_geometry_statuses(problem)),
        "semantics": {
            "waypoint_time_base": None,
            "entry_and_lookahead": "raw observation-anchored FRESH world poses",
            "incoming": "saved actual control pose P immediately before saved boundary B",
        },
    }


def candidate_deformation_metrics(
    problem: LookaheadDirectionProblem, candidate: ArrayLike
) -> dict[str, Any]:
    """Measure candidate deformation from raw FRESH without judging navigation intent."""

    raw = problem.raw
    output = validate_se2_trajectory(candidate, name="candidate")
    if output.shape != raw.shape:
        raise ValueError("candidate shape must equal selected raw FRESH suffix shape")
    correction = se2_log(relative_pose(raw, output))
    translation = np.linalg.norm(correction[:, :2], axis=1)
    yaw = np.abs(correction[:, 2])
    edge = _fresh_motion_residuals(raw, output)
    edge_translation = np.linalg.norm(edge[:, :2], axis=1)
    edge_yaw = np.abs(edge[:, 2])
    rigid = best_fit_left_se2(raw, output)

    def node(index: int) -> dict[str, Any]:
        return {
            "selected_suffix_index": index,
            "raw_fresh_index": problem.entry_index + index,
            "se2_log_raw_inverse_candidate": correction[index].tolist(),
            "translation_m": float(translation[index]),
            "abs_yaw_rad": float(yaw[index]),
        }

    return {
        "entry_displacement_from_raw": node(0),
        "q_node_displacement_from_raw": node(problem.q_relative),
        "endpoint_displacement_from_raw": node(raw.shape[0] - 1),
        "translation_deformation_rms_m": float(np.sqrt(np.mean(translation**2))),
        "yaw_deformation_rms_rad": float(np.sqrt(np.mean(yaw**2))),
        "per_node_translation_deformation_m": translation.tolist(),
        "per_node_abs_yaw_deformation_rad": yaw.tolist(),
        "fresh_relative_motion": {
            "translation_rms_m": float(np.sqrt(np.mean(edge_translation**2))),
            "translation_max_m": float(np.max(edge_translation)),
            "yaw_rms_rad": float(np.sqrt(np.mean(edge_yaw**2))),
            "yaw_max_rad": float(np.max(edge_yaw)),
            "per_edge_se2_log": edge.tolist(),
        },
        "best_fit_single_rigid_transform": {
            "translation_rms_m": rigid["translation_rms_m"],
            "translation_max_m": rigid["translation_max_m"],
            "yaw_rms_rad": rigid["yaw_rms_rad"],
            "yaw_max_rad": rigid["yaw_max_rad"],
            "best_fit_transform_se2": rigid["best_fit_transform_se2"],
            "fit_convention": rigid["fit_convention"],
        },
    }


def follower_config_from_mapping(values: Mapping[str, Any]) -> FollowerConfig:
    if not isinstance(values, Mapping):
        raise TypeError("follower values must be a mapping")
    required = tuple(FollowerConfig.__dataclass_fields__)
    missing = [name for name in required if name not in values]
    if missing:
        raise ValueError(f"follower values are missing: {', '.join(missing)}")
    return FollowerConfig(**{name: values[name] for name in required})


def reconstruct_raw_follower_decision(
    fresh_world: ArrayLike,
    *,
    boundary: ArrayLike,
    follower_values: Mapping[str, Any],
) -> dict[str, Any]:
    """Perform exactly one frozen follower call on complete raw FRESH at ``B``."""

    raw = validate_se2_trajectory(fresh_world, name="fresh_world")
    committed = validate_pose_se2(boundary, name="boundary")
    config = follower_config_from_mapping(follower_values)
    command = TrajectoryFollower(raw, config).forward(committed)
    if not (0 <= command.nearest_index <= command.target_index < raw.shape[0]):
        raise RuntimeError("frozen follower returned invalid k/q indices")
    return {
        "k_fresh": command.nearest_index,
        "q_fresh": command.target_index,
        "q_minus_k": command.target_index - command.nearest_index,
        "first_command_v_omega": [
            command.linear_velocity_mps,
            command.angular_velocity_rps,
        ],
        "goal_reached": command.goal_reached,
        "lookahead_distance_m": config.lookahead_distance_m,
        "call_semantics": "exactly one fresh TrajectoryFollower.forward(B) call",
    }


def validate_reconstructed_first_command(
    reconstructed_command: ArrayLike,
    saved_command: ArrayLike,
    *,
    tolerance: float,
) -> dict[str, Any]:
    """Validate the reconstructed raw first desired command against saved data."""

    reconstructed = np.asarray(reconstructed_command, dtype=np.float64)
    saved = np.asarray(saved_command, dtype=np.float64)
    if reconstructed.shape != (2,) or saved.shape != (2,):
        raise ValueError("reconstructed and saved commands must have shape (2,)")
    if not np.all(np.isfinite(reconstructed)) or not np.all(np.isfinite(saved)):
        raise ValueError("reconstructed and saved commands must be finite")
    limit = _finite_positive(tolerance, "tolerance")
    error = np.abs(reconstructed - saved)
    payload = {
        "reconstructed_v_omega": reconstructed.tolist(),
        "saved_v_omega": saved.tolist(),
        "absolute_error_v_omega": error.tolist(),
        "maximum_absolute_error": float(np.max(error)),
        "tolerance": limit,
        "consistent": bool(np.all(error <= limit)),
    }
    if not payload["consistent"]:
        raise InputReconstructionError(
            f"{INPUT_RECONSTRUCTION_FAILED}: maximum first-command error "
            f"{payload['maximum_absolute_error']:.17g} exceeds {limit:.17g}"
        )
    return payload


def candidate_command_metrics(
    candidate: ArrayLike,
    *,
    boundary: ArrayLike,
    old_last_command: ArrayLike,
    follower_values: Mapping[str, Any],
    original_entry_index: int,
    raw_target_index: int,
    linear_normalization_mps: float = 0.15,
    angular_normalization_rps: float = 0.30,
) -> dict[str, Any]:
    """Evaluate immediate desired-command continuity and post-hoc follower indices."""

    trajectory = validate_se2_trajectory(candidate, name="candidate")
    committed = validate_pose_se2(boundary, name="boundary")
    old = np.asarray(old_last_command, dtype=np.float64)
    if old.shape != (2,) or not np.all(np.isfinite(old)):
        raise ValueError("old_last_command must be finite shape (2,)")
    if not isinstance(original_entry_index, int) or original_entry_index < 0:
        raise ValueError("original_entry_index must be a nonnegative integer")
    if not isinstance(raw_target_index, int) or raw_target_index < original_entry_index:
        raise ValueError("raw_target_index must be an integer at or after original_entry_index")
    linear_scale = _finite_positive(linear_normalization_mps, "linear_normalization_mps")
    angular_scale = _finite_positive(angular_normalization_rps, "angular_normalization_rps")
    command = TrajectoryFollower(
        trajectory, follower_config_from_mapping(follower_values)
    ).forward(committed)
    desired = np.asarray(
        [command.linear_velocity_mps, command.angular_velocity_rps], dtype=np.float64
    )
    delta = desired - old
    candidate_nearest_absolute = original_entry_index + command.nearest_index
    candidate_target_absolute = original_entry_index + command.target_index
    return {
        "old_last_desired_v_omega": old.tolist(),
        "candidate_first_desired_v_omega": desired.tolist(),
        "delta_v_signed_mps": float(delta[0]),
        "delta_omega_signed_rps": float(delta[1]),
        "delta_v_abs_mps": abs(float(delta[0])),
        "delta_omega_abs_rps": abs(float(delta[1])),
        "J_cmd": float(
            math.hypot(abs(float(delta[0])) / linear_scale, abs(float(delta[1])) / angular_scale)
        ),
        "normalization": {
            "linear_mps": linear_scale,
            "angular_rps": angular_scale,
            "evaluation_only_not_objective": True,
        },
        "candidate_nearest_index_relative": command.nearest_index,
        "candidate_target_index_relative": command.target_index,
        "candidate_nearest_index": candidate_nearest_absolute,
        "candidate_target_index": candidate_target_absolute,
        "raw_target_index": raw_target_index,
        "candidate_target_changed_from_raw_q": candidate_target_absolute != raw_target_index,
        "goal_reached": command.goal_reached,
        "semantics": (
            "one frozen TrajectoryFollower spatial probe at exact B; no waypoint timestamps, "
            "physics execution, or optimization feedback"
        ),
    }

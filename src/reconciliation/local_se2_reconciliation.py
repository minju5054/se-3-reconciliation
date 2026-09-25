"""Spatial three-factor SE(2) reconciliation; no clock, GP or correspondence.

All states are world poses. Returned local coordinates use the original A frame.
The only transport is B A^-1 applied to a separate target, never to raw FRESH.
"""
from dataclasses import dataclass
import numpy as np
from reconciliation.se2 import compose_poses, inverse_pose, relative_pose, se2_log, wrap_angle
from reconciliation.trajectory import validate_pose_se2, validate_se2_trajectory


@dataclass(frozen=True)
class LocalSE2Problem:
    A: np.ndarray
    B: np.ndarray
    fresh: np.ndarray
    sigma_translation_m: float = .10
    sigma_yaw_rad: float = np.deg2rad(10.)

    def __post_init__(self):
        for name in ('A', 'B'):
            value = validate_pose_se2(getattr(self, name), name=name)
            value.setflags(write=False)
            object.__setattr__(self, name, value)
        fresh = validate_se2_trajectory(self.fresh, name='original FRESH')
        if len(fresh) < 2:
            raise ValueError('at least two original FRESH poses required')
        if np.linalg.norm(np.diff(fresh[:, :2], axis=0), axis=1).sum() <= 0:
            raise ValueError('positive total original XY arc required')
        for scale in (self.sigma_translation_m, self.sigma_yaw_rad):
            if not np.isfinite(scale) or scale <= 0:
                raise ValueError('normalization scales must be finite and positive')
        fresh.setflags(write=False)
        object.__setattr__(self, 'fresh', fresh)

    @property
    def scales(self):
        return np.array([self.sigma_translation_m]*2 + [self.sigma_yaw_rad])

    @property
    def arc(self):
        return np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.fresh[:, :2], axis=0), axis=1))]

    @property
    def progress(self):
        return self.arc / self.arc[-1]

    @property
    def transport(self):
        return compose_poses(self.B, inverse_pose(self.A))

    @property
    def target(self):
        return compose_poses(self.transport, self.fresh)

    def raw_residuals(self, state):
        x = validate_se2_trajectory(state, name='X')
        if x.shape != self.fresh.shape:
            raise ValueError('X must retain all original FRESH rows')
        return dict(
            L=se2_log(relative_pose(self.target, x)),
            R=se2_log(relative_pose(relative_pose(self.fresh[:-1], self.fresh[1:]),
                                    relative_pose(x[:-1], x[1:]))),
            A=se2_log(relative_pose(self.fresh, x)))

    def residual_blocks(self, state):
        raw = self.raw_residuals(state)
        s = self.progress
        weights = dict(L=(1-s)**2, R=np.ones(len(s)-1), A=s**2)
        return {k: raw[k] / self.scales * np.sqrt(w/w.sum())[:, None]
                for k, w in weights.items()}

    def residual_vector(self, state):
        return np.concatenate([r.ravel() for r in self.residual_blocks(state).values()])

    def costs(self, state):
        blocks = self.residual_blocks(state)
        out = {k: float(np.sum(r*r)) for k, r in blocks.items()}
        out['total'] = sum(out.values())
        return out

    def original_observation_local(self, state):
        return relative_pose(self.A, validate_se2_trajectory(state))


def rigid_fit_diagnostic(fresh, optimized):
    """Closed-form XY Procrustes left transform; yaw residual assessed separately.

    Fits XY least squares only, without iterative optimization or baseline rollout.
    The zero-covariance tie chooses zero rotation deterministically.
    """
    f = validate_se2_trajectory(fresh)
    x = validate_se2_trajectory(optimized)
    if f.shape != x.shape or len(f) < 2:
        raise ValueError('matching N>=2 trajectories required')
    a, b = f[:, :2]-f[:, :2].mean(0), x[:, :2]-x[:, :2].mean(0)
    dot = float(np.sum(a*b))
    cross = float(np.sum(a[:, 0]*b[:, 1]-a[:, 1]*b[:, 0]))
    yaw = float(np.arctan2(cross, dot)) if dot or cross else 0.
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    translation = x[:, :2].mean(0) - rotation @ f[:, :2].mean(0)
    transform = np.r_[translation, yaw]
    fitted = compose_poses(transform, f)
    return dict(definition='closed-form minimum XY squared-error single left SE(2) transform; yaw assessed separately',
                transform=transform.tolist(),
                translation_RMS_m=float(np.sqrt(np.mean(np.sum((fitted[:, :2]-x[:, :2])**2, axis=1)))),
                yaw_RMS_rad=float(np.sqrt(np.mean(wrap_angle(fitted[:, 2]-x[:, 2])**2))))


def planning_diagnostics(problem, state):
    residuals = problem.raw_residuals(state)
    nodes = []
    for j, s in enumerate(problem.progress):
        nodes.append(dict(j=j, s=float(s), w_L=float((1-s)**2), w_A=float(s*s),
                          correction_log=residuals['A'][j].tolist(),
                          correction_translation_m=float(np.linalg.norm(residuals['A'][j, :2])),
                          correction_yaw_rad=float(abs(residuals['A'][j, 2])),
                          target_log=residuals['L'][j].tolist(),
                          target_translation_m=float(np.linalg.norm(residuals['L'][j, :2])),
                          target_yaw_rad=float(abs(residuals['L'][j, 2])),
                          world_XY_displacement_m=float(np.linalg.norm(state[j, :2]-problem.fresh[j, :2]))))
    edge_t = np.linalg.norm(residuals['R'][:, :2], axis=1)
    edge_y = abs(residuals['R'][:, 2])
    return dict(nodes=nodes, relative_edges=residuals['R'].tolist(),
                relative_translation_RMS_m=float(np.sqrt(np.mean(edge_t**2))),
                relative_yaw_RMS_rad=float(np.sqrt(np.mean(edge_y**2))),
                relative_translation_max_m=float(edge_t.max()), relative_yaw_max_rad=float(edge_y.max()),
                rigid_fit=rigid_fit_diagnostic(problem.fresh, state))

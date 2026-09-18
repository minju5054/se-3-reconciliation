"""Locally linear, white-noise-on-local-acceleration SE(2) Gaussian process.

World poses use metres/radians and ``T_world_agent``. Perturbations are right
local; body twist is ``vee(T^-1 dT/dt)``. Dong et al., ICRA 2018, equations
17--28 define the prior used here (https://dongjing3309.github.io/files/Dong18icra.pdf).
The body derivative identity in equation 22 requires ``nu = J_r(xi) xi_dot``.
The inverse printed in that paper's interpolation equation 33 is inconsistent
with equation 22; this implementation follows the derivative identity and tests
it directly. The residual sign is the negative of equation 27, leaving its cost
unchanged. Conditional interpolation is the cubic Hermite mean of the local
integrated-Wiener process, not a second-difference substitute for the GP prior.

The model is locally linear, not an exact global body-acceleration or jerk model.
Each factor/interpolation depends only on its two neighbouring support states.
"""
from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike

from .se2 import compose_poses, relative_pose, retract_pose, se2_exp, se2_log


def _vectors(value: ArrayLike, name: str) -> np.ndarray:
    a = np.asarray(value, dtype=float)
    if a.shape[-1:] != (3,) or not np.all(np.isfinite(a)):
        raise ValueError(f"{name} must be finite (..., 3)")
    return a


def adjoint_algebra(xi: ArrayLike) -> np.ndarray:
    """Lie bracket matrix: ``ad_xi eta = vee([hat(xi), hat(eta)])``."""
    a = _vectors(xi, "xi")
    out = np.zeros(a.shape[:-1] + (3, 3))
    out[..., 0, 1] = -a[..., 2]
    out[..., 1, 0] = a[..., 2]
    out[..., 0, 2] = a[..., 1]
    out[..., 1, 2] = -a[..., 0]
    return out


def _coefficients(w: np.ndarray) -> tuple[np.ndarray, ...]:
    # J_r = I - a ad + b ad^2; Taylor branches avoid cancellation near zero.
    small = np.abs(w) < 1e-3
    safe = np.where(small, 1.0, w)
    w2 = w * w
    a = np.where(small, .5-w2/24+w2*w2/720-w2**3/40320,
                 (1-np.cos(safe))/safe**2)
    b = np.where(small, 1/6-w2/120+w2*w2/5040-w2**3/362880,
                 (safe-np.sin(safe))/safe**3)
    da = np.where(small, -w/12+w*w2/180-w*w2*w2/6720,
                  (safe*np.sin(safe)-2*(1-np.cos(safe)))/safe**3)
    db = np.where(small, -w/60+w*w2/1260-w*w2*w2/60480,
                  (safe*(1-np.cos(safe))-3*(safe-np.sin(safe)))/safe**4)
    return a, b, da, db


def right_jacobian(xi: ArrayLike) -> np.ndarray:
    """J satisfying ``Log(Exp(xi)^-1 Exp(xi+eps*d))/eps -> J_r(xi)d``."""
    x = _vectors(xi, "xi")
    ad = adjoint_algebra(x)
    a, b, _, _ = _coefficients(x[..., 2])
    return np.eye(3) - a[..., None, None]*ad + b[..., None, None]*(ad @ ad)


def right_jacobian_inverse_apply(xi: ArrayLike, body_twist: ArrayLike) -> np.ndarray:
    """Convert body twist to the derivative of right-local logarithm coordinates."""
    j = right_jacobian(xi)
    v = _vectors(body_twist, "body_twist")
    return np.linalg.solve(j, v[..., None])[..., 0]


def right_jacobian_directional(xi: ArrayLike, direction: ArrayLike) -> np.ndarray:
    """Directional derivative ``d J_r(xi + eps*direction)/d eps``."""
    x, d = np.broadcast_arrays(_vectors(xi, "xi"), _vectors(direction, "direction"))
    ad, dad = adjoint_algebra(x), adjoint_algebra(d)
    a, b, da, db = _coefficients(x[..., 2])
    return (-da[..., None, None]*d[..., 2, None, None]*ad
            -a[..., None, None]*dad
            +db[..., None, None]*d[..., 2, None, None]*(ad@ad)
            +b[..., None, None]*(dad@ad+ad@dad))


def process_covariance(h: float, qc: ArrayLike = (1.0, 1.0, 1.0)) -> np.ndarray:
    """Integrated local-acceleration covariance, ordered [xi, xi_dot]."""
    if not np.isfinite(h) or h <= 0:
        raise ValueError("h must be finite and positive")
    q = np.asarray(qc, dtype=float)
    if q.shape == (3,):
        q = np.diag(q)
    if q.shape != (3, 3) or not np.all(np.isfinite(q)) or not np.allclose(q, q.T):
        raise ValueError("qc must be a symmetric positive definite 3x3 matrix or diagonal")
    np.linalg.cholesky(q)
    return np.block([[h**3/3*q, h**2/2*q], [h**2/2*q, h*q]])


def gp_residual(pose0: ArrayLike, twist0: ArrayLike, pose1: ArrayLike,
                twist1: ArrayLike, h: float) -> np.ndarray:
    """Unwhitened residual [xi-h*nu0, J_r(xi)^-1*nu1-nu0]."""
    if not np.isfinite(h) or h <= 0:
        raise ValueError("h must be finite and positive")
    xi = se2_log(relative_pose(pose0, pose1))
    v0 = _vectors(twist0, "twist0")
    d1 = right_jacobian_inverse_apply(xi, twist1)
    return np.concatenate((xi-h*v0, d1-v0), axis=-1)


def whiten_residual(residual: ArrayLike, h: float,
                    qc: ArrayLike = (1., 1., 1.)) -> np.ndarray:
    """Whiten with Cholesky solves; never form a covariance inverse."""
    r = np.asarray(residual, dtype=float)
    if r.shape[-1:] != (6,) or not np.all(np.isfinite(r)):
        raise ValueError("residual must be finite (..., 6)")
    chol = np.linalg.cholesky(process_covariance(h, qc))
    return np.linalg.solve(chol, r.reshape(-1, 6).T).T.reshape(r.shape)


def interpolate_interval(pose0: ArrayLike, twist0: ArrayLike, pose1: ArrayLike,
                         twist1: ArrayLike, h: float, fraction: ArrayLike
                         ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Conditional GP mean pose, body twist and body acceleration.

    ``fraction`` in [0,1] may broadcast with the pose batch. Acceleration is the
    analytic derivative of body twist, including the derivative of J_r. At knots
    pose/twist agree from both sides; acceleration may have a jump.
    """
    if not np.isfinite(h) or h <= 0:
        raise ValueError("h must be finite and positive")
    u = np.asarray(fraction, dtype=float)
    if not np.all(np.isfinite(u)) or np.any((u < -1e-12) | (u > 1+1e-12)):
        raise ValueError("fraction must lie in [0, 1]")
    u = np.clip(u, 0, 1)[..., None]
    x1 = se2_log(relative_pose(pose0, pose1))
    d0 = _vectors(twist0, "twist0")
    d1 = right_jacobian_inverse_apply(x1, twist1)
    xi = (u**3-2*u**2+u)*h*d0 + (-2*u**3+3*u**2)*x1 + (u**3-u**2)*h*d1
    dxi = (3*u**2-4*u+1)*d0 + (-6*u**2+6*u)*x1/h + (3*u**2-2*u)*d1
    ddxi = (6*u-4)*d0/h + (-12*u+6)*x1/h**2 + (6*u-2)*d1/h
    jr = right_jacobian(xi)
    velocity = (jr @ dxi[..., None])[..., 0]
    acceleration = ((right_jacobian_directional(xi, dxi) @ dxi[..., None])
                    + jr @ ddxi[..., None])[..., 0]
    return compose_poses(pose0, se2_exp(xi)), velocity, acceleration


def sample_gp(times: ArrayLike, poses: ArrayLike, twists: ArrayLike,
              query_times: ArrayLike) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Sample a uniform support grid; interior knot acceleration uses its right side."""
    t, q = np.asarray(times, float), np.asarray(query_times, float)
    p, v = _vectors(poses, "poses"), _vectors(twists, "twists")
    if t.ndim != 1 or len(t) < 2 or p.shape != (len(t), 3) or v.shape != p.shape:
        raise ValueError("support arrays must have consistent lengths")
    steps = np.diff(t)
    if np.any(steps <= 0) or not np.allclose(steps, steps[0], atol=1e-12):
        raise ValueError("support times must be a strictly increasing uniform grid")
    if not np.all(np.isfinite(q)) or np.any(q < t[0]-1e-12) or np.any(q > t[-1]+1e-12):
        raise ValueError("query outside support domain")
    indices = np.clip(np.searchsorted(t, q, side="right")-1, 0, len(t)-2)
    return interpolate_interval(p[indices], v[indices], p[indices+1], v[indices+1],
                                float(steps[0]), (q-t[indices])/steps[0])


def gp_factor_jacobian(pose0: ArrayLike, twist0: ArrayLike, pose1: ArrayLike,
                       twist1: ArrayLike, h: float, step: float = 1e-6) -> np.ndarray:
    """Local central-difference factor Jacobian in [delta0, dnu0, delta1, dnu1].

    Pose columns use the repository right retraction. This diagnostic Jacobian
    is independent of the constrained solver's forward-difference chart Jacobian.
    """
    p0, v0, p1, v1 = map(lambda x: np.asarray(x, float), (pose0, twist0, pose1, twist1))
    result = np.empty((6, 12))
    for j in range(12):
        e = np.zeros(3); e[j % 3] = step
        if j < 3:
            plus = gp_residual(retract_pose(p0,e),v0,p1,v1,h)
            minus = gp_residual(retract_pose(p0,-e),v0,p1,v1,h)
        elif j < 6:
            plus = gp_residual(p0,v0+e,p1,v1,h); minus = gp_residual(p0,v0-e,p1,v1,h)
        elif j < 9:
            plus = gp_residual(p0,v0,retract_pose(p1,e),v1,h)
            minus = gp_residual(p0,v0,retract_pose(p1,-e),v1,h)
        else:
            plus = gp_residual(p0,v0,p1,v1+e,h); minus = gp_residual(p0,v0,p1,v1-e,h)
        result[:, j] = (plus-minus)/(2*step)
    return result


def prior_dependency_pattern(support_count: int) -> np.ndarray:
    """Structural boolean Jacobian pattern, six state coordinates per support."""
    if support_count < 2:
        raise ValueError("at least two supports are required")
    pattern = np.zeros((6*(support_count-1), 6*support_count), dtype=bool)
    for i in range(support_count-1):
        pattern[6*i:6*(i+1),6*i:6*(i+2)] = True
    return pattern

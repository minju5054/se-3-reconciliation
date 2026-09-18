"""The single B variant: one fixed same-direction deceleration initialization.

This constructs a seed, not an accepted solution. Original goal, environment,
motion and independent dense checks still determine whether it is feasible.
No RAW rollout, fitted parameter, reanchoring, weight or horizon change is used.
"""
from __future__ import annotations

import numpy as np

from .se2 import compose_poses,se2_exp


def same_curvature_deceleration_seed(problem):
    """X=B Exp(q eta), nu=q_dot eta; q seconds, q_dot dimensionless."""
    times=np.asarray(problem.times,float)
    horizon=float(times[-1])
    if len(times)!=31 or not np.isclose(horizon,3.) or not np.allclose(np.diff(times),.1):
        raise ValueError('this diagnostic variant is fixed to the original 3s / 31 supports')
    eta=np.array(problem.initial_twist,copy=True)
    q=times-times**2/(2*horizon)
    q_dot=1-times/horizon
    poses=compose_poses(problem.boundary_pose,se2_exp(q[:,None]*eta))
    twists=q_dot[:,None]*eta
    # Preserve fixed data bit-for-bit, including world yaw representation.
    poses[0]=problem.boundary_pose
    twists[0]=problem.initial_twist
    return dict(name='fixed_same_curvature_deceleration',poses=poses,twists=twists,
        metadata=dict(formula='X(t)=B Exp((t-t^2/(2T))*eta); nu(t)=(1-t/T)*eta',
            horizon_s=horizon,eta=eta,q_s=q,q_dot=q_dot,q_ddot_per_s=-1/horizon,
            q_units='s',q_dot_units='dimensionless',eta_units=['m/s','m/s','rad/s'],
            frame='original world poses and original body twist; no reanchoring',
            parameters_fitted=False,raw_rollout_used=False,original_goal_used_to_fit=False,
            feasible_status='NOT_EVALUATED; requires original full acceptance',
            replaces_only='boundary_constant_body_twist initialization, index1'))

"""Post-solve local first-order diagnostics; never changes a trajectory vector."""
from __future__ import annotations
import time
import numpy as np
from scipy.optimize import lsq_linear
from .gp_se2_formulation import _constraint_report


def first_order_diagnostic(problem, provider, vector, *, qp_multipliers=None):
    """Fit sign-constrained multipliers only, with original unscaled Jacobian.

    For h=0, g>=0 we use L=f-lambda*h-mu*g, mu>=0. Active inequalities
    have g<=1e-5 (the original tolerance), including violations. Rank uses
    singular values >1e-9*max(1,s_max). Fitting multipliers is not trajectory
    optimization, a sufficiency test, or a global optimality certificate.
    """
    start=time.perf_counter()
    z=np.asarray(vector,float);value=problem.evaluate(z)
    grad=provider.objective_gradient(z);jh=provider.equality_jacobian(z);jg=provider.inequality_jacobian(z)
    h=value['equality'];g=value['inequality'];active=np.flatnonzero(g<=problem.config['inequality_tolerance'])
    a=np.column_stack([jh.T,jg[active].T]);m=len(h)
    singular=np.linalg.svd(a,compute_uv=False);rank_tol=1e-9*max(1.,float(singular[0]) if len(singular) else 0.)
    fit=lsq_linear(a,grad,bounds=(np.r_[np.full(m,-np.inf),np.zeros(len(active))],
        np.full(m+len(active),np.inf)),tol=1e-10,max_iter=200,lsmr_tol=1e-12)
    lam=fit.x[:m];mu=fit.x[m:];stationarity=grad-a@fit.x
    out=dict(primal=_constraint_report(h,g,problem.config),active_inequality_indices=active,
        active_threshold=problem.config['inequality_tolerance'],active_jacobian_shape=list(a.T.shape),
        active_jacobian_rank=int(np.count_nonzero(singular>rank_tol)),rank_tolerance=rank_tol,
        rank_definition='unscaled singular values >1e-9*max(1,s_max)',singular_values=singular,
        rank_deficient=bool(np.count_nonzero(singular>rank_tol)<min(a.shape)),
        equality_multipliers=lam,active_inequality_multipliers=mu,
        minimum_inequality_multiplier=float(np.min(mu,initial=0.)),
        stationarity_inf=float(np.max(np.abs(stationarity),initial=0.)),
        objective_gradient_inf=float(np.max(np.abs(grad),initial=0.)),
        complementarity_inf=float(np.max(np.abs(mu*g[active]),initial=0.)),
        multiplier_fit_success=bool(fit.success),multiplier_fit_message=fit.message,
        multiplier_convention='L=f-lambda*h-mu*g, g>=0, mu>=0; no solver constraint rescaling',
        method='post-solve bounded linear least-squares multiplier fit; vector fixed',
        global_optimum_proven=False,sufficiency_proven=False,
        nonsmooth_limitation='selected branch/generalized derivatives; no everywhere differentiability claim')
    if qp_multipliers is not None:
        q=np.asarray(qp_multipliers,float)
        if q.shape!=(len(h)+len(g),):raise ValueError('unexpected SLSQP multiplier ordering/shape')
        out['slsqp_internal_qp']=dict(multipliers=q,
            stationarity_inf=float(np.max(np.abs(grad-jh.T@q[:len(h)]-jg.T@q[len(h):]))),
            complementarity_inf=float(np.max(np.abs(q[len(h):]*g))),
            minimum_inequality_multiplier=float(np.min(q[len(h):],initial=0.)),
            meaning='multipliers of final internal QP, not nonlinear/global optimality proof')
    out['wall_time_s']=time.perf_counter()-start
    return out

import numpy as np
from reconciliation.gp_se2_diag02_optimality import first_order_diagnostic


class Problem:
    config={'equality_tolerance':1e-5,'inequality_tolerance':1e-5}
    def evaluate(self,z):return {'equality':np.array([z[0]]),'inequality':np.array([z[1]])}


class Provider:
    def objective_gradient(self,z):return np.array([-2.,3.])
    def equality_jacobian(self,z):return np.array([[1.,0.]])
    def inequality_jacobian(self,z):return np.array([[0.,1.]])


def test_sign_constrained_stationarity_with_nonzero_objective_gradient():
    z=np.zeros(2);r=first_order_diagnostic(Problem(),Provider(),z,qp_multipliers=np.array([-2.,3.]))
    assert r['primal']['feasible'] and r['stationarity_inf']<1e-9
    assert r['objective_gradient_inf']==3.
    assert r['active_jacobian_rank']==2
    assert r['slsqp_internal_qp']['stationarity_inf']==0.
    assert r['complementarity_inf']==0.
    assert not r['global_optimum_proven']
    np.testing.assert_array_equal(z,np.zeros(2))


def test_inactive_constraint_is_not_used_to_cancel_gradient():
    r=first_order_diagnostic(Problem(),Provider(),np.array([0.,1.]))
    assert len(r['active_inequality_indices'])==0
    assert r['stationarity_inf']==3.

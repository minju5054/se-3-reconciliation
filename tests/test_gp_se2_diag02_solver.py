"""Only derivative plumbing differs; fixtures do not measure research outcomes."""
import numpy as np
import pytest
from reconciliation.gp_se2_diag_fixtures import make_fixture_problem
from reconciliation import gp_se2_diag02_solver as new
from reconciliation import gp_se2_diag_solver as old


class Provider:
    def __init__(self,n,me,mi):self.n=n;self.me=me;self.mi=mi;self.calls=[]
    def objective_gradient(self,z):self.calls.append('f');return np.zeros(self.n)
    def equality_jacobian(self,z):self.calls.append('h');return np.zeros((self.me,self.n))
    def inequality_jacobian(self,z):self.calls.append('g');return np.zeros((self.mi,self.n))
    def stats(self):return dict(calls=self.calls)


@pytest.mark.parametrize('supplied',[False,True])
def test_three_derivative_callbacks_and_original_fun_values(monkeypatch,supplied):
    p,fixture=make_fixture_problem('S0');z=np.asarray(fixture['known_vector'])
    e=p.evaluate(z);provider=Provider(len(z),len(e['equality']),len(e['inequality']))
    def fake(fun,x,*,jac,method,constraints,callback,options):
        assert fun(x)==p.evaluate(x)['objective'];assert method=='SLSQP'
        for c,key in zip(constraints,['equality','inequality']):
            np.testing.assert_array_equal(c['fun'](x),p.evaluate(x)[key])
            assert ('jac' in c)==supplied
            if supplied:assert c['jac'](x).shape==(len(e[key]),len(x))
        assert (jac is not None)==supplied
        if supplied:assert jac(x).shape==x.shape
        callback(x)
        from types import SimpleNamespace
        return SimpleNamespace(x=x,success=True,status=0,message='fixture',nit=1,nfev=1,
            multipliers=np.zeros(len(e['equality'])+len(e['inequality'])))
    monkeypatch.setattr(new,'minimize',fake)
    result=new.run_instrumented(p,z,derivative_provider=provider if supplied else None,require_single_blas=False)
    assert result['returned_initial_unchanged'] and result['initial_feasible']
    assert not result['recovered_from_infeasible']
    assert result['callback_snapshots'][0]['evaluation_complete']
    assert set(provider.calls)==({'f','h','g'} if supplied else set())
    assert result['candidate_selection'].startswith('minimum original objective')


def test_nonfinite_derivative_failure_is_explicit(monkeypatch):
    p,fixture=make_fixture_problem('S0');z=np.asarray(fixture['known_vector'])
    e=p.evaluate(z);provider=Provider(len(z),len(e['equality']),len(e['inequality']))
    provider.objective_gradient=lambda z:np.full(len(z),np.nan)
    def fake(fun,x,**kw):kw['jac'](x);raise AssertionError('unreachable')
    monkeypatch.setattr(new,'minimize',fake)
    result=new.run_instrumented(p,z,derivative_provider=provider,require_single_blas=False)
    assert result['termination']=='NUMERICAL_FAILURE'
    assert 'nonfinite supplied derivative' in result['message']
    assert result['candidate_found'] # Feasible seed remains separately identifiable.
    assert not result['solver_success']


def test_fd_mode_matches_old_harness_with_deterministic_solver(monkeypatch):
    from types import SimpleNamespace
    def fake(fun,x,**kw):
        for c in kw['constraints']:c['fun'](x)
        fun(x);kw['callback'](x)
        return SimpleNamespace(x=x,success=True,status=0,message='fixture',nit=1,nfev=1)
    monkeypatch.setattr(old,'minimize',fake);monkeypatch.setattr(new,'minimize',fake)
    p,f=make_fixture_problem('S0');q,g=make_fixture_problem('S0')
    a=old.run_instrumented(p,f['known_vector'],require_single_blas=False)
    b=new.run_instrumented(q,g['known_vector'],require_single_blas=False)
    for key in ['initial_vector','latest_iterate','candidate_vector','support_poses','support_twists']:
        np.testing.assert_array_equal(a[key],b[key])
    for key in ['termination','initial_feasible','selected_iterate','candidate_objective',
                'returned_initial_unchanged','candidate_selection']:
        assert a[key]==b[key]

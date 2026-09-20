"""Synthetic implementation/policy tests only; never actual ablation evidence."""
from __future__ import annotations
import copy
import importlib.util
import inspect
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
pytest.importorskip('jax')
from reconciliation.gp_se2_diag05_constraints import InequalityOnlyView,InequalityOnlyDerivatives,G2,margin_diagnostics,constraint_row_metadata
from reconciliation.gp_se2_diag05_validation import PROTOCOL,schedule,verify_point,classify,equality_diagnostics,vector_hash
from reconciliation.gp_se2_diag04_validation import PROTOCOL as P04,directions_and_perturbation
from reconciliation.gp_se2_diag04_constraints import quarter_motion_values,MOTION_ROWS
from reconciliation.gp_se2_diag02_derivatives import DerivativeProvider,DerivativeError
from reconciliation import gp_se2_diag04_solver as solver
ROOT=Path(__file__).resolve().parents[1]


def module(file,name):
    spec=importlib.util.spec_from_file_location(name,ROOT/file);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


fixtures=module('tests/test_gp_se2_diag04_constraints.py','diag05_test_fixtures')


@pytest.fixture(scope='module')
def ready():
    base,z,geometry=fixtures.fixture()
    for name in ('workspace','obstacle'):
        method=getattr(geometry,name)
        def with_metadata(xy,method=method):
            values,jac,meta=method(xy)
            return values,jac,dict(meta,points=[dict(kind='synthetic_affine',smooth=True) for _ in xy])
        setattr(geometry,name,with_metadata)
    _,perturbation=directions_and_perturbation();z=z+perturbation
    original=DerivativeProvider(base,geometry);view=InequalityOnlyView(base);provider=InequalityOnlyDerivatives(view,original);provider.warmup(z)
    return base,z,geometry,original,view,provider


@pytest.mark.parametrize('obstacles,expected',[(False,1293),(True,1383)])
def test_exact_dimensions_prefix_no_new_environment_or_equality(obstacles,expected):
    b,z,env=fixtures.fixture(obstacles=obstacles);old=b.evaluate(z);calls=dict(env.calls);v=InequalityOnlyView(b);e=v.evaluate(z)
    assert old['objective']==e['objective'] and e['equality'] is old['equality']
    assert env.calls==calls
    np.testing.assert_array_equal(old['inequality'],e['inequality'][:-480])
    np.testing.assert_array_equal(e['inequality'][-480:],quarter_motion_values(b,z)['quarter_inequality'])
    dims=v.dimensions(z)
    assert (dims['variable_count'],dims['equality_count'],dims['inequality_count'])==(150,30,expected)
    rows=v.appended_row_metadata(z)
    assert len(rows)==480 and all(r['constraint_kind']=='inequality' for r in rows)
    assert all('lateral' not in r['family'] and all(s not in r['family'] for s in ('goal','workspace','obstacle','gate')) for r in rows)
    metadata=constraint_row_metadata(v,z)
    assert len([r for r in metadata['rows'] if r['constraint_kind']=='equality'])==30
    assert len(set(r['row_index'] for r in rows))==480


def test_literal_all_base_derivatives_and_quarter_primal(ready):
    b,z,geo,original,v,p=ready;a=original.values(z);g=p.values(z)
    assert g['objective']==a['objective'];np.testing.assert_array_equal(g['equality'],a['equality'])
    np.testing.assert_array_equal(p.objective_gradient(z),original.objective_gradient(z))
    np.testing.assert_array_equal(p.equality_jacobian(z),original.equality_jacobian(z))
    np.testing.assert_array_equal(p.inequality_jacobian(z)[:-480],original.inequality_jacobian(z))
    assert p.equality_jacobian(z).shape==(30,150)
    np.testing.assert_allclose(g['quarter_inequality'],v.evaluate(z)['quarter_inequality'],atol=1e-8,rtol=1e-10)
    assert p.stats()['appended_equalities']==0


def test_multistep_directional_gate_and_bad_jacobian_fail(ready,monkeypatch):
    b,z,geo,original,v,p=ready;directions,_=directions_and_perturbation()
    def forbidden(*a,**kw):pytest.fail('verification must not optimize')
    monkeypatch.setattr(solver,'minimize',forbidden)
    result,arrays=verify_point('synthetic',z,b,geo,original,v,p,directions)
    assert result['valid'] and all(result['parity'].values()) and arrays['quarter_jacobian'].shape==(480,150)
    old=p.inequality_jacobian
    def wrong(x):
        j=old(x).copy();j[-1,20]+=1;return j
    monkeypatch.setattr(p,'inequality_jacobian',wrong)
    result,_=verify_point('synthetic_bad',z,b,geo,original,v,p,directions)
    assert not result['valid']


def test_equality_rank_same_vector_and_no_solver_scaling(ready,monkeypatch):
    b,z,geo,original,v,p=ready
    r=equality_diagnostics(original,p,{'initial':z,'latest_iterate':z.copy(),'selected':None})
    assert all(a['literal_equal'] for a in r['equality_parity'] if a['available'])
    assert not r['solver_scaling_changed']
    for source in ('initial','latest_iterate'):
        rows=[q for q in r['records'] if q['source']==source]
        assert [q['grid'] for q in rows]==['G0_ORIGINAL',G2]
        for scale in ('raw','column_scaled'):
            np.testing.assert_array_equal(rows[0]['matrices'][scale]['singular_values'],rows[1]['matrices'][scale]['singular_values'])
            assert rows[0]['matrices'][scale]['matrix_sha256']==rows[1]['matrices'][scale]['matrix_sha256']
    old=p.equality_jacobian
    monkeypatch.setattr(p,'equality_jacobian',lambda x:old(x)+1e-10)
    with pytest.raises(ValueError,match='MISMATCH'):equality_diagnostics(original,p,{'initial':z})


def test_actual_slsqp_callbacks_have_no_quarter_lateral(ready,monkeypatch):
    b,z,geo,original,v,p=ready;seen=[]
    def fake(fun,x,*,jac,method,constraints,callback,options):
        assert method=='SLSQP' and options==dict(maxiter=200,ftol=1e-7,disp=False)
        fun(x);jac(x)
        for c in constraints:seen.append((c['type'],c['fun'](x).shape,c['jac'](x).shape))
        callback(x.copy())
        return SimpleNamespace(x=x,success=False,status=8,message='synthetic failure',nit=1,nfev=1)
    checks=[]
    def full(base,vector,case):
        checks.append(base);assert base is b
        return dict(full_feasible=False,status='synthetic lateral rejection')
    monkeypatch.setattr(solver,'minimize',fake);monkeypatch.setattr(solver,'check_full_candidate',full)
    r=solver.run_refined(v,z,derivative_provider=p,case={},require_single_blas=False)
    assert seen==[('eq',(30,),(30,150)),('ineq',(1383,),(1383,150))]
    assert checks and r['solver_status']==8 and r['minimize_invocations']==1 and not r['selected_full_feasible']
    assert r['new_mpc_solves']==r['new_rollouts']==0 and not r['fallback_used']


def test_margin_order_units_and_near_active_not_feasibility(ready):
    b,z,geo,original,v,p=ready;rows=margin_diagnostics(v,p,z)
    assert [(r['family'],r['unit']) for r in rows]==[(r[0],r[2]) for r in MOTION_ROWS]
    assert all(len(r['margin'])==60 and len(r['row_norms'])==60 for r in rows)
    assert all(r['near_active_threshold']==max(1e-6,10*b.config['inequality_tolerance']) for r in rows)
    assert all(r['violating_rows']<=r['near_active_including_violating'] for r in rows)


def result(*,grid=True,failed=('lateral_velocity',),selected=False,cost=None,success=True):
    latest=dict(collocation=dict(feasible=grid),grid_and_full_feasible=not failed and grid,
        constraint_report=dict(maximum_inequality_violation=0.,body_derivative_identity_valid=True),
        full_acceptance=dict(additional_grid=dict(flags={k:k not in failed for k in ('lateral_velocity','linear_speed','linear_acceleration','original_goal')})))
    return dict(candidate_checks=[dict(objective=4.),latest],candidate_objective=cost,selected_full_feasible=selected,
        solver_success=success,config=dict(ftol=1e-7,inequality_tolerance=1e-5))


@pytest.mark.parametrize('grid,failed,layer',[(False,('lateral_velocity',),'A_GRID_FAIL'),(True,('lateral_velocity',),'B_LATERAL_ONLY'),(True,('linear_speed',),'C_OTHER_FULL_FAILURE'),(True,(),'D_FULL_VALID')])
def test_taxonomy_preserves_lateral_failure(grid,failed,layer):
    r=result(grid=grid,failed=failed);c=classify(r);assert c['layer']==layer
    if failed:assert c['outcome']!='FULL_FEASIBILITY_RECOVERY'
    assert c['objective_delta'] is None


def test_dense_inequality_failure_cannot_be_lateral_only():
    r=result();r['candidate_checks'][1]['constraint_report']['maximum_inequality_violation']=.001
    assert classify(r)['layer']=='C_OTHER_FULL_FAILURE'


def test_benign_seed_retention_vs_real_objective_improvement_and_missing():
    assert classify(result(selected=True,cost=4.),benign=True)['outcome']=='BENIGN_FEASIBILITY_ONLY'
    assert classify(result(selected=True,cost=3.9,failed=()),benign=True)['outcome']=='BENIGN_OPTIMIZATION_RECOVERED'
    r=result();r['candidate_checks'][1]['full_acceptance']={}
    assert classify(r)['outcome']=='SOURCE_OR_DERIVATIVE_BLOCKER' and classify(r)['full_failure_flags'] is None


def test_exact_schedule_five_and_frozen_protocol():
    rows=schedule();assert len(rows)==5 and len({r['solve_id'] for r in rows})==5
    assert all(r['grid']==G2 for r in rows) and [r['pair_index'] for r in rows]==list(range(5))
    assert [r['initialization'] for r in rows]==['I0_FRESH','I1_DECEL','I0_FRESH','I1_DECEL','I1_DECEL']
    assert PROTOCOL['extra_equalities']==0 and PROTOCOL['extra_inequalities']==480
    for k in ('directional_fd_steps','primal_tolerance','objective_derivative_tolerance','constraint_derivative_tolerance','max_iterations','prepared_solve_budget_s','ftol'):
        assert PROTOCOL[k]==P04[k]
    assert all(PROTOCOL[k]==0 for k in ('new_VLA_inference','new_MPC_solve','new_rollout','new_GUI_runtime'))
    a=np.zeros(150);b=a.copy();b[0]=np.nextafter(0.,1.)
    assert vector_hash(a)!=vector_hash(b)


def test_hash_guard_no_overwrite_no_retry_derivative_failure(tmp_path,monkeypatch):
    r=module('scripts/run_gp_se2_diag05.py','diag05_runner_test')
    with pytest.raises(FileExistsError):r.prepare(tmp_path)
    monkeypatch.setattr(r,'verify_frozen',lambda run:{})
    (tmp_path/'derivative_checks').mkdir();(tmp_path/'derivative_checks/validation.json').write_text('{"valid":false}')
    with pytest.raises(ValueError,match='DERIVATIVE_VALIDATION_FAILED'):r.solve(tmp_path)
    (tmp_path/'derivative_checks/validation.json').write_text('{"valid":true}');(tmp_path/'optimization_started.json').write_text('{}')
    with pytest.raises(FileExistsError,match='no retry'):r.solve(tmp_path)
    (tmp_path/'derivative_checks/started.json').write_text('{}')
    with pytest.raises(FileExistsError):r.derivatives(tmp_path)


def test_source_guard_checks_original_and_new_seed_hashes(tmp_path,monkeypatch):
    r=module('scripts/run_gp_se2_diag05.py','diag05_hash_test');source=tmp_path/'old.npy';source.write_bytes(b'old')
    seed=tmp_path/'seed.npy';seed.write_bytes(b'seed')
    ledger=dict(preserved_hashes={str(source):r.digest(source)},user_config_hashes={},experiment_code_sha256={},frozen_input_sha256={'seed.npy':r.digest(seed)})
    (tmp_path/'source.json').write_text(json.dumps(ledger));assert r.verify_frozen(tmp_path)==ledger
    source.write_bytes(b'changed')
    with pytest.raises(ValueError,match='MISMATCH'):r.verify_frozen(tmp_path)
    source.write_bytes(b'old');seed.write_bytes(b'changed')
    with pytest.raises(ValueError,match='MISMATCH'):r.verify_frozen(tmp_path)


def test_plot_saved_numeric_coverage_and_no_optimizer(tmp_path,monkeypatch):
    old=module('tests/test_gp_se2_diag04_plots.py','diag05_old_plot_fixture');data,env=old.fixture(tmp_path)
    p=module('scripts/plot_gp_se2_diag05.py','diag05_plot_test');solves=[]
    for i in range(5):
        group=copy.deepcopy(data['solves'][2*i:2*i+2]);new=copy.deepcopy(group[0]);new.update(grid=G2,solve_id='G2_fixture_'+str(i));group.append(new)
        for j,s in enumerate(group):
            s['provenance']='NEW_DIAG05_G2' if j==2 else 'HISTORICAL_DIAG04_NOT_RERUN'
            s['outcome']=dict(solver_status=0 if j==0 else 8,iterations=1,latest_grid_feasible=False,latest_dense_feasible=False,
                latest_original_full_feasible=False,selected_full_feasible=False,latest_objective=12.,initial_objective=30.,solve_wall_time_s=.5,failure_flags=['lateral_velocity'])
            if j==2:
                for phase in ('initial','latest','selected'):
                    if s['conditioning'].get(phase):s['conditioning'][phase][G2]=s['conditioning'][phase]['G0_ORIGINAL']
                s['margins']={'initial':[dict(minimum_margin=.1) for _ in range(8)],'latest_iterate':[dict(minimum_margin=-1e-6) for _ in range(8)]}
        solves+=group
    data['solves']=solves;(tmp_path/'plot_input.json').write_text(json.dumps(data))
    assert len(p.verify_payload(data))==5
    def forbidden(*a,**kw):pytest.fail('plots may not solve')
    monkeypatch.setattr(solver,'minimize',forbidden);p.generate(tmp_path,environment=env)
    for name in p.PLOTS:
        side=json.loads((tmp_path/'plots'/(name+'.json')).read_text())
        assert side['numeric_data']==p.expected_numeric(name,data)
        assert side['image_sha256']==p.digest(tmp_path/'plots'/(name+'.png'))
    assert (tmp_path/'review_bundle.zip').exists()
    with pytest.raises(FileExistsError):p.generate(tmp_path,environment=env)
    data['solves'].pop()
    with pytest.raises(ValueError):p.verify_payload(data)

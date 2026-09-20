"""Synthetic implementation/policy tests; never actual-event evidence."""
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
from reconciliation.gp_se2_diag06_witnesses import canonical_grid,extract_runs,witness_union,validate_schedule,finest_trace
from reconciliation.gp_se2_diag06_constraints import WitnessView,WitnessDerivatives,G3,witness_values
from reconciliation.gp_se2_diag06_validation import PROTOCOL,schedule,verify_point,equality_diagnostics,classify
from reconciliation.gp_se2_diag05_constraints import InequalityOnlyView,InequalityOnlyDerivatives
from reconciliation.gp_se2_diag04_constraints import MOTION_ROWS,constraint_row_metadata
from reconciliation.gp_se2_diag04_validation import directions_and_perturbation,PROTOCOL as P04
from reconciliation.gp_se2_diag02_derivatives import DerivativeProvider,DerivativeError
from reconciliation import gp_se2_diag04_solver as solver
ROOT=Path(__file__).resolve().parents[1]


def module(file,name):
    spec=importlib.util.spec_from_file_location(name,ROOT/file);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

fixtures=module('tests/test_gp_se2_diag04_constraints.py','diag06_fixtures')


def trace_fixture():
    t=np.array([0,.025,.05,.075,.1,.1,.125,.15,.175,.2])
    return dict(times_s=t,interval_index=np.repeat([0,1],5),local_fraction=np.tile([0,.25,.5,.75,1],2),
        location_type=['sample']*10,knot_side=['right',None,None,None,'left']*2,
        quantities=dict(vx=np.full(10,.4),vy=np.full(10,4.),omega=np.zeros(10),ax=np.zeros(10),ay=np.full(10,200.),alpha=np.zeros(10)))


def test_runs_only_actual_motion_violations_and_earliest_ties():
    trace=trace_fixture();base,_,_=fixtures.fixture();c=base.config
    trace['quantities']['vx'][1:4]=[-2e-5,-3e-5,-3e-5]
    trace['quantities']['vx'][6]=-3e-6 # feasible near-active excluded
    trace['quantities']['ax'][4:6]=[-2.001,-2.002] # one connected run, both knot sides retained
    rows=extract_runs(trace,c,'source')
    assert len(rows)==2
    assert [(r['family'],r['canonical_grid_sample_index']) for r in rows]==[('linear_speed_lower',2),('linear_acceleration_lower',5)]
    assert rows[1]['first_sample_index']==4 and rows[1]['last_sample_index']==5
    assert all(r['tolerance_excess']>0 for r in rows)
    assert not any('lateral' in r['family'] for r in rows)
    assert rows==extract_runs(trace,c,'source')


def test_union_deduplicates_exact_identity_keeps_source_provenance():
    t=trace_fixture();b,_,_=fixtures.fixture();t['quantities']['vx'][2]=-.01
    a=extract_runs(t,b.config,'a');other=extract_runs(t,b.config,'b');union=witness_union(a+other)
    assert len(union)==1 and len(union[0]['source_violations'])==2
    assert validate_schedule(union,canonical_grid(t))
    altered=copy.deepcopy(union);altered[0]['time_s']=np.nextafter(altered[0]['time_s'],1.)
    with pytest.raises(ValueError,match='identity'):validate_schedule(altered,canonical_grid(t))
    bad=copy.deepcopy(union);bad[0]['family']='lateral_velocity'
    with pytest.raises(ValueError):validate_schedule(bad,canonical_grid(t))


def synthetic_rows(base,z):
    trace=finest_trace(base,z);grid=canonical_grid(trace);rows=[]
    for number,(family,quantity,unit,*_) in enumerate(MOTION_ROWS):
        k=number*100+17
        rows.append(dict(witness_row_index=number,family=family,quantity=quantity,unit=unit,
            canonical_grid_sample_index=k,interval_index=grid['interval_index'][k],local_fraction=grid['local_fraction'][k],
            time_s=grid['times_s'][k],knot_side=grid['knot_side'][k],source_violations=[]))
    assert validate_schedule(rows,grid)
    return rows


@pytest.fixture(scope='module')
def ready():
    b,z,geo=fixtures.fixture()
    for name in ('workspace','obstacle'):
        old=getattr(geo,name)
        def metadata(xy,old=old):
            value,jac,meta=old(xy);return value,jac,dict(meta,points=[dict(kind='synthetic_affine',smooth=True) for _ in xy])
        setattr(geo,name,metadata)
    directions,perturbation=directions_and_perturbation();z=z+perturbation
    original=DerivativeProvider(b,geo);g2=InequalityOnlyView(b);p2=InequalityOnlyDerivatives(g2,original)
    rows=synthetic_rows(b,z);g3=WitnessView(b,rows);p3=WitnessDerivatives(g3,p2);p3.warmup(z)
    return b,z,geo,original,g2,p2,g3,p3,directions


def test_exact_objective_equality_g2_prefix_and_shapes(ready):
    b,z,geo,old,g2,p2,g3,p3,d=ready;a=g2.evaluate(z);v=g3.evaluate(z)
    assert a['objective']==v['objective'] and v['equality'] is a['equality']
    np.testing.assert_array_equal(v['inequality'][:-8],a['inequality'])
    np.testing.assert_array_equal(p3.objective_gradient(z),p2.objective_gradient(z))
    np.testing.assert_array_equal(p3.equality_jacobian(z),p2.equality_jacobian(z))
    np.testing.assert_array_equal(p3.inequality_jacobian(z)[:-8],p2.inequality_jacobian(z))
    dims=g3.dimensions(z);assert (dims['variable_count'],dims['equality_count'],dims['inequality_count'])==(150,30,1391)
    assert dims['added_environment_rows']==dims['added_goal_rows']==0
    assert p3.equality_jacobian(z).shape==(30,150)
    np.testing.assert_allclose(p3.values(z)['witness_inequality'],witness_values(b,z,g3.rows),atol=1e-8,rtol=1e-10)
    rows=constraint_row_metadata(g3,z)['rows'];added=[r for r in rows if r['source']=='frozen_extremum_witness']
    assert len(added)==8 and [(r['family'],r['unit']) for r in added]==[(r[0],r[2]) for r in MOTION_ROWS]
    assert all(r['constraint_kind']=='inequality' for r in added)
    assert len([r for r in rows if r['constraint_kind']=='equality'])==30


def test_directional_gate_all_signed_families_and_corruption_blocks(ready,monkeypatch):
    b,z,geo,old,g2,p2,g3,p3,d=ready
    def forbidden(*a,**kw):pytest.fail('derivative gate must not optimize')
    monkeypatch.setattr(solver,'minimize',forbidden)
    r,arrays=verify_point('synthetic',z,b,geo,old,g2,p2,g3,p3,d)
    assert r['valid'] and all(r['parity'].values()) and arrays['witness_jacobian'].shape==(8,150)
    fn=p3.inequality_jacobian
    def wrong(x):
        out=fn(x).copy();out[-1,30]+=1;return out
    monkeypatch.setattr(p3,'inequality_jacobian',wrong)
    r,_=verify_point('bad',z,b,geo,old,g2,p2,g3,p3,d);assert not r['valid']


def test_nonfinite_and_guard_are_not_silently_repaired(ready,monkeypatch):
    b,z,geo,old,g2,p2,g3,p3,d=ready;bad=z.copy();bad[4]=np.nan
    with pytest.raises((ValueError,DerivativeError)):g3.evaluate(bad)
    with pytest.raises((ValueError,DerivativeError)):p3.inequality_jacobian(bad)
    def cut(*a,**kw):raise DerivativeError('synthetic unsupported cut')
    monkeypatch.setattr(old,'_guard_wrap_cuts',cut)
    with pytest.raises(DerivativeError,match='cut'):p3.inequality_jacobian(z)


def test_same_equality_spectrum_at_same_vector(ready):
    b,z,geo,old,g2,p2,g3,p3,d=ready
    r=equality_diagnostics(old,p3,dict(initial=z,latest_iterate=z.copy(),selected=None))
    assert all(v['literal_equal'] for v in r['equality_parity'] if v['available'])
    for source in ('initial','latest_iterate'):
        a,b=[v for v in r['records'] if v['source']==source]
        assert b['grid']==G3
        assert a['matrices']['raw']['matrix_sha256']==b['matrices']['raw']['matrix_sha256']


def test_actual_solver_interface_only_appends_witness_inequalities(ready,monkeypatch):
    b,z,geo,old,g2,p2,g3,p3,d=ready;calls=[]
    def fake(fun,x,*,jac,method,constraints,callback,options):
        fun(x);jac(x)
        for c in constraints:calls.append((c['type'],c['fun'](x).shape,c['jac'](x).shape))
        callback(x.copy());return SimpleNamespace(x=x,success=False,status=8,message='synthetic',nit=1,nfev=1)
    def full(base,vector,case):
        assert base is b;return dict(full_feasible=False,status='synthetic rejection')
    monkeypatch.setattr(solver,'minimize',fake);monkeypatch.setattr(solver,'check_full_candidate',full)
    r=solver.run_refined(g3,z,derivative_provider=p3,case={},require_single_blas=False)
    assert calls==[('eq',(30,),(30,150)),('ineq',(1391,),(1391,150))]
    assert r['solver_status']==8 and r['minimize_invocations']==1 and not r['selected_full_feasible']
    assert r['new_mpc_solves']==r['new_rollouts']==0 and not r['fallback_used']


def result(*,success=True,failed=('lateral_velocity',),selected=False,cost=None):
    return dict(solver_success=success,selected_full_feasible=selected,candidate_objective=cost,
        config=dict(ftol=1e-7,inequality_tolerance=1e-5),candidate_checks=[dict(objective=4.),
            dict(constraint_report=dict(maximum_inequality_violation=0.,body_derivative_identity_valid=True),
                full_acceptance=dict(additional_grid=dict(flags={k:k not in failed for k in ('lateral_velocity','linear_speed','linear_acceleration','original_goal')})))])


@pytest.mark.parametrize('remaining,repaired,success,expected',[
    ([],True,True,'INEQUALITY_GAP_CLOSED_LATERAL_ONLY_REMAINS'),
    ([{}],True,True,'VIOLATION_RELOCATED'),([{}],False,True,'TARGETED_VIOLATION_REMAINS'),
    ([],True,False,'NUMERICAL_REGRESSION')])
def test_taxonomy_never_makes_grid_pass_full_fail_success(remaining,repaired,success,expected):
    r=classify(result(success=success),remaining,[dict(feasible=repaired)])
    assert r['outcome']==expected and r['objective_delta'] is None


def test_benign_retained_optimized_and_regression_distinct():
    for selected,cost,expected in [(True,3.,'BENIGN_OPTIMIZATION_PRESERVED'),(True,4.,'BENIGN_FEASIBILITY_ONLY'),(False,None,'BENIGN_REGRESSION')]:
        assert classify(result(selected=selected,cost=cost),[],[],benign=True)['outcome']==expected
    assert classify(result(selected=True,cost=3.,failed=()),[],[])['outcome']=='FULL_FEASIBILITY_RECOVERED'


def test_schedule_thresholds_no_adaptive_round():
    rows=schedule();assert len(rows)==5 and [r['pair_index'] for r in rows]==list(range(5))
    assert all(r['grid']==G3 for r in rows)
    assert [r['initialization'] for r in rows]==['I0_FRESH','I1_DECEL','I0_FRESH','I1_DECEL','I1_DECEL']
    assert PROTOCOL['extra_equalities']==0 and not PROTOCOL['adaptive_refinement'] and PROTOCOL['additional_refinement_rounds']==0
    for k in ('directional_fd_steps','primal_tolerance','constraint_derivative_tolerance','ftol','prepared_solve_budget_s','max_iterations'):assert PROTOCOL[k]==P04[k]
    assert all(PROTOCOL[k]==0 for k in ('new_VLA_inference','new_MPC_solve','new_rollout','new_GUI_runtime'))


def test_seed_hash_no_overwrite_no_retry_and_failed_gate(tmp_path,monkeypatch):
    r=module('scripts/run_gp_se2_diag06.py','diag06_runner_test')
    with pytest.raises(FileExistsError):r.prepare(tmp_path)
    source=tmp_path/'old.npy';source.write_bytes(b'old');seed=tmp_path/'seed.npy';seed.write_bytes(b'seed')
    ledger=dict(preserved_hashes={str(source):r.digest(source)},user_config_hashes={},experiment_code_sha256={},frozen_input_sha256={'seed.npy':r.digest(seed)})
    (tmp_path/'source.json').write_text(json.dumps(ledger));assert r.verify_frozen(tmp_path)==ledger
    source.write_bytes(b'changed')
    with pytest.raises(ValueError,match='MISMATCH'):r.verify_frozen(tmp_path)
    monkeypatch.setattr(r,'verify_frozen',lambda p:{})
    (tmp_path/'derivative_checks').mkdir();(tmp_path/'derivative_checks/validation.json').write_text('{"valid":false}')
    (tmp_path/'execution_freeze.json').write_text(json.dumps(dict(execution_git_sha=r.revision())))
    with pytest.raises(ValueError,match='verified'):r.solve(tmp_path)
    (tmp_path/'derivative_checks/validation.json').write_text('{"valid":true}');(tmp_path/'optimization_started.json').write_text('{}')
    with pytest.raises(FileExistsError,match='no retry'):r.solve(tmp_path)
    code=inspect.getsource(r.solve)
    assert "z=np.load(run/row['derived_seed_path']" in code and 'latest_iterate' not in code
    assert code.count('run_refined(')==1 and 'extract_runs' not in code and 'witness_union(' not in code
    assert 'run_refined(' not in inspect.getsource(r.validate)


def test_empty_union_and_input_schedule_copy():
    b,z,geo=fixtures.fixture();g3=WitnessView(b,[]);g2=InequalityOnlyView(b)
    np.testing.assert_array_equal(g3.evaluate(z)['inequality'],g2.evaluate(z)['inequality'])
    assert g3.evaluate(z)['witness_inequality'].shape==(0,)
    rows=synthetic_rows(b,z);v=WitnessView(b,rows);rows[0]['local_fraction']=.999
    assert v.rows[0]['local_fraction']!=.999


@pytest.mark.parametrize('obstacles,expected',[(False,1301),(True,1391)])
def test_both_method_dimensions_and_witness_order(obstacles,expected):
    b,z,geo=fixtures.fixture(obstacles=obstacles);rows=synthetic_rows(b,z);v=WitnessView(b,rows)
    assert v.dimensions(z)['inequality_count']==expected
    assert v.dimensions(z)['appended_equality_count']==0
    assert v.dimensions(z)['appended_inequality_count']==488


def test_static_numbers_complete_ledger_and_no_solve(tmp_path,monkeypatch):
    old=module('tests/test_gp_se2_diag04_plots.py','diag06_old_plot_fixture');data,env=old.fixture(tmp_path)
    p=module('scripts/plot_gp_se2_diag06.py','diag06_plot_test');solves=[]
    b,z,_=fixtures.fixture();witnesses=synthetic_rows(b,z)[:2]
    for i in range(5):
        group=copy.deepcopy(data['solves'][2*i:2*i+2]);g2=copy.deepcopy(group[0]);g2.update(grid='G2_QUARTER_INEQUALITY_ONLY',solve_id=f'G2_fixture_{i}')
        g3=copy.deepcopy(group[0]);g3.update(grid=G3,solve_id=f'G3_fixture_{i}');group.extend([g2,g3])
        for j,s in enumerate(group):
            s['pair_index']=i;s['provenance']='NEW_DIAG06_G3' if j==3 else 'HISTORICAL'
            s['outcome']=dict(solver_status=0,iterations=1,latest_grid_feasible=True,latest_dense_feasible=False,
                latest_original_full_feasible=False,selected_full_feasible=False,latest_objective=12.,initial_objective=30.,solve_wall_time_s=.5,failure_flags=['lateral_velocity'])
            for phase in ('latest','selected'):
                if s[phase] is None:continue
                a=s[phase]['samples'];a['local_fraction']=a['local_u'];a['quantities']=dict(zip(['vx','vy','omega','ax','ay','alpha'],np.column_stack([a['body_twists'],a['body_accelerations']]).T.tolist()))
            if j==3:
                margins=[dict(witness_row_index=k,family=w['family'],unit=w['unit'],new_margin=.1,old_sources=[]) for k,w in enumerate(witnesses)]
                s.update(witness_margins=dict(latest_iterate=margins),historical_G2_witness_margins=margins,remaining_motion_runs={})
        solves+=group
    data.update(solves=solves,frozen_witnesses=witnesses)
    (tmp_path/'plot_input.json').write_text(json.dumps(data));(tmp_path/'execution_freeze.json').write_text('{}')
    (tmp_path/'witness_selection').mkdir();(tmp_path/'witness_selection/frozen_union.json').write_text(json.dumps(witnesses))
    (tmp_path/'witness_selection/validation.json').write_text('{"valid":true}')
    (tmp_path/'historical_baselines').mkdir();(tmp_path/'historical_baselines/manifest.json').write_text('{}')
    def forbidden(*a,**kw):pytest.fail('static rendering cannot optimize')
    monkeypatch.setattr(solver,'minimize',forbidden);p.generate(tmp_path,environment=env)
    for name in p.PLOTS:
        side=json.loads((tmp_path/'plots'/(name+'.json')).read_text())
        assert side['numeric_data']==p.expected_numeric(name,data)
        assert side['image_sha256']==p.digest(tmp_path/'plots'/(name+'.png'))
        assert side['label']=='PLAN ONLY / NOT EXECUTED'
    assert (tmp_path/'review_bundle.zip').exists()
    with pytest.raises(FileExistsError):p.generate(tmp_path,environment=env)

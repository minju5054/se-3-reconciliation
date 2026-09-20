"""Synthetic implementation and immutable-artifact checks, not event evidence."""
import ast
import copy
import inspect
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
pytest.importorskip('jax')
from reconciliation.gp_se2_diag08_endpoint_margin import (
 G4,RESERVE_M,PLAN_RADIUS_M,EXEC_RADIUS_M,DEFINITION,G4EndpointMarginView,G4EndpointMarginDerivatives,
 reserve_report,execution_admission,endpoint_geometry,classify,execute_reference,validate_execution_records)
from reconciliation.gp_se2_diag06_constraints import WitnessView,WitnessDerivatives
from reconciliation.gp_se2_diag05_constraints import InequalityOnlyView,InequalityOnlyDerivatives
from reconciliation.gp_se2_diag02_derivatives import DerivativeProvider
from reconciliation.gp_se2_diag04_constraints import constraint_row_metadata
from reconciliation.gp_se2_diag07_lateral_execution import REQUIRED_FLAGS,deduplicate,array_hash
from test_gp_se2_diag04_constraints import fixture
from test_gp_se2_diag07 import fixture as execution_fixture,full as oldfull
from reconciliation.gp_se2_rollout import counterfactual_rollout
ROOT=Path(__file__).resolve().parents[1]


def make(name='S1',obstacles=True):
    base,z,env=fixture(name,obstacles=obstacles)
    rows=[dict(witness_row_index=k,family=f,interval_index=i,local_fraction=u,time_s=(i+u)*.1,
        canonical_grid_sample_index=k,unit=unit) for k,(f,i,u,unit) in enumerate([
        ('linear_speed_lower',4,.8,'m/s'),('linear_acceleration_lower',0,.38,'m/s^2'),('linear_acceleration_lower',1,.37371,'m/s^2')])]
    g3=WitnessView(base,rows);p0=DerivativeProvider(base,env);p2=InequalityOnlyDerivatives(InequalityOnlyView(base),p0)
    p3=WitnessDerivatives(g3,p2);view=G4EndpointMarginView(g3);p4=G4EndpointMarginDerivatives(view,p3)
    return base,z,view,p4


def full(hard=True):
    f=oldfull(hard);f['original_dense_report']={'maximum_inequality_violation':0.}
    f['original_config']={'formulation':{'inequality_tolerance':1e-5}}
    return f


def test_fixed_radius_and_no_execution_tolerance_change():
    assert RESERVE_M==.04 and PLAN_RADIUS_M==.11 and EXEC_RADIUS_M==.15
    assert np.isclose(PLAN_RADIUS_M+RESERVE_M,EXEC_RADIUS_M,rtol=0,atol=np.finfo(float).eps)
    b,z,v,p=make();assert b.config['goal_position_tolerance']==.15
    assert b.config['goal_yaw_tolerance']==pytest.approx(np.pi/12)
    assert b.config['equality_tolerance']==1e-5
    assert DEFINITION['added_equalities']==DEFINITION['added_yaw_rows']==DEFINITION['added_environment_rows']==0


@pytest.mark.parametrize('obstacles,count',[(False,1297),(True,1387)])
def test_exact_dimensions_and_literal_primal_prefix(obstacles,count):
    b,z,v,p=make(obstacles=obstacles);before=copy.deepcopy(b.config);a=v.g3.evaluate(z);g=v.evaluate(z)
    assert g['objective']==a['objective'] and g['equality'] is a['equality']
    assert np.array_equal(g['inequality'][:-1],a['inequality'])
    assert len(g['inequality'])==count and len(g['equality'])==30 and z.shape==(150,)
    assert v.dimensions(z)['G4_added_inequality_count']==1 and v.dimensions(z)['G4_added_equality_count']==0
    assert b.config==before and not hasattr(v,'dense_report')
    metadata=constraint_row_metadata(v,z);last=metadata['rows'][-1]
    assert last['family']=='planning_endpoint_reserve_position' and last['physical_unit']=='m^2'
    assert last['row_index']==count-1 and last['derivative_source_row_index']==720
    assert g['endpoint_margin'][0]==.11**2-np.sum((b.unpack(z)[0][-1,:2]-b.goal_pose[:2])**2)


@pytest.mark.parametrize('name',['S0','S1','S2','S3'])
def test_existing_ad_right_chart_jacobian_and_multi_step_fd(name):
    b,z,v,p=make(name);z=z.copy();z[-5:-2]+=[.003,-.006,.08]
    p.warmup(z);g3=p.g3
    assert np.array_equal(p.equality_jacobian(z),g3.equality_jacobian(z))
    assert np.array_equal(p.objective_gradient(z),g3.objective_gradient(z))
    j=p.inequality_jacobian(z);old=g3.inequality_jacobian(z)
    assert np.array_equal(j[:-1],old) and np.array_equal(j[-1],old[720])
    assert j.shape==(1387,150) and np.isfinite(j).all()
    np.testing.assert_allclose(p.values(z)['endpoint_margin'],v.evaluate(z)['endpoint_margin'],rtol=1e-10,atol=1e-8)
    for d in np.eye(150)[-5:-2]:
        for h in (2e-4,2e-5,2e-6):
            fd=(v.evaluate(z+h*d)['endpoint_margin']-v.evaluate(z-h*d)['endpoint_margin'])/(2*h)
            np.testing.assert_allclose(j[-1]@d,fd[0],rtol=2e-5,atol=2e-5)
    assert p.stats()['endpoint_FD_calls']==p.stats()['endpoint_additional_compilations']==0


def test_reserve_nominal_distance_is_separate_from_squared_grid_allowance():
    p=SimpleNamespace(times=np.linspace(0,3,31),goal_pose=np.zeros(3),config={'inequality_tolerance':1e-5},
        unpack=lambda z:(np.tile([.110001,0,0],(31,1)),np.zeros((31,3))))
    r=reserve_report(p,np.zeros(150))
    assert not r['planning_endpoint_reserve_pass'] and r['solver_row_pass_with_original_allowance']
    assert r['nominal_position_excess_m']>0


@pytest.mark.parametrize('hard',[True,False])
def test_admission_separates_full_valid_candidate_and_invalid_diagnostic(hard):
    a=execution_admission(full(hard),{'planning_endpoint_reserve_pass':True},hard=hard)
    assert a['eligible_for_execution'] and a['plan_valid']==(not hard)
    assert a['deployment_candidate']==(not hard) and a['diagnostic_execution_only']==hard
    if hard:assert a['plan_failure_reason']=='lateral_velocity'


def test_unexpected_full_valid_hard_is_recorded_normally():
    a=execution_admission(full(False),{'planning_endpoint_reserve_pass':True},hard=True)
    assert a['plan_valid'] and a['deployment_candidate'] and a['reference_kind']=='PLAN_VALID_CANDIDATE'


@pytest.mark.parametrize('flag',sorted(REQUIRED_FLAGS-{'lateral_velocity'}))
def test_second_plan_failure_blocks_primary(flag):
    f=full();f['additional_grid']['flags'][flag]=False
    assert not execution_admission(f,{'planning_endpoint_reserve_pass':True},hard=True)['eligible_for_execution']


def test_reserve_failure_blocks_even_full_valid_benign():
    a=execution_admission(full(False),{'planning_endpoint_reserve_pass':False},hard=False)
    assert not a['eligible_for_execution'] and not a['deployment_candidate']
    assert a['ineligible_reasons']==['planning_endpoint_reserve']


@pytest.mark.parametrize('hard',[True,False])
def test_official_computation_wrapper_parity_and_record_audit(hard):
    m,c,r=execution_fixture();ad=execution_admission(full(hard),{'planning_endpoint_reserve_pass':True},hard=hard)
    got=execute_reference(m,c,r,ad);expected=counterfactual_rollout(m,c,r)
    for key in ['states','commands','initial_previous_control','initial_physical_command']:assert got[key]==expected[key]
    for a,b in zip(got['controller_reference_selections'],expected['controller_reference_selections']):
        for key in ['selection','prediction_world','command','previous_control']:assert a[key]==b[key]
    assert len(m.instances)==2 and validate_execution_records(got,c,r,ad)==[]
    assert got['diagnostic_execution_only']==hard and got['deployment_candidate']==(not hard)
    assert not got['actual_deployment_performed']
    damaged=copy.deepcopy(got);damaged['states'][5]['pose_world'][0]+=.01
    assert validate_execution_records(damaged,c,r,ad)


def test_vector_geometry_is_vector_sum_not_sum_of_norms():
    r=endpoint_geometry([0,0,0],[.1,0,0],[.1,.03,0])
    assert r['vector_closure_error_m']==0 and r['angle_plan_track_rad']==pytest.approx(np.pi/2)
    assert r['execution_error_m']!=pytest.approx(r['plan_error_m']+r['tracking_displacement_m'])
    assert r['triangle_conditions_observed'] and r['execution_error_m']<.15
    assert endpoint_geometry([0,0],[0,0],[0,0])['angle_plan_track_rad'] is None


def test_exact_dedup_and_NA():
    ref=np.zeros((30,3));other=ref.copy();other[0,0]=np.nextafter(0.,1.)
    rows=[dict(reference_id=str(i),case_id='synthetic',state_identity_sha256='a',world_reference_sha256=array_hash(v),shape=[30,3],dtype='float64') for i,v in enumerate([ref,ref.copy(),other])]
    unique,aliases=deduplicate(rows)
    assert len(unique)==2 and aliases['0']==aliases['1'] and aliases['0']!=aliases['2']
    a=execution_admission(full(),{'planning_endpoint_reserve_pass':False},hard=True)
    assert classify(hard=True,admission=a,execution=None,historical_error=.17)=='MARGIN_PLAN_INFEASIBLE_OR_SOLVER_FAILURE'


@pytest.mark.parametrize('success,error,expected',[(True,.14,'MARGIN_EXECUTION_RECOVERY'),(False,.16,'MARGIN_IMPROVES_GOAL_BUT_NOT_SUCCESS'),(False,.18,'NO_MARGIN_EXECUTION_BENEFIT')])
def test_classification_does_not_promote_lateral_plan(success,error,expected):
    a=execution_admission(full(),{'planning_endpoint_reserve_pass':True},hard=True)
    e={'primary_success':success,'execution':{'terminal_position_error_m':error}}
    assert classify(hard=True,admission=a,execution=e,historical_error=.17)==expected
    assert not a['deployment_candidate']


def test_exclusive_outputs_and_single_callsite(tmp_path):
    import sys
    sys.path.insert(0,str(ROOT/'scripts'))
    from run_gp_se2_diag08 import prepare,write,solve,LABELS
    d=tmp_path/'exists';d.mkdir()
    with pytest.raises(FileExistsError):prepare(d)
    write(d/'a.json',{'missing':None})
    with pytest.raises(FileExistsError):write(d/'a.json',{})
    assert json.loads((d/'a.json').read_text())['missing'] is None
    calls=[n for n in ast.walk(ast.parse(inspect.getsource(solve))) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='run_refined']
    assert len(calls)==1 and len(LABELS)==5
    assert 'optimization_started.json' in inspect.getsource(solve)
    for path in ['scripts/validate_gp_se2_diag08.py','scripts/plot_gp_se2_diag08.py']:
        calls=[n.func.id for n in ast.walk(ast.parse((ROOT/path).read_text())) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)]
        assert not {'minimize','run_refined','counterfactual_rollout','historical_solve_audit','solve_gp','execute_reference'}.intersection(calls)


def test_png_numeric_sidecar_scope():
    import sys
    sys.path.insert(0,str(ROOT/'scripts'))
    from plot_gp_se2_diag08 import numeric,PLOTS
    value=dict(id='synthetic',case_id='test',classification='test',admission={},context={},
        variants={g:dict(provenance=g,reference=[[0,0,0]],actual=None,plan={},geometry=None,diagnostics=None,predictions=None,execution_flags=None) for g in ['G3','G4']})
    for name in PLOTS:
        n=numeric(name,value)
        assert n['variants']['G4']['provenance']=='G4'
        assert n['id']=='synthetic'

"""Independent saved-artifact corruption tests; no real trajectory optimization."""
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from reconciliation import gp_se2_diag02_solver
from reconciliation.gp_se2_diag_fixtures import make_fixture_problem

SCRIPT=Path(__file__).resolve().parents[1]/'scripts/validate_gp_se2_diag_02.py'
SPEC=importlib.util.spec_from_file_location('diag02_artifacts',SCRIPT)
validator=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(validator)


@pytest.fixture
def saved(monkeypatch):
    problem,fixture=make_fixture_problem('S1')
    def fake(fun,initial,**kw):
        fun(initial)
        for constraint in kw['constraints']:constraint['fun'](initial)
        kw['callback'](initial)
        return SimpleNamespace(x=initial.copy(),success=True,status=0,message='test fake',nit=1,nfev=1)
    monkeypatch.setattr(gp_se2_diag02_solver,'minimize',fake)
    result=gp_se2_diag02_solver.run_instrumented(problem,fixture['known_vector'],require_single_blas=False)
    result['blas_threads']['single_thread_environment_valid']=True
    result['blas_threads']['observed_runtime_single_thread_valid']=True
    return problem,result


def audit(problem,result):
    checks=validator.Checks();validator.validate_attempt(problem,result,'FD_BASELINE',checks,'test');return checks


def test_saved_vectors_reproduce_without_optimization(saved,monkeypatch):
    problem,result=saved
    monkeypatch.setattr(gp_se2_diag02_solver,'minimize',lambda *a,**k:pytest.fail('validator cannot optimize'))
    checks=audit(problem,result)
    assert not checks.errors,checks.errors
    assert checks.count>45


@pytest.mark.parametrize('kind,expected',[
    ('seed','bitwise frozen seed'),('candidate','exact selected candidate_world'),
    ('callback','exact original primal equality_residuals'),('objective','candidate original objective'),
    ('recovery','recovery distinct'),('tolerance','unchanged original formulation'),
    ('profile','primal cache accounting'),('derivative','untouched numerical derivative path'),
    ('selected','minimum feasible original objective selected')])
def test_corrupt_scientific_metadata_rejected(saved,kind,expected):
    problem,original=saved;result=copy.deepcopy(original)
    if kind=='seed':
        result['initial_vector'][0]+=.01;checks=validator.Checks()
        validator.validate_seed(result,original['initial_vector'],checks,'test')
    else:
        if kind=='candidate':result['candidate_world'][4,0]+=.1
        elif kind=='callback':result['callback_snapshots'][0]['equality_residuals'][0]+=.01
        elif kind=='objective':result['candidate_checks'][0]['objective']+=1
        elif kind=='recovery':result['recovered_from_infeasible']=True
        elif kind=='tolerance':result['config']['equality_tolerance']=1
        elif kind=='profile':result['profiling']['evaluate_requests']+=1
        elif kind=='derivative':result['derivative_calls']['objective_gradient']=1
        elif kind=='selected':result['selected_iterate']='nonexistent'
        checks=audit(problem,result)
    assert any(expected in error for error in checks.errors),checks.errors


def test_retained_seed_is_neither_recovery_nor_meaningful_improvement():
    problem,_=make_fixture_problem('S1');ftol=problem.config['ftol']
    flags=validator.outcome_flags(problem,True,False,True,4.,4.-ftol,False)
    assert not flags['infeasible_start_recovered']
    assert not flags['feasible_seed_objective_improved']
    assert not flags['solver_converged_with_full_feasibility']
    assert validator.outcome_flags(problem,False,True,True,4.,1.,True)['infeasible_start_recovered']
    assert validator.outcome_flags(problem,True,True,True,4.,1.,True)['feasible_seed_objective_improved']


def test_no_candidate_is_null_not_numeric_zero():
    problem,_=make_fixture_problem('S1')
    flags=validator.outcome_flags(problem,False,False,False,4.,None,False)
    assert flags['objective_reduction'] is None and flags['relative_objective_reduction'] is None
    assert not flags['infeasible_start_recovered']
    assert not validator.equivalent(None,0)


@pytest.mark.parametrize('corrupt',['numbers','image','source'])
def test_plot_corruption_detected(tmp_path,corrupt):
    image=tmp_path/'plot.png';image.write_bytes(b'test image')
    source=tmp_path/'source.json';source.write_text('{}')
    numbers={'value':[None,1.]}
    side=dict(image=image.name,image_sha256=validator.digest(image),dpi=160,no_new_execution=True,
              numeric_data=copy.deepcopy(numbers),source_sha256={str(source):validator.digest(source)})
    if corrupt=='numbers':side['numeric_data']['value'][0]=0.
    elif corrupt=='image':image.write_bytes(b'changed')
    else:source.write_text('{"changed":true}')
    checks=validator.Checks();validator.validate_plot_sidecar(image,side,numbers,checks,'plot')
    assert checks.errors


def test_output_is_exclusive_and_input_hash_sensitive(tmp_path):
    path=tmp_path/'validation.json';validator.write_exclusive(path,{'valid':True})
    before=path.read_bytes();checks=validator.Checks()
    validator.verify_digest(path,validator.digest(path),checks,'same')
    assert not checks.errors
    with pytest.raises(FileExistsError):validator.write_exclusive(path,{'valid':False})
    assert path.read_bytes()==before
    validator.verify_digest(path,'0'*64,checks,'changed')
    assert checks.errors==['changed']


def test_csv_cannot_turn_missing_candidate_into_zero(tmp_path):
    path=tmp_path/'comparison.csv';path.write_text('method,objective,valid\nM2,0,False\n')
    checks=validator.Checks()
    validator.validate_csv_rows(path,[dict(method='M2',objective=None,valid=False)],checks,'csv')
    assert checks.errors==['csv: source row 0/objective']

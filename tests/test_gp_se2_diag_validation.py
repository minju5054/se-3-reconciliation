"""Saved-artifact corruption tests; no optimization or experimental rerun."""
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from reconciliation import gp_se2_diag_solver
from reconciliation.gp_se2_diag_fixtures import make_fixture_problem

SCRIPT=Path(__file__).resolve().parents[1]/'scripts/validate_gp_se2_diag_01.py'
SPEC=importlib.util.spec_from_file_location('validate_gp_se2_diag_01',SCRIPT)
validator=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(validator)


@pytest.fixture
def saved_feasible(monkeypatch):
    problem,fixture=make_fixture_problem('S1')

    def fake_minimize(fun,initial,**kw):
        fun(initial)
        for constraint in kw['constraints']:
            constraint['fun'](initial)
        kw['callback'](initial)
        return SimpleNamespace(x=initial.copy(),success=True,status=0,message='synthetic fake optimizer',nit=1,nfev=1)

    monkeypatch.setattr(gp_se2_diag_solver,'minimize',fake_minimize)
    result=gp_se2_diag_solver.run_instrumented(problem,fixture['known_vector'],require_single_blas=False)
    # This test does not assert runtime thread policy. A production run must
    # report real observed OpenBLAS counts; here this field is test metadata.
    result['blas_threads']['single_thread_environment_valid']=True
    result['blas_threads']['observed_runtime_single_thread_valid']=True
    return problem,result


def run_attempt(problem,result):
    checks=validator.Checks()
    validator.validate_attempt(problem,result,checks,'test')
    return checks


def test_saved_original_feasible_attempt_revalidates_without_solver(saved_feasible,monkeypatch):
    problem,result=saved_feasible
    monkeypatch.setattr(gp_se2_diag_solver,'minimize',lambda *a,**k:pytest.fail('validator must never optimize'))
    checks=run_attempt(problem,result)
    assert not checks.errors
    assert checks.count>40


@pytest.mark.parametrize('corruption,expected',[
    ('pose','exact candidate_world'),
    ('residual','equality'),
    ('recovery','recovery semantics'),
    ('tolerance','unchanged formulation'),
    ('objective','candidate objective'),
    ('profile','cache accounting'),
    ('budget','budget cost includes initialization'),
])
def test_corruption_is_detected(saved_feasible,corruption,expected):
    problem,result=saved_feasible
    result=copy.deepcopy(result)
    if corruption=='pose':
        result['candidate_world'][3,0]+=.1
    elif corruption=='residual':
        result['callback_snapshots'][0]['equality_residuals'][0]+=.01
    elif corruption=='recovery':
        result['recovered_from_infeasible']=True
    elif corruption=='tolerance':
        result['config']['equality_tolerance']=.1
    elif corruption=='objective':
        result['candidate_checks'][0]['objective']+=1
    elif corruption=='profile':
        result['profiling']['evaluate_requests']+=1
    elif corruption=='budget':
        result['solve_and_pre_solve_wall_time_s']+=1
    checks=run_attempt(problem,result)
    assert any(expected in error for error in checks.errors),checks.errors


def test_null_boolean_and_shape_are_not_silently_numeric_zero():
    assert not validator.equivalent(None,0)
    assert not validator.equivalent(False,0)
    assert not validator.equivalent([None],[0.])
    assert not validator.equivalent(np.zeros((1,2)),np.zeros((2,1)))
    assert not validator.equivalent(float('nan'),float('nan'))
    assert validator.equivalent(np.array([1.,2.]),[1.,2.])


def test_output_write_is_exclusive(tmp_path):
    target=tmp_path/'validation.json'
    validator.write_exclusive(target,{'valid':True})
    before=target.read_bytes()
    with pytest.raises(FileExistsError):
        validator.write_exclusive(target,{'valid':False})
    assert target.read_bytes()==before


def test_hash_checker_detects_changed_historical_file(tmp_path):
    historical=tmp_path/'historical.txt'
    historical.write_text('immutable input')
    expected=validator.digest(historical)
    record=dict(valid=True,historical_files={str(historical):expected},external_files={},
                external_git_sha='same',external_git_status='',user_config_hashes={})
    run=tmp_path/'new_run';run.mkdir()
    (run/'preservation_before.json').write_text(json.dumps(record))
    (run/'preservation_after.json').write_text(json.dumps(record))
    source=dict(core_numerical_source_sha256={},current_full_retained_data_rehash_valid=True,
                source_validation_valid=True,source_hash_validation_valid=True)
    historical.write_text('changed after snapshots')
    checks=validator.Checks()
    validator.validate_preservation(run,source,checks)
    assert any('current '+str(historical) in error for error in checks.errors)


def test_missing_callbacks_are_not_fabricated(saved_feasible):
    problem,result=saved_feasible
    result['callback_snapshots'][0]['iteration']=3
    checks=run_attempt(problem,result)
    assert any('actual ordered callbacks' in error for error in checks.errors)


def test_error_section_is_failed_not_reported_as_zero_success():
    checks=validator.Checks()
    checks.section('missing data',lambda:(_ for _ in ()).throw(FileNotFoundError('no source')))
    assert checks.count==1
    assert checks.errors==['missing data: FileNotFoundError: no source']


@pytest.mark.parametrize('fixture_name',['S0','S1','S2'])
@pytest.mark.parametrize('kind',['known','perturbed'])
def test_synthetic_summary_kind_maps_to_full_attempt_name(fixture_name,kind):
    assert validator.fixture_initialization_matches(fixture_name,kind,fixture_name+'_'+kind)
    assert not validator.fixture_initialization_matches(fixture_name,kind,fixture_name+'_other')
    assert not validator.fixture_initialization_matches(fixture_name,kind,'S9_'+kind)
    assert not validator.fixture_initialization_matches(fixture_name,'unknown',fixture_name+'_unknown')


@pytest.mark.parametrize('phase',['baseline','single_change'])
@pytest.mark.parametrize('method',['M2_GP_NO_OBSTACLE','M3_GP_CONSTRAINED'])
@pytest.mark.parametrize('index',[0,1])
def test_relative_and_absolute_provenance_resolve_to_same_artifact(tmp_path,monkeypatch,phase,method,index):
    monkeypatch.setattr(validator,'ROOT',tmp_path)
    relative=f'data/new_run/actual_event/{phase}/{method}/start_{index:02d}.json'
    expected=tmp_path/relative
    assert validator.source_path_matches(str(expected),expected)
    assert validator.source_path_matches(relative,expected)
    assert not validator.source_path_matches(relative.replace('new_run','old_run'),expected)

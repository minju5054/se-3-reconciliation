"""Saved-artifact corruption tests, using synthetic fixtures only."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import zipfile

import numpy as np
import pytest

from reconciliation import gp_se2_formulation
from reconciliation.gp_se2_diag_fixtures import make_fixture_problem

SCRIPT=Path(__file__).resolve().parents[1]/'scripts/validate_gp_se2_02.py'
SPEC=importlib.util.spec_from_file_location('gp02_artifact_validator',SCRIPT)
validator=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(validator)


def retained(cost, vector, feasible=True):
    return dict(candidate_found=True,candidate_vector=vector,candidate_objective=cost,
        candidate_world=np.zeros((30,3)),support_poses=np.zeros((31,3)),support_twists=np.zeros((31,3)),
        selected_iterate='initial',constraint_report={'feasible':True},full_feasible=feasible)


def aggregate(result,index):
    return dict(result,selected_initialization=index,fallback_used=False)


def test_lower_cost_dense_candidate_not_swapped_after_full_failure():
    a,b=retained(1.,np.zeros(150),False),retained(2.,np.ones(150),True)
    checks=validator.Checks()
    assert validator.validate_method_selection([a,b],aggregate(a,0),checks,'selection')==0
    assert not checks.errors
    checks=validator.Checks()
    validator.validate_method_selection([a,b],aggregate(b,1),checks,'selection')
    assert any('minimum-dense-cost' in e for e in checks.errors)


def test_exact_seed_change_is_detected():
    checks=validator.Checks();saved=np.zeros(150);changed=saved.copy();changed[0]=1e-13
    validator.validate_seed({'initial_vector':changed},saved,checks,'seed')
    assert checks.errors==['seed: bitwise frozen seed']


@pytest.mark.parametrize('corruption',['candidate','cost','source','fallback'])
def test_corrupt_method_selection_rejected(corruption):
    a,b=retained(1.,np.zeros(150)),retained(2.,np.ones(150))
    result=deepcopy(aggregate(a,0))
    if corruption=='candidate':result['candidate_world'][4,0]=.1
    elif corruption=='cost':result['candidate_objective']=.1
    elif corruption=='source':result['selected_iterate']='invented_callback'
    else:result['fallback_used']=True
    checks=validator.Checks();validator.validate_method_selection([a,b],result,checks,'selection')
    assert checks.errors


def unsupported():
    result=dict(initial_vector=np.zeros(150),solver_executed=False,candidate_found=False,
        termination='DERIVATIVE_UNSUPPORTED',candidate_vector=None,candidate_world=None,candidate_objective=None,
        latest_iterate=None,callback_snapshots=[],candidate_checks=[])
    return result,dict(selected_full_feasible=False,final_full_feasible=None)


def test_unsupported_start_is_preserved_without_fake_execution():
    result,summary=unsupported();checks=validator.Checks()
    validator.validate_unsupported(result,np.zeros(150),summary,checks,'unsupported')
    assert not checks.errors
    result['candidate_world']=np.zeros((30,3));summary['selected_full_feasible']=True
    checks=validator.Checks();validator.validate_unsupported(result,np.zeros(150),summary,checks,'unsupported')
    assert len(checks.errors)==2


def test_runtime_guard_effective_status_does_not_rewrite_raw_harness():
    result=dict(termination='NUMERICAL_FAILURE',effective_termination='DERIVATIVE_UNSUPPORTED',
        recorded_derivative_errors=[dict(reason_code='UNSUPPORTED_WRAP_CUT',vector=np.zeros(150))])
    checks=validator.Checks();validator.validate_runtime_failure(result,checks,'runtime')
    assert not checks.errors
    result['termination']='CONVERGED';checks=validator.Checks();validator.validate_runtime_failure(result,checks,'runtime')
    assert any('raw harness' in e for e in checks.errors)


def test_partial_derivative_calls_only_allowed_for_actual_guard_failure():
    calls=dict(objective_gradient=1,equality_jacobian=0,inequality_jacobian=0)
    for errors,expected in [([],False),([{'reason_code':'UNSUPPORTED_WRAP_CUT'}],True)]:
        checks=validator.Checks();proxy=validator.RuntimeGuardChecks(checks,dict(derivative_calls=calls,recorded_derivative_errors=errors))
        proxy.check(False,'example: all three supplied callbacks used')
        assert (not checks.errors)==expected


@pytest.fixture
def rigid_case(monkeypatch):
    problem,_=make_fixture_problem('S1')
    def fake(fun,initial,**kwargs):
        fun(initial)
        kwargs['callback'](initial)
        return SimpleNamespace(x=initial.copy(),success=True,status=0,message='synthetic mocked solver',nit=1,nfev=1)
    monkeypatch.setattr(gp_se2_formulation,'minimize',fake)
    result=gp_se2_formulation.solve_rigid(problem.boundary_pose,problem.initial_twist,problem.common_reference,
        problem.goal_pose,problem.config,problem.obstacle_clearance,problem.gates,problem.workspace_margin)
    monkeypatch.setattr(gp_se2_formulation,'minimize',lambda *a,**kw:pytest.fail('artifact validator must not optimize'))
    return {'problem':problem},result


def test_original_rigid_values_and_two_start_selection_reproduced_without_solve(rigid_case):
    case,result=rigid_case;checks=validator.Checks();validator.validate_rigid(case,result,checks,'rigid')
    assert not checks.errors,checks.errors
    assert checks.count>20


@pytest.mark.parametrize('change',['initialization','cost','candidate','selection'])
def test_rigid_artifact_corruption_detected(rigid_case,change):
    case,result=rigid_case;result=deepcopy(result)
    if change=='initialization':result['attempts'][1]['initial_vector'][0]+=.2
    elif change=='cost':result['attempts'][0]['candidate_checks'][0]['objective']+=1.
    elif change=='candidate':result['candidate_world'][5,0]+=.1
    else:result['selected_iterate']='invented'
    checks=validator.Checks();validator.validate_rigid(case,result,checks,'rigid')
    assert checks.errors


def test_elapsed_only_removal_preserves_execution_values():
    a=dict(primary_success=False,execution={'terminal_position_error_m':.1},timing={'evaluation_wall_s':2.,'rollout_wall_s':3.},independent_rollout_evaluation_wall_s=2.1)
    b=dict(a,timing={'evaluation_wall_s':8.,'rollout_wall_s':3.},independent_rollout_evaluation_wall_s=9.)
    assert validator.equivalent(validator.without_elapsed(a),validator.without_elapsed(b))
    b['execution']={'terminal_position_error_m':0.}
    assert not validator.equivalent(validator.without_elapsed(a),validator.without_elapsed(b))


def test_csv_uses_original_json_cells_and_missing_heterogeneous_columns(tmp_path):
    from run_gp_se2_01 import table
    rows=[dict(method='rigid',failure_reasons=['goal_failure'],deformation={'z':1.,'a':None}),
          dict(method='GP',initial_full_feasible=False,deformation=None)]
    path=tmp_path/'table.csv';table(path,rows)
    checks=validator.Checks();validator.validate_csv_rows(path,rows,checks,'csv')
    assert not checks.errors,checks.errors
    changed=deepcopy(rows);changed[0]['failure_reasons']=[]
    checks=validator.Checks();validator.validate_csv_rows(path,changed,checks,'csv')
    assert checks.errors==['csv: source row 0/failure_reasons']


@pytest.mark.parametrize('kind',['numeric','image','source','label'])
def test_plot_payload_or_provenance_corruption_detected(tmp_path,kind):
    image=tmp_path/'plot.png';image.write_bytes(b'test')
    source=tmp_path/'inputs.json';source.write_text('{}')
    side=dict(image='plot.png',image_sha256=validator.digest(image),dpi=160,
        label='OFFLINE COUNTERFACTUAL HARD-HANDOFF COMPARISON',new_inference=False,new_solver=False,new_execution=False,
        numeric_data={'unavailable':None,'value':[1.]},source_hashes={str(source):validator.digest(source)},
        plotter_sha256=validator.digest(validator.ROOT/'scripts/plot_gp_se2_02.py'))
    expected=deepcopy(side['numeric_data'])
    if kind=='numeric':side['numeric_data']['unavailable']=0.
    elif kind=='image':image.write_bytes(b'changed')
    elif kind=='source':source.write_text('{"changed":true}')
    else:side['label']='LIVE EXECUTION'
    checks=validator.Checks();validator.validate_plot_record(image,side,expected,checks,'plot')
    assert checks.errors


def test_review_allowlist_rejects_added_raw_array_even_if_hashed(tmp_path):
    bundle=tmp_path/'review_bundle';bundle.mkdir()
    records=[]
    for name in ('README.md','raw.npy'):
        (bundle/name).write_bytes(b'fixture')
        records.append(dict(path=name,sha256=validator.digest(bundle/name),bytes=7,source='generated review text'))
    (bundle/'manifest.json').write_text(json.dumps({'allowlisted_files':records}))
    with zipfile.ZipFile(tmp_path/'review_bundle.zip','w') as archive:
        for path in bundle.iterdir():archive.write(path,path.name)
    checks=validator.Checks();validator.validate_bundle_allowlist(bundle,{'README.md'},checks)
    assert 'exact compact review allowlist' in checks.errors
    assert 'review ZIP exact allowlist and CRC' in checks.errors


def test_validation_report_write_is_exclusive(tmp_path):
    path=tmp_path/'validation.json';validator.write_exclusive(path,{'valid':True})
    with pytest.raises(FileExistsError):validator.write_exclusive(path,{'valid':False})
    assert json.loads(path.read_text())['valid']


def test_completion_preserves_frozen_base_package_and_adds_final_status_only(tmp_path):
    import complete_gp_se2_02_review as complete
    run=tmp_path/'synthetic_run';run.mkdir()
    case='synthetic__not_evidence'
    (run/'case_manifest.json').write_text(json.dumps({'selected':[{'case_id':'synthetic/not_evidence','case_directory':case}]}))
    (run/'source.json').write_text('{"synthetic_fixture":true}')
    plots=run/'cases'/case/'plots';plots.mkdir(parents=True)
    for name in ('candidate_world_overlay','actual_rollout_overlay','boundary_zoom','candidate_feasibility_summary'):
        for ext in ('.png','.json'):(plots/(name+ext)).write_bytes(b'synthetic test fixture; not evidence')
    aggregate=run/'aggregate';aggregate.mkdir()
    (aggregate/'outcome_matrix.csv').write_text('synthetic_fixture\nTrue\n')
    (aggregate/'compute_summary.json').write_text('{"synthetic_fixture":true}')
    (run/'final_report.json').write_text(json.dumps(dict(experiment='GP-SE2-02',operational_status='GP_SE2_02_COMPLETED_WITH_LIMITATIONS',execution_interpretation='INSUFFICIENT_EXECUTION_EVIDENCE')))
    (run/'raw.npy').write_bytes(b'must not be bundled')
    (run/'history_supplement_manifest.json').write_text(json.dumps({'images':[]}))
    narrative=tmp_path/'report.md';narrative.write_text('Synthetic fixture only; no experimental results.')
    source_before=(run/'source.json').read_bytes()
    result=complete.complete(run,narrative)
    assert (run/'source.json').read_bytes()==source_before
    assert result['bundle_sha256']==validator.digest(run/'review_bundle.zip')
    saved=json.loads((run/'review_completion.json').read_text())
    assert saved['base_package_sha256']==validator.digest(run/'review_base/review_bundle.zip')
    with zipfile.ZipFile(run/'review_base/review_bundle.zip') as old,zipfile.ZipFile(run/'review_bundle.zip') as final:
        assert 'final_report.json' not in old.namelist()
        assert {'final_report.json','RESULTS.md','aggregate/outcome_matrix.csv','aggregate/compute_summary.json'} <= set(final.namelist())
        assert 'raw.npy' not in final.namelist()
        assert final.read('RESULTS.md')==narrative.read_bytes()
        assert b'interim operational status' in final.read('README.md')
    with pytest.raises(FileExistsError):complete.complete(run,narrative)

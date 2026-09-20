"""Synthetic implementation checks only; no new actual MPC/GP experiments."""
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from reconciliation.gp_se2_diag07_lateral_execution import (
    HARD,BENIGN,REQUIRED_FLAGS,plan_gate,support_reference,array_hash,deduplicate,
    diagnostic_rollout,validate_rollout_records,experiment_interpretation,execution_diagnostics)
from reconciliation.gp_se2_rollout import counterfactual_rollout,candidate_in_capture_frame,execution_metrics
from reconciliation.se2 import local_trajectory_to_world
from test_gp_se2_rollout import mock_module


def full(hard=True):
    flags={k:True for k in REQUIRED_FLAGS};flags['lateral_velocity']=not hard
    return dict(additional_grid=dict(flags=flags),full_feasible=not hard,
                original_plan_report=dict(motion=dict(violations={'lateral_velocity':hard,'linear_speed':False})))


def test_lateral_only_gate_never_promotes():
    a=plan_gate(full(),hard=True)
    assert not a['plan_valid'] and not a['deployment_candidate']
    assert a['diagnostic_execution_only'] and a['plan_failure_reason']=='lateral_velocity'
    assert a['selected_source_in_DIAG06'] is None


@pytest.mark.parametrize('flag',sorted(REQUIRED_FLAGS-{'lateral_velocity'}))
def test_any_additional_plan_failure_blocks_primary(flag):
    f=full();f['additional_grid']['flags'][flag]=False
    with pytest.raises(ValueError,match='NOT_LATERAL_ONLY'):plan_gate(f,hard=True)


def test_missing_flags_and_benign_seed_return_rejected():
    f=full();del f['additional_grid']['flags']['original_route']
    with pytest.raises(ValueError,match='coverage'):plan_gate(f,hard=True)
    with pytest.raises(ValueError,match='BENIGN'):plan_gate(full(False),hard=False,changed_from_seed=False)
    assert plan_gate(full(False),hard=False)['plan_valid']


def test_latest_support_extraction_does_not_use_selected():
    poses=np.arange(93,dtype=float).reshape(31,3);twists=poses*.1;z=np.arange(150,dtype=float)
    saved=dict(latest_iterate=z.tolist(),latest_support_poses=poses.tolist(),latest_support_twists=twists.tolist(),candidate_vector=None)
    problem=SimpleNamespace(unpack=lambda x:(poses,twists))
    zz,p,v,r=support_reference(problem,saved)
    assert np.array_equal(r,poses[1:]) and np.array_equal(zz,z)
    saved['latest_support_poses'][0][0]+=1
    with pytest.raises(ValueError,match='RECONSTRUCTION'):support_reference(problem,saved)


def test_dedup_exact_not_approximate_and_state_sensitive():
    a=np.zeros((30,3));b=a.copy();b[0,0]=np.nextafter(0.,1.)
    rows=[dict(reference_id=str(k),case_id=HARD,state_identity_sha256=state,world_reference_sha256=array_hash(v),shape=[30,3],dtype='float64')
          for k,(state,v) in enumerate([('a',a),('a',a.copy()),('a',b),('b',a)])]
    unique,alias=deduplicate(rows)
    assert len(unique)==3 and alias['0']==alias['1'] and alias['2']!=alias['0']
    assert unique[0]['provenance_labels']==['0','1']


def fixture():
    frozen=dict(case_id='synthetic/only',B_world=[3.,-2.,2.7],u_minus=[.1,.2],previous_control=[.3,-.1],
                original_capture_pose_world=[2.,-3.,-2.9],source_files=[])
    ref=np.array([[3.1,-1.9,2.8],[3.4,-1.8,-3.1],[3.5,-1.7,-2.9]])
    m=mock_module();m.project_body_to_world=lambda path,anchor:local_trajectory_to_world(anchor,path)
    return m,frozen,ref


def test_wrapper_is_unchanged_rollout_parity_no_optimizer(monkeypatch):
    import scipy.optimize
    monkeypatch.setattr(scipy.optimize,'minimize',lambda *a,**k:pytest.fail('optimizer forbidden'))
    m,c,r=fixture();a=diagnostic_rollout(m,c,r,plan_gate(full(),hard=True));b=counterfactual_rollout(m,c,r)
    for key in ['states','commands','initial_pose_world','initial_physical_command','initial_previous_control']:
        assert a[key]==b[key]
    for x,y in zip(a['controller_reference_selections'],b['controller_reference_selections']):
        for key in ['input_pose_world','previous_control','command','selection','prediction_world']:assert x[key]==y[key]
    assert 'candidate_world' not in a and 'candidate_capture_local' not in a
    assert not a['plan_valid'] and len(m.instances)==2 and all(i.closed for i in m.instances)
    assert validate_rollout_records(a,c,r)==[]
    assert len(a['states'])==181 and len(a['commands'])==180 and len(a['controller_reference_selections'])==30
    assert np.array_equal(a['reference_world'],r)
    assert a['actual_lateral_command_dimension'].startswith('nonexistent')


@pytest.mark.parametrize('mutation',['state','memory','reference','schedule','selection'])
def test_saved_record_validation_detects_corruption(mutation):
    m,c,r=fixture();a=diagnostic_rollout(m,c,r,plan_gate(full(),hard=True))
    if mutation=='state':a['states'][5]['pose_world'][0]+=.01
    if mutation=='memory':a['controller_reference_selections'][0]['previous_control'][0]+=.01
    if mutation=='reference':a['reference_world'][0][0]+=.01
    if mutation=='schedule':a['commands'][0]['end_time_s']=.5
    if mutation=='selection':a['controller_reference_selections'][0]['selection']['indices'][0]=0
    assert validate_rollout_records(a,c,r)


def test_capture_transform_not_boundary_and_no_lateral_feedforward():
    m,c,r=fixture();a=diagnostic_rollout(m,c,r,plan_gate(full(),hard=True));local,_=candidate_in_capture_frame(r,c['original_capture_pose_world'])
    assert np.array_equal(local,a['reference_capture_local'])
    assert a['initial_pose_world']==c['B_world'] and not a['GP_body_velocity_feed_forward']
    assert a['initial_previous_control']!=a['initial_physical_command']
    wrong=local_trajectory_to_world(c['B_world'],local)
    assert not np.allclose(wrong[:,:2],r[:,:2])


@pytest.mark.parametrize('outcomes,expected',[
    ([True,True,True],'STRICT_PLAN_LATERAL_REJECTION_CONSERVATIVE_ON_FIXED_EVENT'),
    ([True,False,True],'MIXED_LATERAL_EXECUTION_RESULT'),
    ([False,False,True],'LATERAL_INVALID_REFERENCES_EXECUTION_FAIL'),
    ([True,True,False],'EXECUTION_HARNESS_OR_CONTROL_REGRESSION')])
def test_interpretation_never_claims_lateral_causality(outcomes,expected):
    rows=[dict(case_id=c,primary_success=v) for c,v in zip([HARD,HARD,BENIGN],outcomes)]
    assert experiment_interpretation(rows)==expected


def test_missing_goal_time_preserved_and_no_causal_failure_claim():
    m,c,r=fixture();a=diagnostic_rollout(m,c,r,plan_gate(full(),hard=True));goal=[100.,100.,0.]
    metrics=execution_metrics(a,goal)
    e=dict(execution=metrics,goal=goal,dense_times_s=[0.,3.],dense_poses_world=[c['B_world'],a['states'][-1]['pose_world']],
           environment=dict(clearance_valid=True),route=dict(valid=True))
    e['environment']['workspace_known']=True
    d=execution_diagnostics(a,e,c)
    assert metrics['time_to_goal_s'] is None and d['first_goal_entry_s'] is None
    assert 'EXECUTION_GOAL_FAILURE' in d['failure_taxonomy'] and 'EXECUTION_DWELL_FAILURE' in d['failure_taxonomy']
    assert d['cause_attribution'].startswith('not identified')


def test_exclusive_output_no_retry(tmp_path):
    import sys
    scripts=Path(__file__).resolve().parents[1]/'scripts';sys.path.insert(0,str(scripts))
    from run_gp_se2_ref01 import write
    from run_gp_se2_diag07 import prepare
    path=tmp_path/'already';path.mkdir()
    with pytest.raises(FileExistsError):prepare(path)
    write(path/'test.json',dict(missing=None))
    with pytest.raises(FileExistsError):write(path/'test.json',{})
    assert json.loads((path/'test.json').read_text())['missing'] is None


def test_no_new_optimization_or_selector_implementation():
    root=Path(__file__).resolve().parents[1]
    sources=[root/'src/reconciliation/gp_se2_diag07_lateral_execution.py',root/'scripts/run_gp_se2_diag07.py',root/'scripts/lightnav/gp_se2_diag07_mpc_rollout.py']
    import ast
    forbidden={'minimize','run_refined','solve_gp','solve_rigid','run_instrumented','DerivativeProvider'}
    for path in sources:
        calls=[n.func.id for n in ast.walk(ast.parse(path.read_text())) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)]
        assert not forbidden.intersection(calls)
    text=sources[-1].read_text()
    assert "mkdir(exist_ok=False)" in text and 'for record in request' in text
    assert 'load_official(EXTERNAL)' in text and 'historical_solve_audit(module,frozen)' in text

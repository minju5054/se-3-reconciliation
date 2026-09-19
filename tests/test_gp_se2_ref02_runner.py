"""Frozen design and outcome classification tests; fixtures are not evidence."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('ref02_runner', ROOT/'scripts/run_gp_se2_ref02.py')
runner = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(runner)
from reconciliation import gp_se2_ref02_evaluation as evaluate


def config():
    return yaml.safe_load((ROOT/'configs/gp_se2_ref02.yaml').read_text())


def test_frozen_two_case_three_method_design_and_same_dense_source():
    c = runner.validate_config(config())
    assert [x['case_id'] for x in c['cases']] == list(evaluate.FIXED_CASES)
    assert c['methods'] == list(evaluate.METHODS)
    assert c['source_variants']['B_DENSE_ROW_STEP'] == c['source_variants']['C_DENSE_SOURCE_PROGRESS']
    assert runner.SELECTOR_DEFINITION['reference_selector_changed']
    assert runner.SELECTOR_DEFINITION['MPC_calculation_unchanged']
    assert not runner.SELECTOR_DEFINITION['whole_controller_unchanged_claim']


@pytest.mark.parametrize('section,key,value', [
    ('selector','source_progress_stride', .5), ('selector','source_progress_stride', 2.),
    ('selector','searchsorted_side','right'), ('selector','search_tolerance',1e-8),
    ('selector','round_nearest_progress',True), ('selector','accumulate_overshoot',True),
    ('selector','interpolate_targets',True), ('selector','progress_memory',True),
    ('rollout','horizon_s',4.), ('rollout','control_hz',20.), ('rollout','integration_hz',120.),
    ('mpc','HORIZON',6), ('mpc','MPC_DT_S',.2), ('mpc','Q_WEIGHTS',[1.,1.,1.]),
    ('source_variants','C_DENSE_SOURCE_PROGRESS','R00_NATIVE')])
def test_reject_extra_intervention(section,key,value):
    c=config();c[section][key]=value
    with pytest.raises(ValueError):runner.validate_config(c)


@pytest.mark.parametrize('key', ['new_vla_inference','gp_or_rigid_optimization','new_reference_interpolation','retry'])
def test_scope_is_diagnostic_only(key):
    c=config();c[key]=True
    with pytest.raises(ValueError):runner.validate_config(c)


def test_case_method_order_not_swappable():
    for field in ('cases','methods'):
        c=config();c[field].reverse()
        with pytest.raises(ValueError):runner.validate_config(c)


def test_exclusive_writes_and_no_freeze_no_retry(tmp_path,monkeypatch):
    runner.write(tmp_path/'record.json',{'first':True})
    before=runner.digest(tmp_path/'record.json')
    with pytest.raises(FileExistsError):runner.write(tmp_path/'record.json',{'first':False})
    assert runner.digest(tmp_path/'record.json')==before
    monkeypatch.setattr(runner,'verify',lambda _:(config(),{}))
    with pytest.raises(ValueError,match='freeze'):runner.execute(tmp_path)
    runner.write(tmp_path/'execution_freeze.json',{})
    runner.write(tmp_path/'execution_started.json',{})
    with pytest.raises(FileExistsError,match='no retry'):runner.execute(tmp_path)


def test_frozen_core_or_input_change_blocks_run(tmp_path):
    c=config();(tmp_path/'experiment_config.yaml').write_text(yaml.safe_dump(c))
    (tmp_path/'config_snapshot.yaml').write_text('original configuration')
    core=tmp_path/'math.py';core.write_text('original')
    original=tmp_path/'reference.npy';original.write_bytes(b'original reference')
    runner.write(tmp_path/'source.json',dict(original_config_sha256=runner.digest(tmp_path/'config_snapshot.yaml'),
        experiment_config_sha256=runner.digest(tmp_path/'experiment_config.yaml'),
        copied_source_sha256={str(original):runner.digest(original)},preserved_core_sha256={str(core):runner.digest(core)}))
    runner.verify(tmp_path)
    runner.write(tmp_path/'execution_freeze.json',dict(source_sha256={},input_sha256={'reference.npy':runner.digest(original)}))
    original.write_bytes(b'changed')
    with pytest.raises(ValueError,match='input drift'):runner.verify(tmp_path)


def synthetic_outcomes():
    return [dict(case_id=c,method=m,primary_success=False,failure_reasons=['GOAL_YAW_FAILURE'],
                 **{k:1. for k in evaluate.CONTRAST_METRICS})
            for c in evaluate.FIXED_CASES for m in evaluate.METHODS]


def test_paired_missing_values_are_not_zero_and_all_failures_preserved():
    rows=synthetic_outcomes();rows[1]['time_to_goal_s']=None
    pairs=evaluate.paired_comparisons(rows)
    assert len(pairs)==2*3*len(evaluate.CONTRAST_METRICS)
    q=next(x for x in pairs if x['case_id']==rows[1]['case_id'] and x['metric']=='time_to_goal_s' and x['baseline']==rows[1]['method'])
    assert q['baseline_value'] is None and q['difference'] is None and not q['both_success']
    with pytest.raises(ValueError):evaluate.paired_comparisons(rows[:-1])


def flag_inputs():
    return (synthetic_outcomes(),{c:dict(passed=True) for c in evaluate.FIXED_CASES},
        [dict(baseline='B_DENSE_ROW_STEP',same_nearest_index_and_source_progress=True,same_all_nearest_costs=True) for _ in range(60)])


def test_lower_yaw_failure_is_partial_not_full_recovery():
    rows,repro,matches=flag_inputs();rows[2]['terminal_yaw_error_rad']=.2
    flags=evaluate.result_flags(rows,repro,matches,fixed_dense_preserved=True,selection_validated=True)
    assert flags['large_turn_result']=='PARTIAL_RECOVERY'
    assert not flags['large_turn_success_recovered']


@pytest.mark.parametrize('missing', ['baseline','geometry','nearest','selection'])
def test_recovery_not_confirmed_without_all_intervention_preconditions(missing):
    rows,repro,matches=flag_inputs();rows[2].update(primary_success=True,failure_reasons=[])
    if missing=='baseline':repro[evaluate.FIXED_CASES[0]]['passed']=False
    if missing=='nearest':matches[0]['same_nearest_index_and_source_progress']=False
    flags=evaluate.result_flags(rows,repro,matches,fixed_dense_preserved=missing!='geometry',selection_validated=missing!='selection')
    assert not flags['large_turn_success_recovered']
    assert flags['large_turn_result']=='INTERVENTION_NOT_CONFIRMED'


def test_safety_tradeoff_not_mislabeled_full_recovery():
    rows,repro,matches=flag_inputs();rows[2]['terminal_yaw_error_rad']=.01;rows[2]['failure_reasons']=['MOTION_VIOLATION']
    flags=evaluate.result_flags(rows,repro,matches,fixed_dense_preserved=True,selection_validated=True)
    assert flags['large_turn_result']=='REGRESSION_OR_TRADEOFF' and flags['new_safety_or_motion_regression']
    assert not flags['large_turn_success_recovered']


def test_success_boolean_preserved_separately_from_small_goal_metric():
    rows,repro,matches=flag_inputs();rows[2].update(primary_success=True,failure_reasons=[])
    flags=evaluate.result_flags(rows,repro,matches,fixed_dense_preserved=True,selection_validated=True)
    assert flags['large_turn_success_recovered']
    assert not flags['benign_success_preserved']


def test_matched_nearest_mismatch_is_error_not_target_difference():
    d=dict(nearest_index=0,nearest_original_fractional_row_coordinate=1.,weighted_total_pose_distances=[0.,1.],
           reference_world=np.zeros((5,3)).tolist(),selected_original_fractional_row_coordinates=[1.]*5,
           final_goal_row_in_horizon=False,indices=[0]*5)
    rows=[dict(time_s=i/10,methods={m:deepcopy(d) for m in evaluate.METHODS}) for i in range(30)]
    values=evaluate.compare_selections('synthetic',rows,matched=True)
    assert not any(x['selection_changed'] for x in values)
    rows[0]['methods']['C_DENSE_SOURCE_PROGRESS']['nearest_index']=1
    with pytest.raises(ValueError,match='nearest'):evaluate.compare_selections('synthetic',rows,matched=True)


def test_command_divergence_uses_frozen_tolerance_not_nonzero_roundoff():
    records={m:[dict(time_s=i/10,command=[0.,0.]) for i in range(30)] for m in evaluate.METHODS}
    records[evaluate.METHODS[2]][0]['command'][1]=1e-8
    records[evaluate.METHODS[2]][2]['command'][1]=1e-3
    rows=evaluate.first_command_divergences('synthetic',records)
    assert rows[0]['first_divergent_command_time_s']==.2
    assert rows[0]['first_divergent_linear_command_time_s'] is None

"""Corruption tests for REF-02 saved evidence; synthetic outcomes prove no recovery."""
from copy import deepcopy
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from reconciliation.gp_se2_ref02_reference import METHODS
from reconciliation.gp_se2_ref02_evaluation import FIXED_CASES, CONTRAST_METRICS, paired_comparisons
from test_gp_se2_ref02_rollout import fixture, fixture_runtime, perform, OFFICIAL

SPEC=importlib.util.spec_from_file_location('ref02_artifact_validator',Path(__file__).resolve().parents[1]/'scripts/validate_gp_se2_ref02.py')
v=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(v)


def saved_fixture(fixture, method=METHODS[2]):
    module,context,references,lineages,_=fixture
    rollout=perform(fixture,method)
    rollout['primary_execution_ordinal']=METHODS.index(method)
    pure={name:getattr(module,name) for name in ('build_pose_aligned_reference','project_body_to_world','project_world_to_local')}
    return context,references[method],lineages[method],rollout,pure


@pytest.mark.parametrize('method',METHODS)
def test_saved_runtime_and_actual_solver_inputs_validate_without_extra_solve(fixture,method):
    context,reference,lineage,rollout,pure=saved_fixture(fixture,method)
    audit=v.Audit()
    v.audit_rollout(audit,pure,rollout,context,reference,lineage,(10.,10.,1.),method,METHODS.index(method),'valid synthetic')
    assert not audit.errors
    assert audit.checks>1000


@pytest.mark.parametrize('mutation',['q','overshoot','nearest','selected_progress','cost','actual_world','actual_local',
                                     'memory','physical_start','integration','count','selector_identity','reference_capture'])
def test_corrupt_saved_record_is_rejected(fixture,mutation):
    context,reference,lineage,rollout,pure=saved_fixture(fixture)
    row=rollout['controller_reference_selections'][2]
    if mutation=='q': row['selection_diagnostic']['q_h'][0]+=.01
    if mutation=='overshoot': row['selection_diagnostic']['progress_overshoot'][0]+=.01
    if mutation=='nearest': row['selection_diagnostic']['nearest_index']+=1
    if mutation=='selected_progress': row['selection_diagnostic']['selected_original_fractional_row_coordinates'][0]+=.01
    if mutation=='cost': row['selection_diagnostic']['weighted_total_pose_distances'][0]+=.01
    if mutation=='actual_world': row['actual_submitted_reference_world'][0][0]+=.01
    if mutation=='actual_local': row['actual_controller_reference_local'][0][0]+=.01
    if mutation=='memory': row['actual_controller_previous_control'][0]+=.01
    if mutation=='physical_start': rollout['initial_physical_command'][0]+=.01
    if mutation=='integration': rollout['states'][11]['pose_world'][0]+=.01
    if mutation=='count': rollout['actual_controller_calls']=29
    if mutation=='selector_identity': rollout['wrapper_identity']['official_tracker_solve_unmodified']=False
    if mutation=='reference_capture': row['actual_reference_capture_complete']=False
    audit=v.Audit()
    audit.capture('corruption',lambda:v.audit_rollout(audit,pure,rollout,context,reference,lineage,(10.,10.,1.),METHODS[2],2,'corrupt'))
    assert audit.errors, mutation


def test_independent_oracle_cannot_fallback_to_official_dense_next_row(fixture):
    context,reference,lineage,rollout,pure=saved_fixture(fixture)
    pose=rollout['controller_reference_selections'][0]['input_pose_world']
    c=v.independent_targets(pure,reference,pose,lineage,METHODS[2],(10.,10.,1.))
    b=v.independent_targets(pure,reference,pose,lineage,METHODS[1],(10.,10.,1.))
    assert c['nearest']==b['nearest']
    np.testing.assert_array_equal(c['costs'],b['costs'])
    assert c['indices']!=b['indices']
    bad=deepcopy(lineage);bad[1]['original_fractional_row_coordinate']=bad[0]['original_fractional_row_coordinate']
    with pytest.raises(ValueError,match='strictly increasing'):
        v.independent_targets(pure,reference,pose,bad,METHODS[2],(10.,10.,1.))
    with pytest.raises(ValueError,match='unknown method'):
        v.independent_targets(pure,reference,pose,lineage,'RAW_FALLBACK',(10.,10.,1.))


def test_truthfully_saved_controller_failure_is_not_artifact_failure(fixture):
    _,context,references,lineages,goal=fixture
    runtime=fixture_runtime(failure_at=1)
    failed_fixture=(runtime,context,references,lineages,goal)
    context,reference,lineage,rollout,pure=saved_fixture(failed_fixture)
    assert rollout['controller_failure_count']==1
    audit=v.Audit()
    v.audit_rollout(audit,pure,rollout,context,reference,lineage,(10.,10.,1.),METHODS[2],2,'recorded failure')
    assert not audit.errors


def test_independent_paired_differences_keep_failure_and_na():
    rows=[]
    for case in FIXED_CASES:
        for i,method in enumerate(METHODS):
            row={key:float(i) for key in CONTRAST_METRICS}
            row.update(case_id=case,method=method,primary_success=False)
            if i==1: row['time_to_goal_s']=None
            rows.append(row)
    actual=v.independent_paired(rows)
    assert actual==paired_comparisons(rows)
    assert all(not row['both_success'] for row in actual)
    nulls=[row for row in actual if row['baseline']==METHODS[1] and row['metric']=='time_to_goal_s']
    assert len(nulls)==2 and all(row['difference'] is None for row in nulls)
    with pytest.raises(ValueError,match='six unique'):
        v.independent_paired(rows+[rows[0]])
    with pytest.raises(ValueError,match='six unique'):
        v.independent_paired(rows[:-1])


def test_validator_is_pure_and_never_imports_tracker_or_numerical_controller():
    if not OFFICIAL.exists(): pytest.skip('pinned read-only source unavailable')
    pure=v.official_pure(OFFICIAL)
    assert 'MpcTracker' not in pure and 'MPCController' not in pure and 'casadi' not in pure
    assert 'project_world_to_local' in pure and 'build_pose_aligned_reference' in pure


def test_authoritative_validation_refuses_overwrite_before_reading_source(tmp_path):
    path=tmp_path/'validation.json';path.write_text('{"preserve":true}\n')
    original=path.read_bytes()
    with pytest.raises(FileExistsError,match='overwrite'):
        v.validate(tmp_path)
    assert path.read_bytes()==original


def test_invalid_report_does_not_silently_become_scientific_failure(tmp_path):
    result=v.validate(tmp_path,write_output=False,require_finalized=False)
    assert not result['valid'] and result['errors']
    assert result['optimizer_or_MPC_solves_performed']==0
    assert result['scientific_failures_are_not_validation_failures']

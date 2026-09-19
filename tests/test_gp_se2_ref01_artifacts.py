"""Saved-evidence corruption tests; synthetic outcomes are never research results."""
from copy import deepcopy
import csv
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from reconciliation.gp_se2_ref01_evaluation import FIXED_CASES, VARIANTS
from test_gp_se2_ref01_rollout import fixture, make_rollout

SPEC = importlib.util.spec_from_file_location('ref01_artifact_validator', Path(__file__).resolve().parents[1]/'scripts/validate_gp_se2_ref01.py')
v = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(v)


def test_recursive_audit_rejects_nan_type_boolean_and_missing_values():
    a = v.Audit()
    a.same({'missing': None, 'failure': False, 'value': .2}, {'missing': None, 'failure': False, 'value': .2}, 'valid')
    assert not a.errors
    for actual, expected in [(float('nan'), 1.), (0, False), (0., None), (False, None), ({}, {'missing': None})]:
        before = len(a.errors)
        a.same(actual, expected, 'corrupt')
        assert len(a.errors) > before


def test_independent_affine_lineage_rejects_changed_progress_and_yaw(fixture):
    _, native, prepared, _, _ = fixture
    row = prepared['variants']['R01_RESAMPLE_ONLY']
    audit = v.Audit()
    v.audit_lineage(audit, native, row['reference_world'], row['row_provenance'], 'valid')
    assert not audit.errors
    for key in ('original_fractional_row_coordinate', 'unwrapped_yaw', 'derived_row_index'):
        lineage = deepcopy(row['row_provenance'])
        lineage[5][key] += .05
        audit = v.Audit()
        v.audit_lineage(audit, native, row['reference_world'], lineage, key)
        assert audit.errors


def test_independent_lineage_rejects_wrong_source_interval(fixture):
    _, native, prepared, _, _ = fixture
    row = prepared['variants']['R01_RESAMPLE_ONLY']; lineage = deepcopy(row['row_provenance'])
    lineage[5]['original_right_row_index'] = 100
    audit = v.Audit()
    v.audit_lineage(audit, native, row['reference_world'], lineage, 'wrong')
    assert any('source interval' in error for error in audit.errors)


def audited_rollout(fixture):
    context, native, prepared, module, _ = fixture
    rollout, outcome = make_rollout(fixture)
    rollout['primary_execution_ordinal'] = 0
    rollout['independent_tracker_instance'] = True
    pure = dict(build_pose_aligned_reference=module.build_pose_aligned_reference,
                project_body_to_world=module.project_body_to_world)
    return context, native, prepared['variants']['R00_NATIVE']['row_provenance'], rollout, outcome, pure


def test_scientific_failure_is_valid_saved_evidence(fixture):
    context, native, lineage, rollout, outcome, pure = audited_rollout(fixture)
    from reconciliation.gp_se2_evaluation import evaluate_rollout
    from test_gp_se2_evaluation import environment, route
    outcome = evaluate_rollout(rollout, route([9., 9., 0.]), environment(), fixture[4])
    assert not outcome['primary_success']  # synthetic unreachable fixture goal, not corrupted evidence.
    audit = v.Audit()
    v.audit_rollout(audit, pure, rollout, context, native, lineage, [10, 10, 1], 0, 'failure outcome')
    assert not audit.errors


@pytest.mark.parametrize('mutation', ['state', 'memory', 'selected_progress', 'selection', 'installation', 'schedule', 'ordinal', 'prediction'])
def test_rollout_audit_catches_record_corruption(fixture, mutation):
    context, native, lineage, rollout, _, pure = audited_rollout(fixture)
    rollout = deepcopy(rollout)
    if mutation == 'state': rollout['states'][20]['pose_world'][0] += .03
    elif mutation == 'memory': rollout['controller_reference_selections'][2]['previous_control'][0] += .01
    elif mutation == 'selected_progress': rollout['controller_reference_selections'][2]['selection_diagnostic']['selected_original_fractional_row_coordinates'][0] += .1
    elif mutation == 'selection': rollout['controller_reference_selections'][2]['selection']['indices'][0] += 1
    elif mutation == 'installation': rollout['installed_path_sha256'] = 'wrong'
    elif mutation == 'schedule': rollout['states'][20]['time_s'] += .001
    elif mutation == 'ordinal': rollout['primary_execution_ordinal'] = 3
    elif mutation == 'prediction': rollout['controller_reference_selections'][3]['prediction_world'] = [[0, 0, 0]]
    audit = v.Audit()
    v.audit_rollout(audit, pure, rollout, context, native, lineage, [10, 10, 1], 0, 'corrupt')
    assert audit.errors


def test_independent_contrasts_formula_and_na_preserved():
    rows = [dict(case_id=c, variant=name, terminal_position_error_m=x, time_to_goal_s=None if i == 3 else x)
            for c in FIXED_CASES for i, (name, x) in enumerate(zip(VARIANTS, [2., 5., 7., 13.]))]
    result = v.independent_contrasts(rows)
    p = next(r for r in result if r['metric'] == 'terminal_position_error_m')
    assert [p[k] for k in ('suffix_without_resampling', 'resampling_without_suffix', 'suffix_with_resampling', 'resampling_with_suffix', 'interaction')] == [3., 5., 6., 8., 3.]
    missing = next(r for r in result if r['metric'] == 'time_to_goal_s')
    assert missing['interaction'] is None and missing['suffix_without_resampling'] == 3.
    with pytest.raises(ValueError, match='duplicate'):
        v.independent_contrasts(rows + [rows[0]])
    with pytest.raises(KeyError):
        v.independent_contrasts(rows[:-1])


def test_csv_recomputes_null_boolean_structured_and_scalar_fields(tmp_path):
    path = tmp_path/'rows.csv'
    expected = [dict(metric='x', value=2., absent=None, success=False, rows=[1, 2])]
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, expected[0]); writer.writeheader()
        writer.writerow(dict(metric='x', value=2., absent='', success=False, rows=json.dumps([1, 2])))
    audit = v.Audit(); v.audit_csv(audit, path, expected, 'valid')
    assert not audit.errors
    text = path.read_text(); path.write_text(text.replace('2.0,,False', '2.0,0,False'))
    audit = v.Audit(); v.audit_csv(audit, path, expected, 'bad')
    assert any('missing preserved' in error for error in audit.errors)


def test_selector_verification_rejects_unpinned_external_source(tmp_path):
    path = tmp_path/'mpc.py'; path.write_text('raise RuntimeError("must never run unpinned source")\n')
    with pytest.raises(ValueError, match='source hash mismatch'):
        v.official_selector(path)


def test_authoritative_validator_refuses_overwrite_before_reading(tmp_path):
    target = tmp_path/'validation.json'; target.write_text('{"immutable":true}')
    with pytest.raises(FileExistsError, match='overwrite'):
        v.validate(tmp_path)
    assert target.read_text() == '{"immutable":true}'


def test_missing_artifact_records_failure_not_synthetic_substitution(tmp_path):
    result = v.validate(tmp_path, write_output=False)
    assert not result['valid'] and result['errors']
    assert result['optimizer_or_MPC_solves_performed'] == 0
    assert not list(tmp_path.iterdir())


def test_preflight_never_occupies_authoritative_result_path(tmp_path):
    result = v.validate(tmp_path, require_finalized=False)
    assert result['authoritative'] is False
    assert 'non-authoritative' in result['validation_scope']
    assert (tmp_path/'verification/preflight_validation.json').exists()
    assert not (tmp_path/'validation.json').exists()


def test_pairwise_retains_source_identity_change_even_at_duplicate_geometry():
    # Distinct source-row coordinates can carry identical world poses.
    diagnostic = dict(reference_world=[[1., 2., .3]]*5,
                      selected_original_fractional_row_coordinates=[1., 2., 3., 3., 3.],
                      final_goal_row_in_horizon=True)
    states = [dict(time_s=i/10, variants={name:deepcopy(diagnostic) for name in VARIANTS}) for i in range(30)]
    for state in states:
        state['variants']['R01_RESAMPLE_ONLY']['selected_original_fractional_row_coordinates'][0] += .2
    rows = v.independent_pairwise('synthetic/not_evidence', states, matched=True)
    row = next(r for r in rows if r['baseline'] == 'R00_NATIVE' and r['variant'] == 'R01_RESAMPLE_ONLY')
    assert row['selection_changed'] is True
    assert row['max_selected_xy_difference_m'] == row['max_selected_yaw_difference_rad'] == 0.
    assert row['command_difference'] is None and row['input_pose_difference'] is None
    assert row['same_input_pose'] is True

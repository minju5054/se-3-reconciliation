"""Saved REF-03 corruption fixtures. No research handoff or MPC solve evidence."""
from copy import deepcopy
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from reconciliation.gp_se2_ref02_reference import METHODS
from reconciliation.gp_se2_ref02_rollout import counterfactual_rollout
from reconciliation.gp_se2_ref03_cohort import select_additional
from test_gp_se2_ref03_worker import make_loaded, worker

SPEC=importlib.util.spec_from_file_location('ref03_artifact_validator',Path(__file__).resolve().parents[1]/'scripts/validate_gp_se2_ref03.py')
v=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(v)


def saved_fixture(native_count=7,method=METHODS[2]):
    module,loaded=make_loaded(native_count)
    rollout=counterfactual_rollout(module,loaded['frozen'],loaded['references'][method],loaded['lineages'][method],method,
        original_goal_row_index=len(loaded['frozen']['fresh_world'])-1,constant_reference_metadata=loaded['metadata'][method])
    rollout['primary_execution_ordinal']=17
    pure={n:getattr(module,n) for n in ('build_pose_aligned_reference','project_body_to_world','project_world_to_local')}
    return loaded,rollout,pure


@pytest.mark.parametrize('count',[1,2,7,13])
@pytest.mark.parametrize('method',METHODS)
def test_saved_original_runtime_actual_inputs_arbitrary_count_constant_source(count,method):
    loaded,rollout,pure=saved_fixture(count,method)
    audit=v.Audit()
    v.audit_rollout(audit,pure,rollout,loaded['frozen'],loaded['references'][method],loaded['lineages'][method],
        (10.,10.,1.),method,17,'synthetic saved runtime',loaded['metadata'][method])
    assert not audit.errors and audit.checks>1000


@pytest.mark.parametrize('mutation',['query','overshoot','nearest','actual_input','actual_local','memory','physical',
    'integration','ordinal','count','identity','source_progress','gate_fake_reanchor'])
def test_saved_corruption_is_artifact_failure(mutation):
    loaded,rollout,pure=saved_fixture();row=rollout['controller_reference_selections'][1]
    if mutation=='query':row['selection_diagnostic']['q_h'][0]+=.1
    elif mutation=='overshoot':row['selection_diagnostic']['progress_overshoot'][0]+=.1
    elif mutation=='nearest':row['selection_diagnostic']['nearest_index']+=1
    elif mutation=='actual_input':row['actual_submitted_reference_world'][0][0]+=.01
    elif mutation=='actual_local':row['actual_controller_reference_local'][0][0]+=.01
    elif mutation=='memory':row['actual_controller_previous_control'][0]+=.1
    elif mutation=='physical':rollout['initial_physical_command'][0]+=.1
    elif mutation=='integration':rollout['states'][5]['pose_world'][0]+=.01
    elif mutation=='ordinal':rollout['primary_execution_ordinal']=18
    elif mutation=='count':rollout['actual_controller_calls']=29
    elif mutation=='identity':rollout['wrapper_identity']['official_poll_unmodified']=False
    elif mutation=='source_progress':row['selection_diagnostic']['selected_original_fractional_row_coordinates'][0]+=.1
    else:rollout['candidate_frame_transform']['capture_pose_world'][0]+=.1
    a=v.Audit()
    a.capture('corruption',lambda:v.audit_rollout(a,pure,rollout,loaded['frozen'],loaded['references'][METHODS[2]],
        loaded['lineages'][METHODS[2]],(10.,10.,1.),METHODS[2],17,'bad'))
    assert a.errors


def test_repeated_constant_progress_requires_real_metadata_not_zero_gradient_style_fallback():
    loaded,rollout,pure=saved_fixture(1)
    path=rollout['installed_reference_world'];pose=rollout['initial_pose_world'];lineage=loaded['lineages'][METHODS[2]]
    oracle=v.independent_targets(pure,path,pose,lineage,METHODS[2],(10.,10.,1.),loaded['metadata'][METHODS[2]])
    assert oracle['indices']==[29]*5
    with pytest.raises(ValueError,match='strictly increasing'):
        v.independent_targets(pure,path,pose,lineage,METHODS[2],(10.,10.,1.))
    metadata={**loaded['metadata'][METHODS[2]],'retained_source_row_count':2}
    with pytest.raises(ValueError,match='constant-reference proof'):
        v.independent_targets(pure,path,pose,lineage,METHODS[2],(10.,10.,1.),metadata)


def test_source_only_selection_matches_frozen_diversity_priority_and_no_control_seeding():
    rows=[]
    for i in range(35):
        rows.append(dict(case_id=f'episode_{i%8:02d}/handoff_{i:02d}',episode_id=f'episode_{i%8:02d}',
            ordered_raw_pair=f'old{i%11}:fresh{i%11}',source_integrity_valid=True,additional_eligible=i!=0,
            group_memberships={g:(i+ord(g))%3!=0 for g in ('O','R','P','S')},rotation_dominant=i%2==0))
    expected=select_additional(rows)['selected']
    assert v.independent_additional_selection(rows)==[(r['case_id'],r['selected_group']) for r in expected]
    assert len(set(c for c,g in v.independent_additional_selection(rows)))==24
    assert v.independent_additional_selection(list(reversed(rows)))==v.independent_additional_selection(rows)


def test_pattern_truth_table_keeps_scientific_failure_and_missing_separate():
    manifest=[{'case_id':'x'},{'case_id':'y'}]
    rows=[dict(case_id=case['case_id'],method=m,completed=True,primary_success=False) for case in manifest for m in METHODS]
    assert v.independent_event_patterns(rows,manifest)=={'x':'000','y':'000'}
    rows[1].update(completed=False,primary_success=None)
    assert v.independent_event_patterns(rows,manifest)=={'x':'UNAVAILABLE','y':'000'}
    rows[1]['primary_success']=False
    with pytest.raises(ValueError,match='incomplete'):
        v.independent_event_patterns(rows,manifest)


def test_current_A_probe_origin_and_case_execution_order(tmp_path):
    module,loaded=make_loaded();events=[]
    rows,probes=worker.execute_case(module,loaded,tmp_path/'case','settings',0,events.append)
    pure={n:getattr(module,n) for n in ('build_pose_aligned_reference','project_body_to_world','project_world_to_local')}
    read=lambda p:__import__('json').loads(p.read_text())
    native=read(tmp_path/'case'/METHODS[0]/'rollout.json')
    # Test files are explicit fixture references, never source acquisition arrays.
    methods={}
    for m in METHODS:
        folder=tmp_path/m;folder.mkdir();np.save(folder/'reference_world.npy',loaded['references'][m])
        if m in probes['installation']:
            probes['installation'][m]['source_reference_file']['sha256']=v.digest(folder/'reference_world.npy')
        methods[m]=dict(folder=folder,reference=loaded['references'][m],lineage=loaded['lineages'][m],metadata=loaded['metadata'][m])
    audit=v.Audit();v.audit_probe_records(audit,pure,probes,native,methods,(10.,10.,1.),'probes')
    v.audit_worker_order(audit,events,rows,[{'case_id':loaded['frozen']['case_id']}])
    assert not audit.errors
    corrupt=deepcopy(probes);corrupt['states'][2]['input_pose_world'][0]+=.01
    audit=v.Audit();audit.capture('bad probes',lambda:v.audit_probe_records(audit,pure,corrupt,native,methods,(10.,10.,1.),'bad'))
    assert audit.errors
    bad_events=deepcopy(events);bad_events[2],bad_events[4]=bad_events[4],bad_events[2]
    audit=v.Audit();v.audit_worker_order(audit,bad_events,rows,[{'case_id':loaded['frozen']['case_id']}])
    assert audit.errors


def test_incomplete_A_probes_are_na_without_historical_substitution():
    probes=dict(historical_pose_fallback=False,new_mpc_solves=0,state_integration_performed=False,
        controller_memory_modified=False,planned_selector_calls=60,status='UNAVAILABLE',
        unavailable_reason='CURRENT_A_ROLLOUT_INCOMPLETE_OR_UNAVAILABLE',states=[],selector_calls_attempted=0)
    audit=v.Audit();v.audit_probe_records(audit,{},probes,None,{},(10.,10.,1.),'missing')
    assert not audit.errors
    probes['historical_pose_fallback']=True
    audit=v.Audit();v.audit_probe_records(audit,{},probes,None,{},(10.,10.,1.),'bad')
    assert audit.errors


def test_validator_never_overwrites_or_converts_artifact_failure_to_scientific_failure(tmp_path):
    path=tmp_path/'validation.json';path.write_text('{"preserve":true}\n');original=path.read_bytes()
    with pytest.raises(FileExistsError,match='overwrite'):v.validate(tmp_path)
    assert path.read_bytes()==original
    result=v.validate(tmp_path,write_output=False,require_finalized=False)
    assert not result['valid'] and result['errors']
    assert result['optimizer_or_MPC_solves_performed']==0
    assert result['scientific_failures_are_not_validation_failures']

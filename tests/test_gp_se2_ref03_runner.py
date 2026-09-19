"""Synthetic REF-03 orchestration tests; no actual MPC or transfer evidence."""
from copy import deepcopy
import importlib.util
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT=Path(__file__).resolve().parents[1]
SPEC=importlib.util.spec_from_file_location('ref03_runner_test',ROOT/'scripts/run_gp_se2_ref03.py')
runner=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(runner)
from reconciliation.gp_se2_reference import prepare_reference
from reconciliation.se2 import wrap_angle


def config():
    return yaml.safe_load((ROOT/'configs/gp_se2_ref03.yaml').read_text())


def test_frozen_selector_core_method_order_and_scope():
    value=runner.validate_config(config())
    assert value['methods']==list(runner.METHODS)
    assert value['group_order']==list(runner.GROUPS) and value['target_per_group']==6
    assert value['selector']==dict(source_progress_stride=1.,searchsorted_side='left',search_tolerance=0.)
    assert runner.SELECTOR_DEFINITION['reference_selector_changed']
    assert runner.SELECTOR_DEFINITION['MPC_calculation_unchanged']
    assert not runner.SELECTOR_DEFINITION['whole_controller_unchanged_claim']


@pytest.mark.parametrize('section,key,value',[
    ('selector','source_progress_stride',.5),('selector','source_progress_stride',2.),
    ('selector','searchsorted_side','right'),('selector','search_tolerance',1e-10),
    ('selector','progress_memory',True),('selector','accumulate_overshoot',True),
    ('rollout','horizon_s',4.),('rollout','control_hz',20.),('rollout','integration_hz',120.),
    ('mpc','HORIZON',6),('mpc','MPC_DT_S',.2),('mpc','Q_WEIGHTS',[1.,1.,1.])])
def test_reject_changed_intervention_or_schedule(section,key,value):
    changed=config();changed[section][key]=value
    with pytest.raises(ValueError):runner.validate_config(changed)


@pytest.mark.parametrize('field,value',[
    ('new_VLA_updates',True),('gp_or_rigid_optimization',True),('retry',True),('gui_runtime',True),
    ('blas_threads',2),('target_per_group',7),('starting_git_sha','unreviewed')])
def test_reject_extra_scope_or_unreviewed_configuration(field,value):
    changed=config();changed[field]=value
    with pytest.raises(ValueError):runner.validate_config(changed)


def test_fixed_order_and_complete_numerical_source_hashes():
    for field in ('methods','group_order'):
        changed=config();changed[field].reverse()
        with pytest.raises(ValueError):runner.validate_config(changed)
    changed=config();key=next(iter(changed['frozen_core_sha256']))
    changed['frozen_core_sha256'].pop(key)
    with pytest.raises(ValueError,match='complete fixed core'):runner.validate_config(changed)
    changed=config();changed['frozen_core_sha256'][key]='0'*64
    with pytest.raises(ValueError,match='immutable numerical'):runner.validate_config(changed)


@pytest.mark.parametrize('count',[1,2,7,31,57])
def test_reference_arrays_keep_world_frame_native_count_and_identical_BC(count):
    x=np.arange(count,dtype=float)
    native=np.column_stack((12.+x*.13,-4.+.02*x,wrap_angle(3.+x*.08)))
    frozen=dict(fresh_world=native.tolist(),B_world=[12.12,-4.01,-3.])
    before=deepcopy(frozen)
    prepared,factorized,methods=runner.reference_inputs(frozen,config()['mpc'])
    assert frozen==before
    native_result=methods[runner.METHODS[0]]['value']['reference_world']
    b,c=[methods[m]['value']['reference_world'] for m in runner.METHODS[1:]]
    assert np.array_equal(native_result,native) and len(native_result)==count
    assert b.shape==c.shape==(30,3) and b.dtype==c.dtype==np.float64
    assert b.tobytes()==c.tobytes()
    assert methods[runner.METHODS[1]]['value']['row_provenance']==methods[runner.METHODS[2]]['value']['row_provenance']
    assert np.all(b[:,0]>=12.)  # No transform to B/capture-local coordinates.
    np.testing.assert_array_equal(prepared['goal_world'],native[-1])
    np.testing.assert_allclose(b[-1,:2],native[-1,:2],atol=0.,rtol=0.)
    assert abs(wrap_angle(b[-1,2]-native[-1,2]))<1e-14
    residual=native-np.asarray(frozen['B_world']);residual[:,2]=wrap_angle(residual[:,2])
    original_k=min(int(np.argmin(np.sum(residual**2*np.array([10.,10.,1.]),axis=1)))+1,count-1)
    assert prepared['first_future_row_index']==original_k
    assert methods[runner.METHODS[1]]['value']['row_provenance'][0]['original_fractional_row_coordinate']==original_k
    original=prepare_reference(native,frozen['B_world'],[10.,10.,1.])
    assert np.array_equal(b,original['common_world'])


@pytest.mark.parametrize('count',[1,8])
def test_single_retained_source_row_has_explicit_constant_metadata(count):
    native=np.column_stack((np.arange(count)+20.,np.zeros(count)-2.,np.ones(count)*1.))
    context=dict(fresh_world=native.tolist(),B_world=native[-1].tolist())
    prepared,_,methods=runner.reference_inputs(context,config()['mpc'])
    assert len(prepared['suffix_world'])==1
    for method in runner.METHODS[1:]:
        record=methods[method]
        assert record['metadata']==dict(constant_reference=True,retained_source_row_count=1,original_source_row_index=count-1)
        assert np.array_equal(record['value']['reference_world'],np.tile(native[-1],(30,1)))
        lineage=record['value']['row_provenance']
        assert {r['original_fractional_row_coordinate'] for r in lineage}=={float(count-1)}
    assert (methods[runner.METHODS[0]]['metadata'] is not None)==(count==1)


def test_rotation_only_duplicates_keep_distinct_original_row_identity():
    native=[[12.,-2.,3.0],[12.,-2.,-3.1],[12.,-2.,-2.9],[12.1,-2.,-2.8]]
    context=dict(fresh_world=native,B_world=[12.,-2.,2.5])
    _,_,methods=runner.reference_inputs(context,config()['mpc'])
    rows=methods[runner.METHODS[0]]['value']['row_provenance']
    assert [r['original_fractional_row_coordinate'] for r in rows]==[0.,1.,2.,3.]
    assert rows[0]['world_xy']==rows[1]['world_xy']==rows[2]['world_xy']
    assert len({r['wrapped_yaw'] for r in rows})==4
    assert abs(rows[1]['unwrapped_yaw']-rows[0]['unwrapped_yaw'])<np.pi


def test_exclusive_writes_prepare_roots_and_execute_no_retry(tmp_path,monkeypatch):
    record=tmp_path/'record.json';runner.write(record,{'preserved':True});old=record.read_bytes()
    with pytest.raises(FileExistsError):runner.write(record,{'preserved':False})
    assert record.read_bytes()==old
    with pytest.raises(FileExistsError,match='no overwrite'):runner.prepare(tmp_path,ROOT/'configs/gp_se2_ref03.yaml')
    with pytest.raises(FileExistsError,match='no overwrite'):runner.prepare(tmp_path/'outside-root',ROOT/'configs/gp_se2_ref03.yaml')
    monkeypatch.setattr(runner,'verify',lambda _:(config(),{}))
    monkeypatch.setattr(runner.subprocess,'run',lambda *a,**k:pytest.fail('No controller process permitted in this test'))
    with pytest.raises(ValueError,match='freeze'):runner.execute(tmp_path)
    runner.write(tmp_path/'execution_freeze.json',{})
    runner.write(tmp_path/'execution_started.json',{})
    with pytest.raises(FileExistsError,match='no retry'):runner.execute(tmp_path)


@pytest.mark.parametrize('changed_file,expected',[
    ('config_snapshot.yaml','frozen config'),('experiment_config.yaml','frozen config'),
    ('original.npy','original source'),('preserved.py','original code'),
    ('execution.py','execution code'),('input.npy','execution input')])
def test_freeze_detects_configuration_source_execution_and_input_drift(tmp_path,changed_file,expected):
    (tmp_path/'experiment_config.yaml').write_text(yaml.safe_dump(config()))
    for name in ('config_snapshot.yaml','original.npy','preserved.py','execution.py','input.npy'):
        (tmp_path/name).write_text('original\n')
    runner.write(tmp_path/'source.json',dict(
        original_config_sha256=runner.digest(tmp_path/'config_snapshot.yaml'),
        experiment_config_sha256=runner.digest(tmp_path/'experiment_config.yaml'),
        copied_source_sha256={str(tmp_path/'original.npy'):runner.digest(tmp_path/'original.npy')},
        preserved_core_sha256={str(tmp_path/'preserved.py'):runner.digest(tmp_path/'preserved.py')}))
    runner.write(tmp_path/'execution_freeze.json',dict(
        source_sha256={str(tmp_path/'execution.py'):runner.digest(tmp_path/'execution.py')},
        input_sha256={'input.npy':runner.digest(tmp_path/'input.npy')}))
    runner.verify(tmp_path)
    with (tmp_path/changed_file).open('a') as stream:stream.write('# drift\n')
    with pytest.raises(ValueError,match=expected):runner.verify(tmp_path)


def test_snapshot_has_execution_tests_and_excludes_only_declared_reporting(tmp_path,monkeypatch):
    fake_root=tmp_path/'repository';run=fake_root/'data/run'
    files=('src/reconciliation/gp_se2_ref03_evaluation.py','scripts/run_gp_se2_ref03.py',
        'scripts/lightnav/gp_se2_ref03_mpc.py','tests/test_gp_se2_ref03_runner.py',
        'scripts/validate_gp_se2_ref03.py','tests/test_gp_se2_ref03_artifacts.py','configs/gp_se2_ref03.yaml')
    for name in files:
        path=fake_root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(name)
    run.mkdir(parents=True);runner.write(run/'prepared_input.json',{'fixed':True})
    monkeypatch.setattr(runner,'ROOT',fake_root)
    monkeypatch.setattr(runner,'verify',lambda _:({},{}))
    monkeypatch.setattr(runner,'revision',lambda:'synthetic-test-sha')
    runner.freeze(run)
    freeze=runner.read(run/'execution_freeze.json')
    assert set(freeze['source_sha256'])==set(files)-runner.REPORTING_ONLY
    assert freeze['input_sha256']=={'prepared_input.json':runner.digest(run/'prepared_input.json')}
    assert freeze['no_primary_execution_yet']
    with pytest.raises(FileExistsError):runner.freeze(run)
    final=runner.snapshot(run,'final')
    assert set(final)==set(files)


def _saved_evaluation_fixture(tmp_path,monkeypatch,fixture,*,missing_A):
    """Persist deterministic test-controller records; no numerical MPC is invoked."""
    from reconciliation.gp_se2_environment import HospitalEnvironment
    from test_gp_se2_ref01_rollout import make_rollout
    from test_gp_se2_evaluation import environment,route
    from test_gp_se2_ref03_evaluation import control,metadata
    context,native,prepared,_,physical_config=fixture
    run=tmp_path/'run';run.mkdir();(run/'aggregate').mkdir()
    output=run/'mpc_output';output.mkdir()
    source=dict(environment_path='synthetic-only',preparation_total_wall_s=.1,
        preservation_hash_audit_wall_s=.01,environment_loading_wall_s=.02,cohort_audit_wall_s=.03)
    monkeypatch.setattr(runner,'verify',lambda _:(config(),source))
    monkeypatch.setattr(HospitalEnvironment,'load',lambda _:environment())
    (run/'config_snapshot.yaml').write_text(yaml.safe_dump(physical_config))
    runner.write(run/'mpc_request.json',{'synthetic_fixture':True})
    runner.write(run/'execution_completed.json',{'worker_process_wall_s':.5})
    metas=[control(),metadata()]
    for meta in metas:
        meta['case_directory']=meta['case_id'].replace('/','__')
        meta['relative_directory']=('controls/' if meta['cohort']=='KNOWN_CONTROLS' else 'transfer/')+meta['case_directory']
        case=run/meta['relative_directory'];worker=output/meta['case_directory'];worker.mkdir()
        current_context={**context,'case_id':meta['case_id']}
        goal_route=route(native[-1] if meta['cohort']=='KNOWN_CONTROLS' else [100.,0.,0.])
        runner.write(case/'input_context.json',current_context);runner.write(case/'goal_route.json',goal_route)
        diagnostic=None
        for method,variant in zip(runner.METHODS,('R00_NATIVE','R11_CURRENT_ADAPTER','R11_CURRENT_ADAPTER')):
            target=case/'methods'/method;target.mkdir(parents=True)
            reference=prepared['variants'][variant]['reference_world'];runner.array(target/'reference_world.npy',reference)
            missing=missing_A and meta['cohort']=='ADDITIONAL_TRANSFER' and method==runner.METHODS[0]
            status=dict(attempted=True,completed=not missing,status='TECHNICAL_FAILURE' if missing else 'COMPLETED',
                technical_error='synthetic failure' if missing else None,actual_controller_calls=0 if missing else 30,
                rollout_wall_s=None if missing else .1,official_mpc_solve_wall_s=None if missing else .05)
            runner.write(worker/method/'worker_status.json',status)
            if missing:continue
            rollout,oldmetrics=make_rollout(fixture,variant)
            rollout['candidate_source']={'sha256':runner.digest(target/'reference_world.npy')}
            runner.write(worker/method/'rollout.json',rollout)
            if method==runner.METHODS[1]:diagnostic=rollout['controller_reference_selections']
            if meta['cohort']=='KNOWN_CONTROLS':
                runner.write(case/'historical'/method/'rollout.json',rollout)
                runner.write(case/'historical'/method/'metrics.json',oldmetrics)
        unavailable=missing_A and meta['cohort']=='ADDITIONAL_TRANSFER'
        probes=dict(wall_s=.01,selector_calls_completed=0 if unavailable else 60,status='UNAVAILABLE' if unavailable else 'COMPLETED',
            unavailable_reason='A incomplete' if unavailable else None,states=[])
        if not unavailable:
            probes['states']=[dict(time_s=r['time_s'],input_pose_world=r['input_pose_world'],
                methods={m:deepcopy(r['selection_diagnostic']) for m in runner.METHODS[1:]}) for r in diagnostic]
        runner.write(worker/'matched_state_probes.json',probes)
        runner.write(worker/'ledger.json',{'synthetic_fixture':True});runner.write(worker/'input_validation.json',{'valid':True})
    runner.write(run/'case_manifest.json',{'selected':metas})
    runner.write(output/'provenance.json',{'request_sha256':runner.digest(run/'mpc_request.json')})
    runner.write(output/'summary.json',{'probe_cases_completed':1 if missing_A else 2})
    runner.write(output/'output_hashes.json',{'files':[dict(path=str(path.relative_to(output)),sha256=runner.digest(path))
        for path in sorted(output.rglob('*')) if path.is_file()]})
    return run


from test_gp_se2_ref01_rollout import fixture


@pytest.mark.parametrize('missing_A',[False,True])
def test_saved_evaluation_contract_preserves_failures_missing_and_separate_cohorts(tmp_path,monkeypatch,fixture,missing_A):
    run=_saved_evaluation_fixture(tmp_path,monkeypatch,fixture,missing_A=missing_A)
    # A controller subprocess is forbidden: evaluation must only inspect saved data.
    monkeypatch.setattr(runner.subprocess,'run',lambda *a,**k:pytest.fail('Evaluation launched a process'))
    runner.evaluate(run)
    summary=runner.read(run/'aggregate/summary.json');matrix=runner.read(run/'aggregate/outcome_matrix.json')
    assert len(matrix)==6 and summary['method_count']==6
    assert summary['known_controls']['event_count']==summary['additional_transfer']['event_count']==1
    assert summary['controls_reproduced'] and summary['GP_or_rigid_optimization_solves']==0
    assert summary['additional_transfer']['method_outcomes'][runner.METHODS[2]]['failures']==1
    assert summary['additional_transfer']['method_outcomes'][runner.METHODS[2]]['successes']==0
    if missing_A:
        absent=next(r for r in matrix if r['cohort']=='ADDITIONAL_TRANSFER' and r['method']==runner.METHODS[0])
        assert absent['attempted'] and not absent['completed'] and absent['primary_success'] is None
        assert absent['terminal_position_error_m'] is None
        assert summary['operational_status']=='GP_SE2_REF_03_COMPLETED_WITH_LIMITATIONS'
        assert summary['additional_transfer']['exclusive_patterns']['UNAVAILABLE']==1
        bc=next(r for r in runner.read(run/'aggregate/command_divergence.json') if r['case_id']==absent['case_id'] and r['baseline']==runner.METHODS[1])
        assert bc['available']
    else:
        assert summary['operational_status']=='GP_SE2_REF_03_COMPLETED'
        assert summary['additional_transfer']['exclusive_patterns']['000']==1
    for key,name in [('duplicate_summary','duplicate_summary'),('paired_quality_equal_weight','paired_quality_summary')]:
        assert runner.read(run/f'aggregate/{name}.json')==runner.scope_exports(summary,key)
    assert all(r['difference'] is None for r in runner.read(run/'aggregate/paired_metrics.json') if r['cohort']=='ADDITIONAL_TRANSFER')
    assert runner.read(run/'evaluation_completed.json')['new_mpc_solves']==0
    before=runner.digest(run/'aggregate/summary.json')
    with pytest.raises(FileExistsError,match='no overwrite'):runner.evaluate(run)
    assert runner.digest(run/'aggregate/summary.json')==before


def test_worker_output_tampering_blocks_evaluation_before_started_marker(tmp_path,monkeypatch,fixture):
    run=_saved_evaluation_fixture(tmp_path,monkeypatch,fixture,missing_A=False)
    worker_file=next((run/'mpc_output').rglob('rollout.json'))
    with worker_file.open('a') as stream:stream.write('\n')
    with pytest.raises(ValueError,match='output integrity'):runner.evaluate(run)
    assert not (run/'evaluation_started.json').exists()

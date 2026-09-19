"""Synthetic presentation/serialization tests, never experimental evidence."""
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import zipfile
from types import SimpleNamespace

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
def load(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'scripts'/f'{name}.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
p=load('plot_gp_se2_ref03');bundle=load('package_gp_se2_ref03_review')


def fixture(tmp_path):
    path=np.array([[0.,0.,3.1],[.1,0.,-3.1],[.2,.1,-2.9]])
    context=dict(old_world=path.tolist(),fresh_world=path.tolist(),B_world=[0.,0.,3.1],u_minus=[.1,.2],previous_control=[.2,.4])
    route=dict(goal_world=path[-1].tolist(),route_status='REQUIRED_ORDERED_GATES',gates=[dict(gate_id='test_gate',center_xy=[.1,.05],normal_xy=[1.,0.],half_width_m=.2)])
    past=dict(poses_world=[[-.1,0.,3.1],[0.,0.,3.1]],times_relative_to_B_s=[-.1,0.])
    config=dict(formulation=dict(goal_position_tolerance=.15,goal_yaw_tolerance=np.pi/12),footprint=dict(radius_m=.2,required_clearance_m=.05),evaluation=dict(terminal_goal_dwell_s=.2))
    methods={}
    for method in p.METHODS:
        folder=tmp_path/method;folder.mkdir();np.save(folder/'reference_world.npy',path)
        diagnostic=dict(nearest_original_fractional_row_coordinate=0.,selected_original_fractional_row_coordinates=[1.,2.,2.,2.,2.],
            final_goal_row_in_horizon=True,q_h=[1.,2.,2.,2.,2.] if method==p.METHODS[2] else None,
            progress_overshoot=[0.]*5 if method==p.METHODS[2] else None,endpoint_repeated=True,
            official_reference_unwrapped_yaw=[3.18,3.38,3.38,3.38,3.38])
        rollout=dict(states=[dict(time_s=t,pose_world=row.tolist()) for t,row in zip([0.,1.,3.],path)],
            controller_reference_selections=[dict(time_s=0.,command=[.1,.2],selection_diagnostic=diagnostic),dict(time_s=2.9,command=[.0,.0],selection_diagnostic=diagnostic)])
        metrics=dict(summary=dict(completed=True,primary_success=False,missing_reason=None,failure_reasons=['GOAL_YAW_FAILURE']),
            dense_poses_world=path.tolist(),dense_times_s=[0.,1.,3.],environment=dict(clearance_samples_m=[.3,.2,.1]))
        methods[method]=dict(folder=folder,reference=path.copy(),provenance=[dict(source=i) for i in range(3)],rollout=rollout,metrics=metrics)
    return context,route,methods,past,config


def test_exact_shared_input_identity_no_display_offset(tmp_path):
    _,_,methods,_,_=fixture(tmp_path)
    identity=p.input_identity(methods)
    assert identity['byte_identical'] and identity['array_identical'] and identity['lineage_identical']
    assert identity['shared_curve_drawn_once'] and not identity['spatial_offsets']
    methods[p.METHODS[2]]['reference'][1,0]+=.01
    assert not p.input_identity(methods)['array_identical']


def test_numeric_export_all_plots_original_gate_and_yaw_semantics(tmp_path):
    context,route,methods,past,config=fixture(tmp_path)
    result=p.expected_numeric(context,route,methods,past,config)
    assert tuple(result)==p.PLOT_NAMES
    xy=result['reference_and_rollout_world']
    assert xy is result['boundary_zoom']
    gate=xy['required_gates'][0]
    assert gate['original_gate_definition']==route['gates'][0]
    np.testing.assert_allclose(gate['segment_world_xy'],[[.1,-.15],[.1,.25]])
    a=xy['axes_world_m'];z=xy['boundary_axes_world_m']
    assert a[1]-a[0]==pytest.approx(a[3]-a[2])
    assert z[1]-z[0]==pytest.approx(z[3]-z[2])
    yaw=result['goal_yaw_and_position_error']['methods'][p.METHODS[0]]
    assert yaw['actual_wrapped_yaw_rad'][1]<0<yaw['actual_display_unwrapped_yaw_rad'][1]
    command=result['angular_command_vs_time']['methods'][p.METHODS[1]]
    assert command['times_s']==[0.,2.9,3.]
    assert command['physical_u_minus']!=command['controller_previous_control']
    assert result['selected_progress_and_goal_inclusion']['methods'][p.METHODS[1]]['requested_q_h']==[None,None]


def test_missing_result_is_null_and_does_not_fabricate_rollout_or_command(tmp_path):
    context,route,methods,past,config=fixture(tmp_path)
    methods[p.METHODS[2]]['rollout']=None
    methods[p.METHODS[2]]['metrics']=dict(summary=dict(completed=False,primary_success=None,missing_reason='TECHNICAL_BLOCKER',failure_reasons=None))
    out=p.expected_numeric(context,route,methods,past,config)
    assert out['reference_and_rollout_world']['methods'][p.METHODS[2]]['actual_poses_world'] is None
    assert out['linear_command_vs_time']['methods'][p.METHODS[2]] is None
    assert out['clearance_vs_time']['methods'][p.METHODS[2]] is None
    assert out['selected_progress_and_goal_inclusion']['methods'][p.METHODS[2]] is None
    assert out['goal_yaw_and_position_error']['methods'][p.METHODS[2]]['summary']['primary_success'] is None


def test_hold_line_stops_at_actual_saved_endpoint(tmp_path):
    context,route,methods,past,config=fixture(tmp_path)
    for m in methods.values():
        m['rollout']['states'][-1]['time_s']=2.95
    result=p.expected_numeric(context,route,methods,past,config)
    assert result['linear_command_vs_time']['methods'][p.METHODS[0]]['times_s'][-1]==2.95


def representatives_fixture():
    rows=[];events=[]
    for i,(b,c) in enumerate([(True,False),(False,True),(False,True),(False,False),(True,True)]):
        case=f'episode_{i}/handoff_000';rows.append(dict(case_id=case,relative_directory='transfer/'+case.replace('/','__'),cohort='ADDITIONAL_TRANSFER',stratum=p.STRATA[0],case_role=p.STRATA[0]))
        events.append(dict(case_id=case,success_by_method=dict(zip(p.METHODS,[False,b,c])),overlapping_flags=dict(B_failure_C_success=not b and c)))
    case='known/h0';rows.append(dict(case_id=case,relative_directory='controls/known__h0',cohort='KNOWN_CONTROLS',stratum=None,case_role='REPRO_BENIGN'))
    events.append(dict(case_id=case,success_by_method=dict(zip(p.METHODS,[False,False,False])),overlapping_flags=dict(B_failure_C_success=False)))
    regressions=[dict(case_id=rows[0]['case_id'],method=p.METHODS[2],baseline=p.METHODS[0],comparable=True,new_safety_reasons=[],new_route_reasons=['ROUTE_VIOLATION'])]
    return rows,events,regressions


def test_representatives_regressions_even_failed_baseline_then_hash_first_and_controls():
    rows,events,regressions=representatives_fixture()
    result=p.representative_cases(rows,events,regressions)
    assert result['selected'][0]['case_id']==rows[0]['case_id']
    ids=[r['case_id'] for r in result['selected']]
    key=lambda case:hashlib.sha256(('REF03-v1:'+case).encode()).hexdigest()
    assert min([rows[1]['case_id'],rows[2]['case_id']],key=key) in ids
    assert min([rows[3]['case_id'],rows[4]['case_id']],key=key) in ids
    assert rows[-1]['case_id'] in ids and len(ids)==len(set(ids))
    assert 'not identical path' in result['unchanged_means']
    assert p.representative_cases(list(reversed(rows)),list(reversed(events)),regressions)['selected'][:3]==result['selected'][:3]


def test_sidecar_records_exact_numerical_missing_values_and_no_overwrite(tmp_path):
    source=tmp_path/'source.json';source.write_text('{"synthetic_test_only":true}')
    fig,ax=p.plt.subplots(figsize=(5,3));ax.plot([0,3],[0,1]);p._time(ax,'test')
    assert ax.get_xlim()==(0.,3.)
    target=tmp_path/'figure.png'
    row=p._save(fig,target,dict(value=None,numbers=[1.,2.]),[source],'SYNTHETIC UNIT TEST ONLY')
    side=json.loads(target.with_suffix('.json').read_text())
    assert side['numeric_data']['value'] is None
    assert side['image_sha256']==row['sha256']
    assert side['source_hashes'][str(source)]==hashlib.sha256(source.read_bytes()).hexdigest()
    assert side['primary_time_range_s']==[0.,3.] and not side['new_execution'] and not side['gui_runtime_validated']
    with pytest.raises(FileExistsError):p._save(fig,target,{},[source],'same')


def test_review_allowlist_all_world_overlays_representative_details_only_and_no_raw(tmp_path):
    rows,events,reasons=representatives_fixture();reps=p.representative_cases(rows,events,reasons)
    (tmp_path/'case_manifest.json').write_text(json.dumps(dict(selected=rows)))
    (tmp_path/'representative_selection.json').write_text(json.dumps(reps))
    allowed=bundle.allowed_artifacts(tmp_path)
    assert sum(x.name=='reference_and_rollout_world.png' for x in allowed)==len(rows)
    assert sum(x.name=='linear_command_vs_time.png' for x in allowed)==reps['count']
    assert Path('aggregate/research_interpretation.json') in allowed
    assert all(x.suffix in {'.json','.yaml','.csv','.png'} for x in allowed)
    for path in allowed:
        target=tmp_path/path;target.parent.mkdir(parents=True,exist_ok=True)
        if not target.exists():target.write_text('synthetic packaging fixture only')
    (tmp_path/'raw_rgb.png').write_text('must never enter bundle')
    result=bundle.package(tmp_path)
    with zipfile.ZipFile(tmp_path/'review_bundle.zip') as z:
        assert 'raw_rgb.png' not in z.namelist()
        assert 'aggregate/research_interpretation.json' in z.namelist()
        assert len(z.namelist())==result['file_count']
        assert all(not n.endswith('.npy') for n in z.namelist())
    with pytest.raises(FileExistsError):bundle.package(tmp_path)


def test_all_case_renderers_handle_required_gate_missing_result_and_full_time(tmp_path):
    from shapely.geometry import box
    context,route,methods,past,config=fixture(tmp_path)
    run=tmp_path/'synthetic_render_only';case=run/'transfer/test__event';case.mkdir(parents=True)
    for name in ('source.json','protocol.json','config_snapshot.yaml','case_manifest.json','cohort_coverage.json'):
        (run/name).write_text('{}')
    for name,value in [('input_context.json',context),('goal_route.json',route),('actual_past_execution.json',past)]:
        (case/name).write_text(json.dumps(value))
    for name,item in methods.items():
        target=case/'methods'/name;target.mkdir(parents=True)
        np.save(target/'reference_world.npy',item['reference'])
        (target/'row_provenance.json').write_text(json.dumps(item['provenance']))
        metric=item['metrics']
        if name==p.METHODS[2]:metric=dict(summary=dict(completed=False,primary_success=None,missing_reason='SYNTHETIC_TEST_MISSING',failure_reasons=None))
        else:(target/'rollout.json').write_text(json.dumps(item['rollout']))
        (target/'metrics.json').write_text(json.dumps(metric))
    environment=SimpleNamespace(workspace=box(-5,-5,5,5),obstacles=box(2,2,3,3))
    images=p.plot_case(run,dict(case_id='SYNTHETIC_TEST_ONLY/event',relative_directory='transfer/test__event'),environment,config)
    assert len(images)==7 and {Path(r['path']).stem for r in images}==set(p.PLOT_NAMES)
    for image in images:
        side=json.loads(Path(image['path']).with_suffix('.json').read_text())
        assert side['primary_time_range_s']==[0.,3.]
        assert not side['gui_runtime_validated']


def test_all_aggregate_renderers_keep_counts_nonadditive_and_quality_na(tmp_path):
    scope=dict(event_count=1,method_outcomes={m:dict(successes=1,completed=1) for m in p.METHODS},
        exclusive_patterns={k:int(k=='111') for k in (*p.PATTERNS,'UNAVAILABLE')},
        overlapping_flags={k:dict(count=0,comparable_events=1,unavailable_events=0) for k in p.OVERLAPPING_FLAGS})
    summary=dict(transfer_strata={s:deepcopy(scope) for s in p.STRATA},known_controls=deepcopy(scope),known_control_by_role={},additional_transfer=deepcopy(scope))
    coverage=dict(group_candidate_counts={s:2 for s in 'ORPS'},group_selected_counts={s:1 for s in 'ORPS'},source_event_count=8,
        original_gp01_eligible_count=8,historical_exclusion_union_count=0,additional_selected_count=4,controls_count=0,
        additional_unique_episode_count=4,additional_unique_ordered_raw_pair_count=4)
    quality=[dict(cohort='ADDITIONAL_TRANSFER',stratum=s,baseline=p.METHODS[1],method=p.METHODS[2],metric=metric,difference=None,
        eligible_both_success=False,baseline_value=None,method_value=None) for s in p.STRATA for metric in p.QUALITY_METRICS]
    for name in ('source.json','protocol.json','config_snapshot.yaml','case_manifest.json','cohort_coverage.json'):(tmp_path/name).write_text('{}')
    (tmp_path/'aggregate').mkdir()
    for name in ('summary.json','success_patterns.json','regressions.json','paired_metrics.json'):(tmp_path/'aggregate'/name).write_text('{}')
    images=p.plot_aggregate(tmp_path,[],summary,coverage,[],[],quality)
    assert len(images)==4
    side=json.loads((tmp_path/'aggregate/plots/paired_successful_quality.json').read_text())
    assert all(row['difference'] is None for row in side['numeric_data']['rows'])
    side=json.loads((tmp_path/'aggregate/plots/regressions_and_recoveries.json').read_text())
    assert side['numeric_data']['overlapping_counts_must_not_be_added']

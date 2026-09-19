"""REF-02 plotting/packaging fixtures; no controller runs or research evidence."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]

def module(name, path):
    spec=importlib.util.spec_from_file_location(name, ROOT/path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result)
    return result

plot=module('ref02_plot_test','scripts/plot_gp_se2_ref02.py')
package=module('ref02_package_test','scripts/package_gp_se2_ref02_review.py')


def prepared(tmp_path):
    context=dict(old_world=[[0.,0.,0.],[.1,0.,0.]],fresh_world=[[0.,0.,0.],[1.,0.,.1]],
                 B_world=[0.,0.,0.],u_minus=[.2,.1],previous_control=[.3,.2])
    route=dict(goal_world=[1.,0.,.1])
    methods={};states=[]
    for name in plot.METHODS:
        folder=tmp_path/name;folder.mkdir()
        reference=np.array([[0.,0.,0.],[1.,0.,.1]])
        np.save(folder/'reference_world.npy',reference,allow_pickle=False)
        lineage=[dict(derived_row_index=i,original_fractional_row_coordinate=float(i),world_xy=p[:2].tolist(),wrapped_yaw=float(p[2])) for i,p in enumerate(reference)]
        diagnostics=[];solves=[]
        for i in range(30):
            diag=dict(reference_world=[[1.,0.,.1]]*5,nearest_original_fractional_row_coordinate=0.,
                selected_original_fractional_row_coordinates=[1.]*5,selected_first_target_distance_m=1.,
                selected_last_target_distance_m=1.,horizon_xy_arc_length_m=0.,horizon_yaw_span_rad=0.,
                final_goal_row_in_horizon=True,endpoint_repeated=True,q_h=[1.]*5 if name.startswith('C_') else None,
                progress_overshoot=[0.]*5 if name.startswith('C_') else None)
            diagnostics.append(diag)
            solves.append(dict(time_s=i/10,input_pose_world=[0.,0.,0.],selection_diagnostic=diag,command=[.2,.1]))
        metrics=dict(primary_success=False,failure_reasons=['goal_failure'],minimum_clearance_m=.4,
            dense_poses_world=[[0.,0.,3.],[.2,0.,-3.]],dense_times_s=[0.,3.],environment=dict(clearance_samples_m=[.4,.4]),
            execution=dict(terminal_position_error_m=.8,terminal_yaw_error_rad=.6,terminal_goal_dwell_pass=False,
                           terminal_goal_dwell_s=.2,time_to_goal_s=None,motion_limits_pass=True,controller_failure_count=0))
        methods[name]=dict(folder=folder,reference=reference,provenance=lineage,metrics=metrics,
            rollout=dict(states=[dict(time_s=0.,pose_world=[0.,0.,0.]),dict(time_s=3.,pose_world=[.2,0.,.3])],
                         controller_reference_selections=solves))
        for filename,value in [('row_provenance.json',lineage),('rollout.json',methods[name]['rollout']),('metrics.json',metrics)]:
            (folder/filename).write_text(json.dumps(value))
    for i in range(30):
        states.append(dict(probe_index=i,time_s=i/10,input_pose_world=[0.,0.,0.],methods={n:m['rollout']['controller_reference_selections'][i]['selection_diagnostic'] for n,m in methods.items()}))
    past=dict(poses_world=[[-.1,0.,0.],[0.,0.,0.]],times_relative_to_B_s=[-.1,0.])
    config=dict(formulation=dict(goal_position_tolerance=.15,goal_yaw_tolerance=np.pi/12),
                evaluation=dict(terminal_goal_dwell_s=.2),footprint=dict(required_clearance_m=.05))
    return context,route,methods,past,dict(states=states),config


def test_dense_identity_checks_bytes_not_only_equal_arrays(tmp_path):
    _,_,methods,_,_,_=prepared(tmp_path)
    identity=plot.input_identity(methods)
    assert identity['byte_identical'] and identity['array_identical'] and identity['lineage_identical']
    path=methods['C_DENSE_SOURCE_PROGRESS']['folder']/'reference_world.npy'
    with path.open('ab') as stream:stream.write(b'\0')
    assert np.array_equal(np.load(path,allow_pickle=False),methods['B_DENSE_ROW_STEP']['reference'])
    changed=plot.input_identity(methods)
    assert changed['array_identical'] and not changed['byte_identical']
    assert changed['file_sha256']['B_DENSE_ROW_STEP'] != changed['file_sha256']['C_DENSE_SOURCE_PROGRESS']


def test_series_preserves_q_na_and_exact_solver_yaw(tmp_path):
    context,route,methods,past,probes,config=prepared(tmp_path)
    series=plot.selection_series(probes['states'],matched=True)
    assert series['A_NATIVE']['requested_q_h']==[None]*30
    assert series['B_DENSE_ROW_STEP']['progress_overshoot']==[None]*30
    assert series['C_DENSE_SOURCE_PROGRESS']['requested_q_h']==[[1.]*5]*30
    assert len(series['C_DENSE_SOURCE_PROGRESS']['selection_diagnostics'])==30
    assert series['A_NATIVE']['official_reference_unwrapped_yaw_rad'][0]==[.1]*5


def test_wrapped_error_plot_splits_representation_cut_without_new_samples():
    times=[0.,1.,2.,3.];angles=[3.0,3.1,-3.1,-3.0]
    segments=plot.wrapped_curve_segments(times,angles)
    assert len(segments)==2
    assert sum((r['times_s'] for r in segments),[])==times
    assert sum((r['angles_rad'] for r in segments),[])==angles
    with pytest.raises(ValueError):plot.wrapped_curve_segments([0.],[0.,1.])


def test_all_numeric_exports_preserve_failures_and_command_hold(tmp_path):
    context,route,methods,past,probes,config=prepared(tmp_path)
    numeric=plot.expected_numeric(context,route,methods,past,probes,config)
    assert set(numeric)==set(plot.PLOT_NAMES)==set(package.FIGURES)
    assert len(numeric)==11
    for name in plot.METHODS:
        commands=numeric['linear_command_vs_time'][name]
        assert commands['times_s'][-2:]==[2.9,3.]
        assert commands['applied_commands'][-2:]==[.2,.2]
        assert commands['initial_physical_command']==.2
        assert commands['initial_controller_memory']==.3
        assert numeric['fixed_input_world']['methods'][name]['reference_world']==methods[name]['reference'].tolist()
    for row in numeric['outcome_summary']['rows']:
        assert row['time_to_goal_s'] is None and row['success'] is False
    assert [r['time_s'] for r in numeric['matched_state_targets']['snapshots']]==[0.,1.,2.]
    yaw=numeric['actual_yaw_and_goal_error']['methods']['A_NATIVE']
    assert yaw['actual_wrapped_yaw_rad']==[3.,-3.]
    assert yaw['actual_visual_unwrapped_yaw_rad'][-1]>3.


def test_world_extents_equal_aspect_and_time_axis_fixed_horizon(tmp_path):
    context,route,methods,past,probes,config=prepared(tmp_path)
    xy=plot.geometry_numeric(context,route,methods,past)
    for key in ('axes_world_m','detail_axes_world_m','input_axes_world_m'):
        a,b,c,d=xy[key]
        assert b-a==pytest.approx(d-c)
    fig,ax=plt.subplots();plot.decorate_time(ax,'speed')
    assert ax.get_xlim()==(0.,3.)
    plt.close(fig)


def test_sidecar_exports_dense_hash_identity_and_refuses_overwrite(tmp_path):
    _,_,methods,_,_,_=prepared(tmp_path)
    identity=plot.input_identity(methods)
    source=tmp_path/'source.json';source.write_text('{"synthetic_fixture_only":true}')
    target=tmp_path/'fixture.png'
    fig,ax=plt.subplots();ax.plot([0,1],[0,1])
    plot.save(fig,target,dict(missing=None),[source],identity,'Synthetic plotting fixture; not research evidence')
    side=json.loads(target.with_suffix('.json').read_text())
    assert side['numeric_data']['missing'] is None
    assert side['input_identity']==identity
    assert side['label']=='OFFLINE LOOKAHEAD-SELECTION DIAGNOSTIC'
    assert not side['new_execution'] and not side['gui_runtime_validated']
    fig,ax=plt.subplots()
    with pytest.raises(FileExistsError):plot.save(fig,target,{},[source],identity,'overwrite forbidden')
    plt.close(fig)


def test_package_allowlist_has_all_22_figures_no_binary_environment(tmp_path):
    cases=[dict(case_id=f'case_{i}/event',case_directory=f'case_{i}__event') for i in range(2)]
    (tmp_path/'case_manifest.json').write_text(json.dumps(dict(selected=cases)))
    allowed=package.allowed_artifacts(tmp_path)
    assert len([p for p in allowed if p.suffix=='.png'])==22
    assert len([p for p in allowed if p.name in [n+'.json' for n in package.FIGURES]])==22
    assert all(p.suffix not in ('.npy','.npz','.mp4','.pt') for p in allowed)
    assert all('environment' not in p.parts for p in allowed)
    assert Path('selector_definition.json') in allowed


def test_package_checks_missing_sources_before_creating_partial_output(tmp_path):
    (tmp_path/'case_manifest.json').write_text(json.dumps(dict(selected=[])))
    with pytest.raises(FileNotFoundError):package.package(tmp_path)
    assert not (tmp_path/'review_bundle').exists()
    (tmp_path/'review_bundle.zip').write_bytes(b'original')
    with pytest.raises(FileExistsError):package.package(tmp_path)
    assert (tmp_path/'review_bundle.zip').read_bytes()==b'original'

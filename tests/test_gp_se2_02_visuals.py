"""Synthetic interface/plot tests only; no experimental evidence or MPC solves."""
import importlib.util
import json
from pathlib import Path
import shutil
import zipfile

import numpy as np
from PIL import Image
import pytest
from shapely.geometry import box
import yaml
from reconciliation.gp_se2_environment import HospitalEnvironment

ROOT=Path(__file__).resolve().parents[1]
def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
plots=module('hard_transfer_plots_test',ROOT/'scripts/plot_gp_se2_02.py')
package=module('hard_transfer_package_test',ROOT/'scripts/package_gp_se2_02_review.py')
replay_fixture=module('hard_transfer_replay_fixture',ROOT/'tests/test_gp_se2_comparison_replay.py')
replay=replay_fixture.replay

def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value))

@pytest.fixture
def fixture_run(tmp_path):
    run=tmp_path/'synthetic';run.mkdir(); cfg={'plots':{'dpi':160},'footprint':{'radius_m':.2,'required_clearance_m':.05},
      'formulation':{'goal_position_tolerance':.15,'goal_yaw_tolerance':np.pi/12,'horizon_s':3.,'support_dt_s':.1}}
    (run/'config_snapshot.yaml').write_text(yaml.safe_dump(cfg))
    write(run/'source.json',{'synthetic':True,'environment_path':str(tmp_path/'environment')});write(run/'protocol.json',{'experiment':'GP-SE2-02','synthetic':True})
    row={'case_id':'synthetic/handoff_001','case_directory':'synthetic__handoff_001','role':'SYNTHETIC_ONLY'}
    write(run/'case_manifest.json',{'selected':[row]});case=run/'cases'/row['case_directory']
    t=np.linspace(0,3,31);poses=np.column_stack([.2*t,np.zeros((31,2))]);vel=np.tile([.2,0.,0.],(31,1))
    context={'B_world':[0.,0.,0.],'u_minus':[.2,0.],'previous_control':[.1,.3],
       'old_world':[[-.2,0.,0.],[0.,0.,0.],[.2,0.,0.]],'fresh_world':poses[1:].tolist()}
    write(case/'input_context.json',context);write(case/'goal_route.json',{'goal_world':[.6,0.,0.],'gates':[]})
    write(case/'actual_past_execution.json',{'poses_world':[[-.3,0.,0.],[0.,0.,0.]],'times_relative_to_B_s':[-1.5,0.],'synthetic':True})
    for seed in plots.SEEDS:
        write(case/'initializations'/f'{seed}.json',{'chart_reconstructed_poses':poses.tolist(),'chart_reconstructed_twists':vel.tolist()})
    for method in plots.METHODS:
        p=case/'methods'/method;available=method!='M3_GP_CONSTRAINED';valid=available and method!='SEED_ONLY';performed=valid
        solver={'status':'CANDIDATE' if available else 'NO_CANDIDATE','support_times':t.tolist(),
          'candidate_world':poses[1:].tolist() if available else None,'support_poses':None,'support_twists':None}
        write(p/'solver_result.json',solver)
        if available:np.save(p/'candidate_world.npy',poses[1:])
        write(p/'plan_validation.json',{'plan_valid':valid,'status':'SAMPLED_PLAN_VALID' if valid else 'PLAN_INVALID'})
        metrics={'candidate_available':available,'plan_valid':valid,'rollout_performed':performed,'rollout_success':performed,
           'primary_success':performed,'termination_reasons':[] if performed else ['PLAN_INVALID' if available else 'NO_CANDIDATE'],
           'dense_times_s':t.tolist() if performed else None,'dense_poses_world':poses.tolist() if performed else None,
           'environment':{'clearance_samples_m':[1.]*31} if performed else None}
        write(p/'metrics.json',metrics)
        if performed:write(p/'rollout/rollout.json',{'controller_reference_selections':[{'time_s':float(k)/10,'command':[.2,0.]} for k in range(30)]})
        if method in plots.GP_METHODS:
            for seed in plots.SEEDS:
                q=p/'starts'/seed;sr={'initialization':seed,'termination':'CONVERGED' if available else 'ITERATION_LIMIT','solve_wall_time_s':2.,
                  'post_solve_validation_time_s':.1,'objective_evaluations':100,'profiling':{'evaluator_cache_misses':100},
                  'derivative_calls':{'objective_gradient':30},'latest_support_poses':poses.tolist(),'latest_support_twists':vel.tolist(),
                  'support_poses':poses.tolist() if available else None,'support_twists':vel.tolist() if available else None}
                write(q/'solver_result.json',sr);write(q/'result_summary.json',{'initial_full_feasible':available and seed=='I1_DECEL','final_full_feasible':available});write(q/'full_acceptance.json',{'full_feasible':available})
                write(q/'setup.json',{'derivative_graph_construction_s':.02,'compilation_first_call_warmup_s':.8})
                write(q/'post_full_checks.json',{'rows':[{'iterate':'initial','objective':4.,'discovery_time_s':0.,'full_feasible':available and seed=='I1_DECEL'},
                  {'iterate':'final','objective':.2,'discovery_time_s':2.,'full_feasible':available}], 'full_validation_wall_time_s':.4})
    return run,case


def test_plot_all_six_methods_missing_values_equal_axes_and_numeric_agreement(fixture_run,monkeypatch):
    run,case=fixture_run;env=HospitalEnvironment(box(2,-2,2.2,2),box(-3,-3,3,3))
    monkeypatch.setattr(plots.HospitalEnvironment,'load',lambda p:env)
    result=plots.main(run)
    assert result=={'case_count':1,'image_count':12}
    manifest=plots.read(run/'plot_manifest.json');assert set(manifest['methods'])==set(plots.METHODS)
    provenance=plots.read(case/'plots/plot_provenance.json')
    assert len(provenance['images'])==12
    for row in manifest['images']:
        path=run/row['path'];side=plots.read(path.with_suffix('.json'))
        assert side['image_sha256']==plots.file_sha256(path)
        assert side['source_hashes'][str(run/'config_snapshot.yaml')]==plots.file_sha256(run/'config_snapshot.yaml')
        assert not side['new_execution']
        with Image.open(path) as im: assert min(im.info['dpi'])>=159.9
    for name in ('candidate_world_overlay','actual_rollout_overlay'):
        numeric=plots.read(case/'plots'/f'{name}.json')['numeric_data'];assert numeric['axes_world_m']==provenance['axes_world_m']
        assert numeric['methods']['M3_GP_CONSTRAINED']['actual_rollout_world'] is None
    command=plots.read(case/'plots/linear_command_vs_time.json')['numeric_data']
    assert command['M3_GP_CONSTRAINED'] is None and command['SEED_ONLY'] is None
    assert command['M0_NATIVE']['commands']==[.2]*31
    assert command['M0_NATIVE']['controller_memory']==.1
    cost=plots.read(case/'plots/solver_cost.json')['numeric_data']['M2_GP_NO_OBSTACLE/I0_FRESH']
    assert cost['counts']['primal_misses']==100
    history=plots.read(case/'plots/feasible_objective_history.json')['numeric_data']['M3_GP_CONSTRAINED/I0_FRESH']
    assert all(p['best_full_objective'] is None for p in history['points'])
    assert all(method in (run/'index.html').read_text() for method in plots.METHODS)
    with pytest.raises(FileExistsError):plots.main(run)
    packed=package.package(run)
    assert packed['gui_runtime_validated'] is False
    with zipfile.ZipFile(run/'review_bundle.zip') as archive:
        names=archive.namelist();assert len([p for p in names if p.endswith('.png')])==4
        assert not any(p.endswith('.npy') or '/environment/' in p or p.endswith('solver_result.json') for p in names)
    with pytest.raises(FileExistsError):package.package(run)


def test_actual_past_preserves_exact_boundary_and_no_future(tmp_path):
    case=tmp_path/'case';source=tmp_path/'source';path=source/'episodes/e/execution.csv';path.parent.mkdir(parents=True)
    path.write_text('state_id,sim_time_s,x,y,yaw\n0,0,-1,0,0\n1,1,0,0,0\n2,2,1,0,0\n3,3,2,0,0\n4,4,3,0,0\n5,5,4,0,0\n')
    context={'episode_id':'e','source_root':str(source),'source_files':[{'path':'episodes/e/execution.csv','sha256':plots.file_sha256(path)}],
       'switch_sim_time_s':4.,'switch_state_id':4,'B_world':[3.,0.,0.]}
    write(case/'input_context.json',context)
    result=plots.past_execution(case)
    assert result['times_relative_to_B_s']==[-3.,-2.,-1.,0.]
    assert result['state_ids']==[1,2,3,4]
    assert result['poses_world'][-1]==context['B_world']
    assert not result['future_samples_used']
    assert plots.read(case/'input_context.json')==context


def test_replay_hard_four_cases_six_methods_external_environment_and_fixed_representatives(tmp_path):
    run=replay_fixture.artifact.__wrapped__(tmp_path)
    old=plots.read(run/'case_manifest.json')['selected']
    expected=('episode_001_repeat_01/handoff_002','episode_013_repeat_01/handoff_024',
              'episode_014_repeat_01/handoff_026','episode_000_repeat_01/handoff_019')
    selected=[]
    for case_id in expected:
        directory=case_id.replace('/','__');target=run/'cases'/directory
        shutil.copytree(run/'cases'/old[1]['case_directory'],target)
        shutil.copytree(target/'methods/M0_NATIVE',target/'methods/SEED_ONLY')
        write(target/'actual_past_execution.json',{'poses_world':[[1.9,3.,.1],[2.,3.,.1]]})
        selected.append({'case_id':case_id,'case_directory':directory,'selected_group':'fixed'})
    write(run/'case_manifest.json',{'selected':selected});write(run/'protocol.json',{'experiment':'GP-SE2-02'})
    environment=tmp_path/'external_environment';shutil.move(run/'environment',environment)
    write(run/'source.json',{'environment_path':str(environment)})
    saved=replay.SavedComparison(run)
    assert saved.label==replay.HARD_LABEL and saved.methods[-1]=='SEED_ONLY'
    assert len(saved.cases)==4 and len(saved.methods)==6
    assert [saved.cases[i]['manifest']['case_id'] for i in saved.representative_indices]==list(expected[:2])
    assert len(saved.workspace_rings)==1
    saved.select(method_index=5);saved.seek_end();assert saved.method['poses'] is not None
    assert saved.case['past'][-1].tolist()==[2.,3.,.1]
    saved.verify_unchanged()
    (environment/'geometry/render_geometry.json').write_text('{}')
    with pytest.raises(ValueError,match='source changed'):saved.verify_unchanged()


def test_missing_planned_method_refuses_plotting(fixture_run):
    _,case=fixture_run;(case/'methods/SEED_ONLY/metrics.json').unlink()
    with pytest.raises(FileNotFoundError):plots.load_methods(case)

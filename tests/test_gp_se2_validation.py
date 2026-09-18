"""Synthetic artifact validation fixtures; no experimental performance evidence."""
from copy import deepcopy
import importlib.util
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
from PIL import Image
import pytest
import yaml
from shapely.geometry import LineString, box

from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_evaluation import METHODS, comparisons, evaluate_plan, evaluate_rollout
from reconciliation.gp_se2_formulation import GPProblem
from reconciliation.gp_se2_reference import prepare_reference
from reconciliation.online_mpc_adapter import selection_audit
from reconciliation.robotless_online import integrate_unicycle

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('gp_se2_validator_test',ROOT/'scripts/validate_gp_se2_01.py')
validator=importlib.util.module_from_spec(spec);spec.loader.exec_module(validator)


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(validator.canonical(value),indent=2,allow_nan=False))


def records(root):
    return [{'path':str(p.relative_to(root)),'sha256':validator.digest(p),'bytes':p.stat().st_size}
            for p in sorted(root.rglob('*')) if p.is_file()]


def rollout(candidate,frozen,candidate_path):
    pose=np.asarray(frozen['B_world'],float);u=np.asarray(frozen['u_minus']);states=[{'tick':0,'time_s':0.,'pose_world':pose.tolist()}]
    commands=[];solves=[]
    for tick in range(180):
        if tick%6==0:
            residual=np.asarray(candidate)-pose;residual[:,2]=(residual[:,2]+np.pi)%(2*np.pi)-np.pi
            nearest=int(np.argmin(np.sum(residual**2*[10,10,1],axis=1)))
            reference=np.asarray(candidate)[np.minimum(nearest+1+np.arange(5),len(candidate)-1)]
            solve={'tick':tick,'time_s':tick/60,'input_pose_world':pose.tolist(),
                'previous_control':frozen['previous_control'] if tick==0 else u.tolist(),
                'previous_control_after':u.tolist(),'command':u.tolist(),'success':True,
                'simulation_time_advanced_during_solve_s':0.,
                'selection':selection_audit(candidate,pose,reference,horizon=5,weights=[10,10,1])}
            solves.append(solve)
        commands.append({'tick':tick,'time_s':tick/60,'end_time_s':(tick+1)/60,'command':u.tolist(),
                         'solve_index':tick//6,'held':bool(tick%6)})
        pose=integrate_unicycle(pose,u,1/60);states.append({'tick':tick+1,'time_s':(tick+1)/60,'pose_world':pose.tolist()})
    return {'horizon_s':3.,'control_hz':10.,'integration_hz':60.,'states':states,'commands':commands,
        'controller_reference_selections':solves,'controller_failure_count':0,'initial_pose_world':frozen['B_world'],
        'initial_physical_command':frozen['u_minus'],'initial_previous_control':frozen['previous_control'],
        'candidate_world':np.asarray(candidate).tolist(),'candidate_capture_local':np.asarray(candidate).tolist(),
        'candidate_source':{'path':str(candidate_path),'sha256':validator.digest(candidate_path)},
        'official_gains':{'Q_WEIGHTS':[10,10,1],'R_WEIGHTS':[.1,.1]},'pose_snaps':0,'new_lightnav_updates':0,
        'resolved_limits':{'v_max':.8,'omega_max':3.,'a_v_max':2.,'a_omega_max':5.}}


@pytest.fixture
def frozen_run(tmp_path,monkeypatch):
    run=tmp_path/'synthetic_run';run.mkdir();cfg=yaml.safe_load((ROOT/'configs/gp_se2_01.yaml').read_text())
    (run/'config_snapshot.yaml').write_text(yaml.safe_dump(cfg))
    source_audit=tmp_path/'source_audit';write(source_audit/'source_validation.json',{'valid':True});write(source_audit/'source_hash_validation.json',{'valid':True})
    write(run/'environment/validation.json',{'valid':True,'kind':'synthetic implementation fixture'})
    env=HospitalEnvironment(LineString([(8,-8),(8,8)]),box(-10,-10,10,10))
    monkeypatch.setattr(validator.HospitalEnvironment,'load',lambda path:env)
    selected={'case_id':'episode_fixture/handoff_fixture','case_directory':'episode_fixture__handoff_fixture',
        'episode_id':'episode_fixture','handoff_id':'handoff_fixture','selected_group':'A_SMALL_STRAIGHT'}
    write(run/'case_manifest.json',{'selected':[selected],'selected_count':1,'selection_used_new_optimization_or_rollouts':False})
    code_paths=['src/reconciliation/gp_se2.py','src/reconciliation/gp_se2_rollout.py','scripts/lightnav/gp_se2_mpc_rollout.py']
    for code in code_paths:
        target=run/'implementation_sources'/code;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/code,target)
    write(run/'source.json',{'source_run':str(tmp_path/'synthetic_source'),'source_audit':str(source_audit),
        'source_audit_sha256':{p.name:validator.digest(p) for p in source_audit.glob('*.json')},
        'experiment_source_sha256':{code:validator.digest(ROOT/code) for code in code_paths},'environment_file_hashes':records(run/'environment')})
    write(run/'protocol.json',{'config_sha256':validator.digest(run/'config_snapshot.yaml'),'case_manifest_sha256':validator.digest(run/'case_manifest.json'),
        'methods':list(METHODS),'no_new_inference':True,'case_replacement_forbidden':True})
    case=run/'cases'/selected['case_directory'];case.mkdir(parents=True)
    native=np.column_stack([np.arange(31)*.02,np.zeros(31),np.zeros(31)])
    frozen={'case_id':selected['case_id'],'B_world':[0.,0.,0.],'u_minus':[.2,0.],
        'previous_control':[.2,0.],'original_capture_pose_world':[0.,0.,0.],'fresh_world':native.tolist()}
    monkeypatch.setattr(validator,'load_frozen_context',lambda *args:deepcopy(frozen))
    write(case/'input_context.json',frozen);np.save(case/'F_native.npy',native)
    prep=prepare_reference(native,frozen['B_world'],[10,10,1]);common=prep['common_world']
    np.save(case/'F_common.npy',common);write(case/'reference_preparation.json',prep)
    goal={'route_status':'NOT_REQUIRED_CLEAR_SHORTCUT','goal_world':native[-1].tolist(),'gates':[]}
    write(case/'goal_route.json',goal);write(case/'validation.json',{'historical_solve_audit':{'passed':True,'B_counterfactual_equality_required':False}})
    source_doc=json.loads((run/'source.json').read_text());source_doc['case_input_file_hashes']=records(run/'cases');write(run/'source.json',source_doc)
    problem=GPProblem(frozen['B_world'],[.2,0,0],common,native[-1],cfg['formulation'],lambda p:np.full(len(p),2.))
    starts=problem.initializations();rows=[];image_links=[]
    mpc=tmp_path/'synthetic_mpc_output';mpc.mkdir()
    fake_mpc_source=tmp_path/'synthetic_mpc.py';fake_mpc_source.write_text('# Explicit synthetic validator fixture, not actual official MPC execution.\n')
    fake_hash=validator.digest(fake_mpc_source);monkeypatch.setattr(validator,'PINNED_MPC_SHA256',fake_hash)
    settings={'Q_WEIGHTS':[10,10,1],'R_WEIGHTS':[.1,.1],'HORIZON':5}
    settings_sha=hashlib.sha256(json.dumps(settings,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    write(run/'mpc_request.json',{'kind':'synthetic fixture'});write(mpc/'request.json',{'kind':'synthetic fixture'})
    write(mpc/'provenance.json',{'request_sha256':validator.digest(run/'mpc_request.json'),'lightnav_sha':validator.PINNED_LIGHTNAV_SHA,'mpc_source_sha256':fake_hash,
        'mpc_source':str(fake_mpc_source),'official_settings':settings,'official_settings_sha256':settings_sha,
        'implementation_sha256':{code:validator.digest(ROOT/code) for code in code_paths[1:]},
        'resolved_limits':{'v_max':.8,'omega_max':3.,'a_v_max':2.,'a_omega_max':5.}})
    write(mpc/'summary.json',{'source_files_preserved':True,'official_source_unchanged':True})
    for method in METHODS:
        folder=case/'methods'/method;folder.mkdir(parents=True)
        candidate=(native if method=='M0_NATIVE' else common) if method in METHODS[:2] else None
        solver={'method':method,'candidate_world':candidate,'status':'REFERENCE_AVAILABLE' if candidate is not None else 'NO_FEASIBLE_CANDIDATE_FOUND',
                'attempts':[],'config':problem.config,'infeasibility_proven':False,'support_poses':None,'support_twists':None}
        if candidate is None:
            solver['attempts']=[{'initialization':i['name'],'initial_vector':problem.vector(i['poses'],i['twists']),
                'initial_support_poses':i['poses'],'initial_support_twists':i['twists'],'random_seed':0} for i in starts]
        write(folder/'solver_result.json',solver);write(folder/'constraint_report.json',None)
        plan=evaluate_plan(method,candidate,solver,common,frozen['B_world'],[.2,0,0],goal,env,cfg);write(folder/'plan_validation.json',plan)
        if candidate is not None:
            np.save(folder/'candidate_world.npy',candidate);actual=rollout(candidate,frozen,folder/'candidate_world.npy')
            actual['official_settings_sha256']=settings_sha
            write(folder/'rollout/rollout.json',actual);write(mpc/selected['case_directory']/method/'rollout.json',actual);metrics=evaluate_rollout(actual,goal,env,cfg)
        else:metrics={'primary_success':False,'failure_reasons':['no_candidate'],'execution':None,'environment':None,'route':None,
                     'rollout_performed':False,'status':solver['status']}
        if candidate is None:write(mpc/selected['case_directory']/method/'status.json',{'status':'NO_CANDIDATE','rollout_performed':False})
        write(folder/'metrics.json',metrics);write(folder/'hashes.json',{'files':records(folder)})
        plotdir=folder/'plots';plotdir.mkdir();required=validator.COMMON_PLOTS+(validator.GP_PLOTS if method.startswith(('M2','M3')) else ())
        for name in required:
            Image.new('RGB',(4,4),color='white').save(plotdir/(name+'.png'),dpi=(160,160))
            image_links.append(str((plotdir/(name+'.png')).relative_to(run)))
        write(plotdir/'plot_provenance.json',{'config_sha256':validator.digest(run/'config_snapshot.yaml'),
            'source_sha256':validator.digest(run/'source.json'),'metrics_sha256':validator.digest(folder/'metrics.json'),'images':records(plotdir)})
        rows.append({'case_id':selected['case_id'],'method':method,'candidate_found':candidate is not None,'plan_valid':plan['plan_valid'],
                     'primary_success':metrics['primary_success'],'failure_reasons':metrics['failure_reasons'],'rollout_metrics':metrics})
    write(run/'aggregate/method_results.json',rows);summary=comparisons(rows);summary['selected_cases']=1;write(run/'aggregate/summary.json',summary)
    write(run/'optimization_completion.json',{'all_selected_cases_attempted':True});write(run/'evaluation_completion.json',{'all_cases_methods_evaluated':True,'mpc_output':str(mpc),'mpc_output_summary_sha256':validator.digest(mpc/'summary.json')})
    write(mpc/'output_hashes.json',{'files':records(mpc)})
    (run/'index.html').write_text('<html>'+''.join(f'<a href="{x}">synthetic fixture</a>' for x in image_links)+'</html>')
    return run


def method_path(run,method):return run/'cases/episode_fixture__handoff_fixture/methods'/method


def test_complete_synthetic_artifact_passes_without_claiming_gui(frozen_run):
    result=validator.validate_run(frozen_run)
    assert result['valid'],result['errors']
    assert result['validated_method_records']==5 and result['required_images_checked']==46
    assert result['no_new_mpc_optimization_or_inference']
    assert not result['gui_validated']
    assert result['operational_status']=='GP_SE2_01_COMPLETED_WITH_LIMITATIONS'


def test_corrupt_candidate_hash_is_rejected(frozen_run):
    path=method_path(frozen_run,'M0_ADAPTER')/'candidate_world.npy';value=np.load(path);value[0,0]+=.01;np.save(path,value)
    result=validator.validate_run(frozen_run)
    assert not result['valid']
    assert any('hash mismatch candidate_world.npy' in e for e in result['errors'])


def test_missing_required_no_candidate_image_is_rejected(frozen_run):
    (method_path(frozen_run,'M3_GP_CONSTRAINED')/'plots/factor_costs.png').unlink()
    result=validator.validate_run(frozen_run)
    assert not result['valid'] and any('factor_costs.png' in e for e in result['errors'])


def test_raw_fallback_for_no_candidate_is_rejected(frozen_run):
    target=method_path(frozen_run,'M3_GP_CONSTRAINED')/'rollout';target.mkdir();write(target/'rollout.json',{'fallback':'RAW'})
    result=validator.validate_run(frozen_run)
    assert not result['valid'] and any('fallback rollout' in e for e in result['errors'])


def test_recomputed_primary_outcome_rejects_fabricated_success(frozen_run):
    path=method_path(frozen_run,'M0_NATIVE')/'metrics.json';metrics=json.loads(path.read_text());metrics['primary_success']=False;write(path,metrics)
    result=validator.validate_run(frozen_run)
    assert not result['valid'] and any('recomputed rollout metric primary_success' in e for e in result['errors'])


def test_exact_unicycle_reconstruction_rejects_pose_snap(frozen_run):
    path=method_path(frozen_run,'M0_NATIVE')/'rollout/rollout.json';value=json.loads(path.read_text());value['states'][1]['pose_world'][0]+=.2;write(path,value)
    result=validator.validate_run(frozen_run)
    assert not result['valid'] and any('execution reconstruction' in e for e in result['errors'])


def test_implementation_snapshot_corruption_is_rejected(frozen_run):
    path=frozen_run/'implementation_sources/src/reconciliation/gp_se2.py';path.write_text('changed')
    result=validator.validate_run(frozen_run)
    assert not result['valid'] and any('implementation snapshot mismatch' in e for e in result['errors'])


def test_low_dpi_and_broken_index_links_are_rejected(frozen_run):
    path=method_path(frozen_run,'M0_NATIVE')/'plots/environment_context.png';Image.new('RGB',(4,4)).save(path,dpi=(72,72))
    (frozen_run/'index.html').write_text('<a href="missing.png">missing</a>')
    result=validator.validate_run(frozen_run)
    assert not result['valid'] and any('below160dpi' in e for e in result['errors'])
    assert any('index broken' in e for e in result['errors'])


def test_validation_write_refuses_overwrite(frozen_run,capsys):
    assert validator.main([str(frozen_run),'--write'])==0
    original=(frozen_run/'validation.json').read_bytes()
    assert validator.main([str(frozen_run),'--write'])==1
    assert (frozen_run/'validation.json').read_bytes()==original
    assert 'refusing to overwrite' in capsys.readouterr().out


def test_gui_sidecars_are_bound_to_this_run_and_every_method(frozen_run,tmp_path):
    gui=tmp_path/'synthetic_gui';gui.mkdir();case_id='episode_fixture/handoff_fixture';captures=[]
    for method in METHODS:
        png=gui/(method+'.png');Image.new('RGB',(4,4),'white').save(png)
        metric=method_path(frozen_run,method)/'metrics.json';sha=validator.digest(png)
        write(png.with_suffix('.json'),{'case_id':case_id,'method':method,'experiment_run':str(frozen_run),
            'image_sha256':sha,'config_sha256':validator.digest(frozen_run/'config_snapshot.yaml'),
            'metric_path':str(metric),'metric_sha256':validator.digest(metric)})
        captures.append({'path':png.name,'case_id':case_id,'method':method,'sha256':sha})
    report={'status':'GP_SE2_01_COMPARISON_REPLAY_RUNTIME_VALIDATED','experiment_run':str(frozen_run),
        'label':'OFFLINE COUNTERFACTUAL HANDOFF COMPARISON','source_unchanged':True,'display_playback_only':True,
        'new_inference_count':0,'new_mpc_solves':0,'generated_execution_states':0,
        'captures':captures,'expected_screenshot_count':5,'source_hashes':[],
        'representatives':[{'selected_case_id':case_id}]}
    path=gui/'runtime_validation.json';write(path,report)
    result=validator.validate_run(frozen_run,path)
    assert result['valid'],result['errors']
    assert result['gui_validated']
    report['experiment_run']=str(tmp_path/'another_run');write(path,report)
    result=validator.validate_run(frozen_run,path)
    assert not result['valid'] and any('another experiment run' in e for e in result['errors'])


def test_case_input_hash_and_official_controller_settings_corruption(frozen_run):
    path=frozen_run/'cases/episode_fixture__handoff_fixture/input_context.json'
    data=json.loads(path.read_text());data['previous_control']=[.3,.1];write(path,data)
    result=validator.validate_run(frozen_run)
    assert not result['valid'] and any('frozen case inputs: hash mismatch' in e for e in result['errors'])
    assert any('different previous_control' in e for e in result['errors'])


def test_nested_rollout_hash_manifest_is_in_outer_method_coverage(frozen_run):
    folder=method_path(frozen_run,'M0_NATIVE');completion=json.loads((frozen_run/'evaluation_completion.json').read_text())
    mpc=Path(completion['mpc_output']);original=mpc/'episode_fixture__handoff_fixture/M0_NATIVE'
    nested={'files':records(folder/'rollout')}
    write(folder/'rollout/hashes.json',nested);write(original/'hashes.json',nested)
    outer=[r for r in records(folder) if r['path']!='hashes.json' and not r['path'].startswith('plots/')]
    write(folder/'hashes.json',{'files':outer})
    write(mpc/'output_hashes.json',{'files':[r for r in records(mpc) if r['path']!='output_hashes.json']})
    result=validator.validate_run(frozen_run)
    assert result['valid'],result['errors']

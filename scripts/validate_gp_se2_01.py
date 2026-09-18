#!/usr/bin/env python3
"""Independently validate a frozen GP-SE2-01 run without inference/MPC/optimization.

Checks saved artifacts, source authentication, reference preparation, exact held
unicycle reconstruction, official row-selection semantics, geometry/task metrics,
failure-first aggregates, image provenance and local HTML links. This validates
artifact consistency and the explicitly sampled evaluation, not physical safety.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import hashlib
from html.parser import HTMLParser
import json
from pathlib import Path
import sys
from urllib.parse import unquote, urlsplit

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
import numpy as np
from PIL import Image
import yaml

from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_evaluation import METHODS, comparisons, evaluate_plan, evaluate_rollout
from reconciliation.gp_se2_reference import prepare_reference
from reconciliation.gp_se2_rollout import load_frozen_context
from reconciliation.online_mpc_adapter import selection_audit, PINNED_LIGHTNAV_SHA, PINNED_MPC_SHA256
from reconciliation.robotless_online import integrate_unicycle
from reconciliation.se2 import local_trajectory_to_world, wrap_angle

COMMON_PLOTS=('environment_context','candidate_overlay','rollout_overlay','boundary_zoom',
              'clearance_vs_time','linear_command_vs_time','angular_command_vs_time','goal_error_vs_time')
GP_PLOTS=('interpolated_lateral_velocity','constraint_violations','factor_costs')


def digest(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def read(path):return json.loads(Path(path).read_text())


def canonical(value):
    if isinstance(value,np.ndarray):return canonical(value.tolist())
    if isinstance(value,np.generic):return canonical(value.item())
    if isinstance(value,dict):return {k:canonical(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [canonical(v) for v in value]
    return value


def equivalent(a,b,tolerance=1e-9):
    """Recursive metric comparison without accepting NaN or missing fields."""
    a,b=canonical(a),canonical(b)
    if isinstance(a,dict) and isinstance(b,dict):
        return a.keys()==b.keys() and all(equivalent(a[k],b[k],tolerance) for k in a)
    if isinstance(a,list) and isinstance(b,list):
        return len(a)==len(b) and all(equivalent(x,y,tolerance) for x,y in zip(a,b))
    if isinstance(a,bool) or isinstance(b,bool):return type(a)==type(b) and a==b
    if isinstance(a,(int,float)) and isinstance(b,(int,float)):
        return bool(np.isfinite(a) and np.isfinite(b) and np.isclose(a,b,atol=tolerance,rtol=1e-9))
    return a==b


def same_pose(a,b,tolerance=1e-9):
    a,b=np.asarray(a,float),np.asarray(b,float)
    return bool(a.shape==b.shape and a.shape[-1:]==(3,) and np.isfinite(a).all() and np.isfinite(b).all()
        and np.allclose(a[...,:2],b[...,:2],atol=tolerance,rtol=0)
        and np.allclose(wrap_angle(a[...,2]-b[...,2]),0,atol=tolerance,rtol=0))


def contained(base,path):
    base=Path(base).resolve();result=(base/path).resolve()
    if not result.is_relative_to(base):raise ValueError(f'path escaped artifact root: {path}')
    return result


class Checks:
    def __init__(self):self.errors=[];self.count=0
    def require(self,condition,message):
        self.count+=1
        if not condition:self.errors.append(message)
        return bool(condition)
    @contextmanager
    def section(self,label):
        try:yield
        except Exception as exc:
            self.errors.append(f'{label}: {type(exc).__name__}: {exc}')


def verify_records(base,records,checks,label):
    seen=set()
    for record in records:
        relative=record['path'];checks.require(relative not in seen,f'{label}: duplicate hash entry {relative}');seen.add(relative)
        path=contained(base,relative)
        checks.require(path.is_file(),f'{label}: missing {relative}')
        if path.is_file():
            checks.require(digest(path)==record['sha256'],f'{label}: hash mismatch {relative}')
            if 'bytes' in record:checks.require(path.stat().st_size==record['bytes'],f'{label}: byte count {relative}')
    return seen


def validate_rollout(rollout,candidate,frozen,config,checks,label):
    """Audit actual new execution; never re-solve or substitute saved RAW execution."""
    control=float(config['rollout']['control_hz']);integration=float(config['rollout']['integration_hz']);horizon=float(config['formulation']['horizon_s'])
    stride=round(integration/control);count=round(horizon*integration)
    checks.require(equivalent([rollout['horizon_s'],rollout['control_hz'],rollout['integration_hz']],[horizon,control,integration]),f'{label}: schedule configuration')
    checks.require(same_pose(rollout['initial_pose_world'],frozen['B_world']),f'{label}: different initial B')
    checks.require(equivalent(rollout['initial_physical_command'],frozen['u_minus']),f'{label}: different u_minus')
    checks.require(equivalent(rollout['initial_previous_control'],frozen['previous_control']),f'{label}: different previous_control')
    checks.require(rollout['pose_snaps']==0 and rollout['new_lightnav_updates']==0,f'{label}: pose snap or new inference')
    checks.require(same_pose(rollout['candidate_world'],candidate),f'{label}: rollout candidate differs')
    checks.require(same_pose(local_trajectory_to_world(frozen['original_capture_pose_world'],rollout['candidate_capture_local']),candidate),f'{label}: candidate frame roundtrip')
    states,commands,solves=rollout['states'],rollout['commands'],rollout['controller_reference_selections']
    if not checks.require(len(states)==count+1 and len(commands)==count and len(solves)==count//stride,f'{label}: state/command/solve counts'):return
    checks.require(same_pose(states[0]['pose_world'],frozen['B_world']),f'{label}: first state is not B')
    checks.require(equivalent([s['time_s'] for s in states],np.arange(count+1)/integration),f'{label}: 60Hz state timestamps')
    checks.require(equivalent([s['tick'] for s in states],list(range(count+1))),f'{label}: state ticks')
    previous=frozen['previous_control']
    for index,solve in enumerate(solves):
        tick=index*stride
        checks.require(equivalent([solve['time_s'],solve['tick']],[index/control,tick]),f'{label}: solve {index} schedule')
        checks.require(same_pose(solve['input_pose_world'],states[tick]['pose_world']),f'{label}: solve {index} pose')
        checks.require(equivalent(solve['previous_control'],previous),f'{label}: solve {index} controller memory')
        checks.require(solve['simulation_time_advanced_during_solve_s']==0,f'{label}: solve advanced simulation')
        previous=solve['previous_control_after']
        checks.require(equivalent(previous,solve['command']),f'{label}: poll memory differs from command')
        reference=solve['selection']['reference_world']
        audited=selection_audit(candidate,solve['input_pose_world'],reference,horizon=len(reference),weights=rollout['official_gains']['Q_WEIGHTS'])
        for key in ('indices','nearest_index','endpoint_repeated','nearest_is_final_row'):
            checks.require(audited[key]==solve['selection'][key],f'{label}: solve {index} selected {key}')
    for tick,command in enumerate(commands):
        checks.require(equivalent([command['tick'],command['time_s'],command['end_time_s']],[tick,tick/integration,(tick+1)/integration]),f'{label}: command {tick} timestamps')
        checks.require(command['solve_index']==tick//stride and command['held']==bool(tick%stride),f'{label}: command {tick} hold schedule')
        checks.require(equivalent(command['command'],solves[tick//stride]['command']),f'{label}: command {tick} differs from solve')
        predicted=integrate_unicycle(states[tick]['pose_world'],command['command'],1/integration)
        checks.require(same_pose(predicted,states[tick+1]['pose_world'],1e-10),f'{label}: execution reconstruction at tick {tick}')
    checks.require(rollout['controller_failure_count']==sum(not s['success'] for s in solves),f'{label}: controller failure count')


class LocalLinks(HTMLParser):
    def __init__(self):super().__init__();self.links=[]
    def handle_starttag(self,tag,attrs):
        for name,value in attrs:
            if name in ('href','src') and value:self.links.append(value)


def validate_images(run,target,method,checks):
    plots=target/'plots';provenance=read(plots/'plot_provenance.json')
    for key,path in [('config_sha256',run/'config_snapshot.yaml'),('source_sha256',run/'source.json'),('metrics_sha256',target/'metrics.json')]:
        checks.require(provenance[key]==digest(path),f'{target.name}: plot provenance {key}')
    images=provenance['images']
    if isinstance(images,dict):images=[{'path':key,**(value if isinstance(value,dict) else {'sha256':value})} for key,value in images.items()]
    recorded=verify_records(plots,images,checks,f'{target}: plots')
    required=COMMON_PLOTS+(GP_PLOTS if method.startswith(('M2','M3')) else ())
    for name in required:
        path=plots/(name+'.png');checks.require(path.name in recorded,f'{target}: plot hash missing {path.name}')
        checks.require(path.is_file(),f'{target}: required image missing {path.name}')
        if path.is_file():
            with Image.open(path) as image:
                checks.require(image.format=='PNG',f'{path}: not PNG')
                dpi=image.info.get('dpi',(0.,0.))
                checks.require(len(dpi)==2 and min(dpi)>=159.9,f'{path}: PNG resolution below160dpi')
                image.verify()
    return len(required)


def validate_index(run,checks):
    path=run/'index.html';checks.require(path.is_file(),'run index.html missing')
    if not path.is_file():return 0
    html=LocalLinks();html.feed(path.read_text());count=0
    for link in html.links:
        parsed=urlsplit(link)
        if parsed.scheme or parsed.netloc or not parsed.path:continue
        target=contained(run,unquote(parsed.path))
        checks.require(target.is_file(),f'index broken local link: {link}');count+=1
    checks.require(count>0,'index contains no local artifact links')
    return count


def validate_run(run,gui_validation=None):
    run=Path(run).resolve();checks=Checks();code_drift=[];rows=[];image_count=0;gui_validated=False
    mpc_output=None;mpc_provenance=None
    try:
        cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text());protocol=read(run/'protocol.json')
        source=read(run/'source.json');manifest=read(run/'case_manifest.json')
    except Exception as exc:return {'valid':False,'errors':[f'run metadata: {type(exc).__name__}: {exc}'],'run':str(run)}
    with checks.section('frozen protocol'):
        checks.require(digest(run/'config_snapshot.yaml')==protocol['config_sha256'],'frozen config hash mismatch')
        checks.require(digest(run/'case_manifest.json')==protocol['case_manifest_sha256'],'frozen case manifest hash mismatch')
        checks.require(protocol['methods']==list(METHODS),'protocol method accounting')
        checks.require(protocol['no_new_inference'] and protocol['case_replacement_forbidden'],'protocol permits inference or case replacement')
        checks.require(manifest['selected_count']==len(manifest['selected']),'selected case count')
        checks.require(manifest['selection_used_new_optimization_or_rollouts'] is False,'outcome-dependent case selection')
        case_ids=[r['case_id'] for r in manifest['selected']]
        checks.require(len(case_ids)==len(set(case_ids)),'duplicate selected case')
        for relative,sha in source['experiment_source_sha256'].items():
            snapshot=contained(run/'implementation_sources',relative)
            checks.require(snapshot.is_file() and digest(snapshot)==sha,f'implementation snapshot mismatch: {relative}')
            current=contained(ROOT,relative)
            if not current.is_file() or digest(current)!=sha:code_drift.append(relative)
        for name,sha in source['source_audit_sha256'].items():
            path=contained(source['source_audit'],name)
            checks.require(path.is_file() and digest(path)==sha,f'source audit mismatch: {name}')
        for name in ('source_validation.json','source_hash_validation.json'):
            checks.require(read(Path(source['source_audit'])/name)['valid'],f'failed source audit: {name}')
        verify_records(run/'cases',source['case_input_file_hashes'],checks,'frozen case inputs')
        verify_records(run/'environment',source['environment_file_hashes'],checks,'frozen environment')
        ev=read(run/'environment/validation.json')
        checks.require(ev.get('valid',ev.get('passed',False)),'environment validation failed')
    with checks.section('official MPC runtime provenance'):
        completion=read(run/'evaluation_completion.json');mpc_output=Path(completion['mpc_output'])
        checks.require(digest(mpc_output/'summary.json')==completion['mpc_output_summary_sha256'],'external MPC summary hash mismatch')
        mpc_summary=read(mpc_output/'summary.json');mpc_provenance=read(mpc_output/'provenance.json')
        checks.require(digest(run/'mpc_request.json')==mpc_provenance['request_sha256'],'frozen MPC request hash mismatch')
        checks.require(equivalent(read(run/'mpc_request.json'),read(mpc_output/'request.json')),'actual MPC request differs')
        checks.require(mpc_summary['source_files_preserved'] and mpc_summary['official_source_unchanged'],'external MPC changed source')
        checks.require(mpc_provenance['lightnav_sha']==PINNED_LIGHTNAV_SHA and mpc_provenance['mpc_source_sha256']==PINNED_MPC_SHA256,'official MPC pin mismatch')
        checks.require(digest(mpc_provenance['mpc_source'])==PINNED_MPC_SHA256,'official MPC external source changed after execution')
        settings_sha=hashlib.sha256(json.dumps(mpc_provenance['official_settings'],sort_keys=True,separators=(',',':')).encode()).hexdigest()
        checks.require(settings_sha==mpc_provenance['official_settings_sha256'],'official MPC settings hash mismatch')
        for relative,sha in mpc_provenance['implementation_sha256'].items():
            checks.require(source['experiment_source_sha256'].get(relative)==sha,f'MPC runtime implementation differs from freeze: {relative}')
        verify_records(mpc_output,read(mpc_output/'output_hashes.json')['files'],checks,'external MPC artifacts')
    try:env=HospitalEnvironment.load(run/'environment')
    except (OSError,ValueError,KeyError) as exc:
        checks.errors.append(f'environment load: {exc}');env=None
    for selected in manifest.get('selected',[]):
        label=selected.get('case_id','unknown_case');case=None
        frozen=native=common=goal_route=None
        with checks.section(f'{label}: case path'):
            case=contained(run/'cases',selected['case_directory'])
        if case is None:continue
        with checks.section(f'{label}: inputs'):
            frozen=read(case/'input_context.json');native=np.load(case/'F_native.npy',allow_pickle=False);common=np.load(case/'F_common.npy',allow_pickle=False)
            fresh=load_frozen_context(source['source_run'],selected['episode_id'],selected['handoff_id'])
            checks.require(equivalent(frozen,fresh),f'{label}: frozen input context differs from authenticated source')
            checks.require(np.array_equal(native,np.asarray(fresh['fresh_world'])),f'{label}: native FRESH was changed')
            checks.require(common.shape==(30,3) and np.isfinite(common).all(),f'{label}: common reference grid')
            prep=prepare_reference(native,frozen['B_world'],[10.,10.,1.],first_time_s=cfg['reference']['first_time_s'],horizon_s=cfg['formulation']['horizon_s'],output_dt_s=cfg['reference']['output_dt_s'])
            checks.require(equivalent(read(case/'reference_preparation.json'),prep),f'{label}: reference preparation does not reproduce')
            checks.require(np.array_equal(common,prep['common_world']),f'{label}: F_common differs from declared preparation')
            goal_route=read(case/'goal_route.json')
            checks.require(np.array_equal(np.asarray(goal_route['goal_world']),native[-1]),f'{label}: goal redefined from original FRESH endpoint')
            checks.require(goal_route['route_status']!='UNKNOWN',f'{label}: primary selected unknown route')
            historical=read(case/'validation.json')['historical_solve_audit']
            checks.require(historical['passed'],f'{label}: historical official solve audit failed')
            checks.require(historical['B_counterfactual_equality_required'] is False,f'{label}: conflated historical and counterfactual solves')
        if any(value is None for value in (frozen,native,common,goal_route)):continue
        methods={}
        for method in METHODS:
            target=case/'methods'/method;mlabel=f'{label}/{method}'
            with checks.section(mlabel):
                solver=read(target/'solver_result.json');plan=read(target/'plan_validation.json');metrics=read(target/'metrics.json')
                records=verify_records(target,read(target/'hashes.json')['files'],checks,mlabel)
                actual={str(p.relative_to(target)) for p in target.rglob('*') if p.is_file() and p.relative_to(target).as_posix()!='hashes.json' and 'plots' not in p.relative_to(target).parts}
                checks.require(actual==records,f'{mlabel}: method file/hash coverage differs')
                for required in ('solver_result.json','constraint_report.json','plan_validation.json','metrics.json'):
                    checks.require(required in records,f'{mlabel}: unhashed {required}')
                exists=(target/'candidate_world.npy').is_file();candidate=np.load(target/'candidate_world.npy',allow_pickle=False) if exists else None
                methods[method]=solver
                checks.require((solver['candidate_world'] is not None)==exists,f'{mlabel}: candidate accounting')
                if exists:
                    checks.require('candidate_world.npy' in records,f'{mlabel}: candidate file unhashed')
                    checks.require(same_pose(candidate,solver['candidate_world']),f'{mlabel}: solver/candidate disagreement')
                    checks.require(candidate.ndim==2 and candidate.shape[1:]==(3,) and np.isfinite(candidate).all(),f'{mlabel}: nonfinite candidate')
                    if method!='M0_NATIVE':checks.require(candidate.shape==(30,3),f'{mlabel}: unequal MPC sample count')
                    if method in METHODS[:2]:checks.require(np.array_equal(candidate,native if method=='M0_NATIVE' else common),f'{mlabel}: baseline changed')
                    rollout=read(target/'rollout/rollout.json')
                    checks.require('rollout/rollout.json' in records,f'{mlabel}: rollout unhashed')
                    checks.require(rollout['candidate_source']['sha256']==digest(target/'candidate_world.npy'),f'{mlabel}: rollout candidate source hash')
                    if mpc_provenance is not None:
                        checks.require(rollout['official_settings_sha256']==mpc_provenance['official_settings_sha256'],f'{mlabel}: changed official settings')
                        settings=mpc_provenance['official_settings']
                        checks.require(equivalent(rollout['official_gains']['Q_WEIGHTS'],settings['Q_WEIGHTS']) and equivalent(rollout['official_gains']['R_WEIGHTS'],settings['R_WEIGHTS']),f'{mlabel}: changed MPC gains')
                        checks.require(equivalent(rollout['resolved_limits'],mpc_provenance['resolved_limits']),f'{mlabel}: changed MPC limits')
                        for solve in rollout['controller_reference_selections']:
                            checks.require(len(solve['selection']['reference_world'])==settings['HORIZON'],f'{mlabel}: changed official MPC horizon')
                    if mpc_output is not None:
                        original=mpc_output/selected['case_directory']/method
                        for copied in (target/'rollout').rglob('*'):
                            if copied.is_file():
                                expected=contained(original,str(copied.relative_to(target/'rollout')))
                                checks.require(expected.is_file() and digest(expected)==digest(copied),f'{mlabel}: copied rollout differs from original MPC output: {copied.name}')
                    validate_rollout(rollout,candidate,frozen,cfg,checks,mlabel)
                    if env is not None:
                        calculated=evaluate_rollout(rollout,goal_route,env,cfg)
                        for key in calculated:checks.require(key in metrics and equivalent(metrics[key],calculated[key]),f'{mlabel}: recomputed rollout metric {key}')
                else:
                    checks.require(method not in METHODS[:2],f'{mlabel}: raw baseline missing')
                    checks.require(not (target/'rollout').exists(),f'{mlabel}: no-candidate method has a fallback rollout')
                    checks.require(not metrics['primary_success'] and metrics['failure_reasons']==['no_candidate'] and metrics.get('rollout_performed') is False,f'{mlabel}: no-candidate outcome concealed')
                    checks.require(solver['status']=='NO_FEASIBLE_CANDIDATE_FOUND',f'{mlabel}: no-candidate solver status')
                    if mpc_output is not None:
                        skipped=read(mpc_output/selected['case_directory']/method/'status.json')
                        checks.require(skipped['status']=='NO_CANDIDATE' and skipped['rollout_performed'] is False,f'{mlabel}: external MPC used fallback')
                if method not in METHODS[:2]:
                    checks.require(len(solver['attempts'])==cfg['formulation']['initializations'],f'{mlabel}: initialization accounting')
                    checks.require(solver.get('infeasibility_proven') is False,f'{mlabel}: numerical failure claimed infeasibility proof')
                    for key,value in cfg['formulation'].items():checks.require(equivalent(solver['config'][key],value),f'{mlabel}: changed solver config {key}')
                if method.startswith(('M2','M3')) and exists:
                    support=np.load(target/'support_states.npy',allow_pickle=False);twists=np.load(target/'velocity_states.npy',allow_pickle=False)
                    checks.require(support.shape==(31,3) and twists.shape==(31,3),f'{mlabel}: support grid size')
                    checks.require(same_pose(support,solver['support_poses']) and equivalent(twists,solver['support_twists']),f'{mlabel}: state arrays differ from solver')
                    checks.require(same_pose(candidate,support[1:]),f'{mlabel}: MPC reference not support future grid')
                if env is not None:
                    u=frozen['u_minus'];p=evaluate_plan(method,candidate,solver,common,frozen['B_world'],[u[0],0,u[1]],goal_route,env,cfg)
                    checks.require(equivalent(plan,p),f'{mlabel}: independently recomputed plan differs')
                rows.append(dict(case_id=label,method=method,candidate_found=exists,plan_valid=plan['plan_valid'],primary_success=metrics['primary_success'],failure_reasons=metrics['failure_reasons'],
                    optimizer_wall_s=solver['total_optimization_and_check_wall_s'],deformation=plan.get('deformation'),rollout_metrics=metrics))
                image_count+=validate_images(run,target,method,checks)
        with checks.section(f'{label}: ablation fairness'):
            m2,m3=methods['M2_GP_NO_OBSTACLE'],methods['M3_GP_CONSTRAINED']
            checks.require(equivalent(m2['config'],m3['config']),f'{label}: M2/M3 configuration differs')
            for a,b in zip(m2['attempts'],m3['attempts']):
                for key in ('initialization','initial_vector','initial_support_poses','initial_support_twists','random_seed'):
                    checks.require(equivalent(a[key],b[key]),f'{label}: M2/M3 initialization differs: {key}')
    with checks.section('aggregate accounting'):
        expected={(r['case_id'],m) for r in manifest['selected'] for m in METHODS}
        checks.require({(r['case_id'],r['method']) for r in rows}==expected,'missing per-method evaluation')
        saved=read(run/'aggregate/method_results.json')
        checks.require(len(saved)==len(expected) and {(r['case_id'],r['method']) for r in saved}==expected,'aggregate method coverage')
        lookup={(r['case_id'],r['method']):r for r in rows}
        for row in saved:
            matched=lookup[(row['case_id'],row['method'])]
            for key in ('candidate_found','plan_valid','primary_success','failure_reasons','optimizer_wall_s','deformation','rollout_metrics'):
                checks.require(equivalent(row[key],matched[key]),f"aggregate differs: {row['case_id']}/{row['method']}/{key}")
        recomputed=comparisons(rows);summary=read(run/'aggregate/summary.json')
        for key,value in recomputed.items():checks.require(equivalent(value,summary[key]),f'aggregate comparison mismatch: {key}')
        checks.require(summary['selected_cases']==len(manifest['selected']),'summary selected count')
        checks.require(read(run/'optimization_completion.json')['all_selected_cases_attempted'] is True,'optimization incomplete')
        checks.require(read(run/'evaluation_completion.json')['all_cases_methods_evaluated'] is True,'evaluation incomplete')
    local_links=0
    with checks.section('HTML index'):local_links=validate_index(run,checks)
    if gui_validation is not None:
        with checks.section('Isaac GUI validation'):
            gui_path=Path(gui_validation).resolve();gui=read(gui_path)
            gui_validated=gui.get('status')=='GP_SE2_01_COMPARISON_REPLAY_RUNTIME_VALIDATED'
            checks.require(gui_validated,'Isaac GUI validation did not pass')
            checks.require(Path(gui['experiment_run']).resolve()==run,'Isaac GUI belongs to another experiment run')
            checks.require(gui['label']=='OFFLINE COUNTERFACTUAL HANDOFF COMPARISON','Isaac GUI label mismatch')
            checks.require(gui['source_unchanged'] and gui['display_playback_only'],'Isaac GUI source/playback audit')
            checks.require(all(gui[k]==0 for k in ('new_inference_count','new_mpc_solves','generated_execution_states')),'Isaac GUI generated new experiment data')
            checks.require(len(gui['captures'])==gui['expected_screenshot_count'] and len(gui['captures'])>0,'Isaac GUI screenshot coverage')
            for record in gui['source_hashes']:
                path=Path(record['path'])
                checks.require(path.is_file() and digest(path)==record['sha256'],f'Isaac GUI source hash mismatch: {path}')
            for record in gui['captures']:
                path=contained(gui_path.parent,record['path'])
                checks.require(path.is_file() and digest(path)==record['sha256'],f'Isaac GUI screenshot hash mismatch: {path}')
                sidecar=read(path.with_suffix('.json'))
                checks.require(sidecar['image_sha256']==record['sha256'] and sidecar['config_sha256']==digest(run/'config_snapshot.yaml'),'Isaac screenshot sidecar mismatch')
                checks.require(Path(sidecar['experiment_run']).resolve()==run,'Isaac screenshot belongs to another run')
                checks.require(sidecar['case_id']==record['case_id'] and sidecar['method']==record['method'],'Isaac screenshot case/method mismatch')
                metric=Path(sidecar['metric_path']).resolve()
                checks.require(metric.is_relative_to(run) and metric.is_file() and digest(metric)==sidecar['metric_sha256'],'Isaac screenshot metric hash mismatch')
            expected_cases={r['selected_case_id'] for r in gui['representatives'] if r.get('selected_case_id') is not None}
            actual_captures={(r['case_id'],r['method']) for r in gui['captures']}
            checks.require(actual_captures=={(case,m) for case in expected_cases for m in METHODS} and len(gui['captures'])==len(expected_cases)*len(METHODS),'Isaac representative/method screenshot coverage')
    valid=not checks.errors
    return dict(valid=valid,run=str(run),errors=checks.errors,check_count=checks.count,
        selected_cases=len(manifest.get('selected',[])),validated_method_records=len(rows),required_images_checked=image_count,
        local_index_links_checked=local_links,archived_implementation_hashes_checked=True,
        current_workspace_code_drift=code_drift,gui_validated=gui_validated,
        gui_validation=None if gui_validation is None else {'path':str(Path(gui_validation).resolve()),'sha256':digest(gui_validation) if Path(gui_validation).is_file() else None},
        operational_status=('GP_SE2_01_COMPLETED' if gui_validated else 'GP_SE2_01_COMPLETED_WITH_LIMITATIONS') if valid else 'GP_SE2_01_RUNTIME_NOT_VALIDATED',
        validation_scope='authenticated selected sources, archived implementation, independent sampled geometry/motion and exact unicycle execution, artifacts and failure-first accounting',
        limitations=['sampled GP feasibility is not continuous-time safety proof']+([] if gui_validated else ['Isaac GUI replay not validated by this invocation']),
        full_preserved_bank_rehash='separate final source-bank audit; not repeated by this validator',
        no_new_mpc_optimization_or_inference=True)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run',type=Path)
    parser.add_argument('--gui-validation',type=Path);parser.add_argument('--write',action='store_true')
    args=parser.parse_args(argv)
    result=validate_run(args.run,args.gui_validation)
    if args.write:
        try:
            with (args.run/'validation.json').open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False);stream.write('\n')
        except FileExistsError:
            result={**result,'valid':False,'errors':[*result['errors'],'refusing to overwrite existing validation.json']}
    print(json.dumps(result,indent=2,allow_nan=False))
    return 0 if result['valid'] else 1


if __name__=='__main__':raise SystemExit(main())

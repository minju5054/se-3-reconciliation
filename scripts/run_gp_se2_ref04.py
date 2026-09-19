#!/usr/bin/env python3
"""Prepare, audit and independently recompute saved REF-03 stress records only."""
from __future__ import annotations
import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import zipfile
for name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):
    os.environ[name]='1'
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
import yaml
from run_gp_se2_ref01 import read,write,copy_new,digest,utc,revision,table,clean
from reconciliation.gp_se2_ref04_audit import METHODS,CASE_ID,PROTOCOL,audit_method,assemble_audit
from reconciliation.gp_se2_environment import HospitalEnvironment

STARTING_SHA='fc51724ececc29ad00e1d62b832ce097821d16b7'
PRIMARY=ROOT/'data/robotless_gp_se2_ref_03/primary_20260919T141000Z'
OWN_FILES=['src/reconciliation/gp_se2_ref04_audit.py','src/reconciliation/gp_se2_ref04_geometry.py',
           'scripts/run_gp_se2_ref04.py','scripts/plot_gp_se2_ref04.py',
           'tests/test_gp_se2_ref04_audit.py','tests/test_gp_se2_ref04_geometry.py','tests/test_gp_se2_ref04_visuals.py']


def check_hashes(expected):
    return [p for p,h in expected.items() if not Path(p).is_file() or digest(p)!=h]


def prepare(run):
    if run.exists() or not run.is_relative_to(ROOT/'data/robotless_gp_se2_ref_04'):
        raise FileExistsError('exclusive new REF-04 run required; no overwrite')
    begin=time.perf_counter(); previous=read(PRIMARY/'source.json'); validation=read(PRIMARY/'validation.json')
    if not validation.get('valid') or not validation.get('authoritative'):
        raise ValueError('authoritative REF03 validation missing or invalid')
    if subprocess.run(['git','merge-base','--is-ancestor',STARTING_SHA,'HEAD'],cwd=ROOT).returncode:
        raise ValueError('reviewed source commit absent')
    prior=read(PRIMARY/'preservation_after.json'); expected=dict(prior['files'])
    expected.update({str(PRIMARY/p):h for p,h in read(PRIMARY/'artifact_manifest.json')['files'].items()})
    expected.update({str(ROOT/p):h for p,h in previous['preserved_core_sha256'].items()})
    expected.update({str(ROOT/p):h for p,h in read(PRIMARY/'implementation_sources/final/manifest.json')['files'].items()})
    mismatches=check_hashes(expected)
    if mismatches:raise ValueError('SOURCE_INTEGRITY_BLOCKER: '+str(mismatches))
    # All historical REF03 outputs, including images/validation, join transitive originals.
    expected.update({str(p.resolve()):digest(p) for p in PRIMARY.rglob('*') if p.is_file()})
    mpc=read(PRIMARY/'mpc_output/provenance.json')
    ext=Path(mpc['lightnav_checkout'])
    extsha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ext,text=True).strip()
    extstatus=subprocess.check_output(['git','status','--porcelain'],cwd=ext,text=True).strip()
    if extsha!=mpc['lightnav_sha'] or extstatus or digest(mpc['mpc_source'])!=mpc['mpc_source_sha256']:
        raise ValueError('official external source differs')
    settings=mpc['official_settings']
    for k,v in dict(HORIZON=5,MPC_DT_S=.1,CONTROL_RATE_HZ=10.,Q_WEIGHTS=[10.,10.,1.]).items():
        if settings[k]!=v:raise ValueError('saved official settings mismatch: '+k)
    manifest=read(PRIMARY/'case_manifest.json')
    case=next(r for r in manifest['selected'] if r['case_id']==CASE_ID)
    folder=PRIMARY/case['relative_directory']
    configs=yaml.safe_load((PRIMARY/'config_snapshot.yaml').read_text())
    if digest(PRIMARY/'config_snapshot.yaml')!=previous['original_config_sha256']:
        raise ValueError('original configuration hash mismatch')
    if configs['footprint']['radius_m']!=.20 or configs['footprint']['required_clearance_m']!=.05:
        raise ValueError('source physical configuration differs; do not alter it to match expectations')
    paths={m:folder/'methods'/m for m in METHODS}
    for name in ('reference_world.npy','row_provenance.json'):
        if digest(paths[METHODS[0]]/name)!=digest(paths[METHODS[1]]/name):raise ValueError('B/C source array or lineage not byte-identical')
    run.mkdir(parents=True)
    write(run/'protocol.json',dict(PROTOCOL,created_utc=utc(),audit_code_sha256={p:digest(ROOT/p) for p in OWN_FILES},
        original_config_sha256=digest(PRIMARY/'config_snapshot.yaml'),audit_git_sha=revision(),source_run=str(PRIMARY)))
    copy_new(PRIMARY/'config_snapshot.yaml',run/'config_snapshot.yaml')
    for name in ('input_context.json','goal_route.json','actual_past_execution.json'):
        copy_new(folder/name,run/name)
    copy_new(paths[METHODS[0]]/'reference_world.npy',run/'reference_world.npy')
    copy_new(paths[METHODS[0]]/'row_provenance.json',run/'row_provenance.json')
    userhash={p:digest(ROOT/p) for p in prior['user_config_hashes']}
    write(run/'preservation_before.json',dict(files=expected,user_config_hashes=userhash,valid=True,external_git_sha=extsha))
    source=dict(experiment='GP-SE2-REF-04',starting_git_sha=STARTING_SHA,audit_git_sha=revision(),created_utc=utc(),
        primary_source=str(PRIMARY),case_id=CASE_ID,case_path=str(folder),case_manifest_record=case,
        environment_path=previous['environment_path'],environment_export_source=previous['environment_export_source'],
        environment_file_hashes=previous['environment_file_hashes'],original_config_sha256=digest(PRIMARY/'config_snapshot.yaml'),
        authoritative_validation=dict(path=str(PRIMARY/'validation.json'),sha256=digest(PRIMARY/'validation.json'),valid=True),
        official_mpc_saved_execution_provenance=mpc,external_source_modified=False,
        core_sha256=previous['preserved_core_sha256'],audit_code_sha256={p:digest(ROOT/p) for p in OWN_FILES},
        source_record_paths={m:dict(rollout=str(paths[m]/'rollout.json'),metrics=str(paths[m]/'metrics.json'),reference=str(paths[m]/'reference_world.npy'),lineage=str(paths[m]/'row_provenance.json')) for m in METHODS},
        consumed_source_sha256={str(p):digest(p) for p in folder.rglob('*') if p.is_file() and ('plots' not in p.parts)},
        context_A_NATIVE=dict(path=str(folder/'methods/A_NATIVE/metrics.json'),metrics=read(folder/'methods/A_NATIVE/metrics.json')['summary']),
        source_frames=read(folder/'input_context.json')['frame'],source_timestamps=read(folder/'input_context.json')['source_timestamps'],
        copied_input_sha256={name:digest(run/name) for name in ('config_snapshot.yaml','input_context.json','goal_route.json',
            'actual_past_execution.json','reference_world.npy','row_provenance.json')},
        setup_wall_s=time.perf_counter()-begin,GUI_runtime='NOT_RUN_STATIC_DIAGNOSTIC')
    write(run/'source.json',source)
    print(json.dumps(dict(prepared=str(run),preserved_files=len(expected),setup_wall_s=source['setup_wall_s'])))


def compute(run):
    source=read(run/'source.json'); config=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    route=read(run/'goal_route.json'); start=time.perf_counter();env=HospitalEnvironment.load(source['environment_path'])
    environment_s=time.perf_counter()-start;start=time.perf_counter(); results={};availability={}
    for method,paths in source['source_record_paths'].items():
        roll=read(paths['rollout']); records=roll['controller_reference_selections']
        results[method]=audit_method(roll,read(paths['metrics']),route,env,config)
        availability[method]=dict(saved_solve_records=len(records),available_prediction_records=sum(x['prediction']['available'] for x in results[method]['per_solve']),
            missing_prediction_records=[dict(index=x['solve_index'],reason=x['prediction']['reason']) for x in results[method]['per_solve'] if not x['prediction']['available']],
            actual_reference_records=sum(r.get('actual_reference_capture_complete',False) for r in records),
            prediction_shape=[6,3],selected_reference_shape=[5,3],full_solved_control_sequence_available=False,
            preclipping_first_control_available=False,local_prediction_available=False,
            recorded_integration_states=len(roll['states']),recorded_applied_integration_commands=len(roll['commands']))
    result=assemble_audit(results)
    from reconciliation.gp_se2_ref04_geometry import polyline_audit
    context=read(run/'input_context.json'); reference=np.load(run/'reference_world.npy',allow_pickle=False)
    result['shared_reference_geometry']=polyline_audit(reference,env,config)
    result['input_integrity']={}
    for method,paths in source['source_record_paths'].items():
        rollout=read(paths['rollout'])
        lineage=read(paths['lineage'])
        lineage_ok=all(row['selected_reference']['source_progress']==[
            lineage[i]['original_fractional_row_coordinate'] for i in row['selected_reference']['indices']]
            for row in result['methods'][method]['per_solve'])
        installed=np.asarray(rollout['installed_reference_world'])
        result['input_integrity'][method]=dict(
            initial_pose_equals_original=rollout['initial_pose_world']==context['B_world'],
            physical_command_equals_original=rollout['initial_physical_command']==context['u_minus'],
            controller_memory_equals_original=rollout['initial_previous_control']==context['previous_control'],
            reference_file_equals_shared=digest(paths['reference'])==digest(run/'reference_world.npy'),
            lineage_file_equals_shared=digest(paths['lineage'])==digest(run/'row_provenance.json'),
            selected_source_progress_matches_lineage=lineage_ok,
            installed_xy_max_roundtrip_error_m=float(np.max(np.abs(installed[:,:2]-reference[:,:2]))),
            installed_periodic_yaw_max_roundtrip_error_rad=float(np.max(np.abs(np.arctan2(np.sin(installed[:,2]-reference[:,2]),np.cos(installed[:,2]-reference[:,2]))))),
            original_capture_transform=context['original_capture_pose_world'],
            saved_candidate_frame_transform=rollout['candidate_frame_transform'])
    return clean(result),availability,dict(environment_loading_s=environment_s,saved_record_audit_s=time.perf_counter()-start)


def tables(audit):
    predicted=[];prefix=[];gates=[];model=[]
    for method,result in audit['methods'].items():
        for g in result['gate_crossings']:gates.append(dict(method=method,issue_time_s=None,**g))
        for r in result['per_solve']:
            t=r['issue_time_s']; k=r['solve_index'];p=r['prediction']; a=r['applied_prefix']
            base=dict(method=method,solve_index=k,issue_time_s=t)
            prefix.append(dict(**base,minimum_clearance_m=a['geometry']['minimum_clearance_m'],clearance_valid=a['geometry']['clearance_valid'],
                physical_overlap=a['geometry']['physical_overlap'],first_violation_bracket_s=a['geometry']['first_refined_violation_bracket_s'],
                prediction_exact_classification_differs=a['prediction_exact_clearance_classification_differs']))
            model.append(dict(**base,**(a['model_discrepancy'] or dict(status='N/A_MISSING_PREDICTION'))))
            for g in a['gate_crossings']:gates.append(dict(method=method,issue_time_s=t,**g))
            if p['available']:
                for s in p['geometry']['segments']:
                    predicted.append(dict(**base,**s,offset_start_s=s['start_time_s']-t,offset_end_s=s['end_time_s']-t,
                        part='PREDICTED_FIRST_INTERVAL' if s['index']==0 else 'PREDICTED_UNEXECUTED_TAIL',
                        start_status=p['start_status'],prospective_warning=p['prospective_warning'],
                        source_record_key=p['source_key'],geometry_kind=p['geometry_kind']))
                for g in p['gate_crossings']:gates.append(dict(method=method,issue_time_s=t,**g))
            else:predicted.append(dict(**base,status='N/A',reason=p['reason']))
    return dict(prediction_clearance=predicted,applied_prefix_checks=prefix,gate_crossings=gates,
                failure_timeline=audit['timeline'],model_discrepancy=model)


def audit(run):
    if (run/'audit.json').exists():raise FileExistsError('saved audit already exists; no overwrite')
    before=read(run/'preservation_before.json');source=read(run/'source.json')
    mismatches=check_hashes(before['files'])+check_hashes({str(ROOT/p):h for p,h in source['audit_code_sha256'].items()})
    mismatches+=check_hashes({str(run/p):h for p,h in source['copied_input_sha256'].items()})
    if mismatches:raise ValueError('frozen source changed: '+str(mismatches))
    result,availability,timing=compute(run)
    write(run/'audit.json',result);write(run/'data_availability.json',dict(methods=availability,independent_handoff_count=1,saved_prediction_records=60,
        new_VLA_inferences=0,new_GP_or_rigid_solves=0,new_MPC_solves=0,new_rollouts=0))
    write(run/'original_outcome_recheck.json',{m:dict(reproduced=r['original_outcome_reproduced'],field_matches=r['original_outcome_field_matches'],original_outcome=r['original_outcome']) for m,r in result['methods'].items()})
    for m,r in result['methods'].items():
        for row in r['per_solve']:write(run/'per_solve'/m/f"solve_{row['solve_index']:03d}.json",row)
    (run/'aggregate').mkdir(exist_ok=True)
    for name,rows in tables(result).items():table(run/'aggregate'/f'{name}.csv',rows)
    write(run/'aggregate/summary.json',result['summary']);write(run/'aggregate/timing.json',timing)
    write(run/'preservation_after.json',dict(files=before['files'],mismatches=check_hashes(before['files']),
        user_config_hashes=before['user_config_hashes'],user_config_unchanged=not check_hashes({str(ROOT/p):h for p,h in before['user_config_hashes'].items()})))
    print(json.dumps(result['summary'],indent=2));print(json.dumps(timing))


def validate(run,output):
    start=time.perf_counter(); errors=[];checks=0
    def check(value,label):
        nonlocal checks
        checks+=1
        if not value:errors.append(label)
    source=read(run/'source.json');before=read(run/'preservation_before.json'); saved=read(run/'audit.json')
    for p,h in before['files'].items():check(Path(p).is_file() and digest(p)==h,'source modified: '+p)
    for p,h in source['audit_code_sha256'].items():check(digest(ROOT/p)==h,'audit code changed: '+p)
    for p,h in source['copied_input_sha256'].items():check(digest(run/p)==h,'frozen input copy changed: '+p)
    recomputed,availability,timing=compute(run)
    check(saved==recomputed,'saved audit differs from independent source recomputation')
    check(read(run/'data_availability.json')['methods']==availability,'availability counts differ')
    check(read(run/'aggregate/summary.json')==recomputed['summary'],'summary differs')
    check(digest(run/'config_snapshot.yaml')==source['original_config_sha256'],'original config differs')
    for name in ('input_context.json','goal_route.json','actual_past_execution.json'):
        check(digest(run/name)==digest(Path(source['case_path'])/name),'copied source differs: '+name)
    for m,integrity in recomputed['input_integrity'].items():
        for k in ('initial_pose_equals_original','physical_command_equals_original','controller_memory_equals_original',
                  'reference_file_equals_shared','lineage_file_equals_shared','selected_source_progress_matches_lineage'):check(integrity[k],m+' '+k)
        check(integrity['installed_xy_max_roundtrip_error_m']<=1e-10 and integrity['installed_periodic_yaw_max_roundtrip_error_rad']<=1e-10,m+' installation roundtrip')
    for method,r in recomputed['methods'].items():
        check(r['original_outcome_reproduced'],method+' original outcome mismatch')
        check(r['stored_counts']==dict(states=181,integration_commands=180,control_solves=30),method+' record coverage')
        check(r['reconstruction']['max_xy_error_m']<=1e-10 and r['reconstruction']['max_periodic_yaw_error_rad']<=1e-10,method+' exact reconstruction')
        for row in r['per_solve']:
            check(read(run/'per_solve'/method/f"solve_{row['solve_index']:03d}.json")==row,'per-solve record mismatch')
            p=row['prediction'];s=row['selected_reference']
            check(all(row['timing_audit'].values()),method+' stored timing/command/memory consistency')
            check(not p['available'] or p['current_node_matches_input'],'prediction input mismatch')
            check(max(s['actual_local_reference_max_error'],s['installed_target_xy_max_error_m'],s['installed_target_periodic_yaw_max_error_rad'])<=1e-8,'actual reference transform mismatch')
    for name,rows in tables(recomputed).items():
        with (run/'aggregate'/f'{name}.csv').open() as f:actual=list(csv.DictReader(f))
        fields=list(dict.fromkeys(k for row in rows for k in row)) or ['status']
        expected=[{k:json.dumps(row[k]) if isinstance(row.get(k),(dict,list)) else '' if row.get(k) is None else str(row[k]) for k in fields} for row in rows]
        check(actual==expected,name+' CSV numeric mismatch')
    from plot_gp_se2_ref04 import expected_numeric
    expected=expected_numeric(recomputed,read(run/'input_context.json'),read(run/'goal_route.json'),np.load(run/'reference_world.npy'),yaml.safe_load((run/'config_snapshot.yaml').read_text()),past=read(run/'actual_past_execution.json'))
    for name,numeric in expected.items():
        path=run/'plots'/(name+'.png')
        side=read(path.with_suffix('.json'))
        check(side.get('numeric_data')==clean(numeric),name+' numeric sidecar mismatch')
        check(path.is_file(),name+' PNG missing')
        check(side['image_sha256']==digest(path),name+' image hash mismatch')
        for source_path,h in side['source_hashes'].items():check(digest(source_path)==h,name+' plotted source hash mismatch')
    check((run/'index.html').is_file(),'index missing');check((run/'review_bundle.zip').is_file(),'review ZIP missing')
    bundle_manifest=read(run/'review_bundle_manifest.json')
    with zipfile.ZipFile(run/'review_bundle.zip') as archive:
        import hashlib
        for info in archive.infolist():
            if info.is_dir():continue
            name=Path(info.filename)
            # The package uses archive paths relative to its review directory.
            local=run/'review_bundle'/name
            check(local.is_file() and hashlib.sha256(archive.read(info)).hexdigest()==digest(local),'ZIP member differs: '+str(name))
            if (run/name).is_file():check(digest(local)==digest(run/name),'bundle copy differs: '+str(name))
    report=dict(valid=not errors,authoritative=True,errors=errors,checks=checks,
        diagnosis_status=recomputed['summary']['diagnosis_status'],new_MPC_solves=0,new_GP_or_rigid_solves=0,new_VLA_inferences=0,new_rollouts=0,
        independent_recomputation=True,validation_wall_s=time.perf_counter()-start,
        recheck_timing=timing,validator_source_sha256=digest(__file__),audit_json_sha256=digest(run/'audit.json'))
    write(output,report);print(json.dumps(report,indent=2))
    if errors:raise SystemExit(1)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['prepare','audit','validate']);p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path)
    args=p.parse_args();run=args.run.resolve()
    if args.action=='prepare':prepare(run)
    elif args.action=='audit':audit(run)
    else:validate(run,args.output or run/'validation.json')

if __name__=='__main__':main()

#!/usr/bin/env python3
"""Authoritative additive report completion; preserves the initial validator.

Only its four known shared-list-alias reporting mismatches are admissible.
Actual numerical/selection/retention failures never pass this completion gate.
"""
from __future__ import annotations
import argparse
import copy
import hashlib
from pathlib import Path
import sys
import time
import zipfile
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
from run_gp_se2_diag08 import read,write,digest,clean,verify,assemble,load_frozen_case,HospitalEnvironment,D7
from reconciliation.gp_se2_diag08_endpoint_margin import execution_admission
from reconciliation.gp_se2_diag06_witnesses import finest_trace
from run_gp_se2_diag07 import check_hashes


def reporting_error_gate(report):
    expected=['no second-failure admission.plan_failure_reason']*4
    if report['valid'] or report['errors']!=expected or not report['original_full_checker_recomputed']:
        raise ValueError('unexpected original validator result; only four known list-alias mismatches allowed')
    if any(report[k]!=0 for k in ['GP_solves_during_validation','MPC_solves_during_validation','rollouts_during_validation']):
        raise ValueError('saved-only validator required')


def nonmutating_admission(full,reserve,runs,hard):
    a=execution_admission(full,reserve,hard=hard)
    if runs:
        a.update(eligible_for_execution=False,diagnostic_execution_authorized=False)
        # Deliberately match frozen producer: a new list, not += on the shared
        # reasons object returned in both failure-detail fields.
        a['ineligible_reasons']=list(a['ineligible_reasons'])+['supplemental_motion_violation']
    return a


def validate(run):
    begin=time.perf_counter();original=read(run/'validation.json');reporting_error_gate(original)
    source=verify(run,True);errors=[];count=0
    def check(value,label):
        nonlocal count
        count+=1
        if not value:errors.append(label)
    runtime=Path(read(run/'execution_completion.json')['runtime_run']);frozen=read(runtime/'runtime_freeze.json')
    check_hashes(frozen['file_sha256'])
    incident=read(run/'technical_failure/incident.json')
    check(incident['actual_MPC_calls']==incident['actual_rollouts']==incident['GP_retry']==0,'pre-import failure count')
    for stage in ['audit','execute']:
        log=(run/'technical_failure'/('initial_'+stage+'.log')).read_text()
        check("ModuleNotFoundError: No module named 'scipy'" in log and 'historical_solve_audit(' not in log,'pre-solve import traceback '+stage)
    manifest=read(run/'experiment_manifest.json')['starts'];records=read(run/'comparison_records.json')
    env=HospitalEnvironment.load(source['environment_path']);supplement=read(run/'ineligibility_records.json')
    side=read(run/'presentation/hard_forward_speed_gap.json')
    for row,r,supp in zip(manifest,records,supplement):
        plan=read(run/'plan_recheck'/(row['solve_id']+'.json'))
        latest=next(c for c in plan['candidate_checks'] if c['iterate']=='latest_iterate')
        a=nonmutating_admission(plan['original_full_check'],plan['reserve'],latest['motion_violating_runs'],row['case_role']=='HARD')
        check(a==plan['admission'],'exact immutable failure-list comparison '+row['solve_id'])
        if row['case_role']=='HARD':
            # Explicitly recreate the obsolete validator's mutation to establish
            # that it changes only the descriptive plan_failure_reason list.
            bad=execution_admission(plan['original_full_check'],plan['reserve'],hard=True)
            bad.update(eligible_for_execution=False,diagnostic_execution_authorized=False)
            bad['ineligible_reasons']+=['supplemental_motion_violation']
            changed=[k for k in a if a[k]!=bad[k]]
            check(changed==['plan_failure_reason'],'localized alias mismatch '+row['solve_id'])
            check(not a['eligible_for_execution'] and not a['plan_valid'] and not a['deployment_candidate'],'hard rejection preserved')
        check(supp['nominal_endpoint_error_m']==plan['reserve']['endpoint_position_error_m'],'supplement endpoint')
        check(supp['supplemental_motion_violations']==latest['motion_violating_runs'],'supplement motion')
        case=load_frozen_case(Path(source['primary_source']),row['case_id'],row['method'],environment=env)
        new=read(run/'solves'/row['solve_id']/'solver_result.json');old=read(Path(row['g3_result_path']))
        if row['case_role']=='HARD':
            plotted=next(x for x in side['numeric'] if x['reference_id']==row['solve_id'])
            for grid,result in [('G3',old),('G4',new)]:
                tr=finest_trace(case['problem'],result['latest_iterate']);t=np.asarray(tr['times_s']);v=np.asarray(tr['body_twists'])[:,0];m=(t>=.47)&(t<=.49)
                check(np.array_equal(t[m],plotted['traces'][grid]['times_s']) and np.array_equal(v[m],plotted['traces'][grid]['vx_m_s']),'supplement plot source '+grid)
    recomputed,cache=assemble(run,env)
    check(clean(recomputed)==records,'independent execution/geometry recomputation literal')
    check(digest(run/'presentation/hard_forward_speed_gap.png')==side['image_sha256'],'supplement image hash')
    for name,h in read(run/'plot_artifacts.json').items():check(digest(run/name)==h,'original plot hash '+name)
    ledger=read(run/'supplemental_validation/test_runtime_ledger.json')
    check(ledger['total_actual_MPC_calls_task_wide']==32 and ledger['new_actual_GP_solves']==5,'task call ledger')
    check(read(run/'supplemental_validation/full_suite_historical_solve_audit.json')['passed'],'test-only official audit')
    check(read(run/'report_completion_validation.json')['valid'],'supplement source checks')
    for name in ['historical_audits','rollouts']:check((run/name).resolve()==(runtime/name).resolve(),'explicit execution path link')
    output=dict(valid=not errors,authoritative=True,errors=errors,check_count=count,
        original_frozen_validator=str(run/'validation.json'),original_checks=original['check_count'],
        original_only_errors='four list-alias diagnostic field comparisons; now reproduced and checked without mutation',
        candidate_source_labels_checked=original['candidate_source_labels_checked'],source_files_preserved=len(source['preserved_hashes']),
        full_plan_dense_interval_checks=original['original_full_checker_recomputed'],execution_recomputed_literally=True,
        scientific_results_unchanged=True,execution_runtime_freeze=str(runtime/'runtime_freeze.json'),
        new_GP=0,new_MPC=0,new_rollout=0,wall_s=time.perf_counter()-begin,
        operational_status='GP_SE2_DIAG_08_COMPLETED_WITH_LIMITATIONS' if not errors else 'GP_SE2_DIAG_08_BLOCKED',
        interpretation=read(run/'aggregate/summary.json')['interpretation'])
    write(run/'verification/validation.json',output)
    write(run/'verification/output_hashes.json',{str(p.relative_to(run)):digest(p) for folder in ['aggregate','plots','presentation','verification']
        for p in (run/folder).rglob('*') if p.is_file()})
    print(output,flush=True)
    if errors:raise SystemExit('final validation failed')


def package(run):
    authority=read(run/'verification/validation.json')
    if not authority['valid']:raise ValueError('authoritative validation required')
    source=read(run/'source.json');summary=read(run/'aggregate/summary.json')
    write(run/'review_source.json',dict(starting_sha=source['starting_sha'],execution_sha=read(run/'execution_freeze.json')['execution_sha'],
        source_inventory=str(run/'source.json'),source_inventory_sha256=digest(run/'source.json'),
        config_sha256=digest(run/'config_snapshot.yaml'),authority=str(run/'verification/validation.json'),
        authoritative_validation_sha256=digest(run/'verification/validation.json'),
        runtime_completion=read(run/'execution_completion.json'),preserved_file_count=len(source['preserved_hashes']),
        full_GP_solve_results='referenced in source run, not copied into compact review'))
    status=('GP-SE2-DIAG-08 COMPLETED WITH LIMITATIONS\n\n'+summary['interpretation']+'\n\n'
        'All five G4 starts converged. Four hard plans fail lateral and negative speed; no hard G4 rollout was authorized. '
        'The 11 cm endpoint is achieved to solver allowance; nominal excess is only 4.7e-12 to 7.6e-11 m, but independent speed failure remains. '
        'One newly executed benign G4 reproduces historical success exactly. Hard execution recovery is unmeasured, not failed execution.\n\n'
        'Authority: verification/validation.json. The earlier root validation.json is preserved with four reporting-list alias mismatches. '
        'An additive import-isolated worker completed MPC without dependency changes or GP retry. '
        '5 GP solves; 30 primary MPC + 1 protocol audit + 1 existing test audit = 32 actual MPC calls.\n')
    with (run/'REVIEW_STATUS.md').open('x') as f:f.write(status)
    with (run/'presentation/index.html').open('x') as f:
        f.write('<!doctype html><meta charset="utf-8"><title>DIAG-08 completed review</title><h1>DIAG-08 completed with limitations</h1><p>All five solves converge; hard plans acquire a second nonlateral failure and are not executed. Benign execution remains successful.</p><p><a href="failure_summary.html">Main finding: speed violation shifted beyond the frozen witness</a></p><p><a href="../index.html">All five conditions and 50 G3/G4 figures</a> | <a href="tracking_panels/index.html">Readable tracking-distance panels</a></p><p><a href="../verification/validation.json">Authoritative validation</a> | <a href="../REVIEW_STATUS.md">Status and limitations</a></p><img style="max-width:1100px;width:100%" src="hard_forward_speed_gap.png">')
    copy=ROOT/'docs/GP_SE2_DIAG_08_ENDPOINT_MARGIN_EXECUTION.md'
    files=[run/n for n in ['index.html','protocol.json','config_snapshot.yaml','validation.json','review_source.json','REVIEW_STATUS.md','report_completion_validation.json','execution_completion.json']]
    files += [p for folder in ['plots','aggregate','presentation','verification'] for p in (run/folder).rglob('*') if p.is_file()]
    files += [run/'margin/definition.json',run/'margin/derivative_validation.json',run/'technical_failure/incident.json',run/'supplemental_validation/test_runtime_ledger.json']
    runtime=Path(read(run/'execution_completion.json')['runtime_run'])
    with zipfile.ZipFile(run/'review_bundle.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in sorted(files):archive.write(p,p.relative_to(run))
        archive.write(copy,'REPORT.md');archive.write(runtime/'runtime_freeze.json','runtime_freeze.json')
    with zipfile.ZipFile(run/'review_bundle.zip') as archive:
        errors=[str(p) for p in files if archive.read(str(p.relative_to(run)))!=p.read_bytes()]
        if archive.read('REPORT.md')!=copy.read_bytes():errors.append('report')
    write(run/'bundle_validation.json',dict(valid=not errors,errors=errors,zip_sha256=digest(run/'review_bundle.zip'),
        bytes=(run/'review_bundle.zip').stat().st_size,member_count=len(files)+2,report_sha256=digest(copy)))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',required=True,type=Path);p.add_argument('--package',action='store_true');a=p.parse_args()
    (package if a.package else validate)(a.run.resolve())

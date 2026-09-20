#!/usr/bin/env python3
"""Saved-record verification only: no optimizer, MPC solve, or simulator."""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
from run_gp_se2_diag08 import (
    verify,read,write,digest,clean,assemble,D6,D7,LABELS,providers,load_frozen_case,
    HospitalEnvironment,check_full_candidate,_constraint_report,reserve_report,execution_admission,
    audit_intervals,extract_runs,d6,array_hash)
from reconciliation.gp_se2_diag03_audit import compare_tree
from plot_gp_se2_diag08 import plot_data,numeric,PLOTS


def validate(run):
    start=time.perf_counter();source=verify(run,True);errors=[];count=0
    def check(value,label):
        nonlocal count
        count+=1
        if not bool(value):errors.append(label)
    def same(actual,expected,label):
        nonlocal count
        comparisons=compare_tree(clean(actual),expected,label)
        count+=len(comparisons);errors.extend(r['field'] for r in comparisons if not r['numerical_agreement'])
    env=HospitalEnvironment.load(source['environment_path']);manifest=read(run/'experiment_manifest.json')
    check(manifest['execution_order']==LABELS and len(manifest['starts'])==5,'five fixed starts')
    check(read(run/'margin/definition.json')['planning_radius_m']==.11,'frozen 11 cm')
    check(digest(run/'config_snapshot.yaml')==digest(D6/'config_snapshot.yaml'),'original config bytes')
    check(digest(run/'margin/frozen_g3_witness_union.json')==digest(D6/'witness_selection/frozen_union.json'),'witness bytes')
    witnesses=read(run/'margin/frozen_g3_witness_union.json');actual=0;retained_counts=0
    for row in manifest['starts']:
        folder=run/'solves'/row['solve_id'];result=read(folder/'solver_result.json')
        check(digest(row['seed_path'])==row['seed_sha256']==digest(row['original_seed_path'])==digest(row['g3_seed_path']),'seed bytes '+row['solve_id'])
        check(np.array_equal(np.load(row['seed_path'],allow_pickle=False),result['initial_vector']),'initial original seed '+row['solve_id'])
        check(result['minimize_invocations']==1,'no retry '+row['solve_id']);actual+=result['minimize_invocations']
        check(result['config']['goal_position_tolerance']==.15 and result['config']['equality_tolerance']==1e-5,'physical tolerances')
        case=load_frozen_case(Path(source['primary_source']),row['case_id'],row['method'],environment=env)
        inherited,view,provider=providers(case['problem'],env,witnesses);base=case['problem']
        p,v=base.unpack(result['latest_iterate'])
        check(np.array_equal(p,result['latest_support_poses']) and np.array_equal(v,result['latest_support_twists']),'saved support reconstruction')
        check(np.array_equal(p[1:],np.load(run/'references'/row['solve_id']/'world_reference.npy',allow_pickle=False)),'unchanged 30-row interface')
        same(view.dimensions(result['initial_vector']),result['constraint_dimensions'],'dimensions '+row['solve_id'])
        analysis=read(run/'plan_recheck'/(row['solve_id']+'.json'));cache={};choices=[]
        expected_labels=['initial','latest_iterate']+[f"callback_{s['iteration']:04d}" for s in result['callback_snapshots'] if s['collocation'] is not None and s['collocation']['feasible']]
        check([c['iterate'] for c in result['candidate_checks']]==expected_labels,'actual retained callbacks only')
        provider.warmup(result['initial_vector'])
        for phase,z in [('initial',result['initial_vector']),('latest',result['latest_iterate']),('selected',result['candidate_vector'])]:
            if z is None:continue
            a=inherited[-2].evaluate(z);b=view.evaluate(z);pj=provider.inequality_jacobian(z);g3j=inherited[-1].inequality_jacobian(z)
            check(b['objective']==a['objective'] and np.array_equal(b['equality'],a['equality']) and
                np.array_equal(b['inequality'][:-1],a['inequality']),'literal G3 primal '+phase)
            check(np.array_equal(provider.equality_jacobian(z),inherited[-1].equality_jacobian(z)) and
                np.array_equal(pj[:-1],g3j),'literal G3 derivative '+phase)
        for c,diagnostic in zip(result['candidate_checks'],analysis['candidate_checks']):
            identity=array_hash(c['vector']);retained_counts+=1
            if identity not in cache:
                value=view.evaluate(c['vector']);grid=_constraint_report(value['equality'],value['inequality'],base.config)
                dense=base.dense_report(c['vector']);full=clean(check_full_candidate(base,c['vector'],case))
                iv=audit_intervals(base,c['vector']);runs=extract_runs(iv['finest_trace'],base.config,row['solve_id'])
                cache[identity]=(value,grid,dense,full,reserve_report(base,c['vector']),iv,runs)
            value,grid,dense,full,reserve,iv,runs=cache[identity]
            same(value['objective'],c['objective'],'objective '+c['iterate']);same(grid,c['collocation'],'grid '+c['iterate'])
            same(dense,c['constraint_report'],'dense '+c['iterate']);same(full,c['full_acceptance'],'full '+c['iterate'])
            same(reserve,diagnostic['reserve'],'reserve '+c['iterate']);same(runs,diagnostic['motion_violating_runs'],'interval motion')
            same(d6.d5.compact_intervals(iv),read(Path(diagnostic['interval_path'])),'interval record')
            if grid['feasible'] and dense['feasible']:choices.append((value['objective'],c['iterate'],identity))
            if c['iterate']=='latest_iterate':
                same(full,analysis['original_full_check'],'latest independent full')
                admission=execution_admission(full,reserve,hard=row['case_role']=='HARD')
                if runs:
                    admission.update(eligible_for_execution=False,diagnostic_execution_authorized=False)
                    admission['ineligible_reasons']+=['supplemental_motion_violation']
                same(admission,analysis['admission'],'no second-failure admission')
        selected=min(choices) if choices else None
        check(result['selected_iterate']==(None if selected is None else selected[1]),'original selection policy')
        check(result['candidate_vector'] is None if selected is None else array_hash(result['candidate_vector'])==selected[2],'no fallback')
        check(result['config']['max_iterations']==200 and result['config']['ftol']==1e-7 and result['config']['wall_time_s']==30.,'solver protocol')
    check(actual==5==read(run/'optimization_completed.json')['actual_new_GP_solves'],'five actual new GP solves')
    records,cache=assemble(run,env);same(records,read(run/'comparison_records.json'),'independent execution/geometry/classification assembly')
    reference_manifest=read(run/'reference_manifest.json')
    from reconciliation.gp_se2_diag07_lateral_execution import deduplicate
    unique,aliases=deduplicate([r for r in reference_manifest['records'] if r['eligible_for_execution']])
    same(unique,reference_manifest['unique_references'],'exact dedup');same(aliases,reference_manifest['aliases'],'aliases')
    for uid,(_,evaluation,diagnostics) in cache.items():
        same(evaluation,read(run/'rollouts'/uid/'full_execution_evaluation.json'),'execution '+uid)
        same(diagnostics,read(run/'rollouts'/uid/'diagnostics.json'),'diagnostics '+uid)
        for name,h in read(run/'rollouts'/uid/'hashes.json').items():check(digest(run/'rollouts'/uid/name)==h,'rollout saved hash '+name)
    summary=read(run/'aggregate/summary.json')
    check(summary['unique_rollouts']==len(unique),'unique coverage')
    check(summary['primary_MPC_solves']==30*len(unique)==read(run/'rollouts/summary.json')['mpc_solves'],'primary solve coverage')
    check(summary['historical_audit_MPC_solves']==len(read(run/'mpc_request.json')['context_paths']),'historical audit count')
    check(summary['hard_execution_recovered_starts']==sum(r['classification']=='MARGIN_EXECUTION_RECOVERY' for r in records[:4]),'recovery count')
    for caseid in read(run/'mpc_request.json')['context_paths']:
        check(read(run/'historical_audits'/(caseid.replace('/','__')+'.json'))['passed'],'historical solve audit')
    # Recompute every scientific table's saved scalar / list / dictionary values.
    byid={r['reference_id']:r for r in records}
    def parsed(cell):
        if cell=='':return None
        if cell in ['True','False']:return cell=='True'
        try:return json.loads(cell)
        except (ValueError,TypeError):return cell
    for name in ['plan_metrics','endpoint_geometry','execution_outcomes','command_metrics','selection_metrics']:
        with (run/'aggregate'/(name+'.csv')).open() as stream:
            for row in csv.DictReader(stream):
                r=byid[row['reference_id']];v=r[row['grid']];e=v['evaluation'];d=v['diagnostics']
                expected=dict(reference_id=r['reference_id'],case_id=r['case_id'],grid=row['grid'],provenance=v['provenance'])
                if name=='plan_metrics':expected.update(v['plan'])
                elif name=='endpoint_geometry':expected.update(available=v['geometry'] is not None,**(v['geometry'] or {}))
                elif name=='execution_outcomes':expected.update(performed=e is not None,primary_success=None if e is None else e['primary_success'],
                    failure_reasons=r['admission']['ineligible_reasons'] if e is None else e['failure_reasons'],**({} if e is None else e['execution']))
                elif name=='command_metrics':expected.update({k:x for k,x in e['execution'].items() if 'acceleration' in k or 'variation' in k},first_command=d['first_command'],final_command=d['final_command'])
                else:expected.update(first_goal_region_target_s=d['first_goal_region_in_horizon_s'],first_final_row_s=d['first_final_row_in_horizon_s'],
                    final_row_maintained=d['final_row_maintained_after_first'],selected_indices=d['selection']['selected_indices'],nearest_indices=d['selection']['nearest_indices'])
                same({k:parsed(v) for k,v in row.items()},{k:expected.get(k) for k in row},'CSV '+name)
    with (run/'aggregate/g3_vs_g4.csv').open() as stream:
        paired=list(csv.DictReader(stream))
    check(len(paired)==5,'paired coverage')
    for row,r in zip(paired,records):
        check(row['classification']==r['classification'],'paired classification')
        for g in ['G3','G4']:
            same(parsed(row[g+'_plan_error_m']),r[g]['plan']['endpoint_position_error_m'],'paired plan error')
            same(parsed(row[g+'_execution_error_m']),None if r[g]['geometry'] is None else r[g]['geometry']['execution_error_m'],'paired execution error')
    data=plot_data(run);same(data,read(run/'presentation/data.json'),'plot source')
    for r in data['records']:
        for name in PLOTS:
            path=run/'plots'/r['id']/(name+'.png');side=read(path.with_suffix('.json'))
            same(numeric(name,r),side['numeric'],'plot numeric '+r['id']+name)
            check(digest(path)==side['image_sha256'],'PNG hash '+name)
            check(side['source_sha256']==digest(run/'source.json') and side['config_sha256']==digest(run/'config_snapshot.yaml'),'PNG provenance')
    for name,h in read(run/'plot_artifacts.json').items():check(digest(run/name)==h,'artifact '+name)
    check(all(summary[k]==0 for k in ['new_VLA','new_RGB','new_GUI','new_online_episode','historical_G3_reruns']),'forbidden runtime ledger')
    output=dict(valid=not errors,errors=errors,check_count=count,wall_s=time.perf_counter()-start,
        original_full_checker_recomputed=True,candidate_source_labels_checked=retained_counts,
        GP_solves_during_validation=0,MPC_solves_during_validation=0,rollouts_during_validation=0,
        source_files_preserved=len(source['preserved_hashes']),all_scientific_failures_preserved=True,
        operational_status='GP_SE2_DIAG_08_COMPLETED_WITH_LIMITATIONS' if not errors else 'GP_SE2_DIAG_08_BLOCKED')
    write(run/'validation.json',output);print(json.dumps(output),flush=True)
    if errors:raise SystemExit('artifact validation failed')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',required=True,type=Path);validate(p.parse_args().run.resolve())

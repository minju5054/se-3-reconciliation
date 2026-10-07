#!/usr/bin/env python3
"""Post-collection accounting correction only; keep all frozen code/results intact.

The frozen episode audit counts submitted replies inside the loop, whereas its
outer call auditor correctly includes shutdown-drained replies. This addendum
reconciles the same already-saved messages, never applies a late result, and
retains all frozen source, timing, history, controller and safety checks.
"""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.successive_native_source_acquisition02 import QUALIFIED
from run_successive_native_source_acquisition02 import OUT,namespace,verify
from validate_successive_native_source_acquisition02 import validate,validate_episode_data,bundle,validate_bundle
from validate_robotless_online_handoffs import equal_record
from analyze_join_online02 import jsonlines,csvread


def complete_accounting(events,tail,commands):
    assert tail['physically_applied'] is False and tail['simulation_advanced'] is False
    accepted=[e for e in events+tail['messages'] if e.get('status')=='submitted']
    results=[e for e in events+tail['messages'] if e.get('type')=='solve_result']
    ids=[e['solve_id'] for e in accepted];result_ids=[e['solve_id'] for e in results]
    assert len(set(ids))==len(ids) and len(set(result_ids))==len(result_ids)
    assert set(ids)==set(result_ids),'every accepted solve must have exactly one saved result'
    late=[e['solve_id'] for e in tail['messages'] if e.get('status')=='submitted']
    assert not set(late)&{c.get('solve_id') for c in commands},'late commands must remain unapplied'
    return dict(accepted_in_loop=sum(e.get('status')=='submitted' for e in events),
        accepted_in_shutdown_drain=len(late),accepted_total=len(accepted),solved_total=len(results),
        successful_total=sum(e.get('status')=='command' for e in results),
        stale_rejected_total=sum(e.get('status')=='stale_rejected' for e in results),
        physical_new_applications=sum(c['reason']=='new_solve' for c in commands),
        late_submission_ids=late,late_results_unapplied=True,no_simulation_advance=True,
        unobserved_result_ids=sorted(set(ids)-set(result_ids)))


def revalidate(run):
    verify(run)
    for p,h in read(run/'candidate_seal.json')['files'].items():assert sha(p)==h,p
    original=validate(run);equal_record(original,read(run/'attempt_validation.json'))
    v=validate_episode_data(run);ep=Path(v['episode'])
    counts=complete_accounting(jsonlines(ep/'controller/events.jsonl'),
        read(ep/'controller/post_episode_messages.json'),csvread(ep/'commands.csv'))
    expected=f"AssertionError: ('MPC_submissions', {counts['accepted_total']}, {counts['accepted_in_loop']})"
    assert original['scientific_validator_error']==expected and counts['accepted_in_shutdown_drain']>0
    assert original['classification']=='TECHNICAL_EXECUTION_BLOCKED'
    assert original['calls']['MPC_submissions']==counts['accepted_total']
    assert v['calls']['MPC_submissions']==counts['accepted_in_loop']
    v['calls']['MPC_submissions']=counts['accepted_total']
    v['calls']['MPC_unobserved_result_ids']=counts['unobserved_result_ids']
    for a,b in [('terminal_LightNav_predictions','terminal_predictions'),('buffer_only_LightNav_requests','buffer_only'),
                ('MPC_submissions','MPC_submissions'),('MPC_solved_results','MPC_saved_results'),
                ('MPC_physical_applications','MPC_physical_new_applications'),('integration_intervals','integration_intervals')]:
        assert original['calls'][a]==v['calls'][b],(a,original['calls'][a],v['calls'][b])
    return dict(valid=True,classification=v['classification'],scientific=v,complete_controller_accounting=counts,
        frozen_outer_classification_unchanged=original['classification'],frozen_error=expected,
        original_attempt_sha256=sha(run/'attempt_validation.json'),original_candidate_seal_sha256=sha(run/'candidate_seal.json'),
        new_LightNav_MPC_Isaac_optimizer_calls=0,scientific_qualification_unchanged=True,
        protocol_deviation='Post-freeze saved-only accounting addendum: include accepted submission replies drained at shutdown, as the existing outer call auditor already does. Frozen code, frozen attempt result, scientific collection and selection thresholds are unchanged.',
        addendum_script_sha256=sha(__file__))


def search_summary(run,a):
    manifest=read(OUT/'candidate_manifest.json')
    assert run.name==manifest[0]['candidate_id'] and a['classification'] in QUALIFIED
    assert not any((run.parent/c['candidate_id']/'launch_attempt.json').exists() for c in manifest[1:])
    return dict(experiment='SUCCESSIVE_NATIVE_SOURCE_ACQUISITION_02',source_type='DEVELOPMENT SOURCE',
        classification=a['classification'],selected_candidate=run.name,primary_figure_candidate=run.name,
        candidates=[dict(candidate_id=c['candidate_id'],pose=c['pose_world'],geometry_valid=c['geometry_valid'],
            executed=c['candidate_id']==run.name,reason='12 consecutive applied; all frozen gates pass under complete accounting' if c['candidate_id']==run.name else 'not run: first candidate qualifies') for c in manifest],
        aggregate_call_accounting=read(run/'attempt_validation.json')['calls'],
        source_search_candidates_executed=1,first_qualified_stop=True,retries=0,
        addendum_sha256=sha(run/'accounting_addendum.json'),new_validation_model_MPC_Isaac_optimizer_calls=0)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--seal',action='store_true');a=p.parse_args()
    run=namespace(a.run);result=revalidate(run);v=result['scientific']
    if a.seal:
        # These did not exist: original outer auditor failed before bundle creation.
        assert not (run/'validation.json').exists() and not (run/'source_bundle').exists()
        save(run/'accounting_addendum.json',result);save(run/'validation.json',v);bundle(run,v)
        save(run/'source_bundle/qualification.json',dict(valid_successive_source=v['classification'] in QUALIFIED,
            classification=v['classification'],validation_sha256=sha(run/'validation.json'),
            accounting_addendum_sha256=sha(run/'accounting_addendum.json')))
        save(run/'source_bundle_validation.json',validate_bundle(run,v))
        save(OUT/'accounting_addendum.json',{k:value for k,value in result.items() if k!='scientific'})
        save(OUT/'search_summary.json',search_summary(run,result))
    else:
        equal_record(result,read(run/'accounting_addendum.json'));equal_record(v,read(run/'validation.json'))
        equal_record(validate_bundle(run,v),read(run/'source_bundle_validation.json'))
        equal_record(search_summary(run,result),read(OUT/'search_summary.json'))
    print(result['classification'],result['complete_controller_accounting'],'new scientific calls 0')

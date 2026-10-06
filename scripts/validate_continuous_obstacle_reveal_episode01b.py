#!/usr/bin/env python3
"""Saved-only 01B provenance audit; delegate scientific checks to frozen 01."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.continuous_obstacle_reveal_episode01 import CHUNKS
from run_continuous_obstacle_reveal_episode01b import verify, declaration, namespace, OUT
from analyze_join_online02 import csvread,jsonlines
from validate_robotless_online_handoffs import equal_record
from validate_continuous_obstacle_reveal_episode01 import validate as validate01, bundle


def lines(path):
    return jsonlines(path) if path.exists() else []


def accounting(run):
    ep=run/'episodes/EPISODE_00'
    req=[read(p) for p in sorted((ep/'requests').glob('seq_*_metadata.json'))]
    commands=csvread(ep/'commands.csv') if (ep/'commands.csv').exists() else []
    states=csvread(ep/'execution.csv') if (ep/'execution.csv').exists() else []
    events=lines(ep/'controller/events.jsonl')
    tail=ep/'controller/post_episode_messages.json'
    if tail.exists(): events+=read(tail)['messages']
    solves=[e for e in events if e.get('type')=='solve_result']
    log=(run/'logs/server.log').read_text() if (run/'logs/server.log').exists() else ''
    return dict(full_SimulationApp_launch_attempts=int((run/'launch_attempt.json').exists()),
        scientific_episodes_initialized=int(bool(states)),
        scientific_episodes_completed=int((ep/'completion.json').exists()),
        terminal_LightNav_predictions=sum(r['kind']=='prediction' for r in req),
        buffer_only_LightNav_requests=sum(r['kind']=='buffer_only' for r in req),
        warmups=log.count('[lightnav-ws] warmup done'),
        RGB_captures=len(lines(ep/'capture.jsonl')),
        MPC_submissions=sum(e.get('status')=='submitted' for e in events),
        MPC_solved_results=len(solves), MPC_successful_results=sum(e.get('status')=='command' for e in solves),
        MPC_physical_applications=sum(c['reason']=='new_solve' for c in commands),
        integration_intervals=len(commands), cart_reveals=int((ep/'obstacle_reveal.json').exists()),
        episode_initialization_resets=int(bool(states)), episode_resets_after_initialization=0,
        graph_optimizer=0,canonical_optimizer=0,B_ENTRY=0,Hermite=0,V2=0,GP=0,
        rigid_transport=0,correspondence_optimization=0,retries=0,new_source_searches=0,
        saved_validation_model_MPC_optimizer_Isaac=0)


def validate(run):
    verify(run)
    cfg=declaration(); launch=read(run/'launch_result.json')
    attempt=read(run/'launch_attempt.json'); plan=read(run/'launch_environment.json')
    assert attempt['argv']==plan['argv'] and attempt['full_startup_budget']==1
    assert launch['full_launch_attempts']==1 and launch['no_retry']
    for p,h in launch['actual_library_sha256'].items(): assert sha(p)==h,p
    samples=[read(p) for p in sorted((run/'process_observations').glob('*.json'))]
    expected=read(run/'loader_verification.json')['cases'][1]
    observed=sorted({p for s in samples for p in s['libraries'] if 'libnvJitLink.so' in p})
    for p in observed: assert str(Path(p).resolve())==expected['resolved_path'],p
    if launch['scene_initialized']:
        assert samples and observed, 'actual launch environment/library provenance missing'
    for s in samples:
        # Isaac's own python.sh sets LD_LIBRARY_PATH/PYTHONPATH after sanitation.
        for k in cfg['unset_environment'][2:]: assert s['relevant_environment'][k] is None,k
        for k,v in cfg['set_environment'].items(): assert s['relevant_environment'][k]==v,k
    calls=accounting(run)
    assert calls['full_SimulationApp_launch_attempts']==1
    assert calls['terminal_LightNav_predictions']<=4 and calls['cart_reveals']<=1
    ep=run/'episodes/EPISODE_00'; scientific=None; error=None
    if (ep/'metadata.json').exists():
        try:
            scientific=validate01(run)
            old=scientific['calls']
            for a,b in [('terminal_LightNav_predictions','terminal_predictions'),
                        ('buffer_only_LightNav_requests','buffer_only'),
                        ('MPC_submissions','MPC_submissions'),('MPC_solved_results','MPC_saved_results'),
                        ('MPC_physical_applications','MPC_physical_new_applications'),
                        ('integration_intervals','integration_intervals')]:
                assert calls[a]==old[b],(a,calls[a],old[b])
        except Exception as exc:
            error=f'{type(exc).__name__}: {exc}'
            scientific=None
    else:
        error='No completed episode metadata; scientific validator has no complete record to validate'
    qualified=bool(scientific and scientific['classification']=='CONTINUOUS_OBSTACLE_REVEAL_EPISODE_QUALIFIED')
    return dict(experiment=cfg['experiment'],starting_sha=read(run/'source.json')['starting_sha'],
        scientific_freeze_sha=attempt['scientific_freeze_sha'],run=str(run),
        historical01_preserved=True,historical01_calls=read(ROOT/cfg['historical_results']/'result_summary.json')['calls'],
        protocol_equal=True,intentional_difference='process_launch_environment_only',
        loader_verification=read(run/'loader_verification.json'),actual_nvJitLink_paths=observed,
        actual_library_mapping_observed=bool(observed),launch=launch,
        scientific_episode_validator_passed=scientific is not None,scientific_validator_error=error,
        source_bundle_valid=qualified,partial_data_valid=scientific is not None and not qualified,
        source_bundle=str(run/'source_bundle') if scientific else None,
        classification=scientific['classification'] if scientific else 'TECHNICAL_EXECUTION_BLOCKED',
        calls=calls,scientific=scientific,
        technical_evidence_sha256={str(p.relative_to(run)):sha(p) for p in run.rglob('*')
            if p.is_file() and (p.parent.name in ('logs','process_observations') or
                p.name in ('launch_attempt.json','launch_result.json','execution_start.json','collection_failure.json',
                           'generation_actual.json','server_shutdown.json','schedule_completion.json'))})


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--seal',action='store_true')
    a=p.parse_args();run=namespace(a.run);v=validate(run)
    if a.seal:
        if v['scientific']:
            save(run/'validation.json',v['scientific']);bundle(run,v['scientific'])
            save(run/'source_bundle/qualification.json',dict(valid_continuous_C0_C3_source=v['source_bundle_valid'],
                validated_partial_data=v['partial_data_valid'],classification=v['classification'],
                validation_sha256=sha(run/'validation.json')))
        save(run/'attempt_validation.json',v)
        save(OUT/'execution_audit.json',{k:value for k,value in v.items() if k!='scientific'})
    else:
        equal_record(v,read(run/'attempt_validation.json'))
    print(v['classification'],v['calls'],v['scientific_validator_error'])


if __name__=='__main__':main()

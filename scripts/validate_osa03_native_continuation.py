#!/usr/bin/env python3
"""Authoritative saved-only validation; original execution code stays frozen.

The guard runs before command journal fields are attached. Validate the exact
physical command and all guard quantities, while allowing only those journal
fields to be absent from the pre-application guard snapshot.
"""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
import yaml
from reconciliation.join_source03 import read,save,sha
from reconciliation.osa03_native import replay_prefix,command,evaluate
from reconciliation.robotless_online import integrate_unicycle
from reconciliation.join_online02 import guard_check,preview_activation
from reconciliation.online_mpc_adapter import selection_audit
from run_osa03_native_continuation import source_phase,plain
from run_join_online02 import environments
from validate_robotless_online_handoffs import equal_record


def guard_parity(recomputed,saved):
    got=dict(recomputed);expected=saved['command'];actual=got['command']
    assert set(actual)-set(expected)<= {'command_id','relative_time_s','origin'}
    assert not set(expected)-set(actual)
    got['command']={k:actual[k] for k in expected}
    equal_record(got,saved)


def verify(run):
    revisions=read(run/'reporting_revisions.json')
    # Only the presentation script changed; runner, worker, metrics, protocol stay exact.
    assert set(revisions['files'])=={str(ROOT/'scripts/report_osa03_native_continuation.py')}
    for group in ['files','inputs']:
        for p,h in read(run/'freeze.json')[group].items():
            if p in revisions['files']:
                revision=revisions['files'][p];assert revision['frozen_sha256']==h and revision['current_sha256']==sha(p)
            else:assert sha(p)==h,p
    for p,h in read(run/'source_manifest.json')['hashes'].items():assert sha(p)==h,p
    assert sha(__file__)==revisions['validator_sha256']

def validate(run):
    verify(run);m=read(run/'source_manifest.json');source=Path(m['source']);phase,fresh,raw,provenance=source_phase(source)
    equal_record(phase,read(run/'controller_phase.json'));_,env,_,_=environments(source)
    r=read(run/'rollout.json');cfg=yaml.safe_load((run/'protocol.yaml').read_text())
    assert r['phase']==phase and r['raw_FRESH_sha256']==m['raw_FRESH_sha256']
    replay=replay_prefix(phase,env);activation=replay.pop('activation');equal_record(plain(replay),read(run/'prefix_parity.json'))
    for i,s in enumerate(r['states']):
        assert s['absolute_tick']==phase['B_tick']+i and s['time_s']==i*phase['integration_dt_s']
        if i:
            pred=integrate_unicycle(r['states'][i-1]['pose_world'],command(r['commands'][i-1]),phase['integration_dt_s'])
            np.testing.assert_array_equal(pred,s['pose_world'])
    np.testing.assert_array_equal([s['pose_world'] for s in r['states'][:phase['prefix_steps']+1]],replay['states'])
    assert len(r['commands'])==len(r['states'])-1
    for i,c in enumerate(r['commands']):
        g=guard_check(env,r['states'][int(c['application_tick'])-phase['B_tick']]['pose_world'],c,phase['integration_dt_s']);assert g['safe'];guard_parity(g,r['guards'][i])
    previous=phase['applications'][-1]['result']['command']
    for e in r['events']:
        if e.get('status')=='submitted':np.testing.assert_array_equal(e['previous_command'],previous)
        if e.get('type')=='solve_result' and e.get('status')=='command':
            np.testing.assert_array_equal(e['previous_command'],previous);previous=e['command']
    seen={e['solve_id']:e for e in r['events'] if e.get('type')=='solve_result'}
    for e in seen.values():
        assert e['input_state_id']>=phase['first_new_submit_tick'] and e['input_state_id']%6==0
        if e.get('status')=='command':
            np.testing.assert_array_equal(e['input_pose'],r['states'][e['input_state_id']-phase['B_tick']]['pose_world'])
            assert selection_audit(fresh,e['input_pose'],e['selection']['reference_world'],horizon=provenance['official_settings']['HORIZON'],weights=provenance['official_settings']['Q_WEIGHTS'])==e['selection']
    for c in r['commands'][phase['prefix_steps']:]:
        tick=c['application_tick']
        for e in r['events']:
            if e.get('type')=='solve_result' and e.get('continuation_seen',{}).get('tick')==tick:
                assert activation.accept(e,c['sim_time_s'])
        pp=r['states'][tick-phase['B_tick']]['pose_world']
        st=dict(c,state_id=tick,tick=tick,x=pp[0],y=pp[1],yaw=pp[2])
        activation,expected,_=preview_activation(activation,st,None,None)
        for key in expected:assert expected[key]==c[key],(tick,key)
        if c['reason']=='new_solve':
            e=seen[c['solve_id']];np.testing.assert_array_equal(command(c),e['command']);assert c['application_tick']==e['continuation_seen']['tick']
    if r['safety_abort']:
        a=r['safety_abort'];assert not a['command_applied'];assert not guard_check(env,a['guard']['start_pose'],a['proposed_command'],phase['integration_dt_s'])['safe']
    measured=plain(evaluate(r,fresh,env,read(source/'scenario.json'),cfg));equal_record(measured,read(run/'metrics.json'))
    assert r['new_MPC_submitted']==sum(e.get('status')=='submitted' for e in r['events'])
    assert r['new_MPC_solved']==len(seen) and r['new_VLA_calls']==0 and r['new_optimizer_calls']==0
    return dict(valid=True,source_hashes_preserved=True,prefix_parity=True,command_integration=True,native_selection=True,metric_recomputation=True,classification=measured['classification'],new_validation_model_MPC_optimizer_calls=0)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args()
    r=validate(a.run.resolve());save(a.run/'validation.json',r);print(json.dumps(r))

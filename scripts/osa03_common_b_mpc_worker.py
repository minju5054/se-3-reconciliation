#!/usr/bin/env python3
"""Original asynchronous worker, with a pre-clock common-B initialization only."""
import sys
from pathlib import Path
sys.path[:0]=[str(Path(__file__).resolve().parent),str(Path(__file__).resolve().parents[1]/'src')]
import numpy as np
import online_mpc_worker as original
from reconciliation.join_source03 import read,sha
from reconciliation.online_mpc_adapter import load_official,OfficialMpcAdapter,audited_tracker
old_dispatch=original.dispatch


def initialize(adapter, common, ref, identity):
    assert adapter.episode_id is None and not adapter.used_solve_ids and adapter.pending is None and adapter.tracker._future is None
    adapter.reset(identity)
    installed=adapter.install(episode_id=identity,chunk_id=common['fresh_chunk_id'],reference_version=common['fresh_version'],
        raw_local_path=ref['local_path'],capture_pose=common['fresh_capture_pose'],raw_sha256=ref['local_sha256'],
        setup_only_before_counterfactual_clock=True)
    assert sha(ref['world_path'])==ref['world_sha256']
    expected=np.load(ref['world_path'],allow_pickle=False)
    np.testing.assert_allclose(adapter.world,expected,rtol=0,atol=1e-12)
    tracker=adapter.tracker;first=common['first_FRESH_solve']
    tracker._generation=common['original_generation'];adapter.installed['official_generation']=common['original_generation']
    tracker.command=tuple(common['u_B_plus']);tracker.previous_command=tuple(common['u_mem_B'])
    # Saved diagnostic outputs do not select the next horizon: official submit
    # rebuilds it from _trajectory. They represent the one common command at B.
    tracker.reference=np.array(first['selection']['reference_world']);tracker.prediction=np.array(first['prediction_world'])
    tracker.solve_ms=first['official_solve_ms'];tracker.error=''
    return dict(status='initialized_common_B',identity=identity,B=common['B'],B_tick=common['B_tick'],B_sim_s=common['B_sim_s'],
        u_minus=common['u_minus'],held_command=list(tracker.command),previous_control=list(tracker.previous_command),
        generation=tracker._generation,next_submit_tick=common['next_submit_after_B'],integration_dt_s=common['integration_dt_s'],
        installed_world=adapter.world.tolist(),capture_pose=installed['capture_pose'],reference_version=installed['reference_version'],
        historical_command_identity=first['solve_id'],restored_future_results=0,new_solve_calls=0,
        initialization_before_clock=True,install_events_during_rollout=0)


def dispatch(adapter,request):
    if request['op']=='initialize_common_B':
        assert sha(request['common_path'])==request['common_sha256']
        return initialize(adapter,read(request['common_path']),request['reference'],request['identity'])
    # Installation/reset are not allowed once the comparison clock begins.
    if request['op'] in ['reset','install','restore_saved']:raise ValueError('no runtime install/reset/historical future replay')
    return old_dispatch(adapter,request)


def preflight(checkout,common,refs):
    module,provenance=load_official(checkout)
    def forbidden(*args,**kwargs):raise AssertionError('numerical MPC forbidden during restoration preflight')
    module.MPCController.solve=forbidden
    rows=[]
    for name,ref in refs.items():
        a=OfficialMpcAdapter(module,audited_tracker(module))
        try:rows.append(initialize(a,common,ref,name))
        finally:a.close()
    return dict(passed=True,rows=rows,provenance=provenance,numerical_MPC_calls=0,LightNav_calls=0)


original.dispatch=dispatch
if __name__=='__main__':
    if '--preflight' in sys.argv:
        import argparse,json
        p=argparse.ArgumentParser();p.add_argument('--preflight',action='store_true');p.add_argument('--run',type=Path,required=True);p.add_argument('--checkout',required=True)
        a=p.parse_args();r=preflight(a.checkout,read(a.run/'common_state.json'),read(a.run/'references.json'))
        print(json.dumps(r,allow_nan=False))
    else:original.main()

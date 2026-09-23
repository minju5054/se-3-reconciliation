#!/usr/bin/env python3
"""Authoritative saved-record validation: zero new model/MPC/GP/render calls."""
import argparse,json,sys
from pathlib import Path
import numpy as np,yaml
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from run_join_online02 import read,save,sha,verify,environments
from analyze_join_online02 import analyze,call_counts,csvread,jsonlines,pose_rows
from validate_robotless_online_handoffs import validate_episode,equal_record
from reconciliation.online_history import history_contract
from reconciliation.join_online02 import ORDER,INSTRUCTION,ABORT,guard_check

def validate(run):
    verify(run,reporting=True);cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    contract,sampler=history_contract((ROOT/cfg['paths']['lightnav_checkout']).resolve(),(ROOT/cfg['paths']['checkpoint_path']).resolve())
    assert read(run/'schedule_completion.json')['no_retry']
    assert [x['episode'] for x in read(run/'schedule_completion.json')['episodes']]==ORDER
    sessions=[];reports=[];base,on,_,_=environments(run)
    for eid in ORDER:
        ep=run/'episodes'/eid;meta=read(ep/'metadata.json');dt=meta['resolved_integration_dt_s']
        report=validate_episode(ep,run,contract,sampler,require_plots=False,integration_dt_s=dt)
        assert meta['instruction']==INSTRUCTION and meta['cart_present']==eid.startswith('ON_')
        assert meta['R0']==read(run/'start_selection.json')['selected']['pose_world']
        assert meta['activated_handoffs']<=20
        if meta['initial_activation_sim_time_s'] is not None:assert meta['end_sim_time_s']-meta['initial_activation_sim_time_s']<=25+dt+1e-8
        sessions.append(read(ep/'session_open.json'))
        for p,h in read(ep/'acquisition_extension_manifest.json')['files'].items():assert sha(ep/p)==h
        states=csvread(ep/'execution.csv');commands=csvread(ep/'commands.csv');guards=jsonlines(ep/'guard.jsonl')
        assert len(states)==len(commands)+1
        environment=on if meta['cart_present'] else base
        for i,g in enumerate(guards):
            expected=guard_check(environment,g['start_pose'],g['command'],dt)
            for k,v in expected.items():equal_record(g[k],v,'guard.'+k)
            assert g['cart_present']==meta['cart_present']
            np.testing.assert_allclose(g['start_pose'],[float(states[i][k]) for k in ['x','y','yaw']],rtol=0,atol=1e-10)
            if g['safe']:
                assert i<len(commands)
                for k in ['v_mps','omega_radps']:assert float(commands[i][k])==g['command'][k]
                assert commands[i]['reason']==g['command']['reason']
            else:
                assert i==len(guards)-1==len(commands) and meta['status']==ABORT
                abort=read(ep/'guard_abort.json');assert not abort['command_applied']
                assert abort['state']['state_id']==int(states[-1]['state_id'])
                equal_record(abort['decision'],g)
        assert len(guards)==len(commands)+(meta['status']==ABORT)
        captures={f['frame_id']:f for f in jsonlines(ep/'capture.jsonl')};visibility=jsonlines(ep/'visibility.jsonl')
        assert set(captures)=={v['frame_id'] for v in visibility}
        for v in visibility:
            frame=captures[v['frame_id']]
            assert v['same_render_product_state'] and not v['model_input']
            assert v['pose_world']==frame['pose_world'] and v['camera']==frame['camera']
            assert v['state_id']==frame['rendered_state_id'] and v['sim_time_s']==frame['capture_sim_time_s']
            assert v['cart_transform']==read(run/'scenario.json')['prop']['wrapper_matrix_column']
            if not meta['cart_present']:assert v['instance']['visible_pixels']==0
            mask=np.load(ep/v['mask_path'])['mask'];ids=v['instance']['matched_instance_ids']
            assert sha(ep/v['mask_path'])==v['mask_sha256'] and mask.ndim==2
            prefix=read(run/'scenario.json')['prop']['runtime_prim']
            expected_ids=[int(k) for k,label in v['instance']['idToLabels'].items() if str(label).startswith(prefix)]
            assert sorted(ids)==sorted(expected_ids)
            assert int(np.isin(mask,ids).sum())==v['instance']['visible_pixels']
        report.update(guard_queries=len(guards),abort_command_unapplied=meta['status']==ABORT,visibility_frames=len(visibility))
        reports.append(report)
    # Session IDs must be new: raw login responses also remain in each manifest.
    assert len({s['connection_id'] for s in sessions})==4
    equal_record(read(run/'aggregate/call_counts.json'),call_counts(run),'actual call counts')
    replayed=analyze(run);saved=read(run/'aggregate/analysis.json');equal_record(saved,replayed,'all saved analysis')
    plot=read(run/'review/plot_manifest.json')
    for p,h in plot['source_hashes'].items():assert sha(p)==h
    for item in plot['figures']:
        p=run/'review'/(item['name']+'.png');s=p.with_suffix('.json')
        assert sha(p)==item['png_sha256'] and sha(s)==item['sidecar_sha256']
        side=read(s);assert side['source_hashes']==plot['source_hashes'] and side['image_sha256']==sha(p)
    # Figure numbers are embedded from the same re-derived source; verify metric series explicitly.
    for name,field in [('clearance','minimum_clearance_m'),('lateral','max_lateral_m'),('arc','raw_arc_m')]:
        numbers=read(run/'review'/(name+'_vs_distance.json'))['data'];expected=[]
        for ep in replayed['episodes']:
            for r in ep['rows']:
                if 'world' in r:expected.append(dict(episode=r['episode'],chunk=r['chunk_id'],x=r['observation_cart_center_distance_m'],y=r['raw_geometry']['whole'][field] if name=='clearance' else r[field]))
        equal_record(numbers,expected,'plot numbers')
    return dict(valid=True,reports=reports,analysis_recomputed=True,source_preserved=True,plot_numbers_recomputed=True,
        new_model_predictions=0,new_MPC_solves=0,new_GP_solves=0,new_execution=0,
        scientific_outcome=saved['overall'],scientific_failure_is_not_artifact_failure=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path);a=p.parse_args();run=a.run.resolve()
    result=validate(run);save(a.output or run/'validation.json',result);print(json.dumps(result,indent=2))
if __name__=='__main__':main()

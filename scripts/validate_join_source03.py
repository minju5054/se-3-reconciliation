#!/usr/bin/env python3
"""Independent saved-record SOURCE03 validation; no new model/controller calls."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha,luminance_statistics,mpc_audit,lighting_gate
from run_join_source03 import ORDER,verify,evaluate,lighting_result


def validate(run):
    checks={}
    def check(name,ok):checks[name]=bool(ok)
    try:verify(run);check('frozen_inputs_and_code',True)
    except (ValueError,FileNotFoundError) as e:check('frozen_inputs_and_code:'+str(e),False)
    before=read(run/'initial_source.json')['preserved']
    for p,h in before.items():
        check('preserved:'+p,(ROOT/p).is_file() and sha(ROOT/p)==h)
    check('official_mpc_current',mpc_audit(ROOT)['status']=='MPC_PROVENANCE_MATCHES_PINNED_OFFICIAL')
    cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    for which in ['render','inputs']:
        folder=run/'technical_preflight'/which
        stats=read(folder/'luminance.json')
        for label in ['DARK','BRIGHT']:
            rec=read(folder/f'{label}_ON.json')
            rgb=np.asarray(Image.open(folder/rec['rgb_file']).convert('RGB'))
            mask=np.load(folder/rec['mask_file'])['mask']
            cart=np.isin(mask,rec['instance']['matched_instance_ids'])
            targets=[int(k) for k,v in rec['instance']['idToLabels'].items() if v.startswith(cfg['source']['target_prim'])]
            target=np.isin(mask,targets)
            expected=dict(whole=luminance_statistics(rgb),cart=luminance_statistics(rgb,cart),target=luminance_statistics(rgb,target),
                          cart_pixels=int(cart.sum()),target_pixels=int(target.sum()),rgb_sha256=sha(folder/rec['rgb_file']))
            check(f'luminance:{which}:{label}',expected==stats[label])
            check(f'mask_rgb_same_state:{which}:{label}',rec['same_render_product'] and rec['stable_camera_pose_and_simulation_time'])
            check(f'mask_hash:{which}:{label}',sha(folder/rec['mask_file'])==rec['mask_sha256'])
        check('lighting_gate:'+which,lighting_gate(stats['DARK'],stats['BRIGHT'],cfg['brightness_acceptance'])['valid'])
        mutation=read(folder/'lighting_mutation.json')
        check('protected_scene:'+which,mutation['protected_before']==mutation['protected_after'])
        original=mutation['original'];bright=mutation['bright']
        check('only_one_light_added:'+which,len(bright)==len(original)+1 and all(x in bright for x in original))
    ledger=read(run/'aggregate/ledger.json')
    check('all_declared_conditions_accounted',[x['condition_id'] for x in ledger]==ORDER)
    recalculated={}
    count=0
    sessions=[]
    for row in ledger:
        cid=row['condition_id']
        if row['status']=='COMPLETED':
            new=evaluate(run,cid);stored=read(run/'paired_diagnostics'/cid/'evaluation.json')
            norm=json.loads(json.dumps(new,allow_nan=False))
            check('saved_result_recomputed:'+cid,norm==stored)
            check('all_wire_raw_world_checks:'+cid,all(new['technical_checks'].values()))
            recalculated[cid]=new;count+=new['primary_calls']
            sessions.extend(v['session_id'] for v in new['paired_inputs']['branches'].values())
        else:
            check('unavailable_no_fabricated_output:'+cid,not(run/'paired_diagnostics'/cid/'evaluation.json').exists())
    check('independent_sessions',len(sessions)==len(set(sessions)))
    check('call_budget',count<=21 and count==3*len(recalculated))
    summary=read(run/'aggregate/summary.json')
    check('summary_calls',summary['terminal_predictions']==count)
    check('no_new_MPC_GP_rollout',all(summary[k]==0 for k in ['diagnostic_MPC_solves','GP_rigid_reconciliation_solves','closed_loop_rollouts']))
    if all(c in recalculated for c in ORDER[:2]):
        check('lighting_decision',lighting_result(recalculated[ORDER[0]],recalculated[ORDER[1]],cfg)==summary['lighting'])
    a,b=[read(run/'input_manifests'/f'{c}.json') for c in ORDER[:2]]
    check('dark_bright_identical_history',a['history']==b['history'])
    check('dark_bright_identical_protected_geometry',a['protected_signatures']==b['protected_signatures'])
    a,b=[read(run/'input_manifests'/f'target_bright_H{n}.json') for n in [16,32]]
    check('H16_H32_same_terminal_bytes_pose',a['final_frames']==b['final_frames'])
    check('H16_is_suffix_of_H32',a['history']==b['history'][-15:])
    for c in ORDER[4:]:
        m=read(run/'input_manifests'/f'{c}.json');core=read(run/'input_manifests/core_bright_H16.json')
        check('distance_same_history:'+c,m['history']==core['history'])
        check('distance_same_off_sham:'+c,m['final_frames']['A']==core['final_frames']['A'])
    figures=read(run/'review/figure_manifest.json')
    for row in figures:
        check('figure_bytes:'+row['png'],sha(run/'review'/row['png'])==row['png_sha256'])
        p=run/'review'/row['sidecar'];check('sidecar_bytes:'+p.name,sha(p)==row['sidecar_sha256'])
        side=read(p)
        for source,h in side['source_sha256'].items():check('figure_source:'+source,sha(source)==h)
        stem=row['png'].removesuffix('_trajectory_clearance.png')
        if stem in recalculated:check('figure_numeric:'+stem,side['numbers']==recalculated[stem])
    return dict(valid=all(checks.values()),checks=checks,failed=[k for k,v in checks.items() if not v],
                new_model_calls=0,new_MPC_solves=0,new_GP_solves=0,scientific_failures_are_valid_artifacts=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    p.add_argument('--output',default='validation.json')
    a=p.parse_args();r=validate(a.run.resolve());save(a.run/a.output,r)
    print(json.dumps(dict(valid=r['valid'],checks=len(r['checks']),failed=r['failed'])))
    return 0 if r['valid'] else 2


if __name__=='__main__':raise SystemExit(main())

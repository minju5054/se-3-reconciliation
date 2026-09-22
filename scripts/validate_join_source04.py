#!/usr/bin/env python3
"""Saved-only SOURCE04 validation; never renders, predicts, integrates or solves."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
import yaml
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/lightnav')]
from reconciliation.join_source03 import read,save,sha,mpc_audit
from reconciliation.join_source04 import ORDER,KS,construct_frames,premodel
from run_join_source04 import verify,evaluate
from report_join_source04 import summarize
from join_source02_paired import wire_parity


def normalized(value):
    return json.loads(json.dumps(value,allow_nan=False))


def validate(run):
    checks={}
    def check(key,value):checks[key]=bool(value)
    try:verify(run);check('freeze',True)
    except (ValueError,FileNotFoundError) as e:check('freeze:'+str(e),False)
    source=read(run/'source.json')
    for group in ('preserved','official_contract_sha256'):
        for p,h in source[group].items():check('preserved:'+p,sha(ROOT/p)==h)
    check('official_mpc',mpc_audit(ROOT)['status']=='MPC_PROVENANCE_MATCHES_PINNED_OFFICIAL')
    bank=read(run/'bank_manifest.json');cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    check('exact_bank_size',len(bank)==16)
    identities=[]
    for i,row in enumerate(bank):
        for name in ('OFF','ON'):
            b=row[name];f=b['frame'];rec=read(b['capture_record'])
            check(f'jpeg:{i}:{name}',sha(f['path'])==f['sha256']==rec['rgb_jpeg_sha256'])
            check(f'mask_bytes:{i}:{name}',sha(b['mask_path'])==rec['mask_sha256'])
            mask=np.load(b['mask_path'])['mask'];depth=np.load(b['depth_path'])['distance_m']
            check(f'same_product_state:{i}:{name}',b['same_product'] and b['stable'] and b['depth_same_render_state'])
            check(f'raster_shape:{i}:{name}',mask.shape==depth.shape==(270,480) and Image.open(f['path']).size==(480,270))
            check(f'cart_pixels:{i}:{name}',int(np.isin(mask,b['instance']['matched_instance_ids']).sum())==b['cart_pixels'])
            ids=[int(k) for k,v in b['instance']['idToLabels'].items() if v.startswith(cfg['source']['target_prim'])]
            check(f'target_pixels:{i}:{name}',int(np.isin(mask,ids).sum())==b['target_pixels'])
            check(f'observation_transform:{i}:{name}',np.allclose(b['camera']['T_world_camera'],row['source']['camera']['T_world_camera'],atol=1e-12,rtol=0))
            identities.append(f['sha256'])
    check('no_duplicate_image_padding',len(set(identities))==32)
    conditions={cid:read(run/'conditions'/f'{cid}.json') for cid in ORDER}
    for cid,m in conditions.items():
        check('construction:'+cid,m['frames']==construct_frames(bank,KS[cid]))
        check('premodel:'+cid,m['premodel']==premodel(bank,KS[cid]))
    for a,b in [('K0_OFF','K0_OFF_SHAM'),('K1','K1_SHAM')]:
        check('byte_identical_sham_history:'+a,conditions[a]['frames']==conditions[b]['frames'])
    base=conditions['K0_OFF']['frames']
    for cid,m in conditions.items():
        check('same_pose_id_time:'+cid,[(f['frame_id'],f['pose_world'],f['capture_sim_time_s']) for f in m['frames']]==[(f['frame_id'],f['pose_world'],f['capture_sim_time_s']) for f in base])
    ledger=read(run/'aggregate/ledger.json');check('frozen_order',[r['condition'] for r in ledger]==ORDER)
    results={};sessions=[];next_count=0;predictions=0
    for row in ledger:
        cid=row['condition'];out=run/'predictions'/cid
        if row['status']=='COMPLETED':
            r=evaluate(run,cid);results[cid]=r
            check('recomputed:'+cid,normalized(r)==read(out/'evaluation.json'))
            frames=read(out/'replayed_inputs.json')['frames'];w=wire_parity(out,frames)
            check('wire:'+cid,w['valid'] and w['actual_terminal_predictions_sent']==1)
            check('indices:'+cid,[x['seq'] for x in w['rows']]==list(range(16)))
            check('buffer_then_terminal:'+cid,[x['prediction_request'] for x in w['rows']]==[False]*15+[True])
            check('wire_bank_bytes:'+cid,[f['sha256'] for f in frames]==[f['sha256'] for f in conditions[cid]['frames']])
            close=read(out/'session_close.json');sessions.append(close['connection_id'])
            check('independent_no_retry:'+cid,close['connection_count']==1 and close['wire_login_count']==1 and close['wire_reset_count']==1 and close['retry_count']==close['reconnect_count']==0)
            next_count+=w['actual_next_requests_sent'];predictions+=w['actual_terminal_predictions_sent']
        else:check('missing_not_fabricated:'+cid,not(out/'evaluation.json').exists())
    check('unique_sessions',len(sessions)==len(set(sessions)))
    check('budget',predictions<=7 and predictions==sum(x.get('calls') or 0 for x in ledger))
    summary=read(run/'aggregate/summary.json');check('summary_recomputed',normalized(summarize(results,ledger))==summary)
    check('no_MPC_GP_execution',summary['new_MPC_GP_reconciliation_rollout']==0)
    for fig in read(run/'review/figure_manifest.json'):
        check('figure:'+fig['png'],sha(run/'review'/fig['png'])==fig['sha256'])
        side=read(run/'review'/fig['sidecar'])
        for p,h in side['input_sha256'].items():check('figure_source:'+p,sha(p)==h)
        check('figure_config:'+fig['png'],side['config_sha256']==sha(run/'config_snapshot.yaml'))
        numbers=side['numbers'];stem=fig['png'].removesuffix('.png')
        expected=None
        if stem.startswith('history_'):expected=conditions[stem.removeprefix('history_')]
        elif stem in ('world_trajectory_overlay','affordance_action_geometry'):expected={k:results[k] for k in ('K0_OFF','K1','K2','K4','K8') if k in results}
        elif stem in ('terminal_affordance','apos_vs_K'):expected={k:r['apos'] for k,r in results.items() if stem=='terminal_affordance' or k not in ('K0_OFF_SHAM','K1_SHAM')}
        elif stem=='row_arc_clearance':expected={k:r['geometry_on'] for k,r in results.items() if k not in ('K0_OFF_SHAM','K1_SHAM')}
        elif stem=='geometry_vs_K':expected={k:r['motion'] for k,r in results.items() if k not in ('K0_OFF_SHAM','K1_SHAM')}
        elif stem=='evidence_matrix':expected=summary
        check('figure_numbers:'+fig['png'],expected is not None and normalized(expected)==numbers)
    return dict(valid=all(checks.values()),checks=checks,failed=[k for k,v in checks.items() if not v],
        actual_scientific_predictions=predictions,actual_buffer_only_requests=next_count-predictions,
        new_validation_model_MPC_GP_calls=0,scientific_failure_is_not_corruption=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True);p.add_argument('--output',default='validation.json')
    a=p.parse_args();r=validate(a.run.resolve());save(a.run/a.output,r)
    print(json.dumps(dict(valid=r['valid'],checks=len(r['checks']),failed=r['failed'])))
    return 0 if r['valid'] else 2


if __name__=='__main__':raise SystemExit(main())

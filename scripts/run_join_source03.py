#!/usr/bin/env python3
"""Freeze and run source-only lighting/factor diagnostics; no motion execution."""
from __future__ import annotations
import argparse
from copy import deepcopy
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts/lightnav')]
from reconciliation.join_source03 import (read, save, sha, path_geometry, trajectory_equal,
                                        lighting_classification, mpc_audit)
from reconciliation.join_source02_geometry import project_prop, compose_environment
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.join_source02 import (evaluate_screening, validate_paired_inputs, REQUIRED_SCREENING_CHECKS,
                                         observation_anchored_world)
from join_source02_paired import validate_manifest, wire_parity
from run_join_source02 import source_files

ORDER = ['core_dark_H16', 'core_bright_H16', 'target_bright_H16', 'target_bright_H32',
         'distance_near_H16', 'distance_medium_H16', 'distance_far_H16']


def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args], text=True).strip()


def prepare(run):
    if (run/'freeze.json').exists():
        raise FileExistsError('freeze exists: no overwrite')
    cfg = yaml.safe_load((ROOT/'configs/join_source_03_bright_cause.yaml').read_text())
    base = yaml.safe_load((ROOT/cfg['base_config']).read_text())
    config = {**base, **cfg}
    if read(run/'mpc_audit/result.json')['status'] != 'MPC_PROVENANCE_MATCHES_PINNED_OFFICIAL':
        raise ValueError('MPC gate failed')
    if not read(run/'technical_preflight/inputs/validation.json')['valid']:
        raise ValueError('lighting gate failed')
    with (run/'config_snapshot.yaml').open('x') as f:
        yaml.safe_dump(config, f, sort_keys=False)
    protocol = dict(experiment=cfg['experiment'], declaration=cfg,
        development=dict(condition_order=ORDER),
        input_semantics='actual paused Isaac terminal renders with original authentic DARK history; not online',
        target_history_semantics='conditional new pose from saved actual frame32; geometry change limits causal grounding attribution',
        no_new_MPC_GP_or_rollout=True, no_history_cloning=True)
    save(run/'protocol.json', protocol)
    env = HospitalEnvironment.load(ROOT/cfg['environment'])
    records = {}
    for cid in ORDER:
        folder = run/'paired_inputs'/cid
        m = read(folder/'render_manifest.json')
        # H16/H32 MUST use exactly the same terminal RGB bytes/camera/cart.
        # The extra H32 technical renders remain saved but are never sent.
        if cid == 'target_bright_H32':
            same = read(run/'paired_inputs/target_bright_H16/render_manifest.json')
            m['unused_technical_final_frames'] = m['final_frames']
            for key in ('final_frames', 'prop', 'triangles_path', 'cart_pixels', 'target_pixels', 'protected_signatures'):
                m[key] = deepcopy(same[key])
        if cid.startswith('distance'):
            same = read(run/'paired_inputs/core_bright_H16/render_manifest.json')
            m['unused_technical_off_frame'] = m['final_frames']['A']
            for alias in ('A','SHAM'):
                m['final_frames'][alias] = deepcopy(same['final_frames'][alias])
        tri = np.load(m['triangles_path'])['triangles']
        projection = project_prop(tri)
        on = compose_environment(env, projection, True)
        pose = np.asarray(m['observation_pose_world'])
        forward = np.array([np.cos(pose[2]), np.sin(pose[2])])
        left = np.array([-forward[1], forward[0]])
        centered = tri.reshape(-1, 3)[:, :2]-m['center_xy']
        half = float(np.max(np.abs(centered@forward)))
        width = float(np.max(np.abs(centered@left)))
        passages = []
        for sign in (-1, 1):
            p = np.asarray(m['center_xy'])+sign*(width+.26)*left
            passages.append(on.check_polyline([p-(half+.3)*forward, p+(half+.3)*forward]))
        gates = dict(observation_safe=on.query(pose[:2])['status'] == 'CLEARANCE_VALID',
                     cart_hidden_off=m['cart_pixels']['A'] == 0,
                     cart_visible_on=m['cart_pixels']['B'] >= 20,
                     free_side_passage=any(x['clearance_valid'] for x in passages))
        if cid.startswith('target'):
            gates['target_visible_both_frames'] = all(n >= 20 for n in m['target_pixels'].values())
        if cid.startswith('distance'):
            old = np.asarray(m['source_no_obstacle_path'])
            gates['historical_off_path_blocked'] = not on.check_polyline(old)['clearance_valid']
            gates['cart_within_original_output_range'] = float((old[-1, :2]-m['center_xy'])@forward) >= half
        m.update(config_path=str(run/'config_snapshot.yaml'), projection=projection['metadata'],
            influence=dict(origin_xy=m['center_xy'], forward_xy=forward.tolist(), progress_min_m=-half-.5, progress_max_m=half+.5),
            bypass_plane=dict(point_xy=(np.asarray(m['center_xy'])+(half+.25)*forward).tolist(), forward_xy=forward.tolist()),
            lateral_passages=passages, technical_input_checks=gates, eligible=all(gates.values()))
        validate_manifest(m, protocol)
        save(run/'input_manifests'/f'{cid}.json', m)
        records[cid] = dict(eligible=m['eligible'], checks=gates, cart_pixels=m['cart_pixels'], target_pixels=m['target_pixels'])
    # Identical protected scene fingerprint at each cart state, excluding only fill.
    a, b = [read(run/'input_manifests'/f'{c}.json') for c in ORDER[:2]]
    if a['protected_signatures'] != b['protected_signatures'] or a['history'] != b['history']:
        raise ValueError('DARK/BRIGHT geometry/history identity failed')
    sources = source_files()
    for pattern in ['src/reconciliation/join_source03*.py', 'scripts/*join_source03*.py',
                    'scripts/isaac/*join_source03*.py', 'scripts/lightnav/*join_source03*.py',
                    'tests/test_join_source03*.py', 'configs/join_source_03_bright_cause.yaml']:
        sources.update({str(p.relative_to(ROOT)):sha(p) for p in ROOT.glob(pattern)})
    files = [run/'protocol.json', run/'config_snapshot.yaml', run/'mpc_audit/result.json']
    files.extend((run/'input_manifests').glob('*.json'))
    for m in [read(run/'input_manifests'/f'{c}.json') for c in ORDER]:
        files.extend(Path(f['path']) for f in [*m['history'], *m['final_frames'].values()])
        files.append(Path(m['triangles_path']))
    save(run/'freeze.json', dict(preparation_sha=git('rev-parse', 'HEAD'), source_sha256=sources,
        input_sha256={str(p):sha(p) for p in sorted(set(files))}, input_gates=records,
        frozen_before_model_prediction=True, expected_maximum_predictions=21))
    print(records)


def verify(run, *, pushed=False):
    f = read(run/'freeze.json')
    for paths in [f['source_sha256'], f['input_sha256']]:
        for p, digest in paths.items():
            path = Path(p) if Path(p).is_absolute() else ROOT/p
            if sha(path) != digest:
                raise ValueError(f'frozen source/input changed: {p}')
    if pushed:
        if git('rev-parse','HEAD') != git('rev-parse','@{upstream}'):
            raise ValueError('pre-execution commit/push required')
        if git('status','--porcelain','--',*f['source_sha256']):
            raise ValueError('frozen code has uncommitted changes')
    return f


def evaluate(run, cid):
    """Saved records only; actual wire/reference/response checks before geometry."""
    out = run/'paired_diagnostics'/cid
    m = read(run/'input_manifests'/f'{cid}.json')
    worker = read(out/'summary.json')
    config = yaml.safe_load((run/'config_snapshot.yaml').read_text())
    base = HospitalEnvironment.load(m['environment_export'])
    projection = project_prop(np.load(m['triangles_path'])['triangles'])
    if projection['metadata'] != m['projection']:
        raise ValueError('saved projection drift')
    on = compose_environment(base, projection, True)
    paired, paths, stops, details = {}, {}, {}, {}
    good = True
    for alias, label in [('A','A_OFF'),('B','B_ON'),('SHAM','A_SHAM')]:
        b = out/'branches'/alias
        result = read(b/'paired_result.json')
        if result['status'] not in ('PREDICTION_READY', 'MODEL_STOP'):
            return dict(condition_id=cid, label='TECHNICAL_INVALID', reason=result.get('error', result['status']),
                        primary_calls=worker['actual_terminal_predictions'], details=details)
        frames = read(b/'replayed_inputs.json')['frames']
        wire = wire_parity(b, frames)
        raw = np.load(b/'chunks/terminal/raw_local.npy')
        world = np.load(b/'chunks/terminal/world.npy')
        response = read(b/'chunks/terminal/response.json')['data']
        raw_ok = np.array_equal(raw, np.asarray(response['actions']['actions'], dtype=np.float64))
        world_ok = np.allclose(world, observation_anchored_world(raw,m['final_frames'][alias]['pose_world']), atol=1e-12,rtol=0)
        good = good and wire['valid'] and raw_ok and world_ok
        paired[label] = dict(session_id=result['session_close']['connection_id'], instruction=m['instruction'],
            camera_config=config['camera'], model_config=config['lightnav'], history=m['history'], final_frame=m['final_frames'][alias])
        paths[label], stops[label] = world, response['stop']
        details[alias] = dict(raw_local=raw.tolist(), world=world.tolist(), raw_sha256=sha(b/'chunks/terminal/raw_local.npy'),
            world_sha256=sha(b/'chunks/terminal/world.npy'), wire_valid=wire['valid'], raw_literal_parity=raw_ok,
            world_observation_anchor_parity=world_ok, raw_text=response.get('raw_text'), pointing=response.get('pointing'),
            target_visible=response.get('visible'), stop=response['stop'],
            geometry_off=path_geometry(world,base), geometry_on=path_geometry(world,on),
            client_rtt_s=result['terminal_prediction']['client_rtt_s'], server_timings_ms=response.get('timings_ms'),
            terminal_record_key=f'paired_diagnostics/{cid}/branches/{alias}/chunks/terminal/response.json')
    parity = validate_paired_inputs(paired)
    flags = dict(source_integrity=bool(good), official_contract=worker['source_unchanged'] and worker['checkpoint_stat_unchanged'],
                 paired_inputs=parity['valid'], wire_jpeg_parity=bool(good), visibility=m['cart_pixels']['A']==0 and m['cart_pixels']['B']>=20,
                 geometry_activation=bool(m['technical_input_checks']['free_side_passage']))
    result = evaluate_screening(paths['A_OFF'],paths['B_ON'],paths['A_SHAM'],
        observation_pose_world=m['observation_pose_world'], target_xy=m['target_xy'],
        environment_off=base, environment_on=on, influence=m['influence'], bypass_plane=m['bypass_plane'],
        technical_checks=flags, stops=stops)
    return dict(result, condition_id=cid, details=details, paired_inputs=parity,
                primary_calls=worker['actual_terminal_predictions'], buffer_calls=worker['actual_buffer_only'],
                worker_wall_s=worker['wall_s'])


def lighting_result(dark, bright, config):
    def entry(d):
        return dict(safe=d['whole_raw_on_safe'], meaningful_response=d['off_on']['meaningful'],
                    stop=d['stops']['B_ON'], clearance_m=d['checks']['B_ON']['on']['minimum_clearance_m'])
    if any(d['label']=='TECHNICAL_INVALID' for d in (dark,bright)):
        return 'TECHNICAL_BLOCKER'
    equivalent=trajectory_equal(dark['details']['B']['raw_local'],bright['details']['B']['raw_local'],
        config['comparison']['geometric_equivalence_xy_m'],config['comparison']['geometric_equivalence_yaw_rad'])
    return lighting_classification(entry(dark),entry(bright),equivalent=equivalent,
        material_clearance_gain_m=config['comparison']['material_clearance_gain_m'])


def execute(run):
    verify(run,pushed=True)
    save(run/'execution.json', dict(sha=git('rev-parse','HEAD'), branch=git('branch','--show-current'),
                                   begun_host_monotonic_s=time.monotonic(), no_retry=True))
    config=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    checkout=(ROOT/config['paths']['lightnav_checkout']).resolve()
    ledger, results=[], {}
    for cid in ORDER:
        m=read(run/'input_manifests'/f'{cid}.json')
        reason=None
        if cid.startswith(('target','distance')):
            if any(isinstance(x,dict) and x.get('label')=='TECHNICAL_INVALID' for x in results.values()):
                reason='preceding technical failure; diagnostic blocked'
            elif results.get('lighting_class')=='BRIGHT_RECOVERS_SAFE_RESPONSE':
                reason='conditional follow-up unnecessary: lighting recovered safe response'
            elif not all(c in results for c in ORDER[:2]):
                reason='core paired comparison unavailable'
        if cid=='target_bright_H32' and 'target_bright_H16' in results:
            r=results['target_bright_H16']
            if r.get('whole_raw_on_safe') and r.get('off_on',{}).get('meaningful') and not r.get('stops',{}).get('B_ON'):
                reason='target-visible H16 recovered safe response'
        if not m['eligible']:
            reason='frozen technical input criteria unavailable: '+','.join(k for k,v in m['technical_input_checks'].items() if not v)
        if reason:
            row=dict(condition_id=cid,status='NOT_RUN',reason=reason,primary_calls=0)
        else:
            argv=[str(checkout/'.venv/bin/python'),str(ROOT/'scripts/lightnav/join_source03_paired.py'), '--run',str(run),'--condition',cid]
            log=run/'logs'/f'{cid}.log';log.parent.mkdir(exist_ok=True)
            with log.open('x') as stream:
                p=subprocess.run(argv,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
            if p.returncode:
                save(run/'paired_diagnostics'/cid/'process_failure.json',dict(exit_code=p.returncode,argv=argv))
                row=dict(condition_id=cid,label='TECHNICAL_INVALID',status='ATTEMPTED_ERROR',primary_calls=None)
            else:
                result=evaluate(run,cid)
                save(run/'paired_diagnostics'/cid/'evaluation.json',result)
                results[cid]=result
                row=dict(condition_id=cid,status='COMPLETED',label=result['label'],primary_calls=result['primary_calls'])
        ledger.append(row)
        save(run/'aggregate/ledger_steps'/f'{len(ledger):02}.json',row)
        if cid=='core_bright_H16' and all(c in results for c in ORDER[:2]):
            results['lighting_class']=lighting_result(results[ORDER[0]],results[ORDER[1]],config)
            save(run/'aggregate/lighting_decision.json',dict(classification=results['lighting_class']))
        print(row,flush=True)
    save(run/'aggregate/ledger.json',ledger)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--mode',choices=['prepare','verify','execute'],required=True)
    a=p.parse_args();run=a.run.resolve()
    if a.mode=='prepare':prepare(run)
    elif a.mode=='verify':verify(run,pushed=True)
    else:execute(run)


if __name__=='__main__':main()

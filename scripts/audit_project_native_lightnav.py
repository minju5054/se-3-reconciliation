#!/usr/bin/env python3
"""PROJECT-AUDIT-01: SAVED-ONLY AUDIT. Zero inference/MPC/optimizer/simulator calls.

Creates an exclusive derived directory. Never writes to historical inputs.
Recomputation imports only saved evaluators; scientific execution entry points
are never called. Matching is sealed before execution outcomes are computed.
"""
from __future__ import annotations
import argparse
import base64
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import sys
import csv

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts'), str(ROOT/'scripts/lightnav')]
import numpy as np
import yaml
from reconciliation.project_audit import (shape_features, saved_window, match_controls,
    execution_metrics, summarize_events)
from reconciliation.genuine_source_scan import run_scan
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_join01_environment import RevealEnvironment
from reconciliation.gp_se2_join01_qualification import qualification_from_episode
from reconciliation.join_source03 import mpc_audit, path_geometry
from reconciliation.se2 import local_trajectory_to_world, wrap_angle
from audit_saved_handoff_losses import plain


def sha(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def save(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as f:
        json.dump(plain(value), f, indent=2, allow_nan=False)
        f.write('\n')


def csv_save(path, rows):
    if not rows:
        return
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with Path(path).open('x', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows({k:json.dumps(plain(v), sort_keys=True) if isinstance(v,(list,dict,np.ndarray)) else v for k,v in r.items()} for r in rows)


class Inputs:
    def __init__(self):
        self.hashes = {}

    def record(self, path):
        p = Path(path).resolve()
        h = sha(p)
        if str(p) in self.hashes and self.hashes[str(p)] != h:
            raise ValueError('input changed during audit: '+str(p))
        self.hashes[str(p)] = h
        return p

    def read(self, path):
        return json.loads(self.record(path).read_text())

    def csv(self, path):
        with self.record(path).open() as f:
            return list(csv.DictReader(f))

    def array(self, path):
        a = np.load(self.record(path), allow_pickle=False)
        a.flags.writeable = False
        return a

    def tree(self, path):
        for p in sorted(Path(path).rglob('*')):
            if p.is_file() and '__pycache__' not in p.parts:
                self.record(p)

    def verify(self):
        for path, h in self.hashes.items():
            if sha(path) != h:
                raise ValueError('input changed: '+path)


def git(*args, root=ROOT):
    return subprocess.check_output(['git', '-C', str(root), *args], text=True).strip()


def provenance(cfg, inputs, out):
    external = Path(cfg['external_checkout'])
    checkpoint = Path(cfg['checkpoint'])
    source_files = git('ls-files', root=external).splitlines()
    sources = {p:sha(inputs.record(external/p)) for p in source_files if (external/p).is_file()}
    launches = {'CORPUS': ROOT/inputs.read(ROOT/cfg['corpus']/'server_link.json')['server_launch']['path']}
    launches.update({k: ROOT/v/'server_launch.json' for k,v in cfg['families'].items()})
    records = {k:inputs.read(p) for k,p in launches.items()}
    checkpoint_checks = []
    for row in records['SOURCE04']['checkpoint_files']:
        p = inputs.record(checkpoint/row['path'])
        current_hash = inputs.hashes[str(p)]
        historical = {k:next(r['sha256'] for r in v['checkpoint_files'] if r['path']==row['path']) for k,v in records.items()}
        checkpoint_checks.append(dict(path=str(p),sha256=current_hash,bytes=p.stat().st_size,
            all_historical_match=all(h==current_hash for h in historical.values()), historical=historical))
    revisions = ['eed5f2c','0de41fa','3e08222','1904bd2','5f06f07','7682acd','6b6cb01']
    tracked = git('ls-files','scripts/lightnav','scripts/online_lightnav_worker.py','scripts/online_mpc_worker.py',
                  'scripts/isaac/robotless_online_handoffs.py','src/reconciliation','configs').splitlines()
    history = []
    for path in tracked:
        values = {}
        for rev in revisions:
            exists = subprocess.run(['git','cat-file','-e',rev+':'+path], cwd=ROOT, capture_output=True).returncode==0
            if exists:
                content = subprocess.check_output(['git','show',rev+':'+path],cwd=ROOT)
                values[rev] = hashlib.sha256(content).hexdigest()
            else:
                values[rev] = None
        history.append(dict(component=path,**values))
    csv_save(out/'metrics/historical_component_hashes.csv',history)
    scopes=['scripts/lightnav','scripts/online_lightnav_worker.py','scripts/online_mpc_worker.py',
        'scripts/isaac/robotless_online_handoffs.py','src/reconciliation/online_mpc_adapter.py',
        'src/reconciliation/se2.py','src/reconciliation/gp_se2_reference.py','src/reconciliation/gp_se2_ref02_rollout.py']
    for a,b in zip(revisions,revisions[1:]):
        (out/'metrics'/f'history_{a}_{b}.diff').write_text(git('diff',a,b,'--',*scopes)+'\n')
    result = dict(external_sha=git('rev-parse','HEAD',root=external), external_status=git('status','--porcelain',root=external),
        external_source_sha256=sources,checkpoint_files=checkpoint_checks,launches=records,mpc=mpc_audit(ROOT),
        history_revisions={r:git('rev-parse',r) for r in revisions},
        generation_caveat='Original corpus/JOIN01 launcher records do not serialize all inherited generation environment variables; default source plus logs is weaker than a complete process environment snapshot.')
    save(out/'metrics/provenance.json', result)
    return result


def load_decoder(cfg, inputs):
    path = inputs.record(Path(cfg['external_checkout'])/'src/lightnav/traj_vocab.py')
    spec = importlib.util.spec_from_file_location('_audit_official_rvq',path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    bundle = Path(cfg['checkpoint'])/'action_tokenizer'
    inputs.tree(bundle)
    return module.load_rvq_bundle(bundle,10)


def raw_trace(folder, anchor, inputs, decoder, family, case):
    raw = inputs.array(folder/'raw_local.npy')
    world = inputs.array(folder/'world.npy')
    response = inputs.read(folder/'response.json')['data']
    codes = [int(code) for _,code in sorted((int(l),int(c)) for l,c in re.findall(r'<act_l(\d+)_(\d+)>',response['raw_text']))]
    decoded = np.zeros_like(raw) if decoder.is_stop(codes) else decoder.decode_waypoints(codes).astype(float)
    # Official STOP snap threshold is 1e-3; all non-STOP records checked literally.
    if response['stop'] and np.all(np.abs(decoded)<=1e-3):
        decoded = np.zeros_like(raw)
    expected = local_trajectory_to_world(anchor,raw)
    result = dict(family=family,case=case,raw_path=str(folder/'raw_local.npy'),raw_sha256=sha(folder/'raw_local.npy'),
        world_sha256=sha(folder/'world.npy'),response_sha256=sha(folder/'response.json'),
        raw_response_equal=np.array_equal(raw,np.asarray(response['actions']['actions'],float)),
        saved_token_decode_equal=np.array_equal(raw,decoded),
        observation_transform_equal=bool(np.allclose(world[:,:2],expected[:,:2],atol=1e-12,rtol=0) and np.allclose(wrap_angle(world[:,2]-expected[:,2]),0,atol=1e-12,rtol=0)),
        max_decode_error=float(np.abs(raw-decoded).max()),codes=codes,raw_text=response['raw_text'],
        pointing=response.get('pointing'),stop=response['stop'],target_visible=response.get('visible'),
        intrinsic_waypoint_dt_s=None,anchor_world=anchor)
    if not all(result[k] for k in ('raw_response_equal','saved_token_decode_equal','observation_transform_equal')):
        raise ValueError('raw provenance mismatch: '+case)
    return result


def online_wire(ep, inputs):
    checks=[]
    for p in sorted((ep/'chunks').glob('*/metadata.json')):
        m=inputs.read(p)
        obs=m['observation'];seq=m['seq']
        matching=[]
        for w in sorted((ep/'requests').glob('*_next_request.json')):
            d=inputs.read(w)['data']
            if d['seq']==seq:
                payload=base64.b64decode(d['image']);matching.append((w,payload))
        if len(matching)!=1:raise ValueError('ambiguous request')
        w,payload=matching[0];image=inputs.record(ep/obs['path'])
        good=payload==image.read_bytes() and sha(image)==obs['sha256']
        if not good:raise ValueError('wire/JPEG mismatch')
        checks.append(dict(chunk=p.parent.name,seq=seq,jpeg=str(image),jpeg_sha256=sha(image),wire=str(w),wire_sha256=sha(w),exact=True))
    return checks


def q1(cfg, inputs, out, decoder, environment):
    from run_join_source02 import evaluate_saved, load_environments
    from run_join_source03 import evaluate as evaluate03
    from run_join_source04 import evaluate as evaluate04
    from reconciliation.join_source04 import construct_frames, premodel, ORDER, KS
    records={};raws=[];joins=[];wires={}
    for family,path in cfg['families'].items():
        run=ROOT/path
        inputs.tree(run)
        if family=='JOIN01':
            protocol=inputs.read(run/'protocol.json')
            for ep in sorted((run/'source_event/episodes').iterdir()):
                q=qualification_from_episode(run/'source_event',ep.name,environment,protocol)
                if plain(q)!=inputs.read(run/'qualification'/f'{ep.name}.json'):raise ValueError('JOIN qualification drift')
                c=inputs.read(ep/'handoffs/handoff_000/context.json')
                env=RevealEnvironment(environment,q['obstacle'])
                fresh=inputs.array(ep/c['fresh_world_ref']['path']);old=inputs.array(ep/c['old_world_ref']['path'])
                states=inputs.csv(ep/'execution.csv');commands=inputs.csv(ep/'commands.csv')
                geometry=path_geometry(fresh,env)
                obs=c['R_obs'];B=c['B'];center=q['obstacle']['pose_world']
                travel=sum(abs(float(cmd['v_mps']))*(float(states[int(cmd['application_state_id'])+1]['sim_time_s'])-float(cmd['sim_time_s'])) for cmd in commands if c['obs_state_id']<=int(cmd['application_state_id'])<c['switch_state_id'])
                inflight=[cmd for cmd in commands if c['t_request']['host_monotonic_s']<=float(cmd['host_monotonic_s'])<c['t_ready_host']['host_monotonic_s']]
                joins.append(dict(attempt=ep.name,observation_pose=obs,B=B,u_minus=c['pre_switch_command'],
                    previous_control=c['first_fresh_solve']['previous_command'],first_fresh_command=c['first_fresh_command'],
                    timestamp_records={k:c[k] for k in ('t_obs','t_request','t_ready_host','t_ready_seen_sim','t_install','t_switch')},
                    reveal=q['timing']['reveal'],obstacle=q['obstacle'],obs_to_B_travel_m=travel,
                    observation_to_obstacle_center_m=float(np.linalg.norm(np.array(obs[:2])-center[:2])),
                    observation_edge_clearance_m=env.query(obs[:2])['clearance_m'],B_edge_clearance_m=q['B_query']['clearance_m'],
                    raw_full_geometry=geometry,suffix_geometry=q['fresh_suffix'],old_before=q['original_old'],old_after=q['revealed_old'],
                    mismatch_m=q['maximum_cross_track_m'],mismatch_yaw_deg=q['maximum_projected_pose_yaw_difference_deg'],
                    old_commands_during_request=inflight,all_inflight_commands_OLD=all(x['chunk_id']==c['old_chunk_id'] for x in inflight),
                    qualification=q['flags'],fresh_world=fresh,old_world=old))
                wires[ep.name]=online_wire(ep,inputs)
                for name,anchor in [(c['old_chunk_id'],c['R_old_obs']),(c['fresh_chunk_id'],obs)]:
                    raws.append(raw_trace(ep/'chunks'/name,anchor,inputs,decoder,family,ep.name+'/'+name))
            records[family]=joins
        elif family=='SOURCE02':
            rows=[]
            for p in sorted((run/'development/input_manifests').glob('*.json')):
                manifest=inputs.read(p);manifest['_manifest_path']=str(p)
                if manifest['availability']!='AVAILABLE':
                    rows.append(dict(condition=manifest['condition_id'],availability=manifest['availability']));continue
                r=evaluate_saved(run,manifest)
                stored=inputs.read(run/'development'/manifest['condition_id']/'evaluation.json')
                # The acquisition orchestrator adds exit status after evaluation;
                # it is not an evaluator field. Check it separately, omit nothing else.
                if stored.pop('worker_exit_code') != 0 or plain(r)!=stored:
                    raise ValueError('SOURCE02 drift')
                off,on=load_environments(inputs.read(manifest['placement_path']))
                branches={}
                for alias in ('A','B','SHAM'):
                    folder=run/'development'/manifest['condition_id']/'branches'/alias/'chunks/terminal'
                    trace=raw_trace(folder,manifest['final_frames'][alias]['pose_world'],inputs,decoder,family,manifest['condition_id']+'/'+alias)
                    raws.append(trace);branches[alias]=dict(trace=trace,geometry_on=path_geometry(inputs.array(folder/'world.npy'),on))
                rows.append(dict(condition=manifest['condition_id'],result=r,branches=branches))
            records[family]=rows
        elif family=='SOURCE03':
            rows=[]
            for p in sorted((run/'input_manifests').glob('*.json')):
                cid=p.stem;m=inputs.read(p);r=evaluate03(run,cid)
                if plain(r)!=inputs.read(run/'paired_diagnostics'/cid/'evaluation.json'):raise ValueError('SOURCE03 drift')
                for alias in ('A','B','SHAM'):
                    raws.append(raw_trace(run/'paired_diagnostics'/cid/'branches'/alias/'chunks/terminal',m['final_frames'][alias]['pose_world'],inputs,decoder,family,cid+'/'+alias))
                rows.append(dict(condition=cid,result=r,observation_to_cart_center_m=float(np.linalg.norm(np.array(m['center_xy'])-np.array(m['observation_pose_world'][:2]))),
                    cart_pixels=m['cart_pixels'],target_pixels=m['target_pixels'],side_passages=m['lateral_passages'],technical=m['technical_input_checks']))
            records[family]=rows
        else:
            bank=inputs.read(run/'bank_manifest.json');rows=[]
            for cid in ORDER:
                m=inputs.read(run/'conditions'/f'{cid}.json')
                if m['frames']!=construct_frames(bank,KS[cid]) or m['premodel']!=premodel(bank,KS[cid]):raise ValueError('persistence construction drift')
                r=evaluate04(run,cid)
                if plain(r)!=inputs.read(run/'predictions'/cid/'evaluation.json'):raise ValueError('SOURCE04 drift')
                raws.append(raw_trace(run/'predictions'/cid/'chunks/terminal',m['frames'][-1]['pose_world'],inputs,decoder,family,cid))
                rows.append(dict(condition=cid,result=r,frame_hashes=[f['sha256'] for f in m['frames']],
                    all_bright=all(f['all_bright'] for f in m['frames']),last_K_presence=[f['presence'] for f in m['frames']]))
            records[family]=rows
    # Original online representative, both original OLD and FRESH.
    ep=ROOT/cfg['corpus']/'episodes/episode_013_repeat_01'
    c=inputs.read(ep/'handoffs/handoff_024/context.json')
    for name,anchor in [(c['old_chunk_id'],c['R_old_obs']),(c['fresh_chunk_id'],c['R_obs'])]:
        raws.append(raw_trace(ep/'chunks'/name,anchor,inputs,decoder,'CORPUS','episode_013_repeat_01/'+name))
    wires['CORPUS_episode_013_repeat_01']=online_wire(ep,inputs)
    save(out/'metrics/q1_recomputed.json',records)
    save(out/'metrics/raw_fresh_end_to_end.json',raws)
    save(out/'metrics/online_wire_parity.json',wires)
    return records,raws


def q2(cfg, inputs, out, environment):
    scan_config=yaml.safe_load(inputs.record(ROOT/cfg['source_scan_config']).read_text())
    scan=run_scan(ROOT,scan_config)
    historical=inputs.read(ROOT/cfg['source_scan']/'ledger.json')
    if scan['rows']!=historical:raise ValueError('881 source scan does not reproduce')
    for p,h in scan['source_hashes'].items():
        if sha(inputs.record(p))!=h:raise ValueError('scan source hash mismatch')
    save(out/'metrics/recomputed_source_summary.json',scan['summary'])
    schedule=inputs.read(ROOT/cfg['corpus']/'episode_schedule.json')['episodes']
    categories={r['episode_id']:r['category'] for r in schedule}
    hard_ids=scan['summary']['subsets']['candidate']['case_ids']
    hard_episodes={s.split('/')[0] for s in hard_ids}
    streams={};cases={};features=[]
    for row in scan['rows']:
        cid=row['case_id'];episode=cid.split('/')[0];ep=ROOT/cfg['corpus']/'episodes'/episode
        if episode not in streams:
            streams[episode]=(inputs.csv(ep/'execution.csv'),inputs.csv(ep/'commands.csv'))
        context=inputs.read(row['source_paths']['context'])
        world=inputs.array(row['source_paths']['fresh_world']);old=inputs.array(row['source_paths']['old_world'])
        window=saved_window(context,*streams[episode])
        m=row['mismatch'];motion=row['motion']
        safe=bool(row['entire_raw_environment']['clearance_valid'] and row['boundary_environment']['status']=='CLEARANCE_VALID' and row['past_environment']['clearance_valid'])
        feature=dict(case_id=cid,episode_id=episode,category=categories[episode],source_safe=safe,
            original_candidate=row['flags']['candidate'],original_base_source_valid=row['flags']['base_source_valid'],
            raw_full_safe=row['entire_raw_environment']['clearance_valid'],B_safe=row['boundary_environment']['status']=='CLEARANCE_VALID',
            **shape_features(world),lateral_m=m['signed_lateral_gap_m'],pose_yaw_deg=m['pose_yaw_difference_deg'],
            direction_deg=m['reliable_window_direction_deg'],v_minus_mps=motion['v_minus_mps'],
            omega_minus_radps=motion['omega_minus_radps'],obs_to_B_travel_m=motion['observation_to_B_arc_m'],
            lifetime_s=float(window['times'][-1]),projection_location=m['projection']['projection_location'],
            reference_id=context['fresh_chunk_id'],t_obs=context['t_obs'],t_ready=context['t_ready_seen_sim'],
            t_switch=context['t_switch'],B=context['B'],observation_pose=context['R_obs'],
            physical_u_minus=context['pre_switch_command'],first_fresh_command=context['first_fresh_command'],
            next_switch_sim_s=float(window['absolute_times'][-1]),end_reason=window['end_reason'])
        features.append(feature);cases[cid]=(row,context,window,world,old)
    # Seal matching before accessing ANY computed execution outcomes.
    matches=match_controls(features,hard_ids,cfg['matching'])
    save(out/'metrics/matching_inputs.json',features)
    save(out/'metrics/matching_frozen.json',dict(protocol_sha256=sha(out/'protocol.yaml'),
        sealed_before_outcomes=True,prior_published_outcomes_known=True,matches=matches))
    command_config=yaml.safe_load(inputs.record(ROOT/cfg['saved_loss_config']).read_text())
    records=[]
    for f in features:
        row,context,window,world,old=cases[f['case_id']]
        metric=execution_metrics(window,world,old,context,command_config,cfg['attachment'])
        geometry=environment.check_trajectory(window['times'],window['poses'],radius=.2,required_clearance=.05,
            curved_path_error_bound_m=metric['command']['curve_bound_m'])
        if f['end_reason'] != metric['end_reason']:
            raise ValueError('feature/outcome censoring disagreement')
        records.append({**f, **metric, 'environment': geometry})
    hard=[r for r in records if r['case_id'] in hard_ids]
    previous=inputs.read(ROOT/cfg['historical_loss']/'records.json');lookup={r['case_id']:r for r in records}
    for r in previous:
        new=lookup[r['case_id']]
        for name in ('full','common'):
            for key in ('observation_status','join_time_s','position_auc_m_s','yaw_auc_rad_s','observation_horizon_s'):
                if new[name][key]!=r[name][key]:raise ValueError('13-case metric drift: '+r['case_id']+'/'+key)
        if plain(new['environment'])!=r['environment']:raise ValueError('13-case safety drift')
    pairs=[]
    for match in matches:
        if match['control_id'] is None:continue
        a,b=[lookup[match[k]] for k in ('hard_id','control_id')]
        pair=dict(**match,episode_id=a['episode_id'],control_episode_id=b['episode_id'])
        for label,r in [('hard',a),('control',b)]:
            pair[label]=dict(position_auc_m_s=None if r['common'] is None else r['common']['position_auc_m_s'],
                yaw_auc_rad_s=None if r['common'] is None else r['common']['yaw_auc_rad_s'],
                growth_m=r['maximum_growth_m'],join=r['full']['observation_status'],join_time_s=r['full']['join_time_s'],
                progress=r['original_row_progress_gain'],linear_TV_mps=r['common_command']['linear_TV_mps'] if r['common_command'] else None,
                angular_TV_radps=r['common_command']['angular_TV_radps'] if r['common_command'] else None)
        pairs.append(pair)
    safe_raw=[r for r in records if r['raw_full_safe']]
    safe=[r for r in records if r['source_safe']]
    sequence=[r for r in records if r['episode_id'] in hard_episodes]
    pair_deltas=[dict(episode_id=p['episode_id'],position_auc=p['hard']['position_auc_m_s']-p['control']['position_auc_m_s'],
        yaw_auc=p['hard']['yaw_auc_rad_s']-p['control']['yaw_auc_rad_s']) for p in pairs if p['hard']['position_auc_m_s'] is not None and p['control']['position_auc_m_s'] is not None]
    pair_summary=dict(matched=len(pairs),unmatched=len(matches)-len(pairs),unique_controls=len({p['control_id'] for p in pairs}),
        matched_hard_episodes=len({p['episode_id'] for p in pairs}),
        event_mean_delta_position_auc=None if not pair_deltas else float(np.mean([p['position_auc'] for p in pair_deltas])),
        episode_equal_mean_delta_position_auc=None if not pair_deltas else float(np.mean([np.mean([p['position_auc'] for p in pair_deltas if p['episode_id']==ep]) for ep in sorted({p['episode_id'] for p in pair_deltas})])),
        pairs=pairs,interpretation='No causal estimate. Intrinsic turn is a proxy, route intent not independently observed; realized lifetime may be post-treatment; control reuse disclosed.')
    summary=dict(all=summarize_events(records),safe_raw=summarize_events(safe_raw),safe_B_prefix=summarize_events(safe),
        selected13=summarize_events(hard),seven_episodes=summarize_events(sequence),matched=pair_summary,
        all_safe_raw_post_execution_clearance_valid=sum(r['environment']['clearance_valid'] for r in safe_raw),
        selected13_post_execution_clearance_valid=sum(r['environment']['clearance_valid'] for r in hard))
    save(out/'metrics/execution_records.json',records);save(out/'metrics/q2_summary.json',summary)
    save(out/'metrics/seven_episode_sequences.json',sequence)
    csv_save(out/'metrics/event_metrics.csv',[{k:v for k,v in r.items() if k not in ('trace','physical_u_minus','first_fresh_command')} for r in records])
    csv_save(out/'metrics/episode_metrics.csv',[dict(episode_id=ep,**summarize_events([r for r in safe_raw if r['episode_id']==ep])) for ep in sorted({r['episode_id'] for r in safe_raw})])
    return summary,records


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--config',type=Path,default=ROOT/'configs/project_audit_native_lightnav.yaml')
    a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    (out/'metrics').mkdir();(out/'figures').mkdir()
    (out/'protocol.yaml').write_bytes(a.config.read_bytes());cfg=yaml.safe_load(a.config.read_text())
    inputs=Inputs();inputs.record(a.config)
    save(out/'audit_start.json',dict(utc=datetime.now(timezone.utc).isoformat(),HEAD=git('rev-parse','HEAD'),
        origin_main=git('rev-parse','origin/main'),branch=git('branch','--show-current'),status=git('status','--porcelain'),mode='SAVED-ONLY AUDIT'))
    # Record all requested documentary sources and audit code, before computation.
    documents=[ROOT/'AGENTS.md',ROOT/'README.md',ROOT/'docs/WORK_LOG.md',*sorted((ROOT/'docs').glob('*.md'))]
    save(out/'metrics/document_inventory.json',[dict(path=str(inputs.record(p)),sha256=sha(p),lines=len(p.read_text().splitlines())) for p in documents])
    for p in list((ROOT/'src/reconciliation').glob('*.py'))+list((ROOT/'scripts').glob('*.py'))+list((ROOT/'scripts/lightnav').glob('*.py')):
        inputs.record(p)
    for p in [ROOT/'configs/stage0_jackal_controller_validation.yaml',ROOT/'configs/stage0_lightnav_single_chunk.yaml']:
        inputs.record(p)
    print('SAVED-ONLY AUDIT: source and checkpoint hashes',flush=True)
    prov=provenance(cfg,inputs,out)
    env_path=inputs.read(ROOT/cfg['source_scan']/'source.json')['environment_path']
    inputs.tree(Path(env_path)/'geometry/projected')
    environment=HospitalEnvironment.load(env_path)
    decoder=load_decoder(cfg,inputs)
    print('SAVED-ONLY AUDIT: Q1 raw/token/wire/geometry recomputation',flush=True)
    q1_result,raws=q1(cfg,inputs,out,decoder,environment)
    print('SAVED-ONLY AUDIT: 881 source records, sealed matching then execution costs',flush=True)
    summary,records=q2(cfg,inputs,out,environment)
    print('SAVED-ONLY AUDIT: final immutable-input verification',flush=True)
    inputs.verify()
    save(out/'input_manifest.json',dict(source_sha256=inputs.hashes,config_sha256=sha(out/'protocol.yaml'),new_scientific_calls=0))
    save(out/'audit_summary.json',dict(mode='SAVED-ONLY AUDIT',source_SHA=git('rev-parse','HEAD'),q2=summary,
        q1_raw_traces=len(raws),official_provenance_verified=prov['external_status']=='' and all(r['all_historical_match'] for r in prov['checkpoint_files']),
        internal_neural_cause='INTERNAL_CAUSE_UNRESOLVED_FROM_AVAILABLE_OBSERVABLES',new_model_MPC_optimizer_simulator_calls=0))
    save(out/'validation.json',dict(valid=True,source_files=len(inputs.hashes),all_source_hashes_preserved=True,
        raw_traces=len(raws),source_scan_881_literal_reproduction=True,selected_13_metrics_and_safety_literal_reproduction=True,
        outcomes=len(records),matching_sealed_before_outcomes=True,new_scientific_calls=0))
    print(json.dumps(plain({k:v for k,v in summary.items() if k in ('selected13','matched')}),indent=2),flush=True)


if __name__=='__main__':main()

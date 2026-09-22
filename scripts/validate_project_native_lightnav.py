#!/usr/bin/env python3
"""SAVED-ONLY AUDIT integrity/presentation verification; zero scientific calls.

Verifies preserved input bytes, frozen matching, event/episode summaries and
every plotted numeric payload. Output is exclusive and includes validator hash.
Run before the separately documented append-only WORK_LOG completion entry.
"""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.project_audit import match_controls, summarize_events
from report_project_native_lightnav import digest, dump, document_tables, DOCUMENT


def validate(run):
    output=run/'validation_complete.json'
    if output.exists():raise FileExistsError(output)
    checks={};sources={}
    def read(path):
        p=Path(path).resolve();sources[str(p)]=digest(p)
        return json.loads(p.read_text())
    def check(name,value):checks[name]=bool(value)
    manifest=read(run/'input_manifest.json')
    for path,h in manifest['source_sha256'].items():check('source:'+path,digest(path)==h)
    presentation=read(run/'presentation_manifest.json')
    for path,h in presentation['source_sha256'].items():check('presentation_source:'+path,digest(path)==h)
    for path,h in presentation['outputs_sha256'].items():check('presentation_output:'+path,digest(run/path)==h)
    records=read(run/'metrics/execution_records.json');lookup={r['case_id']:r for r in records}
    features=read(run/'metrics/matching_inputs.json');matching=read(run/'metrics/matching_frozen.json')
    import yaml
    protocol=run/'protocol.yaml';sources[str(protocol)]=digest(protocol);cfg=yaml.safe_load(protocol.read_text())
    hard_ids=[m['hard_id'] for m in matching['matches']]
    check('matching_reproduced',match_controls(features,hard_ids,cfg['matching'])==matching['matches'])
    check('matching_config_hash',digest(protocol)==matching['protocol_sha256'])
    check('matching_feature_record_parity',all(all(r[k]==v for k,v in f.items()) for f in features for r in [lookup[f['case_id']]]))
    summary=read(run/'metrics/q2_summary.json');episodes={lookup[c]['episode_id'] for c in hard_ids}
    groups={'all':records,'safe_raw':[r for r in records if r['raw_full_safe']],
        'safe_B_prefix':[r for r in records if r['source_safe']],
        'selected13':[r for r in records if r['case_id'] in hard_ids],
        'seven_episodes':[r for r in records if r['episode_id'] in episodes]}
    for name,rows in groups.items():check('summary:'+name,summarize_events(rows)==summary[name])
    for pair in summary['matched']['pairs']:
        for label,key in [('hard','hard_id'),('control','control_id')]:
            r=lookup[pair[key]];p=pair[label]
            check('pair:'+pair['hard_id']+'/'+label,p['position_auc_m_s']==r['common']['position_auc_m_s'] and
                p['yaw_auc_rad_s']==r['common']['yaw_auc_rad_s'] and p['join']==r['full']['observation_status'])
    tables=document_tables(DOCUMENT.read_text());sources[str(DOCUMENT)]=digest(DOCUMENT)
    for name,rows in tables.items():
        path=run/(name+'.csv');sources[str(path)]=digest(path)
        with path.open() as f:check('table:'+name,list(csv.DictReader(f))==rows)
    q1=read(run/'metrics/q1_recomputed.json')
    for fig in presentation['figures']:
        check('figure_bytes:'+fig['name'],digest(run/fig['png'])==fig['sha256'])
        check('sidecar_bytes:'+fig['name'],digest(run/fig['sidecar'])==fig['sidecar_sha256'])
        side=read(run/fig['sidecar']);n=side['numbers'];name=fig['name']
        check('figure_config:'+name,side['config_sha256']==digest(protocol))
        check('figure_source_inventory:'+name,all(presentation['source_sha256'].get(p)==h for p,h in side['source_sha256'].items()))
        if name=='q1_causal_timeline':
            check('numbers:'+name,[x['record'] for x in n]==q1['JOIN01'])
            for row in n:
                t=row['record']['timestamp_records'];s0=t['t_obs']['sim_time_s'];h0=t['t_obs']['host_monotonic_s']
                check('clocks:'+row['attempt'],row['simulation_elapsed_s']==[0,t['t_ready_seen_sim']['sim_time_s']-s0,t['t_switch']['sim_time_s']-s0] and
                    row['host_elapsed_s']==[t[k]['host_monotonic_s']-h0 for k in ['t_obs','t_request','t_ready_host','t_switch']])
        elif name=='q1_evidence_matrix':check('numbers:'+name,n==tables['q1_root_cause_matrix'])
        elif name=='method_evidence_matrix':check('numbers:'+name,n==tables[name])
        elif name=='representative_attachment':check('numbers:'+name,n==lookup['episode_013_repeat_01/handoff_024'])
        elif name=='matched_turn_controls':check('numbers:'+name,n==summary['matched'])
        elif name=='corpus_covariates':
            for i,p in enumerate(n):
                rows=[lookup[c] for c in p['case_ids']];key=p['covariate']
                check('scatter:'+str(i),p['x']==[abs(r[key]) if key=='lateral_m' else r[key] for r in rows] and
                    p['y']==[r['common']['position_auc_m_s'] if p['metric']=='auc' else r['initial_growth_0_30s_m'] for r in rows] and
                    all(r['raw_full_safe'] and r['full']['join_success']==p['joined'] for r in rows))
        elif name.startswith('sequence_') or name=='hard_five_chunk_sequence':
            check('sequence_records:'+name,all(x['record']==lookup[x['case_id']] and digest(x['world_path'])==x['world_sha256'] for x in n['references']))
            ep=Path(n['execution_source']).parent
            command_path=ep/'commands.csv';sources[str(command_path)]=digest(command_path)
            with command_path.open() as f:commands=list(csv.DictReader(f))
            lo,hi=n['displayed_interval_sim_s']
            check('sequence_commands:'+name,n['commands']==[c for c in commands if lo<=float(c['sim_time_s'])<hi])
        else:raise ValueError('unvalidated figure '+name)
    for name in ['corpus','source02','source03','source04']:
        path=run/'metrics'/f'{name}_validation.json'
        if path.exists():check('family:'+name,read(path)['valid'])
    sources[str(Path(__file__))]=digest(__file__)
    result=dict(valid=all(checks.values()),checks=len(checks),failed=[k for k,v in checks.items() if not v],
        source_files=len(manifest['source_sha256']),figures=len(presentation['figures']),tables=len(tables),
        checked_input_sha256=sources,core_input_manifest_sha256=digest(run/'input_manifest.json'),
        new_model_MPC_optimizer_simulator_calls=0,source_hashes_preserved=True if all(checks.values()) else None)
    dump(output,result)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',type=Path,required=True)
    args=parser.parse_args();result=validate(args.run.resolve())
    print(json.dumps({k:v for k,v in result.items() if k!='checked_input_sha256'}))
    raise SystemExit(0 if result['valid'] else 2)

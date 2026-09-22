#!/usr/bin/env python3
"""SAVED-ONLY AUDIT presentation. No model, MPC, optimizer or simulator calls.

Refuses existing report outputs. Every figure has a numeric/source-hash sidecar.
Historical arrays are read only; no output is written inside historical runs.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import html
import json
from pathlib import Path
import re
import textwrap

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
DOCUMENT = ROOT/'docs/PROJECT_AUDIT_NATIVE_LIGHTNAV_AND_RECONCILIATION.md'


def digest(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def dump(path, value):
    with Path(path).open('x') as f:
        json.dump(value, f, indent=2, allow_nan=False)
        f.write('\n')


def document_tables(text):
    tables = {}
    for name, block in re.findall(r'<!-- AUDIT_TABLE:(\w+) -->\n((?:\|.*\n)+)', text):
        lines = [line.strip().strip('|').split('|') for line in block.splitlines()]
        header = [c.strip() for c in lines[0]]
        rows = [[c.strip() for c in line] for line in lines[2:]]
        if any(len(row) != len(header) for row in rows):
            raise ValueError('malformed table '+name)
        tables[name] = [dict(zip(header, row)) for row in rows]
    required = {'q1_root_cause_matrix', 'q2_contribution_matrix', 'claim_audit',
                'method_evidence_matrix', 'closest_work_matrix', 'historical_modification_matrix'}
    if set(tables) != required:
        raise ValueError('missing/extra audit tables')
    return tables


class Report:
    def __init__(self, run):
        self.run = run
        self.sources = {}
        self.figures = []
        targets = [run/'index.html', run/'integrity_audit.json', run/'presentation_manifest.json',
                   *run.glob('*.csv'), *run.glob('figures/*')]
        if any(p.exists() for p in targets):
            raise FileExistsError('report output exists; use a fresh audit run')
        self.record(Path(__file__))
        self.record(DOCUMENT)
        self.config = yaml.safe_load(self.record(run/'protocol.yaml').read_text())

    def record(self, path):
        p = Path(path).resolve()
        h = digest(p)
        if str(p) in self.sources and self.sources[str(p)] != h:
            raise ValueError('input changed: '+str(p))
        self.sources[str(p)] = h
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

    def figure(self, fig, name, numbers, description):
        image = self.run/'figures'/f'{name}.png'
        side = image.with_suffix('.json')
        if image.exists() or side.exists():
            raise FileExistsError(image)
        fig.savefig(image, dpi=150, facecolor='white')
        plt.close(fig)
        dump(side, dict(mode='SAVED-ONLY AUDIT', description=description, numbers=numbers,
             source_sha256=dict(self.sources), config_sha256=digest(self.run/'protocol.yaml'),
             frame='Isaac world XY metres, yaw CCW; original observation anchoring',
             clocks='simulator seconds and host monotonic seconds never mixed',
             intrinsic_waypoint_dt_s=None, new_scientific_calls=0))
        self.figures.append(dict(name=name, description=description, png=str(image.relative_to(self.run)),
             sha256=digest(image), sidecar=str(side.relative_to(self.run)), sidecar_sha256=digest(side)))

    def table_figure(self, rows, columns, name, title, widths):
        cells = [[textwrap.fill(str(row[k]), widths[i], break_long_words=False)
                  for i, k in enumerate(columns)] for row in rows]
        heights = [max(c.count('\n')+1 for c in row) for row in cells]
        fig, ax = plt.subplots(figsize=(15, max(5, sum(heights)*.23+1.7)))
        ax.axis('off')
        table = ax.table(cellText=cells, colLabels=columns, cellLoc='left', loc='center',
                         colWidths=np.array(widths)/sum(widths))
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        total = sum(heights)+3
        for (row, col), cell in table.get_celld().items():
            cell.set_height((2 if row == 0 else heights[row-1]+.4)/(total+len(rows)*.4))
            cell.set_facecolor('#dce8f1' if row == 0 else ('#f3f6f8' if row%2 else 'white'))
            cell.set_edgecolor('#ced7dd')
        ax.set_title(title, fontsize=14, pad=20)
        fig.tight_layout()
        self.figure(fig, name, rows, title)

    def timeline(self, joins):
        fig, axes = plt.subplots(3, 3, figsize=(15, 11), layout='constrained')
        numbers = []
        for i, row in enumerate(joins):
            t = row['timestamp_records']
            obs = t['t_obs']['sim_time_s']
            sim = [0, t['t_ready_seen_sim']['sim_time_s']-obs, t['t_switch']['sim_time_s']-obs]
            host0 = t['t_obs']['host_monotonic_s']
            host = [t[k]['host_monotonic_s']-host0 for k in ['t_obs','t_request','t_ready_host','t_switch']]
            for ax, values, labels, unit in [(axes[i,0],sim,['reveal / obs','ready / install','B / first command'],'simulation'),
                    (axes[i,1],host,['obs','request','receipt','B application'],'host monotonic')]:
                ax.plot(values, range(len(values)), 'o-', color='#21618c')
                ax.set_yticks(range(len(values)), labels)
                ax.set_xlabel(f'{unit} elapsed from observation [s]')
                ax.grid(alpha=.2)
                ax.set_title(row['attempt']+' — '+unit)
            ax = axes[i,2]
            for key, label, color in [('old_world','OLD','#999999'),('fresh_world','raw FRESH','#c0392b')]:
                path = np.asarray(row[key]);ax.plot(path[:,0],path[:,1],'.-',label=label,color=color)
            obstacle=row['obstacle'];x,y,yaw=obstacle['pose_world'];dx,dy,_=obstacle['dimensions_m']
            corners=np.array([[-dx,-dy],[dx,-dy],[dx,dy],[-dx,dy]])/2
            rotation=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
            ax.add_patch(Polygon(corners@rotation.T+[x,y],color='#34495e',alpha=.6,label='physical box'))
            ax.scatter(*row['observation_pose'][:2],marker='o',facecolors='none',edgecolors='green',s=70,label='observation')
            ax.scatter(*row['B'][:2],marker='x',color='black',s=70,label='B')
            ax.set_aspect('equal',adjustable='datalim');ax.set_xlabel('world x [m]');ax.set_ylabel('world y [m]')
            ax.set_title(f"B clearance {row['B_edge_clearance_m']:.3f} m; raw -0.200 m")
            ax.legend(fontsize=7,loc='best');ax.grid(alpha=.2)
            numbers.append(dict(attempt=row['attempt'],simulation_elapsed_s=sim,host_elapsed_s=host,record=row))
        fig.suptitle('JOIN01: unsafe raw FRESH precedes downstream attachment — clocks kept separate',fontsize=15)
        self.figure(fig,'q1_causal_timeline',numbers,'Three actual online reveals; footprint radius 0.20 m, required edge margin 0.05 m. Box is physical extent, not inflated footprint.')

    def attachment(self, record):
        tr=record['trace'];t=np.array(tr['times_s'])
        fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True,layout='constrained')
        axes[0].plot(t,tr['distance_m'],label='distance to original FRESH')
        axes[0].axhline(.1,color='#c0392b',ls='--',label='0.10 m tube')
        axes[0].set_ylabel('distance [m]');axes[0].legend()
        axes[1].plot(t,np.degrees(tr['yaw_error_rad']),color='#a65b20')
        axes[1].axhline(15,color='#c0392b',ls='--',label='15 degree tube')
        axes[1].set_ylabel('absolute yaw error [deg]');axes[1].set_xlabel('saved simulation time after B [s]')
        for ax in axes:ax.grid(alpha=.2);ax.axvline(t[-1],color='gray',ls=':')
        axes[1].legend();fig.suptitle('013/01/024 — no tube entry during its 1.55 s active lifetime')
        self.figure(fig,'representative_attachment',record,'Actual saved states only; next reference command excluded. Sampled joint tube requires 0.30 s dwell.')

    def sequence(self, episode, records, neighborhood=False):
        ep=ROOT/self.config['corpus']/'episodes'/episode
        states=self.csv(ep/'execution.csv');commands=self.csv(ep/'commands.csv')
        selected=sorted(records,key=lambda x:x['t_switch']['sim_time_s'])
        if neighborhood:
            selected=[x for x in selected if 22<=int(x['case_id'][-3:])<=26]
        lo=selected[0]['t_switch']['sim_time_s'] if neighborhood else float(states[0]['sim_time_s'])
        hi=selected[-1]['next_switch_sim_s']
        ss=[s for s in states if lo<=float(s['sim_time_s'])<=hi]
        uu=[u for u in commands if lo<=float(u['sim_time_s'])<hi]
        fig=plt.figure(figsize=(14,14),layout='constrained');gs=fig.add_gridspec(7,2)
        xy=fig.add_subplot(gs[:3,0]);distance=fig.add_subplot(gs[0,1]);yaw=fig.add_subplot(gs[1,1]);active=fig.add_subplot(gs[2,1])
        velocity=fig.add_subplot(gs[3,:]);omega=fig.add_subplot(gs[4,:]);progress=fig.add_subplot(gs[5,:]);life=fig.add_subplot(gs[6,:])
        palette=plt.get_cmap('tab10');reference_sources=[]
        for i,r in enumerate(selected):
            color=palette(i%10);start=r['t_switch']['sim_time_s'];end=r['next_switch_sim_s']
            wpath=ep/'chunks'/r['reference_id']/'world.npy';w=self.array(wpath)
            tr=r['trace'];t=np.asarray(tr['times_s'])+start;poses=np.asarray(tr['poses_world'])
            xy.plot(w[:,0],w[:,1],color=color,alpha=.5,lw=1,ls='--')
            xy.plot(poses[:,0],poses[:,1],color=color,lw=2,label=r['reference_id'] if neighborhood else None)
            xy.scatter(*r['observation_pose'][:2],facecolors='none',edgecolors=[color],s=24)
            xy.scatter(*r['B'][:2],color=color,marker='x',s=30)
            distance.plot(t,tr['distance_m'],color=color);yaw.plot(t,np.degrees(tr['yaw_error_rad']),color=color)
            progress.plot(t,[p['progress'] for p in tr['projection']],color=color)
            life.plot([start,end],[i,i],lw=3,color=color)
            if neighborhood:life.text(start,i+.12,r['reference_id'],fontsize=9)
            for ax in [distance,yaw,velocity,omega,progress]:ax.axvline(start,color=color,alpha=.25,lw=.7)
            reference_sources.append(dict(case_id=r['case_id'],world_path=str(wpath),world_sha256=digest(wpath),record=r))
        xy.plot([float(s['x']) for s in ss],[float(s['y']) for s in ss],color='black',lw=.6,alpha=.5)
        xy.set_aspect('equal',adjustable='datalim');xy.set_xlabel('world x [m]');xy.set_ylabel('world y [m]')
        xy.set_title('Dashed: original FRESH; solid: saved execution\nopen circle: observation; x: actual B')
        if neighborhood:xy.legend(fontsize=8)
        times=[float(u['sim_time_s']) for u in uu]
        active.step(times,[int(u['chunk_id'].split('_')[-1]) if u['chunk_id'] else -1 for u in uu],where='post',color='black')
        active.set_title('-1 = no active reference',fontsize=8)
        velocity.step(times,[float(u['v_mps']) for u in uu],where='post');omega.step(times,[float(u['omega_radps']) for u in uu],where='post',color='#a65b20')
        distance.axhline(.1,color='gray',ls='--');yaw.axhline(15,color='gray',ls='--')
        for ax,label in [(distance,'FRESH distance [m]'),(yaw,'yaw error [deg]'),(active,'applied chunk index'),(velocity,'applied v [m/s]'),(omega,'applied omega [rad/s]'),(progress,'original-row progress'),(life,'handoff ordinal')]:
            ax.set_ylabel(label);ax.set_xlabel('absolute saved simulation time [s]');ax.grid(alpha=.2);ax.set_xlim(lo,hi)
        title=episode+(' — C025 and two preceding/following references' if neighborhood else ' — complete saved successive sequence')
        fig.suptitle(title,fontsize=15)
        name='hard_five_chunk_sequence' if neighborhood else 'sequence_'+episode
        self.figure(fig,name,dict(references=reference_sources,commands=uu,execution_source=str(ep/'execution.csv'),
              displayed_interval_sim_s=[lo,hi],initial_pre_handoff_metrics='not invented'),title)

    def comparisons(self, records, summary):
        pairs=summary['matched']['pairs'];fig,axes=plt.subplots(1,2,figsize=(14,6),layout='constrained')
        labels=[p['hard_id'].replace('episode_','').replace('_repeat_','/').replace('/handoff_','/') for p in pairs]
        for ax,key,label in [(axes[0],'position_auc_m_s','position AUC [m s]'),(axes[1],'yaw_auc_rad_s','yaw AUC [rad s]')]:
            x=np.arange(len(pairs));ax.bar(x-.18,[p['hard'][key] for p in pairs],width=.36,label='high mismatch',color='#c0392b')
            ax.bar(x+.18,[p['control'][key] for p in pairs],width=.36,label='matched low mismatch',color='#2471a3')
            ax.set_xticks(x,labels,rotation=50,ha='right');ax.set_ylabel(label);ax.grid(axis='y',alpha=.2);ax.legend()
        fig.suptitle('Frozen matching: 8 pairs, 6 controls, 4 hard episodes; 5 unmatched — descriptive, not causal')
        self.figure(fig,'matched_turn_controls',summary['matched'],'First 54 saved intervals; same shape/speed/category/lifetime calipers; controls reused.')
        safe=[r for r in records if r['raw_full_safe'] and r['common'] is not None]
        fig,axes=plt.subplots(2,4,figsize=(18,9),layout='constrained')
        fields=[('lateral_m','absolute lateral mismatch [m]'),('direction_deg','reliable direction mismatch [deg]'),('obs_to_B_travel_m','obs to B travel [m]'),('v_minus_mps','physical v_minus [m/s]')]
        points=[]
        for col,(key,label) in enumerate(fields):
            valid=[r for r in safe if r[key] is not None]
            for joined,color,name in [(True,'#2471a3','sampled attachment'),(False,'#c0392b','no observed sustained attachment')]:
                group=[r for r in valid if r['full']['join_success']==joined];x=[abs(r[key]) if key=='lateral_m' else r[key] for r in group]
                for row,(metric,ylab) in enumerate([('auc','position AUC [m s]'),('growth','initial growth, first 0.30 s [m]')]):
                    y=[r['common']['position_auc_m_s'] if metric=='auc' else r['initial_growth_0_30s_m'] for r in group]
                    axes[row,col].scatter(x,y,s=12,alpha=.6,color=color,label=name)
                    axes[row,col].set_xlabel(label);axes[row,col].set_ylabel(ylab);axes[row,col].grid(alpha=.2)
                    points.append(dict(covariate=key,metric=metric,joined=joined,case_ids=[r['case_id'] for r in group],x=x,y=y))
        axes[0,0].legend(fontsize=7);fig.suptitle('Safe raw FRESH, observed common windows — event scatter, no IID inference')
        self.figure(fig,'corpus_covariates',points,'Null unreliable directions omitted only from that column; color encodes sampled attachment, not collision.')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',type=Path,required=True)
    args=parser.parse_args();run=args.run.resolve();report=Report(run)
    core=report.read(run/'validation.json')
    if not core['valid']:raise ValueError('core audit did not validate')
    q1=report.read(run/'metrics/q1_recomputed.json');records=report.read(run/'metrics/execution_records.json')
    summary=report.read(run/'metrics/q2_summary.json');tables=document_tables(DOCUMENT.read_text())
    for name,rows in tables.items():
        with (run/(name+'.csv')).open('x',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    plt.rcParams.update({'font.size':10,'axes.titleweight':'semibold','axes.spines.top':False,'axes.spines.right':False})
    report.timeline(q1['JOIN01'])
    report.table_figure(tables['q1_root_cause_matrix'],['Candidate explanation','Current status','Isolating evidence / limitation'],
        'q1_evidence_matrix','Q1: source causation and downstream execution are separate',[36,45,65])
    hard=next(r for r in records if r['case_id']=='episode_013_repeat_01/handoff_024')
    report.attachment(hard)
    episodes=sorted({r['episode_id'] for r in records if r['original_candidate']})
    for ep in episodes:report.sequence(ep,[r for r in records if r['episode_id']==ep])
    report.sequence(hard['episode_id'],[r for r in records if r['episode_id']==hard['episode_id']],True)
    report.comparisons(records,summary)
    report.table_figure(tables['method_evidence_matrix'],list(tables['method_evidence_matrix'][0]),
        'method_evidence_matrix','Saved method evidence: no demonstrated moving-hard optimization attachment benefit',[30,70,55,45])
    lookup={r['case_id']:r for r in records};next_rows=[]
    for r in records:
        if not r['original_candidate']:continue
        nid=r['episode_id']+'/handoff_'+str(int(r['case_id'][-3:])+1).zfill(3);n=lookup.get(nid)
        next_rows.append(dict(case_id=r['case_id'],same_pose_reference_jump_m=r['boundary_reference_distance_jump_m'],
             next_case_id=None if n is None else nid,next_B_inside=None if n is None else n['B_inside_joint_tube'],
             next_join=None if n is None else n['full']['join_success'],
             next_initial_distance_m=None if n is None else n['trace']['distance_m'][0]))
    dump(run/'metrics/next_chunk_summary.json',next_rows)
    safe=[r for r in records if r['raw_full_safe']]
    dump(run/'metrics/additional_descriptive_summary.json',dict(growth_quantiles_m=dict(zip(['min','median','p90','p95','max'],
        np.quantile([r['initial_growth_0_30s_m'] for r in safe],[0,.5,.9,.95,1]).tolist())),
        safe_raw_execution_exceptions=[dict(case_id=r['case_id'],B_safe=r['B_safe'],environment=r['environment']) for r in safe if not r['environment']['clearance_valid']],
        no_statistical_inference=True))
    integrity=dict(mode='SAVED-ONLY AUDIT',new_scientific_calls=0,core_validation=core,
        verdict='PROMISING_BUT_UNPROVEN',raw_and_derived_separate=True,original_arrays_read_only=True,
        prior_outcomes_known=True,matching_before_new_outcomes=True,matching_is_retrospective_not_randomized=True,
        outcome_based_matching=False,controls_reused=True,episode_dependence=True,held_out_evidence=False,
        thresholds_relaxed=False,rejected_candidates_preserved=True,unsafe_rollout_fabricated=False,
        waypoint_intrinsic_dt_s=None,B_reanchoring=False,full_native_deployment_equivalence=False,
        historical_generation_environment_complete=False,internal_neural_cause_unresolved=True,
        concerns_document=str(DOCUMENT),core_input_manifest_sha256=digest(run/'input_manifest.json'),
        expected_later_document_edit='WORK_LOG append; core document hashes describe the pre-audit source snapshot',
        incomplete_prior_audits_preserved=['audit_20260922T_project01','audit_20260922T_project01_v2'])
    for name in ['corpus','source02','source03','source04']:
        path=run/'metrics'/f'{name}_validation.json'
        if path.exists():
            result=report.read(path)
            if not result['valid']:raise ValueError('family validator failure '+name)
            integrity[name+'_validation']=dict(valid=True,checks=len(result.get('checks',{})),
                valid_handoffs=result.get('valid_handoff_count'),sha256=digest(path))
    dump(run/'integrity_audit.json',integrity)
    sections=['<!doctype html><html lang="en"><meta charset="utf-8"><title>PROJECT-AUDIT-01</title>',
        '<style>body{font:16px system-ui;max-width:1450px;margin:40px auto;padding:0 24px;color:#172a3a}img{max-width:100%;border:1px solid #ddd}table{border-collapse:collapse;font-size:13px}th,td{border:1px solid #ddd;padding:8px;vertical-align:top}h2{margin-top:48px}</style>',
        '<h1>PROJECT-AUDIT-01 — SAVED-ONLY AUDIT</h1><p><strong>PROMISING_BUT_UNPROVEN.</strong> Measurable minority transition costs; most safe native events attach; no demonstrated optimization attachment benefit.</p>',
        '<p>Zero new model/MPC/optimizer/simulator calls. Original FRESH at observation anchoring. No real robot claim. Numeric sidecars and source hashes accompany every image.</p>',
        '<p><a href="../../../docs/PROJECT_AUDIT_NATIVE_LIGHTNAV_AND_RECONCILIATION.md">Full report</a> · <a href="audit_summary.json">Summary</a> · <a href="integrity_audit.json">Integrity</a> · <a href="validation.json">Core validation</a> · <a href="metrics/event_metrics.csv">881 event metrics</a></p>']
    for fig in report.figures:
        sections.append(f'<h2>{html.escape(fig["name"])}</h2><p>{html.escape(fig["description"])}</p><a href="{fig["sidecar"]}">Numeric/source sidecar</a><br><img src="{fig["png"]}" alt="{html.escape(fig["description"])}">')
    for name,rows in tables.items():
        sections.append(f'<h2>{name}</h2><a href="{name}.csv">CSV</a><table><tr>'+''.join('<th>'+html.escape(k)+'</th>' for k in rows[0])+'</tr>')
        sections.extend('<tr>'+''.join('<td>'+html.escape(v)+'</td>' for v in row.values())+'</tr>' for row in rows)
        sections.append('</table>')
    sections.append('</html>')
    with (run/'index.html').open('x') as f:f.write('\n'.join(sections))
    for path,h in report.sources.items():
        if digest(path)!=h:raise ValueError('input changed during report '+path)
    dump(run/'presentation_manifest.json',dict(valid=True,figures=report.figures,source_sha256=report.sources,
        outputs_sha256={str(p.relative_to(run)):digest(p) for p in [run/'index.html',run/'integrity_audit.json',*sorted(run.glob('*.csv'))]},new_scientific_calls=0))
    print(json.dumps(dict(valid=True,figures=len(report.figures),tables=len(tables),new_scientific_calls=0)))


if __name__=='__main__':main()

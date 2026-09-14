#!/usr/bin/env python3
"""Freeze the bank or summarize every collected episode without selection gates."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import robotless_screening_artifacts as artifacts
from reconciliation.robotless_handoff_screening import CATEGORIES, METRICS, MOTION, select_representatives, summarize
from reconciliation.robotless_projection_handoff import csv_row
from reconciliation.robotless_single_chunk import save_json_exclusive, sha256_file

PLOT_LABELS = {
    'e_perp_m':('cross_track','Cross-track distance (m)'),
    'abs_e_dir_deg':('direction','Absolute tangent-direction mismatch (deg)'),
    'abs_e_yaw_deg':('yaw','Absolute pose-yaw mismatch (deg)'),
    'normalized_progress':('progress','Normalized FRESH arc-length progress'),
}
PLOT_NAMES = ['distribution_'+PLOT_LABELS[k][0]+'.png' for k in METRICS] + [
    'scatter_cross_track_direction.png','scatter_cross_track_yaw.png','pair_multiplicity.png']
COLORS = dict(zip(CATEGORIES,['#1f77b4','#2ca02c','#ff7f0e','#9467bd','#d62728'],strict=True))


def write_csv(path, rows):
    with path.open('x',newline='') as stream:
        if not rows:
            stream.write('episode_id,tau_s\n')
            return
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader()
        writer.writerows([{key:json.dumps(v,sort_keys=True) if isinstance(v,(dict,list)) else v for key,v in row.items()} for row in rows])


def make_plots(run: Path, rows, summary):
    output=run/'evidence';output.mkdir(exist_ok=True)
    diversity=summary['diversity']; counts=f"Attempted {diversity['attempted']} | valid {diversity['valid']} | invalid {diversity['invalid']} | unique raw pairs {diversity['unique_ordered_pair_count']}"
    records=[]
    def finish(fig,name,plotted):
        path=output/name
        with path.open('xb') as stream: fig.savefig(stream,format='png',dpi=160)
        plt.close(fig)
        records.append({**artifacts.file_record(path,run),'plotted_points':plotted})
    for metric,(short,label) in PLOT_LABELS.items():
        fig,ax=plt.subplots(figsize=(9,5.2),layout='constrained')
        plotted=[]
        for category in CATEGORIES:
            subset=[r for r in rows if r['category']==category and r[metric] is not None]
            x=[r['tau_s']+.05*(int(r['episode_id'].split('_')[-1])/29-.5) for r in subset]
            ax.scatter(x,[r[metric] for r in subset],s=24,alpha=.8,color=COLORS[category],label=category,zorder=3)
            plotted.extend({'episode_id':r['episode_id'],'tau_s':r['tau_s'],'value':r[metric],'display_x':xp,'pair_sha256':r['pair_sha256']} for r,xp in zip(subset,x,strict=True))
        for tau in MOTION['tau_s']:
            values=[r[metric] for r in rows if r['tau_s']==tau and r[metric] is not None]
            if values:
                ax.plot([tau-.035,tau+.035],[np.median(values)]*2,color='black',lw=2,zorder=4)
        ax.set(xlabel='Controlled delay tau (s)',ylabel=label,xticks=MOTION['tau_s'],xlim=(-.07,1.07))
        ax.set_title('Frozen Hospital screening: episode points and median\n'+counts,fontsize=11)
        ax.grid(alpha=.2);ax.spines[['top','right']].set_visible(False)
        ax.legend(fontsize=8,ncols=3,loc='best')
        fig.supxlabel('Horizontal offsets separate episodes visually; every condition uses the labeled controlled delay.',fontsize=8)
        finish(fig,'distribution_'+short+'.png',plotted)
    tau_rows=[r for r in rows if r['tau_s']==1.]
    for metric,short in [('abs_e_dir_deg','direction'),('abs_e_yaw_deg','yaw')]:
        fig,ax=plt.subplots(figsize=(8,5.5),layout='constrained');plotted=[]
        for category in CATEGORIES:
            subset=[r for r in tau_rows if r['category']==category and r[metric] is not None]
            ax.scatter([r['e_perp_m'] for r in subset],[r[metric] for r in subset],s=44,
                color=COLORS[category],alpha=.65,edgecolors='white',linewidths=.5,label=category)
            plotted.extend({'episode_id':r['episode_id'],'tau_s':1.,'x':r['e_perp_m'],'y':r[metric],'pair_sha256':r['pair_sha256']} for r in subset)
        ax.set(xlabel='Cross-track distance at controlled delay tau=1 s (m)',ylabel=PLOT_LABELS[metric][1])
        ax.set_title('Frozen Hospital screening: exact episode coordinates\n'+counts,fontsize=11)
        ax.legend(fontsize=8);ax.grid(alpha=.2);ax.spines[['top','right']].set_visible(False)
        fig.supxlabel('One point per defined episode; coincident points are retained. Pair membership is in unique_pairs.csv.',fontsize=8)
        finish(fig,'scatter_cross_track_'+short+'.png',plotted)
    fig,ax=plt.subplots(figsize=(10,4.6),layout='constrained')
    groups=sorted(diversity['pair_groups'].items(),key=lambda item:(-len(item[1]),item[0]))
    labels=[f'P{i+1:02d}' for i in range(len(groups))]
    ax.bar(labels,[len(ids) for _,ids in groups],color='#576b8a')
    ax.set(xlabel='Ordered raw pair group (same grouping at every controlled delay)',ylabel='Episode count',
        title='Exact raw pair multiplicity\n'+counts)
    ax.tick_params(axis='x',labelrotation=60,labelsize=8)
    ax.spines[['top','right']].set_visible(False)
    finish(fig,'pair_multiplicity.png',[{'label':label,'pair_sha256':pair,'episode_ids':ids,'count':len(ids)} for label,(pair,ids) in zip(labels,groups,strict=True)])
    save_json_exclusive(run/'plot_manifest.json',{'plots':records,'matplotlib_version':matplotlib.__version__,
        'metrics_input':artifacts.file_record(run/'aggregate/transitions.json',run),
        'configuration':artifacts.file_record(run/'config_snapshot.yaml',run),
        'display_convention':'distribution X offsets only; scatter metric coordinates unchanged; no classification thresholds'})


def analyze(run: Path):
    config,manifest,manifest_hash=artifacts.load_frozen(run)
    if (run/'aggregate').exists(): raise FileExistsError('aggregate already exists; inputs/evidence are immutable')
    batch=artifacts.read_json(run/'inference_batch.json')
    if batch['manifest_sha256']!=manifest_hash: raise ValueError('batch manifest mismatch')
    statuses=[];rows=[];inputs=[]
    for episode in manifest['episodes']:
        location=run/'episodes'/episode['episode_id']
        status=artifacts.read_json(location/'validation.json')
        if status['episode_id']!=episode['episode_id'] or status['category']!=episode['category']:
            raise ValueError('episode was replaced')
        for record in status['files']: artifacts.verify_file(location,record)
        statuses.append(status);inputs.append(artifacts.file_record(location/'validation.json',run))
        if status['status']=='VALID_PAIR':
            metrics=artifacts.read_json(location/'derived/projection_metrics.json')
            if metrics['manifest_sha256']!=manifest_hash: raise ValueError('metrics manifest mismatch')
            rows.extend(metrics['conditions'])
    summary=summarize(rows,statuses)
    directory=run/'aggregate';directory.mkdir()
    provenance={'manifest_sha256':manifest_hash,'input_episode_validations':inputs,
        'configuration':artifacts.file_record(run/'config_snapshot.yaml',run),
        'processing_source_sha256':artifacts.phase_sources(['scripts/screen_robotless_handoffs.py'])}
    save_json_exclusive(directory/'transitions.json',{'conditions':rows,**provenance})
    write_csv(directory/'transitions.csv',[csv_row(row) for row in rows])
    save_json_exclusive(directory/'diversity.json',summary['diversity'])
    save_json_exclusive(directory/'statistics.json',summary['statistics'])
    save_json_exclusive(directory/'unique_pairs.json',{'groups':summary['unique_pairs']})
    write_csv(directory/'unique_pairs.csv',summary['unique_pairs'])
    write_csv(directory/'episode_statuses.csv',[{key:s.get(key) for key in ('episode_id','category','status','reason','inference_attempted')} for s in statuses])
    save_json_exclusive(directory/'representatives.json',{'rules':select_representatives(rows),
        'metrics_input':artifacts.file_record(directory/'transitions.json',run),'selection_is_visualization_only':True})
    make_plots(run,rows,summary)
    save_json_exclusive(run/'analysis_manifest.json',{'provenance':provenance,
        'files':[artifacts.file_record(p,run) for p in sorted(directory.iterdir()) if p.is_file()],
        'plot_manifest':artifacts.file_record(run/'plot_manifest.json',run)})
    print(json.dumps(summary['diversity'],indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['freeze','analyze'])
    parser.add_argument('run_directory',type=Path)
    parser.add_argument('--config',type=Path,default=artifacts.ROOT/'configs/robotless_handoff_screening.yaml')
    args=parser.parse_args()
    if args.mode=='freeze':
        artifacts.freeze_bank(args.config.resolve(),args.run_directory.resolve())
        print(sha256_file(args.run_directory/'manifest_frozen.json'))
    else: analyze(args.run_directory.resolve())

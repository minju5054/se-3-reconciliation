#!/usr/bin/env python3
"""One immutable frozen-data analysis and plots; no model inference."""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import robotless_old_conditioned_artifacts as a
from reconciliation.robotless_old_conditioned_handoff import summarize, select_representatives
from reconciliation.robotless_single_chunk import save_json_exclusive, make_observation_time
from screen_robotless_handoffs import write_csv, COLORS

PLOTS=['straight_vs_old_cross_track.png','straight_vs_old_direction.png','straight_vs_old_yaw.png',
    'local_vs_window_direction.png','observation_to_old_distance.png']


def make_plots(run,episodes):
    rows=[r for e in episodes for r in e['conditions'] if r['tau_s']==1.]
    records=[]
    specifications=[('e_perp_straight_m','e_perp_old_m','Cross-track distance (m)','Straight','OLD-conditioned'),
        ('abs_e_dir_straight_deg','abs_e_dir_old_deg','Absolute local direction residual (deg)','Straight','OLD-conditioned'),
        ('abs_e_yaw_straight_deg','abs_e_yaw_old_deg','Absolute pose-yaw residual (deg)','Straight','OLD-conditioned'),
        ('abs_e_dir_old_deg','abs_e_dir_window_deg','Absolute direction residual (deg)','Local segment','0.10 m window')]
    for index,(left,right,label,lname,rname) in enumerate(specifications):
        fig,ax=plt.subplots(figsize=(12,5.5),layout='constrained');points=[]
        for i,row in enumerate(rows):
            values=[row[left],row[right]]
            if all(v is not None for v in values):ax.plot([i-.16,i+.16],values,color=COLORS[row['category']],alpha=.7,lw=1)
            for shift,key,marker in [(-.16,left,'o'),(.16,right,'x')]:
                if row[key] is not None:ax.scatter(i+shift,row[key],marker=marker,s=28,color=COLORS[row['category']],zorder=3)
                else:ax.text(i,1.01,'N/A',transform=ax.get_xaxis_transform(),rotation=90,fontsize=6)
                points.append({'episode_id':row['episode_id'],'metric':key,'value':row[key],'display_x':i+shift})
        ax.set(xlabel='Episode ID suffix (all 30 retained)',ylabel=label,xticks=range(30),
            xticklabels=[r['episode_id'][-3:] for r in rows],xlim=(-.6,29.6),
            title=f'{lname} (circle) to {rname} (cross) — controlled delay = 1 s')
        ax.tick_params(axis='x',rotation=60,labelsize=8);ax.grid(alpha=.2);ax.spines[['top','right']].set_visible(False)
        fig.supxlabel('Each episode has a separate position; paired lines compare metrics, not paths or execution.',fontsize=9)
        path=run/'evidence'/PLOTS[index]
        with path.open('xb') as stream:fig.savefig(stream,format='png',dpi=160)
        plt.close(fig);records.append({**a.file_record(path,run),'points':points})
    fig,axes=plt.subplots(1,2,figsize=(12,4.8),layout='constrained');points=[]
    values=[e['reference']['d_obs_to_old_m'] for e in episodes]
    for i,e in enumerate(episodes):
        axes[0].scatter(i,values[i],color=COLORS[e['category']],s=24)
        points.append({'episode_id':e['episode_id'],'d_obs_to_old_m':values[i]})
    axes[0].set(xlabel='Episode ID suffix',ylabel='Observation-to-OLD distance (m)',xticks=range(0,30,3),title='All raw anchor distances')
    axes[1].plot(sorted(values),[(i+1)/len(values) for i in range(len(values))],marker='.',drawstyle='steps-post')
    axes[1].set(xlabel='Observation-to-OLD distance (m)',ylabel='Fraction of this fixed bank',title='Empirical distribution (30 episodes)')
    for ax in axes:ax.grid(alpha=.2);ax.spines[['top','right']].set_visible(False)
    fig.suptitle('Projection anchor diagnostic — no episode removal or distance threshold')
    path=run/'evidence'/PLOTS[4]
    with path.open('xb') as stream:fig.savefig(stream,format='png',dpi=160)
    plt.close(fig);records.append({**a.file_record(path,run),'points':points})
    save_json_exclusive(run/'plot_manifest.json',{'plots':records,'metrics_input':a.file_record(run/'derived/episode_metrics.json',run),
        'configuration':a.file_record(run/'config_snapshot.yaml',run),'matplotlib_version':matplotlib.__version__,
        'display_convention':'episode-ID separation preserves every value; no metric jitter, thresholds or scores'})


def analyze(run,config_path):
    a.initialize(config_path,run)
    config,record,source,manifest=a.load_inputs(run)
    episodes=a.derive(config,source,manifest);summary=summarize(episodes)
    rows=[r for e in episodes for r in e['conditions']]
    save_json_exclusive(run/'derived/episode_metrics.json',{'episodes':episodes,
        'source_json_sha256':a.sha256_file(run/'source.json'),'config_sha256':a.sha256_file(run/'config_snapshot.yaml'),
        'created_time':make_observation_time(0),'processing_source_sha256':a.processing_sources()})
    write_csv(run/'derived/episode_metrics.csv',rows)
    write_csv(run/'derived/anchor_diagnostics.csv',[{'episode_id':e['episode_id'],'category':e['category'],**e['reference']} for e in episodes])
    save_json_exclusive(run/'derived/statistics.json',summary['statistics'])
    save_json_exclusive(run/'derived/summary.json',{k:v for k,v in summary.items() if k not in ('statistics','unique_pairs')})
    save_json_exclusive(run/'derived/unique_pairs.json',{'groups':summary['unique_pairs']})
    write_csv(run/'derived/unique_pairs.csv',summary['unique_pairs'])
    save_json_exclusive(run/'derived/representatives.json',{'rules':select_representatives(episodes),'selection_scope':'visualization only'})
    write_csv(run/'derived/critical_cases.csv',[{**r,'d_obs_to_old_m':e['reference']['d_obs_to_old_m']} for e in episodes
        for r in e['conditions'] if r['tau_s']==1 and r['episode_id'] in ('episode_006','episode_016','episode_027')])
    make_plots(run,episodes)
    a.load_inputs(run)
    save_json_exclusive(run/'analysis_manifest.json',{'derived_files':[a.file_record(p,run) for p in sorted((run/'derived').iterdir())],
        'plot_manifest':a.file_record(run/'plot_manifest.json',run),'source':a.file_record(run/'source.json',run),
        'created_time':make_observation_time(0),'new_lightnav_inference_count':0,'fresh_reanchored':False})


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory',type=Path)
    parser.add_argument('--config',type=Path,default=a.ROOT/'configs/robotless_old_conditioned_handoff.yaml')
    args=parser.parse_args();analyze(args.run_directory.resolve(),args.config.resolve())

#!/usr/bin/env python3
"""Expose coincident raw episode values without moving their metric coordinates."""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import robotless_screening_artifacts as artifacts
from reconciliation.robotless_handoff_screening import CATEGORIES, METRICS, MOTION
from reconciliation.robotless_single_chunk import save_json_exclusive
from screen_robotless_handoffs import COLORS, PLOT_LABELS


def make_episode_panels(run):
    _,_,manifest_hash=artifacts.load_frozen(run)
    rows=artifacts.read_json(run/'aggregate/transitions.json')['conditions']
    diversity=artifacts.read_json(run/'aggregate/diversity.json')
    fig,axes=plt.subplots(4,4,figsize=(16,12),sharey='row',layout='constrained')
    plotted=[]
    for i,metric in enumerate(METRICS):
        for j,tau in enumerate(MOTION['tau_s']):
            ax=axes[i,j]
            for category in CATEGORIES:
                subset=[r for r in rows if r['tau_s']==tau and r['category']==category and r[metric] is not None]
                ax.scatter([int(r['episode_id'].split('_')[-1]) for r in subset],
                    [r[metric] for r in subset],s=13,color=COLORS[category],label=category,zorder=3)
                plotted.extend({'episode_id':r['episode_id'],'tau_s':tau,'metric':metric,'value':r[metric]} for r in subset)
            ax.set(xlim=(-1,30),xticks=[0,5,10,15,20,25,29],xlabel='Episode ID suffix',
                title=f'Controlled delay = {tau:g} s')
            if j==0: ax.set_ylabel(PLOT_LABELS[metric][1],fontsize=9)
            ax.grid(alpha=.2);ax.spines[['top','right']].set_visible(False)
    handles,labels=axes[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='outside lower center',ncols=5)
    fig.suptitle(f"All raw episode values: attempted {diversity['attempted']}, valid {diversity['valid']}, invalid {diversity['invalid']}; "
        f"{diversity['unique_ordered_pair_count']} exact raw pairs\nSame metric scale across controlled delays; episode positions prevent overlap",fontsize=13)
    path=run/'evidence/episode_metrics_by_id.png'
    with path.open('xb') as stream: fig.savefig(stream,format='png',dpi=160)
    plt.close(fig)
    save_json_exclusive(run/'episode_points_manifest.json',{'manifest_sha256':manifest_hash,
        'metrics_input':artifacts.file_record(run/'aggregate/transitions.json',run),
        'configuration':artifacts.file_record(run/'config_snapshot.yaml',run),
        'image':artifacts.file_record(path,run),'plotted_points':plotted,
        'processing_source_sha256':artifacts.phase_sources(['scripts/plot_robotless_screening_episode_points.py']),
        'display_convention':'one distinct episode ID position per panel; metric values unchanged; no thresholds'})


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory',type=Path)
    make_episode_panels(parser.parse_args().run_directory.resolve())

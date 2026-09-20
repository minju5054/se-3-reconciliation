#!/usr/bin/env python3
"""Readable copies of five tracking-distance panels, using saved sidecars only."""
import argparse
import os
from pathlib import Path
import sys
os.environ.setdefault('MPLCONFIGDIR','/tmp/gp_diag08_readable')
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from run_gp_se2_ref01 import read,write,digest


def render(run):
    folder=run/'presentation/tracking_panels';folder.mkdir(exist_ok=False);outputs=[]
    for row in read(run/'comparison_records.json'):
        source=run/'plots'/row['reference_id']/'endpoint_tracking_error.json';side=read(source)
        fig,ax=plt.subplots(figsize=(10,6))
        for i,grid in enumerate(['G3','G4']):
            geometry=side['numeric']['variants'][grid]['geometry']
            if geometry is None:ax.text(i,.002,'N/A\nnot executed',ha='center')
            else:
                value=geometry['tracking_displacement_m'];ax.bar(i,value,width=.5,color=['#3578b5','#d66e20'][i])
                ax.annotate(f'{value:.9f} m',(i,value),xytext=(0,8),textcoords='offset points',ha='center')
        ax.axhline(.04,color='green',ls=':',label='0.04 m: prior observed scale used to choose reserve\nThis is not a tracker constraint or guarantee.')
        ax.set(xlim=(-.6,1.6),ylim=(0,.055),xticks=[0,1],xticklabels=['G3 historical','G4 new'],ylabel='final execution-to-plan endpoint distance [m]')
        ax.grid(axis='y',alpha=.2);ax.legend(loc='upper center',fontsize=9,frameon=False)
        fig.suptitle('Endpoint tracking displacement — '+row['reference_id'],fontsize=13)
        fig.text(.12,.02,'OFFLINE COUNTERFACTUAL | same values as original PNG; no new trajectory',fontsize=9)
        fig.subplots_adjust(top=.85,bottom=.15,left=.12,right=.96)
        path=folder/(row['reference_id']+'.png');fig.savefig(path,dpi=140);plt.close(fig)
        write(path.with_suffix('.json'),dict(numeric=side['numeric'],source_sidecar=str(source),source_sha256=digest(source),image_sha256=digest(path)))
        outputs.append(path)
    with (folder/'index.html').open('x') as f:
        f.write('<!doctype html><meta charset="utf-8"><h1>Endpoint tracking: readable copies</h1><p>Only legend layout changed. Original figures and sidecars are preserved.</p>')
        for p in outputs:f.write(f'<h2>{p.stem}</h2><img style="max-width:1100px;width:100%" src="{p.name}">')
    write(folder/'validation.json',dict(valid=all(read(p.with_suffix('.json'))['numeric']==read(Path(read(p.with_suffix('.json'))['source_sidecar']))['numeric'] for p in outputs),
        images=len(outputs),new_runtime=0,source_script_sha256=digest(__file__)))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',required=True,type=Path);render(p.parse_args().run.resolve())

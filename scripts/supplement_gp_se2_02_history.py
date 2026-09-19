#!/usr/bin/env python3
"""Report-only full-time views of frozen GP-SE2-02 feasible-objective histories.

Original figures, solver results, selection, acceptance, and all numeric history
values remain unchanged. Explicit common axes expose the pre-feasible N/A span
which matplotlib autoscaling cropped in the original figure.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from plot_gp_se2_02 import GP_METHODS,SEEDS,COLORS,SHORT,read,write,file_sha256

FILENAME='feasible_objective_history_full_time'


def payload(case):
    case=Path(case)
    original=case/'plots/feasible_objective_history.json'
    sidecar=read(original)
    image=original.with_suffix('.png')
    if file_sha256(image)!=sidecar['image_sha256']:
        raise ValueError('original history image hash changed')
    for path,digest in sidecar['source_hashes'].items():
        if file_sha256(path)!=digest:
            raise ValueError('frozen history source changed: '+path)
    history=sidecar['numeric_data']
    expected={method+'/'+seed for method in GP_METHODS for seed in SEEDS}
    if set(history)!=expected:raise ValueError('four planned GP starts are required')
    ends={key:float(read(case/'methods'/key.split('/')[0]/'starts'/key.split('/')[1]/'solver_result.json')['solve_wall_time_s']) for key in expected}
    if any(not np.isfinite(value) or value<0 for value in ends.values()):raise ValueError('invalid prepared solve duration')
    finite=[]
    for key,record in history.items():
        times=[float(p['discovery_time_s']) for p in record['points']]
        if any(t<0 or t>ends[key]+1e-8 for t in times):raise ValueError('discovery time outside prepared solve')
        if times!=sorted(times):raise ValueError('history must retain chronological ordering')
        finite.extend(float(p['best_full_objective']) for p in record['points'] if p['best_full_objective'] is not None)
        if record['seed_full_feasible']:finite.append(float(record['seed_objective']))
    if any(not np.isfinite(value) or value<0 for value in finite):raise ValueError('nonnegative original GP objective expected')
    maximum_time=max(ends.values())
    xlim=[0.,maximum_time if maximum_time>0 else 1.]
    ylim=[0.,max(finite)*1.05 if finite and max(finite)>0 else 1.]
    return dict(numeric_data=history,prepared_solve_end_s=ends,axes=dict(xlim_s=xlim,ylim_original_objective=ylim,
        common_across_all_four_panels=True,zero_time_fallback_display_span=maximum_time==0,
        empty_objective_display_range_only=not finite),
        original_sidecar=str(original.resolve()),original_sidecar_sha256=file_sha256(original),
        original_image_sha256=file_sha256(image),frozen_source_hashes=sidecar['source_hashes'])


def render(case):
    case=Path(case);data=payload(case);target=case/'plots'/f'{FILENAME}.png'
    if target.exists() or target.with_suffix('.json').exists():raise FileExistsError(target)
    figure,axes=plt.subplots(2,2,figsize=(13,9),sharex=True,sharey=True)
    for i,method in enumerate(GP_METHODS):
        for j,seed in enumerate(SEEDS):
            key=method+'/'+seed;axis=axes[i,j];record=data['numeric_data'][key]
            t=np.array([p['discovery_time_s'] for p in record['points']],dtype=float)
            y=np.array([np.nan if p['best_full_objective'] is None else p['best_full_objective'] for p in record['points']],dtype=float)
            valid=np.isfinite(y)
            if np.any(valid):
                first=float(t[np.flatnonzero(valid)[0]])
                axis.step(t,y,where='post',color=COLORS[method],lw=1.8,label='post-certified full-feasible best')
                axis.scatter(t[valid],y[valid],s=12,color=COLORS[method],zorder=4)
                if first>0:
                    axis.axvspan(0,first,color='#b6bcc5',alpha=.12)
                    axis.text(.025,.93,f'N/A before {first:.3f} s',transform=axis.transAxes,fontsize=9)
            else:
                axis.axvspan(0,data['prepared_solve_end_s'][key],color='#b6bcc5',alpha=.12)
                axis.text(.5,.5,'N/A: no full-feasible candidate',ha='center',transform=axis.transAxes,fontsize=10)
            if record['seed_full_feasible']:
                axis.axhline(record['seed_objective'],color='#555555',ls=':',lw=1.1,label='known-feasible seed objective')
            axis.axvline(data['prepared_solve_end_s'][key],color='#888888',ls='--',lw=.8,label='this start solve end')
            axis.set(xlim=data['axes']['xlim_s'],ylim=data['axes']['ylim_original_objective'],
                     title=SHORT[method]+' / '+seed,xlabel='discovery prepared-solve wall time [s]',ylabel='original GP objective')
            axis.grid(alpha=.18);axis.legend(fontsize=7,loc='lower left')
    figure.suptitle(case.name.replace('__','/')+'\nFull-time common-axis supplement; full feasibility certified after solving',fontsize=12)
    figure.text(.015,.012,'Original 48 figures unchanged. Null intervals remain N/A. Equal objective scales include feasible seed baselines; no objective rescaling.',fontsize=8,color='#555555')
    figure.tight_layout(rect=(0,.04,1,.93));figure.savefig(target,dpi=160);plt.close(figure)
    result=dict(**data,image=target.name,image_sha256=file_sha256(target),script_sha256=file_sha256(__file__),dpi=160,
        reporting_only=True,original_autoscale_limitation='Original figure autoscaled finite values and could crop pre-first-feasible N/A times or show different default empty-panel spans.',
        purpose='Expose the complete measured prepared-solve interval with the same X and Y limits for all four starts within each case.',
        numerical_values_changed=False,original_plots_changed=False,solver_rerun=False,acceptance_or_selection_changed=False)
    write(target.with_suffix('.json'),result)
    return dict(path=str(target),sha256=file_sha256(target),sidecar_sha256=file_sha256(target.with_suffix('.json')))


def main(run):
    run=Path(run).resolve();manifest=run/'history_supplement_manifest.json'
    if manifest.exists():raise FileExistsError(manifest)
    cases=[run/'cases'/row['case_directory'] for row in read(run/'case_manifest.json')['selected']]
    for case in cases:
        payload(case)
        if (case/'plots'/f'{FILENAME}.png').exists() or (case/'plots'/f'{FILENAME}.json').exists():raise FileExistsError(case)
    images=[render(case) for case in cases]
    for image in images:image['path']=str(Path(image['path']).relative_to(run))
    result=dict(image_count=len(images),images=images,reporting_only=True,script_sha256=file_sha256(__file__),
        original_plot_manifest_sha256=file_sha256(run/'plot_manifest.json'),original_48_plots_unchanged=True,
        numerical_source_changed=False,solver_rerun=False,reason='Display full pre-feasibility N/A intervals and common axes, retaining original autoscaled figures and data.')
    write(manifest,result);return result

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',required=True,type=Path)
    print(json.dumps(main(parser.parse_args().run),indent=2))

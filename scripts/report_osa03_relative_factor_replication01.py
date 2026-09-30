#!/usr/bin/env python3
"""R01 saved-only scientific figures, paired table and local review GUI."""
import argparse
import csv
import html
import json
import zipfile
from pathlib import Path
import numpy as np
from run_osa03_relative_factor_replication01 import ROOT, prior, plain, read, save, sha, ORDER, LABEL
from analyze_join_online02 import csvread, pose_rows
from report_osa03_relative_factor_ablation01 import write_csv
from validate_robotless_online_handoffs import equal_record


def plot_data(run):
    refs = read(run/'references.json'); c = read(run/'common_state.json'); m = read(run/'source_manifest.json')
    source = Path(m['source']); data = dict(A=c['fresh_capture_pose'], B=c['B'],
        cart_wkb_hex=prior.geometry(source)['cart'].wkb_hex,
        old_prefix=pose_rows(csvread(source/'episodes/REPEAT_01/execution.csv'))[:c['B_tick']+1].tolist(),
        methods={}, summary=read(run/'summary.json'))
    for name, ref in refs.items():
        out = run/'methods'/name
        data['methods'][name] = dict(reference=None if 'world_path' not in ref else np.load(ref['world_path']).tolist(),
            metrics=read(out/'metrics.json') if (out/'metrics.json').exists() else None,
            rollout=read(out/'rollout.json') if (out/'rollout.json').exists() else None)
    return plain(data)


def report(run):
    from validate_osa03_relative_factor_replication01 import validate
    summary, audit = validate(run); assert audit['valid']; equal_record(summary, read(run/'summary.json'))
    data = plot_data(run); out = run/'review'; out.mkdir(exist_ok=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from shapely import wkb
    cart = wkb.loads(data['cart_wkb_hex'], hex=True)
    colors = ['#167c80','#df8a15','#7956ad','#c23166']; styles = ['-','--','-.',':']; markers = ['o','s','^','x']
    inputs = [p for p in run.rglob('*') if p.is_file() and out not in p.parents]
    hashes = {str(p):sha(p) for p in inputs}; manifest = []
    def finish(fig, key):
        fig.tight_layout(); fig.savefig(out/(key+'.png'), dpi=150, bbox_inches='tight'); plt.close(fig)
        save(out/(key+'.json'), dict(data=data, inputs=hashes, processing_sha256=sha(__file__)))
        manifest.append(dict(name=key, png_sha256=sha(out/(key+'.png')), sidecar_sha256=sha(out/(key+'.json'))))
    def lines():
        for n, color, ls, marker in zip(ORDER, colors, styles, markers):
            yield n, data['methods'][n], dict(color=color, ls=ls, marker=marker, markersize=3,
                                             fillstyle='none', label=n)
    for key, execution in [('world_references',False),('world_execution',True)]:
        fig, ax = plt.subplots(figsize=(11,7))
        for shape, fill in [(cart,True),(cart.buffer(.25),False)]:
            for i,g in enumerate(shape.geoms if hasattr(shape,'geoms') else [shape]):
                if fill: ax.fill(*g.exterior.xy, color='#786145', alpha=.5, label='Cart footprint' if i==0 else None)
                else: ax.plot(*g.exterior.xy, ':', color='#786145', label='Center exclusion: .20 radius + .05 margin' if i==0 else None)
        old = np.array(data['old_prefix']); ax.plot(old[:,0],old[:,1],color='gray',label='Recorded R01 OLD prefix')
        for k,m in [('A','x'),('B','*')]: ax.scatter(*data[k][:2],marker=m,color='black',s=70,label=k,zorder=5)
        for n,d,style in lines():
            if d['reference'] is None: continue
            r = np.array(d['reference']); st = dict(style)
            if execution: st.update(ls='--',label=n+' reference')
            ax.plot(r[:,0],r[:,1],**st)
            if execution and d['metrics']:
                mt=d['metrics']; p=np.array(mt['trace']['poses_world']); st=dict(style,ls='-',markevery=13,label=n+' execution')
                ax.plot(p[:,0],p[:,1],**st)
                jt=mt['full']['join_time_s']
                if jt is not None:
                    j=int(np.searchsorted(mt['trace']['times_s'],jt));ax.scatter(*p[j,:2],marker='D',color=style['color'],s=40)
                q=mt['clearance']['minimum_pose'];ax.scatter(*q[:2],marker='x',color=style['color'],s=70)
        ax.set(aspect='equal',xlabel='World X [m]',ylabel='World Y [m]',
            title='R01 '+('execution; diamond = attachment, x = minimum clearance' if execution else 'references'))
        ax.grid(alpha=.2);ax.legend(loc='upper left',bbox_to_anchor=(1.01,1),fontsize=7);finish(fig,key)
    for key,field,ylabel,threshold in [('position','distance_m','Distance to ORIGINAL R01 FRESH [m]',.1),
        ('yaw','yaw_error_rad','Shortest-angle yaw error [deg]',15),('progress','arc_progress_m','Original FRESH projected arc [m]',None),
        ('clearance','dense_clearance_m','Footprint-edge clearance [m]',.05)]:
        fig,ax=plt.subplots(figsize=(10,5))
        for n,d,style in lines():
            if not d['metrics']:continue
            tr=d['metrics']['trace'];t=tr['dense_times_s'] if key=='clearance' else tr['times_s'];y=tr[field]
            if key=='yaw':y=np.degrees(y)
            ax.plot(t,y,markevery=17,**style)
        if threshold is not None:ax.axhline(threshold,color='black',ls=':',label='Frozen threshold')
        ax.set(xlabel='Seconds after R01 B',ylabel=ylabel,title=LABEL);ax.grid(alpha=.2);ax.legend(fontsize=8);finish(fig,key)
    fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True)
    for n,d,style in lines():
        if not d['metrics']:continue
        c=d['metrics']['command'];u=np.array(c['commands']);t=c['command_start_times_s'];end=d['metrics']['trace']['times_s'][-1]
        for j,ax in enumerate(axes):ax.step([*t,end],np.r_[u[:,j],u[-1,j]],where='post',markevery=3,**style)
    for ax,l in zip(axes,['v [m/s]','omega [rad/s]']):ax.set_ylabel(l);ax.grid(alpha=.2);ax.legend(fontsize=8)
    axes[-1].set_xlabel('Seconds after R01 B');fig.suptitle('Unchanged official MPC; shared initial R01 command');finish(fig,'commands')
    fig,ax=plt.subplots(figsize=(12,5))
    for i,(n,d,st) in enumerate(lines()):
        if d['rollout'] is None:continue
        r=d['rollout'];sub=[e['tick'] for e in r['submit_requests']];apps=[c['application_tick'] for c in r['commands'] if c['reason']=='new_solve']
        ax.scatter(sub,np.full(len(sub),i+.12),marker='|',color=st['color']);ax.scatter(apps,np.full(len(apps),i-.12),marker='o',s=14,color=st['color'])
    ax.axvline(146,ls=':',color='gray',label='Primary endpoint B+54');ax.set(yticks=range(4),yticklabels=ORDER,xlabel='Absolute tick',title='Authenticated R00 schedule reused | submit bars, application circles');ax.grid(alpha=.2);ax.legend();finish(fig,'tick_timeline')
    fig,axes=plt.subplots(2,2,figsize=(11,8))
    for n,d,st in lines():
        p=summary['planning'][n]
        if n=='M0_NATIVE' or p is None:continue
        c=np.array(p['per_node_log_correction']);e=np.array(p['relative_edge_log'])
        for ax,y in zip(axes.flat,[p['per_node_displacement_m'],np.degrees(abs(c[:,2])),np.linalg.norm(e[:,:2],axis=1),np.degrees(abs(e[:,2]))]):ax.plot(range(len(y)),y,**st)
    for ax,label in zip(axes.flat,['Node XY correction [m]','Node yaw correction [deg]','Relative-edge translation [m]','Relative-edge yaw [deg]']):ax.set(xlabel='Node / edge',ylabel=label);ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('Reference geometry; No-relative edge residual is diagnostic only');finish(fig,'deformation')
    effects=summary['replication_effects'];fig,ax=plt.subplots(figsize=(12,7));ax.axis('off')
    fmt=lambda x:'N/A' if x is None else f'{x:+.9g}'
    table=ax.table(cellText=[[r['metric'],fmt(r['R00_NoR_minus_Full']),fmt(r['R01_NoR_minus_Full']),str(r['sign_consistent'])] for r in effects],
        colLabels=['No-relative minus Full','R00','R01','Same sign'],loc='center',colWidths=[.46,.21,.21,.12])
    table.auto_set_font_size(False);table.set_fontsize(10);table.scale(1,1.8)
    ax.set_title(summary['classification']+'\nPaired same-scenario replication; identical raw local FRESH',pad=20);finish(fig,'paired_effects')
    keys=list(next(r for r in summary['primary_metrics'].values() if r is not None))
    for filename,source in [('primary.csv','primary_metrics'),('signed_gaps.csv','signed_method_minus_full'),('own_reference_secondary.csv','secondary_own_reference_metrics')]:
        available=next((r for r in summary[source].values() if r is not None),{})
        write_csv(out/filename,[dict(method=n,**(summary[source][n] or {k:None for k in available})) for n in ORDER])
    write_csv(out/'replication_effects.csv',effects);save(out/'plot_manifest.json',manifest)
    sep=summary['pairwise'].get('NO_RELATIVE_minus_FULL_LOCAL_SE2')
    body='<meta charset="utf-8"><title>OSA03 R01 replication</title><style>body{font:16px system-ui;max-width:1300px;margin:32px auto;padding:0 20px;color:#192834}img{max-width:100%}nav{display:flex;gap:14px;flex-wrap:wrap}section{margin:36px 0}pre{white-space:pre-wrap}a{color:#167c80}</style>'
    body+='<h1>OSA03 REPEAT_01 · relative-factor replication</h1><h2>'+html.escape(summary['classification'])+'</h2>'
    body+='<p>Timing-controlled offline replication. Paired same-scenario evidence. R00 and R01 have identical raw local FRESH arrays. Primary evaluation targets original R01 FRESH. No online latency or trajectory-diverse generalization claim.</p>'
    body+='<p>Maximum separations (metres): '+html.escape(str(sep))+'. Curves overlap; markers and styles distinguish them.</p><nav>'
    body+=''.join(f'<a href="#{r["name"]}">{r["name"]}</a>' for r in manifest)+'</nav>'
    for r in manifest:body+=f'<section id="{r["name"]}"><h2>{r["name"]}</h2><img src="review/{r["name"]}.png"><a href="review/{r["name"]}.json">Numeric and hash sidecar</a></section>'
    with (run/'index.html').open('x') as f:f.write(body)
    save(run/'report_validation.json',validate_figures(run))
    with zipfile.ZipFile(run/'review_bundle.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in [run/'index.html',run/'summary.json',run/'validation.json',run/'report_validation.json',*out.iterdir()]:z.write(p,p.relative_to(run))
    print(json.dumps(read(run/'report_validation.json')))


def validate_figures(run):
    data=plot_data(run);summary=read(run/'summary.json');manifest=read(run/'review/plot_manifest.json')
    assert len(manifest)==10
    for entry in manifest:
        key=entry['name'];assert sha(run/'review'/f'{key}.png')==entry['png_sha256']
        assert sha(run/'review'/f'{key}.json')==entry['sidecar_sha256']
        side=read(run/'review'/f'{key}.json');equal_record(side['data'],data)
        assert side['processing_sha256']==sha(__file__)
        for p,h in side['inputs'].items():assert sha(p)==h,p
    for filename,source in [('primary.csv','primary_metrics'),('signed_gaps.csv','signed_method_minus_full'),('own_reference_secondary.csv','secondary_own_reference_metrics')]:
        with (run/'review'/filename).open() as f:
            for row in csv.DictReader(f):
                actual=summary[source][row.pop('method')]
                for k,v in row.items():assert v==('' if actual is None or actual[k] is None else str(actual[k]))
    with (run/'review/replication_effects.csv').open() as f:
        for row,expected in zip(csv.DictReader(f),summary['replication_effects'],strict=True):
            for k,v in row.items():assert v==('' if expected[k] is None else str(expected[k]))
    return dict(valid=True,ten_numeric_sidecars_recomputed=True,CSV_JSON_parity=True,new_calls=0)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--validate-only',action='store_true');a=p.parse_args()
    print(validate_figures(a.run.resolve())) if a.validate_only else report(a.run.resolve())

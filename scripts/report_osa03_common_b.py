#!/usr/bin/env python3
"""Saved-only plots with exact numerical/hash sidecars; never runs a controller."""
import argparse,csv,html,json,sys,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
from reconciliation.join_source03 import read,save,sha
from reconciliation.osa03_common_b import ORDER
from run_osa03_common_b import geometry,plain
from analyze_join_online02 import csvread,pose_rows
from validate_robotless_online_handoffs import equal_record


def plot_data(run):
    refs=read(run/'references.json');c=read(run/'common_state.json');source=Path(read(run/'source_manifest.json')['source'])
    data=dict(B=c['B'],A=c['fresh_capture_pose'],cart_wkb_hex=geometry(source)['cart'].wkb_hex,
        old_prefix=pose_rows(csvread(source/'episodes/REPEAT_00/execution.csv'))[:c['B_tick']+1].tolist(),methods={})
    for name,ref in refs.items():
        root=run/'methods'/name;m=read(root/'metrics.json') if (root/'metrics.json').exists() else None
        r=read(root/'rollout.json') if (root/'rollout.json').exists() else None
        data['methods'][name]=dict(reference=np.load(ref['world_path']).tolist(),reference_safety=ref['safety'],metrics=m,
            submit_requests=None if r is None else r['submit_requests'],
            applications=None if r is None else [dict(tick=x['application_tick'],time_s=x['relative_time_s'],solve_id=x['solve_id']) for x in r['commands'] if x['reason']=='new_solve'])
    return plain(data)


def write_csv(path,rows):
    with path.open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def report(run):
    from validate_osa03_common_b import validate
    summary,validation=validate(run);equal_record(summary,read(run/'summary.json'));assert validation['valid']
    data=plot_data(run);out=run/'review';out.mkdir(exist_ok=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from shapely import wkb
    from matplotlib.patches import Circle
    cart=wkb.loads(data['cart_wkb_hex'],hex=True);colors=dict(zip(ORDER,['#167c80','#df8a15','#7956ad','#c23166']))
    inputs=[run/n for n in ['summary.json','validation.json','references.json','common_state.json','source_manifest.json','protocol.json','freeze.json']]
    inputs += [p for name in ORDER for p in (run/'methods'/name).glob('*.json')]
    inputs += [Path(r[k]) for r in read(run/'references.json').values() for k in ['world_path','local_path']]
    hashes={str(p):sha(p) for p in inputs};manifest=[]
    def finish(fig,key,subset):
        fig.tight_layout();fig.savefig(out/(key+'.png'),dpi=150,bbox_inches='tight');plt.close(fig)
        save(out/(key+'.json'),dict(data=subset,inputs=hashes,processing_sha256=sha(__file__)))
        manifest.append(dict(name=key,png_sha256=sha(out/(key+'.png')),sidecar_sha256=sha(out/(key+'.json'))))
    def shape(ax):
        for g,style in [(cart,'fill'),(cart.buffer(.25),'line')]:
            for i,p in enumerate(g.geoms if hasattr(g,'geoms') else [g]):
                if style=='fill':ax.fill(*p.exterior.xy,color='#786145',alpha=.5,label='Cart mesh footprint' if i==0 else None)
                else:ax.plot(*p.exterior.xy,':',color='#786145',label='Center boundary: radius .20 + margin .05 m' if i==0 else None)
        ax.scatter(*data['B'][:2],c='black',marker='*',s=100,label='Common B',zorder=5)
        ax.scatter(*data['A'][:2],c='gray',marker='x',s=70,label='Original observation A')
        ax.set(aspect='equal',xlabel='World X [m]',ylabel='World Y [m]');ax.grid(alpha=.2)
    for key,execution in [('world_execution',True),('world_references',False)]:
        fig,ax=plt.subplots(figsize=(10,7));shape(ax)
        old=np.array(data['old_prefix']);ax.plot(old[:,0],old[:,1],color='gray',label='Recorded OLD prefix')
        for name,d in data['methods'].items():
            ref=np.array(d['reference']);color=colors[name];ax.plot(ref[:,0],ref[:,1],'--',color=color,lw=1.2,label=name+' reference')
            if execution and d['metrics']:
                m=d['metrics'];p=np.array(m['trace']['poses_world']);ax.plot(p[:,0],p[:,1],color=color,lw=2.2,label=name+' executed')
                jt=m['full']['join_time_s']
                if jt is not None:
                    j=int(np.searchsorted(m['trace']['times_s'],jt));ax.scatter(*p[j,:2],marker='D',color=color,s=40)
                q=m['clearance']['minimum_pose'];ax.scatter(*q[:2],marker='x',color=color);ax.add_patch(Circle(q[:2],.2,fill=False,color=color,alpha=.4))
        ax.set_title('OSA03 common B | '+('offline execution; diamonds = sustained attachment' if execution else 'frozen references only')+'\n'+summary['classification']['primary'],fontsize=10)
        ax.legend(loc='upper left',bbox_to_anchor=(1.01,1),fontsize=7);finish(fig,key,data)
    for key,field,label,threshold in [('position','distance_m','Distance to original FRESH [m]',.10),('yaw','yaw_error_rad','Wrapped yaw error [deg]',15),('progress','arc_progress_m','Original FRESH projected arc [m]',None),('clearance','dense_clearance_m','Footprint-edge clearance [m]',.05)]:
        fig,ax=plt.subplots(figsize=(10,5))
        for name,d in data['methods'].items():
            if not d['metrics']:continue
            tr=d['metrics']['trace'];t=tr['dense_times_s'] if key=='clearance' else tr['times_s'];y=tr[field]
            if key=='yaw':y=np.degrees(y)
            ax.plot(t,y,color=colors[name],label=name)
        if threshold is not None:ax.axhline(threshold,color='black',ls=':',label='Frozen threshold')
        ax.set(xlabel='Seconds after common B',ylabel=label,title='Original-FRESH evaluation | '+summary['classification']['primary']);ax.grid(alpha=.2);ax.legend(fontsize=8);finish(fig,key,data)
    fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True)
    for name,d in data['methods'].items():
        if not d['metrics']:continue
        c=d['metrics']['command'];u=np.array(c['commands']);t=c['command_start_times_s'];end=d['metrics']['trace']['times_s'][-1]
        for j,ax in enumerate(axes):ax.step([*t,end],np.r_[u[:,j],u[-1,j]],where='post',color=colors[name],label=name)
    for ax,l in zip(axes,['v [m/s]','omega [rad/s]']):ax.set_ylabel(l);ax.grid(alpha=.2);ax.legend(fontsize=8)
    axes[-1].set_xlabel('Seconds after common B');fig.suptitle('Shared initial command; independent subsequent official MPC');finish(fig,'commands',data)
    fig,ax=plt.subplots(figsize=(12,5))
    for i,(name,d) in enumerate(data['methods'].items()):
        if d['submit_requests'] is None:continue
        ax.scatter([e['tick'] for e in d['submit_requests']],np.full(len(d['submit_requests']),i+.12),marker='|',color=colors[name])
        ax.scatter([e['tick'] for e in d['applications']],np.full(len(d['applications']),i-.12),marker='o',s=12,color=colors[name])
    ax.axvline(146,ls=':',color='gray',label='Primary exposure endpoint B+54');ax.set(yticks=range(4),yticklabels=ORDER,xlabel='Absolute integration tick',title='Submit | and command application o | shared B = tick 92');ax.legend();ax.grid(alpha=.2);finish(fig,'tick_timeline',data)
    keys=['position_auc_09_m_s','yaw_auc_09_rad_s','sustained_attachment_s','attachment_fractional_row','remaining_arc_at_attachment_m','execution_clearance_lower_bound_m']
    rows=[dict(method=n,available=summary['primary_metrics'][n] is not None,**(summary['primary_metrics'][n] or {k:None for k in keys})) for n in ORDER]
    write_csv(out/'primary.csv',rows)
    gaprows=[dict(method=n,**(summary['signed_method_minus_native'][n] or {k:None for k in keys})) for n in ORDER]
    write_csv(out/'signed_gaps.csv',gaprows);save(out/'plot_manifest.json',manifest)
    body='<meta charset="utf-8"><style>body{font:16px sans-serif;max-width:1300px;margin:30px auto}img{max-width:100%}pre{white-space:pre-wrap}td,th{padding:7px;border:1px solid #ddd}table{border-collapse:collapse}</style><h1>OSA03 common-B method comparison</h1>'
    body+='<p>'+html.escape(summary['classification']['primary'])+'</p><p>One development source, four frozen references. No new VLA or optimization. All metrics target original FRESH. Different application ticks confound reference attribution. No complete bypass/generalization claim.</p>'
    body+='<table><tr>'+''.join('<th>'+html.escape(k)+'</th>' for k in rows[0])+'</tr>'
    for row in rows:body+='<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in row.values())+'</tr>'
    body+='</table>'
    for x in manifest:body+=f'<h2>{x["name"]}</h2><img src="review/{x["name"]}.png"><p><a href="review/{x["name"]}.json">Numeric/hash sidecar</a></p>'
    with (run/'index.html').open('x') as f:f.write(body)
    save(run/'report_validation.json',validate_figures(run))
    with zipfile.ZipFile(run/'review_bundle.zip','x',zipfile.ZIP_DEFLATED) as z:
        for q in [run/'index.html',run/'summary.json',run/'validation.json',run/'report_validation.json',*out.iterdir()]:z.write(q,q.relative_to(run))
    print(json.dumps(read(run/'report_validation.json')))


def validate_figures(run):
    data=plot_data(run);summary=read(run/'summary.json')
    for p in read(run/'review/plot_manifest.json'):
        key=p['name'];assert sha(run/'review'/f'{key}.png')==p['png_sha256'] and sha(run/'review'/f'{key}.json')==p['sidecar_sha256']
        side=read(run/'review'/f'{key}.json');equal_record(data,side['data']);assert side['processing_sha256']==sha(__file__)
        for f,h in side['inputs'].items():assert sha(f)==h
    for filename,source in [('primary.csv','primary_metrics'),('signed_gaps.csv','signed_method_minus_native')]:
        with (run/'review'/filename).open() as f:
            for row in csv.DictReader(f):
                name=row.pop('method');row.pop('available',None);actual=summary[source][name]
                for key,value in row.items():assert value==('' if actual is None or actual[key] is None else str(actual[key]))
    return dict(valid=True,eight_numeric_sidecars_recomputed=True,CSV_JSON_parity=True,new_calls=0)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--validate-only',action='store_true');a=p.parse_args()
    print(validate_figures(a.run.resolve())) if a.validate_only else report(a.run.resolve())

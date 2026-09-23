#!/usr/bin/env python3
"""Static scientific figures, each bound to the complete plotted numeric records."""
from pathlib import Path
import html,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from run_handoff_delay_attribution import read,save,sha,flat,envs
from reconciliation.handoff_delay_attribution import CONDITIONS,paired
COLORS=['#1976d2','#e65100','#00897b','#8e24aa']
SHORT=['delayed native','delayed prepared+SP','latency-free native','latency-free prepared+SP']

def numbers(run,kind,key):
    records=read(run/'event_records.json')
    if kind=='cohort':
        gaps,transitions=paired(records)
        return dict(metrics=[flat(r) for r in records if r['cohort']==key],gaps=[g for g in gaps if g['cohort']==key],transitions=[t for t in transitions if t['cohort']==key])
    event=next(e for e in read(run/'events.json') if e['case_id']==key)
    return dict(event=event,records=[r for r in records if r['case_id']==key])

def emit(fig,run,name,kind,key):
    p=run/'plots'/(name+'.png');fig.savefig(p,dpi=155,bbox_inches='tight');plt.close(fig)
    save(p.with_suffix('.json'),dict(kind=kind,key=key,numbers=numbers(run,kind,key),png_sha256=sha(p),
        source_hashes={x:sha(run/x) for x in ['source_manifest.json','events.json','event_records.json','protocol.json','freeze.json']}))

def draw_geometry(ax,env,points):
    pts=np.vstack(points);lo=pts[:,:2].min(0)-.5;hi=pts[:,:2].max(0)+.5
    from shapely.geometry import box
    for geom in env.parts:
        if not geom.intersects(box(*lo,*hi)):continue
        gs=list(geom.geoms) if hasattr(geom,'geoms') else [geom]
        for g in gs:
            if hasattr(g,'exterior'):
                xy=np.array(g.exterior.coords);ax.fill(xy[:,0],xy[:,1],color='.72',alpha=.5)
                bound=g.buffer(.25).boundary
                for line in list(bound.geoms) if hasattr(bound,'geoms') else [bound]:
                    q=np.array(line.coords);ax.plot(q[:,0],q[:,1],color='.55',lw=.7,ls=':')
    ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1]);ax.set_aspect('equal');ax.set_xlabel('world X [m]');ax.set_ylabel('world Y [m]')

def render(run):
    (run/'plots').mkdir(exist_ok=False);records=read(run/'event_records.json');events=read(run/'events.json');cfg=read(run/'protocol.json');environment=envs(cfg)
    gaps,_=paired(records)
    for cohort in ['GENUINE_13','ONLINE03_ONSET']:
        rr=[r for r in records if r['cohort']==cohort];ids=list(dict.fromkeys(r['case_id'] for r in rr));x=np.arange(len(ids));fig,ax=plt.subplots(figsize=(max(11,len(ids)*.85),5))
        for c,color,label in zip(CONDITIONS,COLORS,SHORT):
            y=[next(r for r in rr if r['case_id']==cid and r['condition']==c)['primary'] for cid in ids]
            ax.plot(x,[v['position_auc_m_s'] if v else np.nan for v in y],'o-',c=color,label=label)
        ax.set_xticks(x,ids,rotation=65,ha='right',fontsize=8);ax.set_ylabel('position AUC to ORIGINAL FRESH [m s]');ax.set_title(cohort+' | equal exposure 54 intervals (0.900000047 s)');ax.legend(fontsize=8);ax.grid(alpha=.2)
        emit(fig,run,cohort+'_position_auc','cohort',cohort)
        fig,ax=plt.subplots(figsize=(max(11,len(ids)*.85),5))
        for comp,color in [('delay_native',COLORS[0]),('delay_lookahead',COLORS[1])]:
            vals=[next(g for g in gaps if g['case_id']==cid and g['comparison']==comp)['position_auc_m_s'] for cid in ids]
            ax.plot(x,[v if v is not None else np.nan for v in vals],'o-',label=comp,c=color)
        ax.axhline(0,c='.4',lw=.7);ax.set_xticks(x,ids,rotation=65,ha='right',fontsize=8);ax.set_ylabel('delayed minus latency-free position AUC [m s]');ax.legend();ax.set_title(cohort+' | delay sensitivity, not recoverable cost');ax.grid(alpha=.2)
        emit(fig,run,cohort+'_delay_gap','cohort',cohort)
        fig,ax=plt.subplots(figsize=(14,max(4,len(ids)*.37)));ax.axis('off')
        abbr={'OBSERVED_SAMPLED_JOIN':'SUSTAINED','NO_TUBE_ENTRY_OBSERVED':'NO ENTRY','TUBE_ENTERED_DWELL_RIGHT_CENSORED':'CENSORED','TRANSIENT_ENTRY_THEN_EXIT':'TRANSIENT'}
        cells=[]
        for cid in ids:
            row=[cid]
            for c in CONDITIONS:
                r=next(r for r in rr if r['case_id']==cid and r['condition']==c);p=r['primary'];row.append(abbr[p['observation_status']] if p else r['status'])
            cells.append(row)
        table=ax.table(cellText=cells,colLabels=['event']+SHORT,loc='center',cellLoc='center',colWidths=[.36,.16,.16,.16,.16]);table.auto_set_font_size(False);table.set_fontsize(8);table.scale(1,1.65)
        ax.set_title(cohort+' | sampled attachment categories; censoring is not failure time',pad=15)
        emit(fig,run,cohort+'_attachment','cohort',cohort)
    for e in events:
        if not e['available']:continue
        rr=[r for r in records if r['case_id']==e['case_id']];fig,axes=plt.subplots(1,3,figsize=(15,4.8));ax=axes[0]
        old,fresh,past,common=[np.array(e[k]) for k in ['old_world','fresh_world','past']]+[np.array(e['references']['common'])]
        ax.plot(old[:,0],old[:,1],'--',c='.65',lw=1,label='OLD prediction');ax.plot(past[:,0],past[:,1],c='.25',lw=3,label='saved OLD obs to B')
        ax.plot(fresh[:,0],fresh[:,1],'--',c='#c2185b',lw=1.5,label='original FRESH');ax.plot(common[:,0],common[:,1],':',c='black',label='fixed prepared FRESH')
        points=[fresh,past]
        for r,color,label in zip(rr,COLORS,SHORT):
            p=r['primary'] or r['partial']
            if not p:continue
            trace=p['trace'];q=np.array(trace['poses_world']);points.append(q);ax.plot(q[:,0],q[:,1],c=color,label=label)
            axes[1].plot(trace['times_s'],trace['distance_m'],c=color,label=label);axes[2].plot(trace['times_s'],np.degrees(trace['yaw_error_rad']),c=color)
            if p['join_time_s'] is not None:
                j=np.searchsorted(trace['times_s'],p['join_time_s']);ax.scatter(q[j,0],q[j,1],marker='*',c=color,s=70)
        for key,marker,label in [('R_obs','o','observation'),('B','X','B')]:
            q=e['context'][key];ax.scatter(*q[:2],marker=marker,s=65,c='black',label=label)
        draw_geometry(ax,environment[e['cohort']],points);ax.legend(fontsize=6.5,loc='best')
        axes[1].axhline(.10,c='k',ls=':',lw=1);axes[2].axhline(15,c='k',ls=':',lw=1)
        axes[1].set_ylabel('distance to original FRESH [m]');axes[2].set_ylabel('wrapped yaw error [deg]')
        for a in axes[1:]:a.set_xlabel('counterfactual elapsed time [s]');a.grid(alpha=.2)
        fig.suptitle(e['case_id']+' | OFFLINE COUNTERFACTUAL | '+e['source_role'],fontsize=11);fig.tight_layout()
        emit(fig,run,e['case_id'].replace('/','__'),'event',e['case_id'])
    plots=sorted((run/'plots').glob('*.png'))
    page='<html><meta charset="utf-8"><title>Handoff delay attribution</title><style>body{font-family:sans-serif;margin:30px}img{max-width:100%}</style><h1>HANDOFF-DELAY-ATTRIBUTION-01</h1><p>Static saved diagnostics. No GUI, new VLA or online execution. Latency-free is not a recoverable-cost bound. Native vs prepared+SP includes preparation. Star: sampled sustained attachment; dotted gray geometry: 0.20m radius + 0.05m margin.</p>'
    for p in plots:page+=f'<h2>{html.escape(p.stem)}</h2><a href="plots/{p.stem}.json">Numeric/source sidecar</a><br><img src="plots/{p.name}">'
    (run/'index.html').write_text(page+'</html>')

def validate_plots(run):
    for p in (run/'plots').glob('*.json'):
        r=read(p);assert r['numbers']==numbers(run,r['kind'],r['key']);assert r['png_sha256']==sha(p.with_suffix('.png'))
        for name,h in r['source_hashes'].items():assert sha(run/name)==h

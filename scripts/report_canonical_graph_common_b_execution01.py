#!/usr/bin/env python3
"""Exactly three compact saved-only PNGs and their numeric sidecar."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from run_canonical_graph_common_b_execution01 import RESULTS,read,save,sha,plain
from reconciliation.canonical_graph_execution01 import CENTRAL,RAW,ENTRY,CANONICAL,PNGS,AUC,ATTACH,END,execution_guard
COLORS=['#2166ac','#1b9e77','#d95f02']
SHORT=['B + raw','B + entry','B + canonical']


def figure_numbers(run,s):
    sources={}
    for spec in s['selected_sources']:
        f=Path(spec['folder']);refs=read(f/'references.json');c=read(f/'common_state.json')
        sources[spec['label']]=dict(B=c['B'],A=c['fresh_capture_pose'],entry=read(f/'entry.json')['correspondence']['target_world'],
            raw=np.load(f/'references/M0_NATIVE_world.npy').tolist(),
            canonical=np.load(f/'canonical_optimized.npy').tolist(),
            references={n:np.load(refs[n]['world_path']).tolist() for n in CENTRAL})
    return plain(dict(geometry=sources,metrics={label:dict(methods=q['methods'],commands=q['commands'],pairwise=q['pairwise']) for label,q in s['sources'].items()},
        primary_target='full original FRESH',null_display='N/A; never cap-imputed'))


def bars(ax,labels,values,title,ylabel,zero=False):
    x=np.arange(len(labels));allvalues=[]
    for j,(n,color,short) in enumerate(zip(CENTRAL,COLORS,SHORT)):
        vals=[v[n] for v in values];pos=x+(j-1)*.24
        ax.bar(pos,[np.nan if v is None else v for v in vals],.22,color=color,label=short)
        for p,v in zip(pos,vals):
            if v is None:ax.text(p,.025,'N/A',rotation=90,ha='center',va='bottom',transform=ax.get_xaxis_transform(),fontsize=8)
            else:allvalues.append(v)
    ax.set_xlim(-.55,len(labels)-.45);ax.set_xticks(x,labels);ax.set_title(title,fontsize=11);ax.set_ylabel(ylabel);ax.grid(axis='y',alpha=.2)
    if zero:ax.axhline(0,color='black',lw=.6)
    ax.margins(y=.16)


def render(data,out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    labels=list(data['geometry']);fig,axes=plt.subplots(2,2,figsize=(11,8))
    for ax,label in zip(axes.flat,labels):
        g=data['geometry'][label];raw=np.asarray(g['raw']);x=np.asarray(g['canonical'])
        ax.plot(raw[:,0],raw[:,1],':',color='.2',lw=2,label='Original FRESH',zorder=4)
        for n,color,style,marker in zip(CENTRAL,COLORS,['--','-.','-'],['o','s','^']):
            w=np.asarray(g['references'][n]);ax.plot(w[:,0],w[:,1],style,color=color,lw=1.5,marker=marker,ms=3,mfc='none',label=SHORT[CENTRAL.index(n)])
        ax.scatter(*g['B'][:2],c='black',marker='x',s=80,zorder=6,label='B')
        ax.scatter(*g['entry'][:2],c='#762a83',marker='*',s=100,zorder=7,label='C3 E*')
        ax.scatter(x[:,0],x[:,1],s=18,facecolors='none',edgecolors=COLORS[2],zorder=6)
        ax.set_title(label+' · fixed B; full original identities retained');ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2)
        ax.set_xlabel('World X (m)');ax.set_ylabel('World Y (m)')
    handles,names=axes.flat[0].get_legend_handles_labels();fig.legend(handles,names,loc='lower center',ncol=6,fontsize=9)
    fig.suptitle('Reference geometry · explicit B connectors included in safety checks')
    fig.tight_layout(rect=(0,.07,1,.95));fig.savefig(out/PNGS[0],dpi=160);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(11,7))
    for ax,key,title,unit in zip(axes.flat,[AUC,ATTACH,END,'remaining_arc_at_attachment_m'],
        ['Early original-FRESH position AUC','Sustained attachment','Original-FRESH endpoint dwell','Original arc remaining at attachment'],['m · s','s after B','s after B','m']):
        values=[{n:None if data['metrics'][l]['methods'][n]['metrics'] is None else data['metrics'][l]['methods'][n]['metrics'][key] for n in CENTRAL} for l in labels]
        bars(ax,labels,values,title,unit)
    handles,names=axes.flat[0].get_legend_handles_labels();fig.legend(handles,names,loc='lower center',ncol=3)
    fig.suptitle('Timing-controlled offline execution · full ORIGINAL FRESH evaluation')
    fig.tight_layout(rect=(0,.06,1,.95));fig.savefig(out/PNGS[1],dpi=160);plt.close(fig)
    fig,axes=plt.subplots(3,2,figsize=(11,9))
    diagnostics=[('physical_jump_v','First new v − physical pre-B v','m/s'),('physical_jump_w','First new ω − physical pre-B ω','rad/s'),
        ('linear_command_TV','Linear command TV','m/s'),('angular_command_TV','Angular command TV','rad/s'),
        (AUC,'Position AUC .9 difference from B + entry','m · s'),(END,'Endpoint dwell difference from B + entry','s')]
    for ax,(key,title,unit) in zip(axes.flat,diagnostics):
        vals=[]
        for label in labels:
            q=data['metrics'][label];v={}
            for n in CENTRAL:
                m=q['methods'][n]['metrics'];cmd=q['commands'][n]
                if key.startswith('physical_jump'):
                    jump=None if cmd is None else cmd['delta_from_physical_u_minus'];value=None if jump is None else jump[0 if key.endswith('v') else 1]
                elif key in [AUC,END]:
                    b=q['methods'][ENTRY]['metrics'];value=None if m is None or b is None or m[key] is None or b[key] is None else m[key]-b[key]
                else:value=None if m is None else m[key]
                v[n]=value
            vals.append(v)
        bars(ax,labels,vals,title,unit,zero=True)
    handles,names=axes.flat[0].get_legend_handles_labels();fig.legend(handles,names,loc='lower center',ncol=3)
    fig.suptitle('Command transition and downstream preservation · no scalar winner score')
    fig.tight_layout(rect=(0,.045,1,.96));fig.savefig(out/PNGS[2],dpi=160);plt.close(fig)
    assert sorted(p.name for p in out.glob('*.png'))==sorted(PNGS)


def report(run):
    with execution_guard(saved_only=True):
        s=read(run/'summary.json');data=figure_numbers(run,s);save(RESULTS/'figure_numeric.json',data)
        render(data,RESULTS/'figures')
        save(RESULTS/'figure_manifest.json',dict(numeric_sha256=sha(RESULTS/'figure_numeric.json'),
            figures=[dict(file=n,sha256=sha(RESULTS/'figures'/n)) for n in PNGS],saved_only=True,new_scientific_calls=0))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();report(a.run.resolve())

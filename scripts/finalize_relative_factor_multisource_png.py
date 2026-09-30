#!/usr/bin/env python3
"""Saved-only presentation correction; retain every frozen scientific/report byte.

Adjust axes and text space immediately before saving the frozen renderer's figures.
The numerical sidecars must remain identical. No solver or rollout is reachable.
"""
import argparse,json,tempfile,shutil
from pathlib import Path
import numpy as np
from matplotlib.figure import Figure
from report_relative_factor_multisource01 import compact_figures,RESULTS
from reconciliation.relative_factor_multisource import geometry
from reconciliation.join_source03 import read,save,sha


def finalize(run):
    tracked=read(RESULTS/'result_summary.json');summary=read(run/'summary.json')
    assert tracked['summary']==summary and tracked['validation']['valid']
    assert sha(run/'result_hashes.json')==tracked['result_hashes_sha256']
    for name,h in read(run/'result_hashes.json').items():assert sha(run/name)==h,name
    old=read(run/'figure_manifest.json');current=read(RESULTS/'figure_manifest.json')
    assert [r['numeric_sidecar'] for r in old['figures']]==[r['numeric_sidecar'] for r in current['figures']]
    for row in current['figures']:assert sha(RESULTS/'figures'/row['file'])==row['sha256']
    original_save=Figure.savefig
    def readable_save(fig,path,*args,**kwargs):
        name=Path(path).name;n=len(summary['selected_sources'])
        if name=='world_execution_overview.png':
            for ax,spec in zip(fig.axes,summary['selected_sources']):
                lo=np.array([ax.get_xlim()[0],ax.get_ylim()[0]]);hi=np.array([ax.get_xlim()[1],ax.get_ylim()[1]])
                cart=geometry(Path(spec['folder']))['cart']
                if cart is not None:
                    b=np.array(cart.bounds);lo=np.minimum(lo,b[:2]-.3);hi=np.maximum(hi,b[2:]+.3)
                center=(hi+lo)/2;half=max(hi-lo)/2
                ax.set_xlim(center[0]-half,center[0]+half);ax.set_ylim(center[1]-half,center[1]+half)
                ax.set_aspect('equal',adjustable='box')
        elif name=='benchmark_primary_metrics.png':
            for ax in fig.axes:ax.set_xlim(-.5,n-.5)
        elif name=='no_relative_minus_full_deltas.png':
            for ax in fig.axes:
                lo,hi=ax.get_xlim();span=max(abs(lo),abs(hi),1e-12)
                ax.set_xlim(min(lo,-.25*span),max(hi,.25*span));ax.set_ylim(-.5,n-.5)
                for label in ax.texts:label.set_fontsize(8)
        elif name=='reference_deformation_overview.png':
            for ax in fig.axes[:n]:
                lo,hi=ax.get_ylim();ax.set_ylim(lo,hi+max(.18,(hi-lo)*.25))
            fig.subplots_adjust(hspace=.50,wspace=.28,top=.90,bottom=.07)
        return original_save(fig,path,*args,**kwargs)
    Figure.savefig=readable_save
    try:
        with tempfile.TemporaryDirectory(prefix='multisource-png-') as tmp:
            folder=Path(tmp)/'figures';records=compact_figures(run,summary,folder)
            assert [r['numeric_sidecar'] for r in records]==[r['numeric_sidecar'] for r in old['figures']]
            assert [r['file'] for r in records]==[r['file'] for r in old['figures']]
            for r in records:shutil.copyfile(folder/r['file'],RESULTS/'figures'/r['file'])
    finally:Figure.savefig=original_save
    audit=dict(reason='Include whole cart footprint; square equal-axis world panels; keep NA/tiny-delta labels inside axes; reserve space for reference annotations',
        science_unchanged=True,numeric_sidecars_identical=True,new_scientific_calls=0,
        frozen_report_sha256=sha(Path(__file__).with_name('report_relative_factor_multisource01.py')),
        presentation_script_sha256=sha(__file__),before={r['file']:r['sha256'] for r in old['figures']},
        after={r['file']:r['sha256'] for r in records},final_png_count=len(records),extra_final_pngs=0)
    audit_path=RESULTS/'presentation_audit.json'
    if audit_path.exists():audit['previous_presentation_pass']=read(audit_path)
    audit_path.write_text(json.dumps(audit,indent=2,allow_nan=False)+'\n')
    final=dict(old,figures=records,presentation_audit_sha256=sha(RESULTS/'presentation_audit.json'))
    (RESULTS/'figure_manifest.json').write_text(json.dumps(final,indent=2,allow_nan=False)+'\n')
    tracked['figures']=[dict(path=str(RESULTS/'figures'/r['file']),sha256=r['sha256']) for r in records]
    tracked['presentation_audit']=audit
    (RESULTS/'result_summary.json').write_text(json.dumps(tracked,indent=2,allow_nan=False)+'\n')
    print(json.dumps(audit))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();finalize(a.run.resolve())

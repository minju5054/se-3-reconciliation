#!/usr/bin/env python3
"""Additive REF-02 layout correction; primary results and plots stay immutable.

The corrected input figure uses bounded table cells below separate title space.
It reads the exact primary numeric sidecar and performs zero numerical solves.
"""
from __future__ import annotations
import argparse
import hashlib
import html
import json
from pathlib import Path
import shutil
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from reconciliation.gp_se2_environment import HospitalEnvironment
from plot_gp_se2_ref02 import LABEL, PLOT_NAMES, METHODS, COLORS, arrows
from plot_gp_se2_02 import base_xy, environment_path


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False); stream.write('\n')


def table_spec(lineage):
    """Explicit axes-relative bounds prevent any row from entering title space."""
    cells = [[str(r['derived_row_index']), f"{r['original_fractional_row_coordinate']:.4f}",
              f"{r['world_xy'][0]:.6f}", f"{r['world_xy'][1]:.6f}", f"{np.rad2deg(r['wrapped_yaw']):.3f}"] for r in lineage]
    height = min(.86, (len(cells)+1)*.030)
    return dict(cells=cells, bbox=[0., .86-height, 1., height], title_y=.97,
                columns=['Row #','Source s','World x [m]','World y [m]','Yaw [deg]'],
                font_size_pt=8., column_widths=[.10,.16,.255,.255,.23])


def render_fixed_input(numeric, environment, config, target, case_id):
    target = Path(target)
    if target.exists():
        raise FileExistsError('corrected figure already exists')
    fig = plt.figure(figsize=(24,16))
    grid = fig.add_gridspec(2,3,width_ratios=[1,1,1.6],left=.035,right=.988,bottom=.075,top=.885,wspace=.25,hspace=.29)
    for row, name in enumerate((METHODS[0], METHODS[1])):
        item = numeric['methods'][name]; ref = np.asarray(item['reference_world']); color = COLORS[name]
        label = 'A: Native input' if row == 0 else 'B and C: byte-identical dense input'
        context_ax, detail_ax, table_ax = [fig.add_subplot(grid[row,col]) for col in range(3)]
        base_xy(context_ax,numeric,environment,numeric['axes_world_m'],config)
        context_ax.plot(ref[:,0],ref[:,1],color=color,marker='o',ms=3,lw=1.)
        context_ax.set_title(label+'\nUnchanged world context',fontsize=11,pad=12)
        context_ax.legend(fontsize=7,loc='lower left')
        bounds=numeric['input_axes_world_m'];side=bounds[1]-bounds[0];native=np.asarray(numeric['fresh_world'])
        detail_ax.plot(native[:,0],native[:,1],':',color='#999999',lw=1,label='original FRESH context')
        detail_ax.plot(ref[:,0],ref[:,1],color=color,marker='o',ms=4,lw=1,label=label)
        arrows(detail_ax,ref,color,side*.06)
        detail_ax.scatter(*ref[0,:2],facecolors='none',edgecolors=color,s=90)
        detail_ax.scatter(*ref[-1,:2],marker='*',color=color,s=90)
        detail_ax.annotate('first #0',ref[0,:2],xytext=(7,-15),textcoords='offset points',fontsize=8)
        detail_ax.annotate(f'last #{len(ref)-1}',ref[-1,:2],xytext=(7,12),textcoords='offset points',fontsize=8)
        detail_ax.set(xlim=bounds[:2],ylim=bounds[2:],xlabel='world x [m]',ylabel='world y [m]',aspect='equal')
        detail_ax.ticklabel_format(useOffset=False);detail_ax.grid(alpha=.2);detail_ax.legend(fontsize=7,loc='lower left')
        detail_ax.set_title(f'Common input-only zoom; {len(ref)} rows\nHeading glyph {side*.06:.4f} m; direction only',fontsize=11,pad=12)
        spec=table_spec(item['row_provenance']);table_ax.axis('off')
        table_ax.text(.5,spec['title_y'],'All row identities shown separately\nSource s is row order, not time or distance',
                      transform=table_ax.transAxes,ha='center',va='top',fontsize=11)
        table=table_ax.table(cellText=spec['cells'],colLabels=spec['columns'],cellLoc='right',colLoc='center',
                            bbox=spec['bbox'],colWidths=spec['column_widths'])
        table.auto_set_font_size(False);table.set_fontsize(spec['font_size_pt'])
        for (r,c),cell in table.get_celld().items():
            if r==0:cell.set_facecolor('#e8edf1');cell.set_text_props(weight='bold')
            elif r%2==0:cell.set_facecolor('#f7f7f7')
    fig.suptitle(case_id+' | Fixed inputs: Native versus shared B/C dense input',fontsize=17,y=.96)
    fig.text(.035,.925,'PRESENTATION CORRECTION ONLY: bounded row tables and separate titles. All primary inputs, rollouts and metrics are unchanged.',fontsize=11)
    fig.text(.035,.035,LABEL+' | B/C input bytes identical | Exact primary numeric values retained | No new controller or optimizer solves',fontsize=9)
    fig.savefig(target,dpi=160);plt.close(fig)


def source_manifest(primary):
    primary=Path(primary)
    if read(primary/'plot_manifest.json')['experiment']!='GP-SE2-REF-02':
        raise ValueError('expected a completed REF-02 primary plot run')
    bundle=primary/'review_bundle';manifest=read(bundle/'manifest.json')
    files={}
    for record in manifest['allowlisted_files']:
        path=bundle/record['path']
        if digest(path)!=record['sha256']:
            raise ValueError('primary review artifact changed: '+record['path'])
        files[str(path.resolve())]=record['sha256']
    preserved={str((primary/p).resolve()):digest(primary/p) for p in ('plot_manifest.json','index.html','review_bundle.zip')}
    for record in read(primary/'plot_manifest.json')['images']:
        path=primary/record['path'];preserved[str(path.resolve())]=digest(path)
        preserved[str(path.with_suffix('.json').resolve())]=digest(path.with_suffix('.json'))
    return manifest,files,preserved


def main(primary, output):
    primary,output=Path(primary).resolve(),Path(output).resolve()
    if output.exists():
        raise FileExistsError('presentation output must be a new run ID')
    if primary==output or primary.is_relative_to(output) or output.is_relative_to(primary):
        raise ValueError('presentation must be separate from the primary run')
    manifest,review_hashes,preserved=source_manifest(primary)
    cases=read(primary/'case_manifest.json')['selected'];config=yaml.safe_load((primary/'config_snapshot.yaml').read_text())
    env=HospitalEnvironment.load(environment_path(primary))
    output.mkdir(parents=True)
    corrected={f"cases/{case['case_directory']}/plots/fixed_input_world{ext}" for case in cases for ext in ('.png','.json')}
    copied=[]
    for record in manifest['allowlisted_files']:
        relative=Path(record['path'])
        if str(relative) in corrected or str(relative) in ('index.html','README.md','plot_manifest.json'):
            continue
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('unsafe primary review path')
        src=primary/'review_bundle'/relative;dst=output/relative;dst.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(src,dst);copied.append(dict(path=str(relative),source=str(src),sha256=digest(dst)))
    images=[];corrections=[]
    for case in cases:
        relative=Path('cases')/case['case_directory']/'plots/fixed_input_world.png'
        original=primary/relative;side=read(original.with_suffix('.json'));target=output/relative
        target.parent.mkdir(parents=True,exist_ok=True)
        render_fixed_input(side['numeric_data'],env,config,target,case['case_id'])
        corrected_side={**side,'image_sha256':digest(target),'renderer_sha256':digest(__file__),
            'primary_image_path':str(original),'primary_image_sha256':digest(original),
            'primary_sidecar_path':str(original.with_suffix('.json')),'primary_sidecar_sha256':digest(original.with_suffix('.json')),
            'presentation_correction':'bounded axes-relative row table below separate title; layout only',
            'new_controller_solves':0,'numeric_values_changed':False,'primary_artifacts_overwritten':False}
        write(target.with_suffix('.json'),corrected_side)
        corrections.append(dict(case_id=case['case_id'],path=str(relative),source_numeric_sha256=digest(original.with_suffix('.json')),
            input_identity=side['input_identity'],numeric_data_equal=corrected_side['numeric_data']==side['numeric_data']))
    for case in cases:
        for name in PLOT_NAMES:
            path=output/'cases'/case['case_directory']/'plots'/(name+'.png')
            images.append(dict(case_id=case['case_id'],path=str(path.relative_to(output)),sha256=digest(path),
                sidecar_sha256=digest(path.with_suffix('.json')),presentation_corrected=name=='fixed_input_world'))
    if any(digest(p)!=h for p,h in preserved.items()) or any(digest(p)!=h for p,h in review_hashes.items()):
        raise ValueError('primary artifacts changed during presentation correction')
    write(output/'presentation_provenance.json',dict(kind='ADDITIVE_RENDERING_ONLY_CORRECTION',primary_run=str(primary),
        primary_files_preserved=preserved,primary_review_files_preserved=review_hashes,renderer_sha256=digest(__file__),
        reason='frozen primary dense-input table title overlaps its first two rows',corrected_images=corrections,
        copied_review_artifacts=copied,new_controller_solves=0,new_optimizer_solves=0,new_inference=0,
        numerical_results_changed=False,primary_artifacts_overwritten=False,gui_runtime_validated=False))
    write(output/'plot_manifest.json',dict(experiment='GP-SE2-REF-02-PRESENTATION',primary_run=str(primary),image_count=len(images),
        corrected_image_count=2,unchanged_image_count=20,images=images,renderer_sha256=digest(__file__)))
    (output/'README.md').write_text('''# REF-02 presentation correction

This is an additive layout correction only. Two fixed-input figures have bounded row tables and separate title space. The other 20 scientific figures and their JSON sidecars are byte-for-byte copies of the frozen primary review bundle. All six rollouts, numerical metrics, reference arrays, selected targets and source hashes are unchanged. No additional controller, trajectory optimizer, inference or GUI run was performed.

Open index.html. The two corrected sidecars retain the exact primary numeric_data and input_identity and link their primary PNG/JSON hashes. presentation_provenance.json records the correction and verifies that the original primary PNGs, index and review ZIP stayed unchanged. This presentation is separate from the authoritative primary experiment.
''')
    lines=['<!doctype html><html><head><meta charset="utf-8"><title>REF-02 presentation</title>',
        '<style>body{font:16px system-ui;max-width:1600px;margin:25px auto;padding:15px}img{max-width:100%}article{margin:35px 0}</style></head><body>',
        '<h1>'+LABEL+'</h1><p>Readable presentation: two input-table layouts corrected; all other 20 figures and every numerical result unchanged. No new controller solves or GUI runtime claim.</p>',
        '<p><a href="presentation_provenance.json">Correction/source hashes</a> · <a href="aggregate/outcomes.csv">All outcomes</a> · <a href="aggregate/summary.json">Summary</a></p>']
    for case in cases:
        lines.append('<h2>'+html.escape(case['case_id'])+'</h2>')
        for name in PLOT_NAMES:
            path=f'cases/{case["case_directory"]}/plots/{name}'
            suffix=' — corrected table layout only' if name=='fixed_input_world' else ''
            lines.append(f'<article><h3>{html.escape(name.replace("_"," ")+suffix)}</h3><a href="{path}.png"><img loading="lazy" src="{path}.png" alt="{name}"></a><p><a href="{path}.json">Exact numbers and source hashes</a></p></article>')
    (output/'index.html').write_text('\n'.join(lines+['</body></html>']))
    records=[dict(path=str(p.relative_to(output)),sha256=digest(p),bytes=p.stat().st_size) for p in sorted(output.rglob('*')) if p.is_file()]
    write(output/'manifest.json',dict(files=records,excluded=['raw RGB','environment export','binary arrays','external source','checkpoints'],
        image_count=22,corrected_image_count=2,new_controller_solves=0,self_excluded='manifest.json; review_bundle.zip; validation.json'))
    archive=output/'review_bundle.zip'
    with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED) as target:
        for record in records:target.write(output/record['path'],record['path'])
        target.write(output/'manifest.json','manifest.json')
    result=dict(valid=True,primary_unchanged=True,numeric_data_unchanged=all(r['numeric_data_equal'] for r in corrections),
        image_count=22,corrected_image_count=2,unchanged_image_count=20,new_controller_solves=0,new_optimizer_solves=0,
        review_bundle_sha256=digest(archive),review_bundle_bytes=archive.stat().st_size,renderer_sha256=digest(__file__),
        gui_runtime_validated=False,validation_scope='presentation content/source identity; primary numerical validation remains authoritative')
    write(output/'validation.json',result)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--primary',required=True,type=Path);parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args();print(json.dumps(main(args.primary,args.output),indent=2))

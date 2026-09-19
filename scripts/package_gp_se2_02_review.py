#!/usr/bin/env python3
"""Package allowlisted GP-SE2-02 tables, all-case overlays and renderer evidence.

No datasets, raw RGB, environment exports, weights, videos or solver vectors.
"""
from __future__ import annotations
import argparse
import hashlib
import html
import json
from pathlib import Path
import shutil
import zipfile


def read(path): return json.loads(Path(path).read_text())
def digest(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,value):
    with Path(path).open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')


def package(run,gui=None):
    run=Path(run).resolve();bundle=run/'review_bundle';archive=run/'review_bundle.zip'
    if bundle.exists() or archive.exists():raise FileExistsError('refusing bundle overwrite')
    mapping=[]
    for name in ('source.json','protocol.json','config_snapshot.yaml','experiment_config.yaml','case_manifest.json','plot_manifest.json'):
        if (run/name).is_file():mapping.append((run/name,Path(name)))
    for name in ('all_starts.csv','candidate_acceptance.csv','primary_outcomes.csv','paired_metrics.csv','regressions.csv','timing.csv','summary.json'):
        source=run/'aggregate'/name
        if source.is_file():mapping.append((source,Path('aggregate')/name))
    manifest=read(run/'case_manifest.json');selected=manifest['selected']
    for row in selected:
        for name in ('candidate_world_overlay','actual_rollout_overlay','boundary_zoom','candidate_feasibility_summary'):
            for ext in ('.png','.json'):
                relative=Path('cases')/row['case_directory']/'plots'/(name+ext)
                source=run/relative
                if not source.is_file():raise FileNotFoundError(source)
                mapping.append((source,relative))
    gui_record=None
    if gui is not None:
        gui=Path(gui).resolve();gui_record=read(gui/'runtime_validation.json')
        if gui_record['status']!='GP_SE2_02_COMPARISON_REPLAY_RUNTIME_VALIDATED':raise ValueError('GUI status is not this experiment replay')
        if Path(gui_record['experiment_run']).resolve()!=run:raise ValueError('GUI belongs to another run')
        mapping.append((gui/'runtime_validation.json',Path('gui/runtime_validation.json')))
        for record in gui_record['captures']:
            if Path(record['path']).name!=record['path']:raise ValueError('unsafe GUI capture path')
            image=gui/record['path']
            if digest(image)!=record['sha256']:raise ValueError('GUI screenshot hash mismatch')
            for p in (image,image.with_suffix('.json')):mapping.append((p,Path('gui')/p.name))
    bundle.mkdir()
    records=[]
    for source,relative in mapping:
        if relative.is_absolute() or '..' in relative.parts:raise ValueError('unsafe bundle path')
        target=bundle/relative;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
        records.append(dict(path=str(relative),source=str(source),sha256=digest(target),bytes=target.stat().st_size))
    (bundle/'README.md').write_text('''# GP-SE2-02 review bundle

Open index.html. Four user-fixed development-corpus cases and all six planned methods are shown, including failures. This is a diagnostic transfer pilot, not a held-out benchmark or a frequency estimate.

Candidate availability, independent plan validity and original-goal/dwell rollout success are separate. GP objective reduction is not a navigation improvement percentage. Rollouts are independently recomputed using the unchanged official MPC from preserved B, physical u_minus and controller memory. Saved Isaac views are playback only.

Adjacent plot JSON files contain plotted values and input/config/source hashes. GUI sidecars contain actual renderer provenance and saved sample readback. Source hashes refer to immutable local files excluded from this compact archive. No raw RGB, full source dataset, environment export, weights, external code, videos or solver vectors are bundled.
''')
    lines=['<!doctype html><html><head><meta charset="utf-8"><title>GP-SE2-02 review</title><style>body{font:16px sans-serif;max-width:1400px;margin:25px auto}img{width:48%;vertical-align:top}</style></head><body><h1>OFFLINE COUNTERFACTUAL HARD-HANDOFF COMPARISON</h1><p>Saved numerical results and actual renderer evidence; see README.md for scope and provenance.</p><h2>Tables and frozen protocol</h2><ul>']
    for record in records:
        if not record['path'].startswith(('cases/','gui/')):lines.append(f'<li><a href="{record["path"]}">{record["path"]}</a></li>')
    lines.append('</ul>')
    for row in selected:
        lines.append(f'<h2>{html.escape(row["case_id"])}</h2>')
        for record in records:
            if record['path'].startswith('cases/'+row['case_directory']) and record['path'].endswith('.png'):
                lines.append(f'<a href="{record["path"]}"><img src="{record["path"]}" alt="{Path(record["path"]).stem}"></a>')
    lines.append('<h2>Actual Isaac renderer evidence</h2>')
    if gui_record:
        for record in records:
            if record['path'].startswith('gui/') and record['path'].endswith('.png'):lines.append(f'<a href="{record["path"]}"><img src="{record["path"]}" alt="saved Isaac view"></a>')
    else:lines.append('<p>GUI_RUNTIME_NOT_VALIDATED; no screenshot was fabricated.</p>')
    (bundle/'index.html').write_text('\n'.join(lines+['</body></html>']))
    for name in ('README.md','index.html'):records.append(dict(path=name,source='generated review text',sha256=digest(bundle/name),bytes=(bundle/name).stat().st_size))
    write(bundle/'manifest.json',dict(allowlisted_files=records,self_excluded='manifest.json',case_count=len(selected),
        gui_runtime_validated=gui_record is not None,gui_evidence_count=0 if gui_record is None else len(gui_record['captures']),
        excluded=['raw RGB','whole dataset','environment export','checkpoint','solver vectors','external source','videos']))
    with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED) as target:
        for path in sorted(bundle.rglob('*')):
            if path.is_file():target.write(path,str(path.relative_to(bundle)))
    result=dict(bundle_sha256=digest(archive),bundle_bytes=archive.stat().st_size,file_count=len(records)+1,
                gui_runtime_validated=gui_record is not None,packager_sha256=digest(__file__))
    write(run/'review_bundle_manifest.json',result)
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',required=True,type=Path);parser.add_argument('--gui',type=Path)
    args=parser.parse_args();print(json.dumps(package(args.run,args.gui),indent=2))

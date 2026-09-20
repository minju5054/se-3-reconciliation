#!/usr/bin/env python3
"""Small additive review ZIP: PNGs, tables and source/hash links, not curve arrays."""
from __future__ import annotations
import argparse
import html
import json
from pathlib import Path
import shutil
import zipfile
from run_gp_se2_diag06 import verify_frozen,read,write,digest
from plot_gp_se2_diag06 import PLOTS


def package(run):
    verify_frozen(run)
    if not read(run/'validation.json')['valid']:raise ValueError('authoritative validation required')
    folder=run/'compact_review';archive=run/'review_bundle_compact.zip'
    if folder.exists() or archive.exists():raise FileExistsError('no overwrite')
    folder.mkdir();names=['source.json','protocol.json','config_snapshot.yaml','experiment_manifest.json',
        'execution_freeze.json','validation.json','reporting_correction.json','witness_selection/frozen_union.json',
        'witness_selection/validation.json','derivative_checks/validation.json']
    names += [str(p.relative_to(run)) for p in (run/'aggregate').glob('*') if p.is_file()]
    names += ['plots/'+n+'.png' for n in PLOTS]
    manifest={}
    for name in names:
        target=folder/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(run/name,target)
        manifest[name]=dict(source_path=str(run/name),sha256=digest(target))
    sources={n:dict(png_sha256=digest(run/'plots'/(n+'.png')),
        numeric_sidecar_path=str(run/'plots'/(n+'.json')),numeric_sidecar_sha256=digest(run/'plots'/(n+'.json')),
        source_hashes=read(run/'plots'/(n+'.json'))['source_hashes']) for n in PLOTS}
    write(folder/'plot_sources.json',sources);write(folder/'member_sources.json',manifest)
    text=['<!doctype html><html><meta charset="utf-8"><title>GP-SE2-DIAG-06 review</title><style>body{font:16px sans-serif;max-width:1300px;margin:30px auto}img{max-width:100%}pre{white-space:pre-wrap}</style>',
        '<h1>GP-SE2-DIAG-06 · frozen motion witnesses</h1><p>PLAN ONLY / NOT EXECUTED</p>',
        '<p>Three fixed inequalities close observed non-lateral gaps at original tolerance. Hard full recovery remains 0/4: lateral velocity fails. Five G3 solves only; G0/G1/G2 historical. No threshold change, second refinement or execution.</p>',
        '<p>This compact packet contains all ten original PNGs and aggregate tables. Full curve numeric sidecars remain in the primary run; <a href="plot_sources.json">source paths and hashes</a> identify them. No RGB/environment export/checkpoint/historical raw arrays included.</p>',
        '<pre>'+html.escape(json.dumps(read(run/'aggregate/summary.json'),indent=2))+'</pre>']
    for n in PLOTS:text.append(f'<h2>{n}</h2><img src="plots/{n}.png">')
    (folder/'index.html').write_text('\n'.join(text)+'</html>')
    (folder/'README.txt').write_text('Small review packet; primary output and original 50 MB complete-sidecar ZIP are preserved. All copied files are byte-identical. Full numeric curves are linked by absolute source path/hash in plot_sources.json. This diagnostic does not prove continuous feasibility or navigation improvement.\n')
    with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in sorted(folder.rglob('*')):
            if p.is_file():z.write(p,p.relative_to(folder))
    errors=[]
    for name,item in manifest.items():
        if digest(folder/name)!=item['sha256'] or digest(item['source_path'])!=item['sha256']:errors.append(name)
    with zipfile.ZipFile(archive) as z:
        for name in z.namelist():
            if z.read(name)!=(folder/name).read_bytes():errors.append('zip '+name)
    write(run/'compact_review_validation.json',dict(valid=not errors,errors=errors,source_validation_sha256=digest(run/'validation.json'),
        package_script_sha256=digest(__file__),archive_sha256=digest(archive),archive_bytes=archive.stat().st_size,
        numeric_data_changed=False,images_changed=False,new_solves=0))
    print(read(run/'compact_review_validation.json'))
    if errors:raise SystemExit(1)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);package(p.parse_args().run.resolve())

#!/usr/bin/env python3
"""Build a small allowlisted REF-03 review with all events and chosen details."""
from __future__ import annotations
import argparse
import hashlib
import html
import json
from pathlib import Path
import shutil
import zipfile

MAJOR='reference_and_rollout_world'
DETAILS=('goal_yaw_and_position_error','linear_command_vs_time','angular_command_vs_time','clearance_vs_time','selected_progress_and_goal_inclusion')
AGGREGATE=('strata_success_patterns','regressions_and_recoveries','paired_successful_quality','cohort_coverage')
TABLES=('outcome_matrix.csv','success_patterns.csv','regressions.csv','recoveries.csv','paired_metrics.csv',
        'selection_mechanism.csv','timing.csv','duplicate_summary.json','paired_quality_summary.json','summary.json',
        'research_interpretation.json')


def read(path):return json.loads(Path(path).read_text())
def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,value):
    with Path(path).open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')


def allowed_artifacts(run):
    run=Path(run);rows=read(run/'case_manifest.json')['selected'];reps=read(run/'representative_selection.json')
    names=[Path(p) for p in ('source.json','protocol.json','config_snapshot.yaml','experiment_config.yaml','case_manifest.json',
                            'cohort_coverage.json','representative_selection.json','plot_manifest.json')]
    names += [Path('aggregate')/p for p in TABLES]
    for name in AGGREGATE:names += [Path('aggregate/plots')/(name+ext) for ext in ('.png','.json')]
    for row in rows:names += [Path(row['relative_directory'])/'plots'/(MAJOR+ext) for ext in ('.png','.json')]
    selected={r['case_id']:r for r in rows}
    for row in reps['selected']:
        if row['case_id'] not in selected or row['relative_directory']!=selected[row['case_id']]['relative_directory']:
            raise ValueError('representative not in frozen full event manifest')
        names += [Path(row['relative_directory'])/'plots'/(name+ext) for name in DETAILS for ext in ('.png','.json')]
    if len(names)!=len(set(names)) or any(p.is_absolute() or '..' in p.parts for p in names):
        raise ValueError('unsafe or duplicated review allowlist')
    return names


def package(run):
    run=Path(run).resolve();bundle=run/'review_bundle';archive=run/'review_bundle.zip'
    if bundle.exists() or archive.exists():raise FileExistsError('refusing review overwrite')
    paths=allowed_artifacts(run)
    for path in paths:
        if not (run/path).is_file():raise FileNotFoundError(run/path)
    bundle.mkdir();records=[]
    for path in paths:
        source=run/path;target=bundle/path;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
        records.append(dict(path=str(path),source=str(source),sha256=digest(target),bytes=target.stat().st_size))
    rows=read(run/'case_manifest.json')['selected'];representatives=read(run/'representative_selection.json')
    (bundle/'README.md').write_text('''# GP-SE2-REF-03 saved transfer evidence

Open index.html. A uses Native rows and official next-row selection. B uses the dense adapter and official next-row selection. C uses byte-identical B geometry and the previously frozen original-source-progress selector. MPC calculation, physical initial command, separate controller memory, original goal/gates, environment, footprint, clearance and success predicates are preserved. This is offline counterfactual execution, not new VLA inference or GP optimization.

The three known controls are separate from the additional source-selected O/R/P/S strata. The additional cohort excludes previously selected event IDs; it is still from the development corpus, not a held-out benchmark. Source row order is neither distance nor physical time. Missing computation remains N/A. Completed failure remains failure. Exclusive A/B/C success patterns are disjoint; conditional regression/recovery flags overlap and must not be summed. Quality differences are computed only for pairs that both pass full original success, with event/episode/ordered-raw-pair dependence shown separately.

Every frozen event has its main world overlay here. Additional detail figures follow the predeclared representative policy: all new C safety/route regressions relative to A or B first, then hash-first B-failure/C-success and same completed B/C-success-status case per stratum, and all known controls. Unchanged status does not mean identical trajectory. The full local output index contains every detail figure for every case. B/C inputs are drawn once without offsets, and actual execution is distinct from predicted references. Applicable original gates are shown. Every primary time axis spans 0–3 seconds.

PNG sidecars retain exact values and source hashes. This archive excludes raw RGB, binary arrays, the whole dataset, environment exports, external source, checkpoints, videos and virtual environments. Source paths refer to preserved local inputs outside this small review. No GUI runtime validation or general navigation improvement is claimed.
''')
    lines=['<!doctype html><html><head><meta charset="utf-8"><title>REF-03 review</title><style>body{font:16px system-ui;max-width:1550px;margin:25px auto;padding:15px}img{max-width:100%}article{margin:35px 0}</style></head><body>',
        '<h1>OFFLINE SOURCE-PROGRESS TRANSFER DIAGNOSTIC</h1><p>Read README.md for scope, dependence and missing-outcome semantics. Every frozen event has its main overlay; representatives have additional detail. Original safety, motion, goal/dwell and route conditions remain in force.</p><h2>Tables</h2><ul>']
    for path in TABLES:lines.append(f'<li><a href="aggregate/{path}">{html.escape(path)}</a></li>')
    lines.append('</ul>')
    for name in AGGREGATE:lines.append(f'<article><h2>{name.replace("_"," ")}</h2><img src="aggregate/plots/{name}.png"><p><a href="aggregate/plots/{name}.json">Numbers and hashes</a></p></article>')
    rep={r['case_id']:r for r in representatives['selected']}
    for cohort in ('KNOWN_CONTROLS','ADDITIONAL_TRANSFER'):
        lines.append('<h2>'+cohort.replace('_',' ')+'</h2>')
        for row in rows:
            if row['cohort']!=cohort:continue
            base=row['relative_directory'];lines.append(f'<h3>{html.escape(row["case_id"])} · {html.escape(row["case_role"])}</h3>')
            figures=[MAJOR]+(list(DETAILS) if row['case_id'] in rep else [])
            if row['case_id'] in rep:lines.append('<p>Representative: '+html.escape('; '.join(rep[row['case_id']]['reasons']))+'</p>')
            for name in figures:lines.append(f'<article><h4>{name.replace("_"," ")}</h4><img loading="lazy" src="{base}/plots/{name}.png"><p><a href="{base}/plots/{name}.json">Numbers and hashes</a></p></article>')
    (bundle/'index.html').write_text('\n'.join(lines+['</body></html>']))
    for name in ('README.md','index.html'):records.append(dict(path=name,source='generated review text',sha256=digest(bundle/name),bytes=(bundle/name).stat().st_size))
    image_count=sum(r['path'].endswith('.png') for r in records)
    write(bundle/'manifest.json',dict(allowlisted_files=records,case_count=len(rows),representative_count=representatives['count'],image_count=image_count,
        self_excluded='manifest.json',gui_runtime_validated=False,excluded=['raw RGB','binary arrays','full dataset','environment export','external source','checkpoint','videos','venv']))
    with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED) as target:
        for path in sorted(bundle.rglob('*')):
            if path.is_file():target.write(path,str(path.relative_to(bundle)))
    result=dict(bundle_sha256=digest(archive),bundle_bytes=archive.stat().st_size,file_count=len(records)+1,
        image_count=image_count,all_case_world_overlay_count=len(rows),representative_count=representatives['count'],gui_runtime_validated=False,
        packager_sha256=digest(__file__))
    write(run/'review_bundle_manifest.json',result)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',required=True,type=Path)
    print(json.dumps(package(parser.parse_args().run),indent=2))

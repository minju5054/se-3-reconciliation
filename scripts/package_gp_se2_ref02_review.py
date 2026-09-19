#!/usr/bin/env python3
"""Small allowlisted REF-02 review: figures, exact numbers, tables and hashes."""
from __future__ import annotations
import argparse
import hashlib
import html
import json
from pathlib import Path
import shutil
import zipfile

FIGURES = ('fixed_input_world', 'matched_state_targets', 'selected_source_progress_vs_time',
           'selected_target_yaw_vs_time', 'goal_in_horizon_vs_time', 'actual_trajectory_overlay',
           'actual_yaw_and_goal_error', 'linear_command_vs_time', 'angular_command_vs_time',
           'clearance_vs_time', 'outcome_summary')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False); stream.write('\n')


def allowed_artifacts(run):
    run = Path(run)
    relative = [Path(n) for n in ('source.json', 'protocol.json', 'selector_definition.json',
                'config_snapshot.yaml', 'case_manifest.json', 'plot_manifest.json')]
    relative.extend(Path('aggregate')/n for n in ('outcomes.csv', 'paired_comparison.csv', 'selection_mechanism.csv', 'timing.csv', 'summary.json'))
    for case in read(run/'case_manifest.json')['selected']:
        base = Path('cases')/case['case_directory']
        if (run/base/'reproduction_check.json').is_file():
            relative.append(base/'reproduction_check.json')
        for figure in FIGURES:
            relative.extend(base/'plots'/(figure+ext) for ext in ('.png','.json'))
    if len(relative) != len(set(relative)) or any(p.is_absolute() or '..' in p.parts for p in relative):
        raise ValueError('unsafe or duplicated review allowlist')
    return relative


def package(run):
    run = Path(run).resolve(); bundle = run/'review_bundle'; archive = run/'review_bundle.zip'
    if bundle.exists() or archive.exists():
        raise FileExistsError('refusing review bundle overwrite')
    relative = allowed_artifacts(run)
    for p in relative:
        if not (run/p).is_file():
            raise FileNotFoundError(run/p)
    cases = read(run/'case_manifest.json')['selected']
    bundle.mkdir(); records = []
    for path in relative:
        source, target = run/path, bundle/path; target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        records.append(dict(path=str(path), source=str(source), sha256=digest(target), bytes=target.stat().st_size))
    (bundle/'README.md').write_text('''# GP-SE2-REF-02 review bundle

Open index.html. This is a saved offline lookahead-selection diagnostic on two fixed development-corpus events. A uses unchanged Native input and official next-row selection. B uses the fixed dense adapter input and official next-row selection. C uses the byte-identical B input and the predeclared source-progress target selection. The controller objective, gains, horizon, integration, original goal and acceptance predicates remain unchanged.

B and C share one input curve. They are never offset for visibility. Every PNG sidecar records their file hashes, array equality and lineage equality. Original fractional row progress is source identity, not physical time or distance. C requested targets and ceiling overshoot are retained in selection sidecars. Official sequential reference yaw, display-only unwrapped yaw and wrapped original-goal error remain distinct. Representation cuts are not shown as artificial turns.

All method outcomes remain visible. Missing goal times are N/A, not zero or three seconds. Every time axis spans the fixed 0–3 second horizon. Static figures use newly computed independent saved rollouts; matched-state probes contain selection only. This report does not claim a new online episode, GP optimization, or simulator GUI runtime validation.

This archive excludes raw RGB, the full dataset, environment exports, external source, binary arrays, checkpoints and videos. Source paths identify authenticated local inputs outside the archive. These fixed-event observations are not a population estimate or a navigation improvement percentage.
''')
    lines=['<!doctype html><html><head><meta charset="utf-8"><title>REF-02 review</title>',
        '<style>body{font:16px system-ui;max-width:1600px;margin:25px auto;padding:15px}img{max-width:100%}article{margin:35px 0}</style></head><body>',
        '<h1>OFFLINE LOOKAHEAD-SELECTION DIAGNOSTIC</h1>',
        '<p>Two fixed events × three methods. B/C share byte-identical inputs; only target selection differs. See README.md for scope, row-identity and yaw semantics. Saved numerical evidence; no GUI runtime claim.</p><h2>Tables and provenance</h2><ul>']
    for record in records:
        if not record['path'].startswith('cases/'):
            lines.append(f'<li><a href="{record["path"]}">{html.escape(record["path"])}</a></li>')
    lines.append('</ul>')
    for case in cases:
        lines.append('<h2>'+html.escape(case['case_id'])+'</h2>')
        for figure in FIGURES:
            path=f'cases/{case["case_directory"]}/plots/{figure}'
            lines.append(f'<article><h3>{html.escape(figure.replace("_"," "))}</h3><a href="{path}.png"><img loading="lazy" src="{path}.png" alt="{figure}"></a><p><a href="{path}.json">Exact numbers, B/C identity and source/config hashes</a></p></article>')
    (bundle/'index.html').write_text('\n'.join(lines+['</body></html>']))
    for name in ('README.md','index.html'):
        records.append(dict(path=name,source='generated review text',sha256=digest(bundle/name),bytes=(bundle/name).stat().st_size))
    write(bundle/'manifest.json',dict(allowlisted_files=records,case_count=len(cases),image_count=len(cases)*len(FIGURES),
        self_excluded='manifest.json',gui_runtime_validated=False,
        excluded=['raw RGB','full dataset','environment export','external source','binary arrays','checkpoints','videos']))
    with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED) as target:
        for path in sorted(bundle.rglob('*')):
            if path.is_file():target.write(path,str(path.relative_to(bundle)))
    result=dict(bundle_sha256=digest(archive),bundle_bytes=archive.stat().st_size,file_count=len(records)+1,
        image_count=len(cases)*len(FIGURES),gui_runtime_validated=False,packager_sha256=digest(__file__))
    write(run/'review_bundle_manifest.json',result)
    return result


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',required=True,type=Path)
    print(json.dumps(package(parser.parse_args().run),indent=2))

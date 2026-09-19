#!/usr/bin/env python3
"""Compact allowlisted REF-01 figures, numerical sidecars, tables and provenance."""
from __future__ import annotations
import argparse
import hashlib
import html
import json
from pathlib import Path
import shutil
import zipfile

FIGURES = ('input_rows_world', 'input_yaw_vs_original_progress', 'matched_state_reference_targets',
           'selected_original_progress_vs_time', 'selected_reference_yaw_vs_time', 'actual_trajectory_overlay',
           'actual_yaw_and_goal_error', 'linear_command_vs_time', 'angular_command_vs_time',
           'clearance_vs_time', 'outcome_summary')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False); stream.write('\n')


def package(run):
    run = Path(run).resolve(); bundle = run / 'review_bundle'; archive = run / 'review_bundle.zip'
    if bundle.exists() or archive.exists():
        raise FileExistsError('refusing review bundle overwrite')
    relative = [Path(name) for name in ('source.json', 'protocol.json', 'config_snapshot.yaml',
                'case_manifest.json', 'plot_manifest.json')]
    relative.extend(Path('aggregate') / name for name in ('input_audit.csv', 'rollout_outcomes.csv',
                    'factor_contrasts.csv', 'selection_diagnostics.csv', 'summary.json'))
    cases = read(run / 'case_manifest.json')['selected']
    for case in cases:
        base = Path('cases') / case['case_directory']
        relative.extend(base / name for name in ('reference_factorization.json', 'reproduction_check.json'))
        for figure in FIGURES:
            relative.extend(base / 'plots' / (figure+ext) for ext in ('.png', '.json'))
    # Fail before creating a partial archive if any required evidence is absent.
    for path in relative:
        if path.is_absolute() or '..' in path.parts:
            raise ValueError('unsafe bundle relative path')
        if not (run / path).is_file():
            raise FileNotFoundError(run / path)
    bundle.mkdir(); records = []
    for path in relative:
        source = run / path; target = bundle / path; target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        records.append(dict(path=str(path), source=str(source), sha256=digest(target), bytes=target.stat().st_size))
    (bundle / 'README.md').write_text('''# GP-SE2-REF-01 review bundle

Open index.html. This is a factor-isolation diagnostic on two fixed development-corpus events. R00 preserves Native rows; R10 selects the original fixed suffix once; R01 resamples Native; R11 resamples that suffix and reproduces the current adapter. Row count differences are intentional. Original fractional row progress is not physical time or distance.

All eight independent official MPC rollouts use the original B, physical incoming command and separately preserved controller memory. Same-state probes use historical Native input poses without solving or integrating. No new LightNav inference, GP/rigid optimization, controller tuning, goal change or horizon extension was performed. The simulator GUI was not run for this diagnostic; static plots show actual saved numerical results, not renderer screenshots.

Every figure has an adjacent JSON sidecar with exact numbers, units and source/config hashes. Yaw distinctions include raw wrapped yaw, original-branch input yaw, actual display-only continuous yaw, original-goal wrapped error and official sequentially unwrapped reference yaw. Missing goal times remain N/A. Dwell is the original required terminal window, not an achieved-duration measurement. Contrasts are event-specific descriptions, not population significance or navigation improvement percentages.

The archive contains only allowlisted report artifacts. Full environment, source datasets, raw RGB, model weights, external source, binary arrays and videos are excluded. Source paths in hash sidecars identify preserved local inputs outside this archive.
''')
    lines = ['<!doctype html><html><head><meta charset="utf-8"><title>REF-01 review</title>',
        '<style>body{font:16px system-ui;max-width:1400px;margin:25px auto;padding:15px}img{max-width:100%}article{margin:30px 0}</style></head><body>',
        '<h1>OFFLINE REFERENCE-PREPARATION DIAGNOSTIC</h1>',
        '<p>Two fixed events × four input variants. Saved independent rollouts and same-state selection probes. See README.md for scope and yaw/row-time semantics. GUI runtime not validated for this diagnostic.</p><h2>Tables and provenance</h2><ul>']
    for record in records:
        if not record['path'].startswith('cases/'):
            lines.append(f'<li><a href="{record["path"]}">{html.escape(record["path"])}</a></li>')
    lines.append('</ul>')
    for case in cases:
        lines.append(f'<h2>{html.escape(case["case_id"])}</h2>')
        for name in FIGURES:
            path = f'cases/{case["case_directory"]}/plots/{name}'
            lines.append(f'<article><h3>{html.escape(name.replace("_", " "))}</h3><a href="{path}.png"><img loading="lazy" src="{path}.png" alt="{name}"></a><p><a href="{path}.json">Exact plotted numbers and source hashes</a></p></article>')
    (bundle / 'index.html').write_text('\n'.join(lines+['</body></html>']))
    for name in ('README.md', 'index.html'):
        records.append(dict(path=name, source='generated review text', sha256=digest(bundle / name), bytes=(bundle / name).stat().st_size))
    write(bundle / 'manifest.json', dict(allowlisted_files=records, case_count=len(cases), image_count=len(cases)*len(FIGURES),
        self_excluded='manifest.json', gui_runtime_validated=False,
        excluded=['full environment', 'source dataset', 'raw RGB', 'model weights', 'external source', 'binary arrays', 'videos']))
    with zipfile.ZipFile(archive, 'x', compression=zipfile.ZIP_DEFLATED) as target:
        for path in sorted(bundle.rglob('*')):
            if path.is_file():
                target.write(path, str(path.relative_to(bundle)))
    result = dict(bundle_sha256=digest(archive), bundle_bytes=archive.stat().st_size,
                  file_count=len(records)+1, image_count=len(cases)*len(FIGURES),
                  gui_runtime_validated=False, packager_sha256=digest(__file__))
    write(run / 'review_bundle_manifest.json', result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('--run', required=True, type=Path)
    print(json.dumps(package(parser.parse_args().run), indent=2))

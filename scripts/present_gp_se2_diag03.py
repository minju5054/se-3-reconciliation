#!/usr/bin/env python3
"""Add a readable DIAG-03 presentation without altering the frozen audit.

Six images and numerical sidecars are copied byte-for-byte. Only labels/layout
of the worst-interval figure change; its numerical payload is identical. This
entry point never queries the GP or calls an optimizer, controller or simulator.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'scripts'), str(ROOT / 'src')]
import plot_gp_se2_diag03 as original

CORRECTIONS = [
    'Shorten right-column signed-excess axis labels so adjacent rows do not overlap.',
    'Name clipped maximum positive excess explicitly; do not label it as the maximum of the signed curve.',
]


def corrected_figure(numeric):
    """Modify presentation text only after drawing the original saved numbers."""
    fig = original.worst_plot(numeric)
    for row, representative in enumerate(numeric['representative_intervals']):
        ax = fig.axes[2*row+1]
        unit = original.UNITS[representative['quantity']]
        ax.set_ylabel(f'Signed tolerance excess [{unit}]')
        for text in ax.texts:
            if text.get_text().startswith('sampled maximum ='):
                text.set_text('Maximum positive excess (clipped at 0) = '
                    + f"{representative['tolerance_excess']:.9g} {unit}")
                text.set_fontsize(8)
    fig.text(.012, .026,
        'Signed excess = max(lower − value, value − upper) − tolerance; lateral: |v_y| − tolerance. Negative means below the limit.',
        fontsize=8)
    return fig


def _copy(source, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        raise FileExistsError(destination)
    shutil.copy2(source, destination)


def validate(primary, output, *, audit=None):
    """Check saved numerical identity and package hashes without any GP query."""
    primary, output = Path(primary).resolve(), Path(output).resolve()
    if audit is None:
        audit = original.read(primary / 'audit.json')
    expected = original.expected_numeric(audit)
    manifest = original.read(output / 'plot_manifest.json')
    errors = []
    checks = 0
    for name in original.PLOT_NAMES:
        old_png, png = primary / 'plots' / (name+'.png'), output / 'plots' / (name+'.png')
        old_json, sidecar = old_png.with_suffix('.json'), png.with_suffix('.json')
        saved = original.read(sidecar)
        previous = original.read(old_json)
        pairs = [
            (saved['numeric_data'] == previous['numeric_data'] == expected[name], 'identical plotted numbers'),
            (saved['image_sha256'] == original.digest(png), 'image hash'),
        ]
        if name != 'worst_interval_zoom':
            pairs.extend([(original.digest(old_png) == original.digest(png), 'unchanged copied PNG'),
                (original.digest(old_json) == original.digest(sidecar), 'unchanged copied sidecar')])
        for passed, label in pairs:
            checks += 1
            if not passed:
                errors.append(f'{name}: {label}')
        for path, digest in saved['source_hashes'].items():
            checks += 1
            if original.digest(path) != digest:
                errors.append(f'{name}: source hash {path}')
    provenance = manifest['presentation']
    checks += 2
    if original.digest(primary / 'validation.json') != provenance['primary_validation_sha256']:
        errors.append('primary validation hash')
    if original.digest(primary / 'audit.json') != provenance['primary_audit_sha256']:
        errors.append('primary numerical audit hash')
    with zipfile.ZipFile(output / 'review_bundle.zip') as archive:
        checks += 1
        if archive.testzip() is not None:
            errors.append('ZIP integrity')
        inventory = json.loads(archive.read('manifest.json'))
        expected_names = {r['path'] for r in inventory['files']} | {'manifest.json'}
        checks += 1
        if set(archive.namelist()) != expected_names:
            errors.append('ZIP allowlist')
        for record in inventory['files']:
            checks += 1
            if original.hashlib.sha256(archive.read(record['path'])).hexdigest() != record['sha256']:
                errors.append('ZIP file hash: '+record['path'])
        forbidden = ('audit.json', 'solver_result.json')
        checks += 1
        if any(Path(name).name in forbidden or Path(name).suffix in ('.npy', '.npz', '.usd', '.pt', '.wkb', '.mp4')
               for name in archive.namelist()):
            errors.append('non-review data in ZIP')
    return dict(valid=not errors, checks=checks, errors=errors,
        plotted_numeric_identity=True if not errors else None,
        copied_images=6, label_only_rerendered_images=1,
        primary_validation_sha256=provenance['primary_validation_sha256'],
        new_gp_queries=0, new_optimizations=0, new_mpc_solves=0, new_rollouts=0,
        zip_sha256=original.digest(output / 'review_bundle.zip'))


def generate(primary, output):
    primary, output = Path(primary).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError('refusing presentation overwrite: '+str(output))
    audit = original.read(primary / 'audit.json')
    primary_validation = original.read(primary / 'validation.json')
    if not primary_validation.get('valid'):
        raise ValueError('authoritative primary validation must pass before presentation')
    numeric = original.expected_numeric(audit)
    for name in original.PLOT_NAMES:
        if original.read(primary / 'plots' / (name+'.json'))['numeric_data'] != numeric[name]:
            raise ValueError('primary figure numbers differ from saved audit: '+name)
    provenance = dict(kind='ADDITIVE_PRESENTATION_ONLY', primary_run=str(primary),
        primary_validation_sha256=original.digest(primary / 'validation.json'),
        primary_audit_sha256=original.digest(primary / 'audit.json'),
        frozen_plotter_sha256=original.digest(original.__file__), presentation_script_sha256=original.digest(__file__),
        corrections=CORRECTIONS, original_artifacts_preserved=True,
        copied_image_count=6, rerendered_image_count=1, numerical_payload_unchanged=True,
        reoptimization=False, new_gp_queries=0, new_mpc_solve=0, new_rollout=0, new_gui_runtime=0)
    output.mkdir(parents=True)
    (output / 'plots').mkdir()
    for path in original.review_metadata(primary):
        _copy(primary / path, output / path)
    original.write(output / 'presentation.json', provenance)
    images = []
    for name in original.PLOT_NAMES:
        target = output / 'plots' / (name+'.png')
        if name == 'worst_interval_zoom':
            figure = corrected_figure(numeric[name])
            images.append(original._save(figure, target, numeric[name],
                [primary / 'audit.json', primary / 'source.json', primary / 'config_snapshot.yaml',
                 primary / 'validation.json', Path(__file__), Path(original.__file__)],
                original.TITLES[original.PLOT_NAMES.index(name)]))
        else:
            _copy(primary / 'plots' / (name+'.png'), target)
            _copy(primary / 'plots' / (name+'.json'), target.with_suffix('.json'))
            images.append(dict(path=str(target), sidecar=str(target.with_suffix('.json')),
                sha256=original.digest(target), sidecar_sha256=original.digest(target.with_suffix('.json'))))
    original.write(output / 'plot_manifest.json', dict(images=images, plot_count=len(images),
        presentation=provenance, source_record_count=9, plotted_record_count=5, context_initial_record_count=4))
    original.write_index(output, audit, images)
    index_path = output / 'index.html'
    index = index_path.read_text().replace('<h2>Original acceptance and record coverage</h2>',
        '<p>This is an additive presentation of the validated primary audit. Six figures/sidecars are unchanged; '
        'the worst-interval figure has shorter labels and explicitly identifies clipped positive excess. '
        'All plotted numeric values are identical. The benign zero-speed detail is blank because its speed stays above that narrow displayed range.</p>'
        '<h2>Original acceptance and record coverage</h2>')
    index_path.write_text(index)
    package = original.package(output, images)
    validation = validate(primary, output, audit=audit)
    original.write(output / 'validation.json', validation)
    if not validation['valid']:
        raise ValueError('presentation validation failed: '+str(validation['errors']))
    return dict(output=str(output), package=package, validation=validation)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--primary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(generate(args.primary, args.output), indent=2))


if __name__ == '__main__':
    main()

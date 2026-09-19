#!/usr/bin/env python3
"""Additive REF-04 legend/colorbar correction; saved numbers remain unchanged."""
from __future__ import annotations
import argparse
from copy import deepcopy
import hashlib
import html
import io
import json
from pathlib import Path
import shutil
import sys
import textwrap
import time
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'scripts')]
import numpy as np
import yaml
import plot_gp_se2_ref04 as frozen
from matplotlib.lines import Line2D
from matplotlib.colors import TwoSlopeNorm

CORRECTED = ('selected_targets_and_predictions', 'predicted_clearance_heatmap', 'prediction_vs_applied_prefix')
UNCHANGED = tuple(n for n in frozen.PLOT_NAMES if n not in CORRECTED)
SCOPE = 'Presentation only: external legends and explicit negative/zero heatmap ticks; numerical arrays, representatives and acceptance unchanged.'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False); stream.write('\n')


def shared_legend(fig, ax, *, selected):
    """One legend outside every data axis; both method colours are explicit."""
    handles, labels = ax.get_legend_handles_labels()
    entries = []
    for handle, label in zip(handles, labels):
        if label.startswith('saved MPC prediction') or label.startswith('saved prediction first Euler'):
            handle = Line2D([], [], color='#555555', ls='--', lw=1.8)
            label = 'saved MPC prediction (method colour)'
        entries.append((handle, label))
    entries += [(Line2D([], [], color=frozen.COLORS[name], lw=2), frozen.SHORT[name]) for name in frozen.METHODS]
    fig.legend([x[0] for x in entries], [x[1] for x in entries], loc='upper center',
        bbox_to_anchor=(.5, .962), ncol=3, fontsize=8, frameon=False)


def corrected_figure(name, numeric, environment):
    """Only styling/layout differs from the frozen seven-figure renderer."""
    plt = frozen.plt
    if name == 'predicted_clearance_heatmap':
        fig, axes = plt.subplots(2, 2, figsize=(13, 11), squeeze=False)
        values = [v for method in frozen.METHODS for row in numeric['methods'][method]
            for kind in ('nodes', 'segments') if (v := frozen._prediction_values(row, kind)) is not None]
        values = np.concatenate(values) if values else np.array([-.01, .01])
        norm = TwoSlopeNorm(vmin=min(float(values.min()), -.01), vcenter=0., vmax=max(float(values.max()), .01))
        cmap = plt.get_cmap('RdYlGn').with_extremes(bad='#cccccc')
        ticks = sorted(set([norm.vmin, 0., norm.vmax] + [x for x in (.05, .1, .2, .3) if x < norm.vmax]))
        for mi, method in enumerate(frozen.METHODS):
            rows = numeric['methods'][method]
            for ki, kind in enumerate(('nodes', 'segments')):
                ax = axes[mi, ki]; size = 6 if kind == 'nodes' else 5
                matrix = np.full((len(rows), size), np.nan)
                for i, row in enumerate(rows):
                    v = frozen._prediction_values(row, kind)
                    if v is not None:
                        if len(v) != size: raise ValueError('prediction shape changed')
                        matrix[i] = v
                extent = (-.05, .55, 2.95, -.05) if kind == 'nodes' else (0., .5, 2.95, -.05)
                im = ax.imshow(np.ma.masked_invalid(matrix), extent=extent, aspect='auto', cmap=cmap, norm=norm, interpolation='nearest')
                ax.axvline(.1, ls='--', color='black', lw=1.1)
                ax.set(xlabel='offset from prediction issue [s]', ylabel='saved prediction issue time [s]',
                    title=frozen.SHORT[method] + (' | nodes' if kind == 'nodes' else ' | swept segment minimum'))
                ax.text(.02, -.14, 'First 0.1 s | later tail; grey = N/A', transform=ax.transAxes, fontsize=8)
                bar = fig.colorbar(im, ax=ax, label='clearance − effective threshold [m]', shrink=.85, ticks=ticks)
                bar.ax.set_yticklabels([f'{x:.4f}' if x < 0 else f'{x:.3f}' for x in ticks])
        fig.text(.5, .945, 'Negative margin = clearance violation; zero is the effective threshold', ha='center', fontsize=10)
        return fig, (0., .055, 1., .91)
    reps = [r for r in numeric['representative_intervals'] if r.get('solve_index') is not None]
    selected = name == 'selected_targets_and_predictions'
    fig, axes = plt.subplots(max(1, len(reps)), 2, figsize=(14, 5.0 * max(1, len(reps)) + 1.3), squeeze=False)
    for ri, rep in enumerate(reps):
        pair = [numeric['methods'][method][ri] for method in frozen.METHODS]
        if selected:
            xy = numeric['geometry']; reference = xy['shared_dense_reference_world']
            parts = [reference, [xy['B_world'], xy['goal_world']]]
            parts += [m['full_execution']['points_world'] for m in xy['methods'].values()]
            bounds = frozen._bounds(parts + [s['prediction'].get('poses_world') for s in pair], padding=.2, minimum_span=1.)
        else:
            pieces = [s['applied_prefix'].get('poses_world') for s in pair]
            pieces += [[s['input_pose_world'], s['applied_prefix'].get('saved_prediction_endpoint')]
                for s in pair if s['applied_prefix'].get('saved_prediction_endpoint') is not None]
            bounds = frozen._bounds(pieces, padding=.012, minimum_span=.05)
        for mi, (method, solve) in enumerate(zip(frozen.METHODS, pair)):
            ax = axes[ri, mi]; frozen._background(ax, environment, bounds)
            if selected:
                frozen._gates(ax, xy['required_gates'])
                frozen._xy_line(ax, reference, label='same stored dense path', color='#aaaaaa', style=':')
                frozen._xy_line(ax, solve['selected_reference'].get('poses_world'), label='five selected targets', color='#984ea3', style=':', marker='o')
                connector = solve['selected_reference'].get('entry_connector')
                frozen._xy_line(ax, None if connector is None else connector.get('points_world'),
                    label='current-to-first target (diagnostic)', color='#a6761d', style=':', width=1.8)
                frozen._xy_line(ax, solve['prediction'].get('poses_world') if solve['prediction'].get('available') else None,
                    label='saved MPC prediction', color=frozen.COLORS[method], style='--', marker='.')
                frozen._xy_line(ax, solve['applied_prefix'].get('poses_world'), label='actual applied 0.1 s prefix', color='black', width=3.)
                ax.scatter(*solve['input_pose_world'][:2], color='black', s=25, marker='x', label='actual solve input')
            else:
                pre = solve['applied_prefix']; endpoint = pre.get('saved_prediction_endpoint')
                frozen._xy_line(ax, [solve['input_pose_world'], endpoint] if endpoint is not None else None,
                    label='saved prediction first Euler segment', color=frozen.COLORS[method], style='--')
                frozen._xy_line(ax, pre.get('poses_world'), label='saved actual applied prefix', color='black', width=2.2)
                for key, marker, color, label in (
                    ('euler_endpoint', 's', '#984ea3', 'Euler endpoint from applied command'),
                    ('exact_endpoint', '+', '#238b45', 'exact held-command endpoint'),
                    ('actual_endpoint', 'x', 'black', 'saved actual endpoint')):
                    point = pre.get(key)
                    if point is not None:
                        ax.scatter(*point[:2], s=60, marker=marker, color=color,
                            facecolors='none' if marker == 's' else color, label=label, zorder=8)
            ax.set_title(f'{frozen.SHORT[method]} | solve {rep["solve_index"]}, t={solve["issue_time_s"]:.1f} s', fontsize=10)
            ax.tick_params(axis='x', labelsize=8)
        if selected:
            axes[ri, 0].text(.01, -.23, '\n'.join(textwrap.wrap('; '.join(rep.get('reasons', [])), 60)), transform=axes[ri, 0].transAxes, fontsize=8)
    if reps:
        shared_legend(fig, axes[0, 0], selected=selected)
    else:
        for ax in axes.flat:
            ax.text(.5, .5, 'N/A: no representative interval', ha='center'); ax.set_axis_off()
    return fig, (0., .05, 1., .865)


def save_corrected(primary, output, name, environment, provenance):
    source = primary / 'plots' / (name + '.json')
    side = read(source); fig, rect = corrected_figure(name, side['numeric_data'], environment)
    fig.suptitle(frozen.TITLES[name], fontsize=12, y=.991)
    fig.text(.015, .014, 'REF-04 presentation correction | same saved numbers | external legend / explicit clearance ticks', fontsize=8)
    fig.tight_layout(rect=rect)
    target = output / 'plots' / (name + '.png')
    fig.savefig(target, dpi=140); frozen.plt.close(fig)
    result = deepcopy(side)
    result.update(image_sha256=digest(target), presentation_only=True, presentation_script_sha256=digest(__file__),
        original_primary_image_sha256=side['image_sha256'], original_primary_sidecar_sha256=digest(source),
        primary_validation_sha256=provenance['primary_validation_sha256'],
        presentation_source_hashes=provenance['primary_authority_hashes'], presentation_change=SCOPE)
    write(target.with_suffix('.json'), result)


def validate(primary, output):
    checks = []; errors = []
    def check(value, message):
        checks.append(message)
        if not value: errors.append(message)
    authority = read(output / 'presentation_source.json')
    for path, sha in authority['primary_authority_hashes'].items():
        check(digest(path) == sha, 'primary authority preserved: ' + path)
    for path in frozen.review_metadata(primary):
        if path.name not in ('index.html', 'plot_manifest.json'):
            check((primary/path).read_bytes() == (output/path).read_bytes(), 'copied metadata/table bytes: ' + str(path))
    check((primary/'validation.json').read_bytes() == (output/'primary_validation.json').read_bytes(), 'primary validation exact copy')
    for name in frozen.PLOT_NAMES:
        original = primary / 'plots' / (name + '.png'); image = output / 'plots' / (name + '.png')
        before, after = read(original.with_suffix('.json')), read(image.with_suffix('.json'))
        check(before['numeric_data'] == after['numeric_data'], 'exact numeric sidecar equality: ' + name)
        check(after['image_sha256'] == digest(image), 'image/sidecar hash: ' + name)
        check(before['source_hashes'] == after['source_hashes'], 'original source keys preserved: ' + name)
        if name in UNCHANGED:
            check(original.read_bytes() == image.read_bytes(), 'unchanged image bytes: ' + name)
            check(original.with_suffix('.json').read_bytes() == image.with_suffix('.json').read_bytes(), 'unchanged sidecar bytes: ' + name)
        else:
            check(after['primary_validation_sha256'] == authority['primary_validation_sha256'], 'corrected image validation provenance: ' + name)
    return dict(valid=not errors, errors=errors, checks=len(checks), presentation_only=True,
        corrected_figures=list(CORRECTED), unchanged_figures=list(UNCHANGED),
        numerical_audit_recomputed=False, new_MPC_solves=0, new_rollouts=0,
        primary_validation_sha256=authority['primary_validation_sha256'])


def zip_bytes(output, paths):
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        for path in paths:
            archive.write(output / path, str(path))
    payload = buffer.getvalue()
    with zipfile.ZipFile(io.BytesIO(payload)) as archive:
        if set(archive.namelist()) != set(map(str, paths)):
            raise ValueError('ZIP member coverage mismatch')
        for path in paths:
            if archive.read(str(path)) != (output / path).read_bytes():
                raise ValueError('ZIP member byte mismatch: ' + str(path))
    return payload


def findings(audit):
    """Compact reading guide derived from stored audit fields, never a new checker."""
    def number(value, digits=6):
        return 'N/A' if value is None else f'{value:.{digits}f}'
    lines = ['# REF-04 findings from the preserved audit', '',
        'One saved obstacle stress event; B and C use identical dense reference geometry. No new MPC solve, optimizer or rollout was run.', '',
        '| Method | Original full outcome | Minimum actual footprint clearance [m] |',
        '|---|---|---|']
    for method in frozen.METHODS:
        record = audit['methods'][method]
        outcome = record['original_outcome'].get('primary_success')
        lines.append(f'| {frozen.SHORT[method]} | '+('N/A' if outcome is None else 'PASS' if outcome else 'FAIL')+
            f' | {number(record["full_execution"].get("minimum_clearance_m"), 9)} |')
    c = audit['methods'][frozen.METHODS[1]]
    first = c['per_solve'][0]
    points = [p.get('clearance_valid') for s in c['per_solve'] for p in s['selected_reference'].get('points', [])]
    polylines = [s['selected_reference'].get('target_polyline', {}).get('clearance_valid') for s in c['per_solve']]
    verified = bool(points) and all(x is True for x in points + polylines)
    lines += ['', 'The preserved target-point and target-polyline checks for C are '+('all valid.' if verified else 'available in audit.json; no aggregate claim is inferred here.'),
        'At the first solve, C target-polyline minimum clearance is '+number(first['selected_reference'].get('target_polyline', {}).get('minimum_clearance_m'),9)+' m; '
        'the diagnostic current-to-first-target connector is '+number(first['selected_reference'].get('entry_connector', {}).get('minimum_clearance_m'),9)+' m. '
        'The same saved MPC prediction reaches '+number(first['prediction'].get('geometry', {}).get('minimum_clearance_m') if first['prediction'].get('geometry') else None,9)+' m.']
    warnings = [s for s in c['per_solve'] if s['prediction'].get('prospective_warning')]
    if warnings:
        warning = warnings[0]; geometry = warning['prediction']['geometry']
        lines += ['', 'First prospective prediction warning: issue t='+number(warning['issue_time_s'])+' s; forecast event bracket '+
            str(geometry.get('first_refined_violation_bracket_s') or geometry.get('first_swept_violation_bracket_s'))+' s.']
    full = c['full_execution']
    lines += ['First actual clearance violation bracket: '+str(full.get('first_refined_violation_bracket_s') or full.get('first_swept_violation_bracket_s'))+' s. '
        'Minimum actual clearance occurs at the stored polyline minimum parameter time '+number(full.get('minimum_time_s'))+' s.']
    for crossing in c['gate_crossings']:
        if crossing['classification'] == 'INVALID_INTERVAL_CROSSING':
            margin = crossing.get('interval_margin_m')
            lines.append('C crosses outside the original finite gate center interval at t='+number(crossing.get('crossing_time_s'))+' s; '
                'signed interval margin '+number(None if margin is None else margin*1000,3)+' mm. Footprint/clearance are not subtracted from that interval again.')
            break
    lines += ['', 'The official MPC has no obstacle or gate constraint. Safe selected target geometry does not certify the predicted motion. '
        'Prediction nodes use forward Euler; actual saved first-command motion uses exact held-command unicycle integration. '
        'Future optimized controls and pre-clipping controls were not logged. Later prediction-versus-execution differences also include replanning. '
        'This local diagnosis does not establish navigation improvement or prescribe a tested controller repair.', '',
        'Start with selected_targets_and_predictions, predicted_clearance_heatmap, executed_clearance_timeline and gate_crossing_zoom. '
        'Exact numbers, all records and original failures remain in audit.json and the primary validation.']
    return '\n'.join(lines)+'\n'


def main(primary, output):
    begin = time.perf_counter()
    primary, output = Path(primary).resolve(), Path(output).resolve()
    if output.exists(): raise FileExistsError('refusing presentation output overwrite')
    validation = read(primary / 'validation.json')
    if not validation['valid'] or not validation.get('authoritative', False):
        raise ValueError('primary authoritative validation did not pass')
    authority_paths = [primary / name for name in ('audit.json','protocol.json','validation.json','source.json','config_snapshot.yaml','plot_manifest.json')]
    authority_paths += [primary / 'plots' / (name + suffix) for name in frozen.PLOT_NAMES for suffix in ('.png','.json')]
    hashes = {str(path): digest(path) for path in authority_paths}
    audit, context, route = [read(primary / name) for name in ('audit.json','input_context.json','goal_route.json')]
    config = yaml.safe_load((primary / 'config_snapshot.yaml').read_text())
    reference = np.load(primary / 'reference_world.npy', allow_pickle=False)
    expected = frozen.expected_numeric(audit, context, route, reference, config, past=read(primary / 'actual_past_execution.json'))
    for name in frozen.PLOT_NAMES:
        side = read(primary / 'plots' / (name + '.json'))
        if side['numeric_data'] != frozen.plain(expected[name]): raise ValueError('primary numeric/source mismatch: ' + name)
        if side['image_sha256'] != digest(primary / 'plots' / (name + '.png')): raise ValueError('primary image hash mismatch')
    output.mkdir(); (output / 'plots').mkdir()
    provenance = dict(scope=SCOPE, primary_run=str(primary), primary_authority_hashes=hashes,
        primary_validation_sha256=digest(primary / 'validation.json'),
        primary_audit_sha256=digest(primary / 'audit.json'), primary_protocol_sha256=digest(primary / 'protocol.json'),
        frozen_plotter_sha256=digest(frozen.__file__), presentation_script_sha256=digest(__file__),
        corrected_figures=list(CORRECTED), unchanged_figures=list(UNCHANGED),
        new_MPC_solves=0, new_rollouts=0, numerical_audit_recomputed=False)
    write(output / 'presentation_source.json', provenance)
    environment = frozen.HospitalEnvironment.load(frozen.environment_path(primary))
    for name in frozen.PLOT_NAMES:
        if name in CORRECTED: save_corrected(primary, output, name, environment, provenance)
        else:
            for suffix in ('.png','.json'): shutil.copy2(primary/'plots'/(name+suffix), output/'plots'/(name+suffix))
    rendering_wall_s = time.perf_counter() - begin
    metadata = [p for p in frozen.review_metadata(primary) if p.name not in ('index.html','plot_manifest.json')]
    for path in metadata:
        destination = output / path; destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(primary / path, destination)
    shutil.copy2(primary / 'validation.json', output / 'primary_validation.json')
    manifest = dict(images=[dict(path=f'plots/{name}.png', sha256=digest(output/'plots'/(name+'.png')),
        sidecar_sha256=digest(output/'plots'/(name+'.json')), presentation_changed=name in CORRECTED) for name in frozen.PLOT_NAMES],
        image_count=7, primary_plot_manifest_sha256=digest(primary/'plot_manifest.json'), scope=SCOPE)
    write(output / 'plot_manifest.json', manifest)
    index = (primary / 'index.html').read_text()
    banner = '<p><strong>Presentation-only correction.</strong> The same seven scientific figures and exact plotted numbers are retained. Three figures have external legends or explicit negative clearance ticks; four are byte-identical copies. No audit, solve or rollout was repeated.</p><p><a href="FINDINGS.md">Compact findings</a> · <a href="presentation_source.json">Presentation provenance</a> · <a href="primary_validation.json">Primary authoritative validation</a> · <a href="validation.json">Presentation validation</a></p>'
    index = index.replace('<h1>Saved MPC prediction / execution audit</h1>', '<h1>Saved MPC prediction / execution audit</h1>'+banner)
    (output / 'index.html').write_text(index)
    (output / 'README.md').write_text('# REF-04 presentation correction\n\n'+SCOPE+'\n\nOpen index.html. Primary scientific output remains at '+str(primary)+'. The primary audit/protocol/validation hashes are in presentation_source.json. All core tables and the saved audit are copied unchanged. No source arrays, RGB, environment export, external source, or model checkpoints are in the review ZIP.\n')
    (output / 'FINDINGS.md').write_text(findings(audit))
    write(output / 'presentation_timing.json', dict(setup_numeric_match_and_render_wall_s=rendering_wall_s,
        before_packaging_wall_s=time.perf_counter()-begin, new_MPC_solves=0, new_rollouts=0))
    report = validate(primary, output)
    if not report['valid']: raise ValueError(report)
    files = [p.relative_to(output) for p in sorted(output.rglob('*')) if p.is_file()]
    zip_bytes(output, files)
    report['ZIP_member_bytes_verified'] = True
    report['primary_authority_preserved_after_render'] = all(digest(path)==sha for path,sha in hashes.items())
    if not report['primary_authority_preserved_after_render']: raise ValueError('primary source changed')
    write(output / 'validation.json', report)
    files.append(Path('validation.json'))
    payload = zip_bytes(output, files)
    with (output / 'review_bundle.zip').open('xb') as stream: stream.write(payload)
    result = dict(bundle_sha256=hashlib.sha256(payload).hexdigest(), bundle_bytes=len(payload), file_count=len(files), image_count=7,
        presentation_entrypoint_wall_s=time.perf_counter()-begin,
        primary_validation_sha256=provenance['primary_validation_sha256'], presentation_validation=report)
    write(output / 'review_bundle_manifest.json', result)
    return result


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--primary', type=Path, required=True); parser.add_argument('--output', type=Path, required=True)
    print(json.dumps(main(**vars(parser.parse_args())), indent=2))

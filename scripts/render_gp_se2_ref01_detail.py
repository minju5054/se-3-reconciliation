#!/usr/bin/env python3
"""Additive, rendering-only REF-01 row-identity detail; primary artifacts immutable.

Use a new presentation run directory. No numerical evaluation, controller solve,
reference generation, or changes to the frozen primary plotter are performed.
"""
from __future__ import annotations
import argparse
import html
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from reconciliation.gp_se2_diagnostics import file_sha256
from reconciliation.gp_se2_environment import HospitalEnvironment
from plot_gp_se2_ref01 import (LABEL, VARIANTS, SHORT, COLORS, STYLES, read, write,
    load_variants, base_xy, environment_path, arrows)


def hash_files(paths):
    return {str(Path(p).resolve()):file_sha256(p) for p in paths}


def table_rows(provenance):
    """Display formatting only; exact values remain in the adjacent JSON."""
    return [[str(r['derived_row_index']), f"{r['original_fractional_row_coordinate']:.4f}",
             f"{r['original_left_row_index']}/{r['original_right_row_index']}",
             f"{r['interpolation_alpha']:.4f}", f"{r['world_xy'][0]:.6f}", f"{r['world_xy'][1]:.6f}",
             f"{np.rad2deg(r['wrapped_yaw']):.4f}", f"{np.rad2deg(r['unwrapped_yaw']):.4f}"] for r in provenance]


def main(source_run, output):
    source_run, output = Path(source_run).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError('presentation output must be a new run ID; no overwrite')
    if output == source_run or source_run.is_relative_to(output):
        raise ValueError('presentation run must not contain the primary run')
    manifest = read(source_run/'case_manifest.json')
    primary_manifest = read(source_run/'plot_manifest.json')
    if primary_manifest['experiment'] != 'GP-SE2-REF-01':
        raise ValueError('wrong primary experiment')
    config = yaml.safe_load((source_run/'config_snapshot.yaml').read_text())
    env = HospitalEnvironment.load(environment_path(source_run))
    preserved = [source_run/'plot_manifest.json', source_run/'index.html']
    preserved.extend(source_run/r['path'] for r in primary_manifest['images'])
    preserved.extend((source_run/r['path']).with_suffix('.json') for r in primary_manifest['images'])
    preserved_hashes = hash_files(preserved)
    output.mkdir(parents=True)
    inputs = [source_run/p for p in ('source.json', 'protocol.json', 'config_snapshot.yaml', 'case_manifest.json', 'plot_manifest.json')]
    images = []
    for row in manifest['selected']:
        case = source_run/'cases'/row['case_directory']
        variants = load_variants(case)
        original_sidecar = case/'plots/input_rows_world.json'
        xy = read(original_sidecar)['numeric_data']
        # Same original XYZ poses, equal metric scales; only the displayed extent changes.
        ref_arrays = [i['reference'] for i in variants.values()]
        # Input-only detail uses one shared metre extent for all four variants.
        cloud = np.vstack(ref_arrays)[:, :2]
        center = (cloud.min(0)+cloud.max(0))/2
        side = max(float(np.ptp(cloud, axis=0).max())*1.18, .025)
        zoom = [center[0]-side/2, center[0]+side/2, center[1]-side/2, center[1]+side/2]
        case_out = output/'cases'/row['case_directory']; case_out.mkdir(parents=True)
        for name in VARIANTS:
            item = variants[name]; ref = item['reference']; line = item['provenance']; color = COLORS[name]
            fig = plt.figure(figsize=(24, 11))
            grid = fig.add_gridspec(1, 3, width_ratios=[1, 1, 1.75], left=.035, right=.99, bottom=.1, top=.87, wspace=.20)
            context_ax, detail_ax, table_ax = [fig.add_subplot(grid[0, i]) for i in range(3)]
            base_xy(context_ax, xy, env, xy['axes_world_m'], config)
            context_ax.plot(ref[:, 0], ref[:, 1], STYLES[name], color=color, marker='o', ms=3, lw=1)
            arrows(context_ax, ref, color, .07)
            context_ax.set_title('Unchanged world context\nOLD/FRESH, past through B, footprint and goal', fontsize=10)
            context_ax.legend(fontsize=7, loc='lower left')
            # The zoom is exclusively input geometry; the left panel provides the
            # environment and original B. Direction glyphs have declared metric length.
            native = np.asarray(xy['fresh_world'])
            detail_ax.plot(native[:, 0], native[:, 1], '--', color='#aaaaaa', lw=1, label='original FRESH context')
            detail_ax.plot(ref[:, 0], ref[:, 1], STYLES[name], color=color, marker='o', ms=4, lw=1.1, label=SHORT[name])
            arrow_length = side*.075
            arrows(detail_ax, ref, color, arrow_length)
            detail_ax.scatter(*ref[0, :2], facecolors='none', edgecolors=color, s=90, zorder=9)
            detail_ax.scatter(*ref[-1, :2], color=color, marker='*', s=90, zorder=9)
            detail_ax.annotate('first #0', ref[0, :2], xytext=(7, -16), textcoords='offset points', fontsize=8)
            detail_ax.annotate(f'last #{len(ref)-1}', ref[-1, :2], xytext=(7, 12), textcoords='offset points', fontsize=8)
            first_source = min(r['original_fractional_row_coordinate'] for r in line)
            removed = native[:int(first_source)]
            if len(removed):
                detail_ax.scatter(removed[:, 0], removed[:, 1], color='#666666', marker='x', s=70, label='excluded prefix; context only')
            detail_ax.set(xlim=zoom[:2], ylim=zoom[2:], xlabel='world x [m]', ylabel='world y [m]', aspect='equal')
            detail_ax.grid(alpha=.2)
            detail_ax.ticklabel_format(useOffset=False, axis='both')
            detail_ax.set_title(f'Same input-only zoom for every variant\nEqual aspect; yaw glyph length {arrow_length:.4f} m (direction only)', fontsize=10)
            detail_ax.legend(fontsize=7, loc='lower left')
            table_ax.axis('off')
            displayed = table_rows(line)
            table = table_ax.table(cellText=displayed,
                colLabels=['Row #', 'Source s', 'Left/right', 'Alpha', 'World x [m]', 'World y [m]', 'Yaw [deg]', 'Unwrapped [deg]'],
                cellLoc='right', colLoc='center', loc='center', colWidths=[.075, .105, .10, .105, .15, .15, .14, .175])
            table.auto_set_font_size(False); table.set_fontsize(7.5)
            table.scale(1, 1.45)
            for (r, c), cell in table.get_celld().items():
                if r == 0:
                    cell.set_facecolor('#e8edf1'); cell.set_text_props(weight='bold')
                elif r % 2 == 0:
                    cell.set_facecolor('#f7f7f7')
            table_ax.set_title('Every row identity shown separately; duplicate XY/yaw retained\ns = original fractional row order, not physical time or distance', fontsize=10, pad=18)
            fig.suptitle(f'{row["case_id"]} | {SHORT[name]} | {len(ref)} unchanged input rows', fontsize=17, y=.96)
            fig.text(.035, .925, 'PRESENTATION CORRECTION ONLY: clearer layout and full row table; primary references, rollouts and results unchanged.', fontsize=11)
            fig.text(.035, .035, LABEL+' | Original primary figures remain preserved. Exact lineage, XY and yaw values are in the JSON sidecar.', fontsize=9)
            target = case_out/(name+'_input_detail.png')
            fig.savefig(target, dpi=160); plt.close(fig)
            sources = inputs+[original_sidecar, case/'input_context.json', case/'goal_route.json',
                item['folder']/'reference_world.npy', item['folder']/'row_provenance.json', item['folder']/'geometry_audit.json']
            write(target.with_suffix('.json'), dict(image=target.name, image_sha256=file_sha256(target),
                source_primary_run=str(source_run), source_hashes=hash_files(sources), renderer_sha256=file_sha256(__file__),
                correction='rendering only: independent subplot spacing, common input-only zoom, complete separate row table',
                numeric_data=dict(reference_world=ref.tolist(), row_provenance=line, formatted_table=displayed,
                    context_geometry=xy, context_axes_world_m=xy['axes_world_m'], input_zoom_axes_world_m=zoom,
                    yaw_glyph_length_m=arrow_length, yaw_glyph_semantics='heading direction; not displacement or speed'),
                no_reference_regeneration=True, new_controller_solves=0, numerical_results_changed=False,
                primary_artifacts_overwritten=False, gui_runtime_validated=False))
            images.append(dict(case_id=row['case_id'], variant=name, path=str(target.relative_to(output)),
                               sha256=file_sha256(target), sidecar_sha256=file_sha256(target.with_suffix('.json'))))
    unchanged = preserved_hashes == hash_files(preserved)
    if not unchanged:
        raise ValueError('primary plots changed while rendering supplement')
    write(output/'source.json', dict(experiment='GP-SE2-REF-01-PRESENTATION', primary_run=str(source_run),
        primary_source_hashes=hash_files(inputs), primary_plot_hashes=preserved_hashes, renderer_sha256=file_sha256(__file__)))
    write(output/'correction_provenance.json', dict(kind='ADDITIVE_RENDERING_ONLY_CORRECTION',
        reason='primary input-row figure had subplot-title overlap and clustered row labels',
        preserved_primary=True, preserved_primary_files=len(preserved_hashes), primary_artifacts_overwritten=False,
        scientific_run_repeated=False, new_mpc_solves=0, new_GP_solves=0,
        images=images, gui_runtime_validated=False))
    lines = ['<!doctype html><html><head><meta charset="utf-8"><title>REF-01 row details</title>',
        '<style>body{font:16px system-ui;max-width:1700px;margin:25px auto;padding:15px}img{max-width:100%}article{margin:35px 0}</style></head><body>',
        '<h1>OFFLINE REFERENCE-PREPARATION DIAGNOSTIC — readable input rows</h1>',
        '<p>Additive rendering correction only. Original primary plots and all numerical outcomes are unchanged. The row table resolves dense annotation overlap. Click an image for full resolution.</p>',
        f'<p><a href="../{html.escape(source_run.name)}/index.html">Original primary index: all 22 plots</a> · <a href="correction_provenance.json">Correction and preservation record</a></p>']
    for row in images:
        p = row['path']; lines.append(f'<article><h2>{html.escape(row["case_id"])} — {html.escape(row["variant"])}</h2><a href="{p}"><img src="{p}" alt="readable input rows and full provenance table"></a><p><a href="{str(Path(p).with_suffix(".json"))}">Exact values and hashes</a></p></article>')
    (output/'index.html').write_text('\n'.join(lines+['</body></html>']))
    result = dict(image_count=len(images), primary_files_unchanged=unchanged,
                  presentation_run=str(output), new_controller_solves=0, gui_runtime_validated=False)
    write(output/'validation.json', result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run', required=True, type=Path); parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(); print(json.dumps(main(args.source_run, args.output), indent=2))

#!/usr/bin/env python3
"""Exactly three static scientific PNGs from validated saved diagnostics."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np

from run_spatial_correspondence_selector_diag01 import RESULTS, read, save, sha, METHODS, RULES
from validate_spatial_correspondence_selector_diag01 import validate
from reconciliation.spatial_correspondence_selector import PNGS
from reconciliation.relative_factor_multisource import geometry

COLORS = ['#2369b4', '#c54b89', '#149079']
MARKERS = ['v', '^', '*']
STYLES = ['-', '--', '-.']
LABELS = ['Native', 'Half', 'Full']
RULE_COLORS = ['#929aa5', '#dc941d', '#7354be', '#148078']
RULE_MARKERS = ['o', 'D', '+']


def polygons(ax, shape, **kwargs):
    if shape.is_empty:
        return
    if shape.geom_type == 'Polygon':
        ax.fill(*shape.exterior.xy, **kwargs)
    elif hasattr(shape, 'geoms'):
        for part in shape.geoms:
            polygons(ax, part, **kwargs)


def compact_figures(summary, prepared, out, environments=None):
    out.mkdir(parents=True, exist_ok=False)
    records = []
    plt.rcParams.update({'font.size':10, 'axes.titlesize':12, 'axes.spines.top':False,
                         'axes.spines.right':False, 'font.family':'DejaVu Sans'})
    def finish(fig, name, numbers):
        fig.savefig(out/name, dpi=180, facecolor='white')
        plt.close(fig)
        records.append(dict(file=name, sha256=sha(out/name), numeric_sidecar=numbers))
    def title(i, sid):
        return f'S{i+1} | '+sid.replace('episode_', 'E').replace('_repeat_', ' / R').replace('/handoff_', ' / H')
    method_handles = [Line2D([], [], color=c, marker=m, ls=l, label=n)
                      for c, m, l, n in zip(COLORS, MARKERS, STYLES, LABELS)]
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    numbers = {}
    for i, (ax, spec) in enumerate(zip(axes.flat, prepared['sources'])):
        sid = spec['source_id']
        q = summary['sources'][sid]
        fresh = np.load(spec['original_world'])
        old = np.load(spec['old_path'])
        B = np.asarray(q['B'])
        env = geometry(Path(spec['folder'])) if environments is None else environments[i]
        points = [fresh[:, :2], old[:, :2], B[None, :2]]
        physical = [q['first_official_selection'][n]['H5_start']['physical_target_world'][:2] for n in METHODS]
        points.append(np.array(physical))
        xy = np.vstack(points)
        lo, hi = xy.min(0)-.23, xy.max(0)+.23
        if env['cart'] is not None:
            bounds = np.asarray(env['cart'].bounds)
            lo, hi = np.minimum(lo, bounds[:2]-.29), np.maximum(hi, bounds[2:]+.29)
        centre = (lo+hi)/2
        half = max(hi-lo)/2
        lo, hi = centre-half, centre+half
        from shapely.geometry import box
        clip = box(*lo, *hi)
        polygons(ax, env['base'].obstacles.buffer(.25).intersection(clip), color='#e5d6d6', alpha=.7, zorder=0)
        polygons(ax, env['base'].obstacles.intersection(clip), color='#a4aab2', zorder=1)
        if env['cart'] is not None:
            polygons(ax, env['cart'].buffer(.25).intersection(clip), color='#e5baba', alpha=.7, zorder=1)
            polygons(ax, env['cart'].intersection(clip), color='#737981', zorder=2)
        ax.plot(*old[:, :2].T, color='#858b94', ls=':', lw=2.5, zorder=3)
        ax.plot(*fresh[:, :2].T, color='#2c333d', marker='.', ms=6, lw=1.5, zorder=4)
        for j, pose in enumerate(fresh):
            ax.annotate(str(j), pose[:2], xytext=(6, 1), textcoords='offset points', fontsize=8, color='#535962')
        ax.scatter(*B[:2], c='black', marker='P', s=100, zorder=8)
        ax.annotate('B', B[:2], xytext=(-14, -15), textcoords='offset points', weight='bold')
        for j, rule in enumerate(RULES[1:]):
            pose = q['correspondences'][rule]['target_world']
            # Nested open markers remain visible if continuous correspondences coincide.
            if j == 2:
                ax.scatter(*pose[:2], color=RULE_COLORS[j+1], marker='+', s=205, linewidths=2, zorder=12)
            else:
                ax.scatter(*pose[:2], edgecolors=RULE_COLORS[j+1], facecolors='none',
                           marker=RULE_MARKERS[j], s=[250, 125][j], linewidths=2, zorder=10+j)
        for j, pose in enumerate(physical):
            ax.scatter(*pose, edgecolors=COLORS[j], facecolors='none', marker=MARKERS[j],
                       s=[260, 135, 95][j], linewidths=1.9, zorder=13+j)
        notes = []
        for j, rule in enumerate(RULES[1:], 1):
            r = q['correspondences'][rule]
            notes.append(f"C{j}: arc {r['arc_m']:.3f} m | left {r['remaining_arc_m']:.3f} m")
        identities = [q['first_official_selection'][n]['H5_start']['original_row_identity'] for n in METHODS]
        tick = q['first_official_selection'][METHODS[0]]['query_tick']
        notes.append(f'First H5 row N/H/F: {identities[0]} / {identities[1]} / {identities[2]} (tick {tick})')
        ax.text(.015, .02, '\n'.join(notes), transform=ax.transAxes, fontsize=9,
                bbox=dict(facecolor='white', edgecolor='#d3d7dd', alpha=.94, boxstyle='round,pad=.4'))
        ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), xlabel='World X [m]', ylabel='World Y [m]', title=title(i, sid))
        ax.set_aspect('equal', adjustable='box')
        ax.grid(alpha=.18)
        numbers[sid] = dict(original_world=fresh.tolist(), old_world=old.tolist(), B=B.tolist(),
            correspondences=q['correspondences'], first_official=q['first_official_selection'])
    handles = [Line2D([], [], color='#2c333d', marker='.', label='Original FRESH (row IDs)'),
               Line2D([], [], color='#858b94', ls=':', label='Recorded OLD to B'),
               Line2D([], [], color='black', ls='', marker='P', label='B')]
    handles += [Line2D([], [], color=c, marker=m, mfc='none', ls='', label=n) for c, m, n in
                zip(RULE_COLORS[1:], RULE_MARKERS, ['C1 XY', 'C2 SE2', 'C3 forward SE2'])]
    handles += [Line2D([], [], color=c, marker=m, mfc='none', ls='', label=n+' first physical H5 target')
                for c, m, n in zip(COLORS, MARKERS, LABELS)]
    handles += [Patch(facecolor='#e5d6d6', label='Geometry + 0.25 m centre exclusion')]
    fig.legend(handles=handles, loc='lower center', ncol=3, fontsize=9, bbox_to_anchor=(.5, .005))
    fig.suptitle('Where does B meet original FRESH?', fontsize=19, y=.99)
    fig.text(.5, .951, 'Continuous targets at B; official targets at the first saved Native submit. No new execution.', ha='center')
    fig.subplots_adjust(top=.91, bottom=.17, hspace=.26, wspace=.25)
    finish(fig, PNGS[0], numbers)

    fig, axes = plt.subplots(2, 2, figsize=(15, 9))
    numbers = {}
    for i, (ax, (sid, q)) in enumerate(zip(axes.flat, summary['sources'].items())):
        numbers[sid] = {}
        for j, method in enumerate(METHODS):
            rows = [r for r in q['matched_selector'] if r['method'] == method]
            ticks = [r['tick'] for r in rows]
            arcs = [r['first_arc_m'] for r in rows]
            ax.plot(ticks, arcs, color=COLORS[j], ls=STYLES[j], marker=MARKERS[j],
                    ms=[10, 7, 5][j], mfc='none', lw=[3., 2., 1.5][j], label=LABELS[j])
            numbers[sid][method] = dict(ticks=ticks, first_original_arc_m=arcs,
                                       first_original_rows=[r['first_row'] for r in rows])
        h, f = [q['progress_reset']['methods'][n] for n in METHODS[1:]]
        count = q['progress_reset']['matched_ticks']
        ax.text(.02, .97, f"Earlier than Native: Half {h['negative_first_count']}/{count}; Full {f['negative_first_count']}/{count}\n"
                f"Max backward arc: Half {h['max_backward_first_arc_m']:.3f} m; Full {f['max_backward_first_arc_m']:.3f} m",
                transform=ax.transAxes, va='top', fontsize=9,
                bbox=dict(facecolor='white', alpha=.9, edgecolor='none'))
        y0, y1 = ax.get_ylim()
        ax.set_ylim(max(0, y0-.02), y1+.22*(y1-y0))
        ax.set(title=title(i, sid), xlabel='Saved Native submit tick (absolute)', ylabel='First H5 original arc [m]')
        ax.grid(alpha=.2)
    fig.suptitle('Same robot pose, different selected original progress', fontsize=18)
    fig.legend(handles=method_handles, loc='lower center', ncol=3, bbox_to_anchor=(.5, .018))
    fig.text(.5, .055, 'Negative modified-minus-Native difference means a backward reset. Nested markers expose coincident selections.', ha='center', fontsize=10)
    fig.tight_layout(rect=(0, .10, 1, .95))
    finish(fig, PNGS[1], numbers)

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    numbers = {}
    metrics = [('arc_m', 'Selected original arc [m]', 1),
               ('position_gap_m', 'Position gap from B [m]', 1),
               ('yaw_gap_rad', 'Yaw mismatch from B [deg]', 180/np.pi),
               ('remaining_arc_m', 'Remaining original arc [m]', 1)]
    x = np.arange(4)
    for ax, (key, label, scale) in zip(axes.flat, metrics):
        numbers[key] = {}
        for j, rule in enumerate(RULES):
            values = [q['correspondences'][rule][key]*scale for q in summary['sources'].values()]
            numbers[key][rule] = values
            ax.bar(x+(j-1.5)*.19, values, width=.18, color=RULE_COLORS[j], label=rule.split('_')[0])
        ax.set(xticks=x, xticklabels=['S1', 'S2', 'S3', 'S4'], title=label)
        ax.grid(axis='y', alpha=.18)
    table = []
    for i, q in enumerate(summary['sources'].values()):
        entries = [q['saved_execution_context'][n]['sustained_attachment_s'] for n in METHODS]
        table.append([f'S{i+1}']+[('N/A (no dwell)' if v is None else f'{v:.4f} s') for v in entries])
    table_ax = fig.add_axes([.15, .015, .7, .14])
    table_ax.axis('off')
    t = table_ax.table(cellText=table, colLabels=['Saved attachment only', 'Native', 'Half', 'Full'],
                       cellLoc='center', loc='center')
    t.auto_set_font_size(False)
    t.set_fontsize(9)
    t.scale(1, 1.25)
    for (row, col), cell in t.get_celld().items():
        cell.set_edgecolor('#d4d9e0')
        if row == 0:
            cell.set_facecolor('#e9edf2')
    fig.suptitle('Continuous correspondence rules | geometry, not a new method', fontsize=18)
    fig.legend(handles=[Patch(facecolor=c, label=r.split('_')[0]) for c, r in zip(RULE_COLORS, RULES)],
               loc='upper center', ncol=4, bbox_to_anchor=(.5, .95))
    fig.text(.5, .17, 'C0 is a row-0 conceptual baseline. Existing attachment outcomes are observational context only.', ha='center', fontsize=10)
    fig.subplots_adjust(top=.88, bottom=.23, hspace=.35, wspace=.25)
    numbers['saved_attachment_table'] = table
    finish(fig, PNGS[2], numbers)
    assert sorted(p.name for p in out.iterdir()) == sorted(PNGS)
    return records


def report(run):
    summary, validation = validate(run, deep=False)
    assert read(run/'validation.json')['old_transport_multisource_R00_R01_validators']
    figures = compact_figures(summary, read(run/'prepared.json'), RESULTS/'figures')
    manifest = dict(summary_sha256=sha(run/'summary.json'), figures=figures,
                    final_png_count=3, extra_pngs=0, new_scientific_calls=0)
    save(run/'figure_manifest.json', manifest)
    save(RESULTS/'figure_manifest.json', manifest)
    save(run/'result_hashes.json', {str(p.relative_to(run)):sha(p) for p in run.rglob('*') if p.is_file()})
    save(RESULTS/'result_summary.json', dict(run=str(run), summary=summary, validation=read(run/'validation.json'),
        result_hashes_sha256=sha(run/'result_hashes.json'),
        table_hashes={p.name:sha(p) for p in RESULTS.glob('*.csv')},
        figures=[dict(path=str(RESULTS/'figures'/r['file']), sha256=r['sha256']) for r in figures]))
    print(json.dumps(dict(figures=[str(RESULTS/'figures'/p) for p in PNGS])))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--run', type=Path, required=True)
    args = p.parse_args()
    report(args.run.resolve())

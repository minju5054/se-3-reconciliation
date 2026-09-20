#!/usr/bin/env python3
"""Read-only presentation correction for saved DIAG-04 results; no new trajectory queries."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import FixedLocator, FuncFormatter
import numpy as np

import plot_gp_se2_diag04 as base

REASONS = [
    'Shorten repeated constraint labels and move legends outside the curves.',
    'Nonnegative constraint residuals use nonnegative axes with sparse labelled ticks.',
    'N/A feasible-objective panels retain their paired recorded elapsed-time range.',
    'Labelled symlog preserves small residuals alongside large G1 excursions on common G0/G1 axes.',
]


def _symlog(ax, threshold, *, ticks=7):
    ax.set_yscale('symlog', linthresh=threshold)
    low, high = ax.get_ylim()
    first = int(np.ceil(np.log10(threshold)))
    last = max(first, int(np.floor(np.log10(max(abs(low), abs(high), threshold)))))
    powers = list(range(first, last+1))
    stride = max(1, int(np.ceil((2*len(powers)+1)/ticks)))
    exponents = sorted(set(powers[::stride]+[last]))
    values = [0.] + [sign*10.**e for e in exponents for sign in (-1.,1.) if low <= sign*10.**e <= high]
    ax.yaxis.set_major_locator(FixedLocator(sorted(values)))
    def label(value, position):
        if value == 0.:
            return '0'
        exponent = int(round(np.log10(abs(value))))
        return ('$-' if value < 0 else '$')+'10^{'+str(exponent)+'}$'
    ax.yaxis.set_major_formatter(FuncFormatter(label))


def _remove_legends(fig):
    for ax in fig.axes:
        legend = ax.get_legend()
        if legend is not None:
            legend.remove()
    for legend in list(fig.legends):
        legend.remove()


def _grid_legend(fig, *, tolerance=False, motion=False):
    _remove_legends(fig)
    if motion:
        base._figure_key(fig)
        return
    handles = [Line2D([], [], color=base.COLORS[g], label=g) for g in base.GRIDS]
    if tolerance:
        handles.append(Line2D([], [], color='black', ls='--', label='original numerical tolerance'))
    fig.legend(handles=handles, loc='lower center', bbox_to_anchor=(.5, .035), ncol=len(handles), fontsize=8, frameon=False)


def adjust_figure(fig, name, data):
    """Alter rendering only. No line x/y data or numeric payload is changed."""
    groups = base.pairs(data['solves'])
    cfg = data['formulation_config']
    if name == 'constraint_violation_history':
        for row, (_, solves) in enumerate(groups):
            end = max([h['elapsed_s'] for s in solves for h in s.get('history', [])] or [0.])
            for col, field in enumerate(('equality_residual', 'inequality_violation')):
                ax = fig.axes[row*2+col]
                values = [h[field] for s in solves for h in s.get('history', []) if h.get(field) is not None]
                tolerance = cfg['equality_tolerance' if col == 0 else 'inequality_tolerance']
                ax.set_ylim(0., max([float(v) for v in values]+[tolerance])*1.25)
                _symlog(ax, tolerance, ticks=6)
                ax.set_xlim(0., max(end, 1e-6))
                ax.set_ylabel('max |lateral residual| [m/s]' if col == 0 else 'max inequality violation')
        _grid_legend(fig, tolerance=True)
        fig.text(.01, .073, 'Symlog with a linear region at the original tolerance. Inequality rows retain mixed original units; values are not rescaled.', fontsize=8)
    elif name == 'feasible_objective_history':
        for row, (_, solves) in enumerate(groups):
            end = max([h['elapsed_s'] for s in solves for h in s.get('history', [])] or [0.])
            for ax in fig.axes[row*2:row*2+2]:
                ax.set_xlim(0., max(end, 1e-6))
            values = [h['objective'] for s in solves for h in s.get('history', []) if h.get('objective') is not None]
            positive = [v for v in values if v > 0]
            if positive and max(positive)/min(positive) > 1e5:
                ax = fig.axes[row*2]
                _symlog(ax, 1., ticks=7)
                ax.set_ylim(0., max(positive)*1.2)
                ax.set_ylabel('original objective (symlog)')
        _grid_legend(fig)
        fig.text(.01, .073, 'N/A is not zero objective. Symlog objective panels are linear below 1. Full-feasibility is checked after solving; x is saved discovery time.', fontsize=8)
    elif name in ('lateral_velocity_vs_time', 'linear_acceleration_vs_time', 'angular_acceleration_vs_time'):
        quantity = {'lateral_velocity_vs_time':'v_y', 'linear_acceleration_vs_time':'a_x', 'angular_acceleration_vs_time':'a_omega'}[name]
        nominal = cfg['equality_tolerance'] if quantity == 'v_y' else cfg['a_v_max' if quantity == 'a_x' else 'a_w_max']
        for ax, (_, solves) in zip(fig.axes, groups):
            values = [value for s in solves for _, result, _ in base.visible_results(s)
                for value in base._quantity(result['samples'], quantity)]
            if values and max(np.abs(values)) > 10*nominal:
                threshold = cfg['equality_tolerance'] if quantity == 'v_y' else nominal
                _symlog(ax, threshold, ticks=8)
                ax.set_ylabel(f'{quantity} [{base.UNITS[quantity]}] (symlog)')
        _grid_legend(fig, motion=True)
        fig.text(.01, .073, 'Common G0/G1 axes. Labelled symlog panels retain signed values and a linear region; bound-excess detail is shown in the worst-interval figure.', fontsize=8)
    elif name == 'worst_interval_comparison':
        for row, (key, solves) in enumerate(groups):
            ids = {s['solve_id'] for s in solves}
            for col, quantity in enumerate(base.QUANTITIES):
                ax = fig.axes[row*5+col]
                records = [r for r in data['representatives'] if r['solve_id'] in ids and r['quantity'] == quantity]
                values = [value for r in records for value in r['tolerance_excess']]
                tolerance = cfg['equality_tolerance' if quantity == 'v_y' else 'inequality_tolerance']
                if values and max(np.abs(values)) > 100*tolerance:
                    _symlog(ax, tolerance, ticks=6)
                    ax.set_ylabel(f'signed excess [{base.UNITS[quantity]}] (symlog)', fontsize=8)
                if values:
                    ax.set_ylim(min(min(values)*1.15, -tolerance), max(max(values)*1.15, tolerance))
                legend = ax.get_legend()
                if legend is not None:
                    legend.remove()
                locations = '; '.join(f"{next(s for s in solves if s['solve_id']==r['solve_id'])['grid'][:2]} {r['result']}: i={r['interval_index']}" for r in records)
                ax.set_title(base.pair_label(key)+' · '+quantity+'\n'+locations, fontsize=7)
        _grid_legend(fig)
        fig.text(.01, .073, 'Each result uses its own maximum observed excess interval. Local u is not shared absolute time. Zero is the original acceptance boundary.', fontsize=9)
    elif name == 'solve_cost_and_evaluations':
        legends = []
        for ax in fig.axes[1:]:
            handles, labels = ax.get_legend_handles_labels()
            legends.extend(zip(handles, labels))
        _remove_legends(fig)
        unique = dict((label, handle) for handle, label in legends)
        fig.legend(list(unique.values()), list(unique), loc='lower center', bbox_to_anchor=(.5, .035), ncol=4, fontsize=8, frameon=False)
    elif name == 'equality_singular_values':
        _remove_legends(fig)
        handles = [Line2D([], [], color=base.COLORS[g], label=g) for g in base.GRIDS]
        handles.extend([Line2D([], [], color='black', ls=style, label=role) for role, style in [('initial', ':'), ('latest', '-'), ('selected', '--')]])
        fig.legend(handles=handles, loc='lower center', bbox_to_anchor=(.5, .035), ncol=5, fontsize=8, frameon=False)
    return fig


def primary_inventory(primary):
    names = ['source.json', 'protocol.json', 'config_snapshot.yaml', 'experiment_manifest.json', 'plot_input.json',
        'plot_manifest.json', 'index.html', 'review_bundle.zip']
    paths = [primary/name for name in names]
    paths.extend(sorted((primary/'aggregate').glob('*.csv')))
    paths.extend(sorted((primary/'plots').glob('*.json')))
    paths.extend(sorted((primary/'plots').glob('*.png')))
    if (primary/'aggregate/summary.json').is_file():
        paths.append(primary/'aggregate/summary.json')
    return {str(path.resolve()): base.digest(path) for path in paths}


def _save(fig, path, data, sources, title, dpi):
    if path.exists() or path.with_suffix('.json').exists():
        raise FileExistsError('presentation cannot overwrite figures')
    fig.suptitle(title, fontsize=13, y=.996)
    fig.text(.01, .008, base.LABEL+' | Presentation only; saved numeric results unchanged.', fontsize=8)
    extra = .105 if path.stem != 'world_xy_and_heading' else .035
    fig.tight_layout(rect=(0, extra, 1, .976))
    fig.savefig(path, dpi=dpi)
    plt.close(fig)
    base.write(path.with_suffix('.json'), dict(image=path.name, image_sha256=base.digest(path), numeric_data=data,
        source_hashes={str(p.resolve()):base.digest(p) for p in sources},
        numerical_plotter_sha256=base.digest(base.__file__), presentation_script_sha256=base.digest(__file__),
        presentation_only=True, dpi=dpi, new_queries=0, new_solves=0, new_execution=0,
        rendering_changes=REASONS))
    return dict(path=str(path), sidecar=str(path.with_suffix('.json')), sha256=base.digest(path), sidecar_sha256=base.digest(path.with_suffix('.json')))


def validate(primary, output):
    primary, output = Path(primary).resolve(), Path(output).resolve()
    source = base.read(output/'source.json')
    checks = []
    errors = []
    for path, sha in source['presentation']['primary_artifact_sha256'].items():
        same = Path(path).is_file() and base.digest(path) == sha
        checks.append(dict(check='primary artifact unchanged', path=path, passed=same))
        if not same:
            errors.append('primary changed: '+path)
    for name in base.PLOT_NAMES:
        saved = base.read(primary/'plots'/(name+'.json'))
        presented = base.read(output/'plots'/(name+'.json'))
        same = saved['numeric_data'] == presented['numeric_data']
        checks.append(dict(check='exact plotted payload equality', image=name, passed=same))
        if not same:
            errors.append('numeric payload changed: '+name)
        if presented['image_sha256'] != base.digest(output/'plots'/(name+'.png')):
            errors.append('image hash mismatch: '+name)
    return dict(valid=not errors, errors=errors, checks=checks, images=10, new_queries=0, new_solves=0, new_rollouts=0,
        scope='presentation numeric payload and primary artifact preservation; original numerical validator remains authoritative')


def generate(primary, output, *, environment=None, dpi=160):
    primary, output = Path(primary).resolve(), Path(output).resolve()
    if dpi < 160:
        raise ValueError('at least 160 dpi required')
    if output.exists():
        raise FileExistsError('refusing presentation run overwrite')
    before = primary_inventory(primary)
    audit = base.read(primary/'plot_input.json')
    numeric = base.expected_numeric(audit)
    for name in base.PLOT_NAMES:
        if numeric[name] != base.read(primary/'plots'/(name+'.json'))['numeric_data']:
            raise ValueError('frozen plot implementation/payload mismatch: '+name)
    output.mkdir(parents=True)
    for relative in base.metadata(primary)+[Path('plot_input.json')]:
        if str(relative) == 'source.json':
            continue
        path = output/relative
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(primary/relative, path)
    source = base.read(primary/'source.json')
    source['presentation'] = dict(presentation_only=True, primary_run=str(primary), primary_artifact_sha256=before,
        script_sha256=base.digest(__file__), frozen_plotter_sha256=base.digest(base.__file__), reasons=REASONS,
        new_queries=0, new_optimization=0, new_mpc_solve=0, new_rollout=0, original_evaluation_unchanged=True)
    base.write(output/'source.json', source)
    if environment is None:
        value = source.get('environment_path', source.get('environment'))
        value = value.get('path') if isinstance(value, dict) else value
        path = Path(value)
        environment = base.HospitalEnvironment.load(path if path.is_absolute() else ROOT/path)
    sources = [primary/'plot_input.json', output/'source.json', output/'config_snapshot.yaml', output/'protocol.json', Path(__file__)]
    sources.extend(sorted((output/'aggregate').glob('*.csv')))
    (output/'plots').mkdir()
    plotters = (lambda d:base.world_plot(d,environment), lambda d:base.motion_plot(d,'v_y'),
        lambda d:base.motion_plot(d,'v_x'), lambda d:base.motion_plot(d,'a_x'), lambda d:base.motion_plot(d,'a_omega'),
        base.worst_plot, base.objective_plot, base.constraint_plot, base.costs_plot, base.singular_plot)
    images = []
    for name, title, render in zip(base.PLOT_NAMES, base.TITLES, plotters):
        fig = adjust_figure(render(numeric[name]), name, numeric[name])
        images.append(_save(fig, output/'plots'/(name+'.png'), numeric[name], sources, title, dpi))
    manifest = dict(images=images, plot_count=10, source_solve_count=10, presentation_only=True,
        primary_run=str(primary), presentation_script_sha256=base.digest(__file__),
        new_queries=0,new_solves=0,new_execution=0)
    base.write(output/'plot_manifest.json',manifest)
    base.write_index(output,audit,images)
    # Add a provenance note to this new index before packaging; primary HTML is untouched.
    index = output/'index.html'
    text = index.read_text().replace('<h1>Quarter-point motion constraint refinement</h1>',
        '<h1>Quarter-point motion constraint refinement</h1><p><strong>Presentation-only rendering correction.</strong> '
        'All values are identical to the frozen primary plots. Axis/legend changes are documented in source.json. No solver or trajectory query was rerun.</p>')
    index.write_text(text)
    manifest['review_bundle'] = base.package(output,images)
    validation = validate(primary,output)
    base.write(output/'validation.json',validation)
    if not validation['valid']:
        raise RuntimeError('presentation preservation validation failed')
    return manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--primary',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(generate(args.primary,args.output),indent=2))


if __name__ == '__main__':
    main()

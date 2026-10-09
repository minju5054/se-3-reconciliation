#!/usr/bin/env python3
"""Saved-only label placement for the frozen severity report; no metric changes."""
import argparse
import json
from pathlib import Path
import tempfile
from unittest.mock import patch

from matplotlib.figure import Figure
from matplotlib.text import Annotation

import report_successive_handoff_severity01 as report


def place_labels(fig, path, *args, **kwargs):
    """Adjust annotations only; point coordinates and plotted paths stay exact."""
    if Path(path).name == report.PNGS[0]:
        for ax in fig.axes[:2]:
            labels = [t for t in ax.texts if isinstance(t, Annotation)]
            labels.sort(key=lambda t: (t.xy[1], t.xy[0], t.get_text()))
            # Spread labels vertically in display coordinates, with leader lines.
            fig.canvas.draw()
            last_y = float('-inf')
            for label in labels:
                x, y = ax.transData.transform(label.xy)
                label_y = max(y, last_y + 15 * fig.dpi / 72)
                last_y = label_y
                dx, dy = 24, (label_y-y) * 72 / fig.dpi
                label.set_visible(False)
                ax.annotate(label.get_text(), label.xy, xytext=(dx, dy),
                            textcoords='offset points', fontsize=8,
                            va='center', arrowprops=dict(arrowstyle='-',
                                                       lw=.5, color='#777'))
    else:
        ax = fig.axes[-1]
        ax.get_legend().set_bbox_to_anchor((.5, .78))
        ax.texts[0].set_position((.05, .04))
    return ORIGINAL_SAVEFIG(fig, path, *args, **kwargs)


ORIGINAL_SAVEFIG = Figure.savefig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    args = parser.parse_args()
    run = report.namespace(args.run)
    report.validate(run)
    summary = report.read(report.OUT/'result_summary.json')
    old = report.read(report.OUT/'figure_manifest.json')
    draft = run/'presentation_draft'
    if not draft.exists():
        draft.mkdir()
        report.save(draft/'figure_manifest.json', old)
        for name in report.PNGS:
            (draft/name).write_bytes((report.OUT/'figures'/name).read_bytes())
    cart = report.environments(Path(summary['source']))[2]
    with tempfile.TemporaryDirectory(prefix='severity-layout-') as temp:
        with patch.object(Figure, 'savefig', place_labels):
            manifest = report.draw(summary, report.read(run/'geometry_details.json'),
                                   cart, Path(temp)/'figures')
        assert manifest['numeric_sidecar'] == old['numeric_sidecar']
        for row, name in zip(manifest['files'], report.PNGS):
            final = report.OUT/'figures'/name
            final.write_bytes(Path(row['path']).read_bytes())
            row.update(path=str(final), sha256=report.sha(final))
    manifest['presentation_note'] = (
        'Post-freeze label placement only; frozen reporter and numeric sidecar unchanged.')
    manifest['layout_script_sha256'] = report.sha(__file__)
    # Keep the initial renderer manifest with the ignored draft; replace derived metadata.
    (report.OUT/'figure_manifest.json').write_text(
        json.dumps(manifest, indent=2, allow_nan=False)+'\n')
    print('Two final PNGs: label placement corrected; numeric sidecar exact.')


if __name__ == '__main__':
    main()

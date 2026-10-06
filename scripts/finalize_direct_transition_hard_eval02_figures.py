#!/usr/bin/env python3
"""Saved-only presentation pass: leave frozen reporter and numeric sidecars intact.

Give scatter annotations headroom and alternate coincident labels above/below.
No reference, metric, selection, classification or scientific execution changes.
"""
import argparse
import json
from pathlib import Path
from unittest.mock import patch
from matplotlib.figure import Figure
from matplotlib.text import Annotation
import report_direct_transition_hard_eval02 as report
from reconciliation.spatial_entry_suffix import no_reconciliation_optimizer


def finalize(run):
    original_save = Figure.savefig

    def save_with_label_room(fig, *args, **kwargs):
        for ax in fig.axes:
            labels = [t for t in ax.texts if isinstance(t, Annotation)]
            if labels:
                ax.margins(y=.18)
                for text in labels:
                    _, offset = text.get_position()
                    text.set_position((6, 6 if offset < 10 else -14))
        return original_save(fig, *args, **kwargs)

    summary = report.read(run/'summary.json')
    assert report.read(report.RESULTS/'validation.json')['valid']
    expected = report.figure_numbers(summary)
    with no_reconciliation_optimizer(), patch.object(Figure, 'savefig', save_with_label_room):
        figures = report.render(summary, report.RESULTS/'figures')
    assert all(f['numeric_sidecar'] == expected for f in figures)
    manifest = dict(summary_sha256=report.sha(run/'summary.json'), figures=figures,
        presentation_only='18% vertical scatter padding; alternate coincident label offsets',
        presentation_script_sha256=report.sha(Path(__file__)),
        frozen_reporter_sha256=report.sha(Path(report.__file__)))
    (report.RESULTS/'figure_manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
    print('Three saved-only PNGs; numeric sidecars unchanged; frozen reporter unchanged')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', type=Path, required=True)
    finalize(parser.parse_args().run.resolve())

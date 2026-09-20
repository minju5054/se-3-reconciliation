"""Presentation-only regression fixture; no experimental curves or GP queries."""
from __future__ import annotations

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('diag03_presentation', ROOT/'scripts/present_gp_se2_diag03.py')
p = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p)


def test_short_labels_and_positive_excess_annotation_leave_curves_unchanged(monkeypatch):
    fig, axes = p.original.plt.subplots(1, 2)
    axes[0].plot([0., .1], [-.2, -.1])
    axes[1].plot([0., .1], [-3., -2.5])
    axes[1].set_ylabel('a very long signed residual expression')
    axes[1].text(.1, .9, 'sampled maximum = 0 rad/s')
    before = [(line.get_xdata().copy(), line.get_ydata().copy()) for ax in axes for line in ax.lines]
    monkeypatch.setattr(p.original, 'worst_plot', lambda data: fig)
    result = p.corrected_figure(dict(representative_intervals=[dict(quantity='omega', tolerance_excess=0.)]))
    assert result is fig
    assert axes[1].get_ylabel() == 'Signed tolerance excess [rad/s]'
    assert axes[1].texts[0].get_text() == 'Maximum positive excess (clipped at 0) = 0 rad/s'
    for (x, y), line in zip(before, [line for ax in axes for line in ax.lines]):
        assert (line.get_xdata() == x).all()
        assert (line.get_ydata() == y).all()
    assert any('Negative means below the limit' in t.get_text() for t in fig.texts)
    p.original.plt.close(fig)

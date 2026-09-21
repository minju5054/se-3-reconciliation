#!/usr/bin/env python3
"""Reporting-only second layout; frozen screening/report code stays unchanged.

Uses saved records only. Moves the world legend outside the data axes so the
unchanged off/sham endpoints remain visible. Never overwrites review_v1.
"""
from pathlib import Path
import report_join_source02 as report


def save_with_external_world_legend(fig, path, numbers, source_hashes):
    original_save = _frozen_save
    hashes = dict(source_hashes)
    hashes[str(Path(__file__).resolve())] = report.sha(Path(__file__))
    if path.name == "paired_world.png":
        fig.set_size_inches(12, 7)
        ax = fig.axes[0]
        ax.set_position([.075, .16, .49, .71])
        legend = ax.get_legend()
        legend.set_loc("upper left")
        legend.set_bbox_to_anchor((1.025, 1.0))
        ax.set_title(ax.get_title(), fontsize=11)
    original_save(fig, path, numbers, hashes)


_frozen_save = report.save_figure


if __name__ == "__main__":
    # Local presentation process only; no numerical/runtime callback is touched.
    report.save_figure = save_with_external_world_legend
    report.main()

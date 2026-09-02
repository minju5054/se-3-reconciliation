#!/usr/bin/env python3
"""Open one or more canonical SE(2) trajectory NPY files."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Inspect and plot N x 3 [x, y, yaw] trajectory .npy files without "
            "coordinate conversion, resampling, or yaw unwrapping."
        )
    )
    parser.add_argument("paths", nargs="+", type=Path, help="trajectory .npy file(s)")
    parser.add_argument(
        "--rows",
        type=int,
        default=5,
        help="leading and trailing rows to print (default: 5; 0 disables row output)",
    )
    parser.add_argument("--save", type=Path, help="save the plot (for example, plot.png)")
    parser.add_argument(
        "--csv-dir",
        type=Path,
        help="also export each trajectory to this directory as x,y,yaw CSV",
    )
    parser.add_argument(
        "--no-show", action="store_true", help="do not open the interactive plot window"
    )
    parser.add_argument(
        "--force", action="store_true", help="allow replacing --save/--csv-dir outputs"
    )
    return parser.parse_args()


def _unique_labels(paths: list[Path]) -> list[str]:
    stems = [path.stem for path in paths]
    if len(stems) == len(set(stems)):
        return stems
    return [str(path) for path in paths]


def main() -> int:
    args = parse_args()
    if args.rows < 0:
        raise ValueError("--rows must be non-negative")

    if args.no_show:
        import matplotlib

        matplotlib.use("Agg")

    from reconciliation.trajectory_view import (
        export_trajectory_csv,
        format_trajectory_report,
        load_trajectory_npy,
        plot_trajectories,
    )

    paths = [path.expanduser().resolve() for path in args.paths]
    labels = _unique_labels(paths)
    loaded = [(label, load_trajectory_npy(path)) for label, path in zip(labels, paths)]

    for index, ((_, trajectory), path) in enumerate(zip(loaded, paths)):
        if index:
            print()
        print(format_trajectory_report(path, trajectory, rows=args.rows))

    if args.csv_dir is not None:
        csv_dir = args.csv_dir.expanduser().resolve()
        destinations = [csv_dir / f"{path.stem}.csv" for path in paths]
        if len(destinations) != len(set(destinations)):
            raise ValueError("duplicate input stems would overwrite the same CSV output")
        if not args.force:
            existing = [path for path in destinations if path.exists()]
            if existing:
                raise FileExistsError(f"CSV output already exists: {existing[0]}")
        for destination, (_, trajectory) in zip(destinations, loaded):
            export_trajectory_csv(destination, trajectory, overwrite=args.force)
            print(f"csv: {destination}")

    save_path = args.save.expanduser().resolve() if args.save is not None else None
    if save_path is not None and save_path.exists() and not args.force:
        raise FileExistsError(f"plot output already exists: {save_path}")

    figure = plot_trajectories(loaded)
    if save_path is not None:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(save_path, dpi=160)
        print(f"plot: {save_path}")
    if not args.no_show:
        from matplotlib import pyplot as plt

        plt.show()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2) from error

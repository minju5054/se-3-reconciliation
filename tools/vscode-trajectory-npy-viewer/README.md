# SE(2) Trajectory NPY Graph Viewer

This repository-local VS Code extension opens a canonical finite `N x 3 [x, y, yaw]` NPY
file as the exact XY/yaw Matplotlib graph produced by `scripts/view_trajectory_npy.py`.

It is a read-only custom editor. It does not parse, transform, interpolate, resample, or
overwrite trajectory data. Rendering invokes this repository's `.venv/bin/python` and tested
plotting CLI without a shell. The workspace must be trusted.

When `reference_trajectory.npy` and `actual_trajectory.npy` are siblings, opening either one
plots both in the same comparison image. Other valid trajectory NPY files are plotted alone.

Install or update it from the repository root:

```bash
.venv/bin/python scripts/install_vscode_trajectory_graph_viewer.py
```

Reload the VS Code window if an editor from a previously installed NPY extension remains open.

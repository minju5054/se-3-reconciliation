"""Analytic world-XY derivatives of the frozen GP-SE2-01 environment queries.

Values have the original ``GPProblem._query`` semantics: footprint and the
conservative grid error are included once, and nonfinite query results become
-1e6. The caller still subtracts the required obstacle clearance. No geometry,
grid, acceptance tolerance, or original query implementation is changed.

Polygon distance and bilinear interpolation are piecewise differentiable. Each
query selects its current nearest boundary feature / floor-indexed grid cell.
At a true distance tie or polygon vertex we report a deterministic active
limiting gradient; at internal grid boundaries we report the positive-coordinate
cell's one-sided gradient. The grid-domain/invalid-cell penalty can be
discontinuous, so there we claim only a derivative of the selected branch, not a
Clarke derivative or a derivative of the whole function. Metadata makes these
distinctions explicit. There is no finite-difference fallback.
"""
from __future__ import annotations

import numpy as np
import shapely
from shapely.geometry import LineString


class UnsupportedEnvironmentDerivative(ValueError):
    """An original query has no supported analytic derivative here."""


class EnvironmentDerivatives:
    """Query values, world-coordinate gradients, and per-point branch metadata.

    ``evaluate(xy, family)``, ``workspace(xy)`` and ``obstacle(xy)`` return
    ``(values, gradients, metadata)``. Input shape is ``(..., 2)``; value shape is
    ``(...)`` and gradient shape is ``(..., 2)``. A single XY point returns a
    scalar value. ``metadata['points']`` follows flattened input order.

    Workspace must be a valid, nonempty Polygon/MultiPolygon. All exterior and
    hole edges enter the dynamic STRtree search. Radius is a constant in metres.
    Obstacle derivatives require the actual frozen bilinear grid; the optional
    no-grid direct-geometry fallback in HospitalEnvironment is explicitly
    unsupported rather than silently substituted with another objective.
    """

    def __init__(self, environment, radius=.20):
        self.environment = environment
        self.radius = float(radius)
        if not np.isfinite(self.radius) or self.radius < 0:
            raise ValueError("finite nonnegative footprint radius required")
        workspace = environment.workspace
        if (workspace.geom_type not in ("Polygon", "MultiPolygon") or
                workspace.is_empty or not workspace.is_valid):
            raise UnsupportedEnvironmentDerivative("valid nonempty polygonal workspace required")
        self._workspace = workspace
        polygons = list(workspace.geoms) if workspace.geom_type == "MultiPolygon" else [workspace]
        starts, ends, normals, feature_records = [], [], [], []
        for polygon_id, polygon in enumerate(polygons):
            for ring_id, ring in enumerate([polygon.exterior, *polygon.interiors]):
                coordinates = np.asarray(ring.coords, dtype=float)[:, :2]
                if not np.all(np.isfinite(coordinates)):
                    raise UnsupportedEnvironmentDerivative("nonfinite workspace coordinates")
                # Translation before the shoelace sum avoids large-origin loss.
                centered = coordinates - coordinates[0]
                signed_area2 = np.sum(centered[:-1, 0] * centered[1:, 1] -
                                      centered[1:, 0] * centered[:-1, 1])
                if signed_area2 == 0:
                    raise UnsupportedEnvironmentDerivative("zero-area workspace ring")
                ring_interior_side = 1. if signed_area2 > 0 else -1.
                workspace_side = ring_interior_side if ring_id == 0 else -ring_interior_side
                for ring_segment, (a, b) in enumerate(zip(coordinates[:-1], coordinates[1:])):
                    delta = b - a
                    length = np.linalg.norm(delta)
                    if length == 0:  # Repeated ring vertices do not add a feature.
                        continue
                    starts.append(a)
                    ends.append(b)
                    normals.append(workspace_side * np.array([-delta[1], delta[0]]) / length)
                    feature_records.append(dict(polygon_id=polygon_id, ring_id=ring_id,
                                                ring_segment=ring_segment,
                                                hole=ring_id > 0))
        self._starts, self._ends = np.asarray(starts), np.asarray(ends)
        self._deltas = self._ends - self._starts
        self._lengths2 = np.sum(self._deltas ** 2, axis=1)
        self._normals = np.asarray(normals)
        self._feature_records = feature_records
        self._boundary_tree = shapely.STRtree([LineString([a, b]) for a, b in zip(starts, ends)])
        self._grid = environment.grid
        if self._grid is not None:
            self._origin = np.asarray(self._grid["origin"], dtype=float)
            self._resolution = np.broadcast_to(np.asarray(self._grid["resolution"], dtype=float), (2,))
            self._distances = np.asarray(self._grid["distances"], dtype=float)
            self._error = float(environment.metadata.get("grid_distance_absolute_error_bound_m", 0.))
            if (self._origin.shape != (2,) or not np.all(np.isfinite(self._origin)) or
                    not np.all(np.isfinite(self._resolution)) or np.any(self._resolution <= 0) or
                    self._distances.ndim != 2 or min(self._distances.shape) < 2 or
                    not np.isfinite(self._error) or self._error < 0):
                raise UnsupportedEnvironmentDerivative("unsupported original distance grid")

    @staticmethod
    def _points(xy):
        points = np.asarray(xy, dtype=float)
        if points.ndim < 1 or points.shape[-1] != 2 or not np.all(np.isfinite(points)):
            raise ValueError("finite world XY points of shape (..., 2) required")
        return points, points.reshape((-1, 2))

    @staticmethod
    def _result(points, values, gradients, family, records, policy):
        values = np.asarray(values).reshape(points.shape[:-1])
        if points.shape == (2,):
            values = float(values)
        return values, gradients.reshape(points.shape), dict(
            family=family, frame="world_xy_m", input_shape=list(points.shape),
            policy=policy, points=records, finite_difference_fallback=False,
            invalid_query_penalty=-1e6, footprint_applied_once=True)

    def evaluate(self, xy, family):
        if family == "workspace":
            return self.workspace(xy)
        if family == "obstacle":
            return self.obstacle(xy)
        raise ValueError("family must be 'workspace' or 'obstacle'")

    def workspace(self, xy):
        points, flat = self._points(xy)
        # Preserve the original GEOS value calculation, including holes/sign.
        values = np.asarray(self.environment.workspace_margin(flat, self.radius), dtype=float)
        if not np.all(np.isfinite(values)):
            raise UnsupportedEnvironmentDerivative("nonfinite polygonal workspace distance")
        gradients = np.zeros_like(flat)
        records = []
        if len(flat):
            geometries = shapely.points(flat)
            covered = shapely.covers(self._workspace, geometries)
            pair_indices, nearest_distances = self._boundary_tree.query_nearest(
                geometries, all_matches=True, return_distance=True)
            order = np.lexsort((pair_indices[1], pair_indices[0]))
            point_ids, segment_ids = pair_indices[:, order]
            nearest_distances = nearest_distances[order]
            offsets = np.searchsorted(point_ids, np.arange(len(flat) + 1))
            for k, point in enumerate(flat):
                active = segment_ids[offsets[k]:offsets[k + 1]]
                if not len(active):
                    raise UnsupportedEnvironmentDerivative("workspace nearest feature not found")
                a, delta = self._starts[active], self._deltas[active]
                fraction = np.clip(np.sum((point - a) * delta, axis=1) / self._lengths2[active], 0., 1.)
                closest = a + fraction[:, None] * delta
                displacement = point - closest
                distance = np.linalg.norm(displacement, axis=1)
                sign = 1. if covered[k] else -1.
                branch_gradients = self._normals[active].copy()
                on_boundary = bool(nearest_distances[offsets[k]] == 0.)
                # On an edge's relative interior, the signed distance gradient
                # is exactly its inward normal. Normalizing tiny projection
                # roundoff at an exact boundary point would give a false
                # tangent gradient. Off-boundary endpoint projections instead
                # use the analytic signed radial gradient.
                endpoint_nonzero = ((fraction == 0.) | (fraction == 1.)) & (distance > 0.)
                if not on_boundary:
                    branch_gradients[endpoint_nonzero] = (
                        sign * displacement[endpoint_nonzero] / distance[endpoint_nonzero, None])
                gradients[k] = branch_gradients[0]
                # Shared vertex projections may tie but have the same first
                # derivative away from the vertex. Different active gradients
                # identify a medial-axis / boundary-corner nonsmooth point.
                distinct = np.any(np.linalg.norm(branch_gradients - branch_gradients[0], axis=1) > 1e-12)
                endpoint = bool(fraction[0] == 0. or fraction[0] == 1.)
                kind = ("boundary_vertex" if on_boundary and endpoint else
                        "boundary_edge" if on_boundary else
                        "nearest_feature_tie" if distinct else
                        "nearest_vertex" if endpoint else "nearest_edge")
                selected = int(active[0])
                records.append(dict(
                    kind=kind, smooth=not bool(distinct), covered_by_workspace=bool(covered[k]),
                    selected_segment_id=selected, active_segment_ids=active.tolist(),
                    nearest_world_xy=closest[0].tolist(),
                    selected_segment_world_xy=[self._starts[selected].tolist(), self._ends[selected].tolist()],
                    selected_inward_normal=self._normals[selected].tolist(),
                    selected_gradient=gradients[k].tolist(),
                    active_gradients=branch_gradients.tolist(),
                    derivative_semantics="active_limiting_gradient" if distinct else "classical_gradient",
                    **self._feature_records[selected]))
        return self._result(points, values, gradients, "workspace", records,
                            "dynamic nearest polygon boundary; minimum segment ID selects exact ties")

    def obstacle(self, xy):
        points, flat = self._points(xy)
        if self._grid is None:
            raise UnsupportedEnvironmentDerivative("obstacle derivative requires original bilinear grid")
        q = (flat - self._origin) / self._resolution
        upper = np.array([self._distances.shape[1] - 1, self._distances.shape[0] - 1])
        in_domain = np.all((q >= 0.) & (q < upper), axis=1)
        values = np.full(len(flat), -1e6)
        gradients = np.zeros_like(flat)
        records = []
        for k, coord in enumerate(q):
            domain_boundary = bool(np.any((coord == 0.) | (coord == upper)))
            grid_axes = np.flatnonzero(coord == np.floor(coord)).tolist()
            cell = None
            bounds = None
            boundary_distance = None
            valid_cell = False
            if in_domain[k]:
                i, j = np.floor(coord).astype(int)
                a, b = coord - [i, j]
                z00, z10 = self._distances[j, i:i + 2]
                z01, z11 = self._distances[j + 1, i:i + 2]
                value = ((1-a)*(1-b)*z00 + a*(1-b)*z10 + (1-a)*b*z01 + a*b*z11
                         - self._error - self.radius)
                cell = [int(i), int(j)]
                bounds = [(self._origin + np.array(cell) * self._resolution).tolist(),
                          (self._origin + (np.array(cell) + 1) * self._resolution).tolist()]
                boundary_distance = (np.minimum(coord - [i, j], 1. - (coord - [i, j])) *
                                     self._resolution).tolist()
                if np.isfinite(value):
                    valid_cell = True
                    values[k] = value
                    gradients[k] = [((1-b)*(z10-z00) + b*(z11-z01)) / self._resolution[0],
                                    ((1-a)*(z01-z00) + a*(z11-z10)) / self._resolution[1]]
            invalid = not valid_cell
            nonsmooth = domain_boundary or bool(grid_axes and in_domain[k])
            records.append(dict(
                kind="invalid_query_penalty" if invalid else "bilinear_grid_boundary" if grid_axes else "bilinear_cell",
                smooth=not nonsmooth, cell_xy=cell, cell_bounds_world_xy=bounds,
                distance_to_grid_boundary_m=boundary_distance,
                grid_boundary_axes=grid_axes, domain_boundary=domain_boundary,
                in_grid_domain=bool(in_domain[k]), invalid_query=invalid,
                selected_gradient=gradients[k].tolist(),
                derivative_semantics="selected_branch_only_possible_discontinuity" if invalid or domain_boundary else
                                     "positive_cell_one_sided_gradient" if grid_axes else "classical_gradient"))
        if not np.all(np.isfinite(gradients)):
            raise UnsupportedEnvironmentDerivative("nonfinite analytic grid gradient")
        return self._result(points, values, gradients, "obstacle", records,
                            "original bilinear floor cell; conservative bound and footprint constants; _query invalid penalty")

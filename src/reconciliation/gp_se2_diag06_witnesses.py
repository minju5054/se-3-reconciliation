"""One frozen union of observed extrema; never adaptive constraint insertion.

Canonical indices refer to the original interval-side ordered 1 ms + .371
trace, including both sides of knots. No float rounding participates in identity.
"""
from __future__ import annotations
import copy
import numpy as np
from .gp_se2_diag04_constraints import MOTION_ROWS
from .gp_se2_diag03_intervals import _runs, sample_interval_grid
from .gp_se2_diag_acceptance import offset_grid

QUANTITIES = {'v_x':'vx', 'omega':'omega', 'a_x':'ax', 'alpha':'alpha'}
CANONICAL_FIELDS = ('times_s','interval_index','local_fraction','location_type','knot_side')


def finest_trace(problem, vector):
    return sample_interval_grid(problem, vector, offset_grid(problem.times, .001))


def canonical_grid(trace):
    return {k:np.asarray(trace[k]).tolist() for k in CANONICAL_FIELDS}


def signed_margins(trace, config):
    output = {}
    for family, quantity, unit, sign, expression, limit in MOTION_ROWS:
        values = np.asarray(trace['quantities'][QUANTITIES[quantity]], np.float64)
        if values.ndim != 1 or len(values) != len(trace['times_s']) or not np.isfinite(values).all():
            raise ValueError('finite canonical trace required')
        output[family] = (values, sign*values+(0. if limit is None else config[limit]))
    return output


def extract_runs(trace, config, source):
    """One minimum-margin sample per connected violating run, earliest-time ties."""
    times = np.asarray(trace['times_s']); intervals = np.asarray(trace['interval_index'])
    u = np.asarray(trace['local_fraction']); tolerance = config['inequality_tolerance']
    if not np.isfinite(times).all() or not np.isfinite(u).all() or np.any((u<0)|(u>1)):
        raise ValueError('invalid canonical sample locations')
    rows=[]
    for family, quantity, unit, sign, expression, limit in MOTION_ROWS:
        values, margins = signed_margins(trace, config)[family]
        for run, (first,last) in enumerate(_runs(margins < -tolerance)):
            indices = np.arange(first,last+1)
            k = min(indices, key=lambda j:(float(margins[j]),float(times[j]),int(j)))
            rows.append(dict(source=source, family=family, quantity=quantity, unit=unit,
                interval_index=int(intervals[k]),canonical_grid_sample_index=int(k),time_s=float(times[k]),
                local_fraction=float(u[k]),knot_side=trace['knot_side'][k],actual_value=float(values[k]),
                signed_margin=float(margins[k]),tolerance=tolerance,tolerance_excess=float(-margins[k]-tolerance),
                observed_run_index=run,first_sample_index=int(first),last_sample_index=int(last),
                first_observed_time_s=float(times[first]),last_observed_time_s=float(times[last]),
                observed_run_sample_count=int(last-first+1),kind='OBSERVED EXTREMUM WITNESS'))
    return rows


def witness_union(per_source):
    merged={}; order={r[0]:i for i,r in enumerate(MOTION_ROWS)}
    for record in per_source:
        key=(record['family'],record['interval_index'],record['canonical_grid_sample_index'])
        if key not in merged:
            merged[key]={k:copy.deepcopy(record[k]) for k in ('family','quantity','unit','interval_index',
                'canonical_grid_sample_index','time_s','local_fraction','knot_side')}
            merged[key]['source_violations']=[]
        merged[key]['source_violations'].append(copy.deepcopy(record))
    rows=[merged[k] for k in sorted(merged,key=lambda k:(order[k[0]],k[1],k[2]))]
    for i,row in enumerate(rows):row['witness_row_index']=i
    return rows


def validate_schedule(rows, grid):
    seen=set(); families={r[0]:r for r in MOTION_ROWS}
    for number,row in enumerate(rows):
        family=row['family'];k=row['canonical_grid_sample_index'];interval=row['interval_index']
        if family not in families or not isinstance(k,int) or not 0<=k<len(grid['times_s']):
            raise ValueError('invalid witness family/index')
        key=(family,interval,k)
        if key in seen or row['witness_row_index']!=number:raise ValueError('duplicate/unordered witness')
        seen.add(key)
        if (interval!=grid['interval_index'][k] or row['time_s']!=grid['times_s'][k]
            or row['local_fraction']!=grid['local_fraction'][k] or row['knot_side']!=grid['knot_side'][k]):
            raise ValueError('canonical sample identity mismatch')
        if (row['quantity'],row['unit'])!=(families[family][1],families[family][2]):
            raise ValueError('witness units mismatch')
    return True

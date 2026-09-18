"""Reproducible source-to-trace provenance for the synthetic math figures.

This module reads saved numbers and existing PNG bytes; it never interpolates a
trajectory, optimizes, regenerates a figure, or changes acceptance. Trace hashes
are recomputed independently of the plotting calls. They link the explicit
plotting expressions to numeric sources, not a pixel-inversion proof.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np


def _read(path):
    return json.loads(Path(path).read_text())


def _digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _series(label, x, y, units):
    x, y = np.asarray(x, dtype='<f8'), np.asarray(y, dtype='<f8')
    if x.ndim != 1 or x.shape != y.shape or not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
        raise ValueError('plot trace must have matching finite one-dimensional x/y values')
    return dict(label=label, sample_count=len(x), x_units=units[0], y_units=units[1],
                x_float64_le_sha256=hashlib.sha256(x.tobytes()).hexdigest(),
                y_float64_le_sha256=hashlib.sha256(y.tobytes()).hexdigest())


def build_math_plot_sources(output, *, visual_review=None):
    """Build the exact numeric/hash provenance without writing any files.

    Optional ``visual_review`` is separately supplied human/agent review
    metadata, never inferred from numeric checks. With no such evidence, the
    generated sidecar explicitly leaves visual review unevaluated.
    """
    root = Path(output)
    records = []
    for name in ('S0', 'S1', 'S2', 'S3'):
        directory = root/name
        numeric = directory/'numeric_results.json'
        result = _read(numeric)
        values = lambda key: np.asarray(result[key])
        times, interior = values('query_times_s'), values('interior_derivative_times_s')
        truth, gp = values('truth_poses_world'), values('gp_poses_world')
        support = np.asarray(result['fixture_input']['support_poses'])
        plots = {}
        traces = [
            _series('worldXY truth: truth_poses_world[:,0:2]', truth[:, 0], truth[:, 1], ('m', 'm')),
            _series('worldXY GP: gp_poses_world[:,0:2]', gp[:, 0], gp[:, 1], ('m', 'm')),
            _series('support poses: fixture_input.support_poses[:,0:2]', support[:, 0], support[:, 1], ('m', 'm')),
            _series('position error: position_error_m', times, values('position_error_m'), ('s', 'm')),
            _series('yaw error: yaw_error_rad', times, values('yaw_error_rad'), ('s', 'rad')),
        ]
        plots['truth_vs_gp_reconstruction.png'] = dict(traces=traces, aspect_world_xy='equal', limits=[])
        traces = [
            _series('GP lateral: gp_body_twists[:,1]', times, values('gp_body_twists')[:, 1], ('s', 'm/s')),
            _series('FD lateral: independent_body_twists[:,1]', interior, values('independent_body_twists')[:, 1], ('s', 'm/s')),
        ]
        for component, unit in ((0, 'm/s^2'), (2, 'rad/s^2')):
            for field, grid in (('gp_body_accelerations', times), ('truth_body_accelerations', times),
                                ('independent_body_accelerations', interior)):
                traces.append(_series(f'{field}[:,{component}]', grid, values(field)[:, component], ('s', unit)))
        config = result['fixture_input']['config']
        # These are the actual frozen plotting constants, not silently adapted
        # lines if an unrelated future config uses different physical limits.
        if config['a_v_max'] != 2 or config['a_w_max'] != 5:
            raise ValueError('saved math figure acceleration limits differ from the frozen 2/5 constants')
        limits = [dict(family='lateral', unit='m/s', lower=-config['equality_tolerance'], upper=config['equality_tolerance']),
                  dict(family='linear_acceleration', unit='m/s^2', lower=-config['a_v_max'], upper=config['a_v_max']),
                  dict(family='angular_acceleration', unit='rad/s^2', lower=-config['a_w_max'], upper=config['a_w_max'])]
        plots['lateral_and_acceleration_vs_time.png'] = dict(traces=traces, limits=limits)
        plots['acceleration_reconstruction_error.png'] = dict(
            traces=[_series(f'gp_body_accelerations[:,{component}]-truth_body_accelerations[:,{component}]', times,
                            values('gp_body_accelerations')[:, component]-values('truth_body_accelerations')[:, component],
                            ('s', unit)) for component, unit in ((0, 'm/s^2'), (2, 'rad/s^2'))],
            limits=[], meaning='magnified signed reconstruction error, not absolute physical limit plot; limits in companion acceleration figure')
        for filename, description in plots.items():
            records.append(dict(png=str((directory/filename).relative_to(root)), png_sha256=_digest(directory/filename),
                                numeric_source=str(numeric.relative_to(root)), numeric_source_sha256=_digest(numeric),
                                synthetic=True, **description))

    directory = root/'collocation_blind_spot'
    numeric = directory/'numeric_results.json'
    result = _read(numeric)
    summary = result['summary']
    values = lambda key: np.asarray(result[key])
    fractions = np.asarray(summary['fractions'])
    lateral = np.asarray(summary['lateral_at_prescribed_points_m_s'])
    tolerance = summary['equality_tolerance_m_s']
    traces = [
        _series('GP lateral: body_twists[:,1]', values('times_s'), values('body_twists')[:, 1], ('s', 'm/s')),
        _series('FD lateral plotted every20: independent_body_twists[::20,1]', values('independent_times_s')[::20],
                values('independent_body_twists')[::20, 1], ('s', 'm/s')),
        _series('support/midpoint: summary fractions indices0,2,4', fractions[[0, 2, 4]]*.1,
                lateral[[0, 2, 4]], ('s', 'm/s')),
        _series('quarters: summary fractions indices1,3', fractions[[1, 3]]*.1, lateral[[1, 3]], ('s', 'm/s')),
    ]
    records.append(dict(png='collocation_blind_spot/lateral_velocity_vs_time.png',
                        png_sha256=_digest(directory/'lateral_velocity_vs_time.png'),
                        numeric_source='collocation_blind_spot/numeric_results.json', numeric_source_sha256=_digest(numeric),
                        synthetic=True, traces=traces,
                        limits=[dict(family='lateral', unit='m/s', lower=-tolerance, upper=tolerance)],
                        maximum_annotation=dict(absolute_lateral_m_s=summary['maximum_absolute_lateral_velocity_m_s'],
                                                time_s=summary['maximum_violation_time_s'])))
    metadata = dict(visual_review=[], visual_findings='NOT_EVALUATED_BY_NUMERIC_PROVENANCE; separate visual inspection required')
    if visual_review is not None:
        if set(visual_review) != {'visual_review', 'visual_findings'}:
            raise ValueError('visual review metadata requires visual_review and visual_findings fields')
        metadata.update(visual_review)
    return dict(schema_version=1, records=records, record_count=len(records),
                trace_array_digest='SHA256 of contiguous little-endian float64 bytes; expression and sample_count identify exact source samples',
                scope='Maps source plotting statements to saved numeric arrays and immutable PNG bytes; not pixel-inversion or continuous-feasibility proof',
                plotting_source='implementation_sources/gp_se2_diag_fixtures.py',
                plotting_source_sha256=_digest(root/'implementation_sources/gp_se2_diag_fixtures.py'), **metadata)


def build_math_integrity_manifest(output):
    """Hash all existing math evidence except this manifest itself."""
    root = Path(output)
    files = [dict(path=str(path.relative_to(root)), bytes=path.stat().st_size, sha256=_digest(path))
             for path in sorted(root.rglob('*')) if path.is_file() and path != root/'integrity_manifest.json']
    return dict(schema_version=1, files=files, file_count=len(files), self_excluded='integrity_manifest.json',
                immutable_evidence='Original math numeric outputs, source snapshots, and PNGs were not rewritten when these supplemental sidecars were added.')


def _write_new(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def write_math_plot_sources(output, *, visual_review=None):
    """Add both sidecars exclusively; never rewrite math artifacts or sidecars."""
    root = Path(output)
    for name in ('plot_sources.json', 'integrity_manifest.json'):
        if (root/name).exists():
            raise FileExistsError(root/name)
    provenance = build_math_plot_sources(root, visual_review=visual_review)
    _write_new(root/'plot_sources.json', provenance)
    manifest = build_math_integrity_manifest(root)
    _write_new(root/'integrity_manifest.json', manifest)
    return dict(plot_record_count=provenance['record_count'], integrity_file_count=manifest['file_count'])


def verify_math_plot_sources(output):
    """Independently recalculate trace/file hashes, units, limits, and inventory.

    Visual review text is checked only as retained metadata; this function never
    claims it has inspected rendered pixels. It reports failures rather than
    changing files or repairing a sidecar.
    """
    root = Path(output)
    errors, checks = [], 0
    try:
        saved = _read(root/'plot_sources.json')
        visual = {key: saved[key] for key in ('visual_review', 'visual_findings')}
        calculated = build_math_plot_sources(root, visual_review=visual)
        checks += 1
        if saved != calculated:
            errors.append('plot_sources differs from independently reconstructed numeric/hash/unit/limit provenance')
        by_png = {row['png']: row for row in calculated['records']}
        checks += 1
        if len(by_png) != 13 or sorted(by_png) != sorted(str(path.relative_to(root)) for path in root.rglob('*.png')):
            errors.append('13 expected math PNGs do not match actual inventory')
        source = _read(root/'source.json')
        for name, expected_hash in source['source_sha256'].items():
            checks += 1
            if _digest(root/'implementation_sources'/name) != expected_hash:
                errors.append('frozen source snapshot mismatch: '+name)
        for name in ('S0', 'S1', 'S2', 'S3'):
            result = _read(root/name/'numeric_results.json')
            q = np.asarray(result['query_times_s'])
            summary = result['summary']
            checks += 4
            position = np.linalg.norm(np.asarray(result['gp_poses_world'])[:, :2]-np.asarray(result['truth_poses_world'])[:, :2], axis=1)
            yaw_difference = np.asarray(result['gp_poses_world'])[:, 2]-np.asarray(result['truth_poses_world'])[:, 2]
            yaw = np.abs(np.arctan2(np.sin(yaw_difference), np.cos(yaw_difference)))
            if not np.array_equal(position, np.asarray(result['position_error_m'])):
                errors.append(name+' position error trace mismatch')
            if not np.array_equal(yaw, np.asarray(result['yaw_error_rad'])):
                errors.append(name+' yaw error trace mismatch')
            if not (len(q) == 3001 and q[0] == 0 and q[-1] == 3 and np.all(np.diff(q)>0)):
                errors.append(name+' dense trace grid differs from frozen 3 s / 1 ms diagnostics')
            if summary['lateral_velocity_max_m_s'] != float(np.max(np.abs(np.asarray(result['gp_body_twists'])[:, 1]))):
                errors.append(name+' lateral maximum summary mismatch')
        blind = _read(root/'collocation_blind_spot/numeric_results.json')
        lateral = np.abs(np.asarray(blind['body_twists'])[:, 1])
        index = int(np.argmax(lateral))
        checks += 1
        if (blind['summary']['maximum_absolute_lateral_velocity_m_s'] != float(lateral[index]) or
                blind['summary']['maximum_violation_time_s'] != blind['times_s'][index]):
            errors.append('blind-spot maximum annotation differs from numeric trace')
        checks += 1
        if _read(root/'integrity_manifest.json') != build_math_integrity_manifest(root):
            errors.append('math file integrity manifest differs from current files')
        return dict(valid=not errors, errors=errors, checks=checks, plot_count=len(by_png),
                    trace_count=sum(len(row['traces']) for row in calculated['records']),
                    verification_scope='numeric sources, derived traces, units/limits, source/PNG hashes; no pixel inversion',
                    wrote_files=False)
    except (OSError, ValueError, TypeError, KeyError, IndexError) as exc:
        errors.append(f'{type(exc).__name__}: {exc}')
        return dict(valid=False, errors=errors, checks=checks, plot_count=None, trace_count=None, wrote_files=False)

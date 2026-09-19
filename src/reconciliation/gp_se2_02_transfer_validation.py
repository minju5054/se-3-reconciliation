"""Read-only validation of saved transfer probes, without another FD sweep.

Re-evaluating supplied derivatives at saved centers checks the recorded matrix
and primal values. Saved central/one-sided FD arrays are checked arithmetically;
only geometry branch identities are queried at the recorded +/- positions.
No optimization, new probe selection or finite-difference sweep is performed.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .gp_se2_02_transfer import (
    TRANSFER_PROTOCOL, _unsupported_reason, build_transfer_points, classify_start,
    vector_sha256,
)
from .gp_se2_diag02_validation import (
    PROTOCOL, _family_reports, _geometry_at, _reports_pass, _smooth_rows, _values,
    constraint_families, error_report, file_sha256, plain, point_record,
)


def _read(path):
    return json.loads(Path(path).read_text())


def validate_supported_point(point, record, arrays, directions, provider, checks):
    """Reproduce one saved point's reports and branch masks, not its FD sweep."""
    problem, geometry = point['problem'], point['geometry']
    z = np.asarray(point['vector'], dtype=np.float64)
    label = 'transfer '+point['name']
    checks.equal(arrays['vector'], z, label+': saved chart', exact=True)
    checks.equal(arrays['directions'], directions, label+': saved directions', exact=True)
    original = _values(problem, z)
    current = provider.values(z)
    primal = {field: np.atleast_1d(np.asarray(current[field], dtype=np.float64))[:, None] for field in original}
    expected = {field: value[:, None] for field, value in original.items()}
    primal_reports = _family_reports(problem, primal, expected, None, primal=True)
    checks.equal(record['primal_reports'], primal_reports, label+': independent original primal reports')
    ad = dict(objective=arrays['objective_gradient'], equality=arrays['equality_jacobian'],
              inequality=arrays['inequality_jacobian'])
    current_ad = dict(objective=np.asarray(provider.objective_gradient(z))[None, :],
                      equality=provider.equality_jacobian(z), inequality=provider.inequality_jacobian(z))
    shape_valid = all(ad[field].shape == (len(original[field]), 150) and np.isfinite(ad[field]).all() for field in ad)
    checks.check(shape_valid, label+': full finite objective/equality/inequality matrices')
    for field in ad:
        checks.equal(ad[field], current_ad[field], label+': independent supplied '+field)
    center_meta = _geometry_at(problem, geometry, z)
    checks.equal(record['geometry_metadata'], center_meta, label+': original analytic branch identities')
    checks.equal(record['provider_geometry_metadata'], provider.geometry_metadata(z), label+': chained provider branch identities')
    passes, reports = [], []
    for index, h in enumerate(PROTOCOL['directional_fd_steps']):
        predicted = {field: ad[field]@directions.T for field in ad}
        fd = {field: arrays[f'directional_{index}_{field}_central_fd'] for field in ad}
        masks = {field: arrays[f'directional_{index}_{field}_smooth_mask'] for field in ad}
        sides = {side: {field: arrays[f'directional_{index}_{field}_{side}_fd'] for field in ad}
                 for side in ('plus', 'minus')}
        geometry_mask = np.ones_like(masks['inequality'], dtype=bool)
        for column, direction in enumerate(directions):
            plus = _geometry_at(problem, geometry, z+h*direction)
            minus = _geometry_at(problem, geometry, z-h*direction)
            geometry_mask[:, column] = _smooth_rows(problem, center_meta, plus, minus)
        # Original DIAG-02 rule: a fixed row that is exactly constant in both
        # saved AD and FD can be checked even at a nonsmooth world geometry.
        geometry_mask |= (predicted['inequality'] == 0) & (fd['inequality'] == 0)
        checks.equal(masks['inequality'], geometry_mask, label+f': branch-consistent mask h={h}', exact=True)
        checks.check(masks['objective'].all() and masks['equality'].all() and masks['inequality'][:723].all(),
                     label+f': no objective/lateral/motion/goal row excluded h={h}')
        sided_reports = []
        for field in ad:
            checks.equal(arrays[f'directional_{index}_{field}_prediction'], predicted[field],
                         label+f': Jd projection {field} h={h}')
            # The exact algebra of independently saved one-sided differences
            # must recover the central difference, up to ordinary rounding.
            checks.equal(fd[field], (sides['plus'][field]+sides['minus'][field])/2,
                         label+f': central and one-sided arithmetic {field} h={h}')
            nearest_plus = np.abs(predicted[field]-sides['plus'][field]) <= np.abs(predicted[field]-sides['minus'][field])
            closest = np.where(nearest_plus, sides['plus'][field], sides['minus'][field])
            for family in [item for item in constraint_families(problem) if item['field'] == field]:
                begin, end = family['start'], family['stop']
                report = error_report(predicted[field][begin:end], closest[begin:end],
                    PROTOCOL['objective_derivative_tolerance' if field == 'objective' else 'constraint_derivative_tolerance'],
                    mask=~masks[field][begin:end], row_offset=begin)
                sided_reports.append(dict(family=family['name'], **report,
                    scope='descriptive closest one-sided difference; no classical differentiability assertion; no relaxation of smooth acceptance'))
        computed = _family_reports(problem, predicted, fd, masks)
        saved = record['directional'][index]
        checks.equal(saved['smooth_reports'], computed, label+f': original family error arithmetic h={h}')
        checks.equal(saved['nonsmooth_one_sided_characterization'], sided_reports,
                     label+f': separate one-sided characterization h={h}')
        checks.check(saved['step'] == h and saved['smooth_pass'] == _reports_pass(computed),
                     label+f': unchanged multistep rule h={h}')
        passes.append(_reports_pass(computed)); reports.append(computed)
    stats = record['provider_stats']
    contract = bool(stats['float64_enabled'] and stats['device_backend'] == 'cpu'
                    and stats['numerical_finite_difference_calls'] == 0 and not stats['numerical_fallback_used'])
    expected_valid = bool(_reports_pass(primal_reports) and shape_valid and all(passes[-2:]) and contract)
    checks.check(record['valid'] == expected_valid and
                 record['status'] == ('VERIFIED' if expected_valid else 'DERIVATIVE_VALIDATION_FAILED'),
                 label+': status follows original two-finest-step rule')
    checks.check(record['required_primal_pass'] == _reports_pass(primal_reports)
                 and record['finite_full_jacobian_shapes'] == shape_valid
                 and record['directional_pass'] == all(passes[-2:])
                 and record['provider_contract_valid'] == contract, label+': qualification flags reproduced')
    checks.check(record['coordinate_pass'] and not record['full_coordinate'] and not record['coordinate_reports']
                 and record['coordinate_sweep_not_repeated'], label+': no repeated full coordinate sweep')
    checks.check(not record['optimizer_executed'] and not record['nonsmooth_classical_derivative_claimed'],
                 label+': optimizer-free branch-limited scope')
    checks.check(record['central_nonsmooth_exclusions'] == sum(row.get('excluded_elements', 0)
                 for step in reports for row in step), label+': nonsmooth exclusions explicitly counted')
    return dict(primal_reports=primal_reports, required_reports=[row for step in reports[-2:] for row in step],
                status=record['status'])


def validate_saved_transfer(directory, primary, checks):
    """Validate all frozen reports using a ``check/equal`` accumulator API."""
    from .gp_se2_diag02_derivatives import DerivativeProvider
    directory = Path(directory)
    protocol = _read(directory/'protocol.json')
    manifest = _read(directory/'point_manifest.json')
    summary = _read(directory/'summary.json')
    for key, expected in TRANSFER_PROTOCOL.items():
        checks.equal(protocol.get(key), expected, 'transfer immutable policy: '+key, exact=True)
    for key, name in (('protocol_sha256', 'protocol.json'), ('source_sha256', 'source.json'), ('directions_sha256', 'directions.json')):
        checks.check(file_sha256(directory/name) == manifest[key], 'transfer frozen file hash: '+name)
    points, starts = build_transfer_points(primary)
    checks.equal(manifest['starts'], starts, 'transfer exact fixed starts and seed hashes', exact=True)
    checks.check(len(points) == len(manifest['points']) == 32 and len(starts) == 16,
                 'transfer all 32 diagnostic points and 16 planned starts')
    directions = np.asarray(_read(directory/'directions.json')['directions'], dtype=np.float64)
    expected_directions = np.random.default_rng(TRANSFER_PROTOCOL['random_seed']).normal(size=(3, 150))
    expected_directions /= np.linalg.norm(expected_directions, axis=1, keepdims=True)
    checks.equal(directions, expected_directions, 'transfer original fixed random directions', exact=True)
    providers, records, reproduced = {}, {}, []
    for point, entry in zip(points, manifest['points']):
        name = point['name']; label = 'transfer '+name
        checks.check(entry['name'] == name, label+': exact point order')
        checks.check(file_sha256(directory/entry['path']) == entry['sha256'], label+': frozen point hash')
        checks.equal(_read(directory/entry['path']), point_record(point), label+': source/vector/config identity', exact=True)
        record = _read(directory/'results'/(name+'.json')); records[name] = record
        for field in ('name', 'case_id', 'method', 'initialization', 'probe', 'start_key', 'seed_vector_sha256'):
            checks.equal(record[field], point[field], label+': report '+field, exact=True)
        checks.check(record['vector_sha256'] == vector_sha256(point['vector']), label+': report vector hash')
        if record['status'] == 'UNSUPPORTED_GATE_CASE':
            checks.check(bool(point['problem'].gates) and not record['valid'], label+': original gate remains unsupported')
            checks.equal(record['original_gates'], point['problem'].gates, label+': no gate deletion', exact=True)
            continue
        key = id(point['problem'])
        if key not in providers:
            providers[key] = DerivativeProvider(point['problem'], point['geometry'])
        provider = providers[key]
        if 'matrix_file' not in record:
            caught = None
            try:
                provider.warmup(point['vector'])
            except Exception as error:
                caught = error
            checks.check(caught is not None, label+': unsupported/error report independently reproduced')
            if caught is not None:
                reason = _unsupported_reason(caught)
                expected_status = 'DERIVATIVE_UNSUPPORTED' if reason else 'DERIVATIVE_VALIDATION_FAILED'
                checks.check(record['status'] == expected_status and not record['valid'], label+': precise domain/error distinction')
                checks.equal(record['guard_diagnostics'], getattr(caught, 'diagnostics', None), label+': exact unsupported location')
                checks.check(record['reason_code'] == (reason or getattr(caught, 'reason_code', type(caught).__name__)),
                             label+': precise failure reason')
            checks.check(record['no_finite_difference_fallback'] and not record['optimizer_executed'], label+': no fallback')
            continue
        matrix = directory/'results'/record['matrix_file']
        checks.check(file_sha256(matrix) == record['matrix_sha256'], label+': recorded numerical array hash')
        with np.load(matrix, allow_pickle=False) as loaded:
            arrays = {field: loaded[field] for field in loaded.files}
        reproduced.append(validate_supported_point(point, record, arrays, directions, provider, checks))
    expected_starts = []
    for start in starts:
        rows = [records[name] for name in start['points']]
        status = classify_start(rows, gates=start['original_gates'])
        expected_starts.append(dict(start, status=status, authorized_for_supplied_jac_solve=status == 'VERIFIED',
            point_statuses=[dict(name=row['name'], status=row['status'], reason_code=row.get('reason_code')) for row in rows]))
    checks.equal(summary['starts'], expected_starts, 'transfer every start authorized by both own points', exact=True)
    checks.equal(summary['start_lookup'], {row['start_key']: row['status'] for row in expected_starts},
                 'transfer lookup reproduces complete ledger', exact=True)
    checks.check(summary['complete'] and summary['point_count'] == 32 and summary['start_count'] == 16,
                 'transfer recorded coverage complete')
    checks.check(summary['all_starts_verified'] == all(row['status'] == 'VERIFIED' for row in expected_starts),
                 'transfer overall status does not hide unsupported starts')
    checks.equal(summary['failed_validation_points'], [name for name, row in records.items() if row['status'] == 'DERIVATIVE_VALIDATION_FAILED'],
                 'transfer all derivative failures retained', exact=True)
    checks.equal(summary['unsupported_points'], [name for name, row in records.items() if row['status'] in ('DERIVATIVE_UNSUPPORTED', 'UNSUPPORTED_GATE_CASE')],
                 'transfer all unsupported points retained', exact=True)
    required = [row for report in reproduced for row in report['required_reports'] if row.get('compared_elements', 0)]
    primal = [row for report in reproduced for row in report['primal_reports'] if row.get('compared_elements', 0)]
    checks.equal(summary['maximum_required_directional_scaled_error'], max((row['maximum_scaled_error'] for row in required), default=None),
                 'transfer summary maximum derivative error')
    checks.equal(summary['maximum_primal_absolute_error'], max((row['maximum_absolute_error'] for row in primal), default=None),
                 'transfer summary maximum primal error')
    checks.check(summary['required_directional_comparison_elements'] == sum(row['compared_elements'] for row in required),
                 'transfer summary compared elements')
    checks.check(summary['required_directional_excluded_elements'] == sum(row.get('excluded_elements', 0) for row in required),
                 'transfer summary excluded elements')
    for key, name in (('protocol_sha256', 'protocol.json'), ('source_sha256', 'source.json'), ('point_manifest_sha256', 'point_manifest.json')):
        checks.check(file_sha256(directory/name) == summary[key], 'transfer summary frozen hash: '+name)
    checks.check(not summary['optimizer_executed'] and summary['no_finite_difference_solver_fallback']
                 and summary['original_authoritative_validation_reused'], 'transfer declared scope')
    return dict(point_count=len(points), start_count=len(starts), independently_replayed_saved_centers=len(reproduced),
                new_finite_difference_sweep=False, original_primal_reevaluated=True, optimizer_executed=False)

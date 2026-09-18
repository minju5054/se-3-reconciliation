"""Frozen, optimizer-free derivative validation in the original 150-D chart.

Finite differences are diagnostic only. The optimizer derivative provider must
remain full-coverage CPU float64 AD plus explicit geometry derivatives, never
silently substitute these numerical probes for a production Jacobian.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import numpy as np


PROTOCOL = {
    'schema_version': 1,
    'experiment': 'GP-SE2-DIAG-02',
    'actual_case_id': 'episode_001_repeat_01/handoff_002',
    'random_seed': 20260919,
    'coordinate_fd_step': 2e-6,
    'directional_fd_steps': [2e-4, 2e-5, 2e-6],
    'direction_count': 3,
    'direction_normalization': 'L2-normalized deterministic standard-normal 150-vector; original mixed-unit chart, no hidden variable scaling',
    'primal_tolerance': {'atol': 1e-8, 'rtol': 1e-10},
    'objective_derivative_tolerance': {'atol': 2e-4, 'rtol': 2e-5},
    'constraint_derivative_tolerance': {'atol': 2e-5, 'rtol': 2e-5},
    'scaled_error_definition': 'abs(AD-FD)/(atol+rtol*max(abs(AD),abs(FD))); pass <=1',
    'smooth_direction_acceptance': 'both two finest steps pass on rows whose geometry branch stays unchanged; all steps and errors retained',
    'nonsmooth_policy': 'no classical central derivative claimed at wrap cuts, true geometric ties, cell/domain boundaries; preserve per-row branch metadata and characterize one-sided selected-branch derivatives separately',
    'full_coordinate_points': [
        'actual_M2_I0', 'actual_M2_I1', 'actual_M3_I0', 'actual_M3_I1',
        'rejected_M2_start0_latest', 'rejected_M3_start0_latest', 'S0', 'S1', 'S2', 'S3'],
    'directional_additional_points': [
        'actual_M2_I0_small_perturbation', 'actual_M3_I0_small_perturbation',
        'nonzero_chart_arbitrary_world', 'rotation_only_yaw_wrap', 'linear_speed_bound',
        'exp_log_small_angle_below', 'exp_log_small_angle_above',
        'right_jacobian_small_angle_below', 'right_jacobian_small_angle_above',
        'relative_yaw_wrap_cut'],
    'small_perturbation_scales': [1e-4, 1e-4, 1e-4, 1e-4, 1e-4],
    'fixed_problem': '3s, .1s supports, first state fixed, original right-local pose/v/omega chart; original objective/constraints unchanged',
    'gate_scope': 'fixed actual event has no gates; nonempty-gate problems explicitly unsupported and cannot pass full coverage',
    'geometry_requirement': 'workspace for M2/M3 and obstacle grid for M3, with separate nonsmooth geometry diagnostic report',
    'failure_policy': 'any required smooth/primal/full-coverage failure blocks actual analytic-derivative solves; no threshold relaxation; implementation fixes require new preserved attempt directory',
    'optimizer_executed': False,
}


def plain(value):
    if isinstance(value, np.ndarray): return plain(value.tolist())
    if isinstance(value, np.generic): return plain(value.item())
    if isinstance(value, dict): return {str(k): plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)): return [plain(v) for v in value]
    if isinstance(value, float) and not np.isfinite(value): return None
    return value


def write_new(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as stream:
        json.dump(plain(value), stream, indent=2, allow_nan=False)
        stream.write('\n')


def file_sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def freeze_protocol(directory):
    path = Path(directory)/'protocol.json'
    write_new(path, dict(PROTOCOL, frozen_utc=datetime.now(timezone.utc).isoformat(),
                         freeze_scope='tolerances, deterministic point construction and finite-difference policy fixed before derivative evaluation'))
    return file_sha256(path)


def freeze_runtime_cut_addendum(directory, evidence_directory):
    """Freeze the supported-domain correction before any actual optimization."""
    directory, evidence_directory = Path(directory), Path(evidence_directory)
    evidence = evidence_directory/'relative_yaw_wrap_cut.json'
    matrices = evidence_directory/'relative_yaw_wrap_cut.npz'
    report = json.loads(evidence.read_text())
    if report['name'] != 'relative_yaw_wrap_cut':
        raise ValueError('runtime-cut policy requires the preserved exact-cut diagnostic')
    output = dict(frozen_utc=datetime.now(timezone.utc).isoformat(), addendum='runtime_cut_domain_guard',
        reason='At the exact relative Log cut, AD selected-expression objective derivative did not match either one-sided original objective limit. The prior characterization gate did not establish a usable gradient there.',
        runtime_policy='Reject adjacent relative Log, FRESH preservation Log, and goal-yaw wrap cuts within 1e-12 rad via explicit DerivativeError. No finite-difference fallback.',
        guard_distance_rad=1e-12, expected_unsupported_point='relative_yaw_wrap_cut',
        expected_supported_points=19, expected_runtime_rejections=1,
        original_protocol_sha256=file_sha256(directory/'protocol.json'),
        frozen_point_manifest_sha256=file_sha256(directory/'point_manifest.json'),
        classical_tolerances_changed=False, physical_acceptance_changed=False, point_vectors_changed=False,
        previous_cut_report=str(evidence.resolve()), previous_cut_report_sha256=file_sha256(evidence),
        previous_cut_matrices=str(matrices.resolve()), previous_cut_matrices_sha256=file_sha256(matrices),
        original_gate_preserved=True, optimizer_executed=False)
    write_new(directory/'protocol_addendum_01.json', output)
    return output


class ConstantEnvironmentDerivatives:
    """Explicit synthetic constant callbacks; no Hospital performance evidence."""

    def evaluate(self, xy, family):
        xy = np.asarray(xy, float)
        return (np.full(xy.shape[:-1], 10.), np.zeros_like(xy),
                dict(family=family, policy='synthetic constant 10m / zero derivative',
                     points=[dict(smooth=True, kind='constant', selected_segment_id=0)
                             for _ in xy.reshape(-1, 2)]))

    def workspace(self, xy): return self.evaluate(xy, 'workspace')
    def obstacle(self, xy): return self.evaluate(xy, 'obstacle')


def build_validation_points(primary, diag01, *, seeds=None):
    """Deterministic point set; never invokes an AD provider or optimizer."""
    from .gp_se2_diagnostics import load_frozen_case
    from .gp_se2_diag_fixtures import make_fixture_problem
    from .gp_se2_formulation import GPProblem
    from .gp_se2_diag02_environment import EnvironmentDerivatives
    from .se2 import compose_poses, se2_exp

    primary, diag01 = Path(primary), Path(diag01)
    rng = np.random.default_rng(PROTOCOL['random_seed'])
    perturbation = rng.normal(size=(30, 5))*np.asarray(PROTOCOL['small_perturbation_scales'])
    points, environment = [], None
    for short, method in (('M2', 'M2_GP_NO_OBSTACLE'), ('M3', 'M3_GP_CONSTRAINED')):
        case = load_frozen_case(primary, PROTOCOL['actual_case_id'], method, environment=environment)
        environment = case['environment']
        geometry = EnvironmentDerivatives(environment, radius=case['config']['footprint']['radius_m'])
        problem = case['problem']
        old = diag01/'actual_event/single_change'/method
        for init in (0, 1):
            source = old/f'start_{init:02d}.json'
            vector = np.asarray(json.loads(source.read_text())['initial_vector'], float)
            if seeds is not None:
                seed_file = Path(seeds)/('I0_FRESH.npy' if init == 0 else 'I1_DECEL.npy')
                if not np.array_equal(vector, np.load(seed_file, allow_pickle=False)):
                    raise ValueError('frozen derivative point differs from actual seed: '+str(seed_file))
            points.append(dict(name=f'actual_{short}_I{init}', problem=problem, geometry=geometry,
                               vector=vector, full_coordinate=True, mode='smooth_with_geometry_classification',
                               source=str(source), source_sha256=file_sha256(source), source_kind='saved actual event initialization'))
        source = diag01/'actual_event/baseline'/method/'start_00.json'
        vector = np.asarray(json.loads(source.read_text())['latest_iterate'], float)
        points.append(dict(name=f'rejected_{short}_start0_latest', problem=problem, geometry=geometry,
                           vector=vector, full_coordinate=True, mode='smooth_with_geometry_classification',
                           source=str(source), source_sha256=file_sha256(source), source_kind='saved rejected actual iterate; never a candidate'))
        original = next(p['vector'] for p in points if p['name'] == f'actual_{short}_I0')
        points.append(dict(name=f'actual_{short}_I0_small_perturbation', problem=problem, geometry=geometry,
                           vector=original+perturbation.ravel(), full_coordinate=False,
                           mode='smooth_with_geometry_classification', perturbation=perturbation.ravel(),
                           source_kind='deterministic derivative diagnostic around saved actual initialization'))
    for name in ('S0', 'S1', 'S2', 'S3'):
        problem, fixture = make_fixture_problem(name)
        points.append(dict(name=name, problem=problem, geometry=ConstantEnvironmentDerivatives(),
                           vector=fixture['known_vector'], full_coordinate=True, mode='smooth',
                           source_kind='SYNTHETIC derivative diagnostic, not performance evidence'))

    def custom(name, boundary, twist, *, delta=None, mode='smooth'):
        times = np.linspace(0, 3, 31)
        poses = compose_poses(boundary, se2_exp(times[:, None]*np.asarray(twist)))
        free = lambda xy: np.full(np.asarray(xy).shape[:-1], 10.)
        problem = GPProblem(boundary, twist, poses[1:], poses[-1], {}, free, free, [], True)
        vector = problem.vector(poses, np.tile(twist, (31, 1)))
        if delta is not None: vector += np.asarray(delta).reshape(-1)
        points.append(dict(name=name, problem=problem, geometry=ConstantEnvironmentDerivatives(), vector=vector,
                           full_coordinate=False, mode=mode, source_kind='SYNTHETIC chart/branch diagnostic'))

    custom('nonzero_chart_arbitrary_world', [2.3, -1.7, .83], [.3, 0, .4],
           delta=np.tile([.013, -.009, .017, .005, -.007], (30, 1)))
    custom('rotation_only_yaw_wrap', [.7, -.2, float(np.pi-.06)], [0, 0, .6])
    custom('linear_speed_bound', [.9, -.4, -.51], [.8, 0, 0])
    for name, angle in (('exp_log_small_angle_below', .9e-4), ('exp_log_small_angle_above', 1.1e-4),
                        ('right_jacobian_small_angle_below', .9e-3), ('right_jacobian_small_angle_above', 1.1e-3)):
        delta = np.zeros((30, 5)); delta[:, 2] = angle
        custom(name, [1.2, -.7, .41], [.3, 0, angle/.1], delta=delta)
    delta = np.zeros((30, 5)); delta[0, 2] = np.pi
    custom('relative_yaw_wrap_cut', [.2, .5, .73], [.3, 0, 0], delta=delta,
           mode='nonsmooth_relative_log_wrap_cut')
    expected = set(PROTOCOL['full_coordinate_points']+PROTOCOL['directional_additional_points'])
    if {p['name'] for p in points} != expected:
        raise ValueError('constructed point inventory differs from frozen protocol')
    return points


def point_record(point):
    problem = point['problem']
    return dict({k: v for k, v in point.items() if k not in ('problem', 'geometry')},
                boundary_pose=problem.boundary_pose, initial_twist=problem.initial_twist,
                common_reference=problem.common_reference, goal_pose=problem.goal_pose,
                support_times=problem.times, config=problem.config, gates=problem.gates,
                include_obstacles=problem.include_obstacles,
                chart_units=['m', 'm', 'rad', 'm/s', 'rad/s'], geometry_class=type(point['geometry']).__name__)


def freeze_points(directory, points):
    directory = Path(directory)
    frozen = json.loads((directory/'protocol.json').read_text())
    if any(frozen.get(k) != v for k, v in PROTOCOL.items()):
        raise ValueError('frozen derivative protocol differs from code; never silently update it')
    target = directory/'points'
    target.mkdir(exist_ok=False)
    rng = np.random.default_rng(PROTOCOL['random_seed'])
    directions = rng.normal(size=(PROTOCOL['direction_count'], 150))
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    write_new(directory/'directions.json', dict(seed=PROTOCOL['random_seed'], directions=directions,
                                               normalization=PROTOCOL['direction_normalization']))
    records = []
    for point in points:
        path = target/(point['name']+'.json')
        write_new(path, point_record(point))
        records.append(dict(name=point['name'], path=str(path.relative_to(directory)), sha256=file_sha256(path),
                            full_coordinate=point['full_coordinate'], mode=point['mode']))
    manifest = dict(frozen_utc=datetime.now(timezone.utc).isoformat(), protocol_sha256=file_sha256(directory/'protocol.json'),
                    direction_sha256=file_sha256(directory/'directions.json'), points=records, optimizer_executed=False)
    write_new(directory/'point_manifest.json', manifest)
    return manifest


def constraint_families(problem):
    """Exact original flat row ordering with residual physical units."""
    if problem.gates:
        raise ValueError('nonempty gates are outside the frozen DIAG-02 actual-event scope')
    n = 3*(len(problem.times)-1)
    families = [dict(name='objective', field='objective', start=0, stop=1, unit='dimensionless'),
                dict(name='lateral_midpoint', field='equality', start=0, stop=len(problem.times)-1, unit='m/s')]
    for i, (name, unit) in enumerate((('linear_speed_lower', 'm/s'), ('linear_speed_upper', 'm/s'),
                                    ('angular_speed_upper', 'rad/s'), ('angular_speed_lower', 'rad/s'),
                                    ('linear_acceleration_upper', 'm/s^2'), ('linear_acceleration_lower', 'm/s^2'),
                                    ('angular_acceleration_upper', 'rad/s^2'), ('angular_acceleration_lower', 'rad/s^2'))):
        families.append(dict(name=name, field='inequality', start=i*n, stop=(i+1)*n, unit=unit))
    start = 8*n
    families.extend([dict(name='goal_squared_distance', field='inequality', start=start, stop=start+1, unit='m^2'),
                     dict(name='goal_yaw_upper_lower', field='inequality', start=start+1, stop=start+3, unit='rad')])
    start += 3
    if problem.workspace_margin is not None:
        families.append(dict(name='workspace', field='inequality', start=start, stop=start+n, unit='m'))
        start += n
    if problem.include_obstacles:
        families.append(dict(name='obstacle', field='inequality', start=start, stop=start+n, unit='m'))
    return families


def error_report(actual, reference, tolerance, *, mask=None, row_offset=0):
    """Elementwise prescribed scaled error; report actual values at worst site."""
    a, b = np.asarray(actual, float), np.asarray(reference, float)
    if a.shape != b.shape or not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)):
        return dict(passed=False, finite=False, shape_actual=list(a.shape), shape_reference=list(b.shape),
                    compared_elements=0, maximum_absolute_error=None, maximum_scaled_error=None)
    valid = np.ones(a.shape, bool) if mask is None else np.broadcast_to(mask, a.shape)
    absolute = np.abs(a-b)
    scale = tolerance['atol']+tolerance['rtol']*np.maximum(np.abs(a), np.abs(b))
    scaled = absolute/scale
    if not np.any(valid):
        return dict(passed=None, finite=True, compared_elements=0, excluded_elements=int(a.size),
                    maximum_absolute_error=None, maximum_scaled_error=None, reason='no classical smooth comparison in this group')
    flat = int(np.argmax(np.where(valid, scaled, -np.inf)))
    index = np.unravel_index(flat, a.shape)
    return dict(passed=bool(np.max(scaled[valid])<=1), finite=True,
                compared_elements=int(np.sum(valid)), excluded_elements=int(a.size-np.sum(valid)),
                maximum_absolute_error=float(np.max(absolute[valid])), maximum_scaled_error=float(scaled[index]),
                worst_row=int(index[0]+row_offset) if len(index)>1 else int(row_offset),
                worst_column=int(index[-1]), actual_at_worst=float(a[index]), reference_at_worst=float(b[index]),
                allowed_error_at_worst=float(scale[index]), tolerance=dict(tolerance))


def _values(problem, vector):
    value = problem.evaluate(vector)
    return {name: np.atleast_1d(np.asarray(value[name], float)).copy()
            for name in ('objective', 'equality', 'inequality')}


def _geometry_at(problem, geometry, vector):
    from .gp_se2 import interpolate_interval
    poses, twists = problem.unpack(vector)
    locations = interpolate_interval(poses[:-1, None], twists[:-1, None], poses[1:, None], twists[1:, None],
                                     problem.config['support_dt_s'], np.array([0., .5, 1.]))[0].reshape(-1, 3)[:, :2]
    output = {}
    for family, applicable in (('workspace', problem.workspace_margin is not None), ('obstacle', problem.include_obstacles)):
        if applicable:
            output[family] = getattr(geometry, family)(locations)[2]
    return output


def _branch_signature(item):
    return (item.get('kind'), item.get('selected_segment_id'), tuple(item.get('active_segment_ids') or []),
            tuple(item.get('cell_xy') or []), bool(item.get('in_grid_domain', True)), bool(item.get('invalid_query', False)))


def _smooth_rows(problem, center, plus, minus):
    # Row count is structural; no numerical evaluation is needed for a mask.
    size = 723+90*int(problem.workspace_margin is not None)+90*int(problem.include_obstacles)
    mask = np.ones(size, bool)
    for family in constraint_families(problem):
        name = family['name']
        if name not in ('workspace', 'obstacle'):
            continue
        a, b, c = [metadata[name]['points'] for metadata in (center, plus, minus)]
        if not len(a) == len(b) == len(c) == 90:
            raise ValueError('geometry metadata must classify every original collocation point')
        mask[family['start']:family['stop']] = [bool(x.get('smooth')) and bool(y.get('smooth')) and bool(z.get('smooth'))
            and _branch_signature(x) == _branch_signature(y) == _branch_signature(z) for x, y, z in zip(a, b, c)]
    return mask


def _family_reports(problem, actual, reference, masks, *, primal=False):
    output = []
    for family in constraint_families(problem):
        field, start, stop = family['field'], family['start'], family['stop']
        tolerance = PROTOCOL['primal_tolerance'] if primal else PROTOCOL[
            'objective_derivative_tolerance' if field == 'objective' else 'constraint_derivative_tolerance']
        a, b = np.asarray(actual[field])[start:stop], np.asarray(reference[field])[start:stop]
        mask = None if masks is None else masks[field][start:stop]
        report = error_report(a, b, tolerance, mask=mask, row_offset=start)
        output.append(dict(family=family['name'], residual_unit=family['unit'], field=field,
                           derivative_column_units='original chart [m,m,rad,m/s,rad/s] repeated; directional columns are normalized mixed-unit directions',
                           **report))
    return output


def _reports_pass(reports):
    return all(row['passed'] is not False for row in reports)


def validate_point(point, provider, directions):
    """Validate one frozen point; returns small report plus raw matrix arrays."""
    started = time.perf_counter()
    problem, geometry, z = point['problem'], point['geometry'], np.asarray(point['vector'], float)
    provider.warmup(z)
    ad_value = provider.values(z)
    original = _values(problem, z)
    ad = dict(objective=np.asarray(provider.objective_gradient(z))[None, :],
              equality=np.asarray(provider.equality_jacobian(z)), inequality=np.asarray(provider.inequality_jacobian(z)))
    primal = {name: np.atleast_1d(np.asarray(ad_value[name], float))[:, None] for name in original}
    expected = {name: value[:, None] for name, value in original.items()}
    primal_reports = _family_reports(problem, primal, expected, None, primal=True)
    finite_shape = all(ad[name].shape == (len(original[name]), 150) and np.all(np.isfinite(ad[name])) for name in ad)
    center_meta = _geometry_at(problem, geometry, z)
    nonsmooth = point['mode'] == 'nonsmooth_relative_log_wrap_cut'
    arrays = dict(vector=z, directions=directions,
                  objective_gradient=ad['objective'], equality_jacobian=ad['equality'], inequality_jacobian=ad['inequality'])
    coordinate_reports = []
    coordinate_pass = True
    if point['full_coordinate']:
        h = PROTOCOL['coordinate_fd_step']
        fd = {name: np.empty_like(matrix) for name, matrix in ad.items()}
        masks = {name: np.ones_like(matrix, bool) for name, matrix in ad.items()}
        for column in range(150):
            delta = np.zeros(150); delta[column] = h
            plus, minus = _values(problem, z+delta), _values(problem, z-delta)
            for name in fd:
                fd[name][:, column] = (plus[name]-minus[name])/(2*h)
            plus_meta, minus_meta = _geometry_at(problem, geometry, z+delta), _geometry_at(problem, geometry, z-delta)
            masks['inequality'][:, column] = _smooth_rows(problem, center_meta, plus_meta, minus_meta)
        # A fixed support can map a nonsmooth world-distance function to a
        # constant optimization row. Exact-zero derivatives are checkable.
        masks['inequality'] |= (ad['inequality'] == 0) & (fd['inequality'] == 0)
        coordinate_reports = _family_reports(problem, ad, fd, masks)
        coordinate_pass = _reports_pass(coordinate_reports)
        arrays.update({f'coordinate_fd_{name}': value for name, value in fd.items()})
        arrays.update({f'coordinate_smooth_mask_{name}': value for name, value in masks.items()})
    directional = []
    for step_index, h in enumerate(PROTOCOL['directional_fd_steps']):
        prediction = {name: matrix@directions.T for name, matrix in ad.items()}
        fd = {name: np.empty_like(matrix) for name, matrix in prediction.items()}
        sided_plus = {name: np.empty_like(matrix) for name, matrix in prediction.items()}
        sided_minus = {name: np.empty_like(matrix) for name, matrix in prediction.items()}
        masks = {name: np.ones_like(matrix, bool) for name, matrix in prediction.items()}
        for column, direction in enumerate(directions):
            plus, minus = _values(problem, z+h*direction), _values(problem, z-h*direction)
            for name in fd:
                fd[name][:, column] = (plus[name]-minus[name])/(2*h)
                sided_plus[name][:, column] = (plus[name]-original[name])/h
                sided_minus[name][:, column] = (original[name]-minus[name])/h
            plus_meta = _geometry_at(problem, geometry, z+h*direction)
            minus_meta = _geometry_at(problem, geometry, z-h*direction)
            masks['inequality'][:, column] = _smooth_rows(problem, center_meta, plus_meta, minus_meta)
        masks['inequality'] |= (prediction['inequality'] == 0) & (fd['inequality'] == 0)
        if nonsmooth:
            masks = {name: np.zeros_like(matrix, bool) for name, matrix in prediction.items()}
        smooth_reports = _family_reports(problem, prediction, fd, masks)
        one_sided_reports = []
        for name in prediction:
            # Keep both limits and selected-branch comparisons; selecting the
            # closest side is descriptive, never evidence of a classical
            # derivative at a cut or geometry tie.
            take_plus = np.abs(prediction[name]-sided_plus[name]) <= np.abs(prediction[name]-sided_minus[name])
            closest = np.where(take_plus, sided_plus[name], sided_minus[name])
            for family in [f for f in constraint_families(problem) if f['field'] == name]:
                start, stop = family['start'], family['stop']
                report = error_report(prediction[name][start:stop], closest[start:stop],
                    PROTOCOL['objective_derivative_tolerance' if name == 'objective' else 'constraint_derivative_tolerance'],
                    mask=~masks[name][start:stop], row_offset=start)
                one_sided_reports.append(dict(family=family['name'], **report,
                    scope='descriptive closest one-sided difference; no classical differentiability assertion; no relaxation of smooth acceptance'))
            arrays[f'directional_{step_index}_{name}_prediction'] = prediction[name]
            arrays[f'directional_{step_index}_{name}_central_fd'] = fd[name]
            arrays[f'directional_{step_index}_{name}_plus_fd'] = sided_plus[name]
            arrays[f'directional_{step_index}_{name}_minus_fd'] = sided_minus[name]
            arrays[f'directional_{step_index}_{name}_smooth_mask'] = masks[name]
        directional.append(dict(step=h, smooth_reports=smooth_reports,
                                smooth_pass=_reports_pass(smooth_reports), nonsmooth_one_sided_characterization=one_sided_reports))
    direction_pass = all(row['smooth_pass'] for row in directional[-2:])
    provider_stats = provider.stats()
    provider_contract = bool(provider_stats.get('float64_enabled') is True and provider_stats.get('device_backend') == 'cpu'
                             and provider_stats.get('numerical_finite_difference_calls') == 0
                             and provider_stats.get('numerical_fallback_used') is False)
    result = dict(name=point['name'], mode=point['mode'], full_coordinate=point['full_coordinate'],
                  required_primal_pass=_reports_pass(primal_reports), primal_reports=primal_reports,
                  finite_full_jacobian_shapes=bool(finite_shape), coordinate_pass=coordinate_pass,
                  coordinate_reports=coordinate_reports, directional_pass=direction_pass, directional=directional,
                  geometry_metadata=center_meta, provider_geometry_metadata=provider.geometry_metadata(z),
                  provider_stats=provider_stats, provider_contract_valid=provider_contract, wall_time_s=time.perf_counter()-started,
                  nonsmooth_classical_derivative_claimed=False,
                  optimizer_executed=False)
    result['valid'] = bool(result['required_primal_pass'] and finite_shape and coordinate_pass and direction_pass and provider_contract)
    return result, arrays


def validate_unsupported_cut(point, provider, addendum):
    """A known unsupported derivative domain must fail explicitly, not fall back."""
    from .gp_se2_diag02_derivatives import DerivativeError
    if point['name'] != addendum['expected_unsupported_point'] or point['mode'] != 'nonsmooth_relative_log_wrap_cut':
        raise ValueError('unsupported-cut exception does not apply to this frozen point')
    for key in ('previous_cut_report', 'previous_cut_matrices'):
        if file_sha256(addendum[key]) != addendum[key+'_sha256']:
            raise ValueError('preserved cut evidence changed')
    original = _values(point['problem'], point['vector'])
    original_finite = all(np.all(np.isfinite(value)) for value in original.values())
    reason, correctly_rejected, guard_diagnostics = None, False, None
    try:
        provider.warmup(point['vector'])
        provider.values(point['vector'])
    except DerivativeError as exc:
        reason = str(exc)
        guard_diagnostics = getattr(exc, 'diagnostics', None)
        correctly_rejected = getattr(exc, 'reason_code', None) == 'UNSUPPORTED_WRAP_CUT'
    stats = provider.stats()
    no_fallback = stats.get('numerical_finite_difference_calls') == 0 and stats.get('numerical_fallback_used') is False
    return dict(name=point['name'], mode=point['mode'], valid=bool(correctly_rejected and original_finite and no_fallback),
        full_coordinate=False, supported_derivative_domain=False, expected_runtime_rejection=True,
        correctly_rejected=correctly_rejected, rejection_reason=reason, original_values_finite=bool(original_finite),
        guard_diagnostics=guard_diagnostics,
        primal_reports=[], coordinate_reports=[], directional=[], provider_stats=stats,
        previous_cut_report=addendum['previous_cut_report'], previous_cut_report_sha256=addendum['previous_cut_report_sha256'],
        previous_cut_matrices=addendum['previous_cut_matrices'], previous_cut_matrices_sha256=addendum['previous_cut_matrices_sha256'],
        numerical_arrays='preserved pre-guard cut diagnostics linked above; new provider intentionally produces no derivative there',
        nonsmooth_classical_derivative_claimed=False, optimizer_executed=False)


def run_validation(directory, points, *, attempt='attempt_01', geometry_report=None):
    """Run checks only after protocol and exact numeric point vectors are frozen."""
    from .gp_se2_diag02_derivatives import DerivativeProvider
    import shutil
    directory = Path(directory)
    expected_names = set(PROTOCOL['full_coordinate_points']+PROTOCOL['directional_additional_points'])
    if len(points) != len(expected_names) or {point['name'] for point in points} != expected_names:
        raise ValueError('required full-coverage point inventory is incomplete or duplicated')
    frozen_protocol = json.loads((directory/'protocol.json').read_text())
    if any(frozen_protocol.get(key) != value for key, value in PROTOCOL.items()):
        raise ValueError('derivative protocol changed after freeze')
    manifest = json.loads((directory/'point_manifest.json').read_text())
    addendum = json.loads((directory/'protocol_addendum_01.json').read_text())
    if (addendum['original_protocol_sha256'] != file_sha256(directory/'protocol.json') or
            addendum['frozen_point_manifest_sha256'] != file_sha256(directory/'point_manifest.json')):
        raise ValueError('runtime-cut addendum refers to different frozen evidence')
    if manifest['protocol_sha256'] != file_sha256(directory/'protocol.json'):
        raise ValueError('frozen protocol hash changed')
    frozen_points = {row['name']: row for row in manifest['points']}
    for point in points:
        entry = frozen_points[point['name']]
        path = directory/entry['path']
        if file_sha256(path) != entry['sha256'] or json.loads(path.read_text()) != plain(point_record(point)):
            raise ValueError('numeric point definition changed: '+point['name'])
    target = directory/attempt
    target.mkdir(exist_ok=False)
    source = target/'implementation_sources'; source.mkdir()
    hashes = {}
    for filename in ('gp_se2_diag02_validation.py', 'gp_se2_diag02_derivatives.py', 'gp_se2_diag02_ad.py',
                     'gp_se2_diag02_environment.py', 'gp_se2.py', 'gp_se2_formulation.py', 'se2.py'):
        path = Path(__file__).parent/filename
        shutil.copy2(path, source/filename); hashes[filename] = file_sha256(path)
    write_new(target/'source.json', dict(source_sha256=hashes, protocol_sha256=file_sha256(directory/'protocol.json'),
              point_manifest_sha256=file_sha256(directory/'point_manifest.json'),
              protocol_addendum_sha256=file_sha256(directory/'protocol_addendum_01.json'), start_utc=datetime.now(timezone.utc).isoformat()))
    directions_file = directory/'directions.json'
    if file_sha256(directions_file) != manifest['direction_sha256']:
        raise ValueError('direction probes changed after freeze')
    directions = np.asarray(json.loads(directions_file.read_text())['directions'], float)
    providers, rows = {}, []
    for point in points:
        key = id(point['problem'])
        try:
            if key not in providers:
                providers[key] = DerivativeProvider(point['problem'], point['geometry'])
            if point['name'] == addendum['expected_unsupported_point']:
                result = validate_unsupported_cut(point, providers[key], addendum)
            else:
                result, arrays = validate_point(point, providers[key], directions)
                with (target/(point['name']+'.npz')).open('xb') as stream:
                    np.savez_compressed(stream, **arrays)
                result['matrix_file'] = point['name']+'.npz'
                result['matrix_sha256'] = file_sha256(target/result['matrix_file'])
        except Exception as exc:
            import traceback
            result = dict(name=point['name'], valid=False, error=f'{type(exc).__name__}: {exc}',
                          traceback=traceback.format_exc(), optimizer_executed=False)
        write_new(target/(point['name']+'.json'), result)
        rows.append(result)
        print(json.dumps(dict(point=point['name'], valid=result['valid'], error=result.get('error'),
                              wall_time_s=result.get('wall_time_s'))), flush=True)
    geometry = None if geometry_report is None else json.loads(Path(geometry_report).read_text())
    geometry_valid = bool(geometry is not None and geometry.get('valid') is True)
    summary = dict(valid=all(row['valid'] for row in rows) and geometry_valid,
        point_count=len(rows), full_coordinate_point_count=sum(p['full_coordinate'] for p in points),
        supported_points_validated=sum(row['valid'] and not row.get('expected_runtime_rejection', False) for row in rows),
        unsupported_points_correctly_rejected=sum(row.get('correctly_rejected', False) for row in rows),
        failed_points=[row['name'] for row in rows if not row['valid']],
        geometry_nonsmooth_report=None if geometry_report is None else str(geometry_report),
        geometry_report_sha256=None if geometry_report is None else file_sha256(geometry_report),
        geometry_separate_checks_valid=geometry_valid, protocol_sha256=file_sha256(directory/'protocol.json'),
        point_manifest_sha256=file_sha256(directory/'point_manifest.json'),
        protocol_addendum_sha256=file_sha256(directory/'protocol_addendum_01.json'),
        no_finite_difference_fallback=True, actual_solver_authorized_by_this_gate=all(row['valid'] for row in rows) and geometry_valid,
        completed_utc=datetime.now(timezone.utc).isoformat(), optimizer_executed=False,
        nonsmooth_scope='piecewise branch diagnostics do not prove classical differentiability at cuts/ties/boundaries')
    write_new(target/'summary.json', summary)
    return summary


def publish_validation_gate(directory, attempt, *, filename='authoritative_validation.json',
                            allow_publication_only_correction=False):
    """Publish one exclusive successful gate only while checked source matches."""
    directory = Path(directory)
    target = directory/attempt
    summary = json.loads((target/'summary.json').read_text())
    if summary.get('valid') is not True or summary.get('actual_solver_authorized_by_this_gate') is not True:
        raise ValueError('failed derivative attempt cannot authorize an actual solve')
    source = json.loads((target/'source.json').read_text())
    publication_corrections = []
    for source_filename, expected in source['source_sha256'].items():
        current_source = Path(__file__).parent/source_filename
        current_hash = file_sha256(current_source)
        if current_hash != expected:
            archived_source = target/'implementation_sources'/source_filename
            only_publisher_changed = False
            if allow_publication_only_correction and source_filename == Path(__file__).name and file_sha256(archived_source) == expected:
                import ast
                def without_publisher(path):
                    tree = ast.parse(path.read_text())
                    tree.body = [node for node in tree.body if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
                                 or node.name != 'publish_validation_gate']
                    return ast.dump(tree, include_attributes=False)
                only_publisher_changed = without_publisher(archived_source) == without_publisher(current_source)
            if not only_publisher_changed:
                raise ValueError('checked derivative source changed before publication: '+source_filename)
            publication_corrections.append(dict(source=source_filename, checked_sha256=expected,
                current_sha256=current_hash, scope='only publish_validation_gate differs; all other module AST nodes exactly equal',
                reason='output filename parameter shadowed by source-file loop; no numerical criterion or derivative check changed'))
    if source['protocol_sha256'] != file_sha256(directory/'protocol.json') or source['point_manifest_sha256'] != file_sha256(directory/'point_manifest.json'):
        raise ValueError('frozen protocol or point manifest changed before gate publication')
    if source['protocol_addendum_sha256'] != file_sha256(directory/'protocol_addendum_01.json'):
        raise ValueError('runtime-cut addendum changed before gate publication')
    if summary.get('supported_points_validated') != 19 or summary.get('unsupported_points_correctly_rejected') != 1:
        raise ValueError('required supported/explicitly-unsupported domain coverage was not verified')
    reports = [json.loads(path.read_text()) for path in sorted(target.glob('*.json'))
               if path.name not in ('source.json', 'summary.json')]
    classical = [row for report in reports for row in report.get('coordinate_reports', [])
                 if row.get('compared_elements', 0)>0]
    directional = [row for report in reports for step in report.get('directional', [])[-2:]
                   for row in step['smooth_reports'] if row.get('compared_elements', 0)>0]
    primal = [row for report in reports for row in report.get('primal_reports', [])]
    gate = dict(summary, authoritative_attempt=attempt,
                attempt_summary_sha256=file_sha256(target/'summary.json'),
                attempt_source_sha256=file_sha256(target/'source.json'),
                checked_source_sha256=source['source_sha256'],
                publication_only_source_corrections=publication_corrections,
                maximum_primal_absolute_error=max(row['maximum_absolute_error'] for row in primal),
                maximum_required_derivative_scaled_error=max(row['maximum_scaled_error'] for row in classical+directional),
                coordinate_comparison_elements=sum(row['compared_elements'] for row in classical),
                coordinate_excluded_elements=sum(row.get('excluded_elements', 0) for row in classical),
                nondifferentiable_log_cut='explicit runtime DerivativeError verified; prior one-sided mismatch evidence preserved; no derivative or fallback produced at the cut',
                published_utc=datetime.now(timezone.utc).isoformat())
    write_new(directory/filename, gate)
    return gate

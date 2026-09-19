"""Frozen derivative-domain transfer smoke; never a trajectory optimization.

The DIAG-02 provider, original primal evaluator, tolerances, directions and
multi-step rules are reused. A deterministic nearby point is a compatibility
probe, not another performance sample or an initialization/restoration option.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time
import traceback

import numpy as np

from .gp_se2_diag02_validation import (
    PROTOCOL as DIAG02_PROTOCOL, file_sha256, plain, point_record,
    validate_point, write_new,
)


FIXED_CASE_IDS = (
    'episode_001_repeat_01/handoff_002',
    'episode_013_repeat_01/handoff_024',
    'episode_014_repeat_01/handoff_026',
    'episode_000_repeat_01/handoff_019',
)
METHODS = ('M2_GP_NO_OBSTACLE', 'M3_GP_CONSTRAINED')
INITIALIZATIONS = ('I0_FRESH', 'I1_DECEL')
PROBES = ('base', 'fixed_perturbation')
SOURCE_NAMES = (
    'gp_se2_diag02_validation.py', 'gp_se2_diag02_derivatives.py',
    'gp_se2_diag02_ad.py', 'gp_se2_diag02_environment.py', 'gp_se2.py',
    'gp_se2_formulation.py', 'se2.py', 'gp_se2_diag_initialization.py',
    'gp_se2_diagnostics.py', 'gp_se2_02_transfer.py',
)
TRANSFER_PROTOCOL = {
    'schema_version': 1,
    'experiment': 'GP-SE2-02',
    'fixed_case_ids': list(FIXED_CASE_IDS),
    'method_order': list(METHODS),
    'initialization_order': list(INITIALIZATIONS),
    'probe_order': list(PROBES),
    'point_count': 32,
    'start_count': 16,
    'random_seed': DIAG02_PROTOCOL['random_seed'],
    'direction_count': DIAG02_PROTOCOL['direction_count'],
    'direction_normalization': DIAG02_PROTOCOL['direction_normalization'],
    'directional_fd_steps': DIAG02_PROTOCOL['directional_fd_steps'],
    'primal_tolerance': DIAG02_PROTOCOL['primal_tolerance'],
    'objective_derivative_tolerance': DIAG02_PROTOCOL['objective_derivative_tolerance'],
    'constraint_derivative_tolerance': DIAG02_PROTOCOL['constraint_derivative_tolerance'],
    'scaled_error_definition': DIAG02_PROTOCOL['scaled_error_definition'],
    'smooth_direction_acceptance': DIAG02_PROTOCOL['smooth_direction_acceptance'],
    'nonsmooth_policy': DIAG02_PROTOCOL['nonsmooth_policy'],
    'small_perturbation_scales': DIAG02_PROTOCOL['small_perturbation_scales'],
    'perturbation_policy': 'one identical fixed 150-vector for every case/method/start; independent RNG with original DIAG-02 seed; never used for optimization',
    'full_coordinate_sweep_repeated': False,
    'start_authorization': 'VERIFIED only when base and its fixed perturbation both pass original primal/full derivative shape/two finest smooth directional checks; unsupported either point blocks only that planned start',
    'gate_policy': 'record UNSUPPORTED_GATE_CASE; never delete a gate or add gate derivatives',
    'failure_policy': 'retain all points and starts; no threshold/provider/formulation change or finite-difference solver fallback',
    'nonclassical_policy': 'geometry branch changes and ties remain separate from classical central PASS; preserve both one-sided arrays and selected branch metadata under original DIAG-02 policy',
    'optimizer_executed': False,
    'performance_sample_scope': 'four previously collected events, not 32 independent performance samples',
}


def vector_sha256(vector):
    """Hash exact float64 chart bytes, distinct from an .npy container hash."""
    value = np.ascontiguousarray(vector, dtype=np.float64)
    if value.shape != (150,) or not np.all(np.isfinite(value)):
        raise ValueError('original finite 150-variable float64 vector required')
    return hashlib.sha256(value.tobytes()).hexdigest()


def seed_vectors(problem):
    """The two prescribed original-chart seeds, without fitting or validation."""
    from .gp_se2_diag_initialization import same_curvature_deceleration_seed
    initial = problem.initializations()[0]
    deceleration = same_curvature_deceleration_seed(problem)
    output = {}
    for name, state in zip(INITIALIZATIONS, (initial, deceleration)):
        vector = problem.vector(state['poses'], state['twists'])
        poses, twists = problem.unpack(vector)
        if poses[0].tobytes() != problem.boundary_pose.tobytes():
            raise ValueError('fixed boundary pose changed')
        if twists[0].tobytes() != problem.initial_twist.tobytes():
            raise ValueError('fixed physical initial twist changed')
        output[name] = np.asarray(vector, dtype=np.float64)
        vector_sha256(output[name])
    return output


def _source_hashes():
    directory = Path(__file__).parent
    return {name: file_sha256(directory/name) for name in SOURCE_NAMES}


def verify_diag02_authority(diag02):
    """Verify final evidence and exact current numerical source, read-only."""
    diag02 = Path(diag02).resolve()
    root_path = diag02/'validation.json'
    gate_path = diag02/'derivative_checks/authoritative_validation.json'
    root, gate = (json.loads(path.read_text()) for path in (root_path, gate_path))
    if root.get('valid') is not True or gate.get('valid') is not True:
        raise ValueError('final DIAG-02 artifact and derivative validation must pass')
    if (gate.get('supported_points_validated') != 19 or
            gate.get('unsupported_points_correctly_rejected') != 1 or
            gate.get('no_finite_difference_fallback') is not True):
        raise ValueError('DIAG-02 authoritative derivative coverage differs')
    corrections = {row['source']: row for row in gate.get('publication_only_source_corrections', [])}
    current = _source_hashes()
    for name, checked in gate['checked_source_sha256'].items():
        if current.get(name) == checked:
            continue
        correction = corrections.get(name, {})
        if (name != 'gp_se2_diag02_validation.py' or
                correction.get('checked_sha256') != checked or
                correction.get('current_sha256') != current.get(name)):
            raise ValueError('verified derivative numerical source changed: '+name)
        # Independently limit the old documented exception to its publisher.
        import ast
        original = diag02/'derivative_checks'/gate['authoritative_attempt']/'implementation_sources'/name
        if file_sha256(original) != checked:
            raise ValueError('original checked derivative source archive changed')
        def mathematical_ast(path):
            tree = ast.parse(path.read_text())
            tree.body = [node for node in tree.body if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
                         or node.name != 'publish_validation_gate']
            return ast.dump(tree, include_attributes=False)
        if mathematical_ast(original) != mathematical_ast(Path(__file__).parent/name):
            raise ValueError('publication exception contains a mathematical change')
    return dict(final_artifact_validation=str(root_path), final_artifact_validation_sha256=file_sha256(root_path),
                authoritative_derivative_validation=str(gate_path), authoritative_derivative_validation_sha256=file_sha256(gate_path),
                checked_source_sha256=gate['checked_source_sha256'], current_source_sha256=current,
                existing_publication_only_corrections=list(corrections.values()),
                original_validation_not_rebuilt=True)


def build_transfer_points(primary):
    """Resolve exactly the four original manifest cases; do not evaluate AD."""
    from .gp_se2_diagnostics import load_frozen_case
    from .gp_se2_diag02_environment import EnvironmentDerivatives
    perturbation = np.random.default_rng(TRANSFER_PROTOCOL['random_seed']).normal(size=(30, 5))
    perturbation *= np.asarray(TRANSFER_PROTOCOL['small_perturbation_scales'])
    environment, points, starts = None, [], []
    for case_index, case_id in enumerate(FIXED_CASE_IDS):
        case_seeds = None
        for method in METHODS:
            case = load_frozen_case(primary, case_id, method, environment=environment)
            environment = case['environment']
            problem = case['problem']
            geometry = EnvironmentDerivatives(environment, radius=case['config']['footprint']['radius_m'])
            seeds = seed_vectors(problem)
            if case_seeds is not None and any(seeds[name].tobytes() != case_seeds[name].tobytes() for name in INITIALIZATIONS):
                raise ValueError('M2/M3 seed vectors differ for '+case_id)
            case_seeds = seeds
            for initialization in INITIALIZATIONS:
                start_key = f'{case_id}/{method}/{initialization}'
                stem = f'case_{case_index:02d}_{method[:2]}_{initialization}'
                names = []
                for probe in PROBES:
                    name = stem+'_'+probe
                    vector = seeds[initialization].copy()
                    if probe == 'fixed_perturbation':
                        vector += perturbation.ravel()
                    names.append(name)
                    points.append(dict(name=name, case_id=case_id, method=method, initialization=initialization,
                        start_key=start_key, probe=probe, problem=problem, geometry=geometry, vector=vector,
                        full_coordinate=False, mode='smooth_with_geometry_classification',
                        perturbation=perturbation.ravel().copy() if probe == 'fixed_perturbation' else np.zeros(150),
                        original_source_hashes=case['hashes'], vector_sha256=vector_sha256(vector),
                        seed_vector_sha256=vector_sha256(seeds[initialization]),
                        source_kind='saved actual event, prescribed original initialization; perturbation is derivative diagnostic only'))
                starts.append(dict(start_key=start_key, case_id=case_id, method=method, initialization=initialization,
                    points=names, seed_vector_sha256=vector_sha256(seeds[initialization]),
                    original_gates=problem.gates, no_gate_scope=not problem.gates))
    return points, starts


def freeze_transfer(directory, primary, diag02):
    """Exclusive freeze of policy, probes, source evidence; no derivatives yet."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    authority = verify_diag02_authority(diag02)
    points, starts = build_transfer_points(primary)
    write_new(directory/'protocol.json', dict(TRANSFER_PROTOCOL, frozen_utc=datetime.now(timezone.utc).isoformat()))
    write_new(directory/'source.json', dict(authority, primary=str(Path(primary).resolve()),
        diag02=str(Path(diag02).resolve()), source_sha256=_source_hashes()))
    directions = np.random.default_rng(TRANSFER_PROTOCOL['random_seed']).normal(size=(3, 150))
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    write_new(directory/'directions.json', dict(directions=directions,
        seed=TRANSFER_PROTOCOL['random_seed'], normalization=TRANSFER_PROTOCOL['direction_normalization']))
    records = []
    for point in points:
        path = directory/'points'/(point['name']+'.json')
        write_new(path, point_record(point))
        records.append(dict(name=point['name'], path=str(path.relative_to(directory)), sha256=file_sha256(path)))
    manifest = dict(points=records, starts=starts, point_count=len(points), start_count=len(starts),
        protocol_sha256=file_sha256(directory/'protocol.json'), source_sha256=file_sha256(directory/'source.json'),
        directions_sha256=file_sha256(directory/'directions.json'), optimizer_executed=False)
    write_new(directory/'point_manifest.json', manifest)
    return manifest


def _unsupported_reason(exception):
    from .gp_se2_diag02_derivatives import DerivativeError
    from .gp_se2_diag02_environment import UnsupportedEnvironmentDerivative
    cause = exception
    while cause is not None:
        if isinstance(cause, UnsupportedEnvironmentDerivative):
            return 'UNSUPPORTED_ENVIRONMENT_DERIVATIVE'
        cause = cause.__cause__
    if isinstance(exception, DerivativeError) and exception.reason_code == 'UNSUPPORTED_WRAP_CUT':
        return exception.reason_code
    return None


def classify_start(point_reports, *, gates=()):
    """No unavailable point becomes a PASS and no unsupported start vanishes."""
    if gates:
        return 'UNSUPPORTED_GATE_CASE'
    if len(point_reports) != 2:
        raise ValueError('both frozen base and perturbation reports required')
    statuses = [report['status'] for report in point_reports]
    if 'DERIVATIVE_VALIDATION_FAILED' in statuses:
        return 'DERIVATIVE_VALIDATION_FAILED'
    if 'DERIVATIVE_UNSUPPORTED' in statuses:
        return 'DERIVATIVE_UNSUPPORTED'
    if statuses != ['VERIFIED', 'VERIFIED']:
        raise ValueError('unknown transfer status')
    return 'VERIFIED'


def run_transfer(directory, primary):
    """Check all 32 frozen points once and preserve every planned start result."""
    from .gp_se2_diag02_derivatives import DerivativeProvider
    directory = Path(directory)
    started = time.perf_counter()
    frozen = json.loads((directory/'protocol.json').read_text())
    if any(frozen.get(key) != value for key, value in TRANSFER_PROTOCOL.items()):
        raise ValueError('transfer protocol changed after freeze')
    source = json.loads((directory/'source.json').read_text())
    if source['source_sha256'] != _source_hashes():
        raise ValueError('transfer/provider/core source changed after freeze')
    if Path(primary).resolve() != Path(source['primary']):
        raise ValueError('original primary source changed')
    authority = verify_diag02_authority(source['diag02'])
    for key in ('final_artifact_validation_sha256', 'authoritative_derivative_validation_sha256'):
        if source[key] != authority[key]:
            raise ValueError('authoritative derivative evidence changed')
    manifest = json.loads((directory/'point_manifest.json').read_text())
    for key, name in (('protocol_sha256', 'protocol.json'), ('source_sha256', 'source.json'), ('directions_sha256', 'directions.json')):
        if manifest[key] != file_sha256(directory/name):
            raise ValueError('frozen transfer '+name+' changed')
    points, starts = build_transfer_points(primary)
    if plain(starts) != manifest['starts'] or len(points) != len(manifest['points']):
        raise ValueError('frozen transfer start inventory changed')
    for point, record in zip(points, manifest['points']):
        path = directory/record['path']
        if (point['name'] != record['name'] or file_sha256(path) != record['sha256'] or
                json.loads(path.read_text()) != plain(point_record(point))):
            raise ValueError('frozen original chart point changed: '+point['name'])
    directions = np.asarray(json.loads((directory/'directions.json').read_text())['directions'], dtype=np.float64)
    target = directory/'results'
    target.mkdir(exist_ok=False)
    providers, reports = {}, {}
    for point in points:
        provider = None
        try:
            if point['problem'].gates:
                result = dict(valid=False, status='UNSUPPORTED_GATE_CASE', reason_code='UNSUPPORTED_GATE_CASE',
                    original_gates=point['problem'].gates, no_gate_removed=True, optimizer_executed=False)
            else:
                key = id(point['problem'])
                if key not in providers:
                    providers[key] = DerivativeProvider(point['problem'], point['geometry'])
                provider = providers[key]
                result, arrays = validate_point(point, provider, directions)
                result['status'] = 'VERIFIED' if result['valid'] else 'DERIVATIVE_VALIDATION_FAILED'
                path = target/(point['name']+'.npz')
                with path.open('xb') as stream:
                    np.savez_compressed(stream, **arrays)
                result.update(matrix_file=path.name, matrix_sha256=file_sha256(path),
                    central_nonsmooth_exclusions=sum(row.get('excluded_elements', 0) for step in result['directional']
                        for row in step['smooth_reports']),
                    excluded_rows_are_not_classical_passes=True,
                    coordinate_sweep_not_repeated=True)
        except Exception as exc:
            unsupported = _unsupported_reason(exc)
            result = dict(valid=False, status='DERIVATIVE_UNSUPPORTED' if unsupported else 'DERIVATIVE_VALIDATION_FAILED',
                reason_code=unsupported or getattr(exc, 'reason_code', type(exc).__name__),
                error=f'{type(exc).__name__}: {exc}', guard_diagnostics=getattr(exc, 'diagnostics', None),
                traceback=traceback.format_exc(), optimizer_executed=False,
                provider_stats=provider.stats() if provider is not None else None,
                no_finite_difference_fallback=True)
        result.update(name=point['name'], case_id=point['case_id'], method=point['method'],
            initialization=point['initialization'], probe=point['probe'], start_key=point['start_key'],
            vector_sha256=point['vector_sha256'], seed_vector_sha256=point['seed_vector_sha256'])
        write_new(target/(point['name']+'.json'), result)
        reports[point['name']] = result
        print(json.dumps(dict(point=point['name'], status=result['status'], error=result.get('error'))), flush=True)
    start_reports = []
    for start in starts:
        rows = [reports[name] for name in start['points']]
        status = classify_start(rows, gates=start['original_gates'])
        start_reports.append(dict(start, status=status, authorized_for_supplied_jac_solve=status == 'VERIFIED',
            point_statuses=[dict(name=row['name'], status=row['status'], reason_code=row.get('reason_code')) for row in rows]))
    required = [row for report in reports.values() for step in report.get('directional', [])[-2:]
                for row in step['smooth_reports'] if row.get('compared_elements', 0)]
    primal = [row for report in reports.values() for row in report.get('primal_reports', []) if row.get('compared_elements', 0)]
    summary = dict(start_count=len(start_reports), point_count=len(reports), starts=start_reports,
        start_lookup={row['start_key']: row['status'] for row in start_reports},
        complete=True, all_starts_verified=all(row['status'] == 'VERIFIED' for row in start_reports),
        failed_validation_points=[name for name, row in reports.items() if row['status'] == 'DERIVATIVE_VALIDATION_FAILED'],
        unsupported_points=[name for name, row in reports.items() if row['status'] in ('DERIVATIVE_UNSUPPORTED', 'UNSUPPORTED_GATE_CASE')],
        maximum_required_directional_scaled_error=max((row['maximum_scaled_error'] for row in required), default=None),
        maximum_primal_absolute_error=max((row['maximum_absolute_error'] for row in primal), default=None),
        required_directional_comparison_elements=sum(row['compared_elements'] for row in required),
        required_directional_excluded_elements=sum(row.get('excluded_elements', 0) for row in required),
        protocol_sha256=file_sha256(directory/'protocol.json'), point_manifest_sha256=file_sha256(directory/'point_manifest.json'),
        source_sha256=file_sha256(directory/'source.json'), wall_time_s=time.perf_counter()-started,
        completed_utc=datetime.now(timezone.utc).isoformat(), no_finite_difference_solver_fallback=True,
        original_authoritative_validation_reused=True, optimizer_executed=False,
        scope='compatibility smoke only; unsupported starts remain in the planned ledger; no performance sample selection')
    write_new(directory/'summary.json', summary)
    return summary

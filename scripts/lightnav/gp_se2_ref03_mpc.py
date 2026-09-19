#!/usr/bin/env python3
"""REF-03 sequential frozen-selector transfer with complete attempted-case ledger.

For each frozen case: new A rollout, B/C selector-only probes at that A's 30
actual solve input poses, then independent B and C rollouts. All MPC computation
and selector implementation come from the immutable REF-02 runtime. Scientific
failures retain the original controller policy and never prune the schedule.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import traceback

for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[_name] = '1'
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))

import numpy as np

from reconciliation.gp_se2_rollout import (
    FAILURE_POLICY, PREVIOUS_CONTROL_POLICY, candidate_in_capture_frame,
    load_frozen_context, save_rollout, verify_source_records,
)
from reconciliation.gp_se2_ref02_reference import METHODS, validate_lineage
from reconciliation.gp_se2_ref02_rollout import array_sha256, counterfactual_rollout, selector_result
from reconciliation.online_mpc_adapter import load_official, sha256

COHORTS = ('KNOWN_CONTROLS', 'ADDITIONAL_TRANSFER')
TRANSFER_STRATA = ('O_OBSTACLE_NEAR', 'R_LARGE_ROTATION', 'P_LARGE_MISMATCH', 'S_BENIGN_STRAIGHT')


def serializable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: serializable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [serializable(item) for item in value]
    return value


def write_new(path, payload):
    with Path(path).open('x') as stream:
        json.dump(serializable(payload), stream, indent=2, allow_nan=False)
        stream.write('\n')


def verify_request(request):
    cases = request['cases']
    ids = [c['episode_id']+'/'+c['handoff_id'] for c in cases]
    if not cases or len(set(ids)) != len(ids) or request['frozen_case_order'] != ids:
        raise ValueError('ordered unique cases must equal the pre-frozen case list')
    cohorts = []
    for case in cases:
        if case['cohort'] not in COHORTS or not case.get('case_role'):
            raise ValueError('explicit control/transfer cohort and case role required')
        if case['cohort'] == COHORTS[1] and (case.get('stratum') not in TRANSFER_STRATA or case['case_role'] != case['stratum']):
            raise ValueError('additional transfer requires its frozen stratum role')
        cohorts.append(COHORTS.index(case['cohort']))
        if tuple(case['methods']) != METHODS:
            raise ValueError('A/B/C method order changed')
        if any(case['methods'][method] is None for method in METHODS):
            raise ValueError('missing references cannot be replaced by fallbacks')
    if cohorts != sorted(cohorts):
        raise ValueError('all known controls must precede additional transfer cases')
    for field, value in [('horizon_s', 3.), ('control_hz', 10.), ('integration_hz', 60.), ('source_progress_stride', 1.)]:
        if request.get(field, value) != value:
            raise ValueError('frozen REF-02 schedule/stride changed')
    for field, expected in [('planned_rollouts', len(cases)*3), ('primary_mpc_solves', len(cases)*90), ('selector_probes', len(cases)*60)]:
        if field in request and request[field] != expected:
            raise ValueError('planned request counts differ from frozen case matrix')
    return True


class _TrackingModule:
    """Record construction only; every computational method remains inherited."""
    def __init__(self, module):
        self._module = module
        self.instances = []
        owner = self

        class RegisteredTracker(module.MpcTracker):
            def __init__(self):
                super().__init__()
                owner.instances.append(self)

        self.MpcTracker = RegisteredTracker

    def __getattr__(self, name):
        return getattr(self._module, name)


def planned_row(case, method, ordinal):
    return dict(case_id=case['episode_id']+'/'+case['handoff_id'], cohort=case['cohort'],
        case_role=case['case_role'], stratum=case.get('stratum'), method=method,
        planned_execution_ordinal=ordinal, attempted=False, completed=False,
        status='NOT_ATTEMPTED', technical_error=None, planned_mpc_solves=30,
        primary_mpc_solve_count=0, actual_controller_calls=0,
        actual_selector_calls_recorded=0, completed_control_cycles=0,
        controller_failure_count=None, rollout_wall_s=None, official_mpc_solve_wall_s=None)


def _partial_capture(tracked):
    records = []
    for tracker in tracked.instances:
        capture = getattr(tracker, '_ref02_capture', {})
        records.append(dict(selector_calls=capture.get('selector_calls', []),
            controller_calls=capture.get('controller_calls', []), identity=capture.get('identity'),
            last_applied_command=list(tracker.command), last_controller_memory=list(tracker.previous_command),
            tracker_error=tracker.error))
    return dict(instances=records,
        actual_controller_calls=sum(len(r['controller_calls']) for r in records),
        actual_selector_calls_recorded=sum(len(r['selector_calls']) for r in records),
        states_and_commands_reconstructed=False,
        limitation='partial technical-failure captures are actual recorded arguments; no missing states, commands or uncompleted selector call is fabricated')


def execute_method(module, loaded, method, directory, settings_hash, ordinal, emit):
    """Attempt once. A returned rollout with controller failures is still complete."""
    case = loaded['case']; row = planned_row(case, method, ordinal)
    method_errors = loaded.get('method_errors', {})
    if loaded.get('case_error') or method in method_errors:
        row.update(status='INPUT_UNAVAILABLE', technical_error=loaded.get('case_error') or method_errors[method])
        target = directory/method; target.mkdir()
        write_new(target/'worker_status.json', row)
        emit(dict(type='METHOD_FINISHED', **row))
        return row, None
    row.update(attempted=True, status='STARTED')
    emit(dict(type='METHOD_STARTED', **row))
    tracked = _TrackingModule(module)
    started = time.perf_counter()
    rollout = None
    try:
        rollout = counterfactual_rollout(tracked, loaded['frozen'], loaded['references'][method],
            loaded['lineages'][method], method, original_goal_row_index=len(loaded['frozen']['fresh_world'])-1,
            constant_reference_metadata=loaded['metadata'].get(method))
        rollout.update(candidate_source=loaded['reference_sources'][method], official_settings_sha256=settings_hash,
            primary_execution_ordinal=ordinal, cohort=case['cohort'], case_role=case['case_role'],
            stratum=case.get('stratum'), task='GP-SE2-REF-03',
            diagnostic_label='OFFLINE FROZEN LOOKAHEAD TRANSFER DIAGNOSTIC')
        save_rollout(directory/method, rollout)
        row.update(completed=True, status='COMPLETED',
            primary_mpc_solve_count=rollout['actual_controller_calls'],
            actual_controller_calls=rollout['actual_controller_calls'],
            actual_selector_calls_recorded=rollout['actual_selector_calls'],
            completed_control_cycles=len(rollout['controller_reference_selections']),
            controller_failure_count=rollout['controller_failure_count'], rollout_wall_s=rollout['rollout_wall_s'],
            official_mpc_solve_wall_s=float(sum(t for t in rollout['official_mpc_solve_wall_s'] if t is not None)))
    except Exception as exc:
        capture = _partial_capture(tracked)
        row.update(status='TECHNICAL_FAILURE', technical_error=f'{type(exc).__name__}: {exc}',
            primary_mpc_solve_count=capture['actual_controller_calls'],
            actual_controller_calls=capture['actual_controller_calls'],
            actual_selector_calls_recorded=capture['actual_selector_calls_recorded'],
            completed_control_cycles=None, rollout_wall_s=time.perf_counter()-started)
        target = directory/method; target.mkdir(exist_ok=True)
        write_new(target/'technical_failure.json', dict(error=row['technical_error'], traceback=traceback.format_exc(),
            retry_performed=False, original_controller_policy_changed=False))
        write_new(target/'partial_actual_calls.json', capture)
        # A serialization/installation fault must never masquerade as a usable A.
        rollout = None
    write_new(directory/method/'worker_status.json', row)
    emit(dict(type='METHOD_FINISHED', **row))
    return row, rollout


def matched_state_probes(module, current_native, loaded):
    """B/C selectors only at the current run's A poses; no historical fallback."""
    begin = time.perf_counter()
    result = dict(status='UNAVAILABLE', unavailable_reason=None, states=[], installation={},
        planned_selector_calls=60, selector_calls_attempted=0, selector_calls_completed=0,
        probe_count=0, method_selection_count=0, new_mpc_solves=0,
        state_integration_performed=False, controller_memory_modified=False,
        probe_definition='B/C selectors at 30 actual A_NATIVE solve inputs generated earlier in this same case/run',
        historical_pose_fallback=False, B_C_fixed_reference_and_nearest_preserved=None)
    if current_native is None:
        result['unavailable_reason'] = 'CURRENT_A_ROLLOUT_INCOMPLETE_OR_UNAVAILABLE'
        result['wall_s'] = time.perf_counter()-begin
        return result
    if any(method in loaded.get('method_errors', {}) for method in METHODS[1:]):
        result['unavailable_reason'] = 'DENSE_REFERENCE_OR_LINEAGE_INPUT_UNAVAILABLE'
        result['wall_s'] = time.perf_counter()-begin
        return result
    solves = current_native['controller_reference_selections']
    if len(solves) != 30 or [r['time_s'] for r in solves] != (np.arange(30)*6/60).tolist():
        result['unavailable_reason'] = 'CURRENT_A_CONTROL_SCHEDULE_INCOMPLETE'
        result['wall_s'] = time.perf_counter()-begin
        return result
    try:
        capture_pose = current_native['candidate_frame_transform']['capture_pose_world']
        installed = {}
        for method in METHODS[1:]:
            reference = loaded['references'][method]
            local, transform = candidate_in_capture_frame(reference, capture_pose)
            installed[method] = module.project_body_to_world(local, capture_pose)
            result['installation'][method] = dict(installed_reference_world=installed[method].tolist(),
                candidate_capture_local=local.tolist(), candidate_frame_transform=transform,
                source_reference_array_sha256=array_sha256(reference),
                installed_path_sha256=array_sha256(installed[method]),
                source_reference_file=loaded['reference_sources'][method],
                array_hash_encoding='C-contiguous little-endian float64 Nx3 bytes',
                reference_shape=list(reference.shape), reference_dtype=str(reference.dtype),
                source_reference_array_modified=False,
                installation_semantics='immutable original capture inverse plus official projection; no tracker construction')
        if not np.array_equal(installed[METHODS[1]], installed[METHODS[2]]):
            raise ValueError('B/C probe-installed arrays differ')
        failures = 0
        for index, solve in enumerate(solves):
            pose = np.asarray(solve['input_pose_world'], dtype=np.float64)
            row = dict(probe_index=index, time_s=float(solve['time_s']), input_pose_world=pose.tolist(),
                methods={}, B_C_nearest_exactly_equal=None)
            for method in METHODS[1:]:
                result['selector_calls_attempted'] += 1
                try:
                    _, diagnostic = selector_result(module, method, installed[method], pose,
                        loaded['lineages'][method], len(loaded['frozen']['fresh_world'])-1,
                        constant_reference_metadata=loaded['metadata'].get(method))
                    row['methods'][method] = diagnostic
                    result['selector_calls_completed'] += 1
                except Exception as exc:
                    row['methods'][method] = dict(status='TECHNICAL_FAILURE', error=f'{type(exc).__name__}: {exc}')
                    failures += 1
            b, c = [row['methods'][m] for m in METHODS[1:]]
            if 'nearest_index' in b and 'nearest_index' in c:
                keys = ('nearest_index', 'nearest_original_fractional_row_coordinate', 'weighted_total_pose_distances',
                        'weighted_xy_contributions', 'weighted_yaw_contributions')
                row['B_C_nearest_exactly_equal'] = all(b[k] == c[k] for k in keys)
                if not row['B_C_nearest_exactly_equal']:
                    row['technical_error'] = 'MATCHED_STATE_NEAREST_RULE_MISMATCH'
                    failures += 1
            result['states'].append(row)
        result.update(status='COMPLETED' if failures == 0 else 'TECHNICAL_FAILURE',
            unavailable_reason=None if failures == 0 else 'PARTIAL_OR_FAILED_SELECTOR_AUDIT',
            probe_count=30, method_selection_count=result['selector_calls_completed'],
            B_C_fixed_reference_and_nearest_preserved=failures == 0)
    except Exception as exc:
        result.update(status='TECHNICAL_FAILURE', unavailable_reason=f'{type(exc).__name__}: {exc}',
            traceback=traceback.format_exc())
    result['wall_s'] = time.perf_counter()-begin
    return result


def execute_case(module, loaded, directory, settings_hash, ordinal_base, emit):
    """A -> current-A B/C probes -> B -> C; no scientific outcome filtering."""
    directory.mkdir(parents=True, exist_ok=False)
    write_new(directory/'input_validation.json', dict(case_error=loaded.get('case_error'),
        method_errors=loaded.get('method_errors', {}), gates_preserved=loaded.get('goal_route', {}).get('gates')))
    if 'frozen' in loaded:
        write_new(directory/'input_context.json', loaded['frozen'])
        write_new(directory/'goal_route.json', loaded['goal_route'])
    rows = []
    a_row, a_rollout = execute_method(module, loaded, METHODS[0], directory, settings_hash, ordinal_base, emit)
    rows.append(a_row)
    emit(dict(type='PROBES_STARTED', case_id=a_row['case_id'], source='current A_NATIVE', planned_selector_calls=60))
    probes = matched_state_probes(module, a_rollout, loaded)
    if a_rollout is not None:
        path = directory/METHODS[0]/'rollout.json'
        probes.update(source_rollout_path=str(path.resolve()), source_rollout_sha256=sha256(path))
    probes.update(case_id=a_row['case_id'], cohort=loaded['case']['cohort'], case_role=loaded['case']['case_role'])
    write_new(directory/'matched_state_probes.json', probes)
    emit(dict(type='PROBES_FINISHED', case_id=a_row['case_id'], status=probes['status'],
        selector_calls_attempted=probes['selector_calls_attempted'], selector_calls_completed=probes['selector_calls_completed']))
    installed_b = None
    for index, method in enumerate(METHODS[1:], 1):
        row, rollout = execute_method(module, loaded, method, directory, settings_hash, ordinal_base+index, emit)
        rows.append(row)
        if rollout is not None and method == METHODS[1]:
            installed_b = rollout['installed_reference_world']
        if rollout is not None and method == METHODS[2] and installed_b is not None and rollout['installed_reference_world'] != installed_b:
            # Preserve the already generated result; report a failed fixed-input audit.
            write_new(directory/'fixed_installed_reference_failure.json', dict(error='B/C actual installed arrays differ'))
    write_new(directory/'ledger.json', dict(methods=rows, probes=dict(status=probes['status'],
        selector_calls_attempted=probes['selector_calls_attempted'], selector_calls_completed=probes['selector_calls_completed'])))
    return rows, probes


def load_case(source, case, input_records):
    """Keep invalid input/missing method reasons without replacing the case."""
    loaded = dict(case=case, method_errors={}, references={}, lineages={}, metadata={}, reference_sources={})

    def checked_json(path):
        p = Path(path).resolve(); input_records.append(dict(path=str(p), sha256=sha256(p)))
        return json.loads(p.read_text())

    try:
        frozen = load_frozen_context(source, case['episode_id'], case['handoff_id'])
        stored = checked_json(case['input_context_path'])
        for key in ('case_id', 'B_world', 'u_minus', 'previous_control', 'original_capture_pose_world', 'fresh_world', 'old_world'):
            if stored[key] != frozen[key]:
                raise ValueError('frozen context differs from raw acquisition: '+key)
        loaded.update(frozen=frozen, goal_route=checked_json(case['goal_route_path']))
        # Gates are preserved and evaluated downstream; no GP derivative restriction applies.
        if 'gates' not in loaded['goal_route']:
            raise ValueError('original goal/route must declare gates')
    except Exception as exc:
        loaded['case_error'] = f'{type(exc).__name__}: {exc}'
        return loaded
    for method in METHODS:
        try:
            entry = case['methods'][method]
            path = Path(entry['reference_path']).resolve()
            record = dict(path=str(path), sha256=sha256(path)); input_records.append(record)
            reference = np.load(path, allow_pickle=False)
            lineage = checked_json(entry['lineage_path'])
            metadata = checked_json(entry['lineage_metadata_path']) if entry.get('lineage_metadata_path') else entry.get('lineage_metadata')
            validate_lineage(reference, lineage, constant_reference_metadata=metadata)
            if reference.dtype != np.dtype('float64'):
                raise ValueError('original float64 reference precision required')
            if method == METHODS[0] and not np.array_equal(reference, frozen['fresh_world']):
                raise ValueError('A must preserve original FRESH rows')
            if method != METHODS[0] and reference.shape != (30, 3):
                raise ValueError('dense reference must retain original 30 prepared rows')
            loaded['references'][method] = reference; loaded['lineages'][method] = lineage
            loaded['metadata'][method] = metadata; loaded['reference_sources'][method] = record
        except Exception as exc:
            loaded['method_errors'][method] = f'{type(exc).__name__}: {exc}'
    if not any(m in loaded['method_errors'] for m in METHODS[1:]):
        b, c = METHODS[1:]
        if (loaded['reference_sources'][b]['sha256'] != loaded['reference_sources'][c]['sha256']
                or loaded['references'][b].tobytes() != loaded['references'][c].tobytes()
                or loaded['lineages'][b] != loaded['lineages'][c]
                or loaded['metadata'][b] != loaded['metadata'][c]):
            for method in METHODS[1:]:
                loaded['method_errors'][method] = 'B/C dense reference, lineage or metadata differs'
    return loaded


def run(request_path, output, checkout):
    request_path, output, checkout = Path(request_path).resolve(), Path(output).resolve(), Path(checkout).resolve()
    if Path(sys.prefix).resolve() != (checkout/'mujoco_demo/.venv').resolve():
        raise ValueError('use existing official-demo isolated MPC environment')
    request = json.loads(request_path.read_text()); verify_request(request)
    source = Path(request['source_root']).resolve()
    if output.is_relative_to(source):
        raise ValueError('results cannot overwrite raw input source')
    output.mkdir(parents=True, exist_ok=False)
    begin = time.perf_counter(); module, provenance = load_official(checkout)
    settings_hash = hashlib.sha256(json.dumps(provenance['official_settings'], sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    if settings_hash != request['expected_official_settings_sha256']:
        raise ValueError('official settings differ from frozen original settings')
    if module.HORIZON != 5 or module.MPC_DT_S != .1 or module.CONTROL_RATE_HZ != 10. or tuple(module.Q_WEIGHTS) != (10., 10., 1.):
        raise ValueError('pinned MPC settings mismatch')
    planned = [planned_row(case, method, i*3+j) for i, case in enumerate(request['cases']) for j, method in enumerate(METHODS)]
    write_new(output/'planned_ledger.json', dict(methods=planned, cases=request['frozen_case_order'],
        primary_rollouts_planned=len(planned), primary_mpc_solves_planned=len(planned)*30,
        selector_probe_calls_planned=len(request['cases'])*60))
    repo = Path(__file__).resolve().parents[2]
    implementation = [Path(__file__).resolve(), *[repo/'src/reconciliation'/name for name in (
        'gp_se2_rollout.py', 'gp_se2_ref02_reference.py', 'gp_se2_ref02_rollout.py')]]
    provenance.update(task='GP-SE2-REF-03', official_settings_sha256=settings_hash,
        request_sha256=sha256(request_path), implementation_sha256={str(p): sha256(p) for p in implementation},
        primary_rollouts_planned=len(planned), primary_mpc_solves_planned=len(planned)*30,
        matched_state_selector_calls_planned=len(request['cases'])*60, historical_or_other_mpc_solves_planned=0,
        previous_control_policy=PREVIOUS_CONTROL_POLICY, failure_policy=FAILURE_POLICY,
        controller_statement='MPC computation unchanged; frozen REF-02 source-progress selector for C',
        tracker_instrumentation='immutable REF-02 actual-input audit; worker-only constructor registration for partial fault records',
        execution_order='each case A; current A poses B/C probes; B; C; then next case',
        source_progress_stride=1., runtime_load_wall_s=time.perf_counter()-begin)
    write_new(output/'provenance.json', provenance); write_new(output/'request.json', request)
    input_records = []; loaded = [load_case(source, case, input_records) for case in request['cases']]
    write_new(output/'input_hashes.json', dict(files=input_records))
    rows, probe_rows = [], []
    with (output/'events.jsonl').open('x') as stream:
        def emit(value):
            stream.write(json.dumps(serializable(value), allow_nan=False)+'\n'); stream.flush()
        for i, entry in enumerate(loaded):
            case = entry['case']; directory = output/(case['episode_id']+'__'+case['handoff_id'])
            case_rows, probes = execute_case(module, entry, directory, settings_hash, 3*i, emit)
            rows.extend(case_rows); probe_rows.append(probes)
            print(json.dumps(dict(case_id=case_rows[0]['case_id'], statuses=[r['status'] for r in case_rows],
                primary_mpc_solves=sum(r['primary_mpc_solve_count'] for r in case_rows), probe_status=probes['status'])), flush=True)
            if 'frozen' in entry:
                verify_source_records(source, entry['frozen']['source_files'])
            if sha256(Path(provenance['mpc_source'])) != provenance['mpc_source_sha256']:
                raise ValueError('external MPC changed during execution; remaining schedule not run')
    for record in input_records:
        if sha256(Path(record['path'])) != record['sha256']:
            raise ValueError('input changed during execution: '+record['path'])
    summary = dict(task='GP-SE2-REF-03', rows=rows, planned_cases=len(request['cases']),
        primary_rollouts_planned=len(planned), rollouts_attempted=sum(r['attempted'] for r in rows),
        independent_rollouts_completed=sum(r['completed'] for r in rows),
        primary_mpc_solves_planned=len(planned)*30, primary_mpc_solves=sum(r['actual_controller_calls'] for r in rows),
        actual_rollout_selector_calls_recorded=sum(r['actual_selector_calls_recorded'] for r in rows),
        actual_controller_calls=sum(r['actual_controller_calls'] for r in rows),
        matched_state_selector_calls_planned=len(request['cases'])*60,
        matched_state_selector_calls_attempted=sum(p['selector_calls_attempted'] for p in probe_rows),
        matched_state_selector_calls_completed=sum(p['selector_calls_completed'] for p in probe_rows),
        matched_probe_wall_s=sum(p['wall_s'] for p in probe_rows),
        technical_method_failures=sum(not r['completed'] for r in rows),
        probe_cases_completed=sum(p['status']=='COMPLETED' for p in probe_rows),
        historical_or_other_mpc_solves=0, total_worker_wall_s=time.perf_counter()-begin,
        original_inputs_and_external_mpc_preserved=True, new_VLA_updates=0,
        GP_or_rigid_optimization_solves=0, retries=0)
    write_new(output/'ledger.json', dict(methods=rows, planned_case_order=request['frozen_case_order']))
    write_new(output/'summary.json', summary)
    write_new(output/'output_hashes.json', {'files': [dict(path=str(p.relative_to(output)), sha256=sha256(p), bytes=p.stat().st_size)
        for p in sorted(output.rglob('*')) if p.is_file()]})
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--lightnav-checkout', type=Path, default=Path('/home/gpuadmin/Workspace/external/LightNav-0-official-demo'))
    args = parser.parse_args(); run(args.request, args.output, args.lightnav_checkout)


if __name__ == '__main__':
    main()

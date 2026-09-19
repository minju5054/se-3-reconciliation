#!/usr/bin/env python3
"""REF-02: 180 same-state selector probes followed by six fixed MPC rollouts.

Run only in the existing official-demo isolated environment. The research-side
selector differs for C; MPC computation, installation, integration and evaluation
policy are unchanged. No GP/rigid optimizer, VLA call or extra audit solve.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[_name] = '1'
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))

import numpy as np

from reconciliation.gp_se2_rollout import (
    FAILURE_POLICY, PREVIOUS_CONTROL_POLICY, candidate_in_capture_frame, load_frozen_context, save_rollout, verify_source_records,
)
from reconciliation.gp_se2_ref01_evaluation import FIXED_CASES
from reconciliation.gp_se2_ref02_reference import METHODS, validate_lineage
from reconciliation.gp_se2_ref02_rollout import array_sha256, counterfactual_rollout, selector_result
from reconciliation.online_mpc_adapter import load_official, sha256


def write_new(path, payload):
    with Path(path).open('x') as stream:
        json.dump(payload, stream, indent=2, allow_nan=False)
        stream.write('\n')


def verify_request(request):
    cases = request['cases']
    if tuple(c['episode_id']+'/'+c['handoff_id'] for c in cases) != FIXED_CASES:
        raise ValueError('REF-02 fixed case IDs/order changed')
    for case in cases:
        if tuple(case['methods']) != METHODS:
            raise ValueError('REF-02 three methods/order changed')
        if any(case['methods'][method] is None for method in METHODS):
            raise ValueError('missing reference is not replaced by a fallback')
    for field, value in [('horizon_s', 3.), ('control_hz', 10.), ('integration_hz', 60.), ('source_progress_stride', 1.)]:
        if request.get(field, value) != value:
            raise ValueError('REF-02 frozen schedule/stride changed')
    return True


def matched_state_probes(module, historical, references, lineages, original_goal_index,
                        *, saved_ref01_probes=None, metadata=None):
    """One selector call/method/state, no tracker construction or MPC solve."""
    solves = historical['controller_reference_selections']
    if len(solves) != 30 or [s['time_s'] for s in solves] != (np.arange(30)*6/60).tolist():
        raise ValueError('historical Native must provide the original 30 control input poses')
    if saved_ref01_probes is not None:
        rows = saved_ref01_probes['states']
        if len(rows) != 30 or any(a['time_s'] != b['time_s'] or a['input_pose_world'] != b['input_pose_world'] for a, b in zip(rows, solves)):
            raise ValueError('REF-01 matched-state inputs differ from historical Native')
    if references[METHODS[1]].dtype != references[METHODS[2]].dtype or references[METHODS[1]].shape != references[METHODS[2]].shape or references[METHODS[1]].tobytes() != references[METHODS[2]].tobytes() or lineages[METHODS[1]] != lineages[METHODS[2]]:
        raise ValueError('B/C dense reference and lineage must be identical')
    # Reproduce the exact original installation arithmetic without constructing a
    # tracker. The tiny capture-frame roundtrip is also present in every rollout;
    # probe the installed arrays, not nearby saved world values.
    capture_pose = historical['candidate_frame_transform']['capture_pose_world']
    installed, installation = {}, {}
    for name in METHODS:
        local, transform = candidate_in_capture_frame(references[name], capture_pose)
        installed[name] = module.project_body_to_world(local, capture_pose)
        installation[name] = dict(installed_reference_world=installed[name].tolist(),
            candidate_capture_local=local.tolist(), candidate_frame_transform=transform,
            source_reference_array_sha256=array_sha256(references[name]),
            installed_path_sha256=array_sha256(installed[name]),
            array_hash_encoding='C-contiguous little-endian float64 Nx3 bytes',
            reference_shape=list(references[name].shape), reference_dtype=str(references[name].dtype),
            source_reference_array_modified=False,
            installation_semantics='original candidate_in_capture_frame plus official project_body_to_world; no tracker or MPC construction')
    if not np.array_equal(installed[METHODS[1]], installed[METHODS[2]]):
        raise ValueError('B/C exact probe-installed arrays differ')
    states = []
    for index, solve in enumerate(solves):
        pose = np.asarray(solve['input_pose_world'], dtype=np.float64)
        methods = {}
        for name in METHODS:
            _, diagnostic = selector_result(module, name, installed[name], pose,
                lineages[name], original_goal_index,
                constant_reference_metadata=(metadata or {}).get(name))
            methods[name] = diagnostic
        b, c = methods[METHODS[1]], methods[METHODS[2]]
        for key in ('nearest_index', 'nearest_original_fractional_row_coordinate',
                    'weighted_total_pose_distances', 'weighted_xy_contributions', 'weighted_yaw_contributions'):
            if b[key] != c[key]:
                raise ValueError('same-state B/C nearest rule mismatch: '+key)
        states.append(dict(probe_index=index, time_s=float(solve['time_s']),
            input_pose_world=pose.tolist(), methods=methods, B_C_nearest_exactly_equal=True))
    return dict(states=states, installation=installation, probe_count=30, method_selection_count=90, new_mpc_solves=0,
        state_integration_performed=False, controller_memory_modified=False,
        B_C_fixed_reference_and_nearest_preserved=True,
        probe_definition='same historical Native solve input poses previously used by REF-01; source progress is a row-order label')


def run(request_path, output, checkout):
    request_path, output, checkout = Path(request_path).resolve(), Path(output).resolve(), Path(checkout).resolve()
    if Path(sys.prefix).resolve() != (checkout/'mujoco_demo/.venv').resolve():
        raise ValueError('use existing official-demo isolated MPC environment')
    request = json.loads(request_path.read_text())
    verify_request(request)
    source = Path(request['source_root']).resolve()
    if output.is_relative_to(source):
        raise ValueError('derived results cannot overwrite the raw source')
    output.mkdir(parents=True, exist_ok=False)
    begin = time.perf_counter()
    module, provenance = load_official(checkout)
    settings_hash = hashlib.sha256(json.dumps(provenance['official_settings'], sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    if settings_hash != request['expected_official_settings_sha256']:
        raise ValueError('loaded official settings differ from frozen source settings')
    if module.HORIZON != 5 or module.MPC_DT_S != .1 or module.CONTROL_RATE_HZ != 10. or tuple(module.Q_WEIGHTS) != (10., 10., 1.):
        raise ValueError('pinned MPC horizon/dt/rate/weights source mismatch')
    repo = Path(__file__).resolve().parents[2]
    implementation = [Path(__file__).resolve(), *[repo/'src/reconciliation'/name for name in (
        'gp_se2_rollout.py', 'gp_se2_ref02_reference.py', 'gp_se2_ref02_rollout.py')]]
    provenance.update(official_settings_sha256=settings_hash, request_sha256=sha256(request_path),
        implementation_sha256={str(p): sha256(p) for p in implementation},
        previous_control_policy=PREVIOUS_CONTROL_POLICY, failure_policy=FAILURE_POLICY,
        primary_mpc_solves_planned=180, historical_or_other_mpc_solves_planned=0,
        matched_state_selector_calls_planned=180, runtime_load_wall_s=time.perf_counter()-begin,
        task='GP-SE2-REF-02', tracker_instrumentation='per-instance private-global selector injection in unchanged official submit code; original _solve/poll; transparent actual controller argument recorder',
        controller_statement='MPC computation is identical; reference selector differs',
        source_progress_stride=1., float_search_policy='strict searchsorted side=left; no tolerance; each target offset from same nearest progress')
    write_new(output/'provenance.json', provenance)
    write_new(output/'request.json', request)
    loaded, input_records = [], []

    def checked_json(path):
        p = Path(path).resolve()
        input_records.append(dict(path=str(p), sha256=sha256(p)))
        return json.loads(p.read_text())

    for case in request['cases']:
        frozen = load_frozen_context(source, case['episode_id'], case['handoff_id'])
        stored = checked_json(case['input_context_path'])
        for key in ('case_id', 'B_world', 'u_minus', 'previous_control', 'original_capture_pose_world', 'fresh_world', 'old_world'):
            if stored[key] != frozen[key]:
                raise ValueError('frozen input context differs from original acquisition: '+key)
        if checked_json(case['goal_route_path'])['gates']:
            raise ValueError('fixed REF-02 cases must preserve original no-gate scope')
        historical = checked_json(case['historical_native_rollout_path'])
        saved_probes = checked_json(case['historical_matched_probes_path']) if case.get('historical_matched_probes_path') else None
        if historical['initial_pose_world'] != frozen['B_world'] or historical['initial_physical_command'] != frozen['u_minus'] or historical['initial_previous_control'] != frozen['previous_control']:
            raise ValueError('historical Native probes have different original initial conditions')
        references, lineages, reference_sources, metadata = {}, {}, {}, {}
        for name in METHODS:
            entry = case['methods'][name]
            path = Path(entry['reference_path']).resolve()
            record = dict(path=str(path), sha256=sha256(path))
            input_records.append(record)
            references[name] = np.load(path, allow_pickle=False)
            lineages[name] = checked_json(entry['lineage_path'])
            reference_sources[name] = record
            metadata[name] = checked_json(entry['lineage_metadata_path']) if entry.get('lineage_metadata_path') else entry.get('lineage_metadata')
            validate_lineage(references[name], lineages[name], constant_reference_metadata=metadata[name])
            if references[name].dtype != np.dtype('float64'):
                raise ValueError('original reference precision must remain float64')
        b, c = METHODS[1:]
        if references[b].shape != (30, 3) or references[c].shape != (30, 3) or reference_sources[b]['sha256'] != reference_sources[c]['sha256'] or references[b].tobytes() != references[c].tobytes() or lineages[b] != lineages[c]:
            raise ValueError('B/C must use byte-identical saved 30-row dense references and lineage')
        if not np.array_equal(references[METHODS[0]], frozen['fresh_world']) or not np.array_equal(references[METHODS[0]], historical['candidate_world']):
            raise ValueError('A must remain original Native FRESH')
        directory = output/(case['episode_id']+'__'+case['handoff_id'])
        directory.mkdir()
        write_new(directory/'input_context.json', frozen)
        loaded.append((case, frozen, historical, saved_probes, references, lineages, reference_sources, metadata, directory))
    write_new(output/'input_hashes.json', dict(files=input_records))
    probe_begin = time.perf_counter()
    for case, frozen, historical, saved_probes, references, lineages, reference_sources, metadata, directory in loaded:
        probes = matched_state_probes(module, historical, references, lineages,
            len(frozen['fresh_world'])-1, saved_ref01_probes=saved_probes, metadata=metadata)
        for name in METHODS:
            probes['installation'][name]['source_reference_file'] = reference_sources[name]
        probes.update(case_id=frozen['case_id'], source_rollout_path=case['historical_native_rollout_path'],
            source_rollout_sha256=sha256(Path(case['historical_native_rollout_path'])),
            source_ref01_probe_path=case.get('historical_matched_probes_path'))
        write_new(directory/'matched_state_probes.json', probes)
    probe_wall_s = time.perf_counter()-probe_begin
    write_new(output/'probes_completed.json', dict(matched_state_selector_calls=180, mpc_solves=0, wall_s=probe_wall_s))
    summaries = []
    rollout_begin = time.perf_counter()
    for case, frozen, _, _, references, lineages, reference_sources, metadata, directory in loaded:
        installed_by_method = {}
        for name in METHODS:
            rollout = counterfactual_rollout(module, frozen, references[name], lineages[name], name,
                original_goal_row_index=len(frozen['fresh_world'])-1, constant_reference_metadata=metadata[name])
            installed_by_method[name] = rollout['installed_reference_world']
            if name == METHODS[2] and installed_by_method[METHODS[1]] != installed_by_method[METHODS[2]]:
                raise ValueError('B/C actual installed world arrays differ')
            rollout.update(candidate_source=reference_sources[name], official_settings_sha256=settings_hash,
                primary_execution_ordinal=len(summaries))
            save_rollout(directory/name, rollout)
            row = dict(case_id=frozen['case_id'], method=name,
                primary_mpc_solve_count=len(rollout['controller_reference_selections']),
                actual_selector_calls=rollout['actual_selector_calls'], actual_controller_calls=rollout['actual_controller_calls'],
                controller_failure_count=rollout['controller_failure_count'], rollout_wall_s=rollout['rollout_wall_s'],
                official_mpc_solve_wall_s=float(sum(t for t in rollout['official_mpc_solve_wall_s'] if t is not None)))
            summaries.append(row)
            write_new(directory/name/'worker_status.json', row)
            print(json.dumps(row), flush=True)
        verify_source_records(source, frozen['source_files'])
    for record in input_records:
        if sha256(Path(record['path'])) != record['sha256']:
            raise ValueError('input changed during worker execution: '+record['path'])
    if sha256(Path(provenance['mpc_source'])) != provenance['mpc_source_sha256']:
        raise ValueError('external MPC changed during execution')
    summary = dict(task='GP-SE2-REF-02', rows=summaries, independent_rollouts=len(summaries),
        primary_mpc_solves=sum(r['primary_mpc_solve_count'] for r in summaries),
        actual_rollout_selector_calls=sum(r['actual_selector_calls'] for r in summaries),
        actual_controller_calls=sum(r['actual_controller_calls'] for r in summaries),
        historical_or_other_mpc_solves=0, matched_state_selector_calls=180,
        matched_probe_wall_s=probe_wall_s, rollout_batch_wall_s=time.perf_counter()-rollout_begin,
        total_worker_wall_s=time.perf_counter()-begin, original_inputs_and_external_mpc_preserved=True,
        new_VLA_updates=0, GP_or_rigid_optimization_solves=0)
    write_new(output/'summary.json', summary)
    write_new(output/'output_hashes.json', {'files': [dict(path=str(p.relative_to(output)), sha256=sha256(p), bytes=p.stat().st_size)
        for p in sorted(output.rglob('*')) if p.is_file()]})
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--lightnav-checkout', type=Path, default=Path('/home/gpuadmin/Workspace/external/LightNav-0-official-demo'))
    args = parser.parse_args()
    run(args.request, args.output, args.lightnav_checkout)


if __name__ == '__main__':
    main()

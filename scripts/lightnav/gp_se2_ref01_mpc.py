#!/usr/bin/env python3
"""REF-01: selector-only matched probes then eight unchanged official MPC rollouts.

Use the existing external official-demo isolated environment. No historical audit
solve, GP/rigid optimizer, VLA call, or new reference selector is implemented here.
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
    FAILURE_POLICY, PREVIOUS_CONTROL_POLICY, counterfactual_rollout,
    load_frozen_context, save_rollout, verify_source_records,
)
from reconciliation.gp_se2_ref01_evaluation import FIXED_CASES, VARIANTS
from reconciliation.gp_se2_ref01_reference import enrich_selection
from reconciliation.online_mpc_adapter import load_official, selection_audit, sha256
from reconciliation.se2 import wrap_angle


def write_new(path, payload):
    with Path(path).open('x') as stream:
        json.dump(payload, stream, indent=2, allow_nan=False)
        stream.write('\n')


def array_sha256(array):
    return hashlib.sha256(np.asarray(array, dtype='<f8').tobytes(order='C')).hexdigest()


def verify_request(request):
    cases = request['cases']
    if tuple(c['episode_id']+'/'+c['handoff_id'] for c in cases) != FIXED_CASES:
        raise ValueError('REF-01 fixed case IDs/order changed')
    for case in cases:
        if tuple(case['variants']) != VARIANTS:
            raise ValueError('REF-01 four variants/order changed')
        if any(case['variants'][variant] is None for variant in VARIANTS):
            raise ValueError('missing reference is not replaced by a fallback')
    for field, value in [('horizon_s', 3.), ('control_hz', 10.), ('integration_hz', 60.)]:
        if request.get(field, value) != value:
            raise ValueError('REF-01 original counterfactual schedule changed')
    return True


def matched_state_probes(module, historical, references, lineages, original_goal_index):
    """Call only the official selector; no tracker construction or MPC solves."""
    solves = historical['controller_reference_selections']
    if len(solves) != 30 or [s['time_s'] for s in solves] != (np.arange(30)*6/60).tolist():
        raise ValueError('historical Native must provide the original 30 control input poses')
    states = []
    for index, solve in enumerate(solves):
        pose = np.asarray(solve['input_pose_world'], dtype=np.float64)
        variants = {}
        for name in VARIANTS:
            reference = module.build_pose_aligned_reference(references[name], pose, horizon=module.HORIZON, weights=module.Q_WEIGHTS)
            audit = selection_audit(references[name], pose, reference, horizon=module.HORIZON, weights=module.Q_WEIGHTS)
            variants[name] = enrich_selection(references[name], pose, reference, lineages[name],
                weights=module.Q_WEIGHTS, horizon=module.HORIZON, original_goal_row_index=original_goal_index, audit=audit)
        states.append(dict(probe_index=index, time_s=float(solve['time_s']), input_pose_world=pose.tolist(), variants=variants))
    return dict(states=states, probe_count=30, variant_selection_count=120, new_mpc_solves=0,
                state_integration_performed=False, controller_memory_modified=False,
                probe_definition='same stored GP-SE2-02 Native solve input pose for all four input variants')


def enrich_rollout(module, rollout, reference, lineage, original_goal_index, variant):
    """Attach diagnostics after execution; never feed them back into the tracker."""
    installed = module.project_body_to_world(rollout['candidate_capture_local'], rollout['candidate_frame_transform']['capture_pose_world'])
    if not np.allclose(installed[:, :2], reference[:, :2], atol=1e-11, rtol=0) or not np.allclose(wrap_angle(installed[:, 2]-reference[:, 2]), 0, atol=1e-11, rtol=0):
        raise ValueError('installed world path differs from frozen reference')
    digest = array_sha256(installed)
    for solve in rollout['controller_reference_selections']:
        solve['selection_diagnostic'] = enrich_selection(installed, solve['input_pose_world'],
            solve['selection']['reference_world'], lineage, weights=module.Q_WEIGHTS,
            horizon=module.HORIZON, original_goal_row_index=original_goal_index, audit=solve['selection'])
        solve['installed_path_sha256'] = digest
        solve['installed_path_hash_encoding'] = 'C-contiguous little-endian float64 Nx3 bytes'
        solve['variant'] = variant
    rollout.update(variant=variant, diagnostic_label='OFFLINE REFERENCE-PREPARATION DIAGNOSTIC',
                   installed_reference_world=installed.tolist(), installed_path_sha256=digest,
                   installed_path_hash_encoding='C-contiguous little-endian float64 Nx3 bytes',
                   source_reference_array_sha256=array_sha256(reference),
                   selection_instrumentation='post-execution independent audit; original _synchronous_solve unchanged',
                   GP_or_rigid_optimization_performed=False)
    return rollout


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
        raise ValueError('loaded official settings differ from frozen GP-SE2-02 settings')
    if module.HORIZON != 5 or module.MPC_DT_S != .1 or module.CONTROL_RATE_HZ != 10.:
        raise ValueError('pinned MPC horizon/dt/control-rate source mismatch')
    implementation = [Path(__file__).resolve(), Path(__file__).resolve().parents[2]/'src/reconciliation/gp_se2_rollout.py',
                      Path(__file__).resolve().parents[2]/'src/reconciliation/gp_se2_ref01_reference.py']
    provenance.update(official_settings_sha256=settings_hash, request_sha256=sha256(request_path),
        implementation_sha256={str(p): sha256(p) for p in implementation},
        previous_control_policy=PREVIOUS_CONTROL_POLICY, failure_policy=FAILURE_POLICY,
        primary_mpc_solves_planned=240, historical_or_other_mpc_solves_planned=0,
        matched_state_selector_calls_planned=240,
        runtime_load_wall_s=time.perf_counter()-begin,
        task='GP-SE2-REF-01', tracker_instrumentation='unchanged gp_se2_rollout._synchronous_solve; diagnostic enrichment only after complete rollout')
    write_new(output/'provenance.json', provenance)
    write_new(output/'request.json', request)
    loaded = []
    input_records = []
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
        goal_route = checked_json(case['goal_route_path'])
        if goal_route['gates']:
            raise ValueError('fixed REF-01 cases must preserve original no-gate scope')
        historical = checked_json(case['historical_native_rollout_path'])
        if historical['initial_pose_world'] != frozen['B_world'] or historical['initial_physical_command'] != frozen['u_minus'] or historical['initial_previous_control'] != frozen['previous_control']:
            raise ValueError('historical Native probe source has different original initial conditions')
        references, lineages, reference_sources = {}, {}, {}
        for name in VARIANTS:
            entry = case['variants'][name]
            path = Path(entry['reference_path']).resolve()
            record = dict(path=str(path), sha256=sha256(path))
            input_records.append(record)
            references[name] = np.load(path, allow_pickle=False)
            lineages[name] = checked_json(entry['lineage_path'])
            reference_sources[name] = record
        if not np.array_equal(references['R00_NATIVE'], frozen['fresh_world']):
            raise ValueError('R00 must remain original native FRESH')
        if not np.array_equal(references['R00_NATIVE'], historical['candidate_world']):
            raise ValueError('historical Native reference differs from R00')
        directory = output/(case['episode_id']+'__'+case['handoff_id'])
        directory.mkdir()
        write_new(directory/'input_context.json', frozen)
        loaded.append((case, frozen, historical, references, lineages, reference_sources, directory))
    write_new(output/'input_hashes.json', dict(files=input_records))
    # All matched-state probes precede every actual primary solve.
    probe_begin = time.perf_counter()
    for case, frozen, historical, references, lineages, _, directory in loaded:
        probes = matched_state_probes(module, historical, references, lineages, len(frozen['fresh_world'])-1)
        probes.update(case_id=frozen['case_id'], source_rollout_path=case['historical_native_rollout_path'],
                      source_rollout_sha256=sha256(Path(case['historical_native_rollout_path'])))
        write_new(directory/'matched_state_probes.json', probes)
    probe_wall_s = time.perf_counter()-probe_begin
    write_new(output/'probes_completed.json', dict(matched_state_selector_calls=240, mpc_solves=0, wall_s=probe_wall_s))
    summaries = []
    rollout_begin = time.perf_counter()
    for case, frozen, _, references, lineages, reference_sources, directory in loaded:
        for name in VARIANTS:
            rollout = counterfactual_rollout(module, frozen, references[name], horizon_s=3., control_hz=10., integration_hz=60.)
            enrich_rollout(module, rollout, references[name], lineages[name], len(frozen['fresh_world'])-1, name)
            rollout.update(candidate_source=reference_sources[name], official_settings_sha256=settings_hash,
                           independent_tracker_instance=True, primary_execution_ordinal=len(summaries))
            save_rollout(directory/name, rollout)
            row = dict(case_id=frozen['case_id'], variant=name, primary_mpc_solve_count=len(rollout['controller_reference_selections']),
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
    summary = dict(task='GP-SE2-REF-01', rows=summaries, independent_rollouts=len(summaries),
        primary_mpc_solves=sum(r['primary_mpc_solve_count'] for r in summaries), historical_or_other_mpc_solves=0,
        matched_state_selector_calls=240, matched_probe_wall_s=probe_wall_s,
        rollout_batch_wall_s=time.perf_counter()-rollout_begin, total_worker_wall_s=time.perf_counter()-begin,
        original_inputs_and_external_mpc_preserved=True, new_VLA_updates=0, GP_or_rigid_optimization_solves=0)
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

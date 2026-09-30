#!/usr/bin/env python3
"""Saved-only R01 reconstruction and R00/R01 contrasts; no numerical solves."""
import argparse
import json
from pathlib import Path
import numpy as np
from run_osa03_relative_factor_replication01 import (
    ROOT, verify, source_phase, common_state, prior, plain, read, save, sha,
    LocalSE2Problem, ORDER, LABEL, equal_record)
from reconciliation.osa03_relative_ablation import comparability, planning, metric_row, gaps
from reconciliation.osa03_relative_replication import selector_diagnostics, paired_effects, classify
from reconciliation.osa03_common_b import references, may_execute, schedule
from reconciliation.graph_optimizer import retract_trajectory
from reconciliation.se2 import local_trajectory_to_world
from reconciliation.osa03_native import evaluate
from validate_osa03_common_b import validate_method
from validate_osa03_relative_factor_ablation01 import validate as validate_r00
from validate_osa03_relative_factor_ablation01 import validate_planning as validate_noR


def validate_planning(out, problem, world, env, include):
    if not include:
        audit = validate_noR(out, problem, world, env)
    else:
        state = problem.fresh.copy(); v = problem.residual_vector(state); cost = float(v@v)
        trace = read(out/'solver_trace.json'); checks = read(out/'feasibility_checks.json'); accepted = 0
        for e in trace:
            if 'candidate' in e:
                candidate = np.asarray(e['candidate'])
                np.testing.assert_array_equal(candidate, retract_trajectory(state, e['delta']))
                v = problem.residual_vector(candidate); cc = float(v@v)
                assert np.isclose(cc, e['candidate_cost'], rtol=1e-13, atol=1e-13)
                feasible = env['on'].check_polyline(candidate)['clearance_valid'] if cc < cost else None
                assert e['candidate_feasible'] == feasible
                decision = 'accepted' if cc < cost and feasible else ('rejected_unsafe' if cc < cost else 'rejected_non_improving')
                assert e['decision'] == decision
                damping = e['damping_before']*(.3 if decision == 'accepted' else 10.)
                assert np.isclose(e['damping'], max(np.finfo(float).eps, damping) if decision == 'accepted' else damping, rtol=1e-14)
                if decision == 'accepted':
                    state = candidate; cost = cc; accepted += 1
            np.testing.assert_array_equal(state, e['state'])
            equal_record(e['factor_costs'], problem.costs(state))
            assert 'R' in e['factor_costs']
        np.testing.assert_array_equal(state, world)
        assert len(checks) == 1+sum(e.get('candidate_cost', float('inf')) < e.get('cost_before', -float('inf')) for e in trace)
        for c in checks:
            equal_record(env['on'].check_polyline(c['state']), c['check'])
        result = read(out/'planning_result.json'); equal_record(result['final'], problem.costs(world))
        assert result['solver']['final_cost'] == trace[-1]['cost']
        audit = dict(valid=True, accepted_steps=accepted,
            unsafe_rejections=sum(e['decision'] == 'rejected_unsafe' for e in trace), new_solves=0)
    trace = read(out/'solver_trace.json'); result = read(out/'planning_result.json')
    np.testing.assert_array_equal(trace[0]['state'], problem.fresh)
    assert result['error'] is None and result['solver']['converged']
    audit.update(iterations=result['solver']['iterations'],
        rejected_steps=sum(e['decision'].startswith('rejected_') for e in trace),
        termination_reason=result['solver']['termination_reason'], original_FRESH_initialization=True)
    return audit


def validate(run):
    verify(run); cfg = read(run/'protocol.json'); manifest = read(run/'source_manifest.json')
    source = Path(manifest['source']); phase, fresh, raw, provenance = source_phase(cfg)
    common = common_state(phase); equal_record(common, read(run/'common_state.json'))
    assert common['first_FRESH_solve']['solve_id'].startswith('REPEAT_01_')
    old = Path(manifest['R00_run']); s00, v00 = validate_r00(old)
    assert v00['valid']; equal_record(s00, read(old/'summary.json'))
    refs = read(run/'references.json'); assert list(refs) == ORDER
    env = prior.geometry(source); scene = read(source/'scenario.json')
    problem = LocalSE2Problem(common['fresh_capture_pose'], common['B'], fresh)
    expected = references(common['fresh_capture_pose'], common['B'], fresh, fresh)
    rollouts = {}; metrics = {}; own = {}; checks = {}; planning_audits = {}; costs = {}
    for name, ref in refs.items():
        if ref['status'] == 'PLANNING_FAILED':
            result = read(run/'planning'/name/'planning_result.json'); assert result['error'] is not None
            assert read(run/'methods'/name/'skipped.json')['status'] == 'PLANNING_FAILED'
            assert not (run/'methods'/name/'rollout.json').exists()
            rollouts[name] = metrics[name] = own[name] = None
            checks[name] = dict(valid=True, planning_failed=True); planning_audits[name] = result
            continue
        for frame in ['world', 'local']:
            assert sha(ref[frame+'_path']) == ref[frame+'_sha256']
        world = np.load(ref['world_path']); local = np.load(ref['local_path'])
        if name in ORDER[:2]:
            np.testing.assert_array_equal(world, expected['M0_NATIVE' if name == 'M0_NATIVE' else 'M1_SE2_TAPER'])
        else:
            include = cfg['relative_factor'][name]; out = run/'planning'/name
            planning_audits[name] = validate_planning(out, problem, world, env, include)
            start = read(out/'optimization_start.json')
            assert start['include_relative'] == include and start['solver_config'] == cfg['solver']
            assert start['initial_world_sha256'] == refs['M0_NATIVE']['world_sha256'] and start['calls'] == 1
            costs[name] = dict(optimized_costs=problem.costs(world, include_relative=include),
                diagnostic_relative_edge_distortion_not_optimized_cost=None if include else problem.costs(world)['R'])
        np.testing.assert_allclose(local_trajectory_to_world(common['fresh_capture_pose'], local), world, atol=1e-12, rtol=0)
        if name == 'M0_NATIVE':
            np.testing.assert_array_equal(local, raw)
        equal_record(ref['safety'], prior.independent_clearance(world, env))
        equal_record(ref['descriptor'], plain(planning(fresh, world, common['B'])))
        r, m, audit = validate_method(run, name, ref, common, fresh, env['on'], scene, provenance['official_settings'])
        rollouts[name] = r; metrics[name] = m; checks[name] = audit; own[name] = None
        if r is None:
            continue
        assert r['raw_FRESH_sha256'] == manifest['FRESH_sha256']
        release = {p['submit_tick']: p['application_tick'] for p in read(run/'schedule.json')['pairs']}
        for wait in r['logical_waits']:
            assert wait['sim_before'] == wait['sim_after'] and wait['pose_before'] == wait['pose_after']
            assert wait['release_tick'] == release[wait['submit_tick']]
        for e in r['events']:
            if e.get('type') == 'solve_result' and not e.get('after_termination'):
                assert e['official_generation'] == e['result_generation'] == common['original_generation']
                assert e['continuation_seen']['tick'] == release[e['input_state_id']]
        if r['commands']:
            om = plain(evaluate(r, world, env['on'], scene, read(run/'metric_protocol.json')))
            equal_record(om, read(run/'methods'/name/'own_reference_metrics.json'))
            own[name] = {k: v for k, v in metric_row(om, r['termination']).items() if k in
                         ['position_auc_03_m_s', 'position_auc_09_m_s', 'yaw_auc_03_rad_s', 'yaw_auc_09_rad_s', 'max_position_error_05_m']}
    gate = comparability(rollouts, read(run/'schedule.json'), common)
    gate['full_window_match'] = {n: r is not None and schedule(r, 180) == read(run/'schedule.json')['full'] for n, r in rollouts.items()}
    rows = {n: metric_row(metrics[n], None if rollouts[n] is None else rollouts[n]['termination']) for n in ORDER}
    pairwise = {}
    for a, b in [('NO_RELATIVE', 'FULL_LOCAL_SE2'), ('FULL_LOCAL_SE2', 'M1_TAPER')]:
        if 'world_path' not in refs[a] or 'world_path' not in refs[b]:
            pairwise[a+'_minus_'+b] = None; continue
        x, y = (np.load(refs[n]['world_path']) for n in [a, b])
        row = dict(max_reference_XY_separation_m=float(np.linalg.norm(x[:, :2]-y[:, :2], axis=1).max()))
        if rollouts[a] and rollouts[b]:
            x, y = (np.asarray([s['pose_world'] for s in rollouts[n]['states']]) for n in [a, b]); k = min(len(x), len(y))
            row['max_execution_XY_separation_m'] = float(np.linalg.norm(x[:k, :2]-y[:k, :2], axis=1).max())
        pairwise[a+'_minus_'+b] = row
    valid = all(c['valid'] for c in checks.values())
    selectors = selector_diagnostics(rollouts)
    counts = dict(optimizer=sum((run/'planning'/n/'optimization_start.json').exists() for n in ORDER[2:]),
        optimizer_per_method={n:int((run/'planning'/n/'optimization_start.json').exists()) for n in ORDER},
        MPC={n:0 if r is None else r['new_MPC_solved'] for n, r in rollouts.items()},
        rollouts=sum(r is not None for r in rollouts.values()), LightNav=0, RGB=0, Isaac=0, source_acquisition=0, R00_reruns=0)
    summary = dict(label=LABEL, scientific_freeze_sha=read(run/'execution_start.json')['sha'],
        primary_metrics=rows, secondary_own_reference_metrics=own, signed_method_minus_native=gaps(rows, 'M0_NATIVE'),
        signed_method_minus_full=gaps(rows, 'FULL_LOCAL_SE2'), schedule_gate=gate, factor_costs=costs,
        planning={n:r.get('descriptor') for n,r in refs.items()}, planning_solver=planning_audits,
        reference_safety={n:r['safety'] for n,r in refs.items()}, pairwise=pairwise, counts=counts,
        termination={n:refs[n]['status'] if r is None else r['termination'] for n,r in rollouts.items()},
        selector_diagnostics=selectors, paired_source_facts=read(run/'paired_source_facts.json'),
        full_metrics={n:None if m is None else {k:m[k] for k in ['full','windows','extrema','endpoint','clearance']} for n,m in metrics.items()})
    summary['classification'] = classify(summary, valid)
    summary['replication_effects'] = paired_effects(s00, summary)
    audit = dict(valid=valid, methods=checks, planning=planning_audits, R00_saved_validator_unchanged=v00['valid'],
        original_FRESH_primary=True, own_reference_secondary=True, logical_schedule_gate=gate['comparable'],
        all_four_observed=gate['all_four_observed'], source_hashes_preserved=True, new_scientific_solves=0)
    return plain(summary), plain(audit)


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--run', type=Path, required=True); p.add_argument('--check-only', action='store_true')
    a = p.parse_args(); summary, audit = validate(a.run.resolve())
    if not a.check_only:
        save(a.run/'summary.json', summary); save(a.run/'validation.json', audit)
        save(a.run/'selector_diagnostics.json', summary['selector_diagnostics'])
    print(json.dumps(dict(classification=summary['classification'], validation=audit, counts=summary['counts'], effects=summary['replication_effects'])))

#!/usr/bin/env python3
"""One R01 paired replication; reuse frozen R00 runtime without editing its bytes."""
import argparse
import json
import sys
import time
from pathlib import Path
from dataclasses import asdict
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts'), str(ROOT/'scripts/isaac')]
import numpy as np
import yaml
import run_osa03_relative_factor_ablation01 as prior
from run_osa03_native_continuation import plain, git
from analyze_join_online02 import csvread, jsonlines
from reconciliation.join_source03 import read, save, sha
from reconciliation.osa03_native import phase_audit
from reconciliation.osa03_common_b import common_state, references, may_execute
from reconciliation.osa03_relative_ablation import ORDER
from reconciliation.osa03_relative_replication import LABEL, compatibility
from reconciliation.local_se2_reconciliation import LocalSE2Problem
from reconciliation.graph_optimizer import SolverConfig, solve_least_squares, OptimizationError
from reconciliation.online_mpc_adapter import PINNED_MPC_SHA256
from reconciliation.robotless_online import stamp
from validate_robotless_online_handoffs import equal_record
CONFIG = ROOT/'configs/osa03_relative_factor_replication_01.yaml'
RESULTS = ROOT/'results/osa03_relative_factor_replication_01'
DOC = ROOT/'docs/OSA03_RELATIVE_FACTOR_REPLICATION_01.md'
FILES = list(dict.fromkeys(prior.FILES + [CONFIG, Path(__file__),
    ROOT/'src/reconciliation/osa03_relative_replication.py',
    ROOT/'scripts/validate_osa03_relative_factor_replication01.py',
    ROOT/'scripts/report_osa03_relative_factor_replication01.py',
    ROOT/'tests/test_osa03_relative_factor_replication01.py']))


def source_phase(cfg):
    """Read exact R01 bytes, anchored by the tracked acquisition validation digest."""
    source = ROOT/cfg['source_run']; tracked = ROOT/cfg['tracked_acquisition']
    assert sha(tracked) == cfg['tracked_acquisition_sha256']
    assert sha(source/'validation.json') == read(tracked)['source_validation_sha256']
    saved = next(r for r in read(source/'validation.json')['rows'] if r['episode_id'] == 'REPEAT_01')
    assert saved['qualified'] and saved['original_validation']['valid']
    ep = source/'episodes/REPEAT_01'; bundle = source/'source_bundle/REPEAT_01'
    for name, h in saved['raw_source_hashes'].items():
        assert sha(ep/name) == h, name
    assert sha(bundle/'hashes.json') == cfg['R01_bundle_hashes_sha256']
    for name, h in read(bundle/'hashes.json').items():
        assert sha(bundle/name) == h, name
    state = read(bundle/'state_and_timing.json'); c = state['context']
    equal_record(c, saved['context']); equal_record(state['B_state'], saved['B_state'])
    assert c['episode_id'] == 'REPEAT_01'
    fresh = np.load(bundle/'derived/fresh_world.npy'); raw = np.load(bundle/'raw/fresh_lightnav.npy')
    for kind in ['fresh', 'old']:
        for frame, dst in [('raw_local', f'raw/{kind}_lightnav.npy'), ('world', f'derived/{kind}_world.npy')]:
            ref = c[f'{kind}_{frame}_ref']
            assert sha(bundle/dst) == ref['sha256'] == sha(ep/ref['path'])
    provenance = read(ep/'metadata.json')['mpc_worker']['provenance']
    assert provenance['mpc_source_sha256'] == PINNED_MPC_SHA256 == sha(provenance['mpc_source'])
    phase = phase_audit(c, csvread(ep/'execution.csv'), csvread(ep/'commands.csv'),
        jsonlines(ep/'controller/events.jsonl'), state['B_state'], fresh, raw,
        provenance['official_settings'], read(ep/'metadata.json')['resolved_integration_dt_s'])
    return phase, fresh, raw, provenance


def authenticate(cfg):
    assert cfg['episode'] == 'REPEAT_01' and cfg['order'] == ORDER
    oldcfg = yaml.safe_load(prior.CONFIG.read_text())
    for k in ['solver', 'relative_factor', 'normalization', 'initialization', 'weights',
              'footprint_radius_m', 'required_edge_clearance_m', 'integration_steps',
              'primary_intervals', 'maximum_rollouts', 'retries', 'wall_wait_timeout_s', 'metric_target']:
        assert cfg[k] == oldcfg[k], k
    assert cfg['solver'] == asdict(SolverConfig()) and cfg['scientific_optimizer_calls'] == 2
    tracked = ROOT/cfg['R00_tracked_result']; assert sha(tracked) == cfg['R00_tracked_result_sha256']
    r00 = read(tracked); old = Path(r00['run'])
    assert sha(old/'result_hashes.json') == r00['result_hashes_sha256']
    for name, h in read(old/'result_hashes.json').items():
        assert sha(old/name) == h, name
    prior.verify(old)
    _, inherited, schedule = prior.authenticate(oldcfg)
    assert schedule == read(old/'schedule.json')
    phase, fresh, raw, provenance = source_phase(cfg); common = common_state(phase)
    gate = compatibility(common, schedule, read(old/'common_state.json'),
                         provenance['official_settings'], inherited['mpc']['official_settings'])
    if not gate['compatible']:
        raise ValueError('TECHNICAL_BLOCKED: incompatible R01 source phase: '+json.dumps(gate))
    return old, inherited, schedule, phase, fresh, raw, provenance, gate


def prepare(run):
    cfg = yaml.safe_load(CONFIG.read_text())
    old, inherited, schedule, phase, fresh, raw, provenance, gate = authenticate(cfg)
    common = common_state(phase); source = ROOT/cfg['source_run']; bundle = source/'source_bundle/REPEAT_01'
    run.mkdir(parents=True, exist_ok=False); (run/'references').mkdir()
    save(run/'protocol.json', cfg); save(run/'schedule.json', schedule)
    save(run/'common_state.json', common); save(run/'controller_phase.json', phase)
    save(run/'compatibility.json', gate); save(run/'metric_protocol.json', read(old/'metric_protocol.json'))
    paths = [p for root in [bundle, source/'episodes/REPEAT_01'] for p in root.rglob('*') if p.is_file()]
    paths += [source/'validation.json', ROOT/cfg['tracked_acquisition'], ROOT/cfg['R00_tracked_result']]
    hashes = dict(inherited['hashes']); hashes.update({str(p): sha(p) for p in paths})
    save(run/'source_manifest.json', dict(source=str(source), episode='REPEAT_01',
        FRESH_sha256=sha(bundle/'raw/fresh_lightnav.npy'), OLD_sha256=sha(bundle/'raw/old_lightnav.npy'),
        FRESH_world_sha256=sha(bundle/'derived/fresh_world.npy'), mpc=provenance, hashes=hashes,
        R00_run=str(old), source_state_and_timing_sha256=sha(bundle/'state_and_timing.json')))
    f = references(common['fresh_capture_pose'], common['B'], fresh, fresh)
    env = prior.geometry(source)
    records = {n: prior.reference_record(run, n, f[k], raw, common, fresh, env)
               for n, k in [('M0_NATIVE', 'M0_NATIVE'), ('M1_TAPER', 'M1_SE2_TAPER')]}
    save(run/'prepared_references.json', records)
    from validate_obstacle_source_acquisition03 import episode
    audit = episode(source, 'REPEAT_01'); assert audit['qualified'] and audit['original_validation']['valid']
    save(run/'source_revalidation.json', audit)
    b0 = source/'source_bundle/REPEAT_00'
    paired = dict(raw_FRESH_identical=sha(b0/'raw/fresh_lightnav.npy') == sha(bundle/'raw/fresh_lightnav.npy'),
        raw_OLD_identical=sha(b0/'raw/old_lightnav.npy') == sha(bundle/'raw/old_lightnav.npy'),
        RGB_different=sha(b0/'raw/fresh_observation.jpg') != sha(bundle/'raw/fresh_observation.jpg'),
        world_FRESH_different=sha(b0/'derived/fresh_world.npy') != sha(bundle/'derived/fresh_world.npy'),
        independent_sessions=read(source/'episodes/REPEAT_00/session_open.json')['connection_id'] !=
                             read(source/'episodes/REPEAT_01/session_open.json')['connection_id'],
        R00_B=read(old/'common_state.json')['B'], R01_B=common['B'])
    assert all(paired[k] for k in ['raw_FRESH_identical', 'raw_OLD_identical', 'RGB_different',
                                 'world_FRESH_different', 'independent_sessions'])
    save(run/'paired_source_facts.json', paired)
    print(json.dumps(dict(prepared=str(run), compatible=gate, B=common['B'], planning_calls=0, MPC_calls=0)))


def preflight(run):
    prior.preflight(run)  # exact official installation/restoration, numerical solve forbidden


def freeze(run):
    with (run/'protocol_document.md').open('xb') as f:
        f.write(DOC.read_bytes())
    files = {str(p): sha(p) for p in FILES}
    inputs = {str(p): sha(p) for p in run.rglob('*') if p.is_file()}
    save(run/'freeze.json', dict(files=files, inputs=inputs, optimizer_calls=2, maximum_rollouts=4, retries=0))
    RESULTS.mkdir(parents=True, exist_ok=True)
    save(RESULTS/'freeze_summary.json', dict(run=str(run), freeze=read(run/'freeze.json'),
        common_state=read(run/'common_state.json'), schedule=read(run/'schedule.json'),
        compatibility=read(run/'compatibility.json'), source_manifest_sha256=sha(run/'source_manifest.json')))


def verify(run, pushed=False):
    frozen = read(run/'freeze.json')
    for group in ['files', 'inputs']:
        for p, h in frozen[group].items():
            assert sha(p) == h, p
    for p, h in read(run/'source_manifest.json')['hashes'].items():
        assert sha(p) == h, p
    authenticate(read(run/'protocol.json'))
    if pushed:
        assert git('rev-parse', 'HEAD') == git('ls-remote', 'origin', 'refs/heads/main').split()[0]
        for p in frozen['files']:
            assert git('rev-parse', 'HEAD:'+str(Path(p).relative_to(ROOT))) == git('hash-object', p), p
        assert not git('diff', 'HEAD', '--', str(RESULTS/'freeze_summary.json'))
        assert git('rev-parse', 'HEAD:'+str(DOC.relative_to(ROOT))) == git('hash-object', str(run/'protocol_document.md'))


def solve_condition(run, name):
    """Same problem/init/solver/gate for both; only R inclusion differs."""
    include = read(run/'protocol.json')['relative_factor'][name]
    out = run/'planning'/name; out.mkdir(parents=True, exist_ok=False)
    common = read(run/'common_state.json'); fresh = np.load(run/'references/M0_NATIVE_world.npy')
    p = LocalSE2Problem(common['fresh_capture_pose'], common['B'], fresh)
    env = prior.geometry(Path(read(run/'source_manifest.json')['source'])); trace = []; checks = []
    def feasible(x):
        check = env['on'].check_polyline(x); checks.append(dict(state=x.copy(), check=check))
        return check['clearance_valid']
    def observe(e):
        trace.append({**e, 'factor_costs': p.costs(e['state'], include_relative=include)})
    save(out/'optimization_start.json', dict(freeze_sha=git('rev-parse', 'HEAD'), started=stamp(), calls=1,
        initialization='original R01 FRESH', initial_world_sha256=sha(run/'references/M0_NATIVE_world.npy'),
        include_relative=include, solver_config=read(run/'protocol.json')['solver']))
    start = time.monotonic(); error = None; x = None
    try:
        solved = solve_least_squares(p.fresh, lambda state: p.residual_vector(state, include_relative=include),
            SolverConfig(**read(run/'protocol.json')['solver']), candidate_feasibility_fn=feasible, iteration_callback=observe)
        status = solved.to_dict()
        if solved.converged:
            x = solved.optimized
        else:
            error = 'planning did not converge; returned state recorded in trace, no rollout'
    except (OptimizationError, FloatingPointError, ValueError) as exc:
        error = str(exc); status = dict(termination_reason=type(exc).__name__, error=error)
    save(out/'solver_trace.json', plain(trace)); save(out/'feasibility_checks.json', plain(checks))
    save(out/'planning_result.json', dict(solver=status, error=error, wall_s=time.monotonic()-start,
        initial=p.costs(fresh, include_relative=include), final=None if x is None else p.costs(x, include_relative=include),
        diagnostic_relative_edge_distortion_not_optimized_cost=None if x is None or include else p.costs(x)['R']))
    return x


def execute(run):
    verify(run, True)
    save(run/'execution_start.json', dict(sha=git('rev-parse', 'HEAD'), started=stamp(), order=ORDER, label=LABEL, retries=0))
    refs = read(run/'prepared_references.json'); cfg = read(run/'protocol.json')
    phase, fresh, raw, _ = source_phase(cfg); c = common_state(phase); env = prior.geometry(ROOT/cfg['source_run'])
    for name in ORDER[2:]:
        x = solve_condition(run, name)
        refs[name] = (dict(status='PLANNING_FAILED', safety={'clearance_valid': False}) if x is None else
                      prior.reference_record(run, name, x, raw, c, fresh, env))
    save(run/'references.json', refs); (run/'methods').mkdir()
    for name in ORDER:
        if not may_execute(refs[name]['safety']):
            out = run/'methods'/name; out.mkdir()
            save(out/'skipped.json', dict(status=refs[name]['status'], metrics=None, rollouts=0))
            continue
        prior.run_method(run, name)  # unchanged official runtime, memory, release, guard, metrics
    verify(run); save(run/'completion.json', dict(completed=stamp(), source_preserved=True, label=LABEL))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--mode', choices=['prepare', 'preflight', 'freeze', 'execute'], required=True)
    args = parser.parse_args(); globals()[args.mode](args.run.resolve())

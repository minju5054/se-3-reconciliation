"""Saved-only RAW C0->C1 authentication and schedule eligibility. No live runtime.

The horizon follows physical application, while installs/submits follow controller
state. Keep both: a next-chunk solve can begin before its first physical command.
"""
from pathlib import Path
import csv
import subprocess

import numpy as np

from .join_source03 import read, sha
from .robotless_online import integrate_unicycle
from .se2 import local_trajectory_to_world


def jsonlines(path):
    import json
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line]


def csvrows(path):
    with Path(path).open() as stream:
        return list(csv.DictReader(stream))


def authenticate_files(files):
    for path, expected in files.items():
        if sha(path) != expected:
            raise ValueError(f'authenticated source hash mismatch: {path}')
    return len(files)


def authenticate(root, cfg):
    """Chain tracked result-commit evidence to the complete saved source seal."""
    result = cfg['source_result_commit']
    committed = subprocess.check_output(
        ['git', 'show', f"{result}:{cfg['source_integrity']}"], cwd=root)
    integrity_path = root / cfg['source_integrity']
    assert committed == integrity_path.read_bytes(), 'source result authentication'
    subprocess.run(['git', 'merge-base', '--is-ancestor', cfg['source_scientific_freeze'], result],
                   cwd=root, check=True)
    source = root / cfg['source']
    integrity = read(integrity_path)
    assert source.resolve() == Path(integrity['run']).resolve()
    pins = {
        source/'final_saved_artifact_seal.json': integrity['complete_candidate_seal_sha256'],
        source/'candidate_seal.json': integrity['original_candidate_seal_sha256'],
        source/'accounting_addendum.json': integrity['accounting_addendum_sha256'],
        source/'validation.json': integrity['validation_sha256'],
        source/'source_bundle/manifest.json': integrity['bundle_manifest_sha256'],
    }
    authenticate_files(pins)
    sealed_count = authenticate_files(read(source/'final_saved_artifact_seal.json')['files'])
    manifest = read(source/'source_bundle/manifest.json')
    episode = Path(manifest['source_episode'])
    assert episode == source.resolve()/'episodes/EPISODE_00'
    input_count = authenticate_files({episode/p: h for p, h in manifest['source_hashes'].items()})
    v = read(source/'validation.json')
    assert v['valid'] and v['provenance_valid']
    assert v['scientific_freeze_sha'] == cfg['source_scientific_freeze']
    return source, episode, dict(valid=True, result_commit=result,
        source_scientific_freeze=cfg['source_scientific_freeze'],
        integrity_sha256=sha(integrity_path), seal_sha256=sha(source/'final_saved_artifact_seal.json'),
        sealed_file_count=sealed_count, episode_input_count=input_count,
        historical_classification=v['classification'])


def horizon(commands, fresh_id):
    ticks = [int(c['application_tick']) for c in commands if c['chunk_id'] == fresh_id]
    if not ticks or ticks != list(range(ticks[0], ticks[-1]+1)):
        raise ValueError('missing or discontinuous physical C1 interval')
    return ticks[0], ticks[-1]+1


def extract_schedule(states, commands, scheduler, events, fresh_id):
    start, stop = horizon(commands, fresh_id)
    by_time = {float(s['sim_time_s']): int(s['tick']) for s in states}
    # Exact recorded simulation times; do not infer ticks from host receipt time.
    def tick(sim):
        return by_time[float(sim)]
    applications = {c['solve_id']: int(c['application_tick']) for c in commands
                    if c['reason'] == 'new_solve'}
    results = {e['solve_id']: e for e in events if e.get('type') == 'solve_result'}
    attempts = [r['start_state_id'] for r in scheduler
                if start <= r['start_state_id'] < stop and r['mpc_submitted']]
    submissions = []
    installs = []
    for row, e in enumerate(events):
        if e.get('status') == 'submitted' and start <= e['input_state_id'] < stop:
            r = results[e['solve_id']]
            submissions.append(dict(event_row=row, submit_tick=e['input_state_id'],
                chunk_id=e['chunk_id'], reference_version=e['reference_version'],
                generation=e['official_generation'], solve_id=e['solve_id'],
                application_tick=applications.get(e['solve_id']),
                result_status=r['status'], result_generation=r['result_generation'],
                result_seen_tick=tick(r['seen_in_isaac']['sim_time_s'])))
        if e.get('status') == 'installed':
            t = tick(e['install_context']['t_install']['sim_time_s'])
            if start <= t < stop:
                installs.append(dict(event_row=row, install_tick=t,
                    reply_seen_tick=tick(e['seen_in_isaac']['sim_time_s']),
                    chunk_id=e['chunk_id'], reference_version=e['reference_version'],
                    generation=e['official_generation']))
    accepted = [s['submit_tick'] for s in submissions]
    assert attempts == accepted, 'source contains rejected/busy/missing submit; not a closed schedule'
    foreign = [dict(kind='install', **i) for i in installs if i['chunk_id'] != fresh_id]
    foreign += [dict(kind='submit', **s) for s in submissions if s['chunk_id'] != fresh_id]
    initial = next(c for c in commands if int(c['application_tick']) == start)
    gen = results[initial['solve_id']]['official_generation']
    generation_changes = [i for i in installs if i['generation'] != gen]
    return dict(start_tick=start, stop_before_application_tick=stop,
        integration_steps=stop-start, initial_generation=gen,
        attempted_submit_ticks=attempts, accepted_submit_ticks=accepted,
        new_application_ticks=[t for t in applications.values() if start < t < stop],
        initial_application_restored=start, submissions=submissions, installs=installs,
        foreign_reference_events=foreign, generation_changes=generation_changes,
        raw_only_eligible=not foreign and not generation_changes)


def require_raw_schedule(schedule):
    if not schedule['raw_only_eligible']:
        raise ValueError('TECHNICAL_BLOCKED: full native schedule requires an in-window non-RAW reference')


def load_boundary(source, episode, cfg):
    r = read(source/'source_bundle/handoffs'/cfg['handoff']/'ready_record.json')
    assert r['fresh_chunk_id'] == cfg['allowed_reference']
    assert r['intrinsic_waypoint_dt'] is None and not r['reconciliation_computed']
    arrays = {}
    refs = {}
    for name in ['old_raw_local', 'old_world', 'fresh_raw_local', 'fresh_world']:
        ref = r[name]
        path = episode/ref['path']
        authenticate_files({path: ref['sha256']})
        arrays[name] = np.load(path, allow_pickle=False)
        arrays[name].setflags(write=False)
        refs[name] = dict(path=str(path), sha256=ref['sha256'])
    for prefix, anchor in [('old', 'A_old'), ('fresh', 'A_fresh')]:
        world = local_trajectory_to_world(r[anchor], arrays[prefix+'_raw_local'])
        np.testing.assert_allclose(world, arrays[prefix+'_world'], rtol=0, atol=cfg['selection_atol'])
    states = csvrows(episode/'execution.csv')
    commands = csvrows(episode/'commands.csv')
    start, stop = horizon(commands, r['fresh_chunk_id'])
    lookup = {int(s['tick']): s for s in states}
    pose = lambda s: [float(s[k]) for k in ['x', 'y', 'yaw']]
    assert r['B'] == pose(lookup[start]) and r['P'] == pose(lookup[start-1])
    cmd = {int(c['application_tick']): c for c in commands}
    control = lambda c: [float(c[k]) for k in ['v_mps', 'omega_radps']]
    assert r['u_minus'] == control(cmd[start-1])
    first = r['first_applied_command']
    assert start == first['application_tick'] and first['solve_id'] == cmd[start]['solve_id']
    plus = control(cmd[start])
    assert plus == r['first_solve']['command']
    events = jsonlines(episode/'controller/events.jsonl')
    memory = r['controller_previous_control']
    assert memory['available'] and events[memory['source_event_row']]['command'] == memory['previous_control']
    dt = read(episode/'metadata.json')['resolved_integration_dt_s']
    # Saved-data consistency only: never present this as an Isaac replay result.
    errors = []
    for t in range(start, stop):
        assert cmd[t]['chunk_id'] == r['fresh_chunk_id']
        np.testing.assert_allclose(float(lookup[t+1]['sim_time_s'])-float(lookup[t]['sim_time_s']),
                                   dt, rtol=0, atol=cfg['pose_atol'])
        errors.append(np.max(np.abs(integrate_unicycle(pose(lookup[t]), control(cmd[t]), dt)-pose(lookup[t+1]))))
    assert max(errors) <= cfg['pose_atol']
    boundary = dict(A=r['A_fresh'], A_old=r['A_old'], B=r['B'], P=r['P'],
        B_tick=start, B_sim_s=first['sim_time_s'], u_minus=r['u_minus'], u_B_plus=plus,
        controller_memory=memory, reference_version=r['reference_version'],
        generation=r['first_solve']['official_generation'], integration_dt_s=dt,
        first_solve_id=first['solve_id'], reference_files=refs,
        clocks={k: r[k] for k in ['t_obs_fresh','t_request_fresh','t_receipt_fresh',
            't_ready_fresh','t_ready_seen','t_install_fresh','t_switch_fresh']},
        saved_kinematic_max_component_error=max(errors), saved_kinematic_check_is_replay=False)
    return boundary, arrays, states, commands, events

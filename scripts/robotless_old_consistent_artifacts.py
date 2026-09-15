"""Immutable planning manifest and per-case derived pilot artifacts."""
import copy
from pathlib import Path
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from reconciliation.robotless_old_consistent_observation import (
    CASE_IDS, METRICS, validate_config, plan_observation, check_old_reproduction, immediate_metrics, raw_array_hash,
)
from reconciliation.robotless_single_chunk import load_config, save_json_exclusive, sha256_file, make_observation_time
from robotless_screening_artifacts import read_json, file_record, verify_file, git, phase_sources
SCREENING_VALIDATED = 'ROBOTLESS_LIGHTNAV_HANDOFF_SCREENING_VALIDATED'
SELECTION_VALIDATED = 'ROBOTLESS_OLD_CONDITIONED_HANDOFF_VALIDATED'


def inventory(source):
    return [file_record(p, source) for p in sorted(source.rglob('*')) if p.is_file()]


def validate_screening(source):
    # The isolated protocol client does not need plotting/validator dependencies.
    from validate_robotless_handoff_screening import validate_run
    return validate_run(source)


def validate_selection(source):
    from validate_robotless_old_conditioned import validate_run
    return validate_run(source)


def processing_sources(extra=()):
    return phase_sources(['src/reconciliation/robotless_old_conditioned_handoff.py',
        'src/reconciliation/robotless_old_consistent_observation.py', 'scripts/robotless_old_consistent_artifacts.py', *extra])


def freeze(config_path, run):
    pilot = load_config(config_path); validate_config(pilot)
    source = (ROOT/pilot['source_run']).resolve(); selection = (ROOT/pilot['selection_run']).resolve()
    if any(run.resolve().is_relative_to(p) for p in (source, selection)):
        raise ValueError('output cannot be inside a frozen input run')
    source_check = validate_screening(source); selection_check = validate_selection(selection)
    if source_check['status'] != SCREENING_VALIDATED or selection_check['status'] != SELECTION_VALIDATED:
        raise ValueError('both existing source validators must pass before freeze')
    config = load_config(source/'config_snapshot.yaml'); config.update(copy.deepcopy(pilot))
    del config['episodes']
    run.mkdir(parents=True, exist_ok=False)
    for name in ('episodes', 'aggregate', 'evidence', 'logs'): (run/name).mkdir()
    with (run/'config_snapshot.yaml').open('x') as stream: yaml.safe_dump(config, stream, sort_keys=False)
    save_json_exclusive(run/'source.json', {'source_run': str(source), 'selection_run': str(selection),
        'source_validation': source_check, 'selection_validation': selection_check,
        'source_files': inventory(source), 'selection_files': inventory(selection),
        'source_manifest_sha256': sha256_file(source/'manifest_frozen.json'),
        'created_time': make_observation_time(0), 'starting_git_sha': git('rev-parse','HEAD'),
        'pilot_config': file_record(config_path, ROOT), 'pilot_config_content': pilot})
    episodes = []
    for episode_id in CASE_IDS:
        location = source/'episodes'/episode_id; meta = read_json(location/'metadata.json')
        old = np.load(location/'derived/chunk_000_world.npy', allow_pickle=False)
        previous = [r for r in read_json(location/'derived/projection_metrics.json')['conditions'] if r['tau_s'] == 0]
        if len(previous) != 1: raise ValueError('one previous tau=0 row required')
        episodes.append({'episode_id': episode_id, 'category': meta['category'], 'instruction': meta['instruction'],
            'plan': plan_observation(meta['R0'], meta['R1'], old), 'previous_tau0': previous[0],
            'source_OLD_hash': raw_array_hash(np.load(location/'raw/chunk_000.npy', allow_pickle=False)),
            'source_inputs': [file_record(location/p, source) for p in ('metadata.json','raw/observation_000.jpg',
                'raw/chunk_000.npy','derived/chunk_000_world.npy','derived/projection_metrics.json')]})
    manifest = {'schema_version': 1, 'frozen_time': make_observation_time(0), 'episodes': episodes,
        'configuration': file_record(run/'config_snapshot.yaml', run), 'source': file_record(run/'source.json', run),
        'selection_basis': 'user-predeclared 006/007/013/016/027 from prior diagnostic results; no additions/deletions/replacements',
        'session_protocol': 'independent connect/login/reset/next(seq0 SOURCE RGB0)/next(seq1 NEW RGB1)/disconnect per case; no retry/reset/reconnect between seq0/1',
        'primary_tau_s': 0., 'actual_execution_time': None, 'processing_source_sha256': processing_sources()}
    save_json_exclusive(run/'manifest_frozen.json', manifest)
    save_json_exclusive(run/'manifest_freeze.json', {'manifest': file_record(run/'manifest_frozen.json', run),
        'time': make_observation_time(0), 'capture_started': False, 'inference_started': False})
    return manifest


def load_frozen(run):
    frozen = read_json(run/'manifest_freeze.json'); verify_file(run, frozen['manifest'])
    manifest = read_json(run/'manifest_frozen.json')
    for key in ('configuration', 'source'): verify_file(run, manifest[key])
    config = load_config(run/'config_snapshot.yaml'); validate_config(config)
    record = read_json(run/'source.json'); source = Path(record['source_run']); selection = Path(record['selection_run'])
    if source != (ROOT/config['source_run']).resolve() or selection != (ROOT/config['selection_run']).resolve():
        raise ValueError('frozen source paths changed')
    if record['source_files'] != inventory(source) or record['selection_files'] != inventory(selection):
        raise ValueError('frozen source file set/bytes changed')
    if tuple(e['episode_id'] for e in manifest['episodes']) != CASE_IDS or manifest['primary_tau_s'] != 0:
        raise ValueError('five-case order or tau changed')
    for episode in manifest['episodes']:
        location = source/'episodes'/episode['episode_id']; metadata = read_json(location/'metadata.json')
        for f in episode['source_inputs']: verify_file(source, f)
        expected = plan_observation(metadata['R0'], metadata['R1'], np.load(location/'derived/chunk_000_world.npy', allow_pickle=False))
        if episode['plan'] != expected: raise ValueError('frozen planning geometry changed')
    return config, manifest, record, frozen['manifest']['sha256']


def derive_case(run, episode):
    config, manifest, record, manifest_hash = load_frozen(run)
    location = run/'episodes'/episode['episode_id']; source = Path(record['source_run'])/'episodes'/episode['episode_id']
    status = read_json(location/'inference_status.json')
    result = {'episode_id': episode['episode_id'], 'category': episode['category'], 'plan': episode['plan'],
        'previous_tau0': episode['previous_tau0'], 'source_OLD_hash': episode['source_OLD_hash'],
        'status': status['status'], 'reason': status['reason'], 'reproduction': None, 'new_FRESH_hash': None,
        'primary_tau_s': 0., 'metrics': None, 'manifest_sha256': manifest_hash, 'actual_execution_time': None}
    if (location/'inference_metadata.json').exists():
        old, fresh = [np.load(location/f'raw/chunk_{i:03d}.npy', allow_pickle=False) for i in (0,1)]
        result['reproduction'] = check_old_reproduction(np.load(source/'raw/chunk_000.npy', allow_pickle=False), old)
        result['new_FRESH_hash'] = raw_array_hash(fresh)
        if result['reproduction']['status'] == 'PLANNING_OLD_MISMATCH':
            result.update(status='PLANNING_OLD_MISMATCH', reason='final seq0 OLD differs exactly from planning OLD; primary metrics excluded')
        elif status['status'] == 'VALID_PAIR':
            result['metrics'] = immediate_metrics(episode['plan'], np.load(location/'derived/chunk_001_world.npy', allow_pickle=False), episode['previous_tau0'])
        result['input_files'] = [file_record(location/p, location) for p in ('metadata.json','inference_metadata.json',
            'raw/chunk_000.npy','raw/chunk_001.npy','derived/chunk_000_world.npy','derived/chunk_001_world.npy')]
    return result


def analyze(run):
    from screen_robotless_handoffs import write_csv
    config, manifest, _, manifest_hash = load_frozen(run)
    results = [derive_case(run, e) for e in manifest['episodes']]
    rows = []
    for result in results:
        location = run/'episodes'/result['episode_id']
        save_json_exclusive(location/'derived/metrics.json', result)
        row = {k: v for k, v in result.items() if k not in ('metrics', 'input_files')}
        row.update(result['metrics'] or dict.fromkeys(METRICS))
        rows.append(row)
    # Keep invalid rows even when the first case lacks all primary metric fields.
    keys = list(dict.fromkeys(key for row in rows for key in row))
    write_csv(run/'aggregate/case_table.csv', [{key: row.get(key) for key in keys} for row in rows])
    valid = [r for r in results if r['status'] == 'VALID_PAIR']
    summary = {'case_count': 5, 'valid_count': len(valid), 'invalid_count': 5-len(valid),
        'statuses': {r['episode_id']: r['status'] for r in results}, 'case_results': results,
        'primary_tau_s': 0., 'manifest_sha256': manifest_hash, 'created_time': make_observation_time(0),
        'processing_source_sha256': processing_sources(), 'interpretation': 'five selected diagnostic cases; no population-frequency claim or threshold classifier'}
    save_json_exclusive(run/'aggregate/summary.json', summary)
    return summary
